/*
 *	task_consumer.c
 *	Subsystem 3 -- its tasks:
 *	  1. the Line & Barcode task and its cyclic handler
 *	  2. TEMPORARY: the template's example consumer task, kept only
 *	     because the example producer (S2) and monitor (S4) still use it
 *
 *	APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, tk_snd_mbf (TMO_POL),
 *	            tk_set_flg, tk_loc_mtx / tk_unl_mtx, tk_get_tim,
 *	            tk_rcv_mbx, tk_rel_mpf, tk_sig_sem
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"	/* IF-01/02: SAMPLE_MSG, pool, slots */
#include "../subsystem4/subsystem4.h"	/* IF-03: mbf_summary, flg_events */

/* ------------------------------------------------------------------ */
/*  1. Line & Barcode task                                            */
/* ------------------------------------------------------------------ */
/*
 *	Every 20 ms the task:
 *	  1. reads IR_LEFT / IR_CENTER, runs the line tracker (region A),
 *	     sends the steering byte to Motion Control (motion_send_steer)
 *	     and JUNCTION / LINE_LOST / LINE_FOUND to the Vehicle Controller;
 *	  2. drains the new barcode edges from bc_ring through the decoder
 *	     (region B), sending NAV_CMD to the Vehicle Controller and
 *	     setting the BARCODE_ERR fault bit;
 *	  3. publishes the packed status word g_s3_status and its 8-bit
 *	     MQTT slot (s1_tlm_set_slot) for Buddy 1's Telemetry Publisher,
 *	     which alone talks to MQTT.
 *	Every send uses TMO_POL: a full buffer never delays this task.
 */

#define S3_CAL_CYCLES		10	/* 10 x 20 ms of white for the baseline */

EXPORT volatile UW	g_s3_status;	/* packed, written only here */
EXPORT S3_DIAG		s3_diag;	/* guarded by mtx_s3_status */

LOCAL S3_DIAG		dg;		/* the task's working copy */
LOCAL S3_LINE		lt;
#if S3_DEBUG_LINE
LOCAL UB		dbg_line;	/* cycles since the last debug line */
#endif
LOCAL S3_BC		bc;
LOCAL UH		bc_overflows_seen;

/* State kept between cycles that is not in lt or bc (task only) */
LOCAL struct {
	UB	last_nav : 2;	/* NAV_xxx of the last valid barcode */
	UB	nav_valid : 1;
	UB	calibrated : 1;
	UB	overflow : 1;	/* bc_ring overflowed (sticky) */
	UB	err_hold;	/* cycles left showing bc_err, 0..50 */
} s3;

/* Calibration of the barcode sensor's white level (FR9): 10 samples of
   at most 4095, so the sum fits in 16 bits */
LOCAL struct {
	UH	sum;
	UH	min, max;	/* 12 bits each */
	UB	n : 4;		/* 0..S3_CAL_CYCLES */
	UB	fault_sent : 1;
} cal;

/* Bound by s3_bind_vc() before the tasks start; 0 = not yet */
LOCAL volatile ID	vc_mbf;
LOCAL volatile ID	fault_flg;

/*
 * Cyclic handler, every S3_PERIOD_MS.  Interrupt context: just wake
 * the task.
 */
EXPORT void line_barcode_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_line_barcode);
}

/* ------------------------------------------------------------------ */
/*  Outputs.  mbf_vc_evt and the fault flag belong to the VC / Buddy  */
/*  1 and may not be bound yet (s3_bind_vc): skip them if so.         */
/* ------------------------------------------------------------------ */

LOCAL void send_vc(UINT type, UW payload)
{
	ID	mbf = vc_mbf;
	UW	w = EVT_PACK(type, payload);

	if (mbf <= 0) return;
	if (tk_snd_mbf(mbf, &w, sizeof(w), TMO_POL) != E_OK) {
		dg.send_fail++;
	}
}

LOCAL void set_fault(UINT bits)
{
	ID	flg = fault_flg;

	if (flg > 0) tk_set_flg(flg, bits);
}

/* Clear only subsystem 3's own bits */
LOCAL void clear_fault(UINT bits)
{
	ID	flg = fault_flg;

	if (flg > 0) tk_clr_flg(flg, ~bits);
}

/* Motion Control (S2) keeps only the newest byte; never waits */
LOCAL void send_steer(void)
{
	if (motion_send_steer(lt.steer) == E_OK) {
		dg.steer_sent++;
	} else {
		dg.send_fail++;
	}
}

/* ------------------------------------------------------------------ */
/*  Region A: line tracker                                            */
/* ------------------------------------------------------------------ */

LOCAL void line_cycle(UW dt_ms, S3_STS_FIELDS *f)
{
	UINT	left   = drv_ir_left();
	UINT	center = drv_ir_center();

	/* The trim only means something while Motion Control follows the
	   line.  Turns, line search, STOP and E-STOP would teach it the
	   wrong pull (and a slow or stalled run would leave it full), so
	   it is cleared there and every new line-follow starts from 0. */
	if (s2_sts_state(motion_get_status()) != S2_ST_SPEED_FOLLOW) {
		lt.trim = 0;
	}

	switch (s3_line_step(&lt, left, center, dt_ms)) {
	case S3_LEV_JUNCTION:
		dg.junctions++;
		send_vc(EVT_JUNCTION, 0);
		tm_printf((UB*)"s3: JUNCTION\n");
		break;
	case S3_LEV_LINE_LOST:
		dg.line_lost++;
		send_vc(EVT_LINE_LOST, 0);
		tm_printf((UB*)"s3: LINE_LOST\n");
		break;
	case S3_LEV_LINE_FOUND:
		send_vc(EVT_LINE_FOUND, 0);
		tm_printf((UB*)"s3: LINE_FOUND\n");
		break;
	default:
		break;
	}
	send_steer();

#if S3_DEBUG_LINE
	if (++dbg_line >= 500 / S3_PERIOD_MS) {
		dbg_line = 0;
		tm_printf((UB*)"s3: L=%d C=%d state=%d steer=%d trim=%d"
			  " (state 0 on line, 1 drifting, 2 junction, 3 lost)\n",
			  left, center, lt.state, lt.steer, lt.trim);
	}
#endif

	f->ir_left    = left ? TRUE : FALSE;
	f->ir_center  = center ? TRUE : FALSE;
	f->line_state = lt.state;
	f->line_pos   = lt.pos;
	f->gap        = lt.gap;
	f->steer      = lt.steer;
}

/* ------------------------------------------------------------------ */
/*  Region B: barcode                                                 */
/* ------------------------------------------------------------------ */

/* White baseline from S3_CAL_CYCLES samples, one per cycle (200 ms) */
LOCAL void calibrate(UH raw)
{
	UH	white;

	if (cal.n == 0 || raw < cal.min) cal.min = raw;
	if (cal.n == 0 || raw > cal.max) cal.max = raw;
	cal.sum = (UH)(cal.sum + raw);
	cal.n++;
	if (cal.n < S3_CAL_CYCLES) return;

	white = (UH)(cal.sum / S3_CAL_CYCLES);
	if (cal.max - cal.min < S3_BC_DELTA_OFF &&
	    white + S3_BC_DELTA_ON < 4000) {
		s3_bc_init(&bc);
		bc_overflows_seen = drv_ir_overflow_count();
		drv_ir_start_edges(white);
		s3.calibrated = 1;
		if (cal.fault_sent) clear_fault(FAULT_IR_RIGHT);
		tm_printf((UB*)"s3: calibrated, white=%d (range %d..%d)\n",
			  white, cal.min, cal.max);
	} else {
		/* Not on plain white, or the sensor is unplugged/saturated:
		   report once, then keep retrying quietly */
		if (!cal.fault_sent) {
			set_fault(FAULT_IR_RIGHT);
			send_vc(EVT_SENSOR_FAULT, FAULT_IR_RIGHT);
			cal.fault_sent = 1;
			tm_printf((UB*)"s3: calibration failed, white=%d"
				  " range %d..%d, retrying\n",
				  white, cal.min, cal.max);
		}
	}
	cal.sum = 0;
	cal.n = 0;
}

#if S3_DEBUG_BARCODE
/* Element widths (ms) of the read that just ended: bar, space, bar...
   Code 39: narrow and wide elements, wide >= 1.5x narrow; a full
   character is 9 elements plus a 1-element gap (29/39/49 in total). */
LOCAL void print_widths(const S3_BC_RESULT *res)
{
	INT	i, n = res->elements;

	if (n > S3_BC_MAX_ELEMS) n = S3_BC_MAX_ELEMS;
	tm_printf((UB*)"s3:   widths ms:");
	for (i = 0; i < n; i++) tm_printf((UB*)" %d", bc.elem[i]);
	tm_printf((UB*)"\n");
}
#endif

LOCAL void report_barcode(INT ev, const S3_BC_RESULT *res)
{
	switch (ev) {
	case S3_BEV_NAV_CMD:
		dg.barcodes_ok++;
		s3.last_nav  = res->cmd;
		s3.nav_valid = 1;
		send_vc(EVT_NAV_CMD, res->cmd);
		tm_printf((UB*)"s3: barcode '%c' -> NAV_CMD %d\n",
			  res->ch, res->cmd);
#if S3_DEBUG_BARCODE
		print_widths(res);
#endif
		break;
	case S3_BEV_BARCODE_ERR:
		dg.barcodes_err++;
		s3.err_hold = S3_TLM_HOLD_CYCLES;
		set_fault(FAULT_BARCODE_ERR);
		tm_printf((UB*)"s3: BARCODE_ERR %d (%d elements)\n",
			  res->err, res->elements);
#if S3_DEBUG_BARCODE
		tm_printf((UB*)"s3:   (err 1 = not Code 39, 2 = no '*' start/stop,"
			  " 3 = valid but not A..D)\n");
		print_widths(res);
#endif
		break;
	case S3_BEV_DISCARD:
		dg.barcodes_discarded++;
#if S3_DEBUG_BARCODE
		if (res->elements > 0) {
			tm_printf((UB*)"s3: barcode dropped: %d elements (a full read is"
				  " 29, 39 or 49)%s\n", res->elements,
				  s3.overflow ? ", edges lost (overflow)" : "");
			print_widths(res);
		}
#endif
		break;
	default:
		break;
	}
}

LOCAL void barcode_cycle(S3_STS_FIELDS *f)
{
	UH		e[S3_BC_RING_SIZE];
	INT		n, i;
	UH		ms, ov, t;
	UH		raw = drv_ir_last_raw();
	S3_BC_RESULT	res;

	if (!s3.calibrated) {
		calibrate(raw);
	} else {
		/* Drain first, then read the clock, so no drained edge is
		   newer than ms */
		n  = drv_ir_read_edges(e, S3_BC_RING_SIZE);
		ov = drv_ir_overflow_count();
		ms = drv_ir_ms();

		if (ov != bc_overflows_seen) {
			/* Edges were lost: these edges straddle the gap, so
			   drop them and the barcode they belong to (8.3) */
			bc_overflows_seen = ov;
			s3.overflow = 1;
			report_barcode(s3_bc_abort(&bc, ms, &res), &res);
			n = 0;
		}

		for (i = 0; i < n; i++) {
			/* 15-bit edge time -> 16-bit ms clock (< 32 s old) */
			t = (UH)(ms - ((ms - e[i]) & S3_EDGE_TIME_MASK));
			report_barcode(s3_bc_edge(&bc, (e[i] & S3_EDGE_BLACK) != 0,
						  t, &res), &res);
		}
		report_barcode(s3_bc_poll(&bc, ms, &res), &res);
	}

	if (s3.err_hold > 0) s3.err_hold--;

#if S3_DEBUG_BARCODE >= 3
	/* 1 ms capture: arm once calibrated, print when the buffer is full */
	if (s3.calibrated) {
		static UB	armed;
		const volatile UH *cb;
		INT		k;

		if (!armed) {
			drv_ir_cap_arm((UH)(drv_ir_threshold() - S3_BC_DELTA_ON +
					      S3_CAP_TRIGGER));
			armed = 1;
			tm_printf((UB*)"s3: capture armed: pass the barcode sensor over"
				  " the barcode (white=%d, on=%d, off=%d)\n",
				  drv_ir_threshold() - S3_BC_DELTA_ON, drv_ir_threshold(),
				  drv_ir_threshold() - S3_BC_DELTA_ON + S3_BC_DELTA_OFF);
		} else if (drv_ir_cap_ready()) {
			cb = drv_ir_cap_buf();
			tm_printf((UB*)"s3: cap begin %d values, %d ms apart\n",
				  S3_CAP_LEN, S3_CAP_DECIM);
			for (k = 0; k < S3_CAP_LEN; k++) {
				if (k % 25 == 0) tm_printf((UB*)"s3: cap %4d:", k);
				tm_printf((UB*)" %d", cb[k]);
				if (k % 25 == 24) tm_printf((UB*)"\n");
			}
			tm_printf((UB*)"\ns3: cap end\n");
			armed = 0;		/* re-arm next cycle */
		}
	}
#endif

#if S3_DEBUG_BARCODE == 2
	/* ADC trace: every 20 ms while the barcode sensor sees anything
	   darker than white (raw > white + 150), else every 0.5 s.  Push
	   the robot slowly across the barcode by hand (~1 cm/s): each bar
	   should rise past thr_on ('B' in the bar graph) and each space
	   fall back near white. */
	{
		static UB	lvl_cyc;
		INT		thr  = drv_ir_threshold();
		INT		white = thr - S3_BC_DELTA_ON;
		INT		k, len;

		if ((INT)raw > white + 150 || ++lvl_cyc >= 500 / S3_PERIOD_MS) {
			lvl_cyc = 0;
			tm_printf((UB*)"s3: bc raw=%4d white=%4d on=%4d cal=%d st=%d ",
				  raw, white, thr, s3.calibrated, bc.state);
			len = ((INT)raw - white) / 100;
			if (len > 30) len = 30;
			for (k = 0; k < len; k++) {
				tm_printf((UB*)((white + (k + 1) * 100 > thr) ? "B" : "#"));
			}
			tm_printf((UB*)"\n");
		}
	}
#endif

	f->bc_state    = bc.state;
	f->last_nav    = s3.last_nav;
	f->nav_valid   = s3.nav_valid;
	f->calibrated  = s3.calibrated;
	f->bc_err      = (s3.err_hold > 0);
	f->bc_overflow = s3.overflow;
	dg.adc = ((UW)raw & S3_DIAG_ADC_MASK) |
		 (((UW)drv_ir_threshold() & S3_DIAG_ADC_MASK) << S3_DIAG_THR_SHIFT);
}

/* ------------------------------------------------------------------ */

EXPORT void line_barcode_task(INT stacd, void *exinf)
{
	SYSTIM		tim;
	UW		now, last, w;
	S3_STS_FIELDS	f;

	(void)stacd; (void)exinf;

	s3_line_init(&lt);
	s3_bc_init(&bc);
	tk_get_tim(&tim);
	last = tim.lo;

	while (1) {
		/* Sleep until line_barcode_cyclic_handler wakes us; drop any
		   wake-ups that queued while we were busy */
		tk_slp_tsk(TMO_FEVR);
		tk_can_wup(TSK_SELF);

		tk_get_tim(&tim);
		now = tim.lo;

		line_cycle(now - last, &f);
		barcode_cycle(&f);
		last = now;

		/* Publish: one aligned store of the packed word, then the
		   8-bit MQTT slot for Buddy 1's Telemetry Publisher */
		w = s3_sts_pack(&f);
		g_s3_status = w;
		if (s1_tlm_set_slot != NULL) {
			(void)s1_tlm_set_slot(S1_SLOT_B3, s3_tlm_slot(w));
		}

		dg.cycles++;
		dg.samples   = drv_ir_sample_count();
		dg.overflows = drv_ir_overflow_count();
		tk_loc_mtx(mtx_s3_status, TMO_FEVR);
		s3_diag = dg;
		tk_unl_mtx(mtx_s3_status);
	}
}

/* ------------------------------------------------------------------ */
/*  Public functions (subsystem3.h)                                   */
/* ------------------------------------------------------------------ */

EXPORT void s3_bind_vc(ID mbf_vc_evt, ID flg_fault)
{
	vc_mbf    = mbf_vc_evt;
	fault_flg = flg_fault;
}

EXPORT UW s3_get_status(void)
{
	return g_s3_status;
}

EXPORT BOOL s3_is_calibrated(void)
{
	return ((g_s3_status >> S3_STS_CAL_SHIFT) & 1u) ? TRUE : FALSE;
}

EXPORT void s3_get_diag(S3_DIAG *d)
{
	tk_loc_mtx(mtx_s3_status, TMO_FEVR);
	*d = s3_diag;
	tk_unl_mtx(mtx_s3_status);
}

/* ------------------------------------------------------------------ */
/*  2. TEMPORARY example consumer                                     */
/* ------------------------------------------------------------------ */

EXPORT void consumer_task(INT stacd, void *exinf)
{
	SAMPLE_MSG	*msg;
	UB		text[S4_SUMMARY_MAX];	/* tm_sprintf adds a '\0' */
	INT		len;
	UW		count;
	ER		er;

	(void)stacd; (void)exinf;

	while (1) {
		/* 1. Wait for the next sample (TMO_FEVR = wait forever) */
		er = tk_rcv_mbx(mbx_samples, (T_MSG **)&msg, TMO_FEVR);
		if (er < E_OK) {
			CHECK(er, "tk_rcv_mbx");
			continue;
		}

		/* 2. Update the shared statistics.  Subsystem 4 reads them
		      too, so lock the mutex around every access (IF-04). */
		tk_loc_mtx(mtx_stats, TMO_FEVR);
		if (app_stats.received == 0 || msg->value < app_stats.min) {
			app_stats.min = msg->value;
		}
		if (app_stats.received == 0 || msg->value > app_stats.max) {
			app_stats.max = msg->value;
		}
		app_stats.sum += msg->value;
		count = ++app_stats.received;
		tk_unl_mtx(mtx_stats);

		/* 3. Every S3_BATCH_SIZE samples, send a text line to subsystem
		      4 through its message buffer (the text is copied), then
		      set FLG_BATCH to wake it up (IF-03). */
		if (count % S3_BATCH_SIZE == 0) {
			/* tm_sprintf: like sprintf, returns the length.
			   The longest line here is ~35 bytes < S4_SUMMARY_MAX. */
			len = tm_sprintf(text, (const UB*)"n=%u last=%d from core %d",
					 count, msg->value, msg->prc - 1);

			/* TMO_POL: if the buffer is full, drop the line rather
			   than block the data path. */
			er = tk_snd_mbf(mbf_summary, text, len, TMO_POL);
			if (er == E_OK) {
				tk_set_flg(flg_events, FLG_BATCH);
			}
		}

		/* 4. Done with the sample (IF-02): return its memory block to
		      the pool, then the slot so the producer may go on. */
		tk_rel_mpf(mpf_samples, msg);
		tk_sig_sem(sem_slots, 1);
	}
}
