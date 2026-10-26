/*
 *	task_imu.c
 *	Subsystem 4 -- the IMU Monitor task, its cyclic handler, the console
 *	log task and the public functions of subsystem4.h.
 *
 *	APIs shown: tk_slp_tsk / tk_wup_tsk / tk_can_wup, tk_set_flg /
 *	            tk_clr_flg, tk_snd_mbf (TMO_POL), tk_loc_mtx / tk_unl_mtx
 *
 *	Every 10 ms (NFR11) the task:
 *	  1. reads the LSM303DLHC (or, in FAULT, retries it every 1 s);
 *	  2. reads Buddy 2's distance (g_motion_status) and takes the
 *	     wrap-safe signed difference (s2_dist_diff);
 *	  3. runs the IMU logic (imu_monitor.c);
 *	  4. reacts: SHOCK -> E-STOP in flg_safety first, then the latch;
 *	     HUMP_START / HUMP_END / SENSOR_FAULT -> mbf_vc_evt (TMO_POL);
 *	  5. publishes the two packed status words under mtx_s4_status.
 *	The task never waits except for its tick and that short mutex.
 */

#include "subsystem4.h"
#include "imu_monitor.h"
#include "drv_lsm303.h"
#include "../subsystem2/subsystem2.h"	/* g_motion_status, flg_safety */
#include <tk/syslib.h>

EXPORT ID		tid_imu;

/* Packed status: written ONLY by the IMU task, each with one aligned
   store, both under mtx_s4_status (s4_pack.h) */
LOCAL volatile UW	s4_w0, s4_w1;

LOCAL IMU_MONITOR	imu;		/* IMU task only */
LOCAL S4_DIAG		dg;		/* IMU task's working copy */
LOCAL S4_DIAG		s4_diag;	/* guarded by mtx_s4_status */

/* Shock latch: one writer per byte (s4_shock_ack in subsystem4.h) */
LOCAL volatile UB	shock_seq;	/* IMU task only: shocks seen, wraps */
LOCAL volatile UB	shock_acked;	/* Telemetry only: last one published */

/* Bound by s4_bind_vc() before the tasks start; 0 = not yet */
LOCAL volatile ID	vc_mbf;
LOCAL volatile ID	fault_flg;

#if APP_TEST
LOCAL volatile UB	inject_shock;	/* set by the test, cleared by the task */
#endif

/*
 * Cyclic handler, every S4_PERIOD_MS.  Interrupt context: just wake the
 * task.
 */
EXPORT void imu_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_imu);
}

/* ------------------------------------------------------------------ */
/*  Outputs                                                           */
/* ------------------------------------------------------------------ */

LOCAL void send_vc(UINT type, UW payload)
{
	ID	mbf = vc_mbf;
	UW	w = EVT_PACK(type, payload);

	if (mbf <= 0 || tk_snd_mbf(mbf, &w, sizeof(w), TMO_POL) != E_OK) {
		dg.evt_drops++;
	}
}

LOCAL void set_fault(UINT bits)
{
	ID	flg = fault_flg;

	if (flg > 0) tk_set_flg(flg, bits);
}

LOCAL void clr_fault(UINT bits)
{
	ID	flg = fault_flg;

	if (flg > 0) tk_clr_flg(flg, ~bits);	/* only our own bits */
}

LOCAL void debug_pin(INT pin, UINT level)
{
	if (pin >= 0) gpio_set_val((UINT)pin, level);
}

/* SHOCK: stop first (Motion wakes on S2_FLG_ESTOP at once), then latch.
   E-STOP is cleared only by the VC on a new START; FAULT_SHOCK only by
   Telemetry (s4_shock_ack). */
LOCAL void on_shock(void)
{
	tk_set_flg(flg_safety, S2_FLG_ESTOP);
	shock_seq = (UB)(shock_seq + 1);
	set_fault(FAULT_SHOCK);
	debug_pin(S4_DEBUG_SHOCK_PIN, 1);
}

LOCAL void on_sensor_fault(void)
{
	set_fault(FAULT_IMU);
	send_vc(EVT_SENSOR_FAULT, FAULT_IMU);
}

LOCAL void handle_events(const IMU_EVENT *ev, INT n)
{
	INT	i;

	for (i = 0; i < n; i++) {
		switch (ev[i].type) {
		case IMU_EV_SHOCK:
			on_shock();
			break;
		case IMU_EV_HUMP_START:
			send_vc(EVT_HUMP_START, 0);
			break;
		case IMU_EV_HUMP_END:
			send_vc(EVT_HUMP_END, S4_EVT_HEIGHT(ev[i].height_mm));
			break;
		case IMU_EV_SENSOR_FAULT:
			on_sensor_fault();
			break;
		default:
			break;
		}
	}
}

LOCAL void publish(void)
{
	S4_STS_FIELDS	f;
	UW		w0, w1;

	/* Build both words locally, then two single stores */
	imu_get_fields(&imu, &f);
	w0 = s4_pack_w0(&f);
	w1 = s4_pack_w1(&f);

	tk_loc_mtx(mtx_s4_status, TMO_FEVR);
	s4_w0 = w0;
	s4_w1 = w1;
	s4_diag = dg;
	tk_unl_mtx(mtx_s4_status);
}

/* ------------------------------------------------------------------ */
/*  IMU Monitor task                                                  */
/* ------------------------------------------------------------------ */

/* Sensor not found at start-up: say why, once (integration aid).
   Lines low      -> no power, shorted, or the cable is not on Grove 6.
   Nothing found  -> SDA/SCL swapped, wrong port or module unpowered.
   0x1E answers but IRA_REG_M != 0x48 -> not an LSM303DLHC (e.g. an
   LSM303D answers WHO_AM_I 0x0F = 0x49, an LSM303AGR mag 0x4F = 0x40). */
LOCAL void imu_diagnose(void)
{
	UB	found[8], v;
	UINT	lv = lsm303_line_levels();
	INT	n, i;
	BOOL	acc = FALSE, mag = FALSE, ira_ok = FALSE;

	tm_printf((UB*)"s4: I2C idle levels SDA=%d SCL=%d (both must be 1),"
		  " stuck-bus recoveries %d\n",
		  lv & 1u, (lv >> 1) & 1u, (INT)lsm303_bus_recoveries());
	n = lsm303_scan(found, 8);
	tm_printf((UB*)"s4: I2C scan: %d device(s)", n);
	for (i = 0; i < n && i < 8; i++) {
		tm_printf((UB*)" 0x%02x", found[i]);
		if (found[i] == LSM303_ACC_ADDR) acc = TRUE;
		if (found[i] == LSM303_MAG_ADDR) mag = TRUE;
	}
	tm_printf((UB*)"\n");
	if (lsm303_read_reg(LSM303_MAG_ADDR, 0x0A, &v)) {
		tm_printf((UB*)"s4: 0x1E IRA_REG_M=0x%02x (LSM303DLHC = 0x48)\n", v);
		ira_ok = (v == 0x48);
	}
	if (lsm303_read_reg(LSM303_MAG_ADDR, 0x0F, &v)) {
		tm_printf((UB*)"s4: 0x1E reg 0x0F=0x%02x\n", v);
	}
	if (lsm303_read_reg(0x1D, 0x0F, &v)) {
		tm_printf((UB*)"s4: 0x1D WHO_AM_I=0x%02x (LSM303D = 0x49)\n", v);
	}

	/* One plain-language verdict for whoever reads the log */
	if ((lv & 3u) != 3u) {
		tm_printf((UB*)"s4: VERDICT a line is held low: IMU unpowered, cable"
			  " shorted, or not plugged into Grove 6 (GP26/GP27)\n");
	} else if (n == 0) {
		tm_printf((UB*)"s4: VERDICT nothing answers: SDA/SCL swapped, wrong"
			  " Grove port, or the module has no power (check VIN/GND)\n");
	} else if (!acc || !mag) {
		tm_printf((UB*)"s4: VERDICT devices answer but not both 0x19 and 0x1E:"
			  " a different IMU chip than the LSM303DLHC -- send this log"
			  " to Buddy 4\n");
	} else if (!ira_ok) {
		tm_printf((UB*)"s4: VERDICT 0x19/0x1E answer but the ID is wrong: not an"
			  " LSM303DLHC -- send this log to Buddy 4\n");
	} else {
		tm_printf((UB*)"s4: VERDICT wiring and chip look OK now (loose"
			  " contact?); the IMU retries every 1 s\n");
	}
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	IMU_SAMPLE	s;
	IMU_EVENT	ev[IMU_MAX_EVENTS];
	SYSTIM		tim;
	UW		now_ms, last_retry, t0, used;
	UW		dist, prev_dist;
	BOOL		acc_ok, mag_ok;
	INT		n;

	(void)stacd; (void)exinf;

	if (S4_DEBUG_SHOCK_PIN >= 0) {
		gpio_set_pin(S4_DEBUG_SHOCK_PIN, GPIO_MODE_OUT);
		gpio_set_val(S4_DEBUG_SHOCK_PIN, 0);
	}
	if (S4_DEBUG_TIMING_PIN >= 0) {
		gpio_set_pin(S4_DEBUG_TIMING_PIN, GPIO_MODE_OUT);
		gpio_set_val(S4_DEBUG_TIMING_PIN, 0);
	}

	/* Bring-up check: both chips of the LSM303DLHC must answer */
	lsm303_bus_init();
	acc_ok = lsm303_probe(LSM303_ACC_ADDR);
	mag_ok = lsm303_probe(LSM303_MAG_ADDR);
	dg.sensor_found = lsm303_init() ? 1 : 0;
	tm_printf((UB*)"s4: I2C1 GP%d/GP%d: accel 0x19 %s, mag 0x1E %s -> %s\n",
		  S4_PIN_IMU_SDA, S4_PIN_IMU_SCL,
		  acc_ok ? "found" : "MISSING", mag_ok ? "found" : "MISSING",
		  dg.sensor_found ? "LSM303DLHC ready, keep the robot STILL for 2 s"
				  : "IMU FAULT, retrying every 1 s");

	if (!dg.sensor_found) {
		imu_diagnose();
	}

	imu_init(&imu);
	if (!dg.sensor_found) {
		imu_set_fault(&imu);
		on_sensor_fault();
	}

	tk_get_tim(&tim);
	last_retry = tim.lo;
	prev_dist = s2_sts_dist(motion_get_status());
	publish();

	while (1) {
		/* Sleep until imu_cyclic_handler wakes us.  Wake-ups that
		   queued meanwhile mean a cycle was missed. */
		tk_slp_tsk(TMO_FEVR);
		if (tk_can_wup(TSK_SELF) > 0) dg.overruns++;

		t0 = in_w(TIMER_TIMERAWL);
		debug_pin(S4_DEBUG_TIMING_PIN, 1);
		tk_get_tim(&tim);
		now_ms = tim.lo;

		/* 1. sensor */
		s.ok = FALSE;
		if (imu.state == S4_ST_FAULT) {
			if (now_ms - last_retry >= IMU_RETRY_MS) {
				last_retry = now_ms;
				if (lsm303_init()) {
					imu_restart(&imu);	/* calibrate again */
					clr_fault(FAULT_IMU);
					dg.sensor_found = 1;
					tm_printf((UB*)"s4: IMU answers again, keep the"
						  " robot STILL for 2 s (calibrating)\n");
				}
			}
		} else {
			lsm303_read(&s);
		}

		/* 2. distance from Motion Control (one aligned word) */
		dist = s2_sts_dist(motion_get_status());

		/* 3.-4. logic and reactions */
		n = imu_step(&imu, &s, s2_dist_diff(prev_dist, dist), now_ms, ev);
		prev_dist = dist;
		handle_events(ev, n);
#if APP_TEST
		if (inject_shock) {
			inject_shock = 0;
			on_shock();
		}
#endif
		if (!s4_shock_pending()) debug_pin(S4_DEBUG_SHOCK_PIN, 0);

		/* 5. timing and status */
		dg.cycles++;
		dg.i2c_errors = lsm303_error_count();
		used = in_w(TIMER_TIMERAWL) - t0;
		dg.last_us = used;
		if (used > dg.wcet_us) dg.wcet_us = used;
		publish();
		debug_pin(S4_DEBUG_TIMING_PIN, 0);
	}
}

/* ------------------------------------------------------------------ */
/*  Console log task (low priority, prints only)                      */
/* ------------------------------------------------------------------ */

LOCAL const char *s4_state_name(INT st)
{
	static const char * const name[] = {
		"CALIBRATING", "LEVEL", "CLIMBING", "DESCENDING", "SHOCK", "FAULT"
	};
	return (st >= 0 && st <= S4_ST_FAULT) ? name[st] : "?";
}

EXPORT void imu_log_task(INT stacd, void *exinf)
{
	S4_STS_FIELDS	f;
	S4_DIAG		d;
	UW		w0, w1;
	INT		last_state = -1, last_count = 0, ticks = 0, a;
	UB		last_seq = 0;

	(void)stacd; (void)exinf;

	while (1) {
		tk_dly_tsk(100);		/* changes: 100 ms, full line: S4_LOG_MS */
		s4_get_status(&w0, &w1);
		s4_sts_unpack(w0, w1, &f);

		if (f.state != last_state) {
			tm_printf((UB*)"s4: state %s\n", s4_state_name(f.state));
			last_state = f.state;
		}
		if (f.hump_count != last_count) {
			tm_printf((UB*)"s4: HUMP_END height=%d mm (humps=%d)\n",
				  f.hump_mm, f.hump_count);
			last_count = f.hump_count;
		}
		if (s4_shock_seq() != last_seq) {
			tm_printf((UB*)"s4: SHOCK -> E-STOP set\n");
			last_seq = s4_shock_seq();
		}

		ticks += 100;
		if (ticks >= S4_LOG_MS) {
			ticks = 0;
			s4_get_diag(&d);
			a = f.pitch_dd < 0 ? -f.pitch_dd : f.pitch_dd;
			tm_printf((UB*)"s4: %s pitch=%c%d.%d hdg=%d rate=%d cal=%d"
				  " hump=%dmm n=%d fails=%d | wcet=%uus last=%uus"
				  " cyc=%u ovr=%u i2c_err=%u evt_drop=%u\n",
				  s4_state_name(f.state), f.pitch_dd < 0 ? '-' : '+',
				  a / 10, a % 10, f.heading_deg, f.rate_dps,
				  f.calibrated, f.hump_mm, f.hump_count, f.fail_count,
				  d.wcet_us, d.last_us, d.cycles, d.overruns,
				  d.i2c_errors, d.evt_drops);
		}
	}
}

/* ------------------------------------------------------------------ */
/*  Public functions (subsystem4.h)                                   */
/* ------------------------------------------------------------------ */

EXPORT void s4_bind_vc(ID mbf_vc_evt, ID flg_fault)
{
	vc_mbf    = mbf_vc_evt;
	fault_flg = flg_fault;
}

EXPORT void s4_get_status(UW *w0, UW *w1)
{
	tk_loc_mtx(mtx_s4_status, TMO_FEVR);
	*w0 = s4_w0;
	*w1 = s4_w1;
	tk_unl_mtx(mtx_s4_status);
}

EXPORT UW s4_get_status_w0(void)
{
	return s4_w0;
}

EXPORT BOOL s4_is_calibrated(void)
{
	return s4_sts_calibrated(s4_w0) ? TRUE : FALSE;
}

EXPORT BOOL s4_in_hump(void)
{
	INT	st = s4_sts_state(s4_w0);

	return (st == S4_ST_CLIMBING || st == S4_ST_DESCENDING) ? TRUE : FALSE;
}

EXPORT UB s4_shock_seq(void)
{
	return shock_seq;
}

EXPORT BOOL s4_shock_pending(void)
{
	return (shock_seq != shock_acked) ? TRUE : FALSE;
}

EXPORT void s4_shock_ack(UB seq)
{
	clr_fault(FAULT_SHOCK);
	shock_acked = seq;
	/* A newer shock came in after seq was read: keep the bit set */
	if (shock_seq != seq) set_fault(FAULT_SHOCK);
}

EXPORT void s4_get_diag(S4_DIAG *d)
{
	tk_loc_mtx(mtx_s4_status, TMO_FEVR);
	*d = s4_diag;
	tk_unl_mtx(mtx_s4_status);
}

#if APP_TEST
EXPORT void s4_test_inject_shock(void)
{
	inject_shock = 1;
}
#endif
