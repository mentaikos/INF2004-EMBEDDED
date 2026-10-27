/*
 *	subsystem3.c
 *	Subsystem 3 -- Line & Barcode: creates its objects, starts its task,
 *	and holds the code the task calls:
 *	  1. IR sensor driver and the 1 ms Barcode ADC ISR
 *	  2. line tracker (6.3 region A)
 *	  3. barcode edge detector and decoder (6.3 region B)
 *	  4. packed status word and MQTT telemetry slot (minimum-bit)
 *	The line tracker, edge detector, decoder and packers are pure logic
 *	with no hardware access, so subsystem3_test.c can feed them made-up
 *	data.
 *
 *	APIs shown: tk_cre_mtx, tk_cre_cyc, tk_sta_cyc, tk_def_int,
 *	            EnableInt, ClearInt, gpio_set_pin, gpio_get_val,
 *	            in_w / out_w (and, for the temporary example consumer,
 *	            tk_cre_mbx)
 */

#include "subsystem3.h"

/* ------------------------------------------------------------------ */
/*  1. IR sensor driver                                               */
/* ------------------------------------------------------------------ */
/*
 *	Why a hardware timer and not a cyclic handler: the kernel tick is
 *	CNF_TIMER_PERIOD (config/config.h), so a cyclic handler cannot run
 *	every 1 ms unless the whole system ticks at 1 ms.  The RP2040 TIMER
 *	counts microseconds and has four alarms; nothing else uses alarm 3.
 *
 *	Why not the kernel's "adc" driver: it waits on an event flag for
 *	each conversion, which an interrupt handler may not do.  A single
 *	conversion takes 2 us, so the ISR starts it and polls READY.
 *	Subsystem 3 is the only user of the ADC (GP28, design review 2).
 */

/* RP2040 ADC registers (same as device/adc/sysdepend/rp2040/adc_sysdep.h) */
#define ADC_BASE		0x4004C000
#define ADC_CS			(ADC_BASE + 0x00)
#define ADC_RESULT		(ADC_BASE + 0x04)
#define ADC_CS_EN		(1 << 0)
#define ADC_CS_START_ONCE	(1 << 2)
#define ADC_CS_READY		(1 << 8)
#define ADC_CS_AINSEL_POS	12

/* RP2040 TIMER alarm 3 (TIMER_xxx base addresses are in sys/sysdef.h) */
#define BC_ALARM		3
#define BC_ALARM_REG		TIMER_ALARM3
#define BC_ALARM_INTNO		3		/* TIMER_IRQ_3 */
#define BC_ALARM_INTPRI		2		/* same level as the ADC driver */
#define BC_ALARM_MARGIN_US	20		/* alarm must be at least this far ahead */

/* Every variable shared with the ISR is a whole byte or halfword with
   one writer, so each store is atomic (no bit-fields here: the M0+ has
   no atomic read-modify-write).  The ring indices are bytes: 256 is a
   multiple of S3_BC_RING_SIZE, so (UB)(head - tail) is the fill level. */
LOCAL UH	bc_ring[S3_BC_RING_SIZE];	/* S3_EDGE_xxx entries */
LOCAL volatile UB	bc_head;	/* written only by the ISR */
LOCAL volatile UB	bc_tail;	/* written only by the task */
LOCAL volatile UH	bc_samples;	/* ISR only, wraps */
LOCAL volatile UH	bc_overflows;	/* ISR only, wraps */
LOCAL volatile UH	bc_ms;		/* ISR only: ms counter, the edge clock */
LOCAL volatile UH	bc_raw;		/* ISR only: latest sample, 12 bits */
LOCAL volatile UB	bc_edges_on;	/* task only: detector running */
LOCAL S3_BC_DET	bc_det;		/* ISR only while bc_edges_on */
LOCAL UW	bc_next_us;	/* ISR only: when the next alarm is due */

/* Stop the compiler (and the bus) reordering ring accesses */
#define MEMORY_BARRIER()	__asm__ volatile ("dmb" ::: "memory")

/*
 * Barcode ADC ISR, every 1 ms.  Interrupt context: no waiting, no
 * tm_printf.  About 3 us including the conversion.  Only edges go into
 * bc_ring, so a barcode needs ~30 entries, not one per millisecond.
 */
LOCAL void bc_alarm_handler(UINT intno)
{
	UW	now;
	UH	ms;
	UB	head;
	UH	value;
	INT	edge;

	out_w(TIMER_INTR, 1 << BC_ALARM);	/* acknowledge (write 1 to clear) */

	/* Re-arm first, from the previous due time, so the period does not
	   drift.  The alarm fires only on an exact match, so a due time that
	   passes before the write lands would stop sampling for ~71 minutes:
	   if it is that close (or already gone), restart from now instead. */
	bc_next_us += S3_SAMPLE_US;
	now = in_w(TIMER_TIMERAWL);
	if ((W)(bc_next_us - now) < BC_ALARM_MARGIN_US) {
		bc_next_us = now + S3_SAMPLE_US;
	}
	out_w(BC_ALARM_REG, bc_next_us);

	/* One conversion of ADC input 2 (GP28) */
	out_w(ADC_CS, (S3_ADC_CH_IR_RIGHT << ADC_CS_AINSEL_POS) |
		      ADC_CS_EN | ADC_CS_START_ONCE);
	while ((in_w(ADC_CS) & ADC_CS_READY) == 0) ;
	value = (UH)(in_w(ADC_RESULT) & 0x0FFF);
	bc_raw = value;
	ms = (UH)(bc_ms + 1);
	bc_ms = ms;
	bc_samples++;

	edge = bc_edges_on ? s3_bc_detect(&bc_det, value) : S3_EDGE_NONE;
	if (edge != S3_EDGE_NONE) {
		head = bc_head;
		if ((UB)(head - bc_tail) >= S3_BC_RING_SIZE) {
			bc_overflows++;		/* task too slow: lose it */
		} else {
			bc_ring[head & (S3_BC_RING_SIZE - 1)] =
				(UH)((ms & S3_EDGE_TIME_MASK) |
				     ((edge == S3_EDGE_TO_BLACK) ? S3_EDGE_BLACK : 0));
			MEMORY_BARRIER();	/* data before the index */
			bc_head = (UB)(head + 1);
		}
	}

	ClearInt(intno);
}

EXPORT ER drv_ir_init(void)
{
	T_DINT	dint = {0};
	ER	er;

	/* Line sensors: digital inputs */
	gpio_set_pin(S3_PIN_IR_LEFT, GPIO_MODE_IN);
	gpio_set_pin(S3_PIN_IR_CENTER, GPIO_MODE_IN);

	/* Barcode sensor: GP28 as an analogue pin (no digital input, no
	   pulls), and make sure the ADC is powered (the kernel's adc driver
	   normally has done this already at boot) */
	out_w(GPIO_CTRL(S3_PIN_IR_RIGHT), GPIO_CTRL_FUNCSEL_NULL);
	clr_w(GPIO(S3_PIN_IR_RIGHT), GPIO_IE | GPIO_PUE | GPIO_PDE);
	set_w(ADC_CS, ADC_CS_EN);
	while ((in_w(ADC_CS) & ADC_CS_READY) == 0) ;

	bc_head = bc_tail = 0;
	bc_samples = bc_overflows = 0;
	bc_ms = 0;
	bc_edges_on = FALSE;

	/* 1 ms sampling interrupt.  EnableInt acts on the NVIC of the core
	   that calls it: this runs from usermain(), on core 0. */
	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)bc_alarm_handler;
	er = tk_def_int(BC_ALARM_INTNO, &dint);
	if (er < E_OK) return er;

	bc_next_us = in_w(TIMER_TIMERAWL) + S3_SAMPLE_US;
	out_w(BC_ALARM_REG, bc_next_us);	/* writing arms the alarm */
	set_w(TIMER_INTE, 1 << BC_ALARM);
	ClearInt(BC_ALARM_INTNO);
	EnableInt(BC_ALARM_INTNO, BC_ALARM_INTPRI);
	return E_OK;
}

EXPORT UINT drv_ir_left(void)
{
	return gpio_get_val(S3_PIN_IR_LEFT) == S3_IR_BLACK_LEVEL;
}

EXPORT UINT drv_ir_center(void)
{
	return gpio_get_val(S3_PIN_IR_CENTER) == S3_IR_BLACK_LEVEL;
}

EXPORT void drv_ir_start_edges(UH white)
{
	/* The ISR does not touch bc_det while bc_edges_on is FALSE */
	bc_edges_on = FALSE;
	MEMORY_BARRIER();
	s3_bc_det_init(&bc_det, white);
	MEMORY_BARRIER();			/* detector before the switch */
	bc_edges_on = TRUE;
}

EXPORT INT drv_ir_read_edges(UH *buf, INT max)
{
	UB	tail = bc_tail;
	UB	head = bc_head;
	INT	n = 0;

	MEMORY_BARRIER();			/* index before the data */
	while (tail != head && n < max) {
		buf[n++] = bc_ring[tail & (S3_BC_RING_SIZE - 1)];
		tail = (UB)(tail + 1);
	}
	MEMORY_BARRIER();			/* data read before freeing it */
	bc_tail = tail;
	return n;
}

EXPORT UH drv_ir_ms(void)
{
	return bc_ms;
}

EXPORT UH drv_ir_last_raw(void)
{
	return bc_raw;
}

EXPORT UH drv_ir_threshold(void)
{
	return bc_det.thr_on;
}

EXPORT UH drv_ir_sample_count(void)
{
	return bc_samples;
}

EXPORT UH drv_ir_overflow_count(void)
{
	return bc_overflows;
}

/* ------------------------------------------------------------------ */
/*  2. Line tracker (6.3 region A)                                    */
/* ------------------------------------------------------------------ */
/*
 *	The line runs BETWEEN the two sensors (IR_LEFT left of the tape,
 *	IR_CENTER right of it):
 *
 *	  L C   meaning (1 = black)                 state      steering
 *	  0 0   line between the sensors (normal)   ON_LINE    gentle, towards
 *	                                                       the last side
 *	  1 0   robot drifted right: line under L   DRIFTING   left
 *	  0 1   robot drifted left:  line under C   DRIFTING   right
 *	  1 1   two cycles in a row: crossing line  JUNCTION   0  -> VC
 *	  0 0   more than 500 ms                    LINE_LOST  -> VC
 *
 *	Both white is the normal centred case, so "lost" cannot be read from
 *	one sample.  Instead the tracker always drifts gently towards the
 *	side the line was last touched: if the line is between the sensors
 *	the robot weaves and touches an edge again within a few hundred ms,
 *	and if the line slipped outside it is brought back under that sensor.
 *	Only when nothing is touched for > S3_LINE_LOST_MS (well over the
 *	150 ms of NFR21) is the line lost; the steering sign then tells the
 *	VC's LINE_SEARCH which side to sweep first.  LINE_FOUND: either
 *	sensor sees the line again.
 */

EXPORT void s3_line_init(S3_LINE *lt)
{
	lt->state      = S3_LINE_ON_LINE;
	lt->both_black = 0;
	lt->white_ms   = 0;
	lt->pos        = 0;
	lt->steer      = 0;
	lt->gap        = 0;
	lt->side_left  = 0;	/* no side yet: drift right first */
}

/* Coarse line position from the two sensors (FR9): - = left, + = right */
LOCAL B line_position(UINT left, UINT center, UINT side_left)
{
	if (left && !center) return -2;		/* line under the left sensor */
	if (center && !left) return 2;		/* line under the right sensor */
	if (left && center)  return 0;		/* crossing line */
	return side_left ? -1 : 1;		/* between: towards the last side */
}

EXPORT INT s3_line_step(S3_LINE *lt, UINT left, UINT center, UW dt_ms)
{
	INT	ev = S3_LEV_NONE;
	B	pos;
	W	steer;
	UW	white;

	left   = left   ? 1 : 0;
	center = center ? 1 : 0;

	if (left != center) lt->side_left = left;	/* which edge touched */

	if (!left && !center) {
		/* Both white: centred, unless nothing is touched for long */
		lt->both_black = 0;
		white = lt->white_ms + dt_ms;
		lt->white_ms = (UH)((white > 0xFFFF) ? 0xFFFF : white);
		if (lt->state == S3_LINE_DRIFTING || lt->state == S3_LINE_JUNCTION) {
			lt->state = S3_LINE_ON_LINE;	/* back between the sensors */
		}
		if (lt->white_ms > S3_LINE_LOST_MS && lt->state != S3_LINE_LOST) {
			lt->state = S3_LINE_LOST;
			ev = S3_LEV_LINE_LOST;
		}
	} else {
		lt->white_ms = 0;

		if (lt->state == S3_LINE_LOST) {
			/* Either sensor on the line again */
			lt->state      = (left && center) ? S3_LINE_JUNCTION
							  : S3_LINE_DRIFTING;
			lt->both_black = (left && center) ? 1 : 0;
			ev = S3_LEV_LINE_FOUND;
		} else if (left && center) {
			if (lt->both_black < 3) lt->both_black++;
			if (lt->both_black >= S3_JUNCTION_SAMPLES &&
			    lt->state != S3_LINE_JUNCTION) {
				lt->state = S3_LINE_JUNCTION;
				ev = S3_LEV_JUNCTION;
			}
		} else {
			lt->both_black = 0;
			lt->state = S3_LINE_DRIFTING;
		}
	}

	/* PD steering on the coarse position.  None at a junction, where
	   both sensors are on the crossing line. */
	pos = line_position(left, center, lt->side_left);
	if (lt->state == S3_LINE_JUNCTION) {
		steer = 0;
	} else {
		steer = (W)S3_STEER_KP * pos + (W)S3_STEER_KD * (pos - lt->pos);
	}
	if (steer >  S3_STEER_MAX) steer =  S3_STEER_MAX;
	if (steer < -S3_STEER_MAX) steer = -S3_STEER_MAX;

	lt->pos   = pos;
	lt->steer = (B)steer;
	lt->gap   = (!left && !center && lt->white_ms > S3_LINE_GAP_MS &&
		     lt->state != S3_LINE_LOST) ? 1 : 0;
	return ev;
}

/* ------------------------------------------------------------------ */
/*  3. Barcode decoder (6.3 region B)                                 */
/* ------------------------------------------------------------------ */
/*
 *	The Barcode ADC ISR turns samples into edges (s3_bc_detect); the
 *	task feeds the edges to the decoder (s3_bc_edge), whose element
 *	widths are the times between edges:
 *
 *	  WAIT_FOR_BAR --black--> MEASURING --quiet zone / 200 ms--> DECODING
 *	  DECODING --valid--> EMIT_NAV_CMD --0.5 s--> WAIT_FOR_BAR
 *	  DECODING --invalid--> WAIT_FOR_BAR   (BARCODE_ERR, FR31)
 *	  MEASURING --200 ms, incomplete--> WAIT_FOR_BAR   (discard, NFR22)
 *	  MEASURING --bc_ring overflow--> WAIT_FOR_BAR     (discard, 8.3)
 */

/* FR29: payload -> command, as in the project write-up (A..D).  If the
   official track barcodes differ, only this table needs to change. */
EXPORT const S3_BC_ENTRY s3_bc_table[] = {
	{ "A", NAV_LEFT },
	{ "B", NAV_RIGHT },
	{ "C", NAV_STRAIGHT },
	{ "D", NAV_U_TURN },
};
EXPORT const INT s3_bc_table_len = sizeof(s3_bc_table) / sizeof(s3_bc_table[0]);

/* Code 39, bar-space-bar-...-bar, 1 = wide (ISO/IEC 16388) */
EXPORT const S3_C39 s3_code39[] = {
	{'0', 0x034}, {'1', 0x121}, {'2', 0x061}, {'3', 0x160}, {'4', 0x031},
	{'5', 0x130}, {'6', 0x070}, {'7', 0x025}, {'8', 0x124}, {'9', 0x064},
	{'A', 0x109}, {'B', 0x049}, {'C', 0x148}, {'D', 0x019}, {'E', 0x118},
	{'F', 0x058}, {'G', 0x00D}, {'H', 0x10C}, {'I', 0x04C}, {'J', 0x01C},
	{'K', 0x103}, {'L', 0x043}, {'M', 0x142}, {'N', 0x013}, {'O', 0x112},
	{'P', 0x052}, {'Q', 0x007}, {'R', 0x106}, {'S', 0x046}, {'T', 0x016},
	{'U', 0x181}, {'V', 0x0C1}, {'W', 0x1C0}, {'X', 0x091}, {'Y', 0x190},
	{'Z', 0x0D0}, {'-', 0x085}, {'.', 0x184}, {' ', 0x0C4}, {'*', 0x094},
	{'$', 0x0A8}, {'/', 0x0A2}, {'+', 0x08A}, {'%', 0x02A},
};
EXPORT const INT s3_code39_len = sizeof(s3_code39) / sizeof(s3_code39[0]);

/* ------------------------------------------------------------------ */
/*  Decoding                                                          */
/* ------------------------------------------------------------------ */

/* One character from 9 widths: the 3 widest are wide.  Returns the
   character, or -1.  *narrow_max gets the widest narrow element. */
LOCAL INT decode_char(const UB *w, UB *narrow_max)
{
	UH	pattern = 0;
	UB	wide_min = 0xFF, nmax = 0;
	INT	i, k, best;

	for (k = 0; k < 3; k++) {
		best = -1;
		for (i = 0; i < 9; i++) {
			if (pattern & (0x100 >> i)) continue;
			if (best < 0 || w[i] > w[best]) best = i;
		}
		pattern |= 0x100 >> best;
		if (w[best] < wide_min) wide_min = w[best];
	}
	for (i = 0; i < 9; i++) {
		if (!(pattern & (0x100 >> i)) && w[i] > nmax) nmax = w[i];
	}
	*narrow_max = nmax;

	/* Wide and narrow must be clearly apart, or it is noise */
	if ((UH)wide_min * 10 < (UH)nmax * S3_BC_MIN_RATIO_X10) return -1;

	for (i = 0; i < s3_code39_len; i++) {
		if (s3_code39[i].pattern == pattern) return s3_code39[i].ch;
	}
	return -1;
}

/* Decode in the order given.  Returns payload length or -S3_BCERR_xxx */
LOCAL INT decode_dir(const UB *w, INT n, char *text, INT max)
{
	INT	k = (n + 1) / 10;	/* characters incl. start and stop */
	INT	i, c, len = 0;
	UB	nmax;

	if (n < 29 || (n + 1) % 10 != 0) return -S3_BCERR_FRAME;

	for (i = 0; i < k; i++) {
		c = decode_char(&w[10 * i], &nmax);
		if (c < 0) return -S3_BCERR_PATTERN;

		/* the gap before this character must look narrow */
		if (i > 0 && w[10 * i - 1] > 3 * (INT)nmax) return -S3_BCERR_PATTERN;

		if (i == 0 || i == k - 1) {
			if (c != '*') return -S3_BCERR_FRAME;
		} else {
			if (c == '*') return -S3_BCERR_FRAME;
			if (len < max - 1) text[len++] = (char)c;
		}
	}
	if (max > 0) text[len] = '\0';
	return len;
}

EXPORT INT s3_code39_decode(const UB *w, INT n, char *text, INT max)
{
	UB	rev[S3_BC_MAX_ELEMS];
	INT	i, fwd, bwd;

	if (n > S3_BC_MAX_ELEMS) return -S3_BCERR_FRAME;

	fwd = decode_dir(w, n, text, max);
	if (fwd >= 0) return fwd;

	/* Read from the other end ('*' backwards is not '*', so a barcode
	   can only decode one way round) */
	for (i = 0; i < n; i++) rev[i] = w[n - 1 - i];
	bwd = decode_dir(rev, n, text, max);
	if (bwd >= 0) return bwd;

	/* Report the more specific error: right patterns, wrong framing */
	return (fwd < bwd) ? fwd : bwd;
}

EXPORT UB s3_bc_lookup(const char *text)
{
	INT		i;
	const char	*a, *b;

	for (i = 0; i < s3_bc_table_len; i++) {
		a = s3_bc_table[i].text;
		b = text;
		while (*a != '\0' && *a == *b) { a++; b++; }
		if (*a == '\0' && *b == '\0') return s3_bc_table[i].cmd;
	}
	return NAV_NONE;
}

/* ------------------------------------------------------------------ */
/*  Edge detector (runs in the Barcode ADC ISR, every 1 ms)           */
/* ------------------------------------------------------------------ */

LOCAL UH clamp_adc(W v)
{
	return (UH)((v < 0) ? 0 : (v > 4095) ? 4095 : v);
}

LOCAL void det_thresholds(S3_BC_DET *d)
{
	W white = (W)(d->white_q8 >> 8);

	d->thr_on  = clamp_adc(white + S3_BC_DELTA_ON);
	d->thr_off = clamp_adc(white + S3_BC_DELTA_OFF);
}

EXPORT void s3_bc_det_init(S3_BC_DET *d, UH white)
{
	d->white_q8 = (UW)white << 8;
	d->black    = FALSE;
	det_thresholds(d);
}

EXPORT INT s3_bc_detect(S3_BC_DET *d, UH adc)
{
	if (d->black) {
		if (adc < d->thr_off) {
			d->black = FALSE;
			return S3_EDGE_TO_WHITE;
		}
	} else if (adc > d->thr_on) {
		d->black = TRUE;
		return S3_EDGE_TO_BLACK;
	} else if (adc < d->thr_off) {
		/* Clearly white: follow the surface brightness slowly */
		d->white_q8 += (W)(((W)adc << 8) - (W)d->white_q8)
			       >> S3_BC_WHITE_SHIFT;
		det_thresholds(d);
	}
	return S3_EDGE_NONE;
}

/* ------------------------------------------------------------------ */
/*  Decoder state machine (task, one call per edge)                   */
/* ------------------------------------------------------------------ */

/* Times are the ISR's 16-bit ms counter, which wraps every 65 s:
   compare signed differences (valid while they are < 32 s apart) */
LOCAL H ms_after(UH t, UH since)
{
	return (H)(UH)(t - since);
}

EXPORT void s3_bc_init(S3_BC *bc)
{
	bc->state    = S3_BC_WAIT_FOR_BAR;
	bc->black    = 0;
	bc->last_ms  = 0;
	bc->until_ms = 0;
	bc->hold     = 0;
	bc->run_max  = 0;
	bc->n        = 0;
}

LOCAL void clear_result(const S3_BC *bc, S3_BC_RESULT *res)
{
	res->cmd      = NAV_NONE;
	res->ch       = 0;
	res->err      = 0;
	res->elements = bc->n;
}

LOCAL INT end_read(S3_BC *bc, UH now_ms, S3_BC_RESULT *res)
{
	char	text[8];
	INT	len;

	clear_result(bc, res);
	bc->state = S3_BC_WAIT_FOR_BAR;

	if (bc->n < 29 || (bc->n + 1) % 10 != 0) {
		return S3_BEV_DISCARD;	/* partial read (NFR22) */
	}

	bc->state = S3_BC_DECODING;
	len = s3_code39_decode(bc->elem, bc->n, text, sizeof(text));
	if (len < 0) {
		bc->state = S3_BC_WAIT_FOR_BAR;
		res->err = (UB)-len;
		return S3_BEV_BARCODE_ERR;
	}

	res->ch  = (len > 0) ? text[0] : 0;
	res->cmd = s3_bc_lookup(text);
	if (res->cmd == NAV_NONE) {
		bc->state = S3_BC_WAIT_FOR_BAR;
		res->err = S3_BCERR_UNKNOWN;
		return S3_BEV_BARCODE_ERR;
	}

	/* Cool-down: the sensor is ignored for 0.5 s (NFR23) */
	bc->state    = S3_BC_EMIT_NAV_CMD;
	bc->until_ms = (UH)(now_ms + S3_BC_COOLDOWN_MS);
	return S3_BEV_NAV_CMD;
}

EXPORT INT s3_bc_abort(S3_BC *bc, UH now_ms, S3_BC_RESULT *res)
{
	INT	ev = S3_BEV_NONE;

	if (bc->state == S3_BC_EMIT_NAV_CMD) {
		return S3_BEV_NONE;	/* cool-down: edges are ignored anyway */
	}
	if (bc->state == S3_BC_MEASURING) {
		clear_result(bc, res);
		ev = S3_BEV_DISCARD;
	}
	bc->state    = S3_BC_WAIT_FOR_BAR;
	bc->n        = 0;
	bc->hold     = 1;	/* the rest of this barcode is not a start */
	bc->until_ms = (UH)(now_ms + S3_BC_EDGE_TIMEOUT_MS);
	return ev;
}

EXPORT INT s3_bc_edge(S3_BC *bc, BOOL black, UH t_ms, S3_BC_RESULT *res)
{
	UH	w;

	black = black ? 1 : 0;

	switch (bc->state) {
	case S3_BC_WAIT_FOR_BAR:
		if (bc->hold) {
			/* After an abort: every edge restarts the quiet wait */
			bc->until_ms = (UH)(t_ms + S3_BC_EDGE_TIMEOUT_MS);
		} else if (black) {
			/* First bar: start measuring */
			bc->state   = S3_BC_MEASURING;
			bc->black   = 1;
			bc->last_ms = t_ms;
			bc->run_max = 0;
			bc->n       = 0;
		}
		return S3_BEV_NONE;

	case S3_BC_MEASURING:
		/* Two edges to the same level, or more elements than any of
		   our barcodes: edges were lost, so this read is no good */
		if ((UINT)black == bc->black || bc->n >= S3_BC_MAX_ELEMS) {
			return s3_bc_abort(bc, t_ms, res);
		}
		w = (UH)(t_ms - bc->last_ms);	/* the element just finished */
		if (w > S3_BC_WIDTH_MAX) w = S3_BC_WIDTH_MAX;	/* cannot happen */
		bc->elem[bc->n++] = (UB)w;
		if (w > bc->run_max) bc->run_max = (UB)w;
		bc->last_ms = t_ms;
		bc->black   = (UB)black;
		return S3_BEV_NONE;

	default:
		return S3_BEV_NONE;	/* cool-down: ignored (NFR23) */
	}
}

EXPORT INT s3_bc_poll(S3_BC *bc, UH now_ms, S3_BC_RESULT *res)
{
	H	quiet;

	switch (bc->state) {
	case S3_BC_EMIT_NAV_CMD:
		if (ms_after(now_ms, bc->until_ms) >= 0) {
			bc->state = S3_BC_WAIT_FOR_BAR;
		}
		return S3_BEV_NONE;

	case S3_BC_WAIT_FOR_BAR:
		if (bc->hold && ms_after(now_ms, bc->until_ms) >= 0) {
			bc->hold = 0;
		}
		return S3_BEV_NONE;

	case S3_BC_MEASURING:
		/* Edges drained this cycle may be newer than now_ms: then
		   quiet is negative, i.e. not quiet */
		quiet = ms_after(now_ms, bc->last_ms);

		/* Quiet zone after a whole number of characters: done.  An
		   inter-character gap is narrow, so it never gets here. */
		if (!bc->black && bc->n >= 29 && (bc->n + 1) % 10 == 0 &&
		    quiet > 2 * (H)bc->run_max) {
			return end_read(bc, now_ms, res);
		}
		if (quiet >= S3_BC_EDGE_TIMEOUT_MS) {
			return end_read(bc, now_ms, res);
		}
		return S3_BEV_NONE;

	default:
		bc->state = S3_BC_WAIT_FOR_BAR;
		return S3_BEV_NONE;
	}
}

/* ------------------------------------------------------------------ */
/*  4. Packed status word and MQTT telemetry slot (subsystem3.h)      */
/* ------------------------------------------------------------------ */
/*
 *	Explicit masks and shifts, never C bit-fields, so the layout is the
 *	same on every compiler and the dashboard decodes fixed positions.
 *	Every value is clamped to its range before packing; signed fields
 *	are sign-extended when unpacked.
 */

LOCAL INT clamp(INT v, INT lo, INT hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* Sign-extend the low 'bits' bits of v */
LOCAL INT sext(UW v, INT bits)
{
	UW	m = 1u << (bits - 1);

	v &= (m << 1) - 1u;
	return (INT)(v ^ m) - (INT)m;
}

LOCAL UW bit(BOOL b, INT shift)
{
	return (b ? 1u : 0u) << shift;
}

EXPORT UW s3_sts_pack(const S3_STS_FIELDS *f)
{
	UW	w;

	w  = ((UW)f->line_state & S3_STS_2BIT_MASK) << S3_STS_LINE_SHIFT;
	w |= ((UW)f->bc_state & S3_STS_2BIT_MASK) << S3_STS_BC_SHIFT;
	w |= ((UW)f->last_nav & NAV_MASK) << S3_STS_NAV_SHIFT;
	w |= bit(f->nav_valid, S3_STS_NAVOK_SHIFT);
	w |= bit(f->calibrated, S3_STS_CAL_SHIFT);
	w |= bit(f->ir_left, S3_STS_IRL_SHIFT);
	w |= bit(f->ir_center, S3_STS_IRC_SHIFT);
	w |= bit(f->gap, S3_STS_GAP_SHIFT);
	w |= ((UW)clamp(f->line_pos, -2, 2) & S3_STS_POS_MASK)
							<< S3_STS_POS_SHIFT;
	w |= ((UW)clamp(f->steer, -S3_STEER_MAX, S3_STEER_MAX)
					& S3_STS_STEER_MASK) << S3_STS_STEER_SHIFT;
	w |= bit(f->bc_err, S3_STS_ERR_SHIFT);
	w |= bit(f->bc_overflow, S3_STS_OVF_SHIFT);
	return w;
}

EXPORT void s3_sts_unpack(UW w, S3_STS_FIELDS *f)
{
	f->line_state  = (w >> S3_STS_LINE_SHIFT) & S3_STS_2BIT_MASK;
	f->bc_state    = (w >> S3_STS_BC_SHIFT) & S3_STS_2BIT_MASK;
	f->last_nav    = (w >> S3_STS_NAV_SHIFT) & NAV_MASK;
	f->nav_valid   = (w >> S3_STS_NAVOK_SHIFT) & 1u;
	f->calibrated  = (w >> S3_STS_CAL_SHIFT) & 1u;
	f->ir_left     = (w >> S3_STS_IRL_SHIFT) & 1u;
	f->ir_center   = (w >> S3_STS_IRC_SHIFT) & 1u;
	f->gap         = (w >> S3_STS_GAP_SHIFT) & 1u;
	f->line_pos    = sext(w >> S3_STS_POS_SHIFT, 3);
	f->steer       = sext(w >> S3_STS_STEER_SHIFT, 8);
	f->bc_err      = (w >> S3_STS_ERR_SHIFT) & 1u;
	f->bc_overflow = (w >> S3_STS_OVF_SHIFT) & 1u;
}

EXPORT UB s3_tlm_slot(UW sts)
{
	UW	slot;

	slot  = ((sts >> S3_STS_LINE_SHIFT) & S3_STS_2BIT_MASK) << 6;
	slot |= ((sts >> S3_STS_BC_SHIFT) & S3_STS_2BIT_MASK) << 4;
	slot |= ((sts >> S3_STS_NAV_SHIFT) & NAV_MASK) << 2;
	slot |= ((sts >> S3_STS_ERR_SHIFT) & 1u) << 1;
	slot |= (sts >> S3_STS_OVF_SHIFT) & 1u;
	return (UB)slot;
}

/* ------------------------------------------------------------------ */
/*  Objects and start-up                                              */
/* ------------------------------------------------------------------ */

EXPORT ID	tid_line_barcode;
EXPORT ID	cyc_line_barcode;
EXPORT ID	mtx_s3_status;

EXPORT ID		mbx_samples;	/* TEMPORARY example, IF-01 */
EXPORT ID		mtx_stats;	/* TEMPORARY example, IF-04 */
EXPORT APP_STATS	app_stats;

/* TEMPORARY: the example consumer's objects, still used by S2 and S4 */
LOCAL ER example_init(void)
{
	T_CMBX	cmbx = {0};
	T_CMTX	cmtx = {0};

	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}
	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	return (mtx_stats < E_OK) ? mtx_stats : E_OK;
}

EXPORT ER subsystem3_init(void)
{
	T_CMTX	cmtx = {0};
	T_CCYC	ccyc = {0};
	ER	er;

	/* Mutex for s3_status.  TA_INHERIT: the Telemetry Publisher (pri
	   20) holding it is raised to our priority (11) while we wait. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_s3_status = tk_cre_mtx(&cmtx);
	if (mtx_s3_status < E_OK) {
		return mtx_s3_status;
	}

	/* Cyclic handler: wakes the task every 20 ms (NFR8).  Started in
	   subsystem3_start(), once the task exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)line_barcode_cyclic_handler;
	ccyc.cyctim = S3_PERIOD_MS;
	ccyc.cycphs = S3_PERIOD_MS;
	cyc_line_barcode = tk_cre_cyc(&ccyc);
	if (cyc_line_barcode < E_OK) {
		return cyc_line_barcode;
	}

	er = example_init();
	return er;
}

EXPORT ER subsystem3_start(void)
{
	ID	tid;
	ER	er;

	/* Sensors and the 1 ms Barcode ADC ISR.  Done here, in usermain(),
	   so the interrupt is enabled on core 0. */
	er = drv_ir_init();
	if (er < E_OK) {
		return er;
	}

	tid_line_barcode = app_start_task(line_barcode_task,
					  S3_PRI_LINE_BARCODE, 0,
					  "line_barcode_task");
	if (tid_line_barcode < E_OK) {
		return tid_line_barcode;
	}
	er = tk_sta_cyc(cyc_line_barcode);
	if (er < E_OK) {
		return er;
	}

	/* TEMPORARY example consumer, pinned to core 0 as before */
	tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
			     "consumer_task");
	return (tid < E_OK) ? tid : E_OK;
}
