/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 *
 *	S3-U02..S3-U13 and S3-U16..S3-U18 feed made-up sensor data to the
 *	line tracker, the edge detector and the barcode decoder, so they
 *	need no track.  They use their own S3_LINE / S3_BC_DET / S3_BC,
 *	never the running task's.  S3-U15 checks the running task and the
 *	1 ms ISR; INT-03 the steering link to Motion Control (S2).
 *	S3-U19..S3-U20 check the packed status word and the MQTT slot.  (S3-U14, the S3 MQTT frame, was dropped: Buddy 1 owns
 *	the telemetry payload.)
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#define T_WHITE		300	/* made-up ADC levels */
#define T_BLACK		3000
#define T_NARROW	12	/* ms per narrow element at test speed */
#define T_WIDE		30

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */

LOCAL UW	rnd_state = 12345;

/* Deterministic pseudo-random 0..range-1, so a failure can be re-run */
LOCAL UW rnd(UW range)
{
	rnd_state = rnd_state * 1103515245 + 12345;
	return (rnd_state >> 16) % range;
}

LOCAL UH pattern_of(char c)
{
	INT i;

	for (i = 0; i < s3_code39_len; i++) {
		if (s3_code39[i].ch == c) return s3_code39[i].pattern;
	}
	return 0;
}

/* Element widths for text (incl. the '*'s).  jitter_pct: each width
   varies by up to +-jitter_pct %.  Returns the element count. */
LOCAL INT make_widths(const char *text, UB narrow, UB wide, INT jitter_pct,
		      UB *w)
{
	INT	n = 0, i;
	UH	p;
	UH	base;

	for (; *text != '\0'; text++) {
		if (n > 0) w[n++] = narrow;	/* inter-character gap */
		p = pattern_of(*text);
		for (i = 0; i < 9; i++) {
			w[n++] = (p & (0x100 >> i)) ? wide : narrow;
		}
	}
	for (i = 0; i < n && jitter_pct > 0; i++) {
		base = w[i];
		w[i] = (UB)(base - base * jitter_pct / 100
			    + base * rnd(2 * jitter_pct + 1) / 100);
	}
	return n;
}

/* A made-up ADC trace, fed the way the system does it: the edge
   detector sees every 1 ms sample (as the ISR does) and the decoder gets
   each edge, plus a poll every S3_PERIOD_MS (as the task does).
   Counts the results. */
typedef struct {
	S3_BC_DET	det;
	S3_BC		bc;
	UW		t;		/* ms */
	INT		nav, err, discard;
	S3_BC_RESULT	last;
} FEED;

LOCAL void count(FEED *f, INT ev, const S3_BC_RESULT *res)
{
	switch (ev) {
	case S3_BEV_NAV_CMD:	 f->nav++;     f->last = *res; break;
	case S3_BEV_BARCODE_ERR: f->err++;     f->last = *res; break;
	case S3_BEV_DISCARD:	 f->discard++; f->last = *res; break;
	default: break;
	}
}

LOCAL void feed_init(FEED *f)
{
	s3_bc_det_init(&f->det, T_WHITE);
	s3_bc_init(&f->bc);
	f->t   = 0;
	f->nav = f->err = f->discard = 0;
}

LOCAL void feed_level(FEED *f, UH level, UW ms)
{
	S3_BC_RESULT	res;
	INT		edge;

	while (ms-- > 0) {
		f->t++;
		edge = s3_bc_detect(&f->det, level);
		if (edge != S3_EDGE_NONE) {
			count(f, s3_bc_edge(&f->bc, edge == S3_EDGE_TO_BLACK,
					    (UH)f->t, &res), &res);
		}
		if (f->t % S3_PERIOD_MS == 0) {
			count(f, s3_bc_poll(&f->bc, (UH)f->t, &res), &res);
		}
	}
}

/* Elements (first one black), then quiet white for quiet_ms */
LOCAL void feed_elements(FEED *f, const UB *w, INT n, UW quiet_ms)
{
	INT i;

	for (i = 0; i < n; i++) {
		feed_level(f, (i % 2 == 0) ? T_BLACK : T_WHITE, w[i]);
	}
	feed_level(f, T_WHITE, quiet_ms);
}

/* ------------------------------------------------------------------ */
/*  Barcode tests                                                     */
/* ------------------------------------------------------------------ */

LOCAL BOOL table_is_code39(void)
{
	INT	i, j, k, wide;

	for (i = 0; i < s3_code39_len; i++) {
		for (wide = 0, k = 0; k < 9; k++) {
			if (s3_code39[i].pattern & (1 << k)) wide++;
		}
		if (wide != 3) return FALSE;
		for (j = 0; j < i; j++) {
			if (s3_code39[j].pattern == s3_code39[i].pattern) return FALSE;
		}
	}
	return s3_code39_len == 44;
}

LOCAL INT decode_text(const UB *w, INT n, char *text)
{
	return s3_code39_decode(w, n, text, 8);
}

LOCAL void test_barcode(INT *pass, INT *fail)
{
	UB	w[S3_BC_MAX_ELEMS], r[S3_BC_MAX_ELEMS];
	char	text[8];
	INT	n, i, len, ok;
	FEED	f;
	S3_BC_RESULT	res;
	TEST_INIT();

	/* S3-U02: the Code 39 table is complete and well formed */
	TEST_CHECK("S3-U02", table_is_code39(),
		   "Code 39 table: 44 unique patterns, 3 wide each");

	/* S3-U03: clean "*A*" decodes to LEFT */
	n = make_widths("*A*", T_NARROW, T_WIDE, 0, w);
	len = decode_text(w, n, text);
	TEST_CHECK("S3-U03", n == 29 && len == 1 && text[0] == 'A' &&
		   s3_bc_lookup(text) == NAV_LEFT,
		   "decode *A* -> LEFT (FR29)");

	/* S3-U04: the same barcode read backwards (after a U-turn) */
	for (i = 0; i < n; i++) r[i] = w[n - 1 - i];
	len = decode_text(r, n, text);
	TEST_CHECK("S3-U04", len == 1 && text[0] == 'A',
		   "decode *A* read in reverse");

	/* S3-U05: 20 reads of each command with +-25 % width noise */
	ok = 0;
	for (i = 0; i < 20 * s3_bc_table_len; i++) {
		char src[8] = "*?*";
		src[1] = s3_bc_table[i % s3_bc_table_len].text[0];
		n = make_widths(src, T_NARROW, T_WIDE, 25, w);
		len = decode_text(w, n, text);
		if (len == 1 && text[0] == src[1]) ok++;
	}
	TEST_CHECK("S3-U05", ok == 20 * s3_bc_table_len,
		   "every command decodes 20/20 with +-25% width noise (NFR24)");

	/* S3-U06: a damaged barcode (one wide bar printed narrow) fails */
	n = make_widths("*B*", T_NARROW, T_WIDE, 0, w);
	for (i = 10; i < 19 && w[i] != T_WIDE; i++) ;
	w[i] = T_NARROW;
	TEST_CHECK("S3-U06", decode_text(w, n, text) < 0,
		   "damaged barcode is rejected");

	/* S3-U07: a valid Code 39 barcode not in the table -> BARCODE_ERR,
	   and the decoder is ready for the next one (FR31) */
	feed_init(&f);
	feed_level(&f, T_WHITE, 50);
	n = make_widths("*Q*", T_NARROW, T_WIDE, 0, w);
	feed_elements(&f, w, n, 300);
	TEST_CHECK("S3-U07", f.err == 1 && f.nav == 0 &&
		   f.last.err == S3_BCERR_UNKNOWN &&
		   f.bc.state == S3_BC_WAIT_FOR_BAR,
		   "unknown barcode -> BARCODE_ERR, back to WAIT_FOR_BAR");

	/* S3-U08: ADC trace of "*D*" -> NAV_CMD U-TURN; a second barcode
	   within the 0.5 s cool-down is ignored (NFR23) */
	feed_init(&f);
	feed_level(&f, T_WHITE, 50);
	n = make_widths("*D*", T_NARROW, T_WIDE, 10, w);
	feed_elements(&f, w, n, 100);
	ok = (f.nav == 1 && f.last.cmd == NAV_U_TURN && f.last.ch == 'D');
	n = make_widths("*A*", 4, 10, 0, w);	/* ~170 ms: all inside it */
	feed_elements(&f, w, n, 50);
	TEST_CHECK("S3-U08", ok && f.nav == 1 && f.err == 0 && f.discard == 0,
		   "trace *D* -> U_TURN, repeat within 0.5 s ignored");

	/* S3-U09: after the cool-down the next barcode is read again */
	feed_level(&f, T_WHITE, S3_BC_COOLDOWN_MS);
	n = make_widths("*A*", T_NARROW, T_WIDE, 0, w);
	feed_elements(&f, w, n, 300);
	TEST_CHECK("S3-U09", f.nav == 2 && f.last.cmd == NAV_LEFT,
		   "barcode read again after the cool-down");

	/* S3-U10: a read that stops half way is discarded after 200 ms of
	   no edges, without a BARCODE_ERR (NFR22) */
	feed_init(&f);
	feed_level(&f, T_WHITE, 50);
	n = make_widths("*C*", T_NARROW, T_WIDE, 0, w);
	feed_elements(&f, w, 14, 0);		/* first 14 elements only */
	feed_level(&f, T_WHITE, S3_BC_EDGE_TIMEOUT_MS - 50);
	ok = (f.discard == 0);			/* not yet */
	feed_level(&f, T_WHITE, 100);
	TEST_CHECK("S3-U10", ok && f.discard == 1 && f.err == 0 && f.nav == 0,
		   "partial read discarded after 200 ms (NFR22)");

	/* S3-U16: bc_ring overflow half way through a barcode: it is
	   discarded with no NAV_CMD, the rest of it is not read as a new
	   barcode, and the next barcode is read normally (8.3) */
	feed_init(&f);
	feed_level(&f, T_WHITE, 50);
	n = make_widths("*B*", T_NARROW, T_WIDE, 0, w);
	feed_elements(&f, w, 12, 0);
	count(&f, s3_bc_abort(&f.bc, f.t, &res), &res);
	ok = (f.discard == 1);
	for (i = 12; i < n; i++) {
		feed_level(&f, (i % 2 == 0) ? T_BLACK : T_WHITE, w[i]);
	}
	feed_level(&f, T_WHITE, 300);
	ok = ok && f.nav == 0 && f.err == 0 && f.discard == 1;
	feed_elements(&f, w, n, 300);
	TEST_CHECK("S3-U16", ok && f.nav == 1 && f.last.cmd == NAV_RIGHT,
		   "bc_ring overflow discards that barcode only (8.3)");

	/* S3-U17: edge detector: hysteresis ignores noise around the
	   threshold, and the white baseline follows a slow lighting change
	   so bars are still seen */
	s3_bc_det_init(&f.det, T_WHITE);
	ok = 0;
	for (i = 0; i < 100; i++) {	/* white + 600: between thr_off/on */
		ok += (s3_bc_detect(&f.det, T_WHITE + 600) != S3_EDGE_NONE);
	}
	ok += (s3_bc_detect(&f.det, T_WHITE + 900) != S3_EDGE_TO_BLACK);
	ok += (s3_bc_detect(&f.det, T_WHITE + 600) != S3_EDGE_NONE);
	ok += (s3_bc_detect(&f.det, T_WHITE + 400) != S3_EDGE_TO_WHITE);
	for (i = 0; i < 2000; i++) {	/* surface gets 400 darker */
		s3_bc_detect(&f.det, T_WHITE + 400);
	}
	ok += (s3_bc_detect(&f.det, T_WHITE + 900) != S3_EDGE_NONE);
	ok += (s3_bc_detect(&f.det, T_BLACK) != S3_EDGE_TO_BLACK);
	TEST_CHECK("S3-U17", ok == 0,
		   "edge detector: hysteresis, white baseline follows light");

	*pass += test_pass_;
	*fail += test_fail_;
}

/* ------------------------------------------------------------------ */
/*  Line tracker tests                                                */
/* ------------------------------------------------------------------ */

LOCAL void test_line(INT *pass, INT *fail)
{
	S3_LINE	lt;
	INT	ev1, ev2, ev3, i, lost_at;
	TEST_INIT();

	/* S3-U11: left only -> DRIFTING and steer left; centre -> ON_LINE,
	   steering 0 */
	s3_line_init(&lt);
	s3_line_step(&lt, 0, 1, 30);
	s3_line_step(&lt, 1, 0, 30);
	ev1 = (lt.state == S3_LINE_DRIFTING && lt.steer < 0);
	s3_line_step(&lt, 0, 1, 30);
	TEST_CHECK("S3-U11", ev1 && lt.state == S3_LINE_ON_LINE && lt.steer == 0,
		   "drift left of line -> steer left, back on line -> 0");

	/* S3-U12: both black once is not a junction; twice is, reported
	   once; it ends when the left sensor leaves the crossing line */
	s3_line_init(&lt);
	ev1 = s3_line_step(&lt, 1, 1, 30);
	ev2 = s3_line_step(&lt, 1, 1, 30);
	ev3 = s3_line_step(&lt, 1, 1, 30);
	s3_line_step(&lt, 0, 1, 30);
	TEST_CHECK("S3-U12", ev1 == S3_LEV_NONE && ev2 == S3_LEV_JUNCTION &&
		   ev3 == S3_LEV_NONE && lt.state == S3_LINE_ON_LINE,
		   "JUNCTION after 2 samples, reported once");

	/* S3-U13: both white for 150 ms is a gap, for more is LINE_LOST
	   (NFR21); centre black again -> LINE_FOUND */
	s3_line_init(&lt);
	s3_line_step(&lt, 0, 1, 30);
	lost_at = -1;
	for (i = 1; i <= 8 && lost_at < 0; i++) {
		if (s3_line_step(&lt, 0, 0, 30) == S3_LEV_LINE_LOST) lost_at = i * 30;
	}
	ev1 = s3_line_step(&lt, 0, 1, 30);
	TEST_CHECK("S3-U13", lost_at == 180 && ev1 == S3_LEV_LINE_FOUND &&
		   lt.state == S3_LINE_ON_LINE,
		   "LINE_LOST only after > 150 ms white, then LINE_FOUND");

	/* S3-U18: right-curve mitigation: both white straight from ON_LINE
	   -> gap, limited right steer, still ON_LINE; after a left drift
	   -> steer left (8.3) */
	s3_line_init(&lt);
	s3_line_step(&lt, 0, 1, 20);
	s3_line_step(&lt, 0, 0, 20);
	s3_line_step(&lt, 0, 0, 20);
	ev1 = (lt.gap && lt.state == S3_LINE_ON_LINE && lt.steer > 0 &&
	       lt.steer < 2 * S3_STEER_KP);
	s3_line_init(&lt);
	s3_line_step(&lt, 1, 0, 20);
	s3_line_step(&lt, 0, 0, 20);
	TEST_CHECK("S3-U18", ev1 && lt.gap && lt.steer < 0,
		   "white from ON_LINE -> gap, limited right steer");

	*pass += test_pass_;
	*fail += test_fail_;
}

/* ------------------------------------------------------------------ */
/*  Packed status word and MQTT slot tests (minimum-bit)              */
/* ------------------------------------------------------------------ */

LOCAL BOOL same_fields(const S3_STS_FIELDS *a, const S3_STS_FIELDS *b)
{
	return a->line_state == b->line_state && a->bc_state == b->bc_state &&
	       a->last_nav == b->last_nav && a->nav_valid == b->nav_valid &&
	       a->calibrated == b->calibrated && a->ir_left == b->ir_left &&
	       a->ir_center == b->ir_center && a->gap == b->gap &&
	       a->line_pos == b->line_pos && a->steer == b->steer &&
	       a->bc_err == b->bc_err && a->bc_overflow == b->bc_overflow;
}

LOCAL void test_packing(INT *pass, INT *fail)
{
	S3_STS_FIELDS	in, out;
	UW		w;
	BOOL		ok;
	TEST_INIT();

	/* S3-U19: status word round-trip at the range limits, negative
	   values sign-extended, out-of-range values clamped, bits 31:24
	   unused */
	in.line_state = S3_LINE_LOST;	in.bc_state = S3_BC_EMIT_NAV_CMD;
	in.last_nav = NAV_U_TURN;	in.nav_valid = TRUE;
	in.calibrated = TRUE;		in.ir_left = TRUE;
	in.ir_center = FALSE;		in.gap = TRUE;
	in.line_pos = -2;		in.steer = -127;
	in.bc_err = FALSE;		in.bc_overflow = TRUE;
	w = s3_sts_pack(&in);
	s3_sts_unpack(w, &out);
	ok = same_fields(&in, &out) && (w >> 24) == 0;
	in.line_pos = 2;		in.steer = 127;
	s3_sts_unpack(s3_sts_pack(&in), &out);
	ok = ok && same_fields(&in, &out);
	in.line_pos = 9;		in.steer = -500;	/* clamped */
	s3_sts_unpack(s3_sts_pack(&in), &out);
	TEST_CHECK("S3-U19", ok && out.line_pos == 2 && out.steer == -127,
		   "status word: round-trip, sign-extend, clamp, 24 bits");

	/* S3-U20: MQTT slot = line(2) bc(2) nav(2) err(1) overflow(1),
	   MSB first, as Buddy 1's S1_SLOT_B3 expects */
	in.line_state = S3_LINE_DRIFTING;	/* 01 */
	in.bc_state = S3_BC_MEASURING;		/* 01 */
	in.last_nav = NAV_STRAIGHT;		/* 10 */
	in.bc_err = TRUE;			/* 1  */
	in.bc_overflow = FALSE;			/* 0  */
	TEST_CHECK("S3-U20",
		   s3_tlm_slot(s3_sts_pack(&in)) == 0x5A && S3_TLM_BITS == 8,
		   "MQTT slot: 8 bits, 01 01 10 1 0 = 0x5A");

	*pass += test_pass_;
	*fail += test_fail_;
}

/* ------------------------------------------------------------------ */
/*  Live tests                                                        */
/* ------------------------------------------------------------------ */

LOCAL void test_live(INT *pass, INT *fail)
{
	S3_DIAG		before, after;
	TEST_INIT();

	/* S3-U15: the task runs every 20 ms (NFR8) and the ISR every 1 ms
	   (NFR16) without losing edges (A6) */
	s3_get_diag(&before);
	tk_dly_tsk(1500);
	s3_get_diag(&after);
	TEST_CHECK("S3-U15",
		   (UH)(after.cycles - before.cycles) >= 72 &&
		   (UH)(after.cycles - before.cycles) <= 78 &&
		   (UH)(after.samples - before.samples) >= 1450 &&
		   (UH)(after.samples - before.samples) <= 1550 &&
		   after.overflows == 0,
		   "20 ms task, 1 ms sampling, bc_ring never overflows");

	/* INT-03 (S3 -> S2 steering): Motion Control accepts one steering
	   byte per cycle and drains its mbf_steer, so none is refused */
	TEST_CHECK("INT-03",
		   (UH)(after.steer_sent - before.steer_sent) >= 72 &&
		   after.send_fail == before.send_fail,
		   "steering bytes reach Motion Control every 20 ms");

	*pass += test_pass_;
	*fail += test_fail_;
}

/* ------------------------------------------------------------------ */

LOCAL APP_STATS read_stats(void)
{
	APP_STATS	s;

	tk_loc_mtx(mtx_stats, TMO_FEVR);
	s = app_stats;
	tk_unl_mtx(mtx_stats);
	return s;
}

EXPORT void subsystem3_test(void)
{
	APP_STATS	before, after;

	TEST_INIT();

	/* TEMPORARY example tests, until S2/S4 drop the example consumer.
	   INT-01 (IF-01): samples from subsystem 2 arrive. */
	before = read_stats();
	tk_dly_tsk(2000);
	after = read_stats();
	TEST_CHECK("INT-01", after.received >= before.received + 5,
		   "samples flow from subsystem 2 to subsystem 3");
	TEST_CHECK("S3-U01",
		   after.received > 0 && after.min <= after.max &&
		   after.min >= 0 && after.max <= 100,
		   "min <= max, both within 0..100");

	test_barcode(&test_pass_, &test_fail_);
	test_line(&test_pass_, &test_fail_);
	test_packing(&test_pass_, &test_fail_);
	test_live(&test_pass_, &test_fail_);

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
