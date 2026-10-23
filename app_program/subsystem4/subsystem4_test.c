/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem4/TEST_LOG.md and the INT-
 *	results in testing/integration/TEST_LOG.md.
 *
 *	S4-U03..S4-U12 feed made-up readings to the IMU logic (imu_monitor.c)
 *	and the packing (s4_pack.h), with their own IMU_MONITOR, never the
 *	running task's: they need no sensor and no track.
 *	S4-U13..S4-U14 check the running IMU task (S4-U14 needs the sensor
 *	plugged into Grove 6 and the robot still since power-up).
 *	INT-05 checks the shock -> E-STOP -> brake link to Motion Control.
 *	Keep the wheels off the table: INT-05 sets and clears E-STOP.
 */

#include "subsystem4.h"
#include "imu_monitor.h"
#include "../subsystem2/subsystem2.h"
#include "../test_report.h"

#if APP_TEST

/* ------------------------------------------------------------------ */
/*  Helpers: a made-up drive over a triangle hump, all integer        */
/* ------------------------------------------------------------------ */

typedef struct {
	INT	starts, ends, shocks, faults;
	INT	height;		/* first HUMP_END height */
} RESULT;

LOCAL UW	t_ms;
LOCAL UW	rnd_state = 12345;

/* Deterministic pseudo-random -amp..+amp */
LOCAL INT rnd(INT amp)
{
	rnd_state = rnd_state * 1103515245u + 12345u;
	return (INT)((rnd_state >> 16) % (UW)(2 * amp + 1)) - amp;
}

LOCAL void count(RESULT *r, const IMU_EVENT *ev, INT n)
{
	INT i;

	for (i = 0; i < n; i++) {
		switch (ev[i].type) {
		case IMU_EV_HUMP_START:	  r->starts++; break;
		case IMU_EV_HUMP_END:
			if (r->ends++ == 0) r->height = ev[i].height_mm;
			break;
		case IMU_EV_SHOCK:	  r->shocks++; break;
		case IMU_EV_SENSOR_FAULT: r->faults++; break;
		default: break;
		}
	}
}

LOCAL void step(IMU_MONITOR *m, H ax, H ay, H az, INT dd, RESULT *r)
{
	IMU_SAMPLE	s;
	IMU_EVENT	ev[IMU_MAX_EVENTS];

	s.ok = TRUE;
	s.ax = ax; s.ay = ay; s.az = az;
	s.mx = 200; s.my = 50; s.mz = -300;
	t_ms += IMU_PERIOD_MS;
	count(r, ev, imu_step(m, &s, dd, t_ms, ev));
}

LOCAL void calibrate(IMU_MONITOR *m)
{
	RESULT	r = {0};
	INT	i;

	imu_init(m);
	for (i = 0; i < IMU_CAL_SAMPLES; i++) step(m, 0, 0, 1000, 0, &r);
}

/* Height of a triangle hump (height h_mm, half-length b_mm, starting at
   x0_mm) under x_mm, in micrometres */
LOCAL W hump_um(W x_mm, W x0_mm, W h_mm, W b_mm)
{
	W	d = x_mm - (x0_mm + b_mm);

	if (d < 0) d = -d;
	return (d < b_mm) ? h_mm * 1000 * (b_mm - d) / b_mm : 0;
}

/* Drive over one hump at 2 mm per 10 ms (200 mm/s).  The body pitch is
   set by the two axles: sin(pitch) = (z_front - z_rear) / wheelbase, so
   ay = 1000 x sin and az = 1000 x cos (about 1000 - ay^2/2000). */
LOCAL void drive_hump(IMU_MONITOR *m, W h_mm, W b_mm, INT shock_step,
		      RESULT *r)
{
	W	x, ay, az;
	INT	k = 0;

	for (x = 0; x < 300 + 2 * b_mm + 600; x += 2, k++) {
		ay = (hump_um(x + IMU_WHEELBASE_MM, 300, h_mm, b_mm) -
		      hump_um(x, 300, h_mm, b_mm)) / IMU_WHEELBASE_MM;
		az = 1000 - ay * ay / 2000;
		if (k == shock_step) az += 3000;
		step(m, 0, (H)ay, (H)az, 2, r);
	}
}

/* ------------------------------------------------------------------ */

LOCAL void test_logic(INT *pass, INT *fail)
{
	IMU_MONITOR	m;
	RESULT		r;
	IMU_SAMPLE	bad;
	IMU_EVENT	ev[IMU_MAX_EVENTS];
	S4_STS_FIELDS	a, b;
	UW		w0, w1, w;
	BOOL		ok;
	INT		i, e;
	TEST_INIT();

	/* S4-U03: status words round trip; negative fields sign-extend;
	   only 31 + 11 = 42 bits used */
	a.state = S4_ST_DESCENDING; a.pitch_dd = -123; a.heading_deg = 359;
	a.rate_dps = -180; a.calibrated = 1; a.hump_mm = 25;
	a.hump_count = 7; a.fail_count = 2;
	w0 = s4_pack_w0(&a);
	w1 = s4_pack_w1(&a);
	s4_sts_unpack(w0, w1, &b);
	TEST_CHECK("S4-U03",
		   b.state == a.state && b.pitch_dd == -123 && b.heading_deg == 359 &&
		   b.rate_dps == -180 && b.calibrated == 1 && b.hump_mm == 25 &&
		   b.hump_count == 7 && b.fail_count == 2 &&
		   (w0 >> 31) == 0 && (w1 >> 11) == 0,
		   "status 42 bits: round trip, sign-extend, unused bits 0");

	/* S4-U04: out-of-range values clamp, never wrap */
	a.pitch_dd = 400; a.heading_deg = 360; a.rate_dps = 999;
	a.hump_mm = 99; a.hump_count = 40; a.fail_count = 9;
	s4_sts_unpack(s4_pack_w0(&a), s4_pack_w1(&a), &b);
	ok = b.pitch_dd == 255 && b.heading_deg == 0 && b.rate_dps == 255 &&
	     b.hump_mm == 31 && b.hump_count == 15 && b.fail_count == 3;
	a.pitch_dd = -400; a.rate_dps = -999;
	s4_sts_unpack(s4_pack_w0(&a), s4_pack_w1(&a), &b);
	TEST_CHECK("S4-U04", ok && b.pitch_dd == -256 && b.rate_dps == -256,
		   "status fields clamp to their ranges");

	/* S4-U05: VC event words (app.h): type in [3:0], 5-bit height */
	w = EVT_PACK(EVT_HUMP_END, S4_EVT_HEIGHT(25));
	ok = EVT_TYPE(w) == EVT_HUMP_END && EVT_PAYLOAD(w) == 25;
	w = EVT_PACK(EVT_HUMP_END, S4_EVT_HEIGHT(40));
	ok = ok && EVT_PAYLOAD(w) == 31;
	w = EVT_PACK(EVT_HUMP_START, 0);
	ok = ok && EVT_TYPE(w) == EVT_HUMP_START && EVT_PAYLOAD(w) == 0;
	w = EVT_PACK(EVT_SENSOR_FAULT, FAULT_IMU);
	TEST_CHECK("S4-U05", ok && EVT_TYPE(w) == EVT_SENSOR_FAULT &&
		   EVT_PAYLOAD(w) == FAULT_IMU,
		   "HUMP_START / HUMP_END(height) / SENSOR_FAULT event words");

	/* S4-U06: integer atan2 and sine (no libm on the robot) */
	ok = TRUE;
	e = imu_atan2_mdeg(1000, 0) - 90000;     ok = ok && e >= -20 && e <= 20;
	e = imu_atan2_mdeg(0, 1000);             ok = ok && e >= -20 && e <= 20;
	e = imu_atan2_mdeg(-1000, -1000) + 135000; ok = ok && e >= -20 && e <= 20;
	e = imu_atan2_mdeg(52, 1000) - 2977;     ok = ok && e >= -20 && e <= 20;
	e = imu_sin_q14(30000) - 8192;           ok = ok && e >= -3 && e <= 3;
	e = imu_sin_q14(-3000) + 857;            ok = ok && e >= -3 && e <= 3;
	e = imu_sin_q14(90000) - 16384;          ok = ok && e >= -80 && e <= 80;
	TEST_CHECK("S4-U06", ok, "integer atan2 within 0.02 deg, sine within 0.5 %");

	/* S4-U07: black hump 25 mm and yellow hump 10 mm (track, 19 Oct):
	   one HUMP_START, one HUMP_END each, height within 3 mm / 2 mm */
	t_ms = 0;
	calibrate(&m);
	r.starts = r.ends = r.shocks = r.faults = r.height = 0;
	drive_hump(&m, 25, 80, -1, &r);
	ok = r.starts == 1 && r.ends == 1 && r.height >= 22 && r.height <= 28;
	tm_printf((UB*)"[TEST]   black hump: %d start, %d end, %d mm\n",
		  r.starts, r.ends, r.height);
	calibrate(&m);
	r.starts = r.ends = r.shocks = r.faults = r.height = 0;
	drive_hump(&m, 10, 90, -1, &r);
	tm_printf((UB*)"[TEST]   yellow hump: %d start, %d end, %d mm\n",
		  r.starts, r.ends, r.height);
	TEST_CHECK("S4-U07", ok && r.starts == 1 && r.ends == 1 &&
		   r.height >= 8 && r.height <= 12,
		   "humps detected, height 25 mm and 10 mm");

	/* S4-U08: 2 m of flat track with +-40 mg noise: no false hump */
	calibrate(&m);
	r.starts = r.ends = r.shocks = r.faults = r.height = 0;
	for (i = 0; i < 1000; i++) {
		step(&m, (H)rnd(40), (H)rnd(40), (H)(1000 + rnd(40)), 2, &r);
	}
	TEST_CHECK("S4-U08", r.starts == 0 && r.ends == 0 && m.state == S4_ST_LEVEL,
		   "no false hump on flat track with noise");

	/* S4-U09: a 3 g spike -> one SHOCK, back to LEVEL after 200 ms */
	calibrate(&m);
	r.starts = r.ends = r.shocks = r.faults = r.height = 0;
	step(&m, 0, 0, 4000, 2, &r);
	ok = (r.shocks == 1 && m.state == S4_ST_SHOCK);
	for (i = 0; i < IMU_SHOCK_HOLD_MS / IMU_PERIOD_MS + 1; i++) {
		step(&m, 0, 0, 1000, 2, &r);
	}
	TEST_CHECK("S4-U09", ok && r.shocks == 1 && m.state == S4_ST_LEVEL,
		   "SHOCK on > 2.5 g, LEVEL again after 200 ms");

	/* S4-U10: shock while climbing a hump -> HUMP_END(0), so the VC
	   leaves HUMP_TRAVERSE (Version C row 9) */
	calibrate(&m);
	r.starts = r.ends = r.shocks = r.faults = r.height = 0;
	drive_hump(&m, 25, 80, 210, &r);	/* x = 420 mm: on the up slope */
	TEST_CHECK("S4-U10", r.shocks == 1 && r.starts >= 1 && r.ends >= 1 &&
		   r.height == 0,
		   "shock during a hump sends HUMP_END(0)");

	/* S4-U11: 3 failed reads -> FAULT + one SENSOR_FAULT; re-init
	   calibrates again */
	calibrate(&m);
	r.starts = r.ends = r.shocks = r.faults = r.height = 0;
	bad.ok = FALSE;
	for (i = 0; i < 5; i++) {
		t_ms += IMU_PERIOD_MS;
		count(&r, ev, imu_step(&m, &bad, 0, t_ms, ev));
	}
	ok = (r.faults == 1 && m.state == S4_ST_FAULT);
	imu_restart(&m);
	TEST_CHECK("S4-U11", ok && m.state == S4_ST_CALIBRATING,
		   "3 failed reads -> FAULT, recovery -> CALIBRATING");

	/* S4-U12: sizes for Week 13 (before / after) */
	tm_printf((UB*)"[TEST]   status unpacked %d B -> packed %d B; event %d B;"
		  " height sum %d B; IMU working state %d B\n",
		  (INT)sizeof(S4_STS_FIELDS), (INT)(2 * sizeof(UW)), (INT)sizeof(UW),
		  (INT)sizeof(m.hump_sum_cmm), (INT)sizeof(IMU_MONITOR));
	TEST_CHECK("S4-U12", sizeof(S4_STS_FIELDS) == 32 && sizeof(m.hump_sum_cmm) == 2,
		   "status 32 B -> 8 B, height sum int16");

	*pass += test_pass_;
	*fail += test_fail_;
}

/* ------------------------------------------------------------------ */
/*  Live tests: the running IMU task                                  */
/* ------------------------------------------------------------------ */

LOCAL void test_live(INT *pass, INT *fail)
{
	S4_DIAG	before, after;
	UW	w0, w1;
	INT	i, st;
	BOOL	ok;
	UB	seq;
	TEST_INIT();

	/* S4-U13: the IMU task runs every 10 ms (NFR11) and one cycle
	   takes well under its period */
	s4_get_diag(&before);
	tk_dly_tsk(1000);
	s4_get_diag(&after);
	tm_printf((UB*)"[TEST]   IMU cycles in 1 s: %d, WCET %u us, overruns %d\n",
		  (INT)(UH)(after.cycles - before.cycles), after.wcet_us,
		  (INT)(UH)(after.overruns - before.overruns));
	TEST_CHECK("S4-U13",
		   (UH)(after.cycles - before.cycles) >= 95 &&
		   (UH)(after.cycles - before.cycles) <= 105 &&
		   after.wcet_us < 1000,
		   "IMU task every 10 ms, WCET < 1 ms");

	/* S4-U14 (needs the sensor): LSM303DLHC found, calibrated, LEVEL */
	s4_get_status(&w0, &w1);
	st = s4_sts_state(w0);
	tm_printf((UB*)"[TEST]   sensor %s, state %d, pitch %d (0.1 deg)\n",
		  after.sensor_found ? "found" : "MISSING", st, s4_sts_pitch_dd(w0));
	TEST_CHECK("S4-U14",
		   after.sensor_found && s4_is_calibrated() && st == S4_ST_LEVEL &&
		   s4_sts_fails(w1) == 0,
		   "sensor found, calibrated, LEVEL (robot still and flat)");

	/* INT-05 (S4 -> S2 flg_safety): a shock sets E-STOP and Motion
	   Control brakes within 50 ms; the shock stays latched until
	   acknowledged.  Then clear E-STOP as the VC would on START. */
	seq = s4_shock_seq();
	s4_test_inject_shock();
	for (i = 0; i < 5 &&
	     s2_sts_state(motion_get_status()) != S2_ST_E_STOP_BRAKE; i++) {
		tk_dly_tsk(S4_PERIOD_MS);
	}
	ok = s2_sts_state(motion_get_status()) == S2_ST_E_STOP_BRAKE &&
	     s4_shock_pending() && s4_shock_seq() == (UB)(seq + 1);
	s4_shock_ack(s4_shock_seq());		/* Telemetry's job */
	ok = ok && !s4_shock_pending();
	tk_clr_flg(flg_safety, ~S2_FLG_ESTOP);	/* VC's job, on START */
	tk_dly_tsk(3 * S2_PERIOD_MS);
	TEST_CHECK("INT-05", ok && s2_sts_state(motion_get_status()) == S2_ST_STOPPED,
		   "shock -> E-STOP -> Motion brakes within 50 ms; latch until acked");

	*pass += test_pass_;
	*fail += test_fail_;
}

/* ------------------------------------------------------------------ */

EXPORT void subsystem4_test(void)
{
	T_RMBF	rmbf;
	UW	before;

	TEST_INIT();

	/* TEMPORARY example tests (IF-03), until subsystem 3 drops its
	   example consumer.  S4-U01: the message buffer accepts the line
	   length; INT-02: summary lines from subsystem 3 arrive. */
	TEST_CHECK("S4-U01",
		   tk_ref_mbf(mbf_summary, &rmbf) == E_OK &&
		   rmbf.maxmsz == S4_SUMMARY_MAX,
		   "message buffer max line = S4_SUMMARY_MAX");
#if S4_EXAMPLE_MONITOR
	before = s4_lines_received;
	tk_dly_tsk(3500);
	TEST_CHECK("INT-02", s4_lines_received >= before + 2,
		   "summary lines flow from subsystem 3 to subsystem 4");
#else
	(void)before;
#endif

	test_logic(&test_pass_, &test_fail_);
	test_live(&test_pass_, &test_fail_);

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
