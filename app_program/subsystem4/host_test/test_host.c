/*
 *	test_host.c
 *	Subsystem 4 -- PC test of the IMU logic (no robot needed).
 *
 *	This folder is two levels below app_program/, so the robot build
 *	does not compile it.  Build and run on a PC (gcc, from this folder):
 *
 *	    gcc -O2 -Wall -Wextra -DS4_HOST_TEST -I.. test_host.c ../imu_monitor.c -lm -o test_host
 *	    ./test_host
 *
 *	It simulates the car (two axles, wheelbase L) driving at constant
 *	speed over the real track humps, makes accelerometer readings in mg
 *	from the body pitch, and checks the hump height that imu_monitor.c
 *	reports.  Floating point is used here only to MAKE the test data;
 *	the code under test is all integer, as on the robot.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "imu_monitor.h"

static int fails = 0;
#define CHECK(cond, ...) do {						\
		printf((cond) ? "  ok:   " : "  FAIL: ");			\
		if (!(cond)) fails++;					\
		printf(__VA_ARGS__); printf("\n");			\
	} while (0)

/* Triangle hump: height h, half-length b, starting at x0 */
typedef struct { float x0, h, b; } hump_t;

static float profile(const hump_t *hp, int nh, float x)
{
	float z = 0;
	for (int i = 0; i < nh; i++) {
		float c = hp[i].x0 + hp[i].b;		/* crest */
		float d = fabsf(x - c);
		if (d < hp[i].b) z += hp[i].h * (1.0f - d / hp[i].b);
	}
	return z;
}

static unsigned rng = 12345;
static float noise(float amp)
{
	rng = rng * 1103515245u + 12345u;
	return amp * (((float)((rng >> 8) & 0xFFFF) / 65535.0f) * 2.0f - 1.0f);
}

typedef struct { int starts, ends, faults, lifts, cause; int heights[8]; int nh; } result_t;

static void count(result_t *r, const IMU_EVENT *ev, int n)
{
	for (int i = 0; i < n; i++) {
		if (ev[i].type == IMU_EV_HUMP_START) r->starts++;
		if (ev[i].type == IMU_EV_HUMP_END) {
			r->ends++;
			if (r->nh < 8) r->heights[r->nh++] = ev[i].height_mm;
		}
		if (ev[i].type == IMU_EV_SENSOR_FAULT) r->faults++;
		if (ev[i].type == IMU_EV_LIFTED) { r->lifts++; r->cause = ev[i].height_mm; }
	}
}

static H mg(float g) { return (H)lroundf(g * 1000.0f); }

static void run(IMU_MONITOR *m, UW *t, float *x_rear, float *dist, long *dist_sent,
		const hump_t *hp, int nh, float speed, float length,
		float noise_g, result_t *r)
{
	const float L = IMU_WHEELBASE_MM;
	int steps = (int)(length / (speed * IMU_PERIOD_MS / 1000.0f));

	for (int k = 0; k < steps; k++) {
		float dx = speed * IMU_PERIOD_MS / 1000.0f;
		float zr0 = profile(hp, nh, *x_rear);
		*x_rear += dx;
		float zr = profile(hp, nh, *x_rear);
		float xf = *x_rear + L, zf, pitch = 0;
		for (int it = 0; it < 4; it++) {	/* keep the axles L apart */
			zf = profile(hp, nh, xf);
			pitch = asinf(fmaxf(-1.0f, fminf(1.0f, (zf - zr) / L)));
			xf = *x_rear + L * cosf(pitch);
		}
		*dist += sqrtf(dx * dx + (zr - zr0) * (zr - zr0));

		/* Buddy 2 reports whole mm: hand over the whole-mm change */
		long now_mm = lroundf(*dist);
		INT dd = (INT)(now_mm - *dist_sent);
		*dist_sent = now_mm;

		IMU_SAMPLE s = { TRUE,
			mg(0.01f + noise(noise_g)),		/* same fixed bias as */
			mg(-0.02f + sinf(pitch) + noise(noise_g)), /* in calibration */
			mg(0.03f + cosf(pitch) + noise(noise_g)),
			200, 50, -300 };

		*t += IMU_PERIOD_MS;
		IMU_EVENT ev[IMU_MAX_EVENTS];
		count(r, ev, imu_step(m, &s, dd, *t, ev));
	}
}

/* 5 Nov: hump under the RIGHT wheel only.  Left wheel and caster stay
   on the floor; the caster is lc mm behind the axle.  The body rolls
   asin(z / track) and pitches asin((z / 2) / lc).  jolt_g: one-sample
   forward jolt as the wheel hits the ramp (0 = none). */
static void run_one_wheel(IMU_MONITOR *m, UW *t, const hump_t *hp, float speed,
			  float length, float noise_g, float lc, float jolt_g,
			  result_t *r)
{
	const float T = IMU_TRACK_MM;
	int steps = (int)(length / (speed * IMU_PERIOD_MS / 1000.0f));
	float x = 0, dist = 0; long sent = 0; int jolted = 0;

	for (int k = 0; k < steps; k++) {
		float dx = speed * IMU_PERIOD_MS / 1000.0f;
		x += dx;
		float z = profile(hp, 1, x);
		float roll = asinf(fminf(1.0f, z / T));
		float pitch = asinf(fminf(1.0f, (z / 2) / lc));
		dist += dx;
		long now_mm = lroundf(dist);
		INT dd = (INT)(now_mm - sent);
		sent = now_mm;

		IMU_SAMPLE s = { TRUE,
			mg(0.01f + sinf(roll) * cosf(pitch) + noise(noise_g)),
			mg(-0.02f + sinf(pitch) + noise(noise_g)),
			mg(0.03f + cosf(roll) * cosf(pitch) + noise(noise_g)),
			200, 50, -300 };
		if (jolt_g > 0 && !jolted && z > 0.5f) {	/* wheel meets the ramp */
			s.ay += mg(jolt_g);
			jolted = 1;
		}
		*t += IMU_PERIOD_MS;
		IMU_EVENT ev[IMU_MAX_EVENTS];
		count(r, ev, imu_step(m, &s, dd, *t, ev));
	}
}

static void calibrate(IMU_MONITOR *m, UW *t)
{
	imu_init(m);
	for (int i = 0; i < IMU_CAL_SAMPLES; i++) {
		IMU_SAMPLE s = { TRUE, mg(0.01f + noise(0.01f)), mg(-0.02f + noise(0.01f)),
				 mg(1.03f + noise(0.01f)), 200, 50, -300 };
		IMU_EVENT ev[IMU_MAX_EVENTS];
		*t += IMU_PERIOD_MS;
		imu_step(m, &s, 0, *t, ev);
	}
}

static void hump_case(const char *name, float h, float half, float speed,
		      float noise_g, int tol)
{
	IMU_MONITOR m; UW t = 0; float x = 0, dist = 0; long sent = 0;
	result_t r = {0};
	hump_t hp = { 300.0f, h, half };

	calibrate(&m, &t);
	run(&m, &t, &x, &dist, &sent, &hp, 1, speed, 300.0f + 2 * half + 600.0f,
	    noise_g, &r);
	printf("%s: h=%.0f mm, half-length %.0f mm (slope %.1f deg), %.0f mm/s, noise %.2f g\n",
	       name, h, half, atanf(h / half) * 57.2958f, speed, noise_g);
	CHECK(r.starts == 1 && r.ends == 1, "one HUMP_START and one HUMP_END (got %d/%d)",
	      r.starts, r.ends);
	if (r.nh) CHECK(abs(r.heights[0] - (int)h) <= tol, "height %d mm vs true %.0f mm (tol %d)",
			r.heights[0], h, tol);
}

static void one_wheel_case(const char *name, float h, float half, float speed,
			   float noise_g, float lc, float jolt_g, int tol)
{
	IMU_MONITOR m; UW t = 0;
	result_t r = {0};
	hump_t hp = { 300.0f, h, half };

	calibrate(&m, &t);
	run_one_wheel(&m, &t, &hp, speed, 300.0f + 2 * half + 600.0f, noise_g,
		      lc, jolt_g, &r);
	printf("%s: h=%.0f mm under the right wheel, caster %.0f mm behind,"
	       " %.0f mm/s, noise %.2f g, jolt %.1f g\n", name, h, lc, speed,
	       noise_g, jolt_g);
	CHECK(r.starts == 1 && r.ends == 1, "one HUMP_START and one HUMP_END (got %d/%d)",
	      r.starts, r.ends);
	if (r.nh) CHECK(abs(r.heights[0] - (int)h) <= tol, "height %d mm vs true %.0f mm (tol %d)",
			r.heights[0], h, tol);
}


int main(void)
{
	printf("== Integer maths ==\n");
	{
		int worst = 0;
		for (int a = -179000; a <= 179000; a += 500) {
			double rad = a / 57295.7795;
			W y = (W)lround(4000 * sin(rad)), x = (W)lround(4000 * cos(rad));
			int e = abs((int)(imu_atan2_mdeg(y, x) - a));
			if (e > worst) worst = e;
		}
		CHECK(worst <= 30, "atan2 within 0.03 deg over the full circle (worst %d mdeg)", worst);
		CHECK(labs(labs((long)imu_atan2_mdeg(0, -1000)) - 180000) <= 10 &&
		      labs((long)imu_atan2_mdeg(1000, 0) - 90000) <= 10,
		      "atan2 at 180 and 90 deg within 0.01 deg (%ld, %ld)",
		      (long)imu_atan2_mdeg(0, -1000), (long)imu_atan2_mdeg(1000, 0));
		worst = 0;
		for (int a = -90000; a <= 90000; a += 250) {
			int e = abs((int)(imu_sin_q14(a) - lround(16384 * sin(a / 57295.7795))));
			if (e > worst) worst = e;
		}
		CHECK(worst <= 80, "sin within 0.5 %% up to 90 deg (worst %d / 16384)", worst);
		worst = 0;
		for (int a = -30000; a <= 30000; a += 100) {
			int e = abs((int)(imu_sin_q14(a) - lround(16384 * sin(a / 57295.7795))));
			if (e > worst) worst = e;
		}
		CHECK(worst <= 3, "sin within 3/16384 up to 30 deg (worst %d)", worst);
	}

	printf("== Hump height on the real track humps ==\n");
	/* Slope lengths are estimates from photos: MEASURE them. */
	hump_case("Black hump", 25.0f, 80.0f, 200.0f, 0.02f, 2);
	hump_case("Yellow hump", 10.0f, 90.0f, 200.0f, 0.02f, 2);
	hump_case("Black hump, slower", 25.0f, 80.0f, 100.0f, 0.02f, 3);
	hump_case("Black hump, noisy", 25.0f, 80.0f, 200.0f, 0.08f, 3);
	hump_case("Black hump, fast", 25.0f, 80.0f, 400.0f, 0.02f, 3);
	hump_case("Yellow hump, fast", 10.0f, 90.0f, 400.0f, 0.02f, 2);
	hump_case("Long ramp (longer than the car)", 25.0f, 300.0f, 200.0f, 0.02f, 3);

	printf("== Flat track: no false humps ==\n");
	{
		IMU_MONITOR m; UW t = 0; float x = 0, dist = 0; long sent = 0; result_t r = {0};
		calibrate(&m, &t);
		run(&m, &t, &x, &dist, &sent, NULL, 0, 200.0f, 2000.0f, 0.08f, &r);
		CHECK(r.starts == 0 && r.ends == 0, "no hump events on 2 m of flat track, 0.08 g noise");
	}

	printf("== Lifted / overturned (12 Nov) ==\n");
	{
		IMU_MONITOR m; UW t = 0; result_t r = {0};
		IMU_EVENT ev[IMU_MAX_EVENTS];
		unsigned rng_saved = rng;	/* later tests keep their noise */
		calibrate(&m, &t);
		/* 2 s of driving with 0.15 g noise, 2-sample jolts of 1 g,
		   a 3 g spike and a sideways knock: no lift */
		for (int k = 0; k < 200; k++) {
			IMU_SAMPLE s = { TRUE, mg(0.01f + noise(0.15f)), mg(-0.02f + noise(0.15f)),
					 mg(1.03f + noise(0.15f)), 200, 50, -300 };
			if (k == 20 || k == 21) s.az += 1000;
			if (k == 60) s.az += 3000;
			if (k == 100) s.ax += 3000;
			t += IMU_PERIOD_MS;
			imu_set_floor(&m, FALSE);
			count(&r, ev, imu_step(&m, &s, 2, t, ev));
		}
		CHECK(r.lifts == 0, "no LIFTED on jolts, a 3 g spike, a knock or 0.15 g noise");
		/* a junction line under both sensors for 0.2 s right after
		   the jolts: no lift */
		for (int k = 0; k < 20; k++) {
			IMU_SAMPLE s = { TRUE, mg(0.01f), mg(-0.02f), mg(1.03f), 200, 50, -300 };
			t += IMU_PERIOD_MS;
			imu_set_floor(&m, TRUE);
			count(&r, ev, imu_step(&m, &s, 2, t, ev));
		}
		CHECK(r.lifts == 0, "no LIFTED crossing a junction line (0.2 s)");
		/* picked up gently: tilted 19 deg, then held level, both line
		   sensors black */
		for (int k = 0; k < 200; k++) {
			float p = (k < 30) ? 0.33f : 0.0f;
			IMU_SAMPLE s = { TRUE, mg(0.01f + noise(0.02f)), mg(-0.02f + p + noise(0.02f)),
					 mg(0.03f + sqrtf(1.0f - p * p) + noise(0.02f)), 200, 50, -300 };
			t += IMU_PERIOD_MS;
			imu_set_floor(&m, TRUE);
			count(&r, ev, imu_step(&m, &s, 0, t, ev));
		}
		CHECK(r.lifts == 1 && r.cause == IMU_LIFT_CAUSE_HELD && m.state == S4_ST_LIFTED,
		      "gentle lift, held level in the air -> LIFTED (no floor)");
		/* upside down for 300 ms */
		calibrate(&m, &t);
		r.lifts = 0;
		for (int k = 0; k < 30; k++) {
			IMU_SAMPLE s = { TRUE, mg(0.01f), mg(-0.02f), mg(0.03f - 1.0f), 200, 50, -300 };
			t += IMU_PERIOD_MS;
			imu_set_floor(&m, FALSE);
			count(&r, ev, imu_step(&m, &s, 0, t, ev));
		}
		CHECK(r.lifts == 1 && r.cause == IMU_LIFT_CAUSE_TILT && m.state == S4_ST_LIFTED,
		      "upside down -> LIFTED (overturned)");
		rng = rng_saved;
	}

	printf("== I2C fault ==\n");
	{
		IMU_MONITOR m; UW t = 0; result_t r = {0};
		IMU_SAMPLE bad = { FALSE, 0, 0, 0, 0, 0, 0 };
		IMU_EVENT ev[IMU_MAX_EVENTS];
		calibrate(&m, &t);
		for (int i = 0; i < 5; i++) { t += 10; count(&r, ev, imu_step(&m, &bad, 0, t, ev)); }
		CHECK(r.faults == 1 && m.state == S4_ST_FAULT, "3 failed reads -> FAULT + one SENSOR_FAULT");
		imu_restart(&m);
		CHECK(m.state == S4_ST_CALIBRATING, "after re-init the IMU calibrates again");
	}

	printf("== Heading and turn rate (28 Oct fix) ==\n");
	{
		/* Still robot, field (200, 50) +- 20 counts of noise: before the
		   fix heading jumped 0..359 and rate hit +-256 deg/s */
		IMU_MONITOR m; UW t = 0; S4_STS_FIELDS f;
		int i, hmin = 999, hmax = -999, rmax = 0, h0 = -1;
		calibrate(&m, &t);
		for (i = 0; i < 500; i++) {
			IMU_SAMPLE s = { TRUE, 0, 0, 1000,
				(H)(200 + lroundf(noise(20.0f))), (H)(50 + lroundf(noise(20.0f))), -300 };
			IMU_EVENT ev[IMU_MAX_EVENTS];
			t += IMU_PERIOD_MS;
			imu_step(&m, &s, 0, t, ev);
			if (i < 50) continue;			/* filters settle */
			imu_get_fields(&m, &f);
			if (h0 < 0) h0 = f.heading_deg;
			{ int d = f.heading_deg - h0;
			  if (d > 180) d -= 360;
			  if (d < -180) d += 360;
			  if (d < hmin) hmin = d;
			  if (d > hmax) hmax = d; }
			if (abs(f.rate_dps) > rmax) rmax = abs(f.rate_dps);
		}
		CHECK(hmax - hmin <= 10, "standing still: heading moves %d deg (<= 10)", hmax - hmin);
		CHECK(rmax <= 5, "standing still: |turn rate| <= 5 deg/s (got %d)", rmax);

		/* Turning on the spot at 90 deg/s, field radius 3000 counts with a
		   hard-iron offset (500, -300): after the first full turn the
		   heading must step 90 deg per second and the rate read ~90 */
		{
			int k, steps = 300, rate_sum = 0, n = 0, h1 = 0, h2 = 0;
			for (k = 0; k < 2 * 400; k++) {		/* 8 s = 2 turns */
				double a = k * IMU_PERIOD_MS * 90.0 / 1000.0 * 3.14159265 / 180.0;
				IMU_SAMPLE s = { TRUE, 0, 0, 1000,
					(H)lround(500 + 3000 * cos(a) + noise(20.0f)),
					(H)lround(-300 + 3000 * sin(a) + noise(20.0f)), -300 };
				IMU_EVENT ev[IMU_MAX_EVENTS];
				t += IMU_PERIOD_MS;
				imu_step(&m, &s, 0, t, ev);
				imu_get_fields(&m, &f);
				if (k >= 400 + 50) { rate_sum += f.rate_dps; n++; }
				if (k == 500) h1 = f.heading_deg;
				if (k == 600) h2 = f.heading_deg;
			}
			(void)steps;
			{ int d = h2 - h1; if (d < 0) d += 360;
			  CHECK(d >= 85 && d <= 95, "turning 90 deg/s: heading moved %d deg in 1 s (85..95)", d); }
			CHECK(n && abs(rate_sum / n - 90) <= 10, "turning 90 deg/s: mean rate %d deg/s (80..100)",
			      n ? rate_sum / n : 0);
		}
	}

	printf("== 5 Nov: hump under ONE wheel (the track ramp) ==\n");
	one_wheel_case("Ramp, right wheel, caster 60 mm", 10.0f, 60.0f, 200.0f, 0.02f, 60.0f, 0, 1);
	one_wheel_case("Ramp, right wheel, caster close", 10.0f, 60.0f, 200.0f, 0.02f, 40.0f, 0, 1);
	one_wheel_case("Ramp, right wheel, jolt at the edge", 10.0f, 60.0f, 200.0f, 0.02f, 40.0f, 0.5f, 1);
	one_wheel_case("Ramp, right wheel, noisy", 10.0f, 60.0f, 200.0f, 0.08f, 40.0f, 0, 2);
	one_wheel_case("Ramp, right wheel, slow", 10.0f, 60.0f, 100.0f, 0.02f, 40.0f, 0, 1);
	one_wheel_case("Black hump, right wheel", 25.0f, 80.0f, 200.0f, 0.02f, 40.0f, 0, 2);

	printf("== Packing ==\n");
	{
		S4_STS_FIELDS a = { S4_ST_DESCENDING, -123, 359, -180, 1, 25, 7, 2 }, b;
		UW w0 = s4_pack_w0(&a), w1 = s4_pack_w1(&a);
		s4_sts_unpack(w0, w1, &b);
		CHECK(b.state == a.state && b.pitch_dd == -123 && b.heading_deg == 359 &&
		      b.rate_dps == -180 && b.calibrated == 1 && b.hump_mm == 25 &&
		      b.hump_count == 7 && b.fail_count == 2, "round trip, negative values sign-extend");
		CHECK((w0 >> 31) == 0 && (w1 >> 11) == 0, "only 31 + 11 = 42 bits used");
		S4_STS_FIELDS big = { S4_ST_LEVEL, 400, 360, 999, 0, 99, 40, 9 };
		s4_sts_unpack(s4_pack_w0(&big), s4_pack_w1(&big), &b);
		CHECK(b.pitch_dd == 255 && b.heading_deg == 0 && b.rate_dps == 255 && b.hump_mm == 31 &&
		      b.hump_count == 15 && b.fail_count == 3, "values clamp at their limits, not wrap");
		big.pitch_dd = -400; big.rate_dps = -999;
		s4_sts_unpack(s4_pack_w0(&big), s4_pack_w1(&big), &b);
		CHECK(b.pitch_dd == -256 && b.rate_dps == -256, "negative limits -25.6 deg and -256 deg/s");
	}

	printf("== Sizes (Week 13 before/after) ==\n");
	{
		IMU_MONITOR m;
		CHECK(sizeof(m.hump_sum_cmm) == 2, "height sum is 2 bytes (was a 4-byte float)");
		printf("  unpacked status S4_STS_FIELDS : %zu bytes\n", sizeof(S4_STS_FIELDS));
		printf("  packed status (2 words)       : %zu bytes, 42 bits used\n", 2 * sizeof(UW));
		printf("  IMU working state IMU_MONITOR : %zu bytes\n", sizeof(IMU_MONITOR));
	}

	printf("\n%s (%d failures)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
	return fails ? 1 : 0;
}