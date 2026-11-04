/*
 *	imu_monitor.h
 *	Subsystem 4 -- IMU Monitor logic (statechart 6.4), private to
 *	subsystem 4.  Pure logic, no hardware and no kernel calls: the IMU
 *	task reads the sensor every 10 ms and calls imu_step(), and the tests
 *	(subsystem4_test.c on the robot, host_test/ on a PC) feed it made-up
 *	readings.
 *
 *	The LSM303DLHC has NO gyroscope (Week 6 feedback):
 *	  pitch     = atan2(a_y, a_z)    accelerometer, low-pass filtered
 *	  heading   = atan2(m_y, m_x)    magnetometer, hard-iron corrected
 *	  turn rate = change in heading / time
 *
 *	All maths is integer / fixed point: the RP2040 has no FPU and the
 *	team build does not link libm (no atan2f, sinf, sqrtf), so atan2 is a
 *	16-step CORDIC and sine a 5th-order polynomial (imu_monitor.c).
 *	Angles are in millidegrees (mdeg, 1/1000 deg) inside the logic.
 */

#ifndef IMU_MONITOR_H
#define IMU_MONITOR_H

#include "imu_types.h"
#include "s4_pack.h"

/* ---------------------------------------------------------------------
 *  Tuning constants.  Week 6 design (statechart 6.4, A4) and the track
 *  humps measured on 19 Oct: black 25 mm, yellow 10 mm.  MEASURE = check
 *  on the real robot (testing/subsystem4/TEST_LOG.md).
 * --------------------------------------------------------------------- */
#define IMU_PERIOD_MS		10	/* NFR11 */
#define IMU_CAL_SAMPLES		200	/* 2 s still at start: accel offsets */

/* Low-pass filters, written as 1/DIV (alpha = 0.25 and 0.2) */
#define IMU_PITCH_LPF_DIV	4	/* ~35 ms time constant */
#define IMU_RATE_LPF_DIV	5

/* Hump thresholds.  Week 6 used +5/-5/2 deg.  The yellow hump (10 mm)
   is shorter than the wheelbase, so the body tilts only about
   asin(10/120) = 4.8 deg and +5 deg would never trigger.  +3 deg finds
   both humps in simulation with no false humps on flat track. */
#define IMU_PITCH_START_MDEG	3000	/* MEASURE: log real pitch first */
#define IMU_PITCH_DESC_MDEG	(-3000)
#define IMU_PITCH_LEVEL_MDEG	1500	/* also where height summing starts */
#define IMU_CLIMB_CONFIRM_MS	50
#define IMU_LEVEL_CONFIRM_MS	100
#define IMU_MIN_HUMP_TRAVEL_MM	20	/* shorter = false trigger */
#define IMU_MAX_HUMP_TRAVEL_MM	600	/* never levelled again = abort */
#define IMU_MAX_STEP_MM		200	/* sanity clamp on one 10 ms step */

/* Height = max(sum(dd x sin(pitch)), wheelbase x sin(peak pitch)).  The
   sum alone is only right for humps longer than the car; on our short
   humps the peak pitch comes when the front axle is on the crest and
   the rear axle still on the flat. */
#define IMU_WHEELBASE_MM	120	/* MEASURE: front-to-rear axle distance */

/* Height sum (Version C gap 2 / row 14): 0.01 mm units, clamped to
   +-63 mm = +-6300, which needs 14 bits signed -> int16 (H) */
#define IMU_SUM_MAX_CMM		6300

/* Shock: |a| > 2.5 g, compared squared (no square root).  The sensor
   runs at +-4 g so 2.5 g is inside its range. */
#define IMU_SHOCK_MG		2500
#define IMU_SHOCK_MG2		((W)IMU_SHOCK_MG * IMU_SHOCK_MG)
#define IMU_SHOCK_HOLD_MS	200

/* Faults: 3 failed reads in a row -> FAULT; the task retries every 1 s */
#define IMU_I2C_FAIL_MAX	3
#define IMU_RETRY_MS		1000

/* ---------------------------------------------------------------------
 *  One sensor reading, already in robot axes (X right, Y forward, Z up)
 * --------------------------------------------------------------------- */
typedef struct {
	BOOL	ok;		/* FALSE = the I2C read failed */
	H	ax, ay, az;	/* accelerometer, mg (+-4000 at +-4 g) */
	H	mx, my, mz;	/* magnetometer, raw counts (12 bits) */
} IMU_SAMPLE;

/* Events of one step.  The IMU task turns them into the team's event
   words (app.h EVT_xxx) and flag bits. */
#define IMU_EV_HUMP_START	1
#define IMU_EV_HUMP_END		2	/* height_mm 0 = aborted, not a hump */
#define IMU_EV_SENSOR_FAULT	3
#define IMU_EV_SHOCK		4

typedef struct {
	UB	type;		/* IMU_EV_xxx */
	UB	height_mm;	/* HUMP_END only, 0..31 */
} IMU_EVENT;

#define IMU_MAX_EVENTS		3

/* ---------------------------------------------------------------------
 *  Working state of the IMU task (never shared: only the IMU task
 *  touches it).  Each field is sized for its range, as the minimum-bit
 *  rule asks for persistent variables (Version C gap 2):
 * --------------------------------------------------------------------- */
typedef struct {
	UB	state;			/* S4_ST_xxx */

	/* calibration */
	UB	cal_n;			/* 0..IMU_CAL_SAMPLES (200) */
	W	cal_sum[3];		/* 200 x +-4096 mg -> 21 bits */
	H	acc_off[3];		/* mg */
	H	mag_min[3], mag_max[3];	/* hard-iron tracking, counts */
	UB	mag_seen;
	UB	calibrated;
	UB	heading_valid;

	/* filtered orientation */
	W	pitch_mdeg;		/* +-180000 -> 19 bits */
	W	heading_mdeg;		/* 0..359999 -> 19 bits */
	W	rate_cdps;		/* turn rate, 0.01 deg/s */

	/* hump tracking */
	UW	t_condition_ms;		/* start of the current confirm wait */
	UB	condition_active;
	UB	armed;			/* left level: summing height */
	UB	hump_started;		/* HUMP_START sent */
	H	hump_travel_mm;		/* saturates, limit is 600 */
	H	hump_sum_cmm;		/* 0.01 mm, +-6300 */
	H	hump_sum_peak_cmm;
	W	hump_peak_mdeg;
	UB	last_hump_mm;		/* 0..31 */
	UB	hump_count;		/* saturates at 255, packed to 4 bits */

	/* shock and faults */
	UW	shock_until_ms;
	UB	fail_count;		/* consecutive, 0..3 */
	UB	have_prev;
	UW	prev_ms;
} IMU_MONITOR;

IMPORT void	imu_init(IMU_MONITOR *m);

/* Back to CALIBRATING (after the sensor recovered from FAULT) */
IMPORT void	imu_restart(IMU_MONITOR *m);

/* Sensor not answering at start-up: go straight to FAULT */
IMPORT void	imu_set_fault(IMU_MONITOR *m);

/* One 10 ms cycle.  dd_mm: distance driven since the previous call
   (signed, from Buddy 2's s2_dist_diff()).  now_ms: system time.
   Writes up to IMU_MAX_EVENTS events to out; returns how many. */
IMPORT INT	imu_step(IMU_MONITOR *m, const IMU_SAMPLE *s, INT dd_mm,
			 UW now_ms, IMU_EVENT *out);

/* Current values in the packing units (s4_pack.h) */
IMPORT void	imu_get_fields(const IMU_MONITOR *m, S4_STS_FIELDS *f);

/* Integer maths, exported for the tests */
IMPORT W	imu_atan2_mdeg(W y, W x);	/* -180000..+180000 */
IMPORT W	imu_sin_q14(W mdeg);		/* 16384 = 1.0; |angle| <= 90 deg */

#define IMU_Q14_ONE	16384

#endif /* IMU_MONITOR_H */
