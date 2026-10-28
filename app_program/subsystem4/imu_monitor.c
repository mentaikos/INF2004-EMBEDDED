/*
 *	imu_monitor.c
 *	Subsystem 4 -- IMU Monitor logic (statechart 6.4).  See imu_monitor.h.
 *
 *	  CALIBRATING --200 samples--> LEVEL
 *	  LEVEL --pitch > +3 deg for 50 ms--> CLIMBING        (HUMP_START)
 *	  CLIMBING --pitch < -3 deg--> DESCENDING
 *	  CLIMBING/DESCENDING --|pitch| < 1.5 deg for 100 ms--> LEVEL
 *	                                              (HUMP_END + height)
 *	  any monitoring state --|a| > 2.5 g--> SHOCK --200 ms--> LEVEL
 *	  any state --3 failed reads--> FAULT        (SENSOR_FAULT)
 *	A hump that is cut short after HUMP_START (shock, false trigger,
 *	fault, never levels) sends HUMP_END with height 0, so the Vehicle
 *	Controller always leaves HUMP_TRAVERSE (Version C row 9).
 */

#include "imu_monitor.h"

/* ------------------------------------------------------------------ */
/*  Integer maths                                                     */
/* ------------------------------------------------------------------ */

/* atan(2^-i) in millidegrees, i = 0..15 */
LOCAL const W cordic_mdeg[16] = {
	45000, 26565, 14036, 7125, 3576, 1790, 895, 448,
	224, 112, 56, 28, 14, 7, 3, 2
};

/*
 * atan2(y, x) in millidegrees, by CORDIC vectoring: rotate (x, y) onto
 * the x axis in 16 shrinking steps and add up the step angles.  Error
 * about 0.01 deg.  Inputs are clamped to 16 bits and scaled by 2^14,
 * so the vector (grown x1.65 by CORDIC) stays well inside 32 bits.
 */
EXPORT W imu_atan2_mdeg(W y, W x)
{
	W	ang = 0, xn;
	INT	i;

	if (x == 0 && y == 0) return 0;
	x = s4_clamp(x, -32767, 32767) * 16384;
	y = s4_clamp(y, -32767, 32767) * 16384;

	/* Left half-plane: turn by 180 deg first */
	if (x < 0) {
		x = -x;
		y = -y;
		ang = (y <= 0) ? 180000 : -180000;
	}
	for (i = 0; i < 16; i++) {
		if (y > 0) {
			xn = x + (y >> i);
			y  = y - (x >> i);
			ang += cordic_mdeg[i];
		} else {
			xn = x - (y >> i);
			y  = y + (x >> i);
			ang -= cordic_mdeg[i];
		}
		x = xn;
	}
	if (ang > 180000)   ang -= 360000;
	if (ang <= -180000) ang += 360000;
	return ang;
}

/*
 * sin(angle) in Q14 (16384 = 1.0), angle in millidegrees, clamped to
 * +-90 deg (the pitch of a car on a hump is far below that).
 * t - t^3/6 + t^5/120: error < 0.5 % at 90 deg, < 0.001 % at 25 deg.
 */
EXPORT W imu_sin_q14(W mdeg)
{
	W	t, t2, t3, t5;

	mdeg = s4_clamp(mdeg, -90000, 90000);
	t  = mdeg * 5719 / 20000;	/* mdeg -> radians in Q14 (x 0.28595) */
	t2 = (t * t) >> 14;
	t3 = (t2 * t) >> 14;
	t5 = (t3 * (t2 >> 4)) >> 10;	/* split shift: t3 * t2 overflows 32 bits */
	return t - t3 / 6 + t5 / 120;
}

/* a / b rounded to the nearest integer (b > 0) */
LOCAL W div_round(W a, W b)
{
	return (a >= 0) ? (a + b / 2) / b : -((-a + b / 2) / b);
}

/* Wrap an angle difference into -180000..+180000 mdeg */
LOCAL W wrap180(W d)
{
	while (d > 180000)   d -= 360000;
	while (d <= -180000) d += 360000;
	return d;
}

/* ------------------------------------------------------------------ */
/*  State handling                                                    */
/* ------------------------------------------------------------------ */

LOCAL void reset_hump(IMU_MONITOR *m)
{
	m->condition_active  = 0;
	m->armed             = 0;
	m->hump_started      = 0;
	m->hump_travel_mm    = 0;
	m->hump_sum_cmm      = 0;
	m->hump_sum_peak_cmm = 0;
	m->hump_peak_mdeg    = 0;
}

EXPORT void imu_restart(IMU_MONITOR *m)
{
	INT	i;

	m->state = S4_ST_CALIBRATING;
	m->cal_n = 0;
	for (i = 0; i < 3; i++) m->cal_sum[i] = 0;
	m->calibrated = 0;
	m->fail_count = 0;
	reset_hump(m);
}

EXPORT void imu_init(IMU_MONITOR *m)
{
	UB	*p = (UB *)m;
	UINT	i;

	for (i = 0; i < sizeof(*m); i++) p[i] = 0;
	imu_restart(m);
}

EXPORT void imu_set_fault(IMU_MONITOR *m)
{
	reset_hump(m);
	m->fail_count = IMU_I2C_FAIL_MAX;
	m->state = S4_ST_FAULT;
}

LOCAL void push(IMU_EVENT *out, INT *n, UB type, UB height)
{
	if (*n < IMU_MAX_EVENTS) {
		out[*n].type      = type;
		out[*n].height_mm = height;
		(*n)++;
	}
}

/* Abort a hump in progress.  If HUMP_START was sent the VC is in
   HUMP_TRAVERSE, so release it with HUMP_END(0). */
LOCAL void abort_hump(IMU_MONITOR *m, IMU_EVENT *out, INT *n)
{
	if (m->hump_started) push(out, n, IMU_EV_HUMP_END, 0);
	reset_hump(m);
}

/* Final height, rounded ONCE to whole mm and clamped to 5 bits */
LOCAL UB hump_result_mm(const IMU_MONITOR *m)
{
	W	geo = (IMU_WHEELBASE_MM * imu_sin_q14(m->hump_peak_mdeg)
		       + IMU_Q14_ONE / 2) / IMU_Q14_ONE;
	W	sum = div_round(m->hump_sum_peak_cmm, 100);
	W	h = (sum > geo) ? sum : geo;

	return (UB)s4_clamp(h, 0, S4_HUMP_MAX_MM);
}

LOCAL void update_orientation(IMU_MONITOR *m, const IMU_SAMPLE *s, W dt_ms)
{
	W	ay = (W)s->ay - m->acc_off[1];
	W	az = (W)s->az - m->acc_off[2];
	W	raw, mx, my, hdg, rate;
	H	mv[3];
	INT	i;

	/* pitch, low-pass */
	raw = imu_atan2_mdeg(ay, az);
	m->pitch_mdeg += (raw - m->pitch_mdeg) / IMU_PITCH_LPF_DIV;

	/* hard-iron: track min/max on each axis as the robot turns */
	mv[0] = s->mx; mv[1] = s->my; mv[2] = s->mz;
	for (i = 0; i < 3; i++) {
		if (!m->mag_seen || mv[i] < m->mag_min[i]) m->mag_min[i] = mv[i];
		if (!m->mag_seen || mv[i] > m->mag_max[i]) m->mag_max[i] = mv[i];
	}
	m->mag_seen = 1;

	mx = (W)s->mx - ((W)m->mag_min[0] + m->mag_max[0]) / 2;
	my = (W)s->my - ((W)m->mag_min[1] + m->mag_max[1]) / 2;
	hdg = imu_atan2_mdeg(my, mx);
	if (hdg < 0) hdg += 360000;

	/* turn rate = heading change / time, low-pass */
	if (m->heading_valid && dt_ms > 0) {
		rate = wrap180(hdg - m->heading_mdeg) * 100 / dt_ms;  /* 0.01 deg/s */
		m->rate_cdps += (rate - m->rate_cdps) / IMU_RATE_LPF_DIV;
	}
	m->heading_mdeg  = hdg;
	m->heading_valid = 1;
}

EXPORT INT imu_step(IMU_MONITOR *m, const IMU_SAMPLE *s, INT dd_mm,
		    UW now_ms, IMU_EVENT *out)
{
	INT	n = 0, i;
	W	dt_ms, p, mag2, step;
	INT	travel;
	BOOL	in_hump;

	/* A 10 ms step is a few mm; the clamp only keeps the sum maths
	   inside 32 bits if a wild value ever arrives. */
	dd_mm = (INT)s4_clamp(dd_mm, -IMU_MAX_STEP_MM, IMU_MAX_STEP_MM);

	dt_ms = m->have_prev ? (W)(now_ms - m->prev_ms) : IMU_PERIOD_MS;
	m->prev_ms   = now_ms;
	m->have_prev = 1;

	/* ---------- I2C failures ---------- */
	if (m->state == S4_ST_FAULT) return 0;	/* the task retries init */

	if (!s->ok) {
		if (m->fail_count < IMU_I2C_FAIL_MAX) m->fail_count++;
		if (m->fail_count >= IMU_I2C_FAIL_MAX) {
			abort_hump(m, out, &n);
			m->state = S4_ST_FAULT;
			push(out, &n, IMU_EV_SENSOR_FAULT, 0);
		}
		return n;			/* skip this sample */
	}
	m->fail_count = 0;

	/* ---------- calibration: the robot must be still ---------- */
	if (m->state == S4_ST_CALIBRATING) {
		m->cal_sum[0] += s->ax;
		m->cal_sum[1] += s->ay;
		m->cal_sum[2] += s->az;
		if (++m->cal_n >= IMU_CAL_SAMPLES) {
			for (i = 0; i < 3; i++) {
				m->acc_off[i] = (H)div_round(m->cal_sum[i], m->cal_n);
			}
			m->acc_off[2] -= 1000;	/* keep 1 g on Z */
			m->pitch_mdeg = 0;
			m->calibrated = 1;
			m->state      = S4_ST_LEVEL;
			reset_hump(m);
		}
		return n;
	}

	update_orientation(m, s, dt_ms);

	/* ---------- shock: unfiltered |a|, any monitoring state ---------- */
	mag2 = (W)s->ax * s->ax + (W)s->ay * s->ay + (W)s->az * s->az;
	if (mag2 > IMU_SHOCK_MG2 && m->state != S4_ST_SHOCK) {
		abort_hump(m, out, &n);
		m->state = S4_ST_SHOCK;
		m->shock_until_ms = now_ms + IMU_SHOCK_HOLD_MS;
		push(out, &n, IMU_EV_SHOCK, 0);	/* task sets E_STOP */
		return n;
	}
	if (m->state == S4_ST_SHOCK) {
		if ((W)(now_ms - m->shock_until_ms) >= 0) {
			m->state = S4_ST_LEVEL;
			reset_hump(m);
		}
		return n;
	}

	/* ---------- hump tracking: sum from the first tilt ---------- */
	p = m->pitch_mdeg;

	/* Start summing as soon as the robot leaves level, not only at the
	   start threshold, so the first part of a ramp is counted. */
	if (m->state == S4_ST_LEVEL && !m->condition_active) {
		if (p > IMU_PITCH_LEVEL_MDEG) {
			m->armed = 1;
		} else if (m->armed) {
			reset_hump(m);		/* tilted a little, came back */
		}
		if (m->armed && m->hump_travel_mm > IMU_MAX_HUMP_TRAVEL_MM) {
			reset_hump(m);		/* constant small tilt: not a hump */
		}
	}
	in_hump = (m->state == S4_ST_CLIMBING || m->state == S4_ST_DESCENDING ||
		   (m->state == S4_ST_LEVEL && m->armed));
	if (in_hump) {
		travel = m->hump_travel_mm + (dd_mm < 0 ? -dd_mm : dd_mm);
		m->hump_travel_mm = (H)s4_clamp(travel, 0, 32767);

		/* this step's rise in 0.01 mm, clamped to +-63 mm */
		step = div_round((W)dd_mm * imu_sin_q14(p) * 100, IMU_Q14_ONE);
		m->hump_sum_cmm = (H)s4_clamp((W)m->hump_sum_cmm + step,
					      -IMU_SUM_MAX_CMM, IMU_SUM_MAX_CMM);
		if (m->hump_sum_cmm > m->hump_sum_peak_cmm) {
			m->hump_sum_peak_cmm = m->hump_sum_cmm;
		}
		if (p > m->hump_peak_mdeg) m->hump_peak_mdeg = p;
	}

	switch (m->state) {
	case S4_ST_LEVEL:
		if (p > IMU_PITCH_START_MDEG) {
			if (!m->condition_active) {
				m->condition_active = 1;
				m->t_condition_ms   = now_ms;
			} else if (now_ms - m->t_condition_ms >= IMU_CLIMB_CONFIRM_MS) {
				m->state            = S4_ST_CLIMBING;
				m->condition_active = 0;
				m->hump_started     = 1;
				push(out, &n, IMU_EV_HUMP_START, 0);
			}
		} else if (m->condition_active) {
			m->condition_active = 0;	/* spike shorter than 50 ms */
		}
		break;

	case S4_ST_CLIMBING:
	case S4_ST_DESCENDING:
		if (p < IMU_PITCH_DESC_MDEG) m->state = S4_ST_DESCENDING;

		if (p < IMU_PITCH_LEVEL_MDEG && p > -IMU_PITCH_LEVEL_MDEG) {
			if (!m->condition_active) {
				m->condition_active = 1;
				m->t_condition_ms   = now_ms;
			} else if (now_ms - m->t_condition_ms >= IMU_LEVEL_CONFIRM_MS) {
				if (m->hump_travel_mm < IMU_MIN_HUMP_TRAVEL_MM) {
					abort_hump(m, out, &n);	/* false trigger */
				} else {
					m->last_hump_mm = hump_result_mm(m);
					m->last_peak_mdeg = m->hump_peak_mdeg;
					if (m->hump_count < 255) m->hump_count++;
					push(out, &n, IMU_EV_HUMP_END, m->last_hump_mm);
					reset_hump(m);
				}
				m->state = S4_ST_LEVEL;
			}
		} else {
			m->condition_active = 0;
		}

		if (m->state != S4_ST_LEVEL &&
		    m->hump_travel_mm > IMU_MAX_HUMP_TRAVEL_MM) {
			abort_hump(m, out, &n);
			m->state = S4_ST_LEVEL;
		}
		break;

	default:
		break;
	}
	return n;
}

EXPORT void imu_get_fields(const IMU_MONITOR *m, S4_STS_FIELDS *f)
{
	f->state       = m->state;
	f->pitch_dd    = (INT)div_round(m->pitch_mdeg, 100);
	f->heading_deg = (INT)(div_round(m->heading_mdeg, 1000) % 360);
	f->rate_dps    = (INT)div_round(m->rate_cdps, 100);
	f->calibrated  = m->calibrated;
	f->hump_mm     = m->last_hump_mm;
	f->hump_count  = m->hump_count;
	f->fail_count  = m->fail_count;
}
