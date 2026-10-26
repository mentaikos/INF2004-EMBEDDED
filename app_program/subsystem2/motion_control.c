/*
 *	motion_control.c
 *	Subsystem 2 -- the Motion Control task (report statechart 6.2).
 *
 *	APIs shown: tk_wai_flg / tk_clr_flg / tk_ref_flg, tk_rcv_mbf (TMO_POL),
 *	            tk_snd_mbf (TMO_POL), tk_set_flg from a cyclic handler
 *
 *	Every 20 ms (NFR14):
 *	  1. encoder snapshot -> per-wheel speed, distance, progress
 *	  2. E-STOP check (flg_safety)                -> E_STOP_BRAKE
 *	  3. latest steering (mbf_steer), all commands (mbf_motion_cmd)
 *	  4. state behaviour -> per-wheel target speed and direction
 *	  5. PID (feed-forward + P + clamped I) -> PWM, stall check
 *	  6. publish the packed status word
 *	With S2_ESTOP_WAKE the task also wakes as soon as S2_FLG_ESTOP is set,
 *	so the brake does not wait for the next 20 ms tick (NFR10).
 *
 *	States (3 bits): STOPPED, SPEED_FOLLOW, MOVE_DISTANCE, TURN
 *	(the ACTIVE superstate), E_STOP_BRAKE, STALL_FAULT.
 *	  - STOP from any state       -> brake -> STOPPED
 *	    (STOP in STALL_FAULT or STOPPED also clears the latched faults:
 *	     this is the VC's "reset")
 *	  - E-STOP bit set            -> brake now, ignore commands, until
 *	                                 the VC clears the bit -> STOPPED
 *	  - PWM on and no encoder pulse for 300 ms (after a 500 ms start-up
 *	    window)                   -> brake, FAULT_STALL, SENSOR_FAULT
 *	  - MOVE(d)/TURN target reached -> brake, MOTION_DONE, STOPPED
 *	  - a new MOVE/TURN pre-empts the current one; SPEED_LIMIT only
 *	    changes the cap.
 *	The task never blocks except in its tick wait (and Buddy 1's
 *	telemetry-slot mutex, a few us every 100 ms, priority inheritance),
 *	and sends to other tasks with TMO_POL only.
 */

#include "subsystem2.h"
#include "motor_driver.h"
#include "encoder.h"
#include "../subsystem1/subsystem1.h"	/* telemetry slot */
#include <tk/syslib.h>

/* ---------------------------------------------------------------------
 *  Private state.  Persistent values use the smallest type that holds
 *  their range (minimum-bit requirement); per-cycle maths uses INT.
 * --------------------------------------------------------------------- */
typedef struct {
	UINT	state   : 3;	/* S2_ST_* */
	UINT	dir_l   : 1;	/* applied direction, 1 = reverse */
	UINT	dir_r   : 1;
	UINT	braking : 1;	/* outputs held at A = B = 0 */
} MOTION_FLAGS;			/* task-private: only this task writes it */

typedef struct {
	H	target;		/* mm/s wanted by the state (magnitude) */
	H	ramp;		/* mm/s after acceleration limit */
	H	meas;		/* mm/s measured (magnitude) */
	H	integ;		/* PID integrator, per mille, clamped */
	UH	duty;		/* per mille actually output */
	UH	since_ms;	/* time since the last pulse (saturates) */
	UH	active_ms;	/* time the output has been on (saturates) */
	UH	settle_ms;	/* braking before a direction change */
	UH	progress;	/* counts since the current MOVE(d)/TURN */
	UB	want_dir;	/* direction wanted by the state */
} WHEEL;

EXPORT volatile UW	g_motion_status;

LOCAL MOTION_FLAGS	mf;
LOCAL WHEEL		wh[2];
LOCAL UB		cmd_type;	/* MOVE_DIST / TURN in progress */
LOCAL UH		goal_counts;	/* average progress that ends it */
LOCAL H			cruise_mm_s;	/* cruise speed of the active state */
LOCAL H			limit_mm_s = S2_MAX_SPEED_CM_S * 10;
LOCAL B			steer;		/* latest steering, + = right */
LOCAL UH		steer_age_ms;
LOCAL UH		dist_mm;	/* 12-bit wrapping distance */
LOCAL H			dist_frac_um;	/* 0..999 um carried, so no drift */
LOCAL UB		still_cycles;	/* for the "stopped" flag */
LOCAL volatile UB	faults;		/* latched FAULT_* bits */
LOCAL volatile UH	evt_drops;	/* events the VC buffer refused */
LOCAL volatile ID	vc_mbf;		/* bound by motion_bind_vc() */
LOCAL volatile ID	fault_flg;
LOCAL UB		tlm_div;	/* telemetry slot divider */

/* ---------------------------------------------------------------------
 *  Small helpers
 * --------------------------------------------------------------------- */
LOCAL INT imin(INT a, INT b) { return a < b ? a : b; }

LOCAL void send_evt(UINT type, UINT payload)
{
	UW	w = S2_EVT_PACK(type, payload);
	ID	mbf = vc_mbf;

	if (mbf <= 0 || tk_snd_mbf(mbf, &w, sizeof(w), TMO_POL) < E_OK) {
		if (evt_drops < 0xFFFFu) evt_drops++;
	}
}

LOCAL void raise_fault(UINT bits)
{
	ID	flg = fault_flg;

	if ((faults & bits) == bits) return;	/* already latched */
	faults |= (UB)bits;
	if (flg > 0) tk_set_flg(flg, bits);
	send_evt(EVT_SENSOR_FAULT, bits);
}

LOCAL void clear_faults(void)
{
	ID	flg = fault_flg;

	if (flg > 0) tk_clr_flg(flg, ~(UINT)(FAULT_STALL | FAULT_ENC));
	faults = 0;
}

LOCAL void wheels_reset(void)
{
	UINT	w;

	for (w = 0; w < 2; w++) {
		wh[w].target = wh[w].ramp = 0;
		wh[w].integ = 0;
		wh[w].duty = 0;
		wh[w].active_ms = 0;
		wh[w].progress = 0;
	}
}

/* Brake both motors now and hold them braked */
LOCAL void brake_now(void)
{
	motor_brake_all();
	mf.braking = 1;
	wheels_reset();
}

LOCAL void enter_state(UINT st)
{
	if (st == S2_ST_STOPPED || st == S2_ST_E_STOP_BRAKE ||
	    st == S2_ST_STALL_FAULT) {
		brake_now();
	}
	mf.state = st;
}

/* Ask for a direction per wheel; a wheel still turning the other way is
   braked for S2_REVERSE_SETTLE_MS first (counts during that time still
   belong to the old direction). */
LOCAL void want_dirs(UINT dl, UINT dr)
{
	UINT	w, cur;

	wh[0].want_dir = (UB)dl;
	wh[1].want_dir = (UB)dr;
	for (w = 0; w < 2; w++) {
		cur = w ? mf.dir_r : mf.dir_l;
		if (cur != wh[w].want_dir) {
			if (wh[w].meas > 0 || wh[w].duty > 0) {
				wh[w].settle_ms = S2_REVERSE_SETTLE_MS;
			} else {
				if (w) mf.dir_r = wh[w].want_dir;
				else   mf.dir_l = wh[w].want_dir;
			}
		}
	}
}

/* Counts -> round(um / um_per_count) */
LOCAL UH um_to_counts(UW um)
{
	UW	c = (um + S2_UM_PER_COUNT / 2) / S2_UM_PER_COUNT;
	return (UH)(c > 0xFFFFu ? 0xFFFFu : c);
}

/* ---------------------------------------------------------------------
 *  Commands
 * --------------------------------------------------------------------- */
LOCAL void start_goal(UINT type, UH counts, INT cruise)
{
	wh[0].progress = wh[1].progress = 0;
	cmd_type    = (UB)type;
	goal_counts = counts;
	cruise_mm_s = (H)cruise;
}

LOCAL void handle_cmd(UW w)
{
	UINT	type = s2_cmd_type(w);
	INT	v = s2_cmd_speed(w) * 10;		/* cm/s -> mm/s */
	INT	a, d;
	UW	um;

	if (type == S2_CMD_STOP) {
		if (mf.state == S2_ST_STALL_FAULT || mf.state == S2_ST_STOPPED) {
			clear_faults();			/* the VC's reset */
		}
		if (mf.state != S2_ST_E_STOP_BRAKE) enter_state(S2_ST_STOPPED);
		return;
	}
	if (type == S2_CMD_SPEED_LIMIT) {		/* internal transition */
		limit_mm_s = (H)v;
		return;
	}
	/* Motion commands: guard [E_STOP clear] and not in a fault */
	if (mf.state == S2_ST_E_STOP_BRAKE || mf.state == S2_ST_STALL_FAULT) {
		return;
	}
	mf.braking = 0;

	switch (type) {
	case S2_CMD_MOVE_FOLLOW:
		if (v == 0) { enter_state(S2_ST_STOPPED); return; }
		cruise_mm_s = (H)v;
		want_dirs(0, 0);
		mf.state = S2_ST_SPEED_FOLLOW;
		break;

	case S2_CMD_MOVE_DIST:
		d = s2_cmd_dist(w);
		if (d == 0) {
			enter_state(S2_ST_STOPPED);
			send_evt(EVT_MOTION_DONE, S2_CMD_MOVE_DIST);
			return;
		}
		if (v == 0) v = S2_DEFAULT_MM_S;
		if (d < 0) v = imin(v, S2_REVERSE_MAX_MM_S);	/* safe reverse */
		um = (UW)(d < 0 ? -d : d) * 10000u;		/* cm -> um */
		start_goal(S2_CMD_MOVE_DIST, um_to_counts(um), v);
		want_dirs(d < 0, d < 0);
		mf.state = S2_ST_MOVE_DISTANCE;
		break;

	case S2_CMD_TURN:
		a = s2_cmd_angle(w);
		if (a == 0) {
			enter_state(S2_ST_STOPPED);
			send_evt(EVT_MOTION_DONE, S2_CMD_TURN);
			return;
		}
		if (v == 0) v = S2_DEFAULT_MM_S;
		/* each wheel travels pi * track * |angle| / 360 */
		um = ((UW)S2_TRACK_MM * 1000u * 355u / 113u) *
		     (UW)(a < 0 ? -a : a) / 360u;
		start_goal(S2_CMD_TURN, um_to_counts(um), v);
		want_dirs(a < 0, a > 0);	/* + = right: left fwd, right rev */
		mf.state = S2_ST_TURN;
		break;

	default:
		break;				/* reserved codes: ignore */
	}
}

/* ---------------------------------------------------------------------
 *  Encoders -> speed, distance, progress, encoder fault
 * --------------------------------------------------------------------- */
LOCAL ENC_SNAPSHOT	enc_prev;
LOCAL UW		cycle_prev_us;

LOCAL void update_speed(UINT w, UINT d, UINT period_us, INT dt_ms)
{
	WHEEL	*p = &wh[w];
	INT	est;

	if (d > 0) {
		p->since_ms = 0;
		if (period_us != ENC_PERIOD_NONE && period_us > 0) {
			est = (INT)(S2_UM_PER_COUNT * 1000u / period_us);
		} else {
			est = (INT)(d * S2_UM_PER_COUNT) / dt_ms;  /* um/ms = mm/s */
		}
		p->meas = (H)((p->meas + est) / 2);
	} else {
		if (p->since_ms <= 0xFFFFu - (UH)dt_ms) p->since_ms += (UH)dt_ms;
		if (p->since_ms >= S2_ZERO_SPEED_MS) {
			p->meas = 0;
		} else {
			/* no edge yet: the speed is at most one count per since_ms */
			est = (INT)(S2_UM_PER_COUNT / p->since_ms);
			if (p->meas > est) p->meas = (H)est;
		}
	}
}

LOCAL void add_distance(INT um)
{
	INT	f = dist_frac_um + um;
	INT	q = f / 1000;

	f -= q * 1000;
	if (f < 0) { f += 1000; q--; }		/* floor division */
	dist_frac_um = (H)f;
	dist_mm = (UH)((dist_mm + q) & S2_STS_DIST_MASK);
}

LOCAL void read_encoders(void)
{
	ENC_SNAPSHOT	s;
	UW		now = in_w(TIMER_TIMERAWL);
	INT		dt_ms = (INT)((now - cycle_prev_us) / 1000u);
	UINT		dl, dr;
	INT		sl, sr;
	BOOL		late;

	cycle_prev_us = now;
	if (dt_ms <= 0) dt_ms = 1;

	encoder_snapshot(&s);
	dl = encoder_delta(enc_prev.cnt.c[ENC_LEFT],  s.cnt.c[ENC_LEFT]);
	dr = encoder_delta(enc_prev.cnt.c[ENC_RIGHT], s.cnt.c[ENC_RIGHT]);
	enc_prev = s;

	/* Part C row 3: a delta above the bound is not believable (noise or
	   a counter wrap): set ENC_FAULT and do not use it. */
	if (dl > S2_ENC_MAX_DELTA || dr > S2_ENC_MAX_DELTA) {
		raise_fault(FAULT_ENC);
		return;
	}

	/* Counts are exact, so distance/progress always use them... */
	sl = mf.dir_l ? -(INT)dl : (INT)dl;
	sr = mf.dir_r ? -(INT)dr : (INT)dr;
	add_distance((sl + sr) * (INT)S2_UM_PER_COUNT / 2);
	if (mf.dir_l == wh[0].want_dir) wh[0].progress += (UH)dl;
	if (mf.dir_r == wh[1].want_dir) wh[1].progress += (UH)dr;

	/* ...but a speed from an over-long interval is not used. */
	late = (dt_ms > S2_ENC_MAX_INTERVAL_MS);
	if (late) {
		raise_fault(FAULT_ENC);
		return;
	}
	update_speed(ENC_LEFT,  dl, s.period_us[ENC_LEFT],  dt_ms);
	update_speed(ENC_RIGHT, dr, s.period_us[ENC_RIGHT], dt_ms);
}

/* ---------------------------------------------------------------------
 *  Per-state targets
 * --------------------------------------------------------------------- */
LOCAL void finish_goal(void)
{
	UINT	type = cmd_type;

	enter_state(S2_ST_STOPPED);
	send_evt(EVT_MOTION_DONE, type);
}

LOCAL void set_targets(void)
{
	INT	v, split, corr, avg, rem;

	switch (mf.state) {
	case S2_ST_SPEED_FOLLOW:
		v = imin(cruise_mm_s, limit_mm_s);
		split = v * steer / S2_STEER_FULL;	/* + steer: left faster */
		wh[0].target = (H)s2_clamp(v + split, 0, S2_TOP_SPEED_MM_S);
		wh[1].target = (H)s2_clamp(v - split, 0, S2_TOP_SPEED_MM_S);
		break;

	case S2_ST_MOVE_DISTANCE:
	case S2_ST_TURN:
		avg = ((INT)wh[0].progress + wh[1].progress) / 2;
		if (avg >= goal_counts) { finish_goal(); return; }
		v = imin(cruise_mm_s, limit_mm_s);
		rem = goal_counts - avg;
		if (rem * 10 <= goal_counts) v = imin(v, S2_SLOW_MM_S);  /* last 10 % */
		/* heading hold: keep both wheels' counts equal */
		corr = ((INT)wh[0].progress - wh[1].progress) * S2_HEADING_K_MM_S;
		corr = s2_clamp(corr, -v / 2, v / 2);
		wh[0].target = (H)(v - corr);
		wh[1].target = (H)(v + corr);
		break;

	default:			/* STOPPED, E_STOP_BRAKE, STALL_FAULT */
		wh[0].target = wh[1].target = 0;
		break;
	}
}

/* ---------------------------------------------------------------------
 *  PID per wheel -> PWM; stall detection
 * --------------------------------------------------------------------- */
LOCAL void drive_wheel(UINT w, INT dt_ms)
{
	WHEEL	*p = &wh[w];
	INT	tgt = p->target, err, ff, out, ni;
	UINT	dir = w ? mf.dir_r : mf.dir_l;

	if (p->settle_ms > 0) {			/* brake before reversing */
		p->settle_ms = (p->settle_ms > (UH)dt_ms && p->meas > 0)
				? (UH)(p->settle_ms - dt_ms) : 0;
		tgt = 0;
		if (p->settle_ms == 0) {
			if (w) mf.dir_r = p->want_dir;
			else   mf.dir_l = p->want_dir;
		}
	}

	/* acceleration limit (stopping is immediate) */
	if (tgt == 0) {
		p->ramp = 0;
	} else if (tgt > p->ramp + S2_ACCEL_MM_S_PER_CYC) {
		p->ramp = (H)(p->ramp + S2_ACCEL_MM_S_PER_CYC);
	} else if (tgt < p->ramp - S2_ACCEL_MM_S_PER_CYC) {
		p->ramp = (H)(p->ramp - S2_ACCEL_MM_S_PER_CYC);
	} else {
		p->ramp = (H)tgt;
	}

	if (p->ramp == 0) {
		p->integ = 0;
		p->duty = 0;
		p->active_ms = 0;
		motor_set(w, 0);			/* A = B = 0: brake */
		return;
	}

	err = p->ramp - p->meas;
	ff  = S2_DEADBAND_PM + p->ramp * S2_KFF_Q8 / 256;
	ni  = s2_clamp(p->integ + err * S2_KI_Q8 / 256, -S2_I_MAX_PM, S2_I_MAX_PM);
	out = ff + err * S2_KP_Q8 / 256 + ni;
	/* anti-windup: only keep the new integral if the output is not
	   saturated in the direction the error pushes */
	if (!((out > MOTOR_DUTY_MAX && err > 0) || (out < 0 && err < 0))) {
		p->integ = (H)ni;
	}
	out = ff + err * S2_KP_Q8 / 256 + p->integ;
	out = s2_clamp(out, 0, MOTOR_DUTY_MAX);

	p->duty = (UH)out;
	motor_set(w, dir ? -out : out);
	if (p->active_ms <= 0xFFFFu - (UH)dt_ms) p->active_ms += (UH)dt_ms;
}

LOCAL BOOL stalled(void)
{
	UINT	w;

	for (w = 0; w < 2; w++) {
		if (wh[w].duty > 0 && wh[w].active_ms >= S2_STALL_ARM_MS &&
		    wh[w].since_ms >= S2_STALL_MS) {
			tm_printf((UB*)"s2: STALL %s wheel: duty %d/1000 for %d ms,"
				  " no encoder pulse for %d ms (counts L=%d R=%d)\n",
				  w ? "RIGHT" : "LEFT", wh[w].duty, wh[w].active_ms,
				  wh[w].since_ms, enc_prev.cnt.c[ENC_LEFT],
				  enc_prev.cnt.c[ENC_RIGHT]);
			return TRUE;
		}
	}
	return FALSE;
}

/* TEMPORARY bring-up aid: once a second while a wheel is driven (or
   turned by hand), print
   the raw 8-bit encoder counters, duty and measured speed.  If duty is
   high but the counts do not move, the wheel is not turning (power) or
   the encoder is not reaching GP0 / GP7.  Set S2_DEBUG to 0 afterwards. */
#ifndef S2_DEBUG
#define S2_DEBUG		1
#endif
#if S2_DEBUG
LOCAL void debug_print(void)
{
	static UB n;
	static UH last;

	if (++n < 1000 / S2_PERIOD_MS) return;
	n = 0;
	/* quiet while stopped, unless a wheel was turned by hand */
	if (wh[0].duty == 0 && wh[1].duty == 0 && last == enc_prev.cnt.both) return;
	last = enc_prev.cnt.both;
	tm_printf((UB*)"s2: st=%d L cnt=%d duty=%d v=%dmm/s | R cnt=%d duty=%d"
		  " v=%dmm/s\n", mf.state,
		  enc_prev.cnt.c[ENC_LEFT], wh[0].duty, wh[0].meas,
		  enc_prev.cnt.c[ENC_RIGHT], wh[1].duty, wh[1].meas);
}
#endif

/* ---------------------------------------------------------------------
 *  Status word (one aligned 32-bit store, single writer)
 * --------------------------------------------------------------------- */
LOCAL void publish_status(void)
{
	INT	sl = wh[0].meas / 10, sr = wh[1].meas / 10;	/* cm/s */
	UINT	stopped;

	if (mf.state == S2_ST_STOPPED && wh[0].meas == 0 && wh[1].meas == 0) {
		if (still_cycles < 2) still_cycles++;
	} else {
		still_cycles = 0;
	}
	stopped = (still_cycles >= 2);

	g_motion_status = s2_sts_pack(mf.dir_l ? -sl : sl, mf.dir_r ? -sr : sr,
				      dist_mm, mf.state, mf.dir_l, mf.dir_r,
				      stopped);

	/* Buddy 1's telemetry slot (IF-S1-01): the tribunal's 26 bits =
	   speed_l 7 | speed_r 7 | distance 12 = status bits [25:0].  Every
	   5th cycle (100 ms) is enough for a 500 ms publisher; the call
	   takes Buddy 1's mutex (priority inheritance) for a few us only. */
	if (++tlm_div >= S2_TLM_EVERY) {
		tlm_div = 0;
		(void)s1_tlm_set_slot(S1_SLOT_B2,
				      (UD)(g_motion_status & S2_TLM_MASK));
	}
}

/* ---------------------------------------------------------------------
 *  One control cycle
 * --------------------------------------------------------------------- */
LOCAL BOOL estop_set(void)
{
	T_RFLG	r;
	return tk_ref_flg(flg_safety, &r) == E_OK && (r.flgptn & S2_FLG_ESTOP);
}

LOCAL void motion_cycle(void)
{
	UW	w;
	B	sb;
	INT	n, dt_ms;
	UW	t0 = cycle_prev_us;

	/* 1. encoders */
	read_encoders();
	dt_ms = (INT)((cycle_prev_us - t0) / 1000u);
	if (dt_ms <= 0) dt_ms = S2_PERIOD_MS;

	/* 2. E-STOP: checked every cycle, before any command */
	if (estop_set()) {
		if (mf.state != S2_ST_E_STOP_BRAKE &&
		    mf.state != S2_ST_STALL_FAULT) {
			enter_state(S2_ST_E_STOP_BRAKE);
		}
	} else if (mf.state == S2_ST_E_STOP_BRAKE) {
		enter_state(S2_ST_STOPPED);	/* VC cleared it on START */
	}

	/* 3. steering: keep only the newest value */
	n = 0;
	while (tk_rcv_mbf(mbf_steer, &sb, TMO_POL) > 0) n++;
	if (n > 0) {
		steer = sb;
		steer_age_ms = 0;
	} else if (steer_age_ms < S2_STEER_TIMEOUT_MS) {
		steer_age_ms += (UH)dt_ms;
	} else {
		steer = 0;			/* line task silent: go straight */
	}

	/*    commands: all of them, in order (a later one pre-empts) */
	while (tk_rcv_mbf(mbf_motion_cmd, &w, TMO_POL) == (INT)sizeof(w)) {
		handle_cmd(w);
	}

	/* 4.-5. targets, PID, PWM */
	set_targets();
	if (mf.braking || mf.state == S2_ST_STOPPED ||
	    mf.state == S2_ST_E_STOP_BRAKE || mf.state == S2_ST_STALL_FAULT) {
		motor_brake_all();		/* re-assert every cycle */
	} else {
		drive_wheel(MOTOR_LEFT,  dt_ms);
		drive_wheel(MOTOR_RIGHT, dt_ms);
		if (stalled()) {
			enter_state(S2_ST_STALL_FAULT);
			raise_fault(FAULT_STALL);
		}
	}

	/* 6. status */
	publish_status();
#if S2_DEBUG
	debug_print();
#endif
}

/* ---------------------------------------------------------------------
 *  Cyclic handler and task
 * --------------------------------------------------------------------- */
EXPORT void motion_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_set_flg(flg_safety, S2_FLG_TICK);	/* interrupt context: no waiting */
}

EXPORT void motion_task(INT stacd, void *exinf)
{
	UINT	ptn, waiptn;
	ER	er;

	(void)stacd; (void)exinf;

	mf.state = S2_ST_STOPPED;
	brake_now();
	encoder_snapshot(&enc_prev);
	cycle_prev_us = in_w(TIMER_TIMERAWL);
	publish_status();

	while (1) {
		waiptn = S2_FLG_TICK;
#if S2_ESTOP_WAKE
		if (mf.state != S2_ST_E_STOP_BRAKE &&
		    mf.state != S2_ST_STALL_FAULT) {
			waiptn |= S2_FLG_ESTOP;	/* wake at once on E-STOP */
		}
#endif
		er = tk_wai_flg(flg_safety, waiptn, TWF_ORW, &ptn, TMO_FEVR);
		if (er < E_OK) {
			CHECK(er, "motion tk_wai_flg");
			brake_now();
			tk_dly_tsk(S2_PERIOD_MS);
			continue;
		}

		if (ptn & S2_FLG_TICK) {
			tk_clr_flg(flg_safety, ~S2_FLG_TICK);
			motion_cycle();
		} else {
			/* woken early by E-STOP: brake in this instant */
			enter_state(S2_ST_E_STOP_BRAKE);
			publish_status();
		}
	}
}

/* ---------------------------------------------------------------------
 *  Public functions
 * --------------------------------------------------------------------- */
EXPORT void motion_bind_vc(ID mbf_vc_evt, ID flg_fault)
{
	vc_mbf    = mbf_vc_evt;
	fault_flg = flg_fault;
}

EXPORT ER motion_send_cmd(UW cmd_word)
{
	return tk_snd_mbf(mbf_motion_cmd, &cmd_word, sizeof(cmd_word), TMO_POL);
}

EXPORT ER motion_send_steer(INT value)
{
	B	b = (B)s2_clamp(value, -127, 127);
	return tk_snd_mbf(mbf_steer, &b, sizeof(b), TMO_POL);
}

EXPORT UW motion_get_status(void)
{
	return g_motion_status;
}

EXPORT UINT motion_get_faults(void)
{
	return faults;
}

EXPORT UH motion_get_evt_drops(void)
{
	return evt_drops;
}
