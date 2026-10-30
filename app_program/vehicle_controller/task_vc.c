/*
 *	task_vc.c
 *	Vehicle Controller task -- the system statechart (design 7.3, FR1).
 *
 *	APIs shown: tk_rcv_mbf with a time-out (event-driven task with a
 *	            20 ms housekeeping tick), tk_ref_flg / tk_clr_flg,
 *	            tk_set_flg, tk_get_tim
 *
 *	Every loop the task waits up to VC_TICK_MS for ONE event word from
 *	mbf_vc_evt, handles it, then does the time-based checks (E-STOP,
 *	time-outs, waiting for "stopped", relaying line samples to Obstacle
 *	Scan) and publishes g_vc_status.
 *
 *	Ownership rules kept here:
 *	  - only the VC sends motion commands (motion_send_cmd, TMO_POL);
 *	    Line & Barcode only sends steering, which Motion applies only
 *	    in SPEED_FOLLOW;
 *	  - only the VC clears S2_FLG_ESTOP, and only on a new START with the
 *	    hazard gone (NFR9);
 *	  - the VC never touches a pin, a sensor or the network.
 *	The E-STOP itself never waits for the VC: IMU, Obstacle Scan and the
 *	Command Handler set S2_FLG_ESTOP and Motion brakes at once (NFR10).
 */

#include "vehicle_controller.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem1/subsystem1.h"	/* heartbeat mission state */

EXPORT volatile UW	g_vc_status;

/* Task-private state: only this task writes it, so bit-fields are safe
   (minimum-bit rule: each field sized for its range). */
LOCAL struct {
	UB	state : 4;	/* VC_ST_xxx */
	UB	nav : 2;	/* pending NAV_xxx */
	UB	nav_valid : 1;
	UB	lost_in_hump : 1;	/* LINE_LOST while the IMU saw a hump */
	UB	reason : 4;	/* VC_R_xxx */
	UB	junctions : 4;	/* wraps */
	UB	peak_hump : 5;	/* mm, 0..31 */
	UB	rescans : 2;
	UB	phase : 2;	/* sub-step inside OBSTACLE_SCAN / REJOIN_LINE */
	UB	side_right : 1;	/* bypass side / search side */
	UB	scan_sent : 1;	/* SCAN_REQ given for this stop */
	UB	align_white : 1; /* PH_REALIGN: sensors left the line once */
	B	lat_cm;		/* bypass sideways distance */
	UB	search_i;	/* LINE_SEARCH sweep number */
	UB	faults;		/* SENSOR_FAULT bits of the last fault */
	UH	t_state;	/* ms when the current state / phase began */
	UH	t_junction;	/* ms of the last junction acted on */
} vc;

/* MOVE(d)/TURN script: one command at a time, next on MOTION_DONE */
LOCAL UW	script[VC_SCRIPT_MAX];
LOCAL UB	script_len, script_idx;
LOCAL UH	t_step;

LOCAL UB		bench_move;	/* VC_BENCH_MOVE_CM run in progress */
LOCAL S5_PROFILE	plan_profile;	/* profile of the approved plan */
LOCAL VC_DIAG		dg;

/* Phases inside OBSTACLE_SCAN */
#define PH_WAIT_STOPPED		0
#define PH_WAIT_PROFILE		1
#define PH_WAIT_ACCEPT		2
#define PH_REVERSING		3
/* Phases inside REJOIN_LINE */
#define PH_APPROACH		0
#define PH_REALIGN		1
#define PH_CONFIRM		2

/* Bypass script step from which a line under the IR sensors is followed
   at once (vc_bypass_script: 0 turn out, 1 sideways, 2 turn back,
   3 past the obstacle, 4 turn towards the line).  Not earlier: on steps
   0..3 the robot is still beside the obstacle. */
#define VC_BYPASS_FOLLOW_STEP	3	/* 30 Oct: was 4.  On a curved track the
					   line bends towards the bypass side and
					   was crossed (and ignored) while driving
					   past the obstacle (step 3), video
					   IMG_0771.  Once the robot is straight
					   again beside the obstacle, a line under
					   the sensors cannot lead into it. */

LOCAL const char *const st_name[] = {
	"INIT", "IDLE", "STARTING", "LINE_FOLLOW", "EXECUTE_NAV_CMD",
	"HUMP_TRAVERSE", "LINE_SEARCH", "OBSTACLE_SCAN", "BYPASS_OBSTACLE",
	"REJOIN_LINE", "E_STOPPED", "FAULT", "COMPLETE"
};

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */
LOCAL UH now_ms(void)
{
	SYSTIM	t;
	tk_get_tim(&t);
	return (UH)t.lo;		/* 16-bit wrap: all waits < 32 s */
}

LOCAL UH since(UH t0) { return (UH)(now_ms() - t0); }

LOCAL void cmd(UW w)
{
	if (motion_send_cmd(w) < E_OK) dg.cmd_fail++;
}

LOCAL void cmd_stop(void)
{
	cmd(S2_CMD_PACK(S2_CMD_STOP, 0, 0, 0));
}

LOCAL void cmd_follow(INT cm_s)
{
	cmd(S2_CMD_PACK(S2_CMD_MOVE_FOLLOW, cm_s, 0, 0));
}

LOCAL void cmd_limit(INT cm_s)
{
	cmd(S2_CMD_PACK(S2_CMD_SPEED_LIMIT, cm_s, 0, 0));
}

LOCAL BOOL estop_set(void)
{
	T_RFLG	r;
	return tk_ref_flg(flg_safety, &r) != E_OK || (r.flgptn & S2_FLG_ESTOP);
}

LOCAL UW s5_status(void)
{
#if VC_USE_ULTRASONIC
	S5_SNAPSHOT	s;
	return (s5_get_status(&s) == E_OK) ? s.status : (UW)S5_ST_SENSOR_FAULT;
#else
	/* Bench mode: Obstacle Scan not running.  Report FORWARD_WATCH with
	   the range "beyond 100 cm", so START and STARTING never wait for it
	   and the VC never enters OBSTACLE_SCAN. */
	return (UW)S5_ST_FORWARD_WATCH | ((UW)S5_RANGE_FAR << 11);
#endif
}

/* VC state -> Buddy 1's 3-bit heartbeat state (subsystem1.h):
   0 INIT, 1 IDLE, 2 RUNNING, 3 E_STOPPED, 4 FAULT, 5 COMPLETE */
LOCAL UINT vc_mission_state(UINT st)
{
	if (VC_IS_RUNNING(st)) return 2;
	switch (st) {
	case VC_ST_INIT:	return 0;
	case VC_ST_E_STOPPED:	return 3;
	case VC_ST_FAULT:	return 4;
	case VC_ST_COMPLETE:	return 5;
	default:		return 1;	/* IDLE */
	}
}

LOCAL void set_state(UINT st)
{
	if (vc.state != st) {
		tm_printf((UB*)"vc: %s -> %s\n", st_name[vc.state], st_name[st]);
	}
	vc.state = (UB)st;
	vc.t_state = now_ms();
	s1_set_mission_state(vc_mission_state(st));	/* Buddy 1 heartbeat */
}

LOCAL void script_clear(void)
{
	script_len = script_idx = 0;
}

LOCAL void script_run(const UW *c, INT n)
{
	INT	i;

	for (i = 0; i < n && i < VC_SCRIPT_MAX; i++) script[i] = c[i];
	script_len = (UB)i;
	script_idx = 0;
	t_step = now_ms();
	if (script_len > 0) cmd(script[0]);
}

/* Line & Barcode currently sees the line (calibrated, not lost, no gap) */
LOCAL BOOL line_seen(void)
{
	S3_STS_FIELDS	f;

	s3_sts_unpack(s3_get_status(), &f);
	return f.calibrated && !f.gap &&
	       (f.line_state == S3_LINE_ON_LINE ||
		f.line_state == S3_LINE_DRIFTING ||
		f.line_state == S3_LINE_JUNCTION);
}

LOCAL BOOL line_lost_now(void)
{
	S3_STS_FIELDS	f;

	s3_sts_unpack(s3_get_status(), &f);
	return f.calibrated && f.line_state == S3_LINE_LOST;
}

/* Which side the line was last seen on: + = right (steer sign) */
LOCAL INT line_side;

LOCAL void track_line_side(void)
{
	S3_STS_FIELDS	f;

	s3_sts_unpack(s3_get_status(), &f);
	if (f.steer > 0) line_side = 1;
	else if (f.steer < 0) line_side = -1;
}

/* ------------------------------------------------------------------ */
/*  State entries                                                     */
/* ------------------------------------------------------------------ */
/* Slow zone (VC_SLOW_ZONE_CM): 1 while the speed is capped for
   something ahead */
LOCAL UB	slow_zone;

LOCAL void check_slow_zone(void)
{
#if VC_SLOW_ZONE_CM > 0
	UINT	r = (s5_status() >> 11) & 127u;	/* obstacle_scan.h range */
	BOOL	near = (r == S5_RANGE_NEAR) ||
		       (r >= S5_RANGE_MIN_CM && r <= VC_SLOW_ZONE_CM);
	BOOL	clear = (r == S5_RANGE_FAR) ||
			(r > VC_SLOW_ZONE_CM + 10 && r <= S5_RANGE_MAX_CM);

	if (near && !slow_zone) {
		slow_zone = 1;
		cmd_limit(VC_SLOW_CM_S);
		tm_printf((UB*)"vc: something %d cm ahead, slowing to %d cm/s\n",
			  (INT)r, VC_SLOW_CM_S);
	} else if (clear && slow_zone) {
		slow_zone = 0;
		cmd_limit(S2_MAX_SPEED_CM_S);
	}
#endif
}

LOCAL void enter_follow(void)
{
	script_clear();
	slow_zone = 0;
	cmd_limit(S2_MAX_SPEED_CM_S);		/* lift any obstacle cap */
	cmd_follow(VC_CRUISE_CM_S);
	vc.lost_in_hump = 0;
	set_state(VC_ST_LINE_FOLLOW);
}

LOCAL void enter_idle(UINT reason)
{
	bench_move = 0;
	script_clear();
	cmd_stop();
	s5_cancel();
	vc.reason = (UB)reason;
	set_state(VC_ST_IDLE);
}

LOCAL void enter_fault(UINT reason, UINT bits)
{
	bench_move = 0;
	script_clear();
	cmd_stop();
	s5_cancel();
	vc.reason = (UB)reason;
	vc.faults = (UB)bits;
	if (bits != 0 && flg_fault > 0) tk_set_flg(flg_fault, bits);
	tm_printf((UB*)"vc: FAULT reason %d bits 0x%x\n", reason, bits);
	set_state(VC_ST_FAULT);
}

LOCAL void enter_complete(void)
{
	bench_move = 0;
	script_clear();
	cmd_stop();
	s5_cancel();
	tm_printf((UB*)"vc: COMPLETE, peak hump %d mm, %d junctions\n",
		  vc.peak_hump, vc.junctions);
	set_state(VC_ST_COMPLETE);
}

LOCAL void enter_search(void)
{
	script_clear();
	vc.search_i = 0;
	vc.side_right = (line_side >= 0);
	{
		UW c = S2_CMD_PACK(S2_CMD_TURN, VC_TURN_CM_S,
				   vc_search_turn(0, line_side), 0);
		script_run(&c, 1);
	}
	set_state(VC_ST_LINE_SEARCH);
}

/* After the obstacle: the IR sensors see a line, so follow it from here
   instead of finishing the scripted rejoin (approach + 90 degree turn).
   Obstacle Scan still has to confirm the line (its LINE_RECOVERY, with a
   watchdog), so follow slowly in REJOIN_LINE/PH_CONFIRM, keep relaying
   line samples (tick), and REJOIN_LINE_CONFIRMED then gives LINE_FOLLOW
   at cruise speed. */
LOCAL void rejoin_follow(void)
{
	UW	t[2];

	if (vc.state == VC_ST_BYPASS_OBSTACLE) {
		s5_rejoin_line();		/* S5: BYPASSING -> LINE_RECOVERY */
		set_state(VC_ST_REJOIN_LINE);
	}
	/* 30 Oct: line under the sensors, at an unknown angle (a curve may
	   have brought it here).  Bring the axle onto it, then turn towards
	   the bypass side -- the way the line continues -- until the
	   sensors find it again (tick, PH_REALIGN), and follow from there.
	   A fixed turn back to the original heading drove off a curved line
	   (video IMG_0771).  VC_ALIGN_MAX_DEG without the line: search. */
	t[0] = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_SLOW_CM_S, 0, VC_JUNCTION_ADV_CM);
	t[1] = S2_CMD_PACK(S2_CMD_TURN, VC_ALIGN_CM_S,
			   vc.side_right ? VC_ALIGN_MAX_DEG : -VC_ALIGN_MAX_DEG, 0);
	vc.phase = PH_REALIGN;
	vc.align_white = 0;
	script_run(t, 2);			/* pre-empts the TURN / MOVE */
	tm_printf((UB*)"vc: line found, turning %s until the sensors are on it\n",
		  vc.side_right ? "right" : "left");
}

/* Obstacle Scan stays in LINE_RECOVERY until it has seen the line for a
   while, and faults (E-STOP) if it never does.  When the VC has already
   moved on (LINE_FOLLOW, or LINE_SEARCH after losing the line again),
   keep feeding it line samples so it can still confirm. */
LOCAL void relay_recovery(void)
{
	if (s5_sts_state(s5_status()) == S5_ST_LINE_RECOVERY) {
		s5_line_observe(line_seen());
	}
}

LOCAL void enter_obstacle_scan(void)
{
	script_clear();
	cmd_limit(VC_SLOW_CM_S);		/* NFR12 */
	cmd_stop();
	vc.phase = PH_WAIT_STOPPED;
	vc.scan_sent = 0;
	set_state(VC_ST_OBSTACLE_SCAN);
}

/* START guards (design 7.3): IR and IMU calibrated, hazard cleared (no
   shock, nothing within 20 cm, no sensor in fault, no stall).  Returns
   VC_R_NONE or the reason; with verbose, prints every guard that fails,
   so the console says exactly why START was refused.  Reads only status
   words, so Buddy 1's Command Handler may call it too (vc_s1_hooks.c). */
EXPORT UINT vc_start_check(BOOL verbose)
{
	UW	s5 = s5_status();
	INT	s4 = s4_sts_state(s4_get_status_w0());
	UINT	why = VC_R_NONE;

	if (!s3_is_calibrated()) {
		why = VC_R_NOT_CALIBRATED;
		if (verbose) tm_printf((UB*)"vc: START refused: IR not calibrated"
				       " (put the barcode sensor over white)\n");
	}
	if (VC_REQUIRE_IMU && !s4_is_calibrated()) {
		why = VC_R_NOT_CALIBRATED;
		if (verbose) tm_printf((UB*)"vc: START refused: IMU not calibrated"
				       " (state %d; 5 = FAULT)\n", s4);
	}
	if (s4 == S4_ST_SHOCK) {
		why = VC_R_HAZARD;
		if (verbose) tm_printf((UB*)"vc: START refused: IMU shock active\n");
	}
	if (s5_sts_state(s5) == S5_ST_SENSOR_FAULT) {
		why = VC_R_HAZARD;
		if (verbose) {
			/* re-test the sensor: a valid echo clears the fault */
			s5_reset();
			tm_printf((UB*)"vc: START refused: ultrasonic fault (no"
				  " echo); re-testing it, send START again in"
				  " 3 s\n");
		}
	}
	if (vc_range_hazard(s5)) {
		why = VC_R_HAZARD;
		if (verbose) tm_printf((UB*)"vc: START refused: obstacle within"
				       " 20 cm (range code %d)\n", (s5 >> 11) & 127u);
	}
	if (s2_sts_state(motion_get_status()) == S2_ST_STALL_FAULT) {
		why = VC_R_HAZARD;
		if (verbose) tm_printf((UB*)"vc: START refused: wheel stall latched"
				       " (send STOP first)\n");
	}
	return why;
}

/* START from IDLE or E_STOPPED.  Guards: vc_start_check().  Action:
   clear E_STOP (only the VC does this), reset Obstacle Scan, then
   STARTING waits until it is watching forward. */
LOCAL void do_start(void)
{
	UINT	why = vc_start_check(TRUE);

	if (why != VC_R_NONE) {
		vc.reason = (UB)why;
		return;
	}
#if !VC_USE_ULTRASONIC
	tm_printf((UB*)"vc: WARNING ultrasonic disabled (VC_USE_ULTRASONIC 0):"
		  " no obstacle detection\n");
#endif
#if !S4_SHOCK_ESTOP
	tm_printf((UB*)"vc: WARNING shock E-STOP disabled (S4_SHOCK_ESTOP 0)\n");
#endif
	cmd_stop();				/* Motion: E_STOP_BRAKE -> STOPPED */
	tk_clr_flg(flg_safety, ~(UINT)S2_FLG_ESTOP);
	s5_reset();			/* needs E_STOP clear and Motion stopped */
	vc.reason = VC_R_NONE;
	vc.rescans = 0;			/* new mission */
	vc.nav_valid = 0;		/* 29 Oct: a barcode from the last run
					   must not turn at this run's first
					   junction */
	set_state(VC_ST_STARTING);
}

/* STOP while in FAULT = the reset: always back to IDLE.  Every fault
   bit is cleared and the ultrasonic is told to re-test itself.  A
   sensor that is still faulty is caught by START (vc_start_check
   refuses it and says why), so the robot still cannot drive with it. */
LOCAL void do_reset(void)
{
	cmd_stop();		/* also clears Motion's STALL / ENC latch */
	if (s5_sts_state(s5_status()) == S5_ST_SENSOR_FAULT) s5_reset();
	if (flg_fault > 0) tk_clr_flg(flg_fault, 0);
	vc.faults = 0;
	tm_printf((UB*)"vc: fault reset\n");
	enter_idle(VC_R_NONE);
}

/* ------------------------------------------------------------------ */
/*  Script finished (last MOTION_DONE)                                */
/* ------------------------------------------------------------------ */
LOCAL void script_done(void)
{
	UW	c;

	switch (vc.state) {
	case VC_ST_EXECUTE_NAV_CMD:
		if (bench_move) {			/* VC_BENCH_MOVE_CM */
			bench_move = 0;
			tm_printf((UB*)"vc: BENCH move done\n");
			enter_idle(VC_R_NONE);
		} else {
			enter_follow();
		}
		break;

	case VC_ST_LINE_SEARCH:
		if (vc.search_i + 1 < VC_SEARCH_STEPS) {
			vc.search_i++;
			c = S2_CMD_PACK(S2_CMD_TURN, VC_TURN_CM_S,
					vc_search_turn(vc.search_i,
						       vc.side_right ? 1 : -1), 0);
			script_run(&c, 1);
		}
		break;			/* else wait for the 3 s time-out */

	case VC_ST_OBSTACLE_SCAN:
		if (vc.phase == PH_REVERSING) {
			vc.phase = PH_WAIT_STOPPED;
			vc.scan_sent = 0;
			vc.t_state = now_ms();
		}
		break;

	case VC_ST_BYPASS_OBSTACLE:
		/* Now angled towards the line: start recovery (Obstacle Scan's
		   watchdog), drive towards the line slowly.  Up to twice the
		   sideways distance: at VC_REJOIN_TURN_DEG the path to the line
		   is longer than lat, and on a curve the line may be further */
		s5_rejoin_line();
		{
			INT d = 2 * (INT)vc.lat_cm + VC_REJOIN_EXTRA_CM;
			if (d > S2_DIST_MAX_CM) d = S2_DIST_MAX_CM;
			c = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_SLOW_CM_S, 0, d);
		}
		set_state(VC_ST_REJOIN_LINE);
		vc.phase = PH_APPROACH;
		script_run(&c, 1);
		break;

	case VC_ST_REJOIN_LINE:
		if (vc.phase == PH_REALIGN) {
			if (line_seen()) {
				vc.phase = PH_CONFIRM;
				cmd_follow(VC_SLOW_CM_S);	/* follow while S5 confirms */
			} else {
				/* Turned VC_ALIGN_MAX_DEG, line never under the
				   sensors: search, first towards the bypass side */
				line_side = vc.side_right ? 1 : -1;
				tm_printf((UB*)"vc: turned %d deg without the line,"
					  " searching\n", VC_ALIGN_MAX_DEG);
				enter_search();
			}
		}
		else if (vc.phase == PH_APPROACH) {
			/* Drove the whole approach without seeing the line (a
			   curve took it away): search for it, first on the side
			   it was left on.  relay_recovery keeps S5 fed. */
			line_side = vc.side_right ? -1 : 1;
			tm_printf((UB*)"vc: line not found after the obstacle,"
				  " searching\n");
			enter_search();
		}
		break;

	default:
		break;
	}
}

/* ------------------------------------------------------------------ */
/*  Events                                                            */
/* ------------------------------------------------------------------ */
LOCAL const char *nav_name(UINT nav)
{
	switch (nav) {
	case NAV_LEFT:     return "LEFT";
	case NAV_RIGHT:    return "RIGHT";
	case NAV_STRAIGHT: return "STRAIGHT";
	default:           return "U-TURN";
	}
}

LOCAL void on_junction(void)
{
	UW	c[VC_SCRIPT_MAX];
	INT	n;

	if (vc.state != VC_ST_LINE_FOLLOW) return;
	if (since(vc.t_junction) < VC_JUNCTION_COOLDOWN_MS) return;
	vc.t_junction = now_ms();
	vc.junctions++;

	if (VC_FINISH_JUNCTIONS > 0 && vc.junctions >= VC_FINISH_JUNCTIONS) {
		enter_complete();
		return;
	}
	if (!vc.nav_valid) {			/* no barcode: straight on */
		tm_printf((UB*)"vc: JUNCTION %d, no barcode: straight on\n",
			  vc.junctions);
		return;
	}
	tm_printf((UB*)"vc: JUNCTION %d, doing barcode %s\n", vc.junctions,
		  nav_name(vc.nav));
	n = vc_nav_script(vc.nav, c);
	vc.nav_valid = 0;
	if (n > 0) {
		set_state(VC_ST_EXECUTE_NAV_CMD);
		script_run(c, n);
	}
}

LOCAL void rescan(const char *why);
#if VC_AUTO_START_MS > 0
LOCAL UB	auto_started;
#endif

LOCAL void on_profile(UW payload)
{
	S5_SNAPSHOT	s;
	UB		gen = (UB)(payload >> 18);
	UINT		act;

	if (vc.state != VC_ST_OBSTACLE_SCAN || vc.phase != PH_WAIT_PROFILE) return;
	if (s5_get_status(&s) != E_OK || s5_sts_generation(s.status) != gen) {
		return;			/* stale: a newer scan will answer */
	}
	act = s5_profile_action(s.profile);
	if (act == S5_ACT_LEFT || act == S5_ACT_RIGHT || act == S5_ACT_CONTINUE) {
		if (s5_approve_plan(gen) == E_OK) {
			plan_profile = s.profile;
			vc.side_right = (act == S5_ACT_RIGHT);
			vc.phase = PH_WAIT_ACCEPT;
			vc.t_state = now_ms();
			return;
		}
	}
	/* STOP / REVERSE_REATTEMPT / approval refused: like NO_CLEAR */
	rescan("plan not usable or approval refused");
}

/* Every rescan counts, whatever the reason, and gives up after
   VC_MAX_RESCANS (28 Oct: a refused plan / an approval that S5 never
   took restarted the scan without counting, and enter_follow() cleared
   the count after every CONTINUE: the robot scanned 20+ times). */
LOCAL void rescan(const char *why)
{
	tm_printf((UB*)"vc: rescan (%s), %d done\n", why, vc.rescans);
	if (vc.rescans >= VC_MAX_RESCANS) {
		enter_fault(VC_R_NO_CLEAR, 0);
		return;
	}
	vc.rescans++;
	vc.phase = PH_WAIT_STOPPED;
	vc.scan_sent = 0;
}

LOCAL void on_no_clear(void)
{
	UW	c;

	if (vc.state != VC_ST_OBSTACLE_SCAN) return;
	if (vc.rescans >= VC_MAX_RESCANS) {
		enter_fault(VC_R_NO_CLEAR, 0);
		return;
	}
	vc.rescans++;
	/* Reverse is the VC's decision (Buddy 5 cannot see behind).  Far
	   enough to get the obstacle out of the turning circle at once */
	{
		S5_SNAPSHOT	s;
		INT		d = VC_REVERSE_CM;
		UINT		cl;

		if (s5_get_status(&s) == E_OK) {
			cl = s5_profile_closest(s.profile);
			if (cl == S5_RANGE_NEAR) {
				d = VC_REVERSE_TO_CM;
			} else if (cl >= S5_RANGE_MIN_CM && cl <= S5_RANGE_MAX_CM &&
				   (INT)cl < VC_REVERSE_TO_CM) {
				d = VC_REVERSE_TO_CM - (INT)cl;
			}
		}
		if (d < VC_REVERSE_CM) d = VC_REVERSE_CM;
		if (d > VC_REVERSE_MAX_CM) d = VC_REVERSE_MAX_CM;
		tm_printf((UB*)"vc: no clear way round, reversing %d cm (try %d of %d)\n",
			  d, vc.rescans, VC_MAX_RESCANS);
		c = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_SLOW_CM_S, 0, -d);
	}
	vc.phase = PH_REVERSING;
	script_run(&c, 1);
}

LOCAL void handle_event(UW w)
{
	UINT	type = EVT_TYPE(w);
	UW	pl = EVT_PAYLOAD(w);
	UINT	st = vc.state;
	BOOL	run = VC_IS_RUNNING(st);

	dg.evt_rx++;

	/* Events valid in every state */
	switch (type) {
	case EVT_HUMP_END:			/* peak for the COMPLETE report */
		if ((pl & 0x1Fu) > vc.peak_hump) vc.peak_hump = (UB)(pl & 0x1Fu);
		break;
	case EVT_NAV_CMD:			/* barcode before its junction (A7) */
		if (run) {
			vc.nav = (UB)(pl & NAV_MASK);
			vc.nav_valid = 1;
			tm_printf((UB*)"vc: barcode %s stored for the next junction\n",
				  nav_name(vc.nav));
		}
		break;
	default:
		break;
	}

	switch (type) {
	case EVT_START:
		tm_printf((UB*)"vc: START received in %s\n", st_name[st]);
		if (st == VC_ST_IDLE || st == VC_ST_E_STOPPED) {
			do_start();
		} else {
			dg.evt_ignored++;
			tm_printf((UB*)"vc: START ignored (only from IDLE or"
				  " E_STOPPED; send STOP first)\n");
		}
		return;

	case EVT_STOP:
		tm_printf((UB*)"vc: STOP received in %s\n", st_name[st]);
		if (run || st == VC_ST_E_STOPPED || st == VC_ST_COMPLETE) {
			enter_idle(VC_R_STOP);
		} else if (st == VC_ST_FAULT) {
			do_reset();
		} else {
			cmd_stop();
		}
		return;

	case EVT_MISSION_END:
		if (run) enter_complete();
		return;

	case EVT_SENSOR_FAULT:
		if (!VC_REQUIRE_IMU && pl == FAULT_IMU) {
			vc.faults |= (UB)pl;	/* bench mode: IMU optional */
			return;
		}
		if (run) enter_fault(VC_R_SENSOR_FAULT, (UINT)pl);
		else vc.faults |= (UB)pl;	/* IDLE: START guard handles it */
		return;

	default:
		break;
	}

	if (!run) {				/* the rest only matter while running */
		dg.evt_ignored++;
		return;
	}

	switch (type) {
	case EVT_MOTION_DONE:
		/* Only the step in progress: a DONE for a command that was
		   pre-empted (wrong type) is stale */
		if (script_len == 0 ||
		    (pl & S2_CMD_TYPE_MASK) != s2_cmd_type(script[script_idx])) {
			dg.evt_ignored++;
			break;
		}
		if (++script_idx < script_len) {
			t_step = now_ms();
			cmd(script[script_idx]);
		} else {
			script_clear();
			script_done();
		}
		break;

	case EVT_JUNCTION:
		on_junction();
		break;

	case EVT_LINE_LOST:
		if (st == VC_ST_LINE_FOLLOW) {
			if (s4_in_hump()) vc.lost_in_hump = 1;	/* tilted sensors */
			else enter_search();
		} else if (st == VC_ST_HUMP_TRAVERSE) {
			vc.lost_in_hump = 1;
		}
		break;

	case EVT_LINE_FOUND:
		vc.lost_in_hump = 0;
		if (st == VC_ST_LINE_SEARCH) enter_follow();
		break;

	case EVT_HUMP_START:
		if (st == VC_ST_LINE_FOLLOW) set_state(VC_ST_HUMP_TRAVERSE);
		/* speed is held: no command (A4, Buddy 4) */
		break;

	case EVT_HUMP_END:
		if (st == VC_ST_HUMP_TRAVERSE) {
			if (vc.lost_in_hump && !line_seen()) enter_search();
			else set_state(VC_ST_LINE_FOLLOW);
			vc.lost_in_hump = 0;
		}
		break;

	case EVT_OBSTACLE_DETECTED:
		if (st != VC_ST_OBSTACLE_SCAN && st != VC_ST_STARTING) {
			enter_obstacle_scan();
		}
		break;

	case EVT_OBSTACLE_PROFILE:
		on_profile(pl);
		break;

	case EVT_NO_CLEAR:
		on_no_clear();
		break;

	case EVT_REJOIN_LINE_CONFIRMED:
		if (st == VC_ST_REJOIN_LINE) {
			vc.rescans = 0;		/* obstacle passed */
			enter_follow();
		}
		break;

	default:
		dg.evt_ignored++;
		break;
	}
}

/* ------------------------------------------------------------------ */
/*  Time-based checks, every loop                                     */
/* ------------------------------------------------------------------ */
LOCAL void tick(void)
{
	UINT	st = vc.state;
	UW	s5, ms = motion_get_status();
	UW	c[VC_SCRIPT_MAX];
	INT	n, lat;

	/* E-STOP seen: Motion has already braked; just follow it */
	if (VC_IS_RUNNING(st) && estop_set()) {
		bench_move = 0;
		script_clear();
		set_state(VC_ST_E_STOPPED);
		return;
	}

	/* A script step that never finished */
	if (script_len > 0 && since(t_step) > VC_STEP_TIMEOUT_MS) {
		enter_fault(VC_R_STEP_TIMEOUT, 0);
		return;
	}

	switch (st) {
#if VC_AUTO_START_MS > 0
	case VC_ST_IDLE:
		/* Bench build only: START by itself once, VC_AUTO_START_MS
		   after reaching IDLE (no dashboard needed) */
		/* retried every VC_AUTO_START_MS while a guard refuses (e.g.
		   IMU still calibrating); never again after the first START */
		if (!auto_started && since(vc.t_state) >= VC_AUTO_START_MS) {
			tm_printf((UB*)"vc: AUTO START (bench build)\n");
			vc.t_state = now_ms();
			do_start();
			if (vc.state != VC_ST_IDLE) auto_started = 1;
		}
		break;
#endif

	case VC_ST_INIT:
		/* Self-test: Motion runs and is braked */
		if (s2_sts_state(ms) == S2_ST_STOPPED ||
		    s2_sts_state(ms) == S2_ST_E_STOP_BRAKE) {
			set_state(VC_ST_IDLE);
		}
		break;

	case VC_ST_STARTING:
		s5 = s5_status();
		if (s5_sts_state(s5) == S5_ST_FORWARD_WATCH &&
		    since(vc.t_state) >= VC_S5_SETTLE_MS) {
#if VC_BENCH_MOVE_CM
			{
				UW c = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_CRUISE_CM_S,
						   0, VC_BENCH_MOVE_CM);
				tm_printf((UB*)"vc: BENCH straight move %d cm\n",
					  VC_BENCH_MOVE_CM);
				bench_move = 1;
				set_state(VC_ST_EXECUTE_NAV_CMD);
				script_run(&c, 1);
			}
#else
			enter_follow();
#endif
		} else if (since(vc.t_state) > VC_S5_READY_MS) {
			tm_printf((UB*)"vc: START abandoned: obstacle scan not"
				  " watching forward (S5 state %d)\n",
				  s5_sts_state(s5));
			enter_idle(VC_R_SCAN_NOT_READY);
		}
		break;

	case VC_ST_LINE_FOLLOW:
	case VC_ST_HUMP_TRAVERSE:
		relay_recovery();
		check_slow_zone();
		track_line_side();
		/* Line & Barcode reports LINE_LOST only on the change; after a
		   turn or a bypass it may already be LOST, so check the state */
		if (st == VC_ST_LINE_FOLLOW && !vc.lost_in_hump &&
		    since(vc.t_state) > VC_LOST_CHECK_MS && line_lost_now() &&
		    !s4_in_hump()) {
			enter_search();
			break;
		}
		if (st == VC_ST_LINE_FOLLOW && vc.lost_in_hump && !s4_in_hump()) {
			vc.lost_in_hump = 0;
			if (!line_seen()) enter_search();
		}
		if (st == VC_ST_HUMP_TRAVERSE && since(vc.t_state) > VC_HUMP_MAX_MS) {
			set_state(VC_ST_LINE_FOLLOW);
		}
		break;

	case VC_ST_LINE_SEARCH:
		relay_recovery();
		if (line_seen()) {
			enter_follow();
		} else if (since(vc.t_state) >= VC_LINE_SEARCH_MS) {
			enter_fault(VC_R_LINE_TIMEOUT, FAULT_LINE_TIMEOUT);
		}
		break;

	case VC_ST_OBSTACLE_SCAN:
		if (vc.phase == PH_WAIT_STOPPED) {
			/* Buddy 5: SCAN_REQ only once Motion reports stopped */
			if (!vc.scan_sent && s2_sts_stopped(ms) &&
			    s5_request_scan() == E_OK) {
				vc.scan_sent = 1;
				vc.phase = PH_WAIT_PROFILE;
				vc.t_state = now_ms();
			}
		} else if (vc.phase == PH_WAIT_PROFILE) {
			if (since(vc.t_state) > VC_SCAN_TIMEOUT_MS) {
				enter_fault(VC_R_SCAN_TIMEOUT, 0);
			}
		} else if (vc.phase == PH_WAIT_ACCEPT) {
			/* Approval is queued: move only once S5 accepted it */
			s5 = s5_status();
			if (s5_sts_state(s5) == S5_ST_BYPASSING) {
				vc.rescans = 0;		/* plan taken */
				n = vc_bypass_script(plan_profile,
						     vc.side_right ? 1 : -1, c, &lat);
				vc.lat_cm = (B)lat;
				set_state(VC_ST_BYPASS_OBSTACLE);
				script_run(c, n);
			} else if (s5_sts_state(s5) == S5_ST_FORWARD_WATCH) {
				/* CONTINUE: count kept, or detect -> scan ->
				   CONTINUE could loop for ever */
				enter_follow();		/* CONTINUE */
			} else if (since(vc.t_state) > VC_APPROVE_WAIT_MS) {
				rescan("S5 did not start the bypass");
			}
		}
		break;

	case VC_ST_BYPASS_OBSTACLE:
		/* Past the obstacle and turning towards the line: a line under
		   the sensors is followed now (28 Oct: it was seen here, then the
		   rejoin drove on past it and timed out) */
		if (script_len > 0 && script_idx >= VC_BYPASS_FOLLOW_STEP &&
		    line_seen()) {
			rejoin_follow();
		}
		break;

	case VC_ST_REJOIN_LINE:
		/* Relay a fresh line sample every <= 60 ms (Buddy 5) */
		{
			BOOL found = line_seen();
			s5_line_observe(found);
			if (vc.phase == PH_APPROACH && found) {
				rejoin_follow();	/* axle on it, turn until on it */
			} else if (vc.phase == PH_REALIGN && script_idx == 1) {
				/* Turning: the sensors must leave the line first
				   (gap after 150 ms of white), then the first time
				   they see it again the robot points along it */
				if (!found) {
					vc.align_white = 1;
				} else if (vc.align_white) {
					script_clear();	/* stops the TURN */
					vc.phase = PH_CONFIRM;
					cmd_follow(VC_SLOW_CM_S);
					tm_printf((UB*)"vc: on the line, following it\n");
				}
			} else if (vc.phase == PH_CONFIRM) {
				/* Following it: lost it again -> search for it
				   (relay_recovery keeps S5 fed) */
				track_line_side();
				if (line_lost_now()) enter_search();
			}
		}
		break;

	default:
		break;
	}
}

LOCAL void publish(void)
{
	g_vc_status = vc_sts_pack(vc.state, vc.nav, vc.nav_valid, vc.peak_hump,
				  script_len ? script_idx + 1u : 0u, vc.reason,
				  vc.junctions, vc.rescans);
}

/* ------------------------------------------------------------------ */
/*  Task                                                              */
/* ------------------------------------------------------------------ */
EXPORT void vc_task(INT stacd, void *exinf)
{
	UW	w;
	INT	n;

	(void)stacd; (void)exinf;

	vc.state = VC_ST_INIT;
	vc.t_state = vc.t_junction = now_ms();
	line_side = 1;
	publish();

	while (1) {
		n = tk_rcv_mbf(mbf_vc_evt, &w, VC_TICK_MS);
		if (n == (INT)sizeof(w)) {
			handle_event(w);
		} else if (n >= 0) {
			dg.evt_ignored++;	/* wrong size: not an event word */
		}
		tick();
		publish();
	}
}

/* ------------------------------------------------------------------ */
/*  Public functions                                                  */
/* ------------------------------------------------------------------ */
EXPORT ER vc_post(UINT type, UW payload)
{
	UW	w = EVT_PACK(type, payload);
	return tk_snd_mbf(mbf_vc_evt, &w, sizeof(w), TMO_POL);
}

EXPORT UW vc_get_status(void)
{
	return g_vc_status;
}

EXPORT UINT vc_get_faults(void)
{
	return vc.faults;
}

EXPORT void vc_get_diag(VC_DIAG *d)
{
	*d = dg;		/* diagnostic only: halfwords, no lock needed */
}