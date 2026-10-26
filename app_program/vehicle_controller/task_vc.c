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
	S5_SNAPSHOT	s;
	return (s5_get_status(&s) == E_OK) ? s.status : (UW)S5_ST_SENSOR_FAULT;
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
LOCAL void enter_follow(void)
{
	script_clear();
	cmd_limit(S2_MAX_SPEED_CM_S);		/* lift any obstacle cap */
	cmd_follow(VC_CRUISE_CM_S);
	vc.lost_in_hump = 0;
	vc.rescans = 0;
	set_state(VC_ST_LINE_FOLLOW);
}

LOCAL void enter_idle(UINT reason)
{
	script_clear();
	cmd_stop();
	s5_cancel();
	vc.reason = (UB)reason;
	set_state(VC_ST_IDLE);
}

LOCAL void enter_fault(UINT reason, UINT bits)
{
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
		if (verbose) tm_printf((UB*)"vc: START refused: ultrasonic fault"
				       " (no echo)\n");
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
	cmd_stop();				/* Motion: E_STOP_BRAKE -> STOPPED */
	tk_clr_flg(flg_safety, ~(UINT)S2_FLG_ESTOP);
	s5_reset();			/* needs E_STOP clear and Motion stopped */
	vc.reason = VC_R_NONE;
	set_state(VC_ST_STARTING);
}

/* STOP while in FAULT = the reset.  Guard: the fault is gone. */
LOCAL void do_reset(void)
{
	UW	s5 = s5_status();
	INT	s4 = s4_sts_state(s4_get_status_w0());

	cmd_stop();		/* also clears Motion's STALL / ENC latch */
	if ((VC_REQUIRE_IMU && s4 == S4_ST_FAULT) ||
	    s5_sts_state(s5) == S5_ST_SENSOR_FAULT) {
		vc.reason = VC_R_FAULT_ACTIVE;
		tm_printf((UB*)"vc: reset refused, sensor still in fault\n");
		return;
	}
	if (flg_fault > 0) {
		tk_clr_flg(flg_fault, ~(UINT)(FAULT_LINE_TIMEOUT | FAULT_NO_ECHO));
	}
	vc.faults = 0;
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
		enter_follow();
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
		/* Now facing the line: start recovery (Obstacle Scan's 3 s
		   watchdog), drive towards the line slowly */
		s5_rejoin_line();
		c = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_SLOW_CM_S, 0,
				vc.lat_cm + VC_REJOIN_EXTRA_CM);
		set_state(VC_ST_REJOIN_LINE);
		vc.phase = PH_APPROACH;
		script_run(&c, 1);
		break;

	case VC_ST_REJOIN_LINE:
		if (vc.phase == PH_REALIGN) {
			vc.phase = PH_CONFIRM;
			cmd_follow(VC_SLOW_CM_S);	/* follow while S5 confirms */
		}
		/* PH_APPROACH done without the line: hold; Obstacle Scan's
		   watchdog reports LINE_TIMEOUT */
		break;

	default:
		break;
	}
}

/* ------------------------------------------------------------------ */
/*  Events                                                            */
/* ------------------------------------------------------------------ */
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
	if (!vc.nav_valid) return;		/* no barcode: straight on */
	n = vc_nav_script(vc.nav, c);
	vc.nav_valid = 0;
	if (n > 0) {
		set_state(VC_ST_EXECUTE_NAV_CMD);
		script_run(c, n);
	}
}

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
	/* Reverse is the VC's decision (Buddy 5 cannot see behind) */
	c = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_SLOW_CM_S, 0, -VC_REVERSE_CM);
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
		if (st == VC_ST_REJOIN_LINE) enter_follow();
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
			enter_follow();
		} else if (since(vc.t_state) > VC_S5_READY_MS) {
			tm_printf((UB*)"vc: START abandoned: obstacle scan not"
				  " watching forward (S5 state %d)\n",
				  s5_sts_state(s5));
			enter_idle(VC_R_SCAN_NOT_READY);
		}
		break;

	case VC_ST_LINE_FOLLOW:
	case VC_ST_HUMP_TRAVERSE:
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
				n = vc_bypass_script(plan_profile,
						     vc.side_right ? 1 : -1, c, &lat);
				vc.lat_cm = (B)lat;
				set_state(VC_ST_BYPASS_OBSTACLE);
				script_run(c, n);
			} else if (s5_sts_state(s5) == S5_ST_FORWARD_WATCH) {
				enter_follow();		/* CONTINUE */
			} else if (since(vc.t_state) > VC_APPROVE_WAIT_MS) {
				vc.phase = PH_WAIT_STOPPED;	/* rescan */
				vc.scan_sent = 0;
			}
		}
		break;

	case VC_ST_REJOIN_LINE:
		/* Relay a fresh line sample every <= 60 ms (Buddy 5) */
		{
			BOOL found = line_seen();
			s5_line_observe(found);
			if (vc.phase == PH_APPROACH && found) {
				/* Turn back along the original direction */
				UW t = S2_CMD_PACK(S2_CMD_TURN, VC_TURN_CM_S,
						   vc.side_right ? 90 : -90, 0);
				vc.phase = PH_REALIGN;
				script_run(&t, 1);	/* pre-empts the MOVE */
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
