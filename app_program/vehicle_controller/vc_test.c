/*
 *	vc_test.c
 *	On-target tests for the Vehicle Controller.  Built only with: make TEST=1
 *	Record results in testing/vehicle_controller/TEST_LOG.md (VC-Uxx) and
 *	testing/integration/TEST_LOG.md (INT-06).
 *
 *	No test sends START, so the robot never drives.  INT-06 sends Motion
 *	a MOVE(d = 0), which finishes at once without turning a wheel.
 */

#include "vehicle_controller.h"
#include "../subsystem2/subsystem2.h"
#include "../test_report.h"

#if APP_TEST

/* Build a Buddy 5 profile word 0: closest [6:0], width [14:7],
   x [22:15] signed, y [29:23] (obstacle_scan.h) */
LOCAL S5_PROFILE make_profile(UINT closest, UINT width, INT x, UINT y)
{
	S5_PROFILE	p;

	p.w0 = (closest & 127u) | ((width & 255u) << 7) |
	       (((UW)x & 255u) << 15) | ((y & 127u) << 23);
	p.w1 = 0;
	return p;
}

LOCAL BOOL is_cmd(UW w, UINT type, INT angle, INT dist)
{
	return s2_cmd_type(w) == type && s2_cmd_angle(w) == angle &&
	       s2_cmd_dist(w) == dist;
}

EXPORT void vc_test(void)
{
	UW		w, c[VC_SCRIPT_MAX];
	INT		n, lat;
	BOOL		ok;
	VC_DIAG		d0, d1;
	S5_PROFILE	p;
	T_RFLG		rf;

	TEST_INIT();

	/* VC-U01: status word round trip and clamping */
	w = vc_sts_pack(VC_ST_REJOIN_LINE, NAV_U_TURN, 1, 25, 3,
			VC_R_LINE_TIMEOUT, 13, 2);
	ok = vc_sts_state(w) == VC_ST_REJOIN_LINE && vc_sts_nav(w) == NAV_U_TURN &&
	     vc_sts_nav_valid(w) == 1 && vc_sts_hump(w) == 25 &&
	     vc_sts_step(w) == 3 && vc_sts_reason(w) == VC_R_LINE_TIMEOUT &&
	     vc_sts_junctions(w) == 13 && vc_sts_rescans(w) == 2;
	w = vc_sts_pack(VC_ST_COMPLETE, 0, 0, 99, 9, 0, 17, 5);
	ok = ok && vc_sts_hump(w) == 31 && vc_sts_step(w) == 7 &&
	     vc_sts_junctions(w) == 1 && vc_sts_rescans(w) == 3 &&
	     vc_sts_state(w) == VC_ST_COMPLETE && (w >> 25) == 0;
	TEST_CHECK("VC-U01", ok, "VC status word pack/unpack and clamping");

	/* VC-U02: barcode command -> junction script */
	n = vc_nav_script(NAV_LEFT, c);
	ok = n == 2 && is_cmd(c[0], S2_CMD_MOVE_DIST, 0, VC_JUNCTION_ADV_CM) &&
	     is_cmd(c[1], S2_CMD_TURN, -90, 0);
	n = vc_nav_script(NAV_RIGHT, c);
	ok = ok && n == 2 && is_cmd(c[1], S2_CMD_TURN, 90, 0);
	n = vc_nav_script(NAV_U_TURN, c);
	ok = ok && n == 2 && is_cmd(c[1], S2_CMD_TURN, 180, 0);
	ok = ok && vc_nav_script(NAV_STRAIGHT, c) == 0;
	TEST_CHECK("VC-U02", ok, "LEFT/RIGHT/U-TURN/STRAIGHT junction scripts");

	/* VC-U03: bypass geometry, both sides, and clamping */
	p = make_profile(20, 10, 0, 20);
	n = vc_bypass_script(p, 1, c, &lat);
	ok = n == 5 && lat == 10 + S5_ROBOT_WIDTH_MM / 20 + S5_CLEARANCE_MARGIN_MM / 10 +
	     VC_BYPASS_EXTRA_CM &&
		 is_cmd(c[0], S2_CMD_TURN, 90, 0) &&
	     is_cmd(c[1], S2_CMD_MOVE_DIST, 0, lat) &&
	     is_cmd(c[2], S2_CMD_TURN, -90, 0) &&
	     is_cmd(c[3], S2_CMD_MOVE_DIST, 0,
		    20 + VC_OBSTACLE_DEPTH_CM + S5_ROBOT_LENGTH_MM / 10) &&
	     is_cmd(c[4], S2_CMD_TURN, -VC_REJOIN_TURN_DEG, 0);
	n = vc_bypass_script(p, -1, c, &lat);
	ok = ok && n == 5 && is_cmd(c[0], S2_CMD_TURN, -90, 0) &&
	     is_cmd(c[2], S2_CMD_TURN, 90, 0) &&
	     is_cmd(c[4], S2_CMD_TURN, VC_REJOIN_TURN_DEG, 0);
	p = make_profile(20, 120, 30, 100);
	vc_bypass_script(p, 1, c, &lat);
	ok = ok && lat == VC_BYPASS_MAX_CM && s2_cmd_dist(c[3]) == S2_DIST_MAX_CM;
	TEST_CHECK("VC-U03", ok, "bypass script geometry, sides and clamps");

	/* VC-U04: line search sweep alternates, first towards the line */
	ok = vc_search_turn(0, 1) == 30 && vc_search_turn(1, 1) == -60 &&
	     vc_search_turn(2, 1) == 90 && vc_search_turn(3, 1) == -120 &&
	     vc_search_turn(0, -1) == -30 && vc_search_turn(1, -1) == 60;
	TEST_CHECK("VC-U04", ok, "LINE_SEARCH sweep angles");

	/* VC-U05: hazard from Obstacle Scan's range field [17:11] */
	ok = vc_range_hazard((UW)15 << 11) && vc_range_hazard((UW)S5_RANGE_NEAR << 11) &&
	     !vc_range_hazard((UW)50 << 11) &&
	     !vc_range_hazard((UW)S5_RANGE_NO_ECHO << 11) &&
	     !vc_range_hazard((UW)S5_RANGE_FAR << 11);
	TEST_CHECK("VC-U05", ok, "hazard = obstacle within 20 cm");

	/* VC-U06: after start-up the VC is IDLE (self-test passed) */
	TEST_CHECK("VC-U06", vc_sts_state(vc_get_status()) == VC_ST_IDLE,
		   "VC reaches IDLE after INIT");

	/* VC-U07: in IDLE, driving events are ignored: no state change, no
	   barcode stored, Motion not moving */
	vc_post(EVT_NAV_CMD, NAV_LEFT);
	vc_post(EVT_JUNCTION, 0);
	vc_post(EVT_OBSTACLE_DETECTED, 0);
	vc_post(EVT_LINE_LOST, 0);
	tk_dly_tsk(4 * VC_TICK_MS);
	w = vc_get_status();
	ok = vc_sts_state(w) == VC_ST_IDLE && vc_sts_nav_valid(w) == 0 &&
	     (s2_sts_state(motion_get_status()) == S2_ST_STOPPED ||
	      s2_sts_state(motion_get_status()) == S2_ST_E_STOP_BRAKE);
	TEST_CHECK("VC-U07", ok, "IDLE ignores driving events, robot stays stopped");

	/* VC-U08: STOP in IDLE keeps IDLE */
	vc_post(EVT_STOP, 0);
	tk_dly_tsk(3 * VC_TICK_MS);
	TEST_CHECK("VC-U08", vc_sts_state(vc_get_status()) == VC_ST_IDLE,
		   "STOP in IDLE stays IDLE");

	/* INT-06: Motion -> VC binding.  MOVE(d = 0) ends at once with
	   MOTION_DONE, which must reach mbf_vc_evt.  Needs E-STOP clear
	   (Motion ignores moves while braking). */
	ok = tk_ref_flg(flg_safety, &rf) == E_OK && !(rf.flgptn & S2_FLG_ESTOP);
	vc_get_diag(&d0);
	motion_send_cmd(S2_CMD_PACK(S2_CMD_MOVE_DIST, 0, 0, 0));
	tk_dly_tsk(5 * S2_PERIOD_MS);
	vc_get_diag(&d1);
	ok = ok && (UH)(d1.evt_rx - d0.evt_rx) >= 1 &&
	     vc_sts_state(vc_get_status()) == VC_ST_IDLE;
	TEST_CHECK("INT-06", ok, "Motion's MOTION_DONE reaches the VC (E-STOP clear)");

	TEST_SUMMARY("vehicle_controller");
}

#endif /* APP_TEST */