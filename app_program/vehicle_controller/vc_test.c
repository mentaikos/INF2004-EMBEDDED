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

/* Build a Buddy 5 profile: word 0 closest [6:0], width [14:7],
   x [22:15] signed, y [29:23]; word 1 first/last edge bin [18:14] /
   [23:19] (obstacle_scan.h) */
LOCAL S5_PROFILE make_profile(UINT closest, UINT width, INT x, UINT y,
			      UINT lo, UINT hi)
{
	S5_PROFILE	p;

	p.w0 = (closest & 127u) | ((width & 255u) << 7) |
	       (((UW)x & 255u) << 15) | ((y & 127u) << 23);
	p.w1 = ((lo & 31u) << 14) | ((hi & 31u) << 19);
	return p;
}

LOCAL INT vc_test_min(INT a, INT b) { return (a < b) ? a : b; }

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

	/* VC-U02: barcode command -> junction script (12 Nov: distances in mm) */
	n = vc_nav_script(NAV_LEFT, c);
	ok = n == 1 + VC_LR_TURNS && is_cmd(c[0], S2_CMD_MOVE_DIST, 0, VC_NAV_ADV_MM) &&
	     is_cmd(c[1], S2_CMD_TURN, -180, 0);	/* 12 Nov: ended by compass + line */
	n = vc_nav_script(NAV_RIGHT, c);
	ok = ok && n == 1 + VC_LR_TURNS && is_cmd(c[1], S2_CMD_TURN, 180, 0);
	n = vc_nav_script(NAV_U_TURN, c);
	ok = ok && n == 1 + VC_UTURN_TURNS && is_cmd(c[1], S2_CMD_TURN, 180, 0) &&
	     is_cmd(c[VC_UTURN_TURNS], S2_CMD_TURN, 180, 0);	/* 12 Nov */
	ok = ok && vc_nav_script(NAV_STRAIGHT, c) == 0;
	TEST_CHECK("VC-U02", ok, "LEFT/RIGHT/U-TURN/STRAIGHT junction scripts");

	/* VC-U03: bypass geometry, both sides, and clamping */
	p = make_profile(20, 10, 0, 20, 9, 15);	/* 75..105 deg */
#if VC_BYPASS_BOX
	{
		INT	deg, deg_n, deg_w, deg_f, lat_r, lat_l, lat_n, lat_w, past_n, past_w;

		/* 19 Nov: angle from the width.  10 cm wide (+ 4 pad), centred,
		   20 cm ahead: edge 7 -> out 7+8+3+4 = 22; near corner needs
		   7+8+3+5 = 23 sideways within 20 -> tan 1.15 -> 50 deg;
		   diagonal 22/sin50 = 29 (18 forward); past: back face at
		   10+20+15 (depth min) = 45, + 6 for the diagonal back to miss
		   it, - 18 = 33 cm */
		n = vc_bypass_script(p, 1, c, &lat, &deg);
		ok = n == 5 && lat == 22 && deg == 50 &&
		     is_cmd(c[0], S2_CMD_TURN, VC_BYPASS_TURN_CMD(50), 0) &&
		     is_cmd(c[1], S2_CMD_MOVE_DIST, 0, VC_BYPASS_MM(29)) &&
		     is_cmd(c[2], S2_CMD_TURN, VC_BYPASS_TURN_CMD(-50), 0) &&
		     is_cmd(c[3], S2_CMD_MOVE_DIST, 0, VC_BYPASS_MM(33)) &&
		     is_cmd(c[4], S2_CMD_TURN, VC_BYPASS_TURN_CMD(-50), 0);
		n = vc_bypass_script(p, -1, c, &lat, &deg);
		ok = ok && n == 5 && lat == 22 && deg == 50 &&
		     is_cmd(c[0], S2_CMD_TURN, VC_BYPASS_TURN_CMD(-50), 0) &&
		     is_cmd(c[2], S2_CMD_TURN, VC_BYPASS_TURN_CMD(50), 0) &&
		     is_cmd(c[4], S2_CMD_TURN, VC_BYPASS_TURN_CMD(50), 0);
		/* off centre, 4 cm to the right: going right is 8 cm further
		   out (and steeper) than going left */
		p = make_profile(20, 10, 4, 20, 9, 15);
		vc_bypass_script(p, 1, c, &lat_r, &deg);
		ok = ok && lat_r == 26 && deg == 55;
		vc_bypass_script(p, -1, c, &lat_l, &deg);
		ok = ok && lat_l == 18 && deg == 45;
		/* wider = further out, steeper, further past: 10, 20, 30 cm
		   wide, 20 cm ahead -> 50, 55, 60 deg */
		vc_bypass_script(make_profile(20, 20, 0, 20, 7, 17), 1, c, &lat_n, &deg_n);
		past_n = s2_cmd_dist(c[3]);
		vc_bypass_script(make_profile(20, 30, 0, 20, 5, 19), 1, c, &lat_w, &deg_w);
		past_w = s2_cmd_dist(c[3]);
		tm_printf((UB*)"[TEST]   20 cm wide: %d deg, out %d cm, past %d mm;"
			  " 30 cm wide: %d deg, out %d cm, past %d mm\n",
			  deg_n, lat_n, past_n, deg_w, lat_w, past_w);
		ok = ok && deg_n == 55 && deg_w == 60 && lat_w > lat_n && past_w > past_n;
		/* the same 20 cm obstacle further away: shallower */
		vc_bypass_script(make_profile(40, 20, 0, 40, 9, 15), 1, c, &lat, &deg_f);
		ok = ok && deg_f == 45;
		/* wide and far: out clamps to 60, past to the S2 limit */
		vc_bypass_script(make_profile(20, 120, 30, 100, 0, 24), 1, c, &lat, &deg);
		ok = ok && lat == VC_BYPASS_MAX_CM &&
		     s2_cmd_dist(c[3]) == VC_BYPASS_MM(vc_test_min(1000, VC_BYPASS_MOVE_MAX_CM));
		/* to the line: lat / sin a + lat / 2 + extra */
		ok = ok && vc_rejoin_approach_cm(20, 90) == 30 + VC_REJOIN_EXTRA_CM &&
		     vc_rejoin_approach_cm(22, 50) == 29 + 11 + VC_REJOIN_EXTRA_CM &&
		     vc_rejoin_expect_cm(22, 50) == 29;
	}
#elif VC_BYPASS_BY_WIDTH && VC_BYPASS_SMOOTH
	n = vc_bypass_script(p, 1, c, &lat, NULL);
	/* 16 Nov: sized by the width, at true size.  10 cm wide, centred
	   (x = centre since 16 Nov): edge 0 + 5 -> lat 5+8+3+4 = 20; corner
	   21 within 20 -> 50 deg; diagonal 27 cm, 16 forward; past 20 +
	   depth (= width 10) + 10 + 3 - 16 = 27.  Driven 10 mm per cm,
	   turns VC_BYPASS_TURN_CMD (real angle). */
	ok = n == 5 && lat == 20 &&
	     is_cmd(c[0], S2_CMD_TURN, VC_BYPASS_TURN_CMD(50), 0) &&
	     is_cmd(c[1], S2_CMD_MOVE_DIST, 0, VC_BYPASS_MM(27)) &&
	     is_cmd(c[2], S2_CMD_TURN, VC_BYPASS_TURN_CMD(-50), 0) &&
	     is_cmd(c[3], S2_CMD_MOVE_DIST, 0, VC_BYPASS_MM(27)) &&
	     is_cmd(c[4], S2_CMD_TURN, -VC_REJOIN_TURN_DEG, 0);
	n = vc_bypass_script(p, -1, c, &lat, NULL);
	ok = ok && n == 5 && lat == 20 &&
	     is_cmd(c[0], S2_CMD_TURN, VC_BYPASS_TURN_CMD(-50), 0) &&
	     is_cmd(c[2], S2_CMD_TURN, VC_BYPASS_TURN_CMD(50), 0) &&
	     is_cmd(c[4], S2_CMD_TURN, VC_REJOIN_TURN_DEG, 0);
	/* Wider = further out and further past.  Both 20 cm ahead,
	   centred: 6 cm wide (80..100 deg) vs 20 cm wide (65..115 deg) */
	{
		INT	lat_n, lat_w, past_n, past_w;

		vc_bypass_script(make_profile(20, 6, 0, 20, 10, 14), 1, c, &lat_n, NULL);
		past_n = s2_cmd_dist(c[1]) + s2_cmd_dist(c[3]);
		vc_bypass_script(make_profile(20, 20, 0, 20, 7, 17), 1, c, &lat_w, NULL);
		past_w = s2_cmd_dist(c[1]) + s2_cmd_dist(c[3]);
		tm_printf((UB*)"[TEST]   6 cm wide: sideways %d cm, path %d mm;"
			  " 20 cm wide: sideways %d cm, path %d mm\n",
			  lat_n, past_n, lat_w, past_w);
		ok = ok && lat_w > lat_n && past_w > past_n;
	}
	/* Wide and far: edge 30 + 60 = 90 -> lat clamps to 60; corner 106
	   within 100 -> 50 deg; diagonal 79, 50 forward; depth = width 120
	   -> VC_DEPTH_MAX_CM; past 100 + depth + 10 + 3 - 50 */
	p = make_profile(20, 120, 30, 100, 0, 24);	/* whole sweep */
	vc_bypass_script(p, 1, c, &lat, NULL);
	ok = ok && lat == VC_BYPASS_MAX_CM &&
	     s2_cmd_angle(c[0]) == VC_BYPASS_TURN_CMD(50) &&
	     s2_cmd_dist(c[1]) == VC_BYPASS_MM(79) &&
	     s2_cmd_dist(c[3]) == VC_BYPASS_MM(vc_test_min(100 + VC_DEPTH_MAX_CM +
		S5_ROBOT_LENGTH_MM / 20 + VC_PASS_MARGIN_CM - 50, VC_BYPASS_MOVE_MAX_CM));
	/* Too close for any diagonal: 90 degrees out; depth = width 30,
	   capped at VC_DEPTH_MAX_CM */
	p = make_profile(20, 30, 0, 2, 6, 18);
	vc_bypass_script(p, 1, c, &lat, NULL);
	ok = ok && is_cmd(c[0], S2_CMD_TURN, VC_BYPASS_TURN_CMD(90), 0) &&
	     is_cmd(c[1], S2_CMD_MOVE_DIST, 0, VC_BYPASS_MM(lat)) &&
	     is_cmd(c[3], S2_CMD_MOVE_DIST, 0, VC_BYPASS_MM(2 +
		    vc_test_min(30, VC_DEPTH_MAX_CM) +
		    S5_ROBOT_LENGTH_MM / 20 + VC_PASS_MARGIN_CM));
#elif VC_BYPASS_SMOOTH
	n = vc_bypass_script(p, 1, c, &lat, NULL);
	/* Values for the placeholder robot size (160 x 200 mm, margin 30)
	   seen 75..105 deg, 20 cm ahead: edge from the next bin out (70 deg)
	   = 20 x cot 70 = 7 cm (30 Oct: was x + width = 10); corner needs
	   7+8+3+5 = 23 cm sideways within 20 cm -> tan >= 1.15 -> 50 deg;
	   lat 7+8+3+4 = 22, diagonal 22/sin50 = 29, of which 18 forward;
	   past: 20 + depth 10 + half robot 10 + margin 3 - 18 = 25 */
	ok = n == 5 && lat == 22 &&
	     is_cmd(c[0], S2_CMD_TURN, 50, 0) &&
	     is_cmd(c[1], S2_CMD_MOVE_DIST, 0, VC_PLAN_MM(29)) &&
	     is_cmd(c[2], S2_CMD_TURN, -50, 0) &&
	     is_cmd(c[3], S2_CMD_MOVE_DIST, 0, VC_PLAN_MM(25)) &&
	     is_cmd(c[4], S2_CMD_TURN, -VC_REJOIN_TURN_DEG, 0);
	n = vc_bypass_script(p, -1, c, &lat, NULL);
	ok = ok && n == 5 && lat == 22 && is_cmd(c[0], S2_CMD_TURN, -50, 0) &&
	     is_cmd(c[2], S2_CMD_TURN, 50, 0) &&
	     is_cmd(c[4], S2_CMD_TURN, VC_REJOIN_TURN_DEG, 0);
	/* Wide and far: lat clamps to 60; 166 needed within 100 -> 60 deg */
	p = make_profile(20, 120, 30, 100, 0, 24);	/* whole sweep */
	vc_bypass_script(p, 1, c, &lat, NULL);
	ok = ok && lat == VC_BYPASS_MAX_CM && s2_cmd_angle(c[0]) == 60 &&
	     s2_cmd_dist(c[1]) == VC_PLAN_MM(70) && s2_cmd_dist(c[3]) == VC_PLAN_MM(89);
	/* Too close for any diagonal: falls back to a 90 degree turn out */
	p = make_profile(20, 30, 0, 2, 6, 18);
	vc_bypass_script(p, 1, c, &lat, NULL);
	ok = ok && is_cmd(c[0], S2_CMD_TURN, 90, 0) &&
	     is_cmd(c[1], S2_CMD_MOVE_DIST, 0, VC_PLAN_MM(lat)) &&
	     is_cmd(c[3], S2_CMD_MOVE_DIST, 0, VC_PLAN_MM(2 + VC_OBSTACLE_DEPTH_CM +
		    S5_ROBOT_LENGTH_MM / 20 + VC_PASS_MARGIN_CM));
#else
	n = vc_bypass_script(p, 1, c, &lat, NULL);
	ok = n == 5 && lat == 7 + S5_ROBOT_WIDTH_MM / 20 + S5_CLEARANCE_MARGIN_MM / 10 +
	     VC_BYPASS_EXTRA_CM &&
		 is_cmd(c[0], S2_CMD_TURN, 90, 0) &&
	     is_cmd(c[1], S2_CMD_MOVE_DIST, 0, VC_PLAN_MM(lat)) &&
	     is_cmd(c[2], S2_CMD_TURN, -90, 0) &&
	     is_cmd(c[3], S2_CMD_MOVE_DIST, 0, VC_PLAN_MM(
		    20 + VC_OBSTACLE_DEPTH_CM + S5_ROBOT_LENGTH_MM / 20 +
		    VC_PASS_MARGIN_CM)) &&
	     is_cmd(c[4], S2_CMD_TURN, -VC_REJOIN_TURN_DEG, 0);
	n = vc_bypass_script(p, -1, c, &lat, NULL);
	ok = ok && n == 5 && is_cmd(c[0], S2_CMD_TURN, -90, 0) &&
	     is_cmd(c[2], S2_CMD_TURN, 90, 0) &&
	     is_cmd(c[4], S2_CMD_TURN, VC_REJOIN_TURN_DEG, 0);
	p = make_profile(20, 120, 30, 100, 0, 24);	/* whole sweep */
	vc_bypass_script(p, 1, c, &lat, NULL);
	ok = ok && lat == VC_BYPASS_MAX_CM && s2_cmd_dist(c[3]) == VC_PLAN_MM(VC_PLAN_MAX_CM);
#endif
	TEST_CHECK("VC-U03", ok, "bypass script geometry, sides and clamps"
		   " (19 Nov: angle and distances from the obstacle's width)");

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

	/* VC-U09 (1 Nov): MQTT barcode map: forward, U-turn, left, right */
	{
		static const char m[4] = { 'A', 'F', 'Z', 'W' };
		static const char none[4] = { 0, 0, 0, 0 };

		vc_set_map(m);
		ok = vc_map_nav('A') == NAV_STRAIGHT && vc_map_nav('F') == NAV_U_TURN &&
		     vc_map_nav('Z') == NAV_LEFT && vc_map_nav('W') == NAV_RIGHT &&
		     vc_map_nav('B') == NAV_NONE && vc_map_nav(0) == NAV_NONE;
		vc_set_map(none);		/* the real map comes by MQTT */
		ok = ok && vc_map_nav('A') == NAV_NONE;
		TEST_CHECK("VC-U09", ok, "barcode map: letter -> movement, unmapped = none");
	}

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