/*
 *	vc_plan.c
 *	Vehicle Controller -- pure planning helpers (no kernel calls), so the
 *	tests can check them without moving the robot.
 *
 *	All motion commands are Buddy 2's packed 32-bit words
 *	(S2_CMD_PACK: type 3, speed 6, angle 9 signed, distance 8 signed);
 *	TURN angle + = right (clockwise), distance - = reverse.
 */

#include "vehicle_controller.h"
#include "../subsystem2/subsystem2.h"

LOCAL INT vc_clamp(INT v, INT lo, INT hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

EXPORT INT vc_nav_script(UINT nav, UW *cmds)
{
	INT	angle;

	switch (nav) {
	case NAV_LEFT:   angle = -90;  break;
	case NAV_RIGHT:  angle = 90;   break;
	case NAV_U_TURN: angle = 180;  break;
	default:         return 0;	/* NAV_STRAIGHT: keep following */
	}
	/* Bring the axle over the junction first: the IR sensors are in
	   front of it, so turning at once would cut the corner. */
	cmds[0] = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_SLOW_CM_S, 0, VC_JUNCTION_ADV_CM);
	cmds[1] = S2_CMD_PACK(S2_CMD_TURN, VC_TURN_CM_S, angle, 0);
	return 2;
}

/*
 * Profile (Buddy 5, obstacle_scan.h): closest point (x right, y forward)
 * and visible width, in cm.  Where the closest point lies inside the
 * visible width is not known, so the far edge on the chosen side is
 * taken as x + width (right) or x - width (left): conservative.
 *
 *	1. TURN a towards the side (a = 90, or 45..85 with VC_BYPASS_SMOOTH)
 *	2. MOVE lat / sin(a): lat = edge + half robot width + margin
 *	3. TURN back                         heading 0
 *	4. MOVE past: y + obstacle depth + half robot length + margin
 *	   - forward part of 2 (30 Oct: was the whole robot length)
 *	5. TURN towards the line             heading -+VC_REJOIN_TURN_DEG
 * Then REJOIN_LINE drives towards the line (lat + extra) and turns back
 * along it when Line & Barcode sees it.
 */
/* tan and sin of 45, 50, ... 90 degrees (x100, x1000).  No libm. */
LOCAL const UH tan100[]  = {100, 119, 143, 173, 214, 275, 373, 567, 1143};
LOCAL const UH sin1000[] = {707, 766, 819, 866, 906, 940, 966, 985, 996, 1000};

LOCAL INT vc_sin1000(INT deg)
{
	return sin1000[(vc_clamp(deg, 45, 90) - 45) / 5];
}

/* Smallest of 45, 50, ... 90 whose diagonal is 'need' cm sideways by the
   time it is 'ahead' cm forward: tan(a) >= need / ahead */
LOCAL INT vc_out_deg(INT need, INT ahead)
{
	INT	i;

	if (ahead <= 0) return 90;
	for (i = 0; i < 9; i++) {
		if ((INT)tan100[i] * ahead >= need * 100) return 45 + 5 * i;
	}
	return 90;
}

EXPORT INT vc_bypass_script(S5_PROFILE p, INT side, UW *cmds, INT *lat_cm)
{
	INT	x = s5_profile_x(p);
	INT	w = (INT)s5_profile_width(p);
	INT	y = (INT)s5_profile_y(p);
	INT	edge, lat, fwd, a = 90, diag, out_fwd = 0;

	side = (side < 0) ? -1 : 1;
	edge = (side > 0) ? x + w : -(x - w);
	if (edge < 0) edge = 0;
	lat = vc_clamp(edge + S5_ROBOT_WIDTH_MM / 20 + S5_CLEARANCE_MARGIN_MM / 10 +
		       VC_BYPASS_EXTRA_CM,
		       VC_BYPASS_MIN_CM, VC_BYPASS_MAX_CM);
	fwd = y + VC_OBSTACLE_DEPTH_CM + S5_ROBOT_LENGTH_MM / 20 + VC_PASS_MARGIN_CM;
	diag = lat;
#if VC_BYPASS_SMOOTH
	/* Turn out only as far as needed to miss the near corner (the
	   obstacle's edge, y cm ahead of the sensor).  The robot's centre is
	   another half length behind the sensor, which covers its own front
	   corner, so y is used as the room ahead. */
	a = vc_out_deg(edge + S5_ROBOT_WIDTH_MM / 20 + S5_CLEARANCE_MARGIN_MM / 10 +
		       VC_BYPASS_CORNER_CM, y);
	if (a < 90) {
		diag = (lat * 1000 + vc_sin1000(a) - 1) / vc_sin1000(a);
		out_fwd = lat * 100 / tan100[(a - 45) / 5];
	}
#endif
	diag = vc_clamp(diag, VC_BYPASS_MIN_CM, S2_DIST_MAX_CM);
	fwd = vc_clamp(fwd - out_fwd, VC_BYPASS_MIN_CM, S2_DIST_MAX_CM);

	cmds[0] = S2_CMD_PACK(S2_CMD_TURN, VC_TURN_CM_S, a * side, 0);
	cmds[1] = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_SLOW_CM_S, 0, diag);
	cmds[2] = S2_CMD_PACK(S2_CMD_TURN, VC_TURN_CM_S, -a * side, 0);
	cmds[3] = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_SLOW_CM_S, 0, fwd);
	cmds[4] = S2_CMD_PACK(S2_CMD_TURN, VC_TURN_CM_S, -VC_REJOIN_TURN_DEG * side, 0);
	if (lat_cm != NULL) *lat_cm = lat;
	return 5;
}

/* Heading after sweep i: +30, -30, +60, -60, +90, -90 (first towards
   side), so each relative turn is +-30 * (i + 1), alternating. */
EXPORT INT vc_search_turn(UINT i, INT side)
{
	INT	a = VC_SEARCH_STEP_DEG * (INT)(i + 1);

	side = (side < 0) ? -1 : 1;
	return ((i & 1u) ? -a : a) * side;
}

EXPORT BOOL vc_range_hazard(UW s5_status)
{
	UINT	range = (s5_status >> 11) & 127u;	/* obstacle_scan.h status */

	if (range == S5_RANGE_NEAR) return TRUE;
	return (range >= S5_RANGE_MIN_CM && range <= S5_DETECT_CM) ? TRUE : FALSE;
}