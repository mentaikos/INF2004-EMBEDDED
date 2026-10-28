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
 *	1. TURN 90 towards the side          heading +-90
 *	2. MOVE lat: edge + half robot width + margin
 *	3. TURN back                         heading 0
 *	4. MOVE past: y + obstacle depth + robot length
 *	5. TURN towards the line             heading -+VC_REJOIN_TURN_DEG
 * Then REJOIN_LINE drives towards the line (lat + extra) and turns back
 * along it when Line & Barcode sees it.
 */
EXPORT INT vc_bypass_script(S5_PROFILE p, INT side, UW *cmds, INT *lat_cm)
{
	INT	x = s5_profile_x(p);
	INT	w = (INT)s5_profile_width(p);
	INT	y = (INT)s5_profile_y(p);
	INT	edge, lat, fwd;

	side = (side < 0) ? -1 : 1;
	edge = (side > 0) ? x + w : -(x - w);
	if (edge < 0) edge = 0;
	lat = vc_clamp(edge + S5_ROBOT_WIDTH_MM / 20 + S5_CLEARANCE_MARGIN_MM / 10 +
		       VC_BYPASS_EXTRA_CM,
		       VC_BYPASS_MIN_CM, VC_BYPASS_MAX_CM);
	fwd = vc_clamp(y + VC_OBSTACLE_DEPTH_CM + S5_ROBOT_LENGTH_MM / 10,
		       VC_BYPASS_MIN_CM, S2_DIST_MAX_CM);

	cmds[0] = S2_CMD_PACK(S2_CMD_TURN, VC_TURN_CM_S, 90 * side, 0);
	cmds[1] = S2_CMD_PACK(S2_CMD_MOVE_DIST, VC_SLOW_CM_S, 0, lat);
	cmds[2] = S2_CMD_PACK(S2_CMD_TURN, VC_TURN_CM_S, -90 * side, 0);
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