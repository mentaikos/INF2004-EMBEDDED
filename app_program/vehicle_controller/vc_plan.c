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
 * Profile (Buddy 5, obstacle_scan.h): closest point (x right, y forward),
 * visible width and the outermost bearings that saw the obstacle.  The
 * edge on the chosen side comes from those bearings (vc_edge_cm).
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

/* cos(0, 5, ... 90 degrees) x1000 */
LOCAL const UH cos1000[] = {1000, 996, 985, 966, 940, 906, 866, 819, 766, 707,
			    643, 574, 500, 423, 342, 259, 174, 87, 0};

/* cos of a sweep angle (0..180, + = right of the centre line) x1000 */
LOCAL INT vc_cos1000(INT deg)
{
	INT	s = 1;

	if (deg > 90) { deg = 180 - deg; s = -1; }
	deg = vc_clamp(deg, 0, 90);
	return s * (INT)cos1000[(deg + 2) / 5];
}

/* How far the obstacle reaches sideways on the bypass side, cm from the
   robot's centre line (30 Oct).  Was x + width: the whole visible width
   added to the closest point, which counts the obstacle twice when the
   closest point is its middle (a bottle came out ~17 cm wide and the
   robot went ~40 cm out).  Now: from the outermost bearing that still
   saw the obstacle (the profile's first/last edge bin), one bin further
   out for the 5-degree steps, on a face square to the robot at the
   closest distance: sideways = closest x cot(angle).  Never more than
   the old estimate.  Obstacle reaching the end of the sweep: the old
   estimate (its far edge was not seen). */
LOCAL INT vc_edge_cm(S5_PROFILE p, INT side)
{
	INT	x = s5_profile_x(p);
	INT	w = (INT)s5_profile_width(p);
	INT	r = (INT)s5_profile_closest(p);
	INT	old = (side > 0) ? x + w : -(x - w);
	INT	i, deg, c, s, e;

	if (r < S5_RANGE_MIN_CM || r > S5_RANGE_MAX_CM) return old;
	i = (side > 0) ? (INT)((p.w1 >> 14) & 31u) - 1	/* right: lo - 1 */
		       : (INT)((p.w1 >> 19) & 31u) + 1;	/* left:  hi + 1 */
	if (i < 0 || i >= S5_ANGLES) return old;
	deg = (INT)s5_angle_deg((UINT)i);
	c = vc_cos1000(deg);			/* + right */
	s = vc_cos1000(90 - deg < 0 ? deg - 90 : 90 - deg);	/* sin, > 0 */
	if (s < 100) return old;		/* almost sideways */
	e = r * c / s;				/* sideways, + right */
	if (side < 0) e = -e;
	return (e < old) ? e : old;
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
	(void)x; (void)w;
	edge = vc_edge_cm(p, side);
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