/* Build instructions in ../README.md. Same behavioural suite as TEST=1. */
#include <stdio.h>
#include <math.h>
#include "../scan_checks.h"

static int failures, passes;
#define S5_CHECK(id, condition, desc) do { \
	if (condition) { passes++; printf("[TEST] %s PASS  %s\n", id, desc); } \
	else { failures++; printf("[TEST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); } \
} while (0)

int main(void)
{
	UB scan[S5_SCAN_BYTES];
	S5_PROFILE p;
	UINT i;
	S5_RUN_CHECKS();
	/* Independent ray/flat-board scene: 100-mm board, 180 mm ahead.
 * Floating point generates test observations only; production is integer. */
	s5_test_fill(scan, S5_RANGE_FAR);
	/* 16 Nov: with the beam -- a bin echoes if any ray within
	   +-S5_WIDTH_BEAM_BINS x 5 deg of it hits the board (nearest hit) */
	for (i = 0; i < S5_ANGLES; i++) {
		double best = 1e9;
		int d;
		for (d = -50 * S5_WIDTH_BEAM_BINS; d <= 50 * S5_WIDTH_BEAM_BINS; d++) {
			double a = (s5_angle_deg(i) + d / 10.0) * 3.141592653589793 / 180.0;
			double r = 180.0 / sin(a), x = r * cos(a);
			if (fabs(x) <= 50 && r < best) best = r;
		}
		if (best < 1e8) s5_scan_set(scan, i, (UINT)lround(best / 10));
	}
	p = s5_make_profile(scan);
	S5_CHECK("S5-H01", s5_profile_width(p) >= 9 && s5_profile_width(p) <= 11 &&
		s5_profile_closest(p) == 18 && s5_profile_valid(p), "independent 100-mm board scene within 10% width");
	/* Entirely absent echoes must never become a CONTINUE/turn plan. */
	s5_test_fill(scan, S5_RANGE_NO_ECHO);
	p = s5_make_profile(scan);
	S5_CHECK("S5-H02", !s5_profile_valid(p) && s5_profile_closest(p) == S5_RANGE_NO_ECHO,
		"no-echo scene cannot authorise movement");
	/* 19 Nov: real scans of the 20 cm box (track, floor echoes at 23 cm
	   on both sides).  Both sides are open: a way round, never REVERSE
	   -- at 19 cm (the floor was taken as obstacle all round) and at
	   16 cm (inside the turning check measured from the sensor). */
	{
		static const UB box19[S5_ANGLES] = {23, 23, 23, 23, 20, 20, 20, 20, 20, 19, 19,
			20, 20, 20, 20, 20, 20, 20, 20, 20, 23, 23, 23, 23, 23};
		static const UB box16[S5_ANGLES] = {23, 23, 23, 23, 17, 17, 17, 17, 17, 16, 16,
			17, 17, 17, 17, 17, 17, 17, 17, 17, 23, 23, 23, 23, 23};
		UINT k, a19, a16;

		for (k = 0; k < S5_ANGLES; k++) s5_scan_set(scan, k, box19[k]);
		p = s5_make_profile(scan);
		a19 = s5_profile_action(p);
		printf("  20 cm box at 19 cm: action %u, width %u cm\n", a19, s5_profile_width(p));
		S5_CHECK("S5-H03", s5_profile_valid(p) &&
			(a19 == S5_ACT_LEFT || a19 == S5_ACT_RIGHT) &&
			s5_profile_width(p) >= 14 && s5_profile_width(p) <= 24,
			"real 20 cm box, floor echoes beside it: a way round, not REVERSE");
		for (k = 0; k < S5_ANGLES; k++) s5_scan_set(scan, k, box16[k]);
		p = s5_make_profile(scan);
		a16 = s5_profile_action(p);
		S5_CHECK("S5-H04", s5_profile_valid(p) &&
			(a16 == S5_ACT_LEFT || a16 == S5_ACT_RIGHT),
			"the same box after coasting to 16 cm: a way round, not REVERSE");
	}
	printf("[TEST] subsystem5 host SUMMARY %d passed, %d failed; scan=%u B, profile=%u B, state=%u B\n",
		passes, failures, (unsigned)S5_SCAN_BYTES, (unsigned)sizeof(S5_PROFILE), (unsigned)sizeof(S5_SCAN));
	return failures ? 1 : 0;
}
