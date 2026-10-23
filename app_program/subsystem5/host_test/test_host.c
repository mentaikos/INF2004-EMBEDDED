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
	for (i = 0; i < S5_ANGLES; i++) {
		double a = s5_angle_deg(i) * 3.141592653589793 / 180.0;
		double r = 180.0 / sin(a), x = r * cos(a);
		if (fabs(x) <= 50) s5_scan_set(scan, i, (UINT)lround(r / 10));
	}
	p = s5_make_profile(scan);
	S5_CHECK("S5-H01", s5_profile_width(p) >= 9 && s5_profile_width(p) <= 11 &&
		s5_profile_closest(p) == 18 && s5_profile_valid(p), "independent 100-mm board scene within 10% width");
	/* Entirely absent echoes must never become a CONTINUE/turn plan. */
	s5_test_fill(scan, S5_RANGE_NO_ECHO);
	p = s5_make_profile(scan);
	S5_CHECK("S5-H02", !s5_profile_valid(p) && s5_profile_closest(p) == S5_RANGE_NO_ECHO,
		"no-echo scene cannot authorise movement");
	printf("[TEST] subsystem5 host SUMMARY %d passed, %d failed; scan=%u B, profile=%u B, state=%u B\n",
		passes, failures, (unsigned)S5_SCAN_BYTES, (unsigned)sizeof(S5_PROFILE), (unsigned)sizeof(S5_SCAN));
	return failures ? 1 : 0;
}
