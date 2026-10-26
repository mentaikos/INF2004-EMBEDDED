/* Shared behavioural tests: built only by TEST=1 or the host runner.
 * The caller supplies S5_CHECK(id, condition, description). */
#ifndef S5_SCAN_CHECKS_H
#define S5_SCAN_CHECKS_H
#include "obstacle_scan.h"

LOCAL void s5_test_fill(UB *scan, UB r)
{
	UINT i;
	for (i = 0; i < S5_SCAN_BYTES; i++) scan[i] = 0;
	for (i = 0; i < S5_ANGLES; i++) s5_scan_set(scan, i, r);
}

LOCAL BOOL s5_check_packing(void)
{
	UB guarded[S5_SCAN_BYTES + 2];
	UINT i, j, r;
	guarded[0] = 0xa5; guarded[S5_SCAN_BYTES + 1] = 0x5a;
	s5_test_fill(guarded + 1, S5_RANGE_UNSCANNED);
	for (i = 0; i < S5_ANGLES; i++) {
		for (r = 0; r <= 100; r++) {
			if (r == 4) continue;
			s5_scan_set(guarded + 1, i, r);
			for (j = 0; j < S5_ANGLES; j++)
				if (s5_scan_get(guarded + 1, j) !=
				    (j == i ? r : S5_RANGE_UNSCANNED)) return FALSE;
		}
		s5_scan_set(guarded + 1, i, S5_RANGE_UNSCANNED);
	}
	return guarded[0] == 0xa5 && guarded[S5_SCAN_BYTES + 1] == 0x5a &&
		(guarded[S5_SCAN_BYTES] & 0x80u) == 0;
}

LOCAL BOOL s5_check_adaptive(void)
{
	UB scan[S5_SCAN_BYTES];
	UW mask;
	UINT i;
	s5_test_fill(scan, S5_RANGE_UNSCANNED);
	for (i = 0; i < S5_ANGLES; i += 6) s5_scan_set(scan, i, S5_RANGE_FAR);
	if (s5_fine_mask(scan) != 0) return FALSE;
	s5_scan_set(scan, S5_CENTRE, 18);
	mask = s5_fine_mask(scan);
	return (mask & ((UW)1u << 11)) && (mask & ((UW)1u << 13)) &&
		!(mask & ((UW)1u << 1)) && !(mask & ((UW)1u << S5_CENTRE));
}

LOCAL BOOL s5_check_profile(void)
{
	UB scan[S5_SCAN_BYTES];
	S5_PROFILE p;
	UINT i;
	s5_test_fill(scan, S5_RANGE_FAR);
	for (i = 10; i <= 14; i++) s5_scan_set(scan, i, 18);
	p = s5_make_profile(scan);
	return s5_profile_closest(p) == 18 && s5_profile_width(p) == 6 &&
		s5_profile_valid(p) && s5_profile_action(p) == S5_ACT_LEFT &&
		s5_profile_left(p) >= 24 && s5_profile_right(p) >= 24 &&
		(p.w0 >> 30) == 0 && (p.w1 >> 28) == 0;
}

LOCAL BOOL s5_check_side(void)
{
	UB scan[S5_SCAN_BYTES];
	S5_PROFILE p;
	s5_test_fill(scan, S5_RANGE_FAR);
	s5_scan_set(scan, S5_CENTRE, 18);
	s5_scan_set(scan, 18, S5_RANGE_NO_ECHO);
	p = s5_make_profile(scan);
	if (!s5_profile_valid(p) || s5_profile_action(p) != S5_ACT_RIGHT ||
	    s5_profile_left(p) != 0) return FALSE;
	s5_scan_set(scan, 6, S5_RANGE_NO_ECHO);
	p = s5_make_profile(scan);
	return !s5_profile_valid(p) && s5_profile_action(p) == S5_ACT_REVERSE_REATTEMPT;
}

LOCAL BOOL s5_check_turn_envelope(void)
{
	UB scan[S5_SCAN_BYTES];
	S5_PROFILE p;
	s5_test_fill(scan, S5_RANGE_FAR);
	s5_scan_set(scan, S5_CENTRE, 10);
	p = s5_make_profile(scan);
	return !s5_profile_valid(p);        /* side fans clear, turn circle occupied */
}

LOCAL BOOL s5_test_plan(S5_SCAN *s, S5_INPUT *in)
{
	S5_OUTPUT out;
	UINT guard = 0;
	s5_scan_init(s);
	in->stopped = TRUE;
	in->flags = S5_SCAN_REQ;
	s5_scan_tick(s, in, &out);
	in->flags = 0;
	while ((s->state == S5_ST_COARSE_SCAN || s->state == S5_ST_FINE_SCAN) && guard++ < 30) {
		in->now_ms = (UH)(in->now_ms + 60);
		s5_scan_sample(s, in, s->index == S5_CENTRE ? 18 : S5_RANGE_FAR, &out);
	}
	return s->state == S5_ST_PROFILE_PLAN && out.event == 7 && s->plan_valid;
}

LOCAL BOOL s5_check_stop_guard(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	s5_scan_init(&s);
	in.flags = S5_SCAN_REQ;
	s5_scan_tick(&s, &in, &out);
	if (s.state != S5_ST_FORWARD_WATCH) return FALSE;
	in.stopped = TRUE;
	s5_scan_tick(&s, &in, &out);
	if (s.state != S5_ST_COARSE_SCAN || s.index != 0) return FALSE;
	in.flags = 0; in.stopped = FALSE;
	s5_scan_tick(&s, &in, &out);
	return s.state == S5_ST_OBSTACLE_DETECTED && out.event == 8 && s.pending == 0;
}

LOCAL BOOL s5_check_approval(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	if (!s5_test_plan(&s, &in)) return FALSE;
	in.flags = S5_BYPASS_APPROVED;
	in.approval_valid = TRUE;
	in.approval_generation = (UB)(s.generation - 1);
	s5_scan_tick(&s, &in, &out);
	if (s.state != S5_ST_PROFILE_PLAN) return FALSE;
	in.approval_generation = s.generation;
	s5_scan_tick(&s, &in, &out);
	return s.state == S5_ST_BYPASSING && !s.plan_valid;
}

LOCAL BOOL s5_check_expiry(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	in.now_ms = 65000;                 /* exercise 16-bit time wrap */
	if (!s5_test_plan(&s, &in)) return FALSE;
	in.now_ms = (UH)(s.plan_ms + S5_PLAN_TTL_MS);
	in.flags = S5_BYPASS_APPROVED;
	in.approval_valid = TRUE; in.approval_generation = s.generation;
	s5_scan_tick(&s, &in, &out);
	return s.state == S5_ST_OBSTACLE_DETECTED && !s.plan_valid && out.event == 8;
}

LOCAL BOOL s5_check_rescan(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	if (!s5_test_plan(&s, &in)) return FALSE;
	in.flags = S5_SCAN_REQ | S5_BYPASS_APPROVED;
	in.approval_valid = TRUE; in.approval_generation = s.generation;
	s5_scan_tick(&s, &in, &out);
	return s.state == S5_ST_COARSE_SCAN && !s.plan_valid;
}

LOCAL BOOL s5_check_fault(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	UINT i;
	s5_scan_init(&s);
	for (i = 0; i < 3; i++) s5_scan_sample(&s, &in, S5_RANGE_NO_ECHO, &out);
	if (s.state != S5_ST_SENSOR_FAULT || out.event != 9 || out.fault != 1) return FALSE;
	for (i = 0; i < S5_RETRY_LIMIT; i++) {
		if (s5_scan_wants_ping(&s, in.now_ms)) return FALSE;
		in.now_ms = (UH)(in.now_ms + S5_RETRY_MS);
		if (!s5_scan_wants_ping(&s, in.now_ms)) return FALSE;
		s5_scan_sample(&s, &in, S5_RANGE_NO_ECHO, &out);
	}
	return !s5_scan_wants_ping(&s, (UH)(in.now_ms + 10000));
}

LOCAL BOOL s5_check_recovery(BOOL lose_line)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	UINT i;
	s5_scan_init(&s);
	s.state = S5_ST_BYPASSING;
	in.flags = S5_REJOIN_LINE;
	in.line_present = in.line_found = TRUE;
	in.line_seq = 10;
	s5_scan_tick(&s, &in, &out);
	in.flags = 0;
	for (i = 0; i < 4; i++) {
		in.now_ms = (UH)(in.now_ms + 60);
		in.line_ms = in.now_ms;
		/* Same old sequence cannot confirm, even with a fresh timestamp. */
		s5_scan_tick(&s, &in, &out);
		if (out.event == 10) return FALSE;
	}
	for (i = 0; i < 3; i++) {
		in.now_ms = (UH)(in.now_ms + 60);
		in.line_ms = in.now_ms;
		in.line_seq++;
		if (lose_line && i == 1) in.flags = S5_LINE_LOST;
		else in.flags = S5_LINE_FOUND;
		s5_scan_tick(&s, &in, &out);
	}
	return lose_line ? s.state == S5_ST_LINE_RECOVERY :
		s.state == S5_ST_FORWARD_WATCH && out.event == 10;
}

LOCAL BOOL s5_check_recovery_timeout(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	s5_scan_init(&s); s.state = S5_ST_BYPASSING;
	in.now_ms = 65000; in.flags = S5_REJOIN_LINE;
	s5_scan_tick(&s, &in, &out);
	in.flags = S5_LINE_FOUND; in.line_present = in.line_found = TRUE;
	in.line_seq = 1; in.line_ms = 65000;
	in.now_ms = (UH)(65000u + S5_RECOVERY_MS);
	s5_scan_tick(&s, &in, &out);
	return s.state == S5_ST_SENSOR_FAULT && out.event == 9 && out.fault == 2;
}

LOCAL BOOL s5_check_critical(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	s5_scan_init(&s);
	in.in_hump = TRUE;
	s5_scan_sample(&s, &in, 18, &out);
	s5_scan_sample(&s, &in, 18, &out);
	if (out.event >= 0) return FALSE;
	s5_scan_sample(&s, &in, 18, &out);
	if (out.event != 6) return FALSE;     /* persistent obstacle is not masked */
	/* Critical (< 5 cm) needs two consecutive readings: one stray NEAR,
	   then a normal reading, must not stop the robot */
	s5_scan_sample(&s, &in, S5_RANGE_NEAR, &out);
	if (out.estop || out.event >= 0) return FALSE;
	s5_scan_sample(&s, &in, 60, &out);
	s5_scan_sample(&s, &in, S5_RANGE_NEAR, &out);
	if (out.estop || out.event >= 0) return FALSE;
	s5_scan_sample(&s, &in, S5_RANGE_NEAR, &out);
	return out.estop && out.event == 6 && s.state == S5_ST_OBSTACLE_DETECTED;
}

LOCAL BOOL s5_check_estop(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	if (!s5_test_plan(&s, &in)) return FALSE;
	in.flags = S5_BYPASS_APPROVED; in.approval_valid = TRUE;
	in.approval_generation = s.generation; in.estop = TRUE;
	s5_scan_tick(&s, &in, &out);
	return !s.plan_valid && s.state == S5_ST_OBSTACLE_DETECTED;
}

LOCAL BOOL s5_check_continue(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	UINT i;
	s5_scan_init(&s); in.stopped = TRUE; in.flags = S5_SCAN_REQ;
	s5_scan_tick(&s, &in, &out); in.flags = 0;
	for (i = 0; i < 5; i++) s5_scan_sample(&s, &in, S5_RANGE_FAR, &out);
	if (s.state != S5_ST_PROFILE_PLAN || s.action != S5_ACT_CONTINUE) return FALSE;
	s5_scan_tick(&s, &in, &out);
	if (s.state != S5_ST_PROFILE_PLAN) return FALSE;
	in.flags = S5_BYPASS_APPROVED; in.approval_valid = TRUE;
	in.approval_generation = s.generation;
	s5_scan_tick(&s, &in, &out);
	return s.state == S5_ST_FORWARD_WATCH && !s.plan_valid;
}

LOCAL BOOL s5_check_restored(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	s5_scan_init(&s); s.state = S5_ST_SENSOR_FAULT;
	s5_scan_sample(&s, &in, S5_RANGE_FAR, &out);
	if (s.state != S5_ST_OBSTACLE_DETECTED || s.plan_valid || out.event >= 0) return FALSE;
	in.estop = TRUE; in.stopped = TRUE; in.flags = S5_RESET | S5_SCAN_REQ;
	s5_scan_tick(&s, &in, &out);
	return s.state == S5_ST_OBSTACLE_DETECTED && !s.plan_valid;
}

LOCAL BOOL s5_check_stale_line(void)
{
	S5_SCAN s;
	S5_INPUT in = {0};
	S5_OUTPUT out;
	UINT i;
	s5_scan_init(&s); s.state = S5_ST_BYPASSING;
	in.flags = S5_REJOIN_LINE;
	s5_scan_tick(&s, &in, &out); in.flags = S5_LINE_FOUND;
	in.line_present = in.line_found = TRUE;
	for (i = 0; i < 4; i++) {
		in.now_ms = (UH)(in.now_ms + 60); in.line_seq++;
		s5_scan_tick(&s, &in, &out);       /* timestamps deliberately stale */
	}
	return s.state == S5_ST_LINE_RECOVERY && s.line_count == 0 && !s.line_fresh;
}

LOCAL BOOL s5_check_unknown_foreground(void)
{
	UB scan[S5_SCAN_BYTES];
	S5_PROFILE p;
	s5_test_fill(scan, S5_RANGE_FAR);
	s5_scan_set(scan, 10, 18);
	s5_scan_set(scan, S5_CENTRE, S5_RANGE_NO_ECHO);
	p = s5_make_profile(scan);
	if (s5_profile_valid(p)) return FALSE;
	s5_scan_set(scan, S5_CENTRE, 18);
	s5_scan_set(scan, 11, S5_RANGE_NO_ECHO);
	p = s5_make_profile(scan);
	return !s5_profile_valid(p);
}

#define S5_RUN_CHECKS() do { \
	S5_CHECK("S5-U03", s5_range_from_echo(1166) == 20 && s5_range_from_echo(0) == S5_RANGE_NO_ECHO, "time of flight and absent echo"); \
	S5_CHECK("S5-U04", s5_range_from_echo(285) == S5_RANGE_NEAR && s5_range_from_echo(6000) == S5_RANGE_FAR && s5_range_from_echo(30001) == S5_RANGE_NO_ECHO, "near/far/no-echo are distinct"); \
	S5_CHECK("S5-U05", s5_check_packing(), "all 25 packed bins preserve neighbours and guards"); \
	S5_CHECK("S5-U06", s5_angle_deg(0) == 30 && s5_angle_deg(24) == 150 && s5_servo_pulse_us(12) == 1500 && s5_servo_settle_ms(0, 24) >= 240, "servo angles, centre pulse and settling"); \
	S5_CHECK("S5-U07", s5_check_adaptive(), "fine scan refines occupied coarse intervals only"); \
	S5_CHECK("S5-U08", s5_check_profile(), "polar width and footprint-aware side clearance"); \
	S5_CHECK("S5-U09", s5_check_side(), "unknown echoes block affected sides"); \
	S5_CHECK("S5-U10", s5_check_turn_envelope(), "turning envelope must clear too"); \
	S5_CHECK("S5-U11", s5_check_stop_guard(), "scan starts stopped and aborts on movement"); \
	S5_CHECK("S5-U12", s5_check_approval(), "only current generation approval starts bypass"); \
	S5_CHECK("S5-U13", s5_check_expiry(), "expired approval rejected across timer wrap"); \
	S5_CHECK("S5-U14", s5_check_rescan(), "rescan discards a simultaneous old approval"); \
	S5_CHECK("S5-U15", s5_check_fault(), "missing echoes fault with bounded retry rate/count"); \
	S5_CHECK("S5-U16", s5_check_recovery(FALSE), "fresh stable observations confirm rejoin once"); \
	S5_CHECK("S5-U17", s5_check_recovery(TRUE), "line-loss relay cancels stability"); \
	S5_CHECK("S5-U18", s5_check_recovery_timeout(), "3-second recovery timeout across timer wrap"); \
	S5_CHECK("S5-U19", s5_check_critical(), "hump filter preserves persistent/critical hazards"); \
	S5_CHECK("S5-U20", s5_check_estop(), "E-STOP invalidates pending approval"); \
	S5_CHECK("S5-U22", s5_check_continue(), "clear scan still needs VC approval"); \
	S5_CHECK("S5-U23", s5_check_restored(), "restored echo/reset cannot override E-STOP"); \
	S5_CHECK("S5-U24", s5_check_stale_line(), "stale line samples never confirm recovery"); \
	S5_CHECK("S5-U25", s5_check_unknown_foreground(), "unknown forward/foreground blocks turn approval"); \
} while (0)
#endif
