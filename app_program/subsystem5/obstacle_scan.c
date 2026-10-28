/*
 *	obstacle_scan.c -- integer ranging, polar geometry and statechart 6.5.
 *	No hardware/kernel calls: the same code runs in host and target tests.
 */
#include "obstacle_scan.h"

#ifndef S5_HOST_TEST
#include "../app.h"
#else
/* app.h's existing four-bit event contract; no second wire format. */
#define EVT_OBSTACLE_DETECTED 6
#define EVT_OBSTACLE_PROFILE 7
#define EVT_NO_CLEAR 8
#define EVT_SENSOR_FAULT 9
#define EVT_REJOIN_LINE_CONFIRMED 10
#endif

/* sin(0,5,...,90 degrees), Q15. No floating point or libm on the Pico. */
LOCAL const UH sin5[] = {
	0, 2856, 5690, 8481, 11207, 13848, 16384, 18794, 21063,
	23170, 25102, 26842, 28378, 29698, 30792, 31651, 32270,
	32643, 32768
};

LOCAL INT clamp(INT v, INT lo, INT hi)
{
	return v < lo ? lo : v > hi ? hi : v;
}

LOCAL INT sin_deg(INT deg)
{
	INT sign = deg < 0 ? -1 : 1;
	if (deg < 0) deg = -deg;
	if (deg > 90) deg = 180 - deg;
	deg = clamp(deg, 0, 90);
	/* Linear between the 5-degree entries: exact on multiples of 5, and
	   bins that are not (S5_SWEEP_HALF_DEG != 60) are not rounded down */
	if (deg >= 90) return sign * sin5[18];
	return sign * (sin5[deg / 5] +
		((INT)sin5[deg / 5 + 1] - (INT)sin5[deg / 5]) * (deg % 5) / 5);
}

LOCAL INT project(INT mm, INT degrees)
{
	return (mm * sin_deg(degrees)) / 32768;
}

EXPORT UB s5_range_from_echo(UW width_us)
{
	UW mm;
	if (width_us < S5_ECHO_MIN_US || width_us > S5_ECHO_TIMEOUT_US)
		return S5_RANGE_NO_ECHO;
	/* d = c*t/2. c=343 m/s: mm = us*343/2000, rounded once.
	 * Classify BEFORE cm rounding so a 49 mm echo stays NEAR. */
	mm = (width_us * 343u + 1000u) / 2000u;
	if (mm < S5_RANGE_MIN_CM * 10u) return S5_RANGE_NEAR;
	if (mm > S5_RANGE_MAX_CM * 10u) return S5_RANGE_FAR;
	return (UB)((mm + 5u) / 10u);
}

EXPORT UINT s5_angle_deg(UINT index)
{
	/* 90 - half .. 90 + half over bins 0..24, rounded to whole degrees */
	return (90u - S5_SWEEP_HALF_DEG) +
		(2u * S5_SWEEP_HALF_DEG * (UINT)clamp((INT)index, 0, S5_ANGLES - 1) +
		 (S5_ANGLES - 1) / 2u) / (S5_ANGLES - 1);
}

EXPORT UINT s5_servo_pulse_us(UINT index)
{
	UINT deg = s5_angle_deg(index);
	if (S5_SERVO_REVERSED) deg = 180u - deg;
	return S5_SERVO_MIN_US +
		((S5_SERVO_MAX_US - S5_SERVO_MIN_US) * deg + 90u) / 180u;
}

EXPORT UINT s5_servo_settle_ms(UINT old_index, UINT new_index)
{
	INT delta = (INT)s5_angle_deg(old_index) - (INT)s5_angle_deg(new_index);
	if (delta < 0) delta = -delta;
	return S5_SERVO_BASE_MS + (UINT)delta * S5_SERVO_MS_PER_DEG;
}

EXPORT void s5_scan_set(UB *scan, UINT index, UINT range)
{
	UINT bit, offset;
	if (index >= S5_ANGLES) return;
	if (range > 100u || range == 4u) range = S5_RANGE_NO_ECHO;
	for (bit = 0; bit < 7; bit++) {
		offset = index * 7u + bit;
		scan[offset >> 3] = (UB)((scan[offset >> 3] & ~(1u << (offset & 7u))) |
			(((range >> bit) & 1u) << (offset & 7u)));
	}
}

EXPORT UB s5_scan_get(const UB *scan, UINT index)
{
	UINT bit, offset, range = 0;
	if (index >= S5_ANGLES) return S5_RANGE_UNSCANNED;
	for (bit = 0; bit < 7; bit++) {
		offset = index * 7u + bit;
		range |= ((scan[offset >> 3] >> (offset & 7u)) & 1u) << bit;
	}
	return (UB)range;
}

LOCAL BOOL measured(UINT r)
{
	return r >= S5_RANGE_MIN_CM && r <= S5_RANGE_MAX_CM;
}

LOCAL BOOL uncertain(UINT r)
{
	return r == S5_RANGE_NO_ECHO || r == S5_RANGE_UNSCANNED;
}

EXPORT UW s5_fine_mask(const UB *scan)
{
	UINT i, j, a, b, nearest = 101;
	UW mask = 0;
	for (i = 0; i < S5_ANGLES; i += 6) {
		a = s5_scan_get(scan, i);
		if (measured(a) && a < nearest) nearest = a;
	}
	for (i = 0; i < 24; i += 6) {
		a = s5_scan_get(scan, i);
		b = s5_scan_get(scan, i + 6);
		if (uncertain(a) || uncertain(b) || a == S5_RANGE_NEAR ||
		    b == S5_RANGE_NEAR || (nearest <= 30 &&
		    ((measured(a) && a <= nearest + S5_PROFILE_BAND_CM) ||
		     (measured(b) && b <= nearest + S5_PROFILE_BAND_CM)))) {
			for (j = i; j <= i + 6; j++)
				if (s5_scan_get(scan, j) == S5_RANGE_UNSCANNED)
					mask |= (UW)1u << j;
		}
	}
	return mask;
}

/* Required turning circle: ceil(sqrt((width/2)^2+(length/2)^2))
 * plus measured sensing/motion allowance. Small bounded integer loop. */
LOCAL UINT turn_radius_mm(void)
{
	UINT r = 0;
	UINT w = (S5_ROBOT_WIDTH_MM + 1) / 2;
	UINT l = (S5_ROBOT_LENGTH_MM + 1) / 2;
	while (r * r < w * w + l * l) r++;
	return r + S5_CLEARANCE_MARGIN_MM + S5_RANGE_ERROR_MM;
}

LOCAL UINT required_width_mm(void)
{
	return S5_ROBOT_WIDTH_MM + 2 * (S5_CLEARANCE_MARGIN_MM + S5_RANGE_ERROR_MM);
}

/* Free wedge outside a visible edge. Missing echoes invalidate that side.
 * Unscanned fine bins are covered only by their measured coarse endpoints;
 * no coarse bin may be absent. A 15-degree beam allowance shrinks the fan.
 * Chord = 2*r*sin(fan/2); floor to 5 degrees is conservative. */
LOCAL UINT side_clearance(const UB *scan, UINT edge, BOOL left, UINT band)
{
	INT i, step = left ? 1 : -1;
	UINT r, radius = 1000, span;
	if ((!left && edge == 0) || (left && edge == 24)) return 0;
	for (i = (INT)edge + step; i >= 0 && i < S5_ANGLES; i += step) {
		r = s5_scan_get(scan, (UINT)i);
		if (r == S5_RANGE_UNSCANNED && i % 6 != 0) continue;
		if (uncertain(r) || r == S5_RANGE_NEAR || (measured(r) && r <= band))
			return 0;
		if (measured(r) && r * 10u < radius) radius = r * 10u;
	}
	span = left ? s5_angle_deg(24) - s5_angle_deg(edge)
		    : s5_angle_deg(edge) - s5_angle_deg(0);
	if (span <= S5_BEAM_ALLOW_DEG) return 0;
	span -= S5_BEAM_ALLOW_DEG;
	radius = radius > S5_RANGE_ERROR_MM ? radius - S5_RANGE_ERROR_MM : 0;
	return (UINT)(2 * project((INT)radius, (INT)(span / 2u)));
}

EXPORT S5_PROFILE s5_make_profile(const UB *scan)
{
	S5_PROFILE p = {0, 0};
	UINT i, r, closest = 101, nearest_i = S5_CENTRE, lo, hi;
	UINT left = 0, right = 0, width = 0, action = S5_ACT_STOP, valid = 0;
	INT x = 0, y = 0, xmin = 1001, xmax = -1001, px;
	BOOL unknown = FALSE, near = FALSE, foreground_known = TRUE;
	for (i = 0; i < S5_ANGLES; i++) {
		r = s5_scan_get(scan, i);
		if (r == S5_RANGE_NEAR) near = TRUE;
		if (r == S5_RANGE_NO_ECHO || (i % 6 == 0 && uncertain(r))) unknown = TRUE;
		if (measured(r) && r < closest) { closest = r; nearest_i = i; }
	}
	lo = hi = nearest_i;
	if (!near && !unknown && closest > S5_DETECT_CM) {
		action = S5_ACT_CONTINUE;
		valid = 1;
	} else if (!near && closest <= S5_DETECT_CM) {
		/* Foreground is the nearest range band; extrema are visible surface
		 * points only. Hidden depth is deliberately not inferred. */
		for (i = 0; i < S5_ANGLES; i++) {
			r = s5_scan_get(scan, i);
			if (!measured(r) || r > closest + S5_PROFILE_BAND_CM) continue;
			if (i < lo) lo = i;
			if (i > hi) hi = i;
			px = project((INT)r * 10, 90 - (INT)s5_angle_deg(i));
			if (px < xmin) xmin = px;
			if (px > xmax) xmax = px;
		}
		width = (UINT)(xmax - xmin + 5) / 10u;
		for (i = lo; i <= hi; i++)
			if (uncertain(s5_scan_get(scan, i))) foreground_known = FALSE;
		/* An unknown forward/foreground return cannot establish a clear
		 * turning envelope, even when one outer side fan looks open. */
		if (uncertain(s5_scan_get(scan, S5_CENTRE))) foreground_known = FALSE;
		left = side_clearance(scan, hi, TRUE, closest + S5_PROFILE_BAND_CM);
		right = side_clearance(scan, lo, FALSE, closest + S5_PROFILE_BAND_CM);
		/* All measured bearings must clear the in-place turning envelope. */
		if (foreground_known && closest * 10u > turn_radius_mm()) {
			if (left >= required_width_mm() || right >= required_width_mm()) {
				action = left >= right ? S5_ACT_LEFT : S5_ACT_RIGHT;
				valid = 1;
			}
		}
		if (!valid) action = S5_ACT_REVERSE_REATTEMPT;
	}
	if (closest <= 100) {
		x = project((INT)closest * 10, 90 - (INT)s5_angle_deg(nearest_i)) / 10;
		y = project((INT)closest * 10, (INT)s5_angle_deg(nearest_i)) / 10;
	} else closest = S5_RANGE_FAR;
	if (near) closest = S5_RANGE_NEAR;
	else if (unknown && closest == S5_RANGE_FAR) closest = S5_RANGE_NO_ECHO;
	p.w0 = (UW)closest | ((UW)clamp((INT)width, 0, 200) << 7) |
		((UW)(x & 255) << 15) | ((UW)clamp(y, 0, 100) << 23);
	p.w1 = (UW)clamp((INT)left / 10, 0, 100) |
		((UW)clamp((INT)right / 10, 0, 100) << 7) |
		((UW)lo << 14) | ((UW)hi << 19) | ((UW)action << 24) | ((UW)valid << 27);
	return p;
}

EXPORT UW s5_profile_event(S5_PROFILE p, UB generation)
{
	return (UW)s5_profile_action(p) | ((UW)s5_profile_width(p) << 3) |
		((UW)s5_profile_closest(p) << 11) | ((UW)generation << 18) |
		((UW)(s5_profile_left(p) * 10u >= required_width_mm()) << 26) |
		((UW)(s5_profile_right(p) * 10u >= required_width_mm()) << 27);
}

LOCAL void output_init(S5_OUTPUT *out)
{
	out->event = -1; out->payload = 0; out->estop = FALSE; out->fault = 0;
}

LOCAL BOOL scanning(const S5_SCAN *s)
{
	return s->state == S5_ST_COARSE_SCAN || s->state == S5_ST_FINE_SCAN;
}

LOCAL void invalidate(S5_SCAN *s)
{
	s->plan_valid = 0;
	s->profile.w1 &= ~((UW)1u << 27);
	s->pending = 0;
	s->index = S5_CENTRE;
	s->action = S5_ACT_STOP;
	s->line_count = s->line_fresh = 0;
}

LOCAL void fault(S5_SCAN *s, UH now, UINT reason, S5_OUTPUT *out)
{
	invalidate(s);
	s->state = S5_ST_SENSOR_FAULT;
	s->invalid_count = s->retries = 0;
	s->retry_ms = now;
	out->event = EVT_SENSOR_FAULT;
	out->fault = reason;
}

EXPORT void s5_scan_init(S5_SCAN *s)
{
	UINT i;
	UB *bytes = (UB *)s;
	for (i = 0; i < sizeof(*s); i++) bytes[i] = 0;
	s->state = S5_ST_FORWARD_WATCH;
	s->index = S5_CENTRE;
	s->range = S5_RANGE_NO_ECHO;
	for (i = 0; i < S5_ANGLES; i++) s5_scan_set(s->scan, i, S5_RANGE_UNSCANNED);
}

LOCAL void begin_scan(S5_SCAN *s)
{
	UINT i;
	invalidate(s);
	s->state = S5_ST_COARSE_SCAN;
	s->invalid_count = 0;
	for (i = 0; i < S5_ANGLES; i++) s5_scan_set(s->scan, i, S5_RANGE_UNSCANNED);
	s->pending = ((UW)1u << 0) | ((UW)1u << 6) | ((UW)1u << 12) |
		((UW)1u << 18) | ((UW)1u << 24);
	s->index = 0;
}

EXPORT void s5_scan_tick(S5_SCAN *s, const S5_INPUT *in, S5_OUTPUT *out)
{
	output_init(out);
	/* Integration fix (VC): a latched echo fault stops pinging after
	   S5_RETRY_LIMIT retries, and nothing else could re-arm it, because
	   the fault also holds E-STOP and RESET was only accepted with
	   E-STOP clear -- so START stayed refused for ever, even after the
	   sensor was fixed.  RESET now re-arms the retry pings while in
	   SENSOR_FAULT.  This moves nothing and clears no flag: the state
	   leaves SENSOR_FAULT only on a valid echo (s5_scan_sample). */
	if ((in->flags & S5_RESET) && s->state == S5_ST_SENSOR_FAULT) {
		s->retries = 0;
		s->retry_ms = (UH)(in->now_ms - S5_RETRY_MS);
	}
	if ((in->flags & S5_CANCEL) || in->estop) {
		if (s->state != S5_ST_FORWARD_WATCH && s->state != S5_ST_SENSOR_FAULT)
			s->state = S5_ST_OBSTACLE_DETECTED;
		invalidate(s);
		return;
	}
	if ((in->flags & S5_RESET) && in->stopped) {
		/* Preserve generation across resets: old approvals stay obsolete. */
		UB generation = (UB)(s->generation + 1u);
		s5_scan_init(s);
		s->generation = generation;
		return;
	}
	if (scanning(s) && !in->stopped) {
		invalidate(s);
		s->state = S5_ST_OBSTACLE_DETECTED;
		out->event = EVT_NO_CLEAR;
		return;
	}
	if ((in->flags & S5_SCAN_REQ) && in->stopped && s->state != S5_ST_SENSOR_FAULT) {
		begin_scan(s);
		return;                       /* discard simultaneous stale approval */
	}
	if (s->state == S5_ST_PROFILE_PLAN) {
		if (!in->stopped || (UH)(in->now_ms - s->plan_ms) >= S5_PLAN_TTL_MS) {
			invalidate(s);
			s->state = S5_ST_OBSTACLE_DETECTED;
			out->event = EVT_NO_CLEAR;
		} else if ((in->flags & S5_BYPASS_APPROVED) && in->approval_valid &&
			   in->approval_generation == s->generation && s->plan_valid) {
			s->plan_valid = 0;             /* approval is single use */
			s->profile.w1 &= ~((UW)1u << 27);
			s->state = s->action == S5_ACT_CONTINUE ? S5_ST_FORWARD_WATCH : S5_ST_BYPASSING;
			s->index = S5_CENTRE;
		}
	}
	if (s->state == S5_ST_BYPASSING && (in->flags & S5_REJOIN_LINE)) {
		s->state = S5_ST_LINE_RECOVERY;
		s->recovery_ms = in->now_ms;
		s->last_line_seq = in->line_seq;  /* never reuse pre-recovery LINE_FOUND */
		s->line_count = s->line_fresh = 0;
		return;
	}
	if (s->state == S5_ST_LINE_RECOVERY) {
		if ((UH)(in->now_ms - s->recovery_ms) >= S5_RECOVERY_MS) {
			fault(s, in->now_ms, 2, out);
			return;
		}
		s->line_fresh = in->line_present &&
			(UH)(in->now_ms - in->line_ms) <= S5_LINE_FRESH_MS;
		if (!s->line_fresh || !in->line_found || (in->flags & S5_LINE_LOST))
			s->line_count = 0;
		else if (in->line_seq != s->last_line_seq) {
			if (s->line_count == 0) s->line_start_ms = in->now_ms;
			if (s->line_count < 3) s->line_count++;
			if (s->line_count >= 3 &&
			    (UH)(in->now_ms - s->line_start_ms) >= S5_LINE_CONFIRM_MS) {
				out->event = EVT_REJOIN_LINE_CONFIRMED;
				out->payload = s->generation;
				s->state = S5_ST_FORWARD_WATCH;
				s->action = S5_ACT_CONTINUE;
			}
		}
		s->last_line_seq = in->line_seq;
	}
}

LOCAL void finish_scan(S5_SCAN *s, UH now, S5_OUTPUT *out)
{
	s->profile = s5_make_profile(s->scan);
	s->action = (UB)s5_profile_action(s->profile);
	s->plan_valid = (UB)s5_profile_valid(s->profile);
	s->index = S5_CENTRE;
	s->generation++;
	s->plan_ms = now;
	if (s->plan_valid) {
		s->state = S5_ST_PROFILE_PLAN;
		out->event = EVT_OBSTACLE_PROFILE;
		out->payload = s5_profile_event(s->profile, s->generation);
	} else {
		s->state = S5_ST_OBSTACLE_DETECTED;
		out->event = EVT_NO_CLEAR;
		out->payload = s->action;
	}
}

EXPORT void s5_scan_sample(S5_SCAN *s, const S5_INPUT *in, UB range, S5_OUTPUT *out)
{
	UINT i, confirm;
	output_init(out);
	if (range > 100u || range == 4 || range == S5_RANGE_UNSCANNED)
		range = S5_RANGE_NO_ECHO;
	s->range = range;
	/* A physical near return is never suppressed by hump masking, but it
	 * must be seen on two consecutive pings: one stray echo (noise, TRIG
	 * crosstalk) no longer E-STOPs the robot.  Cost: one 60 ms ping, about
	 * 6 mm at the 10 cm/s cap that applies within 20 cm (NFR12). */
	if (range == S5_RANGE_NEAR || (measured(range) && range * 10u <= S5_CRITICAL_MM)) {
		if (!s->crit_seen) {
			s->crit_seen = 1;
			return;             /* unconfirmed: this ping is ignored */
		}
		s->crit_seen = 0;
		out->estop = TRUE;
		invalidate(s);
		s->state = S5_ST_OBSTACLE_DETECTED;
		out->event = EVT_OBSTACLE_DETECTED;
		out->payload = (UW)range | ((UW)10u << 7);
		return;
	}
	s->crit_seen = 0;
	if (s->state == S5_ST_SENSOR_FAULT) {
		if (s->retries < S5_RETRY_LIMIT) s->retries++;
		s->retry_ms = in->now_ms;
		if (range != S5_RANGE_NO_ECHO) {
			/* Observation restored, not authorisation to move. No flag clears. */
			s->state = S5_ST_OBSTACLE_DETECTED;
			s->invalid_count = 0;
		}
		return;
	}
	if (range == S5_RANGE_NO_ECHO) {
		if (s->invalid_count < 3) s->invalid_count++;
		if (s->invalid_count >= S5_INVALID_LIMIT) {
			fault(s, in->now_ms, 1, out);
			return;
		}
	} else s->invalid_count = 0;
	if (scanning(s)) {
		if (!in->stopped || in->estop) {
			invalidate(s);
			s->state = S5_ST_OBSTACLE_DETECTED;
			out->event = EVT_NO_CLEAR;
			return;
		}
		s5_scan_set(s->scan, s->index, range);
		s->pending &= ~((UW)1u << s->index);
		if (s->pending == 0 && s->state == S5_ST_COARSE_SCAN) {
			s->pending = s5_fine_mask(s->scan);
			s->state = S5_ST_FINE_SCAN;
		}
		if (s->pending == 0) { finish_scan(s, in->now_ms, out); return; }
		for (i = 0; i < S5_ANGLES; i++)
			if (s->pending & ((UW)1u << i)) { s->index = (UB)i; break; }
	} else if (s->state == S5_ST_FORWARD_WATCH || s->state == S5_ST_BYPASSING ||
		   s->state == S5_ST_LINE_RECOVERY) {
		if (range == S5_RANGE_NO_ECHO || !measured(range) || range > S5_DETECT_CM)
			s->near_count = 0;
		else {
			/* Reject a short hump reflection, never mask a persistent hazard
			 * for the whole hump. Bypass/recovery use the normal two samples. */
			confirm = in->in_hump && s->state == S5_ST_FORWARD_WATCH ? 3u : 2u;
			if (s->near_count < 3) s->near_count++;
			if (s->near_count >= confirm) {
				invalidate(s);
				s->state = S5_ST_OBSTACLE_DETECTED;
				out->event = EVT_OBSTACLE_DETECTED;
				out->payload = (UW)range | ((UW)10u << 7);
			}
		}
	}
}

EXPORT BOOL s5_scan_wants_ping(const S5_SCAN *s, UH now_ms)
{
	if (s->state != S5_ST_SENSOR_FAULT) return TRUE;
	return s->retries < S5_RETRY_LIMIT && (UH)(now_ms - s->retry_ms) >= S5_RETRY_MS;
}

EXPORT UW s5_scan_status(const S5_SCAN *s)
{
	return (UW)s->state | ((UW)s->action << 3) | ((UW)s->index << 6) |
		((UW)s->range << 11) | ((UW)s->plan_valid << 18) |
		((UW)s->line_fresh << 19) | ((UW)s->line_count << 20) |
		((UW)s->generation << 22);
}