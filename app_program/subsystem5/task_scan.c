/* Buddy 5 -- public VC handoff and cyclic-paced Micro T-Kernel task. */
#include "scan_task.h"
#include "drv_ultrasonic.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem3/subsystem3.h"
#include <tk/syslib.h>

EXPORT ID tid_s5_scan;
LOCAL ID vc_events, vc_faults;           /* init-time binding only */
LOCAL S5_SNAPSHOT published;
LOCAL UH published_ms;
LOCAL UH approval;                     /* generation 8 + valid 1 */
LOCAL UW line_sample;                   /* ms16, seq8, found1, present1 */

LOCAL UH now_ms(void)
{
	SYSTIM now;
	tk_get_tim(&now);
	return (UH)now.lo;
}

LOCAL void motion_inputs(S5_INPUT *in)
{
	T_RFLG safety;
	in->stopped = s2_sts_stopped(motion_get_status());
	in->estop = tk_ref_flg(flg_safety, &safety) != E_OK ||
		(safety.flgptn & S2_FLG_ESTOP) != 0;
}

EXPORT void s5_bind_vc(ID events, ID faults)
{
	vc_events = events;
	vc_faults = faults;
}

EXPORT ER s5_get_status(S5_SNAPSHOT *snapshot)
{
	ER er;
	if (snapshot == NULL) return E_PAR;
	if (mtx_s5_status <= 0) return E_OBJ;
	er = tk_loc_mtx(mtx_s5_status, TMO_FEVR);
	if (er < E_OK) return er;
	*snapshot = published;
	return tk_unl_mtx(mtx_s5_status);
}

LOCAL ER request(UINT flags)
{
	return flg_scan > 0 ? tk_set_flg(flg_scan, flags) : E_OBJ;
}

EXPORT ER s5_request_scan(void) { return request(S5_SCAN_REQ); }
EXPORT ER s5_rejoin_line(void) { return request(S5_REJOIN_LINE); }
EXPORT ER s5_cancel(void) { return request(S5_CANCEL); }
EXPORT ER s5_reset(void) { return request(S5_RESET); }

EXPORT ER s5_approve_plan(UB generation)
{
	ER er;
	if (mtx_s5_status <= 0) return E_OBJ;
	er = tk_loc_mtx(mtx_s5_status, TMO_FEVR);
	if (er < E_OK) return er;
	if (s5_sts_state(published.status) != S5_ST_PROFILE_PLAN ||
	    !s5_sts_plan_valid(published.status) ||
	    s5_sts_generation(published.status) != generation ||
	    (UH)(now_ms() - published_ms) >= S5_PLAN_TTL_MS) er = E_OBJ;
	else {
		approval = (UH)generation | 0x100u;
		er = request(S5_BYPASS_APPROVED);
	}
	tk_unl_mtx(mtx_s5_status);
	return er;
}

EXPORT ER s5_line_observe(BOOL found)
{
	ER er;
	UB sequence;
	if (mtx_s5_status <= 0) return E_OBJ;
	er = tk_loc_mtx(mtx_s5_status, TMO_FEVR);
	if (er < E_OK) return er;
	sequence = (UB)((line_sample >> 16) + 1u);
	line_sample = (UW)now_ms() | ((UW)sequence << 16) |
		((UW)(found != FALSE) << 24) | ((UW)1u << 25);
	er = request(found ? S5_LINE_FOUND : S5_LINE_LOST);
	tk_unl_mtx(mtx_s5_status);
	return er;
}

EXPORT void s5_scan_cyclic(void *exinf)
{
	(void)exinf;
	if (tid_s5_scan > 0) tk_wup_tsk(tid_s5_scan);
}

LOCAL void inputs(S5_INPUT *in)
{
	UINT flags = 0;
	UW line;
	UH approved;
	/* Consume metadata and flags together under the short status mutex. */
	tk_loc_mtx(mtx_s5_status, TMO_FEVR);
	tk_wai_flg(flg_scan, S5_SCAN_FLAGS, TWF_ORW | TWF_BITCLR, &flags, TMO_POL);
	line = line_sample;
	approved = approval;
	approval = 0;
	tk_unl_mtx(mtx_s5_status);
	in->now_ms = now_ms();
	in->flags = flags;
	motion_inputs(in);
	in->in_hump = s4_in_hump();
	in->approval_valid = (approved & 0x100u) != 0;
	in->approval_generation = (UB)approved;
	in->line_ms = (UH)line;
	in->line_seq = (UB)(line >> 16);
	in->line_found = (line >> 24) & 1u;
	in->line_present = (line >> 25) & 1u;
}

LOCAL void publish(const S5_SCAN *scan, UH drops, UH overruns);

/* TEMPORARY tuning aid: after every finished scan print what each angle
   saw and the decision, so a REVERSE / NO_CLEAR can be explained from the
   log.  Range codes: 0 = under 5 cm, 1 = over 100 cm (clear), 2 = no
   echo, 3 = not scanned, else cm.  Set S5_DEBUG_SCAN to 0 when done. */
#ifndef S5_DEBUG_SCAN
#define S5_DEBUG_SCAN		1	/* 29 Oct: muted for barcodes; 30 Oct: back on for obstacles */
#endif
#if S5_DEBUG_SCAN
LOCAL void print_scan(const S5_SCAN *scan, INT event)
{
	static const char *act[] = { "STOP", "CONTINUE", "LEFT", "RIGHT",
				     "REVERSE", "?", "?", "?" };
	S5_PROFILE p = scan->profile;
	UINT i;

	tm_printf((UB*)"s5: scan");
	for (i = 0; i < S5_ANGLES; i++) {
		tm_printf((UB*)" %d:%d", s5_angle_deg(i), s5_scan_get(scan->scan, i));
	}
	tm_printf((UB*)"\ns5: -> %s %s closest=%dcm width=%dcm x=%d y=%d"
		  " clearL=%dcm clearR=%dcm (need %dcm) edges %d..%d\n",
		  act[s5_profile_action(p) & 7u],
		  event == EVT_OBSTACLE_PROFILE ? "(plan)" : "(NO_CLEAR)",
		  s5_profile_closest(p), s5_profile_width(p),
		  s5_profile_x(p), s5_profile_y(p),
		  s5_profile_left(p), s5_profile_right(p),
		  (S5_ROBOT_WIDTH_MM + 2 * (S5_CLEARANCE_MARGIN_MM +
					    S5_RANGE_ERROR_MM)) / 10,
		  s5_angle_deg((p.w1 >> 14) & 31u), s5_angle_deg((p.w1 >> 19) & 31u));
}
#endif

LOCAL void deliver(const S5_OUTPUT *out, const S5_SCAN *scan, UH *drops, UH overruns)
{
	UW payload = out->payload, word;
	UINT fault = (out->fault & 1u ? FAULT_NO_ECHO : 0) |
		(out->fault & 2u ? FAULT_LINE_TIMEOUT : 0);
	ER er;
	if (out->estop || fault) CHECK(tk_set_flg(flg_safety, S2_FLG_ESTOP), "S5 safety");
	if (fault && vc_faults > 0) CHECK(tk_set_flg(vc_faults, fault), "S5 fault");
	if (out->event < 0) return;
#if S5_DEBUG_SCAN
	if (out->event == EVT_OBSTACLE_PROFILE || out->event == EVT_NO_CLEAR)
		print_scan(scan, out->event);
#endif
	/* The higher-priority VC can run immediately after tk_snd_mbf.
	 * Publish first so that its event and profile describe the same scan. */
	publish(scan, *drops, overruns);
	if (out->event == EVT_SENSOR_FAULT) payload = fault;
	word = EVT_PACK(out->event, payload);
	er = vc_events > 0 ? tk_snd_mbf(vc_events, &word, sizeof(word), TMO_POL) : E_OBJ;
	if (er < E_OK) {
		(*drops)++;
		/* Losing a stop/hold notification must not leave motors running. */
		if (out->event == EVT_OBSTACLE_DETECTED || out->event == EVT_NO_CLEAR ||
		    out->event == EVT_SENSOR_FAULT)
			CHECK(tk_set_flg(flg_safety, S2_FLG_ESTOP), "S5 event drop");
	}
}

LOCAL void publish(const S5_SCAN *scan, UH drops, UH overruns)
{
	tk_loc_mtx(mtx_s5_status, TMO_FEVR);
	published.status = s5_scan_status(scan);
	published.profile = scan->profile;
	published.event_drops = drops;
	published.overruns = overruns;
	published_ms = scan->plan_ms;         /* original plan time, not refresh time */
	tk_unl_mtx(mtx_s5_status);
}

/* Forward watch on curves: the angle index to watch while following.
   Line & Barcode's steering (+ = right), averaged over ~0.5 s (one call
   per 60 ms cycle, 1/8 per call, Q4), says which way and how sharply
   the track bends; look up to S5_WATCH_MAX_DEG that way (obstacle_scan.h).
   Prints the look angle when it changes, for tuning.  Index 12 is straight ahead and
   lower indices point right (S5_SERVO_REVERSED handles the mounting). */
LOCAL W steer_avg_q4;

LOCAL UINT watch_index(BOOL moving)
{
	S3_STS_FIELDS f;
	W avg, deg;
	INT bins;
	if (!moving) {
		steer_avg_q4 = 0;
		return S5_CENTRE;
	}
	s3_sts_unpack(s3_get_status(), &f);
	steer_avg_q4 += ((W)f.steer * 16 - steer_avg_q4) / 8;
	avg = steer_avg_q4 / 16;
	if (avg > S5_WATCH_DEADBAND) deg = (avg - S5_WATCH_DEADBAND) * S5_WATCH_GAIN;
	else if (avg < -S5_WATCH_DEADBAND) deg = (avg + S5_WATCH_DEADBAND) * S5_WATCH_GAIN;
	else deg = 0;
	if (deg > S5_WATCH_MAX_DEG) deg = S5_WATCH_MAX_DEG;
	if (deg < -S5_WATCH_MAX_DEG) deg = -S5_WATCH_MAX_DEG;
	bins = (INT)(deg * (S5_ANGLES - 1) / (2 * S5_SWEEP_HALF_DEG));
#if S5_DEBUG_SCAN
	{
		static INT last_bins;
		if (bins != last_bins) {
			tm_printf((UB*)"s5: watch %s %d deg (steer avg %d)\n",
				  bins > 0 ? "right" : bins < 0 ? "left" : "ahead",
				  (bins < 0 ? -bins : bins) * 2 * S5_SWEEP_HALF_DEG /
				  (S5_ANGLES - 1), (INT)avg);
			last_bins = bins;
		}
	}
#endif
	return (UINT)(S5_CENTRE - bins);	/* + steer (right) -> lower index */
}

EXPORT void s5_scan_task(INT stacd, void *exinf)
{
	S5_SCAN scan;
	S5_INPUT in;
	S5_OUTPUT out;
	UINT pointed = S5_CENTRE, settle_ms = S5_SERVO_BASE_MS;
	UW point_us, ping_us = 0, gap_us;
	UH drops = 0, overruns = 0;
	UB range;
	ER er;
	BOOL pinged = FALSE;
	(void)stacd; (void)exinf;
	s5_scan_init(&scan);
	publish(&scan, drops, overruns);
	er = drv_us_init();
	if (er < E_OK) {
		CHECK(er, "S5 driver init");
		out.event = EVT_SENSOR_FAULT; out.payload = 0; out.estop = TRUE; out.fault = 1;
		scan.state = S5_ST_SENSOR_FAULT;
		deliver(&out, &scan, &drops, overruns);
		publish(&scan, drops, overruns);
		for (;;) tk_slp_tsk(TMO_FEVR);
	}
	point_us = in_w(TIMER_TIMERAWL);
	for (;;) {
		tk_slp_tsk(TMO_FEVR);
		if (tk_can_wup(TSK_SELF) > 0) overruns++; /* no catch-up ping bursts */
		inputs(&in);
		s5_scan_tick(&scan, &in, &out);
		deliver(&out, &scan, &drops, overruns);
		/* Following the line: look into the bend (curves) */
		if (scan.state == S5_ST_FORWARD_WATCH)
			scan.index = (UB)watch_index(!in.stopped);
		if (scan.index != pointed) {
			settle_ms = s5_servo_settle_ms(pointed, scan.index);
			er = drv_us_point(scan.index);
			CHECK(er, "S5 servo");
			pointed = scan.index;
			point_us = in_w(TIMER_TIMERAWL);
		}
		/* Physical TRIG-to-TRIG >=60 ms, even with scheduler jitter. Large
 * servo movements get extra settling time, as required by their angle. */
		if ((UW)(in_w(TIMER_TIMERAWL) - point_us) >= settle_ms * 1000u &&
		    s5_scan_wants_ping(&scan, in.now_ms)) {
			/* A slightly early cyclic release must not turn a 60-ms ping
			 * into a skipped 120-ms interval. Yield for the remaining gap
			 * (rounded by the 10-ms tick), then recheck Motion/safety. */
			gap_us = (UW)(in_w(TIMER_TIMERAWL) - ping_us);
			if (pinged && gap_us < S5_PERIOD_MS * 1000u)
				tk_dly_tsk((S5_PERIOD_MS * 1000u - gap_us + 999u) / 1000u);
			motion_inputs(&in);
			if ((scan.state == S5_ST_COARSE_SCAN || scan.state == S5_ST_FINE_SCAN) &&
			    (!in.stopped || in.estop)) {
				in.flags = 0;
				s5_scan_tick(&scan, &in, &out);
			} else {
				ping_us = in_w(TIMER_TIMERAWL);
				pinged = TRUE;
				range = drv_us_ping();
				in.now_ms = now_ms();
				in.stopped = s2_sts_stopped(motion_get_status());
				s5_scan_sample(&scan, &in, range, &out);
			}
			deliver(&out, &scan, &drops, overruns);
			/* Start the next servo movement while waiting for the next cyclic
			 * release: a settled 5-degree step then needs only one 60-ms slot. */
			if (scan.index != pointed) {
				settle_ms = s5_servo_settle_ms(pointed, scan.index);
				CHECK(drv_us_point(scan.index), "S5 servo next");
				pointed = scan.index;
				point_us = in_w(TIMER_TIMERAWL);
			}
		}
		publish(&scan, drops, overruns);
	}
}