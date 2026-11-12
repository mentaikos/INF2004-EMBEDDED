/*
 *	vehicle_controller.h
 *	Vehicle Controller (VC) -- the system statechart (design section 5,
 *	FR1).  Public interface; other code includes it as
 *	"../vehicle_controller/vehicle_controller.h".
 *
 *	The VC is the ONLY task that sends motion commands (MOVE / TURN /
 *	STOP / SPEED_LIMIT to Motion Control).  It owns:
 *	  mbf_vc_evt   every buddy's events, one EVT_PACK word each (app.h)
 *	  flg_fault    the team fault flag, FAULT_xxx bits (app.h)
 *	and binds them into every buddy at init (motion_bind_vc, s3_bind_vc,
 *	s4_bind_vc, s5_bind_vc), so they exist before any task runs.
 *	It is also the only code that clears S2_FLG_ESTOP (on a new START).
 *
 *	System statechart (design 7.3):
 *	  INIT -(self-test OK)-> IDLE
 *	  IDLE -(START [IR & IMU calibrated, hazard cleared] / clear E_STOP)->
 *	       RUNNING
 *	  RUNNING = { STARTING (entry: wait for Obstacle Scan ready),
 *	              LINE_FOLLOW (initial), EXECUTE_NAV_CMD, HUMP_TRAVERSE,
 *	              LINE_SEARCH, OBSTACLE_SCAN, BYPASS_OBSTACLE, REJOIN_LINE }
 *	  RUNNING -(E_STOP bit)-> E_STOPPED -(START [hazard cleared])->
 *	       RUNNING.LINE_FOLLOW;  E_STOPPED -(STOP)-> IDLE
 *	  RUNNING -(SENSOR_FAULT, stall, LINE_SEARCH / REJOIN timeout)-> FAULT
 *	       -(STOP = reset [fault cleared])-> IDLE
 *	  RUNNING -(STOP)-> IDLE;  RUNNING -(MISSION_END)-> COMPLETE -(STOP)-> IDLE
 *	  12 Nov: RUNNING -(robot lifted / overturned, S4 has set E_STOP)->
 *	       COMPLETE: lifting the robot ends the run in the demo
 *	  Wi-Fi loss never leaves RUNNING.
 */

#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../app.h"
#include "../subsystem5/obstacle_scan.h"	/* S5_PROFILE */

/* =====================================================================
 *  1. Task
 * ===================================================================== */
#define VC_PRI			12	/* design 7.1 */
#define VC_PRC			TP_PRC1	/* core 0: Motion (10) and IMU (11)
					   are on core 1 */
#define VC_TICK_MS		20	/* wait for an event at most this long */

/* =====================================================================
 *  2. Mission behaviour (PLACEHOLDERS: tune on the track)
 * ===================================================================== */
/* 12 Nov: every speed below is REAL mm/s and every distance REAL mm
   (Motion Control's encoder scale was 0.53 too high, subsystem2.h
   S2_ENC_CPR).  Old value in brackets: x 0.53, so the robot moves
   as before.  Speeds are mm/s because 10.6 cm/s is not a whole cm/s. */
#define VC_CRUISE_MM_S		106	/* (20 cm/s) line following speed (27 Oct: 20 lost the line, slowed to 15;
					   back to 20 after the start-up / sensor-wiring fixes;
					   30 Oct: 30, then 25, then 22 as asked -- if it loses the line on
					   curves, go back down or raise Buddy 3's KP) */
#define VC_SLOW_MM_S		80	/* (15 cm/s = 7.9 real) NFR12: <= 10 cm/s near an obstacle -- now met */
/* 30 Oct: slow zone.  While following, anything the ultrasonic sees
   within this far caps the speed at VC_SLOW_MM_S, released again beyond
   VC_SLOW_ZONE_CM + 10.  At 30 cm/s the robot otherwise reached the
   S5_DETECT_CM (25) stop at full speed and coasted into its own turning
   circle before the scan.  0 = off. */
#define VC_SLOW_ZONE_CM		50
/* 4-5 Nov (videos: ran wide on the loop; IMG_7771: yaw on the
   straight): slow-down while steering.  While following, a Line &
   Barcode steering value of at least VC_CURVE_STEER caps the speed at
   VC_CURVE_MM_S; it is lifted once the steering has stayed below
   VC_CURVE_STEER / 2 for VC_CURVE_HOLD_MS.  Since 4 Nov a straight-line
   edge correction is only 8 (S3_STEER_KP 4), at most ~17 with the KD
   kick and full trim; a curve's growing bend, or a drift that the soft
   correction did not catch, goes past 20.  Slower = a tighter arc for
   the same steering, so the robot gets back to the line before it
   swings across it.  0 = off. */
/* 6 Nov (logs 19:03 / 19:10): "curve" is now decided from the robot's
   REAL turning: left minus right wheel speed (cm/s), averaged over
   VC_CURVE_AVG_N x 20 ms (task_vc.c, check_curve).  Curve at
   VC_CURVE_TURN_MM_S or more; done when below VC_CURVE_DONE_MM_S for
   VC_CURVE_HOLD_MS.  A 13 cm curve at 12 cm/s turns ~9 cm/s; a swing on
   a straight averages ~0.  Simulation (track shapes, 7-15 cm curves):
   all curves held; straights after a curve slow 98 % (line45) ->
   43 % (steering average) -> 24 % (this).  0 = off. */
#define VC_CURVE_TURN_MM_S	26	/* (5 cm/s = 26.5 mm/s) */
#define VC_CURVE_DONE_MM_S	11	/* (2 cm/s = 10.6 mm/s) */
#define VC_CURVE_AVG_N		25	/* 25 x 20 ms = 0.5 s */
#define VC_CURVE_MM_S		64	/* (12 cm/s = 63.5) */
#define VC_CURVE_HOLD_MS	300	/* 6 Nov: 800 for single values; 300 again with the average */
/* 4 Nov: line lost IN a curve (VC_CURVE_STEER or more): keep following
   slowly for this long first -- Line & Barcode keeps arcing towards
   the side the line went (its bend) and usually rolls back onto it --
   and only then stop and sweep (LINE_SEARCH).  Lost on a straight:
   LINE_SEARCH at once, as before. */
#define VC_LOST_GRACE_MS	600
#define VC_TURN_MM_S		80	/* (15 cm/s = 79.5) wheel speed while turning */
#define VC_JUNCTION_ADV_MM	26	/* (5 cm = 26.5 mm) sensors are ahead of the axle */
/* 12 Nov (video 14:26, U-turn): at a junction the robot pivoted with
   its line sensors only just past the cross line -- the wheel axle,
   which it pivots about, was still well before the junction, so it
   turned too early.  The barcode turns (LEFT / RIGHT / U-TURN) now
   drive this far after the junction is seen, to put the axle on it.
   Estimated from the video; set it to the measured distance from the
   line sensors to the wheel axle (+1-2 cm for the junction being seen
   a moment late).  The bypass rejoin keeps VC_JUNCTION_ADV_MM.
   12 Nov: real mm (team: 7 cm, then 9, then 10 cm). */
#define VC_NAV_ADV_MM		100
/* 12 Nov: U-turn (like the ideal video: pivot on the junction, end
   facing back along the same line).  After VC_NAV_ADV_MM, the robot
   turns on the spot (VC_UTURN_TURNS TURN commands of 180, at
   VC_TURN_MM_S) and the turn is ended by sensors, not by the angle:
   the wheels slip in a pivot on the mat (a TURN of 180 gave ~113 deg,
   a slow one far less), so encoder counts say little about the real
   angle.  With VC_UTURN_IMU 1 and the IMU calibrated it is measured by
   the compass (Buddy 4's heading, added up every 20 ms, signed, so
   compass noise cancels):
     1. the line sensors are ignored until VC_UTURN_IMU_BLIND_DEG (past
        the crossing line at ~90 deg);
     2. then the first black (the line it came along, ~180) stops it
        and line following starts;
     3. VC_UTURN_IMU_MAX_DEG with no line, or every TURN done: search.
   IMU not calibrated ("vc: U-turn without the compass"): the first
   TURN is the blind part (~113 deg), the next ones end at the first
   black.  Earlier versions: backup_uturn4 (blind 180 + slow find). */
#define VC_UTURN_IMU		1
#define VC_UTURN_IMU_BLIND_DEG	140
#define VC_UTURN_IMU_MAX_DEG	240
#define VC_UTURN_TURNS		3	/* <= VC_SCRIPT_MAX - 1 */
/* 12 Nov (video IMG_6244: LEFT turned ~50 deg): LEFT and RIGHT at a
   junction work the same way as the U-turn, with the compass: the
   line sensors are ignored until VC_LR_IMU_BLIND_DEG (off the line
   the robot came along, which carries on past a crossing), then the
   first black -- the side line, ~90 deg -- stops the turn and line
   following starts.  VC_LR_IMU_MAX_DEG with no line: search.  The
   compass read ~1.24 x the real angle (U-turn: 223 for ~180), so
   45 is ~36 real and 160 ~130 real.  IMU not calibrated: the old
   fixed TURN of 90, then line following.  0 = always the old way. */
#define VC_LR_IMU		1
#define VC_LR_IMU_BLIND_DEG	45
#define VC_LR_IMU_MAX_DEG	160
#define VC_LR_TURNS		2	/* TURN commands of 180 (as commanded; ~113 deg each on the mat) */
#define VC_JUNCTION_COOLDOWN_MS	800	/* same junction is not counted twice */
#define VC_LINE_SEARCH_MS	3000	/* NFR18: LINE_SEARCH timeout */
#define VC_SEARCH_STEP_DEG	30	/* sweep: +30, -60, +90, -120 ... */
#define VC_SEARCH_STEPS		6
#define VC_LOST_CHECK_MS	2500	/* LINE_FOLLOW: still LOST this long after entry
					   (27 Oct: was 300.  > S3_LINE_LOST_MS, since S3
					   is already LOST after idling centred: give the
					   seek drift time to touch an edge first) */
#define VC_HUMP_MAX_MS		5000	/* HUMP_END never came: carry on */
#define VC_REVERSE_CM		10	/* NO_CLEAR: back off at least this, rescan */
/* 30 Oct (video IMG_0775): on a corner the obstacle came into view only
   ~6-8 cm away, inside the robot's own turning circle (16.9 cm, S5), so
   the plan said REVERSE; two fixed 10 cm reverses were not enough and
   the robot stopped (FAULT).  Now the reverse is long enough to put the
   obstacle about VC_REVERSE_TO_CM away (from the profile's closest
   point), between VC_REVERSE_CM and VC_REVERSE_MAX_CM. */
#define VC_REVERSE_TO_CM	22
#define VC_REVERSE_MAX_CM	30
#define VC_MAX_RESCANS		3	/* then FAULT (obstacle not passable); 30 Oct: was 2 */
/* After backing off, also turn this far before the rescan, towards the
   side Obstacle Scan saw more room on (30 Oct): a straight reverse keeps
   the same viewing angle, so a scan spoilt by the angle (round or slanted
   obstacle, robot not square to it) failed again every time.  0 = only
   reverse, as before. */
#define VC_RESCAN_TURN_DEG	20
#define VC_RESCAN_TURN_MAX_DEG	40	/* total for one obstacle (VC_MAX_RESCANS is 3) */
#define VC_SCAN_TIMEOUT_MS	5000	/* SCAN_REQ -> PROFILE / NO_CLEAR */
#define VC_APPROVE_WAIT_MS	500	/* approval -> S5 BYPASSING */
#define VC_S5_READY_MS		1000	/* START -> S5 FORWARD_WATCH */
#define VC_S5_SETTLE_MS		120	/* >= 2 S5 periods after s5_reset() */
#define VC_STEP_TIMEOUT_MS	15000	/* one MOVE(d)/TURN without MOTION_DONE:
				   127 cm at 10-20 cm/s needs ~7-13 s */
#define VC_OBSTACLE_DEPTH_CM	10	/* assumed obstacle depth (not measurable;
					   30 Oct: was 25, then 15; rejoined too late) */
/* 30 Oct: drive past only until the robot's centre is this far beyond
   the obstacle's back (+ half the robot length), then turn back to the
   line -- was the whole robot length.  Turning on the spot there swings
   the near front corner forward, past the obstacle, and the rear away
   from it. */
#define VC_PASS_MARGIN_CM	3
#define VC_REJOIN_EXTRA_CM	4	/* drive this far past the expected line */
#define VC_BYPASS_MIN_CM	20
#define VC_BYPASS_EXTRA_CM	4	/* 30 Oct: was 12; with the obstacle's edge
					   now measured, not doubled, 12 went too wide */
#define VC_BYPASS_MAX_CM	60
/* Rejoin after a bypass (28 Oct, curved track): the last bypass turn is
   only this many degrees towards the line (not 90), so the robot meets
   it at an angle and can turn onto it; the approach is up to
   2 x sideways distance + VC_REJOIN_EXTRA_CM (a curve may have moved
   the line further away), and if the line is still not found a normal
   LINE_SEARCH follows instead of waiting for Obstacle Scan's time-out. */
#define VC_REJOIN_TURN_DEG	60
/* 30 Oct: when the line is found after the obstacle the robot no longer
   turns a fixed VC_REJOIN_TURN_DEG back: it brings the axle onto the
   line and turns towards the bypass side until the line sensors find
   the line again, at most this far, then follows it.  Works on curves. */
#define VC_ALIGN_MAX_DEG	150
#define VC_ALIGN_MM_S		53	/* (10 cm/s) wheel speed of that turn: slower, so
					   the sensors do not overshoot the line */
/* 30 Oct: line not found on the rejoin (approach driven, or the turn
   onto it lost it): look for it instead of the normal 3 s LINE_SEARCH.
   Turn VC_REJOIN_LOOK_DEG to one side, back, the other side, back; no
   line: reverse VC_REJOIN_BACK_MM and look again; VC_REJOIN_TRIES looks
   in all, then FAULT.  Line seen within VC_REJOIN_ALONG_DEG of the
   track's direction: just follow it, else turn onto it as above.
   VC_REJOIN_TRIES is at most 3 (2-bit counter). */
#define VC_REJOIN_TRIES		3
#define VC_REJOIN_LOOK_DEG	60
#define VC_REJOIN_BACK_MM	53	/* (10 cm) */
#define VC_REJOIN_ALONG_DEG	30
/* Smooth bypass (28-29 Oct): 1 = turn out only as far as needed to miss
   the obstacle's near corner (45..90 deg, from the profile: tan(a) >=
   (corner sideways + half robot + margin + VC_BYPASS_CORNER_CM) /
   distance ahead), drive that diagonal, straighten, pass.  Less turning
   = less turn error and less stop-start.  Too close for a diagonal ->
   90 as before.  0 = the team's 90-degree path, exactly as on d4cd515.
   The rejoin (VC_REJOIN_TURN_DEG, approach, LINE_SEARCH) is the same
   either way. */
#define VC_BYPASS_SMOOTH	1
#define VC_BYPASS_CORNER_CM	5	/* extra room at the near corner */
/* 12 Nov: the bypass, rejoin-approach and reverse distances are
   planned in cm from the ultrasonic (real cm) with the margins above,
   all tuned while the robot drove only 0.53 of every distance.  To
   keep those paths exactly as tuned, the planned cm are driven as
   cm x VC_PLAN_MM_PER_CM_X10 / 10 mm (5.3 mm per planned cm), and
   VC_PLAN_MAX_CM (127) is the old longest move.  10 here = drive the
   planned geometry at full size (wider, longer bypass). */
#define VC_PLAN_MM_PER_CM_X10	53
#define VC_PLAN_MAX_CM		127
#define VC_PLAN_MM(cm)		((INT)(cm) * VC_PLAN_MM_PER_CM_X10 / 10)

/* START needs the IMU calibrated and an IMU fault stops the mission
   (design).  0 = bench mode ONLY: drive without a working IMU (no hump
   height).  Never 0 on the track or in a demo. */
/* Bench only: START by itself this long after IDLE (0 = off, the
   normal setting: START comes from the dashboard).  For a test build
   without WiFi.  The car drives off on its own: keep it on the track. */
#ifndef VC_AUTO_START_MS
#define VC_AUTO_START_MS	0
#endif

#define VC_REQUIRE_IMU		1

/* Obstacle Scan (Buddy 5, ultrasonic + servo).  1 = normal.  0 = bench
   mode ONLY: Obstacle Scan's task is not started (its objects still are),
   and the VC treats the path ahead as always clear -- NO obstacle
   detection, NO stop for an obstacle (NFR1, NFR10, NFR12 not met).
   For testing line following and motion while the ultrasonic is being
   fixed.  Never 0 on the track with people or obstacles, or in a demo. */
#define VC_USE_ULTRASONIC	1

/* Bench test of straight driving (S2-M03), independent of the line
   sensors: 0 = normal.  N (1..127) = on START, drive straight N cm with
   heading hold, then stop and go back to IDLE.  Line events are ignored
   during the move.  Keep 0 for normal runs. */
#define VC_BENCH_MOVE_CM	0	/* 12 Nov: real cm (max 102) */

/* 1 Nov: barcode letter -> movement.  The mapping is NOT fixed in the
   code: it arrives by MQTT before the demo (robot/cmd MAP, Buddy 1) and
   can be sent again at any time.  Until it arrives, a junction after a
   barcode goes straight on.  VC_BENCH_MAP is only for a no-WiFi bench
   build: "" (normal) or 4 letters forward, U-turn, left, right (e.g.
   "AFZW") used until an MQTT MAP replaces them. */
#ifndef VC_BENCH_MAP
#define VC_BENCH_MAP		""
#endif

/* Finish condition (assumption A9, still open): 0 = only EVT_MISSION_END
   ends the mission; N = the N-th junction ends it. */
#define VC_FINISH_JUNCTIONS	0

/* =====================================================================
 *  3. States (4 bits) and reasons (4 bits)
 * ===================================================================== */
#define VC_ST_INIT		0
#define VC_ST_IDLE		1
#define VC_ST_STARTING		2	/* RUNNING entry */
#define VC_ST_LINE_FOLLOW	3
#define VC_ST_EXECUTE_NAV_CMD	4
#define VC_ST_HUMP_TRAVERSE	5
#define VC_ST_LINE_SEARCH	6
#define VC_ST_OBSTACLE_SCAN	7
#define VC_ST_BYPASS_OBSTACLE	8
#define VC_ST_REJOIN_LINE	9
#define VC_ST_E_STOPPED		10
#define VC_ST_FAULT		11
#define VC_ST_COMPLETE		12

#define VC_IS_RUNNING(st)	((st) >= VC_ST_STARTING && (st) <= VC_ST_REJOIN_LINE)

#define VC_R_NONE		0
#define VC_R_SENSOR_FAULT	1	/* SENSOR_FAULT event (bits in vc_get_faults) */
#define VC_R_LINE_TIMEOUT	2	/* LINE_SEARCH > 3 s */
#define VC_R_NO_CLEAR		3	/* no side clear after the rescans */
#define VC_R_STEP_TIMEOUT	4	/* MOTION_DONE never came */
#define VC_R_SCAN_TIMEOUT	5	/* Obstacle Scan never answered */
#define VC_R_NOT_CALIBRATED	6	/* START refused: IR or IMU */
#define VC_R_HAZARD		7	/* START refused: hazard still there */
#define VC_R_SCAN_NOT_READY	8	/* START refused: S5 not watching */
#define VC_R_STOP		9	/* STOP received */
#define VC_R_FAULT_ACTIVE	10	/* reset refused: fault still there */

/* =====================================================================
 *  4. g_vc_status -- one 32-bit packed word, written ONLY by the VC
 *	[3:0]   state       VC_ST_xxx
 *	[5:4]   nav         pending NAV_xxx
 *	[6]     nav_valid   a barcode command is waiting for its junction
 *	[11:7]  peak_hump   highest HUMP_END height this mission, mm (0..31)
 *	[14:12] step        step of the running MOVE/TURN script, 0..7
 *	[18:15] reason      VC_R_xxx of the last refusal / fault / stop
 *	[22:19] junctions   junctions passed, wraps at 16
 *	[24:23] rescans     obstacle rescans of this obstacle, 0..3
 *	[31:25] reserved, 0
 * ===================================================================== */
#define VC_STS_STATE_SHIFT	0
#define VC_STS_NAV_SHIFT	4
#define VC_STS_NAVOK_SHIFT	6
#define VC_STS_HUMP_SHIFT	7
#define VC_STS_STEP_SHIFT	12
#define VC_STS_REASON_SHIFT	15
#define VC_STS_JCT_SHIFT	19
#define VC_STS_RESCAN_SHIFT	23

Inline UW vc_sts_pack(UINT state, UINT nav, UINT nav_valid, UINT hump_mm,
		      UINT step, UINT reason, UINT junctions, UINT rescans)
{
	if (hump_mm > 31u) hump_mm = 31u;
	if (step > 7u) step = 7u;
	if (rescans > 3u) rescans = 3u;
	return ((UW)(state & 0xFu) << VC_STS_STATE_SHIFT) |
	       ((UW)(nav & 0x3u) << VC_STS_NAV_SHIFT) |
	       ((UW)(nav_valid & 1u) << VC_STS_NAVOK_SHIFT) |
	       ((UW)hump_mm << VC_STS_HUMP_SHIFT) |
	       ((UW)step << VC_STS_STEP_SHIFT) |
	       ((UW)(reason & 0xFu) << VC_STS_REASON_SHIFT) |
	       ((UW)(junctions & 0xFu) << VC_STS_JCT_SHIFT) |
	       ((UW)rescans << VC_STS_RESCAN_SHIFT);
}

Inline UINT vc_sts_state(UW w)     { return (w >> VC_STS_STATE_SHIFT) & 0xFu; }
Inline UINT vc_sts_nav(UW w)       { return (w >> VC_STS_NAV_SHIFT) & 0x3u; }
Inline UINT vc_sts_nav_valid(UW w) { return (w >> VC_STS_NAVOK_SHIFT) & 1u; }
Inline UINT vc_sts_hump(UW w)      { return (w >> VC_STS_HUMP_SHIFT) & 0x1Fu; }
Inline UINT vc_sts_step(UW w)      { return (w >> VC_STS_STEP_SHIFT) & 0x7u; }
Inline UINT vc_sts_reason(UW w)    { return (w >> VC_STS_REASON_SHIFT) & 0xFu; }
Inline UINT vc_sts_junctions(UW w) { return (w >> VC_STS_JCT_SHIFT) & 0xFu; }
Inline UINT vc_sts_rescans(UW w)   { return (w >> VC_STS_RESCAN_SHIFT) & 0x3u; }

/* =====================================================================
 *  5. Public objects and functions
 * ===================================================================== */
IMPORT ID		mbf_vc_evt;	/* send: UW w = EVT_PACK(...), TMO_POL */
IMPORT ID		flg_fault;	/* FAULT_xxx bits, owners set their own */
IMPORT volatile UW	g_vc_status;

IMPORT ER	vc_init(void);		/* phase 1, after subsystemN_init() */
IMPORT ER	vc_start(void);		/* phase 2 */
#if APP_TEST
IMPORT void	vc_test(void);
#endif

/* Post an event without waiting (Buddy 1's Command Handler: EVT_START /
   EVT_STOP; the console; tests). */
IMPORT ER	vc_post(UINT type, UW payload);

IMPORT UW	vc_get_status(void);	/* = g_vc_status */

/* START guards; VC_R_NONE = START would be accepted.  verbose prints
   each failing guard on the console. */
IMPORT UINT	vc_start_check(BOOL verbose);
IMPORT UINT	vc_get_faults(void);	/* SENSOR_FAULT bits of the last fault */

typedef struct {
	UH	evt_rx;		/* events received (wraps) */
	UH	evt_ignored;	/* events not valid in the current state */
	UH	cmd_fail;	/* motion commands mbf_motion_cmd refused */
} VC_DIAG;
IMPORT void	vc_get_diag(VC_DIAG *d);

/* =====================================================================
 *  6. Pure planning helpers (vc_plan.c), no kernel calls: tested by
 *     vc_test.c.  side: +1 = right, -1 = left.
 * ===================================================================== */
#define VC_SCRIPT_MAX		6

/* Barcode command at a junction -> MOVE/TURN script; 0 = keep following */
IMPORT INT	vc_nav_script(UINT nav, UW *cmds);

/* 1 Nov: barcode letter -> movement mapping (set by MQTT MAP).
   vc_set_map: letters[] = forward, U-turn, left, right ('A'..'Z').
   vc_map_nav: NAV_xxx for a letter, or NAV_NONE if it is not mapped. */
IMPORT void	vc_set_map(const char letters[4]);
IMPORT UINT	vc_map_nav(char letter);

/* Bypass round the obstacle on one side, from Obstacle Scan's profile:
   turn out, sideways, turn back, past it, turn towards the line.
   *lat_cm gets the sideways distance (the approach uses it). */
IMPORT INT	vc_bypass_script(S5_PROFILE p, INT side, UW *cmds, INT *lat_cm);

/* Relative angle of LINE_SEARCH sweep i (0..), first towards 'side' */
IMPORT INT	vc_search_turn(UINT i, INT side);

/* TRUE if Obstacle Scan's status shows something within 20 cm */
IMPORT BOOL	vc_range_hazard(UW s5_status);

/* Private to the VC */
IMPORT void	vc_task(INT stacd, void *exinf);

#endif /* VEHICLE_CONTROLLER_H */