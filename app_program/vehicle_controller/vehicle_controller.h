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
#define VC_CRUISE_CM_S		15	/* line following speed (27 Oct: 20 lost the line, slowed for stability) */
#define VC_SLOW_CM_S		10	/* NFR12: <= 10 cm/s near an obstacle */
#define VC_TURN_CM_S		10	/* wheel speed while turning (27 Oct: was 15) */
#define VC_JUNCTION_ADV_CM	5	/* sensors are ahead of the axle */
#define VC_JUNCTION_COOLDOWN_MS	800	/* same junction is not counted twice */
#define VC_LINE_SEARCH_MS	3000	/* NFR18: LINE_SEARCH timeout */
#define VC_SEARCH_STEP_DEG	30	/* sweep: +30, -60, +90, -120 ... */
#define VC_SEARCH_STEPS		6
#define VC_LOST_CHECK_MS	2500	/* LINE_FOLLOW: still LOST this long after entry
					   (27 Oct: was 300.  > S3_LINE_LOST_MS, since S3
					   is already LOST after idling centred: give the
					   seek drift time to touch an edge first) */
#define VC_HUMP_MAX_MS		5000	/* HUMP_END never came: carry on */
#define VC_REVERSE_CM		10	/* NO_CLEAR: back off and rescan */
#define VC_MAX_RESCANS		2	/* then FAULT (obstacle not passable) */
#define VC_SCAN_TIMEOUT_MS	5000	/* SCAN_REQ -> PROFILE / NO_CLEAR */
#define VC_APPROVE_WAIT_MS	500	/* approval -> S5 BYPASSING */
#define VC_S5_READY_MS		1000	/* START -> S5 FORWARD_WATCH */
#define VC_S5_SETTLE_MS		120	/* >= 2 S5 periods after s5_reset() */
#define VC_STEP_TIMEOUT_MS	15000	/* one MOVE(d)/TURN without MOTION_DONE:
				   127 cm at 10-20 cm/s needs ~7-13 s */
#define VC_OBSTACLE_DEPTH_CM	15	/* assumed obstacle depth (not measurable) */
#define VC_REJOIN_EXTRA_CM	15	/* drive this far past the expected line */
#define VC_BYPASS_MIN_CM	10
#define VC_BYPASS_MAX_CM	60

/* START needs the IMU calibrated and an IMU fault stops the mission
   (design).  0 = bench mode ONLY: drive without a working IMU (no hump
   height, no shock E-STOP).  Never 0 on the track or in a demo. */
#define VC_REQUIRE_IMU		1

/* Obstacle Scan (Buddy 5, ultrasonic + servo).  1 = normal.  0 = bench
   mode ONLY: Obstacle Scan's task is not started (its objects still are),
   and the VC treats the path ahead as always clear -- NO obstacle
   detection, NO stop for an obstacle (NFR1, NFR10, NFR12 not met).
   For testing line following and motion while the ultrasonic is being
   fixed.  Never 0 on the track with people or obstacles, or in a demo. */
#define VC_USE_ULTRASONIC	0

/* Bench test of straight driving (S2-M03), independent of the line
   sensors: 0 = normal.  N (1..127) = on START, drive straight N cm with
   heading hold, then stop and go back to IDLE.  Line events are ignored
   during the move.  Keep 0 for normal runs. */
#define VC_BENCH_MOVE_CM	0

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
