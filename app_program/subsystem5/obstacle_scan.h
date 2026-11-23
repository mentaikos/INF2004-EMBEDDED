/* Hardware-independent logic and compact data contract.
 * Bit-fields are private to the scan task, never ISR-written. */
#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H
#include "scan_types.h"

#define S5_ANGLES              25
#define S5_CENTRE              12       /* index 12 = 90 degrees (straight ahead) */
/* Sweep: 90 +- S5_SWEEP_HALF_DEG, spread over the 25 bins.  Coarse scan
 * = bins 0, 6, 12, 18, 24.  60 = 30..150 deg (original, 5-deg bins,
 * coarse every 30 deg).  45 = 45..135 (3.75-deg bins, coarse every
 * 22.5 deg).  30 = 60..120 (2.5-deg bins, coarse every 15 deg).
 * Narrower = less servo swing, but less of each side is seen, so a
 * bypass is approved less often (more REVERSE / NO_CLEAR). */
#define S5_SWEEP_HALF_DEG      60       /* 28 Oct: 45 never cleared a 30 cm box */
#define S5_SCAN_BYTES          22       /* 25 * 7 = 175 bits */
#define S5_ALL_ANGLES          0x01ffffffu
#define S5_RANGE_NEAR          0        /* returned echo below 5 cm */
#define S5_RANGE_FAR           1        /* returned echo beyond 100 cm */
#define S5_RANGE_NO_ECHO       2        /* missing/invalid pulse, NOT clear */
#define S5_RANGE_UNSCANNED     3
#define S5_RANGE_MIN_CM        5
#define S5_RANGE_MAX_CM        100
#define S5_DETECT_CM           21       /* 28 Oct: was 20; the robot coasted to
					   ~16 cm, inside its own turning circle
					   (16.8 cm), so every first scan said
					   REVERSE.  19 Nov: 21 again -- at 25 it
					   stopped ~20-23 cm away, in the floor
					   echoes (S5_FLOOR_MIN_CM), and the floor
					   was taken as obstacle all round: REVERSE
					   with both sides open.  The turning check
					   now counts S5_SENSOR_AHEAD_MM, so the
					   ~16 cm stop is no longer "too close". */
/* 19 Nov: the sensor is tilted down and hears the floor 23-32 cm away
 * at most bearings (real scans: open floor 29-30 cm all round, rings at
 * 23-29 cm).  With the obstacle nearer than that, readings in this
 * band are floor: not part of the obstacle, open space beside it. */
#define S5_FLOOR_MIN_CM        23
#define S5_FLOOR_MAX_CM        32
/* 19 Nov: the sensor is this far ahead of the axle the robot turns
 * about (placeholder robot: ~10 cm; 5 cm assumed), so an obstacle at
 * range r is at least r + this from the turning centre. */
#define S5_SENSOR_AHEAD_MM     50
/* 19 Nov: during a scan, the robot counts as stopped unless Motion is
 * driving or the odometer has moved more than this (task_scan.c): one
 * encoder tick from a settling wheel used to abort every scan. */
#define S5_SCAN_DRIFT_MM       15
/* 19 Nov: s5_face_width: a bin is looking at the face if its forward
 * component (r sin a) is within this of the face's */
#define S5_FACE_TOL_MM         12
/* 19 Nov: after the sweep, bins within this many of the closest that
 * missed the obstacle are pinged once more, for s5_face_width only
 * (wscan; the bypass keeps the first readings).  8 = +-40 deg, at most
 * ~1 s more per scan (VC_SCAN_TIMEOUT_MS is 5 s). */
#define S5_WIDTH_REPING_BINS   8
/* After a stop, an obstacle IN THE ROBOT'S PATH (within half the required
 * width of the centre line) this close is planned around, not driven
 * into: without this, the rescan after a 10 cm reverse saw it beyond
 * S5_DETECT_CM, answered CONTINUE, and the robot drove back into it. */
#define S5_PLAN_CM             50       /* > detect + 2 reverses of 10 cm */
/* While going around (BYPASSING), stop for something this close only.
 * The bypass turns sweep the sensor across the obstacle being passed
 * (~19 cm away): with S5_DETECT_CM that re-detected it mid-turn, the VC
 * re-planned from a half-turned heading and the robot clipped the box.
 * Anything within S5_CRITICAL_MM still E-STOPs. */
#define S5_BYPASS_DETECT_CM    10
#define S5_CRITICAL_MM         50       /* TODO validate brake distance */
#define S5_ECHO_TIMEOUT_US     30000u
#define S5_ECHO_MIN_US         100u     /* < 1.7 cm: below the HC-SR04P's 2 cm
					   minimum, so a glitch, not an echo */
#define S5_INVALID_LIMIT       3
#define S5_RETRY_LIMIT         3
#define S5_RETRY_MS            1000
#define S5_PLAN_TTL_MS         1000
#define S5_RECOVERY_MS         30000    /* rejoin drive (up to 127 cm at 10 cm/s)
					   + up to 3 left/right looks with reverses
					   (30 Oct: was 20000); < 32 s (16-bit ms) */
#define S5_LINE_FRESH_MS       120
#define S5_LINE_CONFIRM_MS     120
#define S5_PROFILE_BAND_CM     10
/* 16 Nov (integration fix, for the VC's width-based bypass): the WIDTH
 * reported in the profile is the obstacle itself only: the unbroken run
 * of bearings around the closest reading that stay within this many cm
 * of it.  With the 10 cm band above, a cup at 23 cm with other echoes
 * at 25-31 cm across the sweep was reported 47 cm wide (16 Nov log)
 * and the bypass drove far too wide and too long; its own run (105-135
 * deg, 23-25 cm) gives ~12 cm.  The 10 cm band still decides the
 * foreground for side clearance and the plan (unchanged). */
#define S5_WIDTH_BAND_CM       3
/* 16 Nov, second run: the ultrasonic beam is ~15-20 deg wide, so a
 * bottle (~8 cm) at 24 cm echoed over 55 deg of the sweep and its run
 * still came out 17-23 cm wide.  The beam spread is taken off both
 * ends of the run (this many 5-degree bins each side) before the width
 * and the centre are measured.  MEASURE: a known 8 cm bottle at 20-30
 * cm should log width ~8-12 cm. */
#define S5_WIDTH_BEAM_BINS     2
#define S5_BEAM_ALLOW_DEG      15
/* Side gap check (side_clearance): only things within the strip the robot
 * drives through while passing count -- up to the obstacle face band +
 * this far ahead (obstacle depth + robot length).  Nothing there: the side
 * is taken as this wide (more than any bypass needs). */
#define S5_PASS_DEPTH_MM       450
#define S5_SIDE_FREE_MM        600
/* Missing echoes (30 Oct: robot not square to a round bottle).  A round
 * or angled surface sends the sound off sideways, so some angles of a
 * sweep get no echo although nothing is there.  Before, ONE missing echo
 *   - on a side made that side 0 cm free,
 *   - straight ahead (90 deg) made every plan REVERSE,
 *   - anywhere, with the obstacle > S5_DETECT_CM away and off the
 *     robot's path, gave no plan at all (NO_CLEAR),
 * and backing up straight changed nothing, so the robot faulted.
 * Now up to this many missing echoes per side are passed over, the
 * straight-ahead reading only has to be known when the obstacle is in
 * front, and CONTINUE allows this many in the whole sweep (the forward
 * watch still stops for anything that comes within S5_DETECT_CM). */
#define S5_SIDE_NOECHO_OK      1
#define S5_CONT_NOECHO_OK      2
/* Forward watch on curves (task_scan.c): while following, point the
 * sensor into the bend, so a box placed on a curve is seen ahead of the
 * robot instead of beside the straight-ahead beam.  The bend is taken
 * from Line & Barcode's steering, averaged over ~0.5 s: look angle =
 * (|average| - S5_WATCH_DEADBAND) x S5_WATCH_GAIN degrees, at most
 * S5_WATCH_MAX_DEG; inside the deadband the sensor stays centred, so
 * normal weaving on a straight does not move it.
 * Geometry: on a circle of radius R, a box on the line d cm ahead is
 * asin(d / 2R) off the robot's heading -- 30 deg for R = 25 cm and
 * d = 25 cm, 39 deg for R = 20 cm.  The beam is about +-15 deg wide.
 * Tune on the track: tight circle -> raise GAIN; the sensor jumping
 * about on straights -> raise DEADBAND. */
#define S5_WATCH_MAX_DEG       40
#define S5_WATCH_DEADBAND      10       /* 30 Oct: was 8 (straight-line weave) */
#define S5_WATCH_GAIN          3
/* Only look into a bend once the steering has stayed the same way this
 * long (30 Oct): a straight-line weave changes side sooner, a bend does
 * not.  Raise it if the sensor still moves on straights; lower it if it
 * turns into bends too late. */
#define S5_WATCH_HOLD_MS       600

/* PLACEHOLDERS: measure assembled footprint and motion/sensing errors. */
#define S5_ROBOT_WIDTH_MM      160
#define S5_ROBOT_LENGTH_MM     200
#define S5_CLEARANCE_MARGIN_MM 30
#define S5_RANGE_ERROR_MM      10
#define S5_SERVO_MIN_US        1000     /* calibrate 0/180-degree pulses */
#define S5_SERVO_MAX_US        2000
#define S5_SERVO_REVERSED      0        /* 0: 30 degrees points right */
#define S5_SERVO_BASE_MS       20
#define S5_SERVO_MS_PER_DEG    2        /* conservative; measure */

#define S5_ST_FORWARD_WATCH    0
#define S5_ST_OBSTACLE_DETECTED 1
#define S5_ST_COARSE_SCAN      2
#define S5_ST_FINE_SCAN        3
#define S5_ST_PROFILE_PLAN     4
#define S5_ST_BYPASSING        5
#define S5_ST_LINE_RECOVERY    6
#define S5_ST_SENSOR_FAULT     7

#define S5_ACT_STOP            0
#define S5_ACT_CONTINUE        1
#define S5_ACT_LEFT            2
#define S5_ACT_RIGHT           3
#define S5_ACT_REVERSE_REATTEMPT 4       /* VC checks reverse policy */

#define S5_SCAN_REQ            0x01u
#define S5_BYPASS_APPROVED     0x02u
#define S5_REJOIN_LINE         0x04u
#define S5_LINE_FOUND          0x08u
#define S5_LINE_LOST           0x10u
#define S5_RESET               0x20u
#define S5_CANCEL              0x40u
#define S5_SCAN_FLAGS          0x7fu

/* 8-byte profile, unused bits zero.
 * w0: closest 7, visible width 8, x 8 signed, y 7 (all cm).
 * 16 Nov: x is the CENTRE of the obstacle (of its own run of bearings,
 * beam spread taken off), so its edges are x - width/2 .. x + width/2;
 * y is still the closest point (the near face). 
 * w1: left clearance 7, right clearance 7, first edge index 5,
 *     last edge index 5, action 3, valid 1.
 * x positive right, y forward; distance uses the codes above.
 * Clearance is an observed free-wedge chord, NOT hidden obstacle depth. */
typedef struct { UW w0, w1; } S5_PROFILE;
_Static_assert(sizeof(S5_PROFILE) == 8, "S5 profile must occupy eight bytes");

static inline UINT s5_profile_closest(S5_PROFILE p) { return p.w0 & 127u; }
static inline UINT s5_profile_width(S5_PROFILE p) { return (p.w0 >> 7) & 255u; }
static inline INT s5_profile_x(S5_PROFILE p)
{
	return (INT)(((p.w0 >> 15) & 255u) ^ 128u) - 128;
}
static inline UINT s5_profile_y(S5_PROFILE p) { return (p.w0 >> 23) & 127u; }
static inline UINT s5_profile_left(S5_PROFILE p) { return p.w1 & 127u; }
static inline UINT s5_profile_right(S5_PROFILE p) { return (p.w1 >> 7) & 127u; }
static inline UINT s5_profile_action(S5_PROFILE p) { return (p.w1 >> 24) & 7u; }
static inline BOOL s5_profile_valid(S5_PROFILE p) { return (p.w1 >> 27) & 1u; }

/* Status: state [2:0], action [5:3], angle index [10:6], range [17:11],
 * plan-valid [18], fresh line [19], stable samples [21:20],
 * generation [29:22]. Reserved [31:30] are zero. */
static inline UINT s5_sts_state(UW w) { return w & 7u; }
static inline UINT s5_sts_action(UW w) { return (w >> 3) & 7u; }
static inline UINT s5_sts_generation(UW w) { return (w >> 22) & 255u; }
static inline BOOL s5_sts_plan_valid(UW w) { return (w >> 18) & 1u; }

/* Task-private state. Pending needs 25 bits. Modulo-65536 ms deadlines
 * are all shorter than half that period. */
typedef struct {
	UB state : 3;
	UB action : 3;
	UB plan_valid : 1;
	UB line_fresh : 1;
	UB index : 5;
	UB near_count : 2;
	UB crit_seen : 1;       /* one critical reading, waiting for a second */
	UB invalid_count : 2;
	UB retries : 2;
	UB line_count : 2;
	UB width_pass : 1;      /* 19 Nov: re-pinging for the width (wscan) */
	UB reserved2 : 1;
	UB range : 7;
	UB reserved3 : 1;
	UB generation, last_line_seq;
	UH plan_ms, recovery_ms, line_start_ms, retry_ms;
	UW pending;
	UB scan[S5_SCAN_BYTES];
	UB wscan[S5_SCAN_BYTES];  /* 19 Nov: scan + re-pings, for s5_face_width only */
	S5_PROFILE profile;
} S5_SCAN;

/* Ephemeral inputs/results, no persistent unpacked copy. */
typedef struct {
	UH now_ms, line_ms;
	UINT flags;
	BOOL stopped, estop, in_hump;
	BOOL approval_valid;
	UB approval_generation, line_seq;
	BOOL line_present, line_found;
} S5_INPUT;

typedef struct {
	INT event;                         /* app.h EVT_* contract, -1 none */
	UW payload;
	BOOL estop;
	UINT fault;                        /* 1 = echo, 2 = recovery */
} S5_OUTPUT;

IMPORT UB s5_range_from_echo(UW width_us);
IMPORT UINT s5_angle_deg(UINT index);
IMPORT UINT s5_servo_pulse_us(UINT index);
IMPORT UINT s5_servo_settle_ms(UINT old_index, UINT new_index);
IMPORT void s5_scan_set(UB *scan, UINT index, UINT range);
IMPORT UB s5_scan_get(const UB *scan, UINT index);
IMPORT UW s5_fine_mask(const UB *scan);
IMPORT S5_PROFILE s5_make_profile(const UB *scan);
/* 19 Nov: the obstacle's width for the dashboard and the log, cm (not the
 * profile's width, which the bypass uses) */
IMPORT UINT s5_face_width(const UB *scan);
IMPORT UW s5_profile_event(S5_PROFILE profile, UB generation);
IMPORT void s5_scan_init(S5_SCAN *s);
IMPORT void s5_scan_tick(S5_SCAN *s, const S5_INPUT *in, S5_OUTPUT *out);
IMPORT void s5_scan_sample(S5_SCAN *s, const S5_INPUT *in, UB range,
			   S5_OUTPUT *out);
IMPORT BOOL s5_scan_wants_ping(const S5_SCAN *s, UH now_ms);
IMPORT UW s5_scan_status(const S5_SCAN *s);

#endif /* OBSTACLE_SCAN_H */

/*** end of file ***/
