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
#define S5_SWEEP_HALF_DEG      60
#define S5_SCAN_BYTES          22       /* 25 * 7 = 175 bits */
#define S5_ALL_ANGLES          0x01ffffffu
#define S5_RANGE_NEAR          0        /* returned echo below 5 cm */
#define S5_RANGE_FAR           1        /* returned echo beyond 100 cm */
#define S5_RANGE_NO_ECHO       2        /* missing/invalid pulse, NOT clear */
#define S5_RANGE_UNSCANNED     3
#define S5_RANGE_MIN_CM        5
#define S5_RANGE_MAX_CM        100
#define S5_DETECT_CM           20
#define S5_CRITICAL_MM         50       /* TODO validate brake distance */
#define S5_ECHO_TIMEOUT_US     30000u
#define S5_ECHO_MIN_US         100u     /* < 1.7 cm: below the HC-SR04P's 2 cm
					   minimum, so a glitch, not an echo */
#define S5_INVALID_LIMIT       3
#define S5_RETRY_LIMIT         3
#define S5_RETRY_MS            1000
#define S5_PLAN_TTL_MS         1000
#define S5_RECOVERY_MS         10000
#define S5_LINE_FRESH_MS       120
#define S5_LINE_CONFIRM_MS     120
#define S5_PROFILE_BAND_CM     10
#define S5_BEAM_ALLOW_DEG      15

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
 * w1: left clearance 7, right clearance 7, first edge index 5,
 *     last edge index 5, action 3, valid 1.
 * x positive right, y forward; distance uses the codes above.
 * Clearance is an observed free-wedge chord, NOT hidden obstacle depth. */
typedef struct { UW w0, w1; } S5_PROFILE;
_Static_assert(sizeof(S5_PROFILE) == 8, "S5 profile must occupy eight bytes");

Inline UINT s5_profile_closest(S5_PROFILE p) { return p.w0 & 127u; }
Inline UINT s5_profile_width(S5_PROFILE p) { return (p.w0 >> 7) & 255u; }
Inline INT s5_profile_x(S5_PROFILE p)
{
	return (INT)(((p.w0 >> 15) & 255u) ^ 128u) - 128;
}
Inline UINT s5_profile_y(S5_PROFILE p) { return (p.w0 >> 23) & 127u; }
Inline UINT s5_profile_left(S5_PROFILE p) { return p.w1 & 127u; }
Inline UINT s5_profile_right(S5_PROFILE p) { return (p.w1 >> 7) & 127u; }
Inline UINT s5_profile_action(S5_PROFILE p) { return (p.w1 >> 24) & 7u; }
Inline BOOL s5_profile_valid(S5_PROFILE p) { return (p.w1 >> 27) & 1u; }

/* Status: state [2:0], action [5:3], angle index [10:6], range [17:11],
 * plan-valid [18], fresh line [19], stable samples [21:20],
 * generation [29:22]. Reserved [31:30] are zero. */
Inline UINT s5_sts_state(UW w) { return w & 7u; }
Inline UINT s5_sts_action(UW w) { return (w >> 3) & 7u; }
Inline UINT s5_sts_generation(UW w) { return (w >> 22) & 255u; }
Inline BOOL s5_sts_plan_valid(UW w) { return (w >> 18) & 1u; }

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
	UB reserved2 : 2;
	UB range : 7;
	UB reserved3 : 1;
	UB generation, last_line_seq;
	UH plan_ms, recovery_ms, line_start_ms, retry_ms;
	UW pending;
	UB scan[S5_SCAN_BYTES];
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
IMPORT UW s5_profile_event(S5_PROFILE profile, UB generation);
IMPORT void s5_scan_init(S5_SCAN *s);
IMPORT void s5_scan_tick(S5_SCAN *s, const S5_INPUT *in, S5_OUTPUT *out);
IMPORT void s5_scan_sample(S5_SCAN *s, const S5_INPUT *in, UB range,
			   S5_OUTPUT *out);
IMPORT BOOL s5_scan_wants_ping(const S5_SCAN *s, UH now_ms);
IMPORT UW s5_scan_status(const S5_SCAN *s);

#endif /* OBSTACLE_SCAN_H */