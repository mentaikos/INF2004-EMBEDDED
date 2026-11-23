/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (Buddy 2).  Student ID: 2303689.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 *
 *	What Motion Control does:
 *	  - the ONLY code that drives the motor pins GP8..GP11;
 *	  - runs every 20 ms (NFR14), checks E-STOP every cycle (NFR9/NFR10);
 *	  - counts the wheel encoders (GP7 left, GP0 right) in an ISR;
 *	  - per-wheel speed PI + feed-forward, MOVE(follow, v), MOVE(d), TURN(angle), STOP,
 *	    SPEED_LIMIT(v); MOTION_DONE when a MOVE(d)/TURN finishes;
 *	  - stall detection (PWM on, no pulse for 300 ms) -> STALL_FAULT;
 *	  - publishes ONE packed 32-bit status word (minimum-bit requirement).
 *
 *	Interfaces owned here (the receiver/owner creates the object):
 *	  mbf_motion_cmd  VC -> Motion      one 32-bit packed command word
 *	  mbf_steer       L&B -> Motion     one signed byte (+ = steer right)
 *	  flg_safety      IMU/Obstacle/Cmd set S2_FLG_ESTOP, VC clears it
 *	  g_motion_status Motion -> anyone  packed status word (read only)
 *	Interfaces used but owned by others (bound at run time, see
 *	motion_bind_vc): mbf_vc_evt (events to the VC) and the fault flag.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* =====================================================================
 *  1. Task timing and placement
 * ===================================================================== */
/* Motion Control is the highest-priority application task:
   priority 10, period 20 ms. */
#define S2_PRI_MOTION		10
#define S2_PERIOD_MS		20	/* NFR14 */

/* Core for the Motion task.  Core 0 (TP_PRC1) also runs the USB console
   (pri 3) and the Wi-Fi task (pri 4), which would pre-empt Motion and
   add jitter, so Motion runs on core 1.  Data shared with the encoder
   ISR (core 0) is byte/halfword/word sized and has one writer, so it
   is safe across cores without a lock. */
#define S2_MOTION_PRC		TP_PRC2

/* E-STOP reaction: 1 = wait on flg_safety between cycles so the brake
   is applied as soon as the bit is set (lower than one 20 ms period);
   0 = only check once per 20 ms cycle. */
#define S2_ESTOP_WAKE		1

/* =====================================================================
 *  2. Pins.  Do not change without telling the other buddies.
 * ===================================================================== */
#define S2_PIN_MR_A		8	/* right motor M1A (PWM slice 4 A) */
#define S2_PIN_MR_B		9	/* right motor M1B (PWM slice 4 B) */
#define S2_PIN_ML_A		10	/* left motor  M2A (PWM slice 5 A) */
#define S2_PIN_ML_B		11	/* left motor  M2B (PWM slice 5 B) */
#define S2_PIN_ENC_L		7	/* left encoder  (Grove 7 split cable) */
#define S2_PIN_ENC_R		0	/* right encoder (Grove 1; never build
				   CRASH_REPORT=1) */

/* 1 = the motor is mounted the other way round ("forward" reversed) */
#define S2_INVERT_LEFT		0
#define S2_INVERT_RIGHT		1	/* right motor is mirror-mounted */

/* PWM: 125 MHz / (6249 + 1) = 20 kHz (Robo Pico limit is 20 kHz) */
#define S2_PWM_WRAP		6249

/* =====================================================================
 *  3. Measured constants (testing/subsystem2/TEST_LOG.md S2-M01)
 * ===================================================================== */
#define S2_WHEEL_DIAM_MM	43
/* Counts per wheel turn, from a measured MOVE on the floor.  Every
   speed (mm/s) and distance (mm) in the robot is real. */
#define S2_ENC_CPR		415	/* counts per wheel rev, rising edges of A */
/* Effective track for turns on the spot, 0.1 mm.  With 503 a TURN 180
   turns ~113 deg on the mat; the turns in the VC are tuned to that.
   For true angles set it from a measured turn: 503 x commanded /
   measured (180 / 113 -> ~800), which changes every turn (bypass,
   search, junction). */
#define S2_TRACK_X10_MM		503
/* Motor model: a wheel starts at S2_DEADBAND_PM, then speed rises
   linearly to S2_TOP_SPEED_MM_S at full duty (unloaded).  On the floor
   the load needs more duty: the integrator supplies it. */
#define S2_TOP_SPEED_MM_S	424	/* unloaded speed at 100 % duty */
#define S2_DEADBAND_PM		100	/* duty (per mille) where a wheel starts */
#define S2_KFF_Q8		544	/* feed-forward: (1000-100)/424 * 256 */
#define S2_KP_Q8		181	/* PI gains, Q8 (256 = 1.0); a higher KP */
#define S2_KI_Q8		30	/* swings the duty around the target */
#define S2_I_MAX_PM		300	/* integrator clamp, per mille duty */

/* Derived: micrometres travelled per encoder count (pi ~ 355/113) */
#define S2_UM_PER_COUNT	\
	((S2_WHEEL_DIAM_MM * 1000UL * 355UL / 113UL) / S2_ENC_CPR)

/* =====================================================================
 *  4. Behaviour limits (FR / NFR)
 * ===================================================================== */
#define S2_MAX_SPEED_MM_S	265	/* "no cap"; command field max 511 */
#define S2_REVERSE_MAX_MM_S	53	/* "safe reverse" cap */
#define S2_SLOW_MM_S		42	/* last 10 % of a MOVE(d)/TURN */
#define S2_DEFAULT_MM_S		79	/* MOVE(d)/TURN with speed field 0 */
#define S2_ACCEL_MM_S_PER_CYC	11	/* target ramp up, per 20 ms cycle */
#define S2_DECEL_MM_S_PER_CYC	21	/* target ramp down, per 20 ms cycle */
/* In SPEED_FOLLOW the two limits above apply to the robot's speed only
   (follow_v in set_targets()); each wheel may then change by up to
   S2_FOLLOW_STEP_MM_S_PER_CYC per cycle, so a steering correction
   reaches the wheels in one cycle and the robot does not swing from
   side to side.  MOVE(d) / TURN ramp each wheel with the limits above. */
#define S2_FOLLOW_STEP_MM_S_PER_CYC	42
#define S2_HEADING_K_X100	212	/* heading hold: mm/s x 100 per count of L/R
				   difference (higher makes the robot weave) */
#define S2_STEER_FULL		64	/* steer value giving a 100 % speed split */
#define S2_STEER_TIMEOUT_MS	100	/* no steer message -> steer 0 */
#define S2_SPEED_WIN		5	/* speed = counts in the last 5 cycles (100 ms) */
#define S2_STALL_MS		300	/* PWM on, no pulse -> stall */
#define S2_STALL_ARM_MS		500	/* start-up window before stall can trip */
#define S2_ZERO_SPEED_MS	200	/* no pulse for this long -> speed 0 */
#define S2_REVERSE_SETTLE_MS	60	/* brake before changing direction */
#define S2_ENC_MAX_INTERVAL_MS	40	/* one missed cycle -> ENC_FAULT */
#define S2_ENC_DEBOUNCE_US	500	/* ignore edges closer than this: a real
				   edge at top speed is ~1.5 ms apart */
/* Largest believable count change in one read interval.
   S2_CPR_MEASURED 1: derived from the top speed (x2 margin);
   0: a loose bound that only catches garbage. */
#define S2_CPR_MEASURED		0
#if S2_CPR_MEASURED
#define S2_ENC_MAX_DELTA \
	((S2_TOP_SPEED_MM_S * S2_ENC_MAX_INTERVAL_MS) / S2_UM_PER_COUNT * 2 + 2)
#else
#define S2_ENC_MAX_DELTA	200
#endif

/* =====================================================================
 *  5. mbf_motion_cmd -- one 32-bit packed word (VC -> Motion)
 *	[2:0]   type      S2_CMD_*
 *	[11:3]  speed     0..S2_MAX_SPEED_MM_S mm/s  (MOVE_FOLLOW, SPEED_LIMIT;
 *	                  cruise speed of MOVE_DIST/TURN, 0 = default)
 *	[20:12] angle     -180..+180 deg, signed, + = turn right (clockwise)
 *	[31:21] distance  -1024..+1023 mm, signed, - = reverse
 *  Senders: use motion_send_cmd(S2_CMD_PACK(...)).  Values are clamped.
 * ===================================================================== */
#define S2_CMD_MOVE_FOLLOW	0	/* follow the line at speed v */
#define S2_CMD_MOVE_DIST	1	/* straight for distance d */
#define S2_CMD_TURN		2	/* turn on the spot by angle */
#define S2_CMD_STOP		3	/* brake; in STALL_FAULT / STOPPED also resets faults */
#define S2_CMD_SPEED_LIMIT	4	/* cap all speeds at v (S2_MAX_SPEED_MM_S = no cap) */

#define S2_CMD_TYPE_SHIFT	0
#define S2_CMD_TYPE_MASK	0x7u
#define S2_CMD_SPEED_SHIFT	3
#define S2_CMD_SPEED_MASK	0x1FFu
#define S2_CMD_ANGLE_SHIFT	12
#define S2_CMD_ANGLE_MASK	0x1FFu
#define S2_CMD_DIST_SHIFT	21
#define S2_CMD_DIST_MASK	0x7FFu

#define S2_ANGLE_MIN		(-180)
#define S2_ANGLE_MAX		180
#define S2_DIST_MIN_MM		(-1024)
#define S2_DIST_MAX_MM		1023

static inline INT s2_clamp(INT v, INT lo, INT hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* Sign-extend the low 'bits' bits of v */
static inline INT s2_sext(UW v, INT bits)
{
	UW	m = 1u << (bits - 1);
	v &= (m << 1) - 1u;
	return (INT)(v ^ m) - (INT)m;
}

static inline UW s2_cmd_pack(UINT type, INT speed_mm_s, INT angle_deg, INT dist_mm)
{
	UW	w;
	w  = ((UW)type & S2_CMD_TYPE_MASK) << S2_CMD_TYPE_SHIFT;
	w |= ((UW)s2_clamp(speed_mm_s, 0, S2_MAX_SPEED_MM_S)
					& S2_CMD_SPEED_MASK) << S2_CMD_SPEED_SHIFT;
	w |= ((UW)s2_clamp(angle_deg, S2_ANGLE_MIN, S2_ANGLE_MAX)
					& S2_CMD_ANGLE_MASK) << S2_CMD_ANGLE_SHIFT;
	w |= ((UW)s2_clamp(dist_mm, S2_DIST_MIN_MM, S2_DIST_MAX_MM)
					& S2_CMD_DIST_MASK) << S2_CMD_DIST_SHIFT;
	return w;
}
#define S2_CMD_PACK(type, v, ang, d)	s2_cmd_pack((type), (v), (ang), (d))

static inline UINT s2_cmd_type(UW w)  { return (w >> S2_CMD_TYPE_SHIFT) & S2_CMD_TYPE_MASK; }
static inline INT  s2_cmd_speed(UW w) { return (INT)((w >> S2_CMD_SPEED_SHIFT) & S2_CMD_SPEED_MASK); }
static inline INT  s2_cmd_angle(UW w) { return s2_sext(w >> S2_CMD_ANGLE_SHIFT, 9); }
static inline INT  s2_cmd_dist(UW w)  { return s2_sext(w >> S2_CMD_DIST_SHIFT, 11); }	/* mm */

/* =====================================================================
 *  6. g_motion_status -- one 32-bit packed word, written ONLY by Motion
 *     (single aligned store, so readers always see a whole word).
 *	[6:0]   speed_l   -64..+63 cm/s signed
 *	[13:7]  speed_r   -64..+63 cm/s signed
 *	[25:14] distance  1 mm/LSB, wraps every 4096 mm (use s2_dist_diff)
 *	[28:26] state     S2_ST_*
 *	[29]    dir_l     1 = reverse
 *	[30]    dir_r     1 = reverse
 *	[31]    stopped   1 = STOPPED and both wheels still for 40 ms
 * ===================================================================== */
#define S2_ST_STOPPED		0
#define S2_ST_SPEED_FOLLOW	1
#define S2_ST_MOVE_DISTANCE	2
#define S2_ST_TURN		3
#define S2_ST_E_STOP_BRAKE	4
#define S2_ST_STALL_FAULT	5

#define S2_STS_SPDL_SHIFT	0
#define S2_STS_SPDR_SHIFT	7
#define S2_STS_DIST_SHIFT	14
#define S2_STS_STATE_SHIFT	26
#define S2_STS_DIRL_SHIFT	29
#define S2_STS_DIRR_SHIFT	30
#define S2_STS_STOP_SHIFT	31
#define S2_STS_SPD_MASK		0x7Fu
#define S2_STS_DIST_MASK	0xFFFu
#define S2_STS_STATE_MASK	0x7u

static inline UW s2_sts_pack(INT spd_l, INT spd_r, UW dist_mm, UINT state,
		      UINT dir_l, UINT dir_r, UINT stopped)
{
	UW	w;
	w  = ((UW)s2_clamp(spd_l, -64, 63) & S2_STS_SPD_MASK) << S2_STS_SPDL_SHIFT;
	w |= ((UW)s2_clamp(spd_r, -64, 63) & S2_STS_SPD_MASK) << S2_STS_SPDR_SHIFT;
	w |= (dist_mm & S2_STS_DIST_MASK) << S2_STS_DIST_SHIFT;
	w |= ((UW)state & S2_STS_STATE_MASK) << S2_STS_STATE_SHIFT;
	w |= ((UW)(dir_l & 1u)) << S2_STS_DIRL_SHIFT;
	w |= ((UW)(dir_r & 1u)) << S2_STS_DIRR_SHIFT;
	w |= ((UW)(stopped & 1u)) << S2_STS_STOP_SHIFT;
	return w;
}

/* Telemetry slot for Buddy 1 (S1_SLOT_B2, 26 bits): status bits [25:0] */
#define S2_TLM_MASK		0x03FFFFFFu
#define S2_TLM_EVERY		5	/* cycles between slot updates (100 ms) */

static inline INT  s2_sts_speed_l(UW w) { return s2_sext(w >> S2_STS_SPDL_SHIFT, 7); }
static inline INT  s2_sts_speed_r(UW w) { return s2_sext(w >> S2_STS_SPDR_SHIFT, 7); }
static inline UW   s2_sts_dist(UW w)    { return (w >> S2_STS_DIST_SHIFT) & S2_STS_DIST_MASK; }
static inline UINT s2_sts_state(UW w)   { return (w >> S2_STS_STATE_SHIFT) & S2_STS_STATE_MASK; }
static inline UINT s2_sts_dir_l(UW w)   { return (w >> S2_STS_DIRL_SHIFT) & 1u; }
static inline UINT s2_sts_dir_r(UW w)   { return (w >> S2_STS_DIRR_SHIFT) & 1u; }
static inline UINT s2_sts_stopped(UW w) { return (w >> S2_STS_STOP_SHIFT) & 1u; }

/* Wrap-safe signed difference of two 12-bit distance readings, in mm.
   4093 -> 3 gives +6, 3 -> 4093 gives -6.  Sample at least every ~2 s. */
static inline INT s2_dist_diff(UW old_mm, UW new_mm)
{
	return (INT)((new_mm - old_mm + 2048u) & 0xFFFu) - 2048;
}

/* =====================================================================
 *  7. flg_safety (created here, shared)
 *     Set S2_FLG_ESTOP with tk_set_flg (task or ISR): IMU when the
 *     robot is LIFTED or OVERTURNED, Obstacle on TOO_CLOSE, Command
 *     Handler on MQTT E-STOP.
 *     ONLY the VC clears it (tk_clr_flg(flg_safety, ~S2_FLG_ESTOP))
 *     on a new START.  Bits 1..14 are free for the other buddies
 *     (e.g. B4's LIFTED / IMU_FAULT); bit 15 is private to Motion.
 *     Never wait on flg_safety with TWF_CLR or TWF_BITCLR (that would
 *     clear other owners' bits); clear only your own bit with tk_clr_flg.
 * ===================================================================== */
#define S2_FLG_ESTOP		0x0001u
#define S2_FLG_TICK		0x8000u	/* private: Motion's 20 ms tick */

/* =====================================================================
 *  8. Faults and events (owned by the VC / Buddy 1).  Defaults below;
 *     a shared header may define these first to override them.
 * ===================================================================== */
#ifndef FAULT_STALL
#define FAULT_STALL		0x0001u
#endif
#ifndef FAULT_ENC
#define FAULT_ENC		0x0002u
#endif

/* mbf_vc_evt word: [3:0] event type, [31:4] payload */
#ifndef EVT_SENSOR_FAULT
#define EVT_SENSOR_FAULT	9
#endif
#ifndef EVT_MOTION_DONE
#define EVT_MOTION_DONE		13
#endif
#define S2_EVT_PACK(type, payload)	(((UW)(type) & 0xFu) | ((UW)(payload) << 4))
/* MOTION_DONE payload [2:0] = finished command (S2_CMD_MOVE_DIST/TURN)
   SENSOR_FAULT payload [1:0] = FAULT_STALL / FAULT_ENC bit(s) */

/* =====================================================================
 *  9. Public objects and functions
 * ===================================================================== */
IMPORT ID		mbf_motion_cmd;	/* VC -> Motion, 4-byte messages */
IMPORT ID		mbf_steer;	/* L&B -> Motion, 1-byte messages */
IMPORT ID		flg_safety;	/* E-STOP bit, see section 7 */
IMPORT volatile UW	g_motion_status;

/* VC owner calls this once (in its _init) to receive MOTION_DONE /
   SENSOR_FAULT and to have STALL/ENC_FAULT set in the team fault flag.
   Pass 0 for an object that does not exist yet. */
IMPORT void	motion_bind_vc(ID mbf_vc_evt, ID flg_fault);

/* Helpers for senders (never wait: TMO_POL).  Return E_OK or error. */
IMPORT ER	motion_send_cmd(UW cmd_word);
IMPORT ER	motion_send_steer(INT steer);	/* clamped to -127..127 */

IMPORT UW	motion_get_status(void);	/* = g_motion_status */
IMPORT UINT	motion_get_faults(void);	/* latched FAULT_* bits */
IMPORT UH	motion_get_evt_drops(void);	/* events mbf_vc_evt refused */

/* Private to subsystem 2 */
IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT void	motion_cyclic_isr(void *exinf);

/* =====================================================================
 *  10. Template example compatibility (s2_example_compat.c).
 *      The example consumer (subsystem 3) and monitor (subsystem 4)
 *      use these; nothing in Motion Control does.
 * ===================================================================== */
#define S2_EXAMPLE_PRODUCER	1	/* 0 = stop the example producer */
#define S2_PRI_PRODUCER		15	/* lowest of the old S2 band */
#define S2_PRODUCER_PERIOD_MS	300

typedef struct {
	T_MSG	hdr;		/* used by the kernel, do not touch */
	UW	seq;		/* sample sequence number */
	UW	time_ms;	/* system time when the sample was taken */
	INT	value;		/* the "measurement", 0..100 */
	INT	prc;		/* processor (core) that produced it: 1 or 2 */
} SAMPLE_MSG;

#define S2_POOL_COUNT	4
IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;
IMPORT void	producer_task(INT stacd, void *exinf);
IMPORT INT	s2_make_value(UW seq);
IMPORT ER	s2_example_init(void);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/
