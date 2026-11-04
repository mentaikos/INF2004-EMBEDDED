/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU & terrain ("Physics Watchdog", Buddy 4).
 *	Student ID: 2500701.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 *
 *	What the IMU Monitor does (report statechart 6.4, FR13-FR15):
 *	  - reads the LSM303DLHC every 10 ms (NFR11); it has NO gyroscope:
 *	    pitch from the accelerometer, heading from the magnetometer,
 *	    turn rate from the change in heading;
 *	  - calibrates for 2 s at start (robot still and flat);
 *	  - detects humps (> +3 deg for 50 ms) and measures their peak height
 *	    with Buddy 2's distance: HUMP_START / HUMP_END(height) -> VC;
 *	  - detects shocks (|a| > 2.5 g) -> E-STOP in flg_safety at once;
 *	  - 3 failed I2C reads -> IMU fault, SENSOR_FAULT -> VC, retry every
 *	    1 s, calibrate again once the sensor answers;
 *	  - publishes 42 bits of packed status (s4_pack.h).
 *
 *	Hardware (design review section 2), owned by this subsystem only:
 *	  IMU_SDA  GP26  I2C1 SDA   Grove 6
 *	  IMU_SCL  GP27  I2C1 SCL   Grove 6 (leave Grove 5 unused: it shares GP26)
 *	  I2C1 is driven directly (drv_lsm303.c); the kernel I2C driver stays
 *	  off (DEVCNF_USE_IIC 0).
 *
 *	Interfaces used (owned by others):
 *	  g_motion_status (S2)  distance, read every cycle (s2_sts_dist)
 *	  flg_safety (S2)       IMU sets S2_FLG_ESTOP on a shock
 *	  mbf_vc_evt (VC)       HUMP_START, HUMP_END(height), SENSOR_FAULT
 *	  flg_fault  (VC/B1)    FAULT_IMU, FAULT_SHOCK
 *	The last two are bound at run time with s4_bind_vc(), like
 *	motion_bind_vc() and s3_bind_vc(); until then subsystem 4 skips them.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "s4_pack.h"		/* packed status layout + unpack helpers */

/* =====================================================================
 *  1. Task timing and placement
 * ===================================================================== */
/* Design 7.1: IMU 10 ms, proposed priority 11 (rate-monotonic: it is
   the fastest task after Motion).  Same priority as Line & Barcode (S3,
   11); both are short, so sharing a level is fine.  [DECIDE] the
   template's band for S4 was 18-19. */
#define S4_PRI_IMU		11
#define S4_PERIOD_MS		10	/* NFR11 */

/* Core 1 (TP_PRC2), like Motion: core 0 also runs the USB console (pri 3)
   and the Wi-Fi task (pri 4), which could stall a polled I2C transfer
   past its time-out.  The IMU shares no ISR data, so any core is safe.
   [DECIDE] with the team (SMP vs pinning). */
#define S4_IMU_PRC		TP_PRC2

/* Console log task: prints the IMU status (no %f: fixed point).
   0 = no log task. */
#define S4_PRI_LOG		19
#define S4_LOG_MS		0
/* 1 = print events only (state changes with pitch, HUMP_END height and
   peak pitch, SHOCK, TURN measured by compass), even with S4_LOG_MS 0:
   a few lines per hump / turn, no 1 s status line. */
#define S4_LOG_EVENTS		1

/* Tilt trace on the console (bench / tuning aid): while the robot is
   tilted (|pitch| >= S4_TILT_MIN_DD or on a hump) print EVERY 10 ms
   IMU sample, e.g.  s4: tilt t=+120ms pitch=+5.3 deg CLIMBING
   10 ms is the fastest: the IMU reads the sensor every 10 ms.  The IMU
   task only queues the samples; the low-priority log task prints them,
   so printing never delays the 10 ms loop.  0 = off (before the demo). */
#define S4_LOG_TILT		0
#define S4_TILT_MIN_DD		10	/* 1.0 deg, in 0.1 deg */

/* =====================================================================
 *  2. Pins
 * ===================================================================== */
#define S4_PIN_IMU_SDA		26
#define S4_PIN_IMU_SCL		27

/* Optional debug outputs for a scope / logic analyser; -1 = unused.
   Pick a FREE pin (GP1, GP3, GP6, GP13-15, GP17, GP19 -- GP19 is the
   example LED) and claim it with the team first.
     SHOCK pin   high from the shock until the latch is acknowledged:
                 scope it against Buddy 2's GP8..11 for the shock ->
                 brake latency (S4-T02, Version C row 11)
     TIMING pin  high while one IMU cycle runs: period, jitter and WCET
                 (S4-T01) */
/* SHOCK -> E-STOP.  1 = normal (FR27, NFR10).  0 = bench mode ONLY while
   false shocks are being fixed: SHOCK is still detected, logged and
   latched, but does not set E-STOP.  Never 0 on the track or in a demo. */
#define S4_SHOCK_ESTOP		0

#define S4_DEBUG_SHOCK_PIN	(-1)
#define S4_DEBUG_TIMING_PIN	(-1)

/* =====================================================================
 *  3. Fault bits in the team fault flag, in the design's fault list
 *     order like FAULT_STALL (0x1), FAULT_ENC (0x2) and
 *     FAULT_BARCODE_ERR (0x20).  [DECIDE] Buddy 1 assigns the
 *     positions; a shared header may define these first to override.
 * ===================================================================== */
#ifndef FAULT_IMU
#define FAULT_IMU		0x0004u	/* sensor not answering */
#endif
#ifndef FAULT_SHOCK
#define FAULT_SHOCK		0x0080u	/* shock latch, cleared by Telemetry */
#endif

/* =====================================================================
 *  4. Events to the Vehicle Controller (app.h, one EVT_PACK word each)
 *	EVT_HUMP_START	payload 0
 *	EVT_HUMP_END	payload [4:0] = peak height, mm (0..31);
 *			0 = hump aborted (shock, fault, false trigger):
 *			leave HUMP_TRAVERSE anyway
 *	EVT_SENSOR_FAULT payload = FAULT_IMU
 *  The VC must hold the speed constant from HUMP_START to HUMP_END:
 *  braking or speeding up shows as tilt and spoils the height.
 * ===================================================================== */

/* =====================================================================
 *  5. Public functions
 * ===================================================================== */
/* VC owner calls this once (in its _init).  Pass 0 for an object that
   does not exist yet. */
IMPORT void	s4_bind_vc(ID mbf_vc_evt, ID flg_fault);

/* Both status words from the same cycle (takes mtx_s4_status briefly) */
IMPORT void	s4_get_status(UW *w0, UW *w1);

/* Word 0 only, without a lock (one aligned word): state, pitch,
   heading, turn rate, calibrated.  Unpack with s4_sts_xxx(). */
IMPORT UW	s4_get_status_w0(void);

IMPORT BOOL	s4_is_calibrated(void);	/* VC's START guard */
IMPORT BOOL	s4_in_hump(void);	/* CLIMBING or DESCENDING: S3 may
					   ignore a short line loss, S5 may
					   ignore echoes off the hump */

/* SHOCK latch (Version C row 10): a SHOCK lasts 200 ms but telemetry
   runs every 500 ms, so it is latched until Telemetry has published it.
   Race-free with one writer per byte: the IMU counts shocks, Telemetry
   acknowledges the count it published.
	seq = s4_shock_seq();
	if (s4_shock_pending()) { publish the alert; s4_shock_ack(seq); } */
IMPORT UB	s4_shock_seq(void);
IMPORT BOOL	s4_shock_pending(void);
IMPORT void	s4_shock_ack(UB seq);

/* Telemetry slot (Buddy 1: S1_SLOT_B4 = 44 bits, frame bits 38..81),
   set by the IMU task every 10 ms:
	[43:13]  status word 0 (31 bits: state, pitch, heading, rate, cal)
	[12:2]   status word 1 (11 bits: hump height, count, I2C fails)
	[1]      shock, held 1 s (two 500 ms telemetry frames)
	[0]      0, spare
   The dashboard decodes it with the same shifts as s4_pack.h. */
#define S4_TLM_SHOCK_HOLD_MS	1000
Inline UD s4_tlm_slot(UW w0, UW w1, BOOL shock)
{
	return ((UD)(w0 & 0x7FFFFFFFu) << 13) | ((UD)(w1 & 0x7FFu) << 2) |
	       ((UD)(shock ? 1u : 0u) << 1);
}

/* Diagnostics (console and tests; not sent) */
typedef struct {
	UW	wcet_us;	/* longest IMU cycle since start */
	UW	last_us;	/* last IMU cycle */
	UH	cycles;		/* IMU cycles run (wraps) */
	UH	overruns;	/* cycles where a wake-up was missed */
	UH	i2c_errors;	/* failed transfers (wraps) */
	UH	evt_drops;	/* events mbf_vc_evt refused, or not bound */
	UB	sensor_found;	/* LSM303DLHC answered at start-up */
	H	last_peak_dd;	/* peak pitch of the last hump, 0.1 deg */
} S4_DIAG;

IMPORT void	s4_get_diag(S4_DIAG *d);

/* =====================================================================
 *  6. Private to subsystem 4
 * ===================================================================== */
IMPORT ID	tid_imu;
IMPORT ID	cyc_imu;
IMPORT ID	mtx_s4_status;		/* guards the two status words */
IMPORT void	imu_task(INT stacd, void *exinf);
IMPORT void	imu_cyclic_handler(void *exinf);
IMPORT void	imu_log_task(INT stacd, void *exinf);

#if APP_TEST
/* Test hook: the IMU task runs its shock reaction once, as if the
   sensor had seen 2.5 g (INT-05).  No real knock needed. */
IMPORT void	s4_test_inject_shock(void);
#endif

/* =====================================================================
 *  7. TEMPORARY: the template's example monitor (s4_example_compat.c).
 *     The example consumer in subsystem 3 still sends to mbf_summary /
 *     flg_events (IF-03).  REMOVE together with subsystem 3's
 *     consumer_task; nothing in the IMU uses it.
 * ===================================================================== */
#define S4_EXAMPLE_MONITOR	0	/* 0 = objects only, no monitor task */
#define S4_PRI_MONITOR		18
#define S4_ALARM_MS		2000	/* report anyway if nothing for 2 s */
#define S4_LED_TOGGLE_REPORTS	6	/* pause/resume the LED every N reports */

/* IF-03: send one text line (at most S4_SUMMARY_MAX bytes, no '\0'
   needed) to mbf_summary, then set FLG_BATCH in flg_events. */
#define S4_SUMMARY_MAX		48
#define FLG_BATCH		0x0002	/* set by subsystem 3 */
#define FLG_ALARM		0x0001	/* private: set by our alarm handler */

IMPORT ID	mbf_summary;
IMPORT ID	flg_events;
IMPORT ID	alm_monitor;
IMPORT UW	s4_lines_received;	/* for the tests */
IMPORT ER	s4_example_init(void);
IMPORT void	monitor_task(INT stacd, void *exinf);
IMPORT void	monitor_alarm_handler(void *exinf);

#endif /* SUBSYSTEM4_H */
