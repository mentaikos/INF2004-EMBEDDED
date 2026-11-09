/*
 *	subsystem3.h
 *	Subsystem 3 -- Line following & barcode decoding.  Student ID: 2502193.
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".
 *
 *	Hardware (design review section 2), owned by this subsystem only:
 *	  IR_LEFT    GP2   digital in, 1 = black   line tracking
 *	  IR_CENTER  GP16  digital in, 1 = black   line tracking
 *	  IR_RIGHT   GP28  ADC2                    barcode
 *	  RP2040 TIMER alarm 3 (IRQ 3)             0.25 ms barcode sampling
 *	Subsystem 3 is the only user of the ADC: its ISR reads the ADC
 *	registers directly, so DEVCONF_ADC_PIN_INIT_2 stays FALSE.
 *
 *	Tasks and handlers:
 *	  Line & Barcode task   pri 11, woken every 20 ms by a cyclic handler
 *	  Barcode ADC ISR       TIMER alarm 3, every 0.25 ms; stores only
 *	                        edge times in bc_ring
 *	The task may run on either core: everything it shares with the ISR
 *	has one writer per word (see "IR sensor driver" below).
 *
 *	Uses subsystem 2 (Motion Control): motion_send_steer() every cycle.
 *	Bar widths come from edge times only, so the barcode decoder needs
 *	nothing else from Motion Control.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_LINE_BARCODE	11	/* design 7.1 (template band is 16-17) */
#define S3_PERIOD_MS		20	/* NFR8: line sensors every 20 ms */
/* NFR16 asks for at least every 1 ms.  4x faster so a narrow bar is
   still many samples wide at full speed: at 50 cm/s a 2 mm bar is 4 ms,
   and +-1 ms of edge jitter made narrow and wide overlap.  ~3 us per
   sample, so ~1 % of core 0. */
#define S3_SAMPLE_US		250

/* Barcode times (edge clock, element widths, time-outs) are counted in
   ticks of the ISR's sample counter, one per S3_SAMPLE_US */
#define S3_TICKS_PER_MS		(1000 / S3_SAMPLE_US)
#define S3_MS(ms)		((ms) * S3_TICKS_PER_MS)

#define S3_PIN_IR_LEFT		2
#define S3_PIN_IR_CENTER	16
#define S3_PIN_IR_RIGHT		28	/* ADC input 2 */
#define S3_ADC_CH_IR_RIGHT	2

/* Line tracker states (6.3 region A) */
#define S3_LINE_ON_LINE		0
#define S3_LINE_DRIFTING	1
#define S3_LINE_JUNCTION	2
#define S3_LINE_LOST		3

/* Barcode decoder states (6.3 region B) */
#define S3_BC_WAIT_FOR_BAR	0
#define S3_BC_MEASURING		1
#define S3_BC_DECODING		2
#define S3_BC_EMIT_NAV_CMD	3	/* includes the 0.5 s cool-down */

/* ---------------------------------------------------------------------
 *  Steering correction: ONE signed byte every 20 ms, sent with
 *  motion_send_steer() to Motion Control's mbf_steer (subsystem2.h).
 *  + = turn right; S2_STEER_FULL (64) gives a 100 % left/right speed
 *  split.  Motion Control keeps only the newest value, applies it only
 *  in SPEED_FOLLOW, and goes straight if none arrives for 100 ms.
 * --------------------------------------------------------------------- */

#define S3_STEER_MAX		127	/* motion_send_steer() range */

/* ---------------------------------------------------------------------
 *  g_s3_status -- one 32-bit packed word (minimum-bit, Week 8 tribunal),
 *  written ONLY by the Line & Barcode task with one aligned store, so any
 *  task reads a whole, consistent word without a lock.
 *	[1:0]   line_state  S3_LINE_xxx
 *	[3:2]   bc_state    S3_BC_xxx
 *	[5:4]   last_nav    NAV_xxx of the last valid barcode
 *	[6]     nav_valid   1 = at least one barcode decoded
 *	[7]     calibrated  IR calibration done (VC's START guard)
 *	[8]     ir_left     1 = black
 *	[9]     ir_center   1 = black
 *	[10]    gap         both white > 150 ms, line not lost yet
 *	[13:11] line_pos    -2..+2, signed
 *	[21:14] steer       -127..+127, signed (the byte sent to Motion)
 *	[22]    bc_err      a barcode read failed in the last second (FR31)
 *	[23]    bc_overflow bc_ring overflowed since start (sticky, 8.3)
 *	[31:24] reserved, 0
 *  When the team's g_status/mtx_status exists, this word is copied into
 *  subsystem 3's slot there (one writer per word).
 * --------------------------------------------------------------------- */
#define S3_STS_LINE_SHIFT	0
#define S3_STS_BC_SHIFT		2
#define S3_STS_NAV_SHIFT	4
#define S3_STS_NAVOK_SHIFT	6
#define S3_STS_CAL_SHIFT	7
#define S3_STS_IRL_SHIFT	8
#define S3_STS_IRC_SHIFT	9
#define S3_STS_GAP_SHIFT	10
#define S3_STS_POS_SHIFT	11
#define S3_STS_STEER_SHIFT	14
#define S3_STS_ERR_SHIFT	22
#define S3_STS_OVF_SHIFT	23
#define S3_STS_2BIT_MASK	0x3u
#define S3_STS_POS_MASK		0x7u
#define S3_STS_STEER_MASK	0xFFu

typedef struct {
	UINT	line_state, bc_state, last_nav;
	BOOL	nav_valid, calibrated, ir_left, ir_center, gap;
	INT	line_pos, steer;
	BOOL	bc_err, bc_overflow;
} S3_STS_FIELDS;	/* unpacked view, for packing and for readers */

IMPORT UW	s3_sts_pack(const S3_STS_FIELDS *f);	/* clamps ranges */
IMPORT void	s3_sts_unpack(UW w, S3_STS_FIELDS *f);	/* sign-extends */

IMPORT volatile UW	g_s3_status;
IMPORT UW	s3_get_status(void);		/* = g_s3_status */
IMPORT BOOL	s3_is_calibrated(void);

/* ---------------------------------------------------------------------
 *  MQTT telemetry: subsystem 3's 8-bit slot in Buddy 1's fixed binary
 *  telemetry frame (s1_codec.h: S1_SLOT_B3, frame bits 30..37, MSB
 *  first).  Set every 20 ms with s1_tlm_set_slot(); Buddy 1's Telemetry
 *  Publisher sends it every 500 ms.  Subsystem 3 never writes MQTT.
 *	[7:6] line_state   S3_LINE_xxx
 *	[5:4] bc_state     S3_BC_xxx
 *	[3:2] last_nav     4 Nov: barcodes decoded, modulo 4 (the dashboard
 *	                   sees a new read; the letter is in Buddy 1's
 *	                   heartbeat).  Was NAV_xxx, unused since 1 Nov.
 *	[1]   bc_err       read failed in the last second (held 1 s so the
 *	                   500 ms publisher cannot miss it)
 *	[0]   bc_overflow  bc_ring lost edges since start (sticky)
 *  The first 6 bits are the tribunal's "telemetry cut to 6 bits"; the
 *  last 2 report BARCODE_ERR and the bc_ring overflow flag (tribunal:
 *  "send the overflow flag to telemetry").
 * --------------------------------------------------------------------- */
#define S3_TLM_BITS		8
#define S3_TLM_HOLD_CYCLES	(1000 / S3_PERIOD_MS)	/* bc_err held 1 s */

IMPORT UB	s3_tlm_slot(UW sts);	/* g_s3_status -> 8-bit slot */

/* Buddy 1 (subsystem1.h, IF-S1-01).  Weak until Buddy 1's code is on
   main: subsystem 3 then skips the call and still runs. */
#ifndef S1_SLOT_B3
#define S1_SLOT_B3		1
#endif
IMPORT ER	s1_tlm_set_slot(UINT slot, UD bits) __attribute__((weak));
IMPORT void	s1_set_barcode(char letter) __attribute__((weak));	/* 4 Nov */

/* ---------------------------------------------------------------------
 *  Diagnostic counters, for the console and the tests only (not sent).
 *  16-bit and wrapping: compare them as (UH)(after - before).
 *  s3_get_diag() copies them under subsystem 3's mutex.
 * --------------------------------------------------------------------- */
typedef struct {
	UW	adc;		/* [11:0] last IR_RIGHT sample, [23:12] threshold */
	UH	cycles;		/* task cycles */
	UH	samples;	/* barcode samples taken by the ISR */
	UH	overflows;	/* edges lost because bc_ring was full */
	UH	barcodes_ok;
	UH	barcodes_err;	/* BARCODE_ERR reported (FR31) */
	UH	barcodes_discarded;	/* partial reads dropped (NFR22) */
	UH	junctions;
	UH	line_lost;
	UH	steer_sent;	/* steering bytes Motion Control accepted */
	UH	send_fail;	/* TMO_POL sends that found a buffer full */
} S3_DIAG;

#define S3_DIAG_ADC_MASK	0xFFFu
#define S3_DIAG_THR_SHIFT	12

IMPORT void	s3_get_diag(S3_DIAG *d);

/* Why a barcode read failed (console log and S3_BC_RESULT.err) */
#define S3_BCERR_PATTERN	1	/* not a Code 39 pattern */
#define S3_BCERR_FRAME		2	/* missing start/stop '*' */
#define S3_BCERR_UNKNOWN	3	/* valid Code 39, but not one letter A..Z (1 Nov) */

/* ---------------------------------------------------------------------
 *  Vehicle Controller binding, the same way as motion_bind_vc().  The
 *  VC owner calls s3_bind_vc() once in its _init; until then (or with
 *  0) subsystem 3 skips that output and still runs.
 *	mbf_vc_evt  events, one EVT_PACK() word each (app.h):
 *	            EVT_NAV_CMD (NAV_xxx), EVT_JUNCTION, EVT_LINE_LOST,
 *	            EVT_LINE_FOUND, EVT_SENSOR_FAULT (FAULT_IR_RIGHT)
 *	flg_fault   team fault flag: subsystem 3 sets FAULT_BARCODE_ERR
 *	            on every failed read (FR31)
 * --------------------------------------------------------------------- */
IMPORT void	s3_bind_vc(ID mbf_vc_evt, ID flg_fault);

/* Fault bits, in the design's fault list order like subsystem 2's
   FAULT_STALL / FAULT_ENC.  [DECIDE] Buddy 1 assigns the positions; a
   shared header may define these first to override them. */
#ifndef FAULT_BARCODE_ERR
#define FAULT_BARCODE_ERR	0x0020u
#endif
/* IR_RIGHT calibration failed (sensor not on white, unplugged or
   saturated).  Sent as the EVT_SENSOR_FAULT payload and set in
   flg_fault; cleared by subsystem 3 once calibration succeeds.  Its own
   bit, so it never reads as another owner's fault (e.g. FAULT_STALL). */
#ifndef FAULT_IR_RIGHT
#define FAULT_IR_RIGHT		0x0100u
#endif

/* =====================================================================
 *  Private to subsystem 3.  Declared here (not in extra headers) so the
 *  subsystem keeps to the template's files; only subsystem3.c,
 *  task_consumer.c and subsystem3_test.c use them.
 * ===================================================================== */

/* ---------------------------------------------------------------------
 *  Barcode edge detector (subsystem3.c), run by the Barcode ADC ISR
 *
 *	Pure logic, so the tests can feed it a made-up ADC trace.  Black/
 *	white uses hysteresis around white + delta, and the white baseline
 *	follows the surface slowly while the sensor sees white, so a change
 *	in lighting moves the thresholds with it.
 * --------------------------------------------------------------------- */

/* Thresholds in raw ADC counts (0..4095, higher = darker).  Starting
   values: tune with the A3 logs. */
/* 29 Oct (integration fix, from the ADC trace on the robot): white
   ~270, black bars ~2500.  The old baseline followed EVERY reading
   below thr_off, 1/64 per ms, so while the sensor slid onto a bar it
   chased the rising edge: white climbed 264 -> 663 -> 1302 in one
   barcode, the spaces never fell below thr_off and the bars merged.
   Now it only follows readings within S3_BC_WHITE_BAND of white, and
   ~16x slower; thresholds sit nearer the white/black midpoint. */
/* 29 Oct (2): second trace, sensor re-mounted, printed barcode: white
   ~190, printed bars peak only ~650-900 (the black track tape reads
   ~2500; toner reflects more IR).  +1100 was never reached. */
#define S3_BC_DELTA_ON		450	/* black once > white + this (was 800, then 1100) */
#define S3_BC_DELTA_OFF		300	/* white again once < white + this (was 500, then 750) */
/* 1 Nov (driven pass, ADC trace on the robot, 15 cm/s): the track
   barcode is LARGE (narrow ~8 mm, wide ~28 mm, ~42 cm long), black reads
   ~3700-3900, but the paper between the bars only drops to ~630-830
   (the floor/mat white at calibration is ~204).  OFF at white + 300 =
   ~500 was never reached, so every bar merged into one (dropped,
   "23302 ..." = 2.3 s of black).  Now the thresholds follow the black
   level actually seen: ON / OFF at S3_BC_ON_PCT / S3_BC_OFF_PCT of the
   way from white to the darkest bar so far (S3_BC_AMP_INIT before the
   first bar).  With black ~3700: ON ~2100, OFF ~1600; paper ~830. */
#define S3_BC_AMP_INIT		1800	/* black - white assumed before any bar */
#define S3_BC_AMP_MIN		800	/* never below (noise) */
/* 5 Nov (log 19:22: "dropped: 1 elements" over a barcode the sensor
   visibly crossed): the learnt black level only ever grew, so one very
   dark mark (the track tape, which reads darker than printed toner)
   raised the black switch level above the printed bars for good.  Now
   it is forgotten after S3_BC_AMP_FORGET_MS of white: each barcode is
   judged on its own bars.  Within a barcode the longest white is a
   wide gap (21 mm = 175 ms at 12 cm/s); forgetting there is harmless,
   the next bar is re-learnt. */
#define S3_BC_AMP_FORGET_MS	300
/* 5 Nov (capture 20:48): with the sensor mounted higher, the thick
   (21 mm) bars read only ~+1300 above white and the thin (7 mm) bars
   ~+90, so no level relative to the thick bars can see the thin ones.
   S3_BC_LOW_CONTRAST 1: fixed levels just above white (black above
   white + S3_BC_LC_ON, white again below white + S3_BC_LC_OFF), the
   ISR averages S3_BC_LC_AVG samples (2 ms, ~0.3 mm) against noise, and
   white only follows readings within S3_BC_LC_BAND of it.  The 20:48
   trace replayed this way decodes to 'X' (200/200 with noise up to
   2x the measured; +80 started to miss the thinnest bar, +81).  Needs the paper between the
   bars to read like the floor (true on 5 Nov; on 1 Nov the paper read
   ~+450 and this mode would fail): the real fix is the sensor lower,
   then 0. */
#ifndef S3_BC_LOW_CONTRAST
#define S3_BC_LOW_CONTRAST	1	/* 5 Nov: ON, tuned to the sensor as mounted now (0 = learnt levels) */
#endif
/* 5 Nov (capture 21:58): the small barcode's thin bars peak only
   +55..+70 (big one +81..+101), so 65 / 40 never saw them.  32 / 20,
   with S3_BC_SETTLE below: small 'Z' and big 'X' traces both decode
   200/200 in both directions, also with 2x the noise or the small
   bars 30% fainter; random mat marks still never hold the steering. */
#define S3_BC_LC_ON		32	/* was 65 */
#define S3_BC_LC_OFF		20	/* was 40; white noise sd ~6 */
#define S3_BC_LC_BAND		40
#define S3_BC_LC_AVG		8	/* power of two */
/* 5 Nov (capture 21:58, small barcode 'Z'): the thick bars fade out
   slowly (+60..+40 for ~20 ms) and single noisy samples there crossed
   both levels, making a 0.25 ms bar.  A level change now counts only
   after S3_BC_SETTLE samples in a row past the level; both edges are
   late by the same time, so the widths do not change.  Elements
   shorter than that are not seen (4 ms = 0.6 mm at 15 cm/s). */
#if S3_BC_LOW_CONTRAST
#define S3_BC_SETTLE		S3_MS(4)	/* 3 ms still missed some (replay) */
#else
#define S3_BC_SETTLE		1		/* every sample, as before */
#endif
/* 60 / 48: centred near (paper + black) / 2, the midpoint the blurred
   narrow bars and gaps swing around (paper ~650 > floor ~204).  Fitted
   to the 1 Nov trace: also reads 3 mm bars if the sensor is sharp
   enough (sigma ~1.2 mm); 55 / 40 did not.  Barcode sizes on the track:
   narrow 3, 3.5 and 7 mm (141, 165, 328 mm long), none hard-coded. */
#define S3_BC_ON_PCT		60	/* black once > white + 60 % of (black - white) */
#define S3_BC_OFF_PCT		48	/* white again once < white + 48 % */
#define S3_BC_WHITE_SHIFT	12	/* white baseline EMA: 1/4096 per sample = ~1/1024 per ms (was 1/64) */
#define S3_BC_WHITE_BAND	250	/* follow only readings < white + this */

/* Result of s3_bc_detect() */
#define S3_EDGE_NONE		0
#define S3_EDGE_TO_BLACK	1
#define S3_EDGE_TO_WHITE	2

/* Written only by the ISR (one writer per word; no bit-fields, so every
   store is a whole aligned byte, halfword or word) */
typedef struct {
	UW	white_q8;	/* white baseline, 12.8 fixed point (20 bits) */
	UH	thr_on;		/* go black above this (12 bits) */
	UH	thr_off;	/* go white below this (12 bits) */
	UH	amp;		/* 1 Nov: darkest bar so far - white (12 bits) */
	UH	white_n;	/* 5 Nov: samples since the last black, saturates */
	UB	black;		/* current level, after hysteresis: 0/1 */
	UB	pend;		/* 5 Nov: samples past the level so far */
} S3_BC_DET;

IMPORT void	s3_bc_det_init(S3_BC_DET *d, UH white);
IMPORT INT	s3_bc_detect(S3_BC_DET *d, UH adc);	/* S3_EDGE_xxx */

/* ---------------------------------------------------------------------
 *  IR sensor driver (subsystem3.c)
 *
 *	IR_LEFT / IR_CENTER are plain digital inputs, read by the Line &
 *	Barcode task.  IR_RIGHT is sampled every S3_SAMPLE_US by the Barcode ADC ISR
 *	(RP2040 TIMER alarm 3), which runs the edge detector and stores
 *	only the edges in bc_ring, as S3_EDGE_ENTRY halfwords.  The task
 *	drains it every 20 ms.  Every shared word has one writer: the ISR
 *	writes bc_ring's head, the counters and the detector; the task
 *	writes bc_tail.  So no lock is needed, on either core.
 * --------------------------------------------------------------------- */

/* Logic level the line sensors give on black (A3: confirm on the track) */
#define S3_IR_BLACK_LEVEL	1

/* >= 64 entries (8.3); a power of two so the index wraps with a mask */
#define S3_BC_RING_SIZE		64

/* bc_ring entry, 16 bits: bit 15 = new level (1 = black), bits 14..0 =
   time of the edge, ticks, wrapping every 8.192 s */
#define S3_EDGE_BLACK		0x8000
#define S3_EDGE_TIME_MASK	0x7FFF

IMPORT ER	drv_ir_init(void);	/* pins, ADC, barcode sampling */
IMPORT UINT	drv_ir_left(void);	/* 1 = black */
IMPORT UINT	drv_ir_center(void);	/* 1 = black */

/* Start edge detection from the calibrated white level */
IMPORT void	drv_ir_start_edges(UH white);

/* Copy up to max new edges out of bc_ring; returns how many */
IMPORT INT	drv_ir_read_edges(UH *buf, INT max);

/* Times and counters are 16 bits and wrap: compare differences */
/* S3_DEBUG_BARCODE 3 only (capture) */
IMPORT void	drv_ir_cap_arm(UH trigger);	/* start waiting for a pass */
IMPORT BOOL	drv_ir_cap_ready(void);		/* buffer full */
IMPORT const volatile UH *drv_ir_cap_buf(void);
IMPORT UH	drv_ir_ticks(void);		/* ISR's tick counter (edge clock) */
IMPORT UH	drv_ir_last_raw(void);		/* latest ADC sample, 12 bits */
IMPORT UH	drv_ir_threshold(void);		/* current black threshold */
IMPORT UH	drv_ir_white(void);		/* current white baseline (1 Nov) */
IMPORT UH	drv_ir_sample_count(void);	/* samples taken */
IMPORT UH	drv_ir_overflow_count(void);	/* edges lost, ring full */

/* ---------------------------------------------------------------------
 *  Line tracker, region A (subsystem3.c)
 *
 *	Pure logic, no hardware: the task feeds it one IR_LEFT/IR_CENTER
 *	reading per cycle, so the tests can feed it made-up readings.
 *
 *	Sensor layout: the line runs BETWEEN the two sensors.  IR_LEFT sits
 *	left of the tape and IR_CENTER right of it, so both see white when
 *	the robot is centred.
 * --------------------------------------------------------------------- */

#define S3_JUNCTION_SAMPLES	2	/* both black this many cycles */
#define S3_LINE_LOST_MS	1000	/* 6 Nov: was 500.  Both white for MORE than this (> 150 ms,
					   NFR21): longer than one weave across
					   the gap between the sensors */
#define S3_LINE_GAP_MS		150	/* both white this long: gap = 1 */

/* Steering in Motion Control's units (S2_STEER_FULL = 64 = full split).
   Line under one sensor: KP * 2 + KD * change in position (about a 30 %
   speed split).  Both white: a fixed, small drift (SEEK) towards the
   side the line was last touched, just enough to touch an edge again.
   KD was 6: touching the opposite edge after a drift changes the
   position by 3, so it kicked the split to ~60 % and the robot swung
   hard left and right.  Tune on the track. */
/* 4 Nov (video 20261104_192347, straight with junctions): still a
   visible left-right yaw.  An edge touch was 12 (the inner wheel at 63 %
   of the outer at once) plus a KD kick of 2-8: each touch overshot to
   the other edge.  Now a touch starts at 8 and the KD kick is halved;
   a touch that lasts (a curve) still grows through the bend
   (S3_BEND_STEP every 20 ms), so curves keep their full steering. */
/* 6 Nov (smoother line following): together with Motion Control's
   faster steering response (S2_FOLLOW_STEP_MM_S_PER_CYC, subsystem2.h),
   KP 6 cut the simulated straight-line yaw swing from ~34 to ~20 deg
   (median over sensor spacing 24-30 mm, motor lag, 5 % wheel pull);
   curves of 15 cm radius still held.  Straighter driving keeps both
   sensors white for longer (up to ~0.8 s in the simulation), so
   S3_LINE_LOST_MS went 500 -> 1000.  Tune on the track: weave still ->
   KP 7-8; jerky -> 5. */
#define S3_STEER_KP		6	/* 30 Oct: 7 -> 6; 4 Nov: 6 -> 4; 6 Nov: 4 -> 6 */
#define S3_STEER_KD		1	/* 4 Nov: was 2 */
#define S3_STEER_SEEK		8	/* both white: ~12 % split (the most) */
/* 4 Nov (straight-line weave): the drift used to start at full SEEK the
   moment both sensors went white, in the SAME direction as the edge
   correction just made, so the robot kept turning until it hit the
   other edge: a built-in left-right weave.  Now, with the line between
   the sensors (centred), only the trim is sent for S3_SEEK_DELAY_MS;
   then the drift grows by 1 every S3_SEEK_RAMP_MS up to S3_STEER_SEEK,
   so a line that really slipped away is still found (LINE_LOST after
   500 ms keeps the full SEEK sign for the VC's search). */
#define S3_SEEK_DELAY_MS	150
#define S3_SEEK_RAMP_MS		20
/* 30 Oct (team decision): while a barcode is being read (first bar until
   the read ends: decoded, dropped, or the 3 s limit), the steering sent
   to Motion Control is held at the learnt trim only, so the robot drives
   straight and the barcode sensor crosses the bars at an even speed.
   The line tracker keeps running (junction / line lost still reported).
   0 = off.
   30 Oct (2): a pure hold (trim only) drove the robot off the line when
   the read began while it was angled (LINE_LOST -> search mid-barcode).
   The tracker's steering is now only LIMITED to +-S3_BC_STEER_LIMIT
   during a read: it still follows the line, but the inner wheel never
   drops below ~70 % of the outer one, so the robot cannot pivot over
   the bars.  S3_BC_STEER_LIMIT 0 = the pure hold again. */
#define S3_BC_HOLD_STRAIGHT	1
#define S3_BC_STEER_LIMIT	10	/* inner wheel >= v x (1 - 2x10/64) */
/* 4 Nov (video: overshot the loop right after the junction): ANY black
   under the barcode sensor opens a read -- the crossing line at a
   junction, or the track line itself on a curve -- and a read that is
   not a barcode stays open the whole S3_BC_READ_MAX_MS (5 s, ~1 m), so
   the steering stayed limited to +-10 and the robot could not take the
   next curve.  The limit now holds only while edges keep coming: a real
   barcode has an edge every ~50-200 ms (8 / 28 mm bars at 15 cm/s);
   after this long with no new edge it is released (the read itself
   still runs, so a real barcode is still decoded). */
#define S3_BC_HOLD_QUIET_MS	400
/* 4 Nov (curves): the limit only starts once the read has this many
   elements (3 bars + 3 spaces): a line crossing gives 1-2, a barcode
   29.  And never while the tracker is in a curve (|bend| >=
   S3_BEND_CURVE).  A read with fewer elements and no edge for
   S3_BC_HOLD_QUIET_MS is a line, not a barcode: it is dropped at once
   so a real barcode right after it is not missed. */
#define S3_BC_HOLD_MIN_ELEMS	10	/* 5 Nov: was 6.  10 = the '*' start check passed (s3_bc_edge) */
/* 6 Nov (team: drive straight and steady over the barcode; video
   IMG_0848): the limit above started only after the start '*' (10
   elements), so the '*' itself was read while the robot still swung
   from side to side -- the barcode sensor sits at the right front
   corner, so each swing sped it up and slowed it down (in a simulation
   of the robot it even stopped or went backwards) and the '*' did not
   decode.  The limit now starts at the FIRST bar, also while the
   tracker has a "bend" (the weave builds one on a straight too; the
   4 Nov rule "never in a bend" kept the limit off on most passes):
     - first bar until the '*' check (9 elements): limited while edges
       keep coming, released after S3_BC_HOLD_EARLY_QUIET_MS without
       one (a line crossed in a curve: a moment only);
     - '*' passed: limited, released after S3_BC_HOLD_QUIET_MS;
     - '*' failed (mat marks): released for the rest of that read.
   The VC also keeps the speed unchanged during a read (task_vc.c,
   check_curve), so the robot does not slow down half way across.
   Simulation (robot + line tracker + sensor at the right front corner,
   big barcode): read 1% -> ~68% of passes, line never lost. */
#define S3_BC_HOLD_EARLY_QUIET_MS	300
/* 5 Nov: the drop of a short read ("a line, not a barcode") waits
   longer than the steering release above: at 400 ms a slow pass of the
   big barcode (21 mm wide bar, < 5 cm/s) was dropped after 1-2
   elements.  800 ms: still drops a junction / curve line quickly. */
#define S3_BC_DROP_QUIET_MS	800

/* Trim: cancels the robot's own pull to one side (unequal motors or
   wheels).  Every new touch of the left edge means it drifted right, so
   the trim moves left by one step; a right-edge touch moves it back.  On
   a straight line it settles where the two edges are touched equally
   often.  Added to every correction except at a junction.  Cleared by
   the task whenever Motion Control is not in SPEED_FOLLOW (stopped,
   turning, E-STOP), so each line-follow starts from 0. */
/* Bend (integration fix, 28 Oct): on a curve the line stays under one
   sensor, or slips outside it, for a long time, but the corrections
   above are sized for a straight and could not hold an arc: the robot
   ran off, LINE_LOST, and the VC's stop-and-sweep LINE_SEARCH made
   the turns jerky.  The bend grows by S3_BEND_STEP every cycle the
   line stays under the same sensor (none on the first cycle, so a
   brief touch on a straight is unchanged), keeps growing towards the
   last side once both sensors have been white for S3_BEND_WHITE_MS
   (line slipped outside on the curve), and restarts from 0 when the
   OTHER sensor touches.  KP*2 + trim + S3_BEND_MAX ~ 64 = the inner
   wheel stops: the tightest arc the robot can follow. */
#define S3_BEND_STEP		2	/* per 20 ms cycle */
#define S3_BEND_MAX		40
#define S3_BEND_WHITE_MS	200
/* Growing while both white made the straights weave (a centred robot
   is white most of the time).  It only grows while white once a curve
   is confirmed: the line held under one sensor long enough for the
   bend to reach S3_BEND_CURVE (6 cycles = 120 ms).  Below that, while
   white, the bend fades by 1 per cycle.  (28 Oct fix, restored 30 Oct:
   it had been lost in a merge, which brought the weave back.) */
#define S3_BEND_CURVE		12
/* 6 Nov (logs 16:43 / 16:49: "curve" on the straight, the robot
   slowed to 12 cm/s again and again; inner wheel stopped in curves):
   the bend grew from the 2nd cycle of any edge touch, and in a
   left-right swing each touch lasts long enough to build a bend that
   throws the robot across to the other edge -- a slow swing that the
   VC reads as curves.  The bend now grows only once the same edge has
   stayed touched for S3_BEND_DELAY cycles (a real curve keeps it
   touched; a swing does not).  Simulation (robot + motors + VC):
   straight speed 83 -> 148 mm/s, slowed as a "curve" 96 % -> 32 % of
   the time; curves of 11, 15 and 25 cm radius still held both ways
   (25 cycles lost the 11 cm ones: 12 keeps a margin).
   6 Nov (log 17:00, 12 cycles): straight fine, but half way round a
   curve the robot ran wide: in a real curve the line flickers on and
   off the sensor and the inner sensor touches too; each time the bend
   restarted and had to wait the whole delay again.  Now: 8 cycles, and
   none at all for S3_BEND_CURVE_MEM_MS after a curve was seen (|bend|
   >= S3_BEND_CURVE).  Simulation with 20 % sensor flicker: no curve
   lost (11-25 cm radius, both ways), straight 106-142 mm/s. */
/* 6 Nov (videos IMG_0855 / IMG_0858, right curve lost half way): two
   more holes, both found by a simulation with a flickering sensor:
   (1) going in, every 1-cycle white flicker restarted the count, so on
   a tight curve the bend never built up -- the count now survives
   white gaps up to S3_BEND_TOUCH_GAP_MS; (2) half way round, the outer
   sensor touched (the robot turned a little too far), the bend
   restarted from 0, and with the line between the sensors only the
   small seek drift was left: the robot slid out of the curve.  Inside
   the curve memory the bend now grows again while both are white,
   towards the side the line was last seen (as it already did for a
   bend >= S3_BEND_CURVE).  Simulation, curves of 7-25 cm radius, motor
   lag 80/150 ms, 0/20 % flicker, both ways: off course 19/160 (line43)
   -> 0/160; straight 106 -> 112 mm/s. */
#define S3_BEND_DELAY		10	/* cycles (200 ms); 6 Nov: 12, 8, now 10 */
#define S3_BEND_TOUCH_GAP_MS	60	/* white this long ends a touch */
#define S3_BEND_CURVE_MEM_MS	1500
/* 6 Nov (log 18:55: line lost at the END of the right curve, where it
   turns into the next part of the track): a simulation of whole curves
   with exits and S-bends (7-15 cm radius) still ran off 17/96 times --
   mostly when the robot came into a curve fast, touched the line just
   past S3_BEND_DELAY (bend only ~7) and the line then slipped outside
   both sensors: below S3_BEND_CURVE the bend faded and only the small
   seek drift was left.  A touch longer than S3_BEND_DELAY now also
   lets the bend grow while white ("slipped outside") for
   S3_BEND_SLIP_MS; it does NOT skip the delay on the next touch (that
   brought the weave back on the straights).  Simulation: off course
   17/96 -> 0/96; straight ~99 mm/s (line43 108, steady41 82). */
#define S3_BEND_SLIP_MS		1000

#define S3_TRIM_STEP		2
#define S3_TRIM_MAX		6	/* +-25 % split at most */

#define S3_DEBUG_LINE		0	/* 1: print sensors/steer every 0.5 s */
/* Barcode log (integration aid, 29 Oct):
   0 = only good reads and BARCODE_ERR (as before)
   1 = also dropped (partial) reads, and the element widths in ms of
       every read that ends: shows whether the sensor saw the bars
   2 = 1 + the barcode sensor's ADC level and threshold every 0.5 s
       (raw rises over the threshold on black) */
#define S3_DEBUG_BARCODE	1	/* 3 = capture of one pass (1 Nov: 1, the dump stalls the task) */
/* Level 3 (integration aid, 29 Oct): the ISR records the barcode ADC
   from the first sample darker than white + S3_CAP_TRIGGER, one value
   every S3_CAP_MS ms, S3_CAP_LEN values, then the task prints them
   all ("s3: cap ...") and re-arms.  Shows the true peaks and dips the
   detector sees, for tuning the thresholds off-line. */
#define S3_CAP_TRIGGER		150
#define S3_CAP_MS		2	/* ms per stored value */
#define S3_CAP_DECIM		S3_MS(S3_CAP_MS)	/* samples per stored value */
#define S3_CAP_LEN		1500	/* 3 s of data, 3 KB */

/* Events returned by s3_line_step() */
#define S3_LEV_NONE		0
#define S3_LEV_JUNCTION		1
#define S3_LEV_LINE_LOST	2
#define S3_LEV_LINE_FOUND	3

/* Used only by the Line & Barcode task, so bit-fields are safe */
typedef struct {
	UB	state : 2;	/* S3_LINE_xxx */
	UB	both_black : 2;	/* cycles with L = C = black, saturates at 3 */
	UB	gap : 1;	/* both white > S3_LINE_GAP_MS, not lost yet */
	UB	side_left : 1;	/* line last touched by IR_LEFT (else IR_CENTER) */
	B	pos : 3;	/* coarse line position, -2..+2 */
	UH	white_ms;	/* both white this long, saturates */
	B	steer;		/* steering byte for this cycle */
	B	trim;		/* learnt drift correction, +-S3_TRIM_MAX */
	B	bend;		/* curve correction, +-S3_BEND_MAX */
	UB	touch_n;	/* 6 Nov: cycles the same edge has stayed touched */
	B	touch_pos;	/* 6 Nov: that edge, +-2 (0 = none yet) */
	UH	curve_ms;	/* 6 Nov: a curve (|bend| >= S3_BEND_CURVE) was seen this recently */
	UH	slip_ms;	/* 6 Nov: a touch longer than S3_BEND_DELAY ended this recently */
} S3_LINE;

IMPORT void	s3_line_init(S3_LINE *lt);

/* left, center: 1 = black.  dt_ms: time since the previous call.
   Returns S3_LEV_xxx; lt->state, pos, steer and gap are updated. */
IMPORT INT	s3_line_step(S3_LINE *lt, UINT left, UINT center, UW dt_ms);

/* ---------------------------------------------------------------------
 *  Barcode decoder, region B (subsystem3.c)
 *
 *	Pure logic, no hardware: the task feeds it the edges from bc_ring,
 *	so the tests can feed it made-up edges.  Bar widths are the times
 *	between edges in ticks (not encoder counts).
 *
 *	Barcodes are Code 39: *X* = start '*', payload, stop '*'.  Each
 *	character is 9 elements (5 bars, 4 spaces), exactly 3 of them wide,
 *	with a narrow space between characters, so a k-character barcode has
 *	10k - 1 elements.  Decoding compares widths inside each character, so
 *	it works at any steady speed and in either reading direction.
 *
 *	Speed range: the no-edge time-out (end of a partial read) is
 *	S3_BC_EDGE_TIMEOUT_MS, stretched to S3_BC_TIMEOUT_WIDTHS x the widest
 *	element of the read so far when the robot is slow (a wide element is
 *	at most 3x a narrow one), up to S3_BC_SLOW_TIMEOUT_MS.  Without this
 *	a slow pass (a wide bar > 200 ms, i.e. < ~3 cm/s for 6 mm bars) was
 *	cut off half way and discarded.
 * --------------------------------------------------------------------- */

#define S3_BC_EDGE_TIMEOUT_MS	200	/* NFR22: no edge -> end of read */
#define S3_BC_TIMEOUT_WIDTHS	4	/* ... or this x the widest element */
#define S3_BC_SLOW_TIMEOUT_MS	2000	/* ... but never longer than this */
/* 30 Oct (team decision): a read stays open from its first bar until
   EITHER a whole barcode (29/39/49 elements) followed by a quiet gap
   (> 2x its widest element), OR S3_BC_READ_MAX_MS, whichever comes
   first.  The no-edge time-out above no longer ends a read (it only
   sets the quiet wait after an abort).  A full pass of the track
   barcode took 2.7 s first bar -> last bar on the robot.  Replaces
   NFR22's 200 ms no-edge discard: update the report. */
/* 6 Nov (video IMG_0848): the robot crossed an A4 barcode at only
   ~4 cm/s (~7 s), so the 5 s limit cut every read after 9-11 elements.
   12 s = 297 mm at 2.5 cm/s.  A read whose edges stop (robot stopped)
   still ends after S3_BC_SLOW_TIMEOUT_MS without an edge. */
#define S3_BC_READ_MAX_MS	12000	/* 6 Nov: was 5000.  5 Nov: 5000 again: the 328 mm barcode (Barcode_Sample.pdf)
					   takes 2.7 s at the 12 cm/s curve speed, +quiet.  (1 Nov: was 3000; the 42 cm
					   barcode takes 2.8 s at 15 cm/s, +quiet.  4 Nov: 3000
					   again -- demo barcodes are at most an A4 long,
					   297 mm = 2.0 s at 15 cm/s) */
#define S3_BC_COOLDOWN_MS	500	/* NFR23: ignore sensor after a command */
#define S3_BC_MAX_ELEMS		49	/* up to 5 characters incl. '*' '*' */
#define S3_BC_MIN_RATIO_X10	15	/* narrowest wide >= 1.5 x widest narrow */
/* 1 Nov: speed-tolerant decoding (subsystem3.c, decode_char_fit): each
   character is matched against all 44 Code 39 patterns with a speed
   that may change steadily across it (e.g. the VC slowing for a curve
   on the barcode).  Accepted if the best fit's rms error <
   S3_BC_FIT_MAX_PCT % and the next best pattern fits at least
   S3_BC_FIT_MARGIN_X10 / 10 x worse.  The 1 Nov "Y" read (two wide bars
   73 / 145 ms, wide gap only 1.2x) failed the 1.5x rank check above. */
#define S3_BC_FIT_MAX_PCT	30
#define S3_BC_FIT_MARGIN_X10	15
/* 5 Nov: the barcode sensor read darker than white + this, but never
   reached the black level, for a whole pass: printed once per pass
   ("s3: barcode sensor saw grey ... never black"), so a sensor that is
   too high / beside the bars shows in the log instead of silence. */
#define S3_BC_GREY_DELTA	300

/* 1 Nov: no letter -> command table any more.  Every barcode whose
   payload is ONE letter 'A'..'Z' is a valid read (S3_BEV_NAV_CMD, ch
   set, cmd = NAV_NONE); the letter goes to the Vehicle Controller
   (EVT_BARCODE), which maps it to a movement with the mapping received
   by MQTT.  Other valid Code 39 payloads (digits, symbols, 2+ letters)
   are BARCODE_ERR S3_BCERR_UNKNOWN. */
IMPORT BOOL	s3_bc_is_letter(const char *text);

/* Code 39 patterns: 9 bits, element 0 (first bar) in bit 8, 1 = wide */
typedef struct {
	char	ch;
	UH	pattern;
} S3_C39;

IMPORT const S3_C39	s3_code39[];
IMPORT const INT	s3_code39_len;

/* Result of s3_bc_edge() / s3_bc_poll() / s3_bc_abort() */
#define S3_BEV_NONE		0
#define S3_BEV_NAV_CMD		1	/* valid barcode: ch = letter, rev set (1 Nov) */
#define S3_BEV_BARCODE_ERR	2	/* complete read that failed: err */
#define S3_BEV_DISCARD		3	/* partial read dropped (NFR22) */

/* Result of one call; a local variable, never stored or shared */
typedef struct {
	UB	cmd : 3;	/* NAV_NONE (1 Nov: the VC maps the letter) */
	UB	err : 2;	/* S3_BCERR_xxx */
	UB	rev : 1;	/* 1 = decoded right->left (end '*' seen first) */
	UB	elements;	/* elements in the read, 0..S3_BC_MAX_ELEMS */
	char	ch;		/* first payload character */
} S3_BC_RESULT;

/* Element widths are ticks between edges.  An element can never reach
   S3_BC_SLOW_TIMEOUT_MS (6 Nov: a read ends after that long without an
   edge) plus one 20 ms cycle, so a halfword holds every width.  Used only by the task. */
typedef struct {
	UB	state : 2;	/* S3_BC_xxx */
	UB	black : 1;	/* level after the last edge */
	UB	hold : 1;	/* after an abort: wait for a quiet gap */
	UB	nostart : 1;	/* 6 Nov: this read failed the '*' start check */
	UB	n;		/* elements stored, 0..S3_BC_MAX_ELEMS */
	UH	run_max;	/* widest element of this read, ticks */
	UH	last_t;		/* time of the last edge (16-bit tick clock) */
	UH	start_t;	/* time of the first bar of this read */
	UH	until_t;	/* end of the cool-down or of a hold */
	UH	elem[S3_BC_MAX_ELEMS];	/* widths, ticks */
} S3_BC;

IMPORT void	s3_bc_init(S3_BC *bc);

/* One edge at time t (ticks; black = new level).  Returns S3_BEV_xxx. */
IMPORT INT	s3_bc_edge(S3_BC *bc, BOOL black, UH t, S3_BC_RESULT *res);

/* Time-outs: call every cycle, after the cycle's edges.  S3_BEV_xxx. */
IMPORT INT	s3_bc_poll(S3_BC *bc, UH now, S3_BC_RESULT *res);

/* bc_ring overflowed: drop this barcode, send no NAV_CMD, and start
   again only after S3_BC_EDGE_TIMEOUT_MS without an edge (8.3).
   Returns S3_BEV_DISCARD if a read was in progress, else S3_BEV_NONE. */
IMPORT INT	s3_bc_abort(S3_BC *bc, UH now, S3_BC_RESULT *res);

/* Decode n element widths.  Writes up to max-1 payload characters and a
   '\0' to text.  Returns the payload length, or -S3_BCERR_xxx. */
IMPORT INT	s3_code39_decode(const UH *w, INT n, char *text, INT max);
/* Same; *rev = 1 if it only decoded read backwards (1 Nov) */
IMPORT INT	s3_code39_decode_rev(const UH *w, INT n, char *text, INT max,
				     BOOL *rev);

/* Objects and the task (subsystem3.c, task_consumer.c) */
IMPORT ID	tid_line_barcode;
IMPORT ID	cyc_line_barcode;
IMPORT ID	mtx_s3_status;		/* guards s3_diag */
IMPORT S3_DIAG	s3_diag;
IMPORT void	line_barcode_task(INT stacd, void *exinf);
IMPORT void	line_barcode_cyclic_handler(void *exinf);

/* ---------------------------------------------------------------------
 *  TEMPORARY: the template's example consumer (IF-01, IF-04).  The
 *  example producer (S2) and monitor (S4) still use these; delete this
 *  block and task_consumer.c once both have replaced their examples.
 * --------------------------------------------------------------------- */

#define S3_PRI_CONSUMER		16
#define S3_BATCH_SIZE		5

IMPORT ID	mbx_samples;

typedef struct {
	UW	received;	/* samples received so far */
	INT	min;
	INT	max;
	W	sum;
} APP_STATS;

IMPORT ID		mtx_stats;
IMPORT APP_STATS	app_stats;

IMPORT void	consumer_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */