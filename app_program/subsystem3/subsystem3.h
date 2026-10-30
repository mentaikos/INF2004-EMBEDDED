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
 *	[3:2] last_nav     NAV_xxx (00 = LEFT, also before the first barcode)
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
#define S3_BCERR_UNKNOWN	3	/* valid barcode, not in the table */

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
	UB	black;		/* current level, after hysteresis: 0/1 */
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
#define S3_LINE_LOST_MS	 500	/* both white for MORE than this (> 150 ms,
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
#define S3_STEER_KP		6	/* 30 Oct: was 7 (edge touch 14 -> 12, slightly softer) */
#define S3_STEER_KD		2
#define S3_STEER_SEEK		8	/* both white: ~12 % split */
/* 30 Oct (team decision): while a barcode is being read (first bar until
   the read ends: decoded, dropped, or the 3 s limit), the steering sent
   to Motion Control is held at the learnt trim only, so the robot drives
   straight and the barcode sensor crosses the bars at an even speed.
   The line tracker keeps running (junction / line lost still reported).
   0 = off. */
#define S3_BC_HOLD_STRAIGHT	1

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

#define S3_TRIM_STEP		2
#define S3_TRIM_MAX		6	/* +-25 % split at most */

#define S3_DEBUG_LINE		0	/* 1: print sensors/steer every 0.5 s */
/* Barcode log (integration aid, 29 Oct):
   0 = only good reads and BARCODE_ERR (as before)
   1 = also dropped (partial) reads, and the element widths in ms of
       every read that ends: shows whether the sensor saw the bars
   2 = 1 + the barcode sensor's ADC level and threshold every 0.5 s
       (raw rises over the threshold on black) */
#define S3_DEBUG_BARCODE	3	/* 3 = capture of one pass; back to 1 after */
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
#define S3_BC_READ_MAX_MS	3000	/* longest a read stays open */
#define S3_BC_COOLDOWN_MS	500	/* NFR23: ignore sensor after a command */
#define S3_BC_MAX_ELEMS		49	/* up to 5 characters incl. '*' '*' */
#define S3_BC_MIN_RATIO_X10	15	/* narrowest wide >= 1.5 x widest narrow */

/* Command table (FR29): A left, B right, C straight, D U-turn */
typedef struct {
	const char	*text;	/* payload between the '*'s */
	UB		cmd;	/* NAV_xxx */
} S3_BC_ENTRY;

IMPORT const S3_BC_ENTRY	s3_bc_table[];
IMPORT const INT		s3_bc_table_len;

/* Code 39 patterns: 9 bits, element 0 (first bar) in bit 8, 1 = wide */
typedef struct {
	char	ch;
	UH	pattern;
} S3_C39;

IMPORT const S3_C39	s3_code39[];
IMPORT const INT	s3_code39_len;

/* Result of s3_bc_edge() / s3_bc_poll() / s3_bc_abort() */
#define S3_BEV_NONE		0
#define S3_BEV_NAV_CMD		1	/* valid barcode, cmd/ch filled in */
#define S3_BEV_BARCODE_ERR	2	/* complete read that failed: err */
#define S3_BEV_DISCARD		3	/* partial read dropped (NFR22) */

/* Result of one call; a local variable, never stored or shared */
typedef struct {
	UB	cmd : 3;	/* NAV_xxx, or NAV_NONE */
	UB	err : 2;	/* S3_BCERR_xxx */
	UB	elements;	/* elements in the read, 0..S3_BC_MAX_ELEMS */
	char	ch;		/* first payload character */
} S3_BC_RESULT;

/* Element widths are ticks between edges.  An element can never reach
   S3_BC_READ_MAX_MS (12000 ticks) plus one 20 ms cycle without the
   read ending, so a halfword holds every width.  Used only by the task. */
typedef struct {
	UB	state : 2;	/* S3_BC_xxx */
	UB	black : 1;	/* level after the last edge */
	UB	hold : 1;	/* after an abort: wait for a quiet gap */
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

/* Payload -> NAV_xxx, or NAV_NONE if not in s3_bc_table */
IMPORT UB	s3_bc_lookup(const char *text);

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