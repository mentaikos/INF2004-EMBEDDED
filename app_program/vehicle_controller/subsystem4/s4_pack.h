/*
 *	s4_pack.h
 *	Subsystem 4 -- minimum-bit representation of the IMU status
 *	(Week 8 tribunal change requirement).  Included by subsystem4.h, so
 *	other subsystems get the unpack helpers; it needs no kernel calls,
 *	so the PC test (host_test/) uses the very same code.
 *
 *	IMU status: 42 bits in two aligned 32-bit words, written ONLY by the
 *	IMU task (one aligned store per word, under mtx_s4_status so a
 *	reader taking both words gets them from the same cycle).
 *
 *	word 0 (31 bits used)
 *	  [2:0]   state        3  S4_ST_xxx (6 states)
 *	  [11:3]  pitch        9  signed, 0.1 deg, -25.6..+25.5 deg
 *	  [20:12] heading      9  0..359 deg
 *	  [29:21] turn rate    9  signed, 1 deg/s, -256..+255
 *	  [30]    calibrated   1
 *	  [31]    reserved, 0
 *	word 1 (11 bits used)
 *	  [4:0]   hump height  5  0..31 mm, 1 mm (track humps: 25 mm, 10 mm)
 *	  [8:5]   hump count   4  0..15 (saturates)
 *	  [10:9]  I2C fails    2  0..3 consecutive failed reads
 *	  [31:11] reserved, 0
 *
 *	Out-of-range values are CLAMPED, never wrapped (a 26 deg pitch must
 *	not read back as -25 deg).  Explicit masks and shifts, not C
 *	bit-fields, so the layout is the same on every compiler.
 */

#ifndef S4_PACK_H
#define S4_PACK_H

#include "imu_types.h"

/* IMU Monitor states (statechart 6.4): 6 states -> 3 bits */
#define S4_ST_CALIBRATING	0
#define S4_ST_LEVEL		1
#define S4_ST_CLIMBING		2
#define S4_ST_DESCENDING	3
#define S4_ST_SHOCK		4
#define S4_ST_FAULT		5

/* Unpacked view: the Week 13 "before" representation (8 x 4 = 32 B)
   and what readers unpack into.  The packed form is 2 x 4 = 8 B. */
typedef struct {
	INT	state;		/* S4_ST_xxx */
	INT	pitch_dd;	/* 0.1 deg, + = nose up */
	INT	heading_deg;	/* 0..359 */
	INT	rate_dps;	/* deg/s, sign follows the heading change */
	INT	calibrated;	/* 0/1 */
	INT	hump_mm;	/* height of the last hump */
	INT	hump_count;	/* humps measured since start */
	INT	fail_count;	/* consecutive failed I2C reads */
} S4_STS_FIELDS;

#define S4_W0_STATE_SHIFT	0
#define S4_W0_PITCH_SHIFT	3
#define S4_W0_HDG_SHIFT		12
#define S4_W0_RATE_SHIFT	21
#define S4_W0_CAL_SHIFT		30
#define S4_W1_HUMP_SHIFT	0
#define S4_W1_COUNT_SHIFT	5
#define S4_W1_FAIL_SHIFT	9

#define S4_STATE_MASK		0x7u
#define S4_9BIT_MASK		0x1FFu
#define S4_HUMP_MASK		0x1Fu
#define S4_COUNT_MASK		0xFu
#define S4_FAIL_MASK		0x3u

#define S4_PITCH_MIN_DD		(-256)
#define S4_PITCH_MAX_DD		255
#define S4_RATE_MIN_DPS		(-256)
#define S4_RATE_MAX_DPS		255
#define S4_HUMP_MAX_MM		31
#define S4_COUNT_MAX		15
#define S4_FAIL_MAX		3

Inline W s4_clamp(W v, W lo, W hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* Sign-extend the low 'bits' bits of v */
Inline W s4_sext(UW v, INT bits)
{
	UW	m = 1u << (bits - 1);

	v &= (m << 1) - 1u;
	return (W)(v ^ m) - (W)m;
}

Inline UW s4_pack_w0(const S4_STS_FIELDS *f)
{
	W	hdg = f->heading_deg % 360;

	if (hdg < 0) hdg += 360;
	return  (((UW)f->state & S4_STATE_MASK) << S4_W0_STATE_SHIFT)
	      | (((UW)s4_clamp(f->pitch_dd, S4_PITCH_MIN_DD, S4_PITCH_MAX_DD)
						& S4_9BIT_MASK) << S4_W0_PITCH_SHIFT)
	      | (((UW)hdg & S4_9BIT_MASK) << S4_W0_HDG_SHIFT)
	      | (((UW)s4_clamp(f->rate_dps, S4_RATE_MIN_DPS, S4_RATE_MAX_DPS)
						& S4_9BIT_MASK) << S4_W0_RATE_SHIFT)
	      | ((f->calibrated ? 1u : 0u) << S4_W0_CAL_SHIFT);
}

Inline UW s4_pack_w1(const S4_STS_FIELDS *f)
{
	return  (((UW)s4_clamp(f->hump_mm, 0, S4_HUMP_MAX_MM))
						<< S4_W1_HUMP_SHIFT)
	      | (((UW)s4_clamp(f->hump_count, 0, S4_COUNT_MAX))
						<< S4_W1_COUNT_SHIFT)
	      | (((UW)s4_clamp(f->fail_count, 0, S4_FAIL_MAX))
						<< S4_W1_FAIL_SHIFT);
}

/* Single-field readers.  Word 0 alone is one aligned word, so a reader
   that needs only word-0 fields (state, pitch, ...) needs no lock. */
Inline INT  s4_sts_state(UW w0)    { return (INT)((w0 >> S4_W0_STATE_SHIFT) & S4_STATE_MASK); }
Inline INT  s4_sts_pitch_dd(UW w0) { return (INT)s4_sext(w0 >> S4_W0_PITCH_SHIFT, 9); }
Inline INT  s4_sts_heading(UW w0)  { return (INT)((w0 >> S4_W0_HDG_SHIFT) & S4_9BIT_MASK); }
Inline INT  s4_sts_rate(UW w0)     { return (INT)s4_sext(w0 >> S4_W0_RATE_SHIFT, 9); }
Inline INT  s4_sts_calibrated(UW w0) { return (INT)((w0 >> S4_W0_CAL_SHIFT) & 1u); }
Inline INT  s4_sts_hump_mm(UW w1)  { return (INT)((w1 >> S4_W1_HUMP_SHIFT) & S4_HUMP_MASK); }
Inline INT  s4_sts_hump_count(UW w1) { return (INT)((w1 >> S4_W1_COUNT_SHIFT) & S4_COUNT_MASK); }
Inline INT  s4_sts_fails(UW w1)    { return (INT)((w1 >> S4_W1_FAIL_SHIFT) & S4_FAIL_MASK); }

Inline void s4_sts_unpack(UW w0, UW w1, S4_STS_FIELDS *f)
{
	f->state       = s4_sts_state(w0);
	f->pitch_dd    = s4_sts_pitch_dd(w0);
	f->heading_deg = s4_sts_heading(w0);
	f->rate_dps    = s4_sts_rate(w0);
	f->calibrated  = s4_sts_calibrated(w0);
	f->hump_mm     = s4_sts_hump_mm(w1);
	f->hump_count  = s4_sts_hump_count(w1);
	f->fail_count  = s4_sts_fails(w1);
}

/* EVT_HUMP_END payload (app.h): peak height, 5 bits */
#define S4_EVT_HEIGHT(mm)	((UW)s4_clamp((mm), 0, S4_HUMP_MAX_MM))

#endif /* S4_PACK_H */
