/*
 *	encoder.h
 *	Wheel encoders (single channel), private to subsystem 2.
 *
 *	Protocol (report Part C row 3):
 *	  writer  = encoder ISR only (byte increment, no kernel calls)
 *	  reader  = Motion Control only, at least every 40 ms
 *	  data    = two wrapping uint8 counters in ONE aligned halfword, so
 *	            one 16-bit read is a consistent left/right snapshot;
 *	            delta = (UB)(new - old); the sign comes from the
 *	            commanded direction (the encoders cannot tell it).
 *	  period  = 16-bit time between the last two edges (us, 0xFFFF =
 *	            65 ms or more), for speed at low pulse rates.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "subsystem2.h"

#define ENC_LEFT	0
#define ENC_RIGHT	1
#define ENC_PERIOD_NONE	0xFFFFu

typedef union {
	UH	both;		/* read this in one access */
	UB	c[2];		/* c[ENC_LEFT], c[ENC_RIGHT] */
} ENC_COUNTS;

typedef struct {
	ENC_COUNTS	cnt;
	UH		period_us[2];
} ENC_SNAPSHOT;

IMPORT ER	encoder_init(void);		/* pins, pull-ups, interrupts */
IMPORT void	encoder_snapshot(ENC_SNAPSHOT *s);
Inline UINT	encoder_delta(UB old_c, UB new_c) { return (UB)(new_c - old_c); }

#endif /* ENCODER_H */
