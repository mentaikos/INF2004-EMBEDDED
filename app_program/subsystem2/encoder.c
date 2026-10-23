/*
 *	encoder.c
 *	Wheel encoder ISR and snapshot, private to subsystem 2.
 *
 *	APIs shown: gpio_set_pin, pad pull-up (PADS_BANK0), shared GPIO IRQ
 *
 *	The ISR runs on the core that called gpio_irq_init() (core 0); the
 *	Motion task may run on the other core.  This is still safe without a
 *	lock because:
 *	  - each counter byte and each period halfword has ONE writer (ISR);
 *	  - byte / halfword / word stores and loads are single-copy atomic;
 *	  - the reader takes both counters in ONE halfword load.
 *	No sub-byte bit-fields are written here (team packing rule).
 */

#include "encoder.h"
#include "gpio_irq.h"
#include <tk/syslib.h>

LOCAL volatile ENC_COUNTS	enc_cnt __attribute__((aligned(2)));
LOCAL volatile UH		enc_period[2];	/* written by ISR only */
LOCAL UW			enc_last_us[2];	/* ISR private, full width for debounce/period */

LOCAL void encoder_edge(UINT pin, UINT events)
{
	UINT	i = (pin == S2_PIN_ENC_L) ? ENC_LEFT : ENC_RIGHT;
	UW	now = in_w(TIMER_TIMERAWL);	/* 1 us free-running */
	UW	dt  = now - enc_last_us[i];

	(void)events;
	if (dt < S2_ENC_DEBOUNCE_US) return;	/* contact bounce / noise */
	enc_last_us[i] = now;
	enc_period[i]  = (dt > 0xFFFEu) ? ENC_PERIOD_NONE : (UH)dt;
	enc_cnt.c[i]++;				/* byte store, wraps at 256 */
}

EXPORT ER encoder_init(void)
{
	ER	er;

	enc_cnt.both  = 0;
	enc_period[0] = enc_period[1] = ENC_PERIOD_NONE;

	gpio_set_pin(S2_PIN_ENC_L, GPIO_MODE_IN);
	gpio_set_pin(S2_PIN_ENC_R, GPIO_MODE_IN);
	set_w(GPIO(S2_PIN_ENC_L), GPIO_PUE);	/* open-collector sensors */
	set_w(GPIO(S2_PIN_ENC_R), GPIO_PUE);

	er = gpio_irq_init();
	if (er < E_OK) return er;
	er = gpio_irq_attach(S2_PIN_ENC_L, GPIO_IRQ_EDGE_RISE, encoder_edge);
	if (er < E_OK) return er;
	return gpio_irq_attach(S2_PIN_ENC_R, GPIO_IRQ_EDGE_RISE, encoder_edge);
}

EXPORT void encoder_snapshot(ENC_SNAPSHOT *s)
{
	s->cnt.both     = enc_cnt.both;	/* one 16-bit load: L and R together */
	s->period_us[0] = enc_period[0];
	s->period_us[1] = enc_period[1];
}
