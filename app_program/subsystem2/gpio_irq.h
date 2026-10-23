/*
 *	gpio_irq.h
 *	Shared GPIO edge-interrupt dispatcher (RP2040 IO_IRQ_BANK0, IRQ 13).
 *
 *	The RP2040 has ONE interrupt for all GPIO pins, so only one handler
 *	may be defined for it.  This dispatcher owns that interrupt and calls
 *	a small callback per pin.  Used by the wheel encoders (Buddy 2,
 *	GP0/GP7) and intended for the ultrasonic echo (Buddy 5, GP5):
 *
 *	    #include "../subsystem2/gpio_irq.h"
 *	    gpio_irq_attach(5, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, echo_cb);
 *
 *	Callbacks run in INTERRUPT context: keep them short, never wait,
 *	no tm_printf.  Only tk_set_flg / tk_sig_sem / tk_wup_tsk style calls.
 *	The interrupt is taken on the core that called gpio_irq_init()
 *	(core of usermain()).  [DECIDE] move to a shared folder if the team
 *	prefers; the API stays the same.
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include "../app.h"

#define GPIO_IRQ_EDGE_FALL	0x4u	/* IO_BANK0 event bit 2 */
#define GPIO_IRQ_EDGE_RISE	0x8u	/* IO_BANK0 event bit 3 */

#define GPIO_IRQ_INTNO		13	/* IO_IRQ_BANK0 */
#define GPIO_IRQ_LEVEL		2	/* same level as the other drivers */
#define GPIO_IRQ_MAX_PINS	4

/* cb(pin, events): events = GPIO_IRQ_EDGE_* bits that fired */
typedef void (*GPIO_IRQ_CB)(UINT pin, UINT events);

IMPORT ER gpio_irq_init(void);		/* idempotent; called by subsystem2_init */
IMPORT ER gpio_irq_attach(UINT pin, UINT edges, GPIO_IRQ_CB cb);

#endif /* GPIO_IRQ_H */
