/*
 *	gpio_irq.c
 *	Shared GPIO edge-interrupt dispatcher.  See gpio_irq.h.
 *
 *	APIs shown: tk_def_int, EnableInt, ClearInt, in_w / out_w / set_w
 *
 *	libbsp has no GPIO-interrupt support, so the IO_BANK0 registers are
 *	used directly (RP2040 datasheet 2.19.6).  Each GPIO has 4 event bits
 *	(LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH), 8 GPIOs per register.
 *	Edge events are cleared by writing 1 to the raw INTR register.
 */

#include "gpio_irq.h"
#include <tk/syslib.h>

#define IOB_INTR(r)		(IO_BANK0_BASE + 0x0F0 + (r) * 4)
#define IOB_PROC0_INTE(r)	(IO_BANK0_BASE + 0x100 + (r) * 4)
#define IOB_PROC0_INTS(r)	(IO_BANK0_BASE + 0x120 + (r) * 4)
#define IOB_PROC1_INTE(r)	(IO_BANK0_BASE + 0x130 + (r) * 4)
#define IOB_PROC1_INTS(r)	(IO_BANK0_BASE + 0x150 + (r) * 4)
#define SIO_CPUID		0xd0000000

typedef struct {
	UB		pin;
	UB		edges;
	GPIO_IRQ_CB	cb;
} GPIO_IRQ_ENTRY;

LOCAL GPIO_IRQ_ENTRY	irq_tab[GPIO_IRQ_MAX_PINS];
LOCAL volatile UB	irq_cnt;	/* entries 0..irq_cnt-1 are valid */
LOCAL UB		irq_core;	/* core that takes the interrupt */
LOCAL BOOL		irq_ready;

/* The one handler for IO_IRQ_BANK0 */
LOCAL void gpio_irq_handler(UINT intno)
{
	UINT	i, n, pin, reg, sh, ev;
	UW	ints;

	(void)intno;
	n = irq_cnt;
	for (i = 0; i < n; i++) {
		pin = irq_tab[i].pin;
		reg = pin >> 3;
		sh  = (pin & 7u) * 4u;
		ints = irq_core ? in_w(IOB_PROC1_INTS(reg))
				: in_w(IOB_PROC0_INTS(reg));
		ev = (ints >> sh) & (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);
		if (ev != 0) {
			out_w(IOB_INTR(reg), ev << sh);	/* acknowledge */
			irq_tab[i].cb(pin, ev);
		}
	}
	ClearInt(GPIO_IRQ_INTNO);
}

EXPORT ER gpio_irq_init(void)
{
	T_DINT	dint;
	ER	er;

	if (irq_ready) return E_OK;

	irq_core = (UB)(in_w(SIO_CPUID) & 1u);
	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)gpio_irq_handler;
	er = tk_def_int(GPIO_IRQ_INTNO, &dint);
	if (er < E_OK) return er;

	ClearInt(GPIO_IRQ_INTNO);
	EnableInt(GPIO_IRQ_INTNO, GPIO_IRQ_LEVEL);
	irq_ready = TRUE;
	return E_OK;
}

EXPORT ER gpio_irq_attach(UINT pin, UINT edges, GPIO_IRQ_CB cb)
{
	UINT	reg, sh, n;
	UINT	imask;

	edges &= (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);
	if (pin >= GPIO_NUM || edges == 0 || cb == NULL) return E_PAR;
	if (!irq_ready) return E_OBJ;

	DI(imask);
	n = irq_cnt;
	if (n >= GPIO_IRQ_MAX_PINS) {
		EI(imask);
		return E_LIMIT;
	}
	irq_tab[n].pin   = (UB)pin;
	irq_tab[n].edges = (UB)edges;
	irq_tab[n].cb    = cb;
	irq_cnt = (UB)(n + 1);		/* publish the entry last */
	EI(imask);

	reg = pin >> 3;
	sh  = (pin & 7u) * 4u;
	out_w(IOB_INTR(reg), 0xFu << sh);	/* drop stale edges */
	if (irq_core) {
		set_w(IOB_PROC1_INTE(reg), (UW)edges << sh);
	} else {
		set_w(IOB_PROC0_INTE(reg), (UW)edges << sh);
	}
	return E_OK;
}
