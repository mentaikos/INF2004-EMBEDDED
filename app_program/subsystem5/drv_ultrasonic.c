/*
 * HC-SR04+ (3.3 V echo) + SG90. Uses Buddy 2's GPIO dispatcher,
 * never replaces IRQ 13. ISR timestamps and signals, never waits.
 */
#include "drv_ultrasonic.h"
#include "../subsystem2/gpio_irq.h"
#include <tk/syslib.h>
#include <tk/smp.h>

#define S5_PWM_REG(offset)     (PWM_BASE + 6u * 0x14u + (offset))
#define S5_PWM_DIV             125u     /* 125 MHz / 125 = 1 MHz */
#define S5_PWM_WRAP            19999u   /* 1 MHz / 20000 = 50 Hz */
#define S5_ECHO_EDGES          (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)
#define S5_ECHO_INTR           (IO_BANK0_BASE + 0x0f0u)
#define S5_ECHO_EDGE_BITS      (S5_ECHO_EDGES << (S5_PIN_ECHO * 4u))

/* No sub-byte ISR stores. Timestamp wraps at 65536 us, beyond the
 * 30000-us capture window. Width needs 15 bits but uses an aligned
 * halfword per the tribunal ISR rule. Spinlock guards the whole handoff. */
LOCAL T_SPLOCK echo_lock;
LOCAL UH echo_rise_us, echo_width_us;
LOCAL UB echo_phase;                    /* 0 idle, 1 await rise, 2 await fall */
LOCAL BOOL driver_ready;

/* TEMPORARY bring-up aid (integration): once per 16 pings (~1 s) print
   what the echo line did, so "no echo" can be told apart:
     high = ECHO already high before the trigger (stuck high / wiring)
     norise = no rising edge after the trigger (sensor not answering:
              no power, TRIG/ECHO swapped, wrong Grove port)
     nofall = rose but never fell inside 30 ms (nothing in range)
     ok = complete echoes; w = last echo width in us (58 us per cm)
   Set S5_DEBUG_PING to 0 once the sensor works. */
#ifndef S5_DEBUG_PING
#define S5_DEBUG_PING		1
#endif
#if S5_DEBUG_PING
LOCAL UH dbg_pings, dbg_high, dbg_norise, dbg_nofall, dbg_ok, dbg_w;

LOCAL void dbg_report(void)
{
	if (++dbg_pings < 16) return;
	tm_printf((UB*)"s5: pings=%d ok=%d norise=%d nofall=%d high=%d w=%dus\n",
		  dbg_pings, dbg_ok, dbg_norise, dbg_nofall, dbg_high, dbg_w);
	dbg_pings = dbg_high = dbg_norise = dbg_nofall = dbg_ok = 0;
}
#endif

LOCAL void echo_callback(UINT pin, UINT events)
{
	UINT mask;
	UH now = (UH)in_w(TIMER_TIMERAWL);
	BOOL complete = FALSE;
	(void)pin;
	ISpinLock(&echo_lock, &mask);
	if (echo_phase != 0 && (events & S5_ECHO_EDGES) == S5_ECHO_EDGES) {
		echo_width_us = 0;                /* both pending: timestamps lost */
		echo_phase = 0;
		complete = TRUE;
	} else if (echo_phase == 1 && (events & GPIO_IRQ_EDGE_RISE)) {
		echo_rise_us = now;
		echo_phase = 2;
	} else if (echo_phase == 2 && (events & GPIO_IRQ_EDGE_FALL)) {
		echo_width_us = (UH)(now - echo_rise_us);
		echo_phase = 0;
		complete = TRUE;
	}
	ISpinUnlock(&echo_lock, mask);
	if (complete) tk_sig_sem(sem_echo, 1);
}

EXPORT ER drv_us_init(void)
{
	ER er;
	if (driver_ready) return E_OK;
	er = InitSpinLock(&echo_lock);
	if (er < E_OK) return er;
	er = gpio_set_pin(S5_PIN_TRIG, GPIO_MODE_OUT);
	if (er < E_OK) return er;
	gpio_set_val(S5_PIN_TRIG, 0);
	er = gpio_set_pin(S5_PIN_ECHO, GPIO_MODE_IN);
	if (er < E_OK) return er;
	clr_w(GPIO(S5_PIN_ECHO), GPIO_PUE | GPIO_PDE);
	set_w(GPIO(S5_PIN_ECHO), GPIO_PDE);
	/* Only slice 6, never motor slices 4/5. */
	pwm_set_enabled(S5_PIN_SERVO, FALSE);
	er = pwm_set_pin(S5_PIN_SERVO);
	if (er < E_OK) return er;
	out_w(S5_PWM_REG(PWM_CHx_CSR), 0);
	out_w(S5_PWM_REG(PWM_CHx_DIV), S5_PWM_DIV << 4);
	out_w(S5_PWM_REG(PWM_CHx_CTR), 0);
	pwm_set_wrap(S5_PIN_SERVO, S5_PWM_WRAP);
	pwm_set_cc(S5_PIN_SERVO, s5_servo_pulse_us(S5_CENTRE));
	pwm_set_enabled(S5_PIN_SERVO, TRUE);
	er = gpio_irq_init();               /* idempotent; normally already S2 */
	if (er < E_OK) return er;
	er = gpio_irq_attach(S5_PIN_ECHO, S5_ECHO_EDGES, echo_callback);
	if (er < E_OK) return er;
	driver_ready = TRUE;
	return E_OK;
}

EXPORT ER drv_us_point(UINT index)
{
	if (!driver_ready) return E_OBJ;
	if (index >= S5_ANGLES) return E_PAR;
	return pwm_set_cc(S5_PIN_SERVO, s5_servo_pulse_us(index));
}

EXPORT UB drv_us_ping(void)
{
	UINT mask, pulse_mask;
	UW start, pulse;
	UH width;
	ER er;
	if (!driver_ready) return S5_RANGE_NO_ECHO;
	/* Task and GPIO dispatcher run on core 0: late ISR signals cannot race
 * the drain/arm sequence. Spinlock additionally guards cross-core reads. */
	while (tk_wai_sem(sem_echo, 1, TMO_POL) == E_OK) { }
	ISpinLock(&echo_lock, &mask);
	echo_phase = 0;
	echo_width_us = 0;
	out_w(S5_ECHO_INTR, S5_ECHO_EDGE_BITS);
	if (gpio_get_val(S5_PIN_ECHO)) {
		ISpinUnlock(&echo_lock, mask);
#if S5_DEBUG_PING
		dbg_high++; dbg_report();
#endif
		return S5_RANGE_NO_ECHO;         /* stuck/old high is not a new echo */
	}
	start = in_w(TIMER_TIMERAWL);
	echo_phase = 1;
	ISpinUnlock(&echo_lock, mask);
	/* Tick is 10 ms: use the hardware timer for this short 10-us pulse. */
	DI(pulse_mask);
	gpio_set_val(S5_PIN_TRIG, 1);
	pulse = in_w(TIMER_TIMERAWL);
	while ((UW)(in_w(TIMER_TIMERAWL) - pulse) < 10u) { }
	gpio_set_val(S5_PIN_TRIG, 0);
	EI(pulse_mask);
	er = tk_wai_sem(sem_echo, 1, 30);
	ISpinLock(&echo_lock, &mask);
	width = echo_width_us;
#if S5_DEBUG_PING
	if (er == E_OK) { dbg_ok++; dbg_w = width; }
	else if (echo_phase == 1) dbg_norise++;
	else dbg_nofall++;
#endif
	echo_phase = 0;
	ISpinUnlock(&echo_lock, mask);
#if S5_DEBUG_PING
	dbg_report();
#endif
	/* Timeout rounding may return late: enforce the physical time window. */
	if (er != E_OK || (UW)(in_w(TIMER_TIMERAWL) - start) > S5_ECHO_TIMEOUT_US)
		return S5_RANGE_NO_ECHO;
	return s5_range_from_echo(width);
}
