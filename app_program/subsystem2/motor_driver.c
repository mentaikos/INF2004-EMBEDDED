/*
 *	motor_driver.c
 *	Robo Pico motor driver (two DC motors), private to subsystem 2.
 *
 *	APIs shown: pwm_set_pin, pwm_set_wrap, pwm_set_cc, pwm_set_enabled
 *
 *	Robo Pico truth table, per motor (A, B):
 *	    L L  brake        H L  forward
 *	    H H  coast        L H  backward
 *	All four pins are PWM outputs:
 *	    forward  = PWM on A, B = 0
 *	    reverse  = PWM on B, A = 0
 *	    brake    = A = B = 0 (compare value 0 -> output always LOW)
 *	Both pins of one motor share one PWM slice (right = slice 4,
 *	left = slice 5), so A and B are never both driven with PWM.
 *	20 kHz: clock divider 1 (reset value), wrap 6249.
 */

#include "motor_driver.h"
#include <tk/syslib.h>

#define PWM_SLICE(pin)		(((pin) >> 1) & 7u)
#define PWM_CC_ADDR(pin)	(PWM_BASE + PWM_CHx_CC + PWM_SLICE(pin) * 0x14)

LOCAL const UB motor_pin_a[2] = { S2_PIN_ML_A, S2_PIN_MR_A };
LOCAL const UB motor_pin_b[2] = { S2_PIN_ML_B, S2_PIN_MR_B };
LOCAL const UB motor_invert[2] = { S2_INVERT_LEFT, S2_INVERT_RIGHT };

LOCAL UW duty_to_cc(INT duty_pm)
{
	if (duty_pm <= 0) return 0;
	if (duty_pm > MOTOR_DUTY_MAX) duty_pm = MOTOR_DUTY_MAX;
	return ((UW)duty_pm * (S2_PWM_WRAP + 1u)) / MOTOR_DUTY_MAX;
}

EXPORT void motor_init(void)
{
	UINT	w;

	for (w = 0; w < 2; w++) {
		/* Compare values to 0 BEFORE the pins switch to PWM, so the
		   motors are braked from the very first cycle. */
		pwm_set_wrap(motor_pin_a[w], S2_PWM_WRAP);
		pwm_set_cc(motor_pin_a[w], 0);
		pwm_set_cc(motor_pin_b[w], 0);
		pwm_set_enabled(motor_pin_a[w], TRUE);
		pwm_set_pin(motor_pin_a[w]);
		pwm_set_pin(motor_pin_b[w]);
	}
}

EXPORT void motor_set(UINT wheel, INT duty_pm)
{
	UW	cc;

	if (wheel > MOTOR_RIGHT) return;
	if (motor_invert[wheel]) duty_pm = -duty_pm;
	cc = duty_to_cc(duty_pm < 0 ? -duty_pm : duty_pm);

	/* Always lower the pin that must be 0 first, so A and B are
	   never high together, even for one PWM period. */
	if (duty_pm > 0) {
		pwm_set_cc(motor_pin_b[wheel], 0);
		pwm_set_cc(motor_pin_a[wheel], cc);
	} else if (duty_pm < 0) {
		pwm_set_cc(motor_pin_a[wheel], 0);
		pwm_set_cc(motor_pin_b[wheel], cc);
	} else {
		pwm_set_cc(motor_pin_a[wheel], 0);
		pwm_set_cc(motor_pin_b[wheel], 0);
	}
}

EXPORT void motor_brake_all(void)
{
	/* One 32-bit write per slice sets A and B to 0 together */
	out_w(PWM_CC_ADDR(S2_PIN_ML_A), 0);
	out_w(PWM_CC_ADDR(S2_PIN_MR_A), 0);
}

EXPORT BOOL motor_is_braked(void)
{
	return in_w(PWM_CC_ADDR(S2_PIN_ML_A)) == 0 &&
	       in_w(PWM_CC_ADDR(S2_PIN_MR_A)) == 0;
}
