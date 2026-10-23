/*
 *	motor_driver.h
 *	Robo Pico motor driver, private to subsystem 2.
 *	ONLY motion_control.c calls these (it is the only writer of GP8..11).
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "subsystem2.h"

#define MOTOR_LEFT		0
#define MOTOR_RIGHT		1
#define MOTOR_DUTY_MAX		1000	/* duty in per mille (10 bits) */

IMPORT void	motor_init(void);		/* PWM 20 kHz, both braked */
IMPORT void	motor_set(UINT wheel, INT duty_pm);	/* + fwd, - rev, 0 brake */
IMPORT void	motor_brake_all(void);
IMPORT BOOL	motor_is_braked(void);	/* TRUE if all four compare values are 0 */

#endif /* MOTOR_DRIVER_H */
