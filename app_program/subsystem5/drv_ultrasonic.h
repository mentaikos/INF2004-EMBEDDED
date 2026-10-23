/* Owned by Buddy 5, called only from its core-0 task. */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H
#include "subsystem5.h"
IMPORT ER drv_us_init(void);
IMPORT ER drv_us_point(UINT index);
IMPORT UB drv_us_ping(void);           /* bounded 30 ms semaphore wait */
#endif
