/*
 *	drv_lsm303.h
 *	Subsystem 4 -- LSM303DLHC driver (accelerometer + magnetometer, NO
 *	gyroscope), private to subsystem 4.  Called only by the IMU task.
 *
 *	Robo Pico Grove 6: I2C1, SDA = GP26, SCL = GP27, 400 kHz.
 *	Uses the RP2040 I2C1 registers directly (polling, bounded time-out),
 *	NOT the kernel's "iic" driver, so:
 *	  - no shared config or kernel file changes (DEVCNF_USE_IIC stays 0,
 *	    so the I2C0 default pins GP8/GP9 -- the right motor -- are never
 *	    claimed);
 *	  - a missing or stuck sensor costs at most S4_I2C_TIMEOUT_US per
 *	    transfer, not the kernel driver's 1 s time-out.
 *
 *	Accelerometer (0x19): CTRL_REG1_A = 0x57 (100 Hz, XYZ on)
 *	                      CTRL_REG4_A = 0x98 (BDU, +-4 g, high resolution)
 *	                      12-bit left-justified, 2 mg/LSB
 *	Magnetometer (0x1E):  CRA_REG_M = 0x18 (75 Hz), CRB_REG_M = 0x20
 *	                      (+-1.3 gauss), MR_REG_M = 0x00 (continuous);
 *	                      data X, Z, Y, high byte first.
 *	                      IRA_REG_M (0x0A) reads 0x48 ('H') on this chip.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include "subsystem4.h"
#include "imu_monitor.h"

#define LSM303_ACC_ADDR		0x19
#define LSM303_MAG_ADDR		0x1E

#define S4_I2C_TIMEOUT_US	2000	/* one whole transfer, worst case */

/* Sensor axes -> robot axes (X right, Y forward, Z up).  Change these if
   the board is mounted turned.  Check (S4-M02): robot flat -> az about
   +1000 mg; nose lifted -> ay positive. */
#define LSM303_ROBOT_X(sx, sy, sz)	(sx)
#define LSM303_ROBOT_Y(sx, sy, sz)	(sy)
#define LSM303_ROBOT_Z(sx, sy, sz)	(sz)

/* Reset I2C1 and claim GP26/GP27.  Safe to call again to recover a
   stuck bus. */
IMPORT void	lsm303_bus_init(void);

/* TRUE if a device answers at addr (bring-up scan) */
IMPORT BOOL	lsm303_probe(UB addr);

/* Bus init, find the chip, configure it.  TRUE = the accelerometer works
   (with or without a known magnetometer, see lsm303_chip()). */
IMPORT BOOL	lsm303_init(void);

/* Which chip lsm303_init() found */
#define LSM303_CHIP_NONE	0
#define LSM303_CHIP_DLHC	1	/* LSM303DLHC: accel + mag */
#define LSM303_CHIP_AGR		2	/* LSM303AGR:  accel + mag (other registers) */
#define LSM303_CHIP_ACC_ONLY	3	/* accel works, magnetometer unknown */
IMPORT UINT	lsm303_chip(void);
IMPORT const char *lsm303_chip_name(void);

/* IDs read by the last lsm303_init() (0 = no answer), for the fault log:
   accel WHO_AM_I_A (0x0F), DLHC IRA_REG_M (0x0A), AGR WHO_AM_I_M (0x4F) */
IMPORT void	lsm303_last_ids(UB *acc_id, UB *ira, UB *agr_id);

/* Read both sensors (robot axes).  s->ok = FALSE on any I2C error. */
IMPORT void	lsm303_read(IMU_SAMPLE *s);

/* Bring-up diagnostics: idle line levels (bit 0 SDA, bit 1 SCL), bus
   scan, single register read */
IMPORT UINT	lsm303_line_levels(void);
IMPORT INT	lsm303_scan(UB *found, INT max);
IMPORT BOOL	lsm303_read_reg(UB addr, UB reg, UB *val);

/* Diagnostics: transfers that failed (NACK / time-out), wraps */
IMPORT UH	lsm303_error_count(void);

/* Diagnostics: times SDA was found stuck low and clocked free (wraps) */
IMPORT UH	lsm303_bus_recoveries(void);

#endif /* DRV_LSM303_H */
