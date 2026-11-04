/*
 *	drv_lsm303.c
 *	Subsystem 4 -- LSM303DLHC / LSM303AGR / LSM303D driver on RP2040 I2C1
 *	(GP26/GP27).  lsm303_init() finds which chip the board carries.
 *
 *	APIs shown: in_w / out_w / set_w / clr_w (RP2040 registers)
 *
 *	The I2C1 block is driven by polling, the same way the Pico SDK's
 *	i2c_write_blocking / i2c_read_blocking do it (RP2040 datasheet 4.3),
 *	with a time-out on the whole transfer.  One 10 ms cycle reads
 *	2 x (1 + 6) bytes, about 0.4 ms of bus time at 400 kHz.
 *
 *	Only the IMU task calls this file (single owner of I2C1), so no lock
 *	is needed, and since there is no interrupt it works on either core.
 */

#include "drv_lsm303.h"
#include <tk/syslib.h>

/* RP2040 I2C1 registers (datasheet 4.3.17) */
#define IC_BASE			0x40048000	/* I2C1 */
#define IC_CON			(IC_BASE + 0x00)
#define IC_TAR			(IC_BASE + 0x04)
#define IC_DATA_CMD		(IC_BASE + 0x10)
#define IC_FS_SCL_HCNT		(IC_BASE + 0x1C)
#define IC_FS_SCL_LCNT		(IC_BASE + 0x20)
#define IC_INTR_MASK		(IC_BASE + 0x30)
#define IC_RAW_INTR_STAT	(IC_BASE + 0x34)
#define IC_RX_TL		(IC_BASE + 0x38)
#define IC_TX_TL		(IC_BASE + 0x3C)
#define IC_CLR_INTR		(IC_BASE + 0x40)
#define IC_CLR_TX_ABRT		(IC_BASE + 0x54)
#define IC_CLR_STOP_DET		(IC_BASE + 0x60)
#define IC_ENABLE		(IC_BASE + 0x6C)
#define IC_TXFLR		(IC_BASE + 0x74)
#define IC_RXFLR		(IC_BASE + 0x78)
#define IC_SDA_HOLD		(IC_BASE + 0x7C)
#define IC_TX_ABRT_SOURCE	(IC_BASE + 0x80)
#define IC_DMA_CR		(IC_BASE + 0x88)
#define IC_ENABLE_STATUS	(IC_BASE + 0x9C)
#define IC_FS_SPKLEN		(IC_BASE + 0xA0)

#define CON_MASTER		(1u << 0)
#define CON_SPEED_FAST		(2u << 1)
#define CON_RESTART_EN		(1u << 5)
#define CON_SLAVE_DISABLE	(1u << 6)
#define CON_TX_EMPTY_CTRL	(1u << 8)

#define CMD_READ		(1u << 8)
#define CMD_STOP		(1u << 9)
#define CMD_RESTART		(1u << 10)

#define INTR_TX_EMPTY		(1u << 4)
#define INTR_STOP_DET		(1u << 9)

#define ENABLE_ENABLE		(1u << 0)
#define ENABLE_ABORT		(1u << 1)

#define IC_TX_FIFO_DEPTH	16

/* 400 kHz from clk_sys = 125 MHz, as the Pico SDK computes it:
   period 313 clocks, low 3/5, high the rest; spike filter low/16;
   SDA hold 3 x 125 MHz / 10 MHz + 1 */
#define IC_FS_HCNT_400K		126
#define IC_FS_LCNT_400K		187
#define IC_FS_SPKLEN_400K	11
#define IC_SDA_HOLD_400K	38

/* LSM303DLHC registers */
#define ACC_CTRL_REG1_A		0x20
#define ACC_CTRL_REG4_A		0x23
#define ACC_OUT_X_L_A		0x28
#define ACC_AUTO_INC		0x80	/* MSB of the sub-address */
#define MAG_CRA_REG_M		0x00
#define MAG_CRB_REG_M		0x01
#define MAG_MR_REG_M		0x02
#define MAG_OUT_X_H_M		0x03
#define MAG_IRA_REG_M		0x0A
#define MAG_IRA_VALUE		0x48	/* 'H' */

/* LSM303AGR (many newer GY-511 boards): same addresses, the accelerometer
   registers used here are the same (CTRL_REG1_A, CTRL_REG4_A, OUT_X_L_A),
   but the magnetometer is different (datasheet DocID027765) */
#define ACC_WHO_AM_I_A		0x0F	/* 0x33 on DLHC and AGR */
#define AGR_WHO_AM_I_M		0x4F
#define AGR_WHO_AM_I_VALUE	0x40
#define AGR_CFG_REG_A_M		0x60	/* temp comp, ODR, mode */
#define AGR_CFG_REG_C_M		0x62	/* BDU */
#define AGR_OUTX_L_REG_M	0x68	/* X, Y, Z, little endian, auto-increment */

/* LSM303D (Grove 6-Axis Accelerometer&Compass V2): accel AND mag in one
   chip at ONE address, 0x1E (SA0 high) or 0x1D; WHO_AM_I (0x0F) = 0x49.
   Nothing answers at 0x19 on this board. */
#define D_ADDR_HI		0x1E
#define D_ADDR_LO		0x1D
#define D_WHO_AM_I		0x0F
#define D_WHO_AM_I_VALUE	0x49
#define D_OUT_X_L_M		0x08	/* X, Y, Z, little endian */
#define D_CTRL1			0x20
#define D_CTRL2			0x21
#define D_CTRL5			0x24
#define D_CTRL6			0x25
#define D_CTRL7			0x26
#define D_OUT_X_L_A		0x28	/* X, Y, Z, little endian, 16 bits */

LOCAL UB	chip;			/* LSM303_CHIP_xxx, IMU task only */
LOCAL UB	d_addr;			/* LSM303D: its one address */
LOCAL UB	last_ira, last_agr_id, last_acc_id;	/* for the fault log */
LOCAL UB	last_d_id;		/* 0x1E reg 0x0F (LSM303D = 0x49) */

LOCAL UH	i2c_errors;		/* IMU task only */

LOCAL UW now_us(void)
{
	return in_w(TIMER_TIMERAWL);	/* 1 us free-running counter */
}

/* TRUE once more than S4_I2C_TIMEOUT_US have passed since t0 */
LOCAL BOOL timed_out(UW t0)
{
	return (now_us() - t0) > S4_I2C_TIMEOUT_US;
}

/*
 * One transfer: write wn bytes, then (repeated start) read rn bytes,
 * then STOP.  FALSE on NACK (no device / wrong address) or time-out.
 */
LOCAL BOOL i2c_xfer(UB addr, const UB *wr, INT wn, UB *rd, INT rn)
{
	UW	t0 = now_us();
	UW	cmd;
	INT	i;
	BOOL	ok = TRUE;

	/* The target address can only change while the block is off */
	out_w(IC_ENABLE, 0);
	while ((in_w(IC_ENABLE_STATUS) & 1u) != 0) {
		if (timed_out(t0)) { ok = FALSE; break; }
	}
	if (ok) {
		out_w(IC_TAR, addr);
		out_w(IC_ENABLE, ENABLE_ENABLE);
		(void)in_w(IC_CLR_INTR);	/* drop stale flags */
	}

	/* Write: wait for each byte to leave the shift register.  A NACK
	   aborts the transfer, flushes the FIFO and sends STOP itself. */
	for (i = 0; ok && i < wn; i++) {
		cmd = wr[i];
		if (i == wn - 1 && rn == 0) cmd |= CMD_STOP;
		out_w(IC_DATA_CMD, cmd);
		while ((in_w(IC_RAW_INTR_STAT) & INTR_TX_EMPTY) == 0) {
			if (timed_out(t0)) { ok = FALSE; break; }
		}
		if (ok && in_w(IC_TX_ABRT_SOURCE) != 0) {
			(void)in_w(IC_CLR_TX_ABRT);
			ok = FALSE;
		}
	}

	/* Read: one READ command per byte, RESTART on the first, STOP on
	   the last */
	for (i = 0; ok && i < rn; i++) {
		cmd = CMD_READ;
		if (i == 0 && wn > 0) cmd |= CMD_RESTART;
		if (i == rn - 1)      cmd |= CMD_STOP;
		while (in_w(IC_TXFLR) >= IC_TX_FIFO_DEPTH) {
			if (timed_out(t0)) { ok = FALSE; break; }
		}
		if (!ok) break;
		out_w(IC_DATA_CMD, cmd);
		while (in_w(IC_RXFLR) == 0) {
			if (in_w(IC_CLR_TX_ABRT) & 1u) { ok = FALSE; break; }
			if (timed_out(t0)) { ok = FALSE; break; }
		}
		if (ok) rd[i] = (UB)(in_w(IC_DATA_CMD) & 0xFFu);
	}

	/* Let the STOP finish so the next transfer starts on an idle bus */
	if (ok) {
		while ((in_w(IC_RAW_INTR_STAT) & INTR_STOP_DET) == 0) {
			if (timed_out(t0)) { ok = FALSE; break; }
		}
		(void)in_w(IC_CLR_STOP_DET);
	}

	if (!ok) {
		/* Stop whatever the block is doing; the next call starts clean */
		out_w(IC_ENABLE, ENABLE_ENABLE | ENABLE_ABORT);
		t0 = now_us();
		while ((in_w(IC_ENABLE) & ENABLE_ABORT) != 0 && !timed_out(t0)) ;
		out_w(IC_ENABLE, 0);
		i2c_errors++;
	}
	return ok;
}

LOCAL BOOL write_reg(UB addr, UB reg, UB val)
{
	UB	b[2];

	b[0] = reg;
	b[1] = val;
	return i2c_xfer(addr, b, 2, NULL, 0);
}

LOCAL BOOL read_regs(UB addr, UB reg, UB *buf, INT n)
{
	return i2c_xfer(addr, &reg, 1, buf, n);
}

/* ------------------------------------------------------------------ */
/*  Bus recovery.  If the Pico resets (or is reflashed) in the middle  */
/*  of a read while the sensor stays powered, the sensor can hold SDA  */
/*  low, waiting for clocks that never come: every transfer then fails */
/*  until its power is cut.  Standard fix (I2C spec 3.1.16): with the  */
/*  pins as plain GPIO, clock SCL up to 9 times until SDA is released, */
/*  then send a STOP.  Open drain is emulated: OUT stays 0, OE = 1     */
/*  pulls the line low, OE = 0 lets the pull-up take it high.          */
/* ------------------------------------------------------------------ */

#define BUS_HALF_US		5	/* 100 kHz while clocking the bus free */
#define PIN_BIT(pin)		(1u << (pin))

LOCAL UH	bus_recoveries;		/* IMU task only */

LOCAL void wait_us(UW us)
{
	UW	t0 = now_us();

	while ((now_us() - t0) < us) ;
}

LOCAL void line_low(UINT pin)     { out_w(GPIO_OE_SET, PIN_BIT(pin)); }
LOCAL void line_release(UINT pin) { out_w(GPIO_OE_CLR, PIN_BIT(pin)); }
LOCAL UINT line_level(UINT pin)   { return (in_w(GPIO_IN) >> pin) & 1u; }

LOCAL void bus_recover(void)
{
	INT	i;

	/* Both lines released, as SIO inputs with pull-ups */
	out_w(GPIO_OE_CLR, PIN_BIT(S4_PIN_IMU_SDA) | PIN_BIT(S4_PIN_IMU_SCL));
	out_w(GPIO_OUT_CLR, PIN_BIT(S4_PIN_IMU_SDA) | PIN_BIT(S4_PIN_IMU_SCL));
	out_w(GPIO(S4_PIN_IMU_SDA), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
	out_w(GPIO(S4_PIN_IMU_SCL), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
	out_w(GPIO_CTRL(S4_PIN_IMU_SDA), GPIO_CTRL_FUNCSEL_SIO);
	out_w(GPIO_CTRL(S4_PIN_IMU_SCL), GPIO_CTRL_FUNCSEL_SIO);
	wait_us(BUS_HALF_US);

	if (line_level(S4_PIN_IMU_SDA)) return;	/* bus idle: nothing to do */
	bus_recoveries++;

	for (i = 0; i < 9 && !line_level(S4_PIN_IMU_SDA); i++) {
		line_low(S4_PIN_IMU_SCL);
		wait_us(BUS_HALF_US);
		line_release(S4_PIN_IMU_SCL);
		wait_us(BUS_HALF_US);
	}
	/* STOP: SDA rises while SCL is high */
	line_low(S4_PIN_IMU_SCL);
	wait_us(BUS_HALF_US);
	line_low(S4_PIN_IMU_SDA);
	wait_us(BUS_HALF_US);
	line_release(S4_PIN_IMU_SCL);
	wait_us(BUS_HALF_US);
	line_release(S4_PIN_IMU_SDA);
	wait_us(BUS_HALF_US);
}

EXPORT UH lsm303_bus_recoveries(void)
{
	return bus_recoveries;
}

EXPORT void lsm303_bus_init(void)
{
	UW	t0;

	/* Free a bus left stuck by an earlier reset, before I2C1 takes the
	   pins (the pins switch back to I2C1 at the end of this function) */
	bus_recover();

	/* Pulse the I2C1 reset (atomic set/clear aliases: RESETS is shared
	   with every other peripheral) and wait until it is out of reset */
	set_w(RESETS_RESET, RESETS_RESET_I2C1);
	clr_w(RESETS_RESET, RESETS_RESET_I2C1);
	t0 = now_us();
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C1) == 0 && !timed_out(t0)) ;

	out_w(IC_ENABLE, 0);
	out_w(IC_CON, CON_SLAVE_DISABLE | CON_MASTER | CON_RESTART_EN |
		      CON_TX_EMPTY_CTRL | CON_SPEED_FAST);
	out_w(IC_TX_TL, 0);
	out_w(IC_RX_TL, 0);
	out_w(IC_DMA_CR, 0);
	out_w(IC_INTR_MASK, 0);			/* polled, no interrupt */
	out_w(IC_FS_SCL_HCNT, IC_FS_HCNT_400K);
	out_w(IC_FS_SCL_LCNT, IC_FS_LCNT_400K);
	out_w(IC_FS_SPKLEN, IC_FS_SPKLEN_400K);
	out_w(IC_SDA_HOLD, IC_SDA_HOLD_400K);

	/* GP26 / GP27 -> I2C1 SDA / SCL, input enabled, pull-ups on (the
	   GY-511 board has its own pull-ups too) */
	out_w(GPIO_CTRL(S4_PIN_IMU_SDA), GPIO_CTRL_FUNCSEL_I2C);
	out_w(GPIO(S4_PIN_IMU_SDA), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
	out_w(GPIO_CTRL(S4_PIN_IMU_SCL), GPIO_CTRL_FUNCSEL_I2C);
	out_w(GPIO(S4_PIN_IMU_SCL), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
}

EXPORT BOOL lsm303_probe(UB addr)
{
	UB	d;

	return i2c_xfer(addr, NULL, 0, &d, 1);
}

/*
 * Find out which chip is fitted, then configure it.  The accelerometer
 * (pitch, humps, shock) is set up the same way on both chips:
 *   CTRL_REG1_A = 0x57  100 Hz, X/Y/Z on
 *   CTRL_REG4_A = 0x98  BDU, +-4 g, high resolution (12 bits, ~2 mg/LSB)
 * If no known magnetometer answers but the accelerometer does, run
 * accelerometer-only: everything works except heading / turn rate.
 */
/* LSM303D at addr: check WHO_AM_I, then accel 100 Hz +-4 g, mag 50 Hz */
LOCAL BOOL init_d(UB addr)
{
	UB	v = 0;

	if (!read_regs(addr, D_WHO_AM_I, &v, 1)) return FALSE;
	if (addr == D_ADDR_HI) last_d_id = v;
	if (v != D_WHO_AM_I_VALUE) return FALSE;
	if (!(write_reg(addr, D_CTRL1, 0x6F) &&	/* accel 100 Hz, BDU, XYZ on */
	      write_reg(addr, D_CTRL2, 0x08) &&	/* +-4 g (0.122 mg/LSB) */
	      write_reg(addr, D_CTRL5, 0x70) &&	/* mag high res, 50 Hz */
	      write_reg(addr, D_CTRL6, 0x20) &&	/* mag +-4 gauss */
	      write_reg(addr, D_CTRL7, 0x00))) {	/* mag continuous */
		return FALSE;
	}
	d_addr = addr;
	chip = LSM303_CHIP_D;
	return TRUE;
}

EXPORT BOOL lsm303_init(void)
{
	UB	v;

	chip = LSM303_CHIP_NONE;
	last_ira = last_agr_id = last_acc_id = last_d_id = 0;
	lsm303_bus_init();

	/* DLHC / AGR: the accelerometer answers at 0x19.  If nothing does,
	   it may be an LSM303D (one chip at 0x1E or 0x1D). */
	if (!read_regs(LSM303_ACC_ADDR, ACC_WHO_AM_I_A, &v, 1)) {
		return init_d(D_ADDR_HI) || init_d(D_ADDR_LO);
	}
	last_acc_id = v;
	if (!write_reg(LSM303_ACC_ADDR, ACC_CTRL_REG1_A, 0x57) ||
	    !write_reg(LSM303_ACC_ADDR, ACC_CTRL_REG4_A, 0x98)) return FALSE;

	/* Which magnetometer?  DLHC: IRA_REG_M = 'H'.  AGR: WHO_AM_I_M = 0x40 */
	if (read_regs(LSM303_MAG_ADDR, MAG_IRA_REG_M, &v, 1)) {
		last_ira = v;
		if (v == MAG_IRA_VALUE &&
		    write_reg(LSM303_MAG_ADDR, MAG_CRA_REG_M, 0x18) &&	/* 75 Hz */
		    write_reg(LSM303_MAG_ADDR, MAG_CRB_REG_M, 0x20) &&	/* +-1.3 G */
		    write_reg(LSM303_MAG_ADDR, MAG_MR_REG_M, 0x00)) {	/* continuous */
			chip = LSM303_CHIP_DLHC;
			return TRUE;
		}
	}
	if (read_regs(LSM303_MAG_ADDR, AGR_WHO_AM_I_M, &v, 1)) {
		last_agr_id = v;
		if (v == AGR_WHO_AM_I_VALUE &&
		    write_reg(LSM303_MAG_ADDR, AGR_CFG_REG_A_M, 0x8C) &&	/* temp comp, 100 Hz, continuous */
		    write_reg(LSM303_MAG_ADDR, AGR_CFG_REG_C_M, 0x10)) {	/* BDU */
			chip = LSM303_CHIP_AGR;
			return TRUE;
		}
	}
	chip = LSM303_CHIP_ACC_ONLY;
	return TRUE;
}

EXPORT UINT lsm303_chip(void)
{
	return chip;
}

EXPORT const char *lsm303_chip_name(void)
{
	switch (chip) {
	case LSM303_CHIP_DLHC:		return "LSM303DLHC";
	case LSM303_CHIP_AGR:		return "LSM303AGR";
	case LSM303_CHIP_D:		return "LSM303D";
	case LSM303_CHIP_ACC_ONLY:	return "accelerometer only (no known magnetometer, heading off)";
	default:			return "none";
	}
}

EXPORT void lsm303_last_ids(UB *acc_id, UB *ira, UB *agr_id, UB *d_id)
{
	*acc_id = last_acc_id;
	*ira    = last_ira;
	*agr_id = last_agr_id;
	*d_id   = last_d_id;
}

/* LSM303D accel, 16 bits at +-4 g = 0.122 mg/LSB -> mg, rounded */
LOCAL H d_acc_mg(UB lo, UB hi)
{
	W	raw = (H)(UH)(lo | (hi << 8));

	return (H)((raw * 122 + (raw >= 0 ? 500 : -500)) / 1000);
}

EXPORT void lsm303_read(IMU_SAMPLE *s)
{
	UB	a[6], m[6];
	H	sx, sy, sz, mx, my, mz;

	if (chip == LSM303_CHIP_D) {
		s->ok = read_regs(d_addr, D_OUT_X_L_A | ACC_AUTO_INC, a, 6) &&
			read_regs(d_addr, D_OUT_X_L_M | ACC_AUTO_INC, m, 6);
		if (!s->ok) return;
		sx = d_acc_mg(a[0], a[1]);
		sy = d_acc_mg(a[2], a[3]);
		sz = d_acc_mg(a[4], a[5]);
		mx = (H)(UH)(m[0] | (m[1] << 8));
		my = (H)(UH)(m[2] | (m[3] << 8));
		mz = (H)(UH)(m[4] | (m[5] << 8));
		goto robot_axes;
	}

	s->ok = (chip != LSM303_CHIP_NONE) &&
		read_regs(LSM303_ACC_ADDR, ACC_OUT_X_L_A | ACC_AUTO_INC, a, 6);
	if (s->ok && chip == LSM303_CHIP_DLHC) {
		s->ok = read_regs(LSM303_MAG_ADDR, MAG_OUT_X_H_M, m, 6);
	} else if (s->ok && chip == LSM303_CHIP_AGR) {
		s->ok = read_regs(LSM303_MAG_ADDR, AGR_OUTX_L_REG_M, m, 6);
	}
	if (!s->ok) return;

	/* accel: little endian, 12 bits left-justified -> >> 4, 2 mg/LSB */
	sx = (H)(((H)(UH)(a[0] | (a[1] << 8)) >> 4) * 2);
	sy = (H)(((H)(UH)(a[2] | (a[3] << 8)) >> 4) * 2);
	sz = (H)(((H)(UH)(a[4] | (a[5] << 8)) >> 4) * 2);

	/* mag: raw counts are enough for heading */
	if (chip == LSM303_CHIP_DLHC) {		/* big endian, order X, Z, Y */
		mx = (H)(UH)((m[0] << 8) | m[1]);
		mz = (H)(UH)((m[2] << 8) | m[3]);
		my = (H)(UH)((m[4] << 8) | m[5]);
	} else if (chip == LSM303_CHIP_AGR) {	/* little endian, order X, Y, Z */
		mx = (H)(UH)(m[0] | (m[1] << 8));
		my = (H)(UH)(m[2] | (m[3] << 8));
		mz = (H)(UH)(m[4] | (m[5] << 8));
	} else {				/* accelerometer only: heading 0 */
		mx = my = mz = 0;
	}

robot_axes:
	s->ax = LSM303_ROBOT_X(sx, sy, sz);
	s->ay = LSM303_ROBOT_Y(sx, sy, sz);
	s->az = LSM303_ROBOT_Z(sx, sy, sz);
	s->mx = LSM303_ROBOT_X(mx, my, mz);
	s->my = LSM303_ROBOT_Y(mx, my, mz);
	s->mz = LSM303_ROBOT_Z(mx, my, mz);
}

EXPORT UH lsm303_error_count(void)
{
	return i2c_errors;
}

/* ------------------------------------------------------------------ */
/*  Bring-up diagnostics (added at integration): printed once when the */
/*  sensor does not answer, to tell wiring from the wrong chip.        */
/* ------------------------------------------------------------------ */

/* Idle bus levels after lsm303_bus_init(): bit 0 SDA, bit 1 SCL.
   Both must read 1 (pull-ups); 0 = no power / shorted / swapped cable. */
EXPORT UINT lsm303_line_levels(void)
{
	UW	in = in_w(GPIO_IN);

	return ((in >> S4_PIN_IMU_SDA) & 1u) | (((in >> S4_PIN_IMU_SCL) & 1u) << 1);
}

/* Probe 0x08..0x77; writes up to max answering addresses, returns count */
EXPORT INT lsm303_scan(UB *found, INT max)
{
	UB	a;
	INT	n = 0;

	for (a = 0x08; a <= 0x77; a++) {
		if (lsm303_probe(a)) {
			if (n < max) found[n] = a;
			n++;
		}
	}
	return n;
}

/* One register read for the diagnostics.  TRUE = the device answered */
EXPORT BOOL lsm303_read_reg(UB addr, UB reg, UB *val)
{
	return read_regs(addr, reg, val, 1);
}
