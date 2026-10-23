/*
 *	drv_lsm303.c
 *	Subsystem 4 -- LSM303DLHC driver on RP2040 I2C1 (GP26/GP27).
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

EXPORT void lsm303_bus_init(void)
{
	UW	t0;

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

EXPORT BOOL lsm303_init(void)
{
	UB	id = 0;

	lsm303_bus_init();
	if (!read_regs(LSM303_MAG_ADDR, MAG_IRA_REG_M, &id, 1)) return FALSE;
	if (id != MAG_IRA_VALUE) return FALSE;	/* not an LSM303DLHC */

	return write_reg(LSM303_ACC_ADDR, ACC_CTRL_REG1_A, 0x57) &&
	       write_reg(LSM303_ACC_ADDR, ACC_CTRL_REG4_A, 0x98) &&
	       write_reg(LSM303_MAG_ADDR, MAG_CRA_REG_M, 0x18) &&
	       write_reg(LSM303_MAG_ADDR, MAG_CRB_REG_M, 0x20) &&
	       write_reg(LSM303_MAG_ADDR, MAG_MR_REG_M, 0x00);
}

EXPORT void lsm303_read(IMU_SAMPLE *s)
{
	UB	a[6], m[6];
	H	sx, sy, sz, mx, my, mz;

	s->ok = read_regs(LSM303_ACC_ADDR, ACC_OUT_X_L_A | ACC_AUTO_INC, a, 6) &&
		read_regs(LSM303_MAG_ADDR, MAG_OUT_X_H_M, m, 6);
	if (!s->ok) return;

	/* accel: little endian, 12 bits left-justified -> >> 4, 2 mg/LSB */
	sx = (H)(((H)(UH)(a[0] | (a[1] << 8)) >> 4) * 2);
	sy = (H)(((H)(UH)(a[2] | (a[3] << 8)) >> 4) * 2);
	sz = (H)(((H)(UH)(a[4] | (a[5] << 8)) >> 4) * 2);

	/* mag: big endian, order X, Z, Y; raw counts are enough for heading */
	mx = (H)(UH)((m[0] << 8) | m[1]);
	mz = (H)(UH)((m[2] << 8) | m[3]);
	my = (H)(UH)((m[4] << 8) | m[5]);

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
