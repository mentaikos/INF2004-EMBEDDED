/*
 *	app.h
 *	Definitions shared by the whole team.  Changed only by team agreement.
 *
 *	Anything that belongs to ONE subsystem goes in that subsystem's own
 *	header instead (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>		/* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>	/* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>		/* Board helpers: gpio_set_pin/val, pwm_... */

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.  Each subsystem
 *  has its own band (TEAM.md section 4), defined in its own header.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE		2048	/* bytes; tm_printf needs ~1 KB */

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* ---------------------------------------------------------------------
 *  Vehicle Controller events (design 7.2: mbf_vc_evt).  Every event is
 *  ONE 32-bit word, sent without blocking:
 *	UW w = EVT_PACK(EVT_xxx, payload);
 *	tk_snd_mbf(mbf_vc_evt, &w, sizeof(w), TMO_POL);
 *	[3:0]  type     EVT_xxx
 *	[31:4] payload  event-specific (28 bits), see the list
 *  Codes are the design's event list in order, as subsystem 2 already
 *  uses them.  The Vehicle Controller's owner creates mbf_vc_evt and
 *  hands its ID to each sender (motion_bind_vc(), s3_bind_vc(), ...).
 * --------------------------------------------------------------------- */
#define EVT_NAV_CMD			0	/* NAV_xxx, 2 bits         (S3) */
#define EVT_JUNCTION			1	/*                         (S3) */
#define EVT_LINE_LOST			2	/*                         (S3) */
#define EVT_LINE_FOUND			3	/*                         (S3) */
#define EVT_HUMP_START			4	/*                         (S4) */
#define EVT_HUMP_END			5	/* peak height, mm         (S4) */
#define EVT_OBSTACLE_DETECTED		6	/*                         (S5) */
#define EVT_OBSTACLE_PROFILE		7	/*                         (S5) */
#define EVT_NO_CLEAR			8	/*                         (S5) */
#define EVT_SENSOR_FAULT		9	/* sender's FAULT_xxx bits      */
#define EVT_REJOIN_LINE_CONFIRMED	10	/*                         (S5) */
#define EVT_START			11	/*                         (S1) */
#define EVT_STOP			12	/*                         (S1) */
#define EVT_MOTION_DONE			13	/* finished command        (S2) */
#define EVT_MISSION_END			14	/*                         (VC) */

#define EVT_TYPE_MASK		0xFu
#define EVT_PAYLOAD_SHIFT	4
#define EVT_PACK(type, payload)	\
	(((UW)(type) & EVT_TYPE_MASK) | ((UW)(payload) << EVT_PAYLOAD_SHIFT))
#define EVT_TYPE(w)		((UINT)((w) & EVT_TYPE_MASK))
#define EVT_PAYLOAD(w)		((UW)(w) >> EVT_PAYLOAD_SHIFT)

/* Navigation commands decoded from barcodes (FR8, FR29): 2 bits
   (Week 8 tribunal), the EVT_NAV_CMD payload */
#define NAV_LEFT	0
#define NAV_RIGHT	1
#define NAV_STRAIGHT	2
#define NAV_U_TURN	3
#define NAV_BITS	2
#define NAV_MASK	0x3u
#define NAV_NONE	4	/* "no command"; never sent or packed */

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)							\
	do {								\
		INT check_er_ = (INT)(er);	/* evaluate once */	\
		if (check_er_ < E_OK) {					\
			tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_); \
		}							\
	} while (0)

#endif /* APP_H */
