/*
 *	subsystem5.h
 *	Ultrasonic scan, profiling & recovery (Buddy 5). Student ID: 2503076.
 *	Public interface; see README.md in this folder.
 */
#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H
#include "../app.h"
#include "obstacle_scan.h"

#define S5_PRI_MAIN             14       /* design review task table */
#define S5_SCAN_PRC             TP_PRC1
#define S5_PERIOD_MS            60       /* NFR15; kernel tick is 10 ms */
#define S5_PIN_TRIG             4
#define S5_PIN_ECHO             5
#define S5_PIN_SERVO            12       /* PWM slice 6 A; GP13 shares it */

#ifndef FAULT_NO_ECHO
#define FAULT_NO_ECHO           0x0008u  /* provisional team fault-list order */
#endif
#ifndef FAULT_LINE_TIMEOUT
#define FAULT_LINE_TIMEOUT      0x0040u
#endif

/* flg_scan: VC -> Obstacle. Use the helpers for approvals/line samples:
 * those flags also need a generation/timestamp, not just a bare bit.
 * Only this subsystem clears these bits. Never clear flg_safety here. */
IMPORT ID flg_scan;
IMPORT ID sem_echo;                     /* echo ISR -> scan task, max 1 */
IMPORT ID mtx_s5_status;                /* TA_INHERIT, coherent snapshot */

typedef struct {
	UW status;                         /* layout in obstacle_scan.h */
	S5_PROFILE profile;                /* 8 bytes, explicit masks/shifts */
	UH event_drops, overruns;           /* wrapping diagnostic counters */
} S5_SNAPSHOT;

/* Like the other buddies: bind once during VC init. Zero = unbound. */
IMPORT void s5_bind_vc(ID mbf_vc_evt, ID flg_fault);
IMPORT ER s5_get_status(S5_SNAPSHOT *snapshot);
IMPORT ER s5_request_scan(void);         /* observe Motion stopped first */
IMPORT ER s5_approve_plan(UB generation);
IMPORT ER s5_rejoin_line(void);
IMPORT ER s5_line_observe(BOOL found);   /* relay fresh S3 samples, <=60 ms */
IMPORT ER s5_cancel(void);              /* STOP/mission exit */
IMPORT ER s5_reset(void);               /* stopped, E-STOP clear */

/* Events use app.h EVT_PACK, always TMO_POL:
 * OBSTACLE_DETECTED: [6:0] distance code, [10:7] speed cap 10 cm/s.
 * OBSTACLE_PROFILE: [2:0] action, [10:3] width cm, [17:11] closest code,
 * [25:18] generation, [26] left safe, [27] right safe.
 * Read the complete profile before approving its generation.
 * NO_CLEAR: action in low 3 bits; reverse is only a proposal to the VC.
 * SENSOR_FAULT: FAULT_NO_ECHO / FAULT_LINE_TIMEOUT.
 * REJOIN_LINE_CONFIRMED: generation of the completed bypass.
 * VC alone commands speed limits, turns, reverse and line search. */

#endif /* SUBSYSTEM5_H */
