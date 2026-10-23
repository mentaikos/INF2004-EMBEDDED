/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_mbf, tk_cre_flg, tk_cre_cyc, tk_sta_cyc
 *
 *	Template rule: the receiver/owner creates an object, so Motion
 *	creates mbf_motion_cmd, mbf_steer and flg_safety here, in phase 1,
 *	before any task can send to them.
 */

#include "subsystem2.h"
#include "motor_driver.h"
#include "encoder.h"

EXPORT ID	mbf_motion_cmd;
EXPORT ID	mbf_steer;
EXPORT ID	flg_safety;
LOCAL  ID	cyc_motion;

EXPORT ER subsystem2_init(void)
{
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};
	T_CCYC	ccyc = {0};
	ER	er;

	/* Motors braked first: the pins are safe from here on */
	motor_init();

	/* Commands: 4-byte packed words, room for several in a burst */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 64;
	cmbf.maxmsz = sizeof(UW);
	mbf_motion_cmd = tk_cre_mbf(&cmbf);
	if (mbf_motion_cmd < E_OK) return mbf_motion_cmd;

	/* Steering: 1-byte signed values, Motion keeps only the newest */
	cmbf.bufsz  = 32;
	cmbf.maxmsz = sizeof(B);
	mbf_steer = tk_cre_mbf(&cmbf);
	if (mbf_steer < E_OK) return mbf_steer;

	/* Safety flag: several tasks may wait on it (TA_WMUL) */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_safety = tk_cre_flg(&cflg);
	if (flg_safety < E_OK) return flg_safety;

	/* 20 ms tick (started in subsystem2_start) */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)motion_cyclic_handler;
	ccyc.cyctim = S2_PERIOD_MS;
	ccyc.cycphs = S2_PERIOD_MS;
	cyc_motion = tk_cre_cyc(&ccyc);
	if (cyc_motion < E_OK) return cyc_motion;

	/* Encoders: pins, pull-ups, shared GPIO interrupt.  Runs on the
	   core of usermain(), which therefore takes the encoder interrupt. */
	er = encoder_init();
	if (er < E_OK) return er;

	/* Example objects still used by subsystems 3 and 4 (remove later) */
	return s2_example_init();
}

EXPORT ER subsystem2_start(void)
{
	ID	tid;
	ER	er;

	tid = app_start_task(motion_task, S2_PRI_MOTION, S2_MOTION_PRC,
			     "motion_task");
	if (tid < E_OK) return tid;

	er = tk_sta_cyc(cyc_motion);
	if (er < E_OK) return er;

#if S2_EXAMPLE_PRODUCER
	tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
			     "producer_task");
	if (tid < E_OK) return tid;
#endif
	return E_OK;
}
