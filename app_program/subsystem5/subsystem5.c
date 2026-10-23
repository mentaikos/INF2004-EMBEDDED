/*
 *	subsystem5.c
 *	Subsystem 5 -- Ultrasonic scan: phase-1 objects, phase-2 task.
 *
 *	subsystem5_init():  create this subsystem's kernel objects
 *	                    (semaphores, mailboxes, ...).  Do not start
 *	                    tasks here.
 *	subsystem5_start(): start the core-0 scan task and 60-ms cyclic.
 *
 *	Drivers for this subsystem's hardware go in this folder as
 *	drv_<name>.c / drv_<name>.h, called only from the task that owns
 *	the peripheral (TEAM.md section 3).
 *
 *	Pins, tasks, objects and interfaces are recorded in this folder's
 *	README.md (the template's TEAM.md is absent from this repository).
 */

#include "subsystem5.h"
#include "scan_task.h"

EXPORT ID flg_scan, sem_echo, mtx_s5_status;
LOCAL ID cyc_scan;

EXPORT ER subsystem5_init(void)
{
	T_CSEM csem = {0};
	T_CFLG cflg = {0};
	T_CMTX cmtx = {0};
	T_CCYC ccyc = {0};
	csem.sematr = TA_TFIFO;
	csem.maxsem = 1;
	sem_echo = tk_cre_sem(&csem);
	if (sem_echo < E_OK) return sem_echo;
	cflg.flgatr = TA_TFIFO | TA_WMUL;
	flg_scan = tk_cre_flg(&cflg);
	if (flg_scan < E_OK) return flg_scan;
	cmtx.mtxatr = TA_INHERIT;
	mtx_s5_status = tk_cre_mtx(&cmtx);
	if (mtx_s5_status < E_OK) return mtx_s5_status;
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)s5_scan_cyclic;
	ccyc.cyctim = S5_PERIOD_MS;
	ccyc.cycphs = S5_PERIOD_MS;
	cyc_scan = tk_cre_cyc(&ccyc);
	if (cyc_scan < E_OK) return cyc_scan;
	return E_OK;
}

EXPORT ER subsystem5_start(void)
{
	tid_s5_scan = app_start_task(s5_scan_task, S5_PRI_MAIN, S5_SCAN_PRC, "obstacle_scan");
	if (tid_s5_scan < E_OK) return tid_s5_scan;
	return tk_sta_cyc(cyc_scan);
}
