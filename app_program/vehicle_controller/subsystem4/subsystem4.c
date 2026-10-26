/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU & terrain: creates its objects and starts its
 *	tasks.
 *
 *	APIs shown: tk_cre_mtx (TA_INHERIT), tk_cre_cyc, tk_sta_cyc
 *
 *	Template rule: the owner creates an object in phase 1, before any
 *	task can use it.  The I2C bus and the sensor are set up by the IMU
 *	task itself (drv_lsm303.c), so a missing sensor never stops start-up.
 */

#include "subsystem4.h"

EXPORT ID	cyc_imu;
EXPORT ID	mtx_s4_status;

EXPORT ER subsystem4_init(void)
{
	T_CMTX	cmtx = {0};
	T_CCYC	ccyc = {0};

	/* Mutex for the two status words.  TA_INHERIT: a low-priority reader
	   (e.g. Telemetry, pri 20) holding it is raised to our priority. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_s4_status = tk_cre_mtx(&cmtx);
	if (mtx_s4_status < E_OK) return mtx_s4_status;

	/* 10 ms tick (NFR11), started in subsystem4_start() once the task
	   exists */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)imu_cyclic_handler;
	ccyc.cyctim = S4_PERIOD_MS;
	ccyc.cycphs = S4_PERIOD_MS;
	cyc_imu = tk_cre_cyc(&ccyc);
	if (cyc_imu < E_OK) return cyc_imu;

	/* Example objects still used by subsystem 3 (remove later) */
	return s4_example_init();
}

EXPORT ER subsystem4_start(void)
{
	ID	tid;
	ER	er;

	tid_imu = app_start_task(imu_task, S4_PRI_IMU, S4_IMU_PRC, "imu_task");
	if (tid_imu < E_OK) return tid_imu;

	er = tk_sta_cyc(cyc_imu);
	if (er < E_OK) return er;

#if S4_LOG_MS > 0
	tid = app_start_task(imu_log_task, S4_PRI_LOG, 0, "imu_log_task");
	if (tid < E_OK) return tid;
#endif

#if S4_EXAMPLE_MONITOR
	tid = app_start_task(monitor_task, S4_PRI_MONITOR, 0, "monitor_task");
	if (tid < E_OK) return tid;
#endif
	(void)tid;
	return E_OK;
}
