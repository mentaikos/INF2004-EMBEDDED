/*
 *	s4_example_compat.c
 *	TEMPORARY: the template's example monitor (was task_monitor.c), kept
 *	only because subsystem 3's example consumer still sends to
 *	mbf_summary / flg_events (IF-03, INT-02).  The IMU does not use
 *	anything here.  Delete this file and section 7 of subsystem4.h
 *	together with subsystem 3's consumer_task.
 *
 *	APIs shown: tk_cre_flg, tk_cre_mbf, tk_cre_alm, tk_sta_alm,
 *	            tk_stp_alm, alarm handler, tk_wai_flg, tk_rcv_mbf,
 *	            tk_ref_sem, tk_ref_mpf, tk_ref_tsk, tk_sus_tsk,
 *	            tk_rsm_tsk, tk_chg_pri
 */

#include "subsystem4.h"
#include "../subsystem2/subsystem2.h"	/* sem_slots, mpf_samples (read only) */
#include "../subsystem3/subsystem3.h"	/* IF-04: app_stats, mtx_stats */

EXPORT ID	mbf_summary;	/* IF-03 */
EXPORT ID	flg_events;	/* IF-03 */
EXPORT ID	alm_monitor;
EXPORT UW	s4_lines_received;

EXPORT ER s4_example_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};
	T_CALM	calm = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_events = tk_cre_flg(&cflg);
	if (flg_events < E_OK) return flg_events;

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * S4_SUMMARY_MAX;
	cmbf.maxmsz = S4_SUMMARY_MAX;
	mbf_summary = tk_cre_mbf(&cmbf);
	if (mbf_summary < E_OK) return mbf_summary;

	calm.almatr = TA_HLNG;
	calm.almhdr = (FP)monitor_alarm_handler;
	alm_monitor = tk_cre_alm(&calm);
	return (alm_monitor < E_OK) ? alm_monitor : E_OK;
}

/* Alarm handler: one-shot, interrupt context (never wait) */
EXPORT void monitor_alarm_handler(void *exinf)
{
	(void)exinf;
	tk_set_flg(flg_events, FLG_ALARM);
}

EXPORT void monitor_task(INT stacd, void *exinf)
{
	UINT		flgptn;
	UB		text[S4_SUMMARY_MAX + 1];
	INT		len;
	APP_STATS	snap;
	T_RSEM		rsem;
	T_RMPF		rmpf;
	SYSTIM		now;
	ER		er;

	(void)stacd; (void)exinf;

	while (1) {
		tk_sta_alm(alm_monitor, S4_ALARM_MS);
		er = tk_wai_flg(flg_events, FLG_ALARM | FLG_BATCH,
				TWF_ORW | TWF_CLR, &flgptn, S4_ALARM_MS * 2);
		tk_stp_alm(alm_monitor);

		if (er == E_TMOUT) {
			tm_printf((UB*)"monitor: nothing at all happened?\n");
			continue;
		}
		if (flgptn & FLG_ALARM) {
			tm_printf((UB*)"monitor: alarm -- no batch for %d ms\n",
				  S4_ALARM_MS);
		}

		while ((len = tk_rcv_mbf(mbf_summary, text, TMO_POL)) > 0) {
			text[len] = '\0';
			s4_lines_received++;
			tm_printf((UB*)"monitor: batch  %s\n", text);
		}

		tk_loc_mtx(mtx_stats, TMO_FEVR);
		snap = app_stats;
		tk_unl_mtx(mtx_stats);

		tk_get_tim(&now);
		tk_ref_sem(sem_slots, &rsem);
		tk_ref_mpf(mpf_samples, &rmpf);
		tm_printf((UB*)"monitor: t=%u ms  samples=%u  min=%d max=%d avg=%d"
			  "  free slots=%d  free blocks=%d\n",
			  now.lo, snap.received, snap.min, snap.max,
			  snap.received ? (INT)(snap.sum / (W)snap.received) : 0,
			  rsem.semcnt, (INT)rmpf.frbcnt);

		if (++reports % S4_LED_TOGGLE_REPORTS == 0) {
			if (led_paused) {
				tk_rsm_tsk(tid_led);
				tk_chg_pri(tid_led, S1_PRI_LED);
			} else {
				tk_sus_tsk(tid_led);
				tk_chg_pri(tid_led, S1_PRI_LED + 1);
			}
			led_paused = !led_paused;

			tk_ref_tsk(tid_led, &rtsk);
			tm_printf((UB*)"monitor: LED task %s, state=%s, pri=%d\n",
				  led_paused ? "paused" : "resumed",
				  state_name(rtsk.tskstat), rtsk.tskpri);
		}
	}
}