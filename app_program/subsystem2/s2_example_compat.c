/*
 *	s2_example_compat.c
 *	TEMPORARY: the template's example producer, kept only so the example
 *	code still in subsystem 3 (consumer, INT-01) and subsystem 4
 *	(monitor) builds and runs.  Motion Control does not use anything
 *	here.  Delete this file and section 10 of subsystem2.h once Buddies
 *	3 and 4 have replaced their example code.
 */

#include "subsystem2.h"
#include "../subsystem3/subsystem3.h"	/* IF-01: mbx_samples */

EXPORT ID	sem_slots;
EXPORT ID	mpf_samples;

EXPORT ER s2_example_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};

	csem.sematr  = TA_TFIFO;
	csem.isemcnt = S2_POOL_COUNT;
	csem.maxsem  = S2_POOL_COUNT;
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) return sem_slots;

	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;
	cmpf.blfsz  = sizeof(SAMPLE_MSG);
	mpf_samples = tk_cre_mpf(&cmpf);
	return (mpf_samples < E_OK) ? mpf_samples : E_OK;
}

/* A fake measurement: a triangle wave between 0 and 100 */
EXPORT INT s2_make_value(UW seq)
{
	INT	phase = (INT)(seq % 20);
	return (phase < 10 ? phase : 20 - phase) * 10;
}

EXPORT void producer_task(INT stacd, void *exinf)
{
	UW		seq = 0;
	SAMPLE_MSG	*msg;
	SYSTIM		now;
	ER		er;

	(void)stacd; (void)exinf;

	while (1) {
		er = tk_wai_sem(sem_slots, 1, TMO_FEVR);
		CHECK(er, "tk_wai_sem");

		er = tk_get_mpf(mpf_samples, (void **)&msg, TMO_POL);
		if (er < E_OK) {
			CHECK(er, "tk_get_mpf");
			tk_sig_sem(sem_slots, 1);
			tk_dly_tsk(S2_PRODUCER_PERIOD_MS);
			continue;
		}

		tk_get_tim(&now);
		msg->seq     = seq++;
		msg->time_ms = now.lo;
		msg->value   = s2_make_value(msg->seq);
#if TK_SUPPORT_SMP
		msg->prc     = tk_get_prc();
#else
		msg->prc     = 1;
#endif
		er = tk_snd_mbx(mbx_samples, (T_MSG *)msg);
		CHECK(er, "tk_snd_mbx");

		tk_dly_tsk(S2_PRODUCER_PERIOD_MS);
	}
}
