/*
 *	vehicle_controller.c
 *	Vehicle Controller -- creates its objects, binds them into every
 *	buddy, starts its task.
 *
 *	APIs shown: tk_cre_mbf, tk_cre_flg
 *
 *	usermain() calls vc_init() AFTER every subsystemN_init() (their
 *	objects, e.g. flg_safety, must exist) and BEFORE any _start(), so
 *	each buddy is bound before its task can send an event.
 */

#include "vehicle_controller.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

EXPORT ID	mbf_vc_evt;
EXPORT ID	flg_fault;

/* 4-byte events.  Bursts: S3 (junction + lost + nav), S4 (hump + fault),
   S5 (detected + profile), S2 (done + fault), S1 (start/stop) can all
   arrive inside one VC tick; 256 bytes holds well over 16 of them. */
#define VC_EVT_BUFSZ	256

EXPORT ER vc_init(void)
{
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = VC_EVT_BUFSZ;
	cmbf.maxmsz = sizeof(UW);
	mbf_vc_evt = tk_cre_mbf(&cmbf);
	if (mbf_vc_evt < E_OK) return mbf_vc_evt;

	/* Team fault flag: each owner sets/clears only its own bits */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_fault = tk_cre_flg(&cflg);
	if (flg_fault < E_OK) return flg_fault;

	/* Bind every sender now, before any task runs */
	motion_bind_vc(mbf_vc_evt, flg_fault);
	s3_bind_vc(mbf_vc_evt, flg_fault);
	s4_bind_vc(mbf_vc_evt, flg_fault);
	s5_bind_vc(mbf_vc_evt, flg_fault);
	return E_OK;
}

EXPORT ER vc_start(void)
{
	ID	tid = app_start_task(vc_task, VC_PRI, VC_PRC, "vehicle_ctrl");
	return (tid < E_OK) ? tid : E_OK;
}
