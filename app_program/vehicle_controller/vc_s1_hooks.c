/*
 *	vc_s1_hooks.c
 *	Buddy 1's command hooks (subsystem1.h, IF-S1-02), Vehicle Controller
 *	side.  Subsystem 1 has WEAK versions that only print; these real
 *	versions (same names) replace them at link time, so a dashboard
 *	START / STOP now reaches the system statechart.
 *
 *	Called by Buddy 1's Command Handler task.  They never block: the
 *	event goes into mbf_vc_evt with TMO_POL.  FALSE (START guard not met,
 *	or buffer full) makes Buddy 1 publish robot/ack = FAILED.
 *	The E-STOP hook is Motion Control's (subsystem2.c): it sets the bit
 *	in flg_safety directly and never waits for the VC (NFR10).
 */

#include "vehicle_controller.h"
#include "../subsystem1/subsystem1.h"

/* START: check the guards here too, so a refused START comes back to the
   dashboard as robot/ack = FAILED instead of silently staying IDLE.  The
   VC checks again when it handles the event (state may change between). */
EXPORT BOOL s1_out_start(void)
{
	tm_printf((UB*)"S1 -> VC: START\n");
	if (vc_start_check(TRUE) != VC_R_NONE) return FALSE;
	return (vc_post(EVT_START, 0) == E_OK) ? TRUE : FALSE;
}

EXPORT BOOL s1_out_stop(void)
{
	tm_printf((UB*)"S1 -> VC: STOP\n");
	return (vc_post(EVT_STOP, 0) == E_OK) ? TRUE : FALSE;
}
