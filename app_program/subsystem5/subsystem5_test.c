/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST
#include "scan_checks.h"
#define S5_CHECK(id, cond, desc) TEST_CHECK(id, cond, desc)

EXPORT void subsystem5_test(void)
{
	TEST_INIT();

	TEST_CHECK("S5-U01", tk_get_tid() > 0, "running in a Micro T-Kernel task");
	S5_RUN_CHECKS();
	TEST_CHECK("S5-U21", sem_echo > 0 && flg_scan > 0 && mtx_s5_status > 0,
		   "scan semaphore, event flag and mutex exist");

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
