/* Private subsystem-5 linkage, not an inter-buddy contract. */
#ifndef S5_SCAN_TASK_H
#define S5_SCAN_TASK_H
#include "subsystem5.h"
IMPORT ID tid_s5_scan;
IMPORT void s5_scan_task(INT stacd, void *exinf);
IMPORT void s5_scan_cyclic(void *exinf);
#endif
