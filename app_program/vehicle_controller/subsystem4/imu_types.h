/*
 *	imu_types.h
 *	Subsystem 4 -- the basic types used by the IMU logic and the packing
 *	header.  On the robot these are micro T-Kernel's own types; the PC
 *	test (host_test/, built with -DS4_HOST_TEST) gets the same names from
 *	<stdint.h>, so imu_monitor.c and s4_pack.h compile unchanged on a PC.
 */

#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#ifdef S4_HOST_TEST

#include <stdint.h>

typedef int8_t		B;
typedef int16_t		H;
typedef int32_t		W;
typedef int64_t		D;
typedef uint8_t		UB;
typedef uint16_t	UH;
typedef uint32_t	UW;
typedef uint64_t	UD;
typedef int		INT;
typedef unsigned int	UINT;
typedef INT		BOOL;

#define LOCAL		static
#define EXPORT
#define IMPORT		extern
#define Inline		static inline
#define TRUE		1
#define FALSE		0

#else

#include <tk/tkernel.h>

#endif /* S4_HOST_TEST */

#endif /* IMU_TYPES_H */
