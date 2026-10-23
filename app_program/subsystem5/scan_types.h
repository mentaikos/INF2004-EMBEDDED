/* Fixed-width host equivalents; production uses micro T-Kernel types. */
#ifndef S5_SCAN_TYPES_H
#define S5_SCAN_TYPES_H
#ifdef S5_HOST_TEST
#include <stdint.h>
typedef uint8_t UB;
typedef uint16_t UH;
typedef uint32_t UW;
typedef int32_t W;
typedef int INT;
typedef unsigned int UINT;
typedef UINT BOOL;
#define LOCAL static
#define EXPORT
#define IMPORT extern
#define Inline static inline
#define TRUE 1
#define FALSE 0
#else
#include <tk/tkernel.h>
#endif
#endif
