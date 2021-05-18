#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_io_reg.h"

	.bss

	.global x
x:	.long 0

	.text
	.global entry
entry:
	move r1, x    // Get adress of x
	ld r0, r1, 0  // Read x in r0
	add r0, r0, 20 // Increment x
	st r0, r1, 0  // Save the x in memory

	wake
	halt