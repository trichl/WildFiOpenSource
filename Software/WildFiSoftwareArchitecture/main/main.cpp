#define BOOTLOADER_BUILD

#include "PlatformWildFiTagREV6.h"
WildFiTagREV6 device = WildFiTagREV6();

// THIS IS A DUMMY PROJECT TO ALLOW ECLIPSE SUPPORT -> REAL FIRMWARE OF WILDFI TAG IS LOCATED IN PlatformIO FOLDER

extern "C" void app_main(void) {
    while (true) {
		#ifdef BOOTLOADER_BUILD
    		printf("BOOTLOADER_BUILD!\n");
		#endif
    	printf("Hello World!\n");
    	device.enableInternalTimerInterruptInDeepSleep(4);
    	device.deepSleep();
    }
}
