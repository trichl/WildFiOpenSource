#include "PlatformWildFiTagREV6.h"

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR uint64_t bootCnt = 0;



extern "C" void app_main() {
    while(1) {
        if(bootCnt == 0) {
            printf("HELLO WORLD\n");
            if(device.waitOnChar('a', 3)) {
                printf("WORKS!\n");
            }

        }
        
        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(20);
        device.deepSleep();
    }
}