#include "PlatformWildFiTagREV6.h"
#include "PlatformAttitudeEstimator.h"

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR uint64_t bootCnt = 0;

extern "C" void app_main() {
    while(1) {
        if(bootCnt == 0) {
            printf("HELLO WORLD\n");

        }
        else {

        }
        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(3);
        device.deepSleep();
    }
}
