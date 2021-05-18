#include "PlatformWildFiTagREV5.h"

WildFiTagREV5 device = WildFiTagREV5();

RTC_DATA_ATTR uint64_t bootCnt = 0;

extern "C" void app_main() {
    while(1) {
        printf("HELLO WORLD\n");
        device.deepSleep();
    }
}