#include "PlatformWildFiTagREV6.h"

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR uint64_t bootCnt = 0;

RTC_DATA_ATTR bool wakeStub() {
    //for(uint16_t i = 0; i < 100; i++) {
        ets_delay_us(400*1000);
    //}
    
    return true;
}

extern "C" void app_main() {
    while(1) {
        if(bootCnt == 0) {
            device.disableWakeStubNoBootIfVoltageLow();
            device.customWakeStubFunction(wakeStub);
            device.setWakeStubRejectionInterruptSrc(USE_EXT1_IF_WAKE_UP_REJECTED);
        }
        printf("HELLO WORLD\n");
        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(20);
        device.deepSleep();
    }
}