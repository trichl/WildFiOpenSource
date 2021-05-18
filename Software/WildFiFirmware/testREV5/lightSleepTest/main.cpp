#include "PlatformWildFiTagREV5.h"

WildFiTagREV5 device = WildFiTagREV5();

RTC_DATA_ATTR uint32_t testData[512] =  { 0 };
uint32_t testData2[512] = { 0 };

extern "C" void app_main() {
    while(1) {
        //device.setCPUSpeed(ESP32_10MHZ);
        printf("AWAKE!\n");
        printf("BECAUSE: %d\n", device.getWakeUpReason());

        while(1) {
            printf("%d %d\n", testData[0], testData2[0]);
            testData[0]++;
            testData2[0]++;
    	    //device.enableInternalTimerInterruptInDeepSleep(7); // wake up from internal RTC
            device.shortLightSleep(1000);
        }
    }
}