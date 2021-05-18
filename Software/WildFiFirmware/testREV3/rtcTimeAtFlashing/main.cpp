#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();
RTC_DATA_ATTR uint16_t startCnt = 0;

bool error = false;

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        printf("WAKE NUMBER: %d\n", startCnt);
        printf("V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub());
        //gpio_set_drive_capability(PIN_POWER, GPIO_DRIVE_CAP_3); // TEST ONLY
        
        i2c.begin();
        if(startCnt == 0) {
            device.rtc.setTimestamp(UNIX_TIMESTAMP);
            device.rtc.setRegularInterrupt(7);
        }
        printf("COMPILE TIME: %lu\n", UNIX_TIMESTAMP);  
        printf("TIME: %d\n", device.rtc.getTimestamp(error));
        
        tmElements_t tm;
        breakTime(device.rtc.getTimestamp(error), tm);
        printf("%d.%d.%d %d:%d:%d\n", tm.Day, tm.Month, 1970+tm.Year, tm.Hour, tm.Minute, tm.Second);

        printf("SLEEPY TIME\n\n");
        startCnt++;
        device.enableRTCInterruptInDeepSleep();
        device.deepSleep();
    }
}