#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        printf("AWAKE!\n");
        printf("BECAUSE: %d\n", device.getWakeUpReason());
        device.initPins();
        device.powerAndInitI2C();
        
        bool error = false;
        printf("RTC TIMESTAMP: %d\n", device.rtc.getTimestamp(error));
        device.rtc.setRegularInterrupt(20);

        device.blink(B_RED, B_RED, B_GREEN);

        device.delay(1000);

    	//device.enableInternalRTCInterruptInDeepSleep(4); // wake up from internal RTC
        device.enableWakeUpPinInDeepSleep(); // wake up from touch
        device.enableRTCInterruptInDeepSleep(); // wake up from RTC interrupt

        device.deepSleep(); // 10.7uA
    }
}