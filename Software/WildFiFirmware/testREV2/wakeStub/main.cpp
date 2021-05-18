#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        if(device.getWakeUpReason() != BY_RTC) {
            printf("WOKE FROM WHATEVER!\n");
            printf("Batt Voltage: %d\n", device.readSupplyVoltage());
            device.powerAndInitI2C();
            device.rtc.setRegularInterrupt(10);
            
        }
        else {
            printf("WOKE BY RTC!\n");
        }
        device.enableRTCInterruptInDeepSleep();
        device.deepSleep();
    }
}