#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        printf("AWAKE!\n");
        device.initPins();
        device.powerAndInitI2C();

        printf("WAKE FROM DEEP SLEEP: %d\n", device.wokeFromDeepSleep());

        device.blink(B_RED, B_RED, B_GREEN);
        device.initInternalRTCInterrupt(5);
        device.deepSleep();
    }
}