#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        device.delay(1000);
        device.initPins();
        printf("AWAKE!\n");
        printf("CALC VBATT: %d\n", device.readSupplyVoltage());

        device.initInternalRTCInterrupt(5);
        device.deepSleep();
    }
}