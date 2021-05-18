#include "ESP32TrackerREV4.h"

ESP32TrackerREV4 device = ESP32TrackerREV4();

extern "C" void app_main() {
    while(1) {
        printf("WAKE FROM DEEP SLEEP: %d\n", device.getWakeUpReason());
        device.selfTest(3750, SELFTEST_BARO); 

        device.deepSleep();
    }
}