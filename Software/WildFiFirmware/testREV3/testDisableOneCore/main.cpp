#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

// AVERAGE TWO CORES (in delay) = 26mA, ONE CORE = 24.4mA 
// AVERAGE TWO CORES (busy wait) = 41.7mA, ONE CORE = 32.9mA

// RELATED TO CONFIG_FREERTOS_UNICORE SETTING in menuconfig

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        printf("WAKE FROM DEEP SLEEP: %d\n", device.getWakeUpReason());
        device.sensorPowerOn(true); // for I2C
        device.keepSensorPowerOnInDeepSleep();

        uint32_t cnt = 0;
        while(cnt < (4294967296L/100L)) {
            cnt++;
        }

        device.delay(5000);
        

        device.deepSleep();
    }
}