#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        printf("AWAKE!\n");

        device.delay(1000);
        device.initPins();
        device.powerAndInitI2C();

        printf("CPU SPEED: %d\n", getCpuFrequencyMhz());
        printf("BATT VOLT: %d\n", device.readSupplyVoltage());

        printf("BME ALIVE: %d\n", device.baro.isAlive());
        printf("BMX ALIVE: %d\n", device.imu.isAlive());
        printf("RTC ALIVE: %d\n", device.rtc.isAlive());

        bool error = false;
        uint32_t time = device.rtc.getTimestamp(error);
        printf("RTC TIMESTAMP: %d\n", time);
        printf("RTC TIMESTAMP ERROR: %d\n", error);
        
        device.delay(1000);
        device.ledGreenOn();
        device.delay(1000);
        device.ledRedOn();
        device.delay(1000);

        device.enableInternalRTCInterruptInDeepSleep(5);
        device.deepSleep();
    }
}