#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        printf("WAKE FROM DEEP SLEEP: %d\n", device.getWakeUpReason());

        device.delay(1000); // Test should be around 9mA

        /** TEST I2C */
        device.sensorPowerOn();
        printf("RTC ALIVE: %d\n", device.rtc.isAlive());
        printf("BARO ALIVE: %d\n", device.baro.isAlive());

        uint64_t start = Arduino::millisWrapper();
        while(!device.imu.isAlive()) {
            device.delay(3);
        }
        printf("IMU ALIVE: %d\n", device.imu.isAlive()); // needs time to wake up
        start = Arduino::millisWrapper() - start;
        printf("IMU needed %llu ms\n", start);

        device.delay(1000);

        /** TEST LEDs */
        device.blink(B_RED, B_RED, B_GREEN); // Test LEDs

        /** TEST V_BATT measurement */
        printf("CALC VBATT: %d\n", device.readSupplyVoltage());

        /** TEST EXTERNAL RTC WAKE UP */
        device.rtc.setRegularInterrupt(7);
        device.enableRTCInterruptInDeepSleep();

        /** TEST SLEEP */
        device.deepSleep(); // Test Deep sleep should be around 10uA
    }
}