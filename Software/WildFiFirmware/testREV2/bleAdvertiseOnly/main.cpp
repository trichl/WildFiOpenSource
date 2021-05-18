#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        if(device.getWakeUpReason() != BY_RTC) {
            printf("FIRST START!\n");
            device.powerAndInitI2C();
            device.rtc.setRegularInterrupt(60);
        }
        else {
            printf("AWAKE!\n");
        }
        
        // TOTAL RUNTIME = 1.82s
        // EVERY 60s = 1.69mA average current -> 190mAh battery = 112h runtime = 4.66 days
        uint8_t data[16] = { 0xEF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
        device.startBLE(BLE_ADVERTISE_ONLY, &data[0], 16);
        device.delay(1000); // wait or do other stuff
        device.stopBLE();

        printf("BLE done, sleep\n");

        device.enableRTCInterruptInDeepSleep();
        device.deepSleep();
    }
}