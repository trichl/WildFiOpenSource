#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        printf("AWAKE!\n");

        if(device.getWakeUpReason() != BY_TIMER) {
            device.ledRedOn();
        }

        uint8_t data[16] = { 0xEF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
        device.startBLE(BLE_ADVERTISE_AND_SCAN, &data[0], 16);
        device.delay(1500); // wait or do other stuff
        device.stopBLE();
        
        device.setCPUSpeed(ESP32_10MHZ);
        device.delay(500);
        printf("BIOLOGGER FOUND: %d\n", device.getBLEBiologgerFound());
        for(uint16_t i = 0; i < device.getBLEBiologgerFound(); i++) {
            printf("%d: ADDR: %llX, RSSI SUM: %d, RSSI AVG: %d, , Beacons: %d\n", i, device.getBLEMacAddress(i), device.getBLERSSISum(i), device.getBLERSSIAverage(i), device.getBLEBeaconCnt(i));
            printf(" PAYLOAD: ");
            for(uint8_t p = 0; p < 16; p++) {
                printf("%02X ", device.getBLEPayload(i, p));
            }
            printf("\n");
        }
        device.printOwnBLEAddress();

        if(device.getBLEBiologgerFound() == 0) device.blink(B_RED);
        else if(device.getBLEBiologgerFound() == 1) device.blink(B_GREEN);
        else if(device.getBLEBiologgerFound() == 2) device.blink(B_GREEN, B_GREEN);
        else if(device.getBLEBiologgerFound() >= 3) device.blink(B_GREEN, B_GREEN, B_GREEN);

        printf("BLE done, sleep\n");

        device.enableInternalRTCInterruptInDeepSleep(15);
        device.deepSleep();
    }
}