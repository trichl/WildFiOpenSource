#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

extern "C" void app_main() {
    while(1) {
        // Needs 2.4s for 1.6s broadcast (1.9s for 1.1s broadcast)
        // Every 60s = 3.44mA average (2.72mA) -> 100mAh battery = 29h (36h)
        // Every 10m = 371uA average (290uA) -> 100mAh battery = 11.2d (14.3d)

        /*uint8_t wakeReason = device.getWakeUpReason();
        printf("AWAKE! %d\n", wakeReason);

        if(wakeReason != BY_TIMER) {
            device.ledRedOn();
        }*/
        printf("Batt Voltage: %d\n", device.readSupplyVoltage());

        uint8_t data[16] = { 0xEF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
        device.startBLE(BLE_ADVERTISE_AND_SCAN, &data[0], 16);
        device.delay(4000); // wait or do other stuff
        device.stopBLE();
        
        //device.setCPUSpeed(ESP32_10MHZ);
        //device.delay(10);
        printf("BIOLOGGER FOUND: %d\n", device.getBLEBiologgerFound());
        for(uint16_t i = 0; i < device.getBLEBiologgerFound(); i++) {
            printf("%d: ADDR: %04x, RSSI SUM: %d, RSSI AVG: %d, , Beacons: %d\n", i, device.getBLEBiologgerId(i), device.getBLERSSISum(i), device.getBLERSSIAverage(i), device.getBLEBeaconCnt(i));
            printf("Payload Len: %d\n", device.getBLEPayloadLen(i));
            printf(" PAYLOAD: ");
            for(uint8_t p = 0; p < 16; p++) {
                printf("%02X ", device.getBLEPayload(i, p));
            }
            printf("\n");
        }

        //device.printOwnBLEAddress();
        /*if(device.getBLEBiologgerFound() == 0) device.blink(B_RED);
        else if(device.getBLEBiologgerFound() == 1) device.blink(B_GREEN);
        else if(device.getBLEBiologgerFound() == 2) device.blink(B_GREEN, B_GREEN);
        else if(device.getBLEBiologgerFound() >= 3) device.blink(B_GREEN, B_GREEN, B_GREEN);*/

        printf("BLE done, sleep\n");

        device.enableInternalRTCInterruptInDeepSleep(12);
        device.deepSleep();
    }
}