#include "ESP32TrackerREV3.h"

#define DATA1_LEN       28
#define DATA2_LEN       3840
#define DATA3_LEN       966

ESP32TrackerREV3 device = ESP32TrackerREV3();

uint8_t data1[DATA1_LEN] = { 0 };
uint8_t data2[DATA2_LEN] = { 0 };
uint8_t data3[DATA3_LEN] = { 0 };


#define FIFO_PAGES      4*64 // 3*64
uint32_t flashPointer = 4*64 - 5;
uint16_t flashOffsetPointer = 0;
uint16_t blockErasePointer = 3;


extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        esp_task_wdt_init(120, false);
        device.delay(1000);
        device.sensorPowerOn(true); // for I2C
        device.delay(1000);
        device.keepSensorPowerOnInDeepSleep();
        device.delay(8000);
        
        // FIRST FLASH POWER ON
        if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
        device.delay(2000);
        device.flash.printFlash(0, 64, MT29_CACHE_SIZE);
        device.delay(2000);
        if(!device.flashPowerOff()) { printf("ERROR FLASH OFF\n"); }
        device.delay(3000);
        // SECOND FLASH POWER ON
        /*if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
        device.flash.printFlash(32, 4, 20);
        if(!device.flashPowerOff()) { printf("ERROR FLASH OFF\n"); }
        device.delay(1000);*/

        device.deepSleep();
    }
}