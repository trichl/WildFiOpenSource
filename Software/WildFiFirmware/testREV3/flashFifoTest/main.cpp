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
        uint32_t dataPushed = 0;
        esp_task_wdt_init(120, false);
        sequential_write_status_t ret = MT29_SEQ_WRITE_STATUS_SUCCESS;
        //device.setCPUSpeed(ESP32_10MHZ);
        printf("WAKE FROM DEEP SLEEP: %d\n", device.getWakeUpReason());
        device.sensorPowerOn(true); // for I2C
        device.keepSensorPowerOnInDeepSleep();

        device.delay(4000);
        device.setCPUSpeed(ESP32_10MHZ);
        device.delay(4000);

        for(uint16_t i=0; i<DATA1_LEN; i++) { data1[i] = 0xAA; }
        for(uint16_t i=0; i<DATA2_LEN; i++) { data2[i] = 0xBB; }
        for(uint16_t i=0; i<DATA3_LEN; i++) { data3[i] = 0xCC; }
        
        printf("SPACE LEFT IN FIFO: %d\n", device.flash.fifoGetFreeSpace(blockErasePointer, flashPointer, flashOffsetPointer, FIFO_PAGES));
        printf("NUMBER OF POPABLE BLOCKS: %d\n", device.flash.fifoGetNumberOfPopableBlocks(blockErasePointer, flashPointer, FIFO_PAGES));

        dataPushed = 0;
        while(ret == MT29_SEQ_WRITE_STATUS_SUCCESS) {
            ret = device.flash.fifoPush(blockErasePointer, flashPointer, flashOffsetPointer, data1, DATA1_LEN, data2, DATA2_LEN, data3, DATA3_LEN, false, 20, FIFO_PAGES);
            printf("SPACE LEFT IN FIFO: %d\n", device.flash.fifoGetFreeSpace(blockErasePointer, flashPointer, flashOffsetPointer, FIFO_PAGES));
            if(ret == MT29_SEQ_WRITE_STATUS_SUCCESS) { dataPushed += DATA1_LEN + DATA2_LEN + DATA3_LEN; }
        }
        printf("PUSHED %d Bytes\n", dataPushed);

        printf("NUMBER OF POPABLE BLOCKS: %d\n", device.flash.fifoGetNumberOfPopableBlocks(blockErasePointer, flashPointer, FIFO_PAGES));
        printf("BLOCKERASEPOINTER: %d\n", blockErasePointer);

        if(!device.flash.fifoPopDelete(blockErasePointer, flashPointer, FIFO_PAGES, false)) { printf("ERROR POP\n"); }
        printf("BLOCKERASEPOINTER: %d\n", blockErasePointer);
        printf("NUMBER OF POPABLE BLOCKS: %d\n", device.flash.fifoGetNumberOfPopableBlocks(blockErasePointer, flashPointer, FIFO_PAGES));

        if(!device.flash.fifoPopDelete(blockErasePointer, flashPointer, FIFO_PAGES, false)) { printf("ERROR POP\n"); }
        printf("BLOCKERASEPOINTER: %d\n", blockErasePointer);
        printf("NUMBER OF POPABLE BLOCKS: %d\n", device.flash.fifoGetNumberOfPopableBlocks(blockErasePointer, flashPointer, FIFO_PAGES));

        if(!device.flash.fifoPopDelete(blockErasePointer, flashPointer, FIFO_PAGES, false)) { printf("ERROR POP\n"); }
        printf("BLOCKERASEPOINTER: %d\n", blockErasePointer);
        printf("NUMBER OF POPABLE BLOCKS: %d\n", device.flash.fifoGetNumberOfPopableBlocks(blockErasePointer, flashPointer, FIFO_PAGES));

        if(!device.flash.fifoPopDelete(blockErasePointer, flashPointer, FIFO_PAGES)) { printf("ERROR POP\n"); }
        printf("BLOCKERASEPOINTER: %d\n", blockErasePointer);
        printf("NUMBER OF POPABLE BLOCKS: %d\n", device.flash.fifoGetNumberOfPopableBlocks(blockErasePointer, flashPointer, FIFO_PAGES));

        printf("SPACE LEFT IN FIFO: %d\n", device.flash.fifoGetFreeSpace(blockErasePointer, flashPointer, flashOffsetPointer, FIFO_PAGES));

        ret = MT29_SEQ_WRITE_STATUS_SUCCESS;
        dataPushed = 0;
        while(ret == MT29_SEQ_WRITE_STATUS_SUCCESS) {
            ret = device.flash.fifoPush(blockErasePointer, flashPointer, flashOffsetPointer, data1, DATA1_LEN, data2, DATA2_LEN, data3, DATA3_LEN, false, 20, FIFO_PAGES);
            printf("SPACE LEFT IN FIFO: %d\n", device.flash.fifoGetFreeSpace(blockErasePointer, flashPointer, flashOffsetPointer, FIFO_PAGES));
            if(ret == MT29_SEQ_WRITE_STATUS_SUCCESS) { dataPushed += DATA1_LEN + DATA2_LEN + DATA3_LEN; }
        }
        printf("PUSHED %d Bytes\n", dataPushed);

        printf("NUMBER OF POPABLE BLOCKS: %d\n", device.flash.fifoGetNumberOfPopableBlocks(blockErasePointer, flashPointer, FIFO_PAGES));

        device.delay(500);
        device.lightSleep();
    }
}