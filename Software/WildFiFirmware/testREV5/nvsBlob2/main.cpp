#include "PlatformWildFiTagREV5.h"

WildFiTagREV5 device = WildFiTagREV5();

RTC_DATA_ATTR uint64_t bootCnt = 0;

/** NVS Flash Buffer */
#define NVS_FLASH_BUFFER_NAME                           "flashbuff"                 
#define NVS_FLASH_BUFFER_SIZE                           512

typedef enum {
    PUSH_DATA_SUCCESS = 0,
    PUSH_DATA_PARAM_ERROR,
	PUSH_DATA_NVS_ERROR,
    PUSH_DATA_NVS_GET_BLOB_ERROR,
    PUSH_DATA_NVS_GET_BLOB_ERROR2,
    PUSH_DATA_NVS_SET_BLOB_ERROR,
    PUSH_DATA_NVS_COMMIT_ERROR,
    PUSH_DATA_FLASH_ON_ERROR,
    PUSH_DATA_NVS_SET_BLOB_ERROR2,
    PUSH_DATA_NVS_COMMIT_ERROR2,
    PUSH_DATA_NVS_WRITE_ERROR,
    PUSH_DATA_MT29_SEQ_WRITE_STATUS_BUFFER_ERROR,
    PUSH_DATA_MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR,
    PUSH_DATA_MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR,
    PUSH_DATA_FLASH_OFF_ERROR
} push_data_result_t;

/** NVS storage names */
#define NVS_FLASH_WRITE_PAGE_POINTER                    "flashpgpnt"                    // data writing: pointing on current flash page 0 .. 131071
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffpnt"                   // data writing: pointing on current byte in flash page 0 .. 2047
#define MOCK_FLASH_WRITES                               0 

push_data_result_t pushDataIntoMemory(uint8_t *newData, uint32_t newDataLen, bool *memoryFull, bool debug) {
    // TODO: STORE AFTER DATA TRANSMISSION BECAUSE OF POWER CONSUMPTION!!!
    esp_err_t err;
    nvs_handle_t handle;
    size_t nvsDataLen = 0;  // value will default to 0, if not set yet in NVS
    uint8_t nvsData[NVS_FLASH_BUFFER_SIZE] = { 0 };
    push_data_result_t returnVal = PUSH_DATA_SUCCESS;

    if((newDataLen == 0) || (newData == NULL) || (newDataLen > 512)) { return PUSH_DATA_PARAM_ERROR; }

    // get current size of 512 byte NVS buffer
    if(nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle) != ESP_OK) { return PUSH_DATA_NVS_ERROR; }

    err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, NULL, &nvsDataLen); // 0ms
    if(err == ESP_ERR_NVS_NOT_FOUND) { // first time, blob is empty
        if(debug) { printf("pushData: virgin NVS\n"); }
        nvsDataLen = 0;
    }
    else if(err != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_GET_BLOB_ERROR; }

    // read 512 byte NVS buffer (in case buffer is not empty)
    if(debug) { printf("pushData: %d bytes in NVS\n", nvsDataLen); }
    if(nvsDataLen > 0) {  
        err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, &nvsDataLen); // 1ms
        if(err != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_GET_BLOB_ERROR2; }
        if(debug) {
            for(uint16_t i = 0; i < nvsDataLen; i++) { printf("%02X ", nvsData[i]); } 
            printf("\n");
        }
    }

    if(nvsDataLen + newDataLen <= NVS_FLASH_BUFFER_SIZE) { // if new data fits into NVS, might be 512 afterwards (next cycle will write into flash)
        if(debug) { printf("pushData: FITS %d + %d <= %d\n", nvsDataLen, newDataLen, NVS_FLASH_BUFFER_SIZE); }
        if(debug) { printf("+ "); }

        // append new data
        for(uint32_t i = 0; i < newDataLen; i++) {
            nvsData[nvsDataLen + i] = newData[i];
            if(debug) { printf("%02X ", newData[i]); }
        }
        if(debug) { printf("\n"); }

        // write + commit NVS buffer, nvs_set_blob = TIME EATER
        if(nvs_set_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, nvsDataLen + newDataLen) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_SET_BLOB_ERROR; } // TIME CONSUMING: 7 - 117ms
        if(nvs_commit(handle) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_COMMIT_ERROR; } // 0ms
        nvs_close(handle); // 0ms
    }
    else { // if new data DOES NOT FIT into NVS
        // append bytes to fill 512 byte array completely
	    if(debug) { printf("pushData: DOESNT FIT %d + %d > %d -> WRITE FLASH\n", nvsDataLen, newDataLen, NVS_FLASH_BUFFER_SIZE); }
	    //if(debug) { printf("+ "); }
	    uint32_t newDataLeftPointer = 0;
	    for(newDataLeftPointer = 0; newDataLeftPointer < newDataLen; newDataLeftPointer++) {
	        if(nvsDataLen + newDataLeftPointer >= NVS_FLASH_BUFFER_SIZE) { break; }
	        nvsData[nvsDataLen + newDataLeftPointer] = newData[newDataLeftPointer];
	        //if(debug) { printf("%02X ", newData[newDataLeftPointer]); }
	    }
	    //if(debug) { printf("\n"); }	

        // write 512 byte array into flash memory
        if(!device.flashPowerOn(true)) { nvs_close(handle); return PUSH_DATA_FLASH_ON_ERROR; }
        uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
        uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
        if(debug) { printf("pushData: before: flashPageWritePointer: %d, flashPageWriteOffsetPointer: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer); }
        sequential_write_status_t writeStatus = device.flash.fifoPushSimple(0, flashPageWritePointer, flashPageWriteOffsetPointer, nvsData, NVS_FLASH_BUFFER_SIZE, true, MOCK_FLASH_WRITES);
        if(debug) { printf("pushData: after: flashPageWritePointer: %d, flashOffsetPointer: %d, FIFO space: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer, device.flash.fifoGetFreeSpace(0, flashPageWritePointer, flashPageWriteOffsetPointer, MT29_NUMBER_PAGES)); }
        if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full
            if(debug) { printf("pushData: flash full\n"); }
            *memoryFull = true;
        }
        else { // MT29_SEQ_WRITE_STATUS_SUCCESS or MT29_SEQ_WRITE_STATUS_ERROR, also update pointers in case of ERROR
            //device.flash.fifoPopDelete()
            if(!device.nvsWriteUINT32andUINT16(NVS_FLASH_WRITE_PAGE_POINTER, flashPageWritePointer, NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, flashPageWriteOffsetPointer)) { 
                returnVal = PUSH_DATA_NVS_WRITE_ERROR; // don't stop in case of error
            }
            if(writeStatus == MT29_SEQ_WRITE_STATUS_BUFFER_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_BUFFER_ERROR; } // don't stop in case of error
            else if(writeStatus == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR; } // don't stop in case of error
            else if(writeStatus == MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR; } // don't stop in case of error
        }

        if(!device.flashPowerOff(false)) { returnVal = PUSH_DATA_FLASH_OFF_ERROR; }

        // add rest data to "fresh" 512 byte array
        uint32_t nvsDataCnt = 0;
	    for(uint32_t i = newDataLeftPointer; i < newDataLen; i++) {
            nvsData[nvsDataCnt] = newData[i];
            nvsDataCnt++;
	    	//if(debug) { printf("%02X - ", newData[i]); }	
	    }
    	//if(debug) { printf("\n"); }
        nvsDataLen = nvsDataCnt;
        
        // write + commit rest of data to NVS
        if(nvs_set_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, nvsDataLen) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_SET_BLOB_ERROR2; } // TIME CONSUMING: 7 - 117ms
        if(nvs_commit(handle) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_COMMIT_ERROR2; } // 0ms
        nvs_close(handle); // 0ms
    }
    return returnVal;
}

extern "C" void app_main() {
    while(1) {
        device.initDataNVS(); // 3ms
        if(bootCnt == 0) {
            device.delay(8000);
            //device.selfTest(3750, SELFTEST_FLASH_FULL_ERASE | SELFTEST_NVS_RESET);
            //device.selfTest(3750, SELFTEST_NVS_RESET);
        }
        else {
            uint32_t random = (esp_random() % 256) + 1;
            uint16_t NEW_DATA_SIZE = random;
            uint8_t *newData = (uint8_t *) malloc(NEW_DATA_SIZE);
            bool memoryFull = false;
            for(uint16_t i=0; i<NEW_DATA_SIZE; i++) {
                newData[i] = bootCnt;
            }
            uint32_t millisNow = Timing::millis();
            push_data_result_t res = pushDataIntoMemory(newData, NEW_DATA_SIZE, &memoryFull, false);
            millisNow = (uint32_t) (Timing::millis()) - millisNow;
            uint8_t resByte = res;
            printf("RESULT: %d (%d bytes took %d ms), memoryFull: %d\n", resByte, NEW_DATA_SIZE, millisNow, memoryFull);
            if(res != PUSH_DATA_SUCCESS) {
                printf("--- SERIOUS ERROR ---\n");
                while(1) { ; }
            }
            uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
            uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
            printf("flashPageWritePointer: %d, flashPageWriteOffsetPointer: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer);
        }
        printf("\n");
        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(2);
        device.deepSleep();
    }
}