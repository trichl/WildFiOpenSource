#include "PlatformWildFiTagREV5.h"

WildFiTagREV5 device = WildFiTagREV5();

RTC_DATA_ATTR uint64_t bootCnt = 0;

bool storeTest() {
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    uint8_t data[512] = { 0 };
    esp_err_t err;

    /*for(uint16_t i=0;i<512;i++) {
        data[i] = i;
    }*/

    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if(err != ESP_OK) {
        printf("ERR: %s\n", esp_err_to_name(err));
        return false;
    }
    device.measureTime("nvs_open_from_partition");
    
    
    err = nvs_get_blob(handle, "testblob", NULL, &required_size);
    device.measureTime("nvs_get_blob1");
    
    if(err == ESP_ERR_NVS_NOT_FOUND) {
        printf("not yet set\n");
    }
    else if(err == ESP_OK) {

    }
    else {
        printf("ERR: %s\n", esp_err_to_name(err));
        return false;
    }

    if(required_size > 0) {
        err = nvs_get_blob(handle, "testblob", data, &required_size);
        device.measureTime("nvs_get_blob2");

        if(err != ESP_OK) {
            printf("ERR: %s\n", esp_err_to_name(err));
        }

        for(uint16_t i=0;i<512;i++) {
            printf("%02X ", data[i]);
        } 

        printf("\n");
    }

    /*err = nvs_set_blob(handle, "testblob", data, 512);
    if(err != ESP_OK) {
        printf("ERR: %s\n", esp_err_to_name(err));
        //return false;
    }*/
    nvs_close(handle);

    return true;

}

bool modifyStoreTest() {
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    uint8_t data[512] = { 0 };
    esp_err_t err;

    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if(err != ESP_OK) {
        printf("ERR: %s\n", esp_err_to_name(err));
        return false;
    }
    device.measureTime("nvs_open_from_partition");
    
    
    err = nvs_get_blob(handle, "testblob", NULL, &required_size);
    device.measureTime("nvs_get_blob1");
    
    if(err == ESP_ERR_NVS_NOT_FOUND) {
        printf("not yet set\n");
    }
    else if(err == ESP_OK) {

    }
    else {
        printf("ERR: %s\n", esp_err_to_name(err));
        return false;
    }

    err = nvs_get_blob(handle, "testblob", data, &required_size);
    device.measureTime("nvs_get_blob2");
    if(err != ESP_OK) {
        printf("ERR: %s\n", esp_err_to_name(err));
    }

    for(uint16_t i=0;i<512;i++) {
        printf("%02X ", data[i]);
    }
    printf("\n -> +1\n");

    data[0]++;
    device.measureTime("pre nvs_set_blob");
    err = nvs_set_blob(handle, "testblob", data, 512); // 20ms
    device.measureTime("nvs_set_blob");
    if(err != ESP_OK) {
        printf("ERR: %s\n", esp_err_to_name(err));
        //return false;
    }
    err = nvs_commit(handle); // 0ms
    device.measureTime("nvs_commit");
    if(err != ESP_OK) return false;

    nvs_close(handle); // 1ms
    device.measureTime("nvs_close");

    return true;
}

void resetBlob() {
    nvs_handle_t handle;
    esp_err_t err;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if(err != ESP_OK) {
        printf("ERR: %s\n", esp_err_to_name(err));
        return;
    }
    device.measureTime("nvs_open_from_partition");
    nvs_erase_key(handle, "testblob");
    device.measureTime("nvs_erase_key");

    err = nvs_commit(handle); // 0ms
    device.measureTime("nvs_commit");
    if(err != ESP_OK) { printf("ERR\n"); return; }

    nvs_close(handle); // 1ms
    device.measureTime("nvs_close");
}

uint32_t timeMeasurementStart = 0;
uint32_t timeMeasurement[10] = { 0 };

const uint16_t BLOB_SIZE = 512;

bool append(uint8_t *data, uint16_t dataLen, bool debug) {
    timeMeasurement[0] = ((uint32_t) Timing::millis()) - timeMeasurementStart;
    esp_err_t err;
    nvs_handle_t handle;
    size_t nvsBlobFillLevel = 0;  // value will default to 0, if not set yet in NVS
    uint8_t dataBlob[BLOB_SIZE] = { 0 };
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if(err != ESP_OK) {
        if(debug) { printf("ERR0: %s\n", esp_err_to_name(err)); }
        return false;
    }
    timeMeasurement[1] = ((uint32_t) Timing::millis()) - timeMeasurementStart;
    
    err = nvs_get_blob(handle, "testblob", NULL, &nvsBlobFillLevel); // 0ms

    timeMeasurement[2] = ((uint32_t) Timing::millis()) - timeMeasurementStart;
    
    if(err == ESP_ERR_NVS_NOT_FOUND) { // first time, blob is empty
        if(debug) { printf("not yet set\n"); }
        nvsBlobFillLevel = 0;
    }
    else if(err != ESP_OK) {
        if(debug) { printf("ERR: %s\n", esp_err_to_name(err)); }
        return false;
    }

    if(nvsBlobFillLevel >= BLOB_SIZE) {
        if(debug) { printf("I AM FULL\n"); }
        return false;
    }

    // get existing data
    if(nvsBlobFillLevel > 0) {
        if(debug) { printf("Getting %d bytes\n", nvsBlobFillLevel); }
        err = nvs_get_blob(handle, "testblob", dataBlob, &nvsBlobFillLevel); // 1ms
        timeMeasurement[3] = ((uint32_t) Timing::millis()) - timeMeasurementStart;

        if(err != ESP_OK) {
            if(debug) { printf("ERR: %s\n", esp_err_to_name(err)); }
            return false;
        }

        if(debug) {
            for(uint16_t i=0;i<nvsBlobFillLevel;i++) {
                printf("%02X ", dataBlob[i]);
            } 
            printf("\n");
        }
    }

    // append new data
    uint16_t addedBytes = 0;
    if(debug) { printf("+ "); }
    for(uint16_t i=0; i<dataLen; i++) {
        if(nvsBlobFillLevel + i >= BLOB_SIZE) {
            if(debug) { printf("OVERLOAD\n"); }
            break;
        }
        dataBlob[nvsBlobFillLevel + i] = data[i];
        addedBytes++;
        
        if(debug) { printf("%02X ", data[i]); }
    }
    if(debug) { printf("\n"); }
    timeMeasurement[4] = ((uint32_t) Timing::millis()) - timeMeasurementStart;

    if(debug) { printf("Appended %d bytes\n", addedBytes); }

    err = nvs_set_blob(handle, "testblob", dataBlob, nvsBlobFillLevel + addedBytes); // 7 - 117ms
    timeMeasurement[5] = ((uint32_t) Timing::millis()) - timeMeasurementStart;
    if(err != ESP_OK) {
        if(debug) { printf("ERR: %s\n", esp_err_to_name(err)); }
        //return false;
    }
    err = nvs_commit(handle); // 0ms
    timeMeasurement[6] = ((uint32_t) Timing::millis()) - timeMeasurementStart;
    if(err != ESP_OK) {
        if(debug) { printf("ERR2: %s\n", esp_err_to_name(err)); }
    }
    nvs_close(handle);
    timeMeasurement[7] = ((uint32_t) Timing::millis()) - timeMeasurementStart;

    return true;    
}

RTC_DATA_ATTR uint64_t storeTimeSum = 0;
RTC_DATA_ATTR uint32_t storeTimeCnt = 0;

extern "C" void app_main() {
    while(1) {
        // DataNVS = 8Kbyte (2 x 4KByte page)
        //device.setCPUSpeed(ESP32_10MHZ);
        device.measureTime("boot");
        device.initDataNVS(); // 3ms
        device.measureTime("initDataNVS");

        if(bootCnt == 0) {
            device.delay(6000);
            resetBlob();
        }
        else {
            //if(!modifyStoreTest()) { printf("BLOBERR\n"); }
            const uint16_t TEST_DATA_SIZE = 66;
            uint8_t testData[TEST_DATA_SIZE];
            for(uint16_t i=0; i<TEST_DATA_SIZE; i++) {
                testData[i] = bootCnt;
            }
            timeMeasurementStart = ((uint32_t) Timing::millis());
            
            // 1024 byte add +15: avg 52ms, minimum 7ms, sometimes 200ms
            // 512 byte add +6: avg 25ms, minimum 7ms, sometimes 102ms
            // 512 byte add +15: avg 26ms, minimum 7ms, sometimes 91-117ms
            // 512 byte add +66: avg 24ms, minimum 7ms, sometimes 103ms
                // with nvsTest: avg 25ms
            bool result = append(testData, TEST_DATA_SIZE, false);

            uint32_t nvsTest = device.nvsReadUINT32("testVal");
            if(!device.nvsWriteUINT32("testVal", nvsTest + 1)) { printf("NVS FATALITY\n"); }
            printf("NVS TEST: %d\n", nvsTest);


            storeTimeSum += timeMeasurement[7];
            storeTimeCnt++;

            printf("%d: Start\n", timeMeasurement[0]);
            printf("%d: nvs_open_from_partition\n", timeMeasurement[1]);
            printf("%d: nvs_get_blob\n", timeMeasurement[2]);
            printf("%d: nvs_get_blob2\n", timeMeasurement[3]);
            printf("%d: append_data\n", timeMeasurement[4]);
            printf("%d: nvs_set_blob\n", timeMeasurement[5]);
            printf("%d: nvs_commit\n", timeMeasurement[6]);
            printf("%d: nvs_close\n", timeMeasurement[7]);

            printf("Average append time: %llu\n", (storeTimeSum / storeTimeCnt)); // 26ms

            if(!result) {
                printf("assume full -> reset\n");
                resetBlob();
            }
        }
        

        
        printf("\n");
        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(1);
        device.deepSleep();
    }
}