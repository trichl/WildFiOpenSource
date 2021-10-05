#include "PlatformWildFiTagREV6.h"
#include "PlatformAttitudeEstimator.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR uint32_t bootCnt = 0;

#define NVS_FLASH_BUFFER_NAME                           "flashbuff"                 
#define NVS_FLASH_BUFFER_SIZE                           512

bool readBlob() {
    esp_err_t err;
    nvs_handle_t handle;
    size_t length = 0;
    uint8_t data[1024] = { 0 };
    printf("READ\n");
    if(!device.initDataNVS()) { return false; }
    if(nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle) != ESP_OK) { return false; }

    // read length
    err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, NULL, &length); // 0ms
    if(err == ESP_ERR_NVS_NOT_FOUND) { // first time, blob is empty
        length = 0;
        printf("NOT FOUND\n");
    }
    else if(err != ESP_OK) { printf("ERROR %s\n", esp_err_to_name(err)); nvs_close(handle); return false; }
    printf("LENGTH: %d\n", length);

    // read blob
    if(length > 0) {
        err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, data, &length);
        if(err != ESP_OK) { printf("ERROR %s\n", esp_err_to_name(err)); nvs_close(handle); return false; }
        for(uint16_t i=0; i<length; i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");
    }
    nvs_close(handle);
    return true;
}

bool setBlob(uint16_t newLength) {
    nvs_handle_t handle;
    uint8_t data[1024] = { 0 };

    for(uint16_t i=0; i<newLength; i++) {
        data[i] = i;
    }

    printf("WRITE %d\n", newLength);
    if(!device.initDataNVS()) { return false; }
    if(nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle) != ESP_OK) { printf("ERROR\n"); return false; }
    if(nvs_set_blob(handle, NVS_FLASH_BUFFER_NAME, data, newLength) != ESP_OK) { printf("ERROR\n"); nvs_close(handle); return false; } // TIME CONSUMING: 7 - 117ms
    if(nvs_commit(handle) != ESP_OK) { printf("ERROR\n"); nvs_close(handle); return false; } // 0ms
    nvs_close(handle); // 0ms 
    return true;
}

extern "C" void app_main() {
    while(1) {
        printf("--- %d ---\n", bootCnt);
        device.printDataNVSStats();
        if(bootCnt == 0) {
            device.delay(6000);
            if(!readBlob()) { printf("FAILED\n"); }
        }
        else if(bootCnt == 1) {
            if(!setBlob(0)) { printf("FAILED\n"); }
        }
        else if(bootCnt == 2) {
            if(!readBlob()) { printf("FAILED\n"); }
        }
        else if(bootCnt == 3) {
            if(!setBlob(199)) { printf("FAILED\n"); }
        }
        else if(bootCnt == 4) {
            if(!readBlob()) { printf("FAILED\n"); }
        }
        else {
            printf("NOTHING\n");
        }
        device.delay(10);
        device.enableInternalTimerInterruptInDeepSleep(10);
        bootCnt++;
        device.deepSleep();
    }
}
