#include "ESP32TrackerREV3.h"

#define NVS_FLASH_POINTER                               "flashpnt"

ESP32TrackerREV3 device = ESP32TrackerREV3();

RTC_DATA_ATTR bool firstStart = true;
uint32_t t = 0;
void measureTime(const char* text) {
    uint64_t timeNow = ((uint32_t) Arduino::millisWrapper());
    t = timeNow - t;
    printf("%s TOOK: %d ms\n", text, t);
    t = (uint32_t) Arduino::millisWrapper();
}

//const char* NVS_PARTITION = "nvs2";
const char* NVS_PARTITION = "nvs";

extern "C" void app_main() {
    while(1) {
        esp_err_t error;
        if(firstStart) {
            //device.setCPUSpeed(ESP32_10MHZ);
            device.sensorPowerOn(true); // for I2C
            device.keepSensorPowerOnInDeepSleep();
            device.rtc.setRegularInterrupt(4); 
            firstStart = false;
            measureTime("reset");
            error = nvs_flash_erase_partition(NVS_PARTITION);
            measureTime("nvs_flash_erase_partition");
            printf("RES: %s\n", esp_err_to_name(error));
            device.delay(5000);
        }
        printf("RESET REASON: %d\n", device.getLastResetReason());
        printf("WAKE FROM DEEP SLEEP: %d\n", device.getWakeUpReason());
        
        uint32_t val = 0;
        measureTime("booted");
        error = nvs_flash_init_partition(NVS_PARTITION);
        measureTime("nvs_flash_init_partition");
        printf("RES: %s\n", esp_err_to_name(error));

        nvs_handle_t handle;
        error = nvs_open_from_partition(NVS_PARTITION, "storage", NVS_READWRITE, &handle);
        measureTime("nvs_open");
        printf("RES: %s\n", esp_err_to_name(error));
        
        error = nvs_get_u32(handle, "test", &val);
        measureTime("nvs_get_u32");
        printf("RES: %s\n", esp_err_to_name(error));
        
        val++;

        error = nvs_set_u32(handle, "test", val);
        measureTime("nvs_set_u32");
        printf("RES: %s\n", esp_err_to_name(error));

        error = nvs_commit(handle);
        measureTime("nvs_commit");
        printf("RES: %s\n", esp_err_to_name(error));

	    nvs_close(handle);
        measureTime("nvs_close");

        printf("VAL: %d\n", val);

        nvs_stats_t nvs_stats;
        nvs_get_stats(NVS_PARTITION, &nvs_stats);
        printf("Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)\n", nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);

        printf("-----------\n");


        /*measureTime("booted");
        device.initNVS();
        measureTime("initNVS");
        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_POINTER);
        measureTime("nvsReadUINT32");
        flashPointer++;
        device.nvsWriteUINT32(NVS_FLASH_POINTER, flashPointer);
        measureTime("nvsWriteUINT32");
        printf("FLASH POINTER: %d\n", flashPointer);*/

        
        device.enableRTCInterruptInDeepSleep();
        device.deepSleep();
    }
}