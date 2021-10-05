#include "PlatformWildFiTagREV6.h"
#include "PlatformAttitudeEstimator.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR uint32_t bootCnt = 0;

/** NVS storage names */
#define NVS_FLASH_WRITE_PAGE_POINTER                    "flashpgpnt"                    // data writing: pointing on current flash page 0 .. 131071
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffpnt"                   // data writing: pointing on current byte in flash page 0 .. 2047

#define NVS_FLASH_SEND_PAGE_POINTER                     "sendpgpnt"                     // data sending: pointing on current flash page 0 .. 131071
#define NVS_FLASH_SEND_PAGE_OFFSET_POINTER              "sendoffpnt"                    // data sending: pointing on current byte in flash page 0 .. 2047

#define NVS_FLASH_TAG_ACTIVATED_BY_WIFI                 "activated"                     // tag has seen activation wifi
#define NVS_OWN_ID                                      "ownid"                         // ID in NVS, only needs one time programming

#define NVS_FLASH_SEND_BLOB_POINTER                     "blobpnt"                       // data sending: pointing on current blob pointer

uint16_t nvsDefaultReadUINT16(const char *key) {
	esp_err_t err;
	uint16_t value = 0; // default 0 if not existing
	/*if(!NVSForDataInitialized) {
		return 0;
	}*/
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition("nvs", "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return 0;
    }
	// read
    err = nvs_get_u16(handle, key, &value);
	nvs_close(handle);
    switch (err) {
        case ESP_OK:
			return value;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            // value not initialized
			return 0;
            break;
        default :
            return 0;
    }
	return 0;
}

bool nvsDefaultWriteUINT16(const char *key, uint16_t val) {
	esp_err_t err;
	/*if(!NVSForDataInitialized) {
		return false;
	}*/
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition("nvs", "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return false;
    }
	// write
	err = nvs_set_u16(handle, key, val); // 6-7ms (!)
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    err = nvs_commit(handle);
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    nvs_close(handle);
	return true;
}

extern "C" void app_main() {
    while(1) {
        printf("--- %d ---\n", bootCnt);
        if(bootCnt == 0) {
            device.delay(6000);
            if(!device.initDataNVS()) { printf("FAILED\n"); }
            uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
            uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
            uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_PAGE_POINTER);
            uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
            uint16_t nvsSendBlobPointer = device.nvsReadUINT16(NVS_FLASH_SEND_BLOB_POINTER);
            uint16_t nvsTest = device.nvsReadUINT16("test");
            printf("DataNVS: flashPageWritePointer: %d, flashPageWriteOffsetPointer: %d, sendPagePointer: %d, sendPageOffsetPointer: %d, nvsSendBlobPointer: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer, sendPagePointer, sendPageOffsetPointer, nvsSendBlobPointer);
            printf("DataNVS: test: %d (should be 0)\n", nvsTest);

            uint64_t time = Timing::millis();
            if(!device.initNVS()) { printf("INIT FAILED!\n"); }
            else {
                time = Timing::millis() - time;
                printf("DefaultNVS: init in %dms\n", (uint32_t) time);
                uint16_t nvsTest = nvsDefaultReadUINT16("test");
                printf("DefaultNVS: Read test: %d\n", nvsTest);
                nvsTest++;
                if(!nvsDefaultWriteUINT16("test", nvsTest)) { printf("WRITE FAILED!\n"); }
            }

        
        }
        else {
            if(!device.initNVS()) { printf("INIT FAILED!\n"); }
            else {
                uint16_t nvsTest = nvsDefaultReadUINT16("test");
                printf("Read test: %d\n", nvsTest);
            }
        }
        device.enableInternalTimerInterruptInDeepSleep(20);
        bootCnt++;
        device.deepSleep();
    }
}
