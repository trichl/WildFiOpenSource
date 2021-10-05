#include "PlatformWildFiTagREV6.h"
#include "PlatformAttitudeEstimator.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR uint32_t bootCnt = 0;



#define CONFIGURATION_LENGTH            100
#define CONFIGURATION_NAME_NVS          "configProx"

RTC_DATA_ATTR uint8_t configuration[CONFIGURATION_LENGTH] = { 0 };

bool readConfigurationFromNVS(uint8_t *conf) {
    esp_err_t err;
    nvs_handle_t handle;
    size_t confLength = 0;
    if(!device.initDataNVS()) { return false; }
    if(nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle) != ESP_OK) { return false; }
    err = nvs_get_blob(handle, CONFIGURATION_NAME_NVS, conf, &confLength);
    if(err == ESP_ERR_NVS_NOT_FOUND) { // first time configuration, blob empty
        // use default values?
    }
    else if(err != ESP_OK) { nvs_close(handle); return false; }


    nvs_close(handle);

    return true;
}

extern "C" void app_main() {
    while(1) {
        i2c.begin(I2C_FREQ_HZ_1MHZ);
        if(bootCnt == 0) {
            printf("HELLO WORLD\n");
            device.delay(6000);

            device.keepSensorPowerOnInDeepSleep();
            acc_config_t accConfig = { BMX160_ACCEL_ODR_25HZ, BMX160_ACCEL_BW_RES_AVG8, BMX160_ACCEL_RANGE_8G }  ;
            device.sensorPowerOn(); 
            device.shortLightSleep(120); // wait until bmx is booted

            uint64_t t = Timing::millis();
            if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("ERROR\n"); }
            if(!device.imu.initFIFO(BMX160_INIT_FIFO_FOR_ACC)) { printf("ERROR\n"); }
            if(!device.imu.resetFIFO()) { printf("ERROR\n"); }
            t = Timing::millis() - t;

            printf("IMU init took: %d ms\n", (uint32_t)(t));

        }
        else {
            printf("HI\n");
            uint8_t fifoData[1024];
            uint16_t currentFifoLen = device.imu.getFIFOLength();
            if(!device.imu.readGeneralFIFOInOneGoFast(true, false, false, fifoData, currentFifoLen, false)) { printf("ERROR\n"); }
            printf("FIFOLEN: %d\n", currentFifoLen);
        }
        device.enableInternalTimerInterruptInDeepSleep(10);
        bootCnt++;
        device.deepSleep();
    }
}
