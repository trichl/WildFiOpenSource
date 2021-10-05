#include "PlatformWildFiTagREV6.h"
#include "PlatformAttitudeEstimator.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();
PlatformAttitudeEstimator attEst = PlatformAttitudeEstimator();

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint8_t lastErrorId = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;
RTC_DATA_ATTR uint16_t fifoDataPointerRam = 0;
RTC_DATA_ATTR bmm150_trim_registers trimData = {};
RTC_DATA_ATTR uint32_t bootCnt = 0;

#define TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP           0
#define TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP            1
#define TRACKING_DATA_MODE                              TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP
#define ACC_INTERRUPT_WATERMARK                         960
#define MODE_TESTRUN                                    0
#define TRACKER_MODE                                    MODE_TESTRUN

/** Acc configuration setting */
#define ACC_FREQUENCY                                   BMX160_ACCEL_ODR_50HZ
acc_config_t accConfig = {
                                                        ACC_FREQUENCY,                  // BMX160_ACCEL_ODR_50HZ, acc frequency
                                                        BMX160_ACCEL_BW_RES_AVG1,       // BMX160_ACCEL_BW_RES_AVG8, acc averaging cycles
                                                        BMX160_ACCEL_RANGE_8G           // BMX160_ACCEL_RANGE_2G, acc range (WARNING: changes meaning of LSB value in data)
};

#define USE_MAGNETOMETER                                false
mag_config_t magConfig = {                              BMX160_MAG_ODR_25HZ,
                                                        BMX160_MAG_ACCURACY_REGULAR
};

#define USE_GYRO                                        false
gyro_config_t gyroConfig = {                            BMX160_GYRO_ODR_25HZ,
                                                        BMX160_GYRO_RANGE_125_DPS,
                                                        BMX160_GYRO_BW_NORMAL_MODE
};

void startIMU(bool readTrimData) {
    device.sensorPowerOn();
    device.shortLightSleep(120); // wait until bmx is booted
    if(readTrimData) {
        if(TRACKER_MODE == MODE_TESTRUN) { printf("startIMU: reading trim data\n"); }
        if(!device.imu.magCompensateReadTrimData(&trimData)) { lastErrorId = 153; errorCnt++; }
        if(TRACKER_MODE == MODE_TESTRUN) { device.imu.magCompensatePrintTrimData(&trimData); }
    }
    mag_config_t *magConfigPointer = NULL;
    gyro_config_t *gyroConfigPointer = NULL;
    if(USE_MAGNETOMETER) { magConfigPointer = &magConfig; }
    if(USE_GYRO) { gyroConfigPointer = &gyroConfig; }
    if(!device.imu.start(&accConfig, magConfigPointer, gyroConfigPointer, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 1; errorCnt++; }
    if((TRACKING_DATA_MODE == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) || (TRACKING_DATA_MODE == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP)) {
        if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 2; errorCnt++; }
    }
    uint8_t fifoForWhat = BMX160_INIT_FIFO_FOR_ACC;
    if(USE_MAGNETOMETER) { fifoForWhat |= BMX160_INIT_FIFO_FOR_MAG; }
    if(USE_GYRO) { fifoForWhat |= BMX160_INIT_FIFO_FOR_GYRO; }
    if(!device.imu.initFIFO(fifoForWhat)) { lastErrorId = 3; errorCnt++; }
    if(!device.imu.resetFIFO()) { lastErrorId = 113; errorCnt++; }
    fifoDataPointerRam = 0; // reset RAM data
}

void stopIMU() {
    if(!device.imu.stop()) { lastErrorId = 17; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
    device.sensorPowerOff(); // turn off IMU (and environment sensor) completely
    device.shortLightSleep(100); // wait because otherwise interrupt pin might still be valid
}

void readFifoIMU(uint8_t *data, uint16_t len) {
    if(!device.imu.readGeneralFIFOInOneGoFast(true, USE_MAGNETOMETER, USE_GYRO, data, len, false)) { lastErrorId = 116; errorCnt++; }
    if(USE_MAGNETOMETER) {
        bmx160_fifo_dataset_len_t datasetStructure = BMX160_FIFO_DATASET_LEN_ACC_AND_MAG;
        if(USE_GYRO) { datasetStructure = BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO; }
        if(!device.imu.magCompensateFifoData(data, len, datasetStructure, &trimData)) { lastErrorId = 62; errorCnt++;  }
    }
}

extern "C" void app_main() {
    while(1) {
        if(bootCnt == 0) {
            // IMU and BARO powered on for lower deep sleep current
            i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time

            device.delay(6000);

            // set RTC to wake up on minute change
            if(!device.rtc.setTimeUpdateInterruptMinuteChange()) { printf("ERROR\n"); }
            bool error = false;
            bool rtcInterruptHappened = device.rtc.timeUpdateInterruptMinuteChangeHappened(error);
            printf("RTC INT HAPPENED: %d\n", rtcInterruptHappened);

            // start IMU and configure wake up on FIFO watermark
            device.keepSensorPowerOnInDeepSleep();
            startIMU(false);

        }
        else {
            esp_sleep_wakeup_cause_t wakeupSrc = esp_sleep_get_wakeup_cause();

            i2c.begin(I2C_FREQ_HZ_1MHZ); // 1ms
            uint32_t timestamp = 0;
            uint8_t hundreds = 0;
            if(!device.rtc.getTimestamp(&timestamp, &hundreds)) { printf("RTC_ERROR\n"); }
            uint16_t mil = hundreds * 10;
            bool error = false;
            bool rtcInterruptHappened = device.rtc.timeUpdateInterruptMinuteChangeHappened(error);
            printf("- %d.%03d (RTC INT %d): ", timestamp, mil, rtcInterruptHappened);

            if(wakeupSrc == ESP_SLEEP_WAKEUP_EXT0) {
                printf("BY RTC\n");
                device.delay(1000);
                device.ultraShortDeepSleep();
            }
            else if(wakeupSrc == ESP_SLEEP_WAKEUP_EXT1) {
                printf("BY ACC, ");
            }
            else { printf("BY INT, "); }

            device.ledGreenOn();
            uint8_t fifoData[1024] = { 0 };
            uint16_t currentFifoLen = device.imu.getFIFOLength();
            if(currentFifoLen >= 996) { lastErrorId = 118; errorCnt++; } // data loss possible

            readFifoIMU(fifoData, currentFifoLen);
            printf("%d (%d)\n", currentFifoLen, lastErrorId);
            //if(!device.imu.printFifoData(fifoData, currentFifoLen, BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO, accConfig.range, gyroConfig.range)) { printf("ERROR\n"); }
            device.delay(10); // for printf
            device.ledGreenOff();
        }
        device.enableRTCInterruptInDeepSleep(); // ext0
        device.enableAccInterruptInDeepSleep(); // ext1
        bootCnt++;
        device.deepSleep();
    }
}
