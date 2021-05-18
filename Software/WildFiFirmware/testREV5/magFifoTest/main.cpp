#include "PlatformWildFiTagREV5.h"
#include <math.h>

WildFiTagREV5 device = WildFiTagREV5();

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint8_t lastErrorId = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;
RTC_DATA_ATTR uint16_t fifoDataPointerRam = 0;
RTC_DATA_ATTR bmm150_trim_registers trimData = {};
RTC_DATA_ATTR uint32_t bootCnt = 0;

#define TRACKING_DATA_MODE_1HZ_GPS_AND_ACC              0
#define TRACKING_DATA_MODE_ACC_ONLY                     1
#define TRACKING_DATA_MODE                              TRACKING_DATA_MODE_ACC_ONLY
#define ACC_INTERRUPT_WATERMARK                         960

/** Acc configuration setting */
#define ACC_FREQUENCY                                   BMX160_ACCEL_ODR_25HZ
acc_config_t accConfig = {
                                                        ACC_FREQUENCY,                  // BMX160_ACCEL_ODR_50HZ, acc frequency
                                                        BMX160_ACCEL_BW_RES_AVG8,       // BMX160_ACCEL_BW_RES_AVG8, acc averaging cycles
                                                        BMX160_ACCEL_RANGE_8G           // BMX160_ACCEL_RANGE_2G, acc range (WARNING: changes meaning of LSB value in data)
};

mag_config_t magConfig = {                              BMX160_MAG_ODR_25HZ,
                                                        BMX160_MAG_ACCURACY_REGULAR
};

gyro_config_t gyroConfig = {                            BMX160_GYRO_ODR_25HZ,
                                                        BMX160_GYRO_RANGE_125_DPS,
                                                        BMX160_GYRO_BW_NORMAL_MODE
};

void startIMU(bool readTrimData) {
    device.sensorPowerOn();
    device.shortLightSleep(120);
    if(readTrimData) {
        if(!device.imu.magCompensateReadTrimData(&trimData)) { printf("ERROR READ TRIM REGISTERS\n"); }
        device.imu.magCompensatePrintTrimData(&trimData);
    }
    if(!device.imu.start(&accConfig, &magConfig, &gyroConfig, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 1; errorCnt++; }
    if(TRACKING_DATA_MODE != TRACKING_DATA_MODE_1HZ_GPS_AND_ACC) {
        if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 2; errorCnt++; }
    }
    if(!device.imu.initFIFO(BMX160_INIT_FIFO_FOR_ACC | BMX160_INIT_FIFO_FOR_MAG | BMX160_INIT_FIFO_FOR_GYRO)) { lastErrorId = 3; errorCnt++; }
    if(!device.imu.resetFIFO()) { lastErrorId = 113; errorCnt++; }
    fifoDataPointerRam = 0; // reset RAM data
}

void stopIMU() {
    if(!device.imu.stop()) { lastErrorId = 17; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
    device.sensorPowerOff();
    device.shortLightSleep(100); // wait because otherwise interrupt pin might still be valid
}

extern "C" void app_main() {
    while(1) {
        if(bootCnt == 0) {
            // IMU and BARO powered on for lower deep sleep current
            i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
            startIMU(true);
            device.keepSensorPowerOnInDeepSleep();

            // TEST if IMU stays on in light sleep
            printf("SLEEPIN FOR 10 SECONDS\n");
            device.shortLightSleep(5000);
            uint16_t currentFifoLen = device.imu.getFIFOLength();
            printf("FIFOLEN %d\n", currentFifoLen);
            if(!device.imu.resetFIFO()) { printf("ERROR RESET\n"); }
        }
        else if(bootCnt % 10 == 3) {
            i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
            printf("POWER DOWN\n");
            stopIMU();
            device.enableInternalTimerInterruptInDeepSleep(20);
        }
        else if(bootCnt % 10 == 4) {
            i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
            printf("POWER UP\n");
            startIMU(false);
        }
        else {
            uint8_t fifoData[1024] = { 0 };
            i2c.begin(I2C_FREQ_HZ_1MHZ); // 1ms
            uint16_t currentFifoLen = device.imu.getFIFOLength();
            if(currentFifoLen >= 996) { lastErrorId = 118; errorCnt++; } // data loss possible

            if(!device.imu.readGeneralFIFOInOneGoFast(true, true, true, fifoData, currentFifoLen, false)) { lastErrorId = 117; errorCnt++; }

            printf("Fifo: %d bytes\n", currentFifoLen);
            if(!device.imu.magCompensateFifoData(fifoData, currentFifoLen, BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO, &trimData)) { printf("ERROR\n"); }
            if(!device.imu.printFifoData(fifoData, currentFifoLen, BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO, accConfig.range, gyroConfig.range)) { printf("ERROR\n"); }
            printf("-----%d-----\n", lastErrorId);
        }
        printf("lastErrorId: %d, errorCnt: %d\n", lastErrorId, errorCnt);
        bootCnt++;
        device.enableAccInterruptInDeepSleep();
        device.deepSleep();
    }
}
