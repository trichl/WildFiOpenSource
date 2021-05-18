#include "PlatformWildFiTagREV5.h"
#include <math.h>

WildFiTagREV5 device = WildFiTagREV5();

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint8_t lastErrorId = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;
RTC_DATA_ATTR uint16_t fifoDataPointerRam = 0;
RTC_DATA_ATTR bmm150_trim_registers trimData = {};

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
                                                        BMX160_MAG_ACCURACY_LOW_POWER
};

gyro_config_t gyroConfig = {                            BMX160_GYRO_ODR_25HZ,
                                                        BMX160_GYRO_RANGE_2000_DPS,
                                                        BMX160_GYRO_BW_NORMAL_MODE
};


void startIMU() {
    if(!device.imu.start(&accConfig, &magConfig, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 1; errorCnt++; }
    if(TRACKING_DATA_MODE != TRACKING_DATA_MODE_1HZ_GPS_AND_ACC) {
        if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 2; errorCnt++; }
    }
    if(!device.imu.initFIFO(BMX160_INIT_FIFO_FOR_ACC | BMX160_INIT_FIFO_FOR_MAG)) { lastErrorId = 3; errorCnt++; }
    if(!device.imu.resetFIFO()) { lastErrorId = 113; errorCnt++; }
    fifoDataPointerRam = 0; // reset RAM data
}

void stopIMU() {
    if(!device.imu.stop()) { lastErrorId = 17; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
    // soft reset?
}

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            firstStart = false;
            device.sensorPowerOn(false);
            i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
            device.keepSensorPowerOnInDeepSleep();
            device.delay(100);
            if(!device.imu.start(&accConfig, &magConfig, &gyroConfig, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("ERROR\n"); }
            //device.delay(3000);
            //if(!device.selfTest(3750, SELFTEST_ACC_GYRO_FOC_CHECK | SELFTEST_ACC_GYRO_FOC_EXECUTE_IF_UNSET)) { printf("ERROR\n"); }
        }
        else {
            i2c.begin(I2C_FREQ_HZ_400KHZ);
            BMX160Data gyroData = { };
            BMX160Data accData = { };
            device.imu.getData(&accData, NULL, &gyroData, NULL);
            printf("Acc: %f, %f, %f\n", device.imu.accRawToG(accData.x, BMX160_ACCEL_RANGE_8G), device.imu.accRawToG(accData.y, BMX160_ACCEL_RANGE_8G), device.imu.accRawToG(accData.z, BMX160_ACCEL_RANGE_8G));
            printf("Gyro RAW: %d, %d, %d\n", gyroData.x, gyroData.y, gyroData.z);
            printf("Gyro: %f, %f, %f\n", device.imu.gyroRawToDegreePerSecond(gyroData.x, BMX160_GYRO_RANGE_2000_DPS), device.imu.gyroRawToDegreePerSecond(gyroData.y, BMX160_GYRO_RANGE_2000_DPS), device.imu.gyroRawToDegreePerSecond(gyroData.z, BMX160_GYRO_RANGE_2000_DPS));
        }
        device.enableInternalTimerInterruptInDeepSleep(3);
        device.deepSleep();
    }
}
