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

void readFifoIMU(uint8_t *data, uint16_t len) {
    if(!device.imu.readGeneralFIFOInOneGoFast(true, true, true, data, len, false)) { lastErrorId = 116; errorCnt++; }
    if(!device.imu.magCompensateFifoData(data, len, BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO, &trimData)) { lastErrorId = 62; errorCnt++;  }
}

RTC_DATA_ATTR mag_calibration_t magCalibration = {};
RTC_DATA_ATTR int16_t magHardIronOffsetX = 0;
RTC_DATA_ATTR int16_t magHardIronOffsetY = 0;
RTC_DATA_ATTR int16_t magHardIronOffsetZ = 0;

extern "C" void app_main() {
    while(1) {
        if(bootCnt == 0) {
            // IMU and BARO powered on for lower deep sleep current
            i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
            device.keepSensorPowerOnInDeepSleep();

            startIMU(true);
            stopIMU();

            device.sensorPowerOn();
            device.shortLightSleep(120);
            device.measureTime("Start Calib");
            if(!device.magnetometerCalibrationMode(20, &magCalibration, &trimData)) { printf("ERROR MAG calib\n"); }
            device.measureTime("magnetometerCalibrationMode");
            printf("Result Calib: xmin %d, xmax %d, ymin %d, ymax %d, zmin %d, zmax %d\n", magCalibration.xMin, magCalibration.xMax, magCalibration.yMin, magCalibration.yMax, magCalibration.zMin, magCalibration.zMax);

            magHardIronOffsetX = (magCalibration.xMin + magCalibration.xMax) / 2;
            magHardIronOffsetY = (magCalibration.yMin + magCalibration.yMax) / 2;
            magHardIronOffsetZ = (magCalibration.zMin + magCalibration.zMax) / 2;

            stopIMU();

            startIMU(false);

            // Initialise the estimator (e.g. in the class constructor, none of these are actually strictly required for the estimator to work, and can be set at any time)
            //attEst.est.setMagCalib(0.68, -1.32, 0.0);           // Recommended: Use if you want absolute yaw information as opposed to just relative yaw (Default: (1.0, 0.0, 0.0))
            //attEst.est.setPIGains(2.2, 2.65, 10, 1.25);       // Recommended: Use if the default gains (shown) do not provide optimal estimator performance (Note: Ki = Kp/Ti)
            //attEst.est.setQLTime(2.5);                        // Optional: Use if the default quick learning time is too fast or too slow for your application (Default: 3.0)
            //attEst.est.setAttitude(0.5, 0.5, 0.5, 0.5);       // Optional: Use if you have prior knowledge about the orientation of the robot (Default: Identity orientation)
            //attEst.est.setAttitudeEuler(M_PI, 0.0, 0.0);      // Optional: Use if you have prior knowledge about the orientation of the robot (Default: Identity orientation)
            //attEst.est.setAttitudeFused(M_PI, 0.0, 0.0, 1.0); // Optional: Use if you have prior knowledge about the orientation of the robot (Default: Identity orientation)
            //attEst.est.setGyroBias(0.152, 0.041, -0.079);     // Optional: Use if you have prior knowledge about the gyroscope bias (Default: (0.0, 0.0, 0.0))
            //attEst.est.setAccMethod(Est.ME_FUSED_YAW);        // Optional: Use if you wish to experiment with varying acc-only resolution methods

            // REMOVE! TEST!
            /*if(!device.flashPowerOn(true)) { printf("ERROR\n"); } // turn on flash power already (5ms)
            uint8_t fifoData[1536] = { 0 };
            for(uint16_t i=0; i<1536; i++) { fifoData[i] = 0xFF; }
            uint32_t timeMeas = (uint32_t) Timing::millis();
            uint32_t page = 65536;
            uint16_t offset = 0;
            sequential_write_status_t writeStatus = device.flash.fifoPushSimple(0, page, offset, fifoData, 1536, true, false);
            //device.flash.printFlash(page, 1, 2048);
            timeMeas = ((uint32_t) Timing::millis()) - timeMeas;
            printf("FLASH TEST: %d -> %d.%d in %d ms\n", writeStatus, page, offset, timeMeas);
            device.flashPowerOff(true);*/

        }
        else {
            i2c.begin(I2C_FREQ_HZ_1MHZ); // 1ms
            while(true) {
                device.ledGreenOn();
                uint8_t fifoData[1024] = { 0 };
                uint16_t currentFifoLen = device.imu.getFIFOLength();
                if(currentFifoLen >= 996) { lastErrorId = 118; errorCnt++; } // data loss possible

                readFifoIMU(fifoData, currentFifoLen);

                printf("Fifo: %d bytes\n", currentFifoLen);
                //if(!device.imu.printFifoData(fifoData, currentFifoLen, BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO, accConfig.range, gyroConfig.range)) { printf("ERROR\n"); }
                uint32_t timeMeas = (uint32_t) Timing::millis();
                if(!attEst.feed(fifoData, currentFifoLen, 25, accConfig.range, gyroConfig.range, false, magHardIronOffsetX, magHardIronOffsetY, magHardIronOffsetZ, false, 1)) { printf("ERROR2\n"); }
                timeMeas = ((uint32_t) Timing::millis()) - timeMeas;

                printf("Err: %d, T: %dms\n", lastErrorId, timeMeas);
                device.delay(10); // for printf
                device.ledGreenOff();
                device.enableAccInterruptInDeepSleep();
                device.lightSleep();
            }
        }
        printf("lastErrorId: %d, errorCnt: %d\n", lastErrorId, errorCnt);
        bootCnt++;
        device.enableAccInterruptInDeepSleep();
        device.deepSleep();
    }
}
