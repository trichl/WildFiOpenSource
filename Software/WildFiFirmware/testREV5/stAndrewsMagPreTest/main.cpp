#include "PlatformWildFiTagREV5.h"
#include <math.h>

WildFiTagREV5 device = WildFiTagREV5();

#define MAG_ITERATIONS                                  1
#define MAG_SAMPLE_RATE_SECONDS                         1                  
#define MAG_CALIBRATION_AFTER_START                     true
#define MAG_CALIBRATION_TIME_SECONDS                    40

// BMX160_MAG_ACCURACY_LOW_POWER                        205uA -> new after bugfix: 244uA
// BMX160_MAG_ACCURACY_REGULAR                          505uA -> new after bugfix: 655uA
// BMX160_MAG_ACCURACY_ENHANCED                         811uA -> new after bugfix: 1.09mA
// BMX160_MAG_ACCURACY_HIGH                             2300uA -> new after bugfix: 3.17mA

mag_config_t magConf =                                  { BMX160_MAG_ODR_12_5HZ, BMX160_MAG_ACCURACY_REGULAR };

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint64_t bootCnt = 0;
RTC_DATA_ATTR float averageMagnitudeSum = 0;
RTC_DATA_ATTR uint64_t sampleCnt = 0;

RTC_DATA_ATTR bmm150_trim_registers trimData = {};
RTC_DATA_ATTR mag_calibration_t magCalibration = {};
struct BMX160Data mag;
uint16_t hall = 0;

float calculateMagnitude(int16_t xInt, int16_t yInt, int16_t zInt) {
    float x = xInt;
    float y = yInt;
    float z = zInt;
    float magnitude = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    return magnitude;
}


extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            bool someError = false;
            device.disableWakeStubNoBootIfVoltageLow();
            
            device.delay(3000);
            //device.shortLightSleep(4000);

            firstStart = false;

            i2c.begin(I2C_FREQ_HZ_400KHZ);

            // init rtc
            if(!device.rtc.disableClockOut()) { printf("ERROR disable clock\n"); someError = true; }

                device.sensorPowerOn(false);
                device.keepSensorPowerOnInDeepSleep();
                device.delay(100); // IMPORTANT: wait some time before starting sensor

            // init mag
            if(!device.imu.magCompensateReadTrimData(&trimData)) { printf("ERROR READ TRIM REGISTERS\n"); someError = true; }
            device.imu.magCompensatePrintTrimData(&trimData);

            if(MAG_CALIBRATION_AFTER_START) {
                device.measureTime("Start Calib");
                if(!device.magnetometerCalibrationMode(MAG_CALIBRATION_TIME_SECONDS, &magCalibration, &trimData)) { printf("ERROR MAG calib\n"); someError = true; }
                device.measureTime("magnetometerCalibrationMode");
                printf("Result Calib: xmin %d, xmax %d, ymin %d, ymax %d, zmin %d, zmax %d\n", magCalibration.xMin, magCalibration.xMax, magCalibration.yMin, magCalibration.yMax, magCalibration.zMin, magCalibration.zMax);
            }
            else {
                if(!device.imu.start(NULL, &magConf, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("ERROR IMU Start\n"); someError = true; }
                device.delay(500); // IMPORTANT: otherwise reading schmu data
            }
            device.delay(100);

            if(someError) {
                device.blinkTimes(5, B_RED);
                printf("ERROR - eternal sleep!\n");
            }
            else {
                device.rtc.setRegularInterrupt(MAG_SAMPLE_RATE_SECONDS);
                device.enableRTCInterruptInDeepSleep();
            }
        }
        else {
            int16_t temperatureBmx = 0, temperature = 0;
            bool error = false;
            i2c.begin(I2C_FREQ_HZ_400KHZ);

            // get timestamp
            uint32_t timestamp = device.rtc.getTimestamp(error);

            // get data: temperature BMX
            device.imu.getTemperature(temperatureBmx);
            temperatureBmx = device.imu.toCelsiusx100(temperatureBmx);

            // get data: temperature baro
            if(device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) {
                if(device.baro.performMeasurement()) {
                    while(!device.baro.measurementFinished()) { ; }
                    if(device.baro.getResults()) {
                        temperature = device.baro.getTemperature(error);
                    }
                    else { printf("Baro err 03\n"); }
                }
                else { printf("Baro err 02\n"); }
            }
            else { printf("Baro err 01\n"); }

            while(!device.imu.magDataReady()) { device.delay(10); }
            if(!device.imu.getData(NULL, &mag, NULL, &hall)) { printf("ACC: error2\n"); }

            int16_t xUncomp = device.imu.magXConvertToMicroTesla(mag.x, &trimData);
            int16_t yUncomp = device.imu.magYConvertToMicroTesla(mag.y, &trimData);
            int16_t zUncomp = device.imu.magZConvertToMicroTesla(mag.z, &trimData);

            int16_t xComp = device.imu.magCompensateXandConvertToMicroTesla(mag.x, hall, &trimData);
            int16_t yComp = device.imu.magCompensateYandConvertToMicroTesla(mag.y, hall, &trimData);
            int16_t zComp = device.imu.magCompensateZandConvertToMicroTesla(mag.z, hall, &trimData);

            float magnitudeNotCompensated = calculateMagnitude(xUncomp, yUncomp, zUncomp);
            float magnitudeCompensated = calculateMagnitude(xComp, yComp, zComp);
            averageMagnitudeSum += magnitudeCompensated;
            
            //printf("Unstored: %llu,%d,%d,%d,%d,\n%d,%d,%d,\n%d,%d,%d\n", bootCnt, timestamp, temperatureBmx, temperature, hall, xUncomp, yUncomp, zUncomp, xComp, yComp, zComp);
            
            printf("%05llu: %.2f, ", bootCnt, magnitudeCompensated);

            int16_t xOffset = (magCalibration.xMin + magCalibration.xMax) / 2;
            int16_t yOffset = (magCalibration.yMin + magCalibration.yMax) / 2;
            int16_t zOffset = (magCalibration.zMin + magCalibration.zMax) / 2;

            float magnitudeCompensatedHardIron = calculateMagnitude(xComp - xOffset, yComp - yOffset, zComp - zOffset);
            printf("%.2f\n", magnitudeCompensatedHardIron);

            //printf("Offs: %d,%d,%d\n", xComp - xOffset, yComp - yOffset, zComp - zOffset);
            //printf("Total Avg Mag: %.2f\n", (averageMagnitudeSum / (MAG_ITERATIONS * bootCnt)));

            device.blinkTimes(1,B_GREEN);

            device.enableRTCInterruptInDeepSleep();
        }
        bootCnt++;
        device.deepSleep();
    }
}
