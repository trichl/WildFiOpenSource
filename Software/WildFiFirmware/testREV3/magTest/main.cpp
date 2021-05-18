#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();
RTC_DATA_ATTR uint64_t bootCnt = 0;
struct BMX160Data mag;
uint16_t hall = 0;

mag_config_t magConf = { BMX160_MAG_ODR_1_56HZ };

RTC_DATA_ATTR bmm150_trim_registers trimData = {};

extern "C" void app_main() {
    while(1) {
        if(bootCnt == 0) {
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            device.delay(100); // IMPORTANT: wait some time before starting sensor
            device.measureTime("boot");

            if(!device.imu.magCompensateReadTrimData(&trimData)) { printf("ERROR TRIM REGISTERS\n"); }
            device.imu.magCompensatePrintTrimData(&trimData);

            if(!device.imu.start(NULL, &magConf, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("ACC: error\n"); }
            device.measureTime("start");
        }
        else {
            i2c.begin(I2C_FREQ_HZ_1MHZ);
        }
        if(!device.imu.getData(NULL, &mag, NULL, &hall)) { printf("ACC: error2\n"); }
        printf("Raw\t%d\t%d\t%d\t%d\n", mag.x, mag.y, mag.z, hall);
        printf("Mag\t%d\t%d\t%d\t%d\n", device.imu.toMicroTesla(mag.x), device.imu.toMicroTesla(mag.y), device.imu.toMicroTesla(mag.z), hall);
        printf("Comp\t%d\t%d\t%d\n\n", device.imu.magCompensateXandConvertToMicroTesla(mag.x, hall, &trimData), device.imu.magCompensateYandConvertToMicroTesla(mag.y, hall, &trimData), device.imu.magCompensateZandConvertToMicroTesla(mag.z, hall, &trimData));

        if(bootCnt == 20) {
            //printf("-> STOPPING\n");
            //if(!device.imu.softReset(true)) { printf("ERROR STOP\n"); }
        }

        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(5);
        device.deepSleep();
    }
}