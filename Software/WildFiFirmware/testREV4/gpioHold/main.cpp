#include "ESP32TrackerREV4.h"

ESP32TrackerREV4 device = ESP32TrackerREV4();

RTC_DATA_ATTR uint32_t wakeCnt = 0;

extern "C" void app_main() {
    while(1) {
        printf("HELLO WORLD NO %d, V = %d!\n", wakeCnt, device.readSupplyVoltageFromWakeStub());
        device.blink(B_RED, B_RED, B_RED);
        device.blink(B_GREEN, B_GREEN, B_GREEN);

        if(wakeCnt == 4) {
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            device.delay(100);
            printf("ON\n");
        }
        else if(wakeCnt == 5) {
            i2c.begin(I2C_FREQ_HZ_1MHZ);
            acc_config_t accConfigForFOC = { BMX160_ACCEL_ODR_25HZ, BMX160_ACCEL_BW_RES_AVG8, BMX160_ACCEL_RANGE_8G };
		    if(!device.imu.start(&accConfigForFOC, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("..FAILED (start)\n"); }
            if(!device.imu.initFIFOForAcc()) { printf("..FAILED (fifo)\n"); }
            device.delay(100);
            uint16_t fifoLen = device.imu.getFIFOLength();
            printf("STARTED, LEN: %d\n", fifoLen);
        }
        else if(wakeCnt > 5) {
            i2c.begin(I2C_FREQ_HZ_1MHZ);
            uint16_t fifoLen = device.imu.getFIFOLength();
            printf("STARTED, LEN: %d\n", fifoLen);
            if(fifoLen == 996) {
                device.imu.resetFIFO();
                 printf("FIFO RESET\n");
            }
        }

        device.enableInternalTimerInterruptInDeepSleep(5);
        wakeCnt++;
        device.deepSleep();
    }
}