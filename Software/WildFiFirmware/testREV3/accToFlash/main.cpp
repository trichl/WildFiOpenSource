#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

uint8_t fifoData[1024] = { 0 };
RTC_DATA_ATTR uint16_t startCnt = 0;

// 2.63mA @25Hz ACC, 673ms for read-out @400kHz (OLD I2C DRIVER!)
// 1.45mA @25Hz ACC, 243ms for read-out @1000kHz without printfs
// 195uA @3.12Hz ACC, 243ms for read-out @1000kHz without printfs
// 378uA @6.25Hz ACC, 243ms for read-out @1000kHz without printfs

void info() {
    bool error;
    uint8_t reg;
    device.delay(5);
    reg = device.imu.readRegister(0x02, error);
    printf("ERROR REG: %d\n", reg);
    device.delay(5);
    reg = device.imu.readRegister(0x03, error);
    printf("PMU REG: %d\n", reg);
    device.delay(5);
    reg = device.imu.readRegister(0x47, error);
    printf("FIFO CONF REG: %d\n", reg);
    device.delay(5);
    reg = device.imu.readRegister(BMX160_ACCEL_CONFIG_ADDR, error);
    printf("ACCEL CONF REG: %d\n\n", reg);
    device.delay(5);
}

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        printf("WAKE NUMBER: %d\n", startCnt);
        printf("V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub());
        //gpio_set_drive_capability(PIN_POWER, GPIO_DRIVE_CAP_3); // TEST ONLY
        
        if(startCnt == 0) {
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            device.delay(100);
            info();
            device.imu.init(true, false, false, BMX160_ACCEL_ODR_6_25HZ, BMX160_LATCH_DUR_160_MILLI_SEC);
            device.imu.enableFIFOInterrupt(900);
            device.imu.initFIFOForAcc();
            info();
        }
        else {
            //device.keepSensorPowerOnInDeepSleep();
            i2c.begin();
        }
        bool bmxAlive = device.imu.isAlive();
        printf("BMX ALIVE: %d\n", bmxAlive);

        if(bmxAlive && (startCnt > 0)) {
            uint64_t timeMeas = Arduino::millisWrapper();
            uint16_t fifoReadout = device.imu.readAccFIFOInOneGo(fifoData);
            timeMeas = Arduino::millisWrapper() - timeMeas;
            printf("READOUT TOOK %llu ms\n", timeMeas);
            printf("READOUT LENGTH: %d\n", fifoReadout);
            device.delay(5);
            uint16_t fifoLenNew = device.imu.getFIFOLength();
            printf("BMX FIFO AFTER READ: %d\n", fifoLenNew);
            /*for(uint16_t i=0; i<fifoReadout; i++) {
                printf("%d ", fifoData[i]);
            }*/
            printf("\n");
            //info();
        }
    
        printf("SLEEPY TIME\n\n");
        startCnt++;
        //device.enableInternalTimerInterruptInDeepSleep(4);
        device.enableAccInterruptInDeepSleep();
        device.deepSleep();
    }
}