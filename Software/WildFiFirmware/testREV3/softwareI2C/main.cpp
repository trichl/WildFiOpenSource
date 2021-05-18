#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

RTC_DATA_ATTR uint16_t startCnt = 0;
RTC_DATA_ATTR uint16_t fifoLen = 0;
RTC_DATA_ATTR uint8_t fifoData[984*4] = { 0 };
RTC_DATA_ATTR uint16_t fifoDataPointer = 0;

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

void test() {
    uint8_t result;
    beginSw();
    beginTransmissionSw(0x69);
    result = endTransmissionSw();
    printf("ALIVE 0x69: %d\n", !result);
    beginTransmissionSw(0x50);
    result = endTransmissionSw();
    printf("ALIVE 0x50: %d\n", !result);
}

RTC_DATA_ATTR bool wakeStub() {
    beginSw();
    fifoLen = getAccFIFOLength();
    readAccFIFO(fifoData+fifoDataPointer, fifoLen, false);
    fifoDataPointer += fifoLen;

    if(fifoDataPointer > 984*3) {
        return true;
    }
    return false;
}

extern "C" void app_main() {
    while(1) {
        //device.setCPUSpeed(ESP32_10MHZ);
        printf("WAKE NUMBER: %d\n", startCnt);
        printf("V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub());
        
        if(startCnt == 0) {
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();

            //device.ledRedOff();
            //gpio_hold_en(PIN_LED_RED); // IF NOT THEN +140uA!!!!
            device.delay(100);
            device.imu.init(true, false, false, BMX160_ACCEL_ODR_25HZ, BMX160_LATCH_DUR_160_MILLI_SEC);
            device.imu.enableFIFOInterrupt(984);
            device.imu.initFIFOForAcc();

            device.customWakeStubFunction(wakeStub);
            device.setWakeStubRejectionInterruptSrc(USE_EXT1_IF_WAKE_UP_REJECTED);

            info();
        }
        else {
            printf("last_ack: %d\n", last_ack);
            printf("readQuantitySw: %d\n", readQuantitySw);
            printf("fifoLen: %d\n", fifoLen);
            printf("fifoDataPointer: %d\n", fifoDataPointer);

            for(uint16_t i=0; i<fifoDataPointer; i++) {
                printf("%02X ", fifoData[i]);
            }
            printf("\n");

            fifoDataPointer = 0;
        }
        
        printf("SLEEPY TIME\n\n");
        startCnt++;
        device.enableAccInterruptInDeepSleep();
        device.deepSleep();
    }
}