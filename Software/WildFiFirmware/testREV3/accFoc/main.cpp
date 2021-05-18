#include "ESP32TrackerREV3.h"
#include "math.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

// TODO: FUNCTION: check if FOC already done for acc

RTC_DATA_ATTR bool firstStart = true;

uint32_t t = 0;
void measureTime(const char* text) {
    uint64_t timeNow = ((uint32_t) Arduino::millisWrapper());
    t = timeNow - t;
    printf("%s TOOK: %d ms\n", text, t);
    t = (uint32_t) Arduino::millisWrapper();
}

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            bool error = false;
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            device.rtc.setRegularInterrupt(10);
            printf("Start\n");
            firstStart = false;

            device.delay(9000);
            if(!device.imu.init(true, false, false, BMX160_ACCEL_ODR_25HZ, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("ERROR1\n"); }
            device.delay(100);

            printf("0x77: %02X\n", device.imu.readRegister(0x77, error));
            printf("0x76: %02X\n", device.imu.readRegister(0x76, error));
            printf("0x75: %02X\n", device.imu.readRegister(0x75, error));
            printf("0x74: %02X\n", device.imu.readRegister(0x74, error));
            printf("0x73: %02X\n", device.imu.readRegister(0x73, error));
            printf("0x72: %02X\n", device.imu.readRegister(0x72, error));
            printf("0x71: %02X\n\n", device.imu.readRegister(0x71, error));

            printf("FOC already done and stored in NVM: %d\n",device.imu.accFOCAlreadyDoneAndStoredInNVM());

            /*if(!device.imu.startAccFOCAndStoreInNVM()) { printf("ERROR2\n"); }

            printf("0x77: %02X\n", device.imu.readRegister(0x77, error));
            printf("0x76: %02X\n", device.imu.readRegister(0x76, error));
            printf("0x75: %02X\n", device.imu.readRegister(0x75, error));
            printf("0x74: %02X\n", device.imu.readRegister(0x74, error));
            printf("0x73: %02X\n", device.imu.readRegister(0x73, error));
            printf("0x72: %02X\n", device.imu.readRegister(0x72, error));
            printf("0x71: %02X\n\n", device.imu.readRegister(0x71, error));*/

            if(!device.imu.initFIFOForAcc()) { printf("ERROR\n"); }
        }
        else {
            i2c.begin();
            uint8_t fifoData[1024] = {0};
            uint16_t fifoLen = device.imu.readAccFIFOInOneGo(fifoData);
            printf("READING %d Bytes\n", fifoLen);
            uint16_t i=0;
            while(i<fifoLen/6) {
                int16_t x = (int16_t) ((fifoData[i+1] << 8) | fifoData[i]);
	            int16_t y = (int16_t) ((fifoData[i+3] << 8) | fifoData[i+2]);
	            int16_t z = (int16_t) ((fifoData[i+5] << 8) | fifoData[i+4]);
                printf("X = %d, Y = %d, Z = %d\n", x, y, z);

                double xf = x, yf = y, zf = z, magnitude = 0;
                const double factor = 0.00006103515;
                xf *= factor; yf *= factor; zf *= factor;
                magnitude = sqrt(pow(xf, 2) + pow(yf, 2) + pow(zf, 2));
                printf("DOUBLE: X = %f, Y = %f, Z = %f, Magnitude = %f\n", xf, yf, zf, magnitude);
                i += 6;
            }
        }

        
        device.enableRTCInterruptInDeepSleep();
        device.deepSleep();
    }
}