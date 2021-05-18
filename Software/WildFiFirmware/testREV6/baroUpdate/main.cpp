#include "PlatformWildFiTagREV6.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();

uint64_t t = 0;

extern "C" void app_main() {
    while(1) {
        printf("HELLO\n");
        i2c.begin(I2C_FREQ_HZ_400KHZ);
        device.sensorPowerOn();
        device.shortLightSleep(200);

        t = Timing::millis();
        if(!device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { printf("ERROR INIT\n"); } // 4ms
        t = Timing::millis() - t;
        printf("INIT %llu\n", t);

        while(true) {
            t = Timing::millis();
            if(!device.baro.performMeasurement()) { printf("ERROR PERFORM\n"); } // 3ms
            t = Timing::millis() - t;
            printf("%llu\n", t);

            device.shortLightSleep(50);

            t = Timing::millis();
            if(!device.baro.getResults()) { printf("ERROR GET\n"); } // 31ms
            t = Timing::millis() - t;
            printf("%llu\n", t);

            bool error = false;
            int16_t temperature = device.baro.getTemperature(error);
            if(error) { printf("ERROR GET DATA\n"); }
            uint32_t pressure = device.baro.getPressure(error);
            if(error) { printf("ERROR GET DATA\n"); }
            uint32_t humidity = device.baro.getHumidity(error);
            if(error) { printf("ERROR GET DATA\n"); }

            printf("%d Data: TEMP: %d, PRESS: %d, HUMI: %d\n", ((uint32_t) Timing::millis()), temperature, pressure, humidity);


            device.shortLightSleep(800);
        }



        device.delay(100);
        device.deepSleep();
    }
}
