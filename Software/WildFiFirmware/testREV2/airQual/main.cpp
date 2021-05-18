#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        printf("AWAKE!\n");
        device.initPins();
        device.powerAndInitI2C();
        printf("BME ALIVE: %d\n", device.baro.isAlive());
        if(!device.baro.init()) {
            printf("ERROR\n");
            while(1) {}
        }

        while(1) {
            uint32_t time_trigger = Arduino::millisWrapper(); // beware, returns 64 bit 
            if(device.baro.run()) { // If new data is available
                printf("Timestamp: %d\n", time_trigger);
                printf("Temperature: %f (raw: %f)\n", device.baro.bsec.temperature, device.baro.bsec.rawTemperature);
                printf("Pressure: %f\n", device.baro.bsec.pressure);
                printf("Humidity: %f (raw: %f)\n", device.baro.bsec.humidity, device.baro.bsec.rawHumidity);
                printf("Gas resistance: %f\n", device.baro.bsec.gasResistance);
                printf("Air quality index: %f (accuracy: %d, static: %f)\n", device.baro.bsec.iaq, device.baro.bsec.iaqAccuracy, device.baro.bsec.staticIaq);
                printf("CO2 equivalent: %f\n", device.baro.bsec.co2Equivalent);
                printf("VOC breath equivalent: %f\n", device.baro.bsec.breathVocEquivalent);
                printf("\n");
                device.ledGreenOn();
                device.delay(200);
                device.ledGreenOff();
            }
        }
    }
}