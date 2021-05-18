#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        // strange: without anything POWER pulled up to 3.03V in deep sleep (by whom?!)
        // strange 2: set POWER = 0 INCREASES power consumption!
        // -> POWER & SDA/SCL seems to be SHORTED!!
        device.setCPUSpeed(ESP32_10MHZ);
        device.delay(2000);
        gpio_hold_dis(PIN_POWER);
        device.powerAndInitI2C();
        device.rtc.setRegularInterrupt(10);
        device.enableRTCInterruptInDeepSleep();
        printf("HELLO\n");
        device.delay(2000);
        device.sensorPowerOff();
        device.delay(2000);
        device.sensorPowerOn();
        gpio_hold_en(PIN_POWER);
        gpio_deep_sleep_hold_en();
        
        device.deepSleep(); // 26.6uA in deep sleep -> maybe ACC is shitty with current hardware sample
    }
}