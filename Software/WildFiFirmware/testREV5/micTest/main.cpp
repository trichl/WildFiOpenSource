#include "PlatformWildFiTagREV5.h"

WildFiTagREV5 device = WildFiTagREV5();

RTC_DATA_ATTR uint32_t testData[512] =  { 0 };
uint32_t testData2[512] = { 0 };

extern "C" void app_main() {
    while(1) {
        //device.setCPUSpeed(ESP32_10MHZ);
        printf("AWAKE!\n");

        if(!rtc_gpio_is_valid_gpio(PIN_GPIO_A)) { while(1) { ; } }
        rtc_gpio_init(PIN_GPIO_A);
        rtc_gpio_set_direction(PIN_GPIO_A, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(PIN_GPIO_A, LOW);
        device.delay(6000);
        rtc_gpio_set_level(PIN_GPIO_A, HIGH);
        device.delay(3000);
        rtc_gpio_set_level(PIN_GPIO_A, LOW);


        
        printf("SLEEPING FOREVER\n");
        device.deepSleep();
    }
}