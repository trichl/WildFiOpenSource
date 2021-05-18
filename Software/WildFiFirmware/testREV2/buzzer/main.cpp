#include "ESP32TrackerREV2.h"
#include "driver/ledc.h"
#include <math.h>

ESP32TrackerREV2 device = ESP32TrackerREV2();

RTC_DATA_ATTR float frequency = 1000;

extern "C" void app_main() {
    while(1) {
        // strange: without anything POWER pulled up to 3.03V in deep sleep (by whom?!)
        // strange 2: set POWER = 0 INCREASES power consumption!
        // -> POWER & SDA/SCL seems to be SHORTED!!
        device.setCPUSpeed(ESP32_10MHZ);
        printf("PWM FREQUENCY: %10.1f\n", frequency);
        

        ledc_channel_config_t ledc_channel_left = {0};
        ledc_channel_left.gpio_num = GPIO_NUM_26;
        ledc_channel_left.speed_mode = LEDC_HIGH_SPEED_MODE;
        ledc_channel_left.channel = LEDC_CHANNEL_1;
        ledc_channel_left.intr_type = LEDC_INTR_DISABLE;
        ledc_channel_left.timer_sel = LEDC_TIMER_1;
        ledc_channel_left.duty = 0;  

        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
        ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
        ledc_timer.timer_num = LEDC_TIMER_1;
        ledc_timer.freq_hz = frequency;
        if(frequency < 10000) {
            frequency = frequency + 1000;
        }
        
	    ledc_channel_config(&ledc_channel_left);
	    ledc_timer_config(&ledc_timer);

        float left_duty_fraction = 0.5; // 50% duty cycle = maximum volume!
        uint32_t max_duty = (1 << LEDC_TIMER_10_BIT) - 1;
	    uint32_t left_duty = lroundf(left_duty_fraction * (float)max_duty);
	
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, left_duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);


        device.delay(1500);

        device.enableInternalRTCInterruptInDeepSleep(5);
        device.deepSleep(); // 26.6uA in deep sleep -> maybe ACC is shitty with current hardware sample
    }
}