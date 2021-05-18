#include "ESP32TrackerREV3.h"
#include <sys/time.h>
#include <sys/param.h>
#include "esp_pm.h"
#include "esp32/clk.h"

#define MHZ     1000000

// CHECK MENUCONFIG
#if !CONFIG_PM_ENABLE
    #error "CONFIG_PM_ENABLE missing"
#endif
#if !CONFIG_FREERTOS_USE_TICKLESS_IDLE
    #error "CONFIG_FREERTOS_USE_TICKLESS_IDLE missing"
#endif

ESP32TrackerREV3 device = ESP32TrackerREV3();

// NEW WITH ESP IDF 4.1: ONLY 38ms!!!!!

// TODO: 10MHz klappen ohne tickless idle???


static void switchFreq(int mhz) {
    int xtal_freq = rtc_clk_xtal_freq_get();
    printf("XTAL Freq: %d, Req MHz: %d, Min: %d\n", xtal_freq, mhz, MIN(mhz, xtal_freq));
    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = mhz,
        .min_freq_mhz = MIN(mhz, xtal_freq),
        .light_sleep_enable = true,
    };
    if(esp_pm_configure(&pm_config) != ESP_OK) {
        printf("ERROR\n");
    }
    printf("Waiting for frequency to be set to %d MHz...\n", mhz);
    /*while (esp_clk_cpu_freq() / MHZ != mhz) {
        //vTaskDelay(pdMS_TO_TICKS(200));
        //printf("Frequency is %d MHz\n", esp_clk_cpu_freq() / MHZ);
    }*/
}

extern "C" void app_main() {
    while(1) {
        //device.delay(1000);
        //device.setCPUSpeed(ESP32_10MHZ);
        //device.delay(1000);
        //printf("HELLO!\n");
        
        /*switch_freq(10);
        printf("RUNNING AT 10MHZ!\n");
        device.delay(1000);*/


        device.enableInternalTimerInterruptInDeepSleep(5);
        device.deepSleep();
    }
}