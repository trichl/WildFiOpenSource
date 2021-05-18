#include "PlatformWildFiTagREV6.h"

WildFiTagREV6 device = WildFiTagREV6();

void stressTest() {
    printf("STARTING in 10 SECONDS\n");
    device.delay(10000);

    const uint32_t MAX_PAGES = 131072; // 131072

    uint8_t *testData = NULL;
    if(!device.flash.createBuffer(&testData, MT29_CACHE_SIZE)) { printf("..FAILED (buffer create)\n"); } 

    printf("%llu DELETING FULL MEMORY\n", Timing::millis());
    device.flashPowerOn(true);
    if(device.flash.fullErase()) {
        bool error = false;
        printf("%llu WRITING MEMORY FULL (5mins)\n", Timing::millis());
        for(uint32_t i=0; i<MAX_PAGES; i++) {
            for(uint16_t j=0; j<MT29_CACHE_SIZE; j++) { testData[j] = i; }
            if(!device.flash.write(i, testData, MT29_CACHE_SIZE)) { printf("..FAILED (write page %d)\n", i); error = true; break; }

            if(i % 1000 == 0) { printf("%llu - WRITING MEMORY STATUS PG %d\n", Timing::millis(), i); }
        }

        printf("%llu READING FULL MEMORY (5mins)\n", Timing::millis());
        uint8_t iCompare = 0;
        for(uint16_t j=0; j<MT29_CACHE_SIZE; j++) { testData[j] = 0; }
        for(uint32_t i=0; i<MAX_PAGES; i++) {
            if(!device.flash.read(i, 0, testData, MT29_CACHE_SIZE)) { printf("..FAILED (read page %d)\n", i); error = true; break; }
            for(uint16_t j=0; j<MT29_CACHE_SIZE; j++) {
                iCompare = i;
                if(testData[j] != iCompare) {
                    printf("..FAILED (compare page %d)\n", i);
                    error = true;
                    break;
                }
            }
            if(error) { break; }

            if(i % 1000 == 0) { printf("%llu - READING MEMORY STATUS PG %d\n", Timing::millis(), i); }
        }

        printf("%llu READING RANDOM PAGES\n", Timing::millis());
        uint32_t randomPage;
        for(uint32_t r=0; r<5; r++) {
            randomPage = esp_random() % MAX_PAGES;
            printf("%llu - PAGE %d\n", Timing::millis(), randomPage);
            device.flash.printFlash(randomPage, 1, 30);
        }

        printf("%llu DELETING FULL MEMORY\n", Timing::millis());
        if(!device.flash.fullErase()) { printf("FAILED FULL ERASE\n"); }

        printf("%llu --- DONE --- (ERROR = %d)\n", Timing::millis(), error);

    }
    else { printf("FAILED FULL ERASE\n"); }

    
    device.deepSleep();
}

void flashStrangePowerConsumptionDeepSleep() {
    device.setCPUSpeed(ESP32_10MHZ);
    device.delay(1000);
    device.flashPowerOn(true);

    device.flash.printFlash(0, 1, 512, false);
    device.delay(1000);

    device.flashPowerOff(false);
    device.enableInternalTimerInterruptInDeepSleep(10);
    device.deepSleep();
}

extern "C" void app_main() {
    while(1) {
        flashStrangePowerConsumptionDeepSleep();
    }
}
