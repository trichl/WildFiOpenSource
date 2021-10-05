#include "PlatformWildFiTagREV6.h"
#include "PlatformAttitudeEstimator.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR uint32_t bootCnt = 0;

extern "C" void app_main() {
    while(1) {
        printf("--- %d ---\nPRESS 'w'\n", bootCnt);
        
        if(device.waitOnChar('w', 3)) {
            char select = device.waitOnAnyChar(45);
            //printf("- %c selected:\n", select);


            printf("- %c selected -> READ MEMORY!\nPlease enter start page (0 - %d):\n", select, MT29_NUMBER_PAGES-1);
            uint32_t startPage = device.waitOnNumberInput(60);
            printf("Entered value: %d\n", startPage);
            if(startPage < MT29_NUMBER_PAGES) {
                printf("Please enter number of pages (1 - %d): \n", MT29_NUMBER_PAGES - startPage);
                uint32_t numberPages = device.waitOnNumberInput(60);
                printf("Entered value: %d\n", numberPages);
                if((numberPages > 0) && (numberPages <= (MT29_NUMBER_PAGES - startPage))) {
                    //readFlash(startPage, numberPages, false);
                    printf("Reading: %d - %d\n", startPage, numberPages);
                }
                else { printf("INVALID\n"); }
            }
            else { printf("INVALID\n"); }
        }
        printf("SLEEP!\n");
        device.enableInternalTimerInterruptInDeepSleep(3);
        bootCnt++;
        device.deepSleep();
    }
}
