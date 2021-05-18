#include "PlatformWildFiTagREV5.h"
#include "esp32/ulp.h"
//#include "ulp_main.h"

WildFiTagREV5 device = WildFiTagREV5();

RTC_DATA_ATTR bool firstStart = true;

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            device.measureTime("AWAKE");
            int32_t hall = device.readHallSensor(1);
            device.measureTime("readHallSensor");
            printf("1st HALL: %d\n", hall);
            firstStart = false;
        }
        else {
            printf("%d\n", device.readHallSensor(10));
            device.enableInternalTimerInterruptInDeepSleep(1);
            device.deepSleep();
        }
    }
}