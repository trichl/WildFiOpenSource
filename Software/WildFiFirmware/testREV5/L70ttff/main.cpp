#include "PlatformWildFiTagREV5.h"
#include "ModuleGPS_L70.h"
#include "esp_spiffs.h"

WildFiTagREV5 device = WildFiTagREV5();
GPS_L70 gps = GPS_L70();

const char* SPIFFS_FILE =                               "/spiffs/gps.txt";

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint64_t bootCnt = 0;

void storeOnSpiffs(esp_gps_t *gpsData) {
    FILE* f = fopen(SPIFFS_FILE, "a"); // a = append, w = (over)write
    if(f == NULL) {
        printf("Failed to open file for writing\n");
        return;
    }
    fprintf(f, "%d.%d.%d %d:%d:%d TTFF: %d, LAT: %f, LON: %f, SATS: %d, HDOP: %.2f\n", gpsData->parent.date.day, gpsData->parent.date.month, gpsData->parent.date.year, gpsData->parent.tim.hour, gpsData->parent.tim.minute, gpsData->parent.tim.second, gpsData->parent.ttfMilliseconds, gpsData->parent.latitude, gpsData->parent.longitude, gpsData->parent.sats_in_use, gpsData->parent.dop_h);
    fclose(f);
    printf("WROTE: %d.%d.%d %d:%d:%d TTFF: %d, LAT: %f, LON: %f, SATS: %d, HDOP: %.2f\n", gpsData->parent.date.day, gpsData->parent.date.month, gpsData->parent.date.year, gpsData->parent.tim.hour, gpsData->parent.tim.minute, gpsData->parent.tim.second, gpsData->parent.ttfMilliseconds, gpsData->parent.latitude, gpsData->parent.longitude, gpsData->parent.sats_in_use, gpsData->parent.dop_h);
}

extern "C" void app_main() {
    while(1) {
        //device.setCPUSpeed(ESP32_10MHZ);
        device.initSpiffs(1);
        if(firstStart) {
            device.disableWakeStubNoBootIfVoltageLow();
            printf("FIRST START\n");
            device.delay(6000);
            //device.resetSpiffs(SPIFFS_FILE);
            device.printSpiffs(SPIFFS_FILE);
            firstStart = false;
        }
        printf("TRY GETTING FIX %llu!\n", bootCnt);
        device.gpioBOn();
        device.ledRedOn();

        device.uart2Init(115200);
        gps.init(device.uart2GetQueue());
        device.uart2EnablePatternInterrupt('\n');

        /*device.delay(1000);
        if(!gps.permanentlySetBaudrate115200()) { printf("PERMANENT BAUDRATE CHANGE FAILED\n"); }
        else { printf("PERMANENT BAUDRATE CHANGED SUCCESS\n"); }
        device.delay(100000);*/

        esp_gps_t gpsData = { };
        gps_get_fix_config_t gpsConfig = { };
        gpsConfig.device = &device;
        gpsConfig.timeoutSeconds = 300;
        gpsConfig.minHDOP = 3.0;
        gpsConfig.afterFixMaxWaitOnHDOP = 5;
        gpsConfig.ledOn = true;
        gpsConfig.debug = true;
        if(!gps.tryToGetFix(&gpsData, &gpsConfig)) {
            printf("ERROR in getting fix\n");
            device.blinkTimes(20, B_RED);
        }
        else {
            device.blinkTimes(20, B_GREEN);
        }
        printf("RESULT: TTFF: %d, LAT: %f, LON: %f, HDOP: %f\n", gpsData.parent.ttfMilliseconds, gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.dop_h);
        storeOnSpiffs(&gpsData);
        
        device.gpioBOff();
        device.delay(1000);
        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(10);
        device.deepSleep();
    }
}
