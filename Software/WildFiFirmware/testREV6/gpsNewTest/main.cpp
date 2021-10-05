#include "PlatformWildFiTagREV6.h"
#include "ModuleGPS_L70_REV6.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();
GPS_L70_REV6 gps = GPS_L70_REV6();

RTC_DATA_ATTR bool firstStart = true;

void gpsCheckAliveAndMaybeChangeBaudrate(bool debug) {
    device.shortLightSleep(4000);
    device.gpioBOn();   
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');
    if(gps.isStarted()) {
        if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: GPS working on 115200\n"); }
    }
    else {
        device.gpioBOff();
        if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: GPS not answering on 115200 -> TRYING 9600\n"); }
        device.shortLightSleep(1000);
        device.uart2UpdateBaudrate(9600);
        device.gpioBOn(); // turn on again
        if(gps.isStarted()) {
            if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: GPS answering on 9600 -> CHANGE\n"); }
            gps.permanentlySetBaudrate115200(); // waiting 2 seconds here
            device.gpioBOff();
            device.delay(2000);
            device.uart2UpdateBaudrate(115200);
            device.gpioBOn(); // turn on again
            if(gps.isStarted()) {
                if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: change success, talking on 115200 now\n"); }
            }
            else {
                if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: FATAL, change did not work\n"); }
            }
        }
        else {
            if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: NOT ANSWERING ON 9600 OR 115200 -> NOT CONNECTED?\n"); }
        }
    }
    device.gpioBOff();
}

extern "C" void app_main() {
    while(1) {
        i2c.begin(I2C_FREQ_HZ_400KHZ);
        if(firstStart) {
            firstStart = false;
            gpsCheckAliveAndMaybeChangeBaudrate(true);
        }
        else {
            device.gpioBOn();
            device.uart2Init(115200);
            gps.init(device.uart2GetQueue());
            device.uart2EnablePatternInterrupt('\n');
            esp_gps_t gpsData = { };
            gps_get_fix_config_t gpsConfig = {
                .timeoutSeconds = 300,
                .minHDOP = 1.0f,
                .afterFixMaxWaitOnHDOP = 12,
                .setRTCTime = true,
                .blinkLeds = true,
                .debug = true };
            get_fix_result_t fixResult = gps.tryToGetFixNew(&gpsData, &gpsConfig, &device);
            device.gpioBOff();
            if(fixResult != GPS_FIX_SUCCESS_AND_RTC_UPDATED) {
                printf("GPS: failed %d!\n", fixResult);
            }
            else {
                printf("GPS: SUCCESS: Timestamp: %u, TTFF: %d, LAT: %f, LON: %f, HDOP: %f\n", gpsData.parent.utcTimestamp, gpsData.parent.ttfMilliseconds, gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.dop_h);
            }
        }

        device.enableInternalTimerInterruptInDeepSleep(3);
        device.deepSleep();
    }
}
