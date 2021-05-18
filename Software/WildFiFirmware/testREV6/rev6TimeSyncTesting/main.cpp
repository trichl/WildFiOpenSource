#include "PlatformWildFiTagREV6.h"
#include "ModuleGPS_L70_REV6.h"

WildFiTagREV6 device = WildFiTagREV6();
GPS_L70_REV6 gps = GPS_L70_REV6();

RTC_DATA_ATTR uint64_t bootCnt = 0;

#define MODE_TESTRUN        1
#define TRACKER_MODE        MODE_TESTRUN
const uint8_t TIME_WIFI_LIST_SIZE =                     2;                              // should include time wifi
const char* TIME_WIFI_SSIDS[TIME_WIFI_LIST_SIZE] =      { "mpitime", "RodelbahnSoelden" }; // wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* TIME_WIFI_PASSWORDS[TIME_WIFI_LIST_SIZE] =  { "87654321", "xxxxxx" }; // wifi password for activation/timestamp                  
#define TIME_WIFI_OUTPUT_POWER                          RADIO_MAX_TX_11_DBM             // 19.5dBm will brown out on REV3
#define WIFI_MAX_CONNECT_TIME_SECONDS                   8 

RTC_DATA_ATTR uint64_t timeDiffSum = 0;
RTC_DATA_ATTR uint64_t timeDiffCnt = 0;
RTC_DATA_ATTR uint64_t timeDiffMax = 0;

void getTimeOverWiFi() {
    // try to get time over wifi or GPS
    if(device.initWiFi()) {
        uint8_t foundArrayId = 0;
        uint8_t foundOnChannel = 0;
        bool connectionTimeout = false;
        uint32_t scanStartTime = ((uint32_t) Timing::millis());
        if(device.scanForWiFisOn1and6and11and13WithPriority((TRACKER_MODE == MODE_TESTRUN), TIME_WIFI_SSIDS, TIME_WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, TIME_WIFI_OUTPUT_POWER, 120, 500)) { 
            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500); }
            if(foundOnChannel > 0) { // found wifi, try to connect
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: wifi found (chan %d, index %d)\n", foundOnChannel, foundArrayId); }
                if(device.connectToWiFiAfterScan(TIME_WIFI_SSIDS[foundArrayId], TIME_WIFI_PASSWORDS[foundArrayId], foundOnChannel)) {
                    uint32_t connectStartedTime = ((uint32_t) Timing::millis());
                    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                        device.delay(20);
                        if(((uint32_t) Timing::millis()) - connectStartedTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: TIMEOUT CONNECT!\n"); }
                            connectionTimeout = true;
                            break;
                        }
                    }
                    if(connectionTimeout || (device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: STRANGE could not connect, but wifi was seen (pw wrong, no internet): %d!\n", device.connectedToWiFi()); }
                    }
                    else { // connected to wifi
                        uint32_t timestampUTC = 0;
                        uint16_t millisecondsUTC = 0;
                        int64_t timestampDiffMs = 0;
                        if(!device.getNTPTimestampUTCAndCompareAccuracy(true, timestampUTC, millisecondsUTC, timestampDiffMs, 7000, "pool.ntp.org")) { // will block, will set RTC time
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: UTC get time error!\n"); }
                        }
                        else {
                            if(TRACKER_MODE == MODE_TESTRUN) {
                                printf("Time: set %d + 1, UTC milliseconds: %d -> waited for %dms\n", timestampUTC, millisecondsUTC, 1000 - millisecondsUTC);
                                printf("Time: difference: %lldms\n", timestampDiffMs);
                                if(timestampDiffMs < 0) { timestampDiffMs *= -1; }
                                if(timestampDiffMs > timeDiffMax) { timeDiffMax = timestampDiffMs; }
                                timeDiffSum += timestampDiffMs;
                                timeDiffCnt++;
                            }
                        }
                    }
                    device.disconnectAndStopWiFi();
                }
                else { device.disconnectAndStopWiFi(); }
            }
            else {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: wifi NOT found\n"); }
                device.disconnectAndStopWiFi();
            }
        }
        else {
            device.disconnectAndStopWiFi();
        }
    } 
}

void tryGetTimeOverGPS() {
    // untested
    device.gpioBOn();
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');
    esp_gps_t gpsData = { };
    if(gps.getTimeOnly(&gpsData, 200, &device, true, false)) {
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time over GPS: all done!\n"); }
    }
    else {
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time over GPS: failed!\n"); }
    }
    device.gpioBOff();
}

extern "C" void app_main() {
    while(1) {
        printf("---------\n");
        if(bootCnt == 0) { device.delay(5000); }
        i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time

        if(bootCnt % 2 == 0) {
            tryGetTimeOverGPS();
        }
        else {
            getTimeOverWiFi();
            if(timeDiffCnt > 0) {
                uint64_t avg = timeDiffSum / timeDiffCnt;
                printf("Time Diff Avg: %llu, Max: %llu\n", avg, timeDiffMax);
            }
        }

        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(2);
        device.deepSleep();
    }
}