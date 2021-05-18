#include "PlatformWildFiTagREV6.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR bool firstStart = true;

const uint8_t TIME_WIFI_LIST_SIZE =                     2;                                  // should include time wifi
const char* TIME_WIFI_SSIDS[TIME_WIFI_LIST_SIZE] =      { "RodelbahnSoelden", "mpitime" };  // wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* TIME_WIFI_PASSWORDS[TIME_WIFI_LIST_SIZE] =  { "xxxxxx", "87654321" };        // wifi password for activation/timestamp                  
#define TIME_WIFI_OUTPUT_POWER                          RADIO_MAX_TX_11_DBM                 // 19.5dBm will brown out on REV3
#define TIME_BETWEEN_GET_TIME_RETRIES_SECONDS           120                                 // 120 = 660uA, if not activated: sleep for that time, wake up, check if wifi there and try to activate, 120 = 234uA average
#define TIME_OVER_GPS_BLINK_LED                         true                                // blink the led when trying to get the time



bool getTimeOverWiFiNew() {
    bool hasValidTimestamp = false;
    if(device.initWiFi()) {
        uint8_t foundArrayId = 0;
        uint8_t foundOnChannel = 0;
        bool connectionTimeout = false;
        uint32_t scanStartTime = ((uint32_t) Timing::millis());
        if(device.scanForWiFisOn1and6and11and13WithPriority(true, TIME_WIFI_SSIDS, TIME_WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, TIME_WIFI_OUTPUT_POWER, 120, 500)) { 
            printf("Time: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500);
            if(foundOnChannel > 0) { // found wifi, try to connect
                printf("Time: wifi found (chan %d, index %d), hasValidTimestamp: %d\n", foundOnChannel, foundArrayId, hasValidTimestamp);
                if(device.connectToWiFiAfterScan(TIME_WIFI_SSIDS[foundArrayId], TIME_WIFI_PASSWORDS[foundArrayId], foundOnChannel)) {
                    uint32_t connectStartedTime = ((uint32_t) Timing::millis());
                    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                        device.delay(20);
                        if(((uint32_t) Timing::millis()) - connectStartedTime > (9 * 1000)) { // e.g. password wrong
                            printf("Time: TIMEOUT CONNECT!\n");
                            connectionTimeout = true;
                            break;
                        }
                    }
                    if(connectionTimeout || (device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                        printf("Time: STRANGE could not connect, but wifi was seen (pw wrong, no internet): %d!\n", device.connectedToWiFi());
                    }
                    else { // connected to wifi
                        uint32_t timestampUTC = 0;
                        uint16_t millisecondsUTC = 0;
                        if(!device.getNTPTimestampUTC(true, timestampUTC, millisecondsUTC, 7000, "pool.ntp.org")) { // will block, will set RTC time
                            printf("Time: UTC get time error!\n");
                        }
                        else {
                            printf("Time: set %d + 1, UTC milliseconds: %d -> waited for %dms\n", timestampUTC, millisecondsUTC, 1000 - millisecondsUTC);
                            hasValidTimestamp = true;
                        }
                    }
                    device.disconnectAndStopWiFi();
                }
                else { device.disconnectAndStopWiFi(); }
                // check result of getting timestamp attempt
                if(hasValidTimestamp) { return true; }
                else {
                    printf("Time: wifi was seen, but still something missing (pw wrong, no internet)\n"); 
                    return false;
                }
            }
            else {
                printf("Time: wifi NOT found\n");
                device.disconnectAndStopWiFi();  
                return false;
            }
        }
        else { device.disconnectAndStopWiFi(); return false; }
    }
    return false;
}

bool compareTimeOverWiFi() {
    bool hasValidTimestamp = false;
    if(device.initWiFi()) {
        uint8_t foundArrayId = 0;
        uint8_t foundOnChannel = 0;
        bool connectionTimeout = false;
        uint32_t scanStartTime = ((uint32_t) Timing::millis());
        if(device.scanForWiFisOn1and6and11and13WithPriority(true, TIME_WIFI_SSIDS, TIME_WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, TIME_WIFI_OUTPUT_POWER, 120, 500)) { 
            printf("Time: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500);
            if(foundOnChannel > 0) { // found wifi, try to connect
                printf("Time: wifi found (chan %d, index %d), hasValidTimestamp: %d\n", foundOnChannel, foundArrayId, hasValidTimestamp);
                if(device.connectToWiFiAfterScan(TIME_WIFI_SSIDS[foundArrayId], TIME_WIFI_PASSWORDS[foundArrayId], foundOnChannel)) {
                    uint32_t connectStartedTime = ((uint32_t) Timing::millis());
                    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                        device.delay(20);
                        if(((uint32_t) Timing::millis()) - connectStartedTime > (9 * 1000)) { // e.g. password wrong
                            printf("Time: TIMEOUT CONNECT!\n");
                            connectionTimeout = true;
                            break;
                        }
                    }
                    if(connectionTimeout || (device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                        printf("Time: STRANGE could not connect, but wifi was seen (pw wrong, no internet): %d!\n", device.connectedToWiFi());
                    }
                    else { // connected to wifi
                        uint32_t timestampUTC = 0;
                        uint16_t millisecondsUTC = 0;
                        if(!device.getNTPTimestampUTC(false, timestampUTC, millisecondsUTC, 7000, "pool.ntp.org")) { // will block, will set RTC time
                            printf("Time: UTC get time error!\n");
                        }
                        else {
                            bool error = false;
                            uint8_t second100th = device.rtc.get100thOfSeconds(error);
                            uint16_t rtcMillis = second100th;
                            rtcMillis *= 10;
                            uint32_t rtcTimestamp = device.rtc.getTimestamp(error);
                            printf("Time from Wifi: %d.%d vs. RTC %d.%d\n", timestampUTC, millisecondsUTC, rtcTimestamp, rtcMillis);
                            hasValidTimestamp = true;
                        }
                    }
                    device.disconnectAndStopWiFi();
                }
                else { device.disconnectAndStopWiFi(); }
                // check result of getting timestamp attempt
                if(hasValidTimestamp) { return true; }
                else {
                    printf("Time: wifi was seen, but still something missing (pw wrong, no internet)\n"); 
                    return false;
                }
            }
            else {
                printf("Time: wifi NOT found\n");
                device.disconnectAndStopWiFi();  
                return false;
            }
        }
        else { device.disconnectAndStopWiFi(); return false; }
    }
    return false;
}

RTC_DATA_ATTR uint32_t wakeCnt = 0;

RTC_DATA_ATTR void rtcInterruptResetI2C() {
    beginTransmissionSw(RTC_RV8803C7_ADDRESS);
    writeSw(REG8803_CONTROL); // WITH NEW RTC -> UIE = 0
    writeSw(0); // WARNING: reset all pin interrupts!
    endTransmissionSw();

    beginTransmissionSw(RTC_RV8803C7_ADDRESS);
    writeSw(REG8803_FLAG); // WITH NEW RTC -> UF = 0
    writeSw(0); // WARNING: reset all pin interrupts!
    endTransmissionSw();

    ets_delay_us(1000);

    beginTransmissionSw(RTC_RV8803C7_ADDRESS);
    writeSw(REG8803_CONTROL);
    writeSw(0b00100000); // WARNING: reset all pin interrupts!
    endTransmissionSw();
}

RTC_DATA_ATTR bool wakeStub() { // called multiple times if RTC interrupt not being reset
    while(GPIO_INPUT_GET(25) == 0) {
        //ets_delay_us(500);
        deepsleep_for_us(20000);
    }
    return false;
    
    /*beginSw();
    rtcInterruptResetI2C();
    wakeCnt++;
    if(wakeCnt % 2 == 0) { return true; }
    else { return false; }*/
}

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            firstStart = false;
            i2c.begin(I2C_FREQ_HZ_400KHZ);
            //getTimeOverWiFiNew();


            device.disableWakeStubNoBootIfVoltageLow(); // disable no boot if voltage low (do here, only needed once!)
            device.customWakeStubFunction(wakeStub);
            device.setWakeStubRejectionInterruptSrc(USE_EXT0_IF_WAKE_UP_REJECTED);

            device.rtc.set(19, 59, 57, 0, 8, 3, 2021);
            device.rtc.setTimeUpdateInterruptMinuteChange();
        }
        else {
            printf("HELLO %d\n", wakeCnt);
            i2c.begin(I2C_FREQ_HZ_400KHZ);
            //compareTimeOverWiFi();
        }


        device.enableRTCInterruptInDeepSleep();
        //device.enableInternalTimerInterruptInDeepSleep(5);
        device.deepSleep();
    }
}
