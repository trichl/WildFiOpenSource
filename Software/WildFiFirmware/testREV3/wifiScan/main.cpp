#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

RTC_DATA_ATTR uint32_t success = 0;
RTC_DATA_ATTR uint32_t failed = 0;

const char* SSID = "mpiab";
const char* PASSWORD = "xxxxxx";
const char* SSIDS[3] = { "notworking", "LiWoAb New", SSID };
const char* PASSWORDS[3] = { "notworking", "xxxxxx", PASSWORD };

void option1() {
    if(device.scanForWiFis(false, RADIO_MAX_TX_11_DBM, 120, WIFI_CHANNELS_1_TO_11)) { // WIFI_CHANNELS_1_TO_11
        while(!device.wiFiScanCompleted()) {
            device.delay(20);
        }
        device.measureTime("scanForWiFis");
        //device.printWiFiScanResults();

        uint8_t onChannel = 0;
        if(device.wiFiScanIncludes(SSID, &onChannel)) {
            printf("FOUND ON CHANNEL %d!\n", onChannel);
            device.connectToWiFiAfterScan(SSID, PASSWORD, onChannel);
            while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                device.delay(20);
            }
            device.measureTime("connectToWiFiAfterScan");
            if(device.connectedToWiFi() == WIFI_CONNECT_SUCCESS) { printf("CONNECT RESULT: WIFI_CONNECT_SUCCESS\n"); success++; }
            else if(device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) { printf("CONNECT RESULT: WIFI_CONNECT_FAIL_AP_NOT_FOUND\n"); failed++; }
            else if(device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED) { printf("CONNECT RESULT: WIFI_CONNECT_NEVER_STARTED\n"); failed++; }
            device.delay(1000);
        }
        else failed++;
    }
    device.disconnectAndStopWiFi();
}

void option2() {
    if(device.connectToWiFiDirectly(SSID, PASSWORD, RADIO_MAX_TX_11_DBM, WIFI_CHANNELS_1_TO_11)) {
        while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
            device.delay(20);
        }
        device.measureTime("connectToWiFiDirectly"); // 941 - 1439ms (AP da), 1495ms (AP nicht da - weil scannt alle 11 channels)
        if(device.connectedToWiFi() == WIFI_CONNECT_SUCCESS) { printf("CONNECT RESULT: WIFI_CONNECT_SUCCESS\n"); success++; }
        else if(device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) { printf("CONNECT RESULT: WIFI_CONNECT_FAIL_AP_NOT_FOUND\n"); failed++; }
        else if(device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED) { printf("CONNECT RESULT: WIFI_CONNECT_NEVER_STARTED\n"); failed++; }
        device.delay(1000);
    }
    else printf("ERROR\n");

}

void option3() { // WINNER!!!!!!!!!
    uint8_t arrayId = 0;
    uint8_t connectRetry = 0;
    uint8_t foundOnChannel = 0;
    if(!device.scanForWiFisOn1and6and11(SSIDS, 3, &arrayId, &foundOnChannel, RADIO_MAX_TX_11_DBM, 100, 500)) { printf("----ERROR----\n"); }
    if(foundOnChannel > 0) {
        while(true) {
            device.measureTime("scanForWiFisOn1and6and11"); // 430ms
            printf("Found on channel %d, arrayId %d!\n", foundOnChannel, arrayId);
            device.connectToWiFiAfterScan(SSIDS[arrayId], PASSWORDS[arrayId], foundOnChannel);
            while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                device.delay(20);
            }
            device.measureTime("connectToWiFiAfterScan");  // 1019 - 2000ms
            if(device.connectedToWiFi() == WIFI_CONNECT_SUCCESS) { printf("CONNECT RESULT: WIFI_CONNECT_SUCCESS\n"); success++; device.delay(1000); break; }
            else if(device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) { printf("CONNECT RESULT: WIFI_CONNECT_FAIL_AP_NOT_FOUND\n"); }
            else if(device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED) { printf("CONNECT RESULT: WIFI_CONNECT_NEVER_STARTED\n"); }
            connectRetry++;
            device.delay(100); // wait a bit
            printf("Could not connect, but saw wifi -> retry!\n");
            if(connectRetry >= 2) { printf("Retry counter too high FULL FAIL!!\n"); failed++; break; }
        }
        
    }
    else { 
        device.measureTime("scanForWiFisOn1and6and11"); // 566ms (AP nicht da, immer 120s)
        printf("NOT FOUND\n"); failed++;
    }
}

extern "C" void app_main() {
    while(1) {
        printf("\nAWAKE, %d success, %d failed!\n", success, failed);

        device.measureTime("booted");
        if(device.initWiFi()) {
            device.measureTime("init wifi");
            option3();
        }

        device.enableInternalTimerInterruptInDeepSleep(5);
        device.deepSleep();
    }
}