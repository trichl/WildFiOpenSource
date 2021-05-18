#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

RTC_DATA_ATTR bool firstStart = true;

RTC_DATA_ATTR bool wakeStub() {
    return true;
}

uint32_t t = 0;
void measureTime(const char* text) {
    uint64_t timeNow = ((uint32_t) Arduino::millisWrapper());
    t = timeNow - t;
    printf("%s TOOK: %d ms\n", text, t);
    t = (uint32_t) Arduino::millisWrapper();
}

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            i2c.begin();
            if(!device.rtc.setTimestamp(UNIX_TIMESTAMP+30)) { printf("ERROR\n"); }
            printf("TRYING TO BROWN THE SHIT OUT\n");
            device.rtc.setRegularInterrupt(20);
            firstStart = false;
            device.delay(4000);
        }

        device.disableWakeStubNoBootIfVoltageLow();
        device.customWakeStubFunction(wakeStub);
        device.setWakeStubRejectionInterruptSrc(USE_EXT0_IF_WAKE_UP_REJECTED);

        // WIFI BROWNOUT TEST
        printf("INIT WIFI\n");
        if(!device.initWiFi()) { printf("ERROR\n"); }
        printf("WIFI START\n");
        if(esp_wifi_start() != ESP_OK) { printf("ERROR\n"); }
        printf("SET MAX TX POWER\n");
        if(esp_wifi_set_max_tx_power(78) != ESP_OK) { printf("SOME ERROR SETTING TX POWER\n"); }; // 20dBm -> BROWNOUT!!!
        int8_t txPwr = 0;
        esp_wifi_get_max_tx_power(&txPwr);
        printf("TX POWER: %d\n", txPwr);

        wifi_active_scan_time_t scanTimeActive = {
            .min = 0, // in ms per channel
            .max = 120 // in ms per channel (14 channels in total, default = 120ms per channel = 1680ms scan time)
        };
        wifi_scan_time_t scanTime = {
            .active = scanTimeActive
        };
        wifi_scan_config_t scanConf = {
            .ssid = NULL, // don't scan for specific SSID
            .bssid = NULL, // don'T scan for specific BSSID
            .channel = 0, // 0 = all channel scan
            .show_hidden = true, // show hidden wifis
            .scan_type = WIFI_SCAN_TYPE_ACTIVE, // default = active scan (sends out beacon instead of passively waiting for one)
            .scan_time = scanTime
        };
        
        printf("NOW SHOULD BROWNOUT\n");
        if(esp_wifi_scan_start(&scanConf, true) != ESP_OK)  { printf("ERROR\n"); } // HERE BROWNOUT with tx power = 78 (*0.25 = 19.5dBm)
        printf("DID NOT BROWNOUT\n");

        device.printWiFiScanResults();

        device.ledGreenOn();
        device.delay(1000);
        device.ledGreenOff();

        printf("\n");
        device.enableRTCInterruptInDeepSleep();
        device.deepSleep();
    }
}