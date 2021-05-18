#include "PlatformWildFiTagREV6.h"

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint8_t bootCnt = 0;

#define PROXIMITY_LONGRANGE         false
#define PROXIMITY_DBM               RADIO_MAX_TX_19_5_DBM
#define PROXIMITY_DATARATE          WIFI_PHY_RATE_1M_L

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            device.disableWakeStubNoBootIfVoltageLow();
            firstStart = false;
            if(!device.fullRFCalibration()) { printf("CALIB ERROR\n"); } // 145 - 160ms
        }
        else {
            if(!device.onlyLoadRFCalibration()) { printf("ERROR0\n"); } // 23ms
            if(!device.initESPNOWStationary(PROXIMITY_LONGRANGE, PROXIMITY_DBM, true, PROXIMITY_DATARATE)) { printf("ERROR1\n"); } // 23ms
            if(!device.addESPNOWBroadcastReceiverStationary()) { printf("ERROR2\n"); } // 0ms
            uint8_t data[250];
            for(uint8_t i=0; i<250; i++) { data[i] = 0xFF; }
            if(!device.broadcastESPNOWData(&data[0], 250)) { printf("ERROR3\n"); }
            device.stopESPNOW(); // 5ms
            esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
        }
        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(1);
        device.deepSleep();
    }
}