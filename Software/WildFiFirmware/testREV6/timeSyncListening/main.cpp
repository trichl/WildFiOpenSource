#include "PlatformWildFiTagREV6.h"
#include "ModuleGPS_L70_REV6.h"

WildFiTagREV6 device = WildFiTagREV6();
GPS_L70_REV6 gps = GPS_L70_REV6();

RTC_DATA_ATTR uint64_t bootCnt = 0;

#define MODE_TESTRUN        1
#define TRACKER_MODE        MODE_TESTRUN
/** Proximity settings */
#define PROXIMITY_FREQUENCY_IN_MINUTES                  1                               // WARNING: 60 / PROXIMITY_FREQUENCY_IN_MINUTES without REST!
#define PROXIMITY_LISTENING_INTERVAL_MS                 1000
#define PROXIMITY_DBM                                   RADIO_MAX_TX_19_5_DBM
#define PROXIMITY_DATARATE                              WIFI_PHY_RATE_1M_L              // WIFI_PHY_RATE_1M_L
#define PROXIMITY_LONGRANGE                             false                           // 0.25 to 0.5MBPS
#define PROXIMITY_AIR_TIME_US                           5200                            // 5200, this is measured, CHANGE FOR DIFFERENT DATARATE, calculated would be 1907 = (PROXIMITY_DATA_LEN * 8 * 1000 * 1000) / (1 * 1024 * 1024)

#define PROXIMITY_MAX_TAGS_IN_PARALLEL                  150                             // previous LIMITATION: max. 512 byte in one NVS write cycle = 9 + (3 * X) -> max. 167 tags
#define PROXIMITY_FULL_RF_CALIB_EVERY                   100                             // perform full RF calibration after X times proximity detection

#define PROXIMITY_DATA_LEN                              250

/** Activation settings */
#define ACTIVATION_BY_GATEWAY_LISTENING_TIME            120                             // gateway broadcasts every 100ms
#define ACTIVATION_BY_GATEWAY_INTERVAL_SECONDS          60                              // (if 120ms listening) every 5min = 65.7uA, every 10min = 38.4uA, every 15min = 29.3uA

/** Proximity Group ID */
#define PROXIMITY_OWN_GROUP_0                           0x12
#define PROXIMITY_OWN_GROUP_1                           0x34
#define PROXIMITY_OWN_GROUP_2                           0x56
#define PROXIMITY_OWN_GROUP_3                           0x78

/** All messages payload data types */
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY          0xAA
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND         0xCC
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND     0xDD
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED      0xEE

/** All messages message type (first byte of all message) */
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE                0

/** Proximity payload data structure (250 bytes max!) */
#define PROXIMITY_DATA_LEN                              250
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_0                 1
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_1                 2
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_2                 3
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_3                 4
#define PROX_PAYLOAD_OFFSET_OWN_ID_0                    5
#define PROX_PAYLOAD_OFFSET_OWN_ID_1                    6
#define PROX_PAYLOAD_OFFSET_SENDOFFSET_0                7
#define PROX_PAYLOAD_OFFSET_SENDOFFSET_1                8
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_0                9
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_1                10
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_2                11
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_3                12
#define PROX_PAYLOAD_OFFSET_TSLASTSYNCTYPE              13
#define PROX_PAYLOAD_OFFSET_VOLTAGE_0                   14
#define PROX_PAYLOAD_OFFSET_VOLTAGE_1                   15
#define PROX_PAYLOAD_OFFSET_LASTERRORID                 16
#define PROX_PAYLOAD_OFFSET_ERRORCNT_0                  17
#define PROX_PAYLOAD_OFFSET_ERRORCNT_1                  18
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_0                 19
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_1                 20
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_2                 21
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_3                 22
#define PROX_PAYLOAD_OFFSET_SW_VERSION                  23
#define PROX_PAYLOAD_OFFSET_CONF_VERSION                24

RTC_DATA_ATTR uint8_t lastErrorId = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;

bool timerFinished = false;
bool timeSyncGotTimestamp = false;
uint32_t timeSyncTimestamp = 0;
uint32_t timeSyncTimestampLastSync = 0;

void wifiPromiscuousTimeSync(void* buffer, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
    if(type == WIFI_PKT_MGMT) { // all esp now messages are MGMT frames
        if(p->rx_ctrl.rate == PROXIMITY_DATARATE) { // using the correct proximity data rate
            if(p->payload[ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE] == 0x04) { // is ESP NOW frame
                // proximity message
                if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + PROXIMITY_DATA_LEN) { // normally 43 bytes additionally + 250 bytes payload
                    // same group id
                    if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_GROUP_0] == PROXIMITY_OWN_GROUP_0) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_GROUP_1] == PROXIMITY_OWN_GROUP_1) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_GROUP_2] == PROXIMITY_OWN_GROUP_2) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_GROUP_3] == PROXIMITY_OWN_GROUP_3)) {
                        if(p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY) { // proximity message
                            timeSyncTimestampLastSync = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_0] << 24)
                                | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_1] << 16)
                                | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_2] << 8)
                                | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_3];
                            // TODO: fill timeSyncTimestamp
                            // TODO: fill offset
                            timeSyncGotTimestamp = true;
                        }
                    }
                }
            }
        }
    }
}

static void proxReceiveCallback(const uint8_t *mac_addr, const uint8_t *data, int data_len) { } // do not do anything here, handled by promiscous sniffer
static void timerCallback(void* arg) { timerFinished = true; }

bool timeSyncViaListening() {
    if(!device.initESPNOWStationary(PROXIMITY_LONGRANGE, PROXIMITY_DBM, true, PROXIMITY_DATARATE)) { lastErrorId = 59; errorCnt++; return false; } // 23ms
    if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 60; errorCnt++; return false; } // 0ms

    esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuousTimeSync);
    esp_wifi_set_promiscuous(true);
    esp_now_register_recv_cb(proxReceiveCallback); // 0ms

    const esp_timer_create_args_t timerArgs = { .callback = &timerCallback };
    esp_timer_handle_t timer;
    if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { lastErrorId = 61; errorCnt++; return false; }
    if(esp_timer_start_once(timer, 110 * 1000) != ESP_OK) { lastErrorId = 62; errorCnt++; return false; }

    while(!timerFinished) {
        vTaskDelay(10 / portTICK_PERIOD_MS); // will send current cpu to sleep (10ms accuracy, will wake up a cycle before that)
        if(timeSyncGotTimestamp) {
            if(esp_timer_stop(timer) != ESP_OK) { lastErrorId = 63; errorCnt++; } // stop timer
            break;
        }
    }

    return true;
}

extern "C" void app_main() {
    while(1) {
        if(bootCnt == 0) {
            if(!device.fullRFCalibration()) { lastErrorId = 68; errorCnt++; printf("ERROR\n"); } // 145 - 160ms
            device.delay(5000);
            device.enableInternalTimerInterruptInDeepSleep(2);
            bootCnt++;
            device.deepSleep();
        }
        else {
            if(!device.onlyLoadRFCalibration()) { lastErrorId = 70; errorCnt++; } // 5ms, using RF data in RTC memory
            timeSyncViaListening(); // 110ms listening
            device.stopESPNOW(); // 5ms
            esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
            uint64_t sleepTimeInUs = 900ULL * 1000ULL; // 900ms sleeping
            esp_sleep_enable_timer_wakeup(sleepTimeInUs);
            bootCnt++;
            device.deepSleep();
        }
    }
}