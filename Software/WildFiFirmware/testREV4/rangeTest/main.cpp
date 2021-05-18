#include "ESP32TrackerREV4.h"
#include <math.h>

ESP32TrackerREV4 device = ESP32TrackerREV4();

#define ROLE_SENDER
//#define ROLE_RECEIVER

#define DATARATE            WIFI_PHY_RATE_1M_L
#define DATALEN             250
#define LONGRANGE           true
#define DBM_SETTING         RADIO_MAX_TX_19_5_DBM

uint8_t receiveBlink = 0;

bool isProximityMessage(wifi_promiscuous_pkt_t* p, wifi_promiscuous_pkt_type_t &type) {
    if(type == WIFI_PKT_MGMT) { // all esp now messages are MGMT frames
        if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + DATALEN) { // normally 43 bytes additionally + 250 bytes payload
           //if(p->rx_ctrl.rate == DATARATE) { // using the correct proximity data rate
                if(p->payload[ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE] == 0x04) { // is ESP NOW frame
                    return true;
                }
            //}
        }
    }
    return false;
}

void wifiPromiscuous(void* buffer, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
    if(isProximityMessage(p, type)) {
        printf("RECV RSSI: %d, PHY: %d\n", p->rx_ctrl.rssi, p->rx_ctrl.rate);
        if(p->rx_ctrl.rssi < -90) { receiveBlink = 1; }
        else if(p->rx_ctrl.rssi < -80) { receiveBlink = 2; }
        else if(p->rx_ctrl.rssi < -70) { receiveBlink = 3; }
        else if(p->rx_ctrl.rssi < -45) { receiveBlink = 4; }
        else if(p->rx_ctrl.rssi < -30) { receiveBlink = 5; }
        else if(p->rx_ctrl.rssi < 0) { receiveBlink = 6; }
    }
}

static void proxReceiveCallback(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
} // do not do anything here, handled by promiscous sniffer

void receive() {
    if(!device.initESPNOWStationary(LONGRANGE, DBM_SETTING, true, DATARATE)) { printf("ERROR\n"); return; } // 23ms
    if(!device.addESPNOWBroadcastReceiverStationary()) { printf("ERROR\n"); return; } // 0ms

    esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuous);
    esp_wifi_set_promiscuous(true);
    esp_now_register_recv_cb(proxReceiveCallback); // 0ms

    while(true) {
        device.delay(100);
        if(receiveBlink > 0) {
            printf("BLINK: %d\n", receiveBlink);
            if(receiveBlink == 1) { device.blink(B_GREEN); }
            else if(receiveBlink == 2) { device.blink(B_GREEN, B_GREEN); }
            else if(receiveBlink == 3) { device.blink(B_GREEN, B_GREEN, B_GREEN); }
            else if(receiveBlink == 4) { device.blink(B_GREEN, B_GREEN, B_GREEN); device.blink(B_RED); }
            else if(receiveBlink == 5) { device.blink(B_GREEN, B_GREEN, B_GREEN); device.blink(B_RED, B_RED); }
            else if(receiveBlink == 6) { device.blink(B_GREEN, B_GREEN, B_GREEN); device.blink(B_RED, B_RED, B_RED); }
            receiveBlink = 0;
        }
    }

    device.stopESPNOW(); //5ms
}

void send() {
    uint8_t data[DATALEN] = { 0 };
    for(uint8_t i = 0; i < DATALEN; i++) { data[i] = i; }
    if(!device.initESPNOWStationary(LONGRANGE, DBM_SETTING, true, DATARATE)) { printf("ERROR\n"); return; } // 23ms
    if(!device.addESPNOWBroadcastReceiverStationary()) { printf("ERROR\n"); return; } // 0ms
    device.broadcastESPNOWData(data, DATALEN); // spit it out
    device.stopESPNOW(); //5ms
}

extern "C" void app_main() {
    while(1) {
        if(!device.customPhyInit()) { // after reset: 153ms, otherwise + 120ms during esp now init
            printf("PHY INIT ERROR\n");
        }
        #ifdef ROLE_SENDER
            send();
            device.blink(B_RED);
            device.disableWakeStubNoBootIfVoltageLow();
            device.enableInternalTimerInterruptInDeepSleep(2);
            device.deepSleep();
        #else
            receive();
        #endif
    }
}
