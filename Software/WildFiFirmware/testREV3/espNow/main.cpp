#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

// Auf Autodach: 170m mit 19.5dBm (in Gehäuse)
// Auf Pfahl (Line of Sight): 400m mit 19.5dBm (in Gehäuse)

RTC_DATA_ATTR bool firstStart = true;
extern uint16_t adcValue;

RTC_DATA_ATTR bool wakeStub() {
    if(adcValue < 1000) {
        return false; // don't boot at all
    }
    return true;
}

uint32_t t = 0;
void measureTime(const char* text) {
    uint64_t timeNow = ((uint32_t) Arduino::millisWrapper());
    t = timeNow - t;
    printf("%s TOOK: %d ms\n", text, t);
    t = (uint32_t) Arduino::millisWrapper();
}

void sendToReceiver() {
    if(!device.initESPNOW(true, RADIO_MAX_TX_19_5_DBM)) { printf("ERROR\n"); }
    measureTime("initESPNOW");
    static uint8_t receiverMac[6] = {0xD8, 0xA0, 0x1D, 0x69, 0xE8, 0xFC};
    if(!device.addESPNOWReceiver(receiverMac)) { printf("ERROR\n"); }

    const uint16_t voltage = (uint16_t) device.readSupplyVoltageFromWakeStub();
    uint8_t data[250] = { 0 };
    data[0] = voltage & 0xFF;
    data[1] = voltage >> 8;

    int8_t txPwr = 0;
	esp_wifi_get_max_tx_power(&txPwr);
	printf("TX POWER: %d\n", txPwr);

    measureTime("sendESPNOWData before");
    if(device.sendESPNOWData(&data[0], 250)) {
        measureTime("sendESPNOWData");
    }
    else {
        printf("Error\n");
    }
    while(1) {
        if(device.getESPNOWSendingStatus() == ESP_NOW_FINISHED_BUT_FAILED) {
            measureTime("getESPNOWSendingStatus");
            printf("Receiver did not get the message\n");
            break;
        }
        else if(device.getESPNOWSendingStatus() == ESP_NOW_FINISHED_AND_RECEIVED) {
            measureTime("getESPNOWSendingStatus");
            printf("Receiver GOT THE MESSAGE\n");
            break;
        }
    }
}

void sendBroadcast() {
    if(!device.initESPNOW(true, RADIO_MAX_TX_19_5_DBM)) { printf("ERROR\n"); }
    measureTime("initESPNOW");
    if(!device.addESPNOWBroadcastReceiver()) { printf("ERROR\n"); }

    const uint16_t voltage = (uint16_t) device.readSupplyVoltageFromWakeStub();
        
    uint8_t data[250] = { 0 };
    data[0] = voltage & 0xFF;
    data[1] = voltage >> 8;

    int8_t txPwr = 0;
	esp_wifi_get_max_tx_power(&txPwr);
	printf("TX POWER: %d\n", txPwr);

    measureTime("broadcastESPNOWData before");
    if(device.broadcastESPNOWData(&data[0], 250)) {
        measureTime("broadcastESPNOWData after");
        printf("Sent with success\n");
    }
    else {
        printf("Error sending the data\n");
    }
}

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            if(!device.rtc.setTimestamp(UNIX_TIMESTAMP+30)) { printf("ERROR\n"); }
            printf("HELLO\n");
            printf("TIME SET TO: %lu\n", (UNIX_TIMESTAMP+30));
            device.rtc.setRegularInterrupt(8);
            firstStart = false;
        }

        printf("Vwakestub: %d\n", device.readSupplyVoltageFromWakeStub());
        printf("Vadc = %d\n", device.readSupplyVoltage());
        device.disableWakeStubNoBootIfVoltageLow();
        device.customWakeStubFunction(wakeStub);
        device.setWakeStubRejectionInterruptSrc(USE_EXT0_IF_WAKE_UP_REJECTED);

        t = (uint32_t) Arduino::millisWrapper();

        sendBroadcast();
        //sendToReceiver();

        printf("\n");
        device.enableRTCInterruptInDeepSleep();
        device.deepSleep();
    }
}