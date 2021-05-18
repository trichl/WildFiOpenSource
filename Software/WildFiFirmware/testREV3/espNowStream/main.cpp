#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

// 2 blocks = 2 * 64 * 2048 Bytes in 4.5s (4.212 on air), avg (11dbm) = 212mA (985uWh = 263uAh = 0.263mAh for 2 blocks, for 2048 blöcke = 269mAh)
// 10 blocks = 10 * 64 * 2048 Bytes in 21.6s, avg (11dbm) = 216mA (4.85mWh / 3.75 = 1.293mAh)
// wifi comparison: 70 seconds for 10 blocks = 5.89mWh / 3.75V = 1.57mAh (but new LDO and 19.5dBm)

RTC_DATA_ATTR bool firstStart = true;
extern uint16_t adcValue;
uint32_t startTime;
bool gotBeaconFromGateway = false;

RTC_DATA_ATTR bool wakeStub() {
    if(adcValue < 1000) {
        return false; // don't boot at all
    }
    return true;
}

static void dataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    printf("RECEIVED DATA!\n");
    gotBeaconFromGateway = true;
}

void broadcast() {
    if(!device.initESPNOW(false, RADIO_MAX_TX_11_DBM, true, WIFI_PHY_RATE_1M_L)) { printf("ERROR\n"); return; } // 320ms
    if(!device.addESPNOWBroadcastReceiver()) { printf("ERROR2\n"); return; }
    uint8_t data[250] = { 0 };
    device.broadcastESPNOWData(data, 250);
}

void sendToReceiver(uint16_t number) {
    if(!device.initESPNOW(false, RADIO_MAX_TX_11_DBM, true, WIFI_PHY_RATE_1M_L)) { printf("ERROR\n"); } // 320ms
    device.measureTime("initESPNOW");
    
    //static uint8_t receiverMac[6] = {0xD8, 0xA0, 0x1D, 0x69, 0xE8, 0xFC}; // PicoKit
    static uint8_t receiverMac[6] = {0xAC, 0x67, 0xB2, 0x2B, 0x4E, 0x88}; // ESP32 CAM
    if(!device.addESPNOWReceiver(receiverMac)) { printf("ERROR\n"); }

    /*uint32_t startTime = ((uint32_t) Arduino::millisWrapper());
    esp_now_register_recv_cb(dataReceived);
    while(gotBeaconFromGateway == false) {
        if(((uint32_t) Arduino::millisWrapper()) - startTime > waitTime) {
            printf("no gateway seen");
            return;
        }
    }
    device.measureTime("gotBeaconFromGateway");*/
    //device.delay(3000);
    
    uint8_t data[250] = { 0 };
    //data[0] = voltage & 0xFF;
    //data[1] = voltage >> 8;

    int8_t txPwr = 0;
	esp_wifi_get_max_tx_power(&txPwr);
	printf("TX POWER: %d\n", txPwr);

    device.measureTime("sendESPNOWData before");
    bool messageNotReceived = false;
    uint32_t duration = 0;
    uint32_t cnt = 0;
    for(uint16_t message=0; message<number; message++) {
        for(uint8_t i=0; i<250; i++) {
            data[i] = message+1;
        }
        if(!device.sendESPNOWData(&data[0], 250)) { // 1ms, was sendESPNowData
            printf("Error\n");
            return;
        }
        cnt++;
        while(1) {
            if(device.getESPNOWSendingStatus() == ESP_NOW_FINISHED_BUT_FAILED) {
                device.measureTime("getESPNOWSendingStatus");
                printf("Receiver did not get the message\n");
                messageNotReceived = true;
                break;
            }
            else if(device.getESPNOWSendingStatus() == ESP_NOW_FINISHED_AND_RECEIVED) { // 10-12ms (long range), 1-4ms in normal mode
                break;
            }
            //device.delay(1);
        }
        duration += device.measureTime("sendESPNOWData", true);
        if(messageNotReceived) {
            break;
        }
    }
    printf("Average send time = %d msX10\n", (duration*10/cnt));
    /*if(esp_now_deinit() != ESP_OK) { // called before deep sleep
        printf("ERROR DEINIT\n");
    }*/
}

void stream() {
    if(!device.initESPNOW(false, RADIO_MAX_TX_19_5_DBM, true, WIFI_PHY_RATE_18M)) { printf("ERROR\n"); } // 320ms
    device.measureTime("initESPNOW");
    
    //static uint8_t receiverMac[6] = {0xD8, 0xA0, 0x1D, 0x69, 0xE8, 0xFC}; // PicoKit
    static uint8_t receiverMac[6] = {0xAC, 0x67, 0xB2, 0x2B, 0x4E, 0x88}; // ESP32 CAM
    if(!device.addESPNOWReceiver(receiverMac)) { printf("ERROR\n"); }

    uint16_t flashBlockPointer = 0; // 0..2047
    uint8_t flashPageInBlockPointer = 0; // 0..63
    uint8_t flashSubPagePointer = 0; // 0..8
    uint16_t flashMaxNumberOfBlocksToTransmit = 8; // 8 blocks = exactly 1 Megabyte, @18M in only 4.892s!
    uint32_t flashPointer = 64 * 10;
    printf("BEFORE TRANSM: flashBlockPointer %d, flashPageInBlockPointer %d, flashSubPagePointer %d\n", flashBlockPointer, flashPageInBlockPointer, flashSubPagePointer);
    esp_now_stream_status_t status = device.doESPNOWFlashStream(10, flashPointer, flashBlockPointer, flashPageInBlockPointer, flashSubPagePointer, flashMaxNumberOfBlocksToTransmit, 3300,
        false, true, false, true);
    device.measureTime("doESPNOWFlashStream");
    printf("Status = %d\n", status);
    if(status != ESP_NOW_STREAM_DATA_FINISHED) { printf("ERROR!\n"); }
    printf("AFTER TRANSM: flashBlockPointer %d, flashPageInBlockPointer %d, flashSubPagePointer %d\n", flashBlockPointer, flashPageInBlockPointer, flashSubPagePointer);
}

extern "C" void app_main() {
    while(1) {
        if(!device.customPhyInit()) {
            printf("PHY INIT ERROR\n");
        }
        if(firstStart) {
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            if(!device.rtc.disableClockOut()) { printf("ERROR\n"); }
            if(!device.rtc.setTimestamp(UNIX_TIMESTAMP+30)) { printf("ERROR\n"); }
            printf("HELLO\n");
            printf("TIME SET TO: %lu\n", (UNIX_TIMESTAMP+30));
            device.rtc.setRegularInterrupt(25);
            firstStart = false; 
        }
        /*else {
            i2c.begin(I2C_FREQ_HZ_400KHZ);
            if(!device.rtc.disableClockOut()) {
                printf("ERROR RTC\n");
            }
        }*/


        //printf("Vwakestub: %d\n", device.readSupplyVoltageFromWakeStub());
        //printf("Vadc = %d\n", device.readSupplyVoltage());
        device.disableWakeStubNoBootIfVoltageLow();
        device.customWakeStubFunction(wakeStub);
        device.setWakeStubRejectionInterruptSrc(USE_EXT0_IF_WAKE_UP_REJECTED);
        //device.measureTime("booted");

        //sendToReceiver(10); // 5 * 240 = 1200 Byte, 100 * 240 = 24kByte (1s in total), 20 * 240 = 4800 byte, 260 * 240 = 62400 Byte
        //stream();
        broadcast();

        printf("\n");
        device.enableRTCInterruptInDeepSleep();
        device.deepSleep();
    }
}