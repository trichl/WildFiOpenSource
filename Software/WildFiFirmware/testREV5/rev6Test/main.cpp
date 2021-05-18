#include "PlatformWildFiTagREV5.h"
#include <math.h>
#include "ModuleRTC_RV8803C7.h"

WildFiTagREV5 device = WildFiTagREV5();

RTC_RV8803C7 rtc = RTC_RV8803C7();

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint64_t bootCnt = 0;

#define TRANSMISSION_METHOD_ESP_NOW_GATEWAY_AROUND_LEN  2
#define ACTIVATION_AND_TRANS_ESPNOW_RESPONSE_LEN        8
#define ACTIVATION_BY_ESPNOW_RESPONSE                   0x55
#define TRANSMISSION_METHOD_ESP_NOW_HELLO_RESPONSE      0x77
#define TRANSMISSION_METHOD_ESP_NOW_GATEWAY_AROUND      0x66
#define TRANSMISSION_METHOD_ESP_NOW_GATEWAY_AROUND_LEN  2
#define ACTIVATION_AND_TRANS_ESPNOW_RESPONSE_LEN        8
#define ESPNOW_GATEWAY_AROUND_LISTENING_TIME            50                              // also for data transmission, gateway broadcasts every 20ms

uint8_t gatewayAroundEspNowMac[6] = { 0 };
bool gatewayAroundMessageEspNowReceived = false;
uint8_t gatewayAroundMessageCommandByte = 0;
bool timerFinished = false;

bool isGatewayAroundMessage(wifi_promiscuous_pkt_t* p, wifi_promiscuous_pkt_type_t &type) {
    if(type == WIFI_PKT_MGMT) { // all esp now messages are MGMT frames
        if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + TRANSMISSION_METHOD_ESP_NOW_GATEWAY_AROUND_LEN) { // normally 43 bytes additionally + 250 bytes payload
            if(p->rx_ctrl.rate == WIFI_PHY_RATE_18M) { // using the correct proximity data rate
                if(p->payload[ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE] == 0x04) { // is ESP NOW frame
                    if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 0] == TRANSMISSION_METHOD_ESP_NOW_GATEWAY_AROUND)) { // gateway around message
                        return true;
                    } 
                }
            }
        }
    }
    return false;
}

void wifiPromiscuousGatewayAround(void* buffer, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
    if(isGatewayAroundMessage(p, type)) {
        gatewayAroundMessageEspNowReceived = true;
        gatewayAroundEspNowMac[0] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+0];
        gatewayAroundEspNowMac[1] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+1];
        gatewayAroundEspNowMac[2] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+2];
        gatewayAroundEspNowMac[3] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+3];
        gatewayAroundEspNowMac[4] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+4];
        gatewayAroundEspNowMac[5] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+5];
        gatewayAroundMessageCommandByte = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 1];
    }
}

static void proxReceiveCallback(const uint8_t *mac_addr, const uint8_t *data, int data_len) { } // do not do anything here, handled by promiscous sniffer

static void timerCallback(void* arg) { timerFinished = true; }

bool gatewaySeenEspNow(uint8_t sendAnswer, uint8_t *commandByte) {
    uint8_t data[ACTIVATION_AND_TRANS_ESPNOW_RESPONSE_LEN] = { 0 };

    esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuousGatewayAround);
    esp_wifi_set_promiscuous(true);
    esp_now_register_recv_cb(proxReceiveCallback); // 0ms

    const esp_timer_create_args_t timerArgs = { .callback = &timerCallback };
    esp_timer_handle_t timer;
    if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { return false; }
    if(esp_timer_start_once(timer, ESPNOW_GATEWAY_AROUND_LISTENING_TIME * 1000) != ESP_OK) { return false; }

    gatewayAroundMessageEspNowReceived = false;
    gatewayAroundMessageCommandByte = 0;
    while(!timerFinished) {
        vTaskDelay(10 / portTICK_PERIOD_MS); // will send current cpu to sleep (10ms accuracy, will wake up a cycle before that)
        if(gatewayAroundMessageEspNowReceived) {
            if(esp_timer_stop(timer) != ESP_OK) { } // stop timer
            break;
        }
    }

    // send an answer
    if((sendAnswer != 0) && gatewayAroundMessageEspNowReceived) {
        uint16_t voltage = 3750;
        data[0] = sendAnswer;
        data[1] = voltage >> 8;
        data[2] = voltage & 0xFF;
        data[3] = 0;
        data[4] = 0;
        data[5] = 0;
        data[6] = gatewayAroundMessageCommandByte; // mirror the command byte
        data[7] = 100;
        device.broadcastESPNOWData(data, ACTIVATION_AND_TRANS_ESPNOW_RESPONSE_LEN); // spit it out
    }
    *commandByte = gatewayAroundMessageCommandByte;
    return gatewayAroundMessageEspNowReceived;
}

void testSend() {
    if(!device.initESPNOWStationary(0, RADIO_MAX_TX_19_5_DBM, true, WIFI_PHY_RATE_18M)) { // 165ms, performs full calibration I guess (because custom PHY function overflows slow rtc)
        device.stopESPNOW(); // 5ms
        esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
        return;
    }
    if(!device.addESPNOWBroadcastReceiverStationary()) { return; } // necessary for response
    uint8_t commandByte = 0;
    bool gatewaySeen = gatewaySeenEspNow(TRANSMISSION_METHOD_ESP_NOW_HELLO_RESPONSE, &commandByte); // fill commandByte here, also send a hello message when gateway was seen
    if(!gatewaySeen) {
        printf("%d ESPNOW: no gateway seen\n", ((uint32_t) Timing::millis()));
        device.stopESPNOW(); // 5ms
        esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
        return;
    }
    else {
        if(!device.addESPNOWReceiverStationary(gatewayAroundEspNowMac)) {
            printf("%d ESPNOW: addESPNOWReceiver ERROR!\n", ((uint32_t) Timing::millis()));
            device.stopESPNOW(); // 5ms
            esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
            return;
        }
        uint32_t sendPagePointer = 0;
        uint16_t sendPageOffsetPointer = 0;
        uint32_t flashPointer = (2048 * 64) - 1;
        uint16_t flashOffsetPointer = 0;
        esp_now_stream_status_t espNowStatus = device.doESPNOWFlashStreamNew(
            gatewayAroundEspNowMac,
            NULL, 0,
            &sendPagePointer, &sendPageOffsetPointer, 
            flashPointer, flashOffsetPointer, 
            1024*64*2048,
            500, 8, // millis to wait when one message failed (but acks happened before), number of retries
            3300,
            1, false, false); // 0 or (TRACKER_MODE == MODE_TESTRUN) ? 1 : 0, never mock the sending

        printf("STATUS: %d\n", espNowStatus);
        device.stopESPNOW(); // 5ms
        esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
    }
}

void mockFlash() {
    const uint32_t NUMBER_PAGES = 128;
    uint8_t *data = NULL;
    if(!device.flash.createBuffer(&data, MT29_CACHE_SIZE)) {
        printf("ERROR BUFFER\n");
		return;
	}
    for(uint32_t i=0; i<NUMBER_PAGES; i++) {
        for(uint16_t x=0; x<MT29_CACHE_SIZE; x++) {
            data[x] = i;
        }
        if(!device.flash.partialWrite(i, 0, data, MT29_CACHE_SIZE)) {
            printf("ERROR WRITE\n");
            return;
        }
    }
    printf("DONE FLASH WRITING\n");
    if(!device.flash.printFlash(0, NUMBER_PAGES, 2, false)) {
        printf("PRINT ERROR\n");
    }
}

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            firstStart = false;
            printf("HELLO WORLD!\n");

            device.delay(10000);
            device.initDataNVS();
            i2c.begin(I2C_FREQ_HZ_400KHZ);
            if(!device.flashPowerOn(true)) { printf("ERROR\n"); } // turn on flash power already (10ms)
            //mockFlash();
            testSend();
            if(!device.flashPowerOff(false)) { printf("ERROR\n");  } // turn on flash power already (10ms)


            /*if(!rtc.set(23, 59, 55, 4, 5, 3, 2021)) { printf("ERROR SET\n"); }

            //if(!rtc.setTimeUpdateInterruptMinuteChange()) { printf("ERROR MIN CHANGE\n"); }

            bool error = false;
            uint8_t hundreds = rtc.get100thOfSeconds(error);
            if(error) { printf("ERROR HUND\n"); }
            printf("Hundreds: %d\n", hundreds);
            device.delay(200);
            hundreds = rtc.get100thOfSeconds(error);
            if(error) { printf("ERROR HUND\n"); }
            printf("Hundreds: %d\n", hundreds);

            //if(!rtc.setRegularInterrupt(10)) { printf("ERROR regInterrupt\n"); }

            if(!rtc.setDailyInterrupt(0, 2)) { printf("ERROR regInterrupt\n"); }
            */


            /*
            uint32_t timestamp = 0;
            device.delay(980);
            uint64_t start = Timing::millis();
            while(true) {
                timestamp = rtc.getTimestamp(error);
                if(error) { printf("ERROR TIMESTAMP\n"); break; }
                printf("Timestamp: %d\n", timestamp);
                if(Timing::millis() - start > 50) { break; }
            }*/

            /*if(!device.selfTest(3750,
                SELFTEST_VOLTAGE |
                SELFTEST_LEDS |
                SELFTEST_HALLSENSOR |
                //SELFTEST_I2C |
                //SELFTEST_RTC |
                SELFTEST_ACC_GYRO_FOC_CHECK |
                //SELFTEST_ACCFOC_EXECUTE_IF_UNSET |
                SELFTEST_BARO |
                //SELFTEST_FLASH_BAD_BLOCKS |
                //SELFTEST_FLASH_READ_WRITE |
                SELFTEST_FLASH_FULL_ERASE |
                //SELFTEST_NVS_RESET |
                SELFTEST_CPU_CLOCK_DOWN |
                //SELFTEST_WIFI_SCAN |
                SELFTEST_ESPNOW_BROADCAST)) {
                    printf("ERROR\n");
            }*/
        }
        else {
            /*
            i2c.begin(I2C_FREQ_HZ_400KHZ);
            bool error = false;
            uint32_t timestamp = rtc.getTimestamp(error);
            uint8_t hundreds = rtc.get100thOfSeconds(error);
            if(error) { printf("ERROR TIMESTAMP\n"); }
            printf("%llu Timestamp: %d.%d\n", bootCnt, timestamp, hundreds);

            if(rtc.dailyOrHourlyInterruptHappened(error)) { printf("wake by daily/hourly interrupt\n"); }
            else { printf("wake by other interrupt\n"); }

            if(!rtc.resetInterruptFlags()) { printf("ERRORx\n"); }
            */

            /*if(bootCnt == 7) {
                if(!rtc.disableRegularInterrupt()) { printf("DISABLE ERROR\n"); }
                printf("INTERRUPT DISABLED!\n");
            }*/
        }
        //device.enableInternalTimerInterruptInDeepSleep(3);
        bootCnt++;
        //device.enableRTCInterruptInDeepSleep();
        device.deepSleep();
    }
}
