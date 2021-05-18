#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

post_task_parameters_t restParams;
post_task_stream_flash_parameters_t restStreamParams;
const uint32_t DATA_LEN = 16*2048; // >= 28*2048 does not work, 24*2048 WORKS!!!
const uint16_t TASK_STACK_SIZE = 8192;
const uint16_t ITERATIONS = 3;
uint8_t *payload;
uint8_t *dmaStreamBuffer;

RTC_DATA_ATTR bool gotTime = false;
RTC_DATA_ATTR bool firstStart = true;
uint32_t timestampUTC = 0;
uint16_t milliseconds = 0;
uint32_t waitTime = 0;

#define EVERY_X_SECONDS                 5

bool getTimeOverWiFi() {
    if(!device.initWiFi()) { printf("ERROR1\n"); }
    if(!device.connectToWiFiDirectly("Tracker1", "Tracker1", RADIO_MAX_TX_11_DBM, 6)) {
        printf("ERROR2\n");
        return false;
    }
    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) { ; }
    if(device.connectedToWiFi() == WIFI_CONNECT_SUCCESS) {
        timestampUTC = 0;
        milliseconds = 0;
        if(!device.getNTPTimestampUTC(timestampUTC, milliseconds, 9000, "pool.ntp.org")) {
            printf("UTC GET ERROR!\n");
            device.disconnectAndStopWiFi();
            return false;
        }
        if(timestampUTC > 1600000000) { // looks like a valid timestamp from NTP
            if(milliseconds > 1000) {
                milliseconds = 1000;
                printf("StrangeError\n");
            }
            device.delay(1000 - milliseconds); // wait milliseconds until next second is full
            if(!device.rtc.setTimestamp(timestampUTC+1)) {
                device.disconnectAndStopWiFi();
                return false;
            }
            else {
                // assume time is 12.000 now ->
                /*waitTime = 1000 * (EVERY_X_SECONDS - ((timestampUTC+1) % EVERY_X_SECONDS));
                if(waitTime == EVERY_X_SECONDS*1000) {
                    waitTime = 0;
                }
                device.delay(waitTime);
                device.rtc.setRegularInterrupt(EVERY_X_SECONDS);*/
                
                bool error = false;
                waitTime = ((uint32_t) Arduino::millisWrapper());
                while(true) {
                    uint32_t timestamp = device.rtc.getTimestamp(error);
                    if(error) {
                        printf("I2C ERROR\n");
                        return false;
                    }
                    if(timestamp % EVERY_X_SECONDS == 0) {
                        break;
                    }
                    device.delay(10);
                }
                waitTime = ((uint32_t) Arduino::millisWrapper()) - waitTime;
                device.rtc.setRegularInterrupt(EVERY_X_SECONDS);
                
                /*bool error = false;
                waitTime = ((uint32_t) Arduino::millisWrapper());
                while(true) {
                    bool oneSecondPassed = device.rtc.regularInterruptHappened(error);
                    if(error) {
                        printf("ERROR\n");
                        break;
                    }
                    if(oneSecondPassed) {
                        break;
                    }
                    device.delay(4);
                }
                waitTime = ((uint32_t) Arduino::millisWrapper()) - waitTime;*/

                printf("SUCCESS: Timestamp: %d (set to %d), Milliseconds: %d, Waittime: %d\n", timestampUTC, timestampUTC+1, milliseconds, waitTime);
                device.disconnectAndStopWiFi();
                return true;
            }
        }
    }
    else {
        printf("Couldn't connect to WIFI\n");
    }
    return false;
}

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            firstStart = false;
            device.disableWakeStubNoBootIfVoltageLow();
            device.sensorPowerOn(false);
            device.keepSensorPowerOnInDeepSleep();
            device.delay(3000);
            gpio_hold_dis(PIN_LED_RED);
            device.blink(B_RED, B_RED, B_RED);
            gpio_hold_en(PIN_LED_RED);
        }
        else {
            if(device.readSupplyVoltageFromWakeStub() < 3600) {
                gpio_hold_dis(PIN_LED_RED);
                device.blink(B_RED);
                gpio_hold_en(PIN_LED_RED);
                device.deepSleep();
            }
        }

        if(gotTime == false) {
            i2c.begin();
            i2c.setClock(I2C_FREQ_HZ_400KHZ);
            if(getTimeOverWiFi()) {
                gotTime = true;
                device.enableRTCInterruptInDeepSleep();
                printf("GOT TIME -> START\n");
                device.blink(B_GREEN, B_GREEN, B_GREEN);
            }
            else {
                device.enableInternalTimerInterruptInDeepSleep(10);
            }
        }
        else {
            //device.measureTime("BOOT");
            //device.setCPUSpeed(ESP32_10MHZ);
            //device.measureTime("setCPUSpeed");
            device.ledGreenOn();
            device.delay(50);
            device.ledGreenOff();
            //device.measureTime("ledGreen");
            bool error = false;
            i2c.begin();
            //device.measureTime("i2c.begin");
            i2c.setClock(I2C_FREQ_HZ_400KHZ);
            //device.measureTime("i2c.setClock");
            uint32_t timestamp = device.rtc.getTimestamp(error);
            //device.measureTime("getTimestamp");
            if(error) {
                printf("I2C PROBLEM!\n");
            }
            else {
                tmElements_t tm;
                breakTime(timestamp, tm);
                //device.measureTime("breakTime");
                printf("TIMESTAMP: %d, %d:%d:%d\n", timestamp, tm.Hour, tm.Minute, tm.Second);
                //device.measureTime("printf");
            }
            device.enableRTCInterruptInDeepSleep();
            //device.measureTime("enableRTCInterruptInDeepSleep");
        }
        

        device.deepSleep();
    }
}