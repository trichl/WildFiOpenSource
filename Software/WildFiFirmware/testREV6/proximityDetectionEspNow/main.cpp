#include "PlatformWildFiTagREV6.h"
#include "ModuleGPS_L70_REV6.h"
#include "configGeneralProximity.h"
#include "configForTest.h"
#include "configOneTimeProgramming.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();
GPS_L70_REV6 gps = GPS_L70_REV6();

// next free error id = 130 (further free )

/* HIGHER PRIORITY */
// TEST: proximity detection with group id filtering
// TEST: id in NVS
// TEST: blob sending function
// TEST: new GPS method with ZDA
    // WARNING: >900ms wait time might fuck up receiving order?!? -> add separate delay in function??
// TEST: memory full

// TODO: randomize gpsFixHours array every day between active hours
// TODO: try out 33 MBPS all the way?
// TODO: proximity aussetzen wenn lange kein GPS re-sync
// TODO: parameter als gateway nachricht
// TODO: reset command

/* CHERRY ON THE CAKE */
// TODO: store acceleration statistics in own proximity msg
// TODO: add barometric pressure, temperature, humidity to data
// TODO: time correction algorithm BETWEEN tags
    // check max/min temp from proximity msg? note how many different tags seen in last day and use average or median?
// TODO: testen long range mit nur esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR); -> wird aber in esp idf example so wie implementiert genutzt
// TODO: tryToGetFix stop more early when no time (not even time, date comes later) received after 60 seconds -> not easy, because only time updates + time there after backup mode
// TODO: Acceleration data burst right before proximity detection

// LIMITATION: max 150 results in proximity detection (more = more space in RAM, also caters for limit of NVS buffer)
// WARNING: measured antenna degredation after some runtime, hence re-doing phyinit from time to time

/*
EXAMPLE DATA: 
 0: id: 01, mac: D8 A0 1D 5D FE E8, recMsgs: 9, rssiMin 39, rssiMax 41, rssiSum 358, rssiAvg 39, timeDiffSum 794, timeDiffAvg 88, lastSync 5257m, approxDist 1.259 - 1.359 - 1.585m
 1: id: 03, mac: D8 A0 1D 5D FF 78, recMsgs: 8, rssiMin 48, rssiMax 49, rssiSum 390, rssiAvg 48, timeDiffSum 1472, timeDiffAvg 184, lastSync 5237m, approxDist 2.113 - 2.712 - 4.467m
 2: id: 02, mac: D8 A0 1D 5D FF CC, recMsgs: 8, rssiMin 44, rssiMax 45, rssiSum 357, rssiAvg 44, timeDiffSum 1408, timeDiffAvg 176, lastSync 5250m, approxDist 1.679 - 1.995 - 2.818m
*/

proximity_entry_t scanResults[PROXIMITY_MAX_TAGS_IN_PARALLEL];
uint16_t scanResultsPointer = 0;
bool scanStop = false;
uint8_t lastCommandReceived = GATEWAY_AROUND_OFFSET_COMMAND_NOTHING;

RTC_DATA_ATTR uint16_t ownTagId = 0;
RTC_DATA_ATTR tracker_state_t state = ST_FIRST_START_HARD_RESET;
RTC_DATA_ATTR uint32_t startCnt = 0;
RTC_DATA_ATTR uint8_t lastErrorId = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;
RTC_DATA_ATTR uint8_t currentMinute = 0, currentHour = 0;
RTC_DATA_ATTR uint16_t lastSync = 1;
RTC_DATA_ATTR uint16_t lastSeenSomeone = 0;
RTC_DATA_ATTR uint16_t proxDetectionsSinceLastFullRFCalib = 0;

/** Time correction between tags */
RTC_DATA_ATTR int16_t RTC_performCorrectionOnNextWakeStubMs = 0; // signed! correction based on received messages

extern uint16_t adcValue;
int64_t timeStartProximity = 0;

/** System start */
RTC_DATA_ATTR bool isActivated = false;
RTC_DATA_ATTR bool hasValidTimestamp = false;
RTC_DATA_ATTR bool bootRefused = false;

RTC_IRAM_ATTR void timeCorrectionShiftBetweenTags(int16_t milliseconds) {
    // at this exact point it is 10:58:00 in my time (seconds = 0)
    if((milliseconds >= 1000) || (milliseconds == 0) || (milliseconds <= -1000)) { return; }
    if(milliseconds < 0) { // I AM TOO FAST! TURN ME BACK! my time: 10:58:00_000, real time 10:57:59_550 -> wait until real time should have minute change, then reset seconds to 0
        milliseconds = -milliseconds; // positive value
        beginSw(); // TODO: add to milliseconds delay? measure how long it takes?
        ets_delay_us(((uint16_t) (milliseconds)) * 1000);
        beginTransmissionSw(RTC_RV8803C7_ADDRESS);
	    writeSw(REG8803_SECONDS);
	    writeSw(0); // reset seconds! (BCD coding of 0 is 0) -> turning clock BACK
	    endTransmissionSw();
    }
    else { // I AM TOO SLOW! TURN ME AHEAD! my time: 10:58:00_000, real time 10:58:00_300 -> wait until real time should reach next second, then reset seconds to 1
        beginSw(); // TODO: add to milliseconds delay? measure how long it takes?
        milliseconds = 1000 - milliseconds;
        ets_update_cpu_frequency_rom(40000000 / 1000000); // re-do calibration (already done in wake stub with ets_get_detected_xtal_freq() = 39997440 = 25ms less after 1 second)
        for(uint8_t i=0; i<(milliseconds / TIME_CORRECTION_WAKE_STUB_FEED_INTERVAL); i++) { // if 512 to wait -> 512 / 300 = 1 -> run one time
            ets_delay_us(TIME_CORRECTION_WAKE_STUB_FEED_INTERVAL * 1000);
            REG_WRITE(TIMG_WDTFEED_REG(0), 1);
        }
        ets_delay_us((milliseconds % TIME_CORRECTION_WAKE_STUB_FEED_INTERVAL) * 1000); // wait rest of the time -> 512 % 300 = 212
        beginTransmissionSw(RTC_RV8803C7_ADDRESS);
	    writeSw(REG8803_SECONDS);
	    writeSw(1); // set to next second (BCD coding of 1 is 1) -> turning clock AHEAD
	    endTransmissionSw();
    }
}

RTC_DATA_ATTR void rtcWaitUntilINTPinLow() {
    deepsleep_for_us(30000); // wait definitely 30ms before checking again
    while(GPIO_INPUT_GET(25) == 0) {
        deepsleep_for_us(30000); // maybe waiting longer if interrupt pin still LOW (interrupt happens)
    }
}

RTC_DATA_ATTR bool wakeStub() { // called multiple times if RTC interrupt not being reset
    if(state != ST_TRACK) { return true; } // NOT doing the stuff when in power down state, just booting up 
    if(bootRefused) { bootRefused = false; return false; } // INT pin still LOW but boot was refused and rtcWaitUntilINTPinLow called (restarting wakeStub) -> return false
    currentMinute++;
    if(currentMinute >= 60) { currentMinute = 0; currentHour++; } // 12:59 -> 13:00 (next full hour)
    if(currentHour >= 24) { currentHour = 0; } // 23:59 -> 00:00 (next day)
    if(lastSync < 65535) { lastSync++; }
    if(lastSeenSomeone < 65535) { lastSeenSomeone++; }

    /** ON MINUTE 0 OF MODULO - PROXIMITY DETECTION AND MAYBE GPS */
    if((currentMinute % PROXIMITY_FREQUENCY_IN_MINUTES) == 0) {
        return true; // time to do proximity detection -> ALWAYS TRUE IF PROXIMITY DETECTION EVERY 1 MINUTE
    }
    /** ON MINUTE 1 - X - OTHER STUFF */
    else if(RTC_performCorrectionOnNextWakeStubMs != 0) { // not waking up, but a correction request is there -> good time to correct and then sleep afterwards again
        timeCorrectionShiftBetweenTags(RTC_performCorrectionOnNextWakeStubMs);
        RTC_performCorrectionOnNextWakeStubMs = 0; // correction done, don't redo it after boot-up
        bootRefused = true;
        rtcWaitUntilINTPinLow(); // I2C already started, clear RTC interrupt (otherwise wakeStub called again if returning false because interrupt pin is low for ~8ms)
    }
    else { // not waking up and no time correction needed -> reset interrupt pin and sleep
        bootRefused = true;
        rtcWaitUntilINTPinLow(); // clear RTC interrupt (otherwise wakeStub called again if returning false because interrupt pin is low for ~8ms)
    }
    return true;
}

void startIMU() {
    acc_config_t accConfig = { BMX160_ACCEL_ODR_0_78HZ, BMX160_ACCEL_BW_RES_AVG8, BMX160_ACCEL_RANGE_8G }  ;
    device.sensorPowerOn(); 
    device.shortLightSleep(120); // wait until bmx is booted
    if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 31; errorCnt++; }
    if(!device.imu.initFIFO(BMX160_INIT_FIFO_FOR_ACC)) { lastErrorId = 32; errorCnt++; }
    if(!device.imu.resetFIFO()) { lastErrorId = 33; errorCnt++; }
}

void stopIMU() {
    if(!device.imu.stop()) { lastErrorId = 37; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
    device.sensorPowerOff(); // turn off IMU (and environment sensor) completely
    device.shortLightSleep(100); // wait because otherwise interrupt pin might still be valid
}

uint16_t getApproximatedAirTimeMs() {
    // TODO: not valid in long range mode!!!!
    if(PROXIMITY_AIR_TIME_US % 1000 < 500) { return (PROXIMITY_AIR_TIME_US / 1000); } // floor
    else { return (PROXIMITY_AIR_TIME_US / 1000) + 1; } // ceil
}

float getApproximatedDistance(uint8_t rssi, uint8_t N) {
    const int8_t A_rssiAt1Meter = -35; // measured once
    int8_t rssiNegative = rssi;
    rssiNegative = -rssiNegative;
    if((N < 2) || (N > 4)) { return 0.0; } // not valid
    float result = pow(10.0, ((float) (A_rssiAt1Meter - rssiNegative)) / ((float)(10 * N)));
    return result;
}

bool macsAreSame(uint8_t *mac1, uint8_t *mac2) {
    return ((mac1[0] == mac2[0]) && (mac1[1] == mac2[1]) && (mac1[2] == mac2[2]) && (mac1[3] == mac2[3]) && (mac1[4] == mac2[4]) && (mac1[5] == mac2[5]));
}

int16_t calculateTimeDifference(uint8_t *realPayload) {
    uint16_t receivedTimeOffset = 0;
    receivedTimeOffset = realPayload[PROX_PAYLOAD_OFFSET_SENDOFFSET_0]; 
    receivedTimeOffset = (receivedTimeOffset << 8) | realPayload[PROX_PAYLOAD_OFFSET_SENDOFFSET_1];

    int64_t myTimeOffsetTemp = (esp_timer_get_time() - timeStartProximity) / 1000;
    int16_t receivedTimeOffsetSigned = receivedTimeOffset + getApproximatedAirTimeMs();
    int16_t myTimeOffsetSigned = myTimeOffsetTemp;
    int16_t timeDifferenceSigned = receivedTimeOffsetSigned - myTimeOffsetSigned;

    return timeDifferenceSigned;
}

void checkIfTimeCorrectionNeeded() {
    if(scanResultsPointer == 0) { // no time correction when nobody around
        return;
    }
    int16_t timeDifferenceOfMinLastSync = (scanResults[0].timeDifferenceSumMs / scanResults[0].receivedMessages);
    uint16_t minLastSync = scanResults[0].lastSync;
    for(uint16_t i=1; i<scanResultsPointer; i++) {
        if(scanResults[i].lastSync < minLastSync) {
            timeDifferenceOfMinLastSync = (scanResults[i].timeDifferenceSumMs / scanResults[i].receivedMessages);
            minLastSync = scanResults[i].lastSync;
        } 
    }
    if(TRACKER_MODE == MODE_TESTRUN) { printf("TIMECORR: MinLastSync %d minutes, timeDiffToSyncTo %d\n", minLastSync, timeDifferenceOfMinLastSync); }
    // TODO: only if my RTC_lastSync is some minutes ago or if time difference is REALLY high?
    if((timeDifferenceOfMinLastSync > 5) || (timeDifferenceOfMinLastSync < -5)) {
        if(TRACKER_MODE == MODE_TESTRUN) { printf("TIMECORR: do correct in next wake stub!\n"); }
        RTC_performCorrectionOnNextWakeStubMs = timeDifferenceOfMinLastSync; // next wake stub: perform the sync
        lastSync = 1; // reset MY last sync! -> means everyone else might update to my time
    }
}

void checkForHugeTimeDifferences() {
    if(scanResultsPointer == 0) { return; } // nobody around
    int16_t timeDifferenceAvg = 0;
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        timeDifferenceAvg = (scanResults[i].timeDifferenceSumMs / scanResults[i].receivedMessages);
        if(timeDifferenceAvg > 600) {
            if(TRACKER_MODE == MODE_TESTRUN) { printf("WARNING: huge timediff %dms (no %d) detected!\n", timeDifferenceAvg, i); }
            lastErrorId = 41;
            errorCnt++;
        }
    }   
}

void printScanResult() {
    printf("SCAN RESULT: found %d tag(s)\n", scanResultsPointer);
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        printf(" %d: id: %04X, gw: %d, mac: %02X %02X %02X %02X %02X %02X, recMsgs: %d, rssiMin %d, rssiMax %d, rssiSum %d, rssiAvg %d, timeDiffSum %d, timeDiffAvg %d, lastSync %dm, approxDist %.3f - %.3f - %.3fm\n", i, scanResults[i].id, scanResults[i].isGateway, scanResults[i].mac[0], scanResults[i].mac[1], scanResults[i].mac[2], scanResults[i].mac[3], scanResults[i].mac[4], scanResults[i].mac[5],
            scanResults[i].receivedMessages, scanResults[i].rssiMin, scanResults[i].rssiMax, scanResults[i].rssiSum, (scanResults[i].rssiSum / scanResults[i].receivedMessages), scanResults[i].timeDifferenceSumMs, (scanResults[i].timeDifferenceSumMs / scanResults[i].receivedMessages), scanResults[i].lastSync,
            getApproximatedDistance(scanResults[i].rssiSum / scanResults[i].receivedMessages, 4), getApproximatedDistance(scanResults[i].rssiSum / scanResults[i].receivedMessages, 3), getApproximatedDistance(scanResults[i].rssiSum / scanResults[i].receivedMessages, 2));
    }
}

void addProximityMessageToScanResult(wifi_promiscuous_pkt_t* p) {
    uint8_t rssiAbs;
    int8_t rssiTemp;
    if(scanResultsPointer >= PROXIMITY_MAX_TAGS_IN_PARALLEL) { lastErrorId = 66; errorCnt++; } // should not happen -> if so, stop adding
    else {
        // check if already in list
        for(uint16_t i=0; i<scanResultsPointer; i++) {
            if(macsAreSame(&scanResults[i].mac[0], &p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC])) { // WARNING: compares real MAC not ID
                scanResults[i].timeDifferenceSumMs = scanResults[i].timeDifferenceSumMs + calculateTimeDifference(&p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD]); // be fast here!
                scanResults[i].receivedMessages = scanResults[i].receivedMessages + 1; // increment message counter
                if(p->rx_ctrl.rssi < 0) { rssiTemp = -(p->rx_ctrl.rssi); } // calculate absolute rssi value
                else { rssiTemp = p->rx_ctrl.rssi; }
                rssiAbs = rssiTemp;
                if(rssiAbs > scanResults[i].rssiMax) { scanResults[i].rssiMax = rssiAbs; }
                if(rssiAbs < scanResults[i].rssiMin) { scanResults[i].rssiMin = rssiAbs; }
                scanResults[i].rssiSum = scanResults[i].rssiSum + rssiAbs;
                return;
            }
        }
        // not in list -> add 
        uint16_t id = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_ID_0];
        scanResults[scanResultsPointer].timeDifferenceSumMs = calculateTimeDifference(&p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD]); // be fast here!
        scanResults[scanResultsPointer].id = (id << 8) | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_ID_1];
        scanResults[scanResultsPointer].mac[0] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+0];
        scanResults[scanResultsPointer].mac[1] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+1];
        scanResults[scanResultsPointer].mac[2] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+2];
        scanResults[scanResultsPointer].mac[3] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+3];
        scanResults[scanResultsPointer].mac[4] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+4];
        scanResults[scanResultsPointer].mac[5] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+5];
        scanResults[scanResultsPointer].receivedMessages = 1;
        scanResults[scanResultsPointer].isGateway = false;

        uint16_t lastSync = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_LASTSYNC_0];
        lastSync = (lastSync << 8) | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_LASTSYNC_1];
        scanResults[scanResultsPointer].lastSync = lastSync; // only add lastSync from FIRST message, assuming that this will not change -> also faster

        if(p->rx_ctrl.rssi < 0) { rssiTemp = -(p->rx_ctrl.rssi); } // calculate absolute rssi value
        else { rssiTemp = p->rx_ctrl.rssi; }
        rssiAbs = rssiTemp;

        scanResults[scanResultsPointer].rssiMax = rssiAbs;
        scanResults[scanResultsPointer].rssiMin = rssiAbs;
        scanResults[scanResultsPointer].rssiSum = rssiAbs;
        
        scanResultsPointer++;
        lastSeenSomeone = 0;
    }
}

void addGatewayAroundMessageToScanResult(wifi_promiscuous_pkt_t* p) {
    uint8_t rssiAbs;
    int8_t rssiTemp;
    if(scanResultsPointer >= PROXIMITY_MAX_TAGS_IN_PARALLEL) { lastErrorId = 18; errorCnt++; } // should not happen -> if so, stop adding
    else {
        // check if already in list
        for(uint16_t i=0; i<scanResultsPointer; i++) {
            if(macsAreSame(&scanResults[i].mac[0], &p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC])) { // WARNING: compares real MAC not ID
                scanResults[i].receivedMessages = scanResults[i].receivedMessages + 1; // increment message counter
                if(p->rx_ctrl.rssi < 0) { rssiTemp = -(p->rx_ctrl.rssi); } // calculate absolute rssi value
                else { rssiTemp = p->rx_ctrl.rssi; }
                rssiAbs = rssiTemp;
                if(rssiAbs > scanResults[i].rssiMax) { scanResults[i].rssiMax = rssiAbs; }
                if(rssiAbs < scanResults[i].rssiMin) { scanResults[i].rssiMin = rssiAbs; }
                scanResults[i].rssiSum = scanResults[i].rssiSum + rssiAbs;
                return;
            }
        }
        // not in list -> add 
        uint16_t id = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_ID_0];
        scanResults[scanResultsPointer].id = (id << 8) | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_ID_1];
        scanResults[scanResultsPointer].mac[0] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+0];
        scanResults[scanResultsPointer].mac[1] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+1];
        scanResults[scanResultsPointer].mac[2] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+2];
        scanResults[scanResultsPointer].mac[3] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+3];
        scanResults[scanResultsPointer].mac[4] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+4];
        scanResults[scanResultsPointer].mac[5] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+5];
        scanResults[scanResultsPointer].receivedMessages = 1;

        scanResults[scanResultsPointer].timeDifferenceSumMs = 0; // do not use time difference sum
        scanResults[scanResultsPointer].lastSync = 0xFFFF; // do not use last sync for gateways
        scanResults[scanResultsPointer].isGateway = true;

        if(p->rx_ctrl.rssi < 0) { rssiTemp = -(p->rx_ctrl.rssi); } // calculate absolute rssi value
        else { rssiTemp = p->rx_ctrl.rssi; }
        rssiAbs = rssiTemp;

        scanResults[scanResultsPointer].rssiMax = rssiAbs;
        scanResults[scanResultsPointer].rssiMin = rssiAbs;
        scanResults[scanResultsPointer].rssiSum = rssiAbs;
        
        scanResultsPointer++;
        lastSeenSomeone = 0;
    }
}

void wifiPromiscuous(void* buffer, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
    if(!scanStop) {
        if(type == WIFI_PKT_MGMT) { // all esp now messages are MGMT frames
            if(p->rx_ctrl.rate == PROXIMITY_DATARATE) { // using the correct proximity data rate
                if(p->payload[ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE] == 0x04) { // is ESP NOW frame
                    // proximity message
                    if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + PROXIMITY_DATA_LEN) { // normally 43 bytes additionally + 250 bytes payload
                        // same group id
                        if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_GROUP_0] == PROXIMITY_OWN_GROUP_0) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_GROUP_1] == PROXIMITY_OWN_GROUP_1) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_GROUP_2] == PROXIMITY_OWN_GROUP_2) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_GROUP_3] == PROXIMITY_OWN_GROUP_3)) {
                            if(p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY) { // proximity message
                                addProximityMessageToScanResult(p);
                            }
                        }
                    }
                    // gateway around message
                    else if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + 8) { // normally 43 bytes additionally + 8 bytes payload
                        // same group id
                        if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_0] == PROXIMITY_OWN_GROUP_0) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_1] == PROXIMITY_OWN_GROUP_1) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_2] == PROXIMITY_OWN_GROUP_2) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_3] == PROXIMITY_OWN_GROUP_3)) {
                            if(p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND) { // gateway around message
                                addGatewayAroundMessageToScanResult(p);
                            }
                        }
                    }
                }
            }
        }
    }
}

void wifiPromiscuousActivation(void* buffer, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
    if(type == WIFI_PKT_MGMT) { // all esp now messages are MGMT frames
        if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + 8) { // gateway around msg is 8 bytes long
            if(p->rx_ctrl.rate == PROXIMITY_DATARATE) { // using the correct proximity data rate
                if(p->payload[ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE] == 0x04) { // is ESP NOW frame
                    if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND)) { // gateway around message
                        if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_0] == PROXIMITY_OWN_GROUP_0) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_1] == PROXIMITY_OWN_GROUP_1) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_2] == PROXIMITY_OWN_GROUP_2) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_3] == PROXIMITY_OWN_GROUP_3)) { // same group id
                            lastCommandReceived = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_COMMAND];
                        }
                    } 
                }
            }
        }
    }
}

static void proxReceiveCallback(const uint8_t *mac_addr, const uint8_t *data, int data_len) { } // do not do anything here, handled by promiscous sniffer

bool timerFinished = false;

static void timerCallback(void* arg) {
    timerFinished = true;
}

bool getTimeOverWiFiNew() {
    bool gotTimeOverWifi = false;
    if(device.initWiFi()) {
        uint8_t foundArrayId = 0;
        uint8_t foundOnChannel = 0;
        bool connectionTimeout = false;
        uint32_t scanStartTime = ((uint32_t) Timing::millis());
        if(device.scanForWiFisOn1and6and11and13WithPriority((TRACKER_MODE == MODE_TESTRUN), TIME_WIFI_SSIDS, TIME_WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, TIME_WIFI_OUTPUT_POWER, 120, 500)) { 
            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500); }
            if(foundOnChannel > 0) { // found wifi, try to connect
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: wifi found (chan %d, index %d), hasValidTimestamp: %d\n", foundOnChannel, foundArrayId, hasValidTimestamp); }
                if(device.connectToWiFiAfterScan(TIME_WIFI_SSIDS[foundArrayId], TIME_WIFI_PASSWORDS[foundArrayId], foundOnChannel)) {
                    uint32_t connectStartedTime = ((uint32_t) Timing::millis());
                    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                        device.delay(20);
                        if(((uint32_t) Timing::millis()) - connectStartedTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: TIMEOUT CONNECT!\n"); }
                            connectionTimeout = true;
                            break;
                        }
                    }
                    if(connectionTimeout || (device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: STRANGE could not connect, but wifi was seen (pw wrong, no internet): %d!\n", device.connectedToWiFi()); }
                    }
                    else { // connected to wifi
                        uint32_t timestampUTC = 0;
                        uint16_t millisecondsUTC = 0;
                        if(!device.getNTPTimestampUTC(true, timestampUTC, millisecondsUTC, 7000, "pool.ntp.org")) { // will block, will set RTC time
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: UTC get time error!\n"); }
                        }
                        else {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: set %d + 1, UTC milliseconds: %d -> waited for %dms\n", timestampUTC, millisecondsUTC, 1000 - millisecondsUTC); }
                            gotTimeOverWifi = true;
                        }
                    }
                    device.disconnectAndStopWiFi();
                }
                else { device.disconnectAndStopWiFi(); }
                // check result of getting timestamp attempt
                if(gotTimeOverWifi) { return true; }
                else {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: wifi was seen, but still something missing (pw wrong, no internet)\n"); }
                    return false;
                }
            }
            else {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: wifi NOT found\n"); }
                device.disconnectAndStopWiFi();  
                return false;
            }
        }
        else { device.disconnectAndStopWiFi(); return false; }
    }
    return false;
}

void fillDataPayload(uint8_t *data, uint8_t dataLen) {
    int64_t temp = (esp_timer_get_time() - timeStartProximity) / 1000; // current time within sending window in ms
    uint16_t sendOffset = (uint16_t) temp;
    uint16_t voltage = (uint16_t) adcValue;
    data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] = ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY;
    data[PROX_PAYLOAD_OFFSET_OWN_GROUP_0] = PROXIMITY_OWN_GROUP_0;
    data[PROX_PAYLOAD_OFFSET_OWN_GROUP_1] = PROXIMITY_OWN_GROUP_1;
    data[PROX_PAYLOAD_OFFSET_OWN_GROUP_2] = PROXIMITY_OWN_GROUP_2;
    data[PROX_PAYLOAD_OFFSET_OWN_GROUP_3] = PROXIMITY_OWN_GROUP_3;
    data[PROX_PAYLOAD_OFFSET_OWN_ID_0] = ownTagId >> 8;
    data[PROX_PAYLOAD_OFFSET_OWN_ID_1] = ownTagId & 0xFF;
    data[PROX_PAYLOAD_OFFSET_SENDOFFSET_0] = sendOffset >> 8;
    data[PROX_PAYLOAD_OFFSET_SENDOFFSET_1] = sendOffset & 0xFF;
    data[PROX_PAYLOAD_OFFSET_LASTSYNC_0] = lastSync >> 8;
    data[PROX_PAYLOAD_OFFSET_LASTSYNC_1] = lastSync & 0xFF;
    data[PROX_PAYLOAD_OFFSET_VOLTAGE_0] = voltage >> 8;
    data[PROX_PAYLOAD_OFFSET_VOLTAGE_1] = voltage & 0xFF;
    data[PROX_PAYLOAD_OFFSET_LASTERRORID] = lastErrorId;
    data[PROX_PAYLOAD_OFFSET_ERRORCNT_0] = errorCnt >> 8;
    data[PROX_PAYLOAD_OFFSET_ERRORCNT_1] = errorCnt & 0xFF;
}

bool isGatewayInScanResults() {
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        if(scanResults[i].isGateway == true) { return true; }
    }
    return false;
}

bool activationDetection() {
    uint8_t data[250] = { 0 };
    lastCommandReceived = GATEWAY_AROUND_OFFSET_COMMAND_NOTHING;
    if(!device.initESPNOWStationary(PROXIMITY_LONGRANGE, PROXIMITY_DBM, true, PROXIMITY_DATARATE)) { lastErrorId = 59; errorCnt++; return false; } // 23ms
    if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 60; errorCnt++; return false; } // 0ms

    esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuousActivation);
    esp_wifi_set_promiscuous(true);
    esp_now_register_recv_cb(proxReceiveCallback); // 0ms

    const esp_timer_create_args_t timerArgs = { .callback = &timerCallback };
    esp_timer_handle_t timer;
    if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { lastErrorId = 61; errorCnt++; return false; }
    if(esp_timer_start_once(timer, ACTIVATION_BY_GATEWAY_LISTENING_TIME * 1000) != ESP_OK) { lastErrorId = 62; errorCnt++; return false; }

    while(!timerFinished) {
        vTaskDelay(10 / portTICK_PERIOD_MS); // will send current cpu to sleep (10ms accuracy, will wake up a cycle before that)
        if(lastCommandReceived == GATEWAY_AROUND_OFFSET_COMMAND_ACTIVATE) {
            if(esp_timer_stop(timer) != ESP_OK) { lastErrorId = 63; errorCnt++; } // stop timer
            break;
        }
    }

    // send tag around message (in any case)
    data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] = ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND;
    data[TAG_AROUND_OFFSET_OWN_GROUP_0] = PROXIMITY_OWN_GROUP_0;
    data[TAG_AROUND_OFFSET_OWN_GROUP_1] = PROXIMITY_OWN_GROUP_1;
    data[TAG_AROUND_OFFSET_OWN_GROUP_2] = PROXIMITY_OWN_GROUP_2;
    data[TAG_AROUND_OFFSET_OWN_GROUP_3] = PROXIMITY_OWN_GROUP_3;
    data[TAG_AROUND_OFFSET_OWN_ID_0] = ownTagId >> 8;
    data[TAG_AROUND_OFFSET_OWN_ID_1] = ownTagId & 0xFF;
    data[TAG_AROUND_OFFSET_VOLTAGE_0] = adcValue >> 8;
    data[TAG_AROUND_OFFSET_VOLTAGE_1] = adcValue & 0xFF;
    data[TAG_AROUND_OFFSET_LASTERRORID] = lastErrorId;
    data[TAG_AROUND_OFFSET_ERRORCNT_0] = errorCnt >> 8;
    data[TAG_AROUND_OFFSET_ERRORCNT_1] = errorCnt & 0xFF;
    data[TAG_AROUND_OFFSET_CMD_BYTE_MIRRORED] = lastCommandReceived;
    data[TAG_AROUND_OFFSET_STATE] = (uint8_t) state;
    data[TAG_AROUND_OFFSET_IS_ACTIVATED] = isActivated;
    data[TAG_AROUND_OFFSET_IS_ACTIVATED] = TAG_AROUND_OFFSET_HAS_VALID_TIMESTAMP;
    device.broadcastESPNOWData(data, 16); // spit it out  

    if(lastCommandReceived == GATEWAY_AROUND_OFFSET_COMMAND_ACTIVATE) {
        // send activation message
        data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] = ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED;
        data[GOT_ACTIVATED_OFFSET_OWN_GROUP_0] = PROXIMITY_OWN_GROUP_0;
        data[GOT_ACTIVATED_OFFSET_OWN_GROUP_1] = PROXIMITY_OWN_GROUP_1;
        data[GOT_ACTIVATED_OFFSET_OWN_GROUP_2] = PROXIMITY_OWN_GROUP_2;
        data[GOT_ACTIVATED_OFFSET_OWN_GROUP_3] = PROXIMITY_OWN_GROUP_3;
        data[GOT_ACTIVATED_OFFSET_OWN_ID_0] = ownTagId >> 8;
        data[GOT_ACTIVATED_OFFSET_OWN_ID_1] = ownTagId & 0xFF;
        device.broadcastESPNOWData(data, 7); // spit it out
        return true;
    }
    return false;
}

bool proximityDetection() { // 1065ms
    uint8_t data[PROXIMITY_DATA_LEN] = { 0 };

    if(!device.initESPNOWStationary(PROXIMITY_LONGRANGE, PROXIMITY_DBM, true, PROXIMITY_DATARATE)) { lastErrorId = 19; errorCnt++; return false; } // 23ms
    if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 20; errorCnt++; return false; } // 0ms

    esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuous);
    esp_wifi_set_promiscuous(true);
    esp_now_register_recv_cb(proxReceiveCallback); // 0ms

    const esp_timer_create_args_t timerArgs = { .callback = &timerCallback };
    esp_timer_handle_t timer;
    if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { lastErrorId = 21; errorCnt++; return false; }
    if(esp_timer_start_once(timer, PROXIMITY_LISTENING_INTERVAL_MS * 1000) != ESP_OK) { lastErrorId = 22; errorCnt++; return false; }

    timeStartProximity = esp_timer_get_time();
    int64_t timeLastSent = timeStartProximity;
    // 6ms because @1MBPS I see 5.2ms sending peaks (guess with ACK), sending time should normally be: (250 * 8 * 1000 * 1000) / (1 * 1024 * 1024) = 1.9ms
    int64_t timeBetweenSending = ((ownTagId * 6 * 1000) % (100 * 1000)); // (6ms * ownTagId) -> adding MAX. 96ms, so modulo, from id 17 it will wrap around again (but with different offset because 100/6 has rest)
    uint8_t sendCounter = 0;

    while(!timerFinished) {
        if((esp_timer_get_time() - timeLastSent) >= timeBetweenSending) {
            timeLastSent = esp_timer_get_time(); // reset send counter
            timeBetweenSending = 100 * 1000; // 100ms until next 
            fillDataPayload(data, PROXIMITY_DATA_LEN); // fill data frame
            device.broadcastESPNOWData(data, PROXIMITY_DATA_LEN); // spit it out
            sendCounter++;
            if(sendCounter == 2) {
                if(USE_LEDS) { device.ledGreenOff(); }
            }
            if(sendCounter < 10) { // = 10 = last message, don't delay here because shorter than 100ms
                // TODO: TEST vTaskDelayUntil
                vTaskDelay(90 / portTICK_PERIOD_MS); // will send current cpu to sleep (10ms accuracy, will wake up a cycle before that)
            }
        }
    }
    // ESP NOW still running! to send out data in case a gateway has been seen
    return true;
}

void printArray(uint8_t *pointer, uint16_t len) {
	for(uint16_t i=0; i<len; i++) { printf("%02X ", pointer[i]); }
	printf("\n");
}

void printArrayWithBreak(uint8_t *pointer, uint16_t len, uint16_t breakAfter) {
	for(uint16_t i=0; i<len; i++) {
		printf("%02X ", pointer[i]);
		if(i % breakAfter == (breakAfter - 1)) { printf("\n\n"); }
	}
	printf("\n");
}

push_data_result_t pushDataIntoMemory(uint8_t *newData, uint32_t newDataLen, bool *memoryFull, bool debug) { // MAX 512 byte
    esp_err_t err;
    nvs_handle_t handle;
    size_t nvsDataLen = 0;  // value will default to 0, if not set yet in NVS
    uint8_t nvsData[NVS_FLASH_BUFFER_SIZE] = { 0 };
    push_data_result_t returnVal = PUSH_DATA_SUCCESS;

    if((newDataLen == 0) || (newData == NULL) || (newDataLen > 512)) { return PUSH_DATA_PARAM_ERROR; }

    // get current size of 512 byte NVS buffer
    if(nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle) != ESP_OK) { return PUSH_DATA_NVS_ERROR; }

    err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, NULL, &nvsDataLen); // 0ms
    if(err == ESP_ERR_NVS_NOT_FOUND) { // first time, blob is empty
        if(debug) { printf("pushData: virgin NVS\n"); }
        nvsDataLen = 0;
    }
    else if(err != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_GET_BLOB_ERROR; }

    // read 512 byte NVS buffer (in case buffer is not empty)
    if(debug) { printf("pushData: %d bytes in NVS\n", nvsDataLen); }
    if(nvsDataLen > 0) {  
        err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, &nvsDataLen); // 1ms
        if(err != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_GET_BLOB_ERROR2; }
        /*if(debug) {
            for(uint16_t i = 0; i < nvsDataLen; i++) { printf("%02X ", nvsData[i]); } 
            printf("\n");
        }*/
    }

    if(nvsDataLen + newDataLen <= NVS_FLASH_BUFFER_SIZE) { // if new data fits into NVS, might be 512 afterwards (next cycle will write into flash)
        if(debug) { printf("pushData: FITS %d + %d <= %d\n", nvsDataLen, newDataLen, NVS_FLASH_BUFFER_SIZE); }
        //if(debug) { printf("+ "); }

        // append new data
        for(uint32_t i = 0; i < newDataLen; i++) {
            nvsData[nvsDataLen + i] = newData[i];
            //if(debug) { printf("%02X ", newData[i]); }
        }
        //if(debug) { printf("\n"); }

        // write + commit NVS buffer, nvs_set_blob = TIME EATER
        if(nvs_set_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, nvsDataLen + newDataLen) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_SET_BLOB_ERROR; } // TIME CONSUMING: 7 - 117ms
        if(nvs_commit(handle) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_COMMIT_ERROR; } // 0ms
        nvs_close(handle); // 0ms
    }
    else { // if new data DOES NOT FIT into NVS
        // append bytes to fill 512 byte array completely
	    if(debug) { printf("pushData: DOESNT FIT %d + %d > %d -> WRITE FLASH\n", nvsDataLen, newDataLen, NVS_FLASH_BUFFER_SIZE); }
	    uint32_t newDataLeftPointer = 0;
	    for(newDataLeftPointer = 0; newDataLeftPointer < newDataLen; newDataLeftPointer++) {
	        if(nvsDataLen + newDataLeftPointer >= NVS_FLASH_BUFFER_SIZE) { break; }
	        nvsData[nvsDataLen + newDataLeftPointer] = newData[newDataLeftPointer];
	    }

        // write 512 byte array into flash memory
        // WARNING: flash needs to be activated here
        uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
        uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);

        if(debug) { printf("pushData: before: writePointer: %d.%d\n", flashPageWritePointer, flashPageWriteOffsetPointer); }
        sequential_write_status_t writeStatus = device.flash.fifoPushSimple(0, flashPageWritePointer, flashPageWriteOffsetPointer, nvsData, NVS_FLASH_BUFFER_SIZE, true, MOCK_FLASH_WRITES);
        if(debug) { printf("pushData: after: writePointer: %d.%d, FIFO space: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer, device.flash.fifoGetFreeSpace(0, flashPageWritePointer, flashPageWriteOffsetPointer, MT29_NUMBER_PAGES)); }
        if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full
            if(debug) { printf("pushData: flash full\n"); }
            *memoryFull = true;
        }
        else { // MT29_SEQ_WRITE_STATUS_SUCCESS or MT29_SEQ_WRITE_STATUS_ERROR, also update pointers in case of ERROR
            if(!device.nvsWriteUINT32andUINT16(NVS_FLASH_WRITE_PAGE_POINTER, flashPageWritePointer, NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, flashPageWriteOffsetPointer)) { 
                returnVal = PUSH_DATA_NVS_WRITE_ERROR; // don't stop in case of error
            }
            if(writeStatus == MT29_SEQ_WRITE_STATUS_BUFFER_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_BUFFER_ERROR; } // don't stop in case of error
            else if(writeStatus == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR; } // don't stop in case of error
            else if(writeStatus == MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR; } // don't stop in case of error

            // handle blob pointer
            uint16_t nvsSendBlobPointer = device.nvsReadUINT16(NVS_FLASH_SEND_BLOB_POINTER);
            if(nvsSendBlobPointer > 0) { // already sent something with mocked pointer addresses from blob
                uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_PAGE_POINTER);
                uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
                if(debug) { printf("pushData: did send %d bytes from blob before, sendPointer %d.%d -> ", nvsSendBlobPointer, sendPagePointer, sendPageOffsetPointer); }
                sendPageOffsetPointer += nvsSendBlobPointer;
                if(sendPageOffsetPointer >= MT29_CACHE_SIZE) {
                    sendPageOffsetPointer = sendPageOffsetPointer % MT29_CACHE_SIZE;
                    sendPagePointer++;
                    if(sendPagePointer >= MT29_NUMBER_PAGES) { sendPagePointer = 0; lastErrorId = 30; errorCnt++; } // this should never happen with 512 bytes chunks
                    if(!device.nvsWriteUINT32(NVS_FLASH_SEND_PAGE_POINTER, sendPagePointer)) { returnVal = PUSH_DATA_NVS_WRITE_ERROR; }
                }
                if(!device.nvsWriteUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER, sendPageOffsetPointer)) { lastErrorId = 45; errorCnt++; }
                if(!device.nvsWriteUINT16(NVS_FLASH_SEND_BLOB_POINTER, 0)) { returnVal = PUSH_DATA_NVS_WRITE_ERROR; } // reset blob send pointer to 0
                if(debug) { printf("%d.%d\n", sendPagePointer, sendPageOffsetPointer); }
            }
        }

        // add rest data to "fresh" 512 byte array
        uint32_t nvsDataCnt = 0;
	    for(uint32_t i = newDataLeftPointer; i < newDataLen; i++) {
            nvsData[nvsDataCnt] = newData[i];
            nvsDataCnt++;
	    	//if(debug) { printf("%02X - ", newData[i]); }	
	    }
    	//if(debug) { printf("\n"); }
        nvsDataLen = nvsDataCnt;
        
        // write + commit rest of data to NVS
        if(nvs_set_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, nvsDataLen) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_SET_BLOB_ERROR2; } // TIME CONSUMING: 7 - 117ms
        if(nvs_commit(handle) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_COMMIT_ERROR2; } // 0ms
        nvs_close(handle); // 0ms
    }
    return returnVal;
}

void storeGPSDataInFlash(uint32_t timestamp, esp_gps_t *gpsData, bool *memoryFull) {
    uint8_t *sensorData = NULL; // pointer to allocated memory
    const uint16_t sensorDataLen = 17; // required space needed in flash

    if(!device.flash.createBuffer(&sensorData, sensorDataLen)) { lastErrorId = 24; errorCnt++; return; }
    else {
        if(sensorData == NULL) { lastErrorId = 25; errorCnt++; return; }
        float temp = gpsData->parent.dop_h * 10.0;
        if(temp > 255.) temp = 255.;
        uint8_t hdopTimesTen = temp;
        uint32_t tempTtf = gpsData->parent.ttfMilliseconds / 1000;
        uint8_t ttfSeconds;
        if(tempTtf > 255) { ttfSeconds = 255; }
        else { ttfSeconds = tempTtf; }
        temp = gpsData->parent.latitude * 1000000;
        int32_t latTimes1Mil = temp;
        temp = gpsData->parent.longitude * 1000000;
        int32_t lonTimes1Mil = temp;

        sensorData[0] = 0x12;
        sensorData[1] = 0x34;
        sensorData[2] = 0x58;
        HelperBits::addData4(timestamp, &sensorData[3]); // add timestamp, 4 bytes
        HelperBits::addData4Signed(latTimes1Mil, &sensorData[7]); // add lat
        HelperBits::addData4Signed(lonTimes1Mil, &sensorData[11]); // add lon
        sensorData[15] = hdopTimesTen; // add hdop
        sensorData[16] = ttfSeconds; // add ttf

        if(TRACKER_MODE == MODE_TESTRUN) { printf("Store: (%d bytes) ", sensorDataLen); printArray(sensorData, sensorDataLen); }
        push_data_result_t res = pushDataIntoMemory(sensorData, sensorDataLen, memoryFull, (TRACKER_MODE == MODE_TESTRUN)); // average 25ms
        if(res != PUSH_DATA_SUCCESS) { lastErrorId = 93 + res; errorCnt++; } // 14 error codes (0 = sucess), 94 - 107
        heap_caps_free(sensorData);
    }
}

void storeProximityDataInFlash(uint32_t timestamp, bool *memoryFull) {
    uint8_t *sensorData = NULL; // pointer to allocated memory
    const uint16_t headerDataLen = 9;
    const uint16_t proxDataLen = scanResultsPointer * 3;
	if(TRACKER_MODE == MODE_TESTRUN) { printf("Store: %d results -> %d + %d bytes\n", scanResultsPointer, headerDataLen, proxDataLen); }

    if(!device.flash.createBuffer(&sensorData, headerDataLen + proxDataLen)) { lastErrorId = 34; errorCnt++; return; }
    else {
        if(sensorData == NULL) { lastErrorId = 35; errorCnt++; return; }
        
        // serialize data for storage
        sensorData[0] = 0x12;
        sensorData[1] = 0x34;
        sensorData[2] = 0x57;
        HelperBits::addData4(timestamp, &sensorData[3]); // add timestamp, 4 bytes 
        HelperBits::addData2(proxDataLen, &sensorData[7]); // add prox data len
        uint16_t currentPointer = headerDataLen;    
	    
		// fill with proximity messages
	    for(uint16_t i=0; i<scanResultsPointer; i++) {
	        uint16_t rssiAvg = 0;
	        if(scanResults[i].receivedMessages != 0) { rssiAvg = scanResults[i].rssiSum / scanResults[i].receivedMessages; } // receivedMessages = 0 should not happen
	        sensorData[currentPointer] = scanResults[i].id >> 8;
            currentPointer++;
            sensorData[currentPointer] = scanResults[i].id & 0x00FF;
            currentPointer++;
	        sensorData[currentPointer] = rssiAvg;
            currentPointer++;
	    }
	    if(currentPointer != headerDataLen + proxDataLen) { heap_caps_free(sensorData); lastErrorId = 39; errorCnt++; return; } // needs to be the same        
        
        // store data in flash memory
        if(TRACKER_MODE == MODE_TESTRUN) {
            printArray(sensorData, headerDataLen + proxDataLen);
        }
        push_data_result_t res = pushDataIntoMemory(sensorData, headerDataLen + proxDataLen, memoryFull, (TRACKER_MODE == MODE_TESTRUN)); // average 25ms
        if(res != PUSH_DATA_SUCCESS) { lastErrorId = 79 + res; errorCnt++; } // 14 error codes (0 = sucess) -> 80 - 93
        heap_caps_free(sensorData); 
    }
}

void readFullFlash() {
    esp_task_wdt_init(120, false); // set task watchdog timeout to 120 seconds
    const uint32_t READ_PAGES = 16; // 42058
    if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
    device.delay(8000);
    if(!device.flash.printFlash(0, READ_PAGES, MT29_CACHE_SIZE, false)) { printf("ERROR FLASH2\n"); }
    if(!device.flashPowerOff(true)) { printf("ERROR FLASH3\n"); }
}

void testSend(uint8_t *mac) {
    const uint8_t DATALEN = 100;
    uint8_t data[DATALEN] = { 0 };
    for(uint8_t i=0; i<DATALEN; i++) { data[i] = i; }
    data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] = ESP_NOW_FLASH_STREAM_FIRST_BYTE;
    printf("NOW-TEST: send to %02X %02X %02X %02X %02X %02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    if(!device.sendESPNOWDataToMac(mac, &data[0], DATALEN)) { // 1ms, dataLen is normally 247 -> but send preamble as well, so +3 Byte
        printf("NOW-TEST: error, sendESPNOWData\n");
    }
    printf("NOW-TEST: wait on ACK\n");
    while(1) { // wait for ACK
        if(device.getESPNOWSendingStatus() == ESP_NOW_FINISHED_BUT_FAILED) { // 330ms (long range), 88ms in normal mode
            printf("NOW-TEST: no ACK received\n");
            break;
        }
        else if(device.getESPNOWSendingStatus() == ESP_NOW_FINISHED_AND_RECEIVED) { // 10-12ms (long range), 1-4ms in normal mode
            printf("NOW-TEST: ACK OKAY\n");
            break;
        }
    }
}
void sendBlobData(uint8_t *macAddress, uint16_t *blobSendPointer, uint32_t bytesToSendFromFlash, uint32_t sendPagePointerMock, uint16_t sendPageOffsetPointerMock) {
	const uint32_t millisWaitIfSendFailed = 500;
	const uint32_t maxSendRetries = 3;
    esp_err_t err;
    nvs_handle_t handle;
    size_t nvsDataLen = 0;  // value will default to 0, if not set yet in NVS
    uint8_t nvsData[NVS_FLASH_BUFFER_SIZE] = { 0 };
    bool receivedAck = false, haveSeenAnyAck = false;
    if(bytesToSendFromFlash == 0) { // only when all data from flash is already been sent
        // get current size of 512 byte NVS buffer
        if(nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle) != ESP_OK) { lastErrorId = 13; errorCnt++; return; }
        err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, NULL, &nvsDataLen); // 0ms
        if(err == ESP_ERR_NVS_NOT_FOUND) { // first time, blob is empty
            nvsDataLen = 0;
        }
        else if(err != ESP_OK) { lastErrorId = 16; errorCnt++; nvs_close(handle); return; }

        uint16_t bytesToSendFromBlob = nvsDataLen - (*blobSendPointer);
        if((*blobSendPointer) > nvsDataLen) { lastErrorId = 23; errorCnt++; nvs_close(handle); return; }
        if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTransBlob: %d in blob, %d sent -> %d bytes to send from blob\n", nvsDataLen, *blobSendPointer, bytesToSendFromBlob); }

        if((bytesToSendFromBlob > 0) && (nvsDataLen > 0)) { // only if not already sent out everything from the blob
            // now get the real blob
            if(nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, &nvsDataLen) != ESP_OK) { lastErrorId = 26; errorCnt++; nvs_close(handle); return; }
            uint8_t data[250] = { 0 };
            uint8_t dataMsgLen = 7;
            uint16_t blobSendPointerTemp = (*blobSendPointer);
            sendPageOffsetPointerMock += (*blobSendPointer); // start from last send pointer

			// put blob data in esp now messages
            while(bytesToSendFromBlob > 0) {
				data[0] = ESP_NOW_FLASH_STREAM_FIRST_BYTE;
				data[1] = sendPagePointerMock >> 9; // upper 8 bits of 17 bits
				data[2] = (sendPagePointerMock >> 1) & 0xFF; // middle 8 bits of 17 bits
				data[3] = ((sendPagePointerMock & 0b1) << 7) | ((sendPageOffsetPointerMock >> 4) & 0b1111111); // 1 bit of sendPagePointer, upper 7 bits of sendPageOffsetPointerTemp = i
				data[4] = ((sendPageOffsetPointerMock & 0b1111) << 4) | 0b1111;
				data[5] = ownTagId >> 8;
                data[6] = ownTagId & 0x00FF;
                dataMsgLen = 7;

                while(dataMsgLen < 250) {
                    data[dataMsgLen] = nvsData[blobSendPointerTemp];
                    dataMsgLen++;
                    blobSendPointerTemp++;
                    if(blobSendPointerTemp >= nvsDataLen) { break; } // reached end of write pointer
                }

                if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTransBlob: 1 msg: %d + %d bytes with mockPointer %d.%d\n", 7, dataMsgLen - 7, sendPagePointerMock, sendPageOffsetPointerMock); }
				// send
				uint16_t sendRetryCounter = 0;
				while(true) {
					sendRetryCounter++;
					if(!device.sendESPNOWDataToMac(macAddress, &data[0], dataMsgLen)) { lastErrorId = 27; errorCnt++; return; }
					receivedAck = false;
					uint32_t startSendingTime = ((uint32_t) Timing::millis());
					while(true) { // wait for ACK
						esp_now_sending_status_t statusSend = device.getESPNOWSendingStatus();
						if(statusSend == ESP_NOW_FINISHED_BUT_FAILED) { // 330ms (long range), 88ms in normal mode
							if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTransBlob: no ack received\n"); }
							break;
						}
						else if(statusSend == ESP_NOW_FINISHED_AND_RECEIVED) { // 10-12ms (long range), 1-4ms in normal mode
							receivedAck = true;
							if(!haveSeenAnyAck) { haveSeenAnyAck = true; } // to see if gateway was in reach at all
							break;
						}
						if(((uint32_t) Timing::millis()) - startSendingTime > 500) {
							if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTransBlob: fatal ack timeout\n"); }
							nvs_close(handle);
                            lastErrorId = 28; errorCnt++; 
							return;
						}
					}
					if(receivedAck) { // successfully transmitted message
						// update mock send pointer
						sendPageOffsetPointerMock += (dataMsgLen - 7);
						if(sendPageOffsetPointerMock >= MT29_CACHE_SIZE) {
		                    sendPageOffsetPointerMock = sendPageOffsetPointerMock % MT29_CACHE_SIZE;
		                    sendPagePointerMock++;
						}
		                // this should never happen with 512 bytes chunks 
						if(sendPagePointerMock >= MT29_NUMBER_PAGES) { sendPagePointerMock = 0; lastErrorId = 29; errorCnt++; }
		
						// update real NVS pointer now
		                *blobSendPointer = blobSendPointerTemp;
		                bytesToSendFromBlob = nvsDataLen - (*blobSendPointer);
						break;
					} 
					else {
						if(haveSeenAnyAck) { // got at least one ack before
							if(sendRetryCounter >= maxSendRetries) { // after 3 tries failed
								if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTransBlob: %d.%d retry %d -> STOP\n", sendPagePointerMock, sendPageOffsetPointerMock, sendRetryCounter); }
								nvs_close(handle); 	
								return;
							}
							else { // had some successful transmission before - retry!
								Timing::delay(millisWaitIfSendFailed); // wait a tiny bit
								if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTransBlob: %d.%d retry %d\n", sendPagePointerMock, sendPageOffsetPointerMock, sendRetryCounter); }	
							}
						} 
						else { nvs_close(handle); return; } // no ack at all -> gateway not there, immediately stop
					}
				}
            }
        }
        nvs_close(handle); 
    } 
}

void dataTransmissionToGateway() {
    bool anyGatewaySeen = false;
    uint8_t *gatewayMac = NULL;
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        if(scanResults[i].isGateway == true) { 
            anyGatewaySeen = true;
            gatewayMac = scanResults[i].mac;            
            if(!device.addESPNOWReceiverStationary(gatewayMac)) { lastErrorId = 43; errorCnt++; return; }
            break;
        }
    }
    if(anyGatewaySeen) {
        if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTrans: gateway seen\n"); }
        uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_PAGE_POINTER);
        uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
        uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
        uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
        uint16_t nvsSendBlobPointer = device.nvsReadUINT16(NVS_FLASH_SEND_BLOB_POINTER);

        uint32_t sendBytePointer = (sendPagePointer * MT29_CACHE_SIZE) + sendPageOffsetPointer;
	    uint32_t writeBytePointer = (flashPageWritePointer * MT29_CACHE_SIZE) + flashPageWriteOffsetPointer;
	    uint32_t bytesToSendFromFlash = device.flash.fifoGetNumberOfPopableBytes(sendBytePointer, writeBytePointer);

        if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTrans: flashPageWritePointer: %d, flashPageWriteOffsetPointer: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer); }
        if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTrans: before: bytesToSendFromFlash: %d, sendPagePointer: %d, sendPageOffsetPointer: %d, nvsSendBlobPointer: %d\n", bytesToSendFromFlash, sendPagePointer, sendPageOffsetPointer, nvsSendBlobPointer); }
    
        if(gatewayMac != NULL) {
            //testSend(gatewayMac);
            uint8_t customPrefix[2];
            customPrefix[0] = ownTagId >> 8;
            customPrefix[1] = ownTagId & 0x00FF;
            uint32_t sendPagePointerOld = sendPagePointer;
            uint16_t sendPageOffsetPointerOld = sendPageOffsetPointer;
            uint16_t nvsSendBlobPointerOld = nvsSendBlobPointer;

            // stream the data
            esp_now_stream_status_t espNowStatus = device.doESPNOWFlashStreamNew(
                gatewayMac,
                customPrefix, 2,
                &sendPagePointer, &sendPageOffsetPointer, 
                flashPageWritePointer, flashPageWriteOffsetPointer, 
                ESP_NOW_FLASH_STREAM_NO_LIMIT,
                500, 8, // millis to wait when one message failed (but acks happened before), number of retries
                ESPNOW_MIN_BATT_VOLTAGE_DURING_TRANSM,
                (TRACKER_MODE == MODE_TESTRUN) ? 1 : 0, MOCK_FLASH_READ, false);
            // BLOCKS WILL NEVER BE DELETED AT THE MOMENT
            if(espNowStatus != ESP_NOW_STREAM_DATA_FINISHED) {
                if((espNowStatus != ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR) && (espNowStatus != ESP_NOW_STREAM_NO_DATA_TO_SEND)) { // no data to send happens when function is called with minBlocksToTransmit = 0 (forcing a gateway scan) 
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTrans: STREAM ERROR, status: %d!\n", espNowStatus); }
                    lastErrorId = 108 + espNowStatus;  errorCnt++; // max. 108 + 12 -> add some buffer = 130
                }
                else { // normal case, no gateway found
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTrans: nothing (neverAck %d, noData %d)!\n", (espNowStatus == ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR), (espNowStatus == ESP_NOW_STREAM_NO_DATA_TO_SEND)); }
                }
            }

            // update bytes to send and see if blob data needs to be transmitted
            sendBytePointer = (sendPagePointer * MT29_CACHE_SIZE) + sendPageOffsetPointer; // update
            writeBytePointer = (flashPageWritePointer * MT29_CACHE_SIZE) + flashPageWriteOffsetPointer; // update
            bytesToSendFromFlash = device.flash.fifoGetNumberOfPopableBytes(sendBytePointer, writeBytePointer);
            sendBlobData(gatewayMac, &nvsSendBlobPointer, bytesToSendFromFlash, sendPagePointer, sendPageOffsetPointer);

            // update pointer
            if(sendPagePointer != sendPagePointerOld) {
                if(!device.nvsWriteUINT32(NVS_FLASH_SEND_PAGE_POINTER, sendPagePointer)) { lastErrorId = 44; errorCnt++; }
            }
            if(sendPageOffsetPointer != sendPageOffsetPointerOld) {
                if(!device.nvsWriteUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER, sendPageOffsetPointer)) { lastErrorId = 45; errorCnt++; }
            }
            if(nvsSendBlobPointer != nvsSendBlobPointerOld) {
                if(!device.nvsWriteUINT16(NVS_FLASH_SEND_BLOB_POINTER, nvsSendBlobPointer)) { lastErrorId = 11; errorCnt++; }
            }
            if(TRACKER_MODE == MODE_TESTRUN) { printf("dataTrans: after: bytesToSendFromFlash: %d, sendPagePointer: %d, sendPageOffsetPointer: %d, nvsSendBlobPointer: %d\n", bytesToSendFromFlash, sendPagePointer, sendPageOffsetPointer, nvsSendBlobPointer); }
        }
    }
}

void gpsCheckAliveAndMaybeChangeBaudrate(bool debug) {
    device.shortLightSleep(4000);
    device.gpioBOn();   
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');
    if(gps.isStarted()) {
        if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: GPS working on 115200\n"); }
    }
    else {
        device.gpioBOff();
        if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: GPS not answering on 115200 -> TRYING 9600\n"); }
        device.shortLightSleep(1000);
        device.uart2UpdateBaudrate(9600);
        device.gpioBOn(); // turn on again
        if(gps.isStarted()) {
            if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: GPS answering on 9600 -> CHANGE\n"); }
            gps.permanentlySetBaudrate115200(); // waiting 2 seconds here
            device.gpioBOff();
            device.delay(2000);
            device.uart2UpdateBaudrate(115200);
            device.gpioBOn(); // turn on again
            if(gps.isStarted()) {
                if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: change success, talking on 115200 now\n"); }
            }
            else {
                if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: FATAL, change did not work\n"); }
            }
        }
        else {
            if(debug) { printf("gpsCheckAliveAndMaybeChangeBaudrate: NOT ANSWERING ON 9600 OR 115200 -> NOT CONNECTED?\n"); }
        }
    }
    device.gpioBOff();
}

bool hourEqualsGPSTrySetting(uint8_t hourIn) {
    for(uint8_t i = 0; i < sizeof(gpsFixHours); i++) {
        if(gpsFixHours[i] == hourIn) {
            return true;
        }
    }
    return false;
}

extern "C" void app_main() {
    while(1) {
        if((TRACKER_MODE == MODE_TESTRUN) || (TRACKER_MODE == MODE_PRODUCTIVE)) {
            /** ---------------- FIRST START HARD RESET STATE ---------------- */
            if(state == ST_FIRST_START_HARD_RESET) {
                if(!device.fullRFCalibration()) { lastErrorId = 68; errorCnt++; } // 145 - 160ms

                if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: HARD RESET!\n"); }
                device.disableWakeStubNoBootIfVoltageLow(); // disable no boot if voltage low (do here, only needed once!)
                if(USE_LEDS) { device.blinkTimes(3, B_RED); }
                
                i2c.begin(I2C_FREQ_HZ_400KHZ);

                // disable previous interrupts
                if(!device.rtc.disableTimeUpdateInterruptMinuteChange()) { lastErrorId = 38; errorCnt++; }

                // GPS: check if working
                gpsCheckAliveAndMaybeChangeBaudrate(TRACKER_MODE == MODE_TESTRUN);

                // RTC: get current timestamp
                bool error = false;
                uint32_t timestamp = device.rtc.getTimestamp(error);
                if(error) { lastErrorId = 2; errorCnt++; }

                // check NVS if already activated & timestamp is valid
                if(!device.initDataNVS()) { lastErrorId = 3; errorCnt++; }
                uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
                uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_PAGE_POINTER);
                uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
                uint16_t nvsSendBlobPointer = device.nvsReadUINT16(NVS_FLASH_SEND_BLOB_POINTER);
                printf("Reset: FIRMWARE: V%d, CONFIG: %s V%d\n", WILDFI_SOFTWARE_VERSION, WILDFI_CONFIG_NAME, WILDFI_CONFIG_VERSION);
                printf("Reset: flashPageWritePointer: %d, flashPageWriteOffsetPointer: %d, sendPagePointer: %d, sendPageOffsetPointer: %d, nvsSendBlobPointer: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer, sendPagePointer, sendPageOffsetPointer, nvsSendBlobPointer);
            

                // read tag id from nvs
                if(TAG_ID_SOURCE == TAG_ID_USE_MAC_LAST_TWO_BYTES) {
                    uint8_t myMac[6] = { 0 };
                    esp_efuse_mac_get_default(myMac);
                    ownTagId = (myMac[4] << 8 ) | myMac[5];
                }
                else if(TAG_ID_SOURCE == TAG_ID_USE_VALUE_IN_NVS) { ownTagId = device.nvsReadUINT16(NVS_OWN_ID); }
                else { ownTagId = 0xFFFF; }

                if(ownTagId == 0) {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: WARNING tag id = 0, using 0xFFFF\n"); }
                    ownTagId = 0xFFFF;
                    lastErrorId = 12; errorCnt++;
                }
                printf("Reset: TAG ID (src %d): 0x%04X\n", TAG_ID_SOURCE, ownTagId);

                // check for brownouts
                if(device.getLastResetReason() == ESP_RST_BROWNOUT) { lastErrorId = 4; errorCnt++; }

                // activation state
                if(ACTIVATION_MODE == ACTIVATION_MODE_SKIP) { isActivated = true; } // store state in RTC -> always activated
                else if(ACTIVATION_MODE == ACTIVATION_MODE_STORE_PERMANENTLY) {
                    uint16_t activated = device.nvsReadUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI);
                    isActivated = (activated > 0); // store state in RTC
                }
                else if(ACTIVATION_MODE == ACTIVATION_MODE_ON_EVERY_START) { isActivated = false; } // store state in RTC -> not activated after reset

                // time state
                bool timeIsValid = device.rtc.timeIsValidNoUndervoltage(error);
                if(error) { lastErrorId = 7; errorCnt++; }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp %d, timeValid %d\n", timestamp, timeIsValid); }
                if((timestamp > 1600000000) && timeIsValid) { hasValidTimestamp = true; }
                else { hasValidTimestamp = false; }

                if(isActivated) {
                    if(hasValidTimestamp) {
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp valid and activated -> START!\n"); }
                        state = ST_START;
                    }
                    else {
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp invalid but activated -> GET TIME!\n"); }
                        state = ST_GET_TIME;
                    }
                }
                else { // is NOT activated - time irrelevant, go to activation state
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: NOT activated (mode %d) -> ACTIVATION!\n", ACTIVATION_MODE); }
                    state = ST_WAIT_FOR_ACTIVATION;
                }
                device.enableInternalTimerInterruptInDeepSleep(SLEEP_TIME_AFTER_START); // restart system into next state
            }
            /** ---------------- WAIT FOR ACTIVATION STATE ---------------- */
            else if(state == ST_WAIT_FOR_ACTIVATION) { // custom wake stub not running -> no fifo
                if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, stay in that mode but sleep for an hour or more -> needs to be here because otherwise setHourlyInterrupt might not have been set!
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: Undervoltage (%d)! Don't scan, sleep on RTC interrupt!\n", device.readSupplyVoltageFromWakeStub()); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for an hour or more
                }
                else {
                    if(!device.onlyLoadRFCalibration()) { lastErrorId = 70; errorCnt++; } // 5ms, using RF data in RTC memory
                    bool activationMessageSeen = activationDetection();
                    device.stopESPNOW(); // 5ms
                    esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                    if(activationMessageSeen) {
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: DONE\n"); }
                        if(ACTIVATION_MODE == ACTIVATION_MODE_ON_EVERY_START) { isActivated = true; } // just store in RTC variable, do not store in NVS
                        else {
                            if(!device.initDataNVS()) { lastErrorId = 64; errorCnt++; }
                            else {
                                if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 1)) { lastErrorId = 65; errorCnt++; } // write activation into NVS
                                else { isActivated = true; } // now is activated!
                            }
                        }
                        if(isActivated) {
                            if(hasValidTimestamp) {
                                state = ST_START;
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: all done, time still valid -> move to START in 1s!\n"); }
                            }
                            else {
                                state = ST_GET_TIME;
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: all done but no time -> GET TIME\n"); }
                            }
                            device.enableInternalTimerInterruptInDeepSleep(1); // restart in 1 second and start
                        }
                        else {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: gateway was seen, but other error -> SLEEP\n"); }
                            device.enableInternalTimerInterruptInDeepSleep(ACTIVATION_BY_GATEWAY_INTERVAL_SECONDS);
                        }
                    }
                    else {
                        device.enableInternalTimerInterruptInDeepSleep(ACTIVATION_BY_GATEWAY_INTERVAL_SECONDS);
                    }
                }
            }
            /** ---------------- GET TIME STATE ---------------- */
            else if(state == ST_GET_TIME) { // custom wake stub not running
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: try getting time! V = %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, go to power down state
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: undervoltage (%d)! Don't scan!\n", device.readSupplyVoltageFromWakeStub()); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                }
                else {
                    i2c.begin(I2C_FREQ_HZ_400KHZ);
                    if(GET_FIRST_TIME_OVER_WIFI) { // first check over wifi
                        if(getTimeOverWiFiNew()) {
                            hasValidTimestamp = true;
                        }
                    }
                    if((!hasValidTimestamp ) && GET_FIRST_TIME_OVER_GPS) { // if didn't get time over wifi -> try gps
                        device.gpioBOn();
                        device.uart2Init(115200);
                        gps.init(device.uart2GetQueue());
                        device.uart2EnablePatternInterrupt('\n');
                        esp_gps_t gpsData = { };
                        if(gps.getTimeOnly(&gpsData, GPS_TIMEOUT_SECONDS, &device, TIME_OVER_GPS_BLINK_LED, (TRACKER_MODE == MODE_TESTRUN))) {
                            hasValidTimestamp = true;
                        }
                        device.gpioBOff();
                    }
                    // evaluate result
                    if(hasValidTimestamp) {
                        state = ST_START;
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: all done and already activated -> MOVE TO START in %ds!\n", TIME_SLEEP_AFTER_GOT_TIME); }
                        if(USE_LEDS) { device.blinkTimes(6, B_GREEN); }
                        device.enableInternalTimerInterruptInDeepSleep(TIME_SLEEP_AFTER_GOT_TIME); // restart in x seconds and move to activation or start
                    }
                    else {
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: failed -> sleep %ds!\n", TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); }
                        device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS);
                    }
                }
            }
            /** ---------------- START STATE ---------------- */
            else if(state == ST_START) {
                i2c.begin(I2C_FREQ_HZ_400KHZ);
                if(device.rtc.setTimeUpdateInterruptMinuteChange()) {
                    bool error = false;
                    currentMinute = device.rtc.getMinutes(error);; // remember current minute
                    currentHour = device.rtc.getHours(error);; // remember current hour
                    if(!error) {
                        if(USE_LEDS) { device.blinkTimes(5, B_BOTH); }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: %d:%02d\n", currentHour, currentMinute); }
                        state = ST_TRACK;
                        lastSync = 1; // just synchronized!

                        if(USE_IMU) {
                            device.keepSensorPowerOnInDeepSleep();
                            startIMU();
                        }

                        device.customWakeStubFunction(wakeStub);
                        device.setWakeStubRejectionInterruptSrc(USE_EXT0_IF_WAKE_UP_REJECTED);
                        device.enableRTCInterruptInDeepSleep();
                    }
                    else { lastErrorId = 5; errorCnt++; device.enableInternalTimerInterruptInDeepSleep(120); }
                }
                else { lastErrorId = 6; errorCnt++; device.enableInternalTimerInterruptInDeepSleep(120); }
            }
            /** ---------------- TRACKING STATE ---------------- */
            else if(state == ST_TRACK) {
                if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("State: POWER DOWN for %ds\n", FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
                    state = ST_PWRDWN;
                    if(USE_IMU) {
                        i2c.begin(I2C_FREQ_HZ_400KHZ);
                        stopIMU();
                    }
                    device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                    // WARNING: RTC interrupts still running! not disabled, but shouldn't be a problem
                }
                else {
                    // ---------------- PROXIMITY DETECTION ----------------

                    if(!device.onlyLoadRFCalibration()) { lastErrorId = 69; errorCnt++; } // 5ms, using RF data in RTC memory
                    if(USE_LEDS) { device.ledGreenOn(); } // will be turned off in proximity detection
                    if(!device.flashPowerOn(false)) { lastErrorId = 11; errorCnt++; } // no delay needed because proximity detection first

                    scanStop = false;
                    bool proximitySuccess = proximityDetection(); // perform proximity detection
                    scanStop = true; // do not store more incoming messages in scanResults (ESP NOW still running)
                    bool gatewaySeen = isGatewayInScanResults(); // check if any gateway is in the scan results
                    proxDetectionsSinceLastFullRFCalib++;

                    if(!proximitySuccess) { // some fatal error
                        device.stopESPNOW(); // 5ms
                        esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep                           
                    }
                    else {
                        if(!gatewaySeen) { // no gateway seen or severe proximity error happened - stop ESP NOW and proceed with storing the data
                            device.stopESPNOW(); // 5ms
                            esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                        }
                        // store data (also when gateway seen, first store it and afterwards do transmission)
                        i2c.begin(I2C_FREQ_HZ_400KHZ); // needed for current minute update
                        //checkIfTimeCorrectionNeeded();
                        checkForHugeTimeDifferences();
                        if(!device.initDataNVS()) { lastErrorId = 8; errorCnt++; }
                        if(TRACKER_MODE == MODE_TESTRUN) { printScanResult(); }
                        bool error = false, memoryFull = false;
                        uint32_t timestamp = device.rtc.getTimestamp(error);
                        if(error) { timestamp = 0xFFFFFFFF; lastErrorId = 9; errorCnt++; }
                        storeProximityDataInFlash(timestamp, &memoryFull); // needs flash to be turned on
                        //device.flash.printFlash(0, 1, 512); // TEMPORARY
                        if(memoryFull) { // memory full = still continue operation, ignore that data is not stored
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: memory full!\n"); }
                        }
                        // transmit data
                        if(gatewaySeen) { // NVS initialized
                            dataTransmissionToGateway(); // needs flash, nvs and RF
                            device.stopESPNOW(); // 5ms
                            esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                        }
                    }

                    // ---------------- IMU DATA ----------------

                    if(USE_IMU) {
                        uint8_t fifoData[1024];
                        uint16_t fifoLen = 0;
                        if(!device.imu.readGeneralFIFOInOneGoFast(true, false, false, fifoData, fifoLen, false)) { lastErrorId = 42; errorCnt++; }
                        else {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("IMU: %d byte in fifo!\n", fifoLen); }
                        }
                        // TODO: create statistics from fifo
                        // TODO: store statistics
                    }
                    if(!device.flashPowerOff(false)) { lastErrorId = 10; errorCnt++; } // IMPORTANT: turn off here

                    // ---------------- TIME UPDATE / CORRECTION RTC VARIABLES ----------------

                    bool error = false;
                    uint8_t currentMinuteOld = currentMinute;
                    currentMinute = device.rtc.getMinutes(error);; // remember current minute, just for safety, already incremented in wake stub, just in case
                    currentHour = device.rtc.getHours(error);; // remember current hour, just for safety, already incremented in wake stub, just in case
                    if(error) { lastErrorId = 17; errorCnt++; }
                    if(currentMinuteOld != currentMinute) { // can happen if GPS takes longer than 1 minute
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("State: WARNING, minutes unequal: new %d vs. old %d\n", currentMinute, currentMinuteOld); }
                    }

                    // ---------------- GPS ATTEMPT ----------------

                    if(((currentMinute == 0) || (GPS_FORCED_AFTER_EVERY_PROXIMITY)) // only at full hours or when forced
                        && hourEqualsGPSTrySetting(currentHour)) { // only when hour is in list
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("GPS: start %d:%d (force: %d)\n", currentHour, currentMinute, GPS_FORCED_AFTER_EVERY_PROXIMITY); }
                        device.gpioBOn();
                        device.uart2Init(115200);
                        gps.init(device.uart2GetQueue());
                        device.uart2EnablePatternInterrupt('\n');
                        esp_gps_t gpsData = { };
                        gps_get_fix_config_t gpsConfig = {
                            .timeoutSeconds = GPS_TIMEOUT_SECONDS,
                            .minHDOP = GPS_MIN_HDOP,
                            .afterFixMaxWaitOnHDOP = GPS_AFTER_FIX_MAX_WAIT_ON_HDOP,
                            .setRTCTime = true,
                            .blinkLeds = USE_LEDS,
                            .debug = (TRACKER_MODE == MODE_TESTRUN) };
                        get_fix_result_t fixResult = gps.tryToGetFix(&gpsData, &gpsConfig, &device);
                        if(fixResult != GPS_FIX_SUCCESS_AND_RTC_UPDATED) {
                            device.gpioBOff();
                            lastErrorId = 72 + fixResult; errorCnt++; // buffer for next error id: 80
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("GPS: failed %d!\n", fixResult); }
                        }
                        else {
                            device.gpioBOff();
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("GPS: SUCCESS: Timestamp: %u, TTFF: %d, LAT: %f, LON: %f, HDOP: %f\n", gpsData.parent.utcTimestamp, gpsData.parent.ttfMilliseconds, gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.dop_h); }
                            lastSync = 1; // just synced the time
                            if(!device.flashPowerOn(false)) { lastErrorId = 11; errorCnt++; } // flash power on (again) for storing
                            bool error = false, memoryFull = false;
                            uint32_t timestamp = device.rtc.getTimestamp(error); // get current timestamp to store on flash
                            if(error) { timestamp = 0xFFFFFFFF; lastErrorId = 15; errorCnt++; }
                            storeGPSDataInFlash(timestamp, &gpsData, &memoryFull); 
                            //device.flash.printFlash(0, 1, 512); // TEMPORARY
                            if(!device.flashPowerOff(false)) { lastErrorId = 10; errorCnt++; }
                            if(memoryFull) { // memory full = still continue operation, ignore that data is not stored
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: memory full!\n"); }
                            }
                        }

                        // update time again
                        uint8_t currentMinuteOld = currentMinute;
                        currentMinute = device.rtc.getMinutes(error);; // remember current minute, just for safety, already incremented in wake stub, just in case
                        currentHour = device.rtc.getHours(error);; // remember current hour, just for safety, already incremented in wake stub, just in case
                        if(error) { lastErrorId = 14; errorCnt++; }
                        if(currentMinuteOld != currentMinute) { // can happen if GPS takes longer than 1 minute
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("State: WARNING, after GPS minutes unequal: new %d vs. old %d\n", currentMinute, currentMinuteOld); }
                        }
                    }

                    // ---------------- DEBUG OUTPUT ----------------

                    if((lastSeenSomeone > (60*24)) && (lastErrorId != 40)) { lastErrorId = 40; errorCnt++; } // haven't seen anyone for 24 hours -> add error ONCE
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("State: %d:%02d (lastSync: %dmin, lastSeenSomeone: %dmin)\n", currentHour, currentMinute, lastSync, lastSeenSomeone); }
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("State: lastErrorId: %d, errorCnt: %d\n\n", lastErrorId, errorCnt); }

                    // ----------------CHECK IF FULL CALIB NEEDED ----------------

                    if(proxDetectionsSinceLastFullRFCalib >= PROXIMITY_FULL_RF_CALIB_EVERY) {
                        proxDetectionsSinceLastFullRFCalib = 0;
                        state = ST_RF_FULL_CALIB;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else {
                        device.enableRTCInterruptInDeepSleep();
                    }
                }
            }
            /** ---------------- POWER DOWN STATE ---------------- */
            else if(state == ST_PWRDWN) { // WARNING: RTC interrupts still running! not disabled
                if(TRACKER_MODE == MODE_TESTRUN) { printf("(PWR_DWN) V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() <= BATT_RESTART_VOLTAGE) { // voltage still too low for restarting
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time 
                }
                else { // voltage okay again, restart into tracking state
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("RESTART!\n"); }
                    state = ST_TRACK;
                    i2c.begin(I2C_FREQ_HZ_400KHZ); // needed for current minute update
                    bool error = false;
                    // update time here because might have slept for long time! remember that RTC interrupts are still activated (but ESP32 does not react on it)
                    uint8_t currentMinuteOld = currentMinute;
                    currentMinute = device.rtc.getMinutes(error);; // remember current minute, just for safety, already incremented in wake stub, just in case
                    currentHour = device.rtc.getHours(error);; // remember current hour, just for safety, already incremented in wake stub, just in case
                    if(error) { lastErrorId = 67; errorCnt++; }
                    if(currentMinuteOld != currentMinute) { // can happen if has been sleeping long time due to undervoltage
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("State: WARNING, minutes unequal: new %d vs. old %d\n", currentMinute, currentMinuteOld); }
                    }
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("State: %d:%02d (lastSync: %dmin)\n", currentHour, currentMinute, lastSync); }
                    device.enableRTCInterruptInDeepSleep();
                }
            }
            /** ---------------- FULL RF CALIBRATION STATE ---------------- */
            else if(state == ST_RF_FULL_CALIB) { // WARNING: RTC interrupts still running! not disabled
                if(!device.fullRFCalibration()) { lastErrorId = 71; errorCnt++; }  // 145 - 160ms  
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Full calib state: done\n"); }
                state = ST_TRACK; // back to track
                device.enableRTCInterruptInDeepSleep();
            }
        }
        else { // not in productive or test run
            if(TRACKER_MODE == MODE_SELFTEST) { printf("Mode: SELFTEST\n");  device.selfTest(SELFTEST_VOLTAGE_REF, SELFTEST_PARAMETERS); }
            else if(TRACKER_MODE == MODE_ONE_TIME_PROGRAMMING) {
                printf("Mode: ONE TIME PROGRAMMING\n"); 
                if(device.selfTest(SELFTEST_VOLTAGE_REF, SELFTEST_PARAMETERS)) {
                    printf("Writing ID %04X into NVS\n", OWN_TAG_ID);
                    if(device.nvsWriteUINT16(NVS_OWN_ID, OWN_TAG_ID)) { printf("DONE!\n"); }
                    else { printf("ERROR!\n"); }
                }
            }
            //else if(TRACKER_MODE == MODE_READFLASH) { printf("Mode: READ FLASH\n"); readFullFlash(); }
            //else if(TRACKER_MODE == MODE_MOCK_FLASH_STATE) { printf("Mode: MOCK FLASH STATE\n"); mockFlashState(); }
        }
        startCnt++;
        device.deepSleep();
    }
}
