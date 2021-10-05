#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

uint32_t t = 0;

// TODO: "persistent connection" when transmitting multiple requests https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_client.html

// after connection to wifi: around 2.5s for one iteration and 16*2048 = 32768 byte, average = 71mA -> 13.107 bytes per second
// calculated comparison to ESP NOW: 11ms for 250 byte (let's say 240 byte without meta info) = 21.818 byte per second
// with stream: 1MByte (base64 encoded, so less real data) in 53 seconds, average 91.3mA, 4.98mWh @ 3.75V = 1.328mAh

post_task_parameters_t restParams;
post_task_stream_flash_parameters_t restStreamParams;
const uint32_t DATA_LEN = 16*2048; // >= 28*2048 does not work, 24*2048 WORKS!!!
const uint16_t TASK_STACK_SIZE = 8192;
const uint16_t ITERATIONS = 3;
uint8_t *payload;
uint8_t *dmaStreamBuffer;

#define WIFI_SCAN_CHANNEL                       6
#define TAG_NAME                                "StreamTest" 
#define REST_PAYLOAD_PREFIX_1                   "[{\"tag\":\""
#define REST_PAYLOAD_PREFIX_2                   "\",\"data\":\""

const char* REST_URL = RESTDB_ENDPOINT2;
const char* REST_CONTENT_TYPE = "application/json";
const char* REST_ADD_HEADER_KEY = "x-apikey";
const char* REST_ADD_HEADER_VALUE = RESTDB_APIKEY1;
const char* REST_PAYLOAD_PREFIX = REST_PAYLOAD_PREFIX_1 TAG_NAME REST_PAYLOAD_PREFIX_2;
const char* REST_PAYLOAD_POSTFIX = "\"}]";

bool streamFinished = false;
uint8_t *dmaBuffer2048Bytes = NULL;

const char* NVS_PARTITION = "nvs2";
RTC_DATA_ATTR bool firstStart = true;

void scan() {
    if(!device.initWiFi()) { printf("ERROR1\n"); }
    device.measureTime("initWiFi");
    if(!device.scanForWiFis(true, RADIO_MAX_TX_19_5_DBM, 120, 1)) { printf("ERROR\n"); }
    device.measureTime("scanForWiFis");
    device.printWiFiScanResults();
    device.measureTime("printWiFiScanResults");
    device.disconnectAndStopWiFi();
    device.measureTime("disconnectAndStopWiFi");
}

void post() {
    if(!device.initWiFi()) { printf("ERROR1\n"); }
    device.measureTime("initWiFi");
    if(!device.connectToWiFiDirectly("Korntal", "xxxxxxx", RADIO_MAX_TX_19_5_DBM, 1)) { printf("ERROR2\n"); }
    //if(!device.connectToWiFiDirectly("Tracker1", "Tracker1", RADIO_MAX_TX_19_5_DBM, 1)) { printf("ERROR2\n"); }
    device.measureTime("connectToWiFiDirectly");
    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) { ; }
    printf("ConnectedToWiFi = %d\n", device.connectedToWiFi());
    device.measureTime("while(!device.connectedToWiFi())");
    if(device.connectedToWiFi() == WIFI_CONNECT_SUCCESS) {
        payload = (uint8_t *) malloc(DATA_LEN);
        if(payload == NULL) {
            printf("ERROR MALLOC\n");
        }
        else {
            for(uint32_t i = 0; i < DATA_LEN; i++) {
                payload[i] = 100 - (uint8_t) i;
            }
            for(uint16_t i=0; i<ITERATIONS; i++) {
                printf("--- SEND ITERATION %d\n", i);
                restParams.url = REST_URL;
                restParams.contentType = REST_CONTENT_TYPE;
                restParams.additionalHeaderKey = REST_ADD_HEADER_KEY;
                restParams.additionalHeaderValue = REST_ADD_HEADER_VALUE;
                restParams.prefix = REST_PAYLOAD_PREFIX;
                restParams.data = &payload[0];
                restParams.dataLen = DATA_LEN;
                restParams.postfix = REST_PAYLOAD_POSTFIX;
                device.doWiFiPOSTCall(&restParams, TASK_STACK_SIZE);
                while(device.getWiFiPOSTCallStatus() == HTTP_POST_DATA_RUNNING) {
                    device.delay(10);
                }
                device.measureTime("while(!httpStuffFinished)");
                printf("POST STATUS: %d\n", device.getWiFiPOSTCallStatus());
                printf("RETURN CODE: %d\n", device.getWiFiPostReturnCode());
            }
        }
        device.disconnectAndStopWiFi();
        device.measureTime("disconnectAndStopWiFi");
        for(uint16_t i=0; i<200; i++) {
            printf("BATT VOLTAGE AFTER %dms: %d\n", i*20, device.readSupplyVoltage());
            device.delay(20);
        }
    }
}

void flashTest() {
    if(!device.flash.createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
		printf("TEST: FLASH DMA RESERVE ERROR!\n");
    }

    device.flash.erase(0);

    uint32_t status = 0;

    for(uint32_t j=0; j<64; j++) {
        for(uint32_t i=0; i<2048; i++) { dmaBuffer2048Bytes[i] = j+1; }
        device.flash.partialWrite(j, 0, dmaBuffer2048Bytes, 2048);
    }

    /*for(uint32_t i=0; i<2048; i++) { dmaBuffer2048Bytes[i] = 0xCC; }
    device.flash.partialWrite(0, 0, dmaBuffer2048Bytes, 100);
    status = device.flash.getFeatures(GET_FEATURES_STATUS);
    printf("Status 0: %d\n", status);

    for(uint32_t i=0; i<2048; i++) { dmaBuffer2048Bytes[i] = 0xDD; }
    device.flash.partialWrite(0, 100, dmaBuffer2048Bytes, 2048-100);
    status = device.flash.getFeatures(GET_FEATURES_STATUS);
    printf("Status 1: %d\n", status);*/

    /*for(uint32_t i=0; i<2048; i++) { dmaBuffer2048Bytes[i] = 0xAA; }
    device.flash.partialWrite(0, 0, dmaBuffer2048Bytes, 512);
    status = device.flash.getFeatures(GET_FEATURES_STATUS);
    printf("Status 0: %d\n", status);

    for(uint32_t i=0; i<2048; i++) { dmaBuffer2048Bytes[i] = 0xBB; }
    device.flash.partialWrite(0, 512, dmaBuffer2048Bytes, 512);
    status = device.flash.getFeatures(GET_FEATURES_STATUS);
    printf("Status 1: %d\n", status);

    for(uint32_t i=0; i<2048; i++) { dmaBuffer2048Bytes[i] = 0xCC; }
    device.flash.partialWrite(0, 512, dmaBuffer2048Bytes, 512);
    status = device.flash.getFeatures(GET_FEATURES_STATUS);
    printf("Status 2: %d\n", status);

    for(uint32_t i=0; i<2048; i++) { dmaBuffer2048Bytes[i] = 0xDD; }
    device.flash.partialWrite(0, 1536, dmaBuffer2048Bytes, 512);
    status = device.flash.getFeatures(GET_FEATURES_STATUS);
    printf("Status 3: %d\n", status);*/

    device.flash.printFlash(0, 5, 10);

    status = device.flash.getFeatures(GET_FEATURES_STATUS);
    printf("Status After Print: %d\n", status);

    if(!device.flash.read(0, 0, dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
        printf("Error, flash read failed\n");
    }
    status = device.flash.getFeatures(GET_FEATURES_STATUS);
    printf("Status Read: %d\n", status);
}

void testTimeSync() {
    device.measureTime("Before");
    if(!device.initWiFi()) { printf("ERROR1\n"); }
    //if(!device.connectToWiFiDirectly("Korntal", "xxxxxxx", RADIO_MAX_TX_19_5_DBM, 1)) { printf("ERROR2\n"); }
    if(!device.connectToWiFiDirectly("Tracker1", "Tracker1", RADIO_MAX_TX_19_5_DBM, 7)) { printf("ERROR2\n"); }
    //if(!device.connectToWiFiDirectly("guest", "xxxxxxx", RADIO_MAX_TX_19_5_DBM, 7)) { printf("ERROR2\n"); }
    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) { ; }
    if(device.connectedToWiFi() == WIFI_CONNECT_SUCCESS) {
        device.measureTime("Wifi connected");
        uint32_t timestampUTC = 0;
        if(!device.getNTPTimestampUTC(timestampUTC, 6000, "pool.ntp.org")) {
            printf("UTC GET ERROR!\n");
        }
        device.measureTime("GOT time?");
        printf("Timestamp: %d\n", timestampUTC);
        device.disconnectAndStopWiFi();
    }
    else {
        device.measureTime("Wifi not connected");
        printf("Couldn't connect to WIFI\n");
    }
}

void streamDirectConnect() {
    if(!device.initWiFi()) { printf("ERROR1\n"); }
    device.measureTime("initWiFi");

    //uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save WatchDog register
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

    if(!device.connectToWiFiDirectly("Korntal", "xxxxxxx", RADIO_MAX_TX_19_5_DBM, WIFI_SCAN_CHANNEL)) { printf("ERROR2\n"); } // old LDO: BROWNOUT!
    //if(!device.connectToWiFiDirectly("Tracker1", "Tracker1", RADIO_MAX_TX_19_5_DBM, 7)) { printf("ERROR2\n"); }
    //if(!device.connectToWiFiDirectly("guest", "xxxxxxx", RADIO_MAX_TX_19_5_DBM, 7)) { printf("ERROR2\n"); }
    device.measureTime("connectToWiFiDirectly");
    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) { ; }
    printf("ConnectedToWiFi = %d\n", device.connectedToWiFi());
    device.measureTime("while(!device.connectedToWiFi())");
    if(device.connectedToWiFi() == WIFI_CONNECT_SUCCESS) {
        if(!device.flash.createBuffer(&dmaStreamBuffer, MT29_CACHE_SIZE)) {
            printf("ERROR BUFF\n");
            device.disconnectAndStopWiFi();
            return;
        }
        restStreamParams.url = REST_URL;
        restStreamParams.contentType = REST_CONTENT_TYPE;
        restStreamParams.additionalHeaderKey = REST_ADD_HEADER_KEY;
        restStreamParams.additionalHeaderValue = REST_ADD_HEADER_VALUE;
        restStreamParams.prefix = REST_PAYLOAD_PREFIX;
        restStreamParams.dataDMA2048Bytes = &dmaStreamBuffer[0];
        restStreamParams.postfix = REST_PAYLOAD_POSTFIX;
        restStreamParams.flashObject = &device.flash;
        restStreamParams.flashBlockToSendNextPointer = 0;
        restStreamParams.flashHalfBlockToSendNextPointer = 1;
        restStreamParams.flashMaxNumberOfBlocksToTransmit = 3;
        restStreamParams.deviceObject = &device;
        restStreamParams.minBatteryVoltageToContinue = 500; // 3650
        restStreamParams.debug = true;
        device.doWiFiPOSTStreamCallFlash(&restStreamParams, 8192);
        while(device.getWiFiPOSTCallStatus() == HTTP_POST_DATA_RUNNING) { // TODO: TIMEOUT???
            device.delay(50);
        }
        uint16_t blocks = 0, halfBlocks = 0;
        device.restPostStreamGetSuccessfullyTransmittedBlocks(blocks, halfBlocks);
        printf("SUCCESSFULLY TRANSMITTED BLOCKS: %d, HALF: %d\n", blocks, halfBlocks);
        heap_caps_free(dmaStreamBuffer);
        device.disconnectAndStopWiFi();
    }
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector again
}

void stream() {
    device.measureTime("startStream");
    if(!device.initWiFi()) { printf("ERROR1\n"); }
    device.measureTime("initWiFi");

    if(!device.scanForWiFis(false, RADIO_MAX_TX_19_5_DBM, 120, WIFI_SCAN_CHANNEL)) { printf("ERROR\n"); }
    device.printWiFiScanResults();
    device.measureTime("scanForWiFis");
    while(!device.wiFiScanCompleted()) {
        device.delay(20);
    }
    device.measureTime("wiFiScanCompleted");

    if(device.wiFiScanIncludes("Tracker1")) {
        //if(!device.connectToWiFiAfterScan("Korntal", "xxxxxxx", WIFI_SCAN_CHANNEL)) { printf("ERROR\n"); }
        if(!device.connectToWiFiAfterScan("Tracker1", "Tracker1", WIFI_SCAN_CHANNEL)) { printf("ERROR\n"); }
        device.measureTime("connectToWiFiAfterScan");
        while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
            device.delay(20);
        }
        device.measureTime("connectedToWiFi");
        if(device.connectedToWiFi() == WIFI_CONNECT_SUCCESS) {
            if(!device.flash.createBuffer(&dmaStreamBuffer, MT29_CACHE_SIZE)) {
                printf("ERROR BUFF\n");
                device.disconnectAndStopWiFi();
                return;
            }
            restStreamParams.url = REST_URL;
            restStreamParams.contentType = REST_CONTENT_TYPE;
            restStreamParams.additionalHeaderKey = REST_ADD_HEADER_KEY;
            restStreamParams.additionalHeaderValue = REST_ADD_HEADER_VALUE;
            restStreamParams.prefix = REST_PAYLOAD_PREFIX;
            restStreamParams.dataDMA2048Bytes = &dmaStreamBuffer[0];
            restStreamParams.postfix = REST_PAYLOAD_POSTFIX;
            restStreamParams.flashObject = &device.flash;
            restStreamParams.flashBlockToSendNextPointer = 1;
            restStreamParams.flashHalfBlockToSendNextPointer = 0;
            restStreamParams.flashMaxNumberOfBlocksToTransmit = 8;
            restStreamParams.deviceObject = &device;
            restStreamParams.minBatteryVoltageToContinue = 500; // 3650
            restStreamParams.debug = true;
            uint16_t successfullyTransmittedBlocks = 0;
            uint16_t successfullyTransmittedHalfBlocks = 0;
            uint32_t wifiStartTime = ((uint32_t) Arduino::millisWrapper()); // reset timer
            device.doWiFiPOSTStreamCallFlash(&restStreamParams, 8192);
            while(device.getWiFiPOSTCallStatus() == HTTP_POST_DATA_RUNNING) { // TODO: TIMEOUT???
                device.delay(100);
                /* -------- WATCHDOG POST TASK -------- */
                if(((uint32_t) Arduino::millisWrapper()) - wifiStartTime > 12000) { // additional watchdog in case task is not timing out by itself
                    printf("%d WATCHDOG: TASK TIMEOUT triggered, check if blocks were transmitted or not!\n", ((uint32_t) Arduino::millisWrapper()));
                    uint16_t successfullyTransmittedBlocksNew = 0;
                    uint16_t successfullyTransmittedHalfBlocksNew = 0; // don't look on half blocks
                    device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocksNew, successfullyTransmittedHalfBlocksNew);
                    if(successfullyTransmittedBlocksNew - successfullyTransmittedBlocks > 0) {
                        printf("%d WATCHDOG: %d blocks transfered, continue waiting!\n", ((uint32_t) Arduino::millisWrapper()), (successfullyTransmittedBlocksNew - successfullyTransmittedBlocks));
                        wifiStartTime = ((uint32_t) Arduino::millisWrapper()); // reset timer
                        successfullyTransmittedBlocks = successfullyTransmittedBlocksNew; // update last checked block value
                    }
                    else {
                        printf("%d WATCHDOG: NO blocks transfered, KILL TASK!\n", ((uint32_t) Arduino::millisWrapper()));
                        //device.killPOSTTask(); // brutal! don't do! leads to reset
                        break; // only BREAK from loop, maybe some block transmitted
                    }
                }
            }
            uint16_t blocks = 0, halfBlocks = 0;
            device.restPostStreamGetSuccessfullyTransmittedBlocks(blocks, halfBlocks);
            printf("STATUS RETURN: %d\n", device.getWiFiPOSTCallStatus());
            printf("SUCCESSFULLY TRANSMITTED BLOCKS: %d, HALF: %d\n", blocks, halfBlocks);
            heap_caps_free(dmaStreamBuffer);
            device.measureTime("doWiFiPOSTStreamCallFlash");
        }
    }
    device.disconnectAndStopWiFi();
}

extern "C" void app_main() {
    while(1) {
        printf("AWAKE!\n");
        if(firstStart) {
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            device.delay(3000);
            t = (uint32_t) Arduino::millisWrapper();
            device.flashPowerOn();
            stream();
            //flashTest();
            device.flashPowerOff();
            firstStart = false;
        }
        /*device.measureTime("boot");
        if(!device.initDataNVS()) { printf("ERROR\n"); }
        device.measureTime("initDataNVS");
        device.printDataNVSStats();
        device.measureTime("printDataNVSStats");
        uint32_t test = device.nvsReadUINT32("test");
        device.measureTime("nvsReadUINT32");
        printf("VALUE: %d\n", test);
        test++;
        device.nvsWriteUINT32("test", test);
        device.measureTime("nvsWriteUINT32");*/

        printf("SLEEPIN\n");
        device.enableInternalTimerInterruptInDeepSleep(60);
        device.deepSleep();
    }
}