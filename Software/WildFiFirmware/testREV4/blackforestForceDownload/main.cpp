#include "ESP32TrackerREV4.h"

ESP32TrackerREV4 device = ESP32TrackerREV4();

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint32_t bootCnt = 0;

#define TAG_NAME                                        "PANG11"                        // ALWAYS 6 CHARACTERS! otherwise REST_ADD_HEADER_VALUE not correct, unique id for post calls


/** NVS storage names */
#define NVS_FLASH_SEND_NEXT_BLOCK_POINTER               "blockpntsent"                  // data transmission: pointing on next block to transmit
#define NVS_FLASH_WRITE_POINTER                         "flashpnt"                      // data writing: pointing on current flash page
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffsetpnt"                // data writing: pointing on current byte in flash page
#define NVS_FLASH_TAG_ACTIVATED_BY_WIFI                 "activated"                     // tag has seen activation wifi
// ONLY FOR ESP NOW DATA TRANSMISSION
    #define NVS_FLASH_SEND_NEXT_PAGE_POINTER            "pagepntsent"                   // data transmission: pointing on next page within block to transmit
    #define NVS_FLASH_SEND_NEXT_SUBPAGE_POINTER         "subpagesent"                   // data transmission: pointing on next subpage within page to transmit
// ONLY FOR WIFI DATA TRANSMISSION
    #define NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER      "halfblocksent"                 // data transmission: sometimes only half a block transmitted -> save and do not retransmit
#define WIFI_OUTPUT_POWER                               RADIO_MAX_TX_11_DBM             // 19.5dBm will brown out on REV3
#define WIFI_MIN_BATT_VOLTAGE                           3500                            // don't start scan if battery voltage too low
#define WIFI_MIN_BATT_VOLTAGE_DURING_TRANSM             3200                            // don't continue wifi transmission if voltage dropped below that threshold after one block transmission
#define WIFI_MAX_POST_TASK_TIME_SECONDS                 45                              // max time for sending one successfully transmitted block
const uint8_t DATATR_KNOWN_WIFI_LIST_SIZE =             1;                              // IMPORTANT: DO NOT FORGET TO CHANGE THAT!!!!!!!!!
const char* DATATR_WIFI_SSIDS[DATATR_KNOWN_WIFI_LIST_SIZE] = {                                        // first priority: channel (6, 1, 11), within channel the name
                                                        /*"guest", "LiWoAb New",*/ "mpidata"
};                      
const char* DATATR_WIFI_PASSWORDS[DATATR_KNOWN_WIFI_LIST_SIZE] = {                                    // first priority: channel (6, 1, 11), within channel the name
                                                        /*"xxxxxxx", "xxxxxxx",*/ "87654321"
}; 
const char* REST_URL =                                  "http://192.168.43.1:8080/store";
const char* REST_CONTENT_TYPE =                         "application/octet-stream";
const char* REST_ADD_HEADER_KEY =                       "Content-Length";
const char* REST_ADD_HEADER_VALUE =                     "65563"; // 131099 for full blocks, 65563 for half blocks, strlen(PREFIX_CONSTRUCTED: "ABCDEF:PPPPPPPP:VVVV:") 21 + strlen(taskParams.postfix) 6 + (PAGES_IN_ONE_GO * 2048 = 65536)
#define REST_USE_BASE64_ENCODING                        false
#define REST_SEND_FULL_BLOCKS                           false
#define REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX            true
const char* REST_PAYLOAD_PREFIX =                       TAG_NAME ":PPPPPPPP:VVVV:"; // P = page address, V = battery voltage
const char* REST_PAYLOAD_POSTFIX =                      TAG_NAME;
post_task_stream_flash_parameters_t restStreamParams;
uint8_t *dmaBuffer2048Bytes = NULL;
uint8_t *additionalDataAfterPrefixPointer = NULL;

#define MODE_TESTRUN                                    0                               // with debug output, not writing into flash memory, not incrementing NVS_FLASH_POINTER
#define TRACKER_MODE                                    MODE_TESTRUN

/** Configuration Data Transmission: Common */
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY       1                               // 12*60 = 00:00 and 12:00 (UTC), 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission nothing transmitted -> re-try more seldomly
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY     1                               // 1 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission something transmitted -> re-try more often
#define DATATRANSM_MIN_BLOCKS_TO_TRANSMIT               1                               // only start scan (and data transmission) if there is enough data to send
#define DATATRANSM_MAX_BLOCKS_TO_TRANSMIT               150                             // during one connection, burst maximum this amount of blocks (maximum memory blocks = 2048)

/** Mocking for data transmission */
#define MOCK_FLASH_DELETE                               1                               // 1 = flash block will not be deleted after successful data transmission, but all pointers are updated

/** Connect to WIFI Configuration */
#define WIFI_MAX_CONNECT_TIME_SECONDS                   8                               // (used for get time, activation and data transmission) connect should take <1s


RTC_DATA_ATTR uint8_t lastErrorId = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;
RTC_DATA_ATTR uint32_t timestampNextDataTransmission = 0;

void setNextDataTransmissionTimestamp(bool forceMode, uint32_t currentTimestamp, uint32_t onMinute) { // when time = 17:53 and transmission interval is 2 hours -> set to 18:00
    return;
}

bool itsTimeForDataTransmission(uint32_t currentTimestamp) {
    return true;
}

bool dataTransmissionWifi(bool forceMode, uint16_t minBlocksToTransmit, uint32_t currentTimestamp, uint32_t batteryVoltage, uint32_t flashPointer, const char** SSIDS_TO_USE, const char** PASSWORDS_TO_USE, const uint8_t SSIDS_SIZE) { // perform a wifi scan (366ms scan only, 1515ms in total if connecting)
    uint32_t wifiStartTime = ((uint32_t) Arduino::millisWrapper());
    bool somethingTransmitted = false; // at least ONE block deleted (so more space in memory than before)

    /* -------- GET POINTER VALUES FROM NVS -------- */
    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER);
    uint16_t flashHalfBlockToSendNextPointer = 0;
    if(!REST_SEND_FULL_BLOCKS) { device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER); }

    /* -------- GET NUMBER OF BLOCKS TO TRANSMIT -------- */
    uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockToSendNextPointer, flashPointer);

    /* -------- CHECK CONDITIONS TO EXECUTE WIFI SCAN -------- */
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: next scan %d, blocksToTransmit %d, batteryV %d\n", ((uint32_t) Arduino::millisWrapper()), timestampNextDataTransmission, blocksToTransmit, batteryVoltage); }
    if(batteryVoltage > WIFI_MIN_BATT_VOLTAGE) { // only if enough juice in battery
        if(blocksToTransmit >= minBlocksToTransmit) { // only if enough data to transmit
            if(forceMode || itsTimeForDataTransmission(currentTimestamp)) { // only if last scan at least x seconds ago
                if((TRACKER_MODE == MODE_TESTRUN) && (forceMode)) { printf("%d WIFI: FORCING SCAN!\n", ((uint32_t) Arduino::millisWrapper())); }
                /* -------- NEW SCAN ALLOWED -------- */
                if(!device.initWiFi()) { lastErrorId = 63; errorCnt++; setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); return somethingTransmitted; }
                uint8_t foundArrayId = 0;
                uint8_t foundOnChannel = 0;
                if(!device.scanForWiFisOn1and6and11(SSIDS_TO_USE, SSIDS_SIZE, &foundArrayId, &foundOnChannel, WIFI_OUTPUT_POWER, 120, 500)) { device.disconnectAndStopWiFi(); lastErrorId = 64; errorCnt++; setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); return somethingTransmitted; }
                else {
                    if(foundOnChannel > 0) {
                        /* -------- MY WIFI FOUND -------- */
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: FOUND NO %d (CHAN: %d)!\n", ((uint32_t) Arduino::millisWrapper()), foundArrayId, foundOnChannel); }
                        uint8_t connectionAttemptCounter = 0;
                        while(true) { // try multiple times to connect, because wifi has already been seen
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: CONNECTION ATTEMPT %d!\n", ((uint32_t) Arduino::millisWrapper()), connectionAttemptCounter); }
                            if(!device.connectToWiFiAfterScan(SSIDS_TO_USE[foundArrayId], PASSWORDS_TO_USE[foundArrayId], foundOnChannel)) { lastErrorId = 66; errorCnt++; }
                            while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                                device.delay(20);
                                if(((uint32_t) Arduino::millisWrapper()) - wifiStartTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: TIMEOUT CONNECT!\n", ((uint32_t) Arduino::millisWrapper())); }
                                    device.disconnectAndStopWiFi();
                                    lastErrorId = 67; errorCnt++;
                                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); // re-try not so often
                                    return somethingTransmitted; // severe error, return immediately, no re-try
                                }
                            }
                            if((device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                                if(connectionAttemptCounter < 2) { // retry two times
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: access point seen, but could not connect: %d! -> RETRY ONCE\n", ((uint32_t) Arduino::millisWrapper()), device.connectedToWiFi()); }
                                }
                                else {
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: access point seen, but could not connect: %d! -> CANCEL\n", ((uint32_t) Arduino::millisWrapper()), device.connectedToWiFi()); }
                                    device.disconnectAndStopWiFi();
                                    lastErrorId = 68; errorCnt++;
                                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); // re-try not so often
                                    return somethingTransmitted; // severe error
                                }
                            }
                            else {
                                break; // connected!
                            }
                            connectionAttemptCounter++;
                        }
                        /* -------- CONNECTED TO WIFI -------- */
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: CONNECT TOOK %dms (MAX %d)!\n", ((uint32_t) Arduino::millisWrapper()), ((uint32_t) Arduino::millisWrapper()) - wifiStartTime, (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)); }
                        /* -------- FLASH OPERATION -------- */
                        if(!device.flash.createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: FLASH DMA RESERVE ERROR!\n", ((uint32_t) Arduino::millisWrapper())); }
                            device.disconnectAndStopWiFi();
                            lastErrorId = 69; errorCnt++;
                            setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); // re-try not so often
                            return somethingTransmitted;
                        }
                        /* -------- DMA BUFFER FOR FLASH CREATED -------- */
                        uint16_t blocksToActuallyTransmit = blocksToTransmit;
                        if(blocksToActuallyTransmit > DATATRANSM_MAX_BLOCKS_TO_TRANSMIT) { // for REST POST routine: limit maximum number of blocks to transmit (will be less if voltage low or errors)
                            blocksToActuallyTransmit = DATATRANSM_MAX_BLOCKS_TO_TRANSMIT;
                        }
                        /* -------- FILL PARAMETERS -------- */
                        restStreamParams.url = REST_URL;
                        restStreamParams.contentType = REST_CONTENT_TYPE;
                        restStreamParams.additionalHeaderKey = REST_ADD_HEADER_KEY;
                        restStreamParams.additionalHeaderValue = REST_ADD_HEADER_VALUE;
                        restStreamParams.prefix = REST_PAYLOAD_PREFIX;
                        restStreamParams.constructCustomPrefix = REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX;
                        restStreamParams.dataDMA2048Bytes = &dmaBuffer2048Bytes[0];
                        restStreamParams.postfix = REST_PAYLOAD_POSTFIX;
                        restStreamParams.flashObject = &device.flash;
                        restStreamParams.flashBlockToSendNextPointer = flashBlockToSendNextPointer;
                        restStreamParams.flashHalfBlockToSendNextPointer = flashHalfBlockToSendNextPointer;
                        restStreamParams.flashMaxNumberOfBlocksToTransmit = blocksToActuallyTransmit;
                        restStreamParams.deviceObject = &device;
                        restStreamParams.minBatteryVoltageToContinue = WIFI_MIN_BATT_VOLTAGE_DURING_TRANSM;
                        restStreamParams.useBase64Encoding = REST_USE_BASE64_ENCODING;
                        restStreamParams.debug = (TRACKER_MODE == MODE_TESTRUN);
                        /* -------- START POST TASK -------- */
                        uint16_t successfullyTransmittedBlocks = 0;
                        uint16_t successfullyTransmittedHalfBlocks = 0;
                        wifiStartTime = ((uint32_t) Arduino::millisWrapper()); // reset watchdog timer
                        if(REST_SEND_FULL_BLOCKS) { device.doWiFiPOSTStreamCallFlashFullBlock(&restStreamParams, 8192); }
                        else { device.doWiFiPOSTStreamCallFlash(&restStreamParams, 8192); }
                        while(device.getWiFiPOSTCallStatus() == HTTP_POST_DATA_RUNNING) {
                            device.delay(100);
                            /* -------- WATCHDOG POST TASK -------- */
                            if(((uint32_t) Arduino::millisWrapper()) - wifiStartTime > (WIFI_MAX_POST_TASK_TIME_SECONDS * 1000)) { // additional watchdog in case task is not timing out by itself
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: TASK TIMEOUT triggered, check if blocks were transmitted or not!\n", ((uint32_t) Arduino::millisWrapper())); }
                                uint16_t successfullyTransmittedBlocksNew = 0;
                                uint16_t successfullyTransmittedHalfBlocksNew = 0; // don't look on half blocks
                                device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocksNew, successfullyTransmittedHalfBlocksNew);
                                if(successfullyTransmittedBlocksNew - successfullyTransmittedBlocks > 0) {
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: %d blocks transfered, continue waiting!\n", ((uint32_t) Arduino::millisWrapper()), (successfullyTransmittedBlocksNew - successfullyTransmittedBlocks)); }
                                    wifiStartTime = ((uint32_t) Arduino::millisWrapper()); // reset timer
                                    successfullyTransmittedBlocks = successfullyTransmittedBlocksNew; // update last checked block value
                                }
                                else {
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO blocks transfered, KILL TASK!\n", ((uint32_t) Arduino::millisWrapper())); }
                                    //device.killPOSTTask(); // brutal! don't do! leads to reset
                                    break; // only BREAK from loop, maybe some block transmitted
                                }
                            }
                        }
                        /* -------- POST TASK FINISHED -------- */
                        if(device.getWiFiPOSTCallStatus() != HTTP_POST_DATA_FINISHED_ALL_GOOD) { // might still be RUNNING! when watchdog kicked in (will then lead to client write error because of software connection abort)
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: POST was not successful: %d!\n", ((uint32_t) Arduino::millisWrapper()), device.getWiFiPOSTCallStatus()); }
                            lastErrorId = 70 + device.getWiFiPOSTCallStatus(); // 19 error codes -> 70 + 19 = 89, next errorid should be 90
                            errorCnt++;
                        }
                        device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks);
                        
                        if(TRACKER_MODE == MODE_TESTRUN) {
                            if(REST_SEND_FULL_BLOCKS) { printf("%d WIFI: successfully transmitted blocks: %d (half block transmission deactivated)\n", ((uint32_t) Arduino::millisWrapper()), successfullyTransmittedBlocks); }
                            else { printf("%d WIFI: successfully transmitted blocks: %d, half blocks: %d\n", ((uint32_t) Arduino::millisWrapper()), successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks); }
                        }
                        //heap_caps_free(dmaBuffer2048Bytes); // CAREFUL: task might still running if it supposed to be killed? -> DO NOT FREE MEMORY AT ALL
                        // also do not free memory of restPrefixPointer
                        /* -------- DELETING SUCCESSFULLY TRANSMITTED BLOCKS (ONLY FULLY TRANSMITTED) -------- */
                        uint16_t flashBlockToSendNextPointerBeforeDelete = flashBlockToSendNextPointer; // remember old value for NVS update
                        for(uint16_t delBlocks=0; delBlocks<successfullyTransmittedBlocks; delBlocks++) { // deleting is based ONLY on fully transmitted blocks
                            if(!device.flash.fifoPopDelete(flashBlockToSendNextPointer, flashPointer, MT29_NUMBER_PAGES, MOCK_FLASH_DELETE)) { // delete block from flash and increment flashBlockToSendNextPointer
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: FIFO POP DELETE FAILED!\n", ((uint32_t) Arduino::millisWrapper())); }
                                device.disconnectAndStopWiFi();
                                lastErrorId = 90; errorCnt++;
                                setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); // re-try not so often
                                return somethingTransmitted; // error here means -> no pointers are updated, retransmitting maybe already deleted blocks, but very unlikely to happen
                            } 
                        }
                        /* -------- UPDATING NVS POINTER -------- */
                        // FULL BLOCK POINTER
                        if(flashBlockToSendNextPointerBeforeDelete != flashBlockToSendNextPointer) { // some blocks are now fully transmitted
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: updating flashBlockToSendNextPointer (old: %d, new: %d)\n", ((uint32_t) Arduino::millisWrapper()), flashBlockToSendNextPointerBeforeDelete, flashBlockToSendNextPointer); }
                            device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER, flashBlockToSendNextPointer);
                            somethingTransmitted = true; // ONLY HERE, not when half blocks were transfered because blocks are not deleted
                        }
                        // HALF BLOCK POINTER
                        if(!REST_SEND_FULL_BLOCKS) {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: halfBlock points to: %d, successfullyTransmittedHalfBlocks: %d\n", ((uint32_t) Arduino::millisWrapper()), flashHalfBlockToSendNextPointer, successfullyTransmittedHalfBlocks); }
                            if((flashHalfBlockToSendNextPointer == 0) && (successfullyTransmittedHalfBlocks == 0)) { // do not update, before no half transmitted block, afterwards also not, means everything went smooth
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 1) happy case, no half blocks before or after\n", ((uint32_t) Arduino::millisWrapper())); }
                            }
                            else if((flashHalfBlockToSendNextPointer == 0) && (successfullyTransmittedHalfBlocks == 1)) { // before no half transmissions, now a half transmission (maybe only 0,5 blocks transmitted or maybe 10,5) -> update!
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 2) half block update -> before all good, now a half transmission\n", ((uint32_t) Arduino::millisWrapper())); }
                                device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER, 1);
                            }
                            else if((flashHalfBlockToSendNextPointer == 1) && (successfullyTransmittedHalfBlocks == 0)) { // before only half a block was transmitted, now finished this block + maybe more (0,5 or 5,5 blocks transmitted) OR (bug before) 0,0 blocks transmitted
                                if(flashBlockToSendNextPointerBeforeDelete != flashBlockToSendNextPointer) { // there WAS an actual block transmission, means we update NVS
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 3A) half block before, now actually some blocks transmitted -> half block update\n", ((uint32_t) Arduino::millisWrapper())); }
                                    device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER, 0);
                                }
                                else {
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 3B) nothing at all transmitted, keep flashHalfBlockToSendNextPointer = %d\n", ((uint32_t) Arduino::millisWrapper()), flashHalfBlockToSendNextPointer); }
                                }
                            }
                            else if((flashHalfBlockToSendNextPointer == 1) && (successfullyTransmittedHalfBlocks == 1)) { // before a half transmissions, now some blocks AND a half transmission again
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 4) bad luck, half block transmission before AND after, keep half block pointer\n", ((uint32_t) Arduino::millisWrapper())); }
                            }
                        }
                    }
                    else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: SCANNED, but not found\n", ((uint32_t) Arduino::millisWrapper())); }
                    device.disconnectAndStopWiFi(); // disconnect here because before that no wifi actions
                }
                /* -------- UPDATING SCANNING FREQUENCY AFTER A SCAN (OR DATATRANSMISSION!!!) WAS PERFORMED -------- */
                if(somethingTransmitted) { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); } // data to transmit and wifi found and data transmitted -> try more frequently (if enough data)
                else { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); } // data to transmit, but wifi not found or data transmission not acked -> try less frequently
            }
            else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> next time %d, current time %d, wait %ds\n", ((uint32_t) Arduino::millisWrapper()), timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp); }
        }
        else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> Blocks to transmit %d < %d\n", ((uint32_t) Arduino::millisWrapper()), blocksToTransmit, minBlocksToTransmit); }
    }
    else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> Battery too low\n", ((uint32_t) Arduino::millisWrapper())); }
    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            printf("FIRST START\n");
            device.delay(5000);
            firstStart = false;
        }

        if(!device.flashPowerOn(true)) { printf("ERROR FLASH\n"); } // turn on flash power already (5ms)
        if(!device.initDataNVS()) { printf("ERROR NVS\n"); }
        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
        uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER);

        /*
        PANG11
        flashPointer: 131071, flashOffsetPointer = 42, flashBlockToSendNextPointer: 150
        FIFO free space in memory: 19662806, blocksToTransmit: 1897
        */
        printf("flashPointer: %d, flashOffsetPointer = %d, flashBlockToSendNextPointer: %d\n", flashPointer, flashOffsetPointer, flashBlockToSendNextPointer);
        uint32_t freeSpaceBytesInFIFO = device.flash.fifoGetFreeSpace(flashBlockToSendNextPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES);
        uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockToSendNextPointer, flashPointer);
        printf("FIFO free space in memory: %d, blocksToTransmit: %d\n", freeSpaceBytesInFIFO, blocksToTransmit);

        bool somethingTransmitted = false;
        somethingTransmitted = dataTransmissionWifi(true, 1, 0, 3750, flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
        
        printf("Something transmitted: %d, errorCnt %d, lastErrorId %d\n", somethingTransmitted, errorCnt, lastErrorId);

        if(!device.flashPowerOff()) { printf("ERROR FLASH\n"); } // turn on flash power already (5ms)

        device.enableInternalTimerInterruptInDeepSleep(10);
        printf("\n");
        bootCnt++;
        device.deepSleep();
    }
}