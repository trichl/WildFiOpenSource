#include "PlatformWildFiTagREV6.h"

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR uint64_t bootCnt = 0;

// TASK SIZE:               NO IMPACT! (8*8192)
// DMA_BUFFER_SIZE = 8:     FASTER
// BUFFER NOT IN DMA:       NO IMPACT
// PERFORMANCE SDKCONF:     
// device.setCPUSpeed(240)  NEEDS MORE POWER

#define WIFI_OUTPUT_POWER                               RADIO_MAX_TX_19_5_DBM             // 19.5dBm will brown out on REV3

//const char* REST_URL =                                  "http://a445d13db675.ngrok.io/";
const char* REST_URL =                                  "http://192.168.43.1:8080/store";
const char* REST_CONTENT_TYPE =                         "application/octet-stream";
const char* REST_ADD_HEADER_KEY =                       "Content-Length";
const char* REST_ADD_HEADER_VALUE =                     "131099"; // 131099 for full blocks, 65563 for half blocks, strlen(PREFIX_CONSTRUCTED: "ABCDEF:PPPPPPPP:VVVV:") 21 + strlen(taskParams.postfix) 6 + (PAGES_IN_ONE_GO * 2048 = 65536)
#define REST_USE_BASE64_ENCODING                        false
#define REST_SEND_FULL_BLOCKS                           true
#define REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX            true
#define TAG_NAME                                        "TST001"
const char* REST_PAYLOAD_PREFIX =                       TAG_NAME ":PPPPPPPP:VVVV:"; // P = page address, V = battery voltage
const char* REST_PAYLOAD_POSTFIX =                      TAG_NAME;
post_task_stream_flash_parameters_t restStreamParams;
//uint8_t *dmaBuffer2048Bytes = NULL;
uint8_t *additionalDataAfterPrefixPointer = NULL;

const uint8_t DATATR_KNOWN_WIFI_LIST_SIZE =             2;  
const char* DATATR_WIFI_SSIDS[DATATR_KNOWN_WIFI_LIST_SIZE] = {                                        // first priority: channel (6, 1, 11), within channel the name
                                                        "mpidata",
                                                        "RodelbahnSoelden"
};                      
const char* DATATR_WIFI_PASSWORDS[DATATR_KNOWN_WIFI_LIST_SIZE] = {                                    // first priority: channel (6, 1, 11), within channel the name
                                                        "87654321",
                                                        "xxxxxxxx"
}; 

bool dataTransmissionWifi(bool forceMode, uint16_t minBlocksToTransmit, uint32_t currentTimestamp, uint32_t batteryVoltage, uint32_t flashPointer, const char** SSIDS_TO_USE, const char** PASSWORDS_TO_USE, const uint8_t SSIDS_SIZE) { // perform a wifi scan (366ms scan only, 1515ms in total if connecting)
    uint32_t wifiStartTime = ((uint32_t) Timing::millis());
    bool somethingTransmitted = false; // at least ONE block deleted (so more space in memory than before)

    /* -------- GET POINTER VALUES FROM NVS -------- */
    uint16_t flashBlockDeletedPointer = 0;
    uint16_t flashHalfBlockToSendNextPointer = 0;

    /* -------- GET NUMBER OF BLOCKS TO TRANSMIT -------- */
    uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockDeletedPointer, flashPointer);

    /* -------- CHECK CONDITIONS TO EXECUTE WIFI SCAN -------- */
    if(batteryVoltage > 3300) { // only if enough juice in battery
        if(blocksToTransmit >= minBlocksToTransmit) { // only if enough data to transmit
            if(forceMode) { // only if last scan at least x seconds ago
                if(forceMode) { printf("%d WIFI: FORCING SCAN!\n", ((uint32_t) Timing::millis())); }
                /* -------- NEW SCAN ALLOWED -------- */
                if(!device.initWiFi()) { return somethingTransmitted; }
                uint8_t foundArrayId = 0;
                uint8_t foundOnChannel = 0;
                if(!device.scanForWiFisOn1and6and11(SSIDS_TO_USE, SSIDS_SIZE, &foundArrayId, &foundOnChannel, WIFI_OUTPUT_POWER, 120, 500)) { device.disconnectAndStopWiFi(); return somethingTransmitted; }
                else {
                    if(foundOnChannel > 0) {
                        /* -------- MY WIFI FOUND -------- */
                        printf("%d WIFI: FOUND NO %d (CHAN: %d)!\n", ((uint32_t) Timing::millis()), foundArrayId, foundOnChannel);
                        uint8_t connectionAttemptCounter = 0;
                        while(true) { // try multiple times to connect, because wifi has already been seen
                            printf("%d WIFI: CONNECTION ATTEMPT %d!\n", ((uint32_t) Timing::millis()), connectionAttemptCounter);
                            if(!device.connectToWiFiAfterScan(SSIDS_TO_USE[foundArrayId], PASSWORDS_TO_USE[foundArrayId], foundOnChannel)) { }
                            while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                                device.delay(20);
                                if(((uint32_t) Timing::millis()) - wifiStartTime > (8 * 1000)) { // e.g. password wrong
                                    printf("%d WIFI: TIMEOUT CONNECT!\n", ((uint32_t) Timing::millis()));
                                    device.disconnectAndStopWiFi();
                                    return somethingTransmitted; // severe error, return immediately, no re-try
                                }
                            }
                            if((device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                                if(connectionAttemptCounter < 2) { // retry two times
                                    printf("%d WIFI: access point seen, but could not connect: %d! -> RETRY ONCE\n", ((uint32_t) Timing::millis()), device.connectedToWiFi());
                                }
                                else {
                                    printf("%d WIFI: access point seen, but could not connect: %d! -> CANCEL\n", ((uint32_t) Timing::millis()), device.connectedToWiFi());
                                    device.disconnectAndStopWiFi();
                                    return somethingTransmitted; // severe error
                                }
                            }
                            else {
                                break; // connected!
                            }
                            connectionAttemptCounter++;
                        }
                        /* -------- CONNECTED TO WIFI -------- */
                        printf("%d WIFI: CONNECT TOOK %dms (MAX %d)!\n", ((uint32_t) Timing::millis()), ((uint32_t) Timing::millis()) - wifiStartTime, (8 * 1000));
                        /* -------- FLASH OPERATION -------- */
                        /*if(!device.flash.createBuffer(&dmaBuffer2048Bytes, DMA_BUFFER_SIZE)) {
                            printf("%d WIFI: FLASH DMA RESERVE ERROR!\n", ((uint32_t) Timing::millis()));
                            device.disconnectAndStopWiFi();
                            return somethingTransmitted;
                        }*/
                        /* -------- DMA BUFFER FOR FLASH CREATED -------- */
                        uint16_t blocksToActuallyTransmit = blocksToTransmit;
                        /* -------- FILL PARAMETERS -------- */
                        restStreamParams.url = REST_URL;
                        restStreamParams.contentType = REST_CONTENT_TYPE;
                        restStreamParams.additionalHeaderKey = REST_ADD_HEADER_KEY;
                        restStreamParams.additionalHeaderValue = REST_ADD_HEADER_VALUE;
                        restStreamParams.prefix = REST_PAYLOAD_PREFIX;
                        restStreamParams.constructCustomPrefix = REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX;
                        //restStreamParams.dataDMA2048Bytes = &dmaBuffer2048Bytes[0];
                        restStreamParams.postfix = REST_PAYLOAD_POSTFIX;
                        restStreamParams.flashObject = &device.flash;
                        restStreamParams.flashBlockToSendNextPointer = flashBlockDeletedPointer;
                        restStreamParams.flashHalfBlockToSendNextPointer = flashHalfBlockToSendNextPointer;
                        restStreamParams.flashMaxNumberOfBlocksToTransmit = blocksToActuallyTransmit;
                        restStreamParams.deviceObject = &device;
                        restStreamParams.minBatteryVoltageToContinue = 3300;
                        restStreamParams.useBase64Encoding = REST_USE_BASE64_ENCODING;
                        restStreamParams.debug = true;
                        /* -------- START POST TASK -------- */
                        uint16_t successfullyTransmittedBlocks = 0;
                        uint16_t successfullyTransmittedHalfBlocks = 0;
                        wifiStartTime = ((uint32_t) Timing::millis()); // reset watchdog timer
                        if(REST_SEND_FULL_BLOCKS) { device.doWiFiPOSTStreamCallFlashFullBlock(&restStreamParams); }
                        else { device.doWiFiPOSTStreamCallFlashHalfBlock(&restStreamParams); }
                        while(device.getWiFiPOSTCallStatus() == HTTP_POST_DATA_RUNNING) {
                            device.delay(100);
                            /* -------- WATCHDOG POST TASK -------- */
                            if(((uint32_t) Timing::millis()) - wifiStartTime > (45 * 1000)) { // additional watchdog in case task is not timing out by itself
                                printf("%d WIFI: TASK TIMEOUT triggered, check if blocks were transmitted or not!\n", ((uint32_t) Timing::millis()));
                                uint16_t successfullyTransmittedBlocksNew = 0;
                                uint16_t successfullyTransmittedHalfBlocksNew = 0; // don't look on half blocks
                                device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocksNew, successfullyTransmittedHalfBlocksNew);
                                if(successfullyTransmittedBlocksNew - successfullyTransmittedBlocks > 0) {
                                    printf("%d WIFI: %d blocks transfered, continue waiting!\n", ((uint32_t) Timing::millis()), (successfullyTransmittedBlocksNew - successfullyTransmittedBlocks));
                                    wifiStartTime = ((uint32_t) Timing::millis()); // reset timer
                                    successfullyTransmittedBlocks = successfullyTransmittedBlocksNew; // update last checked block value
                                }
                                else {
                                    printf("%d WIFI: NO blocks transfered, KILL TASK!\n", ((uint32_t) Timing::millis()));
                                    //device.killPOSTTask(); // brutal! don't do! leads to reset
                                    break; // only BREAK from loop, maybe some block transmitted
                                }
                            }
                        }
                        /* -------- POST TASK FINISHED -------- */
                        if(device.getWiFiPOSTCallStatus() != HTTP_POST_DATA_FINISHED_ALL_GOOD) { // might still be RUNNING! when watchdog kicked in (will then lead to client write error because of software connection abort)
                            printf("%d WIFI: POST was not successful: %d!\n", ((uint32_t) Timing::millis()), device.getWiFiPOSTCallStatus());
                        }
                        device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks);
                        
                        if(REST_SEND_FULL_BLOCKS) { printf("%d WIFI: successfully transmitted blocks: %d (half block transmission deactivated)\n", ((uint32_t) Timing::millis()), successfullyTransmittedBlocks); }
                        //heap_caps_free(dmaBuffer2048Bytes); // CAREFUL: task might still running if it supposed to be killed? -> DO NOT FREE MEMORY AT ALL
                        // also do not free memory of restPrefixPointer
                        /* -------- DELETING SUCCESSFULLY TRANSMITTED BLOCKS (ONLY FULLY TRANSMITTED) -------- */
                        // DO NOT DELETE BLOCKS FOR THIS TEST HERE!!!
                        /*
                        uint16_t flashBlockDeletedPointerBeforeDelete = flashBlockDeletedPointer; // remember old value for NVS update
                        for(uint16_t delBlocks=0; delBlocks<successfullyTransmittedBlocks; delBlocks++) { // deleting is based ONLY on fully transmitted blocks
                            if(!device.flash.fifoPopDelete(flashBlockDeletedPointer, flashPointer, MT29_NUMBER_PAGES, true)) { // delete block from flash and increment flashBlockDeletedPointer
                                printf("%d WIFI: FIFO POP DELETE FAILED!\n", ((uint32_t) Timing::millis()));
                                device.disconnectAndStopWiFi();
                                return somethingTransmitted; // error here means -> no pointers are updated, retransmitting maybe already deleted blocks, but very unlikely to happen
                            } 
                        }
                        */
                        /* -------- UPDATING NVS POINTER -------- */
                        // FULL BLOCK POINTER
                        //if(flashBlockDeletedPointerBeforeDelete != flashBlockDeletedPointer) { // some blocks are now fully transmitted
                            //printf("%d WIFI-NVS: updating flashBlockDeletedPointer (old: %d, new: %d)\n", ((uint32_t) Timing::millis()), flashBlockDeletedPointerBeforeDelete, flashBlockDeletedPointer);
                            //device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER, flashBlockDeletedPointer);
                            somethingTransmitted = true; // ONLY HERE, not when half blocks were transfered because blocks are not deleted
                        //}
                    }
                    else printf("%d WIFI: SCANNED, but not found\n", ((uint32_t) Timing::millis()));
                    device.disconnectAndStopWiFi(); // disconnect here because before that no wifi actions
                }
                /* -------- UPDATING SCANNING FREQUENCY AFTER A SCAN (OR DATATRANSMISSION!!!) WAS PERFORMED -------- */
            }
        }
        else { printf("%d WIFI: NO -> Blocks to transmit %d < %d\n", ((uint32_t) Timing::millis()), blocksToTransmit, minBlocksToTransmit); }
    }
    else { printf("%d WIFI: NO -> Battery too low\n", ((uint32_t) Timing::millis())); }
    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}


extern "C" void app_main() {
    while(1) {
        //device.setCPUSpeed(240);
        printf("HELLO WORLD\n");
        device.delay(4000);
        uint32_t flashPointer = 64 * 20; // 64 * 20
        uint64_t start = Timing::millis();
        device.flashPowerOn(true);
        if(!dataTransmissionWifi(true, 1, 0, 3750, flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE)) {
            printf("RESULT: FAIL\n");
        }
        start = Timing::millis() - start;
        uint64_t transmittedBytes = flashPointer;
        transmittedBytes *= 2048;
        double dataRate = transmittedBytes;
        dataRate = (dataRate / start) * 1000;
        printf("RESULT: %llu byte in %llu ms (%.01f byte/s)\n", transmittedBytes, start, dataRate);
        device.flashPowerOff(true);

        bootCnt++;
        device.deepSleep();
    }
}