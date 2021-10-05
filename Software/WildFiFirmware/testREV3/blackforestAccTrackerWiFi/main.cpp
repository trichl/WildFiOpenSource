#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

// POWER CONSUMPTION AVERAGE (NO WIFI): 433uA @80MHz (445uA @10MHz -> NVS Init takes 10 times longer) -> NEW! 160uA! (more with new LDO)
// @6.25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 1 hour (6.25 * 6 Byte = 37.5 Bytes per second)
// @25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 15min (25 * 6 Byte = 150 Bytes per second)
// @25Hz: memory full after 20 days, 503uA testrun (productive: 463uA (!!!)) average WITHOUT wifi data transmission, 1.34mA with WiFi transmission (sample 1 block -> transmit 1 block in 8s)
// 10 Blocks = 10 * 64 * 2048 = 1.310.720 (1.25MByte) in 70 seconds, avg. 80.9mA, 5.89mWh / 3.75V = 1.57mAh

// TEST: new state ST_MEMFULL! -> errorCnt was 1 once, don't know where

// TODO: problem: old LDO -> sometimes flash reads out first page or only 00000000000 or only FFFFFFFFF or first page, happens randomly! guess voltage drop
// TODO: problem: partial page programming should only be done with 512 bytes, not randomly

// TODO (LOW PRIO): new data with >= REV4B: hall sensor
// TODO (LOW PRIO): new data: sample one time MAG data?
// TODO (LOW PRIO): new data: LAST WIFI STATUS (last scan wifi found or not and number of wifis seen), or last TRANSMISSION AVERAGE TIME FOR ONE BLOCK, or last scan RSSI of wifi?
// TODO (LOW PRIO): 1MHz I2C should not work with RTC?!? -> dynamic I2C speed switching -> TEST i2c.changeClockSpeed(I2C_FREQ_HZ_400KHZ)
// TODO (LOW PRIO): switching between 10 and 80MHz or DFS?

// TODO: on DOG sometimes 2g exceeded!
// TODO: Antennenfuß unten zur MCU hin locker nach Hundetest
// TODO: TEST neue Software (sollte keine Duplikate mehr geben)
// TODO: CHECK IF FIFO >= 996 -> add error cnt

// TEST: instead of wake stub, get data after boot (maybe faster = less power?)

// TODO: ADD TIME SYNC ALGORITHM FROM ESP NOW PROX DETECTION

#define TAG_NAME                                "TagOnCola"                     // unique id for post calls

#define MODE_TESTRUN                            0                               // with debug output, not writing into flash memory, not incrementing NVS_FLASH_POINTER
#define MODE_PRODUCTIVE                         1                               // normal tracker mode, fully operational, flash memory should be empty before and NVS_FLASH_POINTER reset to 0
#define MODE_READFLASH                          2                               // reads out full flash (10.000 pages = 30min) and then goes to sleep
#define MODE_RESETALL                           3                               // deletes complete flash and NVS_FLASH_POINTER
#define MODE_SELFTEST                           4                               // runs selftest on flash memory
#define MODE_ACC_FOC                            5                               // ONLY ONE TIME when tag is FLAT on table (z-Axis = 1g), is stored into ACC NVM memory

/** --- CURRENT TRACKER MODE --- */
#define TRACKER_MODE                            MODE_TESTRUN
#define MOCK_FLASH_WRITES                       0                               // 1 = flash will not be written, but all pointers will be updated and data transmitted
#define MOCK_FLASH_DELETES                      0                               // 1 = flash will not be deleted after successfull data transmission, but all pointers are updated

acc_config_t accConfig = {
    BMX160_ACCEL_ODR_25HZ,                                                      // Acc frequency
    BMX160_ACCEL_BW_RES_AVG8,                                                   // Acc averaging cycles
    BMX160_ACCEL_RANGE_2G                                                       // Acc range (WARNING: changes meaning of LSB value in data)
};

#define BATT_MIN_VOLTAGE                        1000                            // 3550, hibernate below that voltage and
#define BATT_RESTART_VOLTAGE                    1500                            // 3650, wait until voltage above that threshold again
#define FIRST_UNDER_VOLTAGE_SLEEP_TIME          2*3600                          // first time UV detected -> stop imu fifo and sleep for that time
#define FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME      12*3600                         // second and more times UV detected -> sleep for that time
#define ACC_INTERRUPT_WATERMARK                 960                             // imu is generating interrupt when fifo is at that fill level
#define ACC_RAM_SIZE_1                          (ACC_INTERRUPT_WATERMARK + 12)  // ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2 = RTC memory for storing fifo data in wake stub
#define ACC_RAM_SIZE_2                          4                               // see above

#define WIFI_MIN_BATT_VOLTAGE                   1000                            // 3550, don't start scan if battery voltage too low
#define WIFI_MIN_BATT_VOLTAGE_DURING_TRANSM     1000                            // 3300, don't continue wifi transmission if voltage dropped below that threshold after one block transmission
#define WIFI_MIN_BLOCKS_TO_TRANSMIT             1                               // only start scan (and data transmission) if there is enough data to send
#define WIFI_MAX_BLOCKS_TO_TRANSMIT             10                              // during one connection, burst maximum this amount of blocks (keep in mind: 2 transmissions per block)
#define WIFI_MIN_TIME_BETWEEN_SCANS_SECONDS     60                              // 3600+1800 = 1,5hr, don't scan/transmit too frequently
#define WIFI_OUTPUT_POWER                       RADIO_MAX_TX_19_5_DBM           // 19.5dBm will brown out on REV3
#define WIFI_MAX_SCAN_TIME_SECONDS              5                               // scan should only take <1s
#define WIFI_MAX_CONNECT_TIME_SECONDS           8                               // connect should take <1s
#define WIFI_MAX_POST_TASK_TIME_SECONDS         45                              // max time for sending one successfully transmitted block

#define START_TIME_BETWEEN_ACTIVATION_SCANS     60                              // 900, if not activated: sleep for that time, wake up, check if wifi there and try to activate, 120 = 234uA average
#define ST_MEMFULL_WIFI_SCAN_INTERVAL_SECONDS   3600                            // if fifo memory is full -> go into special state and try to transmit the data every X seconds

const uint8_t KNOWN_WIFI_LIST_SIZE = 2; // IMPORTANT: DO NOT FORGET TO CHANGE THAT!!!!!!!!!
const char* WIFI_SSIDS[KNOWN_WIFI_LIST_SIZE] = { "guest", "Tracker1" };                      // left most = highest priority
const char* WIFI_PASSWORDS[KNOWN_WIFI_LIST_SIZE] = { "xxxxxxx", "Tracker1" };  // left most = highest priority
#define WIFI_SCAN_CHANNEL                       6                               // wifi channel for data transmission, only work on ONE wifi channel! keep in mind!

#define START_WIFI_SCAN_CHANNEL                 6                               // wifi scan channel for activation/timestamp
const char* START_WIFI_SSID = "LiWoAb New";                                     // wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* START_WIFI_PASSWORD = "xxxxxx";                                // wifi password for activation/timestamp

const char* REST_URL = RESTDB_ENDPOINT2;
const char* REST_CONTENT_TYPE = "application/json";
const char* REST_ADD_HEADER_KEY = "x-apikey";
const char* REST_ADD_HEADER_VALUE = RESTDB_APIKEY1;
#define REST_PAYLOAD_PREFIX_1                           "[{\"tag\":\""
#define REST_PAYLOAD_PREFIX_2                           "\",\"data\":\""
const char* REST_PAYLOAD_PREFIX = REST_PAYLOAD_PREFIX_1 TAG_NAME REST_PAYLOAD_PREFIX_2;
const char* REST_PAYLOAD_POSTFIX = "\"}]";
post_task_stream_flash_parameters_t restStreamParams;
uint8_t *dmaBuffer2048Bytes = NULL;

// NVS storage names
#define NVS_FLASH_WRITE_POINTER                         "flashpnt"              // data writing: pointing on current flash page
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffsetpnt"        // data writing: pointing on current byte in flash page
#define NVS_FLASH_SEND_NEXT_BLOCK_POINTER               "blockpntsent"          // data transmission: pointing on next block to transmit
#define NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER          "halfblocksent"         // data transmission: sometimes only half a block transmitted -> save and do not retransmit
#define NVS_FLASH_TAG_ACTIVATED_BY_WIFI                 "activated"             // tag has seen activation wifi

// state enums
typedef enum { ST_FIRST_START_HARD_RESET = 0, ST_WAIT_FOR_ACTIVATION = 1, ST_START = 2, ST_TRACK = 3, ST_PWRDWN = 4, ST_MEMFULL = 5 } tracker_state_t;

// RTC variables
RTC_DATA_ATTR uint32_t lastTimeWifi = 0; // REMEMBER: will be reset after hard reset due to BROWNOUT
RTC_DATA_ATTR uint16_t fifoLenRam = 0;
RTC_DATA_ATTR uint8_t fifoDataRam[ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2] = { 0 };
RTC_DATA_ATTR uint16_t fifoDataPointerRam = 0;
RTC_DATA_ATTR bool fifoStopped = false;

RTC_DATA_ATTR tracker_state_t state = ST_FIRST_START_HARD_RESET;
RTC_DATA_ATTR uint32_t startCnt = 0;
RTC_DATA_ATTR uint8_t lastErrorId = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;

RTC_DATA_ATTR bool isActivated = false;
RTC_DATA_ATTR bool hasValidTimestamp = false;

// global variables (non RTC)
bool error = false;
uint64_t t = 0;
uint8_t fifoDataNewest[1024] = { 0 };
uint8_t *sensorData = NULL; 
uint16_t sensorDataPointer = 0;

void addData4(uint32_t d) {
    if(sensorData == NULL) { return; }
    sensorData[sensorDataPointer] = d >> 24; sensorData[sensorDataPointer+1] = d >> 16; sensorData[sensorDataPointer+2] = d >>  8; sensorData[sensorDataPointer+3] = d; sensorDataPointer += 4;
}

void addData2Signed(int16_t d) { // WARNING: signed!
    if(sensorData == NULL) { return; }
    sensorData[sensorDataPointer] = d >> 8; sensorData[sensorDataPointer+1] = d; sensorDataPointer += 2;
}

void addData2(uint16_t d) { // WARNING: signed!
    if(sensorData == NULL) { return; }
    sensorData[sensorDataPointer] = d >> 8; sensorData[sensorDataPointer+1] = d; sensorDataPointer += 2;
}

void addData1(uint8_t d) {
    if(sensorData == NULL) { return; }
    sensorData[sensorDataPointer] = d; sensorDataPointer++;
}

void writeTest() {
    esp_task_wdt_init(120, false);
    device.delay(8000);
    if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
    if(!device.flash.erase(0)) { printf("ERROR FLASH DELETE\n"); }
    uint8_t *testData = NULL;
    if(!device.flash.createBuffer(&testData, MT29_CACHE_SIZE)) { printf("ERROR BUFF\n"); }
    for(uint16_t page = 0; page < 64; page++) {
        for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) { testData[i] = page; }
        if(!device.flash.write(page, testData, MT29_CACHE_SIZE)) { printf("ERROR\n"); }
    }
    device.flash.printFlash(32, 4, 20);
    if(!device.flashPowerOff()) { printf("ERROR FLASH OFF\n"); }
}

void accFoc() {
    device.sensorPowerOn(true);
    device.delay(10000);
    acc_config_t accConfigForFOC = { BMX160_ACCEL_ODR_25HZ, BMX160_ACCEL_BW_RES_AVG8, BMX160_ACCEL_RANGE_2G };
    if(!device.imu.start(&accConfigForFOC, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("ERROR1\n"); }
    else {
        if(device.imu.accFOCAlreadyDoneAndStoredInNVM()) {
            printf("NOTHING TO DO -> ALREADY CALIBRATED!\n");
        }
        else {
            if(!device.imu.startAccFOCAndStoreInNVM()) {
                printf("FATAL ERROR DURING FOC!!!!\n");
            }
            else {
                printf("FOC DONE AND VALUES STORED IN NVM!\n");
            }
        }
    }
    device.delay(60000);
    device.enableInternalTimerInterruptInDeepSleep(3000);
    device.deepSleep();
}

void selftest() {
    esp_task_wdt_init(120, false);
    printf("V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub());
    device.disableWakeStubNoBootIfVoltageLow();

    if(!device.initDataNVS()) { printf("ERROR NVS\n"); }
    uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
    printf("FLASH POINTER: %d\n", flashPointer);
    uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
    printf("FLASH OFFSET POINTER: %d\n", flashOffsetPointer);

    device.delay(8000);

    if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
    if(!device.flash.selfTestBadBlocks()) { printf("FOUND BAD BLOCKS!\n"); }

    uint8_t *testData = NULL;
    if(!device.flash.createBuffer(&testData, MT29_CACHE_SIZE)) { printf("ERROR BUFF\n"); }
    for(uint16_t i=0; i<200; i++) {
        testData[i] = i;
    }
    if(!device.flash.write(0, testData, 200)) { printf("ERROR\n"); }
    if(!device.flash.write(2, testData, 200)) { printf("ERROR\n"); }
    device.flash.printFlash(0, 4, 300);
    if(!device.flash.erase(0)) { printf("ERROR FLASH DELETE\n"); }
    device.flash.printFlash(0, 10, 300);
    
    if(!device.flashPowerOff()) { printf("ERROR FLASH OFF\n"); }

    device.delay(60000);

    device.enableInternalTimerInterruptInDeepSleep(3000);
    device.deepSleep();
}

void resetAll() {
    esp_task_wdt_init(120, false); // set task watchdog to 120 seconds
    printf("V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub());
    device.disableWakeStubNoBootIfVoltageLow();

    // reset flash pointer
    if(!device.resetDataNVS()) { printf("NVS ERASE ERROR\n"); }
    if(!device.initDataNVS()) { printf("ERROR NVS INIT\n"); }
    
    uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
    printf("NVS_FLASH_WRITE_POINTER AFTER DELETE: %d\n", flashPointer);
    uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
    printf("NVS_FLASH_WRITE_PAGE_OFFSET_POINTER AFTER DELETE: %d\n", flashOffsetPointer);
    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER);
    printf("NVS_FLASH_SEND_NEXT_BLOCK_POINTER AFTER DELETE: %d\n", flashBlockToSendNextPointer);
    uint16_t flashHalfBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER);
    printf("NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER AFTER DELETE: %d\n", flashHalfBlockToSendNextPointer);
    uint32_t activated = device.nvsReadUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI);
    printf("NVS_FLASH_TAG_ACTIVATED_BY_WIFI AFTER DELETE: %d\n", activated);

    device.delay(8000); // wait long so that serial monitor reset does not happen while already deleting the flash

    // delete flash
    if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
    if(!device.flash.fullErase()) { printf("FULL ERASE ERROR\n"); }
    //if(!device.flash.erase(0)) { printf("ERROR FLASH DELETE BLOCK 0\n"); }
    device.flash.printFlash(0, 5, 20); // read first 5 pages, should be all FF
    if(!device.flashPowerOff()) { printf("ERROR FLASH OFF\n"); }

    device.delay(60000);
    device.enableInternalTimerInterruptInDeepSleep(3000);
    device.deepSleep();    
}

void readFullFlash() {
    esp_task_wdt_init(120, false); // set task watchdog timeout to 120 seconds
    const uint32_t READ_PAGES = 42058; // 42058
    if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
    device.delay(8000);
    device.initDataNVS();
    printf("FLASH POINTER: %d\n",  device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER));
    printf("FLASH OFFSET POINTER: %d\n", device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER));
    if(!device.flash.printFlash(0, READ_PAGES, MT29_CACHE_SIZE, true)) { printf("ERROR FLASH2\n"); }
    if(!device.flashPowerOff()) { printf("ERROR FLASH3\n"); }
    device.delay(60000);
    device.enableInternalTimerInterruptInDeepSleep(6000);
    device.deepSleep();
}

bool dataTransmissionWifi(bool forceMode, uint32_t currentTimestamp, uint32_t batteryVoltage, uint32_t flashPointer) { // perform a wifi scan (366ms scan only, 1515ms in total if connecting)
    uint32_t wifiStartTime = ((uint32_t) Arduino::millisWrapper());
    bool somethingTransmitted = false; // at least ONE block deleted (so more space in memory than before)

    /* -------- GET POINTER VALUES FROM NVS -------- */
    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER);
    uint16_t flashHalfBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER);

    /* -------- GET NUMBER OF BLOCKS TO TRANSMIT -------- */
    uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockToSendNextPointer, flashPointer);

    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: lastScan %d, blocksToTransmit %d, batteryV %d\n", ((uint32_t) Arduino::millisWrapper()), (currentTimestamp-lastTimeWifi), blocksToTransmit, batteryVoltage); }
    if(batteryVoltage > WIFI_MIN_BATT_VOLTAGE) { // only if enough juice in battery
        if(blocksToTransmit >= WIFI_MIN_BLOCKS_TO_TRANSMIT) { // only if enough data to transmit
            if(forceMode || (currentTimestamp - lastTimeWifi > WIFI_MIN_TIME_BETWEEN_SCANS_SECONDS)) { // only if last scan at least x seconds ago
                if((TRACKER_MODE == MODE_TESTRUN) && (forceMode)) { printf("%d WIFI: FORCING SCAN!\n", ((uint32_t) Arduino::millisWrapper())); }
                lastTimeWifi = currentTimestamp; // scan was performed -> update -> WARNING, WILL GET LOST AFTER HARD RESET!
                /* -------- NEW SCAN ALLOWED -------- */
                if(!device.initWiFi()) { lastErrorId = 1; errorCnt++; return somethingTransmitted; }
                if(!device.scanForWiFis(false, WIFI_OUTPUT_POWER, 120, WIFI_SCAN_CHANNEL)) { lastErrorId = 2; errorCnt++; return somethingTransmitted; }
                while(!device.wiFiScanCompleted()) {
                    device.delay(20);
                    if(((uint32_t) Arduino::millisWrapper()) - wifiStartTime > (WIFI_MAX_SCAN_TIME_SECONDS * 1000)) {
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: TIMEOUT SCAN!\n", ((uint32_t) Arduino::millisWrapper())); }
                        device.disconnectAndStopWiFi();
                        lastErrorId = 3; errorCnt++;
                        return somethingTransmitted;
                    }
                }
                /* -------- SCAN FINISHED -------- */
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: SCAN TOOK %dms (MAX %d)!\n", ((uint32_t) Arduino::millisWrapper()), ((uint32_t) Arduino::millisWrapper()) - wifiStartTime, (WIFI_MAX_SCAN_TIME_SECONDS * 1000)); }
                wifiStartTime = ((uint32_t) Arduino::millisWrapper()); // reset timer
                uint8_t knownWifiFoundPointer = 0; // pointing to id in WIFI_SSIDS array
                if(device.wiFiScanIncludesArray(WIFI_SSIDS, KNOWN_WIFI_LIST_SIZE, knownWifiFoundPointer)) {
                    /* -------- MY WIFI FOUND -------- */
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: FOUND NO %d!\n", ((uint32_t) Arduino::millisWrapper()), knownWifiFoundPointer); }
                    if(!device.connectToWiFiAfterScan(WIFI_SSIDS[knownWifiFoundPointer], WIFI_PASSWORDS[knownWifiFoundPointer], WIFI_SCAN_CHANNEL)) { lastErrorId = 45; errorCnt++; }
                    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                        device.delay(20);
                        if(((uint32_t) Arduino::millisWrapper()) - wifiStartTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: TIMEOUT CONNECT!\n", ((uint32_t) Arduino::millisWrapper())); }
                            device.disconnectAndStopWiFi();
                            lastErrorId = 4; errorCnt++;
                            return somethingTransmitted;
                        }
                    }
                    if((device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: ACCESS POINT NOT FOUND OR OTHER ISSUE!\n", ((uint32_t) Arduino::millisWrapper())); }
                        device.disconnectAndStopWiFi();
                        lastErrorId = 5; errorCnt++;
                        return somethingTransmitted;
                    }
                    /* -------- CONNECTED TO WIFI -------- */
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: CONNECT TOOK %dms (MAX %d)!\n", ((uint32_t) Arduino::millisWrapper()), ((uint32_t) Arduino::millisWrapper()) - wifiStartTime, (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)); }
                    /* -------- FLASH OPERATION -------- */
                    if(!device.flash.createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
		                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: FLASH DMA RESERVE ERROR!\n", ((uint32_t) Arduino::millisWrapper())); }
                        device.disconnectAndStopWiFi();
                        lastErrorId = 8; errorCnt++;
                        return somethingTransmitted;
                    }
                    /* -------- DMA BUFFER FOR FLASH CREATED -------- */
                    uint16_t blocksToActuallyTransmit = blocksToTransmit;
                    if(blocksToActuallyTransmit > WIFI_MAX_BLOCKS_TO_TRANSMIT) { // for REST POST routine: limit maximum number of blocks to transmit (will be less if voltage low or errors)
                        blocksToActuallyTransmit = WIFI_MAX_BLOCKS_TO_TRANSMIT;
                    }
                    restStreamParams.url = REST_URL;
                    restStreamParams.contentType = REST_CONTENT_TYPE;
                    restStreamParams.additionalHeaderKey = REST_ADD_HEADER_KEY;
                    restStreamParams.additionalHeaderValue = REST_ADD_HEADER_VALUE;
                    restStreamParams.prefix = REST_PAYLOAD_PREFIX;
                    restStreamParams.dataDMA2048Bytes = &dmaBuffer2048Bytes[0];
                    restStreamParams.postfix = REST_PAYLOAD_POSTFIX;
                    restStreamParams.flashObject = &device.flash;
                    restStreamParams.flashBlockToSendNextPointer = flashBlockToSendNextPointer;
                    restStreamParams.flashHalfBlockToSendNextPointer = flashHalfBlockToSendNextPointer;
                    restStreamParams.flashMaxNumberOfBlocksToTransmit = blocksToActuallyTransmit;
                    restStreamParams.deviceObject = &device;
                    restStreamParams.minBatteryVoltageToContinue = WIFI_MIN_BATT_VOLTAGE_DURING_TRANSM;
                    restStreamParams.debug = (TRACKER_MODE == MODE_TESTRUN);
                    /* -------- START POST TASK -------- */
                    uint16_t successfullyTransmittedBlocks = 0;
                    uint16_t successfullyTransmittedHalfBlocks = 0;
                    wifiStartTime = ((uint32_t) Arduino::millisWrapper()); // reset watchdog timer
                    device.doWiFiPOSTStreamCallFlash(&restStreamParams, 8192);
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
                        lastErrorId = 56 + device.getWiFiPOSTCallStatus(); // 19 error codes -> 56 + 19 = 75, next errorid should be 76
                        errorCnt++;
                    }
                    device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks);
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: successfully transmitted blocks: %d, half blocks: %d\n", ((uint32_t) Arduino::millisWrapper()), successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks); }
                    //heap_caps_free(dmaBuffer2048Bytes); // CAREFUL: task might still running if it supposed to be killed? -> DO NOT FREE MEMORY AT ALL
                    /* -------- DELETING SUCCESSFULLY TRANSMITTED BLOCKS (ONLY FULLY TRANSMITTED) -------- */
                    uint16_t flashBlockToSendNextPointerBeforeDelete = flashBlockToSendNextPointer; // remember old value for NVS update
                    for(uint16_t delBlocks=0; delBlocks<successfullyTransmittedBlocks; delBlocks++) { // deleting is based ONLY on fully transmitted blocks
                        if(!device.flash.fifoPopDelete(flashBlockToSendNextPointer, flashPointer, MT29_NUMBER_PAGES, MOCK_FLASH_DELETES)) { // delete block from flash and increment flashBlockToSendNextPointer
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: FIFO POP DELETE FAILED!\n", ((uint32_t) Arduino::millisWrapper())); }
                            device.disconnectAndStopWiFi();
                            lastErrorId = 10; errorCnt++;
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
                else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: SCANNED, but not found\n", ((uint32_t) Arduino::millisWrapper())); }
                device.disconnectAndStopWiFi(); // disconnect here because before that no wifi actions
            }
            else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> recently connected (%ds ago < %d)\n", ((uint32_t) Arduino::millisWrapper()), currentTimestamp - lastTimeWifi, WIFI_MIN_TIME_BETWEEN_SCANS_SECONDS); }
        }
        else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> Blocks to transmit (%d) too low\n", ((uint32_t) Arduino::millisWrapper()), blocksToTransmit); }
    }
    else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> Battery too low\n", ((uint32_t) Arduino::millisWrapper())); }

    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}

RTC_DATA_ATTR bool wakeStub() {
    if(fifoStopped) { // in certain states -> power ESP32 directly
        return true;
    }
    else {
        beginSw(); // start i2c
        fifoLenRam = getAccFIFOLength();
        if((fifoDataPointerRam + fifoLenRam) > (ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2)) { // storing fifo would exceed ACC array in RTC memory
            return true; // FIFO FULL, start the ESP32
        }
        readAccFIFO(fifoDataRam+fifoDataPointerRam, fifoLenRam, false);
        fifoDataPointerRam += fifoLenRam;
        return false;
    }
}

extern "C" void app_main() {
    while(1) {
        //device.setCPUSpeed(ESP32_10MHZ); // CPU runs @80MHz is more energy efficient than @10MHz (about 5%) if NVS is used
        if(TRACKER_MODE == MODE_TESTRUN) { printf("WakeStub: FIFO STOPPED: %d\n", fifoStopped); }
        if(TRACKER_MODE != MODE_PRODUCTIVE) {
            if(TRACKER_MODE == MODE_READFLASH) { printf("Mode: READ FLASH\n"); readFullFlash(); }
            else if(TRACKER_MODE == MODE_RESETALL) { printf("Mode: RESET ALL\n"); resetAll(); }
            else if(TRACKER_MODE == MODE_SELFTEST) { printf("Mode: SELFTEST\n"); selftest(); }
            else if(TRACKER_MODE == MODE_TESTRUN) { printf("Mode: TESTRUN\n"); }
            else if(TRACKER_MODE == MODE_ACC_FOC) { printf("Mode: ACC FOC -> DEVICE FLAT ON TABLE???\n"); accFoc(); }
        }
        /** ---------------- VERY FIRST ENTRY POINT AFTER HARD RESET (RTC memory = reset), only executed ONCE, NEVER after Deepsleep! ---------------- */
        if(state == ST_FIRST_START_HARD_RESET) { // custom wake stub not running -> no fifo
            if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: HARD RESET!\n"); }
            // disable no boot if voltage low (do here)
            device.disableWakeStubNoBootIfVoltageLow();

            // IMU and BARO powered on for lower deep sleep current
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            device.delay(100);

            // RTC: disable clock out (lower deep sleep current)
            if(!device.rtc.disableClockOut()) { lastErrorId = 79; errorCnt++; }
            device.delay(5);

            // RTC: get current timestamp
            uint32_t timestamp = device.rtc.getTimestamp(error);
            if(error) { lastErrorId = 54; errorCnt++; }

            // check NVS if already activated & timestamp is valid
            if(!device.initDataNVS()) { lastErrorId = 52; errorCnt++; }
            uint16_t activated = device.nvsReadUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI);
            if(device.getLastResetReason() == ESP_RST_BROWNOUT) { lastErrorId = 76; errorCnt++; }
            isActivated = (activated > 0); // store state in RTC
            hasValidTimestamp = (timestamp > 1600000000); // store state in RTC, 13.09.2020 12:26
            if(isActivated && hasValidTimestamp) { // can happen when browning out during data transmission or other kind of error reset
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp valid and activated -> START!\n"); }
                lastTimeWifi = timestamp; // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
                state = ST_START;
            }
            else {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: NVS not activated (%d) or timestamp lost (%d) -> WAIT FOR ACTIVATION!\n", isActivated, hasValidTimestamp); }
                state = ST_WAIT_FOR_ACTIVATION;
            }
            //device.blink(B_RED, B_RED, B_RED); // after plugging to power supply -> blink red
            device.enableInternalTimerInterruptInDeepSleep(15); // restart system
        }
        /** ---------------- WAIT FOR ACTIVATION STATE ---------------- */
        else if(state == ST_WAIT_FOR_ACTIVATION) { // custom wake stub not running -> no fifo
            bool wifiError = false;
            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: NOT ACTIVATED or timestamp lost!\n"); }
            if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, go to power down state
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: Undervoltage (%d)! Don't scan!\n", device.readSupplyVoltageFromWakeStub()); }
                device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
            }
            else {
                if(device.initWiFi()) {
                    if(device.scanForWiFis(false, RADIO_MAX_TX_13_DBM, 120, START_WIFI_SCAN_CHANNEL)) {
                        uint32_t wifiStartTime = ((uint32_t) Arduino::millisWrapper());
                        while(!device.wiFiScanCompleted()) {
                            device.delay(20);
                            if(((uint32_t) Arduino::millisWrapper()) - wifiStartTime > (WIFI_MAX_SCAN_TIME_SECONDS * 1000)) {
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: TIMEOUT SCAN!\n"); }
                                device.disconnectAndStopWiFi();
                                lastErrorId = 49; errorCnt++;
                                wifiError = true;
                            }
                        }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: SCAN TOOK %dms (MAX %d)!\n", ((uint32_t) Arduino::millisWrapper()) - wifiStartTime, (WIFI_MAX_SCAN_TIME_SECONDS * 1000)); }
                        if((!wifiError) && device.wiFiScanIncludes(START_WIFI_SSID)) {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: activation wifi found, isActivated: %d, hasValidTimestamp: %d\n", isActivated, hasValidTimestamp); }
                            if(!isActivated) { // not yet activated
                               if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: wasn't activated -> activate now!\n"); }
                                if(!device.initDataNVS()) { lastErrorId = 51; errorCnt++; }
                                else {
                                    if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 1)) { lastErrorId = 53; errorCnt++; } // write activation into NVS
                                    else {
                                        isActivated = true; // now is activated!
                                    }
                                }   
                            }
                            if(!hasValidTimestamp) { // no valid timestamp (full power down -> need to connect and get time)
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: timestamp not okay -> try to connect and get time\n"); }
                                if(!device.connectToWiFiAfterScan(START_WIFI_SSID, START_WIFI_PASSWORD, START_WIFI_SCAN_CHANNEL)) { lastErrorId = 55; errorCnt++; }
                                else {
                                    uint32_t wifiStartTime = ((uint32_t) Arduino::millisWrapper());
                                    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                                        device.delay(20);
                                        if(((uint32_t) Arduino::millisWrapper()) - wifiStartTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: TIMEOUT CONNECT!\n"); }
                                            wifiError = true;
                                            break;
                                        }
                                    }
                                    if(wifiError || (device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: ACCESS POINT NOT FOUND OR OTHER ISSUE!\n"); }
                                        device.disconnectAndStopWiFi();
                                        lastErrorId = 56; errorCnt++;
                                    }
                                    else { // connected to wifi
                                        uint32_t timestampUTC = 0;
                                        uint16_t millisecondsUTC = 0;
                                        if(!device.getNTPTimestampUTC(timestampUTC, millisecondsUTC, 7000, "pool.ntp.org")) { // will block
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: UTC get time error!\n"); }
                                        }
                                        else {
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: UTC milliseconds: %d\n", millisecondsUTC); }
                                            if(timestampUTC > 1600000000) { // looks like a valid timestamp from NTP
                                                i2c.begin(I2C_FREQ_HZ_400KHZ);
                                                if(millisecondsUTC > 1000) { millisecondsUTC = 1000; lastErrorId = 78; errorCnt++; }
                                                device.delay(1000 - millisecondsUTC); // wait milliseconds until next second is full
                                                if(!device.rtc.setTimestamp(timestampUTC+1)) { lastErrorId = 50; errorCnt++; }
                                                else {
                                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: UTC success, SET TO: %d!\n", timestampUTC+1); }
                                                    hasValidTimestamp = true;
                                                } 
                                            }
                                        }
                                        device.disconnectAndStopWiFi();
                                    }
                                }
                            }
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: isActivated: %d, hasValidTimestamp: %d\n", isActivated, hasValidTimestamp); }
                            if(isActivated && hasValidTimestamp) {
                                state = ST_START;
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: all done -> move to START!\n"); }
                                device.enableInternalTimerInterruptInDeepSleep(1); // restart in 1 second and start
                            }
                            else {
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: wifi was seen, but still something missing -> SLEEP\n"); }
                                device.enableInternalTimerInterruptInDeepSleep(START_TIME_BETWEEN_ACTIVATION_SCANS);
                            }
                        }
                        else {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: activation wifi NOT found -> SLEEP\n"); }
                            device.enableInternalTimerInterruptInDeepSleep(START_TIME_BETWEEN_ACTIVATION_SCANS); // sleep some time and try again after some time
                        }                                            
                    }
                    else { lastErrorId = 48; errorCnt++; device.enableInternalTimerInterruptInDeepSleep(START_TIME_BETWEEN_ACTIVATION_SCANS); }
                }
                else {lastErrorId = 47; errorCnt++; device.enableInternalTimerInterruptInDeepSleep(START_TIME_BETWEEN_ACTIVATION_SCANS); }
            }
        }
        /** ---------------- START STATE ---------------- */
        else if(state == ST_START) {
            // init BMX
            i2c.begin(I2C_FREQ_HZ_1MHZ);
            if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 12; errorCnt++; }
            if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 14; errorCnt++; }
            if(!device.imu.initFIFOForAcc()) { lastErrorId = 15; errorCnt++; }

            // print flash pointer (also in productive)
            if(!device.initDataNVS()) { lastErrorId = 17; errorCnt++; }
            uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
            uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
            
            printf("Start: FLASH POINTER: %d, FLASH OFFSET POINTER: %d\n", flashPointer, flashOffsetPointer);
            printf("Start: FREE HEAP BYTES: %d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));

            // custom wake stub function
            device.customWakeStubFunction(wakeStub);
            device.setWakeStubRejectionInterruptSrc(USE_EXT1_IF_WAKE_UP_REJECTED);

            // move to next state
            device.blink(B_GREEN, B_GREEN, B_GREEN);
            state = ST_TRACK;
            device.enableAccInterruptInDeepSleep();
        }
        /** ---------------- POWER DOWN STATE ---------------- */
        else if(state == ST_PWRDWN) {
            if(TRACKER_MODE == MODE_TESTRUN) { printf("(PWR_DWN) V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
            if(device.readSupplyVoltageFromWakeStub() <= BATT_RESTART_VOLTAGE) { // voltage still too low for restarting
                device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time 
            }
            else { // voltage okay again, restart into tracking state, restart IMU
                if(TRACKER_MODE == MODE_TESTRUN) { printf("RESTART!\n"); }
                i2c.begin(I2C_FREQ_HZ_1MHZ);
                if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 18; }
                if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 20; errorCnt++; }
                if(!device.imu.initFIFOForAcc()) { lastErrorId = 21; errorCnt++; }
                if(!device.imu.resetFIFO()) { lastErrorId = 22; errorCnt++; } // empty fifo, do not read it
                fifoDataPointerRam = 0; // reset RAM data
                fifoStopped = false;
                state = ST_TRACK;
                device.enableAccInterruptInDeepSleep(); // wake up from acc interrupt
            }
        }
        /** ---------------- MEMORY FULL STATE ---------------- */
        else if(state == ST_MEMFULL) {
            if(TRACKER_MODE == MODE_TESTRUN) { printf("State: (MEM_FULL) V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
            if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, go to power down state
                if(TRACKER_MODE == MODE_TESTRUN) { printf("State: (MEM_FULL) undervoltage during MEM_FULL state -> go to power down\n"); }
                state = ST_PWRDWN; // GO TO POWER DOWN, BUT KEEP IN MIND THAT AFTER BATT VOLTAGE OKAY AGAIN -> MOVE TO ST_TRACK although memory still full (will go to this state again)
                device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
            }
            else { // voltage okay again, then just try to get rid of the data
                if(TRACKER_MODE == MODE_TESTRUN) { printf("State: MEMORY FULL -> TRYING TO FIND WIFI!\n"); }
                // get rtc timestamp (1ms)
                i2c.begin(I2C_FREQ_HZ_400KHZ);
                uint32_t timestamp = device.rtc.getTimestamp(error);
                if(error) { lastErrorId = 23; errorCnt++; }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("State: TIMESTAMP: %d\n", timestamp); }

                // turn on flash power already (5ms)
                if(!device.flashPowerOn(false)) { lastErrorId = 24; errorCnt++; }
                    
                // get pointers from NVS (15ms)
                if(!device.initDataNVS()) { lastErrorId = 25; errorCnt++; }
                uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: flashPointer: %d, flashOffsetPointer: %d\n", flashPointer, flashOffsetPointer); }

                // check free space in memory
                if(TRACKER_MODE == MODE_TESTRUN) {
                    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be called again in dataTransmission, but not important
                    uint32_t freeSpaceBytesInFIFO = device.flash.fifoGetFreeSpace(flashBlockToSendNextPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES);
                    printf("Flash: FIFO free space in memory: %d\n", freeSpaceBytesInFIFO);
                }
                
                // wifi data transmission (FORCE mode!)
                uint32_t timeNow = ((uint32_t) Arduino::millisWrapper());
                bool somethingTransmitted = dataTransmissionWifi(true, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer);
                if(TRACKER_MODE == MODE_TESTRUN) { printf("WIFI needed %dms\n", ((uint32_t) Arduino::millisWrapper()) - timeNow); }

                // turn off flash
                if(!device.flashPowerOff()) { lastErrorId = 26; errorCnt++; } 

                // check if some data was transmitted in dataTransmissionWifi
                if(TRACKER_MODE == MODE_TESTRUN) { printf("State: something transmitted: %d\n", somethingTransmitted); }
                if(somethingTransmitted) { // at least one block or half a block was transmitted -> enable IMU again, go into tracking state again
                    if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 27; errorCnt++; } // init IMU again
                    if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 29; errorCnt++; }
                    if(!device.imu.initFIFOForAcc()) { lastErrorId = 30; errorCnt++; }
                    if(!device.imu.resetFIFO()) { lastErrorId = 31; errorCnt++; } // empty fifo, do not read it
                    fifoDataPointerRam = 0; // reset RAM data
                    fifoStopped = false;
                    state = ST_TRACK;
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n----- SLEEP -----\n", lastErrorId, errorCnt); }
                    device.enableAccInterruptInDeepSleep(); // wake up from acc interrupt
                }
                else { // nothing transmitted, stay in this state
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n----- SLEEP -----\n", lastErrorId, errorCnt); }
                    device.enableInternalTimerInterruptInDeepSleep(ST_MEMFULL_WIFI_SCAN_INTERVAL_SECONDS); // sleep some time before trying again transmission
                }
            }
        }
        /** ---------------- TRACKING STATE ---------------- */
        else if(state == ST_TRACK) {
            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: WAKE NUMBER: %d, V_BATT_WakeStub: %d\n", ((uint32_t) Arduino::millisWrapper()), startCnt, device.readSupplyVoltageFromWakeStub()); }
            // I2C start (1ms)
            i2c.begin(I2C_FREQ_HZ_1MHZ);
            if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: POWER DOWN\n", ((uint32_t) Arduino::millisWrapper())); }
                state = ST_PWRDWN;
                if(!device.imu.stop()) { lastErrorId = 32; errorCnt++; } // turn off imu, WARNING: will not fully turn of MAG if turned on before
                fifoStopped = true;
                device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
            }
            else {
                // reserve some memory for flash (0ms)
                if(!device.flash.createBuffer(&sensorData, 100)) { lastErrorId = 33; errorCnt++; }

                // start bme measurement (7ms)
                bool bmeOk = false;
                int16_t temperature = 0;
                int16_t temperatureBmx = 0;
                uint32_t pressure = 0;
                uint32_t humidity = 0;
                if(device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) {
                    if(device.baro.performMeasurement()) {
                        bmeOk = true;
                    }
                }

                // get fifo data (101ms -> NEW: 15ms)
                t = Arduino::millisWrapper();
                uint16_t fifoDataNewestLen = device.imu.readAccFIFOInOneGo(fifoDataNewest);
                if(TRACKER_MODE == MODE_TESTRUN) {
                    printf("%d FIFO: RAM FIFO DATA POINTER: %d, LAST FIFO LEN: %d\n", ((uint32_t) Arduino::millisWrapper()), fifoDataPointerRam, fifoLenRam);
                    printf("%d FIFO: FIFO READ TOOK: %lld ms\n", ((uint32_t) Arduino::millisWrapper()), (Arduino::millisWrapper() - t));
                    printf("%d FIFO: READOUT LENGTH: %d\n", ((uint32_t) Arduino::millisWrapper()), fifoDataNewestLen);
                    device.delay(1);
                    uint16_t fifoLenNew = device.imu.getFIFOLength();
                    printf("%d FIFO: BMX FIFO AFTER READ: %d\n", ((uint32_t) Arduino::millisWrapper()), fifoLenNew);
                }

                // get bmx temperature
                device.imu.getTemperature(temperatureBmx);

                // get rtc timestamp (1ms)
                uint32_t timestamp = device.rtc.getTimestamp(error);
                if(error) { lastErrorId = 34; errorCnt++; }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Data: TIMESTAMP: %d\n", ((uint32_t) Arduino::millisWrapper()), timestamp); }

                // get bme data (1ms)
                if(bmeOk) {
                    if(device.baro.getResults()) {
                        temperature = device.baro.getTemperature(error);
                        if(error) { lastErrorId = 35; errorCnt++; }
                        pressure = device.baro.getPressure(error);
                        if(error) { lastErrorId = 36; errorCnt++; }
                        humidity = device.baro.getHumidity(error);
                        if(error) { lastErrorId = 37; errorCnt++; }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Data: TEMP: %d, TEMP BMX: %d, PRESS: %d, HUMI: %d\n", ((uint32_t) Arduino::millisWrapper()), temperature, temperatureBmx, pressure, humidity); }
                    }
                }
                else { lastErrorId = 38; errorCnt++; }

                // add rest of data, already to DMA memory
                addData4(timestamp);
                addData4(startCnt);
                addData4(device.readSupplyVoltageFromWakeStub());
                addData2Signed(temperature);
                addData4(humidity);
                addData4(pressure);
                addData2Signed(temperatureBmx);
                addData2(fifoDataNewestLen+fifoDataPointerRam);
                addData1(lastErrorId);
                addData2(errorCnt);

                // print data
                if(TRACKER_MODE == MODE_TESTRUN) {
                    printf("%d Data Header: ", ((uint32_t) Arduino::millisWrapper()));
                    for(uint16_t i=0; i<sensorDataPointer; i++) { printf("%02X ", sensorData[i]); }
                    /*printf("\nData Newest: ");
                    for(uint16_t i=0; i<fifoDataNewestLen; i++) { printf("%02X ", fifoDataNewest[i]); }
                    printf("\nData RAM: ");
                    for(uint16_t i=0; i<fifoDataPointerRam; i++) { printf("%02X ", fifoDataRam[i]); }*/
                    printf("\n");
                }

                // power on flash (NO WAIT after power on -> because initNVS takes at least 15ms @80MHz)
                if(!device.flashPowerOn(false)) { lastErrorId = 39; errorCnt++; } // turn on flash power already (5ms)
                    
                // get pointers from NVS (15ms -> NEW: 87ms -> should be less now with DataNVS -> yes: 12-15ms)
                t = Arduino::millisWrapper();
                if(!device.initDataNVS()) { lastErrorId = 40; errorCnt++; }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d NVS: initialized, took %lld ms\n", ((uint32_t) Arduino::millisWrapper()), (Arduino::millisWrapper() - t)); }
                uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important
                if(TRACKER_MODE == MODE_TESTRUN) {
                    printf("%d Flash: Need to store: %d Bytes sensor data, %d Bytes RAM data, %d Bytes new data\n", ((uint32_t) Arduino::millisWrapper()), sensorDataPointer, fifoDataPointerRam, fifoDataNewestLen);
                    printf("%d Flash: before storing: flashPointer: %d, flashOffsetPointer: %d, flashBlockToSendNextPointer: %d\n", ((uint32_t) Arduino::millisWrapper()), flashPointer, flashOffsetPointer, flashBlockToSendNextPointer);
                }
                    
                // store data
                uint32_t timeNow = ((uint32_t) Arduino::millisWrapper());
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO space left before storing: %d\n", ((uint32_t) Arduino::millisWrapper()), device.flash.fifoGetFreeSpace(flashBlockToSendNextPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                sequential_write_status_t writeStatus = device.flash.fifoPush(flashBlockToSendNextPointer, flashPointer, flashOffsetPointer, sensorData, sensorDataPointer, fifoDataRam, fifoDataPointerRam, fifoDataNewest, fifoDataNewestLen, MOCK_FLASH_WRITES);
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: after storing: flashPointer: %d, flashOffsetPointer: %d\n", ((uint32_t) Arduino::millisWrapper()), flashPointer, flashOffsetPointer); }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO space left after storing: %d\n", ((uint32_t) Arduino::millisWrapper()), device.flash.fifoGetFreeSpace(flashBlockToSendNextPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                timeNow = ((uint32_t) Arduino::millisWrapper()) - timeNow;
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: storage took: %dms\n", ((uint32_t) Arduino::millisWrapper()), timeNow); }
                if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full -> go into special WIFI TRANSMISSION MODE!
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO full -> go into ST_MEMFULL state\n", ((uint32_t) Arduino::millisWrapper())); }
                    if(!device.imu.stop()) { lastErrorId = 46; errorCnt++; } // turn off imu, WARNING: will not fully turn of MAG if turned on before
                    fifoStopped = true;
                    state = ST_MEMFULL;
                    if(!device.flashPowerOff()) { lastErrorId = 41; errorCnt++; } // important!
                    fifoDataPointerRam = 0; // reset pointer
                    heap_caps_free(sensorData); // important, free sensorData memory
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n----- SLEEP -----\n", ((uint32_t) Arduino::millisWrapper()), lastErrorId, errorCnt); }
                    device.enableInternalTimerInterruptInDeepSleep(ST_MEMFULL_WIFI_SCAN_INTERVAL_SECONDS); // sleep for some time
                }
                else { // MT29_SEQ_WRITE_STATUS_SUCCESS or MT29_SEQ_WRITE_STATUS_ERROR, also update pointers in case of ERROR
                    // update pointer
                    device.nvsWriteUINT32x2(NVS_FLASH_WRITE_POINTER, flashPointer, NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, flashOffsetPointer);
                    if(writeStatus == MT29_SEQ_WRITE_STATUS_BUFFER_ERROR) { lastErrorId = 42; errorCnt++; }
                    else if(writeStatus == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR) { lastErrorId = 77; errorCnt++; }

                    // fifo data in ram
                    fifoDataPointerRam = 0; // reset pointer

                    // free memory
                    heap_caps_free(sensorData);

                    // wifi data transmission (0ms if not connecting)
                    timeNow = ((uint32_t) Arduino::millisWrapper());
                    bool somethingTransmitted = dataTransmissionWifi(false, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer);
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: needed %dms, something transmitted: %d\n", ((uint32_t) Arduino::millisWrapper()), ((uint32_t) Arduino::millisWrapper()) - timeNow, somethingTransmitted); }
                
                    // turn off flash power (3ms)
                    if(!device.flashPowerOff()) { lastErrorId = 43; errorCnt++; }

                    // check if data transmission took long time (fifo full already, most probably missed the acc interrupt)
                    uint16_t fifoLenAfterWifi = device.imu.getFIFOLength();
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO LEN before sleep: %d (reset when >= %d)\n", ((uint32_t) Arduino::millisWrapper()), fifoLenAfterWifi, ACC_INTERRUPT_WATERMARK); }
                    if(fifoLenAfterWifi >= ACC_INTERRUPT_WATERMARK) { // fifo full again -> reset it
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO RESET! DATA lost!\n", ((uint32_t) Arduino::millisWrapper())); }
                        if(!device.imu.resetFIFO()) { lastErrorId = 44; errorCnt++; } // empty FIFO, do not read
                        fifoDataPointerRam = 0; // reset RAM data (not tested yet!)
                    }

                    // print error count
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n----- SLEEP -----\n", ((uint32_t) Arduino::millisWrapper()), lastErrorId, errorCnt); }

                    device.enableAccInterruptInDeepSleep();
                }
            }
        }
        startCnt++;
        device.deepSleep();
    }
}