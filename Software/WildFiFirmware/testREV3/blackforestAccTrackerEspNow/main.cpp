#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

// @6.25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 1 hour (6.25 * 6 Byte = 37.5 Bytes per second)
// @25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 15min (25 * 6 Byte = 150 Bytes per second)
// @25Hz: memory full after 20 days, ???uA testrun (productive: 443uA) average WITHOUT esp now data transmission, 609uA (!!!) with esp now data transmission (sample 1 block -> transmit 1 block in 977-1064ms)

// TEST: average power consumption
// TEST: new state ST_MEMFULL! -> errorCnt was 1 once, don't know where

// TODO: problem: old LDO -> sometimes flash reads out first page or only 00000000000 or only FFFFFFFFF or first page, happens randomly! guess voltage drop
// TODO: problem: partial page programming should only be done with 512 bytes, not randomly

// TODO (LOW PRIO): new data with >= REV4B: hall sensor
// TODO (LOW PRIO): new data: sample one time MAG data?
// TODO (LOW PRIO): 1MHz I2C should not work with RTC?!? -> dynamic I2C speed switching -> TEST i2c.changeClockSpeed(I2C_FREQ_HZ_400KHZ)
// TODO (LOW PRIO): switching between 10 and 80MHz or DFS?

// TODO: on DOG sometimes 2g exceeded!
// TODO: Antennenfuß unten zur MCU hin locker nach Hundetest
// TODO: custom PHY initialization -> RTC overflow by over 1868 bytes
// TODO: CHECK IF FIFO >= 996 -> add error cnt

// TEST: instead of wake stub, get data after boot (maybe faster = less power?)

// TODO: ADD TIME SYNC ALGORITHM FROM ESP NOW PROX DETECTION
// TODO: ADD DEFINE FOR PHY SELECTION OF ESP NOW!

// TEST: initEspNowStationary -> should be less power consumption

#define MODE_TESTRUN                            0                               // with debug output, not writing into flash memory, not incrementing NVS_FLASH_POINTER
#define MODE_PRODUCTIVE                         1                               // normal tracker mode, fully operational, flash memory should be empty before and NVS_FLASH_POINTER reset to 0
#define MODE_READFLASH                          2                               // reads out full flash (10.000 pages = 30min) and then goes to sleep
#define MODE_RESETALL                           3                               // deletes complete flash and NVS_FLASH_POINTER
#define MODE_SELFTEST                           4                               // runs selftest on flash memory
#define MODE_ACC_FOC                            5                               // ONLY ONE TIME when tag is FLAT on table (z-Axis = 1g), is stored into ACC NVM memory

/** --- CURRENT TRACKER MODE --- */
#define TRACKER_MODE                            MODE_PRODUCTIVE
#define MOCK_FLASH_WRITES                       0                               // 1 = flash will not be written, but all pointers will be updated and data transmitted
#define MOCK_FLASH_READ                         0                               // 1 = during transmissions dummy data is sent
#define MOCK_FLASH_DELETE                       0                               // 1 = flash block will not be deleted after successful data transmission, but all pointers are updated

acc_config_t accConfig = {
    BMX160_ACCEL_ODR_50HZ,                                                      // Acc frequency
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

#define WIFI_OUTPUT_POWER                       RADIO_MAX_TX_11_DBM             // 19.5dBm will brown out on REV3
#define WIFI_MAX_SCAN_TIME_SECONDS              5                               // scan should only take <1s
#define WIFI_MAX_CONNECT_TIME_SECONDS           8                               // connect should take <1s

#define ESPNOW_OUTPUT_POWER                     RADIO_MAX_TX_19_5_DBM           // 19.5dBm will brown out on REV3
#define ESPNOW_MIN_BATT_VOLTAGE                 1000                            // 3550, don't start scan if battery voltage too low
#define ESPNOW_MIN_TIME_BETWEEN_CHECKS_SECONDS  3600+1800                       // 3600+1800 = 1,5hr, don't scan/transmit too frequently, ONLY VALID IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT)
#define ESPNOW_MIN_BLOCKS_TO_TRANSMIT           1                               // only start scan (and data transmission) if there is enough data to send
#define ESPNOW_MAX_BLOCKS_TO_TRANSMIT           10                              // during one connection, burst maximum this amount of blocks
#define ESPNOW_MIN_BATT_VOLTAGE_DURING_TRANSM   1000                            // 3300, don't continue transmission if voltage dropped below that threshold after one block transmission

#define START_TIME_BETWEEN_ACTIVATION_SCANS     60                              // 900, if not activated: sleep for that time, wake up, check if wifi there and try to activate, 120 = 234uA average
#define ST_MEMFULL_WIFI_SCAN_INTERVAL_SECONDS   3600                            // if fifo memory is full -> go into special state and try to transmit the data every X seconds

#define START_WIFI_SCAN_CHANNEL                 1                               // wifi scan channel for activation/timestamp
const char* START_WIFI_SSID = "LiWoAb New";                                     // wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* START_WIFI_PASSWORD = "xxxxxx";                                // wifi password for activation/timestamp

//static uint8_t espNowReceiverMac[6] = {0xD8, 0xA0, 0x1D, 0x69, 0xE8, 0xFC}; // PicoKit
static uint8_t espNowReceiverMac[6] = {0xAC, 0x67, 0xB2, 0x2B, 0x4E, 0x88}; // ESP32 CAM (first one)

// NVS storage names
#define NVS_FLASH_WRITE_POINTER                         "flashpnt"              // data writing: pointing on current flash page
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffsetpnt"        // data writing: pointing on current byte in flash page
#define NVS_FLASH_TAG_ACTIVATED_BY_WIFI                 "activated"             // tag has seen activation wifi

// NVS storage names for ESP NOW
#define NVS_FLASH_SEND_NEXT_BLOCK_POINTER               "blockpntsent"          // data transmission: pointing on next block to transmit
#define NVS_FLASH_SEND_NEXT_PAGE_POINTER                "pagepntsent"           // data transmission: pointing on next page within block to transmit
#define NVS_FLASH_SEND_NEXT_SUBPAGE_POINTER             "subpagesent"           // data transmission: pointing on next subpage within page to transmit

// state enums
typedef enum { ST_FIRST_START_HARD_RESET = 0, ST_WAIT_FOR_ACTIVATION = 1, ST_START = 2, ST_TRACK = 3, ST_PWRDWN = 4, ST_MEMFULL = 5 } tracker_state_t;

// RTC variables
RTC_DATA_ATTR uint32_t lastTimeEspNow = 0; // REMEMBER: will be reset after hard reset due to BROWNOUT
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
    uint8_t flashPageToSendNextPointer = device.nvsReadUINT8(NVS_FLASH_SEND_NEXT_PAGE_POINTER);
    printf("NVS_FLASH_SEND_NEXT_PAGE_POINTER AFTER DELETE: %d\n", flashPageToSendNextPointer);
    uint8_t flashSubPageToSendNextPointer = device.nvsReadUINT8(NVS_FLASH_SEND_NEXT_SUBPAGE_POINTER);
    printf("NVS_FLASH_SEND_NEXT_SUBPAGE_POINTER AFTER DELETE: %d\n", flashSubPageToSendNextPointer);

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

bool dataTransmissionEspNow(bool forceMode, uint32_t currentTimestamp, uint32_t batteryVoltage, uint32_t flashPointer) { // perform a wifi scan (366ms scan only, 1515ms in total if connecting)
    uint32_t espNowStartTime;
    bool somethingTransmitted = false; // at least ONE block deleted (so more space in memory than before)

    /* -------- GET POINTER VALUES FROM NVS -------- */
    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER);
    uint8_t flashPageToSendNextPointer = device.nvsReadUINT8(NVS_FLASH_SEND_NEXT_PAGE_POINTER);
    uint8_t flashSubPageToSendNextPointer = device.nvsReadUINT8(NVS_FLASH_SEND_NEXT_SUBPAGE_POINTER);

    /* -------- GET NUMBER OF BLOCKS TO TRANSMIT -------- */
    uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockToSendNextPointer, flashPointer);
    uint16_t blocksToTransmitLimited = blocksToTransmit;
    if(blocksToTransmitLimited > ESPNOW_MAX_BLOCKS_TO_TRANSMIT) { // limit maximum number of blocks to transmit (will be less if voltage low or errors)
        blocksToTransmitLimited = ESPNOW_MAX_BLOCKS_TO_TRANSMIT;
    }

    /* -------- CHECK CONDITIONS -------- */
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: lastScan %d, blocksToTransmit %d, blockPnt %d, pagePnt %d, subPagePnt %d, batteryV %d\n", ((uint32_t) Arduino::millisWrapper()), (currentTimestamp-lastTimeEspNow), blocksToTransmit, flashBlockToSendNextPointer, flashPageToSendNextPointer, flashSubPageToSendNextPointer, batteryVoltage); }
    if(batteryVoltage > ESPNOW_MIN_BATT_VOLTAGE) { // only if enough juice in battery
        if(blocksToTransmit >= ESPNOW_MIN_BLOCKS_TO_TRANSMIT) { // only if enough data to transmit
            if(forceMode || (currentTimestamp - lastTimeEspNow > ESPNOW_MIN_TIME_BETWEEN_CHECKS_SECONDS)) { // only if last scan at least x seconds ago OR force the transmission try
                /* -------- CONDITIONS OKAY -> TRY TO SEND -------- */
                if((TRACKER_MODE == MODE_TESTRUN) && (forceMode)) { printf("%d ESPNOW: FORCING TRANSMISSION TRY!\n", ((uint32_t) Arduino::millisWrapper())); }
                lastTimeEspNow = currentTimestamp; // scan was performed -> update -> WARNING, WILL GET LOST AFTER HARD RESET!
                espNowStartTime = ((uint32_t) Arduino::millisWrapper());
                //  NO!! rtc_slow_seg overflow by 1868 bytes
                /*if(!device.customPhyInit()) { // init PHY (well, don't init because will be called after deep sleep, so just copy the RTC data)
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: custom PHY init failed!\n", ((uint32_t) Arduino::millisWrapper())); }
                    lastErrorId = 61; errorCnt++;
                    return false;
                }*/
                if(!device.initESPNOWStationary(false, ESPNOW_OUTPUT_POWER, true, WIFI_PHY_RATE_18M)) { // 165ms, performs full calibration I guess (because custom PHY function overflows slow rtc)
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: initESPNOW ERROR!\n", ((uint32_t) Arduino::millisWrapper())); }
                    lastErrorId = 44; errorCnt++;
                    return false;
                } 
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: INIT TOOK %dms!\n", ((uint32_t) Arduino::millisWrapper()), ((uint32_t) Arduino::millisWrapper()) - espNowStartTime); }
                if(!device.addESPNOWReceiverStationary(espNowReceiverMac)) {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: addESPNOWReceiver ERROR!\n", ((uint32_t) Arduino::millisWrapper())); }
                    lastErrorId = 45; errorCnt++;
                    return false;
                }
                /* -------- TRY TO TRANSMIT DATA -------- */
                uint16_t flashBlockToSendNextPointerBefore = flashBlockToSendNextPointer; // 0 .. 2047
                uint8_t flashPageToSendNextPointerBefore = flashPageToSendNextPointer; // 0 .. 63
                uint8_t flashSubPageToSendNextPointerBefore = flashSubPageToSendNextPointer; // 0 .. 8
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: before transmission, blockToSendNext: %d, pageInBlock: %d, subPage: %d!\n", ((uint32_t) Arduino::millisWrapper()), flashBlockToSendNextPointer, flashPageToSendNextPointer, flashSubPageToSendNextPointer); }
                esp_now_stream_status_t espNowStatus = device.doESPNOWFlashStream(10, flashPointer, flashBlockToSendNextPointer, flashPageToSendNextPointer, flashSubPageToSendNextPointer, blocksToTransmitLimited, ESPNOW_MIN_BATT_VOLTAGE_DURING_TRANSM,
                    (TRACKER_MODE == MODE_TESTRUN), MOCK_FLASH_READ, false, MOCK_FLASH_DELETE); // never mock the sending
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: after transmission, blockToSendNext: %d, pageInBlock: %d, subPage: %d, status: %d!\n", ((uint32_t) Arduino::millisWrapper()), flashBlockToSendNextPointer, flashPageToSendNextPointer, flashSubPageToSendNextPointer, espNowStatus); }

                if(espNowStatus != ESP_NOW_STREAM_DATA_FINISHED) {
                    if(espNowStatus != ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR) {
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: STREAM ERROR, status: %d!\n", ((uint32_t) Arduino::millisWrapper()), espNowStatus); }
                        lastErrorId = 45 + espNowStatus;  errorCnt++; // max. 45 + 8 = 53 -> next id = 54 -> add some buffer = 60
                    }
                    else { // normal case, no gateway found
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: no gateway found!\n", ((uint32_t) Arduino::millisWrapper())); }
                    }
                }

                /* -------- UPDATING NVS POINTER -------- */
                if(flashBlockToSendNextPointerBefore != flashBlockToSendNextPointer) { // some blocks are now fully transmitted
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: updating flashBlockToSendNextPointer (old: %d, new: %d)\n", ((uint32_t) Arduino::millisWrapper()), flashBlockToSendNextPointerBefore, flashBlockToSendNextPointer); }
                    device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER, flashBlockToSendNextPointer);
                    somethingTransmitted = true;
                }
                if(flashPageToSendNextPointerBefore != flashPageToSendNextPointer) { // some blocks are now fully transmitted
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: updating flashPageToSendNextPointer (old: %d, new: %d)\n", ((uint32_t) Arduino::millisWrapper()), flashPageToSendNextPointerBefore, flashPageToSendNextPointer); }
                    device.nvsWriteUINT8(NVS_FLASH_SEND_NEXT_PAGE_POINTER, flashPageToSendNextPointer);
                    somethingTransmitted = true;
                }
                if(flashSubPageToSendNextPointerBefore != flashSubPageToSendNextPointer) { // some blocks are now fully transmitted
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: updating flashSubPageToSendNextPointer (old: %d, new: %d)\n", ((uint32_t) Arduino::millisWrapper()), flashSubPageToSendNextPointerBefore, flashSubPageToSendNextPointer); }
                    device.nvsWriteUINT8(NVS_FLASH_SEND_NEXT_SUBPAGE_POINTER, flashSubPageToSendNextPointer);
                    somethingTransmitted = true;
                }
            }
            else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: NO -> recently connected (%ds ago < %d)\n", ((uint32_t) Arduino::millisWrapper()), currentTimestamp - lastTimeEspNow, ESPNOW_MIN_TIME_BETWEEN_CHECKS_SECONDS); }
        }
        else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: NO -> Blocks to transmit (%d) too low\n", ((uint32_t) Arduino::millisWrapper()), blocksToTransmit); }
    }
    else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: NO -> Battery too low\n", ((uint32_t) Arduino::millisWrapper())); }
    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}

RTC_DATA_ATTR bool wakeStub() {
    if(fifoStopped) { // in certain states imu deactivated (memory full or power down) -> power ESP32 directly
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
            if(!device.rtc.disableClockOut()) { lastErrorId = 62; errorCnt++; }
            device.delay(5);

            // RTC: get current timestamp
            uint32_t timestamp = device.rtc.getTimestamp(error);
            if(error) { lastErrorId = 41; errorCnt++; }

            // check NVS if already activated & timestamp is valid
            if(!device.initDataNVS()) { lastErrorId = 42; errorCnt++; }
            uint16_t activated = device.nvsReadUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI);
            if(device.getLastResetReason() == ESP_RST_BROWNOUT) { lastErrorId = 43; errorCnt++; }
            isActivated = (activated > 0); // store state in RTC
            hasValidTimestamp = (timestamp > 1600000000); // store state in RTC, anything below 13.09.2020 12:26 would be wrong
            if(isActivated && hasValidTimestamp) { // can happen when browning out during data transmission or other kind of error reset
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp valid and activated -> START!\n"); }
                lastTimeEspNow = timestamp; // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
                state = ST_START;
            }
            else {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: NVS not activated (%d) or timestamp lost (%d) -> WAIT FOR ACTIVATION!\n", isActivated, hasValidTimestamp); }
                state = ST_WAIT_FOR_ACTIVATION;
            }

            // after hard reset: perform a full PHY calibration -> NO!! rtc_slow_seg overflow by 1868 bytes
            /*if(!device.customPhyInit()) {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: CUSTOM PHY INIT FAILED!!! SHIZZLE\n"); }
                lastErrorId = 60; errorCnt++;
            }*/

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
                        uint32_t espNowStartTime = ((uint32_t) Arduino::millisWrapper());
                        while(!device.wiFiScanCompleted()) {
                            device.delay(20);
                            if(((uint32_t) Arduino::millisWrapper()) - espNowStartTime > (WIFI_MAX_SCAN_TIME_SECONDS * 1000)) {
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: TIMEOUT SCAN!\n"); }
                                device.disconnectAndStopWiFi();
                                lastErrorId = 34; errorCnt++;
                                wifiError = true;
                            }
                        }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: SCAN TOOK %dms (MAX %d)!\n", ((uint32_t) Arduino::millisWrapper()) - espNowStartTime, (WIFI_MAX_SCAN_TIME_SECONDS * 1000)); }
                        if((!wifiError) && device.wiFiScanIncludes(START_WIFI_SSID)) {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: activation wifi found, isActivated: %d, hasValidTimestamp: %d\n", isActivated, hasValidTimestamp); }
                            if(!isActivated) { // not yet activated
                               if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: wasn't activated -> activate now!\n"); }
                                if(!device.initDataNVS()) { lastErrorId = 35; errorCnt++; }
                                else {
                                    if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 1)) { lastErrorId = 36; errorCnt++; } // write activation into NVS
                                    else {
                                        isActivated = true; // now is activated!
                                    }
                                }   
                            }
                            if(!hasValidTimestamp) { // no valid timestamp (full power down -> need to connect and get time)
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: timestamp not okay -> try to connect and get time\n"); }
                                if(!device.connectToWiFiAfterScan(START_WIFI_SSID, START_WIFI_PASSWORD, START_WIFI_SCAN_CHANNEL)) { lastErrorId = 37; errorCnt++; }
                                else {
                                    uint32_t connectStartedTime = ((uint32_t) Arduino::millisWrapper());
                                    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                                        device.delay(20);
                                        if(((uint32_t) Arduino::millisWrapper()) - connectStartedTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: TIMEOUT CONNECT!\n"); }
                                            wifiError = true;
                                            break;
                                        }
                                    }
                                    if(wifiError || (device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: ACCESS POINT NOT FOUND OR OTHER ISSUE!\n"); }
                                        device.disconnectAndStopWiFi();
                                        lastErrorId = 38; errorCnt++;
                                    }
                                    else { // connected to wifi
                                        uint32_t timestampUTC = 0;
                                        uint16_t millisecondsUTC = 0;
                                        if(!device.getNTPTimestampUTC(timestampUTC, millisecondsUTC, 7000, "pool.ntp.org")) { // will block
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: UTC get time error!\n"); }
                                        }
                                        else {
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: UTC milliseconds: %d -> wait for %dms\n", millisecondsUTC, 1000 - millisecondsUTC); }
                                            if(timestampUTC > 1600000000) { // looks like a valid timestamp from NTP
                                                i2c.begin(I2C_FREQ_HZ_400KHZ);
                                                if(millisecondsUTC > 1000) { millisecondsUTC = 1000; lastErrorId = 39; errorCnt++; }
                                                device.delay(1000 - millisecondsUTC); // wait milliseconds until next second is full
                                                if(!device.rtc.setTimestamp(timestampUTC+1)) { lastErrorId = 40; errorCnt++; }
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
                    else { lastErrorId = 32; errorCnt++; device.enableInternalTimerInterruptInDeepSleep(START_TIME_BETWEEN_ACTIVATION_SCANS); }
                }
                else {lastErrorId = 33; errorCnt++; device.enableInternalTimerInterruptInDeepSleep(START_TIME_BETWEEN_ACTIVATION_SCANS); }
            }
        }
        /** ---------------- START STATE ---------------- */
        else if(state == ST_START) {
            // init BMX
            i2c.begin(I2C_FREQ_HZ_1MHZ);
            if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 1; errorCnt++; }
            if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 2; errorCnt++; }
            if(!device.imu.initFIFOForAcc()) { lastErrorId = 3; errorCnt++; }

            // print flash pointer (also in productive)
            if(!device.initDataNVS()) { lastErrorId = 4; errorCnt++; }
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
                if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 5; }
                if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 6; errorCnt++; }
                if(!device.imu.initFIFOForAcc()) { lastErrorId = 7; errorCnt++; }
                if(!device.imu.resetFIFO()) { lastErrorId = 8; errorCnt++; } // empty fifo, do not read it
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
                if(error) { lastErrorId = 9; errorCnt++; }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("State: TIMESTAMP: %d\n", timestamp); }

                // turn on flash power already (5ms)
                if(!device.flashPowerOn(false)) { lastErrorId = 10; errorCnt++; }
                    
                // get pointers from NVS (15ms)
                if(!device.initDataNVS()) { lastErrorId = 11; errorCnt++; }
                uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: flashPointer: %d, flashOffsetPointer: %d\n", flashPointer, flashOffsetPointer); }

                // check free space in memory
                if(TRACKER_MODE == MODE_TESTRUN) {
                    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be called again in dataTransmission, but not important
                    uint32_t freeSpaceBytesInFIFO = device.flash.fifoGetFreeSpace(flashBlockToSendNextPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES);
                    printf("Flash: FIFO free space in memory: %d\n", freeSpaceBytesInFIFO);
                }
                
                // esp now data transmission (FORCE mode!)
                uint32_t timeNow = ((uint32_t) Arduino::millisWrapper());
                bool somethingTransmitted = dataTransmissionEspNow(true, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer);
                if(TRACKER_MODE == MODE_TESTRUN) { printf("ESP NOW needed %dms\n", ((uint32_t) Arduino::millisWrapper()) - timeNow); }

                // turn off flash
                if(!device.flashPowerOff()) { lastErrorId = 12; errorCnt++; } 

                // check if some data was transmitted in dataTransmissionEspNow
                if(TRACKER_MODE == MODE_TESTRUN) { printf("State: something transmitted: %d\n", somethingTransmitted); }
                if(somethingTransmitted) { // at least one block or half a block was transmitted -> enable IMU again, go into tracking state again
                    if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 13; errorCnt++; } // init IMU again
                    if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 14; errorCnt++; }
                    if(!device.imu.initFIFOForAcc()) { lastErrorId = 15; errorCnt++; }
                    if(!device.imu.resetFIFO()) { lastErrorId = 16; errorCnt++; } // empty fifo, do not read it
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
                if(!device.imu.stop()) { lastErrorId = 17; errorCnt++; } // turn off imu, WARNING: will not fully turn of MAG if turned on before
                fifoStopped = true;
                device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
            }
            else {
                // reserve some memory for flash (0ms)
                if(!device.flash.createBuffer(&sensorData, 100)) { lastErrorId = 18; errorCnt++; }

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
                if(error) { lastErrorId = 23; errorCnt++; }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Data: TIMESTAMP: %d\n", ((uint32_t) Arduino::millisWrapper()), timestamp); }

                // get bme data (1ms)
                if(bmeOk) {
                    if(device.baro.getResults()) {
                        temperature = device.baro.getTemperature(error);
                        if(error) { lastErrorId = 20; errorCnt++; }
                        pressure = device.baro.getPressure(error);
                        if(error) { lastErrorId = 21; errorCnt++; }
                        humidity = device.baro.getHumidity(error);
                        if(error) { lastErrorId = 22; errorCnt++; }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Data: TEMP: %d, TEMP BMX: %d, PRESS: %d, HUMI: %d\n", ((uint32_t) Arduino::millisWrapper()), temperature, temperatureBmx, pressure, humidity); }
                    }
                }
                else { lastErrorId = 19; errorCnt++; }

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
                    printf("\n");
                }

                // power on flash (NO WAIT after power on -> because initNVS takes at least 15ms @80MHz)
                if(!device.flashPowerOn(false)) { lastErrorId = 24; errorCnt++; } // turn on flash power already (5ms)
                    
                // get pointers from NVS (15ms -> NEW: 87ms -> should be less now with DataNVS -> yes: 12-16ms)
                t = Arduino::millisWrapper();
                if(!device.initDataNVS()) { lastErrorId = 25; errorCnt++; }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d NVS: initialized, took %lld ms\n", ((uint32_t) Arduino::millisWrapper()), (Arduino::millisWrapper() - t)); }
                uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
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
                    if(!device.imu.stop()) { lastErrorId = 26; errorCnt++; } // turn off imu, WARNING: will not fully turn of MAG if turned on before
                    fifoStopped = true;
                    state = ST_MEMFULL;
                    if(!device.flashPowerOff()) { lastErrorId = 27; errorCnt++; } // important!
                    fifoDataPointerRam = 0; // reset pointer
                    heap_caps_free(sensorData); // important, free sensorData memory
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n----- SLEEP -----\n", ((uint32_t) Arduino::millisWrapper()), lastErrorId, errorCnt); }
                    device.enableInternalTimerInterruptInDeepSleep(ST_MEMFULL_WIFI_SCAN_INTERVAL_SECONDS); // sleep for some time
                }
                else { // MT29_SEQ_WRITE_STATUS_SUCCESS or MT29_SEQ_WRITE_STATUS_ERROR, also update pointers in case of ERROR
                    // update pointer
                    device.nvsWriteUINT32x2(NVS_FLASH_WRITE_POINTER, flashPointer, NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, flashOffsetPointer);
                    if(writeStatus == MT29_SEQ_WRITE_STATUS_BUFFER_ERROR) { lastErrorId = 28; errorCnt++; }
                    else if(writeStatus == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR) { lastErrorId = 29; errorCnt++; }

                    // fifo data in ram
                    fifoDataPointerRam = 0; // reset pointer

                    // free memory
                    heap_caps_free(sensorData);

                    // esp now data transmission (0ms if not connecting)
                    timeNow = ((uint32_t) Arduino::millisWrapper());
                    bool somethingTransmitted = dataTransmissionEspNow(false, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer);
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: needed %dms, something transmitted: %d\n", ((uint32_t) Arduino::millisWrapper()), ((uint32_t) Arduino::millisWrapper()) - timeNow, somethingTransmitted); }
                
                    // turn off flash power (3ms)
                    if(!device.flashPowerOff()) { lastErrorId = 30; errorCnt++; }

                    // check if data transmission took long time (fifo full already, most probably missed the acc interrupt)
                    uint16_t fifoLenAfterWifi = device.imu.getFIFOLength();
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO LEN before sleep: %d (reset when >= %d)\n", ((uint32_t) Arduino::millisWrapper()), fifoLenAfterWifi, ACC_INTERRUPT_WATERMARK); }
                    if(fifoLenAfterWifi >= ACC_INTERRUPT_WATERMARK) { // fifo full again -> reset it
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO RESET! DATA lost!\n", ((uint32_t) Arduino::millisWrapper())); }
                        if(!device.imu.resetFIFO()) { lastErrorId = 31; errorCnt++; } // empty FIFO, do not read
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