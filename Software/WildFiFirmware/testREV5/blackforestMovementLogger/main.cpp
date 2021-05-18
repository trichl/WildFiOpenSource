#include "PlatformWildFiTagREV5.h"
#include "ModuleGPS_L70.h"
#include "configGeneralSettings.h"
#include "configParagliders.h"

WildFiTagREV5 device = WildFiTagREV5();
GPS_L70 gps = GPS_L70();

// Next open error id: lastErrorId = 158, 116

// WARNING: THIS VERSION RUNS ON SINGLE CORE!
// WARNING: ULP NEEDS TO BE DISABLED IN MENUCONFIG, OTHERWISE RTC SLOW MEMORY OVERFLOW (+512Kbyte for ULP)

// NO DATA TRANSMISSION UNICORE + NEW ALGORITHM WITHOUT WAKE STUB (PRODUCTIVE)
    // @200Hz, 918: 3.00mA (2.69mA with RTC_FAST_MEM 7x, -> 2.35mA with UNICORE <-)
        // 3.46mA(+) WITH data transmission Android app @11DBM, live data every 4 blocks (long connection time of wifi - actual data transmission is faster than esp now)
        // 3.00mA WITH data transmission ESP NOW @2DBM (!!!!) + WIFI_PHY_RATE_18M, live data every 4 blocks
            // 300uWh for 4 blocks (pretty much always, very deterministic)
// NO DATA TRANSMISSION: POWER CONSUMPTION AVERAGE IN PRODUCTIVE (NO WIFI):
    // @6.25Hz: 160uA (more with new LDO)
    // @50Hz: 853uA, in 16.25s producing 29 + 3840 + 972 = 4841 Bytes = 10.42 days until memory full
    // @200Hz: 3.2mA, in 3.85s producing 29 + 3600 + 954 Bytes = 4583 Bytes -> 4583 / 3.85 = 1190.39 Bytes per second -> ((256*1024*1024) / 1190.39) / (3600) = 62.6h until memory full -> after 3hrs around 100 blocks
        // 4.36mA WITH data transmission Android app @11DBM, live data every 4 blocks
// WIFI
    // @6.25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 1 hour (6.25 * 6 Byte = 37.5 Bytes per second)
    // @25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 15min (25 * 6 Byte = 150 Bytes per second)
    // @25Hz: memory full after 20 days, 503uA testrun (productive: 463uA (!!!)) average WITHOUT wifi data transmission, 1.34mA with WiFi transmission (sample 1 block -> transmit 1 block in 8s)
    // 10 Blocks = 10 * 64 * 2048 = 1.310.720 (1.25MByte) in 70 seconds, avg. 80.9mA, 5.89mWh / 3.75V = 1.57mAh
        // with own Android app, no BASE64 encoding: 10 Blocks in 11.2 seconds! and 100 Blocks in 94 - 121 seconds! (DEBUG ON)
            // 62.68 MByte in 9min 30s (avg: 102mA @ 11DBM)
            // 28.06 MByte = 214 Blocks (1 x 150 blocks, 1 x 64 blocks in total 3min 14s) = 21.9mWh = 5.84mAh (@11DBM)
    // FAILED: old LDO -> sometimes flash reads out first page or only 00000000000 or only FFFFFFFFF or first page, happens randomly! guess voltage drop
// ESPNOW
    // @6.25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 1 hour (6.25 * 6 Byte = 37.5 Bytes per second)
    // @25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 15min (25 * 6 Byte = 150 Bytes per second)
    // @25Hz: memory full after 20 days, ???uA testrun (productive: 443uA) average WITHOUT esp now data transmission, 609uA (!!!) with esp now data transmission (sample 1 block -> transmit 1 block in 977-1064ms)
    // FAILED: custom PHY initialization -> RTC overflow by over 1868 bytes -> PHY init with each ESP NOW start (+ more milliseconds)

/** FOR WILD DOGS */
// TEST: data transmission of two or three tags in parallel

/** FOR HANNAH */
// TODO: RF full calibration does not work after wake up
// TODO: merge all from above into REV6 software
// TODO: force run?
// TODO: integrate nighttime disabled from REV6 Software:

/*
#define NIGHTTIME_ALWAYS_NIGHT                          0
#define NIGHTTIME_USE_HOURS                             1
#define NIGHTTIME_DISABLED                              2
*/

/** GENERAL */
// WARNING: if setting time needs more than 1 second -> loosing lightSleep sync! (should be solved)
// WARNING: problem: partial page programming should only be done with 512 bytes, not randomly
    // WARNING: proximity detection data corruption after around 120 partial page writes (single page)
    // seems to be no issue if less than 4 partial page writes (independent of length)
// TODO: detect GPS at start (by trying to set baud rate to 115200)
// TODO: force tracking and de-activation is not considered in accTrackingMode() yet
// TODO (LOW PRIO): new data with: ESP32 hall sensor
// TODO (LOW PRIO): 1MHz I2C should not work with RTC?!? -> dynamic I2C speed switching -> TEST i2c.changeClockSpeed(I2C_FREQ_HZ_400KHZ)
// TODO (LOW PRIO): fifoPush -> add read back of written bytes? (ECC wrong anyhow)
// TODO (LOW PRIO): addData from HelperBits
// TODO: ADD MAG + GYRO DATA TO FIFO
// TODO (WAIT): RTC_SLOW memory will be 8kByte instead of 4kByte in the next releases -> tested but seems buggy (fast wake up not working anymore)
    // https://github.com/espressif/esp-idf/commit/ef10c2576ff14afa033ef22105406399abc570af -> maybe modify esp idf manually?

// RTC variables
RTC_DATA_ATTR uint32_t timestampNextDataTransmission = 0;
RTC_DATA_ATTR uint32_t memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY;
RTC_DATA_ATTR uint8_t fifoDataRam[ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2] = { 0 }; // RTC_FAST_ATTR
RTC_DATA_ATTR uint16_t fifoDataPointerRam = 0;

RTC_DATA_ATTR tracker_state_t state = ST_FIRST_START_HARD_RESET;
RTC_DATA_ATTR uint32_t startCnt = 0;
RTC_DATA_ATTR uint8_t lastErrorId = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;

RTC_DATA_ATTR bool isActivated = false;
RTC_DATA_ATTR bool hasValidTimestamp = false;
RTC_DATA_ATTR uint8_t customRFCalibrationCnt = 0;
RTC_DATA_ATTR bool espNowForceTrackingOnce = false;
RTC_DATA_ATTR uint8_t espNowForceTrackingOnceCnt = 0;
RTC_DATA_ATTR uint64_t nightTimeCnt = 0;

RTC_DATA_ATTR bmm150_trim_registers trimData = {};

// global variables (non RTC)
bool error = false;
uint64_t t = 0;
uint8_t fifoDataNewest[1200] = { 0 };
uint8_t *sensorData = NULL; 
uint16_t sensorDataPointer = 0;
uint8_t gatewayAroundEspNowMac[6] = { 0 };
bool gatewayAroundMessageEspNowReceived = false;
uint8_t gatewayAroundMessageCommandByte = 0;
bool timerFinished = false;
extern uint16_t adcValue;

void startIMU(bool readTrimData) {
    device.sensorPowerOn();
    device.shortLightSleep(120); // wait until bmx is booted
    if(readTrimData) {
        if(TRACKER_MODE == MODE_TESTRUN) { printf("startIMU: reading trim data\n"); }
        if(!device.imu.magCompensateReadTrimData(&trimData)) { lastErrorId = 153; errorCnt++; }
        if(TRACKER_MODE == MODE_TESTRUN) { device.imu.magCompensatePrintTrimData(&trimData); }
    }
    mag_config_t *magConfigPointer = NULL;
    gyro_config_t *gyroConfigPointer = NULL;
    if(USE_MAGNETOMETER) { magConfigPointer = &magConfig; }
    if(USE_GYRO) { gyroConfigPointer = &gyroConfig; }
    if(!device.imu.start(&accConfig, magConfigPointer, gyroConfigPointer, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 1; errorCnt++; }
    if((TRACKING_DATA_MODE == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) || (TRACKING_DATA_MODE == TRACKING_DATA_MODE_ACC_ONLY)) {
        if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 2; errorCnt++; }
    }
    uint8_t fifoForWhat = BMX160_INIT_FIFO_FOR_ACC;
    if(USE_MAGNETOMETER) { fifoForWhat |= BMX160_INIT_FIFO_FOR_MAG; }
    if(USE_GYRO) { fifoForWhat |= BMX160_INIT_FIFO_FOR_GYRO; }
    if(!device.imu.initFIFO(fifoForWhat)) { lastErrorId = 3; errorCnt++; }
    if(!device.imu.resetFIFO()) { lastErrorId = 113; errorCnt++; }
    fifoDataPointerRam = 0; // reset RAM data
}

void stopIMU() {
    if(!device.imu.stop()) { lastErrorId = 17; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
    device.sensorPowerOff(); // turn off IMU (and environment sensor) completely
    device.shortLightSleep(100); // wait because otherwise interrupt pin might still be valid
}

void readFifoIMU(uint8_t *data, uint16_t len) {
    if(!device.imu.readGeneralFIFOInOneGoFast(true, USE_MAGNETOMETER, USE_GYRO, data, len, false)) { lastErrorId = 116; errorCnt++; }
    if(USE_MAGNETOMETER) {
        bmx160_fifo_dataset_len_t datasetStructure = BMX160_FIFO_DATASET_LEN_ACC_AND_MAG;
        if(USE_GYRO) { datasetStructure = BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO; }
        if(!device.imu.magCompensateFifoData(data, len, datasetStructure, &trimData)) { lastErrorId = 62; errorCnt++;  }
    }
}

void resetActivation() { // WARNING: NVS needs to be initalized
    state = ST_WAIT_FOR_ACTIVATION;
    isActivated = 0;
    if(ACTIVATION_MODE == ACTIVATION_MODE_STORE_PERMANENTLY) {
        if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 0)) { lastErrorId = 154; errorCnt++; } // reset activation
    }
}

void mockFlashState() {
    device.delay(8000);
    if(!device.resetDataNVS()) { printf("NVS ERASE ERROR\n"); }
    if(!device.initDataNVS()) { printf("ERROR NVS INIT\n"); }
    if(!device.nvsWriteUINT32(NVS_FLASH_WRITE_POINTER, 131071 - 6)) { printf("ERROR\n"); } // 131071 = last page
    if(!device.nvsWriteUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, 0)) { printf("ERROR\n"); }
    #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
        if(!device.nvsWriteUINT32(NVS_FLASH_SEND_POINTER, 0)) { printf("ERROR\n"); }
        if(!device.nvsWriteUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER, 0)) { printf("ERROR\n"); }
    #endif
    #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
        if(!device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER, 0)) { printf("ERROR\n"); }
        if(!device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER, 0)) { printf("ERROR\n"); }
    #endif    
    printf(" - DONE -\n");  
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
    if(!device.flashPowerOff(true)) { printf("ERROR FLASH3\n"); }
}

bool nightTimeModeDataTransIsDeepestNight() { // when in night time mode and data transmission is activated - don't try a data transmission in the middle of the night
    bool error = false;
    uint8_t currentHour = 0;
    if(!hasValidTimestamp) { return false; } // can't judge this without valid time
    currentHour = device.rtc.getHours(error);
    if(error) { lastErrorId = 132; errorCnt++; return false; }
	for(uint8_t i=0; i<sizeof(NIGHT_TIME_DATA_TRANS_DEEP_NIGHT_HOURS); i++) {
		if(currentHour == NIGHT_TIME_DATA_TRANS_DEEP_NIGHT_HOURS[i]) { return true; }
	}
    return false;
}

bool isTimeToGoToBed() {
    bool error = false;
    uint8_t currentHour = 0, currentMinute = 0;
    uint16_t minutesOfDay = 0;
    const uint16_t OFF_MINUTES_OF_DAY = (NIGHTTIME_TURN_OFF_HOUR * 60) + NIGHTTIME_TURN_OFF_MINUTE; // 0 ........ 1439
    const uint16_t ON_MINUTES_OF_DAY = (NIGHTTIME_TURN_ON_HOUR * 60) + NIGHTTIME_TURN_ON_MINUTE; // 0 ........ 1439
    if(!NIGHTTIME_ACTIVATED) { return false; } // never time to go to bed
    if(!hasValidTimestamp) { return false; } // never time to go to bed (when GET TIME state is skipped by configuration)
    currentHour = device.rtc.getHours(error);
    if(error) { lastErrorId = 102; errorCnt++; return false; }
    currentMinute = device.rtc.getMinutes(error);
    if(error) { lastErrorId = 104; errorCnt++; return false; }
    minutesOfDay = (currentHour * 60) + currentMinute; // calculate minutes passed that day, 0 ........ 1439
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d NIGHTTIME: (%d:%d) sleep between %d:%d - %d:%d -> ", ((uint32_t) Timing::millis()), currentHour, currentMinute, NIGHTTIME_TURN_OFF_HOUR, NIGHTTIME_TURN_OFF_MINUTE, NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE); }
    if(OFF_MINUTES_OF_DAY < ON_MINUTES_OF_DAY) { // ----OFF___ON--------
        if((minutesOfDay >= OFF_MINUTES_OF_DAY) && (minutesOfDay < ON_MINUTES_OF_DAY)) { // "<" important because interrupt happens at exact minute (14:15) -> would go to sleep again
            if(TRACKER_MODE == MODE_TESTRUN) { printf("YES (case 1)\n"); }
            return true; 
        }
    }
    else { // ___ON----------OFF__
        if((minutesOfDay >= OFF_MINUTES_OF_DAY) || (minutesOfDay < ON_MINUTES_OF_DAY)) { // "<" important because interrupt happens at exact minute (14:15) -> would go to sleep again
            if(TRACKER_MODE == MODE_TESTRUN) { printf("YES (case 2)\n"); }
            return true; 
        }
    }
    if(TRACKER_MODE == MODE_TESTRUN) { printf("NO\n"); }
    return false; 
}

uint32_t calculateMemoryFullSleepTime(uint32_t currentTimestamp, uint32_t onMinute) { // sleeping with internal ESP32 timer, but trying to not add up the time drift error by correcting the sleep time depending on current timestamp
    // WARNING: when called during random time or when time interval changes: might over jump the NEXT interval when closer to next time (because assuming "too early" case)
    // WARNING: if SKIP_GET_TIME = true -> might enter state with random timestamp (still works though, but not synced with real time)
    if(TRACKER_MODE == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: onMinute %d\n", onMinute); }
    uint32_t waitTime = 0;
    uint32_t secondsBetweenSleepInterval = onMinute * 60;
    uint32_t moduloValue = currentTimestamp % secondsBetweenSleepInterval; // between 0 .. secondsBetweenSleepInterval-1
    if(TRACKER_MODE == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: modulo value: %d\n", moduloValue); }
    if(moduloValue > (secondsBetweenSleepInterval / 2)) { // currently too early (14:13 but should be 14:15, modulo = 13min) -> assuming 14:15 should be right time
        waitTime = (secondsBetweenSleepInterval - moduloValue) + secondsBetweenSleepInterval;
        if(TRACKER_MODE == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: assuming too early by %ds -> sleep for %d + %d = %ds\n", (secondsBetweenSleepInterval - moduloValue), (secondsBetweenSleepInterval - moduloValue), secondsBetweenSleepInterval, (secondsBetweenSleepInterval - moduloValue) + secondsBetweenSleepInterval); }
    }
    else { // currently too late (14:17 but should be 14:15, modulo = 2min) -> assuming 14:15 should be right time, if modulo = 0 = on time, then sleeping for exactly 
        waitTime = secondsBetweenSleepInterval - moduloValue;
        if(TRACKER_MODE == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: assuming too late by %ds -> sleep for %d - %d = %ds\n", moduloValue, secondsBetweenSleepInterval, moduloValue, secondsBetweenSleepInterval - moduloValue); }
    }
    return waitTime;
}

void setNextDataTransmissionTimestamp(bool forceMode, uint32_t currentTimestamp, uint32_t onMinute) { // when time = 17:53 and transmission interval is 2 hours -> set to 18:00
    uint32_t secondsPassedSinceLastDesiredTime;
    uint32_t secondsUntilNextDesiredTime;
    if(forceMode || (currentTimestamp == 0)) { // in force mode: don't update next data transmission time
        if(TRACKER_MODE == MODE_TESTRUN) { printf("NEXTDATATRANSM: force mode, don't update\n"); }
        return;
    } 
    if(currentTimestamp < 1600000000) {
        if(TRACKER_MODE == MODE_TESTRUN) { printf("NEXTDATATRANSM: no synced time, don't update\n"); }
        return;
    }
    secondsPassedSinceLastDesiredTime = currentTimestamp % (60 * onMinute);
    secondsUntilNextDesiredTime = (60 * onMinute) - secondsPassedSinceLastDesiredTime;
    timestampNextDataTransmission = currentTimestamp + secondsUntilNextDesiredTime; // substract this from timestamp as a mocked last time transmission
    if(TRACKER_MODE == MODE_TESTRUN) { printf("NEXTDATATRANSM: next transmission: %d, current time %d, difference %ds, onMinute %d\n", timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp, onMinute); }
}

bool itsTimeForDataTransmission(uint32_t currentTimestamp) {
    if(currentTimestamp == 0) { return false; }
    return currentTimestamp >= timestampNextDataTransmission;
}

void handleCustomRFCalibration() {
    if(customRFCalibrationCnt == 0) { // never did a full calibration
        if(!device.fullRFCalibration()) { lastErrorId = 140; errorCnt++; } // 145 - 160ms
        if(TRACKER_MODE == MODE_TESTRUN) { printf("RF: first time, FULL\n"); } 
    }
    else {
        if(customRFCalibrationCnt >= ESPNOW_CUSTOM_RF_FULL_CALIB_EVERY_X_TIMES) { // every now and then: do full RF calibration
            customRFCalibrationCnt = 0;
            if(!device.fullRFCalibration()) { lastErrorId = 141; errorCnt++; } // 145 - 160ms
            if(TRACKER_MODE == MODE_TESTRUN) { printf("RF: FULL CALIB\n"); } 
        }
        else {
            if(!device.onlyLoadRFCalibration()) { lastErrorId = 142; errorCnt++; } // 5ms, using RF data in RTC memory
            if(TRACKER_MODE == MODE_TESTRUN) { printf("RF: LOAD CALIB\n"); } 
        }        
    }
    customRFCalibrationCnt++;
}

#if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
bool dataTransmissionWifi(bool forceMode, uint16_t minBlocksToTransmit, uint32_t currentTimestamp, uint32_t batteryVoltage, uint32_t flashPointer, const char** SSIDS_TO_USE, const char** PASSWORDS_TO_USE, const uint8_t SSIDS_SIZE) { // perform a wifi scan (366ms scan only, 1515ms in total if connecting)
    uint32_t wifiStartTime = ((uint32_t) Timing::millis());
    bool somethingTransmitted = false; // at least ONE block deleted (so more space in memory than before)

    /* -------- GET POINTER VALUES FROM NVS -------- */
    uint16_t flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER);
    uint16_t flashHalfBlockToSendNextPointer = 0;
    if(!REST_SEND_FULL_BLOCKS) { flashHalfBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER); }

    /* -------- GET NUMBER OF BLOCKS TO TRANSMIT -------- */
    uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockDeletedPointer, flashPointer);

    /* -------- CHECK CONDITIONS TO EXECUTE WIFI SCAN -------- */
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: next scan %d, blocksToTransmit %d, batteryV %d\n", ((uint32_t) Timing::millis()), timestampNextDataTransmission, blocksToTransmit, batteryVoltage); }
    if(batteryVoltage > WIFI_MIN_BATT_VOLTAGE) { // only if enough juice in battery
        if(blocksToTransmit >= minBlocksToTransmit) { // only if enough data to transmit
            if(forceMode || itsTimeForDataTransmission(currentTimestamp)) { // only if last scan at least x seconds ago
                if((TRACKER_MODE == MODE_TESTRUN) && (forceMode)) { printf("%d WIFI: FORCING SCAN!\n", ((uint32_t) Timing::millis())); }
                /* -------- NEW SCAN ALLOWED -------- */
                if(!device.initWiFi()) { lastErrorId = 63; errorCnt++; setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); return somethingTransmitted; }
                uint8_t foundArrayId = 0;
                uint8_t foundOnChannel = 0;
                if(!device.scanForWiFisOn1and6and11(SSIDS_TO_USE, SSIDS_SIZE, &foundArrayId, &foundOnChannel, WIFI_OUTPUT_POWER, 120, 500)) { device.disconnectAndStopWiFi(); lastErrorId = 64; errorCnt++; setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); return somethingTransmitted; }
                else {
                    if(foundOnChannel > 0) {
                        /* -------- MY WIFI FOUND -------- */
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: FOUND NO %d (CHAN: %d)!\n", ((uint32_t) Timing::millis()), foundArrayId, foundOnChannel); }
                        uint8_t connectionAttemptCounter = 0;
                        while(true) { // try multiple times to connect, because wifi has already been seen
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: CONNECTION ATTEMPT %d!\n", ((uint32_t) Timing::millis()), connectionAttemptCounter); }
                            if(!device.connectToWiFiAfterScan(SSIDS_TO_USE[foundArrayId], PASSWORDS_TO_USE[foundArrayId], foundOnChannel)) { lastErrorId = 66; errorCnt++; }
                            while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                                device.delay(20);
                                if(((uint32_t) Timing::millis()) - wifiStartTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: TIMEOUT CONNECT!\n", ((uint32_t) Timing::millis())); }
                                    device.disconnectAndStopWiFi();
                                    lastErrorId = 67; errorCnt++;
                                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); // re-try not so often
                                    return somethingTransmitted; // severe error, return immediately, no re-try
                                }
                            }
                            if((device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                                if(connectionAttemptCounter < 2) { // retry two times
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: access point seen, but could not connect: %d! -> RETRY ONCE\n", ((uint32_t) Timing::millis()), device.connectedToWiFi()); }
                                }
                                else {
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: access point seen, but could not connect: %d! -> CANCEL\n", ((uint32_t) Timing::millis()), device.connectedToWiFi()); }
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
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: CONNECT TOOK %dms (MAX %d)!\n", ((uint32_t) Timing::millis()), ((uint32_t) Timing::millis()) - wifiStartTime, (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)); }
                        /* -------- FLASH OPERATION -------- */
                        if(!device.flash.createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: FLASH DMA RESERVE ERROR!\n", ((uint32_t) Timing::millis())); }
                            device.disconnectAndStopWiFi();
                            lastErrorId = 69; errorCnt++;
                            setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); // re-try not so often
                            return somethingTransmitted;
                        }
                        /* -------- DMA BUFFER FOR FLASH CREATED -------- */
                        uint16_t blocksToActuallyTransmit = blocksToTransmit;
                        if(blocksToActuallyTransmit > WIFI_MAX_BLOCKS_TO_TRANSMIT) { // for REST POST routine: limit maximum number of blocks to transmit (will be less if voltage low or errors)
                            blocksToActuallyTransmit = WIFI_MAX_BLOCKS_TO_TRANSMIT;
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
                        restStreamParams.flashBlockToSendNextPointer = flashBlockDeletedPointer;
                        restStreamParams.flashHalfBlockToSendNextPointer = flashHalfBlockToSendNextPointer;
                        restStreamParams.flashMaxNumberOfBlocksToTransmit = blocksToActuallyTransmit;
                        restStreamParams.deviceObject = &device;
                        restStreamParams.minBatteryVoltageToContinue = WIFI_MIN_BATT_VOLTAGE_DURING_TRANSM;
                        restStreamParams.useBase64Encoding = REST_USE_BASE64_ENCODING;
                        restStreamParams.debug = (TRACKER_MODE == MODE_TESTRUN);
                        /* -------- START POST TASK -------- */
                        uint16_t successfullyTransmittedBlocks = 0;
                        uint16_t successfullyTransmittedHalfBlocks = 0;
                        wifiStartTime = ((uint32_t) Timing::millis()); // reset watchdog timer
                        if(REST_SEND_FULL_BLOCKS) { device.doWiFiPOSTStreamCallFlashFullBlock(&restStreamParams, 8192); }
                        else { device.doWiFiPOSTStreamCallFlash(&restStreamParams, 8192); }
                        while(device.getWiFiPOSTCallStatus() == HTTP_POST_DATA_RUNNING) {
                            device.delay(100);
                            /* -------- WATCHDOG POST TASK -------- */
                            if(((uint32_t) Timing::millis()) - wifiStartTime > (WIFI_MAX_POST_TASK_TIME_SECONDS * 1000)) { // additional watchdog in case task is not timing out by itself
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: TASK TIMEOUT triggered, check if blocks were transmitted or not!\n", ((uint32_t) Timing::millis())); }
                                uint16_t successfullyTransmittedBlocksNew = 0;
                                uint16_t successfullyTransmittedHalfBlocksNew = 0; // don't look on half blocks
                                device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocksNew, successfullyTransmittedHalfBlocksNew);
                                if(successfullyTransmittedBlocksNew - successfullyTransmittedBlocks > 0) {
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: %d blocks transfered, continue waiting!\n", ((uint32_t) Timing::millis()), (successfullyTransmittedBlocksNew - successfullyTransmittedBlocks)); }
                                    wifiStartTime = ((uint32_t) Timing::millis()); // reset timer
                                    successfullyTransmittedBlocks = successfullyTransmittedBlocksNew; // update last checked block value
                                }
                                else {
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO blocks transfered, KILL TASK!\n", ((uint32_t) Timing::millis())); }
                                    //device.killPOSTTask(); // brutal! don't do! leads to reset
                                    break; // only BREAK from loop, maybe some block transmitted
                                }
                            }
                        }
                        /* -------- POST TASK FINISHED -------- */
                        if(device.getWiFiPOSTCallStatus() != HTTP_POST_DATA_FINISHED_ALL_GOOD) { // might still be RUNNING! when watchdog kicked in (will then lead to client write error because of software connection abort)
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: POST was not successful: %d!\n", ((uint32_t) Timing::millis()), device.getWiFiPOSTCallStatus()); }
                            lastErrorId = 70 + device.getWiFiPOSTCallStatus(); // 19 error codes -> 70 + 19 = 89, next errorid should be 90
                            errorCnt++;
                        }
                        device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks);
                        
                        if(TRACKER_MODE == MODE_TESTRUN) {
                            if(REST_SEND_FULL_BLOCKS) { printf("%d WIFI: successfully transmitted blocks: %d (half block transmission deactivated)\n", ((uint32_t) Timing::millis()), successfullyTransmittedBlocks); }
                            else { printf("%d WIFI: successfully transmitted blocks: %d, half blocks: %d\n", ((uint32_t) Timing::millis()), successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks); }
                        }
                        //heap_caps_free(dmaBuffer2048Bytes); // CAREFUL: task might still running if it supposed to be killed? -> DO NOT FREE MEMORY AT ALL
                        // also do not free memory of restPrefixPointer
                        /* -------- DELETING SUCCESSFULLY TRANSMITTED BLOCKS (ONLY FULLY TRANSMITTED) -------- */
                        uint16_t flashBlockDeletedPointerBeforeDelete = flashBlockDeletedPointer; // remember old value for NVS update
                        for(uint16_t delBlocks=0; delBlocks<successfullyTransmittedBlocks; delBlocks++) { // deleting is based ONLY on fully transmitted blocks
                            if(!device.flash.fifoPopDelete(flashBlockDeletedPointer, flashPointer, MT29_NUMBER_PAGES, MOCK_FLASH_DELETE)) { // delete block from flash and increment flashBlockDeletedPointer
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: FIFO POP DELETE FAILED!\n", ((uint32_t) Timing::millis())); }
                                device.disconnectAndStopWiFi();
                                lastErrorId = 90; errorCnt++;
                                setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); // re-try not so often
                                return somethingTransmitted; // error here means -> no pointers are updated, retransmitting maybe already deleted blocks, but very unlikely to happen
                            } 
                        }
                        /* -------- UPDATING NVS POINTER -------- */
                        // FULL BLOCK POINTER
                        if(flashBlockDeletedPointerBeforeDelete != flashBlockDeletedPointer) { // some blocks are now fully transmitted
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: updating flashBlockDeletedPointer (old: %d, new: %d)\n", ((uint32_t) Timing::millis()), flashBlockDeletedPointerBeforeDelete, flashBlockDeletedPointer); }
                            device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER, flashBlockDeletedPointer);
                            somethingTransmitted = true; // ONLY HERE, not when half blocks were transfered because blocks are not deleted
                        }
                        // HALF BLOCK POINTER
                        if(!REST_SEND_FULL_BLOCKS) {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: halfBlock points to: %d, successfullyTransmittedHalfBlocks: %d\n", ((uint32_t) Timing::millis()), flashHalfBlockToSendNextPointer, successfullyTransmittedHalfBlocks); }
                            if((flashHalfBlockToSendNextPointer == 0) && (successfullyTransmittedHalfBlocks == 0)) { // do not update, before no half transmitted block, afterwards also not, means everything went smooth
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 1) happy case, no half blocks before or after\n", ((uint32_t) Timing::millis())); }
                            }
                            else if((flashHalfBlockToSendNextPointer == 0) && (successfullyTransmittedHalfBlocks == 1)) { // before no half transmissions, now a half transmission (maybe only 0,5 blocks transmitted or maybe 10,5) -> update!
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 2) half block update -> before all good, now a half transmission\n", ((uint32_t) Timing::millis())); }
                                device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER, 1);
                            }
                            else if((flashHalfBlockToSendNextPointer == 1) && (successfullyTransmittedHalfBlocks == 0)) { // before only half a block was transmitted, now finished this block + maybe more (0,5 or 5,5 blocks transmitted) OR (bug before) 0,0 blocks transmitted
                                if(flashBlockDeletedPointerBeforeDelete != flashBlockDeletedPointer) { // there WAS an actual block transmission, means we update NVS
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 3A) half block before, now actually some blocks transmitted -> half block update\n", ((uint32_t) Timing::millis())); }
                                    device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER, 0);
                                }
                                else {
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 3B) nothing at all transmitted, keep flashHalfBlockToSendNextPointer = %d\n", ((uint32_t) Timing::millis()), flashHalfBlockToSendNextPointer); }
                                }
                            }
                            else if((flashHalfBlockToSendNextPointer == 1) && (successfullyTransmittedHalfBlocks == 1)) { // before a half transmissions, now some blocks AND a half transmission again
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 4) bad luck, half block transmission before AND after, keep half block pointer\n", ((uint32_t) Timing::millis())); }
                            }
                        }
                    }
                    else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: SCANNED, but not found\n", ((uint32_t) Timing::millis())); }
                    device.disconnectAndStopWiFi(); // disconnect here because before that no wifi actions
                }
                /* -------- UPDATING SCANNING FREQUENCY AFTER A SCAN (OR DATATRANSMISSION!!!) WAS PERFORMED -------- */
                if(somethingTransmitted) { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); } // data to transmit and wifi found and data transmitted -> try more frequently (if enough data)
                else { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); } // data to transmit, but wifi not found or data transmission not acked -> try less frequently
            }
            else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> next time %d, current time %d, wait %ds\n", ((uint32_t) Timing::millis()), timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp); }
        }
        else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> Blocks to transmit %d < %d\n", ((uint32_t) Timing::millis()), blocksToTransmit, minBlocksToTransmit); }
    }
    else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> Battery too low\n", ((uint32_t) Timing::millis())); }
    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}
#endif

#if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW

bool isGatewayAroundMessage(wifi_promiscuous_pkt_t* p, wifi_promiscuous_pkt_type_t &type) {
    if(type == WIFI_PKT_MGMT) { // all esp now messages are MGMT frames
        if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + ESPNOW_META_MSG_GATEWAY_AROUND_LEN) { // normally 43 bytes additionally + 250 bytes payload
            if(p->rx_ctrl.rate == ESPNOW_DATA_RATE) { // using the correct proximity data rate
                if(p->payload[ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE] == 0x04) { // is ESP NOW frame
                    if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 0] == ESPNOW_META_MSG_GATEWAY_AROUND)) { // gateway around message
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

bool gatewaySeenEspNow(uint8_t *commandByte) {
    uint8_t data[ESPNOW_META_MSG_TAG_AROUND_LEN] = { 0 };

    esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuousGatewayAround);
    esp_wifi_set_promiscuous(true);
    esp_now_register_recv_cb(proxReceiveCallback); // 0ms

    const esp_timer_create_args_t timerArgs = { .callback = &timerCallback };
    esp_timer_handle_t timer;
    if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { lastErrorId = 122; errorCnt++; return false; }
    if(esp_timer_start_once(timer, ESPNOW_GATEWAY_AROUND_LISTENING_TIME * 1000) != ESP_OK) { lastErrorId = 123; errorCnt++; return false; }

    gatewayAroundMessageEspNowReceived = false;
    gatewayAroundMessageCommandByte = 0;
    while(!timerFinished) {
        vTaskDelay(10 / portTICK_PERIOD_MS); // will send current cpu to sleep (10ms accuracy, will wake up a cycle before that)
        if(gatewayAroundMessageEspNowReceived) {
            if(esp_timer_stop(timer) != ESP_OK) { lastErrorId = 124; errorCnt++; } // stop timer
            break;
        }
    }

    // ALWAYS send an answer
    if(gatewayAroundMessageEspNowReceived) {
        uint16_t voltage = (uint16_t) adcValue;
        data[0] = ESPNOW_META_MSG_TAG_AROUND;
        data[1] = voltage >> 8;
        data[2] = voltage & 0xFF;
        data[3] = lastErrorId;
        data[4] = errorCnt >> 8;
        data[5] = errorCnt & 0xFF;
        data[6] = gatewayAroundMessageCommandByte; // mirror the command byte
        data[7] = state;
        data[8] = isActivated;
        data[9] = hasValidTimestamp;
        device.broadcastESPNOWData(data, ESPNOW_META_MSG_TAG_AROUND_LEN); // spit it out
    }
    *commandByte = gatewayAroundMessageCommandByte;
    return gatewayAroundMessageEspNowReceived;
}

bool dataTransmissionEspNow(bool forceMode, uint32_t minBytesToTransmit, uint32_t currentTimestamp, uint32_t batteryVoltage, uint32_t flashPointer, uint16_t flashOffsetPointer, uint8_t *commandByte) { // perform a wifi scan (366ms scan only, 1515ms in total if connecting)
    uint32_t espNowStartTime;
    bool somethingTransmitted = false; // at least ONE block deleted (so more space in memory than before)

    /* -------- GET POINTER VALUES FROM NVS -------- */
    uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER);
    uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);

    /* -------- GET NUMBER OF BYTES TO TRANSMIT -------- */
    uint32_t sendBytePointer = (sendPagePointer * MT29_CACHE_SIZE) + sendPageOffsetPointer;
	uint32_t writeBytePointer = (flashPointer * MT29_CACHE_SIZE) + flashOffsetPointer;
    uint32_t bytesToTransmit = device.flash.fifoGetNumberOfPopableBytes(sendBytePointer, writeBytePointer);

    /* -------- CHECK CONDITIONS -------- */
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: next scan %d, bytesToTransmit %d, sendPnt %d, sendOffsetPnt %d, batteryV %d\n", ((uint32_t) Timing::millis()), timestampNextDataTransmission, bytesToTransmit, sendPagePointer, sendPageOffsetPointer, batteryVoltage); }
    if(batteryVoltage > ESPNOW_MIN_BATT_VOLTAGE) { // only if enough juice in battery
        if(bytesToTransmit >= minBytesToTransmit) { // only if enough data to transmit
            if(forceMode || itsTimeForDataTransmission(currentTimestamp)) { // only if last scan at least x seconds ago
                /* -------- CONDITIONS OKAY -> TRY TO SEND -------- */
                if((TRACKER_MODE == MODE_TESTRUN) && (forceMode)) { printf("%d ESPNOW: FORCING TRANSMISSION TRY!\n", ((uint32_t) Timing::millis())); }
                espNowStartTime = ((uint32_t) Timing::millis());
                #if ESPNOW_CUSTOM_RF_CALIBRATION == true
                    handleCustomRFCalibration();
                #endif
                if(!device.initESPNOWStationary(ESPNOW_LONG_RANGE, ESPNOW_OUTPUT_POWER, true, ESPNOW_DATA_RATE)) { // 165ms, performs full calibration I guess (because custom PHY function overflows slow rtc)
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: initESPNOW ERROR!\n", ((uint32_t) Timing::millis())); }
                    lastErrorId = 44; errorCnt++;
                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY);
                    device.stopESPNOW(); // 5ms
                    esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                    return false;
                }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: INIT TOOK %dms!\n", ((uint32_t) Timing::millis()), ((uint32_t) Timing::millis()) - espNowStartTime); }
                if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 121; errorCnt++; } // necessary for response
                bool gatewaySeen = gatewaySeenEspNow(commandByte); // fill commandByte here, also send a hello message when gateway was seen
                if(!gatewaySeen) {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: no gateway seen\n", ((uint32_t) Timing::millis())); }
                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY);
                    device.stopESPNOW(); // 5ms
                    esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                    return false;
                }
                else {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: command byte 0x%02X\n", ((uint32_t) Timing::millis()), *commandByte); }
                    if(!device.addESPNOWReceiverStationary(gatewayAroundEspNowMac)) {
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: addESPNOWReceiver ERROR!\n", ((uint32_t) Timing::millis())); }
                        lastErrorId = 45; errorCnt++;
                        setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY);
                        device.stopESPNOW(); // 5ms
                        esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                        return false;
                    }
                    /* -------- TRY TO TRANSMIT DATA -------- */
                    uint32_t sendPagePointerBefore = sendPagePointer ; // 0 .. 2048 * 64
                    uint16_t sendPageOffsetPointerBefore = sendPageOffsetPointer; // 0 .. 2048

                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: before transmission, sendPagePointer: %d, sendPageOffsetPointer: %d!\n", ((uint32_t) Timing::millis()), sendPagePointer, sendPageOffsetPointer); }

                    esp_now_stream_status_t espNowStatus = device.doESPNOWFlashStreamNew(
                        gatewayAroundEspNowMac,
                        NULL, 0,
                        &sendPagePointer, &sendPageOffsetPointer, 
                        flashPointer, flashOffsetPointer, 
                        ESPNOW_MAX_BYTES_TO_TRANSMIT,
                        500, 8, // millis to wait when one message failed (but acks happened before), number of retries
                        ESPNOW_MIN_BATT_VOLTAGE_DURING_TRANSM,
                        (TRACKER_MODE == MODE_TESTRUN) ? 1 : 0, MOCK_FLASH_READ, false); // 0 or (TRACKER_MODE == MODE_TESTRUN) ? 1 : 0, never mock the sending

                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: after transmission, sendPagePointer: %d, sendPageOffsetPointer: %d, status: %d!\n", ((uint32_t) Timing::millis()), sendPagePointer, sendPageOffsetPointer, espNowStatus); }

                    if(espNowStatus != ESP_NOW_STREAM_DATA_FINISHED) {
                        if((espNowStatus != ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR) && (espNowStatus != ESP_NOW_STREAM_NO_DATA_TO_SEND)) { // no data to send happens when function is called with minBlocksToTransmit = 0 (forcing a gateway scan) 
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: STREAM ERROR, status: %d!\n", ((uint32_t) Timing::millis()), espNowStatus); }
                            lastErrorId = 45 + espNowStatus;  errorCnt++; // max. 45 + 12 = 57 -> next id = 58 -> add some buffer = 60
                        }
                        else { // normal case, no gateway found
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: no gateway found or no data to send!\n", ((uint32_t) Timing::millis())); }
                        }
                    }
                    /* -------- STOPPING ESP NOW ALREADY -------- */
                    device.stopESPNOW(); // 5ms
                    esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep

                    /* -------- DELETING SUCCESSFULLY TRANSMITTED FULL BLOCKS -------- */
                    bool errorDuringDelete = false;
                    uint16_t sendBlockBefore = sendPagePointerBefore / MT29_PAGES_PER_BLOCK;
                    uint16_t sendBlockAfter = sendPagePointer / MT29_PAGES_PER_BLOCK; 
                    uint16_t blocksToDelete = 0; 
                    if(sendBlockBefore > sendBlockAfter) { blocksToDelete = (MT29_NUMBER_BLOCKS - sendBlockBefore) + sendBlockAfter; } // DDD_____|________|________|DDDDDDDD|DDDDDDDD|
                    else { blocksToDelete = sendBlockAfter - sendBlockBefore; } // ________|DDDDDDDD|DDDDDDDD|DDDDDDDD|DDDDD___|
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: send block before %d, after %d -> delete %d blocks\n", ((uint32_t) Timing::millis()), sendBlockBefore, sendBlockAfter, blocksToDelete); }
                    for(uint16_t delBlock=sendBlockBefore; delBlock<(sendBlockBefore+blocksToDelete); delBlock++) { // deleting is based ONLY on fully transmitted blocks
                        if(!device.flash.erase(delBlock % MT29_NUMBER_BLOCKS)) {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: BLOCK DELETE FAILED!\n", ((uint32_t) Timing::millis())); }
                            lastErrorId = 34; errorCnt++;
                            errorDuringDelete = true;
                        }
                        else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: DELETED block %d\n", ((uint32_t) Timing::millis()), delBlock % MT29_NUMBER_BLOCKS); }
                    }
                    /* -------- UPDATING NVS POINTER (even if errorDuringDelete happened, because don't know at which block it happened) -------- */
                    if(sendPagePointerBefore != sendPagePointer) { // some pages are now transmitted
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW-NVS: updating sendPagePointer (old: %d, new: %d)\n", ((uint32_t) Timing::millis()), sendPagePointerBefore, sendPagePointer); }
                        device.nvsWriteUINT32((NVS_FLASH_SEND_POINTER), sendPagePointer);
                        somethingTransmitted = true;
                    }
                    if(sendPageOffsetPointerBefore != sendPageOffsetPointer) { // some bytes are transmitted
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW-NVS: updating sendPageOffsetPointer (old: %d, new: %d)\n", ((uint32_t) Timing::millis()), sendPageOffsetPointerBefore, sendPageOffsetPointer); }
                        device.nvsWriteUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER, sendPageOffsetPointer);
                        somethingTransmitted = true;
                    }
                    /* -------- UPDATING SCANNING FREQUENCY AFTER A SCAN WAS PERFORMED -------- */
                    if(somethingTransmitted && (!errorDuringDelete)) { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); } // data to transmit and wifi found and data transmitted -> try more frequently (if enough data)
                    else { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); } // data to transmit, but wifi not found or data transmission not acked -> try less frequently
                }
            }
            else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: NO -> next time %d, current time %d, wait %ds\n", ((uint32_t) Timing::millis()), timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp); }
        }
        else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: NO -> bytes to transmit %d < %d\n", ((uint32_t) Timing::millis()), bytesToTransmit, minBytesToTransmit); }
    }
    else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: NO -> Battery too low\n", ((uint32_t) Timing::millis())); }
    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}
#endif

void tryGetTimeOverGPS() {
    // untested
    device.gpioBOn();
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');
    esp_gps_t gpsData = { };
    if(gps.getTimeOnly(&gpsData, USE_GPS_TO_GET_TIME_TIMEOUT_SECONDS, &device, TRACKING_DATA_MODE_1HZ_GPS_BLINK, (TRACKER_MODE == MODE_TESTRUN))) {
        hasValidTimestamp = true;
        setNextDataTransmissionTimestamp(false, gpsData.parent.utcTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
        if(isActivated) {
            state = ST_START;
            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time over GPS: all done and already activated -> MOVE TO START in %ds!\n", TIME_SLEEP_AFTER_GOT_TIME); }
        }
        else {
            state = ST_WAIT_FOR_ACTIVATION;
            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time over GPS: all done -> move to ACTIVATION in %ds!\n", TIME_SLEEP_AFTER_GOT_TIME); }
        }
        if(DO_THE_BLINK) { device.blinkTimes(6, B_GREEN); }
        device.enableInternalTimerInterruptInDeepSleep(TIME_SLEEP_AFTER_GOT_TIME); // restart in x seconds and move to activation
    }
    else {
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time over GPS: failed!\n"); }
        lastErrorId = 119; errorCnt++;
        device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS);
    }
    device.gpioBOff();
}

void accTrackingMode() {
    /** --- COLLECT FIFO DATA (previously in wake stub) --- */
    i2c.begin(I2C_FREQ_HZ_1MHZ); // 1ms
    uint16_t currentFifoLen = device.imu.getFIFOLength();
    if(currentFifoLen >= 996) { lastErrorId = 118; errorCnt++; } // data loss possible
    if((fifoDataPointerRam + currentFifoLen) <= (ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2)) { // still space to put all fifo data into the RTC RAM
        readFifoIMU(fifoDataRam+fifoDataPointerRam, currentFifoLen);
        fifoDataPointerRam += currentFifoLen;
        startCnt--; // do not count that as start
        device.enableAccInterruptInDeepSleep();
    }
    /** --- NORMAL WAKE UP AFTER RTC RAM IS FULL --- */
    else {
        // TODO: STUPID, because this will be reset after every wakeup
        if(!espNowForceTrackingOnce) { espNowForceTrackingOnceCnt = 0; } // UNTESTED, if this is a regular start: reset the counter
        if(TRACKER_MODE == MODE_TESTRUN) { printf("-----\nState: %d\n", state); }
        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: wake no: %d, V_BATT_WakeStub: %d\n", ((uint32_t) Timing::millis()), startCnt, device.readSupplyVoltageFromWakeStub()); }
        if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) {
            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: POWER DOWN for %ds\n", ((uint32_t) Timing::millis()), FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
            state = ST_PWRDWN;
            stopIMU();
            espNowForceTrackingOnce = false; // stop force mode when coming back after under voltage 
            device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
        }
        else if((!espNowForceTrackingOnce) && isTimeToGoToBed()) {
            state = ST_NIGHT_TIME;
            stopIMU();
            if(DO_THE_BLINK) { device.blinkTimes(5, B_RED); }
            #if NIGHTTIME_MODE == NIGHTTIME_MODE_ONLY_SLEEP
                if(!device.rtc.setDailyInterrupt(NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE)) { lastErrorId = 133; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
            #endif
            #if NIGHTTIME_MODE == NIGHTTIME_MODE_TRY_DATATRANS
                if(!device.rtc.setRegularInterrupt(NIGHTTIME_MODE_TRY_DATATRANS_WAKEUP_SECONDS)) { lastErrorId = 143; errorCnt++; }
                device.enableInternalTimerInterruptInDeepSleep(1); // immediately try a data transmission -> set this interrupt as well
                device.enableRTCInterruptInDeepSleep();
            #endif
        }
        /*else if(espNowForceTrackingOnce) && ???) {
            // TODO, also resetting of this variable at some point:
            espNowForceTrackingOnce = false; // if this was set before 
        }*/
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
            // get fifo data (101ms -> NEW: 16ms -> NEW NEW: 14ms)
            t = Timing::millis();
            uint16_t fifoDataNewestLen = device.imu.getFIFOLength(); // get it again because there might be more data now
            readFifoIMU(fifoDataNewest, fifoDataNewestLen);
            if(fifoDataNewestLen >= 996) { lastErrorId = 36; errorCnt++; } // data loss possible
            if(TRACKER_MODE == MODE_TESTRUN) {
                printf("%d FIFO: RAM pointer: %d (MAX %d) + read %d bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataPointerRam, (ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2), fifoDataNewestLen, (Timing::millis() - t));
                uint16_t fifoLenNew = device.imu.getFIFOLength();
                printf("%d FIFO: BMX FIFO AFTER READ: %d\n", ((uint32_t) Timing::millis()), fifoLenNew);
            }
            // get bmx temperature
            uint16_t temperatureBmxRaw = 0;
            device.imu.getTemperatureRaw(temperatureBmxRaw);
            temperatureBmx = device.imu.toCelsiusx100(temperatureBmxRaw);
            // get rtc timestamp (1ms)
            uint32_t timestamp = device.rtc.getTimestamp(error);
            if(error) { lastErrorId = 23; errorCnt++; }
            // get bme data (1ms)
            if(bmeOk) {
                if(device.baro.getResults()) {
                    temperature = device.baro.getTemperature(error);
                    if(error) { lastErrorId = 20; errorCnt++; }
                    pressure = device.baro.getPressure(error);
                    if(error) { lastErrorId = 21; errorCnt++; }
                    humidity = device.baro.getHumidity(error);
                    if(error) { lastErrorId = 22; errorCnt++; }
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Data: TIMESTAMP: %d, TEMP: %d, TEMP BMX: %d, PRESS: %d, HUMI: %d\n", ((uint32_t) Timing::millis()), timestamp, temperature, temperatureBmx, pressure, humidity); }
                }
            }
            else { lastErrorId = 19; errorCnt++; }
            // add rest of data, already to DMA memory
            HelperBits::addData1_AndIncrementPointer(0x12, sensorData, &sensorDataPointer);
            HelperBits::addData1_AndIncrementPointer(0x34, sensorData, &sensorDataPointer);
            if((USE_MAGNETOMETER) && (USE_GYRO)) { HelperBits::addData1_AndIncrementPointer(0x5E, sensorData, &sensorDataPointer); }
            else if((!USE_MAGNETOMETER) && (USE_GYRO)) { HelperBits::addData1_AndIncrementPointer(0x5F, sensorData, &sensorDataPointer); }
            else if((USE_MAGNETOMETER) && (!USE_GYRO)) { HelperBits::addData1_AndIncrementPointer(0x60, sensorData, &sensorDataPointer); }
            else if((!USE_MAGNETOMETER) && (!USE_GYRO)) { HelperBits::addData1_AndIncrementPointer(0x61, sensorData, &sensorDataPointer); }
            HelperBits::addData4_AndIncrementPointer(timestamp, sensorData, &sensorDataPointer);
            HelperBits::addData1_AndIncrementPointer(0xFF, sensorData, &sensorDataPointer); // milliseconds unknown
            HelperBits::addData1_AndIncrementPointer(lastErrorId, sensorData, &sensorDataPointer);
            HelperBits::addData2_AndIncrementPointer(errorCnt, sensorData, &sensorDataPointer);
            HelperBits::addData2_AndIncrementPointer((uint16_t) device.readSupplyVoltageFromWakeStub(), sensorData, &sensorDataPointer);
            HelperBits::addData2Signed_AndIncrementPointer(temperature, sensorData, &sensorDataPointer);
            HelperBits::addData4_AndIncrementPointer(humidity, sensorData, &sensorDataPointer);
            HelperBits::addData4_AndIncrementPointer(pressure, sensorData, &sensorDataPointer);
            HelperBits::addData2Signed_AndIncrementPointer(temperatureBmx, sensorData, &sensorDataPointer);
            HelperBits::addData2_AndIncrementPointer(fifoDataNewestLen+fifoDataPointerRam, sensorData, &sensorDataPointer);

            // print data
            if(TRACKER_MODE == MODE_TESTRUN) {
                printf("%d Data Header: ", ((uint32_t) Timing::millis()));
                for(uint16_t i=0; i<sensorDataPointer; i++) { printf("%02X ", sensorData[i]); }
                printf("\n");
            }
            // power on flash (NO WAIT after power on -> because initNVS takes at least 15ms @80MHz)
            if(!device.flashPowerOn(false)) { lastErrorId = 24; errorCnt++; } // turn on flash power already (5ms)
            // get pointers from NVS (15ms -> NEW: 87ms -> should be less now with DataNVS -> yes: 12-16ms, unicore only 3ms??? - observe!)
            t = Timing::millis();
            if(!device.initDataNVS()) { lastErrorId = 25; errorCnt++; }
            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d NVS: initialized, took %lld ms\n", ((uint32_t) Timing::millis()), (Timing::millis() - t)); }
            uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
            uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
            uint16_t flashBlockDeletedPointer = 0;
            #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
            #endif
            #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
            #endif
            if(TRACKER_MODE == MODE_TESTRUN) {
                printf("%d Flash: Need to store: %d Bytes sensor data, %d Bytes RAM data, %d Bytes new data\n", ((uint32_t) Timing::millis()), sensorDataPointer, fifoDataPointerRam, fifoDataNewestLen);
                printf("%d Flash: before storing: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer, flashBlockDeletedPointer);
            }
            // store data
            uint32_t timeNow = ((uint32_t) Timing::millis());
            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO space left before storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
            sequential_write_status_t writeStatus = device.flash.fifoPush(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, sensorData, sensorDataPointer, fifoDataRam, fifoDataPointerRam, fifoDataNewest, fifoDataNewestLen, MOCK_FLASH_WRITES);
            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: after storing: flashPointer: %d, flashOffsetPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer); }
            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO space left after storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
            timeNow = ((uint32_t) Timing::millis()) - timeNow;
            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: storage took: %dms\n", ((uint32_t) Timing::millis()), timeNow); }
            if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full -> go into special WIFI TRANSMISSION MODE!
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO full, stop IMU -> go into ST_MEMFULL state in 5 seconds\n", ((uint32_t) Timing::millis())); }
                stopIMU();
                memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY; // after memory full, try frequently to get rid of data
                state = ST_MEMFULL;
                if(!device.flashPowerOff(false)) { lastErrorId = 27; errorCnt++; } // important!
                fifoDataPointerRam = 0; // reset pointer
                heap_caps_free(sensorData); // important, free sensorData memory
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }
                device.enableInternalTimerInterruptInDeepSleep(5); // immediately try to get rid of the data
            }
            else { // MT29_SEQ_WRITE_STATUS_SUCCESS or MT29_SEQ_WRITE_STATUS_ERROR, also update pointers in case of ERROR
                // update pointer
                device.nvsWriteUINT32x2(NVS_FLASH_WRITE_POINTER, flashPointer, NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, flashOffsetPointer);
                if(writeStatus == MT29_SEQ_WRITE_STATUS_BUFFER_ERROR) { lastErrorId = 28; errorCnt++; }
                else if(writeStatus == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR) { lastErrorId = 40; errorCnt++; }
                // fifo data in ram
                fifoDataPointerRam = 0; // reset pointer
                // free memory
                heap_caps_free(sensorData);
                // data transmission (0ms if not connecting)
                timeNow = ((uint32_t) Timing::millis());
                bool somethingTransmitted = false;
                #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                uint8_t commandByte = 0; // unused here
                somethingTransmitted = dataTransmissionEspNow(false, ESPNOW_MIN_BYTES_TO_TRANSMIT, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte);
                #endif
                #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                somethingTransmitted = dataTransmissionWifi(false, WIFI_MIN_BLOCKS_TO_TRANSMIT, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                #endif
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d DATATRANSM: needed %dms, something transmitted: %d\n", ((uint32_t) Timing::millis()), ((uint32_t) Timing::millis()) - timeNow, somethingTransmitted); }
                // turn off flash power (3ms)
                if(!device.flashPowerOff(false)) { lastErrorId = 30; errorCnt++; }
                // check if data transmission took long time (fifo full already, most probably missed the acc interrupt)
                uint16_t fifoLenAfterWifi = device.imu.getFIFOLength();
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO LEN before sleep: %d (reset when >= %d)\n", ((uint32_t) Timing::millis()), fifoLenAfterWifi, ACC_INTERRUPT_WATERMARK); }
                if(fifoLenAfterWifi >= ACC_INTERRUPT_WATERMARK) { // fifo full again -> reset it
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO RESET! DATA lost!\n", ((uint32_t) Timing::millis())); }
                    if(!device.imu.resetFIFO()) { lastErrorId = 60; errorCnt++; } // empty FIFO, do not read
                    fifoDataPointerRam = 0; // reset RAM data (not tested yet!)
                }
                // print error count
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }
                device.enableAccInterruptInDeepSleep();
            }
        }
    }    
}

static bool validTimeTimerFinished = false;
static void validTimeCallback(void* arg) { validTimeTimerFinished = true; }

void special1HzModeAddData(esp_gps_t *gpsData, uint16_t currentFifoLen, uint8_t *buffer, uint16_t *bufferPointer) {
    bool error = false;
    uint32_t timestamp = device.rtc.getTimestamp(error);
    if(error) { lastErrorId = 149; errorCnt++; }
    float temp = gpsData->parent.dop_h * 10.0;
    if(temp > 255.) temp = 255.;
    uint8_t hdopTimesTen = temp;
    temp = gpsData->parent.latitude * 1000000;
    int32_t latTimes1Mil = temp;
    temp = gpsData->parent.longitude * 1000000;
    int32_t lonTimes1Mil = temp;
    HelperBits::addData1_AndIncrementPointer(0x12, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(0x34, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(0x56, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(timestamp, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(lastErrorId, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(latTimes1Mil, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(lonTimes1Mil, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(hdopTimesTen, buffer, bufferPointer);
    HelperBits::addData2_AndIncrementPointer(currentFifoLen, buffer, bufferPointer);
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: UTC %d, LAT %d, LON %d, HDOP %d\n", ((uint32_t) Timing::millis()), timestamp, latTimes1Mil, lonTimes1Mil, hdopTimesTen); }  
}

void special1HzMode() {
    const uint16_t GPS_HEADER_DATA_LEN = 19; // 4 byte header indicator (including lastErrorId), 4 byte timestamp, 4 byte lat, 4 byte lon, 1 byte HDOP, 2 byte fifo length
    const uint16_t FIFO_DATA_LEN = 1024 * 8;
    const uint8_t RUBBISH_MINIMUM_LENGTH_NMEA = 10; // 17
    bool gotTimeAfterFix = false;
    bool keepTrackingRunning = true;
    uint8_t nmeaMessageCounter = 0;
    int64_t ttffStartUs = 0;
    bool errorDuringInit = false;
    uint16_t fifoDataPointer = 0;
    uint16_t addExtraDelay = 0;

    if(!espNowForceTrackingOnce) { espNowForceTrackingOnceCnt = 0; } // if this is a regular start: reset the counter

    i2c.begin(I2C_FREQ_HZ_400KHZ); // DO NOT use 1MHz
    
    if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // do not even start when voltage is low
        if(TRACKER_MODE == MODE_TESTRUN) { printf("1HzGPS: POWER DOWN before start for %ds\n", FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
        state = ST_PWRDWN;
        stopIMU();
        espNowForceTrackingOnce = false; // stop force mode when coming back after under voltage 
        device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
    }
    else if((!espNowForceTrackingOnce) && isTimeToGoToBed()) { // do not even start when it's bed time
        // TESTED PATH
        state = ST_NIGHT_TIME;
        stopIMU();
        if(DO_THE_BLINK) { device.blinkTimes(5, B_RED); }
        #if NIGHTTIME_MODE == NIGHTTIME_MODE_ONLY_SLEEP
            if(!device.rtc.setDailyInterrupt(NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE)) { lastErrorId = 139; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
            device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
        #endif
        #if NIGHTTIME_MODE == NIGHTTIME_MODE_TRY_DATATRANS
            // TESTED PATH
            if(!device.rtc.setRegularInterrupt(NIGHTTIME_MODE_TRY_DATATRANS_WAKEUP_SECONDS)) { lastErrorId = 144; errorCnt++; }
            device.enableInternalTimerInterruptInDeepSleep(2); // immediately try a data transmission -> set this interrupt as well
            device.enableRTCInterruptInDeepSleep();
        #endif
    }
    else {
        if(!device.initDataNVS()) { lastErrorId = 14; errorCnt++; }
        if((TRACKER_MODE == MODE_TESTRUN) && espNowForceTrackingOnce) { printf("1HzGPS: FORCE TRACKING ONCE\n"); }

        // try one time data transmission
        bool somethingTransmitted = false;
        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
        if(!device.flashPowerOn(true)) { lastErrorId = 137; errorCnt++; } // turn on flash power already (10ms)
        uint32_t timeNow = ((uint32_t) Timing::millis());
        uint8_t commandByte = 0; // just for putting device back into activation state
        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
        // esp now data transmission (FORCE mode! but only executed if bytes to transmit)
        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
        somethingTransmitted = dataTransmissionEspNow(true, ESPNOW_MIN_BYTES_TO_TRANSMIT, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte);
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
        #endif
        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
        // wifi data transmission (FORCE mode!)
        somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
        #endif
        if(!device.flashPowerOff(true)) { lastErrorId = 138; errorCnt++; } // turn off flash
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: something transmitted: %d\n", somethingTransmitted); }
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }

        // check if tag shall be deactivated
        if(commandByte == COMMAND_BYTE_DEACTIVATE) {
            if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO DE-ACTIVATE\n"); }
            stopIMU();
            espNowForceTrackingOnce = false; // reset everything, also this
            resetActivation(); // NVS already initialized
            device.enableInternalTimerInterruptInDeepSleep(1);
        }
        else {
            // start with GPS + acc sampling
            device.gpioBOn();
            device.uart2Init(115200);
            gps.init(device.uart2GetQueue());
            device.uart2EnablePatternInterrupt('\n');
            esp_gps_t gpsData = { };

            if(!gps.isStarted()) { errorDuringInit = true; }
            ttffStartUs = esp_timer_get_time(); // start timer
            gpsData.parent.ttfMilliseconds = 0;
            if(!gps.setNMEAMessagesMinimum1HzWithZDA()) { errorDuringInit = true; }
            uart_flush(UART2_PORT_NUMBER);

            const esp_timer_create_args_t validTimeArgs = { .callback = &validTimeCallback };
            esp_timer_handle_t validTimeTimer;
            if(esp_timer_create(&validTimeArgs, &validTimeTimer) != ESP_OK) { errorDuringInit = true; }

            size_t availableRam = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
            if(TRACKER_MODE == MODE_TESTRUN) { printf("1HzGPS: enter, available RAM: %d\n", availableRam); }

            uint8_t *uart2Data = (uint8_t*) malloc(UART2_RX_BUFFER);
            if(uart2Data == NULL) { errorDuringInit = true; }

            uint8_t *fifoData = (uint8_t*) malloc(FIFO_DATA_LEN);
            if(fifoData == NULL) { errorDuringInit = true; }

            if(errorDuringInit) { // SHOULD NOT HAPPEN, but could happen if GPS is not connected -> sleep for FIRST_UNDER_VOLTAGE_SLEEP_TIME and try again afterwards 
                lastErrorId = 128; errorCnt++;
                free(uart2Data);
                free(fifoData);
                device.gpioBOff();
                device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME);
                return;
            }

            device.imu.resetFIFO(); // reset fifo to start fresh
            device.enableUart2InterruptInLightSleep();
            esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
            device.lightSleep();

            while(keepTrackingRunning) {
                // wake up
                //uart_flush(UART2_PORT_NUMBER); // WHY??
                nmeaMessageCounter = 0;
                uart_event_t event;
                size_t bufferedSize;

                // turn on green LED
                if(TRACKING_DATA_MODE_1HZ_GPS_BLINK && DO_THE_BLINK) {
                    if(gpsData.parent.fix > 0) { device.ledGreenOn(); }
                    else { device.ledRedOn(); }
                }

                // check if wakeup due to timeout
                esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
                if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("1HzGPS: GPS timeout wake -> sleep for %d\n", TRACKING_DATA_MODE_1HZ_GPS_NOT_RESPONDING); }
                    lastErrorId = 146; errorCnt++;
                    device.enableInternalTimerInterruptInDeepSleep(TRACKING_DATA_MODE_1HZ_GPS_NOT_RESPONDING); // sleep for some time
                    keepTrackingRunning = false;
                    break;
                    // WARNING: IMU keeps running here!
                }

                // wait until all uart messages arrived
                while(true) {
                    if(xQueueReceive(*(device.uart2GetQueue()), (void * )&event, 0)) {
                        if(event.type == UART_PATTERN_DET) { // '\n' detected
                            nmeaMessageCounter++;
                            uart_get_buffered_data_len(UART2_PORT_NUMBER, &bufferedSize); // get length of message in UART buffer
                            if((nmeaMessageCounter == 1) && (bufferedSize < RUBBISH_MINIMUM_LENGTH_NMEA)) { nmeaMessageCounter--; } // maybe \n in first 20 bytes of rubbish detected (happened) -> do not count as NMEA message

                            if(nmeaMessageCounter >= 3) {
                                bzero(uart2Data, UART2_RX_BUFFER); // write all zeros into buffer
                                int readLen = uart_read_bytes(UART2_PORT_NUMBER, uart2Data, bufferedSize, 100 / portTICK_PERIOD_MS); // pos + 1 to also read the pattern itself (\n)
                                if(readLen > 0) { // string looks like this: "???????,,,,0.00,0.00,050180,,,N*4C\r\n$GPRMC,000150.800,V,,,,,0.00,0.00,050180,,,N*4D\r\n$GPGGA,000225.800,,,,,0,0,,,M,,M,,*45\r\n" -> ~20 characters lost at start (@115200)
                                    uart2Data[readLen] = '\0'; // make sure the line is a standard string
                                    for(uint16_t a = 0; a < readLen; a++) {
                                        if(uart2Data[a] == '\0') { uart2Data[a] = 'X'; } // make sure that rubbish at start does not include string termination
                                    }
                                    //for(uint8_t rubbish = 0; rubbish < RUBBISH_LENGTH_AFTER_LIGHT_SLEEP; rubbish++) { uart2Data[rubbish] = 'X'; } // instead of rubbish that might include string terminations or line breaks, just add Xs
                                    char *uart2DataChar = (char *) uart2Data;
                                    //if(TRACKER_MODE == MODE_TESTRUN) { printf("1HzGPS: RAW: %s (%d)\n", uart2DataChar, bufferedSize); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                    
                                    // extract complete $GP messages
                                    bool foundRealStart = false;
                                    char *validPart = strchr(uart2DataChar, '$'); // substring: first start of $
                                    while(!foundRealStart) {
                                        if(validPart == NULL) { break; } // no more '$' in string
                                        else {
                                            if(strlen(validPart) < 5) { break; } // should be an error
                                            else {
                                                if(strncmp("$GP", validPart, 3) == 0) { foundRealStart = true; break; } // first time that after $ comes valid GPS message -> stop
                                                else { validPart = strchr(validPart + 1, '$'); } // + 1 because pointing on next char value
                                            }
                                        }
                                    }
                                    if(validPart != NULL) {
                                        if(foundRealStart) {
                                            uint16_t numberFullNMEAs = 0;
                                            char *temp = validPart;
                                            for(numberFullNMEAs=0; temp[numberFullNMEAs]; temp[numberFullNMEAs]=='$' ? numberFullNMEAs++ : *temp++);
                                            if(numberFullNMEAs != 2) {
                                                lastErrorId = 5; errorCnt++;
                                                if(TRACKER_MODE == MODE_TESTRUN) { printf("1HzGPS: WARNING: received %d msgs!\n", numberFullNMEAs); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                            }
                                            if(strncmp("$GPRMC", validPart, 6) != 0) { // first message (after corrupted ZDA) should be GPRMC
                                                // HAPPENS sometimes, mixed order (RMC, GGA, ZDA) but next time it's normal again, 500ms delay seems good
                                                lastErrorId = 6; errorCnt++;
                                                if(TRACKER_MODE == MODE_TESTRUN) { printf("1HzGPS: WARNING: rmc not first msg -> wait 500ms!\n"); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                                addExtraDelay = 500;
                                            }
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("1HzGPS: valid: %s", validPart); }  // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                            get_decode_result_t decodeRes = gps.gpsDecodeLine(&gpsData, validPart, readLen + 1);
                                            if(decodeRes == GPS_DECODE_RESULT_CRC_ERR) {
                                                if(TRACKER_MODE == MODE_TESTRUN) { printf("1HzGPS: crc decode line error\n"); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                                lastErrorId = 7; errorCnt++; 
                                            }
                                            else if(decodeRes == GPS_DECODE_RESULT_UNKNOWN_STATEMENT_ERR) {
                                                if(TRACKER_MODE == MODE_TESTRUN) { printf("1HzGPS: unknown statement decode line error\n"); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                                lastErrorId = 4; errorCnt++; 
                                            }
                                        }
                                        else { lastErrorId = 8; errorCnt++; }
                                    }
                                    else { lastErrorId = 13; errorCnt++; }

                                    // calculate timestamp
                                    bool couldUpdateTime = gps.updateUTCTimestamp(&gpsData); // returns true if time could be set (means timestamp is valid)

                                    // check if got a valid time for the first time
                                    if(couldUpdateTime
                                        && (!gotTimeAfterFix)
                                        && ((gpsData.parent.fix > 0) || (!hasValidTimestamp))) { // first time getting a valid GPS time! (only executed once) -> ONLY AFTER GETTING A FIX (or if hasValidTimestamp is not set yet)
                                        // --- BE QUICK HERE ---
                                        gotTimeAfterFix = true;
                                        
                                        // delay from UART: (max. 2 * 80 Byte*(8+2) * (1 / 115200 = 0.00868ms/bit) = 13.88ms)
                                        validTimeTimerFinished = false;

                                        // start timer
                                        uint32_t waitTimeUs = gpsData.parent.tim.thousand;
                                        // TODO: add estimateUARTSendTimeMs
                                        waitTimeUs = (1000 - waitTimeUs) * 1000;
                                        if(waitTimeUs == 0) { waitTimeUs = 1; }
                                        if(waitTimeUs > 900000) { waitTimeUs = 900000; } // WARNING: ADDING INACCURACY, but otherwise might take a second to wait -> GPS receiving gets fucked up
                                        if(esp_timer_start_once(validTimeTimer, waitTimeUs) == ESP_OK) {
                                            while(!validTimeTimerFinished) { ; } // busy waiting until full second
                                            tmElements_t timeStruct;
                                            breakTime(gpsData.parent.utcTimestamp + 1, timeStruct);
                                            if(device.rtc.setTimestamp(gpsData.parent.utcTimestamp + 1)) { 
                                                if(device.rtc.set(timeStruct.Hour, timeStruct.Minute, timeStruct.Second, timeStruct.Wday, timeStruct.Day, timeStruct.Month, timeStruct.Year)) {
                                                    hasValidTimestamp = true; // IMPORTANT: update global variable
                                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d 1HzGPS: updated RTC to: %u -> %d:%d:%d (wait: %u ms)\n", ((uint32_t) Timing::millis()), gpsData.parent.utcTimestamp + 1, timeStruct.Hour, timeStruct.Minute, timeStruct.Second, (waitTimeUs/1000)); }    
                                                }
                                                else { lastErrorId = 129; errorCnt++; }
                                            }
                                            else {lastErrorId = 130; errorCnt++; }
                                        }
                                    }
                                    //if(TRACKER_MODE == MODE_TESTRUN) { printf("%d 1HzGPS: %llds: %d.%d.%d %d:%d:%d, LAT: %f, LON: %f, SATS: %d, HDOP: %.2f, FIX: %d\n", ((uint32_t) Timing::millis()), (esp_timer_get_time()-ttffStartUs)/1000000ULL, gpsData.parent.date.day, gpsData.parent.date.month, gpsData.parent.date.year, gpsData.parent.tim.hour, gpsData.parent.tim.minute, gpsData.parent.tim.second, gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.sats_in_use, gpsData.parent.dop_h, gpsData.parent.fix); }
                                }
                                break; // go into light sleep
                            }
                        }
                    }        
                } // finished with getting gps data
                
                // read voltage
                uint16_t voltage = device.readSupplyVoltage();
                if(voltage < BATT_MIN_VOLTAGE) {
                    // PATH TESTED
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d 1HzGPS: POWER DOWN for %ds\n", ((uint32_t) Timing::millis()), FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
                    state = ST_PWRDWN;
                    stopIMU();
                    device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                    keepTrackingRunning = false;
                    break;
                }       
                else if((!hasValidTimestamp) && ((esp_timer_get_time() - ttffStartUs) / 1000000ULL > TRACKING_DATA_MODE_1HZ_NO_TIME_TIMEOUT_SECONDS)) { // no time after some seconds -> sleep for a while to not drain the battery
                    // PATH TESTED
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d 1HzGPS: didn't get time after %ds, sleep for %ds\n", ((uint32_t) Timing::millis()), TRACKING_DATA_MODE_1HZ_NO_TIME_TIMEOUT_SECONDS, TRACKING_DATA_MODE_1HZ_NO_TIME_SLEEP_SECONDS); }
                    device.enableInternalTimerInterruptInDeepSleep(TRACKING_DATA_MODE_1HZ_NO_TIME_SLEEP_SECONDS); // sleep for some time
                    keepTrackingRunning = false;
                    break;
                    // WARNING: IMU keeps running here!
                }
                else {
                    // get fifo data (101ms -> NEW: 16ms -> NEW NEW: 14ms)
                    t = Timing::millis();
                    uint16_t currentFifoLen = device.imu.getFIFOLength(); // get size of data in fifo

                    // still space to put all fifo data into the RAM
                    if((fifoDataPointer + currentFifoLen + GPS_HEADER_DATA_LEN) <= FIFO_DATA_LEN) {
                        // add header data
                        uint16_t fifoDataPointerOld = fifoDataPointer;
                        special1HzModeAddData(&gpsData, currentFifoLen, fifoData, &fifoDataPointer);

                        // read acc data into fifoData
                        if(currentFifoLen > 0) {
                            readFifoIMU(fifoData+fifoDataPointer, currentFifoLen);
                            if(currentFifoLen >= 996) { lastErrorId = 61; errorCnt++; } // data loss possible
                            fifoDataPointer += currentFifoLen;
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: pnt %d -> %d (MAX %d), %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataPointerOld, fifoDataPointer, FIFO_DATA_LEN, currentFifoLen, (Timing::millis() - t)); }
                        }
                    }
                    else { // no more space in RAM light sleep memory -> store RAM + newest data from FIFO into flash
                        if(!device.flashPowerOn(false)) { lastErrorId = 15; errorCnt++; } // turn on flash power already

                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: full: %d -> write flash\n", ((uint32_t) Timing::millis()), currentFifoLen); }
                        uint16_t fifoDataNewestPointer = 0;

                        // add header data to fifoDataNewest
                        special1HzModeAddData(&gpsData, currentFifoLen, fifoDataNewest, &fifoDataNewestPointer);

                        // read acc data into fifoDataNewest
                        if(currentFifoLen > 0) {
                            readFifoIMU(fifoDataNewest+fifoDataNewestPointer, currentFifoLen);
                            if(currentFifoLen >= 996) { lastErrorId = 65; errorCnt++; } // data loss possible
                            fifoDataNewestPointer += currentFifoLen;
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: fifonewest pnt %d (MAX 1024), read %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataNewestPointer, currentFifoLen, (Timing::millis() - t)); }
                        }
                
                        // read NVS pointer
                        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                        uint16_t flashBlockDeletedPointer = 0;
                        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                            flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                        #endif
                        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                            flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                        #endif
                        if(TRACKER_MODE == MODE_TESTRUN) {
                            printf("%d Flash: Need to store: %d Bytes fifoData + %d Bytes fifoDataNew\n", ((uint32_t) Timing::millis()), fifoDataPointer, fifoDataNewestPointer);
                            printf("%d Flash: before storing: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer, flashBlockDeletedPointer);
                        }

                        // store data
                        uint32_t timeNow = ((uint32_t) Timing::millis());
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO space left before storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                        sequential_write_status_t writeStatus = device.flash.fifoPush(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, fifoData, fifoDataPointer, fifoDataNewest, fifoDataNewestPointer, NULL, 0, MOCK_FLASH_WRITES);
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: after storing: flashPointer: %d, flashOffsetPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer); }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO space left after storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                        timeNow = ((uint32_t) Timing::millis()) - timeNow;
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: storage took: %dms\n", ((uint32_t) Timing::millis()), timeNow); }
                        if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full -> go into special WIFI TRANSMISSION MODE!
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO full, stop IMU -> go into ST_MEMFULL state in 5 seconds\n", ((uint32_t) Timing::millis())); }
                            stopIMU();
                            memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY; // after memory full, try frequently to get rid of data
                            state = ST_MEMFULL;
                            if(!device.flashPowerOff(false)) { lastErrorId = 26; errorCnt++; } // important!
                            fifoDataPointer = 0; // reset pointer
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }
                            device.enableInternalTimerInterruptInDeepSleep(5); // immediately try to get rid of the data
                            keepTrackingRunning = false;
                            break;
                        }
                        else { // MT29_SEQ_WRITE_STATUS_SUCCESS or MT29_SEQ_WRITE_STATUS_ERROR, also update pointers in case of ERROR
                            // update pointer
                            device.nvsWriteUINT32x2(NVS_FLASH_WRITE_POINTER, flashPointer, NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, flashOffsetPointer);
                            if(writeStatus == MT29_SEQ_WRITE_STATUS_BUFFER_ERROR) { lastErrorId = 16; errorCnt++; }
                            else if(writeStatus == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR) { lastErrorId = 29; errorCnt++; }
                            // fifo data in ram
                            fifoDataPointer = 0; // reset pointer
                            // turn off flash power (3ms)
                            if(!device.flashPowerOff(false)) { lastErrorId = 101; errorCnt++; }
                            // check if everything took too long (fifo full already, most probably missed the acc interrupt)
                            uint16_t fifoLenAfterWifi = device.imu.getFIFOLength();
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO LEN before sleep: %d (reset when >= %d)\n", ((uint32_t) Timing::millis()), fifoLenAfterWifi, ACC_INTERRUPT_WATERMARK); }
                            if(fifoLenAfterWifi >= ACC_INTERRUPT_WATERMARK) { // fifo full again -> reset it
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO RESET! DATA lost!\n", ((uint32_t) Timing::millis())); }
                                if(!device.imu.resetFIFO()) { lastErrorId = 31; errorCnt++; } // empty FIFO, do not read
                                fifoDataPointer = 0; // reset RAM data
                            }
                            // print error count
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }

                            // stored data -> check if it's time to sleep now (so that data is stored)
                            if(((!espNowForceTrackingOnce) && isTimeToGoToBed()) // time to go to bed (no force tracking mode)
                                || ((espNowForceTrackingOnce) && ((esp_timer_get_time() - ttffStartUs) / 1000000ULL > COMMAND_BYTE_FORCE_TRACKING_DURATION_SECONDS))) { // OR force tracking mode and time of force is over
                                // PATH TESTED
                                state = ST_NIGHT_TIME;
                                stopIMU();
                                if(DO_THE_BLINK) { device.blinkTimes(5, B_RED); }
                                #if NIGHTTIME_MODE == NIGHTTIME_MODE_ONLY_SLEEP
                                    if(!device.rtc.setDailyInterrupt(NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE)) { lastErrorId = 103; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                                    device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
                                #endif
                                #if NIGHTTIME_MODE == NIGHTTIME_MODE_TRY_DATATRANS
                                    // PATH TESTED
                                    if(!device.rtc.setRegularInterrupt(NIGHTTIME_MODE_TRY_DATATRANS_WAKEUP_SECONDS)) { lastErrorId = 145; errorCnt++; }
                                    device.enableInternalTimerInterruptInDeepSleep(2); // immediately try a data transmission -> set this interrupt as well
                                    device.enableRTCInterruptInDeepSleep();
                                #endif
                                keepTrackingRunning = false;
                                break;
                            }
                        } 
                    }
                }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("\n"); }
                if(addExtraDelay > 0) { device.delay(addExtraDelay); addExtraDelay = 0; } // in case of unsynced GPS messages
                if(TRACKING_DATA_MODE_1HZ_GPS_BLINK && DO_THE_BLINK) { device.ledGreenOff(); device.ledRedOff(); }
                uart_flush(UART2_PORT_NUMBER);
                device.enableUart2InterruptInLightSleep();
                esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
                device.lightSleep();
            }
            // end of big while loop
            free(uart2Data);
            free(fifoData);
            device.gpioBOff();
            if(TRACKING_DATA_MODE_1HZ_GPS_BLINK && DO_THE_BLINK) { device.ledGreenOff(); device.ledRedOff(); }
            espNowForceTrackingOnce = false; // if this was set before
        }
    }
}

uint16_t imuHighFrequencyLightSleepTrackingModeHeaderLength() {
    return 27;
}

void imuHighFrequencyLightSleepTrackingModeAddData(uint16_t voltage, uint16_t currentFifoLen, uint8_t *buffer, uint16_t *bufferPointer) { 
    bool error = false;
    uint32_t timestamp = device.rtc.getTimestamp(error);
    if(error) { lastErrorId = 149; errorCnt++; }
    int16_t temperature = 0, temperatureBmx;
    uint16_t temperatureBmxRaw = 0;
    uint32_t pressure = 0;
    uint32_t humidity = 0;

    if(!device.imu.getTemperatureRaw(temperatureBmxRaw)) { lastErrorId = 158; errorCnt++; }
    temperatureBmx = device.imu.toCelsiusx100(temperatureBmxRaw);
    if(device.baro.getResults()) { // normally waits up to 31ms, but measurement was triggered before lightsleep
        temperature = device.baro.getTemperature(error);
        if(error) { lastErrorId = 20; errorCnt++; }
        pressure = device.baro.getPressure(error);
        if(error) { lastErrorId = 21; errorCnt++; }
        humidity = device.baro.getHumidity(error);
        if(error) { lastErrorId = 22; errorCnt++; }
    }

    HelperBits::addData1_AndIncrementPointer(0x12, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(0x34, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(0x5E, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(timestamp, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(0xFF, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(lastErrorId, buffer, bufferPointer);
    HelperBits::addData2_AndIncrementPointer(errorCnt, buffer, bufferPointer);
    HelperBits::addData2_AndIncrementPointer((uint16_t) voltage, buffer, bufferPointer);
    HelperBits::addData2Signed_AndIncrementPointer(temperature, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(humidity, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(pressure, buffer, bufferPointer);
    HelperBits::addData2Signed_AndIncrementPointer(temperatureBmx, buffer, bufferPointer);
    HelperBits::addData2_AndIncrementPointer(currentFifoLen, buffer, bufferPointer);
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: UTC %d, TEMP %d/%d, PRESS %d, HUM %d\n", ((uint32_t) Timing::millis()), timestamp, temperature, temperatureBmx, pressure, humidity); } 
}

void imuHighFrequencyLightSleepTrackingMode() {
    const uint16_t FIFO_DATA_LEN = 1024 * 16;
    bool keepTrackingRunning = true;
    bool errorDuringInit = false;
    uint16_t fifoDataPointer = 0;
    int64_t trackStartUs = 0;

    if(!espNowForceTrackingOnce) { espNowForceTrackingOnceCnt = 0; } // if this is a regular start: reset the counter

    i2c.begin(I2C_FREQ_HZ_1MHZ); // USING 1MHZ
    
    if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // do not even start when voltage is low
        if(TRACKER_MODE == MODE_TESTRUN) { printf("imuHD: POWER DOWN before start for %ds\n", FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
        state = ST_PWRDWN;
        stopIMU();
        espNowForceTrackingOnce = false; // stop force mode when coming back after under voltage 
        device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
    }
    else if((!espNowForceTrackingOnce) && isTimeToGoToBed()) { // do not even start when it's bed time
        // TESTED PATH
        state = ST_NIGHT_TIME;
        stopIMU();
        if(DO_THE_BLINK) { device.blinkTimes(5, B_RED); }
        #if NIGHTTIME_MODE == NIGHTTIME_MODE_ONLY_SLEEP
            if(!device.rtc.setDailyInterrupt(NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE)) { lastErrorId = 139; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
            device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
        #endif
        #if NIGHTTIME_MODE == NIGHTTIME_MODE_TRY_DATATRANS
            // TESTED PATH
            if(!device.rtc.setRegularInterrupt(NIGHTTIME_MODE_TRY_DATATRANS_WAKEUP_SECONDS)) { lastErrorId = 144; errorCnt++; }
            device.enableInternalTimerInterruptInDeepSleep(2); // immediately try a data transmission -> set this interrupt as well
            device.enableRTCInterruptInDeepSleep();
        #endif
    }
    else {
        if(!device.initDataNVS()) { lastErrorId = 14; errorCnt++; }
        if((TRACKER_MODE == MODE_TESTRUN) && espNowForceTrackingOnce) { printf("imuHD: FORCE TRACKING ONCE\n"); }

        // try one time data transmission
        bool somethingTransmitted = false;
        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
        if(!device.flashPowerOn(true)) { lastErrorId = 137; errorCnt++; } // turn on flash power already (10ms)
        uint32_t timeNow = ((uint32_t) Timing::millis());
        uint8_t commandByte = 0; // just for putting device back into activation state
        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
        // esp now data transmission (FORCE mode! but only executed if bytes to transmit)
        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
        somethingTransmitted = dataTransmissionEspNow(true, ESPNOW_MIN_BYTES_TO_TRANSMIT, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte);
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
        #endif
        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
        // wifi data transmission (FORCE mode!)
        somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
        #endif
        if(!device.flashPowerOff(true)) { lastErrorId = 138; errorCnt++; } // turn off flash
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: something transmitted: %d\n", somethingTransmitted); }
        if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }

        // check if tag shall be deactivated
        if(commandByte == COMMAND_BYTE_DEACTIVATE) {
            if(TRACKER_MODE == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO DE-ACTIVATE\n"); }
            stopIMU();
            espNowForceTrackingOnce = false; // reset everything, also this
            resetActivation(); // NVS already initialized
            device.enableInternalTimerInterruptInDeepSleep(1);
        }
        else {
            trackStartUs = esp_timer_get_time(); // start timer
            uint8_t *fifoData = (uint8_t*) malloc(FIFO_DATA_LEN);
            if(fifoData == NULL) { errorDuringInit = true; }

            // NEW: initalize barometer
            if(!device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { lastErrorId = 117; errorCnt++; } // 4ms

            if(errorDuringInit) { // SHOULD NOT HAPPEN, but could happen if GPS is not connected -> sleep for FIRST_UNDER_VOLTAGE_SLEEP_TIME and try again afterwards 
                lastErrorId = 128; errorCnt++;
                free(fifoData);
                device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME);
                return;
            }

            if(!device.baro.performMeasurement()) { lastErrorId = 147; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
            device.imu.resetFIFO(); // reset fifo to start fresh
            device.enableAccInterruptInDeepSleep();
            esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
            device.lightSleep();

            while(keepTrackingRunning) {
                // read voltage
                uint16_t voltage = device.readSupplyVoltage();
                if(voltage < BATT_MIN_VOLTAGE) {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d imuHD: POWER DOWN for %ds\n", ((uint32_t) Timing::millis()), FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
                    state = ST_PWRDWN;
                    stopIMU();
                    device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                    keepTrackingRunning = false;
                    break;
                }
                else {
                    // get fifo data (101ms -> NEW: 16ms -> NEW NEW: 14ms)
                    t = Timing::millis();
                    uint16_t currentFifoLen = device.imu.getFIFOLength(); // get size of data in fifo

                    // still space to put all fifo data into the RAM
                    if((fifoDataPointer + currentFifoLen + imuHighFrequencyLightSleepTrackingModeHeaderLength()) <= FIFO_DATA_LEN) {
                        // add header data
                        uint16_t fifoDataPointerOld = fifoDataPointer;
                        imuHighFrequencyLightSleepTrackingModeAddData(voltage, currentFifoLen, fifoData, &fifoDataPointer);

                        // read acc data into fifoData
                        if(currentFifoLen > 0) {
                            readFifoIMU(fifoData+fifoDataPointer, currentFifoLen);
                            if(currentFifoLen >= 996) { lastErrorId = 61; errorCnt++; } // data loss possible
                            fifoDataPointer += currentFifoLen;
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: pnt %d -> %d (MAX %d), %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataPointerOld, fifoDataPointer, FIFO_DATA_LEN, currentFifoLen, (Timing::millis() - t)); }
                        }
                    }
                    else { // no more space in RAM light sleep memory -> store RAM + newest data from FIFO into flash
                        if(!device.flashPowerOn(false)) { lastErrorId = 15; errorCnt++; } // turn on flash power already
                        if(DO_THE_BLINK) { device.ledRedOn(); }

                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: full: %d -> write flash\n", ((uint32_t) Timing::millis()), currentFifoLen); }
                        uint16_t fifoDataNewestPointer = 0;

                        // add header data to fifoDataNewest
                        imuHighFrequencyLightSleepTrackingModeAddData(voltage, currentFifoLen, fifoDataNewest, &fifoDataNewestPointer);

                        // read acc data into fifoDataNewest
                        if(currentFifoLen > 0) {
                            readFifoIMU(fifoDataNewest+fifoDataNewestPointer, currentFifoLen);
                            if(currentFifoLen >= 996) { lastErrorId = 65; errorCnt++; } // data loss possible
                            fifoDataNewestPointer += currentFifoLen;
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: fifonewest pnt %d (MAX 1024), read %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataNewestPointer, currentFifoLen, (Timing::millis() - t)); }
                        }
                
                        // read NVS pointer
                        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                        uint16_t flashBlockDeletedPointer = 0;
                        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                            flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                        #endif
                        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                            flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                        #endif
                        if(TRACKER_MODE == MODE_TESTRUN) {
                            printf("%d Flash: Need to store: %d Bytes fifoData + %d Bytes fifoDataNew\n", ((uint32_t) Timing::millis()), fifoDataPointer, fifoDataNewestPointer);
                            printf("%d Flash: before storing: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer, flashBlockDeletedPointer);
                        }

                        // store data
                        uint32_t timeNow = ((uint32_t) Timing::millis());
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO space left before storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                        sequential_write_status_t writeStatus = device.flash.fifoPush(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, fifoData, fifoDataPointer, fifoDataNewest, fifoDataNewestPointer, NULL, 0, MOCK_FLASH_WRITES);
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: after storing: flashPointer: %d, flashOffsetPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer); }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO space left after storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                        timeNow = ((uint32_t) Timing::millis()) - timeNow;
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: storage took: %dms\n", ((uint32_t) Timing::millis()), timeNow); }
                        if(DO_THE_BLINK) { device.ledRedOff(); }
                        if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full -> go into special WIFI TRANSMISSION MODE!
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO full, stop IMU -> go into ST_MEMFULL state in 5 seconds\n", ((uint32_t) Timing::millis())); }
                            stopIMU();
                            memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY; // after memory full, try frequently to get rid of data
                            state = ST_MEMFULL;
                            if(!device.flashPowerOff(false)) { lastErrorId = 26; errorCnt++; } // important!
                            fifoDataPointer = 0; // reset pointer
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }
                            device.enableInternalTimerInterruptInDeepSleep(5); // immediately try to get rid of the data
                            keepTrackingRunning = false;
                            break;
                        }
                        else { // MT29_SEQ_WRITE_STATUS_SUCCESS or MT29_SEQ_WRITE_STATUS_ERROR, also update pointers in case of ERROR
                            // update pointer
                            device.nvsWriteUINT32x2(NVS_FLASH_WRITE_POINTER, flashPointer, NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, flashOffsetPointer);
                            if(writeStatus == MT29_SEQ_WRITE_STATUS_BUFFER_ERROR) { lastErrorId = 16; errorCnt++; }
                            else if(writeStatus == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR) { lastErrorId = 29; errorCnt++; }
                            // fifo data in ram
                            fifoDataPointer = 0; // reset pointer
                            // turn off flash power (3ms)
                            if(!device.flashPowerOff(false)) { lastErrorId = 101; errorCnt++; }
                            // check if everything took too long (fifo full already, most probably missed the acc interrupt)
                            uint16_t fifoLenAfterWifi = device.imu.getFIFOLength();
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO LEN before sleep: %d (reset when >= %d)\n", ((uint32_t) Timing::millis()), fifoLenAfterWifi, ACC_INTERRUPT_WATERMARK); }
                            if(fifoLenAfterWifi >= ACC_INTERRUPT_WATERMARK) { // fifo full again -> reset it
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d FIFO: FIFO RESET! DATA lost!\n", ((uint32_t) Timing::millis())); }
                                if(!device.imu.resetFIFO()) { lastErrorId = 31; errorCnt++; } // empty FIFO, do not read
                                fifoDataPointer = 0; // reset RAM data
                            }
                            // print error count
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }

                            // stored data -> check if it's time to sleep now (so that data is stored)
                            if(((!espNowForceTrackingOnce) && isTimeToGoToBed()) // time to go to bed (no force tracking mode)
                                || ((espNowForceTrackingOnce) && ((esp_timer_get_time() - trackStartUs) / 1000000ULL > COMMAND_BYTE_FORCE_TRACKING_DURATION_SECONDS))) { // OR force tracking mode and time of force is over
                                // PATH TESTED
                                state = ST_NIGHT_TIME;
                                stopIMU();
                                if(DO_THE_BLINK) { device.blinkTimes(5, B_RED); }
                                #if NIGHTTIME_MODE == NIGHTTIME_MODE_ONLY_SLEEP
                                    if(!device.rtc.setDailyInterrupt(NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE)) { lastErrorId = 103; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                                    device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
                                #endif
                                #if NIGHTTIME_MODE == NIGHTTIME_MODE_TRY_DATATRANS
                                    if(!device.rtc.setRegularInterrupt(NIGHTTIME_MODE_TRY_DATATRANS_WAKEUP_SECONDS)) { lastErrorId = 145; errorCnt++; }
                                    device.enableInternalTimerInterruptInDeepSleep(2); // immediately try a data transmission -> set this interrupt as well
                                    device.enableRTCInterruptInDeepSleep();
                                #endif
                                keepTrackingRunning = false;
                                break;
                            }
                        } 
                    }
                }
                if(TRACKER_MODE == MODE_TESTRUN) { printf("\n"); }
                if(!device.baro.performMeasurement()) { lastErrorId = 148; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
                device.enableAccInterruptInDeepSleep();
                device.lightSleep();
            }
            // end of big while loop
            free(fifoData);
            espNowForceTrackingOnce = false; // if this was set before
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
        device.delay(2000);
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

extern "C" void app_main() {
    while(1) {
        if((TRACKER_MODE == MODE_TESTRUN) || (TRACKER_MODE == MODE_PRODUCTIVE)) {
            if((TRACKER_MODE == MODE_TESTRUN) && (state != ST_TRACK)) { printf("-----\nState: %d\n", state); }
            /** ---------------- VERY FIRST ENTRY POINT AFTER HARD RESET (RTC memory = reset), only executed ONCE, NEVER after Deepsleep! ---------------- */
            if(state == ST_FIRST_START_HARD_RESET) { // custom wake stub not running -> no fifo
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: HARD RESET!\n"); }
                device.disableWakeStubNoBootIfVoltageLow(); // disable no boot if voltage low (do here, only needed once!)
                if(DO_THE_BLINK) { device.blinkTimes(3, B_RED); }

                // full RF calibration in case ESP NOW IS USED
                #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                    #if ESPNOW_CUSTOM_RF_CALIBRATION == true
                        handleCustomRFCalibration();
                    #endif
                #endif

                // keeping power on values (PIN_POWER, PIN_LED_RED) in deep sleep
                device.keepSensorPowerOnInDeepSleep();

                // RTC: disable clock out (lower deep sleep current)
                i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
                if(!device.rtc.disableClockOut()) { lastErrorId = 62; errorCnt++; }
                device.delay(10);

                // RTC: get current timestamp
                uint32_t timestamp = device.rtc.getTimestamp(error);
                if(error) { lastErrorId = 41; errorCnt++; }

                // check NVS if already activated & timestamp is valid, print write pointer also when in PRODUCTIVE
                if(!device.initDataNVS()) { lastErrorId = 42; errorCnt++; }
                uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                uint8_t myMac[6] = { 0 };
                esp_efuse_mac_get_default(myMac);
                printf("Reset: FIRMWARE: V%d, CONFIG: %s V%d\n", WILDFI_SOFTWARE_VERSION, WILDFI_CONFIG_NAME, WILDFI_CONFIG_VERSION);
                #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                    uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER);
                    uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
                    printf("Reset: FLASH: w %d.%d, s %d.%d, HEAP: %d\n", flashPointer, flashOffsetPointer, sendPagePointer, sendPageOffsetPointer, heap_caps_get_free_size(MALLOC_CAP_8BIT));
                #endif
                #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                    uint16_t sendNextBlockPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                    uint16_t sendNextHalfBlockPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER); 
                    printf("Reset: FLASH: %d.%d, sb %d.%d, HEAP: %d\n", flashPointer, flashOffsetPointer, sendNextBlockPointer, sendNextHalfBlockPointer, heap_caps_get_free_size(MALLOC_CAP_8BIT));
                #endif
                printf("Reset: MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
                if(device.getLastResetReason() == ESP_RST_BROWNOUT) { lastErrorId = 43; errorCnt++; }

                // activation state
                if(ACTIVATION_MODE == ACTIVATION_MODE_SKIP) { isActivated = true; } // store state in RTC -> always activated
                else if(ACTIVATION_MODE == ACTIVATION_MODE_STORE_PERMANENTLY) {
                    uint16_t activated = device.nvsReadUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI);
                    isActivated = (activated > 0); // store state in RTC
                }
                else if(ACTIVATION_MODE == ACTIVATION_MODE_ON_EVERY_START) { isActivated = false; } // store state in RTC -> not activated after reset

                if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 98; errorCnt++; } // disable, just in case something weird happened

                hasValidTimestamp = (timestamp > 1600000000); // store state in RTC, anything below 13.09.2020 12:26 would be wrong
                if(hasValidTimestamp || SKIP_GET_TIME) { // some brownout or other reset of MCU -> time still ok
                    setNextDataTransmissionTimestamp(false, timestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
                    if(isActivated) { // also already activated -> start immediately
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp ok %d (skip %d) and activated -> START!\n", hasValidTimestamp, SKIP_GET_TIME); }
                        state = ST_START;
                    }
                    else { // time okay, but not activated -> go to activation
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp ok %d (skip %d), NOT activated (mode %d) -> activation state!\n", hasValidTimestamp, SKIP_GET_TIME, ACTIVATION_MODE); }
                        if(ACTIVATION_SOURCE == ACTIVATION_BY_WIFI) {
                            if(!device.rtc.setHourlyInterrupt(ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE)) { lastErrorId = 95; errorCnt++; } // time valid, so better activate the rtc interrupt
                        }
                        state = ST_WAIT_FOR_ACTIVATION;
                    }
                }
                else { // time not okay, move to time state
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp not valid, activation = %d (mode %d)!\n", isActivated, ACTIVATION_MODE); }
                    state = ST_GET_TIME;
                }
                device.enableInternalTimerInterruptInDeepSleep(SLEEP_TIME_AFTER_START); // restart system into next state
            }
            /** ---------------- GET TIME STATE ---------------- */
            else if(state == ST_GET_TIME) {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: try getting time! V = %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, go to power down state
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: undervoltage (%d)! Don't scan!\n", device.readSupplyVoltageFromWakeStub()); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                }
                else {
                    // try to get time over wifi or GPS
                    if(device.initWiFi()) {
                        uint8_t foundArrayId = 0;
                        uint8_t foundOnChannel = 0;
                        bool connectionTimeout = false;
                        uint32_t scanStartTime = ((uint32_t) Timing::millis());
                        i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
                        if(device.scanForWiFisOn1and6and11and13WithPriority((TRACKER_MODE == MODE_TESTRUN), TIME_WIFI_SSIDS, TIME_WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, TIME_WIFI_OUTPUT_POWER, 120, 500)) { 
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500); }
                            if(foundOnChannel > 0) { // found wifi, try to connect
                                if(foundArrayId == 0) { // enter special download mode to force data transmission, afterwards delete memory and sleep forever
                                    device.disconnectAndStopWiFi();
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: FOUND FORCE DOWNLOAD WIFI (chan %d, index %d)\n", foundOnChannel, foundArrayId); }
                                    state = ST_FORCE_DOWNLOAD;
                                    device.enableInternalTimerInterruptInDeepSleep(3);
                                }
                                else { // just get time
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
                                            lastErrorId = 38; errorCnt++;
                                        }
                                        else { // connected to wifi
                                            uint32_t timestampUTC = 0;
                                            uint16_t millisecondsUTC = 0;
                                            if(!device.getNTPTimestampUTC(true, timestampUTC, millisecondsUTC, 7000, "pool.ntp.org")) { // will block, will set RTC time
                                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: UTC get time error!\n"); }
                                                lastErrorId = 39; errorCnt++;
                                            }
                                            else {
                                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: set %d + 1, UTC milliseconds: %d -> waited for %dms\n", timestampUTC, millisecondsUTC, 1000 - millisecondsUTC); }
                                                hasValidTimestamp = true;
                                                setNextDataTransmissionTimestamp(false, timestampUTC+1, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
                                            }
                                        }
                                        device.disconnectAndStopWiFi();
                                    }
                                    else { lastErrorId = 37; errorCnt++; device.disconnectAndStopWiFi(); }
                                    // check result of getting timestamp attempt
                                    if(hasValidTimestamp) {
                                        if(isActivated) {
                                            state = ST_START;
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: all done and already activated -> MOVE TO START in %ds!\n", TIME_SLEEP_AFTER_GOT_TIME); }
                                        }
                                        else {
                                            state = ST_WAIT_FOR_ACTIVATION;
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: all done -> move to ACTIVATION in %ds!\n", TIME_SLEEP_AFTER_GOT_TIME); }
                                        }
                                        if(DO_THE_BLINK) { device.blinkTimes(6, B_GREEN); }
                                        device.enableInternalTimerInterruptInDeepSleep(TIME_SLEEP_AFTER_GOT_TIME); // restart in x seconds and move to activation
                                    }
                                    else {
                                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: wifi was seen, but still something missing (pw wrong, no internet) -> SLEEP for %ds\n", TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); }
                                        if(!USE_GPS_TO_GET_TIME_AT_BEGINNING) { device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); }
                                        else { tryGetTimeOverGPS(); }
                                    }
                                }
                            }
                            else {
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: wifi NOT found\n"); }
                                device.disconnectAndStopWiFi();
                                if(!USE_GPS_TO_GET_TIME_AT_BEGINNING) { device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); } // sleep some time and try again afterwards   
                                else { tryGetTimeOverGPS(); }
                            }
                        }
                        else {
                            lastErrorId = 32; errorCnt++; device.disconnectAndStopWiFi();
                            if(!USE_GPS_TO_GET_TIME_AT_BEGINNING) { device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); }
                            else { tryGetTimeOverGPS(); }
                        }
                    }
                    else {
                        lastErrorId = 33; errorCnt++;
                        if(!USE_GPS_TO_GET_TIME_AT_BEGINNING) { device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); }
                        else { tryGetTimeOverGPS(); }
                    }
                }
            }
            /** ---------------- WAIT FOR ACTIVATION STATE ---------------- */
            else if(state == ST_WAIT_FOR_ACTIVATION) { // custom wake stub not running
                i2c.begin(I2C_FREQ_HZ_400KHZ);
                #if ACTIVATION_SOURCE == ACTIVATION_BY_ESPNOW
                    if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, stay in that mode but sleep
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: Undervoltage (%d)! Don't scan, sleep for %d!\n", device.readSupplyVoltageFromWakeStub(), FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); }
                        device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME);
                    }
                    else {
                        #if ESPNOW_CUSTOM_RF_CALIBRATION == true
                            handleCustomRFCalibration();
                        #endif
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: NOT ACTIVATED (src: ESPNOW)!\n"); }
                        if(!device.initESPNOWStationary(ESPNOW_LONG_RANGE, ESPNOW_OUTPUT_POWER, true, ESPNOW_DATA_RATE)) { lastErrorId = 120; errorCnt++; } // 23ms
                        if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 107; errorCnt++; } // 0ms
                        uint8_t commandByte = 0; // currently unused
                        bool gatewaySeen = gatewaySeenEspNow(&commandByte);
                        if(gatewaySeen && (commandByte == COMMAND_BYTE_ACTIVATE)) {
                            // after tag around message: confirm with got activated message
                            isActivated = true;
                            uint8_t data[ESPNOW_META_MSG_GOT_ACTIVATED_LEN] = { 0 };
                            data[0] = ESPNOW_META_MSG_GOT_ACTIVATED;
                            device.broadcastESPNOWData(data, ESPNOW_META_MSG_GOT_ACTIVATED_LEN); // spit it out
                        }
                        device.stopESPNOW(); // 5ms
                        esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                        if(isActivated) {
                            if(ACTIVATION_MODE == ACTIVATION_MODE_STORE_PERMANENTLY) {
                                if(!device.initDataNVS()) { lastErrorId = 125; errorCnt++; }
                                else {
                                    if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 1)) { lastErrorId = 126; errorCnt++; } // write activation into NVS
                                }
                            }
                            state = ST_START;
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: all done -> move to START in 1s!\n"); }
                            device.enableInternalTimerInterruptInDeepSleep(1); // restart in 1 second and start
                        }
                        else {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: NO, seen: %d, cmd: 0x%02X -> SLEEP %ds\n", gatewaySeen, commandByte, ACTIVATION_BY_ESPNOW_RETRY_TIME_SECONDS); }
                            device.enableInternalTimerInterruptInDeepSleep(ACTIVATION_BY_ESPNOW_RETRY_TIME_SECONDS);
                        }
                    }
                #endif
                #if ACTIVATION_SOURCE == ACTIVATION_BY_WIFI  
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: NOT ACTIVATED (src: WIFI)!\n"); }
                    if(device.getWakeUpReason() == BY_EXT0) { // wake up by RTC -> was before in ACTIVATION STATE!
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: booted through RTC interrupt -> CLEAR\n"); }
                        if(!device.rtc.resetInterruptFlags()) { lastErrorId = 97; errorCnt++; }
                    }
                    else { // can only be BY_TIMER, first time entering activation state -> activate hourlyInterrupt
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: first time activation -> enable hourlyInterrupt!\n"); }
                        if(!device.rtc.setHourlyInterrupt(ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE)) { // DANGER: interrupt pin not cleared automatically here, never forget to deactivate that!
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: could not set RTC interrupt!\n"); }
                            lastErrorId = 35; errorCnt++;
                        }
                    }
                    if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, stay in that mode but sleep for an hour or more -> needs to be here because otherwise setHourlyInterrupt might not have been set!
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: Undervoltage (%d)! Don't scan, sleep on RTC interrupt!\n", device.readSupplyVoltageFromWakeStub()); }
                        device.enableRTCInterruptInDeepSleep(); // sleep for an hour or more
                    }
                    else {
                        uint8_t currentHour = device.rtc.getHours(i2cError);
                        if(!i2cError) {
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: Current hour: %d, wake on minute %d, wake on modulo %d, wake-up reason %d\n", currentHour, ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE, ACTIVATION_WAKE_UP_ON_MODULO_HOURS, device.getWakeUpReason()); }
                            if((currentHour % ACTIVATION_WAKE_UP_ON_MODULO_HOURS == 0) || (device.getWakeUpReason() != BY_EXT0)) { // if BY_TIMER means first scan ever -> try it immediately                
                                if(device.initWiFi()) {
                                    uint8_t foundArrayId = 0;
                                    uint8_t foundOnChannel = 0;
                                    uint32_t scanStartTime = ((uint32_t) Timing::millis());
                                    if(DO_THE_BLINK) { device.ledGreenOn(); } // visual feedback during activation scan
                                    if(device.scanForWiFisOn1and6and11(ACTIVATION_WIFI_SSIDS, 1, &foundArrayId, &foundOnChannel, ACTIVATION_WIFI_OUTPUT_POWER, 120, 500)) { 
                                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500); }
                                        if(foundOnChannel > 0) { // found wifi, set activation = true
                                            device.disconnectAndStopWiFi();
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: activation wifi found (chan %d, index %d), isActivated: %d, hasValidTimestamp: %d\n", foundOnChannel, foundArrayId, isActivated, hasValidTimestamp); }
                                            if(ACTIVATION_MODE == ACTIVATION_MODE_ON_EVERY_START) { isActivated = true; } // just store in RTC variable, do not store in NVS
                                            else {
                                                if(!device.initDataNVS()) { lastErrorId = 91; errorCnt++; }
                                                else {
                                                    if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 1)) { lastErrorId = 92; errorCnt++; } // write activation into NVS
                                                    else { isActivated = true; } // now is activated!
                                                }
                                            }

                                            if(isActivated) {
                                                state = ST_START;
                                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: all done -> disable RTC interrupt, move to START in 1s!\n"); }
                                                if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 99; errorCnt++; } // IMPORTANT: disable RTC interrupt (otherwise int pin driven permanently after one hour)
                                                device.enableInternalTimerInterruptInDeepSleep(1); // restart in 1 second and start
                                            }
                                            else {
                                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: wifi was seen, but other error -> SLEEP (rtc)\n"); }
                                                device.enableRTCInterruptInDeepSleep();
                                            }
                                        }
                                        else {
                                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: activation wifi NOT found -> SLEEP (rtc)\n"); }
                                            device.disconnectAndStopWiFi();
                                            device.enableRTCInterruptInDeepSleep();
                                        }  
                                    }
                                    else { device.disconnectAndStopWiFi(); lastErrorId = 93; errorCnt++; device.enableRTCInterruptInDeepSleep(); }
                                    if(DO_THE_BLINK) { device.ledGreenOff(); } // visual feedback during activation scan
                                }
                                else { lastErrorId = 94; errorCnt++; device.enableRTCInterruptInDeepSleep(); }
                            }
                            else {
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: DON'T SCAN -> SLEEP\n"); }
                                device.enableRTCInterruptInDeepSleep(); // sleep again
                            }
                        }
                        else { lastErrorId = 96; errorCnt++; device.enableRTCInterruptInDeepSleep(); }
                    }
                #endif
            }
            /** ---------------- START STATE ---------------- */
            else if(state == ST_START) {
                // blink a couple of times
                if(DO_THE_BLINK) { device.blinkTimes(5, B_BOTH); }

                // init BMX
                i2c.begin(I2C_FREQ_HZ_1MHZ);
                startIMU(true); // reading trim data here
           
                // move to next state
                state = ST_TRACK;

                if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_ACC_ONLY) { device.enableAccInterruptInDeepSleep(); }
                else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_1HZ_GPS_AND_ACC) { device.enableInternalTimerInterruptInDeepSleep(1); }
                else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
            }
            /** ---------------- POWER DOWN STATE (IMU stopped!) ---------------- */
            else if(state == ST_PWRDWN) {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("(PWR_DWN) V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() <= BATT_RESTART_VOLTAGE) { // voltage still too low for restarting
                    // PATH TESTED
                    uint8_t commandByte = 0;
                    if(POWER_DOWN_MODE_DATA_TRANS_WHEN_VOLTAGE_HALF_OK && (BATT_RESTART_VOLTAGE > BATT_MIN_VOLTAGE)) {
                        if(device.readSupplyVoltageFromWakeStub() > (BATT_MIN_VOLTAGE + ((BATT_RESTART_VOLTAGE - BATT_MIN_VOLTAGE) / 2))) { // voltage recovered half
                            // try one time data transmission
                            if(!device.initDataNVS()) { lastErrorId = 152; errorCnt++; }
                            i2c.begin(I2C_FREQ_HZ_400KHZ); // DO NOT use 1MHz
                            bool somethingTransmitted = false;
                            uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                            if(!device.flashPowerOn(true)) { lastErrorId = 150; errorCnt++; } // turn on flash power already (10ms)
                            uint32_t timeNow = ((uint32_t) Timing::millis());

                            #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                            // esp now data transmission (FORCE mode! minBytes to transmit = 0 to send out a tag around msg)
                            uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                            somethingTransmitted = dataTransmissionEspNow(true, 0, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte);
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Pwrdwn: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            #endif
                            #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                            // wifi data transmission (FORCE mode!)
                            somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Pwrdwn: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            #endif
                            if(!device.flashPowerOff(false)) { lastErrorId = 151; errorCnt++; } // turn off flash
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Pwrdwn: something transmitted: %d\n", somethingTransmitted); }
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Pwrdwn: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                        }
                    }
                    if(commandByte == COMMAND_BYTE_DEACTIVATE) {
                        // PATH TESTED
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Pwrdwn: RECEIVED CMD TO DE-ACTIVATE\n"); }
                        resetActivation(); // NVS already initialized
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else {
                        // PATH TESTED
                        device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                    }
                }
                else { // voltage okay again, restart into tracking state, restart IMU
                    // PATH TESTED
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("RESTART!\n"); }
                    i2c.begin(I2C_FREQ_HZ_1MHZ);
                    startIMU(false);
                    state = ST_TRACK;
                    if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_ACC_ONLY) { device.enableAccInterruptInDeepSleep(); }
                    else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_1HZ_GPS_AND_ACC) { device.enableInternalTimerInterruptInDeepSleep(1); }
                    else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                }
            }
            /** ---------------- TRACKING STATE ---------------- */
            else if(state == ST_TRACK) {
                if (TRACKING_DATA_MODE == TRACKING_DATA_MODE_ACC_ONLY)                  { accTrackingMode(); }
                else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_1HZ_GPS_AND_ACC)       { special1HzMode(); }
                else if (TRACKING_DATA_MODE == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP)   { imuHighFrequencyLightSleepTrackingMode(); }
            }
            /** ---------------- NIGHT TIME STATE (IMU OFF!) ---------------- */
            else if(state == ST_NIGHT_TIME) { // wake up by regular RTC interrupt or by daily interrupt
                nightTimeCnt++;
                i2c.begin(I2C_FREQ_HZ_400KHZ);
                if(isTimeToGoToBed()) { // still time to be in bed
                    /** OPTION 1: do not do anything while in sleep mode -> then this is an error case */
                    #if NIGHTTIME_MODE == NIGHTTIME_MODE_ONLY_SLEEP
                        // that would be a strange error (RTC should wake up when sleep time is over), but happened once
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("STILL BED TIME - SHOULD NEVER HAPPEN! Wake up by %d\n", device.getWakeUpReason()); }
                        lastErrorId = 111; errorCnt++;
                        if(!device.rtc.setDailyInterrupt(NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE)) { lastErrorId = 112; errorCnt++; } // just in case, set daily interrupt again, WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                        device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
                    #endif
                    /** OPTION 2: transmit data every now and then, even in sleep */
                    #if NIGHTTIME_MODE == NIGHTTIME_MODE_TRY_DATATRANS
                        // PATH TESTED
                        if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, go to power down
                            // PATH TESTED
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: undervoltage during night time -> PWR DWN\n"); }
                            if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 106; errorCnt++; } 
                            state = ST_PWRDWN;
                            nightTimeCnt = 0;
                            device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                        }
                        else if(nightTimeModeDataTransIsDeepestNight()) {
                            // PATH TESTED
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: deepest night - don't even scan\n"); }
                            device.enableRTCInterruptInDeepSleep();
                        }
                        else if(nightTimeCnt % NIGHTTIME_MODE_TRY_DATATRANS_MODULO != 1) { // wake up and check if it's time to restart, but don't try a data transmission
                            // PATH TESTED
                            device.enableRTCInterruptInDeepSleep();
                        }
                        else {
                            // PATH TESTED
                            uint8_t commandByte = 0;
                            bool somethingTransmitted = false;
                            if(!device.initDataNVS()) { lastErrorId = 134; errorCnt++; }
                            uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                            uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: flashPointer: %d, flashOffsetPointer: %d\n", flashPointer, flashOffsetPointer); }
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: transmit try even when nothing to transmit\n"); }
                            if(!device.flashPowerOn(true)) { lastErrorId = 135; errorCnt++; } // turn on flash power already (5ms)
                            uint32_t timeNow = ((uint32_t) Timing::millis());

                            #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                            // esp now data transmission (FORCE mode!), IMPORTANT: calling with minBytesToTransmit = 0 -> FORCING a scan to get commandByte even if no data to send
                            somethingTransmitted = dataTransmissionEspNow(true, 0, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte);
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            #endif
                            
                            #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                            // wifi data transmission (FORCE mode!)
                            somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            #endif

                            if(!device.flashPowerOff(true)) { lastErrorId = 136; errorCnt++; } // turn off flash
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: something transmitted: %d\n", somethingTransmitted); }
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }

                            // use command byte
                            if(commandByte == COMMAND_BYTE_FORCE_TRACKING) {
                                // PATH TESTED
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: RECEIVED COMMAND TO FORCE TRACKING (no %d)\n", espNowForceTrackingOnceCnt); }
                                if(espNowForceTrackingOnceCnt < COMMAND_BYTE_FORCE_TRACKING_MAX) {
                                    espNowForceTrackingOnceCnt++;
                                    if(DO_THE_BLINK) { device.blinkTimes(5, B_RED); } // put BEFORE reactivation of fifo because at high sampling frequencies fifo might already be full after 5x blinky
                                    #if NIGHTTIME_MODE == NIGHTTIME_MODE_ONLY_SLEEP // SHOULD NOT COME HERE, just in case
                                        if(!device.rtc.resetInterruptFlags()) { lastErrorId = 108; errorCnt++; } // IMPORTANT: does not reset by itself!
                                        if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 109; errorCnt++; } // IMPORTANT: otherwise daily interrupt triggers again in tracking and stays on (not resetting itself)
                                    #endif
                                    #if NIGHTTIME_MODE == NIGHTTIME_MODE_TRY_DATATRANS
                                        if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 110; errorCnt++; } 
                                    #endif 

                                    startIMU(false);
                                    state = ST_TRACK;
                                    nightTimeCnt = 0;
                                    espNowForceTrackingOnce = true;
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                                    if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_ACC_ONLY) { device.enableAccInterruptInDeepSleep(); }
                                    else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_1HZ_GPS_AND_ACC) { device.enableInternalTimerInterruptInDeepSleep(1); }
                                    else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                                }
                                else {
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: refused\n"); }
                                    device.enableRTCInterruptInDeepSleep();
                                }
                            }
                            else if(commandByte == COMMAND_BYTE_DEACTIVATE) {
                                // PATH TESTED
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: RECEIVED CMD TO DE-ACTIVATE\n"); }
                                #if NIGHTTIME_MODE == NIGHTTIME_MODE_ONLY_SLEEP // SHOULD NOT COME HERE, just in case
                                    if(!device.rtc.resetInterruptFlags()) { lastErrorId = 156; errorCnt++; } // IMPORTANT: does not reset by itself!
                                    if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 157; errorCnt++; } // IMPORTANT: otherwise daily interrupt triggers again in tracking and stays on (not resetting itself)
                                #endif
                                #if NIGHTTIME_MODE == NIGHTTIME_MODE_TRY_DATATRANS
                                    if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 155; errorCnt++; }
                                #endif
                                nightTimeCnt = 0;
                                resetActivation(); // NVS already initialized
                                device.enableInternalTimerInterruptInDeepSleep(1);
                            }
                            else { device.enableRTCInterruptInDeepSleep(); } // continue sleeping
                        }
                    #endif
                }
                else { // night time over ->  back to tracking
                    // PATH TESTED
                    if(DO_THE_BLINK) { device.blinkTimes(5, B_RED); } // put BEFORE reactivation of fifo because at high sampling frequencies fifo might already be full after 5x blinky
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Night trans: Night time over -> restart IMU\n"); }
                    #if NIGHTTIME_MODE == NIGHTTIME_MODE_ONLY_SLEEP  
                        if(!device.rtc.resetInterruptFlags()) { lastErrorId = 105; errorCnt++; } // IMPORTANT: does not reset by itself!
                        if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 114; errorCnt++; } // IMPORTANT: otherwise daily interrupt triggers again in tracking and stays on (not resetting itself)
                    #endif
                    #if NIGHTTIME_MODE == NIGHTTIME_MODE_TRY_DATATRANS
                        if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 127; errorCnt++; } 
                    #endif 

                    startIMU(false);
                    state = ST_TRACK;
                    nightTimeCnt = 0;
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                    if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_ACC_ONLY) { device.enableAccInterruptInDeepSleep(); }
                    else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_1HZ_GPS_AND_ACC) { device.enableInternalTimerInterruptInDeepSleep(1); }
                    else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                }
            }
            /** ---------------- MEMORY FULL STATE (IMU OFF!) ---------------- */
            else if(state == ST_MEMFULL) {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("State: (MEM_FULL) V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, go to sleep (not power down, because IMU already stopped)
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("State: (MEM_FULL) undervoltage during MEM_FULL state -> SLEEP for %ds\n", FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                }
                else { // voltage okay (again), then just try to get rid of the data
                    #if defined(TRANSMISSION_METHOD_ESP_NOW) || defined(TRANSMISSION_METHOD_WIFI)
                    // get rtc timestamp (1ms)
                    i2c.begin(I2C_FREQ_HZ_400KHZ);
                    uint32_t timestamp = device.rtc.getTimestamp(error);
                    if(error) { lastErrorId = 9; errorCnt++; }
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("State: TIMESTAMP: %d\n", timestamp); }
                    // get pointers from NVS (15ms)
                    if(!device.initDataNVS()) { lastErrorId = 11; errorCnt++; }
                    uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                    uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: flashPointer: %d, flashOffsetPointer: %d\n", flashPointer, flashOffsetPointer); }
                    // check free space in memory
                    uint16_t flashBlockDeletedPointer = 0;
                    #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                        flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                    #endif
                    #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                        flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                    #endif
                    uint32_t freeSpaceBytesInFIFO = device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES);
                    uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockDeletedPointer, flashPointer);
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: FIFO free space in memory: %d, blocksToTransmit: %d\n", freeSpaceBytesInFIFO, blocksToTransmit); }
                    if((freeSpaceBytesInFIFO >= ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES) // restart when this "watermark" is reached
                        || (blocksToTransmit == 0)) { // restart when memory is "totally empty" (except the one block where recording started)
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: FIFO free space %d >= %d -> RESTART\n", freeSpaceBytesInFIFO, ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES); }
                        startIMU(false);
                        state = ST_TRACK;
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                        if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_ACC_ONLY) { device.enableAccInterruptInDeepSleep(); }
                        else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_1HZ_GPS_AND_ACC) { device.enableInternalTimerInterruptInDeepSleep(1); }
                        else if(TRACKING_DATA_MODE == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                        else { device.enableInternalTimerInterruptInDeepSleep(1); }
                    }
                    else { // transmit more before exiting this mode!
                        // turn on flash power already (5ms)
                        if(!device.flashPowerOn(true)) { lastErrorId = 10; errorCnt++; }

                        bool somethingTransmitted = false;
                        uint32_t timeNow;

                        if(DO_THE_BLINK) { device.ledGreenOn(); }

                        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                        // esp now data transmission (FORCE mode!)
                        timeNow = ((uint32_t) Timing::millis());
                        uint8_t commandByte = 0; // unused here - NO COMMANDS ACCEPTED WHEN MEMORY IS FULL
                        somethingTransmitted = dataTransmissionEspNow(true, 1, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte);
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                        #endif
                        
                        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                        // wifi data transmission (FORCE mode!, minBlocksToTransmit = 1, otherwise function returns without trying to send data when WIFI_MIN_BLOCKS_TO_TRANSMIT is > 1)
                        timeNow = ((uint32_t) Timing::millis());
                        somethingTransmitted = dataTransmissionWifi(true, 1, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                        #endif

                        if(DO_THE_BLINK) { device.ledGreenOff(); }

                        // turn off flash
                        if(!device.flashPowerOff(true)) { lastErrorId = 12; errorCnt++; } 

                        // adapt sleep interval based on transmission success
                        if(somethingTransmitted) {
                            memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY;
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("State: something transmitted -> wake up more frequently\n"); }
                        }
                        else { // could be wifi not found, but also connection issue or something else
                            if(memFullWakeUpOnMinute == ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY) { memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_LESS_FREQU; } // nothing transmitted now, but the session before -> try a again with small frequency one more time
                            else if(memFullWakeUpOnMinute == ST_MEMFULL_TRY_EVERY_FULL_MINUTE_LESS_FREQU) { memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY; } // again nothing transmitted -> more seldomly
                            else { memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY; }
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("State: nothing transmitted this time -> wake up more seldomly\n"); }
                        }
                        timestamp = device.rtc.getTimestamp(error); // get RTC timestamp AGAIN because after data transmission it might be totally different
                        uint32_t sleepyTime = calculateMemoryFullSleepTime(timestamp, memFullWakeUpOnMinute); 

                        device.enableInternalTimerInterruptInDeepSleep(sleepyTime); // sleep some time before trying again transmission
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("State: next transmission in %ds, memFullWakeUpOnMinute %d\n", sleepyTime, memFullWakeUpOnMinute); }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                    }
                    #else
                    if(DO_THE_BLINK) { device.blink(B_GREEN, B_GREEN); } // no data transmission, just blink from time to time
                    device.enableInternalTimerInterruptInDeepSleep(3600); // sleep some time before blinking again
                    #endif
                }
            }
            /** ---------------- FORCE DOWNLOAD STATE (IMU NEVER STARTED!) ---------------- */
            else if(state == ST_FORCE_DOWNLOAD) {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("State: forcing download, V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, go to sleep (not power down, because IMU already stopped)
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("State: undervoltage during ST_FORCE_DOWNLOAD -> SLEEP for %ds\n", FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                }
                else { // voltage okay (again), then just try to get rid of the data
                    #if defined(TRANSMISSION_METHOD_ESP_NOW) || defined(TRANSMISSION_METHOD_WIFI)
                    // do not get rtc timestamp here -> after reset most propably = 0
                    // get pointers from NVS
                    if(!device.initDataNVS()) { lastErrorId = 115; errorCnt++; }
                    uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                    uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                    uint16_t flashBlockDeletedPointer = 0;
                    #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                        flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                    #endif
                    #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                        flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                    #endif
                    uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockDeletedPointer, flashPointer);
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d, blocksToTransmit: %d\n", flashPointer, flashOffsetPointer, flashBlockDeletedPointer, blocksToTransmit); }
                    if(blocksToTransmit == 0) { // ALL memory free now -> go back to time state (maybe some rest left)
                        if(TRACKER_MODE == MODE_TESTRUN) {
                            printf("Flash: blocks to transmit ZERO\n");
                            uint32_t freeSpaceBytesInFIFO = device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES);
                            uint32_t remainingBytesInMemory = MT29_NUMBER_BYTES - freeSpaceBytesInFIFO;
                            printf("Flash: but still %d bytes in flash memory (no full block, so can't be transmitted -> next time)\n", remainingBytesInMemory);
                        }
                        if(DO_THE_BLINK) { device.blinkTimes(10, B_BOTH); } // blink 10 times before going to sleep FOREVER, no wake up interrupt!!!
                        // SLEEP FOREVER
                    }
                    else { // transmit more before exiting this mode!
                        // turn on flash power already (5ms)
                        if(!device.flashPowerOn(true)) { lastErrorId = 131; errorCnt++; }

                        bool somethingTransmitted = false;
                        uint32_t timeNow;

                        if(DO_THE_BLINK) { device.ledRedOn(); } // turn on led during transmission

                        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
                        // esp now data transmission (FORCE mode!)
                        timeNow = ((uint32_t) Timing::millis());
                        uint8_t commandByte = 0; // unused here - NO COMMANDS ACCEPTED IN FORCE MODE
                        somethingTransmitted = dataTransmissionEspNow(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte);
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                        #endif
                        
                        #if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
                        // wifi data transmission (FORCE mode!, IMPORTANT: only use FORCE_DOWNLOAD_WIFI_SSID, minBlocksToTransmit = 1, otherwise function returns without trying to send data when WIFI_MIN_BLOCKS_TO_TRANSMIT is > 1)
                        timeNow = ((uint32_t) Timing::millis());
                        somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, TIME_WIFI_SSIDS, TIME_WIFI_PASSWORDS, 1); // IMPORTANT: 1 = only use FORCE_DOWNLOAD_WIFI_SSID in TIME_WIFI_SSIDS list
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                        #endif

                        if(DO_THE_BLINK) { device.ledRedOff(); } // turn off led during transmission

                        // turn off flash
                        if(!device.flashPowerOff(true)) { lastErrorId = 100; errorCnt++; }

                        device.enableInternalTimerInterruptInDeepSleep(5); // sleep some time before trying again transmission
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("State: something transmitted: %d -> anyhow restart in 5s\n", somethingTransmitted); }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                    }
                    #else
                    if(DO_THE_BLINK) { device.blink(B_RED, B_RED); } // no data transmission, just blink from time to time
                    device.enableInternalTimerInterruptInDeepSleep(30); // sleep some time before blinking again
                    #endif                    
                }
            }
        }
        else { // not in productive or test run
            if(TRACKER_MODE == MODE_SELFTEST) {
                if(device.selfTest(SELFTEST_VOLTAGE_REF, SELFTEST_PARAMETERS)) {
                    #if SELFTEST_GPS_SET_BAUDRATE_PERMANTENLY == true
                        gpsCheckAliveAndMaybeChangeBaudrate(true);
                    #endif 
                }
            }
            else if(TRACKER_MODE == MODE_READFLASH) { printf("Mode: READ FLASH\n"); readFullFlash(); }
            else if(TRACKER_MODE == MODE_MOCK_FLASH_STATE) { printf("Mode: MOCK FLASH STATE\n"); mockFlashState(); }
        }
        startCnt++;
        device.deepSleep();
    }
}