#include "PlatformWildFiTagREV6.h"
#include "passwords.h"
#include "ModuleGPS_L70_REV6.h"
#include "configDefault.h"

WildFiTagREV6 device = WildFiTagREV6();
GPS_L70_REV6 gps = GPS_L70_REV6();

// Next open error id: lastErrorId = 193 (further free: a lot, 124)

// WARNING: THIS VERSION RUNS ON SINGLE CORE!
// WARNING: ULP NEEDS TO BE DISABLED IN MENUCONFIG, OTHERWISE RTC SLOW MEMORY OVERFLOW (+512Kbyte for ULP)
// WARNING: CHANGED ESP IDF TO SUPPORT 8kByte RTC MEMORY (https://github.com/espressif/esp-idf/commit/ef10c2576ff14afa033ef22105406399abc570af)

// 2021 BRAND NEW MEASUREMENTS
    // DEEP SLEEP SD (PRODUCTIVE, NO DATA TRANSM, 6 * 996 RTC BUFFER, 960 FIFO WATERMARK)
        // 6.25HZ ACC = 98uA
        // 25HZ ACC = 321uA
        // 50HZ ACC = 627uA
    // LIGHT SLEEP HD (PRODUCTIVE, NO DATA TRANSM, 960 FIFO WATERMARK)
        // 9-AXIS 25HZ = 3.62mA
        // 9-AXIS 50HZ = 5.06mA
        // 200HZ ACC = 1.94mA
        // 800HZ ACC = 4.68mA (900 FIFO WATERMARK)
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
// ESPNOW
    // @6.25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 1 hour (6.25 * 6 Byte = 37.5 Bytes per second)
    // @25Hz: 1 Block in Flash = 64 * 2048 = 128kBye will be full after < 15min (25 * 6 Byte = 150 Bytes per second)
    // @25Hz: memory full after 20 days, ???uA testrun (productive: 443uA) average WITHOUT esp now data transmission, 609uA (!!!) with esp now data transmission (sample 1 block -> transmit 1 block in 977-1064ms)
    // FAILED: custom PHY initialization -> RTC overflow by over 1868 bytes -> PHY init with each ESP NOW start (+ more milliseconds)

// TODO: for magnetometer calibration use BMX160_MAG_ACCURACY_REGULAR or better?
// [ TEST: data transmission of two or three tags in parallel ]

/** GENERAL */
// WARNING: ST_FULL_RF_CALIB for imuDeepSleepTrackingMode, what happens if sampling ultra-high frequency (maybe ACC interrupt not detected)
// WARNING: FLP mode does not work, as the 3 UART messages (ZDA, GPRMC, GPGGA) do not come in one burst but distributed over whole second (leading to UART timeout) -> messages arrive within 700ms (absolute bullshit)
// WARNING: problem: partial page programming should only be done with 512 bytes, not randomly
    // WARNING: proximity detection data corruption after around 120 partial page writes (single page)
    // seems to be no issue if less than 4 partial page writes (independent of length)
// TODO (LOW PRIO): data transmission DURING light sleep tracking (like in 1HzGPS mode)
// TODO (LOW PRIO): new data with: ESP32 hall sensor
// TODO (LOW PRIO): 1MHz I2C should not work with RTC?!? -> dynamic I2C speed switching -> TEST i2c.changeClockSpeed(I2C_FREQ_HZ_400KHZ) -> TESTED, works
// TODO (LOW PRIO): fifoPush -> add read back of written bytes? (no, ECC wrong anyhow)

// RTC variables
RTC_DATA_ATTR tag_config_t config = { }; // tag configuration (read from NVS)

RTC_DATA_ATTR uint32_t timestampNextDataTransmission = 0;
RTC_DATA_ATTR uint32_t memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY;
RTC_DATA_ATTR uint8_t fifoDataRam[RTC_RAM_BUFFER] = { 0 };
RTC_DATA_ATTR uint16_t fifoDataPointerRam = 0;

RTC_DATA_ATTR tracker_state_t state = ST_BOOT;
RTC_DATA_ATTR uint32_t startCnt = 0;
RTC_DATA_ATTR uint8_t lastErrorId = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;

RTC_DATA_ATTR bool isActivated = false;
RTC_DATA_ATTR bool hasValidTimestamp = false;
RTC_DATA_ATTR uint8_t getTimeAttemptCnt = 0;
RTC_DATA_ATTR uint8_t customRFCalibrationCnt = 0;
RTC_DATA_ATTR uint32_t nightTimeOverrideForcedTimestampStart;
RTC_DATA_ATTR uint64_t nightTimeCnt = 0;
RTC_DATA_ATTR uint16_t commandByteTimeSyncActivationTrys = 0;

RTC_DATA_ATTR bmm150_trim_registers trimData = {};
RTC_DATA_ATTR int16_t magHardIronOffsetX = 0;
RTC_DATA_ATTR int16_t magHardIronOffsetY = 0;
RTC_DATA_ATTR int16_t magHardIronOffsetZ = 0;

RTC_DATA_ATTR bool environmentSensorConnected = false;

RTC_DATA_ATTR uint8_t deadReckoningState = DR_STATE_GPS;
RTC_DATA_ATTR uint32_t deadReckoningImuTimestampStart = 0;

RTC_DATA_ATTR bool activateWhenNoGWAroundActive = false;

// global variables (non RTC)
const uint8_t DATATR_KNOWN_WIFI_LIST_SIZE = 1;
const char* DATATR_WIFI_SSIDS[DATATR_KNOWN_WIFI_LIST_SIZE] = { DATATR_WIFI_SSID };  // first priority: channel (6, 1, 11), within channel the name                     
const char* DATATR_WIFI_PASSWORDS[DATATR_KNOWN_WIFI_LIST_SIZE] = { DATATR_WIFI_PASSWORD }; // first priority: channel (6, 1, 11), within channel the name
const uint8_t TIME_WIFI_LIST_SIZE = 4; 
const char* TIME_WIFI_SSIDS[TIME_WIFI_LIST_SIZE] = { FORCE_DOWNLOAD_WIFI_SSID, WIFI_SSID1, WIFI_SSID2, WIFI_SSID3 }; // wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* TIME_WIFI_PASSWORDS[TIME_WIFI_LIST_SIZE] = { FORCE_DOWNLOAD_WIFI_PASSWORD, WIFI_PASSWORD1, WIFI_PASSWORD2, WIFI_PASSWORD3 }; // wifi password for activation/timestamp  
const char* ACTIVATION_WIFI_SSIDS[1] = { ACTIVATION_WIFI_SSID };                 // mpistart, wifi name to scan for at beginning -> if found -> stark tracking (NO PASSWORD NEEDED)
post_task_stream_flash_parameters_t restStreamParams;
bool error = false;
uint64_t t = 0;
uint8_t fifoDataNewest[1200] = { 0 };
uint8_t *sensorData = NULL; 
uint16_t sensorDataPointer = 0;
uint8_t gatewayAroundEspNowMac[6] = { 0 };
uint8_t gatewayAroundConfig[ESPNOW_META_MSG_GATEWAY_AROUND_V2_LEN] = { 0 };
bool gatewayAroundMessageEspNowReceived = false;
uint8_t gatewayAroundMessageCommandByte = COMMAND_BYTE_NOTHING;
bool timerFinished = false;
extern uint32_t batteryVoltageWakeStub;

void printStoredWiFis() {
    printf("*** STORED WIFIS ***\n");
    for(uint16_t i=0;i<DATATR_KNOWN_WIFI_LIST_SIZE;i++) {
        printf("- Data %d: %s / %c***\n", i, DATATR_WIFI_SSIDS[i], DATATR_WIFI_PASSWORDS[i][0]);
    }
    for(uint16_t i=0;i<TIME_WIFI_LIST_SIZE;i++) {
        printf("- Time %d: %s / %c***\n", i, TIME_WIFI_SSIDS[i], TIME_WIFI_PASSWORDS[i][0]);
    }
    printf("- Activation 0: %s / NA***\n", ACTIVATION_WIFI_SSIDS[0]);
    printf("*** END STORED WIFIS ***\n");
}

void startIMU(bool readTrimData) {
    acc_config_t accConfig = { config.accFrequency, config.accAvg, config.accRange };
    mag_config_t magConfig = { config.magFrequency, config.magAccuracy };
    gyro_config_t gyroConfig = { config.gyroFrequency, config.gyroRange, config.gyroMode };

    device.sensorPowerOn();
    device.shortLightSleep(120); // wait until bmx is booted
    if(readTrimData) {
        if(config.trackerMode == MODE_TESTRUN) { printf("startIMU: reading trim data\n"); }
        if(!device.imu.magCompensateReadTrimData(&trimData)) { lastErrorId = 153; errorCnt++; }
        if(config.trackerMode == MODE_TESTRUN) { device.imu.magCompensatePrintTrimData(&trimData); }
    }
    mag_config_t *magConfigPointer = NULL;
    gyro_config_t *gyroConfigPointer = NULL;
    if(config.useMagnetometer) { magConfigPointer = &magConfig; }
    if(config.useGyro) { gyroConfigPointer = &gyroConfig; }
    if(!device.imu.start(&accConfig, magConfigPointer, gyroConfigPointer, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 1; errorCnt++; }
    if((config.trackingDataMode == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) || (config.trackingDataMode == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP) || (config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING)) {
        if(!device.imu.enableFIFOInterrupt(config.accInterruptWatermark)) { lastErrorId = 2; errorCnt++; }
        if(config.trackerMode == MODE_TESTRUN) { printf("startIMU: running with FIFO interrupt\n"); }
    }
    uint8_t fifoForWhat = BMX160_INIT_FIFO_FOR_ACC;
    if(config.useMagnetometer) { fifoForWhat |= BMX160_INIT_FIFO_FOR_MAG; }
    if(config.useGyro) { fifoForWhat |= BMX160_INIT_FIFO_FOR_GYRO; }
    if(!device.imu.initFIFO(fifoForWhat)) { lastErrorId = 3; errorCnt++; }
    if(!device.imu.resetFIFO()) { lastErrorId = 113; errorCnt++; }
    fifoDataPointerRam = 0; // reset RAM data
    if(config.trackerMode == MODE_TESTRUN) { printf("startIMU: IMU started (errors: %d/%d)\n", lastErrorId, errorCnt); }
}

void startIMUOnlyActivity() {
    // 22.2 uA average power consumption
    acc_config_t accConfigOnlyActivity = { BMX160_ACCEL_ODR_12_5HZ, BMX160_ACCEL_BW_RES_AVG1, BMX160_ACCEL_RANGE_8G };
    device.sensorPowerOn();
    device.shortLightSleep(120); // wait until bmx is booted
    if(!device.imu.start(&accConfigOnlyActivity, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 187; errorCnt++; }
    //if(!device.imu.enableAccAnyMotionInterruptXYZ(3, 6)) { lastErrorId = 188; errorCnt++; } // 3 = 3 consecutive slopes higher than threshold, 2G: 1 = 3.91mg, 70 = 273.7mg, 8G = 15.64mg
    if(!device.imu.enableAccSignificantMotionInterruptXYZ(config.activityThresholdInactiveToActiveThr, ACTIVITY_THRESHOLD_INACTIVE_TO_ACTIVE_SKIP, ACTIVITY_THRESHOLD_INACTIVE_TO_ACTIVE_PROOF)) { lastErrorId = 188; errorCnt++; }
    if(config.trackerMode == MODE_TESTRUN) { printf("startIMUOnlyActivity: listening.. (errors: %d/%d)\n", lastErrorId, errorCnt); }
}

void stopIMU(bool doLightSleep = true) {
    if(!device.imu.stop()) { lastErrorId = 17; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
    device.sensorPowerOff(); // turn off IMU (and environment sensor) completely
    if(doLightSleep) { device.shortLightSleep(100); } // wait because otherwise interrupt pin might still be valid
}

bool readFifoIMU(uint8_t *data, uint16_t len, bool evaluateODBA) {
    if(!device.imu.readGeneralFIFOInOneGoFast(true, config.useMagnetometer, config.useGyro, data, len, false)) { lastErrorId = 116; errorCnt++; }
    if(config.useMagnetometer) {
        bmx160_fifo_dataset_len_t datasetStructure = BMX160_FIFO_DATASET_LEN_ACC_AND_MAG;
        if(config.useGyro) { datasetStructure = BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO; }
        if(!device.imu.magCompensateFifoData(data, len, datasetStructure, &trimData, magHardIronOffsetX, magHardIronOffsetY, magHardIronOffsetZ)) { lastErrorId = 62; errorCnt++;  }
    }
    if(evaluateODBA) { // if return value not needed then no need to calculate ODBA
        if(config.activityActivationEnabled) {
            if(nightTimeOverrideForcedTimestampStart == 0) { // NO overriding active -> only go into inactive state when no night time commanded!
                uint32_t activityLvl = device.imu.calculateFifoODBAAverage(data, len, config.useMagnetometer, config.useGyro, config.accRange, false); // no debugging
                if(config.trackerMode == MODE_TESTRUN) { printf("readFifoIMU: activityLvl: %d of %d\n", activityLvl, config.activityThresholdActiveToInactiveAvg); }
                if(activityLvl >= ((uint32_t) (config.activityThresholdActiveToInactiveAvg))) { return true; } // animal is active
                else { return false; } // animal is not active
            }
        }
    }
    return true; // default: animal is always active
}

void resetActivation() { // WARNING: NVS needs to be initalized
    state = ST_WAIT_FOR_ACTIVATION;
    isActivated = 0;
    if(config.activationMode == ACTIVATION_MODE_STORE_PERMANENTLY) {
        if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 0)) { lastErrorId = 154; errorCnt++; } // reset activation
    }
}

void resetRTCVariables() { // only called if config was changed
    // NOT RESETTING: all RTC variables in PlatformWildFiTagREV6.cpp
    //tag_config_t config = { }; // no need to reset
    timestampNextDataTransmission = 0;
    memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY;
    //fifoDataRam[RTC_RAM_BUFFER] = { 0 }; // no need to reset
    fifoDataPointerRam = 0;
    startCnt = 0;
    lastErrorId = 0;
    errorCnt = 0;
    isActivated = false;
    hasValidTimestamp = false;
    getTimeAttemptCnt = 0;
    customRFCalibrationCnt = 0;
    nightTimeOverrideForcedTimestampStart = 0;
    nightTimeCnt = 0;
    //trimData = {}; // no need to reset
    magHardIronOffsetX = 0;
    magHardIronOffsetY = 0;
    magHardIronOffsetZ = 0;
    environmentSensorConnected = false;
    deadReckoningState = DR_STATE_GPS;
    deadReckoningImuTimestampStart = 0;
    commandByteTimeSyncActivationTrys = 0;
    activateWhenNoGWAroundActive = false;
}

bool nightTimeModeDataTransIsDeepestNight() { // when in night time mode and data transmission is activated - don't try a data transmission in the middle of the night
    bool error = false;
    uint8_t currentHour = 0;
    if(!hasValidTimestamp) { return false; } // can't judge this without valid time
    currentHour = device.rtc.getHours(error);
    if(error) { lastErrorId = 132; errorCnt++; return false; }
    if(config.trackerMode == MODE_TESTRUN) {
        printf("isDeepestNight: ");
    	for(uint8_t hr=0; hr<24; hr++) {
		    bool isDeepestNight = (config.nightTimeDataTransDeepestNightHours >> (23 - hr)) & 1UL;
		    if(isDeepestNight) { 
                if(hr == currentHour) { printf("[D]"); }
                else { printf("D"); }
            }
		    else {
                if(hr == currentHour) { printf("[_]"); }
                else { printf("_"); }
            }
	    }
        printf("\n");
    }
    for(uint8_t hr=0; hr<24; hr++) {
		bool isDeepestNight = (config.nightTimeDataTransDeepestNightHours >> (23 - hr)) & 1UL;
		if(currentHour == hr) {
			if(isDeepestNight) {
                return true;
			}
		}
	}
    return false;
}

void enableNightTimeInterrupts() {
    if(config.doTheBlink) { device.blinkTimes(5, B_RED); }
    if(config.nightTimeMode == NIGHTTIME_MODE_ONLY_SLEEP) {
        if(!device.rtc.setDailyInterrupt(config.nightTimeTurnOnHour, config.nightTimeTurnOnMinute)) { lastErrorId = 133; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
        device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
    }
    else if(config.nightTimeMode == NIGHTTIME_MODE_TRY_DATATRANS) {
        if(!device.rtc.setRegularInterrupt(config.nightTimeModeTryDataTransWakeupSeconds)) { lastErrorId = 143; errorCnt++; }
        device.enableInternalTimerInterruptInDeepSleep(2); // immediately try a data transmission -> set this interrupt as well
        device.enableRTCInterruptInDeepSleep();
    }
}

void disableNightTimeInterrupts() {
    if(config.nightTimeMode == NIGHTTIME_MODE_ONLY_SLEEP) { // SHOULD NOT COME HERE, just in case
        if(!device.rtc.resetInterruptFlags()) { lastErrorId = 156; errorCnt++; } // IMPORTANT: does not reset by itself!
        if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 157; errorCnt++; } // IMPORTANT: otherwise daily interrupt triggers again in tracking and stays on (not resetting itself)
    }
    else if(config.nightTimeMode == NIGHTTIME_MODE_TRY_DATATRANS) {
        if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 155; errorCnt++; }
    }
    nightTimeCnt = 0;
}

void enterWaitForActivityMode() {
    if(config.trackerMode == MODE_TESTRUN) { printf("enterWaitForActivityMode: STOP, no activity\n"); }
    stopIMU(false); // don't wait at end
    state = ST_WAIT_FOR_ACTIVITY;
    device.shortLightSleep(1000);
    startIMUOnlyActivity();
    if(!device.rtc.setRegularInterruptLong(config.activityTransmissionInterval)) { lastErrorId = 189; errorCnt++; } // use same interval as in night time
    device.enableAccInterruptInDeepSleep();
    device.enableRTCInterruptInDeepSleep();
}

bool isTimeToGoToBed(uint16_t nightTimeBasedOnVoltageDebounceAdd = 0) { // nightTimeBasedOnVoltageDebounceAdd only when coming back from night time
    bool error = false;
    uint8_t currentHour = 0, currentMinute = 0;
    uint16_t minutesOfDay = 0;
    uint16_t offMinutesOfDay = (config.nightTimeTurnOffHour * 60) + config.nightTimeTurnOffMinute; // 0 ........ 1439
    uint16_t onMinutesOfDay = (config.nightTimeTurnOnHour * 60) + config.nightTimeTurnOnMinute; // 0 ........ 1439

    if(!hasValidTimestamp) { return false; } // never time to go to bed (when GET TIME state is skipped by configuration)

    // highest priority: evalute if tracking is forced via gateway command, do not enter sleep mode when forced mode activated
    if(nightTimeOverrideForcedTimestampStart > 0) { // overriding active
        uint32_t currentTimestamp = 0;
        if(device.rtc.getTimestamp(&currentTimestamp, NULL)) {
            if((currentTimestamp - nightTimeOverrideForcedTimestampStart) > ((uint32_t) (config.commandByteForceTrackingDurationSeconds))) {
                // forced tracking over, check conditions below
                nightTimeOverrideForcedTimestampStart = 0;
            }
            else { return false; } // forced tracking still active -> do not enter night time
        }
        else { lastErrorId = 185; errorCnt++; }
    } // if equals zero, then forced tracking not commanded, check conditions below
    
    // evaluate normal conditions
    if(config.nightTimeEnter == NIGHTTIME_ALWAYS_NIGHT) { return true; } // always time to go to bed (only force mode possible)
    else if(config.nightTimeEnter == NIGHTTIME_DISABLED) { return false; } // never time to go to bed
    else if(config.nightTimeEnter == NIGHTTIME_DURATION_BASED_ON_VOLTAGE_37V) {
        if(device.readSupplyVoltageFromWakeStub() < (3700 + nightTimeBasedOnVoltageDebounceAdd)) { return true; } // if battery voltage lower, always night!
    }
    else if(config.nightTimeEnter == NIGHTTIME_DURATION_BASED_ON_VOLTAGE_38V) {
        if(device.readSupplyVoltageFromWakeStub() < (3800 + nightTimeBasedOnVoltageDebounceAdd)) { return true; } // if battery voltage lower, always night!
    }
    else if(config.nightTimeEnter == NIGHTTIME_DURATION_BASED_ON_VOLTAGE_39V) {
        if(device.readSupplyVoltageFromWakeStub() < (3900 + nightTimeBasedOnVoltageDebounceAdd)) { return true; } // if battery voltage lower, always night!
    }
    else if(config.nightTimeEnter == NIGHTTIME_DURATION_BASED_ON_VOLTAGE_40V) {
        if(device.readSupplyVoltageFromWakeStub() < (4000 + nightTimeBasedOnVoltageDebounceAdd)) { return true; } // if battery voltage lower, always night!
    }
    else if(config.nightTimeEnter == NIGHTTIME_ONLY_BELOW_360V) {
        if(device.readSupplyVoltageFromWakeStub() < (3600 + nightTimeBasedOnVoltageDebounceAdd)) { return true; } // if battery voltage lower, always night!
        else { return false; }
    }
    else if(config.nightTimeEnter == NIGHTTIME_ONLY_BELOW_365V) {
        if(device.readSupplyVoltageFromWakeStub() < (3650 + nightTimeBasedOnVoltageDebounceAdd)) { return true; } // if battery voltage lower, always night!
        else { return false; }
    }
    else if(config.nightTimeEnter == NIGHTTIME_ONLY_BELOW_370V) {
        if(device.readSupplyVoltageFromWakeStub() < (3700 + nightTimeBasedOnVoltageDebounceAdd)) { return true; } // if battery voltage lower, always night!
        else { return false; }
    }
    else if(config.nightTimeEnter == NIGHTTIME_ONLY_BELOW_375V) {
        if(device.readSupplyVoltageFromWakeStub() < (3750 + nightTimeBasedOnVoltageDebounceAdd)) { return true; } // if battery voltage lower, always night!
        else { return false; }
    }
    else if(config.nightTimeEnter == NIGHTTIME_ONLY_BELOW_380V) {
        if(device.readSupplyVoltageFromWakeStub() < (3800 + nightTimeBasedOnVoltageDebounceAdd)) { return true; } // if battery voltage lower, always night!
        else { return false; }
    }
    currentHour = device.rtc.getHours(error);
    if(error) { lastErrorId = 102; errorCnt++; return false; }
    currentMinute = device.rtc.getMinutes(error);
    if(error) { lastErrorId = 104; errorCnt++; return false; }
    minutesOfDay = (currentHour * 60) + currentMinute; // calculate minutes passed that day, 0 ........ 1439
    if(config.trackerMode == MODE_TESTRUN) { printf("%d NIGHTTIME (V: %d): (%d:%d) sleep between %d:%d - %d:%d -> ", ((uint32_t) Timing::millis()), device.readSupplyVoltageFromWakeStub(), currentHour, currentMinute, config.nightTimeTurnOffHour, config.nightTimeTurnOffMinute, config.nightTimeTurnOnHour, config.nightTimeTurnOnMinute); }
    if(offMinutesOfDay < onMinutesOfDay) { // ----OFF___ON--------
        if((minutesOfDay >= offMinutesOfDay) && (minutesOfDay < onMinutesOfDay)) { // "<" important because interrupt happens at exact minute (14:15) -> would go to sleep again
            if(config.trackerMode == MODE_TESTRUN) { printf("YES (case 1)\n"); }
            return true; 
        }
    }
    else { // ___ON----------OFF__
        if((minutesOfDay >= offMinutesOfDay) || (minutesOfDay < onMinutesOfDay)) { // "<" important because interrupt happens at exact minute (14:15) -> would go to sleep again
            if(config.trackerMode == MODE_TESTRUN) { printf("YES (case 2)\n"); }
            return true; 
        }
    }
    if(config.trackerMode == MODE_TESTRUN) { printf("NO\n"); }
    return false; 
}

uint32_t calculateMemoryFullSleepTime(uint32_t currentTimestamp, uint32_t onMinute) { // sleeping with internal ESP32 timer, but trying to not add up the time drift error by correcting the sleep time depending on current timestamp
    // WARNING: when called during random time or when time interval changes: might over jump the NEXT interval when closer to next time (because assuming "too early" case)
    // WARNING: if config.skipGetTime = true -> might enter state with random timestamp (still works though, but not synced with real time)
    if(config.trackerMode == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: onMinute %d\n", onMinute); }
    uint32_t waitTime = 0;
    uint32_t secondsBetweenSleepInterval = onMinute * 60;
    uint32_t moduloValue = currentTimestamp % secondsBetweenSleepInterval; // between 0 .. secondsBetweenSleepInterval-1
    if(config.trackerMode == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: modulo value: %d\n", moduloValue); }
    if(moduloValue > (secondsBetweenSleepInterval / 2)) { // currently too early (14:13 but should be 14:15, modulo = 13min) -> assuming 14:15 should be right time
        waitTime = (secondsBetweenSleepInterval - moduloValue) + secondsBetweenSleepInterval;
        if(config.trackerMode == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: assuming too early by %ds -> sleep for %d + %d = %ds\n", (secondsBetweenSleepInterval - moduloValue), (secondsBetweenSleepInterval - moduloValue), secondsBetweenSleepInterval, (secondsBetweenSleepInterval - moduloValue) + secondsBetweenSleepInterval); }
    }
    else { // currently too late (14:17 but should be 14:15, modulo = 2min) -> assuming 14:15 should be right time, if modulo = 0 = on time, then sleeping for exactly 
        waitTime = secondsBetweenSleepInterval - moduloValue;
        if(config.trackerMode == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: assuming too late by %ds -> sleep for %d - %d = %ds\n", moduloValue, secondsBetweenSleepInterval, moduloValue, secondsBetweenSleepInterval - moduloValue); }
    }
    return waitTime;
}

void setNextDataTransmissionTimestamp(bool forceMode, uint32_t currentTimestamp, uint32_t onMinute) { // when time = 17:53 and transmission interval is 2 hours -> set to 18:00
    uint32_t secondsPassedSinceLastDesiredTime;
    uint32_t secondsUntilNextDesiredTime;
    if(forceMode || (currentTimestamp == 0)) { // in force mode: don't update next data transmission time
        if(config.trackerMode == MODE_TESTRUN) { printf("NEXTDATATRANSM: force mode, don't update\n"); }
        return;
    } 
    if(currentTimestamp < 1600000000) {
        if(config.trackerMode == MODE_TESTRUN) { printf("NEXTDATATRANSM: no synced time, don't update\n"); }
        return;
    }
    secondsPassedSinceLastDesiredTime = currentTimestamp % (60 * onMinute);
    secondsUntilNextDesiredTime = (60 * onMinute) - secondsPassedSinceLastDesiredTime;
    timestampNextDataTransmission = currentTimestamp + secondsUntilNextDesiredTime; // substract this from timestamp as a mocked last time transmission
    if(config.trackerMode == MODE_TESTRUN) { printf("NEXTDATATRANSM: next transmission: %d, current time %d, difference %ds, onMinute %d\n", timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp, onMinute); }
}

bool itsTimeForDataTransmission(uint32_t currentTimestamp) {
    if(currentTimestamp == 0) { return false; }
    return currentTimestamp >= timestampNextDataTransmission;
}

void handleCustomRFCalibration(bool allowFullRfCalibration) {
    if(customRFCalibrationCnt == 0) { // never did a full calibration
        if(!device.fullRFCalibration()) { lastErrorId = 140; errorCnt++; } // 145 - 160ms
        if(config.trackerMode == MODE_TESTRUN) { printf("RF: first time, full calib (err %d/%d)\n", lastErrorId, errorCnt); } 
    }
    else {
        if(allowFullRfCalibration && (customRFCalibrationCnt >= ESPNOW_CUSTOM_RF_FULL_CALIB_EVERY_X_TIMES)) { // every now and then: do full RF calibration   
            if(!device.fullRFCalibration()) { lastErrorId = 141; errorCnt++; } // 145 - 160ms
            if(config.trackerMode == MODE_TESTRUN) { printf("RF %d/%d: full calib (err %d/%d)\n", customRFCalibrationCnt, ESPNOW_CUSTOM_RF_FULL_CALIB_EVERY_X_TIMES, lastErrorId, errorCnt); } 
            customRFCalibrationCnt = 0;
        }
        else {
            if(!device.onlyLoadRFCalibration()) { lastErrorId = 142; errorCnt++; } // 5ms, using RF data in RTC memory
            if(config.trackerMode == MODE_TESTRUN) { printf("RF %d/%d: load only calib (err %d/%d)\n", customRFCalibrationCnt, ESPNOW_CUSTOM_RF_FULL_CALIB_EVERY_X_TIMES, lastErrorId, errorCnt); } 
        }        
    }
    customRFCalibrationCnt++;
}

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
    if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: next scan %d, blocksToTransmit %d, batteryV %d\n", ((uint32_t) Timing::millis()), timestampNextDataTransmission, blocksToTransmit, batteryVoltage); }
    if(batteryVoltage > config.dataTransBattMinVoltage) { // only if enough juice in battery
        if(blocksToTransmit >= minBlocksToTransmit) { // only if enough data to transmit
            if(forceMode || itsTimeForDataTransmission(currentTimestamp)) { // only if last scan at least x seconds ago
                if((config.trackerMode == MODE_TESTRUN) && (forceMode)) { printf("%d WIFI: FORCING SCAN!\n", ((uint32_t) Timing::millis())); }
                /* -------- NEW SCAN ALLOWED -------- */
                if(!device.initWiFi()) { lastErrorId = 63; errorCnt++; setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly); return somethingTransmitted; }
                uint8_t foundArrayId = 0;
                uint8_t foundOnChannel = 0;
                if(!device.scanForWiFisOn1and6and11(SSIDS_TO_USE, SSIDS_SIZE, &foundArrayId, &foundOnChannel, config.dataTransOutputPower, 120, 500)) { device.disconnectAndStopWiFi(); lastErrorId = 64; errorCnt++; setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly); return somethingTransmitted; }
                else {
                    if(foundOnChannel > 0) {
                        /* -------- MY WIFI FOUND -------- */
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: FOUND NO %d (CHAN: %d)!\n", ((uint32_t) Timing::millis()), foundArrayId, foundOnChannel); }
                        uint8_t connectionAttemptCounter = 0;
                        while(true) { // try multiple times to connect, because wifi has already been seen
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: CONNECTION ATTEMPT %d!\n", ((uint32_t) Timing::millis()), connectionAttemptCounter); }
                            if(!device.connectToWiFiAfterScan(SSIDS_TO_USE[foundArrayId], PASSWORDS_TO_USE[foundArrayId], foundOnChannel)) { lastErrorId = 66; errorCnt++; }
                            while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                                device.delay(20);
                                if(((uint32_t) Timing::millis()) - wifiStartTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                                    if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: TIMEOUT CONNECT!\n", ((uint32_t) Timing::millis())); }
                                    device.disconnectAndStopWiFi();
                                    lastErrorId = 67; errorCnt++;
                                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly); // re-try not so often
                                    return somethingTransmitted; // severe error, return immediately, no re-try
                                }
                            }
                            if((device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                                if(connectionAttemptCounter < 2) { // retry two times
                                    if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: access point seen, but could not connect: %d! -> RETRY ONCE\n", ((uint32_t) Timing::millis()), device.connectedToWiFi()); }
                                }
                                else {
                                    if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: access point seen, but could not connect: %d! -> CANCEL\n", ((uint32_t) Timing::millis()), device.connectedToWiFi()); }
                                    device.disconnectAndStopWiFi();
                                    lastErrorId = 68; errorCnt++;
                                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly); // re-try not so often
                                    return somethingTransmitted; // severe error
                                }
                            }
                            else {
                                break; // connected!
                            }
                            connectionAttemptCounter++;
                        }
                        /* -------- CONNECTED TO WIFI -------- */
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: CONNECT TOOK %dms (MAX %d)!\n", ((uint32_t) Timing::millis()), ((uint32_t) Timing::millis()) - wifiStartTime, (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)); }
                        /* -------- CALCULATE DATA TO TRANSMIT -------- */
                        uint16_t blocksToActuallyTransmit = blocksToTransmit;
                        if(blocksToActuallyTransmit > config.wifiMaxBlocksToTransmit) { // for REST POST routine: limit maximum number of blocks to transmit (will be less if voltage low or errors)
                            blocksToActuallyTransmit = config.wifiMaxBlocksToTransmit;
                        }
                        /* -------- CONSTRUCT HEADER VALUE 2 (DROPBOX, NEW) -------- */
                        #if (REST_ADD_HEADER2_SPRINTF == true)
                            char headerValue2Constructed[300] = { }; // WARNING: never exceed this limit
                            uint8_t sendMac[6] = { 0 };
                            esp_efuse_mac_get_default(sendMac);
                            sprintf(headerValue2Constructed, REST_ADD_HEADER_VALUE2, sendMac[0], sendMac[1], sendMac[2], sendMac[3], sendMac[4], sendMac[5], currentTimestamp);
                        #endif
                        /* -------- FILL PARAMETERS -------- */
                        restStreamParams.url = REST_URL;
                        restStreamParams.contentType = REST_CONTENT_TYPE;
                        restStreamParams.additionalHeaderKey1 = REST_ADD_HEADER_KEY1;
                        restStreamParams.additionalHeaderValue1 = REST_ADD_HEADER_VALUE1;
                        restStreamParams.additionalHeaderKey2 = REST_ADD_HEADER_KEY2;
                        #if (REST_ADD_HEADER2_SPRINTF == true)
                            restStreamParams.additionalHeaderValue2 = headerValue2Constructed;
                        #else
                            restStreamParams.additionalHeaderValue2 = REST_ADD_HEADER_VALUE2;
                        #endif
                        restStreamParams.prefix = REST_PAYLOAD_PREFIX;
                        restStreamParams.constructCustomPrefix = REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX;
                        restStreamParams.postfix = REST_PAYLOAD_POSTFIX;
                        restStreamParams.flashObject = &device.flash;
                        restStreamParams.flashBlockToSendNextPointer = flashBlockDeletedPointer;
                        restStreamParams.flashHalfBlockToSendNextPointer = flashHalfBlockToSendNextPointer;
                        restStreamParams.flashMaxNumberOfBlocksToTransmit = blocksToActuallyTransmit;
                        restStreamParams.deviceObject = &device;
                        restStreamParams.minBatteryVoltageToContinue = config.dataTransBattMinVoltageDuringTrans;
                        restStreamParams.useBase64Encoding = REST_USE_BASE64_ENCODING;
                        restStreamParams.debug = (config.trackerMode == MODE_TESTRUN);
                        /* -------- START POST TASK -------- */
                        uint16_t successfullyTransmittedBlocks = 0;
                        uint16_t successfullyTransmittedHalfBlocks = 0;
                        wifiStartTime = ((uint32_t) Timing::millis()); // reset watchdog timer
                        if(REST_SEND_FULL_BLOCKS) { device.doWiFiPOSTStreamCallFlashFullBlock(&restStreamParams); }
                        else { device.doWiFiPOSTStreamCallFlashHalfBlock(&restStreamParams); }
                        while(device.getWiFiPOSTCallStatus() == HTTP_POST_DATA_RUNNING) {
                            device.delay(100);
                            /* -------- WATCHDOG POST TASK -------- */
                            if(((uint32_t) Timing::millis()) - wifiStartTime > (WIFI_MAX_POST_TASK_TIME_SECONDS * 1000)) { // additional watchdog in case task is not timing out by itself
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: TASK TIMEOUT triggered, check if blocks were transmitted or not!\n", ((uint32_t) Timing::millis())); }
                                uint16_t successfullyTransmittedBlocksNew = 0;
                                uint16_t successfullyTransmittedHalfBlocksNew = 0; // don't look on half blocks
                                device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocksNew, successfullyTransmittedHalfBlocksNew);
                                if(successfullyTransmittedBlocksNew - successfullyTransmittedBlocks > 0) {
                                    if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: %d blocks transfered, continue waiting!\n", ((uint32_t) Timing::millis()), (successfullyTransmittedBlocksNew - successfullyTransmittedBlocks)); }
                                    wifiStartTime = ((uint32_t) Timing::millis()); // reset timer
                                    successfullyTransmittedBlocks = successfullyTransmittedBlocksNew; // update last checked block value
                                }
                                else {
                                    if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: NO blocks transfered, KILL TASK!\n", ((uint32_t) Timing::millis())); }
                                    //device.killPOSTTask(); // brutal! don't do! leads to reset
                                    break; // only BREAK from loop, maybe some block transmitted
                                }
                            }
                        }
                        /* -------- POST TASK FINISHED -------- */
                        if(device.getWiFiPOSTCallStatus() != HTTP_POST_DATA_FINISHED_ALL_GOOD) { // might still be RUNNING! when watchdog kicked in (will then lead to client write error because of software connection abort)
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: POST was not successful: %d!\n", ((uint32_t) Timing::millis()), device.getWiFiPOSTCallStatus()); }
                            lastErrorId = 70 + device.getWiFiPOSTCallStatus(); // 19 error codes -> 70 + 19 = 89, next errorid should be 90
                            errorCnt++;
                        }
                        device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks);
                        
                        if(config.trackerMode == MODE_TESTRUN) {
                            if(REST_SEND_FULL_BLOCKS) { printf("%d WIFI: successfully transmitted blocks: %d (half block transmission deactivated)\n", ((uint32_t) Timing::millis()), successfullyTransmittedBlocks); }
                            else { printf("%d WIFI: successfully transmitted blocks: %d, half blocks: %d\n", ((uint32_t) Timing::millis()), successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks); }
                        }
                        //heap_caps_free(dmaBuffer2048Bytes); // CAREFUL: task might still running if it supposed to be killed? -> DO NOT FREE MEMORY AT ALL
                        // also do not free memory of restPrefixPointer
                        /* -------- DELETING SUCCESSFULLY TRANSMITTED BLOCKS (ONLY FULLY TRANSMITTED) -------- */
                        uint16_t flashBlockDeletedPointerBeforeDelete = flashBlockDeletedPointer; // remember old value for NVS update
                        for(uint16_t delBlocks=0; delBlocks<successfullyTransmittedBlocks; delBlocks++) { // deleting is based ONLY on fully transmitted blocks
                            if(!device.flash.fifoPopDelete(flashBlockDeletedPointer, flashPointer, MT29_NUMBER_PAGES, MOCK_FLASH_DELETE)) { // delete block from flash and increment flashBlockDeletedPointer
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: FIFO POP DELETE FAILED!\n", ((uint32_t) Timing::millis())); }
                                device.disconnectAndStopWiFi();
                                lastErrorId = 90; errorCnt++;
                                setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly); // re-try not so often
                                return somethingTransmitted; // error here means -> no pointers are updated, retransmitting maybe already deleted blocks, but very unlikely to happen
                            } 
                        }
                        /* -------- UPDATING NVS POINTER -------- */
                        // FULL BLOCK POINTER
                        if(flashBlockDeletedPointerBeforeDelete != flashBlockDeletedPointer) { // some blocks are now fully transmitted
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI-NVS: updating flashBlockDeletedPointer (old: %d, new: %d)\n", ((uint32_t) Timing::millis()), flashBlockDeletedPointerBeforeDelete, flashBlockDeletedPointer); }
                            device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER, flashBlockDeletedPointer);
                            somethingTransmitted = true; // ONLY HERE, not when half blocks were transfered because blocks are not deleted
                        }
                        // HALF BLOCK POINTER
                        if(!REST_SEND_FULL_BLOCKS) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI-NVS: halfBlock points to: %d, successfullyTransmittedHalfBlocks: %d\n", ((uint32_t) Timing::millis()), flashHalfBlockToSendNextPointer, successfullyTransmittedHalfBlocks); }
                            if((flashHalfBlockToSendNextPointer == 0) && (successfullyTransmittedHalfBlocks == 0)) { // do not update, before no half transmitted block, afterwards also not, means everything went smooth
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 1) happy case, no half blocks before or after\n", ((uint32_t) Timing::millis())); }
                            }
                            else if((flashHalfBlockToSendNextPointer == 0) && (successfullyTransmittedHalfBlocks == 1)) { // before no half transmissions, now a half transmission (maybe only 0,5 blocks transmitted or maybe 10,5) -> update!
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 2) half block update -> before all good, now a half transmission\n", ((uint32_t) Timing::millis())); }
                                device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER, 1);
                            }
                            else if((flashHalfBlockToSendNextPointer == 1) && (successfullyTransmittedHalfBlocks == 0)) { // before only half a block was transmitted, now finished this block + maybe more (0,5 or 5,5 blocks transmitted) OR (bug before) 0,0 blocks transmitted
                                if(flashBlockDeletedPointerBeforeDelete != flashBlockDeletedPointer) { // there WAS an actual block transmission, means we update NVS
                                    if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 3A) half block before, now actually some blocks transmitted -> half block update\n", ((uint32_t) Timing::millis())); }
                                    device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER, 0);
                                }
                                else {
                                    if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 3B) nothing at all transmitted, keep flashHalfBlockToSendNextPointer = %d\n", ((uint32_t) Timing::millis()), flashHalfBlockToSendNextPointer); }
                                }
                            }
                            else if((flashHalfBlockToSendNextPointer == 1) && (successfullyTransmittedHalfBlocks == 1)) { // before a half transmissions, now some blocks AND a half transmission again
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI-NVS: (case 4) bad luck, half block transmission before AND after, keep half block pointer\n", ((uint32_t) Timing::millis())); }
                            }
                        }
                    }
                    else if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: SCANNED, but not found\n", ((uint32_t) Timing::millis())); }
                    device.disconnectAndStopWiFi(); // disconnect here because before that no wifi actions
                }
                /* -------- UPDATING SCANNING FREQUENCY AFTER A SCAN (OR DATATRANSMISSION!!!) WAS PERFORMED -------- */
                if(somethingTransmitted) { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); } // data to transmit and wifi found and data transmitted -> try more frequently (if enough data)
                else { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly); } // data to transmit, but wifi not found or data transmission not acked -> try less frequently
            }
            else if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: NO -> next time %d, current time %d, wait %ds\n", ((uint32_t) Timing::millis()), timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp); }
        }
        else if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: NO -> Blocks to transmit %d < %d\n", ((uint32_t) Timing::millis()), blocksToTransmit, minBlocksToTransmit); }
    }
    else if(config.trackerMode == MODE_TESTRUN) { printf("%d WIFI: NO -> Battery too low\n", ((uint32_t) Timing::millis())); }
    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}

bool isGatewayAroundMessage(wifi_promiscuous_pkt_t* p, wifi_promiscuous_pkt_type_t &type) {
    if(type == WIFI_PKT_MGMT) { // all esp now messages are MGMT frames
        if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + ESPNOW_META_MSG_GATEWAY_AROUND_LEN) { // normally 43 bytes additionally + payload
            if(p->rx_ctrl.rate == config.espNowDataRate) { // using the correct proximity data rate
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

bool isGatewayAroundMessageV2(wifi_promiscuous_pkt_t* p, wifi_promiscuous_pkt_type_t &type) {
    if(type == WIFI_PKT_MGMT) { // all esp now messages are MGMT frames
        if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + ESPNOW_META_MSG_GATEWAY_AROUND_V2_LEN) { // normally 43 bytes additionally + payload
            if(p->rx_ctrl.rate == config.espNowDataRate) { // using the correct proximity data rate
                if(p->payload[ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE] == 0x04) { // is ESP NOW frame
                    if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 0] == ESPNOW_META_MSG_GATEWAY_AROUND_V2)) { // gateway around message
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
    // p->rx_ctrl.rssi
    if(!gatewayAroundMessageEspNowReceived) {
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
        if(isGatewayAroundMessageV2(p, type)) { 
            if(((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 248] == 0x00) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 249] == 0x00))
                || ((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 248] == WILDFI_SOFTWARE_VERSION) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 249] == WILDFI_CONFIG_VERSION))) {
                gatewayAroundMessageEspNowReceived = true;
                gatewayAroundEspNowMac[0] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+0];
                gatewayAroundEspNowMac[1] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+1];
                gatewayAroundEspNowMac[2] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+2];
                gatewayAroundEspNowMac[3] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+3];
                gatewayAroundEspNowMac[4] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+4];
                gatewayAroundEspNowMac[5] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+5];
                gatewayAroundMessageCommandByte = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 1];

                for(uint8_t i=0; i<(ESPNOW_META_MSG_GATEWAY_AROUND_V2_LEN - 2); i++) { // store complete configuration message
                    gatewayAroundConfig[i] = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 2 + i];
                }
            }
        }
    }
}

static void proxReceiveCallback(const uint8_t *mac_addr, const uint8_t *data, int data_len) { } // do not do anything here, handled by promiscous sniffer

static void timerCallback(void* arg) { timerFinished = true; }

void sendTagAroundMessage(uint32_t bytesToTransmit) {
    uint8_t data[ESPNOW_META_MSG_TAG_AROUND_V2_LEN] = { 0 };
    uint16_t voltage = (uint16_t) batteryVoltageWakeStub;
    data[0] = ESPNOW_META_MSG_TAG_AROUND_V2;
    data[1] = voltage >> 8;
    data[2] = voltage & 0xFF;
    data[3] = lastErrorId;
    data[4] = errorCnt >> 8;
    data[5] = errorCnt & 0xFF;
    data[6] = gatewayAroundMessageCommandByte; // mirror the command byte
    data[7] = state;
    data[8] = isActivated;
    data[9] = hasValidTimestamp;
    data[10] = WILDFI_SOFTWARE_VERSION;
    data[11] = WILDFI_CONFIG_VERSION;
    data[12] = magHardIronOffsetX >> 8;
    data[13] = magHardIronOffsetX & 0xFF;
    data[14] = magHardIronOffsetY >> 8;
    data[15] = magHardIronOffsetY & 0xFF;
    data[16] = magHardIronOffsetZ >> 8;
    data[17] = magHardIronOffsetZ & 0xFF;
    data[18] = startCnt >> 24;
    data[19] = startCnt >> 16;
    data[20] = startCnt >> 8;
    data[21] = startCnt;
    data[22] = bytesToTransmit >> 24;
    data[23] = bytesToTransmit >> 16;
    data[24] = bytesToTransmit >> 8;
    data[25] = bytesToTransmit;
    device.broadcastESPNOWData(data, ESPNOW_META_MSG_TAG_AROUND_V2_LEN); // spit it out
}

bool gatewaySeenEspNow(uint8_t *commandByte, uint32_t bytesToTransmit) {
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
            //if(esp_timer_stop(timer) != ESP_OK) { lastErrorId = 124; errorCnt++; } // VERY seldomly error 124 happens (ONCE seen), I guess happens when timer (started once) stopped already
            esp_timer_stop(timer); // stop timer
            break;
        }
    }

    // ALWAYS send a tag around message (if gateway was seen)
    if(gatewayAroundMessageEspNowReceived) {
        sendTagAroundMessage(bytesToTransmit);
    }
    *commandByte = gatewayAroundMessageCommandByte;
    return gatewayAroundMessageEspNowReceived;
}

bool dataTransmissionEspNow(bool forceMode, uint32_t minBytesToTransmit, uint32_t currentTimestamp, uint32_t batteryVoltage, uint32_t flashPointer, uint16_t flashOffsetPointer, uint8_t *commandByte, bool allowFullRfCalibration) { // perform a wifi scan (366ms scan only, 1515ms in total if connecting)
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
    if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: next scan %d, bytesToTransmit %d, sendPnt %d, sendOffsetPnt %d, batteryV %d\n", ((uint32_t) Timing::millis()), timestampNextDataTransmission, bytesToTransmit, sendPagePointer, sendPageOffsetPointer, batteryVoltage); }
    if(batteryVoltage > config.dataTransBattMinVoltage) { // only if enough juice in battery
        if(bytesToTransmit >= minBytesToTransmit) { // only if enough data to transmit
            if(forceMode || itsTimeForDataTransmission(currentTimestamp)) { // only if last scan at least x seconds ago
                /* -------- CONDITIONS OKAY -> TRY TO SEND -------- */
                if((config.trackerMode == MODE_TESTRUN) && (forceMode)) { printf("%d ESPNOW: FORCING TRANSMISSION TRY!\n", ((uint32_t) Timing::millis())); }
                espNowStartTime = ((uint32_t) Timing::millis());
                if(config.espNowCustomRFCalibration) {
                    handleCustomRFCalibration(allowFullRfCalibration);
                }
                if(!device.initESPNOWStationary(config.espNowLongRange, config.dataTransOutputPower, true, (wifi_phy_rate_t) config.espNowDataRate)) { // 165ms, performs full calibration I guess (because custom PHY function overflows slow rtc)
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: initESPNOW ERROR!\n", ((uint32_t) Timing::millis())); }
                    lastErrorId = 44; errorCnt++;
                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly);
                    device.stopESPNOW(); // 5ms
                    esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                    return false;
                }
                if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: INIT TOOK %dms!\n", ((uint32_t) Timing::millis()), ((uint32_t) Timing::millis()) - espNowStartTime); }
                if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 121; errorCnt++; } // necessary for response
                bool gatewaySeen = gatewaySeenEspNow(commandByte, bytesToTransmit); // fill commandByte here, also send a hello message when gateway was seen
                if(!gatewaySeen) {
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: no gateway seen\n", ((uint32_t) Timing::millis())); }
                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly);
                    device.stopESPNOW(); // 5ms
                    esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                    return false;
                }
                else if((*commandByte == COMMAND_BYTE_DO_NOT_SEND) || (*commandByte == COMMAND_BYTE_ACTIVATE_WHEN_NO_GW)) {
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: gateway doesn't want data\n", ((uint32_t) Timing::millis())); }
                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly);
                    device.stopESPNOW(); // 5ms
                    esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                    return false;                  
                }
                else {
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: command byte 0x%02X\n", ((uint32_t) Timing::millis()), *commandByte); }
                    if(!device.addESPNOWReceiverStationary(gatewayAroundEspNowMac)) {
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: addESPNOWReceiver ERROR!\n", ((uint32_t) Timing::millis())); }
                        lastErrorId = 45; errorCnt++;
                        setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly);
                        device.stopESPNOW(); // 5ms
                        esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                        return false;
                    }
                    /* -------- TRY TO TRANSMIT DATA -------- */
                    uint32_t sendPagePointerBefore = sendPagePointer ; // 0 .. 2048 * 64
                    uint16_t sendPageOffsetPointerBefore = sendPageOffsetPointer; // 0 .. 2048

                    if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: OLD: sendPagePointer: %d, sendPageOffsetPointer: %d!\n", ((uint32_t) Timing::millis()), sendPagePointer, sendPageOffsetPointer); }

                    esp_now_stream_status_t espNowStatus = device.doESPNOWFlashStreamNew(
                        gatewayAroundEspNowMac,
                        NULL, 0,
                        &sendPagePointer, &sendPageOffsetPointer, 
                        flashPointer, flashOffsetPointer, 
                        config.espNowMaxBytesToTransmit,
                        500, 8, // millis to wait when one message failed (but acks happened before), number of retries
                        config.dataTransBattMinVoltageDuringTrans,
                        (config.trackerMode == MODE_TESTRUN) ? 1 : 0, MOCK_FLASH_READ, false); // 0 or (config.trackerMode == MODE_TESTRUN) ? 1 : 0, never mock the sending

                    if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: NEW: sendPagePointer: %d, sendPageOffsetPointer: %d, status: %d!\n", ((uint32_t) Timing::millis()), sendPagePointer, sendPageOffsetPointer, espNowStatus); }

                    /* -------- AFTER TRANSMISSION: SEND A SECOND TAG AROUND MESSAGE WITH NEW BYTESTOTRANSMIT -------- */
                    sendBytePointer = (sendPagePointer * MT29_CACHE_SIZE) + sendPageOffsetPointer;
                    bytesToTransmit = device.flash.fifoGetNumberOfPopableBytes(sendBytePointer, writeBytePointer); // update bytes to transmit
                    sendTagAroundMessage(bytesToTransmit);

                    /* -------- EVALUATE TRANSMISSION ERRORS -------- */
                    if(espNowStatus != ESP_NOW_STREAM_DATA_FINISHED) {
                        if((espNowStatus != ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR) && (espNowStatus != ESP_NOW_STREAM_NO_DATA_TO_SEND)) { // no data to send happens when function is called with minBlocksToTransmit = 0 (forcing a gateway scan) 
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: STREAM ERROR, status: %d!\n", ((uint32_t) Timing::millis()), espNowStatus); }
                            lastErrorId = 45 + espNowStatus;  errorCnt++; // max. 45 + 12 = 57 -> next id = 58 -> add some buffer = 60
                        }
                        else { // normal case, no gateway found
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: no gateway found or no data to send!\n", ((uint32_t) Timing::millis())); }
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
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: send block before %d, after %d -> delete %d blocks\n", ((uint32_t) Timing::millis()), sendBlockBefore, sendBlockAfter, blocksToDelete); }
                    for(uint16_t delBlock=sendBlockBefore; delBlock<(sendBlockBefore+blocksToDelete); delBlock++) { // deleting is based ONLY on fully transmitted blocks
                        if(!device.flash.erase(delBlock % MT29_NUMBER_BLOCKS)) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: BLOCK DELETE FAILED!\n", ((uint32_t) Timing::millis())); }
                            lastErrorId = 34; errorCnt++;
                            errorDuringDelete = true;
                        }
                        else if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: DELETED block %d\n", ((uint32_t) Timing::millis()), delBlock % MT29_NUMBER_BLOCKS); }
                    }
                    /* -------- UPDATING NVS POINTER (even if errorDuringDelete happened, because don't know at which block it happened) -------- */
                    if(sendPagePointerBefore != sendPagePointer) { // some pages are now transmitted
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW-NVS: updating sendPagePointer (old: %d, new: %d)\n", ((uint32_t) Timing::millis()), sendPagePointerBefore, sendPagePointer); }
                        device.nvsWriteUINT32((NVS_FLASH_SEND_POINTER), sendPagePointer);
                        somethingTransmitted = true;
                    }
                    if(sendPageOffsetPointerBefore != sendPageOffsetPointer) { // some bytes are transmitted
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW-NVS: updating sendPageOffsetPointer (old: %d, new: %d)\n", ((uint32_t) Timing::millis()), sendPageOffsetPointerBefore, sendPageOffsetPointer); }
                        device.nvsWriteUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER, sendPageOffsetPointer);
                        somethingTransmitted = true;
                    }
                    /* -------- UPDATING SCANNING FREQUENCY AFTER A SCAN WAS PERFORMED -------- */
                    if(somethingTransmitted && (!errorDuringDelete)) { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); } // data to transmit and wifi found and data transmitted -> try more frequently (if enough data)
                    else { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, config.dataTransTryEveryFullMinSeldomly); } // data to transmit, but wifi not found or data transmission not acked -> try less frequently
                }
            }
            else if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: NO -> next time %d, current time %d, wait %ds\n", ((uint32_t) Timing::millis()), timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp); }
        }
        else if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: NO -> bytes to transmit %d < %d\n", ((uint32_t) Timing::millis()), bytesToTransmit, minBytesToTransmit); }
    }
    else if(config.trackerMode == MODE_TESTRUN) { printf("%d ESPNOW: NO -> Battery too low\n", ((uint32_t) Timing::millis())); }
    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}

void tryGetTimeOverGPS(uint16_t sleepTimeWhenGetTimeFailed) {
    // PATH TESTED
    device.gpioBOn();
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');
    esp_gps_t gpsData = { };

    // CHANGED to tryToGetFix to collect orbit data already and to only get time after getting real fix
    //if(gps.getTimeOnly(&gpsData, USE_GPS_TO_GET_TIME_TIMEOUT_SECONDS, &device, TRACKING_DATA_MODE_1HZ_GPS_BLINK, (config.trackerMode == MODE_TESTRUN))) {
    gps_get_fix_config_t gpsConfig = {
        .timeoutSeconds = USE_GPS_TO_GET_TIME_TIMEOUT_SECONDS,
        .timeoutNotEvenTimeSeconds = 120,
        .minHDOP = 1.0f, // normally not reachable
        .afterFixMaxWaitOnHDOP = 10, // always wait 10 seconds after getting fix, THEN set the time
        .waitAfterFixUntilZeroMs = true, // after collecting additional orbit data: wait (if needed) until GPS sends messages at .000 ms
        .setRTCTime = true,
        .blinkLeds = true, // always on
        .debug = (config.trackerMode == MODE_TESTRUN) };
    get_fix_result_t fixResult = gps.tryToGetFix(&gpsData, &gpsConfig, &device);
    if(fixResult == GPS_FIX_SUCCESS_AND_RTC_UPDATED) {
        hasValidTimestamp = true;
        setNextDataTransmissionTimestamp(false, gpsData.parent.utcTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
        if(isActivated) {
            state = ST_START;
            if(config.trackerMode == MODE_TESTRUN) { printf("Time over GPS: all done and already activated -> MOVE TO START in %ds!\n", TIME_SLEEP_AFTER_GOT_TIME); }
        }
        else {
            state = ST_WAIT_FOR_ACTIVATION;
            if(config.trackerMode == MODE_TESTRUN) { printf("Time over GPS: all done -> move to ACTIVATION in %ds!\n", TIME_SLEEP_AFTER_GOT_TIME); }
        }
        device.blinkTimes(6, B_GREEN);
        device.enableInternalTimerInterruptInDeepSleep(TIME_SLEEP_AFTER_GOT_TIME); // restart in x seconds and move to activation
    }
    else {
        if(config.trackerMode == MODE_TESTRUN) { printf("Time over GPS: failed!\n"); }
        lastErrorId = 119; errorCnt++;
        device.enableInternalTimerInterruptInDeepSleep(sleepTimeWhenGetTimeFailed);
    }
    device.gpioBOff();
}

void imuDeepSleepTrackingMode() {
    /** --- COLLECT FIFO DATA (previously in wake stub) --- */
    i2c.begin(I2C_FREQ_HZ_1MHZ); // 1ms
    uint16_t currentFifoLen = device.imu.getFIFOLength();
    if(currentFifoLen >= IMU_FIFO_DEFINITELY_FULL) { lastErrorId = 118; errorCnt++; } // data loss possible
    if((fifoDataPointerRam + currentFifoLen) <= RTC_RAM_BUFFER) { // still space to put all fifo data into the RTC RAM
        readFifoIMU(fifoDataRam+fifoDataPointerRam, currentFifoLen, false);
        fifoDataPointerRam += currentFifoLen;
        startCnt--; // do not count that as start
        device.enableAccInterruptInDeepSleep();
    }
    /** --- NORMAL WAKE UP AFTER RTC RAM IS FULL --- */
    else {
        if(config.trackerMode == MODE_TESTRUN) { printf("-----\nState: %d\n", state); }
        if(config.trackerMode == MODE_TESTRUN) { printf("%d State: wake no: %d, V_BATT_WakeStub: %d\n", ((uint32_t) Timing::millis()), startCnt, device.readSupplyVoltageFromWakeStub()); }
        if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) {
            if(config.trackerMode == MODE_TESTRUN) { printf("%d State: POWER DOWN for %ds\n", ((uint32_t) Timing::millis()), FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
            state = ST_PWRDWN;
            stopIMU();
            device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
        }
        else if(isTimeToGoToBed()) {
            state = ST_NIGHT_TIME;
            stopIMU();
            enableNightTimeInterrupts();
        }
        else {
            if(config.doTheBlink) { device.ledGreenOn(); }
            // reserve some memory for flash (0ms)
            if(!device.flash.createBuffer(&sensorData, 100)) { lastErrorId = 18; errorCnt++; }
            // start bme measurement (7ms)
            bool bmeOk = false;
            int16_t temperature = 0;
            int16_t temperatureBmx = 0;
            uint32_t pressure = 0;
            uint32_t humidity = 0;
            if(environmentSensorConnected) {
                if(device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { // 4ms
                    if(device.baro.performMeasurement()) {
                        bmeOk = true;
                    }
                }
            }
            // get fifo data (101ms -> NEW: 16ms -> NEW NEW: 14ms)
            t = Timing::millis();
            uint16_t fifoDataNewestLen = device.imu.getFIFOLength(); // get it again because there might be more data now
            bool animalActive = readFifoIMU(fifoDataNewest, fifoDataNewestLen, true); // animalActive only false if activity detection activated
            if(fifoDataNewestLen >= IMU_FIFO_DEFINITELY_FULL) { lastErrorId = 36; errorCnt++; } // data loss possible
            if(config.trackerMode == MODE_TESTRUN) {
                printf("%d FIFO: RAM pointer: %d (MAX %d) + read %d bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataPointerRam, RTC_RAM_BUFFER, fifoDataNewestLen, (Timing::millis() - t));
                uint16_t fifoLenNew = device.imu.getFIFOLength();
                printf("%d FIFO: BMX FIFO AFTER READ: %d\n", ((uint32_t) Timing::millis()), fifoLenNew);
            }
            // get bmx temperature
            uint16_t temperatureBmxRaw = 0;
            device.imu.getTemperatureRaw(temperatureBmxRaw);
            temperatureBmx = device.imu.toCelsiusx100(temperatureBmxRaw);
            // get rtc timestamp (1ms)
            uint32_t timestamp = 0;
            uint8_t milliseconds = 0;
            if(!device.rtc.getTimestamp(&timestamp, &milliseconds)) { lastErrorId = 23; errorCnt++; }
            // get bme data (1ms)
            if(environmentSensorConnected) {
                if(bmeOk) {
                    if(device.baro.getResults()) {
                        temperature = device.baro.getTemperature(error);
                        if(error) { lastErrorId = 20; errorCnt++; }
                        pressure = device.baro.getPressure(error);
                        if(error) { lastErrorId = 21; errorCnt++; }
                        humidity = device.baro.getHumidity(error);
                        if(error) { lastErrorId = 22; errorCnt++; }
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d Data: TIMESTAMP: %d, TEMP: %d, TEMP BMX: %d, PRESS: %d, HUMI: %d\n", ((uint32_t) Timing::millis()), timestamp, temperature, temperatureBmx, pressure, humidity); }
                    }
                }
                else { lastErrorId = 19; errorCnt++; }
            }
            // add rest of data, already to DMA memory
            HelperBits::addData1_AndIncrementPointer(0x12, sensorData, &sensorDataPointer);
            HelperBits::addData1_AndIncrementPointer(0x34, sensorData, &sensorDataPointer);
            if((config.useMagnetometer) && (config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5E, sensorData, &sensorDataPointer); }
            else if((!config.useMagnetometer) && (config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5F, sensorData, &sensorDataPointer); }
            else if((config.useMagnetometer) && (!config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x60, sensorData, &sensorDataPointer); }
            else if((!config.useMagnetometer) && (!config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x61, sensorData, &sensorDataPointer); }
            HelperBits::addData4_AndIncrementPointer(timestamp, sensorData, &sensorDataPointer);
            HelperBits::addData1_AndIncrementPointer(milliseconds, sensorData, &sensorDataPointer);
            HelperBits::addData1_AndIncrementPointer(lastErrorId, sensorData, &sensorDataPointer);
            HelperBits::addData2_AndIncrementPointer(errorCnt, sensorData, &sensorDataPointer);
            HelperBits::addData2_AndIncrementPointer((uint16_t) device.readSupplyVoltageFromWakeStub(), sensorData, &sensorDataPointer);
            HelperBits::addData2Signed_AndIncrementPointer(temperature, sensorData, &sensorDataPointer);
            HelperBits::addData4_AndIncrementPointer(humidity, sensorData, &sensorDataPointer);
            HelperBits::addData4_AndIncrementPointer(pressure, sensorData, &sensorDataPointer);
            HelperBits::addData2Signed_AndIncrementPointer(temperatureBmx, sensorData, &sensorDataPointer);
            HelperBits::addData2_AndIncrementPointer(fifoDataNewestLen+fifoDataPointerRam, sensorData, &sensorDataPointer);

            // print data
            if(config.trackerMode == MODE_TESTRUN) {
                printf("%d Data Header: ", ((uint32_t) Timing::millis()));
                for(uint16_t i=0; i<sensorDataPointer; i++) { printf("%02X ", sensorData[i]); }
                printf("\n");
            }
            // power on flash (NO WAIT after power on -> because initNVS takes at least 15ms @80MHz)
            if(!device.flashPowerOn(false)) { lastErrorId = 24; errorCnt++; } // turn on flash power already (5ms)
            // get pointers from NVS (15ms -> NEW: 87ms -> should be less now with DataNVS -> yes: 12-16ms, unicore only 3ms??? - observe!)
            t = Timing::millis();
            if(!device.initDataNVS()) { lastErrorId = 25; errorCnt++; }
            if(config.trackerMode == MODE_TESTRUN) { printf("%d NVS: initialized, took %lld ms\n", ((uint32_t) Timing::millis()), (Timing::millis() - t)); }
            uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
            uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
            uint16_t flashBlockDeletedPointer = 0;
            if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
            }
            else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
            }
            if(config.trackerMode == MODE_TESTRUN) {
                printf("%d Flash: Need to store: %d Bytes sensor data, %d Bytes RAM data, %d Bytes new data\n", ((uint32_t) Timing::millis()), sensorDataPointer, fifoDataPointerRam, fifoDataNewestLen);
                printf("%d Flash: before storing: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer, flashBlockDeletedPointer);
            }
            // store data
            uint32_t timeNow = ((uint32_t) Timing::millis());
            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left before storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
            sequential_write_status_t writeStatus = device.flash.fifoPush(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, sensorData, sensorDataPointer, fifoDataRam, fifoDataPointerRam, fifoDataNewest, fifoDataNewestLen, MOCK_FLASH_WRITES);
            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: after storing: flashPointer: %d, flashOffsetPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer); }
            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left after storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
            timeNow = ((uint32_t) Timing::millis()) - timeNow;
            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: storage took: %dms\n", ((uint32_t) Timing::millis()), timeNow); }
            if(config.doTheBlink) { device.ledGreenOff(); }
            if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full -> go into special WIFI TRANSMISSION MODE!
                if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO full, stop IMU -> go into ST_MEMFULL state in 5 seconds\n", ((uint32_t) Timing::millis())); }
                stopIMU();
                memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY; // after memory full, try frequently to get rid of data
                state = ST_MEMFULL;
                if(!device.flashPowerOff(false)) { lastErrorId = 27; errorCnt++; } // important!
                fifoDataPointerRam = 0; // reset pointer
                heap_caps_free(sensorData); // important, free sensorData memory
                if(config.trackerMode == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }
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
                uint8_t commandByte = 0; // unused here
                if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                    // IMPORTANT HERE: not allowing for RF calibration!
                    somethingTransmitted = dataTransmissionEspNow(false, config.espNowMinBytesToTransmit, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte, false);
                }
                else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                    somethingTransmitted = dataTransmissionWifi(false, config.wifiMinBlocksToTransmit, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                }
                if(config.trackerMode == MODE_TESTRUN) { printf("%d DATATRANSM: needed %dms, something transmitted: %d\n", ((uint32_t) Timing::millis()), ((uint32_t) Timing::millis()) - timeNow, somethingTransmitted); }
                // turn off flash power (3ms)
                if(!device.flashPowerOff(false)) { lastErrorId = 30; errorCnt++; }
                // check if data transmission took long time (fifo full already, most probably missed the acc interrupt)
                uint16_t fifoLenAfterWifi = device.imu.getFIFOLength();
                if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: FIFO LEN before sleep: %d (reset when >= %d)\n", ((uint32_t) Timing::millis()), fifoLenAfterWifi, config.accInterruptWatermark); }
                if(fifoLenAfterWifi >= config.accInterruptWatermark) { // fifo full again -> reset it
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: FIFO RESET! DATA lost!\n", ((uint32_t) Timing::millis())); }
                    if(!device.imu.resetFIFO()) { lastErrorId = 60; errorCnt++; } // empty FIFO, do not read
                    fifoDataPointerRam = 0; // reset RAM data (not tested yet!)
                }
                // print error count
                if(config.trackerMode == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }

                // check if reconfig needed
                bool configChanged = false;
                if(commandByte == COMMAND_BYTE_CHANGE_CONFIG) {
                    // PATH TESTED
                    if(!checkIfReconfigNeeded(&device, &config, gatewayAroundConfig, commandByte, 0, &configChanged, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 182; errorCnt++; }
                }

                // evaluate commands
                if(configChanged) {
                    // PATH TESTED
                    if(config.trackerMode == MODE_TESTRUN) { printf("DATATRANSM: CONFIGURATION CHANGED -> RESTART\n"); }
                    stopIMU();
                    device.blinkTimes(12, B_BOTH);
                    resetRTCVariables();
                    state = ST_FIRST_START;
                    device.enableInternalTimerInterruptInDeepSleep(1);
                }
                else if(commandByte == COMMAND_BYTE_DEACTIVATE) {
                    // PATH TESTED
                    if(config.trackerMode == MODE_TESTRUN) { printf("DATATRANSM: RECEIVED CMD TO DE-ACTIVATE\n"); }
                    stopIMU();
                    resetActivation(); // NVS already initialized
                    device.enableInternalTimerInterruptInDeepSleep(1);
                }
                else if(commandByte == COMMAND_BYTE_ACTIVATE_WHEN_NO_GW) {
                    // PATH UNTESTED
                    if(config.trackerMode == MODE_TESTRUN) { printf("DATATRANSM: RECEIVED CMD TO ACTIVATE ONLY WHEN NO GW AROUND\n"); }
                    stopIMU();
                    resetActivation(); // NVS already initialized
                    device.enableInternalTimerInterruptInDeepSleep(60); // sleep longer to give gateway time to store data
                }
                else if(commandByte == COMMAND_BYTE_MAG_CALIBRATION) {
                    // PATH TESTED
                    if(config.trackerMode == MODE_TESTRUN) { printf("DATATRANSM: RECEIVED CMD TO CALIB MAG\n"); }
                    stopIMU();
                    state = ST_MAG_CALIBRATION;
                    device.enableInternalTimerInterruptInDeepSleep(1);
                }
                else if(commandByte == COMMAND_BYTE_TIME_RESYNC) {
                    // PATH TESTED
                    if(config.trackerMode == MODE_TESTRUN) { printf("DATATRANSM: RECEIVED CMD TO RESYNC TIME\n"); }
                    stopIMU();
                    state = ST_TIME_RESYNC;
                    device.enableInternalTimerInterruptInDeepSleep(1);
                }
                else if((config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) && config.espNowCustomRFCalibration && (customRFCalibrationCnt >= ESPNOW_CUSTOM_RF_FULL_CALIB_EVERY_X_TIMES)) { // every now and then: do full RF calibration
                    // PATH TESTED
                    state = ST_FULL_RF_CALIB;
                    device.enableShortInternalTimerInterruptInDeepSleep(10); // sleep only for 10 ms
                    // IMU still running here
                }
                else if(!animalActive) {
                    // PATH TESTED
                    enterWaitForActivityMode(); // re-configures IMU
                }
                else { device.enableAccInterruptInDeepSleep(); }
            }
        }
    }    
}

static bool validTimeTimerFinished = false;
static void validTimeCallback(void* arg) { validTimeTimerFinished = true; }

uint16_t gpsTrackingModeHeaderLength() {
    return 32;
}

void gpsTrackingModeAddData(esp_gps_t *gpsData, uint16_t currentFifoLen, uint8_t *buffer, uint16_t *bufferPointer) {
    uint32_t timestamp = 0;
    uint8_t milliseconds = 0;
    if(!device.rtc.getTimestamp(&timestamp, &milliseconds)) { lastErrorId = 149; errorCnt++; }
    float temp = gpsData->parent.dop_h * 10.0;
    if(temp > 255.) temp = 255.;
    uint8_t hdopTimesTen = temp;
    temp = gpsData->parent.latitude * 1000000;
    int32_t latTimes1Mil = temp;
    temp = gpsData->parent.longitude * 1000000;
    int32_t lonTimes1Mil = temp;
    int16_t temperature = 0, temperatureBmx;
    uint16_t temperatureBmxRaw = 0;
    uint32_t pressure = 0;
    uint32_t humidity = 0;

    if(gpsData->parent.fix == 0) { // set values to zero in case GPS fix is not valid
        hdopTimesTen = 0;
        latTimes1Mil = 0;
        lonTimes1Mil = 0;
    }

    if(!device.imu.getTemperatureRaw(temperatureBmxRaw)) { lastErrorId = 158; errorCnt++; }
    temperatureBmx = device.imu.toCelsiusx100(temperatureBmxRaw);
    if(environmentSensorConnected) {
        if(device.baro.getResults()) { // normally waits up to 31ms, but measurement was triggered before lightsleep
            temperature = device.baro.getTemperature(error);
            if(error) { lastErrorId = 20; errorCnt++; }
            pressure = device.baro.getPressure(error);
            if(error) { lastErrorId = 21; errorCnt++; }
            humidity = device.baro.getHumidity(error);
            if(error) { lastErrorId = 22; errorCnt++; }
        }
    }

    HelperBits::addData1_AndIncrementPointer(0x12, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(0x34, buffer, bufferPointer);
    if((config.useMagnetometer) && (config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5A, buffer, bufferPointer); }
    else if((!config.useMagnetometer) && (config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5B, buffer, bufferPointer); }
    else if((config.useMagnetometer) && (!config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5C, buffer, bufferPointer); }
    else if((!config.useMagnetometer) && (!config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5D, buffer, bufferPointer); }
    HelperBits::addData4_AndIncrementPointer(timestamp, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(milliseconds, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(lastErrorId, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(latTimes1Mil, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(lonTimes1Mil, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(hdopTimesTen, buffer, bufferPointer);
    HelperBits::addData2Signed_AndIncrementPointer(temperature, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(humidity, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(pressure, buffer, bufferPointer);
    HelperBits::addData2Signed_AndIncrementPointer(temperatureBmx, buffer, bufferPointer);
    HelperBits::addData2_AndIncrementPointer(currentFifoLen, buffer, bufferPointer);
    if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: UTC %d.%03d, LAT %d, LON %d, HDOP %d, TEMP %d/%d, PRESS %d, HUM %d\n", ((uint32_t) Timing::millis()), timestamp, ((uint16_t) milliseconds) * 10, latTimes1Mil, lonTimes1Mil, hdopTimesTen, temperature, temperatureBmx, pressure, humidity); }  
}

void deadReckoning() {
    if(deadReckoningState == DR_STATE_GPS) { // IMU running!
        const uint16_t FIFO_DATA_LEN = (1024 + gpsTrackingModeHeaderLength()) * 4;
        bool keepTrackingRunning = true;
        bool updatedRTCInThisSession = false;
        int64_t ttffStartUs = 0;
        bool errorDuringInit = false;
        get_uart_result_t res = GPS_UART_RESULT_SUCESS;
        uint16_t fifoDataPointer = 0;
        uint16_t fifoErrorLimit = IMU_FIFO_DEFINITELY_FULL; // if greater or equal than that, set error
        bool couldUpdateTime = false;
        uint16_t uartMillisWait = 0;
        float minHdop = (float) (config.drMinHdopXTen);
        minHdop = minHdop / 10.0f;
        uint64_t startTimeWaitAfterFirstFix = 0;
        if((config.accFrequency == BMX160_ACCEL_ODR_50HZ) && config.useMagnetometer && config.useGyro) { fifoErrorLimit = 1001; } // exception because fifo sometimes 1000 (but no error)

        i2c.begin(I2C_FREQ_HZ_400KHZ); // DO NOT use 1MHz

        //device.disableAccInterruptInDeepSleep(); // not waking up by ACC interrupt
        
        if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // do not even start when voltage is low
            // TESTED PATH
            if(config.trackerMode == MODE_TESTRUN) { printf("DR: POWER DOWN before start for %ds\n", FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
            state = ST_PWRDWN;
            stopIMU();
            device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
        }
        else if(isTimeToGoToBed()) { // do not even start when it's bed time
            // TESTED PATH
            if(config.trackerMode == MODE_TESTRUN) { printf("DR: sleep time (V %d)\n", device.readSupplyVoltageFromWakeStub()); }
            state = ST_NIGHT_TIME;
            stopIMU();
            enableNightTimeInterrupts();
        }
        else {
            if(!device.initDataNVS()) { lastErrorId = 14; errorCnt++; }
            if((config.trackerMode == MODE_TESTRUN) && (nightTimeOverrideForcedTimestampStart > 0)) { printf("DR: FORCE TRACKING\n"); }

            // try one time data transmission
            bool somethingTransmitted = false;
            uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
            if(!device.flashPowerOn(true)) { lastErrorId = 137; errorCnt++; } // turn on flash power already (10ms)
            uint32_t timeNow = ((uint32_t) Timing::millis());
            uint8_t commandByte = 0; // just for putting device back into activation state
            if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                // esp now data transmission (FORCE mode! but only executed if bytes to transmit)
                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                somethingTransmitted = dataTransmissionEspNow(true, config.espNowMinBytesToTransmit, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte, true);
                if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
            }
            if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                // wifi data transmission (FORCE mode!)
                somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
            }
            if(!device.flashPowerOff(true)) { lastErrorId = 138; errorCnt++; } // turn off flash
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: something transmitted: %d\n", somethingTransmitted); }
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }

            // check if reconfig needed
            bool configChanged = false;
            if(commandByte == COMMAND_BYTE_CHANGE_CONFIG) {
                // TESTED PATH
                if(!checkIfReconfigNeeded(&device, &config, gatewayAroundConfig, commandByte, 0, &configChanged, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 182; errorCnt++; }
            }

            // evaluate commands
            if(configChanged) {
                // TESTED PATH
                if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: CONFIGURATION CHANGED -> RESTART\n"); }
                stopIMU();
                device.blinkTimes(12, B_BOTH);
                resetRTCVariables();
                state = ST_FIRST_START;
                device.enableInternalTimerInterruptInDeepSleep(1);
            }
            else if(commandByte == COMMAND_BYTE_DEACTIVATE) {
                // TESTED PATH
                if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO DE-ACTIVATE\n"); }
                stopIMU();
                resetActivation(); // NVS already initialized
                device.enableInternalTimerInterruptInDeepSleep(1);
            }
            else if(commandByte == COMMAND_BYTE_ACTIVATE_WHEN_NO_GW) {
                // PATH UNTESTED
                if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO ACTIVATE ONLY WHEN NO GW AROUND\n"); }
                stopIMU();
                resetActivation(); // NVS already initialized
                device.enableInternalTimerInterruptInDeepSleep(60); // sleep longer to give gateway time to store data
            }
            else if(commandByte == COMMAND_BYTE_MAG_CALIBRATION) {
                // TESTED PATH
                if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO CALIB MAG\n"); }
                stopIMU();
                state = ST_MAG_CALIBRATION;
                device.enableInternalTimerInterruptInDeepSleep(1);
            }
            else if(commandByte == COMMAND_BYTE_TIME_RESYNC) {
                // UNTESTED
                if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO RESYNC TIME\n"); }
                stopIMU();
                state = ST_TIME_RESYNC;
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
                if(config.trackerMode == MODE_TESTRUN) { printf("DR: enter, available RAM: %d\n", availableRam); }

                uint8_t *fifoData = (uint8_t*) malloc(FIFO_DATA_LEN);
                if(fifoData == NULL) { errorDuringInit = true; }

                if(environmentSensorConnected) {
                    if(!device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { lastErrorId = 117; errorCnt++; } // 4ms
                }

                if(errorDuringInit) { // SHOULD NOT HAPPEN, but could happen if GPS is not connected -> sleep for FIRST_UNDER_VOLTAGE_SLEEP_TIME and try again afterwards
                    if(config.trackerMode == MODE_TESTRUN) { printf("DR: ERROR DURING INIT, SLEEP\n"); }
                    lastErrorId = 128; errorCnt++;
                    free(fifoData);
                    device.gpioBOff();
                    device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME);
                    return;
                }

                if(environmentSensorConnected) {
                    if(!device.baro.performMeasurement()) { lastErrorId = 147; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
                }

                uint16_t fifoLenAfterTransmission = device.imu.getFIFOLength(); // get size of data in fifo
                if(fifoLenAfterTransmission >= IMU_FIFO_DEFINITELY_FULL) { // IMPORTANT: not using fifoErrorLimit, because this doesn't set an error (normal situation after transmission)
                    device.imu.resetFIFO(); // reset fifo to start fresh
                    if(config.trackerMode == MODE_TESTRUN) { printf("DR: FIFO reset after transmission\n"); }
                }
                
                device.enableUart2InterruptInLightSleep();
                esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
                device.lightSleep();

                while(keepTrackingRunning) {
                    // wake up
                    // turn on green LED
                    if(TRACKING_DATA_MODE_DR_GPS_BLINK && config.doTheBlink) {
                        if(gpsData.parent.fix > 0) { device.ledGreenOn(); }
                        else { device.ledRedOn(); }
                    }

                    // STOP CONDITION TIMEOUT: check if wakeup due to timeout
                    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
                    if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
                        if(config.trackerMode == MODE_TESTRUN) { printf("DR: GPS timeout wake -> sleep for %d\n", TRACKING_DATA_MODE_DR_GPS_NOT_RESPONDING); }
                        lastErrorId = 146; errorCnt++;
                        device.enableInternalTimerInterruptInDeepSleep(TRACKING_DATA_MODE_DR_GPS_NOT_RESPONDING); // sleep for some time
                        keepTrackingRunning = false;
                        break;
                        // WARNING: IMU keeps running here!
                    }

                    // STOP CONDITION UART FAIL: wait until all uart messages are received (either stops after getting '\n' and having GPRMC and GPGGA or after 50ms when not receiving an event anymore)
                    res = gps.afterLightSleepWaitForGPRMCandGPGGA(&gpsData, &couldUpdateTime, &uartMillisWait, false); // no debug
                    if(res != GPS_UART_RESULT_SUCESS) {
                        lastErrorId = 174 + res; errorCnt++; // 175 - 180, next free: 181
                        if((res == GPS_UART_RESULT_INIT_ERROR) || (res == GPS_UART_RESULT_BUFF_OVERFLOW_ERROR)) { // fatal errors, stop for some time
                            device.enableInternalTimerInterruptInDeepSleep(15); // sleep for 15s
                            keepTrackingRunning = false;
                            break;
                            // WARNING: IMU keeps running here!
                        }
                    }

                    // update RTC
                    if(couldUpdateTime
                        && (!updatedRTCInThisSession)
                        && (gpsData.parent.tim.thousand == 0) // ONLY if GPS is sending messages at 0 ms
                        && ((gpsData.parent.fix > 0) || (!hasValidTimestamp))) { // first time getting a valid GPS time! (only executed once) -> ONLY AFTER GETTING A FIX (or if hasValidTimestamp is not set yet)
                        // --- BE QUICK HERE ---
                        updatedRTCInThisSession = true;
                        // TODO: add estimateUARTSendTimeMs
                        // delay from UART: (max. 2 * 80 Byte*(8+2) * (1 / 115200 = 0.00868ms/bit) = 13.88ms)
                        validTimeTimerFinished = false;
                        // if debugging: get current milliseconds to compare
                        uint8_t seconds100thOld = 0;
                        uint16_t millisecondsOld = 0;
                        uint32_t timestampOld = 0;
                        if(config.trackerMode == MODE_TESTRUN) {
                            device.rtc.getTimestamp(&timestampOld, &seconds100thOld);
                            millisecondsOld = seconds100thOld * 10;
                        }
                        // start timer
                        tmElements_t timeStruct;
                        bool waitingError = false;
                        uint32_t waitTimeUs = gpsData.parent.tim.thousand;
                        waitTimeUs = (1000 - waitTimeUs) * 1000;
                        if(waitTimeUs == 0) { waitTimeUs = 1; } // should not happen, but just in case
                        if(waitTimeUs == 1000000) { // 1 full second to wait (milliseconds of GPS = 0) -> don't wait
                            breakTime(gpsData.parent.utcTimestamp, timeStruct); // use THIS second
                            waitTimeUs = 0;
                        }
                        else {
                            if(esp_timer_start_once(validTimeTimer, waitTimeUs) == ESP_OK) {
                                while(!validTimeTimerFinished) { ; } // busy waiting until full second
                                breakTime(gpsData.parent.utcTimestamp + 1, timeStruct);
                            }
                            else { waitingError = true; }
                        }
                        if(!waitingError) {
                            if(device.rtc.set(timeStruct.Hour, timeStruct.Minute, timeStruct.Second, timeStruct.Wday, timeStruct.Day, timeStruct.Month, timeStruct.Year)) {
                                hasValidTimestamp = true; // IMPORTANT: update global variable
                                if(config.trackerMode == MODE_TESTRUN) { printf("DR: updated RTC to: %u -> %d:%d:%d (wait: %u ms)\n", gpsData.parent.utcTimestamp + 1, timeStruct.Hour, timeStruct.Minute, timeStruct.Second, (waitTimeUs/1000)); }    
                            }
                            else { lastErrorId = 129; errorCnt++; }
                        }
                        else { lastErrorId = 160; errorCnt++; } // same error id as above
                        if(config.trackerMode == MODE_TESTRUN) {
                            int64_t timestampMsOld = timestampOld;
                            timestampMsOld = (timestampMsOld * 1000) + millisecondsOld;
                            int64_t timestampMsNow = gpsData.parent.utcTimestamp;
                            timestampMsNow = (timestampMsNow * 1000) + gpsData.parent.tim.thousand;
                            int64_t timestampDiff = timestampMsOld - timestampMsNow;
                            printf("DR: RTC update: time diff %lld, %lld -> %lld\n", timestampDiff, timestampMsOld, timestampMsNow);
                            printf("DR: RTC update: %d:%d:%d (wait: %u ms)\n", timeStruct.Hour, timeStruct.Minute, timeStruct.Second, (waitTimeUs/1000));
                        }                                       
                    }

                    // remember time when first got a fix for hdop timeout
                    if(gpsData.parent.fix > 0) {
                        if(startTimeWaitAfterFirstFix == 0) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("DR: FIRST FIX!\n"); }
                            startTimeWaitAfterFirstFix = esp_timer_get_time();
                        } // remember first time got a fix
                    }
                    
                    // STOP CONDITION UNDERVOLTAGE: read voltage
                    uint16_t voltage = device.readSupplyVoltage(true);
                    if(voltage < config.battMinVoltage) {
                        // PATH TESTED
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d DR: POWER DOWN for %ds\n", ((uint32_t) Timing::millis()), FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
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
                        if((fifoDataPointer + currentFifoLen + gpsTrackingModeHeaderLength()) <= FIFO_DATA_LEN) {
                            // add header data
                            uint16_t fifoDataPointerOld = fifoDataPointer;
                            gpsTrackingModeAddData(&gpsData, currentFifoLen, fifoData, &fifoDataPointer);

                            // read acc data into fifoData
                            if(currentFifoLen > 0) {
                                readFifoIMU(fifoData+fifoDataPointer, currentFifoLen, false);
                                if(currentFifoLen >= fifoErrorLimit) { lastErrorId = 61; errorCnt++; } // data loss possible
                                fifoDataPointer += currentFifoLen;
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: pnt %d -> %d (MAX %d, STACK %d), %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataPointerOld, fifoDataPointer, FIFO_DATA_LEN, uxTaskGetStackHighWaterMark(NULL), currentFifoLen, (Timing::millis() - t)); }
                            }
                        }
                        else { // no more space in RAM light sleep memory -> store RAM + newest data from FIFO into flash
                            if(!device.flashPowerOn(false)) { lastErrorId = 15; errorCnt++; } // turn on flash power already

                            if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: full: %d -> write flash\n", ((uint32_t) Timing::millis()), currentFifoLen); }
                            uint16_t fifoDataNewestPointer = 0;

                            // add header data to fifoDataNewest
                            gpsTrackingModeAddData(&gpsData, currentFifoLen, fifoDataNewest, &fifoDataNewestPointer);

                            // read acc data into fifoDataNewest
                            if(currentFifoLen > 0) {
                                readFifoIMU(fifoDataNewest+fifoDataNewestPointer, currentFifoLen, false);
                                if(currentFifoLen >= fifoErrorLimit) { lastErrorId = 65; errorCnt++; } // data loss possible
                                fifoDataNewestPointer += currentFifoLen;
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: fifonewest pnt %d (MAX 1024), read %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataNewestPointer, currentFifoLen, (Timing::millis() - t)); }
                            }
                    
                            // read NVS pointer
                            uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                            uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                            uint16_t flashBlockDeletedPointer = 0;
                            if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                                flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                            }
                            else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                                flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                            }
                            if(config.trackerMode == MODE_TESTRUN) {
                                printf("%d Flash: Need to store: %d Bytes fifoData + %d Bytes fifoDataNew\n", ((uint32_t) Timing::millis()), fifoDataPointer, fifoDataNewestPointer);
                                printf("%d Flash: before storing: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer, flashBlockDeletedPointer);
                            }

                            // store data
                            uint32_t timeNow = ((uint32_t) Timing::millis());
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left before storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                            sequential_write_status_t writeStatus = device.flash.fifoPush(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, fifoData, fifoDataPointer, fifoDataNewest, fifoDataNewestPointer, NULL, 0, MOCK_FLASH_WRITES);
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: after storing: flashPointer: %d, flashOffsetPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer); }
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left after storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                            timeNow = ((uint32_t) Timing::millis()) - timeNow;
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: storage took: %dms\n", ((uint32_t) Timing::millis()), timeNow); }
                            // STOP CONDITION MEMORY FULL
                            if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full -> go into special WIFI TRANSMISSION MODE!
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO full, stop IMU -> go into ST_MEMFULL state in 5 seconds\n", ((uint32_t) Timing::millis())); }
                                stopIMU();
                                memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY; // after memory full, try frequently to get rid of data
                                state = ST_MEMFULL;
                                if(!device.flashPowerOff(false)) { lastErrorId = 26; errorCnt++; } // important!
                                fifoDataPointer = 0; // reset pointer
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }
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
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: FIFO LEN before sleep: %d (reset when >= %d)\n", ((uint32_t) Timing::millis()), fifoLenAfterWifi, config.accInterruptWatermark); }
                                if(fifoLenAfterWifi >= config.accInterruptWatermark) { // fifo full again -> reset it
                                    if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: FIFO RESET! DATA lost!\n", ((uint32_t) Timing::millis())); }
                                    if(!device.imu.resetFIFO()) { lastErrorId = 31; errorCnt++; } // empty FIFO, do not read
                                    fifoDataPointer = 0; // reset RAM data
                                }
                                // print error count
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }

                                // STOP CONDITION NIGHT TIME: stored data -> check if it's time to sleep now (so that data is stored)
                                if(isTimeToGoToBed()) { // time to go to bed
                                    // PATH TESTED
                                    state = ST_NIGHT_TIME;
                                    stopIMU();
                                    enableNightTimeInterrupts();
                                    keepTrackingRunning = false;
                                    break;
                                }
                                // STOP CONDITION SHIFT TO IMU MODE: if GPS fix with certain quality or timeout reached
                                // (got a fix AND (hdop good enough OR hdop wait time over)) OR DR gps time over
                                else if(((gpsData.parent.fix > 0) && ((gpsData.parent.dop_h < minHdop) || ((esp_timer_get_time() - startTimeWaitAfterFirstFix) / 1000000ULL > TRACKING_DATA_MODE_DR_HDOP_WAIT_SECONDS)))
                                    || ((esp_timer_get_time() - ttffStartUs) / 1000000ULL > TRACKING_DATA_MODE_DR_GPS_MAX_TIME)) { // no fix after X seconds -> shift to IMU logging
                                    // PATH TESTED
                                    if(config.trackerMode == MODE_TESTRUN) {
                                        printf("%d DR: move to IMU tracking (FIX %d AND (HDOP %.2f<%.2f OR TIME %d>%d)) OR TO %d>%d\n", ((uint32_t) Timing::millis()), gpsData.parent.fix, gpsData.parent.dop_h, minHdop, (uint32_t) (((esp_timer_get_time() - startTimeWaitAfterFirstFix) / 1000000ULL)), TRACKING_DATA_MODE_DR_HDOP_WAIT_SECONDS, (uint32_t) ((esp_timer_get_time() - ttffStartUs) / 1000000ULL), TRACKING_DATA_MODE_DR_GPS_MAX_TIME);
                                    }
                                    device.enableShortInternalTimerInterruptInDeepSleep(10); // sleep for some time
                                    keepTrackingRunning = false;
                                    deadReckoningState = DR_STATE_IMU;
                                    break;
                                    // WARNING: IMU keeps running here!
                                }
                            } 
                        }
                    }
                    if(config.trackerMode == MODE_TESTRUN) { printf("\n"); }
                    //if(addExtraDelay > 0) { device.delay(addExtraDelay); addExtraDelay = 0; } // in case of unsynced GPS messages
                    if(TRACKING_DATA_MODE_DR_GPS_BLINK && config.doTheBlink) { device.ledGreenOff(); device.ledRedOff(); }
                    if(environmentSensorConnected) {
                        if(!device.baro.performMeasurement()) { lastErrorId = 148; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
                    }
                    uart_flush(UART2_PORT_NUMBER);
                    device.enableUart2InterruptInLightSleep();
                    esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
                    device.lightSleep();
                }
                // end of big while loop (jumps here on break = stop condition)
                free(fifoData);
                device.gpioBOff();
                if(TRACKING_DATA_MODE_DR_GPS_BLINK && config.doTheBlink) { device.ledGreenOff(); device.ledRedOff(); }
            }
        }
    }
    else { // IMU running, wake up from deep sleep!
        /** --- QUICK WAKE UP --- */
        i2c.begin(I2C_FREQ_HZ_1MHZ); // 1ms
        uint32_t timestampNow = 0;
        device.rtc.getTimestamp(&timestampNow, NULL);
        if(deadReckoningImuTimestampStart == 0) {
            deadReckoningImuTimestampStart = timestampNow;
        }
        uint16_t currentFifoLen = device.imu.getFIFOLength();
        if(currentFifoLen >= IMU_FIFO_DEFINITELY_FULL) { lastErrorId = 118; errorCnt++; } // data loss possible
        if((fifoDataPointerRam + currentFifoLen) <= RTC_RAM_BUFFER_DR) { // still space to put all fifo data into the RTC RAM
            readFifoIMU(fifoDataRam+fifoDataPointerRam, currentFifoLen, false);
            fifoDataPointerRam += currentFifoLen;
            startCnt--; // do not count that as start
            device.enableAccInterruptInDeepSleep();
        }
        /** --- NORMAL WAKE UP AFTER RTC RAM IS FULL --- */
        else {
            if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) {
                // PATH TESTED
                if(config.trackerMode == MODE_TESTRUN) { printf("%d DR: POWER DOWN for %ds\n", ((uint32_t) Timing::millis()), FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
                state = ST_PWRDWN;
                stopIMU();
                deadReckoningState = DR_STATE_GPS; // return to GPS mode
                deadReckoningImuTimestampStart = 0;
                device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
            }
            else {
                // PATH TESTED
                if(config.doTheBlink) { device.ledGreenOn(); }
                // reserve some memory for flash (0ms)
                if(!device.flash.createBuffer(&sensorData, 100)) { lastErrorId = 18; errorCnt++; }
                // start bme measurement (7ms)
                bool bmeOk = false;
                int16_t temperature = 0;
                int16_t temperatureBmx = 0;
                uint32_t pressure = 0;
                uint32_t humidity = 0;
                if(environmentSensorConnected) {
                    if(device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { // 4ms
                        if(device.baro.performMeasurement()) {
                            bmeOk = true;
                        }
                    }
                }
                // get fifo data (101ms -> NEW: 16ms -> NEW NEW: 14ms)
                t = Timing::millis();
                uint16_t fifoDataNewestLen = device.imu.getFIFOLength(); // get it again because there might be more data now
                bool animalActive = readFifoIMU(fifoDataNewest, fifoDataNewestLen, true);
                if(fifoDataNewestLen >= IMU_FIFO_DEFINITELY_FULL) { lastErrorId = 36; errorCnt++; } // data loss possible
                if(config.trackerMode == MODE_TESTRUN) {
                    printf("%d FIFO: RAM pointer: %d (MAX %d) + read %d bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataPointerRam, RTC_RAM_BUFFER_DR, fifoDataNewestLen, (Timing::millis() - t));
                }
                // get bmx temperature
                uint16_t temperatureBmxRaw = 0;
                device.imu.getTemperatureRaw(temperatureBmxRaw);
                temperatureBmx = device.imu.toCelsiusx100(temperatureBmxRaw);
                // get rtc timestamp (1ms)
                uint32_t timestamp = 0;
                uint8_t milliseconds = 0;
                if(!device.rtc.getTimestamp(&timestamp, &milliseconds)) { lastErrorId = 23; errorCnt++; }
                // get bme data (1ms)
                if(environmentSensorConnected) {
                    if(bmeOk) {
                        if(device.baro.getResults()) {
                            temperature = device.baro.getTemperature(error);
                            if(error) { lastErrorId = 20; errorCnt++; }
                            pressure = device.baro.getPressure(error);
                            if(error) { lastErrorId = 21; errorCnt++; }
                            humidity = device.baro.getHumidity(error);
                            if(error) { lastErrorId = 22; errorCnt++; }
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d Data: TIMESTAMP: %d, TEMP: %d, TEMP BMX: %d, PRESS: %d, HUMI: %d\n", ((uint32_t) Timing::millis()), timestamp, temperature, temperatureBmx, pressure, humidity); }
                        }
                    }
                    else { lastErrorId = 19; errorCnt++; }
                }
                // add rest of data, already to DMA memory
                HelperBits::addData1_AndIncrementPointer(0x12, sensorData, &sensorDataPointer);
                HelperBits::addData1_AndIncrementPointer(0x34, sensorData, &sensorDataPointer);
                if((config.useMagnetometer) && (config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5E, sensorData, &sensorDataPointer); }
                else if((!config.useMagnetometer) && (config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5F, sensorData, &sensorDataPointer); }
                else if((config.useMagnetometer) && (!config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x60, sensorData, &sensorDataPointer); }
                else if((!config.useMagnetometer) && (!config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x61, sensorData, &sensorDataPointer); }
                HelperBits::addData4_AndIncrementPointer(timestamp, sensorData, &sensorDataPointer);
                HelperBits::addData1_AndIncrementPointer(milliseconds, sensorData, &sensorDataPointer);
                HelperBits::addData1_AndIncrementPointer(lastErrorId, sensorData, &sensorDataPointer);
                HelperBits::addData2_AndIncrementPointer(errorCnt, sensorData, &sensorDataPointer);
                HelperBits::addData2_AndIncrementPointer((uint16_t) device.readSupplyVoltageFromWakeStub(), sensorData, &sensorDataPointer);
                HelperBits::addData2Signed_AndIncrementPointer(temperature, sensorData, &sensorDataPointer);
                HelperBits::addData4_AndIncrementPointer(humidity, sensorData, &sensorDataPointer);
                HelperBits::addData4_AndIncrementPointer(pressure, sensorData, &sensorDataPointer);
                HelperBits::addData2Signed_AndIncrementPointer(temperatureBmx, sensorData, &sensorDataPointer);
                HelperBits::addData2_AndIncrementPointer(fifoDataNewestLen+fifoDataPointerRam, sensorData, &sensorDataPointer);

                // print data
                if(config.trackerMode == MODE_TESTRUN) {
                    printf("%d Data Header: ", ((uint32_t) Timing::millis()));
                    for(uint16_t i=0; i<sensorDataPointer; i++) { printf("%02X ", sensorData[i]); }
                    printf("\n");
                }
                // power on flash (NO WAIT after power on -> because initNVS takes at least 15ms @80MHz)
                if(!device.flashPowerOn(false)) { lastErrorId = 24; errorCnt++; } // turn on flash power already (5ms)
                // get pointers from NVS (15ms -> NEW: 87ms -> should be less now with DataNVS -> yes: 12-16ms, unicore only 3ms??? - observe!)
                t = Timing::millis();
                if(!device.initDataNVS()) { lastErrorId = 25; errorCnt++; }
                if(config.trackerMode == MODE_TESTRUN) { printf("%d NVS: initialized, took %lld ms\n", ((uint32_t) Timing::millis()), (Timing::millis() - t)); }
                uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                uint16_t flashBlockDeletedPointer = 0;
                if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                    flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                }
                else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                    flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                }
                if(config.trackerMode == MODE_TESTRUN) {
                    printf("%d Flash: Need to store: %d Bytes sensor data, %d Bytes RAM data, %d Bytes new data\n", ((uint32_t) Timing::millis()), sensorDataPointer, fifoDataPointerRam, fifoDataNewestLen);
                    printf("%d Flash: before storing: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer, flashBlockDeletedPointer);
                }
                // store data
                uint32_t timeNow = ((uint32_t) Timing::millis());
                if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left before storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                sequential_write_status_t writeStatus = device.flash.fifoPush(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, sensorData, sensorDataPointer, fifoDataRam, fifoDataPointerRam, fifoDataNewest, fifoDataNewestLen, MOCK_FLASH_WRITES);
                if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: after storing: flashPointer: %d, flashOffsetPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer); }
                if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left after storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                timeNow = ((uint32_t) Timing::millis()) - timeNow;
                if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: storage took: %dms\n", ((uint32_t) Timing::millis()), timeNow); }
                if(config.doTheBlink) { device.ledGreenOff(); }
                if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full -> go into special WIFI TRANSMISSION MODE!
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO full, stop IMU -> go into ST_MEMFULL state in 5 seconds\n", ((uint32_t) Timing::millis())); }
                    stopIMU();
                    deadReckoningState = DR_STATE_GPS; // return to GPS mode
                    deadReckoningImuTimestampStart = 0;
                    memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY; // after memory full, try frequently to get rid of data
                    state = ST_MEMFULL;
                    if(!device.flashPowerOff(false)) { lastErrorId = 27; errorCnt++; } // important!
                    fifoDataPointerRam = 0; // reset pointer
                    heap_caps_free(sensorData); // important, free sensorData memory
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }
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

                    // turn off flash power (3ms)
                    if(!device.flashPowerOff(false)) { lastErrorId = 30; errorCnt++; }

                    // print error count
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }

                    // STOP CONDITION: DR burst time over
                    if((timestampNow - deadReckoningImuTimestampStart) > config.drImuSeconds) {
                        // PATH TESTED
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d DR: IMU time over\n", ((uint32_t) Timing::millis())); }
                        deadReckoningState = DR_STATE_GPS; // return to GPS mode
                        deadReckoningImuTimestampStart = 0;
                        device.enableShortInternalTimerInterruptInDeepSleep(20);
                    }
                    // STOP CONDITION: night time
                    else if(isTimeToGoToBed()) {
                        // PATH TESTED
                        state = ST_NIGHT_TIME;
                        stopIMU();
                        deadReckoningState = DR_STATE_GPS; // return to GPS mode
                        deadReckoningImuTimestampStart = 0;
                        enableNightTimeInterrupts();
                    }
                    // STOP CONDITION: animal not active
                    else if(!animalActive) {
                        // PATH TESTED
                        deadReckoningState = DR_STATE_GPS; // return to GPS mode
                        deadReckoningImuTimestampStart = 0;
                        enterWaitForActivityMode(); // re-configures IMU
                    }
                    else {
                        device.enableAccInterruptInDeepSleep();
                    }
                }
            }
        }
    }
}

void imuAnd1HzGPSTrackingMode() {
    const uint16_t FIFO_DATA_LEN = (1024 + gpsTrackingModeHeaderLength()) * 16;
    bool keepTrackingRunning = true;
    bool updatedRTCInThisSession = false;
    int64_t ttffStartUs = 0;
    bool errorDuringInit = false;
    get_uart_result_t res = GPS_UART_RESULT_SUCESS;
    uint16_t fifoDataPointer = 0;
    uint16_t fifoErrorLimit = IMU_FIFO_DEFINITELY_FULL; // if greater or equal than that, set error
    bool couldUpdateTime = false;
    uint16_t uartMillisWait = 0;
    if((config.accFrequency == BMX160_ACCEL_ODR_50HZ) && config.useMagnetometer && config.useGyro) { fifoErrorLimit = 1001; } // exception because fifo sometimes 1000 (but no error)

    i2c.begin(I2C_FREQ_HZ_400KHZ); // DO NOT use 1MHz
    
    if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // do not even start when voltage is low
        // PATH TESTED
        if(config.trackerMode == MODE_TESTRUN) { printf("1HzGPS: POWER DOWN before start for %ds\n", FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
        state = ST_PWRDWN;
        stopIMU();
        device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
    }
    else if(isTimeToGoToBed()) { // do not even start when it's bed time
        // PATH TESTED
        state = ST_NIGHT_TIME;
        stopIMU();
        enableNightTimeInterrupts();
    }
    else {
        if(!device.initDataNVS()) { lastErrorId = 14; errorCnt++; }
        if((config.trackerMode == MODE_TESTRUN) && (nightTimeOverrideForcedTimestampStart > 0)) { printf("1HzGPS: FORCE TRACKING\n"); }

        // try one time data transmission
        bool somethingTransmitted = false;
        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
        if(!device.flashPowerOn(true)) { lastErrorId = 137; errorCnt++; } // turn on flash power already (10ms)
        uint32_t timeNow = ((uint32_t) Timing::millis());
        uint8_t commandByte = 0; // just for putting device back into activation state
        if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
            // esp now data transmission (FORCE mode! but only executed if bytes to transmit)
            uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
            somethingTransmitted = dataTransmissionEspNow(true, config.espNowMinBytesToTransmit, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte, true);
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
        }
        else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
            // wifi data transmission (FORCE mode!)
            somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
        }
        if(!device.flashPowerOff(true)) { lastErrorId = 138; errorCnt++; } // turn off flash
        if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: something transmitted: %d\n", somethingTransmitted); }
        if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }

        // check if reconfig needed
        bool configChanged = false;
        if(commandByte == COMMAND_BYTE_CHANGE_CONFIG) {
            // PATH TESTED
            if(!checkIfReconfigNeeded(&device, &config, gatewayAroundConfig, commandByte, 0, &configChanged, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 182; errorCnt++; }
        }

        // evaluate commands
        if(configChanged) {
            // PATH TESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: CONFIGURATION CHANGED -> RESTART\n"); }
            stopIMU();
            device.blinkTimes(12, B_BOTH);
            resetRTCVariables();
            state = ST_FIRST_START;
            device.enableInternalTimerInterruptInDeepSleep(1);
        }
        else if(commandByte == COMMAND_BYTE_DEACTIVATE) {
            // PATH TESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO DE-ACTIVATE\n"); }
            stopIMU();
            resetActivation(); // NVS already initialized
            device.enableInternalTimerInterruptInDeepSleep(1);
        }
        else if(commandByte == COMMAND_BYTE_ACTIVATE_WHEN_NO_GW) {
            // PATH UNTESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO ACTIVATE ONLY WHEN NO GW AROUND\n"); }
            stopIMU();
            resetActivation(); // NVS already initialized
            device.enableInternalTimerInterruptInDeepSleep(60); // sleep longer to give gateway time to store data
        }
        else if(commandByte == COMMAND_BYTE_MAG_CALIBRATION) {
            // PATH TESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO CALIB MAG\n"); }
            stopIMU();
            state = ST_MAG_CALIBRATION;
            device.enableInternalTimerInterruptInDeepSleep(1);
        }
        else if(commandByte == COMMAND_BYTE_TIME_RESYNC) {
            // UNTESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO RESYNC TIME\n"); }
            stopIMU();
            state = ST_TIME_RESYNC;
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
            if(TRACKING_DATA_MODE_1HZ_GPS_FITNESS_LOW_POWER) {
                // WARNING: DOES NOT WORK!!!
                if(!gps.setFLPMode(false)) { errorDuringInit = true; }
            }
            if(!gps.setNMEAMessagesMinimum1HzWithZDA()) { errorDuringInit = true; }
            uart_flush(UART2_PORT_NUMBER);

            const esp_timer_create_args_t validTimeArgs = { .callback = &validTimeCallback };
            esp_timer_handle_t validTimeTimer;
            if(esp_timer_create(&validTimeArgs, &validTimeTimer) != ESP_OK) { errorDuringInit = true; }

            size_t availableRam = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
            if(config.trackerMode == MODE_TESTRUN) { printf("1HzGPS: enter, available RAM: %d\n", availableRam); }

            uint8_t *fifoData = (uint8_t*) malloc(FIFO_DATA_LEN);
            if(fifoData == NULL) { errorDuringInit = true; }

            // NEW: initalize barometer
            if(environmentSensorConnected) {
                if(!device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { lastErrorId = 117; errorCnt++; } // 4ms
            }

            if(errorDuringInit) { // SHOULD NOT HAPPEN, but could happen if GPS is not connected -> sleep for FIRST_UNDER_VOLTAGE_SLEEP_TIME and try again afterwards
                if(config.trackerMode == MODE_TESTRUN) { printf("1HzGPS: ERROR DURING INIT, SLEEP\n"); }
                lastErrorId = 128; errorCnt++;
                free(fifoData);
                device.gpioBOff();
                device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME);
                return;
            }
            if(environmentSensorConnected) {
                if(!device.baro.performMeasurement()) { lastErrorId = 147; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
            }
            uint16_t fifoLenAfterTransmission = device.imu.getFIFOLength(); // get size of data in fifo
            if(fifoLenAfterTransmission >= IMU_FIFO_DEFINITELY_FULL) { // IMPORTANT: not using fifoErrorLimit, because this doesn't set an error (normal situation after transmission)
                device.imu.resetFIFO(); // reset fifo to start fresh
                if(config.trackerMode == MODE_TESTRUN) { printf("1HzGPS: FIFO reset after transmission\n"); }
            }
            device.enableUart2InterruptInLightSleep();
            esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
            device.lightSleep();

            while(keepTrackingRunning) {
                // wake up
                // turn on green LED
                if(TRACKING_DATA_MODE_1HZ_GPS_BLINK && config.doTheBlink) {
                    if(gpsData.parent.fix > 0) { device.ledGreenOn(); }
                    else { device.ledRedOn(); }
                }

                // STOP CONDITION TIMEOUT: check if wakeup due to timeout
                esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
                if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
                    if(config.trackerMode == MODE_TESTRUN) { printf("1HzGPS: GPS timeout wake -> sleep for %d\n", TRACKING_DATA_MODE_1HZ_GPS_NOT_RESPONDING); }
                    lastErrorId = 146; errorCnt++;
                    device.enableInternalTimerInterruptInDeepSleep(TRACKING_DATA_MODE_1HZ_GPS_NOT_RESPONDING); // sleep for some time
                    keepTrackingRunning = false;
                    break;
                    // WARNING: IMU keeps running here!
                }

                // STOP CONDITION UART FAIL: wait until all uart messages are received (either stops after getting '\n' and having GPRMC and GPGGA or after 50ms when not receiving an event anymore)
                res = gps.afterLightSleepWaitForGPRMCandGPGGA(&gpsData, &couldUpdateTime, &uartMillisWait, false); // no debug
                if(res != GPS_UART_RESULT_SUCESS) {
                    lastErrorId = 174 + res; errorCnt++; // 175 - 180, next free: 181
                    if((res == GPS_UART_RESULT_INIT_ERROR) || (res == GPS_UART_RESULT_BUFF_OVERFLOW_ERROR)) { // fatal errors, stop for some time
                        device.enableInternalTimerInterruptInDeepSleep(15); // sleep for 15s
                        keepTrackingRunning = false;
                        break;
                        // WARNING: IMU keeps running here!
                    }
                }

                // update RTC
                if(couldUpdateTime
                    && (!updatedRTCInThisSession)
                    && (gpsData.parent.tim.thousand == 0) // ONLY if GPS is sending messages at 0 ms
                    && ((gpsData.parent.fix > 0) || (!hasValidTimestamp))) { // first time getting a valid GPS time! (only executed once) -> ONLY AFTER GETTING A FIX (or if hasValidTimestamp is not set yet)
                    // --- BE QUICK HERE ---
                    updatedRTCInThisSession = true;
                    // TODO: add estimateUARTSendTimeMs
                    // delay from UART: (max. 2 * 80 Byte*(8+2) * (1 / 115200 = 0.00868ms/bit) = 13.88ms)
                    validTimeTimerFinished = false;
                    // if debugging: get current milliseconds to compare
                    uint8_t seconds100thOld = 0;
                    uint16_t millisecondsOld = 0;
                    uint32_t timestampOld = 0;
                    if(config.trackerMode == MODE_TESTRUN) {
                        device.rtc.getTimestamp(&timestampOld, &seconds100thOld);
                        millisecondsOld = seconds100thOld * 10;
                    }
                    // start timer
                    tmElements_t timeStruct;
                    bool waitingError = false;
                    uint32_t waitTimeUs = gpsData.parent.tim.thousand;
                    waitTimeUs = (1000 - waitTimeUs) * 1000;
                    if(waitTimeUs == 0) { waitTimeUs = 1; } // should not happen, but just in case
                    //if(waitTimeUs > 900000) { waitTimeUs = 900000; } // WARNING: ADDING INACCURACY, but otherwise might take a second to wait -> GPS receiving gets fucked up
                    if(waitTimeUs == 1000000) { // 1 full second to wait (milliseconds of GPS = 0) -> don't wait
                        breakTime(gpsData.parent.utcTimestamp, timeStruct); // use THIS second
                        waitTimeUs = 0;
                    }
                    else {
                        if(esp_timer_start_once(validTimeTimer, waitTimeUs) == ESP_OK) {
                            while(!validTimeTimerFinished) { ; } // busy waiting until full second
                            breakTime(gpsData.parent.utcTimestamp + 1, timeStruct);
                        }
                        else { waitingError = true; }
                    }
                    if(!waitingError) {
                        if(device.rtc.set(timeStruct.Hour, timeStruct.Minute, timeStruct.Second, timeStruct.Wday, timeStruct.Day, timeStruct.Month, timeStruct.Year)) {
                            hasValidTimestamp = true; // IMPORTANT: update global variable
                            if(config.trackerMode == MODE_TESTRUN) { printf("1HzGPS: updated RTC to: %u -> %d:%d:%d (wait: %u ms)\n", gpsData.parent.utcTimestamp + 1, timeStruct.Hour, timeStruct.Minute, timeStruct.Second, (waitTimeUs/1000)); }    
                        }
                        else { lastErrorId = 129; errorCnt++; }
                    }
                    else { lastErrorId = 160; errorCnt++; } // same error id as above
                    if(config.trackerMode == MODE_TESTRUN) {
                        int64_t timestampMsOld = timestampOld;
                        timestampMsOld = (timestampMsOld * 1000) + millisecondsOld;
                        int64_t timestampMsNow = gpsData.parent.utcTimestamp;
                        timestampMsNow = (timestampMsNow * 1000) + gpsData.parent.tim.thousand;
                        int64_t timestampDiff = timestampMsOld - timestampMsNow;
                        printf("1HzGPS: RTC update: time diff %lld, %lld -> %lld\n", timestampDiff, timestampMsOld, timestampMsNow);
                        printf("1HzGPS: RTC update: %d:%d:%d (wait: %u ms)\n", timeStruct.Hour, timeStruct.Minute, timeStruct.Second, (waitTimeUs/1000));
                    }                                       
                }
                
                // STOP CONDITION UNDERVOLTAGE: read voltage
                uint16_t voltage = device.readSupplyVoltage(true);
                if(voltage < config.battMinVoltage) {
                    // PATH TESTED
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d 1HzGPS: POWER DOWN for %ds\n", ((uint32_t) Timing::millis()), FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
                    state = ST_PWRDWN;
                    stopIMU();
                    device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                    keepTrackingRunning = false;
                    break;
                }
                // STOP CONDITION DID NOT GET TIME: check if timeout reached
                else if((!hasValidTimestamp) && ((esp_timer_get_time() - ttffStartUs) / 1000000ULL > TRACKING_DATA_MODE_1HZ_NO_TIME_TIMEOUT_SECONDS)) { // no time after some seconds -> sleep for a while to not drain the battery
                    // PATH TESTED
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d 1HzGPS: didn't get time after %ds, sleep for %ds\n", ((uint32_t) Timing::millis()), TRACKING_DATA_MODE_1HZ_NO_TIME_TIMEOUT_SECONDS, TRACKING_DATA_MODE_1HZ_NO_TIME_SLEEP_SECONDS); }
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
                    if((fifoDataPointer + currentFifoLen + gpsTrackingModeHeaderLength()) <= FIFO_DATA_LEN) {
                        // add header data
                        uint16_t fifoDataPointerOld = fifoDataPointer;
                        gpsTrackingModeAddData(&gpsData, currentFifoLen, fifoData, &fifoDataPointer);

                        // read acc data into fifoData
                        if(currentFifoLen > 0) {
                            readFifoIMU(fifoData+fifoDataPointer, currentFifoLen, false);
                            if(currentFifoLen >= fifoErrorLimit) { lastErrorId = 61; errorCnt++; } // data loss possible
                            fifoDataPointer += currentFifoLen;
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: pnt %d -> %d (MAX %d, STACK %d), %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataPointerOld, fifoDataPointer, FIFO_DATA_LEN, uxTaskGetStackHighWaterMark(NULL), currentFifoLen, (Timing::millis() - t)); }
                        }
                    }
                    else { // no more space in RAM light sleep memory -> store RAM + newest data from FIFO into flash
                        if(!device.flashPowerOn(false)) { lastErrorId = 15; errorCnt++; } // turn on flash power already

                        if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: full: %d -> write flash\n", ((uint32_t) Timing::millis()), currentFifoLen); }
                        uint16_t fifoDataNewestPointer = 0;

                        // add header data to fifoDataNewest
                        gpsTrackingModeAddData(&gpsData, currentFifoLen, fifoDataNewest, &fifoDataNewestPointer);

                        // read acc data into fifoDataNewest
                        bool animalActive = true; // default: true
                        if(currentFifoLen > 0) {
                            animalActive = readFifoIMU(fifoDataNewest+fifoDataNewestPointer, currentFifoLen, true);
                            if(currentFifoLen >= fifoErrorLimit) { lastErrorId = 65; errorCnt++; } // data loss possible
                            fifoDataNewestPointer += currentFifoLen;
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: fifonewest pnt %d (MAX 1024), read %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataNewestPointer, currentFifoLen, (Timing::millis() - t)); }
                        }
                
                        // read NVS pointer
                        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                        uint16_t flashBlockDeletedPointer = 0;
                        if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                            flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                        }
                        if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                            flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                        }
                        if(config.trackerMode == MODE_TESTRUN) {
                            printf("%d Flash: Need to store: %d Bytes fifoData + %d Bytes fifoDataNew\n", ((uint32_t) Timing::millis()), fifoDataPointer, fifoDataNewestPointer);
                            printf("%d Flash: before storing: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer, flashBlockDeletedPointer);
                        }

                        // store data
                        uint32_t timeNow = ((uint32_t) Timing::millis());
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left before storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                        sequential_write_status_t writeStatus = device.flash.fifoPush(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, fifoData, fifoDataPointer, fifoDataNewest, fifoDataNewestPointer, NULL, 0, MOCK_FLASH_WRITES);
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: after storing: flashPointer: %d, flashOffsetPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer); }
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left after storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                        timeNow = ((uint32_t) Timing::millis()) - timeNow;
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: storage took: %dms\n", ((uint32_t) Timing::millis()), timeNow); }
                        // STOP CONDITION MEMORY FULL
                        if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full -> go into special WIFI TRANSMISSION MODE!
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO full, stop IMU -> go into ST_MEMFULL state in 5 seconds\n", ((uint32_t) Timing::millis())); }
                            stopIMU();
                            memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY; // after memory full, try frequently to get rid of data
                            state = ST_MEMFULL;
                            if(!device.flashPowerOff(false)) { lastErrorId = 26; errorCnt++; } // important!
                            fifoDataPointer = 0; // reset pointer
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }
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
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: FIFO LEN before sleep: %d (reset when >= %d)\n", ((uint32_t) Timing::millis()), fifoLenAfterWifi, config.accInterruptWatermark); }
                            if(fifoLenAfterWifi >= config.accInterruptWatermark) { // fifo full again -> reset it
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: FIFO RESET! DATA lost!\n", ((uint32_t) Timing::millis())); }
                                if(!device.imu.resetFIFO()) { lastErrorId = 31; errorCnt++; } // empty FIFO, do not read
                                fifoDataPointer = 0; // reset RAM data
                            }
                            // print error count
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }

                            // STOP CONDITION NIGHT TIME: stored data -> check if it's time to sleep now (so that data is stored)
                            if(isTimeToGoToBed()) { // time to go to bed
                                // PATH TESTED
                                state = ST_NIGHT_TIME;
                                stopIMU();
                                enableNightTimeInterrupts();
                                keepTrackingRunning = false;
                                break;
                            }

                            // STOP CONDITION FREQUENTLY INTERRUPT FOR DATA TRANSMISSION: only checked after data is stored
                            if((config.gps1HzInterruptAfterSeconds > 0)) {
                                if((esp_timer_get_time() - ttffStartUs) / 1000000ULL > config.gps1HzInterruptAfterSeconds) {
                                    // PATH TESTED
                                    if(config.trackerMode == MODE_TESTRUN) { printf("%d 1HzGPS: short stop to check for gateway (%ds)\n", ((uint32_t) Timing::millis()), config.gps1HzInterruptAfterSeconds); }
                                    device.enableShortInternalTimerInterruptInDeepSleep(20);
                                    keepTrackingRunning = false;
                                    break;
                                    // WARNING: IMU keeps running here!
                                }
                            }

                            // STOP CONDITION: animal not active
                            if(!animalActive) {
                                if(gpsData.parent.fix > 0) { // only stop once got a GPS fix
                                    // PATH TESTED
                                    enterWaitForActivityMode(); // re-configures IMU
                                    keepTrackingRunning = false;
                                    break;
                                }
                            }
                        } 
                    }
                }
                if(config.trackerMode == MODE_TESTRUN) { printf("\n"); }
                //if(addExtraDelay > 0) { device.delay(addExtraDelay); addExtraDelay = 0; } // in case of unsynced GPS messages
                if(TRACKING_DATA_MODE_1HZ_GPS_BLINK && config.doTheBlink) { device.ledGreenOff(); device.ledRedOff(); }
                if(environmentSensorConnected) {
                    if(!device.baro.performMeasurement()) { lastErrorId = 148; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
                }
                uart_flush(UART2_PORT_NUMBER);
                device.enableUart2InterruptInLightSleep();
                esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
                device.lightSleep();
            }
            // end of big while loop (jumps here on break = stop condition)
            free(fifoData);
            device.gpioBOff();
            if(TRACKING_DATA_MODE_1HZ_GPS_BLINK && config.doTheBlink) { device.ledGreenOff(); device.ledRedOff(); }
        }
    }
}

uint16_t imuLightSleepTrackingModeHeaderLength() {
    return 27;
}

void imuLightSleepTrackingModeAddData(uint16_t voltage, uint16_t currentFifoLen, uint8_t *buffer, uint16_t *bufferPointer) { 
    uint32_t timestamp = 0;
    uint8_t milliseconds = 0;
    if(!device.rtc.getTimestamp(&timestamp, &milliseconds)) { lastErrorId = 149; errorCnt++; }
    int16_t temperature = 0, temperatureBmx;
    uint16_t temperatureBmxRaw = 0;
    uint32_t pressure = 0;
    uint32_t humidity = 0;

    if(!device.imu.getTemperatureRaw(temperatureBmxRaw)) { lastErrorId = 158; errorCnt++; }
    temperatureBmx = device.imu.toCelsiusx100(temperatureBmxRaw);
    if(environmentSensorConnected) {
        if(device.baro.getResults()) { // normally waits up to 31ms, but measurement was triggered before lightsleep
            temperature = device.baro.getTemperature(error);
            if(error) { lastErrorId = 20; errorCnt++; }
            pressure = device.baro.getPressure(error);
            if(error) { lastErrorId = 21; errorCnt++; }
            humidity = device.baro.getHumidity(error);
            if(error) { lastErrorId = 22; errorCnt++; }
        }
    }

    HelperBits::addData1_AndIncrementPointer(0x12, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(0x34, buffer, bufferPointer);
    if((config.useMagnetometer) && (config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5E, buffer, bufferPointer); }
    else if((!config.useMagnetometer) && (config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x5F, buffer, bufferPointer); }
    else if((config.useMagnetometer) && (!config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x60, buffer, bufferPointer); }
    else if((!config.useMagnetometer) && (!config.useGyro)) { HelperBits::addData1_AndIncrementPointer(0x61, buffer, bufferPointer); }
    HelperBits::addData4_AndIncrementPointer(timestamp, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(milliseconds, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(lastErrorId, buffer, bufferPointer);
    HelperBits::addData2_AndIncrementPointer(errorCnt, buffer, bufferPointer);
    HelperBits::addData2_AndIncrementPointer((uint16_t) voltage, buffer, bufferPointer);
    HelperBits::addData2Signed_AndIncrementPointer(temperature, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(humidity, buffer, bufferPointer);
    HelperBits::addData4_AndIncrementPointer(pressure, buffer, bufferPointer);
    HelperBits::addData2Signed_AndIncrementPointer(temperatureBmx, buffer, bufferPointer);
    HelperBits::addData2_AndIncrementPointer(currentFifoLen, buffer, bufferPointer);
    if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: UTC %d, TEMP %d/%d, PRESS %d, HUM %d\n", ((uint32_t) Timing::millis()), timestamp, temperature, temperatureBmx, pressure, humidity); } 
}

void imuLightSleepTrackingMode() {
    const uint16_t FIFO_DATA_LEN = 1024 * 16;
    bool keepTrackingRunning = true;
    bool errorDuringInit = false;
    uint16_t fifoDataPointer = 0;

    i2c.begin(I2C_FREQ_HZ_1MHZ); // USING 1MHZ
    
    if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // do not even start when voltage is low
        if(config.trackerMode == MODE_TESTRUN) { printf("imuHD: POWER DOWN before start for %ds\n", FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
        state = ST_PWRDWN;
        stopIMU();
        device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
    }
    else if(isTimeToGoToBed()) { // do not even start when it's bed time
        // TESTED PATH
        state = ST_NIGHT_TIME;
        stopIMU();
        enableNightTimeInterrupts();
    }
    else {
        if(!device.initDataNVS()) { lastErrorId = 14; errorCnt++; }
        if((config.trackerMode == MODE_TESTRUN) && (nightTimeOverrideForcedTimestampStart > 0)) { printf("imuHD: FORCE TRACKING\n"); }
        
        // try one time data transmission
        bool somethingTransmitted = false;
        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
        if(!device.flashPowerOn(true)) { lastErrorId = 137; errorCnt++; } // turn on flash power already (10ms)
        uint32_t timeNow = ((uint32_t) Timing::millis());
        uint8_t commandByte = 0; // just for putting device back into activation state
        if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
            // esp now data transmission (FORCE mode! but only executed if bytes to transmit)
            uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
            somethingTransmitted = dataTransmissionEspNow(true, config.espNowMinBytesToTransmit, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte, true);
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
        }
        else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
            // wifi data transmission (FORCE mode!)
            somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
        }
        if(!device.flashPowerOff(true)) { lastErrorId = 138; errorCnt++; } // turn off flash
        if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: something transmitted: %d\n", somethingTransmitted); }
        if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }

        // check if reconfig needed
        bool configChanged = false;
        if(commandByte == COMMAND_BYTE_CHANGE_CONFIG) {
            // PATH TESTED
            if(!checkIfReconfigNeeded(&device, &config, gatewayAroundConfig, commandByte, 0, &configChanged, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 182; errorCnt++; }
        }

        // evaluate commands
        if(configChanged) {
            // PATH TESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: CONFIGURATION CHANGED -> RESTART\n"); }
            stopIMU();
            device.blinkTimes(12, B_BOTH);
            resetRTCVariables();
            state = ST_FIRST_START;
            device.enableInternalTimerInterruptInDeepSleep(1);
        }
        else if(commandByte == COMMAND_BYTE_DEACTIVATE) {
            // PATH TESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO DE-ACTIVATE\n"); }
            stopIMU();
            resetActivation(); // NVS already initialized
            device.enableInternalTimerInterruptInDeepSleep(1);
        }
        else if(commandByte == COMMAND_BYTE_ACTIVATE_WHEN_NO_GW) {
            // PATH UNTESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO ACTIVATE ONLY WHEN NO GW AROUND\n"); }
            stopIMU();
            resetActivation(); // NVS already initialized
            device.enableInternalTimerInterruptInDeepSleep(60); // sleep longer to give gateway time to store data
        }
	    else if(commandByte == COMMAND_BYTE_MAG_CALIBRATION) {
            // PATH TESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO CALIB MAG\n"); }
            stopIMU();
            state = ST_MAG_CALIBRATION;
            device.enableInternalTimerInterruptInDeepSleep(1);
        }
        else if(commandByte == COMMAND_BYTE_TIME_RESYNC) {
            // UNTESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("Pretrans: RECEIVED CMD TO RESYNC TIME\n"); }
            stopIMU();
            state = ST_TIME_RESYNC;
            device.enableInternalTimerInterruptInDeepSleep(1);
        }
        else {
            uint8_t *fifoData = (uint8_t*) malloc(FIFO_DATA_LEN);
            if(fifoData == NULL) { errorDuringInit = true; }

            // NEW: initalize barometer
            if(environmentSensorConnected) {
                if(!device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { lastErrorId = 117; errorCnt++; } // 4ms
            }

            if(errorDuringInit) { // SHOULD NOT HAPPEN, but could happen if GPS is not connected -> sleep for FIRST_UNDER_VOLTAGE_SLEEP_TIME and try again afterwards 
                lastErrorId = 128; errorCnt++;
                free(fifoData);
                device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME);
                return;
            }

            if(environmentSensorConnected) {
                if(!device.baro.performMeasurement()) { lastErrorId = 147; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
            }
            device.imu.resetFIFO(); // reset fifo to start fresh
            device.enableAccInterruptInDeepSleep();
            esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
            device.lightSleep();

            while(keepTrackingRunning) {
                // read voltage
                uint16_t voltage = device.readSupplyVoltage(true);
                // STOP CONDITION: voltage low
                if(voltage < config.battMinVoltage) {
                    if(config.trackerMode == MODE_TESTRUN) { printf("%d imuHD: POWER DOWN for %ds\n", ((uint32_t) Timing::millis()), FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
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
                    if((fifoDataPointer + currentFifoLen + imuLightSleepTrackingModeHeaderLength()) <= FIFO_DATA_LEN) {
                        // add header data
                        uint16_t fifoDataPointerOld = fifoDataPointer;
                        imuLightSleepTrackingModeAddData(voltage, currentFifoLen, fifoData, &fifoDataPointer);

                        // read acc data into fifoData
                        if(currentFifoLen > 0) {
                            readFifoIMU(fifoData+fifoDataPointer, currentFifoLen, false);
                            if(currentFifoLen >= IMU_FIFO_DEFINITELY_FULL) { lastErrorId = 61; errorCnt++; } // data loss possible
                            fifoDataPointer += currentFifoLen;
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: pnt %d -> %d (MAX %d), %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataPointerOld, fifoDataPointer, FIFO_DATA_LEN, currentFifoLen, (Timing::millis() - t)); }
                        }
                    }
                    else { // no more space in RAM light sleep memory -> store RAM + newest data from FIFO into flash
                        if(!device.flashPowerOn(false)) { lastErrorId = 15; errorCnt++; } // turn on flash power already
                        if(config.doTheBlink) { device.ledRedOn(); }

                        if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: full: %d -> write flash\n", ((uint32_t) Timing::millis()), currentFifoLen); }
                        uint16_t fifoDataNewestPointer = 0;

                        // add header data to fifoDataNewest
                        imuLightSleepTrackingModeAddData(voltage, currentFifoLen, fifoDataNewest, &fifoDataNewestPointer);

                        // read acc data into fifoDataNewest
                        bool animalActive = true;
                        if(currentFifoLen > 0) {
                            animalActive = readFifoIMU(fifoDataNewest+fifoDataNewestPointer, currentFifoLen, true);
                            if(currentFifoLen >= IMU_FIFO_DEFINITELY_FULL) { lastErrorId = 65; errorCnt++; } // data loss possible
                            fifoDataNewestPointer += currentFifoLen;
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: fifonewest pnt %d (MAX 1024), read %d acc bytes in %lldms\n", ((uint32_t) Timing::millis()), fifoDataNewestPointer, currentFifoLen, (Timing::millis() - t)); }
                        }
                
                        // read NVS pointer
                        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                        uint16_t flashBlockDeletedPointer = 0;
                        if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                            flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                        }
                        if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                            flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                        }
                        if(config.trackerMode == MODE_TESTRUN) {
                            printf("%d Flash: Need to store: %d Bytes fifoData + %d Bytes fifoDataNew\n", ((uint32_t) Timing::millis()), fifoDataPointer, fifoDataNewestPointer);
                            printf("%d Flash: before storing: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer, flashBlockDeletedPointer);
                        }

                        // store data
                        uint32_t timeNow = ((uint32_t) Timing::millis());
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left before storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                        sequential_write_status_t writeStatus = device.flash.fifoPush(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, fifoData, fifoDataPointer, fifoDataNewest, fifoDataNewestPointer, NULL, 0, MOCK_FLASH_WRITES);
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: after storing: flashPointer: %d, flashOffsetPointer: %d\n", ((uint32_t) Timing::millis()), flashPointer, flashOffsetPointer); }
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO space left after storing: %d\n", ((uint32_t) Timing::millis()), device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES)); }
                        timeNow = ((uint32_t) Timing::millis()) - timeNow;
                        if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: storage took: %dms\n", ((uint32_t) Timing::millis()), timeNow); }
                        if(config.doTheBlink) { device.ledRedOff(); }
                        // STOP CONDITION: memory full
                        if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { // flash fifo is full -> go into special WIFI TRANSMISSION MODE!
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d Flash: FIFO full, stop IMU -> go into ST_MEMFULL state in 5 seconds\n", ((uint32_t) Timing::millis())); }
                            stopIMU();
                            memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY; // after memory full, try frequently to get rid of data
                            state = ST_MEMFULL;
                            if(!device.flashPowerOff(false)) { lastErrorId = 26; errorCnt++; } // important!
                            fifoDataPointer = 0; // reset pointer
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }
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
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: FIFO LEN before sleep: %d (reset when >= %d)\n", ((uint32_t) Timing::millis()), fifoLenAfterWifi, config.accInterruptWatermark); }
                            if(fifoLenAfterWifi >= config.accInterruptWatermark) { // fifo full again -> reset it
                                if(config.trackerMode == MODE_TESTRUN) { printf("%d FIFO: FIFO RESET! DATA lost!\n", ((uint32_t) Timing::millis())); }
                                if(!device.imu.resetFIFO()) { lastErrorId = 31; errorCnt++; } // empty FIFO, do not read
                                fifoDataPointer = 0; // reset RAM data
                            }
                            // print error count
                            if(config.trackerMode == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Timing::millis()), lastErrorId, errorCnt); }

                            // STOP CONDITION: stored data -> check if it's time to sleep now (so that data is stored)
                            if(isTimeToGoToBed()) { // time to go to bed
                                // PATH TESTED
                                state = ST_NIGHT_TIME;
                                stopIMU();
                                enableNightTimeInterrupts();
                                keepTrackingRunning = false;
                                break;
                            }

                            // STOP CONDITION: animal not active
                            if(!animalActive) {
                                // PATH TESTED
                                enterWaitForActivityMode(); // re-configures IMU
                                keepTrackingRunning = false;
                                break;
                            }
                        } 
                    }
                }
                if(config.trackerMode == MODE_TESTRUN) { printf("\n"); }
                if(environmentSensorConnected) {
                    if(!device.baro.performMeasurement()) { lastErrorId = 148; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
                }
                device.enableAccInterruptInDeepSleep();
                device.lightSleep();
            }
            // end of big while loop
            free(fifoData);
        }
    }
}

void gpsCheckAliveAndMaybeChangeBaudrateWrapper() {
    gps.checkAliveAndMaybeChangeBaudrate(&device, true, true);
}

void gpsTestWrapper() {
    device.gpioBOn();
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');
    esp_gps_t gpsData = { };
    gps_get_fix_config_t gpsConfig = {
        .timeoutSeconds = 300,
        .timeoutNotEvenTimeSeconds = 180,
        .minHDOP = 3.0f,
        .afterFixMaxWaitOnHDOP = 10,
        .waitAfterFixUntilZeroMs = true,
        .setRTCTime = false,
        .blinkLeds = true, // always on
        .debug = true };
    get_fix_result_t fixResult = gps.tryToGetFix(&gpsData, &gpsConfig, &device);
    if(fixResult == GPS_FIX_SUCCESS_AND_RTC_UPDATED) { printf("GPS SUCCESS: %.6f/%.6f in %ds\n", gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.ttfMilliseconds / 1000); }
    else { printf("GPS FAILED\n"); }
    device.gpioBOff();
}

extern "C" void app_main() {
    while(1) {
        if((config.trackerMode == MODE_TESTRUN) || (config.trackerMode == MODE_PRODUCTIVE)) {
            if((config.trackerMode == MODE_TESTRUN) && (state != ST_TRACK)) { printf("-----\nState: %d\n", state); }
            /** ---------------- BOOT STATE ---------------- */
            if(state == ST_BOOT) { // not doing anything here!
                esp_reset_reason_t resetReason = device.getLastResetReason();
                if(resetReason != ESP_RST_POWERON) {
                    lastErrorId = 161 + resetReason; errorCnt++; // check for brownouts, errorIds 161 - 171 (next free: 172)
                } 
                state = ST_SERIAL_MENUE;
                if(!device.serialMenueGetSelfTestDone(NVS_FLASH_SELFTEST_DONE)) { device.enableInternalTimerInterruptInDeepSleep(1); } // restart system into next state IMMEDIATELY
                else { device.enableInternalTimerInterruptInDeepSleep(BOOT_DELAY_SECONDS); } // restart system into next state
            }
            /** ---------------- SERIAL MENUE STATE ---------------- */
            else if(state == ST_SERIAL_MENUE) {
                if(!device.serialMenue(true, NVS_FLASH_SELFTEST_DONE, NULL, NVS_FLASH_TAG_ACTIVATED_BY_WIFI, gpsCheckAliveAndMaybeChangeBaudrateWrapper, gpsTestWrapper)) { // WARNING: CPU clocked to 10 MHz
                    state = ST_FIRST_START;
                }
                device.enableShortInternalTimerInterruptInDeepSleep(100); // 100ms
            }
            /** ---------------- FIRST START STATE ---------------- */
            else if(state == ST_FIRST_START) { // custom wake stub not running -> no fifo
                // keeping power on values (PIN_POWER, PIN_LED_RED) in deep sleep
                device.keepSensorPowerOnInDeepSleep();

                i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
                device.delay(10);

                // read config (BEFORE THAT: ALL CONFIG VALUES ARE ZERO!)
                if(!readConfigFromNVS(&device, &config, true)) { lastErrorId = 181; errorCnt++; }
                uint8_t configErrorId = 0;
                if(!configIsPlausible(&config, &configErrorId)) {
                    lastErrorId = 185; errorCnt++;
                    printf("Reset: ERROR config not plausible (errorId: %d)\n", configErrorId);
                    device.blinkTimes(25, B_RED);
                }
                printConfigurationHash(&config);
                printStoredWiFis();

                // RTC: get current timestamp (without millis)
                uint32_t timestamp = 0;
                if(!device.rtc.getTimestamp(&timestamp, NULL)) { lastErrorId = 41; errorCnt++; }

                // ENV: check if working
                device.sensorPowerOn();
                device.delay(200);
                environmentSensorConnected = i2c.isAlive(BARO_BME680_ADDRESS);
                bool imuFOCDone = device.imu.accGyroFOCAlreadyDoneAndStoredInNVM();
                device.sensorPowerOff();
                if(!environmentSensorConnected) {
                    printf("Reset: NO ENV SENSOR\n");
                    device.blinkTimes(10, B_RED);
                }
                else { printf("Reset: ENV SENSOR ALIVE\n"); }
                if(!imuFOCDone) {
                    printf("Reset: NO FOC OF IMU\n");
                    device.blinkTimes(15, B_RED);
                }
                else { printf("Reset: IMU FOC DONE\n"); }
                device.delay(200);

                // check if GPS connected
                if((config.trackingDataMode == TRACKING_DATA_MODE_1HZ_GPS_AND_IMU) || (config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING)) {
                    gps.checkAliveAndMaybeChangeBaudrate(&device, true, config.trackerMode == MODE_TESTRUN);
                }

                // check NVS if already activated & timestamp is valid, print write pointer also when in PRODUCTIVE
                if(!device.initDataNVS()) { lastErrorId = 42; errorCnt++; }
                uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                uint8_t myMac[6] = { 0 };
                esp_efuse_mac_get_default(myMac);
                printf("Reset: FIRMWARE: V%d, CONFIG: %s V%d\n", WILDFI_SOFTWARE_VERSION, WILDFI_CONFIG_NAME, WILDFI_CONFIG_VERSION);
                if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                    uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER);
                    uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
                    printf("Reset: FLASH: w %d.%d, s %d.%d, HEAP: %d\n", flashPointer, flashOffsetPointer, sendPagePointer, sendPageOffsetPointer, heap_caps_get_free_size(MALLOC_CAP_8BIT));
                }
                else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                    uint16_t sendNextBlockPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                    uint16_t sendNextHalfBlockPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER); 
                    printf("Reset: FLASH: %d.%d, sb %d.%d, HEAP: %d\n", flashPointer, flashOffsetPointer, sendNextBlockPointer, sendNextHalfBlockPointer, heap_caps_get_free_size(MALLOC_CAP_8BIT));
                }
                printf("Reset: MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
                if(device.getLastResetReason() == ESP_RST_BROWNOUT) { lastErrorId = 43; errorCnt++; }

                // read magnetometer offsets from NVS
                bool neverWritten = false;
                magHardIronOffsetX = device.nvsReadINT16(NVS_MAG_CALIB_OFFSET_X, &neverWritten);
                magHardIronOffsetY = device.nvsReadINT16(NVS_MAG_CALIB_OFFSET_Y, &neverWritten);
                magHardIronOffsetZ = device.nvsReadINT16(NVS_MAG_CALIB_OFFSET_Z, &neverWritten);
                printf("Reset: Mag Offsets %d/%d/%d\n", magHardIronOffsetX, magHardIronOffsetY, magHardIronOffsetZ);
                printf("Reset: Errors %d/%d\n", lastErrorId, errorCnt);

                // activation state
                if(config.activationMode == ACTIVATION_MODE_SKIP) { isActivated = true; } // store state in RTC -> always activated
                else if(config.activationMode == ACTIVATION_MODE_STORE_PERMANENTLY) {
                    uint16_t activated = device.nvsReadUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI);
                    isActivated = (activated > 0); // store state in RTC
                }
                else if(config.activationMode == ACTIVATION_MODE_ON_EVERY_START) { isActivated = false; } // store state in RTC -> not activated after reset

                if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 98; errorCnt++; } // disable, just in case something weird happened

                bool timeIsValid = device.rtc.timeIsValidNoUndervoltage(error);
                if(error) { lastErrorId = 159; errorCnt++; }
                if((timestamp > 1628689096) && timeIsValid) { hasValidTimestamp = true; }
                else {
                    hasValidTimestamp = false;
                    if(!device.rtc.set(0, 0, 0, 0, 1, 1, 2000))  { lastErrorId = 173; errorCnt++; } // resetting time
                    device.delay(20);
                    if(!device.rtc.getTimestamp(&timestamp, NULL)) { lastErrorId = 174; errorCnt++; } // update timestamp
                }
                if(config.trackerMode == MODE_TESTRUN) {
                    tmElements_t timeStruct;
                    breakTime(timestamp, timeStruct);
                    printf("Reset: timestamp %u (%02d:%02d:%02d, %02d.%02d.%d), voltageOk %d -> hasValidTimestamp %d\n", timestamp, timeStruct.Hour, timeStruct.Minute, timeStruct.Second, timeStruct.Day, timeStruct.Month, timeStruct.Year, timeIsValid, hasValidTimestamp);
                }
                
                if(hasValidTimestamp || config.skipGetTime) { // some brownout or other reset of MCU -> time still ok
                    setNextDataTransmissionTimestamp(false, timestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
                    if(isActivated) { // also already activated -> start immediately
                        if(config.trackerMode == MODE_TESTRUN) { printf("Reset: timestamp ok %d (skip %d) and activated -> START!\n", hasValidTimestamp, config.skipGetTime); }
                        state = ST_START;
                    }
                    else { // time okay, but not activated -> go to activation
                        if(config.trackerMode == MODE_TESTRUN) { printf("Reset: timestamp ok %d (skip %d), NOT activated (mode %d) -> activation state!\n", hasValidTimestamp, config.skipGetTime, ACTIVATION_MODE); }
                        if(config.activationSource == ACTIVATION_BY_WIFI) {
                            if(!device.rtc.setHourlyInterrupt(ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE)) { lastErrorId = 95; errorCnt++; } // time valid, so better activate the rtc interrupt
                        }
                        state = ST_WAIT_FOR_ACTIVATION;
                    }
                }
                else { // time not okay, move to time state
                    if(config.trackerMode == MODE_TESTRUN) { printf("Reset: timestamp not valid, activation = %d (mode %d)!\n", isActivated, ACTIVATION_MODE); }
                    getTimeAttemptCnt = 0; // first try
                    state = ST_GET_TIME;
                }
                device.enableInternalTimerInterruptInDeepSleep(SLEEP_TIME_AFTER_START); // restart system into next state
            }
            /** ---------------- GET TIME STATE ---------------- */
            else if(state == ST_GET_TIME) {
                if(config.trackerMode == MODE_TESTRUN) { printf("Time: try getting time! V = %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // voltage low, go to power down state
                    if(config.trackerMode == MODE_TESTRUN) { printf("Time: undervoltage (%d)! Don't scan!\n", device.readSupplyVoltageFromWakeStub()); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                }
                else {
                    // try to get time over wifi or GPS
                    uint16_t sleepTimeWhenGetTimeFailed = config.timeBetweenGetTimeRetriesSeconds;
                    if(getTimeAttemptCnt < 2) { sleepTimeWhenGetTimeFailed = 30; } // shorter time
                    getTimeAttemptCnt++;
                    if(getTimeAttemptCnt > 100) { getTimeAttemptCnt = 100; } // avoid overflow
                    if(device.initWiFi()) {
                        uint8_t foundArrayId = 0;
                        uint8_t foundOnChannel = 0;
                        bool connectionTimeout = false;
                        uint32_t scanStartTime = ((uint32_t) Timing::millis());
                        i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
                        if(device.scanForWiFisOnAllChannels((config.trackerMode == MODE_TESTRUN), TIME_WIFI_SSIDS, TIME_WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, TIME_WIFI_OUTPUT_POWER, 120, 500)) { 
                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500); }
                            if(foundOnChannel > 0) { // found wifi, try to connect
                                if(foundArrayId == 0) { // enter special download mode to force data transmission, afterwards delete memory and sleep forever
                                    device.disconnectAndStopWiFi();
                                    if(config.trackerMode == MODE_TESTRUN) { printf("Time: FOUND FORCE DOWNLOAD WIFI (chan %d, index %d)\n", foundOnChannel, foundArrayId); }
                                    state = ST_FORCE_DOWNLOAD;
                                    device.enableInternalTimerInterruptInDeepSleep(3);
                                }
                                else { // just get time
                                    device.ledGreenOn();
                                    if(config.trackerMode == MODE_TESTRUN) { printf("Time: wifi found (chan %d, index %d), hasValidTimestamp: %d\n", foundOnChannel, foundArrayId, hasValidTimestamp); }
                                    if(device.connectToWiFiAfterScan(TIME_WIFI_SSIDS[foundArrayId], TIME_WIFI_PASSWORDS[foundArrayId], foundOnChannel)) {
                                        uint32_t connectStartedTime = ((uint32_t) Timing::millis());
                                        while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                                            device.delay(20);
                                            if(((uint32_t) Timing::millis()) - connectStartedTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                                                if(config.trackerMode == MODE_TESTRUN) { printf("Time: TIMEOUT CONNECT!\n"); }
                                                connectionTimeout = true;
                                                break;
                                            }
                                        }
                                        if(connectionTimeout || (device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: STRANGE could not connect, but wifi was seen (pw wrong, no internet): %d!\n", device.connectedToWiFi()); }
                                            lastErrorId = 38; errorCnt++;
                                        }
                                        else { // connected to wifi
                                            uint32_t timestampUTC = 0;
                                            uint16_t millisecondsUTC = 0;
                                            if(!device.getNTPTimestampUTC(true, timestampUTC, millisecondsUTC, 7000, "pool.ntp.org")) { // will block, will set RTC time
                                                if(config.trackerMode == MODE_TESTRUN) { printf("Time: UTC get time error!\n"); }
                                                lastErrorId = 39; errorCnt++;
                                            }
                                            else {
                                                if(config.trackerMode == MODE_TESTRUN) { printf("Time: set %d + 1, UTC milliseconds: %d -> waited for %dms\n", timestampUTC, millisecondsUTC, 1000 - millisecondsUTC); }
                                                hasValidTimestamp = true;
                                                setNextDataTransmissionTimestamp(false, timestampUTC+1, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
                                            }
                                        }
                                        device.disconnectAndStopWiFi();
                                    }
                                    else { lastErrorId = 37; errorCnt++; device.disconnectAndStopWiFi(); }
                                    device.ledGreenOff();
                                    // check result of getting timestamp attempt
                                    if(hasValidTimestamp) {
                                        if(isActivated) {
                                            state = ST_START;
                                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: all done and already activated -> MOVE TO START in %ds!\n", TIME_SLEEP_AFTER_GOT_TIME); }
                                        }
                                        else {
                                            state = ST_WAIT_FOR_ACTIVATION;
                                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: all done -> move to ACTIVATION in %ds!\n", TIME_SLEEP_AFTER_GOT_TIME); }
                                        }
                                        device.blinkTimes(6, B_GREEN);
                                        device.enableInternalTimerInterruptInDeepSleep(TIME_SLEEP_AFTER_GOT_TIME); // restart in x seconds and move to activation
                                    }
                                    else {
                                        if(config.trackerMode == MODE_TESTRUN) { printf("Time: wifi was seen, but still something missing (pw wrong, no internet) -> SLEEP for %ds\n", sleepTimeWhenGetTimeFailed); }
                                        if(!USE_GPS_TO_GET_TIME_AT_BEGINNING) { device.enableInternalTimerInterruptInDeepSleep(sleepTimeWhenGetTimeFailed); }
                                        else { tryGetTimeOverGPS(sleepTimeWhenGetTimeFailed); }
                                    }
                                }
                            }
                            else {
                                if(config.trackerMode == MODE_TESTRUN) { printf("Time: wifi NOT found\n"); }
                                device.disconnectAndStopWiFi();
                                if(!USE_GPS_TO_GET_TIME_AT_BEGINNING) { device.enableInternalTimerInterruptInDeepSleep(sleepTimeWhenGetTimeFailed); } // sleep some time and try again afterwards   
                                else { tryGetTimeOverGPS(sleepTimeWhenGetTimeFailed); }
                            }
                        }
                        else {
                            lastErrorId = 32; errorCnt++; device.disconnectAndStopWiFi();
                            if(!USE_GPS_TO_GET_TIME_AT_BEGINNING) { device.enableInternalTimerInterruptInDeepSleep(sleepTimeWhenGetTimeFailed); }
                            else { tryGetTimeOverGPS(sleepTimeWhenGetTimeFailed); }
                        }
                    }
                    else {
                        lastErrorId = 33; errorCnt++;
                        if(!USE_GPS_TO_GET_TIME_AT_BEGINNING) { device.enableInternalTimerInterruptInDeepSleep(sleepTimeWhenGetTimeFailed); }
                        else { tryGetTimeOverGPS(sleepTimeWhenGetTimeFailed); }
                    }
                }
            }
            /** ---------------- WAIT FOR ACTIVATION STATE ---------------- */
            else if(state == ST_WAIT_FOR_ACTIVATION) { // custom wake stub not running
                i2c.begin(I2C_FREQ_HZ_400KHZ);
                nightTimeOverrideForcedTimestampStart = 0; // reset all forced tracking commands when coming back into activation mode (e.g., after deactivation, change config, mag calibration)
                if(config.activationSource == ACTIVATION_BY_ESPNOW) {
                    if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // voltage low, stay in that mode but sleep
                        // PATH TESTED
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activation: Undervoltage (%d)! Don't scan, sleep for %d!\n", device.readSupplyVoltageFromWakeStub(), FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); }
                        device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME);
                    }
                    else {
                        if(config.espNowCustomRFCalibration) { handleCustomRFCalibration(true); }
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activation: NOT ACTIVATED (src: ESPNOW)!\n"); }
                        if(!device.initESPNOWStationary(config.espNowLongRange, config.dataTransOutputPower, true, (wifi_phy_rate_t) config.espNowDataRate)) { lastErrorId = 120; errorCnt++; } // 23ms
                        if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 107; errorCnt++; } // 0ms
                        uint8_t commandByte = 0; // currently unused
                        if(config.doTheBlink) { device.ledRedOn(); } // visual feedback during activation scan
                        bool gatewaySeen = gatewaySeenEspNow(&commandByte, 0xFFFFFFFF); // when in activation: do not send how many bytes to transmit (because would need to look into NVS)
                        if(config.doTheBlink) { device.ledRedOff(); } // visual feedback during activation scan
                        if(gatewaySeen && (commandByte == COMMAND_BYTE_ACTIVATE)) {
                            // after tag around message: confirm with got activated message
                            isActivated = true;
                            uint8_t data[ESPNOW_META_MSG_GOT_ACTIVATED_LEN] = { 0 };
                            data[0] = ESPNOW_META_MSG_GOT_ACTIVATED;
                            device.broadcastESPNOWData(data, ESPNOW_META_MSG_GOT_ACTIVATED_LEN); // spit it out
                        }
                        else if(gatewaySeen && (commandByte == COMMAND_BYTE_ACTIVATE_WHEN_NO_GW)) {
                            // if command received to wait with start until no gateway is seen, then set this variable
                            activateWhenNoGWAroundActive = true;
                        }
                        if(!gatewaySeen) {
                            // next time when NO gateway was seen, but COMMAND_BYTE_ACTIVATE_WHEN_NO_GW was received previously: activate
                            if(activateWhenNoGWAroundActive) {
                                activateWhenNoGWAroundActive = false;
                                isActivated = true;
                                // no broadcast here because will not be received!
                            }
                        }
                        device.stopESPNOW(); // 5ms
                        esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep

                        // check if reconfig needed
                        bool configChanged = false;
                        if(commandByte == COMMAND_BYTE_CHANGE_CONFIG) {
                            // PATH TESTED
                            if(!checkIfReconfigNeeded(&device, &config, gatewayAroundConfig, commandByte, 0, &configChanged, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 182; errorCnt++; }
                        }

                        // evaluate commands
                        if(configChanged) {
                            // PATH TESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: CONFIGURATION CHANGED -> RESTART\n"); }
                            device.blinkTimes(12, B_BOTH);
                            resetRTCVariables();
                            state = ST_FIRST_START;
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        // only allow mag calibration if not already calibrated (otherwise endless loop of mag calibration mode when gateway keeps on)
                        else if((commandByte == COMMAND_BYTE_MAG_CALIBRATION) && (magHardIronOffsetX == 0) && (magHardIronOffsetY == 0) && (magHardIronOffsetZ == 0)) {
                            // PATH TESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: RECEIVED CMD TO CALIB MAG\n"); }
                            state = ST_MAG_CALIBRATION;
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        else if(commandByte == COMMAND_BYTE_TIME_RESYNC) {
                            // PATH TESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: RECEIVED CMD TO RESYNC TIME\n"); }
                            state = ST_TIME_RESYNC;
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        else if((commandByte == COMMAND_BYTE_TIME_SYNC_ACTIVATION) && (commandByteTimeSyncActivationTrys < 5)) {
                            // PATH TESTED
                            commandByteTimeSyncActivationTrys++;
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: RECEIVED CMD TO RESYNC TIME ONCE (%d of 5)\n", commandByteTimeSyncActivationTrys); }
                            state = ST_TIME_RESYNC;
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        else {
                            // check if is activated
                            if(isActivated) {
                                if(config.activationMode == ACTIVATION_MODE_STORE_PERMANENTLY) {
                                    if(!device.initDataNVS()) { lastErrorId = 125; errorCnt++; }
                                    else {
                                        if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 1)) { lastErrorId = 126; errorCnt++; } // write activation into NVS
                                    }
                                }
                                state = ST_START;
                                if(config.trackerMode == MODE_TESTRUN) { printf("Activation: all done -> move to START in 1s!\n"); }
                                device.enableInternalTimerInterruptInDeepSleep(1); // restart in 1 second and start
                            }
                            else {
                                if((ACTIVATION_BY_ESPNOW_RETRY_NO_MAG_TIME_ENABLED == true) && (magHardIronOffsetX == 0) && (magHardIronOffsetY == 0) && (magHardIronOffsetZ == 0)) { // mag not yet calibrated -> retry activation more often
                                    if(config.trackerMode == MODE_TESTRUN) { printf("Activation: NO, seen: %d, cmd: 0x%02X -> SLEEP SHORT %ds (mag not calibrated)\n", gatewaySeen, commandByte, ACTIVATION_BY_ESPNOW_RETRY_NO_MAG_TIME_SEC); }
                                    device.enableInternalTimerInterruptInDeepSleep(ACTIVATION_BY_ESPNOW_RETRY_NO_MAG_TIME_SEC);
                                }
                                else {
                                    if(config.trackerMode == MODE_TESTRUN) { printf("Activation: NO, seen: %d, cmd: 0x%02X -> SLEEP %ds\n", gatewaySeen, commandByte, config.activationByEspNowRetrySeconds); }
                                    device.enableInternalTimerInterruptInDeepSleep(config.activationByEspNowRetrySeconds);
                                }
                            }
                        }
                    }
                }
                else if(config.activationSource == ACTIVATION_BY_WIFI) {  
                    if(config.trackerMode == MODE_TESTRUN) { printf("Activation: NOT ACTIVATED (src: WIFI)!\n"); }
                    if(device.getWakeUpReason() == BY_EXT0) { // wake up by RTC -> was before in ACTIVATION STATE!
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activation: booted through RTC interrupt -> CLEAR\n"); }
                        if(!device.rtc.resetInterruptFlags()) { lastErrorId = 97; errorCnt++; }
                    }
                    else { // can only be BY_TIMER, first time entering activation state -> activate hourlyInterrupt
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activation: first time activation -> enable hourlyInterrupt!\n"); }
                        if(!device.rtc.setHourlyInterrupt(ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE)) { // DANGER: interrupt pin not cleared automatically here, never forget to deactivate that!
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: could not set RTC interrupt!\n"); }
                            lastErrorId = 35; errorCnt++;
                        }
                    }
                    if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // voltage low, stay in that mode but sleep for an hour or more -> needs to be here because otherwise setHourlyInterrupt might not have been set!
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activation: Undervoltage (%d)! Don't scan, sleep on RTC interrupt!\n", device.readSupplyVoltageFromWakeStub()); }
                        device.enableRTCInterruptInDeepSleep(); // sleep for an hour or more
                    }
                    else {
                        bool i2cError = false;
                        uint8_t currentHour = device.rtc.getHours(i2cError);
                        if(!i2cError) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: Current hour: %d, wake on minute %d, wake on modulo %d, wake-up reason %d\n", currentHour, ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE, ACTIVATION_WAKE_UP_ON_MODULO_HOURS, device.getWakeUpReason()); }
                            if((currentHour % ACTIVATION_WAKE_UP_ON_MODULO_HOURS == 0) || (device.getWakeUpReason() != BY_EXT0)) { // if BY_TIMER means first scan ever -> try it immediately                
                                if(device.initWiFi()) {
                                    uint8_t foundArrayId = 0;
                                    uint8_t foundOnChannel = 0;
                                    uint32_t scanStartTime = ((uint32_t) Timing::millis());
                                    if(config.doTheBlink) { device.ledGreenOn(); } // visual feedback during activation scan
                                    if(device.scanForWiFisOn1and6and11(ACTIVATION_WIFI_SSIDS, 1, &foundArrayId, &foundOnChannel, ACTIVATION_WIFI_OUTPUT_POWER, 120, 500)) { 
                                        if(config.trackerMode == MODE_TESTRUN) { printf("Activation: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500); }
                                        if(foundOnChannel > 0) { // found wifi, set activation = true
                                            device.disconnectAndStopWiFi();
                                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: activation wifi found (chan %d, index %d), isActivated: %d, hasValidTimestamp: %d\n", foundOnChannel, foundArrayId, isActivated, hasValidTimestamp); }
                                            if(config.activationMode == ACTIVATION_MODE_ON_EVERY_START) { isActivated = true; } // just store in RTC variable, do not store in NVS
                                            else {
                                                if(!device.initDataNVS()) { lastErrorId = 91; errorCnt++; }
                                                else {
                                                    if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 1)) { lastErrorId = 92; errorCnt++; } // write activation into NVS
                                                    else { isActivated = true; } // now is activated!
                                                }
                                            }

                                            if(isActivated) {
                                                state = ST_START;
                                                if(config.trackerMode == MODE_TESTRUN) { printf("Activation: all done -> disable RTC interrupt, move to START in 1s!\n"); }
                                                if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 99; errorCnt++; } // IMPORTANT: disable RTC interrupt (otherwise int pin driven permanently after one hour)
                                                device.enableInternalTimerInterruptInDeepSleep(1); // restart in 1 second and start
                                            }
                                            else {
                                                if(config.trackerMode == MODE_TESTRUN) { printf("Activation: wifi was seen, but other error -> SLEEP (rtc)\n"); }
                                                device.enableRTCInterruptInDeepSleep();
                                            }
                                        }
                                        else {
                                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: activation wifi NOT found -> SLEEP (rtc)\n"); }
                                            device.disconnectAndStopWiFi();
                                            device.enableRTCInterruptInDeepSleep();
                                        }  
                                    }
                                    else { device.disconnectAndStopWiFi(); lastErrorId = 93; errorCnt++; device.enableRTCInterruptInDeepSleep(); }
                                    if(config.doTheBlink) { device.ledGreenOff(); } // visual feedback during activation scan
                                }
                                else { lastErrorId = 94; errorCnt++; device.enableRTCInterruptInDeepSleep(); }
                            }
                            else {
                                if(config.trackerMode == MODE_TESTRUN) { printf("Activation: DON'T SCAN -> SLEEP\n"); }
                                device.enableRTCInterruptInDeepSleep(); // sleep again
                            }
                        }
                        else { lastErrorId = 96; errorCnt++; device.enableRTCInterruptInDeepSleep(); }
                    }
                }
            }
            /** ---------------- START STATE ---------------- */
            else if(state == ST_START) {
                // full RF calibration in case ESP NOW is used and activation was skipped
                if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                    if(config.espNowCustomRFCalibration) {
                        handleCustomRFCalibration(true);
                    }
                }

                // blink a couple of times
                if(config.doTheBlink) { device.blinkTimes(5, B_BOTH); }

                // init BMX
                i2c.begin(I2C_FREQ_HZ_400KHZ);
                startIMU(true); // reading trim data here
        
                // move to next state
                state = ST_TRACK;

                if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP) { device.enableAccInterruptInDeepSleep(); }
                else if(config.trackingDataMode == TRACKING_DATA_MODE_1HZ_GPS_AND_IMU) { device.enableShortInternalTimerInterruptInDeepSleep(20); }
                else if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                else if(config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING) { device.enableShortInternalTimerInterruptInDeepSleep(20); }
            }
            /** ---------------- POWER DOWN STATE (IMU stopped!) ---------------- */
            else if(state == ST_PWRDWN) {
                nightTimeOverrideForcedTimestampStart = 0; // reset all forced commands
                if(config.trackerMode == MODE_TESTRUN) { printf("(PWR_DWN) V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() <= config.battRestartVoltage) { // voltage still too low for restarting
                    // PATH TESTED
                    uint8_t commandByte = 0;
                    if(POWER_DOWN_MODE_DATA_TRANS_WHEN_VOLTAGE_HALF_OK && (config.battRestartVoltage > config.battMinVoltage)) {
                        if(device.readSupplyVoltageFromWakeStub() > (config.battMinVoltage + ((config.battRestartVoltage - config.battMinVoltage) / 2))) { // voltage recovered half
                            // try one time data transmission
                            if(!device.initDataNVS()) { lastErrorId = 152; errorCnt++; }
                            i2c.begin(I2C_FREQ_HZ_400KHZ); // DO NOT use 1MHz
                            bool somethingTransmitted = false;
                            uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                            if(!device.flashPowerOn(true)) { lastErrorId = 150; errorCnt++; } // turn on flash power already (10ms)
                            uint32_t timeNow = ((uint32_t) Timing::millis());

                            if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                                // esp now data transmission (FORCE mode! minBytes to transmit = 0 to send out a tag around msg)
                                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                                somethingTransmitted = dataTransmissionEspNow(true, 0, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte, true);
                                if(config.trackerMode == MODE_TESTRUN) { printf("Pwrdwn: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            }
                            else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                                // wifi data transmission (FORCE mode!)
                                somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                                if(config.trackerMode == MODE_TESTRUN) { printf("Pwrdwn: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            }
                            if(!device.flashPowerOff(false)) { lastErrorId = 151; errorCnt++; } // turn off flash
                            if(config.trackerMode == MODE_TESTRUN) { printf("Pwrdwn: something transmitted: %d\n", somethingTransmitted); }
                            if(config.trackerMode == MODE_TESTRUN) { printf("Pwrdwn: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                        }
                    }
                    // check if reconfig needed
                    bool configChanged = false;
                    if(commandByte == COMMAND_BYTE_CHANGE_CONFIG) {
                        // PATH TESTED
                        if(!device.initDataNVS()) { lastErrorId = 152; errorCnt++; }
                        if(!checkIfReconfigNeeded(&device, &config, gatewayAroundConfig, commandByte, 0, &configChanged, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 182; errorCnt++; }
                    }

                    // evaluate commands
                    if(configChanged) {
                        // PATH TESTED
                        if(config.trackerMode == MODE_TESTRUN) { printf("Pwrdwn: CONFIGURATION CHANGED -> RESTART\n"); }
                        device.blinkTimes(12, B_BOTH);
                        resetRTCVariables();
                        state = ST_FIRST_START;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else if(commandByte == COMMAND_BYTE_DEACTIVATE) {
                        // PATH TESTED
                        if(!device.initDataNVS()) { lastErrorId = 152; errorCnt++; }
                        if(config.trackerMode == MODE_TESTRUN) { printf("Pwrdwn: RECEIVED CMD TO DE-ACTIVATE\n"); }
                        resetActivation(); // NVS already initialized
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    // DO NOT ALLOW to run MAG calibration, because power low!
                    else {
                        // PATH TESTED
                        device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                    }
                }
                else { // voltage okay again, restart into tracking state, restart IMU
                    // PATH TESTED
                    if(config.trackerMode == MODE_TESTRUN) { printf("RESTART!\n"); }
                    i2c.begin(I2C_FREQ_HZ_1MHZ);
                    startIMU(false);
                    state = ST_TRACK;
                    if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP) { device.enableAccInterruptInDeepSleep(); }
                    else if(config.trackingDataMode == TRACKING_DATA_MODE_1HZ_GPS_AND_IMU) { device.enableInternalTimerInterruptInDeepSleep(1); }
                    else if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                    else if(config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING) { device.enableShortInternalTimerInterruptInDeepSleep(20); }
                }
            }
            /** ---------------- TRACKING STATE ---------------- */
            else if(state == ST_TRACK) {
                if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP)          { imuDeepSleepTrackingMode(); }
                else if(config.trackingDataMode == TRACKING_DATA_MODE_1HZ_GPS_AND_IMU)       { imuAnd1HzGPSTrackingMode(); }
                else if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP)    { imuLightSleepTrackingMode(); }
                else if(config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING)        { deadReckoning(); }
            }
            /** ---------------- NIGHT TIME STATE (IMU OFF!) ---------------- */
            else if(state == ST_NIGHT_TIME) { // wake up by regular RTC interrupt or by daily interrupt
                nightTimeCnt++;
                i2c.begin(I2C_FREQ_HZ_400KHZ);
                if(isTimeToGoToBed(NIGHTTIME_DURATION_BASED_ON_V_RETURN_DEBOUNCE)) { // still time to be in bed (add 40 mV for debouncing when using NIGHTTIME_DURATION_BASED_ON_VOLTAGE_xxV)
                    /** OPTION 1: do not do anything while in sleep mode -> then this is an error case */
                    if(config.nightTimeMode == NIGHTTIME_MODE_ONLY_SLEEP) {
                        // that would be a strange error (RTC should wake up when sleep time is over), but happened once
                        if(config.trackerMode == MODE_TESTRUN) { printf("STILL BED TIME - SHOULD NEVER HAPPEN! Wake up by %d\n", device.getWakeUpReason()); }
                        lastErrorId = 111; errorCnt++;
                        if(!device.rtc.setDailyInterrupt(config.nightTimeTurnOnHour, config.nightTimeTurnOnMinute)) { lastErrorId = 112; errorCnt++; } // just in case, set daily interrupt again, WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                        device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
                    }
                    /** OPTION 2: transmit data every now and then, even in sleep */
                    if(config.nightTimeMode == NIGHTTIME_MODE_TRY_DATATRANS) {
                        // PATH TESTED
                        if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // voltage low, go to power down
                            // PATH TESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: undervoltage during night time -> PWR DWN\n"); }
                            if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 106; errorCnt++; } 
                            state = ST_PWRDWN;
                            nightTimeCnt = 0;
                            device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                        }
                        else if(nightTimeModeDataTransIsDeepestNight()) {
                            // PATH TESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: deepest night - don't even scan\n"); }
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
                            if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: flashPointer: %d, flashOffsetPointer: %d\n", flashPointer, flashOffsetPointer); }
                            if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: transmit try even when nothing to transmit\n"); }
                            if(!device.flashPowerOn(true)) { lastErrorId = 135; errorCnt++; } // turn on flash power already (5ms)
                            uint32_t timeNow = ((uint32_t) Timing::millis());

                            if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                                // esp now data transmission (FORCE mode!), IMPORTANT: calling with minBytesToTransmit = 0 -> FORCING a scan to get commandByte even if no data to send
                                somethingTransmitted = dataTransmissionEspNow(true, 0, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte, true);
                                if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            }
                            else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                                // wifi data transmission (FORCE mode!)
                                somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                                if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            }

                            if(!device.flashPowerOff(true)) { lastErrorId = 136; errorCnt++; } // turn off flash
                            if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: something transmitted: %d\n", somethingTransmitted); }
                            if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }

                            // check if reconfig needed
                            bool configChanged = false;
                            uint8_t nightTimeModeBeforeConfigChange = config.nightTimeMode;
                            if(commandByte == COMMAND_BYTE_CHANGE_CONFIG) {
                                // PATH TESTED
                                if(!checkIfReconfigNeeded(&device, &config, gatewayAroundConfig, commandByte, 0, &configChanged, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 182; errorCnt++; }
                            }

                            // evaluate commands
                            if(configChanged) {
                                // PATH TESTED
                                if(config.trackerMode == MODE_TESTRUN) { printf("DATATRANSM: CONFIGURATION CHANGED -> RESTART\n"); }
                                device.blinkTimes(12, B_BOTH);
                                resetRTCVariables();
                                if(nightTimeModeBeforeConfigChange == NIGHTTIME_MODE_ONLY_SLEEP) { // SHOULD NOT COME HERE, just in case
                                    if(!device.rtc.resetInterruptFlags()) { lastErrorId = 156; errorCnt++; } // IMPORTANT: does not reset by itself!
                                    if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 157; errorCnt++; } // IMPORTANT: otherwise daily interrupt triggers again in tracking and stays on (not resetting itself)
                                }
                                if(nightTimeModeBeforeConfigChange == NIGHTTIME_MODE_TRY_DATATRANS) {
                                    if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 155; errorCnt++; }
                                }
                                state = ST_FIRST_START;
                                device.enableInternalTimerInterruptInDeepSleep(1);
                            }
                            else if(commandByte == COMMAND_BYTE_FORCE_TRACKING) {
                                // PATH TESTED
                                if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: RECEIVED COMMAND TO FORCE TRACKING\n"); }
                                if(config.doTheBlink) { device.blinkTimes(5, B_RED); } // put BEFORE reactivation of fifo because at high sampling frequencies fifo might already be full after 5x blinky
                                disableNightTimeInterrupts();

                                if(!device.rtc.getTimestamp(&nightTimeOverrideForcedTimestampStart, NULL)) { lastErrorId = 186; errorCnt++; } // store current timestamp in nightTimeOverrideForcedTimestamp
                                startIMU(false);
                                state = ST_TRACK;
                                if(config.trackerMode == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                                if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP) { device.enableAccInterruptInDeepSleep(); }
                                else if(config.trackingDataMode == TRACKING_DATA_MODE_1HZ_GPS_AND_IMU) { device.enableInternalTimerInterruptInDeepSleep(1); }
                                else if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                                else if(config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING) { device.enableShortInternalTimerInterruptInDeepSleep(20); }
                            }
                            else if(commandByte == COMMAND_BYTE_DEACTIVATE) {
                                // PATH TESTED
                                if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: RECEIVED CMD TO DE-ACTIVATE\n"); }
                                disableNightTimeInterrupts();
                                resetActivation(); // NVS already initialized
                                device.enableInternalTimerInterruptInDeepSleep(1);
                            }
                            else if(commandByte == COMMAND_BYTE_MAG_CALIBRATION) {
                                // PATH TESTED
                                if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: RECEIVED CMD TO CALIB MAG\n"); }
                                disableNightTimeInterrupts();
                                state = ST_MAG_CALIBRATION;
                                device.enableInternalTimerInterruptInDeepSleep(1);
                            }
                            else if(commandByte == COMMAND_BYTE_TIME_RESYNC) {
                                // UNTESTED
                                if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: RECEIVED CMD TO RESYNC TIME\n"); }
                                disableNightTimeInterrupts();
                                state = ST_TIME_RESYNC;
                                device.enableInternalTimerInterruptInDeepSleep(1);
                            }
                            else { device.enableRTCInterruptInDeepSleep(); } // continue sleeping
                        }
                    }
                }
                else { // night time over ->  back to tracking
                    // PATH TESTED
                    if(config.doTheBlink) { device.blinkTimes(5, B_RED); } // put BEFORE reactivation of fifo because at high sampling frequencies fifo might already be full after 5x blinky
                    if(config.trackerMode == MODE_TESTRUN) { printf("Night trans: Night time over -> restart IMU\n"); }
                    disableNightTimeInterrupts();

                    startIMU(false);
                    state = ST_TRACK;
                    if(config.trackerMode == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                    if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP) { device.enableAccInterruptInDeepSleep(); }
                    else if(config.trackingDataMode == TRACKING_DATA_MODE_1HZ_GPS_AND_IMU) { device.enableInternalTimerInterruptInDeepSleep(1); }
                    else if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                    else if(config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING) { device.enableInternalTimerInterruptInDeepSleep(1); }
                }
            }
            /** ---------------- MEMORY FULL STATE (IMU OFF!) ---------------- */
            else if(state == ST_MEMFULL) {
                if(config.trackerMode == MODE_TESTRUN) { printf("State: (MEM_FULL) V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // voltage low, go to sleep (not power down, because IMU already stopped)
                    if(config.trackerMode == MODE_TESTRUN) { printf("State: (MEM_FULL) undervoltage during MEM_FULL state -> SLEEP for %ds\n", FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                }
                else { // voltage okay (again), then just try to get rid of the data
                    if((config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) || (config.transmissionMethod == TRANSMISSION_METHOD_WIFI)) {
                        // get rtc timestamp (1ms)
                        i2c.begin(I2C_FREQ_HZ_400KHZ);
                        uint32_t timestamp = 0;
                        if(!device.rtc.getTimestamp(&timestamp, NULL)) { lastErrorId = 9; errorCnt++; }
                        if(config.trackerMode == MODE_TESTRUN) { printf("State: TIMESTAMP: %d\n", timestamp); }
                        // get pointers from NVS (15ms)
                        if(!device.initDataNVS()) { lastErrorId = 11; errorCnt++; }
                        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                        if(config.trackerMode == MODE_TESTRUN) { printf("Flash: flashPointer: %d, flashOffsetPointer: %d\n", flashPointer, flashOffsetPointer); }
                        // check free space in memory
                        uint16_t flashBlockDeletedPointer = 0;
                        if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                            flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                        }
                        if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                            flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                        }
                        uint32_t freeSpaceBytesInFIFO = device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES);
                        uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockDeletedPointer, flashPointer);
                        if(config.trackerMode == MODE_TESTRUN) { printf("Flash: FIFO free space in memory: %d, blocksToTransmit: %d\n", freeSpaceBytesInFIFO, blocksToTransmit); }
                        if((freeSpaceBytesInFIFO >= ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES) // restart when this "watermark" is reached
                            || (blocksToTransmit == 0)) { // restart when memory is "totally empty" (except the one block where recording started)
                            if(config.trackerMode == MODE_TESTRUN) { printf("Flash: FIFO free space %d >= %d -> RESTART\n", freeSpaceBytesInFIFO, ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES); }
                            startIMU(false);
                            state = ST_TRACK;
                            if(config.trackerMode == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                            if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP) { device.enableAccInterruptInDeepSleep(); }
                            else if(config.trackingDataMode == TRACKING_DATA_MODE_1HZ_GPS_AND_IMU) { device.enableInternalTimerInterruptInDeepSleep(1); }
                            else if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                            else if(config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING) { device.enableShortInternalTimerInterruptInDeepSleep(20); }
                            else { device.enableInternalTimerInterruptInDeepSleep(1); }
                        }
                        else { // transmit more before exiting this mode!
                            // turn on flash power already (5ms)
                            if(!device.flashPowerOn(true)) { lastErrorId = 10; errorCnt++; }

                            bool somethingTransmitted = false;
                            uint32_t timeNow;

                            if(config.doTheBlink) { device.ledGreenOn(); }

                            uint8_t commandByte = 0; // unused here - NO COMMANDS ACCEPTED WHEN MEMORY IS FULL
                            if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                                // esp now data transmission (FORCE mode!)
                                timeNow = ((uint32_t) Timing::millis());
                                somethingTransmitted = dataTransmissionEspNow(true, 1, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte, true);
                                if(config.trackerMode == MODE_TESTRUN) { printf("ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            }
                            else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                                // wifi data transmission (FORCE mode!, minBlocksToTransmit = 1, otherwise function returns without trying to send data when config.wifiMinBlocksToTransmit is > 1)
                                timeNow = ((uint32_t) Timing::millis());
                                somethingTransmitted = dataTransmissionWifi(true, 1, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                                if(config.trackerMode == MODE_TESTRUN) { printf("WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            }

                            if(config.doTheBlink) { device.ledGreenOff(); }

                            // turn off flash
                            if(!device.flashPowerOff(true)) { lastErrorId = 12; errorCnt++; } 

                            // adapt sleep interval based on transmission success
                            if(somethingTransmitted) {
                                memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY;
                                if(config.trackerMode == MODE_TESTRUN) { printf("State: something transmitted -> wake up more frequently\n"); }
                            }
                            else { // could be wifi not found, but also connection issue or something else
                                if(memFullWakeUpOnMinute == ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY) { memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_LESS_FREQU; } // nothing transmitted now, but the session before -> try a again with small frequency one more time
                                else if(memFullWakeUpOnMinute == ST_MEMFULL_TRY_EVERY_FULL_MINUTE_LESS_FREQU) { memFullWakeUpOnMinute = config.memFullTryEveryFullMinSeldomly; } // again nothing transmitted -> more seldomly
                                else { memFullWakeUpOnMinute = config.memFullTryEveryFullMinSeldomly; }
                                if(config.trackerMode == MODE_TESTRUN) { printf("State: nothing transmitted this time -> wake up more seldomly\n"); }
                            }
                            device.rtc.getTimestamp(&timestamp, NULL);
                            uint32_t sleepyTime = calculateMemoryFullSleepTime(timestamp, memFullWakeUpOnMinute); 

                            device.enableInternalTimerInterruptInDeepSleep(sleepyTime); // sleep some time before trying again transmission
                            if(config.trackerMode == MODE_TESTRUN) { printf("State: next transmission in %ds, memFullWakeUpOnMinute %d\n", sleepyTime, memFullWakeUpOnMinute); }
                            if(config.trackerMode == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                        }
                    }
                    else {
                        if(config.doTheBlink) { device.blink(B_GREEN, B_GREEN); } // no data transmission, just blink from time to time
                        device.enableInternalTimerInterruptInDeepSleep(3600); // sleep some time before blinking again
                    }
                }
            }
            /** ---------------- FULL RF CALIB (ONLY CALLED BY DEEP SLEEP TRACKING MODE, IMU RUNNING) ---------------- */
            else if(state == ST_FULL_RF_CALIB) {
                // PATH TESTED
                handleCustomRFCalibration(true);
                if(config.trackerMode == MODE_TESTRUN) { printf("Full calib state: done\n"); }
                state = ST_TRACK; // back to track
                device.enableAccInterruptInDeepSleep(); // IMU still running, interrupt configured
            }
            /** ---------------- TIME RE-SYNC (IMU STOPPED) ---------------- */
            else if(state == ST_TIME_RESYNC) {
                // UNTESTED
                bool updatedTimestamp = false;
                if(config.trackerMode == MODE_TESTRUN) { printf("Time resync state: start\n"); }
                if(device.initWiFi()) {
                    uint8_t foundArrayId = 0;
                    uint8_t foundOnChannel = 0;
                    bool connectionTimeout = false;
                    uint32_t scanStartTime = ((uint32_t) Timing::millis());
                    i2c.begin(I2C_FREQ_HZ_400KHZ); // for setting RTC time
                    if(device.scanForWiFisOnAllChannels((config.trackerMode == MODE_TESTRUN), TIME_WIFI_SSIDS, TIME_WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, TIME_WIFI_OUTPUT_POWER, 120, 500)) { 
                        if(config.trackerMode == MODE_TESTRUN) { printf("Time: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500); }
                        if(foundOnChannel > 0) { // found wifi, try to connect
                            device.ledGreenOn();
                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: wifi found (chan %d, index %d), hasValidTimestamp: %d\n", foundOnChannel, foundArrayId, hasValidTimestamp); }
                            if(device.connectToWiFiAfterScan(TIME_WIFI_SSIDS[foundArrayId], TIME_WIFI_PASSWORDS[foundArrayId], foundOnChannel)) {
                                uint32_t connectStartedTime = ((uint32_t) Timing::millis());
                                while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                                    device.delay(20);
                                    if(((uint32_t) Timing::millis()) - connectStartedTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
                                        if(config.trackerMode == MODE_TESTRUN) { printf("Time: TIMEOUT CONNECT!\n"); }
                                        connectionTimeout = true;
                                        break;
                                    }
                                }
                                if(connectionTimeout || (device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                                    if(config.trackerMode == MODE_TESTRUN) { printf("Time: STRANGE could not connect, but wifi was seen (pw wrong, no internet): %d!\n", device.connectedToWiFi()); }
                                    lastErrorId = 38; errorCnt++;
                                }
                                else { // connected to wifi
                                    uint32_t timestampUTC = 0;
                                    uint16_t millisecondsUTC = 0;
                                    if(!device.getNTPTimestampUTC(true, timestampUTC, millisecondsUTC, 7000, "pool.ntp.org")) { // will block, will set RTC time
                                        if(config.trackerMode == MODE_TESTRUN) { printf("Time: UTC get time error!\n"); }
                                        lastErrorId = 39; errorCnt++;
                                    }
                                    else {
                                        if(config.trackerMode == MODE_TESTRUN) { printf("Time: set %d + 1, UTC milliseconds: %d -> waited for %dms\n", timestampUTC, millisecondsUTC, 1000 - millisecondsUTC); }
                                        hasValidTimestamp = true;
                                        updatedTimestamp = true;
                                        setNextDataTransmissionTimestamp(false, timestampUTC+1, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
                                    }
                                }
                            }
                            else { lastErrorId = 37; errorCnt++; }
                            device.ledGreenOff();
                        }
                        else {
                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: wifi NOT found\n"); }
                        }
                    }
                    else { lastErrorId = 32; errorCnt++; }
                    device.disconnectAndStopWiFi();
                }
                else { lastErrorId = 33; errorCnt++; }

                // check result of getting timestamp attempt
                if(isActivated) {
                    state = ST_START;
                    if(config.trackerMode == MODE_TESTRUN) { printf("Time: all done and already activated -> MOVE TO START in %ds!\n", 1); }
                }
                else {
                    state = ST_WAIT_FOR_ACTIVATION;
                    if(config.trackerMode == MODE_TESTRUN) { printf("Time: all done -> move to ACTIVATION in %ds!\n", 1); }
                }
                if(updatedTimestamp) { 
                    if(config.trackerMode == MODE_TESTRUN) { printf("Time resync state: success\n"); }
                    device.blinkTimes(6, B_GREEN);
                }
                else {
                    if(config.trackerMode == MODE_TESTRUN) { printf("Time resync state: failed\n"); }
                }
                device.enableInternalTimerInterruptInDeepSleep(60); // restart in x seconds and move to activation
            }
            /** ---------------- MAG CALIBRATION STATE ---------------- */
            else if(state == ST_MAG_CALIBRATION) {
                // PATH TESTED
                bool calibModeError = false;
                mag_calibration_t magCalibration = {};
                int16_t magHardIronOffsetXNew = 0;
                int16_t magHardIronOffsetYNew = 0;
                int16_t magHardIronOffsetZNew = 0;

                i2c.begin(I2C_FREQ_HZ_400KHZ);
                device.sensorPowerOn();
                device.shortLightSleep(120);
                if(config.trackerMode == MODE_TESTRUN) { printf("magCalibrationState: START!\n"); }
                if(!device.magnetometerCalibrationMode(MAG_CALIBRATION_MODE_DURATION_SECONDS, &magCalibration, BMX160_MAG_ODR_12_5HZ, BMX160_MAG_ACCURACY_REGULAR, true)) { calibModeError = true; }

                magHardIronOffsetXNew = (magCalibration.xMin + magCalibration.xMax) / 2;
                magHardIronOffsetYNew = (magCalibration.yMin + magCalibration.yMax) / 2;
                magHardIronOffsetZNew = (magCalibration.zMin + magCalibration.zMax) / 2;

                if(config.trackerMode == MODE_TESTRUN) { printf("magCalibrationState: (x16) xmin %d, xmax %d, ymin %d, ymax %d, zmin %d, zmax %d -> error %d\n", magCalibration.xMin, magCalibration.xMax, magCalibration.yMin, magCalibration.yMax, magCalibration.zMin, magCalibration.zMax, calibModeError); }
                
                stopIMU();
                device.sensorPowerOff();

                if(!device.initDataNVS()) { calibModeError = true; }
                if(!calibModeError) { // only write when no calib error
                    bool neverWritten = false;
                    int16_t magHardIronOffsetXOld = device.nvsReadINT16(NVS_MAG_CALIB_OFFSET_X, &neverWritten);
                    if(config.trackerMode == MODE_TESTRUN) { printf("magCalibrationState: OLD: x: %d (%duT, neverWritten: %d), ", magHardIronOffsetXOld, magHardIronOffsetXOld/16, neverWritten); }
                    int16_t magHardIronOffsetYOld = device.nvsReadINT16(NVS_MAG_CALIB_OFFSET_Y, &neverWritten);
                    if(config.trackerMode == MODE_TESTRUN) { printf("y: %d (%duT, neverWritten: %d), ", magHardIronOffsetYOld, magHardIronOffsetYOld/16, neverWritten); }
                    int16_t magHardIronOffsetZOld = device.nvsReadINT16(NVS_MAG_CALIB_OFFSET_Z, &neverWritten);
                    if(config.trackerMode == MODE_TESTRUN) { printf("z: %d (%duT, neverWritten: %d)\n", magHardIronOffsetZOld, magHardIronOffsetZOld/16, neverWritten); }
                    if(!device.nvsWriteINT16(NVS_MAG_CALIB_OFFSET_X, magHardIronOffsetXNew)) { calibModeError = true; }
                    if(!device.nvsWriteINT16(NVS_MAG_CALIB_OFFSET_Y, magHardIronOffsetYNew)) { calibModeError = true; }
                    if(!device.nvsWriteINT16(NVS_MAG_CALIB_OFFSET_Z, magHardIronOffsetZNew)) { calibModeError = true; }
                    if(config.trackerMode == MODE_TESTRUN) { printf("magCalibrationState: NEW: x: %d (%duT), y: %d (%duT), z: %d (%duT)\n", magHardIronOffsetXNew, magHardIronOffsetXNew/16, magHardIronOffsetYNew, magHardIronOffsetYNew/16, magHardIronOffsetZNew, magHardIronOffsetZNew/16); }
                    magHardIronOffsetX = magHardIronOffsetXNew;
                    magHardIronOffsetY = magHardIronOffsetYNew;
                    magHardIronOffsetZ = magHardIronOffsetZNew;
                }

                if(calibModeError) { lastErrorId = 183; errorCnt++; }
                resetActivation(); // going back into ST_WAIT_FOR_ACTIVATION!
                device.setCPUSpeed(ESP32_10MHZ);
                if(!calibModeError) { device.blinkTimes(10, B_RED); }
                device.enableInternalTimerInterruptInDeepSleep(15); // sleep for longer time
            }
            /** ---------------- WAIT FOR ACTIVITY STATE (IMU RUNNING IN SPECIAL MODE) ---------------- */
            else if(state == ST_WAIT_FOR_ACTIVITY) {
                i2c.begin(I2C_FREQ_HZ_400KHZ);
                if(device.getWakeUpReason() == BY_EXT0) { // wake up by RTC, regular interrupt from time to time
                    if(config.trackerMode == MODE_TESTRUN) { printf("Activity: woke up from regular interrupt\n"); }
                    if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // voltage low, go to power down
                        // PATH TESTED
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activity: undervoltage -> PWR DWN\n"); }
                        stopIMU();
                        if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 190; errorCnt++; } 
                        state = ST_PWRDWN;
                        device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                    }
                    else if(isTimeToGoToBed()) {
                        // PATH TESTED
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activity: NIGHT TIME\n"); }
                        stopIMU();
                        if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 190; errorCnt++; }
                        state = ST_NIGHT_TIME;
                        enableNightTimeInterrupts();
                    }
                    else {
                        // PATH TESTED
                        uint8_t commandByte = 0;
                        bool somethingTransmitted = false;
                        if(!device.initDataNVS()) { lastErrorId = 134; errorCnt++; }
                        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activity: flashPointer: %d, flashOffsetPointer: %d\n", flashPointer, flashOffsetPointer); }
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activity: transmit try even when nothing to transmit\n"); }
                        if(!device.flashPowerOn(true)) { lastErrorId = 135; errorCnt++; } // turn on flash power already (5ms)
                        uint32_t timeNow = ((uint32_t) Timing::millis());

                        if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                            // esp now data transmission (FORCE mode!), IMPORTANT: calling with minBytesToTransmit = 0 -> FORCING a scan to get commandByte even if no data to send
                            somethingTransmitted = dataTransmissionEspNow(true, 0, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte, true);
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activity: ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                        }
                        else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                            // wifi data transmission (FORCE mode!)
                            somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activity: WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                        }

                        if(!device.flashPowerOff(true)) { lastErrorId = 136; errorCnt++; } // turn off flash
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activity: something transmitted: %d\n", somethingTransmitted); }
                        if(config.trackerMode == MODE_TESTRUN) { printf("Activity: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }

                        // check if reconfig needed
                        bool configChanged = false;
                        if(commandByte == COMMAND_BYTE_CHANGE_CONFIG) {
                            // PATH TESTED
                            if(!checkIfReconfigNeeded(&device, &config, gatewayAroundConfig, commandByte, 0, &configChanged, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 182; errorCnt++; }
                        }

                        // evaluate commands
                        if(configChanged) {
                            // PATH TESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activity: CONFIGURATION CHANGED -> RESTART\n"); }
                            stopIMU();
                            if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 190; errorCnt++; }
                            device.blinkTimes(12, B_BOTH);
                            resetRTCVariables();
                            state = ST_FIRST_START;
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        else if(commandByte == COMMAND_BYTE_DEACTIVATE) {
                            // PATH TESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activity: RECEIVED CMD TO DE-ACTIVATE\n"); }
                            stopIMU();
                            if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 190; errorCnt++; }
                            resetActivation(); // NVS already initialized
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        else if(commandByte == COMMAND_BYTE_ACTIVATE_WHEN_NO_GW) {
                            // PATH UNTESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activity: RECEIVED CMD TO ACTIVATE ONLY WHEN NO GW AROUND\n"); }
                            stopIMU();
                            if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 190; errorCnt++; }
                            resetActivation(); // NVS already initialized
                            device.enableInternalTimerInterruptInDeepSleep(60); // sleep longer to give gateway time to store data
                        }
                        else if(commandByte == COMMAND_BYTE_FORCE_TRACKING) {
                            // PATH TESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activity: RECEIVED COMMAND TO FORCE TRACKING\n"); }
                            if(config.doTheBlink) { device.blinkTimes(5, B_RED); } // put BEFORE reactivation of fifo because at high sampling frequencies fifo might already be full after 5x blinky
                            if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 191; errorCnt++; } // IMPORTANT: disable regular interrupt
                            stopIMU(false);
                            device.shortLightSleep(1000);
                            if(!device.rtc.getTimestamp(&nightTimeOverrideForcedTimestampStart, NULL)) { lastErrorId = 192; errorCnt++; } // store current timestamp in nightTimeOverrideForcedTimestamp
                            startIMU(false);
                            state = ST_TRACK;
                            if(config.trackerMode == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                            if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP) { device.enableAccInterruptInDeepSleep(); }
                            else if(config.trackingDataMode == TRACKING_DATA_MODE_1HZ_GPS_AND_IMU) { device.enableInternalTimerInterruptInDeepSleep(1); }
                            else if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                            else if(config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING) { device.enableShortInternalTimerInterruptInDeepSleep(20); }
                            else { device.enableInternalTimerInterruptInDeepSleep(1); }
                        }
                        else if(commandByte == COMMAND_BYTE_MAG_CALIBRATION) {
                            // PATH TESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activity: RECEIVED CMD TO CALIB MAG\n"); }
                            stopIMU();
                            if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 190; errorCnt++; }
                            state = ST_MAG_CALIBRATION;
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        else if(commandByte == COMMAND_BYTE_TIME_RESYNC) {
                            // UNTESTED
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activity: RECEIVED CMD TO RESYNC TIME\n"); }
                            stopIMU();
                            if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 190; errorCnt++; }
                            state = ST_TIME_RESYNC;
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        else {
                            // PATH TESTED
                            device.enableAccInterruptInDeepSleep();
                            device.enableRTCInterruptInDeepSleep();
                        }
                    }
                }
                else { // wake up by ACC interrupt (SignificantMotion) -> back to tracking
                    // PATH TESTED
                    if(config.trackerMode == MODE_TESTRUN) { printf("Activity: woke up from activity -> back to tracking\n"); }
                    if(!device.rtc.disableRegularInterrupt()) { lastErrorId = 190; errorCnt++; } // IMPORTANT: disable regular interrupt
                    stopIMU(false);
                    device.shortLightSleep(1000);
                    if(config.doTheBlink) { device.ledGreenOn(); }
                    startIMU(false);
                    if(config.doTheBlink) { device.ledGreenOff(); }
                    state = ST_TRACK;
                    if(config.trackerMode == MODE_TESTRUN) { printf("Activity: LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                    if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP) { device.enableAccInterruptInDeepSleep(); }
                    else if(config.trackingDataMode == TRACKING_DATA_MODE_1HZ_GPS_AND_IMU) { device.enableInternalTimerInterruptInDeepSleep(1); }
                    else if(config.trackingDataMode == TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) { device.enableInternalTimerInterruptInDeepSleep(1); }
                    else if(config.trackingDataMode == TRACKING_DATA_MODE_DEAD_RECKONING) { device.enableShortInternalTimerInterruptInDeepSleep(20); }
                    else { device.enableInternalTimerInterruptInDeepSleep(1); }
                }
            }
            /** ---------------- FORCE DOWNLOAD STATE (IMU NEVER STARTED!) ---------------- */
            else if(state == ST_FORCE_DOWNLOAD) {
                if(config.trackerMode == MODE_TESTRUN) { printf("State: forcing download, V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // voltage low, go to sleep (not power down, because IMU already stopped)
                    if(config.trackerMode == MODE_TESTRUN) { printf("State: undervoltage during ST_FORCE_DOWNLOAD -> SLEEP for %ds\n", FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                }
                else { // voltage okay (again), then just try to get rid of the data
                    if((config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) || (config.transmissionMethod == TRANSMISSION_METHOD_WIFI)) {
                        // do not get rtc timestamp here -> after reset most propably = 0
                        // get pointers from NVS
                        if(!device.initDataNVS()) { lastErrorId = 115; errorCnt++; }
                        uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                        uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                        uint16_t flashBlockDeletedPointer = 0;
                        if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                            flashBlockDeletedPointer = device.nvsReadUINT32(NVS_FLASH_SEND_POINTER) / MT29_PAGES_PER_BLOCK;
                        }
                        else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                            flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be read again in data transmission, not so important, needed for fifo push
                        }
                        uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockDeletedPointer, flashPointer);
                        if(config.trackerMode == MODE_TESTRUN) { printf("Flash: flashPointer: %d, flashOffsetPointer: %d, flashBlockDeletedPointer: %d, blocksToTransmit: %d\n", flashPointer, flashOffsetPointer, flashBlockDeletedPointer, blocksToTransmit); }
                        if(blocksToTransmit == 0) { // ALL memory free now -> go back to time state (maybe some rest left)
                            if(config.trackerMode == MODE_TESTRUN) {
                                printf("Flash: blocks to transmit ZERO\n");
                                uint32_t freeSpaceBytesInFIFO = device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES);
                                uint32_t remainingBytesInMemory = MT29_NUMBER_BYTES - freeSpaceBytesInFIFO;
                                printf("Flash: but still %d bytes in flash memory (no full block, so can't be transmitted -> next time)\n", remainingBytesInMemory);
                            }
                            if(config.doTheBlink) { device.blinkTimes(10, B_BOTH); } // blink 10 times before going to sleep FOREVER, no wake up interrupt!!!
                            // SLEEP FOREVER
                        }
                        else { // transmit more before exiting this mode!
                            // turn on flash power already (5ms)
                            if(!device.flashPowerOn(true)) { lastErrorId = 131; errorCnt++; }

                            bool somethingTransmitted = false;
                            uint32_t timeNow;

                            if(config.doTheBlink) { device.ledRedOn(); } // turn on led during transmission

                            uint8_t commandByte = 0; // unused here - NO COMMANDS ACCEPTED IN FORCE MODE
                            if(config.transmissionMethod == TRANSMISSION_METHOD_ESP_NOW) {
                                // esp now data transmission (FORCE mode!)
                                timeNow = ((uint32_t) Timing::millis());
                                somethingTransmitted = dataTransmissionEspNow(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, flashOffsetPointer, &commandByte, true);
                                if(config.trackerMode == MODE_TESTRUN) { printf("ESP NOW needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            }
                            else if(config.transmissionMethod == TRANSMISSION_METHOD_WIFI) {
                                // wifi data transmission (FORCE mode!, IMPORTANT: only use FORCE_DOWNLOAD_WIFI_SSID, minBlocksToTransmit = 1, otherwise function returns without trying to send data when config.wifiMinBlocksToTransmit is > 1)
                                timeNow = ((uint32_t) Timing::millis());
                                somethingTransmitted = dataTransmissionWifi(true, 1, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, TIME_WIFI_SSIDS, TIME_WIFI_PASSWORDS, 1); // IMPORTANT: 1 = only use FORCE_DOWNLOAD_WIFI_SSID in TIME_WIFI_SSIDS list
                                if(config.trackerMode == MODE_TESTRUN) { printf("WIFI needed %dms\n", ((uint32_t) Timing::millis()) - timeNow); }
                            }

                            if(config.doTheBlink) { device.ledRedOff(); } // turn off led during transmission

                            // turn off flash
                            if(!device.flashPowerOff(true)) { lastErrorId = 100; errorCnt++; }

                            device.enableInternalTimerInterruptInDeepSleep(5); // sleep some time before trying again transmission
                            if(config.trackerMode == MODE_TESTRUN) { printf("State: something transmitted: %d -> anyhow restart in 5s\n", somethingTransmitted); }
                            if(config.trackerMode == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                        }
                    }
                    else {
                        if(config.doTheBlink) { device.blink(B_RED, B_RED); } // no data transmission, just blink from time to time
                        device.enableInternalTimerInterruptInDeepSleep(30); // sleep some time before blinking again
                    }                   
                }
            }
        }
        else { // not in productive or test run
            /*if(config.trackerMode == MODE_SELFTEST) {
                device.serialMenue(true, NULL, NVS_FLASH_TAG_ACTIVATED_BY_WIFI, gpsCheckAliveAndMaybeChangeBaudrateWrapper); // WARNING: CPU clocked to 10 MHz
                esp_sleep_enable_timer_wakeup(1000000ULL); // nothing pressed -> sleep for 1 second
            }
            else if(config.trackerMode == MODE_READFLASH) { printf("Mode: READ FLASH\n"); readFullFlash(); }
            else if(config.trackerMode == MODE_MOCK_FLASH_STATE) { printf("Mode: MOCK FLASH STATE\n"); mockFlashState(); }*/
        }
        startCnt++;
        device.deepSleep();
    }
}