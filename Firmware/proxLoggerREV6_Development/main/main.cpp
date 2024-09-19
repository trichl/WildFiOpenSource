#include "PlatformWildFiTagREV6.h"
#include "passwords.h"
#include "ModuleGPS_L70_REV6.h"
#include "config.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();
GPS_L70_REV6 gps = GPS_L70_REV6();

// next free error id = 193

/* TESTING */
// TEST MORE: hard and soft iron offset calculation mode as gateway command
// TEST MORE: blob sending function with memory wrap around
// TEST MORE: PROXIMITY_COMMAND_FULL_RESET
// TEST: ST_WIFI_SYNC

/* HIGHER PRIORITY */

/* CHERRY ON THE CAKE */
// [ TODO: ESP_NOW_FLASH_STREAM_NO_LIMIT (4 byte) as configuration parameter? ]
// [ TODO: minimum voltage for starting GPS? maybe not ]
// [ TODO: option to NOT do proximity detection (only imu for example or gps) ]
// [ TODO: option for periodic wifi resync ]
// [ TODO: initDataNVS maybe in sensorInitTask? would save 4ms! ]
// [ TODO: special get time mode without GPS: 900ms deep sleep -> listen for 110ms -> repeat -> see timeSyncListening, needs 13.3mA, see test "timeSyncListening" ]
// [ TODO: try out WIFI_PHY_RATE_11M_L all the way for proximity detection? ]

// LIMITATION: max 150 results in proximity detection (more = more space in RAM, also caters for limit of NVS buffer -> FIXED, no more NVS buffer limit)
// LIMITATION: when IMU_GET_FULL_BURSTS > 0 and gateway around -> only get short burst (because long burst uses light sleep but RF still on to transmit data afterwards)
// WARNING: GPS fix after getting .000 millisecond messages went to LAT/LNG = 0 once (while seeing 7 satellites) -> intensively tested, never happened again
// WARNING: GPS time accuracy can be horrible for first fixes -> wait at least some seconds to get a good accuracy for time -> solved by waiting on .000 millisecond messages
// WARNING: measured antenna degredation (or temperature effect?) after some runtime, hence re-doing phyinit from time to time
// WARNING: PROBLEM SEEN ONCE: proximity scan = 0 tags, but GW on, other tags were seen correctly, sometimes GW seen for first time, other tags (older SW) saw GW correctly -> had similiar issue afterwards, seems to be GW
// WARNING: using command from gateway with smallest ever received RSSI -> but data is being sent to gateway where MAX RSSI of all messages is lowest, so maybe command is taken from different gateway than data is being sent to

/** Normal Variables */
proximity_entry_t scanResults[PROXIMITY_MAX_TAGS_IN_PARALLEL];
uint16_t scanResultsPointer = 0;
bool scanStop = false;
bool timerFinished = false;
extern uint32_t batteryVoltageWakeStub;
int64_t timeStartProximity = 0;

/** RTC Variables */
RTC_DATA_ATTR uint16_t ownTagId = 0; // id of the tag, used for proximity detection
RTC_DATA_ATTR tracker_state_t state = ST_BOOT; // current system state
RTC_DATA_ATTR uint32_t startCnt = 0; // start counter
RTC_DATA_ATTR uint8_t lastErrorId = 0; // last error id
RTC_DATA_ATTR uint16_t errorCnt = 0; // error counter
RTC_DATA_ATTR uint8_t currentMinute = 0, currentHour = 0; // current time for wake stub
RTC_DATA_ATTR uint16_t currentDayCounter = 0; // counting runtime in days
RTC_DATA_ATTR bool currentDayCounterChanged = true; // for re-calculating randomized GPS
RTC_DATA_ATTR uint32_t timestampLastSync = 0; // last time tag was re-synchronized
RTC_DATA_ATTR last_sync_type_t lastSyncType = SYNC_TYPE_NONE; // last type of re-synchronization
RTC_DATA_ATTR uint32_t syncCounter = 0; // number of re-synchronizations (via GPS, WiFi or between tags)
RTC_DATA_ATTR uint32_t gpsCounterSinceLastRTCSync = 0; // number of GPS fix attempts since last RTC sync
RTC_DATA_ATTR uint32_t timestampLastSeenSomeone = 0; // last time seen someone
RTC_DATA_ATTR uint16_t proxDetectionsSinceLastFullRFCalib = 0; // counter to do full RF calibration from time to time
RTC_DATA_ATTR bmm150_trim_registers trimData = { }; // magnetometer read registers
RTC_DATA_ATTR tag_config_t config = { }; // tag configuration (read from NVS)
RTC_DATA_ATTR uint8_t proximityFrequency = 1;
RTC_DATA_ATTR uint32_t freeMemory = 0; // free memory bytes for proximity message
RTC_DATA_ATTR uint32_t getTimeCounter = 0;
RTC_DATA_ATTR uint32_t gpsFixHours = 0;
RTC_DATA_ATTR bool environmentSensorConnected = false;
RTC_DATA_ATTR int16_t performCorrectionOnNextWakeStubMs = 0; // signed! correction based on received messages
RTC_DATA_ATTR int16_t magHardIronOffsetX = 0;
RTC_DATA_ATTR int16_t magHardIronOffsetY = 0;
RTC_DATA_ATTR int16_t magHardIronOffsetZ = 0;
RTC_DATA_ATTR uint32_t proximityDetectionCnt = 0;

/** System start */
RTC_DATA_ATTR bool isActivated = false;
RTC_DATA_ATTR bool hasValidTimestamp = false;
RTC_DATA_ATTR bool bootRefused = false;
RTC_DATA_ATTR uint8_t delayAfterActivationAndGotTime = 0;

/** Configuration */
const uint8_t TIME_WIFI_LIST_SIZE =                     3;                                              // should include time wifi
const char* TIME_WIFI_SSIDS[TIME_WIFI_LIST_SIZE] =      { WIFI_SSID1, WIFI_SSID2, WIFI_SSID3 };      // wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* TIME_WIFI_PASSWORDS[TIME_WIFI_LIST_SIZE] =  { WIFI_PASSWORD1, WIFI_PASSWORD2, WIFI_PASSWORD3 };        // wifi password for activation/timestamp  

/** Reconfiguration and gateway commands */
uint8_t nearestGatewayCommand = PROXIMITY_COMMAND_NOTHING;
uint8_t nearestGatewayRssi = 0xFF;
uint8_t nearestGatewayConfiguration[GATEWAY_AROUND_LEN] = { 0 };

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

RTC_DATA_ATTR bool isTimeToGoToBedWakeStub() {
    uint16_t minutesOfDay = 0;
    const uint16_t OFF_MINUTES_OF_DAY = (config.nightTimeTurnOffHour * 60) + config.nightTimeTurnOffMinute; // 0 ........ 1439
    const uint16_t ON_MINUTES_OF_DAY = (config.nightTimeTurnOnHour * 60) + config.nightTimeTurnOnMinute; // 0 ........ 1439
    if(!hasValidTimestamp) { return false; } // never time to go to bed (when GET TIME state is skipped by configuration)
    if(config.nightTimeEnter == NIGHTTIME_ALWAYS_NIGHT) { return true; } // always time to go to bed
    else if(config.nightTimeEnter == NIGHTTIME_DISABLED) { return false; } // never time to go to bed
    minutesOfDay = (currentHour * 60) + currentMinute; // calculate minutes passed that day, 0 ........ 1439
    if(OFF_MINUTES_OF_DAY < ON_MINUTES_OF_DAY) { // ----OFF___ON--------
        if((minutesOfDay > OFF_MINUTES_OF_DAY) && (minutesOfDay < ON_MINUTES_OF_DAY)) {
            return true; // bed time
        }
    }
    else { // ___ON----------OFF__
        if((minutesOfDay > OFF_MINUTES_OF_DAY) || (minutesOfDay < ON_MINUTES_OF_DAY)) {
            return true; // bed time
        }
    }
    return false; // wake up
}

RTC_DATA_ATTR bool wakeStub() { // called multiple times if RTC interrupt not being reset
    if(state == ST_TRACK) {
        if(bootRefused) { bootRefused = false; return false; } // INT pin still LOW but boot was refused and rtcWaitUntilINTPinLow called (restarting wakeStub) -> return false
        currentMinute++;
        if(currentMinute >= 60) { currentMinute = 0; currentHour++; } // 12:59 -> 13:00 (next full hour)
        if(currentHour >= 24) { currentHour = 0; currentDayCounter++; currentDayCounterChanged = true; } // 23:59 -> 00:00 (next day)

        /** HIGHEST PRIORITY: IF TIME TO SLEEP, THEN DO NOT WAKE UP, but stay in that state */
        if(isTimeToGoToBedWakeStub()) {
			if(NIGHTTIME_HOURLY_DATA_TRANSMISSION && (currentMinute == 0)) { // NEW: if hourly data transmission at night are activated, then wake up when minute == 0
				state = ST_NIGHTTIME_DATATRANS; // will transit back to ST_TRACK afterwards
				return true;
			}
			else {
				bootRefused = true;
				rtcWaitUntilINTPinLow(); // clear RTC interrupt (otherwise wakeStub called again if returning false because interrupt pin is low for ~8ms)
				return false;
			}
        }
        /** ON MINUTE 0 OF MODULO - PROXIMITY DETECTION AND MAYBE GPS */
        else if((currentMinute % proximityFrequency) == 0) {
            return true; // time to do proximity detection -> ALWAYS TRUE IF PROXIMITY DETECTION EVERY 1 MINUTE
        }
        /** ON MINUTE 1 - X - OTHER STUFF */
        else if(performCorrectionOnNextWakeStubMs != 0) { // not waking up, but a correction request is there -> good time to correct and then sleep afterwards again
            timeCorrectionShiftBetweenTags(performCorrectionOnNextWakeStubMs);
            performCorrectionOnNextWakeStubMs = 0; // correction done, don't redo it after boot-up
            bootRefused = true;
            rtcWaitUntilINTPinLow(); // I2C already started, clear RTC interrupt (otherwise wakeStub called again if returning false because interrupt pin is low for ~8ms)
            return false;
        }
        else { // not waking up and no time correction needed -> reset interrupt pin and sleep
            bootRefused = true;
            rtcWaitUntilINTPinLow(); // clear RTC interrupt (otherwise wakeStub called again if returning false because interrupt pin is low for ~8ms)
            return false;
        }
    }
    else {
        return true; // NOT doing the stuff when in power down state or other state, just booting up 
    }
}

void resetRTCVariables() {
    // NOT RESETTING: all RTC variables in PlatformWildFiTagREV6.cpp
    ownTagId = 0; // id of the tag, used for proximity detection
    startCnt = 0; // start counter
    lastErrorId = 0; // last error id
    errorCnt = 0; // error counter
    currentMinute = 0, currentHour = 0; // current time for wake stub
    currentDayCounter = 0; // counting runtime in days
    currentDayCounterChanged = true; // for re-calculating randomized GPS
    timestampLastSync = 0; // last time tag was re-synchronized
    lastSyncType = SYNC_TYPE_NONE; // last type of re-synchronization
    syncCounter = 0; // number of re-synchronizations
    gpsCounterSinceLastRTCSync = 0;
    timestampLastSeenSomeone = 0; // last time seen someone
    proxDetectionsSinceLastFullRFCalib = 0; // counter to do full RF calibration from time to time
    //trimData = { }; // magnetometer read registers -> NO NEED TO RESET
    //config = { }; // tag configuration (read from NVS) -> NO NEED TO RESET
    proximityFrequency = 1;
    freeMemory = 0; // free memory bytes for proximity message
    getTimeCounter = 0;
    gpsFixHours = 0;
    environmentSensorConnected = false;
    performCorrectionOnNextWakeStubMs = 0; // signed! correction based on received messages
    isActivated = false;
    hasValidTimestamp = false;
    bootRefused = false;
    proximityDetectionCnt = 0;
    delayAfterActivationAndGotTime = 0;
}

void startIMU() {
    acc_config_t accConfig = { config.accFrequency, config.accAvg, config.accRange };
    mag_config_t magConfig = { config.magFrequency, config.magAccuracy };
    gyro_config_t gyroConfig = { config.gyroFrequency, config.gyroRange, config.gyroMode };

    mag_config_t *magConfigPointer = NULL;
    gyro_config_t *gyroConfigPointer = NULL;
    if(config.imuMode == IMU_ACC_MAG_GYRO) {
        magConfigPointer = &magConfig;
        gyroConfigPointer = &gyroConfig;
    }
    if(!device.imu.start(&accConfig, magConfigPointer, gyroConfigPointer, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 31; errorCnt++; }
    uint8_t fifoForWhat = BMX160_INIT_FIFO_FOR_ACC;
    if(config.imuMode == IMU_ACC_MAG_GYRO) {
        fifoForWhat |= BMX160_INIT_FIFO_FOR_MAG;
        fifoForWhat |= BMX160_INIT_FIFO_FOR_GYRO;
    }
    if(!device.imu.initFIFO(fifoForWhat)) { lastErrorId = 32; errorCnt++; }
    if(!device.imu.resetFIFO()) { lastErrorId = 33; errorCnt++; }
}

void stopIMU() {
    if(!device.imu.stop()) { lastErrorId = 37; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
}

void readFifoIMU(uint8_t *data, uint16_t len) {
    if(!device.imu.readGeneralFIFOInOneGoFast(true, (config.imuMode == IMU_ACC_MAG_GYRO), (config.imuMode == IMU_ACC_MAG_GYRO), data, len, false)) { lastErrorId = 42; errorCnt++; }
    else {
        if(config.imuMode == IMU_ACC_MAG_GYRO) {
            bmx160_fifo_dataset_len_t datasetStructure = BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO;
            if(!device.imu.magCompensateFifoData(data, len, datasetStructure, &trimData, magHardIronOffsetX, magHardIronOffsetY, magHardIronOffsetZ)) { lastErrorId = 139; errorCnt++;  }
        }
    }
}

void fillIMUTrimData() {
    device.sensorPowerOn();
    device.shortLightSleep(120); // wait until bmx is booted
    if(config.trackerMode == MODE_TESTRUN) { printf("fillIMUTrimData: reading trim data\n"); }
    if(!device.imu.magCompensateReadTrimData(&trimData)) { lastErrorId = 137; errorCnt++; }
    if(config.trackerMode == MODE_TESTRUN) { device.imu.magCompensatePrintTrimData(&trimData); }
    if(!device.imu.stop()) { lastErrorId = 138; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
    device.sensorPowerOff(); // turn off IMU (and environment sensor) completely
    device.shortLightSleep(100); // wait because otherwise interrupt pin might still be valid
}

uint16_t getApproximatedAirTimeMs() {
    if(config.proximityAirTimeUs % 1000 < 500) { return (config.proximityAirTimeUs / 1000); } // floor
    else { return (config.proximityAirTimeUs / 1000) + 1; } // ceil
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
	// find all tags that have the highest latest sync value
	// among these tags (including own id), find the lowest id
	// if own id is in pool of highest latest sync values and is lowest id: do not re-sync, otherwise check for timediff
	// when othersBestLastSync == timestampLastSync (same time re-synced: check for time difference if id < ownTagId
	// for gateways timestampLastSync is 0 and timeDifferenceSumMs is 0: will never be used as time reference

    if(scanResultsPointer == 0) { return; } // no time correction when nobody around
	
	// first, find the highest timestampLastSync value in the group (excluded own timestampLastSync)
	uint32_t othersBestLastSync = scanResults[0].timestampLastSync;
    for(uint16_t i=1; i<scanResultsPointer; i++) { // find the tags with best time = most recently synchronized
        if(scanResults[i].timestampLastSync > othersBestLastSync) { // better
            othersBestLastSync = scanResults[i].timestampLastSync;
        } 
    }
	
	// among all tags with othersBestLastSync: find the one with lowest id and use the time of that tag as reference time
	uint16_t othersLowestId = 0;
	bool othersLowestIdSet = false;
	int16_t othersBestLastSyncTimeDiff = 0;
	for(uint16_t i=0; i<scanResultsPointer; i++) {
        if(scanResults[i].timestampLastSync == othersBestLastSync) { // tag is within group of highestLatestSync
			if(!othersLowestIdSet) { // first tag found with best last sync value
				othersLowestId = scanResults[i].id;
				othersBestLastSyncTimeDiff = (scanResults[i].timeDifferenceSumMs / scanResults[i].receivedMessages);
				othersLowestIdSet = true;
			}
			else { // there are more tags with best last sync value
				if(scanResults[i].id < othersLowestId) {
					othersLowestId = scanResults[i].id;
					othersBestLastSyncTimeDiff = (scanResults[i].timeDifferenceSumMs / scanResults[i].receivedMessages);
				}
			}
        } 
    }

	// decision tree: check if resync should be done
	// only if timestampLastSync is some time ago (get timestamp)? -> no, because might be horribly wrong
    // LIMITATION: if timestamps are different and difference > 1 s and normaly proximity interval <= 1 s then no re-sync possible
	bool doResync = false;
	int16_t minDifference = config.timeCorrectionDiffMs; // casting to signed integer
	if(othersBestLastSync == timestampLastSync) { // there are one or more tags with the same othersBestLastSync, including myself
		if(othersLowestId < ownTagId) { // other tags have a smaller id
			if((othersBestLastSyncTimeDiff > minDifference) || (othersBestLastSyncTimeDiff < -minDifference)) { // and time difference is higher than threshold
				doResync = true;
			}
		}
	}
	else if(othersBestLastSync > timestampLastSync) { // other tags were synchronized more recently (no need to check ids)
		if((othersBestLastSyncTimeDiff > minDifference) || (othersBestLastSyncTimeDiff < -minDifference)) { // and time difference is higher than threshold
			doResync = true;
		}
	}
	else { } // I got most recently re-synced -> do not to anything
	
	// debug output
	if(config.trackerMode == MODE_TESTRUN) { printf("TIMECORR: othersBestLastSync %d (me: %d), othersBestLastSyncTimeDiff %d, othersLowestId %d (me: %d) -> doResync: %d\n", othersBestLastSync, timestampLastSync, othersBestLastSyncTimeDiff, othersLowestId, ownTagId, doResync); }
	
	// perform resynchronization
    if(doResync) { 
		if(config.trackerMode == MODE_TESTRUN) { printf("TIMECORR: START, |%d| > %d!\n", othersBestLastSyncTimeDiff, minDifference); }
		//RTC_performCorrectionOnNextWakeStubMs = timeDifferenceOfMinLastSync; // next wake stub: perform the sync -> NO, do it now immediately
		int16_t milliseconds = othersBestLastSyncTimeDiff;
		int16_t fullSecondCorrection = milliseconds / 1000; // can be positive or negative!!, e.g. -3200 ms -> -3
		bool didCorrectTime = false;
		if(milliseconds >= 0) { fullSecondCorrection = milliseconds / 1000; }
		else { fullSecondCorrection = -((-milliseconds) / 1000); }
		milliseconds = milliseconds - (fullSecondCorrection * 1000); // busy waiting only for sub-second stuff, e.g. -3200 - (-3 * 1000) = -200

		// do sub-second correction
		if(milliseconds != 0) {
			device.setCPUSpeed(ESP32_10MHZ); // reduce CPU speed

			// busy wait until next full second (1 - 1000ms)
			bool error = false;
			uint8_t oldSecond = device.rtc.getSeconds(error);
			uint8_t currentSecond = 0;
			if(error) { return; }
			while(true) {
				currentSecond = device.rtc.getSeconds(error);
				if(error) { return; }
				if(currentSecond != oldSecond) { // second wrapped around
					if((currentSecond == 59) && (milliseconds >= 0)) { // time needs to be turned ahead -> 59 + 1 (see below) = 60 = will lead to error if only setting seconds
						oldSecond = currentSecond; // restart busy waiting
						ets_delay_us(3000);
					}
					else { break; } // stop busy waiting
				}
				else { ets_delay_us(3000); } // wait 3ms before polling again
			}

			// busy wait for time reset (at this exact point it is 10:58:00 in my time (seconds = 0))
			if(milliseconds < 0) { // I AM TOO FAST! TURN ME BACK! my time: 10:58:00_000, real time 10:57:59_550 -> wait until real time should have minute change, then reset seconds to 0
				milliseconds = -milliseconds; // positive value
				ets_delay_us(((uint16_t) (milliseconds)) * 1000);
				device.rtc.setSeconds(currentSecond); // reset seconds -> turning clock BACK
			}
			else { // I AM TOO SLOW! TURN ME AHEAD! my time: 10:58:00_000, real time 10:58:00_300 -> wait until real time should reach next second, then reset seconds to 1
				milliseconds = 1000 - milliseconds;
				for(uint8_t i=0; i<(milliseconds / TIME_CORRECTION_WAKE_STUB_FEED_INTERVAL); i++) { // if 512 to wait -> 512 / 300 = 1 -> run one time
					ets_delay_us(TIME_CORRECTION_WAKE_STUB_FEED_INTERVAL * 1000);
					REG_WRITE(TIMG_WDTFEED_REG(0), 1);
				}
				ets_delay_us((milliseconds % TIME_CORRECTION_WAKE_STUB_FEED_INTERVAL) * 1000); // wait rest of the time -> 512 % 300 = 212
				device.rtc.setSeconds(currentSecond + 1); // set to next second -> turning clock AHEAD
			}
			
			device.setCPUSpeed(ESP32_80MHZ); // turn the speed up again
			didCorrectTime = true;
		}

		// do full second correction
		if(fullSecondCorrection != 0) {
			// TESTED with +2 and -2 s difference
			uint32_t timestampToCorrect = 0;
			tmElements_t timeStruct;
			if(!device.rtc.getTimestamp(&timestampToCorrect, NULL)) { lastErrorId = 131; errorCnt++; } // get time here because timer just started (shouldn't delay stuff)
			else {
				uint32_t secondsCorrectionUnsigned = 0;
				if(fullSecondCorrection >= 0) {
					secondsCorrectionUnsigned = (uint32_t) fullSecondCorrection;
					timestampToCorrect = timestampToCorrect + secondsCorrectionUnsigned;
				}
				else {
					fullSecondCorrection = -fullSecondCorrection;
					secondsCorrectionUnsigned = (uint32_t) fullSecondCorrection;
					if(timestampToCorrect > secondsCorrectionUnsigned) {
						timestampToCorrect = timestampToCorrect - secondsCorrectionUnsigned;
					}
				}
				breakTime(timestampToCorrect, timeStruct);
				device.rtc.set(timeStruct.Hour, timeStruct.Minute, timeStruct.Second, timeStruct.Wday, timeStruct.Day, timeStruct.Month, timeStruct.Year);
				didCorrectTime = true;
			}
		}

		// set all the last sync settings
		if(didCorrectTime) {
			if(config.trackerMode == MODE_TESTRUN) { printf("TIMECORR: done! Full secs: %d\n", fullSecondCorrection); }
			timestampLastSync = othersBestLastSync; // copy last sync from other tag
			lastSyncType = SYNC_TYPE_NEIGHBOR;
			syncCounter++;
		}
    }
}

void checkForHugeTimeDifferences() {
    if(scanResultsPointer == 0) { return; } // nobody around
    int16_t timeDifferenceAvg = 0;
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        timeDifferenceAvg = (scanResults[i].timeDifferenceSumMs / scanResults[i].receivedMessages);
        if(timeDifferenceAvg > 600) {
            if(config.trackerMode == MODE_TESTRUN) { printf("WARNING: huge timediff %dms (no %d) detected!\n", timeDifferenceAvg, i); }
            lastErrorId = 41;
            errorCnt++;
        }
    }   
}

void printScanResult() {
    printf("Scan: %d tag(s)\n", scanResultsPointer);
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        printf(" %d: id: %04X, gw: %d, mac: %02X %02X %02X %02X %02X %02X, recMsgs: %d, rssiMin %d, rssiMax %d, rssiSum %d, rssiAvg %d, timeDiffSum %d, timeDiffAvg %d, lastSync %d, approxDist %.3f - %.3f - %.3fm\n", i, scanResults[i].id, scanResults[i].tagType, scanResults[i].mac[0], scanResults[i].mac[1], scanResults[i].mac[2], scanResults[i].mac[3], scanResults[i].mac[4], scanResults[i].mac[5],
            scanResults[i].receivedMessages, scanResults[i].rssiMin, scanResults[i].rssiMax, scanResults[i].rssiSum, (scanResults[i].rssiSum / scanResults[i].receivedMessages), scanResults[i].timeDifferenceSumMs, (scanResults[i].timeDifferenceSumMs / scanResults[i].receivedMessages), scanResults[i].timestampLastSync,
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
        scanResults[scanResultsPointer].tagType = TAG_TYPE_TAG;

        uint32_t timestampLastSyncOther = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_0] << 24)
            | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_1] << 16)
            | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_2] << 8)
            | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_3];
        scanResults[scanResultsPointer].timestampLastSync = timestampLastSyncOther; // only add lastSync from FIRST message, assuming that this will not change -> also faster

        if(p->rx_ctrl.rssi < 0) { rssiTemp = -(p->rx_ctrl.rssi); } // calculate absolute rssi value
        else { rssiTemp = p->rx_ctrl.rssi; }
        rssiAbs = rssiTemp;

        scanResults[scanResultsPointer].rssiMax = rssiAbs;
        scanResults[scanResultsPointer].rssiMin = rssiAbs;
        scanResults[scanResultsPointer].rssiSum = rssiAbs;
        
        scanResultsPointer++;
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
        scanResults[scanResultsPointer].timestampLastSync = 0; // do not use last sync for gateways
        if(p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_COMMAND] == PROXIMITY_COMMAND_DO_NOT_SEND) { scanResults[scanResultsPointer].tagType = TAG_TYPE_GATEWAY_NO_DATA; }
        else { scanResults[scanResultsPointer].tagType = TAG_TYPE_GATEWAY; }

        if(p->rx_ctrl.rssi < 0) { rssiTemp = -(p->rx_ctrl.rssi); } // calculate absolute rssi value
        else { rssiTemp = p->rx_ctrl.rssi; }
        rssiAbs = rssiTemp;

        scanResults[scanResultsPointer].rssiMax = rssiAbs;
        scanResults[scanResultsPointer].rssiMin = rssiAbs;
        scanResults[scanResultsPointer].rssiSum = rssiAbs;
        
        scanResultsPointer++;

        // configuration command
        // IMPORTANT: commands only taken from gateways that do NOT send PROXIMITY_COMMAND_DO_NOT_SEND (type TAG_TYPE_GATEWAY only)
        // so that data flows to the same gateway where command is coming from
        if(p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_COMMAND] != PROXIMITY_COMMAND_DO_NOT_SEND) {
            if(rssiAbs < nearestGatewayRssi) {
                nearestGatewayRssi = rssiAbs;
                nearestGatewayCommand = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_COMMAND];
                for(uint8_t i=0; i<(GATEWAY_AROUND_LEN - GATEWAY_AROUND_OFFSET_START_CONFIG); i++) { // store complete configuration message
                    nearestGatewayConfiguration[i] = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_START_CONFIG + i];
                }
            }
        }
    }
}

void wifiPromiscuous(void* buffer, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
    if(!scanStop) {
        if(type == WIFI_PKT_MGMT) { // all esp now messages are MGMT frames
            if(p->rx_ctrl.rate == config.proximityDatarate) { // using the correct proximity data rate
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
                    else if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + GATEWAY_AROUND_LEN) { // normally 43 bytes additionally + 8 bytes payload
                        // same group id
                        if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_0] == PROXIMITY_OWN_GROUP_0) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_1] == PROXIMITY_OWN_GROUP_1) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_2] == PROXIMITY_OWN_GROUP_2) && (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + GATEWAY_AROUND_OFFSET_OWN_GROUP_3] == PROXIMITY_OWN_GROUP_3)) {
                            if(p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND) { // gateway around message
                                addGatewayAroundMessageToScanResult(p);
                            }
                        }
                    }
                }
            }
            // FOR DEBUGGING: sniffs management packets
            //printf("- %d: %02X%02X%02X%02X%02X\n", p->rx_ctrl.sig_len, p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 0], p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 1], p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 2], p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 3], p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 4]);
        }
    }
}

static void proxReceiveCallback(const uint8_t *mac_addr, const uint8_t *data, int data_len) { } // do not do anything here, handled by promiscous sniffer
static void timerCallback(void* arg) { timerFinished = true; }

bool getTimeOverWiFiNew() {
    bool gotTimeOverWifi = false;
    if(device.initWiFi()) {
        uint8_t foundArrayId = 0;
        uint8_t foundOnChannel = 0;
        bool connectionTimeout = false;
        uint32_t scanStartTime = ((uint32_t) Timing::millis());
        if(device.scanForWiFisOnAllChannels((config.trackerMode == MODE_TESTRUN), TIME_WIFI_SSIDS, TIME_WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, config.timeWifiOutputPower, 120, 500)) { 
            if(config.trackerMode == MODE_TESTRUN) { printf("Time: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Timing::millis()) - scanStartTime, 500); }
            if(foundOnChannel > 0) { // found wifi, try to connect
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
                    }
                    else { // connected to wifi
                        uint32_t timestampUTC = 0;
                        uint16_t millisecondsUTC = 0;
                        if(!device.getNTPTimestampUTC(true, timestampUTC, millisecondsUTC, 7000, "pool.ntp.org")) { // will block, will set RTC time
                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: UTC get time error!\n"); }
                        }
                        else {
                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: set %d + 1, UTC milliseconds: %d -> waited for %dms\n", timestampUTC, millisecondsUTC, 1000 - millisecondsUTC); }
                            gotTimeOverWifi = true;
                        }
                    }
                    device.disconnectAndStopWiFi();
                }
                else { device.disconnectAndStopWiFi(); }
                // check result of getting timestamp attempt
                if(gotTimeOverWifi) { return true; }
                else {
                    if(config.trackerMode == MODE_TESTRUN) { printf("Time: wifi was seen, but still something missing (pw wrong, no internet)\n"); }
                    return false;
                }
            }
            else {
                if(config.trackerMode == MODE_TESTRUN) { printf("Time: wifi NOT found\n"); }
                device.disconnectAndStopWiFi();  
                return false;
            }
        }
        else { device.disconnectAndStopWiFi(); return false; }
    }
    return false;
}

void fillDataPayload(uint8_t *data, uint32_t sendingTimestamp, uint8_t dataLen) {
    int64_t temp = (esp_timer_get_time() - timeStartProximity) / 1000; // current time within sending window in ms
    uint16_t sendOffset = (uint16_t) temp; // max value: 65535 = 65 seconds
    uint16_t voltage = (uint16_t) batteryVoltageWakeStub;
    data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] = ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY;
    data[PROX_PAYLOAD_OFFSET_OWN_GROUP_0] = PROXIMITY_OWN_GROUP_0;
    data[PROX_PAYLOAD_OFFSET_OWN_GROUP_1] = PROXIMITY_OWN_GROUP_1;
    data[PROX_PAYLOAD_OFFSET_OWN_GROUP_2] = PROXIMITY_OWN_GROUP_2;
    data[PROX_PAYLOAD_OFFSET_OWN_GROUP_3] = PROXIMITY_OWN_GROUP_3;
    data[PROX_PAYLOAD_OFFSET_OWN_ID_0] = ownTagId >> 8;
    data[PROX_PAYLOAD_OFFSET_OWN_ID_1] = ownTagId & 0xFF;
    data[PROX_PAYLOAD_OFFSET_SENDOFFSET_0] = sendOffset >> 8;
    data[PROX_PAYLOAD_OFFSET_SENDOFFSET_1] = sendOffset & 0xFF;
    data[PROX_PAYLOAD_OFFSET_TSLASTSYNC_0] = timestampLastSync >> 24;
    data[PROX_PAYLOAD_OFFSET_TSLASTSYNC_1] = timestampLastSync >> 16;
    data[PROX_PAYLOAD_OFFSET_TSLASTSYNC_2] = timestampLastSync >> 8;
    data[PROX_PAYLOAD_OFFSET_TSLASTSYNC_3] = timestampLastSync;
    data[PROX_PAYLOAD_OFFSET_TSLASTSYNCTYPE] = lastSyncType;
    data[PROX_PAYLOAD_OFFSET_VOLTAGE_0] = voltage >> 8;
    data[PROX_PAYLOAD_OFFSET_VOLTAGE_1] = voltage & 0xFF;
    data[PROX_PAYLOAD_OFFSET_LASTERRORID] = lastErrorId;
    data[PROX_PAYLOAD_OFFSET_ERRORCNT_0] = errorCnt >> 8;
    data[PROX_PAYLOAD_OFFSET_ERRORCNT_1] = errorCnt & 0xFF;
    data[PROX_PAYLOAD_OFFSET_TIMESTAMP_0] = sendingTimestamp >> 24;
    data[PROX_PAYLOAD_OFFSET_TIMESTAMP_1] = sendingTimestamp >> 16;
    data[PROX_PAYLOAD_OFFSET_TIMESTAMP_2] = sendingTimestamp >> 8;
    data[PROX_PAYLOAD_OFFSET_TIMESTAMP_3] = sendingTimestamp;
    data[PROX_PAYLOAD_OFFSET_SW_VERSION] = WILDFI_SOFTWARE_VERSION;
    data[PROX_PAYLOAD_OFFSET_CONF_VERSION] = WILDFI_CONFIG_VERSION;
    data[PROX_PAYLOAD_OFFSET_SYNCCOUNTER_0] = syncCounter >> 24;
    data[PROX_PAYLOAD_OFFSET_SYNCCOUNTER_1] = syncCounter >> 16;
    data[PROX_PAYLOAD_OFFSET_SYNCCOUNTER_2] = syncCounter >> 8;
    data[PROX_PAYLOAD_OFFSET_SYNCCOUNTER_3] = syncCounter;
    data[PROX_PAYLOAD_OFFSET_FREEMEMORY_0] = freeMemory >> 24;
    data[PROX_PAYLOAD_OFFSET_FREEMEMORY_1] = freeMemory >> 16;
    data[PROX_PAYLOAD_OFFSET_FREEMEMORY_2] = freeMemory >> 8;
    data[PROX_PAYLOAD_OFFSET_FREEMEMORY_3] = freeMemory;
    data[PROX_PAYLOAD_OFFSET_STARTCNT_0] = startCnt >> 24;
    data[PROX_PAYLOAD_OFFSET_STARTCNT_1] = startCnt >> 16;
    data[PROX_PAYLOAD_OFFSET_STARTCNT_2] = startCnt >> 8;
    data[PROX_PAYLOAD_OFFSET_STARTCNT_3] = startCnt;
}

bool isAnyDataGatewayInScanResults() {
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        if(scanResults[i].tagType == TAG_TYPE_GATEWAY) { return true; }
    }
    return false;
}

bool isAnyDataOrNoDataGatewayInScanResults() {
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        if((scanResults[i].tagType == TAG_TYPE_GATEWAY) || (scanResults[i].tagType == TAG_TYPE_GATEWAY_NO_DATA)) { return true; }
    }
    return false;
}

void activationDetection() {
    uint8_t data[250] = { 0 };
    if(!device.initESPNOWStationary(config.proximityLongRange, config.proximityDbm, true, (wifi_phy_rate_t) config.proximityDatarate)) { lastErrorId = 59; errorCnt++; return; } // 23ms
    if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 60; errorCnt++; return; } // 0ms

    esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuous);
    esp_wifi_set_promiscuous(true);
    esp_now_register_recv_cb(proxReceiveCallback); // 0ms

    const esp_timer_create_args_t timerArgs = { .callback = &timerCallback };
    esp_timer_handle_t timer;
    if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { lastErrorId = 61; errorCnt++; return; }
    if(esp_timer_start_once(timer, ((uint32_t) config.activationByGatewayListeningTime) * 1000) != ESP_OK) { lastErrorId = 62; errorCnt++; return; }

    while(!timerFinished) { // always listen for full time
        vTaskDelay(10 / portTICK_PERIOD_MS); // will send current cpu to sleep (10ms accuracy, will wake up a cycle before that)
    }

    // send tag around message (ALWAYS)
    data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] = ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND;
    data[TAG_AROUND_OFFSET_OWN_GROUP_0] = PROXIMITY_OWN_GROUP_0;
    data[TAG_AROUND_OFFSET_OWN_GROUP_1] = PROXIMITY_OWN_GROUP_1;
    data[TAG_AROUND_OFFSET_OWN_GROUP_2] = PROXIMITY_OWN_GROUP_2;
    data[TAG_AROUND_OFFSET_OWN_GROUP_3] = PROXIMITY_OWN_GROUP_3;
    data[TAG_AROUND_OFFSET_OWN_ID_0] = ownTagId >> 8;
    data[TAG_AROUND_OFFSET_OWN_ID_1] = ownTagId & 0xFF;
    data[TAG_AROUND_OFFSET_VOLTAGE_0] = batteryVoltageWakeStub >> 8;
    data[TAG_AROUND_OFFSET_VOLTAGE_1] = batteryVoltageWakeStub & 0xFF;
    data[TAG_AROUND_OFFSET_LASTERRORID] = lastErrorId;
    data[TAG_AROUND_OFFSET_ERRORCNT_0] = errorCnt >> 8;
    data[TAG_AROUND_OFFSET_ERRORCNT_1] = errorCnt & 0xFF;
    data[TAG_AROUND_OFFSET_CMD_BYTE_MIRRORED] = nearestGatewayCommand;
    data[TAG_AROUND_OFFSET_STATE] = (uint8_t) state;
    data[TAG_AROUND_OFFSET_IS_ACTIVATED] = isActivated;
    data[TAG_AROUND_OFFSET_HAS_VALID_TIMESTAMP] = hasValidTimestamp;
    device.broadcastESPNOWData(data, 16); // spit it out  

    if((nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE) || (nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_06_00) || (nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_12_00) || (nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_15_00) || (nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_20_00)) {
        // send activation message
        data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] = ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED;
        data[GOT_ACTIVATED_OFFSET_OWN_GROUP_0] = PROXIMITY_OWN_GROUP_0;
        data[GOT_ACTIVATED_OFFSET_OWN_GROUP_1] = PROXIMITY_OWN_GROUP_1;
        data[GOT_ACTIVATED_OFFSET_OWN_GROUP_2] = PROXIMITY_OWN_GROUP_2;
        data[GOT_ACTIVATED_OFFSET_OWN_GROUP_3] = PROXIMITY_OWN_GROUP_3;
        data[GOT_ACTIVATED_OFFSET_OWN_ID_0] = ownTagId >> 8;
        data[GOT_ACTIVATED_OFFSET_OWN_ID_1] = ownTagId & 0xFF;
        device.broadcastESPNOWData(data, 7); // spit it out
    }
}

bool useEnvironmentSensor() {
    return (environmentSensorConnected && config.environmentActivated);
}

void startSensorTask(void *pvParameter) {
    // needs between 16 to 110 ms
    uint64_t timeMeasureSensor = 0;
    if((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) {
        timeMeasureSensor = Timing::millis();
        startIMU(); // 9ms (acc only) or 103ms (full imu)
        timeMeasureSensor = Timing::millis() - timeMeasureSensor;
        if(config.trackerMode == MODE_TESTRUN) { printf("startSensorTask: IMU started after %dms\n", ((uint32_t) (timeMeasureSensor))); }
    }
    if(useEnvironmentSensor()) {
        timeMeasureSensor = Timing::millis();
        // FOR BSEC USAGE AND AIR QUALITY CALC (see: https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BME680-Using-BSEC-in-backend/td-p/7521):
        // if(!device.baro.init(UINT8_C(2), UINT8_C(1), UINT8_C(5) , BME680_FILTER_SIZE_0, UINT16_C(400), UINT16_C(1943))) { lastErrorId = 14; errorCnt++; } // 4ms
        if(!device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { lastErrorId = 14; errorCnt++; } // 4ms
        if(!device.baro.performMeasurement()) { lastErrorId = 17; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering reading out afterwards)
        timeMeasureSensor = Timing::millis() - timeMeasureSensor;
        if(config.trackerMode == MODE_TESTRUN) { printf("startSensorTask: ENV started after %dms\n", ((uint32_t) (timeMeasureSensor))); }
    }
    vTaskDelete(NULL);
}

void proximityDetectionZeroMs(uint32_t *currentTimestamp) {
    // TESTED
    if(!device.rtc.getTimestamp(currentTimestamp, NULL)) { lastErrorId = 131; errorCnt++; } // get time here because timer just started (shouldn't delay stuff)
    if(((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) || useEnvironmentSensor()) { // create own task to initialize IMU and ENV
        // do NOT call in own task, just start all sensors immediately
        uint64_t timeMeasureSensor = 0;
        device.shortLightSleep(10); // sleep for 10 ms (IMPORTANT! otherwise not enough time for baro to start-up)
        if((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) {
            timeMeasureSensor = Timing::millis();
            startIMU(); // 9ms (acc only) or 103ms (full imu)
            timeMeasureSensor = Timing::millis() - timeMeasureSensor;
            if(config.trackerMode == MODE_TESTRUN) { printf("startSensor: IMU started after %dms\n", ((uint32_t) (timeMeasureSensor))); }
        }
        if(useEnvironmentSensor()) {
            timeMeasureSensor = Timing::millis();
            // FOR BSEC USAGE AND AIR QUALITY CALC (see: https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BME680-Using-BSEC-in-backend/td-p/7521):
            // if(!device.baro.init(UINT8_C(2), UINT8_C(1), UINT8_C(5) , BME680_FILTER_SIZE_0, UINT16_C(400), UINT16_C(1943))) { lastErrorId = 14; errorCnt++; } // 4ms
            if(!device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { lastErrorId = 14; errorCnt++; } // 4ms
            if(!device.baro.performMeasurement()) { lastErrorId = 17; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering reading out afterwards)
            timeMeasureSensor = Timing::millis() - timeMeasureSensor;
            if(config.trackerMode == MODE_TESTRUN) { printf("startSensor: ENV started after %dms\n", ((uint32_t) (timeMeasureSensor))); }
        }
    }
    device.shortLightSleep(90); // sleep for 100 ms
    if(config.useLeds) { device.ledGreenOff(); }
    device.shortLightSleep(900); // sleep for 900 ms
    proximityDetectionCnt++; // important, for longer proximity detection
}

bool proximityDetection(uint32_t *currentTimestamp, uint16_t proximityDuration) { // 1065ms
    uint8_t data[PROXIMITY_DATA_LEN] = { 0 };

    if(!device.initESPNOWStationary(config.proximityLongRange, config.proximityDbm, true, (wifi_phy_rate_t) config.proximityDatarate)) { lastErrorId = 19; errorCnt++; return false; } // 23ms
    if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 20; errorCnt++; return false; } // 0ms

    esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuous);
    esp_wifi_set_promiscuous(true);
    esp_now_register_recv_cb(proxReceiveCallback); // 0ms

    const esp_timer_create_args_t timerArgs = { .callback = &timerCallback };
    esp_timer_handle_t timer;
    if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { lastErrorId = 21; errorCnt++; return false; }
    if(esp_timer_start_once(timer, proximityDuration * 1000) != ESP_OK) { lastErrorId = 22; errorCnt++; return false; }

    timeStartProximity = esp_timer_get_time();
    int64_t timeLastSent = timeStartProximity;
    int64_t timeSendingWindowSize = getApproximatedAirTimeMs() + 1; // 6ms for 1MBPS, 6ms because @1MBPS I see 5.2ms sending peaks (guess with ACK), sending time should normally be: (250 * 8 * 1000 * 1000) / (1 * 1024 * 1024) = 1.9ms
    int64_t timeBetweenSending = ((ownTagId * timeSendingWindowSize * 1000) % (100 * 1000)); // (6ms * ownTagId) -> adding 0..96ms, so modulo, from id 17 it will wrap around again (but with different offset because 100/6 has rest)
    uint8_t sendCounter = 0;
    uint16_t taskDelayMs;
    //uint64_t timeMeasureSensor = 0;
    if(!device.rtc.getTimestamp(currentTimestamp, NULL)) { lastErrorId = 131; errorCnt++; } // get time here because timer just started (shouldn't delay stuff)

	if(state != ST_NIGHTTIME_DATATRANS) { // only do IMU stuff in ST_TRACK
		if(((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) || useEnvironmentSensor()) { // create own task to initialize IMU and ENV
			if(xTaskCreate(&startSensorTask, "startSensorTask", 2048, NULL, 5, NULL) != pdPASS) { lastErrorId = 140; errorCnt++; } // priority 5 = higher than this here
		}
	}

    // task delay 
    uint32_t firstTaskDelayMs = timeBetweenSending / 1000; // e.g. 56ms
    firstTaskDelayMs = firstTaskDelayMs - (firstTaskDelayMs % 10); // e.g. 50ms
    if(firstTaskDelayMs > 10) {
        firstTaskDelayMs = firstTaskDelayMs - 10; // e.g. 40ms
        vTaskDelay(firstTaskDelayMs / portTICK_PERIOD_MS);
    }

    while(!timerFinished) {
        if((esp_timer_get_time() - timeLastSent) >= timeBetweenSending) {
            timeLastSent = esp_timer_get_time(); // reset send counter
            timeBetweenSending = 100 * 1000; // 100ms until next 
            fillDataPayload(data, *currentTimestamp, PROXIMITY_DATA_LEN); // fill data frame
            device.broadcastESPNOWData(data, PROXIMITY_DATA_LEN); // spit it out
            sendCounter++;
            taskDelayMs = 90;

            if(sendCounter == 2) { // after 100ms + 0..96ms
                if(config.useLeds) { device.ledGreenOff(); }
            }
            if(sendCounter < (proximityDuration / 100)) { // if NOT last message (= 10 = last message, don't delay here because shorter than 100ms)
                //if(taskDelayMs > 0) { // might not sleep at all when IMU got activated with full 9-axis (= 103ms)
                    vTaskDelay(taskDelayMs / portTICK_PERIOD_MS); // will send current cpu to sleep (10ms accuracy, will wake up a cycle before that)
                //}
            }
        }
    }
    /*if((((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) || useEnvironmentSensor()) && (config.trackerMode == MODE_TESTRUN)) {
        printf("Prox: sensor start %dms\n", ((uint32_t) (timeMeasureSensor)));
    }*/
    // ESP NOW still running! to send out data in case a gateway has been seen
    proximityDetectionCnt++;
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

push_data_result_t pushDataIntoMemory(uint8_t *newData, uint32_t newDataLen, bool *memoryFull, bool debug) { // no limit for pushing data
    esp_err_t err;
    nvs_handle_t handle;
    size_t nvsDataLen = 0;  // value will default to 0, if not set yet in NVS
    uint8_t nvsData[NVS_FLASH_BUFFER_SIZE] = { 0 };
    push_data_result_t returnVal = PUSH_DATA_SUCCESS;
    uint32_t pushDuration = (uint32_t) Timing::millis();

    if((newDataLen == 0) || (newData == NULL)) { return PUSH_DATA_PARAM_ERROR; }

    // get current size of NVS buffer
    if(nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle) != ESP_OK) { return PUSH_DATA_NVS_ERROR; }
    err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, NULL, &nvsDataLen); // 0ms
    if(err == ESP_ERR_NVS_NOT_FOUND) { // first time, blob is empty
        if(debug) { printf("pushData: virgin NVS\n"); }
        nvsDataLen = 0;
    }
    else if(err != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_GET_BLOB_ERROR; }

    // read NVS buffer (in case buffer is not empty)
    //if(debug) { printf("pushData: %d bytes in NVS\n", nvsDataLen); }
    if(nvsDataLen > 0) {  
        err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, &nvsDataLen); // 1ms
        if(err != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_GET_BLOB_ERROR2; }
        /*if(debug) {
            for(uint16_t i = 0; i < nvsDataLen; i++) { printf("%02X ", nvsData[i]); } 
            printf("\n");
        }*/
    }
    
	if(nvsDataLen + newDataLen < NVS_FLASH_BUFFER_SIZE) { // if new data fits into NVS (NVS_FLASH_BUFFER_SIZE never reached)
        if(debug) { printf("pushData: NVS fit %d + %d < %d\n", nvsDataLen, newDataLen, NVS_FLASH_BUFFER_SIZE); }
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
	else { // need to write to flash
		if(debug) { printf("pushData: write flash %d + %d >= %d\n", nvsDataLen, newDataLen, NVS_FLASH_BUFFER_SIZE); }
        uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
        uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
        uint16_t flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_BLOCK_DELETED_POINTER);
				
		// first check if there is enough space in flash memory
        // TEST: CONSIDER ERASED DATA
		uint32_t freeSpaceInFlash = device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPageWritePointer, flashPageWriteOffsetPointer, MT29_NUMBER_PAGES);
		uint32_t dataToStoreInFlash = (nvsDataLen + newDataLen) - ((nvsDataLen + newDataLen) % NVS_FLASH_BUFFER_SIZE);
		if(debug) {
            //printf("pushData: %d bytes to flash (rest: %d)\n", dataToStoreInFlash, ((nvsDataLen + newDataLen) % NVS_FLASH_BUFFER_SIZE));
            printf("pushData: writePointer %d.%d (del: %d), space: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer, flashBlockDeletedPointer, freeSpaceInFlash);
        }
		if(freeSpaceInFlash <= dataToStoreInFlash) { // IMPORTANT: if "<" and just fits in memory -> wrap around of pointers to 0.0, then overwriting existing data! horror!
            if(debug) { printf("pushData: flash full\n"); }
            *memoryFull = true;			
		}
		else { // enough space, so start writing
			// FIRST: write NVS_FLASH_BUFFER_SIZE bytes of NVS + newData[part1]
			uint32_t newDataLeftPointer = 0;
			sequential_write_status_t writeStatus;
			for(newDataLeftPointer = 0; newDataLeftPointer < newDataLen; newDataLeftPointer++) {
	        	if(nvsDataLen + newDataLeftPointer >= NVS_FLASH_BUFFER_SIZE) { break; }
	        	nvsData[nvsDataLen + newDataLeftPointer] = newData[newDataLeftPointer];
	    	}
	    	if(debug) { printf("pushData: A: %d bytes\n", NVS_FLASH_BUFFER_SIZE); }
            // TEST: CONSIDER ERASED DATA
	    	writeStatus = device.flash.fifoPushSimple(flashBlockDeletedPointer, flashPageWritePointer, flashPageWriteOffsetPointer, nvsData, NVS_FLASH_BUFFER_SIZE, true, true, debug);
	    	if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_MEM_FULL_FATAL; } // SHOULD NOT HAPPEN, checked before, don't stop in case of error
            else if(writeStatus == MT29_SEQ_WRITE_STATUS_BUFFER_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_BUFFER_ERROR; } // don't stop in case of error
            else if(writeStatus == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR; } // don't stop in case of error
            else if(writeStatus == MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR) { // don't stop in case of error
                // SELDOMLY HAPPENS!
                returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR1;
            }
            
            // SECOND: check if more than NVS_FLASH_BUFFER_SIZE bytes left (then directly write to flash)
            uint32_t leftDataToWrite = newDataLen - newDataLeftPointer;
            //if(debug) { printf("pushData: B: left %d bytes\n", leftDataToWrite); }
            if(leftDataToWrite >= NVS_FLASH_BUFFER_SIZE) {
            	uint32_t flashWriteLength = leftDataToWrite - (leftDataToWrite % NVS_FLASH_BUFFER_SIZE);
            	if(debug) { printf("pushData: B: writing %d bytes to flash directly\n", flashWriteLength); }
                // TEST: CONSIDER ERASED DATA
	 	    	writeStatus = device.flash.fifoPushSimple(flashBlockDeletedPointer, flashPageWritePointer, flashPageWriteOffsetPointer, newData + newDataLeftPointer, flashWriteLength, true, true, debug);
		    	if(writeStatus == MT29_SEQ_WRITE_STATUS_MEMORY_FULL) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_MEM_FULL_FATAL; } // SHOULD NOT HAPPEN, checked before, don't stop in case of error
	            else if(writeStatus == MT29_SEQ_WRITE_STATUS_BUFFER_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_BUFFER_ERROR; } // don't stop in case of error
	            else if(writeStatus == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR) { returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR; } // don't stop in case of error
	            else if(writeStatus == MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR) { // don't stop in case of error
                    // SELDOMLY HAPPENS!
                    returnVal = PUSH_DATA_MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR2;
                } 
	            newDataLeftPointer += flashWriteLength; 
            }
            
            leftDataToWrite = newDataLen - newDataLeftPointer;
            if(debug) { printf("pushData: C: %d bytes into NVS\n", leftDataToWrite); }
            
            // THIRD: if there is a rest -> write that to NVS
            if(leftDataToWrite > 0) {
		         // add rest data to "fresh" NVS_FLASH_BUFFER_SIZE byte array
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
            else { // reset NVS length to 0 (tested separately, works fine)
            	if(nvsDataLen > 0) { // only write zero if not already is zero
	            	if(nvs_set_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, 0) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_SET_BLOB_ERROR2; } // TIME CONSUMING: 7 - 117ms
			        if(nvs_commit(handle) != ESP_OK) { nvs_close(handle); return PUSH_DATA_NVS_COMMIT_ERROR2; } // 0ms
			        nvs_close(handle); // 0ms
		    	}
            }
            
            if(!device.nvsWriteUINT32andUINT16(NVS_FLASH_WRITE_PAGE_POINTER, flashPageWritePointer, NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, flashPageWriteOffsetPointer)) { returnVal = PUSH_DATA_NVS_WRITE_ERROR; } // don't stop in case of error
	    	
            // update free space variable for proximity detection
            freeMemory = device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPageWritePointer, flashPageWriteOffsetPointer);

            // handle sending blob pointer
            uint16_t nvsSendBlobPointer = device.nvsReadUINT16(NVS_FLASH_SEND_BLOB_POINTER);
            if(nvsSendBlobPointer > 0) { // already sent something with mocked pointer addresses from blob
                uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_PAGE_POINTER);
                uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
                if(debug) { printf("pushData: did send %d bytes from blob before, sendPointer %d.%d -> ", nvsSendBlobPointer, sendPagePointer, sendPageOffsetPointer); }
                sendPageOffsetPointer += nvsSendBlobPointer;
                if(sendPageOffsetPointer >= MT29_CACHE_SIZE) {
                    sendPageOffsetPointer = sendPageOffsetPointer % MT29_CACHE_SIZE;
                    sendPagePointer++;
                    if(sendPagePointer >= MT29_NUMBER_PAGES) { // this should never happen with 512 bytes chunks because nvsSendBlobPointer is maximum 511
						sendPagePointer = 0; lastErrorId = 30; errorCnt++;
					} 
                    if(!device.nvsWriteUINT32(NVS_FLASH_SEND_PAGE_POINTER, sendPagePointer)) { returnVal = PUSH_DATA_NVS_WRITE_ERROR; }
                }
                if(!device.nvsWriteUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER, sendPageOffsetPointer)) { returnVal = PUSH_DATA_NVS_WRITE_ERROR; }
                if(!device.nvsWriteUINT16(NVS_FLASH_SEND_BLOB_POINTER, 0)) { returnVal = PUSH_DATA_NVS_WRITE_ERROR; } // reset blob send pointer to 0
                if(debug) { printf("%d.%d\n", sendPagePointer, sendPageOffsetPointer); }
        	}
		}
	}

    pushDuration = ((uint32_t) Timing::millis()) - pushDuration;
    if(debug) { printf("pushData: %dms\n", pushDuration); }

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

        if(config.trackerMode == MODE_TESTRUN) { printf("Store: (%d bytes) ", sensorDataLen); printArray(sensorData, sensorDataLen); }
        push_data_result_t res = pushDataIntoMemory(sensorData, sensorDataLen, memoryFull, (config.trackerMode == MODE_TESTRUN)); // average 25ms
        if(res != PUSH_DATA_SUCCESS) { lastErrorId = 93 + res; errorCnt++; } // 14 error codes (0 = sucess), 94 - 107
        heap_caps_free(sensorData);
    }
}
                        
void storeProximityDataInFlash(uint32_t timestamp, uint8_t imuMode, uint8_t *fifoData, uint16_t fifoLen, bool withEnv, int16_t temperature, uint32_t pressure, uint32_t humidity, bool *memoryFull) {
    uint8_t *sensorData = NULL; // pointer to allocated memory
    uint16_t headerDataLen = 0;
    bool withImu = (imuMode == IMU_ACC_ONLY) || (imuMode == IMU_ACC_MAG_GYRO);
    if((!withImu) && (!withEnv)) { headerDataLen = 9; } // only proximity
    else if((withImu) && (!withEnv)) { headerDataLen = 11; } // proximity + imu
    else if((!withImu) && (withEnv)) { headerDataLen = 19; } // proximity + env
    else { headerDataLen = 21; } // proximity + imu + env
    const uint16_t proxDataLen = scanResultsPointer * 3;

    if(!device.flash.createBuffer(&sensorData, headerDataLen + proxDataLen + fifoLen)) { lastErrorId = 34; errorCnt++; return; }
    else {
        if(sensorData == NULL) { lastErrorId = 35; errorCnt++; return; }
        
        // serialize data for storage
        sensorData[0] = 0x12;
        sensorData[1] = 0x34;

        if((!withImu) && (!withEnv)) { // only proximity, 9 byte header
            sensorData[2] = 0x57;
            HelperBits::addData4(timestamp, &sensorData[3]); // add timestamp, 4 bytes 
            HelperBits::addData2(proxDataLen, &sensorData[7]); // add prox data len
            if(config.trackerMode == MODE_TESTRUN) { printf("Store: only prox: H%d + P%d + F%d bytes\n", headerDataLen, proxDataLen, fifoLen); }
        }
        else if((withImu) && (!withEnv)) { // proximity + imu, 11 byte header
            if(imuMode == IMU_ACC_ONLY) { sensorData[2] = 0x62; }
            else { sensorData[2] = 0x65; }
            HelperBits::addData4(timestamp, &sensorData[3]); // add timestamp, 4 bytes 
            HelperBits::addData2(proxDataLen, &sensorData[7]); // add prox data len
            HelperBits::addData2(fifoLen, &sensorData[9]); // add fifo data len
            if(config.trackerMode == MODE_TESTRUN) { printf("Store: prox+imu: H%d + P%d + F%d bytes\n", headerDataLen, proxDataLen, fifoLen); }
        }
        else if((!withImu) && (withEnv)) { // proximity + env, 19 byte header
            sensorData[2] = 0x63;
            HelperBits::addData4(timestamp, &sensorData[3]); // add timestamp, 4 bytes 
            HelperBits::addData2(temperature, &sensorData[7]); // add temperature
            HelperBits::addData4(humidity, &sensorData[9]); // add humidity
            HelperBits::addData4(pressure, &sensorData[13]); // add pressure
            HelperBits::addData2(proxDataLen, &sensorData[17]); // add prox data len
            if(config.trackerMode == MODE_TESTRUN) { printf("Store: prox+env: H%d + P%d + F%d bytes\n", headerDataLen, proxDataLen, fifoLen); }
        }
        else { // proximity + imu + env, 21 byte header
            if(imuMode == IMU_ACC_ONLY) { sensorData[2] = 0x64; }
            else { sensorData[2] = 0x66; }
            HelperBits::addData4(timestamp, &sensorData[3]); // add timestamp, 4 bytes 
            HelperBits::addData2(temperature, &sensorData[7]); // add temperature
            HelperBits::addData4(humidity, &sensorData[9]); // add humidity
            HelperBits::addData4(pressure, &sensorData[13]); // add pressure
            HelperBits::addData2(proxDataLen, &sensorData[17]); // add prox data len
            HelperBits::addData2(fifoLen, &sensorData[19]); // add fifo data len
            if(config.trackerMode == MODE_TESTRUN) { printf("Store: prox+env+imu: H%d + P%d + F%d bytes\n", headerDataLen, proxDataLen, fifoLen); }
        }
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

        // fill fifo data
        if(withImu) {
            for(uint16_t i=0; i<fifoLen; i++) {
                sensorData[currentPointer] = fifoData[i];
                currentPointer++;
            }
        }
	    if(currentPointer != headerDataLen + proxDataLen + fifoLen) { heap_caps_free(sensorData); lastErrorId = 39; errorCnt++; return; } // needs to be the same        
        
        // store data in flash memory
        if(config.trackerMode == MODE_TESTRUN) {
            //printArray(sensorData, headerDataLen + proxDataLen + fifoLen);
        }
        push_data_result_t res = pushDataIntoMemory(sensorData, headerDataLen + proxDataLen + fifoLen, memoryFull, (config.trackerMode == MODE_TESTRUN)); // average 25ms
        if(res != PUSH_DATA_SUCCESS) { lastErrorId = 79 + res; errorCnt++; } // 14 error codes (0 = sucess) -> 80 - 93
        heap_caps_free(sensorData); 
    }
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
        // get current size of NVS_FLASH_BUFFER_SIZE byte NVS buffer
        if(nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle) != ESP_OK) { lastErrorId = 13; errorCnt++; return; }
        err = nvs_get_blob(handle, NVS_FLASH_BUFFER_NAME, NULL, &nvsDataLen); // 0ms
        if(err == ESP_ERR_NVS_NOT_FOUND) { // first time, blob is empty
            nvsDataLen = 0;
        }
        else if(err != ESP_OK) { lastErrorId = 16; errorCnt++; nvs_close(handle); return; }

        uint16_t bytesToSendFromBlob = nvsDataLen - (*blobSendPointer);
        if((*blobSendPointer) > nvsDataLen) { lastErrorId = 23; errorCnt++; nvs_close(handle); return; }
        if(config.trackerMode == MODE_TESTRUN) { printf("dataTransBlob: %d in blob, %d sent -> %d bytes to send from blob\n", nvsDataLen, *blobSendPointer, bytesToSendFromBlob); }

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

                if(config.trackerMode == MODE_TESTRUN) { printf("dataTransBlob: 1 msg: %d + %d bytes with mockPointer %d.%d\n", 7, dataMsgLen - 7, sendPagePointerMock, sendPageOffsetPointerMock); }
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
							if(config.trackerMode == MODE_TESTRUN) { printf("dataTransBlob: no ack received\n"); }
							break;
						}
						else if(statusSend == ESP_NOW_FINISHED_AND_RECEIVED) { // 10-12ms (long range), 1-4ms in normal mode
							receivedAck = true;
							if(!haveSeenAnyAck) { haveSeenAnyAck = true; } // to see if gateway was in reach at all
							break;
						}
						if(((uint32_t) Timing::millis()) - startSendingTime > 500) {
							if(config.trackerMode == MODE_TESTRUN) { printf("dataTransBlob: fatal ack timeout\n"); }
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
		                // this can't happen, because before data from blob is sent all other data from flash needs to be sent -> meaning real send point would already be zero!
						if(sendPagePointerMock >= MT29_NUMBER_PAGES) { sendPagePointerMock = 0; lastErrorId = 29; errorCnt++; }
		
						// update real NVS pointer now
		                *blobSendPointer = blobSendPointerTemp;
		                bytesToSendFromBlob = nvsDataLen - (*blobSendPointer);
						break;
					} 
					else {
						if(haveSeenAnyAck) { // got at least one ack before
							if(sendRetryCounter >= maxSendRetries) { // after 3 tries failed
								if(config.trackerMode == MODE_TESTRUN) { printf("dataTransBlob: %d.%d retry %d -> STOP\n", sendPagePointerMock, sendPageOffsetPointerMock, sendRetryCounter); }
								nvs_close(handle); 	
								return;
							}
							else { // had some successful transmission before - retry!
								Timing::delay(millisWaitIfSendFailed); // wait a tiny bit
								if(config.trackerMode == MODE_TESTRUN) { printf("dataTransBlob: %d.%d retry %d\n", sendPagePointerMock, sendPageOffsetPointerMock, sendRetryCounter); }	
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
    uint8_t lowestRssi = 0xFF;
    uint16_t gatewayCounter = 0;
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        if(scanResults[i].tagType == TAG_TYPE_GATEWAY) { 
            if(scanResults[i].rssiMax < lowestRssi) { // low RSSI = should be closer, take highest RSSI of all gateway messages (max value)
                anyGatewaySeen = true;
                lowestRssi = scanResults[i].rssiMax;
                gatewayMac = scanResults[i].mac;
            }
            gatewayCounter++;        
        }
    }
    if(anyGatewaySeen && (gatewayMac != NULL) && (lowestRssi != 0xFF)) {
        if(!device.addESPNOWReceiverStationary(gatewayMac)) { lastErrorId = 43; errorCnt++; return; }

        if(config.trackerMode == MODE_TESTRUN) { printf("dataTrans: %d gateway(s) seen, lowest RSSI: %d\n", gatewayCounter, lowestRssi); }
        uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_PAGE_POINTER);
        uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
        uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
        uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
        uint16_t nvsSendBlobPointer = device.nvsReadUINT16(NVS_FLASH_SEND_BLOB_POINTER);

        uint32_t sendBytePointer = (sendPagePointer * MT29_CACHE_SIZE) + sendPageOffsetPointer;
	    uint32_t writeBytePointer = (flashPageWritePointer * MT29_CACHE_SIZE) + flashPageWriteOffsetPointer;
	    uint32_t bytesToSendFromFlash = device.flash.fifoGetNumberOfPopableBytes(sendBytePointer, writeBytePointer);

        if(config.trackerMode == MODE_TESTRUN) { printf("dataTrans: flashPageWritePointer: %d, flashPageWriteOffsetPointer: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer); }
        if(config.trackerMode == MODE_TESTRUN) { printf("dataTrans: before: bytesToSendFromFlash: %d, sendPagePointer: %d, sendPageOffsetPointer: %d, nvsSendBlobPointer: %d\n", bytesToSendFromFlash, sendPagePointer, sendPageOffsetPointer, nvsSendBlobPointer); }
    
        if(gatewayMac != NULL) {
            //testSend(gatewayMac);
            uint8_t customPrefix[2];
            customPrefix[0] = ownTagId >> 8;
            customPrefix[1] = ownTagId & 0x00FF;
            uint32_t sendPagePointerOld = sendPagePointer;
            uint16_t sendPageOffsetPointerOld = sendPageOffsetPointer;
            uint16_t nvsSendBlobPointerOld = nvsSendBlobPointer;

            // TEST: CONSIDER ERASED DATA (should work)
            // stream the data
            esp_now_stream_status_t espNowStatus = device.doESPNOWFlashStreamNew(
                gatewayMac,
                customPrefix, 2,
                &sendPagePointer, &sendPageOffsetPointer, 
                flashPageWritePointer, flashPageWriteOffsetPointer, 
                ESP_NOW_FLASH_STREAM_NO_LIMIT,
                500, 8, // millis to wait when one message failed (but acks happened before), number of retries
                config.battMinVoltageDuringTransmission,
                (config.trackerMode == MODE_TESTRUN) ? 1 : 0, MOCK_FLASH_READ, false);

            if(espNowStatus != ESP_NOW_STREAM_DATA_FINISHED) {
                if((espNowStatus != ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR) && (espNowStatus != ESP_NOW_STREAM_NO_DATA_TO_SEND)) { // no data to send happens when function is called with minBlocksToTransmit = 0 (forcing a gateway scan) 
                    if(config.trackerMode == MODE_TESTRUN) { printf("dataTrans: STREAM ERROR, status: %d!\n", espNowStatus); }
                    lastErrorId = 108 + espNowStatus;  errorCnt++; // max. 108 + 12 -> add some buffer = 130
                }
                else { // normal case, no gateway found
                    if(config.trackerMode == MODE_TESTRUN) { printf("dataTrans: nothing (neverAck %d, noData %d)!\n", (espNowStatus == ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR), (espNowStatus == ESP_NOW_STREAM_NO_DATA_TO_SEND)); }
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
            if(config.trackerMode == MODE_TESTRUN) { printf("dataTrans: after: bytesToSendFromFlash: %d, sendPagePointer: %d, sendPageOffsetPointer: %d, nvsSendBlobPointer: %d\n", bytesToSendFromFlash, sendPagePointer, sendPageOffsetPointer, nvsSendBlobPointer); }
        }
    }
}

bool randomHourOkayWithNightTime(uint8_t hour) {
    uint16_t minutesOfDay = 0;
    const uint16_t OFF_MINUTES_OF_DAY = (config.nightTimeTurnOffHour * 60) + config.nightTimeTurnOffMinute; // 0 ........ 1439
    const uint16_t ON_MINUTES_OF_DAY = (config.nightTimeTurnOnHour * 60) + config.nightTimeTurnOnMinute; // 0 ........ 1439
    if(config.nightTimeEnter == NIGHTTIME_ALWAYS_NIGHT) { return true; } // always ok
    else if(config.nightTimeEnter == NIGHTTIME_DISABLED) { return true; } // always ok
    minutesOfDay = (hour * 60) + 0; // ASSUMING ALWAYS ON FULL HOUR (GPS)
    if(OFF_MINUTES_OF_DAY < ON_MINUTES_OF_DAY) { // ----OFF___ON--------
        if((minutesOfDay > OFF_MINUTES_OF_DAY) && (minutesOfDay < ON_MINUTES_OF_DAY)) {
            return false; // hour within night time 
        }
    }
    else { // ___ON----------OFF__
        if((minutesOfDay > OFF_MINUTES_OF_DAY) || (minutesOfDay < ON_MINUTES_OF_DAY)) {
            return false; // hour within night time 
        }
    }
    return true; 
}

bool hourEqualsGPSTrySetting(uint8_t hourIn) {
    if(config.trackerMode == MODE_TESTRUN) {
        printf("hourEqualsGPSTrySetting: ");
    	for(uint8_t hr=0; hr<24; hr++) {
		    bool shouldTryGPS = (gpsFixHours >> hr) & 1UL;
		    if(shouldTryGPS) { 
                if(hr == hourIn) { printf("[G]"); }
                else { printf("G"); }
            }
		    else {
                if(randomHourOkayWithNightTime(hr)) { // not night time
                    if(hr == hourIn) { printf("[_]"); }
                    else { printf("_"); }
                }
                else {
                     if(hr == hourIn) { printf("[s]"); }
                    else { printf("s"); }                   
                }
            }
	    }
        printf("\n");
    }
    for(uint8_t hr=0; hr<24; hr++) {
		bool shouldTryGPS = (gpsFixHours >> hr) & 1UL;
		if(hourIn == hr) {
			if(shouldTryGPS) {
				if(config.trackerMode == MODE_TESTRUN) { printf("hourEqualsGPSTrySetting: YES (%d)\n", hourIn); }
                return true;
			}
		}
	}
    return false;
}

bool updateHourAndMinuteForWakeStub() {
    bool error = false;
    uint8_t currentMinuteOld = currentMinute;
    uint8_t currentHourOld = currentHour;
    currentMinute = device.rtc.getMinutes(error); // update current minute
    currentHour = device.rtc.getHours(error); // update current hour
    if(error) {
        lastErrorId = 5; errorCnt++;
        return false;
    }
    if(currentMinuteOld != currentMinute) { // can happen if GPS takes longer than 1 minute and after starting
        if(config.trackerMode == MODE_TESTRUN) { printf("updateHourAndMinuteForWakeStub: WARNING, minutes unequal: old %d:%d vs. new %d:%d\n", currentHourOld, currentMinuteOld, currentHour, currentMinute); }
    }
    if(currentHourOld > currentHour) { // day wrapped around
        if(config.trackerMode == MODE_TESTRUN) { printf("updateHourAndMinuteForWakeStub: next day %d:%d -> %d:%d\n", currentHourOld, currentMinuteOld, currentHour, currentMinute); }
        currentDayCounter++;
        currentDayCounterChanged = true;
    }
    return true;
}

void gpsRandomizer() {
    if(config.gpsRandomizeFixes) {
        if(currentDayCounterChanged) {
            uint32_t safetyCounter = 0;
            currentDayCounterChanged = false;
            if(config.trackerMode == MODE_TESTRUN) { printf("gpsRandomizer: day changed (%d) -> randomize\n", currentDayCounter); }
            uint64_t timeMeasure = Timing::millis();
            if(config.gpsRandomizeFixesPerDay >= 23) {
                gpsFixHours = 0x00FFFFFF;
            }
            else if(config.gpsRandomizeFixesPerDay == 0) {
                gpsFixHours = 0;
            }
            else {
                bootloader_random_enable(); // true random numbers
                uint8_t itCounter = 0;
                gpsFixHours = 0;
                
                while(true) {
                    uint32_t randomNumber = esp_random() % 24; // 0 .. 23
                    uint32_t alreadySet = (gpsFixHours & (1UL << randomNumber));
                    if(alreadySet == 0) { // take that time, still free
                        if(randomHourOkayWithNightTime((uint8_t) randomNumber)) { // check if random number is within night time
                            if(config.trackerMode == MODE_TESTRUN) { printf("gpsRandomizer: hr %d\n", randomNumber); }
                            gpsFixHours |= (1UL << randomNumber);
                            itCounter++;
                            if(itCounter >= config.gpsRandomizeFixesPerDay) { break; }
                        }
                    }
                    else {
                        //if(config.trackerMode == MODE_TESTRUN) { printf("gpsRandomizer: %d -> REPEAT\n", randomNumber); }
                    }
                    safetyCounter++;
                    if(safetyCounter > 4000) {
                        lastErrorId = 136; errorCnt++;
                        if(config.trackerMode == MODE_TESTRUN) { printf("gpsRandomizer: COULD NOT FILL RANDOM FIXES (night time too long?)\n"); }
                        break;
                    }
                }
                bootloader_random_disable();
            }
            timeMeasure = Timing::millis() - timeMeasure;
            if(config.trackerMode == MODE_TESTRUN) {
                printf("gpsRandomizer: DONE, took %dms (%d its): ", (uint32_t)(timeMeasure), safetyCounter);
                for(uint8_t hr=0; hr<24; hr++) {
                    bool shouldTryGPS = (gpsFixHours >> hr) & 1UL;
                    if(shouldTryGPS) { printf("G"); }
                    else { 
                        if(randomHourOkayWithNightTime(hr)) { printf("_"); } // awake at that hour
                        else { printf("s"); } // not awake
                    }
                }
                printf("\n");
            }
        }
    }
    else { gpsFixHours = config.gpsFixHourBits; } // not randomized -> use default hour setting
}

bool checkIfFlashMemoryCanBeFreed() {
    uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_PAGE_POINTER); // 0 .. 131071
    uint16_t flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_BLOCK_DELETED_POINTER);
    uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER); // writePointers to update free memory variable
    uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER); // writePointers to update free memory variable

    if(config.trackerMode == MODE_TESTRUN) {
        uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);  // also not relevant!
        uint16_t nvsSendBlobPointer = device.nvsReadUINT16(NVS_FLASH_SEND_BLOB_POINTER); // also not relevant
        printf("checkIfFlashMemoryCanBeFreed: writePointer: %d.%d, sendPointer: %d.%d, sendPointerBlob: %d, blockDeletePointer: %d\n", flashPageWritePointer, flashPageWriteOffsetPointer, sendPagePointer, sendPageOffsetPointer, nvsSendBlobPointer, flashBlockDeletedPointer);
    }

    uint32_t blockSend = sendPagePointer / MT29_PAGES_PER_BLOCK; // e.g. sendPagePointer (0 .. 131071) = 64 -> blockSend = 1 (delete only 0)
    if(config.trackerMode == MODE_TESTRUN) { printf("checkIfFlashMemoryCanBeFreed: blocks: %d - %d\n", flashBlockDeletedPointer, blockSend); }

    // SCENARIO 1
    //  flashBlockDeletedPointer
    //                       sendBytePointer              writeBytePointer
    // |xxxxxxxx|xxxxxxxx|DDDDDDDD|DDDDDDDD|DDDDDDDD|DDDDD___|

    // SCENARIO 2
    //                            flashBlockDeletedPointer
    //                         writeBytePointer
    //            sendBytePointer                                  
    // xxxxxxxx|DDDDDDDD|DDDDDD__|xxxxxxxx|xxxxxxxx|

    bool eraseError = false;
    bool somethingDeleted = false;
    while(flashBlockDeletedPointer != blockSend) {
        if(!device.flash.erase(flashBlockDeletedPointer)) { eraseError = true; break; }
        if(config.trackerMode == MODE_TESTRUN) { printf("checkIfFlashMemoryCanBeFreed: deleted block %d\n", flashBlockDeletedPointer); }
        flashBlockDeletedPointer = device.flash.fifoIncrementBlocksErasedPointer(flashBlockDeletedPointer);
        somethingDeleted = true;
    }
    if(somethingDeleted) {
        device.nvsWriteUINT16(NVS_FLASH_BLOCK_DELETED_POINTER, flashBlockDeletedPointer); // update pointer, also in case of error
        if(config.trackerMode == MODE_TESTRUN) { printf("checkIfFlashMemoryCanBeFreed: deleted something, new blockDeletePointer: %d\n", flashBlockDeletedPointer); }
        freeMemory = device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPageWritePointer, flashPageWriteOffsetPointer); // udate free memory variable
    }
    else {
        if(config.trackerMode == MODE_TESTRUN) { printf("checkIfFlashMemoryCanBeFreed: nothing deleted\n"); }
    }
    if(eraseError) { lastErrorId = 141; errorCnt++; }
    return somethingDeleted; // something deleted -> memory shouldn't be full now
}

void imuLightSleepTrackingModeAddData(uint16_t voltage, uint16_t currentFifoLen, uint8_t *buffer, uint16_t *bufferPointer) {
    uint32_t timestamp = 0;
    uint8_t milliseconds = 0;
    if(!device.rtc.getTimestamp(&timestamp, &milliseconds)) { lastErrorId = 143; errorCnt++; }
    int16_t temperature = 0, temperatureBmx = 0;
    uint16_t temperatureBmxRaw = 0;
    uint32_t pressure = 0;
    uint32_t humidity = 0;

    if(!device.imu.getTemperatureRaw(temperatureBmxRaw)) { lastErrorId = 144; errorCnt++; }
    temperatureBmx = device.imu.toCelsiusx100(temperatureBmxRaw);
    bool error = false;
    if(useEnvironmentSensor()) {
        if(device.baro.getResults()) { // normally waits up to 31ms, but measurement was triggered before lightsleep
            temperature = device.baro.getTemperature(error);
            if(error) { lastErrorId = 145; errorCnt++; }
            pressure = device.baro.getPressure(error);
            if(error) { lastErrorId = 146; errorCnt++; }
            humidity = device.baro.getHumidity(error);
            if(error) { lastErrorId = 147; errorCnt++; }
        }
    }

    HelperBits::addData1_AndIncrementPointer(0x12, buffer, bufferPointer);
    HelperBits::addData1_AndIncrementPointer(0x34, buffer, bufferPointer);
    if(config.imuMode == IMU_ACC_MAG_GYRO) { HelperBits::addData1_AndIncrementPointer(0x5E, buffer, bufferPointer); }
    else { HelperBits::addData1_AndIncrementPointer(0x61, buffer, bufferPointer); }
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
    if(config.trackerMode == MODE_TESTRUN) { printf("imuLightSleepTrackingMode: UTC %d.%d, TEMP %d/%d, PRESS %d, HUM %d\n", timestamp, milliseconds, temperature, temperatureBmx, pressure, humidity); } 
}

void imuLightSleepTrackingMode(bool baroAlreadyInitialized, uint32_t burstDurationMilliseconds) { // IMU needs to be on, flash needs to be off
    const uint16_t imuLightSleepTrackingModeHeaderLength = 27;
    const uint16_t FIFO_DATA_LEN = ((1024 + imuLightSleepTrackingModeHeaderLength) * 32) + 1; // WARNING: 16 bit, maximum 2^16
    bool keepTrackingRunning = true;
    bool memoryFull = false;
    uint16_t fifoDataPointer = 0;
    int64_t trackStartUs = 0;

    if(burstDurationMilliseconds <= 100) { return; }

    // change clock speed to 1MHz
    i2c.changeClockSpeed(I2C_FREQ_HZ_1MHZ); // works

    // initialize fifo interrupts (WARNING: IMU still running and recording data)
    if(!device.imu.enableFIFOInterrupt(IMU_BURST_FIFO_SIZE)) { lastErrorId = 148; errorCnt++; return; }

    // malloc the fifo RAM
    uint8_t *fifoData = (uint8_t*) malloc(FIFO_DATA_LEN);
    if(fifoData == NULL) { lastErrorId = 149; errorCnt++; return; }

    // initalize barometer if not already initialized
    if(useEnvironmentSensor()) {
        if(!baroAlreadyInitialized) {
            if(!device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { lastErrorId = 150; errorCnt++; } // 4ms
        }
    }

    // start timer
    trackStartUs = esp_timer_get_time(); // start timer

    if(useEnvironmentSensor()) {
        if(!device.baro.performMeasurement()) { lastErrorId = 151; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
    }
    device.enableAccInterruptInDeepSleep();
    esp_sleep_enable_timer_wakeup(50000000UL); // in case imu does not answer (50 second timeout)
    device.lightSleep();

    while(keepTrackingRunning) {
        uint16_t voltage = device.readSupplyVoltage(true);
        if(voltage < config.battMinVoltage) { // check voltage
            // UNTESTED
            keepTrackingRunning = false;
            break;
        }
        else if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
            // UNTESTED
            if(config.trackerMode == MODE_TESTRUN) { printf("imuLightSleepTrackingMode: IMU not triggering\n"); }
            lastErrorId = 152; errorCnt++;
            keepTrackingRunning = false;
            break;
        }
        else {
            // get fifo data (101ms -> NEW: 16ms -> NEW NEW: 14ms)
            uint64_t t = Timing::millis();
            uint16_t currentFifoLen = device.imu.getFIFOLength(); // get size of data in fifo

            uint16_t fifoDataPointerOld = fifoDataPointer;
            imuLightSleepTrackingModeAddData(voltage, currentFifoLen, fifoData, &fifoDataPointer);

            // read acc data into fifoData
            if(currentFifoLen > 0) {
                readFifoIMU(fifoData+fifoDataPointer, currentFifoLen);
                if(currentFifoLen >= 996) { lastErrorId = 153; errorCnt++; } // data loss possible
                fifoDataPointer += currentFifoLen;
                if(config.trackerMode == MODE_TESTRUN) { printf("imuLightSleepTrackingMode: fill RAM %d -> %d (MAX %d), %d fifo bytes in %lldms\n", fifoDataPointerOld, fifoDataPointer, FIFO_DATA_LEN, currentFifoLen, (Timing::millis() - t)); }
            }

            // check if time is over
            bool timeIsUp = false;
            if((esp_timer_get_time() - trackStartUs) / 1000ULL >= burstDurationMilliseconds) {
                if(config.trackerMode == MODE_TESTRUN) { printf("imuLightSleepTrackingMode: STOP, time over, storing now\n"); }
                timeIsUp = true;
            }

            // check if data needs to be stored
            if(((fifoDataPointer + 1024 + imuLightSleepTrackingModeHeaderLength) >= FIFO_DATA_LEN) // no more space in RAM memory after next light sleep
                || timeIsUp) { // time is up
                if(config.useLeds) { device.ledRedOn(); }
                // WARNING: no delay before, SOMETIMES there was a flash writing error!
                if(!device.flashPowerOn(true)) { lastErrorId = 154; errorCnt++; } // turn on flash power already
                
                // store data
                push_data_result_t res = pushDataIntoMemory(fifoData, fifoDataPointer, &memoryFull, (config.trackerMode == MODE_TESTRUN)); // average 25ms
                if(res != PUSH_DATA_SUCCESS) { lastErrorId = 154 + res; errorCnt++; } // 14 error codes (0 = sucess), 155 - 168
                if(!device.flashPowerOff(false)) { lastErrorId = 171; errorCnt++; } // important!
                if(config.useLeds) { device.ledRedOff(); }
                if(config.trackerMode == MODE_TESTRUN) { printf("imuLightSleepTrackingMode: stored RAM: %d bytes in %lldms\n", fifoDataPointer, (Timing::millis() - t)); }
                fifoDataPointer = 0;
                if(memoryFull) {                  
                    keepTrackingRunning = false;
                    if(config.trackerMode == MODE_TESTRUN) { printf("imuLightSleepTrackingMode: memory full\n"); }
                    break;
                }
                else if(timeIsUp) {
                    keepTrackingRunning = false;
                    break;
                }
            }
        }
        if(config.trackerMode == MODE_TESTRUN) { printf("\n"); }
        if(useEnvironmentSensor()) {
            if(!device.baro.performMeasurement()) { lastErrorId = 172; errorCnt++; } // 3ms, trigger baro measurement (takes around 31 ms from here, so do it before entering light sleep)
        }
        device.enableAccInterruptInDeepSleep();
        esp_sleep_enable_timer_wakeup(30000000UL); // in case imu does not answer (30 second timeout)
        device.lightSleep();
    }
    // end of big while loop
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER); 
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1);
    free(fifoData);

    // change clock speed back to 400kHz
    i2c.changeClockSpeed(I2C_FREQ_HZ_400KHZ); // works
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
        .usePPSReference = false,
        .setSYSTime = false, 
        .setRTCTime = true,
        .blinkLeds = true, // always on
        .debug = true };
    get_fix_result_t fixResult = gps.tryToGetFixV2(&gpsData, &gpsConfig, &device);
    if(fixResult == GPS_FIX_SUCCESS_AND_RTC_UPDATED) { printf("GPS SUCCESS: %.6f/%.6f in %ds\n", gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.ttfMilliseconds / 1000); }
    else { printf("GPS FAILED\n"); }
    device.gpioBOff();
}

void gpsTimeServer() {
    uint8_t hundreds = 0;
    bool updatedTime = false;
    uint16_t broadcastDurationSeconds = 60;
    uint8_t data[PROXIMITY_DATA_LEN] = { 0 };
    uint16_t proximityDuration = 1000;
    uint32_t currentTimestamp = 0;
    esp_gps_t gpsData = { };

    // update time
    device.gpioBOn();
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');
    gps_get_fix_config_t gpsConfig = {
        .timeoutSeconds = GPS_TIMEOUT_SECONDS + ((uint32_t) config.gpsFirstFixCollectOrbitDataSeconds), // give it more time to collect orbit data
        .timeoutNotEvenTimeSeconds = GPS_TIMEOUT_NOT_EVEN_TIME_SECONDS,
        .minHDOP = 1.0f, // NOT config.gpsMinHdopTimesTen to definitely collect sufficient orbit data
        .afterFixMaxWaitOnHDOP = config.gpsFirstFixCollectOrbitDataSeconds,
        .waitAfterFixUntilZeroMs = true, // after collecting additional orbit data: wait (if needed) until GPS sends messages at .000 ms
        .usePPSReference = false,
        .setSYSTime = false, 
        .setRTCTime = true,
        .blinkLeds = true, // do blink!
        .debug = true };
    get_fix_result_t fixResult = gps.tryToGetFixV2(&gpsData, &gpsConfig, &device);
    if(fixResult == GPS_FIX_SUCCESS_AND_RTC_UPDATED) {
        hasValidTimestamp = true;
        lastSyncType = SYNC_TYPE_GPS;
        syncCounter++;
        gpsCounterSinceLastRTCSync = 0; // just synchronized
        updatedTime = true;
        if(!device.rtc.getTimestamp(&timestampLastSync, &hundreds)) { lastErrorId = 135; errorCnt++; timestampLastSync = 0; } // update timestampLastSync
    }
    device.gpioBOff();

    // broadcast time
    if(updatedTime) {
        for(uint16_t i=0; i<(broadcastDurationSeconds/2); i++) {
            // wait until next full second
            if(!device.rtc.getTimestamp(&currentTimestamp, &hundreds)) { lastErrorId = 135; errorCnt++; return; }
            uint16_t waitTime = 1000 - (((uint16_t) hundreds)*10);
            ets_delay_us(waitTime * 1000);

            // broadcast time
            if(!device.initESPNOWStationary(config.proximityLongRange, config.proximityDbm, true, (wifi_phy_rate_t) config.proximityDatarate)) { lastErrorId = 19; errorCnt++; return; } // 23ms
            if(!device.addESPNOWBroadcastReceiverStationary()) { lastErrorId = 20; errorCnt++; return; } // 0ms
            esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuous);
            esp_wifi_set_promiscuous(true);
            esp_now_register_recv_cb(proxReceiveCallback); // 0ms

            const esp_timer_create_args_t timerArgs = { .callback = &timerCallback };
            esp_timer_handle_t timer;
            if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { lastErrorId = 21; errorCnt++; return; }
            if(esp_timer_start_once(timer, proximityDuration * 1000) != ESP_OK) { lastErrorId = 22; errorCnt++; return; }

            timeStartProximity = esp_timer_get_time();
            int64_t timeLastSent = timeStartProximity;
            int64_t timeBetweenSending = 0; // immediately start sending
            uint8_t sendCounter = 0;
            uint16_t taskDelayMs;
            if(!device.rtc.getTimestamp(&currentTimestamp, NULL)) { lastErrorId = 131; errorCnt++; } // get time here because timer just started (shouldn't delay stuff)

            while(!timerFinished) {
                if((esp_timer_get_time() - timeLastSent) >= timeBetweenSending) {
                    timeLastSent = esp_timer_get_time(); // reset send counter
                    timeBetweenSending = 100 * 1000; // 100ms until next 
                    fillDataPayload(data, currentTimestamp, PROXIMITY_DATA_LEN); // fill data frame
                    device.broadcastESPNOWData(data, PROXIMITY_DATA_LEN); // spit it out
                    sendCounter++;
                    taskDelayMs = 90;

                    if(sendCounter == 2) { // after 100ms + 0..96ms
                        if(config.useLeds) { device.ledGreenOff(); }
                    }
                    if(sendCounter < (proximityDuration / 100)) { // if NOT last message (= 10 = last message, don't delay here because shorter than 100ms)
                        vTaskDelay(taskDelayMs / portTICK_PERIOD_MS); // will send current cpu to sleep (10ms accuracy, will wake up a cycle before that)
                    }
                }
            }

            // sleep a bit before restarting
            esp_sleep_enable_timer_wakeup(900000UL); // 900 ms
            device.lightSleep();
        }
    }    
}

extern "C" void app_main() {
    while(1) {
        if((config.trackerMode == MODE_TESTRUN) || (config.trackerMode == MODE_PRODUCTIVE)) {
            /** ---------------- BOOT STATE ---------------- */
            if(state == ST_BOOT) { // not doing anything here!
                esp_reset_reason_t resetReason = device.getLastResetReason();
                if(resetReason != ESP_RST_POWERON) {
                    lastErrorId = 173 + resetReason; errorCnt++; // check for brownouts, errorIds 173 - 183 (next free: 184)
                } 
                state = ST_SERIAL_MENUE;
                if(!device.serialMenueGetSelfTestDone(NVS_FLASH_SELFTEST_DONE)) { device.enableInternalTimerInterruptInDeepSleep(1); } // restart system into next state IMMEDIATELY
                else { device.enableInternalTimerInterruptInDeepSleep(BOOT_DELAY_SECONDS); } // restart system into next state
            }
            /** ---------------- SERIAL MENUE STATE ---------------- */
            else if(state == ST_SERIAL_MENUE) {
                if(!device.serialMenue(true, NVS_FLASH_SELFTEST_DONE, NVS_OWN_ID, NVS_FLASH_TAG_ACTIVATED_BY_WIFI, gpsCheckAliveAndMaybeChangeBaudrateWrapper, gpsTestWrapper)) { // WARNING: CPU clocked to 10 MHz
                    state = ST_FIRST_START;
                }
                esp_sleep_enable_timer_wakeup(100000ULL); // 100ms
            }
            /** ---------------- FIRST START STATE ---------------- */
            else if(state == ST_FIRST_START) {
                if(!device.fullRFCalibration()) { lastErrorId = 68; errorCnt++; } // 145 - 160ms

                i2c.begin(I2C_FREQ_HZ_400KHZ);

                // read config
                if(!readConfigFromNVS(&device, &config, true)) { lastErrorId = 4; errorCnt++; }
                uint8_t configErrorId = 0;
                if(!configIsPlausible(&config, &configErrorId)) {
                    lastErrorId = 185; errorCnt++;
                    printf("Reset: ERROR config not plausible (errorId: %d)\n", configErrorId);
                    device.blinkTimes(25, B_RED);
                }
                printConfigurationHash(&config);
                gpsFixHours = config.gpsFixHourBits;

                // disable previous interrupts
                if(!device.rtc.disableTimeUpdateInterruptMinuteChange()) { lastErrorId = 38; errorCnt++; }

                // GPS: check if working
                gps.checkAliveAndMaybeChangeBaudrate(&device, true, config.trackerMode == MODE_TESTRUN);

                // RTC: get current timestamp
                uint32_t timestamp = 0;
                if(!device.rtc.getTimestamp(&timestamp, NULL)) { lastErrorId = 2; errorCnt++; }

                // ENV: check if working
                device.sensorPowerOn();
                device.delay(200);
                environmentSensorConnected = i2c.isAlive(BARO_BME680_ADDRESS);
                device.sensorPowerOff();
                if(!environmentSensorConnected) {
                    printf("Reset: NO ENV SENSOR\n");
                    device.blinkTimes(10, B_RED);
                }
                else { printf("Reset: ENV SENSOR ALIVE\n"); }
                device.delay(200);

                // check NVS if already activated & timestamp is valid
                if(!device.initDataNVS()) { lastErrorId = 3; errorCnt++; }
                uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
                uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_PAGE_POINTER);
                uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
                uint16_t nvsSendBlobPointer = device.nvsReadUINT16(NVS_FLASH_SEND_BLOB_POINTER);
                uint16_t flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_BLOCK_DELETED_POINTER);
                uint32_t blobSize = device.nvsGetBlobSize(NVS_FLASH_BUFFER_NAME);
                freeMemory = device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPageWritePointer, flashPageWriteOffsetPointer);
                printf("Reset: FIRMWARE: V%d, CONFIG: %s V%d\n", WILDFI_SOFTWARE_VERSION, WILDFI_CONFIG_NAME, WILDFI_CONFIG_VERSION);
                printf("Reset: freeSpace: %u, writePointer: %d.%d, sendPointer: %d.%d, sendPointerBlob: %d, blobSize: %d, blockDeletePointer: %d\n", freeMemory, flashPageWritePointer, flashPageWriteOffsetPointer, sendPagePointer, sendPageOffsetPointer, nvsSendBlobPointer, blobSize, flashBlockDeletedPointer);

                // read tag id from nvs
                if(config.tagIdSource == TAG_ID_USE_MAC_LAST_TWO_BYTES) {
                    uint8_t myMac[6] = { 0 };
                    esp_efuse_mac_get_default(myMac);
                    ownTagId = (myMac[4] << 8 ) | myMac[5];
                }
                else if(config.tagIdSource == TAG_ID_USE_VALUE_IN_NVS) {
                    bool neverWritten = false;
                    ownTagId = device.defaultNvsReadUINT16(NVS_OWN_ID, &neverWritten);
                }
                else { ownTagId = 0xFFFF; }

                if(ownTagId == 0) {
                    if(config.trackerMode == MODE_TESTRUN) { printf("Reset: WARNING tag id = 0, using 0xFFFF\n"); }
                    ownTagId = 0xFFFF;
                    lastErrorId = 12; errorCnt++;
                }
                printf("Reset: TAG ID (src %d): 0x%04X\n", config.tagIdSource, ownTagId);

                // read trim registers for mag data compensation
                fillIMUTrimData();

                // read magnetometer offsets from NVS
                bool neverWritten = false;
                magHardIronOffsetX = device.nvsReadINT16(NVS_MAG_CALIB_OFFSET_X, &neverWritten);
                magHardIronOffsetY = device.nvsReadINT16(NVS_MAG_CALIB_OFFSET_Y, &neverWritten);
                magHardIronOffsetZ = device.nvsReadINT16(NVS_MAG_CALIB_OFFSET_Z, &neverWritten);
                printf("Reset: Mag Offsets %d/%d/%d\n", magHardIronOffsetX, magHardIronOffsetY, magHardIronOffsetZ);

                // start with high frequency proximity (as if someone was seen)
                proximityFrequency = config.proximityFrequencyMinuteSeenSomeone;

                // activation state
                if(config.activationMode == ACTIVATION_MODE_SKIP) { isActivated = true; } // store state in RTC -> always activated
                else if(config.activationMode == ACTIVATION_MODE_STORE_PERMANENTLY) {
                    uint16_t activated = device.nvsReadUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI);
                    isActivated = (activated > 0); // store state in RTC variable
                }
                else if(config.activationMode == ACTIVATION_MODE_ON_EVERY_START) { isActivated = false; } // store state in RTC -> not activated after reset

                // time state
                bool error = false;
                bool timeIsValid = device.rtc.timeIsValidNoUndervoltage(error);
                if(error) { lastErrorId = 7; errorCnt++; }
                if((timestamp > 1628689096) && timeIsValid) { hasValidTimestamp = true; }
                else {
                    hasValidTimestamp = false;
                    if(!device.rtc.set(0, 0, 0, 0, 1, 1, 2000))  { lastErrorId = 187; errorCnt++; } // resetting time
                    device.delay(20);
                    if(!device.rtc.getTimestamp(&timestamp, NULL)) { lastErrorId = 188; errorCnt++; } // update timestamp
                }
                if(config.trackerMode == MODE_TESTRUN) {
                    printf("Reset: timestamp %d, timeValid %d -> hasValidTimestamp %d\n", timestamp, timeIsValid, hasValidTimestamp);
                    printf("Reset: lastErrorId: %d, errorCnt: %d\n", lastErrorId, errorCnt);
                }

                if(config.trackerMode == MODE_GPS_TIME_SERVER) { state = ST_GPS_TIME_SERVER; }
                else { state = ST_ACTIVATION; } // go into activation state for quick scan
                esp_sleep_enable_timer_wakeup(100000ULL); // 100ms
            }
            /** ---------------- WAIT FOR ACTIVATION STATE ---------------- */
            else if(state == ST_ACTIVATION) { // custom wake stub not running
                if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // voltage low, stay in that mode but sleep for an hour or more -> needs to be here because otherwise setHourlyInterrupt might not have been set!
                    if(config.trackerMode == MODE_TESTRUN) { printf("Activation: Undervoltage (%d)! Don't scan, sleep long!\n", device.readSupplyVoltageFromWakeStub()); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for an hour or more
                }
                else {
                    if(config.trackerMode == MODE_TESTRUN) { printf("Activation: activated %d, mode %d\n", isActivated, config.activationMode); }
                    if(!device.onlyLoadRFCalibration()) { lastErrorId = 70; errorCnt++; } // 5ms, using RF data in RTC memory
                    device.ledRedOn();
                    nearestGatewayCommand = PROXIMITY_COMMAND_NOTHING;
                    activationDetection(); // sends one message always (tag around) + maybe a message when getting activated
                    device.ledRedOff();
                    device.stopESPNOW(); // 5ms
                    esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep

                    // check commands (PROXIMITY_COMMAND_DEACTIVATE not evaluated!)
                    bool fullResetCommanded = false, resyncTimeCommanded = false, syncTimeOnceCommanded = false;
                    if(nearestGatewayCommand == PROXIMITY_COMMAND_CHANGE_CONFIG) {
                        bool changedSomething = false;
                        if(!checkIfReconfigNeeded(&device, &config, nearestGatewayConfiguration, nearestGatewayCommand, nearestGatewayRssi, &changedSomething, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 184; errorCnt++; }
                        if(changedSomething) { device.blinkTimes(12, B_BOTH); }
                    }
                    else if((nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE) || (nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_06_00) || (nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_12_00) || (nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_15_00) || (nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_20_00)) {
                        if(!isActivated) { // activate in case not already activated (or activation disabled)
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: DONE!\n"); }
                            if(config.activationMode == ACTIVATION_MODE_ON_EVERY_START) { isActivated = true; } // just store in RTC variable, do not store in NVS
                            else {
                                if(!device.initDataNVS()) { lastErrorId = 64; errorCnt++; }
                                else {
                                    if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 1)) { lastErrorId = 65; errorCnt++; } // write activation into NVS
                                    else { isActivated = true; } // now is activated!
                                }
                            }
                        }

                        // NEW: add activation delay
                        if(nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_06_00) { delayAfterActivationAndGotTime = PROXIMITY_COMMAND_ACTIVATE_AT_06_00; }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_12_00) { delayAfterActivationAndGotTime = PROXIMITY_COMMAND_ACTIVATE_AT_12_00; }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_15_00) { delayAfterActivationAndGotTime = PROXIMITY_COMMAND_ACTIVATE_AT_15_00; }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_ACTIVATE_AT_20_00) { delayAfterActivationAndGotTime = PROXIMITY_COMMAND_ACTIVATE_AT_20_00; }
                        else { delayAfterActivationAndGotTime = 0; } // PROXIMITY_COMMAND_ACTIVATE
                    }
                    else if(nearestGatewayCommand == PROXIMITY_COMMAND_FULL_RESET) { fullResetCommanded = true; }
                    else if(nearestGatewayCommand == PROXIMITY_COMMAND_DEACTIVATE) { } // do not do anything
                    else if(nearestGatewayCommand == PROXIMITY_COMMAND_MAG_CALIB) { } // do not do anything (otherwise endless loop!)
                    else if(nearestGatewayCommand == PROXIMITY_COMMAND_RESYNC_TIME_BY_WIFI) { resyncTimeCommanded = true; }
                    else if(nearestGatewayCommand == PROXIMITY_COMMAND_FIRST_SYNC_TIME_IN_ACTIVATION) {
                        if(!hasValidTimestamp) { syncTimeOnceCommanded = true; } // only enter this mode when time not yet synchronized
                        // otherwise command is ignored
                    }
                    
                    // state transition
                    if(fullResetCommanded) {
                        state = ST_FULL_RESET;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else if(resyncTimeCommanded) {
                        // TESTED
                        state = ST_WIFI_SYNC_IN_ACTIVATION;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else if(syncTimeOnceCommanded) { // only when time not yet synchronized
                        // TESTED
                        device.blinkGreenRedAlternating(10);
                        state = ST_GET_TIME_BACK_TO_ACTIVATION;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else {
                        if(isActivated) {
                            if(hasValidTimestamp) {
                                state = ST_START;
                                if(config.trackerMode == MODE_TESTRUN) { printf("Activation: all done, time still valid -> move to START in 1s!\n"); }
                            }
                            else {
                                state = ST_GET_TIME;
                                if(config.trackerMode == MODE_TESTRUN) { printf("Activation: all done but no time -> GET TIME\n"); }
                            }
                            device.enableInternalTimerInterruptInDeepSleep(1); // restart in 1 second and start
                        }
                        else {
                            if(config.trackerMode == MODE_TESTRUN) { printf("Activation: still not activated -> SLEEP\n"); }
                            device.enableInternalTimerInterruptInDeepSleep(config.activationByGatewaySleepSeconds);
                        }
                    }
                }
            }
            /** ---------------- GET TIME STATE ---------------- */
            else if((state == ST_GET_TIME) || (state == ST_GET_TIME_BACK_TO_ACTIVATION)) { // custom wake stub not running
                if(config.trackerMode == MODE_TESTRUN) { printf("Time: try getting time! V = %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) { // voltage low, stay here, but sleep long
                    if(config.trackerMode == MODE_TESTRUN) { printf("Time: undervoltage (%d)! Don't scan!\n", device.readSupplyVoltageFromWakeStub()); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                }
                else {
                    i2c.begin(I2C_FREQ_HZ_400KHZ);
                    if(config.getFirstTimeOverWiFi) { // first check over wifi
                        device.ledGreenOn();
                        if(getTimeOverWiFiNew()) {
                            hasValidTimestamp = true;
                            lastSyncType = SYNC_TYPE_WIFI;
                            syncCounter++;
                            if(!device.rtc.getTimestamp(&timestampLastSync, NULL)) { lastErrorId = 135; errorCnt++; timestampLastSync = 0; } // update timestampLastSync
                        }
                        device.ledGreenOff();
                    }
                    if((!hasValidTimestamp ) && config.getFirstTimeOverGPS) { // if didn't get time over wifi -> try gps
                        device.gpioBOn();
                        device.uart2Init(115200);
                        gps.init(device.uart2GetQueue());
                        device.uart2EnablePatternInterrupt('\n');
                        esp_gps_t gpsData = { };
                        // CHANGED to tryToGetFix to collect orbit data already and to only get time after getting real fix
                        /*if(gps.getTimeOnly(&gpsData, GPS_TIMEOUT_SECONDS, &device, TIME_OVER_GPS_BLINK_LED, (config.trackerMode == MODE_TESTRUN))) {
                            hasValidTimestamp = true;
                        }*/
                        // get time only from GPS, do not store fix
                        gps_get_fix_config_t gpsConfig = {
                            .timeoutSeconds = GPS_TIMEOUT_SECONDS + ((uint32_t) config.gpsFirstFixCollectOrbitDataSeconds), // give it more time to collect orbit data
                            .timeoutNotEvenTimeSeconds = GPS_TIMEOUT_NOT_EVEN_TIME_SECONDS,
                            .minHDOP = 1.0f, // NOT config.gpsMinHdopTimesTen to definitely collect sufficient orbit data
                            .afterFixMaxWaitOnHDOP = config.gpsFirstFixCollectOrbitDataSeconds,
                            .waitAfterFixUntilZeroMs = true, // after collecting additional orbit data: wait (if needed) until GPS sends messages at .000 ms
                            .usePPSReference = false,
                            .setSYSTime = false, 
                            .setRTCTime = true,
                            .blinkLeds = true, // at very first fix: do blink!
                            .debug = (config.trackerMode == MODE_TESTRUN) };
                        get_fix_result_t fixResult = gps.tryToGetFixV2(&gpsData, &gpsConfig, &device);
                        if(fixResult == GPS_FIX_SUCCESS_AND_RTC_UPDATED) {
                            hasValidTimestamp = true;
                            lastSyncType = SYNC_TYPE_GPS;
                            syncCounter++;
                            gpsCounterSinceLastRTCSync = 0; // just synchronized
                            if(!device.rtc.getTimestamp(&timestampLastSync, NULL)) { lastErrorId = 135; errorCnt++; timestampLastSync = 0; } // update timestampLastSync
                        }
                        device.gpioBOff();
                    }
                    // evaluate result
                    if(state == ST_GET_TIME_BACK_TO_ACTIVATION) { // FORCED GET TIME: go back to activation immediately (also if got time was not successful -> might loop back to that state) 
                        // TESTED
                        if(config.trackerMode == MODE_TESTRUN) { printf("Time: back to activation! Got timestamp: %d\n", hasValidTimestamp); }
                        state = ST_ACTIVATION;
                        device.enableInternalTimerInterruptInDeepSleep(15); // add a bit of delay
                    }
                    else { // NORMAL CASE: normal activation
                        if(hasValidTimestamp) {
                            state = ST_START;
                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: all done -> START!\n"); }
                            device.blinkTimes(6, B_GREEN);
                            // NEW: add delay after activation
                            if(delayAfterActivationAndGotTime == PROXIMITY_COMMAND_ACTIVATE_AT_06_00) {
                                if(!device.rtc.setDailyInterrupt(6, 0)) { lastErrorId = 190; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                                device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
                            }
                            else if(delayAfterActivationAndGotTime == PROXIMITY_COMMAND_ACTIVATE_AT_12_00) {
                                if(!device.rtc.setDailyInterrupt(12, 0)) { lastErrorId = 190; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                                device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
                            }
                            else if(delayAfterActivationAndGotTime == PROXIMITY_COMMAND_ACTIVATE_AT_15_00) {
                                if(!device.rtc.setDailyInterrupt(15, 0)) { lastErrorId = 190; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                                device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
                            }
                            else if(delayAfterActivationAndGotTime == PROXIMITY_COMMAND_ACTIVATE_AT_20_00) {
                                if(!device.rtc.setDailyInterrupt(20, 0)) { lastErrorId = 190; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                                device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
                            }
                            else {
                                device.enableInternalTimerInterruptInDeepSleep(1); // restart in 1 second and move to start
                            }
                        }
                        else {
                            uint16_t sleepTimeNextTry = config.timeBetweenGetTimeRetriesSeconds;
                            getTimeCounter++;
                            if(getTimeCounter < 3) { sleepTimeNextTry = FIRST_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS; } // first 3 times: try very frequently to get time
                            if(config.trackerMode == MODE_TESTRUN) { printf("Time: failed -> sleep %ds (tried %d times)!\n", sleepTimeNextTry, getTimeCounter); }
                            device.enableInternalTimerInterruptInDeepSleep(sleepTimeNextTry);
                        }
                    }
                }
            }
            /** ---------------- GPS TIME SERVER STATE ---------------- */
            else if(state == ST_GPS_TIME_SERVER) {
                uint16_t voltage = device.readSupplyVoltage(true);
                if(voltage < config.battMinVoltage) { // check voltage
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep long
                }
                else {
                    gpsTimeServer();
                    device.enableInternalTimerInterruptInDeepSleep(5); // sleep a bit before restarting server
                }
            }
            /** ---------------- START STATE ---------------- */
            else if(state == ST_START) {
                i2c.begin(I2C_FREQ_HZ_400KHZ);

                // NEW: reset daily interrupt
                if(delayAfterActivationAndGotTime != 0) {
                    // TESTED with 3 pm interrupt
                    delayAfterActivationAndGotTime = 0;
                    if(!device.rtc.resetInterruptFlags()) { lastErrorId = 191; errorCnt++; } // IMPORTANT: does not reset by itself!
                    if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 192; errorCnt++; } // IMPORTANT: otherwise daily interrupt triggers again in tracking and stays on (not resetting itself)
                }

                if(device.rtc.setTimeUpdateInterruptMinuteChange()) {
                    bool error = false;
                    currentMinute = device.rtc.getMinutes(error); // update current minute (do not call updateHourAndMinuteForWakeStub, because first time)
                    currentHour = device.rtc.getHours(error); // update current hour (do not call updateHourAndMinuteForWakeStub, because first time)
                    if(!error) {
                        device.blinkTimes(5, B_BOTH);
                        if(config.trackerMode == MODE_TESTRUN) { printf("Time: %d:%02d\n\n", currentHour, currentMinute); }
                        state = ST_TRACK;
                        device.customWakeStubFunction(wakeStub);
                        device.setWakeStubRejectionInterruptSrc(USE_EXT0_IF_WAKE_UP_REJECTED);
                        device.enableRTCInterruptInDeepSleep();
                    }
                    else { lastErrorId = 134; errorCnt++; device.enableInternalTimerInterruptInDeepSleep(120); }
                }
                else { lastErrorId = 6; errorCnt++; device.enableInternalTimerInterruptInDeepSleep(120); }
            }
			/** ---------------- TRANSMIT DATA DURING NIGHTTIME STATE ---------------- */
            else if(state == ST_NIGHTTIME_DATATRANS) {
				// UNTESTED
				if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) {
                    if(config.trackerMode == MODE_TESTRUN) { printf("Nighttime: POWER DOWN for %ds\n", FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
                    state = ST_PWRDWN;
                    device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                    // WARNING: RTC interrupts still running! not disabled, but shouldn't be a problem
                }
				else {
                    // ---------------- PROXIMITY DETECTION ----------------
                    uint16_t proximityDuration = 180; // shorter duration (gateway msgs every 80 ms -> 180 ms should be enough to see two gateway msgs)

                    i2c.begin(I2C_FREQ_HZ_400KHZ); // needed for current minute update and imu
                    if(!device.onlyLoadRFCalibration()) { lastErrorId = 69; errorCnt++; } // 5ms, using RF data in RTC memory
                    if(config.useLeds) { device.ledGreenOn(); } // will be turned off in proximity detection
                    if(!device.flashPowerOn(false)) { lastErrorId = 11; errorCnt++; } // no delay needed because proximity detection first

                    scanStop = false;
                    uint32_t timestamp = 0; // used for storing data!
                    bool proximitySuccess = true;
                    proximitySuccess = proximityDetection(&timestamp, proximityDuration); // perform proximity detection and return timestamp
                    scanStop = true; // do not store more incoming messages in scanResults (ESP NOW still running)
                    bool dataGatewaySeenAndTimeToSend = isAnyDataGatewayInScanResults(); // check if any gateway is in the scan results
                    bool memoryFull = false;
					proxDetectionsSinceLastFullRFCalib++; // also increment when proximityDuration = 0 -> so that RF gets re-calibrated from time to time

                    if(!proximitySuccess) { // some fatal error
                        device.stopESPNOW(); // 5ms
                        esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                        if(config.trackerMode == MODE_TESTRUN) { printf("STATE: FATAL!\n"); }              
                    }
                    else {
                        if(!dataGatewaySeenAndTimeToSend) { // no gateway seen or not the time to send data - stop ESP NOW and proceed with storing the data
                            device.stopESPNOW(); // 5ms
                            esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                        }
                        
                        if(!device.initDataNVS()) { lastErrorId = 8; errorCnt++; } // 4ms
                        if(config.trackerMode == MODE_TESTRUN) { printScanResult(); }

                        // ---------------- TRANSMIT DATA TO GATEWAY (IF SEEN) ----------------
						if(dataGatewaySeenAndTimeToSend) { // NVS initialized
							dataTransmissionToGateway(); // needs flash, nvs and RF
							device.stopESPNOW(); // 5ms
							esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
						}
                    }
                    if(!device.flashPowerOff(false)) { lastErrorId = 10; errorCnt++; } // IMPORTANT: turn off here

                    // ---------------- TIME UPDATE / CORRECTION RTC VARIABLES ----------------

                    updateHourAndMinuteForWakeStub(); // data transmission or IMU burst might took long time

                    // ---------------- CHECK COMMANDS ----------------

                    bool fullResetCommanded = false, deactivationCommanded = false, magCalibCommanded = false, resyncTimeCommanded = false;
                    if(proximityDuration > 0) {
                        if(nearestGatewayCommand == PROXIMITY_COMMAND_CHANGE_CONFIG) {
                            bool changedSomething = false;
                            if(!checkIfReconfigNeeded(&device, &config, nearestGatewayConfiguration, nearestGatewayCommand, nearestGatewayRssi, &changedSomething, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 184; errorCnt++; }
                            if(changedSomething) { device.blinkTimes(12, B_BOTH); }
                        }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_DEACTIVATE) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("-> DEACTIVATION COMMANED <-\n"); }
                            deactivationCommanded = true;
                        }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_FULL_RESET) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("-> FULL RESET COMMANED <-\n"); }
                            fullResetCommanded = true;
                        }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_MAG_CALIB) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("-> MAG CALIBRATION COMMANDED <-\n"); }
                            magCalibCommanded = true;                        
                        }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_RESYNC_TIME_BY_WIFI) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("-> RE-SYNC COMMANDED <-\n"); }
                            resyncTimeCommanded = true;                        
                        }
                    }

                    // ---------------- DEBUG OUTPUT ----------------

                    if(config.trackerMode == MODE_TESTRUN) { printf("Nighttime: %d:%02d, frequency: %d, tsLastSync: %d (type: %d, cnt: %d), tsLastSeenSomeone: %d\n", currentHour, currentMinute, proximityFrequency, timestampLastSync, lastSyncType, syncCounter, timestampLastSeenSomeone); }
                    if(config.trackerMode == MODE_TESTRUN) { printf("Nighttime: lastErrorId: %d, errorCnt: %d\n", lastErrorId, errorCnt); }
                    if(config.trackerMode == MODE_TESTRUN) { printf("(%dms)\n\n", ((uint32_t) Timing::millis())); }

                    // ---------------- STATE TRANSITION ----------------
                    
                    if(fullResetCommanded) {
                        state = ST_FULL_RESET;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else if(deactivationCommanded) { // dataNVS initialized always, because command can only be received if proximitySuccess = true
                        state = ST_ACTIVATION; // should be fine, as activation state is entered always, even when skipped, and isActivated is normally set in state before
                        isActivated = false; // revoke activation -> gateway nearby required
                        if(config.activationMode == ACTIVATION_MODE_STORE_PERMANENTLY) {
                            if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 0)) { lastErrorId = 186; errorCnt++; } // reset activation
                        }
                        device.setCPUSpeed(ESP32_10MHZ);
                        device.blinkTimes(10, B_RED);
                        device.enableInternalTimerInterruptInDeepSleep(15);
                    }
                    else if(magCalibCommanded) {
                        state = ST_MAG_CALIBRATION;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else if(resyncTimeCommanded) {
                        state = ST_WIFI_SYNC;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
					// auto-resyncs should not be done during night time!
                    /*else if(WIFI_DAILY_RESYNC_ENABLED && (currentHour == WIFI_DAILY_RESYNC_HOUR) && (currentMinute == WIFI_DAILY_RESYNC_MINUTE)) {
                        state = ST_WIFI_SYNC;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }*/
                    else {
                        if(proxDetectionsSinceLastFullRFCalib >= PROXIMITY_FULL_RF_CALIB_EVERY) {
                            proxDetectionsSinceLastFullRFCalib = 0;
                            state = ST_RF_FULL_CALIB;
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        else {
							state = ST_TRACK; // IMPORTANT: state transition back to normal, otherwise wake stub not running
                            device.enableRTCInterruptInDeepSleep(); // normal wake-up on RTC
                        }
                    }					
				}
			}
            /** ---------------- TRACKING STATE ---------------- */
            else if(state == ST_TRACK) {
                if(device.readSupplyVoltageFromWakeStub() < config.battMinVoltage) {
                    if(config.trackerMode == MODE_TESTRUN) { printf("State: POWER DOWN for %ds\n", FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
                    state = ST_PWRDWN;
                    device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                    // WARNING: RTC interrupts still running! not disabled, but shouldn't be a problem
                }		
                else {
                    // ---------------- PROXIMITY DETECTION ----------------
                    uint16_t proximityDuration = config.proximityListeningIntervalMs;
                    if(DAILY_LONGER_PROXIMITY_ENABLED) { // NEW: longer proximity detection once per day
                        if((currentHour == DAILY_LONGER_PROXIMITY_HOUR) && (currentMinute == DAILY_LONGER_PROXIMITY_MINUTE)) {
                            proximityDuration = DAILY_LONGER_PROXIMITY_DURATION_MS;
                        }
                    }
                    if(PROXIMITY_LISTENING_INTERVAL_PROLONGED_ENABLED) {
                        if(proximityDuration < 1000) { // NEW: if duration is less than 1 second: every X th time make it longer
                            if((proximityDetectionCnt % PROXIMITY_LISTENING_INTERVAL_1S_EVERY_X_TIMES) == 0) { proximityDuration = 1000; }
                        }
                    }

                    i2c.begin(I2C_FREQ_HZ_400KHZ); // needed for current minute update and imu
                    if(proximityDuration > 0) {
                        if(!device.onlyLoadRFCalibration()) { lastErrorId = 69; errorCnt++; } // 5ms, using RF data in RTC memory
                    }
                    if(config.useLeds) { device.ledGreenOn(); } // will be turned off in proximity detection
                    if(((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) || useEnvironmentSensor()) { device.sensorPowerOn(); } // turn on power, will be configured during proximity detection 
                    if(!device.flashPowerOn(false)) { lastErrorId = 11; errorCnt++; } // no delay needed because proximity detection first

                    scanStop = false;
                    uint32_t timestamp = 0; // used for storing data!
                    bool proximitySuccess = true;
                    if(proximityDuration == 0) { proximityDetectionZeroMs(&timestamp); } // special mode when duration is set to 0, only sleep for 1 second
                    else { proximitySuccess = proximityDetection(&timestamp, proximityDuration); } // perform proximity detection and return timestamp
                    scanStop = true; // do not store more incoming messages in scanResults (ESP NOW still running)
                    bool dataGatewaySeenAndTimeToSend = isAnyDataGatewayInScanResults(); // check if any gateway is in the scan results
                    bool memoryFull = false;
                    bool keepIMUOn = ((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) && (config.imuBurstMillis > 0); // IMU stays on when burst shall be recorded afterwards
                    proxDetectionsSinceLastFullRFCalib++; // also increment when proximityDuration = 0 -> so that RF gets re-calibrated from time to time

                    // NEW: send data to gateway only if modulo of tag id equals to proximity counter
                    // IMPORTANT: Will not have an effect on GPS being turned off when a gateway is around!
                    // IMPORTANT: proximityDetectionCnt is not synced between tags, means no super equal distribution of sending windows, but simple solution when a lot of tags transmit data in parallel
                    if(DATA_TRANSMISSION_MODULO_MODE) {
                        uint16_t sendingWindow = (ownTagId % DATA_TRANSMISSION_MODULO); // define tag individual sending window
                        uint16_t currentWindow = (uint16_t) (proximityDetectionCnt % DATA_TRANSMISSION_MODULO); // get current window of proximity detection
                        if(proximityDetectionCnt > 1) { // first proximity detection: ALWAYS try to send data because brownout could have happened
                            if(config.trackerMode == MODE_TESTRUN) { printf("State: transmitWindowCorrect: %d = %d?\n", currentWindow, sendingWindow); }
                            if(sendingWindow != currentWindow) {
                                if(config.trackerMode == MODE_TESTRUN) { printf("State: transmitWindowCorrect: NO!\n"); } 
                                dataGatewaySeenAndTimeToSend = false; // if it is not the time slot for this tag to send data just pretend that no gateway is around!
                            }
                        }
                    }

                    if(!proximitySuccess) { // some fatal error
                        if(proximityDuration > 0) {
                            device.stopESPNOW(); // 5ms
                            esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                        }
                        if((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) { 
                            if(!keepIMUOn) { stopIMU(); } // IMU stays on in case of proximity error, to continue recording
                        }
                        if(((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) || useEnvironmentSensor()) {
                            if(!keepIMUOn) { device.sensorPowerOff(); } // IMU stays on in case of proximity error, to continue recording
                        }
                        if(config.trackerMode == MODE_TESTRUN) { printf("STATE: FATAL!\n"); }              
                    }
                    else {
                        if(!dataGatewaySeenAndTimeToSend) { // no gateway seen or not the time to send data - stop ESP NOW and proceed with storing the data
                            if(proximityDuration > 0) {
                                device.stopESPNOW(); // 5ms
                                esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                            }
                        }
                        
                        if(proximityDuration > 0) { checkForHugeTimeDifferences(); } // will just set an error
                        if(!device.initDataNVS()) { lastErrorId = 8; errorCnt++; } // 4ms
                        if(config.trackerMode == MODE_TESTRUN) { printScanResult(); }

                        // ---------------- READ ENVIRONMENT DATA ----------------
                        int16_t temperature = 0;
                        uint32_t pressure = 0;
                        uint32_t humidity = 0;
                        int16_t hall = 0;
                        if(useEnvironmentSensor()) {
                            if(device.baro.getResults()) { // normally waits up to 31ms, but measurement was triggered before during proximity detection
                                bool error = false;
                                temperature = device.baro.getTemperature(error);
                                if(error) { lastErrorId = 67; errorCnt++; }
                                pressure = device.baro.getPressure(error);
                                if(error) { lastErrorId = 132; errorCnt++; }
                                humidity = device.baro.getHumidity(error);
                                if(error) { lastErrorId = 133; errorCnt++; }
                            }
                            // NEW:
                            /*int32_t hallTemp = device.readHallSensor(8);
                            if(hallTemp >= 32767) { hallTemp = 32767; }
                            if(hallTemp <= -32768) { hallTemp = -32768; }
                            hall = (int16_t) hallTemp;*/
                            if(config.trackerMode == MODE_TESTRUN) { printf("ENV: %d deg, %d press, %d hum, %d hall\n", temperature, pressure, humidity, hall); }
                        }

                        // ---------------- READ IMU DATA ----------------
                        uint8_t fifoData[1024];
                        uint16_t currentFifoLen = 0;
                        if((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) {
                            /*if(IMU_GET_FULL_BURSTS_CNT > 0) {
                                if(!dataGatewaySeenAndTimeToSend) { // WARNING: only possible when no gateway seen (no data transmission), otherwise RF is still on and can't use light sleep
                                    if(!device.imu.enableFIFOInterrupt(IMU_BURST_FIFO_SIZE)) { lastErrorId = 130; errorCnt++; }
                                    device.enableAccInterruptInDeepSleep();
                                    device.lightSleep(); // sleep until FIFO full, WARNING: STOPS ESPNOW!
                                    device.disableAccInterruptInDeepSleep();
                                }
                            }*/
                            currentFifoLen = device.imu.getFIFOLength();
                            readFifoIMU(fifoData, currentFifoLen);
                            if(config.trackerMode == MODE_TESTRUN) { printf("IMU: %d byte in fifo!\n", currentFifoLen); }
                            if(!keepIMUOn) { stopIMU(); } // IMU stays on in case of burst
                        }
                        if(((config.imuMode == IMU_ACC_ONLY) || (config.imuMode == IMU_ACC_MAG_GYRO)) || useEnvironmentSensor()) {
                            if(!keepIMUOn) { device.sensorPowerOff(); } // IMU stays on in case of burst
                        }

                        // ---------------- TRANSMIT DATA TO GATEWAY (IF SEEN) ----------------
                        if(proximityDuration > 0) {
                            if(dataGatewaySeenAndTimeToSend) { // NVS initialized
                                dataTransmissionToGateway(); // needs flash, nvs and RF
                                device.stopESPNOW(); // 5ms
                                esp_phy_rf_deinit(PHY_WIFI_MODULE); // otherwise +1.4mA in deep sleep
                            }
                        }

                        // ---------------- STORE DATA ----------------
                        storeProximityDataInFlash(timestamp, config.imuMode, fifoData, currentFifoLen, useEnvironmentSensor(), temperature, pressure, humidity, &memoryFull);
                        //device.flash.printFlash(0, 1, 512); // TEMPORARY
                        if(memoryFull) { // memory full = still continue operation, ignore that data is not stored, still participate in proximity detection
                            // if memory full here: one dataset could not be stored
                            if(config.trackerMode == MODE_TESTRUN) { printf("Flash: memory full!\n"); }
                            if(config.freeMemoryIfFull) {
                                if(checkIfFlashMemoryCanBeFreed()) { // if could delete anything
                                    memoryFull = false;
                                    if(config.trackerMode == MODE_TESTRUN) { printf("Flash: could delete some blocks!\n"); }
                                }
                            }
                        }

                        // update timestampLastSeenSomeone
                        if(scanResultsPointer > 0) { timestampLastSeenSomeone = timestamp; } // actually seen someone in results -> update counter
                        uint32_t timeNobodySeenSeconds = timestamp - timestampLastSeenSomeone; // just to see when last time seen someone
                        if((timestampLastSeenSomeone > 0) && (timeNobodySeenSeconds > (3600*24)) && (lastErrorId != 40)) { lastErrorId = 40; errorCnt++; } // haven't seen anyone for 24 hours -> add error ONCE

                        // change proximity frequency if seen someone
                        if(scanResultsPointer > 0) { proximityFrequency = config.proximityFrequencyMinuteSeenSomeone; }
                        else { proximityFrequency = config.proximityFrequencyMinute; }
                    }
                    if(!device.flashPowerOff(false)) { lastErrorId = 10; errorCnt++; } // IMPORTANT: turn off here

                    // ---------------- IMU LIGHT SLEEP TRACKING ----------------

                    if(keepIMUOn) { // imu still on here
                        if(!memoryFull) { // only if memory is not full
                            uint16_t currentFifoLen = device.imu.getFIFOLength();
                            if(currentFifoLen >= 996) { 
                                if(config.trackerMode == MODE_TESTRUN) { printf("STATE: IMU is full (data transmission too long?) -> reset!\n"); }
                                if(!device.imu.resetFIFO()) { lastErrorId = 142; errorCnt++; }
                            }
                            imuLightSleepTrackingMode(useEnvironmentSensor(), config.imuBurstMillis);
                            stopIMU();
                            device.sensorPowerOff();
                        }
                    }

                    // ---------------- TIME UPDATE / CORRECTION RTC VARIABLES ----------------

                    updateHourAndMinuteForWakeStub(); // data transmission or IMU burst might took long time

                    // ---------------- GPS ATTEMPT ----------------

                    bool justSynchronizedRTCViaGPS = false; // do not re-do RTC sync between tags after GPS synchronization
                    gpsRandomizer(); // will re-randomize GPS fix attempts when new day started

                    // NEW: fix can be prevented when gateway was seen
                    bool fixPreventedBecauseGatewayAround = false;
                    if(GPS_ONLY_TAKE_FIX_WHEN_NO_GATEWAYS_SEEN) {
                        if(isAnyDataOrNoDataGatewayInScanResults()) { 
                            fixPreventedBecauseGatewayAround = true;
                            if(config.trackerMode == MODE_TESTRUN) { printf("GPS: fix prevented because gateway(s) around\n"); }
                        }
                    }

                    // NEW: fix can be prevented when voltage too low
                    bool fixPreventedDueToLowVoltage = false;
                    if(device.readSupplyVoltageFromWakeStub() < GPS_MINIMUM_VOLTAGE) {
                        fixPreventedDueToLowVoltage = true;
                        if(config.trackerMode == MODE_TESTRUN) { printf("GPS: fix prevented because voltage %d < %d\n", device.readSupplyVoltageFromWakeStub(), GPS_MINIMUM_VOLTAGE); }
                    }

                    // check if GPS fix should be taken
                    if(((currentMinute == 0) || config.gpsForcedAfterEveryProximity) // only at full hours or when forced
                        && hourEqualsGPSTrySetting(currentHour) // only when hour is in list
                        && (!memoryFull) // only when memory is not full (doesn't make sense otherwise)
                        && (!fixPreventedBecauseGatewayAround)
                        && (!fixPreventedDueToLowVoltage)) {
                        if(config.trackerMode == MODE_TESTRUN) { printf("GPS: start %d:%d (force: %d, gpsCntSinceLastSync: %d (max %d))\n", currentHour, currentMinute, config.gpsForcedAfterEveryProximity, gpsCounterSinceLastRTCSync, config.gpsSyncRTCFrequency); }
                        device.gpioBOn();
                        device.uart2Init(115200);
                        gps.init(device.uart2GetQueue());
                        device.uart2EnablePatternInterrupt('\n');
                        esp_gps_t gpsData = { };
                        float minHdopFloat = config.gpsMinHdopTimesTen;
                        minHdopFloat = minHdopFloat / 10.0f;
                        bool doRTCSync = false;
                        if(gpsCounterSinceLastRTCSync >= ((uint32_t) config.gpsSyncRTCFrequency)) { doRTCSync = true; }
                        gps_get_fix_config_t gpsConfig = {
                            .timeoutSeconds = GPS_TIMEOUT_SECONDS,
                            .timeoutNotEvenTimeSeconds = GPS_TIMEOUT_NOT_EVEN_TIME_SECONDS,
                            .minHDOP = minHdopFloat,
                            .afterFixMaxWaitOnHDOP = GPS_AFTER_FIX_MAX_WAIT_ON_HDOP,
                            .waitAfterFixUntilZeroMs = false,
                            .usePPSReference = false,
                            .setSYSTime = false, 
                            .setRTCTime = true,
                            .blinkLeds = config.useLeds,
                            .debug = (config.trackerMode == MODE_TESTRUN) };
                        if(doRTCSync) { // modify GPS settings
                            //gpsConfig.timeoutSeconds = GPS_TIMEOUT_SECONDS;
                            //gpsConfig.timeoutNotEvenTimeSeconds = GPS_TIMEOUT_NOT_EVEN_TIME_SECONDS;
                            //gpsConfig.minHDOP = minHdopFloat;
                            //gpsConfig.afterFixMaxWaitOnHDOP = GPS_AFTER_FIX_MAX_WAIT_ON_HDOP;
                            gpsConfig.waitAfterFixUntilZeroMs = true; // CHANGED: additionally wait until getting time at .000 ms
                            gpsConfig.setRTCTime = true; // CHANGED: set RTC time
                            //gpsConfig.blinkLeds = config.useLeds;
                            //gpsConfig.debug = (config.trackerMode == MODE_TESTRUN);
                            if(config.trackerMode == MODE_TESTRUN) { printf("GPS: do RTC re-sync!\n"); }
                        }
                        get_fix_result_t fixResult = gps.tryToGetFixV2(&gpsData, &gpsConfig, &device);
                        device.gpioBOff();
                        gpsCounterSinceLastRTCSync++; // tried one more time

                        if((fixResult == GPS_FIX_SUCCESS_AND_RTC_UPDATED) || (fixResult == GPS_FIX_SUCCESS_NO_RTC_UPDATE)) { // got a fix
                            if(config.trackerMode == MODE_TESTRUN) { printf("GPS: SUCCESS: Timestamp: %u, TTFF: %d, LAT: %f, LON: %f, HDOP: %f\n", gpsData.parent.utcTimestamp, gpsData.parent.ttfMilliseconds, gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.dop_h); }
                            if(!device.flashPowerOn(false)) { lastErrorId = 11; errorCnt++; } // flash power on (again) for storing
                            if(!device.rtc.getTimestamp(&timestamp, NULL)) { timestamp = 0; lastErrorId = 15; errorCnt++; } // update current timestamp to store on flash
                            
                            if(fixResult == GPS_FIX_SUCCESS_AND_RTC_UPDATED) { // freshly synchronized, update all sync variables
                                timestampLastSync = timestamp;
                                lastSyncType = SYNC_TYPE_GPS;
                                syncCounter++;
                                gpsCounterSinceLastRTCSync = 0;
                                justSynchronizedRTCViaGPS = true;
                            }
                            if(doRTCSync && (fixResult == GPS_FIX_SUCCESS_NO_RTC_UPDATE)) { // RTC should have been updated
                                lastErrorId = 72 + fixResult; errorCnt++; // buffer for next error id: 80
                                if(config.trackerMode == MODE_TESTRUN) { printf("GPS: failed to update RTC!\n"); }
                            }

                            storeGPSDataInFlash(timestamp, &gpsData, &memoryFull); 
                            //device.flash.printFlash(0, 1, 512); // DEBUG
                            if(memoryFull) { // memory full = still continue operation, ignore that data is not stored
                                if(config.trackerMode == MODE_TESTRUN) { printf("Flash: memory full after GPS!\n"); }
                                if(config.freeMemoryIfFull) { checkIfFlashMemoryCanBeFreed(); } // do not reset memory full variable, as nothing happens afterwards
                            }
                            if(!device.flashPowerOff(false)) { lastErrorId = 10; errorCnt++; }
                        }
                        else {
                            lastErrorId = 72 + fixResult; errorCnt++; // buffer for next error id: 80
                            if(config.trackerMode == MODE_TESTRUN) { printf("GPS: failed %d!\n", fixResult); }
                        }

                        // update time again
                        updateHourAndMinuteForWakeStub(); // GPS might took long time
                    }

                    // ---------------- PERFORM TIME CORRECTION BETWEEN TAGS IF NEEDED ----------------

                    if(proximityDuration > 0) {
                        if(config.timeCorrectionBetweenTags && (!justSynchronizedRTCViaGPS)) { checkIfTimeCorrectionNeeded(); } // WARNING: clock down to 10MHz, do not synchronize again if GPS was used for syncing
                    }

                    // ---------------- CHECK COMMANDS ----------------

                    bool fullResetCommanded = false, deactivationCommanded = false, magCalibCommanded = false, resyncTimeCommanded = false;
                    if(proximityDuration > 0) {
                        if(nearestGatewayCommand == PROXIMITY_COMMAND_CHANGE_CONFIG) {
                            bool changedSomething = false;
                            if(!checkIfReconfigNeeded(&device, &config, nearestGatewayConfiguration, nearestGatewayCommand, nearestGatewayRssi, &changedSomething, (config.trackerMode == MODE_TESTRUN))) { lastErrorId = 184; errorCnt++; }
                            if(changedSomething) { device.blinkTimes(12, B_BOTH); }
                        }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_DEACTIVATE) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("-> DEACTIVATION COMMANED <-\n"); }
                            deactivationCommanded = true;
                        }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_FULL_RESET) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("-> FULL RESET COMMANED <-\n"); }
                            fullResetCommanded = true;
                        }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_MAG_CALIB) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("-> MAG CALIBRATION COMMANDED <-\n"); }
                            magCalibCommanded = true;                        
                        }
                        else if(nearestGatewayCommand == PROXIMITY_COMMAND_RESYNC_TIME_BY_WIFI) {
                            if(config.trackerMode == MODE_TESTRUN) { printf("-> RE-SYNC COMMANDED <-\n"); }
                            resyncTimeCommanded = true;                        
                        }
                    }

                    // ---------------- DEBUG OUTPUT ----------------

                    if(config.trackerMode == MODE_TESTRUN) { printf("State: %d:%02d, frequency: %d, tsLastSync: %d (type: %d, cnt: %d), tsLastSeenSomeone: %d\n", currentHour, currentMinute, proximityFrequency, timestampLastSync, lastSyncType, syncCounter, timestampLastSeenSomeone); }
                    if(config.trackerMode == MODE_TESTRUN) { printf("State: lastErrorId: %d, errorCnt: %d\n", lastErrorId, errorCnt); }
                    if(config.trackerMode == MODE_TESTRUN) { printf("(%dms)\n\n", ((uint32_t) Timing::millis())); }

                    // ---------------- STATE TRANSITION ----------------
                    
                    if(fullResetCommanded) {
                        state = ST_FULL_RESET;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else if(deactivationCommanded) { // dataNVS initialized always, because command can only be received if proximitySuccess = true
                        state = ST_ACTIVATION; // should be fine, as activation state is entered always, even when skipped, and isActivated is normally set in state before
                        isActivated = false; // revoke activation -> gateway nearby required
                        if(config.activationMode == ACTIVATION_MODE_STORE_PERMANENTLY) {
                            if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 0)) { lastErrorId = 186; errorCnt++; } // reset activation
                        }
                        device.setCPUSpeed(ESP32_10MHZ);
                        device.blinkTimes(10, B_RED);
                        device.enableInternalTimerInterruptInDeepSleep(15);
                    }
                    else if(magCalibCommanded) {
                        state = ST_MAG_CALIBRATION;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else if(resyncTimeCommanded) {
                        state = ST_WIFI_SYNC;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else if(WIFI_DAILY_RESYNC_ENABLED && (currentHour == WIFI_DAILY_RESYNC_HOUR) && (currentMinute == WIFI_DAILY_RESYNC_MINUTE)) {
                        state = ST_WIFI_SYNC;
                        device.enableInternalTimerInterruptInDeepSleep(1);
                    }
                    else {
                        if(proxDetectionsSinceLastFullRFCalib >= PROXIMITY_FULL_RF_CALIB_EVERY) {
                            proxDetectionsSinceLastFullRFCalib = 0;
                            state = ST_RF_FULL_CALIB;
                            device.enableInternalTimerInterruptInDeepSleep(1);
                        }
                        else {
                            device.enableRTCInterruptInDeepSleep(); // normal wake-up on RTC
                        }
                    }
                }
            }
            /** ---------------- POWER DOWN STATE ---------------- */
            else if(state == ST_PWRDWN) { // WARNING: RTC interrupts still running! not disabled
                if(config.trackerMode == MODE_TESTRUN) { printf("(PWR_DWN) V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() <= config.battRestartVoltage) { // voltage still too low for restarting
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time 
                }
                else { // voltage okay again, restart into tracking state
                    if(config.trackerMode == MODE_TESTRUN) { printf("RESTART!\n"); }
                    state = ST_TRACK;
                    i2c.begin(I2C_FREQ_HZ_400KHZ); // needed for current minute update
                    updateHourAndMinuteForWakeStub(); // update time here because might have slept for long time! remember that RTC interrupts are still activated (but ESP32 does not react on it)
                    if(config.trackerMode == MODE_TESTRUN) { printf("State: %d:%02d (tsLastSync: %d)\n", currentHour, currentMinute, timestampLastSync); }
                    device.enableRTCInterruptInDeepSleep();
                }
            }
            /** ---------------- FULL RF CALIBRATION STATE ---------------- */
            else if(state == ST_RF_FULL_CALIB) { // WARNING: RTC interrupts still running! not disabled
                if(!device.fullRFCalibration()) { lastErrorId = 71; errorCnt++; }  // 145 - 160ms  
                if(config.trackerMode == MODE_TESTRUN) { printf("Full calib state: done\n"); }
                state = ST_TRACK; // back to track

                // update minute / hours, very unlikely changing here, but just to be sure
                i2c.begin(I2C_FREQ_HZ_400KHZ); // needed for current minute update and imu
                updateHourAndMinuteForWakeStub();

                device.enableRTCInterruptInDeepSleep();
            }
            /** ---------------- WIFI SYNC STATE ---------------- */
            else if(state == ST_WIFI_SYNC) { // WARNING: RTC interrupts still running! not disabled
                // UNTESTED
                if(config.trackerMode == MODE_TESTRUN) { printf("Wifi sync state: re-sync\n"); }

                i2c.begin(I2C_FREQ_HZ_400KHZ);
                device.ledGreenOn();
                if(getTimeOverWiFiNew()) {
                    hasValidTimestamp = true;
                    lastSyncType = SYNC_TYPE_WIFI;
                    syncCounter++;
                    if(!device.rtc.getTimestamp(&timestampLastSync, NULL)) { lastErrorId = 135; errorCnt++; timestampLastSync = 0; } // update timestampLastSync
                    device.blinkTimes(6, B_GREEN);
                }
                device.ledGreenOff();
                state = ST_TRACK; // back to track

                // update minute / hours, very unlikely changing here, but just to be sure
                updateHourAndMinuteForWakeStub();
                device.enableRTCInterruptInDeepSleep();
            }
            /** ---------------- WIFI SYNC STATE IN ACTIVATION ---------------- */
            else if(state == ST_WIFI_SYNC_IN_ACTIVATION) { // WARNING: RTC interrupts still running! not disabled
                // TESTED
                if(config.trackerMode == MODE_TESTRUN) { printf("Wifi sync state in activation: re-sync\n"); }

                hasValidTimestamp = false; // set to false so that even if that attempt fails the tag tries to get time when getting activated

                i2c.begin(I2C_FREQ_HZ_400KHZ);
                device.ledGreenOn();
                if(getTimeOverWiFiNew()) {
                    hasValidTimestamp = true;
                    lastSyncType = SYNC_TYPE_WIFI;
                    syncCounter++;
                    if(!device.rtc.getTimestamp(&timestampLastSync, NULL)) { lastErrorId = 135; errorCnt++; timestampLastSync = 0; } // update timestampLastSync
                    device.blinkTimes(6, B_GREEN);
                }
                device.ledGreenOff();
                state = ST_ACTIVATION; // back to activation here!
                device.enableInternalTimerInterruptInDeepSleep(60); // sleep for a minute, so that constant command will not drain battery
            }
            /** ---------------- MAG CALIBRATION STATE ---------------- */
            else if(state == ST_MAG_CALIBRATION) {
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

                if(config.trackerMode == MODE_TESTRUN) { printf("magCalibrationState: xmin %d, xmax %d, ymin %d, ymax %d, zmin %d, zmax %d -> error %d\n", magCalibration.xMin, magCalibration.xMax, magCalibration.yMin, magCalibration.yMax, magCalibration.zMin, magCalibration.zMax, calibModeError); }
                
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

                if(calibModeError) { lastErrorId = 189; errorCnt++; }
                updateHourAndMinuteForWakeStub(); // update minute / hours
                
                state = ST_ACTIVATION; // should be fine, as activation state is entered always, even when skipped, and isActivated is normally set in state before
                isActivated = false; // revoke activation -> gateway nearby required
                if(config.activationMode == ACTIVATION_MODE_STORE_PERMANENTLY) {
                    if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 0)) { lastErrorId = 186; errorCnt++; } // reset activation
                }
                device.setCPUSpeed(ESP32_10MHZ);
                if(!calibModeError) { device.blinkTimes(10, B_RED); }
                device.enableInternalTimerInterruptInDeepSleep(15);
            }
            /** ---------------- FULL RESET STATE ---------------- */
            else if(state == ST_FULL_RESET) { // without errorids, because will be gone after restart
                bool resettingError = false;
                bool flashDeleted = false;
                if(config.trackerMode == MODE_TESTRUN) { printf("Full reset: performing now\n"); }

                if(!device.initDataNVS()) { resettingError = true; }
                uint32_t flashPageWritePointer = device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_POINTER);
                uint16_t flashPageWriteOffsetPointer = device.nvsReadUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                uint32_t sendPagePointer = device.nvsReadUINT32(NVS_FLASH_SEND_PAGE_POINTER);
                uint16_t sendPageOffsetPointer = device.nvsReadUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER);
                uint16_t nvsSendBlobPointer = device.nvsReadUINT16(NVS_FLASH_SEND_BLOB_POINTER);
                uint16_t flashBlockDeletedPointer = device.nvsReadUINT16(NVS_FLASH_BLOCK_DELETED_POINTER);
                uint32_t blobSize = device.nvsGetBlobSize(NVS_FLASH_BUFFER_NAME);
                uint16_t activated = device.nvsReadUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI);
                freeMemory = device.flash.fifoGetFreeSpace(flashBlockDeletedPointer, flashPageWritePointer, flashPageWriteOffsetPointer);
                if(config.trackerMode == MODE_TESTRUN) { printf("Full reset: freeSpace: %u, writePointer: %d.%d, sendPointer: %d.%d, sendPointerBlob: %d, blobSize: %d, blockDeletePointer: %d, activated: %d\n", freeMemory, flashPageWritePointer, flashPageWriteOffsetPointer, sendPagePointer, sendPageOffsetPointer, nvsSendBlobPointer, blobSize, flashBlockDeletedPointer, activated); }

                if(freeMemory < MT29_NUMBER_BYTES) {
                    if(config.trackerMode == MODE_TESTRUN) { printf("Full reset: flash not empty (%u byte) -> full erase\n", (MT29_NUMBER_BYTES - freeMemory)); }
                    if(!device.flashPowerOn(true)) { resettingError = true; }
                    if(!device.flash.fullErase()) { resettingError = true; }
                    if(!device.flashPowerOff(true)) { resettingError = true; }
                    flashDeleted = true;
                }

                // soft resetting, so that even if state gets called multiple times no problem with wearing memory
                bool writeError = false;
                uint8_t setZeroCounter = 0;
                if(flashPageWritePointer != 0) {
                    if(!device.nvsWriteUINT32(NVS_FLASH_WRITE_PAGE_POINTER, 0)) { writeError = true; }
                    else { setZeroCounter++; }
                }
                if(flashPageWriteOffsetPointer != 0) {
                    if(!device.nvsWriteUINT16(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, 0)) { writeError = true; }
                    else { setZeroCounter++; }
                }
                if(sendPagePointer != 0) {
                    if(!device.nvsWriteUINT32(NVS_FLASH_SEND_PAGE_POINTER, 0)) { writeError = true; }
                    else { setZeroCounter++; }
                }
                if(sendPageOffsetPointer != 0) {
                    if(!device.nvsWriteUINT16(NVS_FLASH_SEND_PAGE_OFFSET_POINTER, 0)) { writeError = true; }
                    else { setZeroCounter++; }
                }
                if(nvsSendBlobPointer != 0) {
                    if(!device.nvsWriteUINT16(NVS_FLASH_SEND_BLOB_POINTER, 0)) { writeError = true; }
                    else { setZeroCounter++; }
                }
                if(flashBlockDeletedPointer != 0) {
                    if(!device.nvsWriteUINT16(NVS_FLASH_BLOCK_DELETED_POINTER, 0)) { writeError = true; }
                    else { setZeroCounter++; }
                }
                if(blobSize > 0) {
                    nvs_handle_t handle;
                    uint8_t nvsData[NVS_FLASH_BUFFER_SIZE] = { 0 };
                    if(nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle) != ESP_OK) { writeError = true; }
                    if(nvs_set_blob(handle, NVS_FLASH_BUFFER_NAME, nvsData, 0) != ESP_OK) { writeError = true; } // set to zero, independently tested, works fine
			        if(nvs_commit(handle) != ESP_OK) { writeError = true; } // 0ms
			        nvs_close(handle); // 0ms
                    setZeroCounter++;
                }
                if(activated != 0) {
                    if(!device.nvsWriteUINT16(NVS_FLASH_TAG_ACTIVATED_BY_WIFI, 0)) { writeError = true; }
                    else { setZeroCounter++; }
                }
                //if(!device.resetNVS()) { resettingError = true; } // this is the alternative

                // blink crazy police
                device.setCPUSpeed(ESP32_10MHZ);
                device.blinkGreenRedAlternating(20);

                if(config.trackerMode == MODE_TESTRUN) { printf("Full reset: NVS set to zero %d, flashDeleted %d, writeError %d, resettingError %d\n", setZeroCounter, flashDeleted, writeError, resettingError); }
                resetRTCVariables(); // fully reset RTC variables!
                state = ST_FIRST_START; // go back to start state
                device.enableInternalTimerInterruptInDeepSleep(120); // sleep for two minutes to not get into endless loop when a resetting gateway is near
            }
        }
        else { // not in productive or test run
            if(config.trackerMode == MODE_SELFTEST) { printf("Mode: SELFTEST\n");  device.selfTest(SELFTEST_VOLTAGE_REF, SELFTEST_PARAMETERS); }
        }
        startCnt++;
        device.deepSleep();
    }
}
