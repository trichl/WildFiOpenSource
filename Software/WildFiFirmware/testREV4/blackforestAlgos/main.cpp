#include "ESP32TrackerREV4.h"

ESP32TrackerREV4 device = ESP32TrackerREV4();

  
#define MODE_TESTRUN                                    0 
#define TRACKER_MODE                                    MODE_TESTRUN

#define DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY     3
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY       12*60

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint32_t bootCnt = 0;
RTC_DATA_ATTR uint32_t wakeupInterval = DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY;

#define TAG_NAME                                        "TAG100"   
const char* REST_PAYLOAD_PREFIX =                       TAG_NAME ":BBBB:HHHH:VVVV:";
char* restPrefixPointer;

post_task_stream_flash_parameters_t restStreamParams;

uint32_t calculateMemoryFullSleepTime(uint32_t currentTimestamp, uint32_t onMinute) { // sleeping with internal ESP32 timer, but trying to not add up the time drift error by correcting the sleep time depending on current timestamp
    // WARNING: when called during random time or when time interval changes: might over jump the NEXT interval when closer to next time (because assuming "too early" case)
    if(TRACKER_MODE == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: onMinute %d\n", onMinute); }
    uint32_t waitTime = 0;
    uint32_t secondsBetweenSleepInterval = onMinute * 60;
    uint32_t moduloValue = currentTimestamp % secondsBetweenSleepInterval; // between 0 .. secondsBetweenSleepInterval-1
    if(TRACKER_MODE == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: modulo value: %d\n", moduloValue); }
    if(moduloValue > (secondsBetweenSleepInterval / 2)) { // currently too early (14:13 but should be 14:15, modulo = 13min) -> assuming 14:15 should be right time
        waitTime = (secondsBetweenSleepInterval - moduloValue) + secondsBetweenSleepInterval;
        if(TRACKER_MODE == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: assuming too early (might skip one when interval changed) by %ds -> sleep for %d + %d = %ds\n", (secondsBetweenSleepInterval - moduloValue), (secondsBetweenSleepInterval - moduloValue), secondsBetweenSleepInterval, (secondsBetweenSleepInterval - moduloValue) + secondsBetweenSleepInterval); }
    }
    else { // currently too late (14:17 but should be 14:15, modulo = 2min) -> assuming 14:15 should be right time, if modulo = 0 = on time, then sleeping for exactly the value
        waitTime = secondsBetweenSleepInterval - moduloValue;
        if(TRACKER_MODE == MODE_TESTRUN) { printf("CALCMEMFULLSLEEP: assuming too late by %ds -> sleep for %d - %d = %ds\n", moduloValue, secondsBetweenSleepInterval, moduloValue, secondsBetweenSleepInterval - moduloValue); }
    }
    return waitTime;
}

extern "C" void app_main() {
    while(1) {
        if(firstStart) {
            printf("FIRST START\n");
            device.delay(5000);
            firstStart = false;
            i2c.begin(I2C_FREQ_HZ_400KHZ);
            device.rtc.setTimestamp(1604580496); // 12:48:16
            uint32_t sleepTime = calculateMemoryFullSleepTime(1604580496, wakeupInterval);
            device.enableInternalTimerInterruptInDeepSleep(sleepTime);

            // CONST CHAR TEST
            uint16_t flashPointer = 2041;
            uint16_t halfBlock = 0;
            uint16_t voltage = 4200;
            restPrefixPointer = (char *) malloc(strlen(REST_PAYLOAD_PREFIX)+1);
            strcpy(restPrefixPointer, REST_PAYLOAD_PREFIX);
            if(strlen(restPrefixPointer) >= 17 + 4) {
                Helper::addIntegerAsHexToCharArray(restPrefixPointer, 7, flashPointer);
                Helper::addIntegerAsHexToCharArray(restPrefixPointer, 12, halfBlock);
                Helper::addIntegerAsHexToCharArray(restPrefixPointer, 17, voltage);
            }

            restStreamParams.prefix = restPrefixPointer;

            printf("String: %s (strlen: %d)\n", restPrefixPointer, strlen(restPrefixPointer));
            free(restPrefixPointer);
        }
        else {
            printf("WAKE FROM DEEP SLEEP: %d\n", device.getWakeUpReason());
            i2c.begin(I2C_FREQ_HZ_400KHZ);
            bool error = false;
            uint32_t timestamp = device.rtc.getTimestamp(error); // 12:48:16 -> 12:50:00
            uint32_t gotTimestampAtMillis = ((uint32_t) Arduino::millisWrapper());
            tmElements_t timeStruct;
            breakTime(timestamp, timeStruct);
            printf("TIMESTAMP: %d = %d:%d:%d\n", timestamp, timeStruct.Hour, timeStruct.Minute, timeStruct.Second);

            uint32_t sleepTime = 1;
            if(bootCnt == 2) {
                wakeupInterval = DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY;
                printf("CHANGING WAKE UP INTERVAL\n");
            }
            else if(bootCnt == 4) {
                wakeupInterval = DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY;
                printf("CHANGING WAKE UP INTERVAL\n");
            }
            else if(bootCnt == 7) {
                wakeupInterval = DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY;
                printf("CHANGING WAKE UP INTERVAL\n");
            }
            sleepTime = calculateMemoryFullSleepTime(timestamp, wakeupInterval);

            
            device.enableInternalTimerInterruptInDeepSleep(sleepTime+5);
        }
        printf("\n");
        bootCnt++;
        device.deepSleep();
    }
}