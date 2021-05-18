#include "ESP32TrackerREV4.h"

ESP32TrackerREV4 device = ESP32TrackerREV4();

// Next open error id: lastErrorId = 116

// NO DATA TRANSMISSION: POWER CONSUMPTION AVERAGE IN PRODUCTIVE (NO WIFI):
    // @6.25Hz: 160uA (more with new LDO)
    // @50Hz: 853uA, in 16.25s producing 29 + 3840 + 972 = 4841 Bytes = 10.42 days until memory full
    // @200Hz: 3.2mA, in 3.85s producing 29 + 3600 + 954 Bytes = 4583 Bytes -> 4583 / 3.85 = 1190.39 Bytes per second -> ((256*1024*1024) / 1190.39) / (3600) = 62.6h until memory full
        // 4.36mA WITH data transmission @11DBM, live data every 4 blocks
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

// TODO: problem: partial page programming should only be done with 512 bytes, not randomly

// TODO (LOW PRIO): new data with >= REV4B: hall sensor
// TODO (LOW PRIO): new data: sample one time MAG data?
// TODO (LOW PRIO): 1MHz I2C should not work with RTC?!? -> dynamic I2C speed switching -> TEST i2c.changeClockSpeed(I2C_FREQ_HZ_400KHZ)
// TEST (LOW PRIO): instead of wake stub, get data after boot (maybe faster = less power?)
// TEST: on DOG sometimes 2g exceeded! -> TEST WITH HIGHER G!
// FAIL: Antennenfuß unten zur MCU hin locker nach Hundetest

// TODO EXPERIMENTAL: modify readAccFIFOInOneGo (no delays, bool to skip wait on mode change at the end, ...)
// TODO EXPERIMENTAL: FIFO SIZE higher for 200HZ??
// TODO EXPERIMENTAL: SINGLE CORE OPERATION!!!!!

// ADD MAG DATA TO FIFO

// IMPORTANT: CHANGING ACC_INTERRUPT_WATERMARK FROM 960 to 900 FOR 200 HZ SAMPLING!!!!

/** Data transmission to use */
//#define TRANSMISSION_METHOD_ESP_NOW                                                     // transmit data over esp now
#define TRANSMISSION_METHOD_WIFI                                                        // transmit data over wifi

#define MODE_TESTRUN                                    0                               // with debug output, not writing into flash memory, not incrementing NVS_FLASH_POINTER
#define MODE_PRODUCTIVE                                 1                               // normal tracker mode, fully operational, flash memory should be empty before and NVS_FLASH_POINTER reset to 0
#define MODE_SELFTEST                                   2                               // runs selftest on flash memory
#define MODE_READFLASH                                  3                               // reads out full flash (10.000 pages = 30min) and then goes to sleep
#define MODE_MOCK_FLASH_STATE                           4                               // bring flash pointers at certain position to test stuff like memory full state

#define ACTIVATION_MODE_SKIP                            0                               // activation mode is skipped
#define ACTIVATION_MODE_STORE_PERMANENTLY               1                               // activation information is stored permanently in NVS (still valid after resets)
#define ACTIVATION_MODE_ON_EVERY_START                  2                               // activation information is stored only during a power cycle (lost after battery disconnected)

/** --- CURRENT TRACKER MODE --- */
#define TRACKER_MODE                                    MODE_TESTRUN
acc_config_t accConfig = {
                                                        BMX160_ACCEL_ODR_200HZ,          // BMX160_ACCEL_ODR_50HZ, acc frequency
                                                        BMX160_ACCEL_BW_RES_AVG2,       // BMX160_ACCEL_BW_RES_AVG8, acc averaging cycles
                                                        BMX160_ACCEL_RANGE_2G           // BMX160_ACCEL_RANGE_2G, acc range (WARNING: changes meaning of LSB value in data)
};
/** Selftest mode parameters */
//#define SELFTEST_PARAMETERS                             SELFTEST_VOLTAGE | SELFTEST_HALLSENSOR | SELFTEST_I2C | SELFTEST_RTC | SELFTEST_ACCFOC_CHECK | SELFTEST_BARO | SELFTEST_WIFI_SCAN // SELFTEST_VOLTAGE | SELFTEST_LEDS | SELFTEST_HALLSENSOR | SELFTEST_I2C | SELFTEST_RTC | SELFTEST_ACCFOC_CHECK | SELFTEST_ACCFOC_EXECUTE_IF_UNSET | SELFTEST_BARO | SELFTEST_FLASH_BAD_BLOCKS | SELFTEST_FLASH_READ_WRITE | SELFTEST_FLASH_FULL_ERASE | SELFTEST_NVS_RESET | SELFTEST_WIFI_SCAN | SELFTEST_ESPNOW_BROADCAST
#define SELFTEST_PARAMETERS                             SELFTEST_ACCFOC_CHECK | SELFTEST_FLASH_FULL_ERASE | SELFTEST_NVS_RESET // SELFTEST_VOLTAGE | SELFTEST_LEDS | SELFTEST_HALLSENSOR | SELFTEST_I2C | SELFTEST_RTC | SELFTEST_ACCFOC_CHECK | SELFTEST_ACCFOC_EXECUTE_IF_UNSET | SELFTEST_BARO | SELFTEST_FLASH_BAD_BLOCKS | SELFTEST_FLASH_READ_WRITE | SELFTEST_FLASH_FULL_ERASE | SELFTEST_NVS_RESET | SELFTEST_WIFI_SCAN | SELFTEST_ESPNOW_BROADCAST
#define SELFTEST_VOLTAGE_REF                            3750                            // during selftest routine (if voltage check enabled): check if this value (from power supply) is actually measured

/** Night time mode (UTC TIME!) */
#define NIGHTTIME_ACTIVATED                             1                               // when activated then loggers stops logging at certain time of day
#define NIGHTTIME_TURN_OFF_HOUR                         9                               // stop logging at this UTC hour
#define NIGHTTIME_TURN_OFF_MINUTE                       23                              // stop logging at this UTC minute
#define NIGHTTIME_TURN_ON_HOUR                          9                               // start again logging at this UTC hour
#define NIGHTTIME_TURN_ON_MINUTE                        45                              // start again logging at this UTC minute

/** Battery protection */
#define BATT_MIN_VOLTAGE                                3550                            // 3550, hibernate below that voltage and
#define BATT_RESTART_VOLTAGE                            3650                            // 3650, wait until voltage above that threshold again
#define FIRST_UNDER_VOLTAGE_SLEEP_TIME                  1*3600                          // 1*3600, first time UV detected -> stop imu fifo and sleep for that time
#define FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME              12*3600                         // 12*3600, second and more times UV detected -> sleep for that time
#define SLEEP_TIME_AFTER_START                          3                               // 10, after initial start, wait this amount of seconds before entering next state (to give time for putting tag into housing and stuff)

/** Force download WIFI Configuration (immediately after wake-up, during time wifi scan, enter special mode to download all data and then sleep forever) */
const char* FORCE_DOWNLOAD_WIFI_SSID =                  "mpiforce";                     // mpidownload, enter special mode during get time state when this wifi is seen
const char* FORCE_DOWNLOAD_WIFI_PASSWORD =              "ABCDEFGH";                     // ABCDEFGH, enter special mode during get time state when this wifi is seen

/** Time over WIFI Configuration (immediately after wake-up) on channel 6, then 1, then 11, each for 120ms */
const uint8_t TIME_WIFI_LIST_SIZE =                     2;
const char* TIME_WIFI_SSIDS[TIME_WIFI_LIST_SIZE] =      { FORCE_DOWNLOAD_WIFI_SSID, "mpitime" };        // mpitime, wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* TIME_WIFI_PASSWORDS[TIME_WIFI_LIST_SIZE] =  { FORCE_DOWNLOAD_WIFI_PASSWORD, "87654321" };   // 87654321, wifi password for activation/timestamp                  
#define TIME_WIFI_OUTPUT_POWER                          RADIO_MAX_TX_11_DBM             // RADIO_MAX_TX_11_DBM, 19.5dBm will brown out on REV3
#define TIME_BETWEEN_GET_TIME_RETRIES_SECONDS           900                             // 900, 120 = 660uA, if not activated: sleep for that time, wake up, check if wifi there and try to activate, 120 = 234uA average
#define TIME_SLEEP_AFTER_GOT_TIME                       5                               // 5, sleep for that time until re-starting the system

/** Activation over WIFI Configuration (only start if this wifi has been seen) -> every 60min = 35.3uA (channels 1, 6, 11 each 120ms), every 60min = 35.1uA (channels 1-11 each 30ms), every 60min = 26.6uA (only one channel) */
#define ACTIVATION_MODE                                 ACTIVATION_MODE_SKIP            // how shall the tag be activated
const char* ACTIVATION_WIFI_SSIDS[1] =                  { "mpistart" };                 // mpistart, wifi name to scan for at beginning -> if found -> stark tracking (NO PASSWORD NEEDED)
#define ACTIVATION_WIFI_OUTPUT_POWER                    RADIO_MAX_TX_11_DBM             // RADIO_MAX_TX_11_DBM, 19.5dBm will brown out on REV3
#define ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE           0                               // 0, on which minute shall be checked (40 = 14:40, 15:40, ..)
#define ACTIVATION_WAKE_UP_ON_MODULO_HOURS              1                               // 1, 1 = every hour, 2 = on even hours (14, 16, ..), 3 = on all hours that are dividable by 3, ..

/** Configuration Data Transmission: Common */
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY       12*60                           // 12*60 = 00:00 and 12:00 (UTC), 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission nothing transmitted -> re-try more seldomly
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY     2                               // 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission something transmitted -> re-try more often
#define DATATRANSM_MIN_BLOCKS_TO_TRANSMIT               4                               // 1, only start scan (and data transmission) if there is enough data to send
#define DATATRANSM_MAX_BLOCKS_TO_TRANSMIT               150                             // 150, during one connection, burst maximum this amount of blocks (keep in mind: wifi needs 2 transmissions per block)

/** Memory Full Configuration */
#define ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES       2048*64*2048                    // 2048*64*2048, only exiting mode when totally empty = 2048*64*2048 = 268435456, 1 block = 64*2048
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY       6*60                            // 6*60, 12*60 = 00:00 and 12:00 (UTC), 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission nothing transmitted -> re-try more seldomly
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_LESS_FREQU     5                               // last data transmission something transmitted, current data transmission NOTHING -> use this minute interval for one time
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY     1                               // 1, 12hrs = 60*12 = 720 = 00:00 and 12:00, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission something transmitted -> re-try more often

/** Configuration Data Transmission: ESP NOW */
#ifdef TRANSMISSION_METHOD_ESP_NOW
//static uint8_t espNowReceiverMac[6] =                 {0xD8, 0xA0, 0x1D, 0x69, 0xE8, 0xFC}; // PicoKit
static uint8_t espNowReceiverMac[6] =                   {0xAC, 0x67, 0xB2, 0x2B, 0x4E, 0x88}; // ESP32 CAM (first one)
#define ESPNOW_OUTPUT_POWER                             RADIO_MAX_TX_2_DBM              // 19.5dBm will brown out on REV3
#define ESPNOW_DATA_RATE                                WIFI_PHY_RATE_18M               // well tested with 18M
#define ESPNOW_LONG_RANGE                               0                               // long range only with 1MBPS -> ESPNOW_DATA_RATE should be 1MBPS!
#define ESPNOW_MIN_BATT_VOLTAGE                         1000                            // 3550, don't start scan if battery voltage too low
#define ESPNOW_MIN_BATT_VOLTAGE_DURING_TRANSM           1000                            // 3300, don't continue transmission if voltage dropped below that threshold after one block transmission
#endif

/** Configuration Data Transmission: WIFI */
#ifdef TRANSMISSION_METHOD_WIFI
#define TAG_NAME                                        "COCK01"                        // ALWAYS 6 CHARACTERS! otherwise REST_ADD_HEADER_VALUE not correct, unique id for post calls
#define WIFI_OUTPUT_POWER                               RADIO_MAX_TX_11_DBM             // 19.5dBm will brown out on REV3
#define WIFI_MIN_BATT_VOLTAGE                           3500                            // 3500, don't start scan if battery voltage too low
#define WIFI_MIN_BATT_VOLTAGE_DURING_TRANSM             3200                            // 3200, don't continue wifi transmission if voltage dropped below that threshold after one block transmission
#define WIFI_MAX_POST_TASK_TIME_SECONDS                 45                              // 45, max time for sending one successfully transmitted block
const uint8_t DATATR_KNOWN_WIFI_LIST_SIZE =             1;                              // IMPORTANT: DO NOT FORGET TO CHANGE THAT!!!!!!!!!
const char* DATATR_WIFI_SSIDS[DATATR_KNOWN_WIFI_LIST_SIZE] = {                                        // first priority: channel (6, 1, 11), within channel the name
                                                        /*"guest", "Tracker1", "LiWoAb New",*/ "mpidata"
};                      
const char* DATATR_WIFI_PASSWORDS[DATATR_KNOWN_WIFI_LIST_SIZE] = {                                    // first priority: channel (6, 1, 11), within channel the name
                                                        /*"xxxxxxx", "Tracker1", "xxxxxxx",*/ "87654321"
}; 
/*
const char* REST_URL =                                  "https://timmtracker-3489.restdb.io/rest/debug";
const char* REST_CONTENT_TYPE =                         "application/json";
const char* REST_ADD_HEADER_KEY =                       "x-apikey";
const char* REST_ADD_HEADER_VALUE =                     "04ef802eae678a82cc9d1e32e0ac48e8a6000";
#define REST_USE_BASE64_ENCODING                        1
*/
const char* REST_URL =                                  "http://192.168.43.1:8080/store";
const char* REST_CONTENT_TYPE =                         "application/octet-stream";
const char* REST_ADD_HEADER_KEY =                       "Content-Length";
const char* REST_ADD_HEADER_VALUE =                     "65563"; // strlen(PREFIX_CONSTRUCTED: "ABCDEF:PPPPPPPP:VVVV:") 21 + strlen(taskParams.postfix) 6 + (PAGES_IN_ONE_GO * 2048 = 65536)
#define REST_USE_BASE64_ENCODING                        0
#define REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX            1
const char* REST_PAYLOAD_PREFIX =                       TAG_NAME ":PPPPPPPP:VVVV:"; 
const char* REST_PAYLOAD_POSTFIX =                      TAG_NAME;
post_task_stream_flash_parameters_t restStreamParams;
uint8_t *dmaBuffer2048Bytes = NULL;
uint8_t *additionalDataAfterPrefixPointer = NULL;
#endif

/** Connect to WIFI Configuration */
#define WIFI_MAX_CONNECT_TIME_SECONDS                   8                               // (used for get time, activation and data transmission) connect should take <1s

/** NVS storage names */
#define NVS_FLASH_SEND_NEXT_BLOCK_POINTER               "blockpntsent"                  // data transmission: pointing on next block to transmit
#define NVS_FLASH_WRITE_POINTER                         "flashpnt"                      // data writing: pointing on current flash page
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffsetpnt"                // data writing: pointing on current byte in flash page
#define NVS_FLASH_TAG_ACTIVATED_BY_WIFI                 "activated"                     // tag has seen activation wifi
#ifdef TRANSMISSION_METHOD_ESP_NOW
    #define NVS_FLASH_SEND_NEXT_PAGE_POINTER            "pagepntsent"                   // data transmission: pointing on next page within block to transmit
    #define NVS_FLASH_SEND_NEXT_SUBPAGE_POINTER         "subpagesent"                   // data transmission: pointing on next subpage within page to transmit
#endif
#ifdef TRANSMISSION_METHOD_WIFI
    #define NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER      "halfblocksent"                 // data transmission: sometimes only half a block transmitted -> save and do not retransmit
#endif

/** IMU data temporary in RAM */
#define ACC_INTERRUPT_WATERMARK                         900                             // 960 for <= 50Hz, imu is generating interrupt when fifo is at that fill level
#define ACC_RAM_SIZE_1                                  (ACC_INTERRUPT_WATERMARK + 12)  // ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2 = RTC memory for storing fifo data in wake stub
#define ACC_RAM_SIZE_2                                  4                               // see above

/** Mocking for data transmission */
#define MOCK_FLASH_WRITES                               0                               // 1 = flash will not be written, but all pointers will be updated and data transmitted
#define MOCK_FLASH_READ                                 0                               // (ESP NOW ONLY) 1 = during transmissions dummy data is sent
#define MOCK_FLASH_DELETE                               0                               // 1 = flash block will not be deleted after successful data transmission, but all pointers are updated

// state enums
typedef enum { ST_FIRST_START_HARD_RESET = 0, ST_GET_TIME = 1, ST_WAIT_FOR_ACTIVATION = 2, ST_START = 3, ST_TRACK = 4, ST_PWRDWN = 5, ST_MEMFULL = 6, ST_NIGHT_TIME = 7, ST_FORCE_DOWNLOAD = 8 } tracker_state_t;

// RTC variables
RTC_DATA_ATTR uint32_t timestampNextDataTransmission = 0;
RTC_DATA_ATTR uint32_t memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY;
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

void mockFlashState() {
    device.delay(8000);
    if(!device.resetDataNVS()) { printf("NVS ERASE ERROR\n"); }
    if(!device.initDataNVS()) { printf("ERROR NVS INIT\n"); }
    if(!device.nvsWriteUINT32(NVS_FLASH_WRITE_POINTER, 131071 - 6)) { printf("ERROR\n"); } // 131071 = last page
    if(!device.nvsWriteUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER, 0)) { printf("ERROR\n"); }
    if(!device.nvsWriteUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER, 0)) { printf("ERROR\n"); }
    #ifdef TRANSMISSION_METHOD_ESP_NOW
        if(!device.nvsWriteUINT8(NVS_FLASH_SEND_NEXT_PAGE_POINTER, 0)) { printf("ERROR\n"); }
        if(!device.nvsWriteUINT8(NVS_FLASH_SEND_NEXT_SUBPAGE_POINTER, 0)) { printf("ERROR\n"); }
    #endif
    #ifdef TRANSMISSION_METHOD_WIFI
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
    if(!device.flashPowerOff()) { printf("ERROR FLASH3\n"); }
}

bool isTimeToGoToBed() {
    bool error = false;
    uint8_t currentHour = 0, currentMinute = 0;
    uint16_t minutesOfDay = 0;
    const uint16_t OFF_MINUTES_OF_DAY = (NIGHTTIME_TURN_OFF_HOUR * 60) + NIGHTTIME_TURN_OFF_MINUTE; // 0 ........ 1439
    const uint16_t ON_MINUTES_OF_DAY = (NIGHTTIME_TURN_ON_HOUR * 60) + NIGHTTIME_TURN_ON_MINUTE; // 0 ........ 1439
    if(!NIGHTTIME_ACTIVATED) { return false; } // never time to go to bed
    currentHour = device.rtc.getHours(error);
    if(error) { lastErrorId = 102; errorCnt++; return false; }
    currentMinute = device.rtc.getMinutes(error);
    if(error) { lastErrorId = 104; errorCnt++; return false; }
    minutesOfDay = (currentHour * 60) + currentMinute; // calculate minutes passed that day, 0 ........ 1439
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d NIGHTTIME: (%d:%d) sleep between %d:%d - %d:%d -> ", ((uint32_t) Arduino::millisWrapper()), currentHour, currentMinute, NIGHTTIME_TURN_OFF_HOUR, NIGHTTIME_TURN_OFF_MINUTE, NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE); }
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
    secondsPassedSinceLastDesiredTime = currentTimestamp % (60 * onMinute);
    secondsUntilNextDesiredTime = (60 * onMinute) - secondsPassedSinceLastDesiredTime;
    timestampNextDataTransmission = currentTimestamp + secondsUntilNextDesiredTime; // substract this from timestamp as a mocked last time transmission
    if(TRACKER_MODE == MODE_TESTRUN) { printf("NEXTDATATRANSM: next transmission: %d, current time %d, difference %ds, onMinute %d\n", timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp, onMinute); }
}

bool itsTimeForDataTransmission(uint32_t currentTimestamp) {
    if(currentTimestamp == 0) { return false; }
    return currentTimestamp >= timestampNextDataTransmission;
}

#ifdef TRANSMISSION_METHOD_WIFI
bool dataTransmissionWifi(bool forceMode, uint32_t currentTimestamp, uint32_t batteryVoltage, uint32_t flashPointer, const char** SSIDS_TO_USE, const char** PASSWORDS_TO_USE, const uint8_t SSIDS_SIZE) { // perform a wifi scan (366ms scan only, 1515ms in total if connecting)
    uint32_t wifiStartTime = ((uint32_t) Arduino::millisWrapper());
    bool somethingTransmitted = false; // at least ONE block deleted (so more space in memory than before)

    /* -------- GET POINTER VALUES FROM NVS -------- */
    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER);
    uint16_t flashHalfBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER);

    /* -------- GET NUMBER OF BLOCKS TO TRANSMIT -------- */
    uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockToSendNextPointer, flashPointer);

    /* -------- CHECK CONDITIONS TO EXECUTE WIFI SCAN -------- */
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: next scan %d, blocksToTransmit %d, batteryV %d\n", ((uint32_t) Arduino::millisWrapper()), timestampNextDataTransmission, blocksToTransmit, batteryVoltage); }
    if(batteryVoltage > WIFI_MIN_BATT_VOLTAGE) { // only if enough juice in battery
        if(blocksToTransmit >= DATATRANSM_MIN_BLOCKS_TO_TRANSMIT) { // only if enough data to transmit
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
                            lastErrorId = 70 + device.getWiFiPOSTCallStatus(); // 19 error codes -> 70 + 19 = 89, next errorid should be 90
                            errorCnt++;
                        }
                        device.restPostStreamGetSuccessfullyTransmittedBlocks(successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks);
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: successfully transmitted blocks: %d, half blocks: %d\n", ((uint32_t) Arduino::millisWrapper()), successfullyTransmittedBlocks, successfullyTransmittedHalfBlocks); }
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
                /* -------- UPDATING SCANNING FREQUENCY AFTER A SCAN (OR DATATRANSMISSION!!!) WAS PERFORMED -------- */
                if(somethingTransmitted) { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); } // data to transmit and wifi found and data transmitted -> try more frequently (if enough data)
                else { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); } // data to transmit, but wifi not found or data transmission not acked -> try less frequently
            }
            else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> next time %d, current time %d, wait %ds\n", ((uint32_t) Arduino::millisWrapper()), timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp); }
        }
        else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> Blocks to transmit (%d) too low\n", ((uint32_t) Arduino::millisWrapper()), blocksToTransmit); }
    }
    else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d WIFI: NO -> Battery too low\n", ((uint32_t) Arduino::millisWrapper())); }
    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}
#endif

#ifdef TRANSMISSION_METHOD_ESP_NOW
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
    if(blocksToTransmitLimited > DATATRANSM_MAX_BLOCKS_TO_TRANSMIT) { // limit maximum number of blocks to transmit (will be less if voltage low or errors)
        blocksToTransmitLimited = DATATRANSM_MAX_BLOCKS_TO_TRANSMIT;
    }
    /* -------- CHECK CONDITIONS -------- */
    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: next scan %d, blocksToTransmit %d, blockPnt %d, pagePnt %d, subPagePnt %d, batteryV %d\n", ((uint32_t) Arduino::millisWrapper()), timestampNextDataTransmission, blocksToTransmit, flashBlockToSendNextPointer, flashPageToSendNextPointer, flashSubPageToSendNextPointer, batteryVoltage); }
    if(batteryVoltage > ESPNOW_MIN_BATT_VOLTAGE) { // only if enough juice in battery
        if(blocksToTransmit >= DATATRANSM_MIN_BLOCKS_TO_TRANSMIT) { // only if enough data to transmit
            if(forceMode || itsTimeForDataTransmission(currentTimestamp)) { // only if last scan at least x seconds ago
                /* -------- CONDITIONS OKAY -> TRY TO SEND -------- */
                if((TRACKER_MODE == MODE_TESTRUN) && (forceMode)) { printf("%d ESPNOW: FORCING TRANSMISSION TRY!\n", ((uint32_t) Arduino::millisWrapper())); }
                espNowStartTime = ((uint32_t) Arduino::millisWrapper());
                //  NO!! rtc_slow_seg overflow by 1868 bytes
                /*if(!device.customPhyInit()) { // init PHY (well, don't init because will be called after deep sleep, so just copy the RTC data)
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: custom PHY init failed!\n", ((uint32_t) Arduino::millisWrapper())); }
                    lastErrorId = 61; errorCnt++;
                    return false;
                }*/
                if(!device.initESPNOWStationary(ESPNOW_LONG_RANGE, ESPNOW_OUTPUT_POWER, true, ESPNOW_DATA_RATE)) { // 165ms, performs full calibration I guess (because custom PHY function overflows slow rtc)
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: initESPNOW ERROR!\n", ((uint32_t) Arduino::millisWrapper())); }
                    lastErrorId = 44; errorCnt++;
                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY);
                    return false;
                } 
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: INIT TOOK %dms!\n", ((uint32_t) Arduino::millisWrapper()), ((uint32_t) Arduino::millisWrapper()) - espNowStartTime); }
                if(!device.addESPNOWReceiverStationary(espNowReceiverMac)) {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: addESPNOWReceiver ERROR!\n", ((uint32_t) Arduino::millisWrapper())); }
                    lastErrorId = 45; errorCnt++;
                    setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY);
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
                /* -------- UPDATING SCANNING FREQUENCY AFTER A SCAN WAS PERFORMED -------- */
                if(somethingTransmitted) { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); } // data to transmit and wifi found and data transmitted -> try more frequently (if enough data)
                else { setNextDataTransmissionTimestamp(forceMode, currentTimestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY); } // data to transmit, but wifi not found or data transmission not acked -> try less frequently
            }
            else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: NO -> next time %d, current time %d, wait %ds\n", ((uint32_t) Arduino::millisWrapper()), timestampNextDataTransmission, currentTimestamp, timestampNextDataTransmission - currentTimestamp); }
        }
        else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: NO -> Blocks to transmit (%d) too low\n", ((uint32_t) Arduino::millisWrapper()), blocksToTransmit); }
    }
    else if(TRACKER_MODE == MODE_TESTRUN) { printf("%d ESPNOW: NO -> Battery too low\n", ((uint32_t) Arduino::millisWrapper())); }
    return somethingTransmitted; // might be updated by fifoPopDelete or same like before
}
#endif

RTC_DATA_ATTR bool wakeStub() {
    if(fifoStopped) { // in certain states imu deactivated (memory full or power down) -> power ESP32 directly
        return true; // IMPORTANT: if not stopped then might be rejected put START configures setWakeStubRejectionInterruptSrc for EXT1 = Acc (not RTC)
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
        if((TRACKER_MODE == MODE_TESTRUN) || (TRACKER_MODE == MODE_PRODUCTIVE)) {
            if(TRACKER_MODE == MODE_TESTRUN) { printf("-----\nState: %d, Fifo stopped wakestub: %d\n", state, fifoStopped); }
            /** ---------------- VERY FIRST ENTRY POINT AFTER HARD RESET (RTC memory = reset), only executed ONCE, NEVER after Deepsleep! ---------------- */
            if(state == ST_FIRST_START_HARD_RESET) { // custom wake stub not running -> no fifo
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: HARD RESET!\n"); }
                // disable no boot if voltage low (do here, only needed once!)
                device.disableWakeStubNoBootIfVoltageLow();
                device.blinkTimes(3, B_RED);

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
                if(TRACKER_MODE == MODE_TESTRUN) {
                    uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                    uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER);
                    printf("Reset: flashPointer: %d, flashOffsetPointer = %d, flashBlockToSendNextPointer: %d\n", flashPointer, flashOffsetPointer, flashBlockToSendNextPointer);
                }
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
                if(hasValidTimestamp) { // some brownout or other reset of MCU -> time still ok
                    setNextDataTransmissionTimestamp(false, timestamp, DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY); // IMPORTANT: set here, so that after (re)start not immediately try to transmit data again
                    if(isActivated) { // also already activated -> start immediately
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp valid and activated -> START!\n"); }
                        state = ST_START;
                    }
                    else { // time okay, but not activated -> go to activation
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp valid, NOT activated (mode %d) -> activation state!\n", ACTIVATION_MODE); }
                        if(!device.rtc.setHourlyInterrupt(ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE)) { lastErrorId = 95; errorCnt++; } // time valid, so better activate the rtc interrupt
                        state = ST_WAIT_FOR_ACTIVATION;
                    }
                }
                else { // time not okay, move to time state
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: timestamp not valid, activation = %d (mode %d)!\n", isActivated, ACTIVATION_MODE); }
                    state = ST_GET_TIME;
                }
                /*if(!device.customPhyInit()) { // after hard reset: perform a full PHY calibration -> NO!! rtc_slow_seg overflow by 1868 bytes
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Reset: CUSTOM PHY INIT FAILED!!! SHIZZLE\n"); }
                    lastErrorId = 60; errorCnt++;
                }*/
                device.enableInternalTimerInterruptInDeepSleep(SLEEP_TIME_AFTER_START); // restart system into next state
            }
            /** ---------------- GET TIME STATE ---------------- */
            else if(state == ST_GET_TIME) { // custom wake stub not running -> no fifo
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: try getting time! V = %d\n", device.readSupplyVoltageFromWakeStub()); }
                if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) { // voltage low, go to power down state
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: undervoltage (%d)! Don't scan!\n", device.readSupplyVoltageFromWakeStub()); }
                    device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time
                }
                else {
                    if(device.initWiFi()) {
                        uint8_t foundArrayId = 0;
                        uint8_t foundOnChannel = 0;
                        bool connectionTimeout = false;
                        uint32_t scanStartTime = ((uint32_t) Arduino::millisWrapper());
                        if(device.scanForWiFisOn1and6and11WithPriority((TRACKER_MODE == MODE_TESTRUN), TIME_WIFI_SSIDS, TIME_WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, TIME_WIFI_OUTPUT_POWER, 120, 500)) { 
                            if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Arduino::millisWrapper()) - scanStartTime, 500); }
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
                                        uint32_t connectStartedTime = ((uint32_t) Arduino::millisWrapper());
                                        while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                                            device.delay(20);
                                            if(((uint32_t) Arduino::millisWrapper()) - connectStartedTime > (WIFI_MAX_CONNECT_TIME_SECONDS * 1000)) { // e.g. password wrong
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
                                            i2c.begin(I2C_FREQ_HZ_400KHZ);
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
                                        device.blinkTimes(6, B_GREEN);
                                        device.enableInternalTimerInterruptInDeepSleep(TIME_SLEEP_AFTER_GOT_TIME); // restart in x seconds and move to activation
                                    }
                                    else {
                                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: wifi was seen, but still something missing (pw wrong, no internet) -> SLEEP for %ds\n", TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); }
                                        device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS);
                                    }
                                }
                            }
                            else {
                                if(TRACKER_MODE == MODE_TESTRUN) { printf("Time: wifi NOT found -> SLEEP for %ds\n", TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); }
                                device.disconnectAndStopWiFi();
                                device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); // sleep some time and try again afterwards   
                            }
                        }
                        else { lastErrorId = 32; errorCnt++; device.disconnectAndStopWiFi(); device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); }
                    }
                    else { lastErrorId = 33; errorCnt++; device.enableInternalTimerInterruptInDeepSleep(TIME_BETWEEN_GET_TIME_RETRIES_SECONDS); }
                }
            }
            /** ---------------- WAIT FOR ACTIVATION STATE ---------------- */
            else if(state == ST_WAIT_FOR_ACTIVATION) { // custom wake stub not running -> no fifo
                bool i2cError = false;
                if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: NOT ACTIVATED!\n"); }
                i2c.begin(I2C_FREQ_HZ_400KHZ);
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
                                uint32_t scanStartTime = ((uint32_t) Arduino::millisWrapper());
                                device.ledGreenOn(); // visual feedback during activation scan
                                if(device.scanForWiFisOn1and6and11(ACTIVATION_WIFI_SSIDS, 1, &foundArrayId, &foundOnChannel, ACTIVATION_WIFI_OUTPUT_POWER, 120, 500)) { 
                                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Activation: SCAN TOOK %dms (MAX %d per channel)!\n", ((uint32_t) Arduino::millisWrapper()) - scanStartTime, 500); }
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
                                device.ledGreenOff(); // visual feedback during activation scan
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
            }
            /** ---------------- START STATE ---------------- */
            else if(state == ST_START) {
                // blink a couple of times
                device.blinkTimes(5, B_BOTH);

                // init BMX
                i2c.begin(I2C_FREQ_HZ_1MHZ);
                if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 1; errorCnt++; }
                if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 2; errorCnt++; }
                if(!device.imu.initFIFOForAcc()) { lastErrorId = 3; errorCnt++; }
                if(!device.imu.resetFIFO()) { lastErrorId = 113; errorCnt++; } // empty fifo because after hard reset there might still be some data in the fifo

                // print flash pointer (also in productive)
                if(!device.initDataNVS()) { lastErrorId = 4; errorCnt++; }
                uint32_t flashPointer = device.nvsReadUINT32(NVS_FLASH_WRITE_POINTER);
                uint16_t flashOffsetPointer = (uint16_t) device.nvsReadUINT32(NVS_FLASH_WRITE_PAGE_OFFSET_POINTER);
                
                printf("Start: FLASH POINTER: %d, FLASH OFFSET POINTER: %d, FREE HEAP: %d\n", flashPointer, flashOffsetPointer, heap_caps_get_free_size(MALLOC_CAP_8BIT));

                // custom wake stub function (BEFORE THAT: no wake stub!)
                device.customWakeStubFunction(wakeStub);
                device.setWakeStubRejectionInterruptSrc(USE_EXT1_IF_WAKE_UP_REJECTED);

                // move to next state
                state = ST_TRACK;
                device.enableAccInterruptInDeepSleep();
            }
            /** ---------------- POWER DOWN STATE (IMU stopped!) ---------------- */
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
            /** ---------------- TRACKING STATE ---------------- */
            else if(state == ST_TRACK) {
                if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: WAKE NUMBER: %d, V_BATT_WakeStub: %d\n", ((uint32_t) Arduino::millisWrapper()), startCnt, device.readSupplyVoltageFromWakeStub()); }
                // I2C start (1ms)
                i2c.begin(I2C_FREQ_HZ_1MHZ);
                if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) {
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: POWER DOWN for %ds\n", ((uint32_t) Arduino::millisWrapper()), FIRST_UNDER_VOLTAGE_SLEEP_TIME); }
                    state = ST_PWRDWN;
                    if(!device.imu.stop()) { lastErrorId = 17; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
                    fifoStopped = true; // WARNING: setWakeStubRejectionInterruptSrc set to EXT1 = Acc -> wake stub needs to return TRUE always in that case
                    device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
                }
                else if(isTimeToGoToBed()) {
                    state = ST_NIGHT_TIME;
                    if(!device.imu.stop()) { lastErrorId = 101; errorCnt++; } // turn off imu, WARNING: will not fully turn off MAG if turned on before
                    fifoStopped = true; // WARNING: setWakeStubRejectionInterruptSrc set to EXT1 = Acc -> wake stub needs to return TRUE always in that case
                    device.blinkTimes(5, B_RED);
                    if(!device.rtc.setDailyInterrupt(NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE)) { lastErrorId = 103; errorCnt++; } // WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                    device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
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
                    if(fifoDataNewestLen >= 996) { lastErrorId = 100; errorCnt++; } // data lost possible
                    if(TRACKER_MODE == MODE_TESTRUN) {
                        printf("%d FIFO: RAM FIFO DATA POINTER: %d, LAST FIFO LEN: %d\n", ((uint32_t) Arduino::millisWrapper()), fifoDataPointerRam, fifoLenRam);
                        printf("%d FIFO: READ IN %lld ms\n", ((uint32_t) Arduino::millisWrapper()), (Arduino::millisWrapper() - t));
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
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d Flash: FIFO full, stop IMU -> go into ST_MEMFULL state in 5 seconds\n", ((uint32_t) Arduino::millisWrapper())); }
                        if(!device.imu.stop()) { lastErrorId = 26; errorCnt++; } // turn off imu, WARNING: will not fully turn of MAG if turned on before
                        fifoStopped = true; // WARNING: setWakeStubRejectionInterruptSrc set to EXT1 = Acc -> wake stub needs to return TRUE always in that case
                        memFullWakeUpOnMinute = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY; // after memory full, try frequently to get rid of data
                        state = ST_MEMFULL;
                        if(!device.flashPowerOff()) { lastErrorId = 27; errorCnt++; } // important!
                        fifoDataPointerRam = 0; // reset pointer
                        heap_caps_free(sensorData); // important, free sensorData memory
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Arduino::millisWrapper()), lastErrorId, errorCnt); }
                        device.enableInternalTimerInterruptInDeepSleep(5); // immediately try to get rid of the data
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
                        // data transmission (0ms if not connecting)
                        timeNow = ((uint32_t) Arduino::millisWrapper());
                        bool somethingTransmitted = false;
                        #ifdef TRANSMISSION_METHOD_ESP_NOW
                        somethingTransmitted = dataTransmissionEspNow(false, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer);
                        #endif
                        #ifdef TRANSMISSION_METHOD_WIFI
                        somethingTransmitted = dataTransmissionWifi(false, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                        #endif
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d DATATRANSM: needed %dms, something transmitted: %d\n", ((uint32_t) Arduino::millisWrapper()), ((uint32_t) Arduino::millisWrapper()) - timeNow, somethingTransmitted); }
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
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("%d State: LAST ERROR ID: %d (CNT %d)\n", ((uint32_t) Arduino::millisWrapper()), lastErrorId, errorCnt); }
                        device.enableAccInterruptInDeepSleep();
                    }
                }
            }
            /** ---------------- NIGHT TIME STATE (IMU OFF!) ---------------- */
            else if(state == ST_NIGHT_TIME) {
                i2c.begin(I2C_FREQ_HZ_400KHZ);
                if(isTimeToGoToBed()) { // that would be a strange error (RTC should wake up when sleep time is over), but happened once
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("STILL BED TIME - SHOULD NEVER HAPPEN! Wake up by %d\n", device.getWakeUpReason()); }
                    lastErrorId = 111; errorCnt++;
                    fifoStopped = true; // just in case, set fifoStopped again
                    if(!device.rtc.setDailyInterrupt(NIGHTTIME_TURN_ON_HOUR, NIGHTTIME_TURN_ON_MINUTE)) { lastErrorId = 112; errorCnt++; } // just in case, set daily interrupt again, WARNING: NO AUTO CLEAR OF INTERRUPT FLAG!
                    device.enableRTCInterruptInDeepSleep(); // sleep until RTC wakes system up again
                }
                else {
                    device.blinkTimes(5, B_RED); // put BEFORE reactivation of fifo because at high sampling frequencies fifo might already be full after 5x blinky
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("State: Night time over -> restart IMU\n"); }
                    if(!device.rtc.resetInterruptFlags()) { lastErrorId = 105; errorCnt++; } // IMPORTANT: does not reset by itself!
                    if(!device.rtc.disableHourlyDailyInterrupt()) { lastErrorId = 106; errorCnt++; } // IMPORTANT: otherwise daily interrupt triggers again in tracking and stays on (not resetting itself)
                    if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 107; errorCnt++; } // init IMU again
                    if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 108; errorCnt++; }
                    if(!device.imu.initFIFOForAcc()) { lastErrorId = 109; errorCnt++; }
                    if(!device.imu.resetFIFO()) { lastErrorId = 110; errorCnt++; } // empty fifo, do not read it
                    fifoDataPointerRam = 0; // reset RAM data
                    fifoStopped = false;
                    state = ST_TRACK;
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                    device.enableAccInterruptInDeepSleep(); // wake up from acc interrupt
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
                    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be called again in dataTransmission, but not important
                    uint32_t freeSpaceBytesInFIFO = device.flash.fifoGetFreeSpace(flashBlockToSendNextPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES);
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: FIFO free space in memory: %d\n", freeSpaceBytesInFIFO); }
                    if(freeSpaceBytesInFIFO >= ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES) { // okay, enough memory free now, we are save -> restart
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: FIFO free space %d >= %d -> RESTART\n", freeSpaceBytesInFIFO, ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES); }
                        if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { lastErrorId = 13; errorCnt++; } // init IMU again
                        if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { lastErrorId = 14; errorCnt++; }
                        if(!device.imu.initFIFOForAcc()) { lastErrorId = 15; errorCnt++; }
                        if(!device.imu.resetFIFO()) { lastErrorId = 16; errorCnt++; } // empty fifo, do not read it
                        fifoDataPointerRam = 0; // reset RAM data
                        fifoStopped = false;
                        state = ST_TRACK;
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                        device.enableAccInterruptInDeepSleep(); // wake up from acc interrupt
                    }
                    else { // transmit more before exiting this mode!
                        // turn on flash power already (5ms)
                        if(!device.flashPowerOn(true)) { lastErrorId = 10; errorCnt++; }

                        bool somethingTransmitted = false;
                        uint32_t timeNow;

                        device.ledGreenOn();

                        #ifdef TRANSMISSION_METHOD_ESP_NOW
                        // esp now data transmission (FORCE mode!)
                        timeNow = ((uint32_t) Arduino::millisWrapper());
                        somethingTransmitted = dataTransmissionEspNow(true, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer);
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("ESP NOW needed %dms\n", ((uint32_t) Arduino::millisWrapper()) - timeNow); }
                        #endif
                        
                        #ifdef TRANSMISSION_METHOD_WIFI
                        // wifi data transmission (FORCE mode!)
                        timeNow = ((uint32_t) Arduino::millisWrapper());
                        somethingTransmitted = dataTransmissionWifi(true, timestamp, device.readSupplyVoltageFromWakeStub(), flashPointer, DATATR_WIFI_SSIDS, DATATR_WIFI_PASSWORDS, DATATR_KNOWN_WIFI_LIST_SIZE);
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("WIFI needed %dms\n", ((uint32_t) Arduino::millisWrapper()) - timeNow); }
                        #endif

                        device.ledGreenOff();

                        // turn off flash
                        if(!device.flashPowerOff()) { lastErrorId = 12; errorCnt++; } 

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
                        timestamp = device.rtc.getTimestamp(error); // get RTC timestamp AGAIN because after wifi transmission it might be totally different
                        uint32_t sleepyTime = calculateMemoryFullSleepTime(timestamp, memFullWakeUpOnMinute); 

                        device.enableInternalTimerInterruptInDeepSleep(sleepyTime); // sleep some time before trying again transmission
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("State: next transmission in %ds, memFullWakeUpOnMinute %d\n", sleepyTime, memFullWakeUpOnMinute); }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                    }
                    #else
                    device.blink(B_GREEN, B_GREEN); // no data transmission, just blink from time to time
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
                    uint16_t flashBlockToSendNextPointer = device.nvsReadUINT16(NVS_FLASH_SEND_NEXT_BLOCK_POINTER); // will be called again in dataTransmission, but not important
                    uint16_t blocksToTransmit = device.flash.fifoGetNumberOfPopableBlocks(flashBlockToSendNextPointer, flashPointer);
                    if(TRACKER_MODE == MODE_TESTRUN) { printf("Flash: flashPointer: %d, flashOffsetPointer: %d, flashBlockToSendNextPointer: %d, blocksToTransmit: %d\n", flashPointer, flashOffsetPointer, flashBlockToSendNextPointer, blocksToTransmit); }
                    if(blocksToTransmit == 0) { // ALL memory free now -> go back to time state
                        if(TRACKER_MODE == MODE_TESTRUN) {
                            printf("Flash: blocks to transmit ZERO\n");
                            uint32_t freeSpaceBytesInFIFO = device.flash.fifoGetFreeSpace(flashBlockToSendNextPointer, flashPointer, flashOffsetPointer, MT29_NUMBER_PAGES);
                            uint32_t remainingBytesInMemory = MT29_NUMBER_BYTES - freeSpaceBytesInFIFO;
                            printf("Flash: but still %d bytes in flash memory (no full block, so can't be transmitted -> next time)\n", remainingBytesInMemory);
                        }
                        device.blinkTimes(10, B_BOTH); // blink 10 times before going to sleep FOREVER, no wake up interrupt!!!
                        // SLEEP FOREVER
                    }
                    else { // transmit more before exiting this mode!
                        // turn on flash power already (5ms)
                        if(!device.flashPowerOn(true)) { lastErrorId = 10; errorCnt++; }

                        bool somethingTransmitted = false;
                        uint32_t timeNow;

                        device.ledRedOn(); // turn on led during transmission

                        #ifdef TRANSMISSION_METHOD_ESP_NOW
                        // esp now data transmission (FORCE mode!)
                        timeNow = ((uint32_t) Arduino::millisWrapper());
                        somethingTransmitted = dataTransmissionEspNow(true, 0, device.readSupplyVoltageFromWakeStub(), flashPointer);
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("ESP NOW needed %dms\n", ((uint32_t) Arduino::millisWrapper()) - timeNow); }
                        #endif
                        
                        #ifdef TRANSMISSION_METHOD_WIFI
                        // wifi data transmission (FORCE mode!, IMPORTANT: only use FORCE_DOWNLOAD_WIFI_SSID)
                        timeNow = ((uint32_t) Arduino::millisWrapper());
                        somethingTransmitted = dataTransmissionWifi(true, 0, device.readSupplyVoltageFromWakeStub(), flashPointer, TIME_WIFI_SSIDS, TIME_WIFI_PASSWORDS, 1); // IMPORTANT: 1 = only use FORCE_DOWNLOAD_WIFI_SSID in TIME_WIFI_SSIDS list
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("WIFI needed %dms\n", ((uint32_t) Arduino::millisWrapper()) - timeNow); }
                        #endif

                        device.ledRedOff(); // turn off led during transmission

                        // turn off flash
                        if(!device.flashPowerOff()) { lastErrorId = 12; errorCnt++; }

                        device.enableInternalTimerInterruptInDeepSleep(5); // sleep some time before trying again transmission
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("State: something transmitted: %d -> anyhow restart in 5s\n", somethingTransmitted); }
                        if(TRACKER_MODE == MODE_TESTRUN) { printf("LAST ERROR ID: %d (CNT %d)\n", lastErrorId, errorCnt); }
                    }
                    #else
                    device.blink(B_RED, B_RED); // no data transmission, just blink from time to time
                    device.enableInternalTimerInterruptInDeepSleep(30); // sleep some time before blinking again
                    #endif                    
                }
            }
        }
        else { // not in productive or test run
            if(TRACKER_MODE == MODE_SELFTEST) { printf("Mode: SELFTEST\n");  device.selfTest(SELFTEST_VOLTAGE_REF, SELFTEST_PARAMETERS); }
            else if(TRACKER_MODE == MODE_READFLASH) { printf("Mode: READ FLASH\n"); readFullFlash(); }
            else if(TRACKER_MODE == MODE_MOCK_FLASH_STATE) { printf("Mode: MOCK FLASH STATE\n"); mockFlashState(); }
        }
        startCnt++;
        device.deepSleep();
    }
}