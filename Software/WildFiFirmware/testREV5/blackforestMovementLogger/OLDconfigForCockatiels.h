#ifndef configFor_h
#define configFor_h

#include "PlatformWildFiTagREV5.h"                                                      // for BMX160_ACCEL_ODR_200HZ, ..
#include "configGeneralSettings.h"                                                      // for FORCE_DOWNLOAD_WIFI_SSID and FORCE_DOWNLOAD_WIFI_PASSWORD

/** --- CURRENT TRACKER MODE: MODE_TESTRUN, MODE_PRODUCTIVE, MODE_SELFTEST, ... --- */
#define TRACKER_MODE                                    MODE_PRODUCTIVE
#define TRACKING_DATA_MODE                              TRACKING_DATA_MODE_ACC_ONLY     // TRACKING_DATA_MODE_1HZ_GPS_AND_ACC or TRACKING_DATA_MODE_ACC_ONLY

/** Selftest mode parameters, all: SELFTEST_VOLTAGE | SELFTEST_LEDS | SELFTEST_HALLSENSOR | SELFTEST_I2C | SELFTEST_RTC | SELFTEST_ACCFOC_CHECK | SELFTEST_ACCFOC_EXECUTE_IF_UNSET | SELFTEST_BARO | SELFTEST_FLASH_BAD_BLOCKS | SELFTEST_FLASH_READ_WRITE | SELFTEST_FLASH_FULL_ERASE | SELFTEST_NVS_RESET | SELFTEST_CPU_CLOCK_DOWN | SELFTEST_WIFI_SCAN | SELFTEST_ESPNOW_BROADCAST */
#define SELFTEST_PARAMETERS                             SELFTEST_VOLTAGE | SELFTEST_LEDS | SELFTEST_HALLSENSOR | SELFTEST_I2C | SELFTEST_RTC | SELFTEST_ACCFOC_CHECK | SELFTEST_ACCFOC_EXECUTE_IF_UNSET | SELFTEST_BARO | SELFTEST_FLASH_FULL_ERASE | SELFTEST_NVS_RESET | SELFTEST_WIFI_SCAN

/** Data transmission to use */
#define TRANSMISSION_METHOD                             TRANSMISSION_METHOD_WIFI        // how to transmit data

/** INDIVIDUAL SETTINGS - PLEASE CHANGE FOR EACH TAG */
#if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
#define TAG_NAME                                        "COCK03"                        // ALWAYS 6 CHARACTERS! otherwise REST_ADD_HEADER_VALUE not correct, unique id for post calls
#endif

/** Acc configuration setting */
#define ACC_FREQUENCY                                   BMX160_ACCEL_ODR_200HZ
acc_config_t accConfig = {
                                                        ACC_FREQUENCY,                  // BMX160_ACCEL_ODR_50HZ, acc frequency
                                                        BMX160_ACCEL_BW_RES_AVG2,       // BMX160_ACCEL_BW_RES_AVG8, acc averaging cycles
                                                        BMX160_ACCEL_RANGE_2G           // BMX160_ACCEL_RANGE_2G, acc range (WARNING: changes meaning of LSB value in data)
};

/** IMU data temporary in RAM (DEPENDING ON ACC FREQUENCY!) */
#if ACC_FREQUENCY == BMX160_ACCEL_ODR_200HZ
    #define ACC_INTERRUPT_WATERMARK                     918                             // imu is generating interrupt when fifo is at that fill level
#else
    #define ACC_INTERRUPT_WATERMARK                     960                             // fine for <= 50Hz, other stuff not tested, imu is generating interrupt when fifo is at that fill level
#endif

/** Night time mode (UTC TIME!) */
#define NIGHTTIME_ACTIVATED                             true                            // when activated then loggers stops logging at certain time of day
#define NIGHTTIME_TURN_ON_HOUR                          7                               // start again logging at this UTC hour
#define NIGHTTIME_TURN_ON_MINUTE                        0                               // start again logging at this UTC minute
#define NIGHTTIME_TURN_OFF_HOUR                         15                              // stop logging at this UTC hour
#define NIGHTTIME_TURN_OFF_MINUTE                       30                              // stop logging at this UTC minute

/** Battery protection */
#define BATT_MIN_VOLTAGE                                3550                            // hibernate below that voltage and
#define BATT_RESTART_VOLTAGE                            3650                            // wait until voltage above that threshold again
#define FIRST_UNDER_VOLTAGE_SLEEP_TIME                  1*3600                          // 1*3600, first time UV detected -> stop imu fifo and sleep for that time
#define FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME              12*3600                         // 12*3600, second and more times UV detected -> sleep for that time

/** IMU data temporary in RAM (not for continous tracking 1Hz mode in light sleep) */
#define ACC_RAM_SIZE_1                                  996                             // ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2 = RTC memory for storing fifo data in wake stub
#define ACC_RAM_SIZE_2                                  4                               // see above

/** Time over WIFI Configuration (immediately after wake-up) on channel 6, then 1, then 11, each for 120ms */
const uint8_t TIME_WIFI_LIST_SIZE =                     2;                              // should include time wifi
const char* TIME_WIFI_SSIDS[TIME_WIFI_LIST_SIZE] =      { FORCE_DOWNLOAD_WIFI_SSID, "mpitime" };        // wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* TIME_WIFI_PASSWORDS[TIME_WIFI_LIST_SIZE] =  { FORCE_DOWNLOAD_WIFI_PASSWORD, "87654321" };   // wifi password for activation/timestamp                  
#define TIME_WIFI_OUTPUT_POWER                          RADIO_MAX_TX_11_DBM             // 19.5dBm will brown out on REV3
#define TIME_BETWEEN_GET_TIME_RETRIES_SECONDS           120                             // 120 = 660uA, if not activated: sleep for that time, wake up, check if wifi there and try to activate, 120 = 234uA average

/** Activation over WIFI Configuration (only start if this wifi has been seen) -> every 60min = 35.3uA (channels 1, 6, 11 each 120ms), every 60min = 35.1uA (channels 1-11 each 30ms), every 60min = 26.6uA (only one channel) */
#define ACTIVATION_MODE                                 ACTIVATION_MODE_SKIP            // ACTIVATION_MODE_SKIP or ACTIVATION_MODE_STORE_PERMANENTLY or ACTIVATION_MODE_ON_EVERY_START: how shall the tag be activated
const char* ACTIVATION_WIFI_SSIDS[1] =                  { "mpistart" };                 // mpistart, wifi name to scan for at beginning -> if found -> stark tracking (NO PASSWORD NEEDED)
#define ACTIVATION_WIFI_OUTPUT_POWER                    RADIO_MAX_TX_11_DBM             // RADIO_MAX_TX_11_DBM, 19.5dBm will brown out on REV3
#define ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE           0                               // on which minute shall be checked (40 = 14:40, 15:40, ..)
#define ACTIVATION_WAKE_UP_ON_MODULO_HOURS              1                               // 1 = every hour, 2 = on even hours (14, 16, ..), 3 = on all hours that are dividable by 3, ..

/** Configuration Data Transmission: Common */
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY       1*60                            // 12*60 = 00:00 and 12:00 (UTC), 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission nothing transmitted -> re-try more seldomly
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY     2                               // 1 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission something transmitted -> re-try more often

/** Memory Full Configuration */
#define ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES       2048*64*2048                    // only exiting state when at least that memory is free -> 2048*64*2048 = until memory totally free except the one block that is not completely full (2048*64*2048 will never be reached actually - minBlocksToTransmit will be 0)
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY       1*60                            // 6*60, 12*60 = 00:00 and 12:00 (UTC), 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission nothing transmitted -> re-try more seldomly
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_LESS_FREQU     5                               // last data transmission something transmitted, current data transmission NOTHING -> use this minute interval for one time
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY     1                               // 1, 12hrs = 60*12 = 720 = 00:00 and 12:00, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission something transmitted -> re-try more often

/** Configuration Data Transmission: ESP NOW */
#if TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW
#define ESPNOW_OUTPUT_POWER                             RADIO_MAX_TX_2_DBM              // 19.5dBm will brown out on REV3
#define ESPNOW_DATA_RATE                                WIFI_PHY_RATE_18M               // well tested with 18M
#define ESPNOW_LONG_RANGE                               0                               // long range only with 1MBPS -> ESPNOW_DATA_RATE should be 1MBPS!
#define ESPNOW_MIN_BATT_VOLTAGE                         3500                            // 3500, don't start scan if battery voltage too low
#define ESPNOW_MIN_BATT_VOLTAGE_DURING_TRANSM           3200                            // 3200, don't continue transmission if voltage dropped below that threshold after one block transmission
#define ESPNOW_MIN_BYTES_TO_TRANSMIT                    2048                            // only start scan (and data transmission) if there are enough data bytes to send
#define ESPNOW_MAX_BYTES_TO_TRANSMIT                    ESP_NOW_FLASH_STREAM_NO_LIMIT   // maximum bytes to transmit in one go
#endif

/** Configuration Data Transmission: WIFI */
#if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
#define WIFI_OUTPUT_POWER                               RADIO_MAX_TX_11_DBM             // 19.5dBm will brown out on REV3
#define WIFI_MIN_BATT_VOLTAGE                           3500                            // don't start scan if battery voltage too low
#define WIFI_MIN_BATT_VOLTAGE_DURING_TRANSM             3200                            // don't continue wifi transmission if voltage dropped below that threshold after one block transmission
#define WIFI_MAX_POST_TASK_TIME_SECONDS                 45                              // max time for sending one successfully transmitted block
#define WIFI_MIN_BLOCKS_TO_TRANSMIT                     10                              // only start scan (and data transmission) if there is enough data to send
#define WIFI_MAX_BLOCKS_TO_TRANSMIT                     200                             // during one connection, burst maximum this amount of blocks (maximum memory blocks = 2048)
const uint8_t DATATR_KNOWN_WIFI_LIST_SIZE =             1;                              // IMPORTANT: DO NOT FORGET TO CHANGE THAT!!!!!!!!!
const char* DATATR_WIFI_SSIDS[DATATR_KNOWN_WIFI_LIST_SIZE] = {                                        // first priority: channel (6, 1, 11), within channel the name
                                                        /*"guest", "LiWoAb New",*/ "mpidata"
};                      
const char* DATATR_WIFI_PASSWORDS[DATATR_KNOWN_WIFI_LIST_SIZE] = {                                    // first priority: channel (6, 1, 11), within channel the name
                                                        /*"xxxxxxx", "xxxxxxx",*/ "87654321"
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
const char* REST_ADD_HEADER_VALUE =                     "131099"; // 131099 for full blocks, 65563 for half blocks, strlen(PREFIX_CONSTRUCTED: "ABCDEF:PPPPPPPP:VVVV:") 21 + strlen(taskParams.postfix) 6 + (PAGES_IN_ONE_GO * 2048 = 65536)
#define REST_USE_BASE64_ENCODING                        false
#define REST_SEND_FULL_BLOCKS                           true
#define REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX            true
const char* REST_PAYLOAD_PREFIX =                       TAG_NAME ":PPPPPPPP:VVVV:"; // P = page address, V = battery voltage
const char* REST_PAYLOAD_POSTFIX =                      TAG_NAME;
post_task_stream_flash_parameters_t restStreamParams;
uint8_t *dmaBuffer2048Bytes = NULL;
uint8_t *additionalDataAfterPrefixPointer = NULL;
#endif

#endif