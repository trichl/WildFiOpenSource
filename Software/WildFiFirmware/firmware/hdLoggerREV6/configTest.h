#ifndef configFor_h
#define configFor_h

#include "PlatformWildFiTagREV6.h"                                                      // for BMX160_ACCEL_ODR_200HZ, ..
#include "configGeneralSettings.h"                                                      // for FORCE_DOWNLOAD_WIFI_SSID and FORCE_DOWNLOAD_WIFI_PASSWORD

#define WILDFI_CONFIG_NAME                              "TESTIE"
#define WILDFI_CONFIG_VERSION                           10

/** --- CURRENT TRACKER MODE: MODE_TESTRUN, MODE_PRODUCTIVE, MODE_SELFTEST, MODE_MOCK_FLASH_STATE ... --- */
#define TRACKER_MODE                                    MODE_TESTRUN
#define TRACKING_DATA_MODE                              TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP // select tracking firmware: TRACKING_DATA_MODE_1HZ_GPS_AND_IMU or TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP or TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP
#define DO_THE_BLINK                                    true

/** Data transmission to use */
#define TRANSMISSION_METHOD                             TRANSMISSION_METHOD_WIFI     // how to transmit data, TRANSMISSION_METHOD_ESP_NOW or TRANSMISSION_METHOD_WIFI

/** TRACKING_DATA_MODE_1HZ_GPS_AND_ACC settings */
#define TRACKING_DATA_MODE_1HZ_NO_TIME_TIMEOUT_SECONDS  400                             // 300, when skip getting time: timeout when first trying to get time (before that night time not possible)
#define TRACKING_DATA_MODE_1HZ_NO_TIME_SLEEP_SECONDS    60                              // 3600, did not get very first time -> sleep for that amount until trying it again to not drain the battery
#define TRACKING_DATA_MODE_1HZ_GPS_NOT_RESPONDING       3600                            // 3600, if GPS detached during operation this will define the time until tag tries again to start GPS
#define TRACKING_DATA_MODE_1HZ_GPS_BLINK                true                            // only valid if DO_THE_BLINK = true, blink shortly (red = no fix, green = got fix) if GPS is active
#define TRACKING_DATA_MODE_1HZ_GPS_FITNESS_LOW_POWER    true                            // set fitness low power mode at start

/** INDIVIDUAL SETTINGS - PLEASE CHANGE FOR EACH TAG */
#if TRANSMISSION_METHOD == TRANSMISSION_METHOD_WIFI
#define TAG_NAME                                        "DOG001"                        // ALWAYS 6 CHARACTERS! otherwise REST_ADD_HEADER_VALUE not correct, unique id for post calls
#endif

/** IMU configuration setting (acc can't be turned off) */
#define ACC_FREQUENCY                                   BMX160_ACCEL_ODR_800HZ
acc_config_t accConfig = {
                                                        ACC_FREQUENCY,                  // BMX160_ACCEL_ODR_50HZ, acc frequency
                                                        BMX160_ACCEL_BW_RES_AVG1,       // BMX160_ACCEL_BW_RES_AVG8, acc averaging cycles
                                                        BMX160_ACCEL_RANGE_8G           // BMX160_ACCEL_RANGE_8G, acc range (WARNING: changes meaning of LSB value in data)
};

#define USE_MAGNETOMETER                                false
mag_config_t magConfig = {                              BMX160_MAG_ODR_25HZ,
                                                        BMX160_MAG_ACCURACY_REGULAR
};

#define USE_GYRO                                        false
gyro_config_t gyroConfig = {                            BMX160_GYRO_ODR_25HZ,
                                                        BMX160_GYRO_RANGE_125_DPS,
                                                        BMX160_GYRO_BW_NORMAL_MODE
};

/** Night time mode (UTC TIME!) */
#define NIGHTTIME_ENTER                                 NIGHTTIME_USE_HOURS             // NIGHTTIME_ALWAYS_NIGHT or NIGHTTIME_USE_HOURS or NIGHTTIME_DISABLED -> when activated then logger stops logging at certain time of day
#define NIGHTTIME_MODE                                  NIGHTTIME_MODE_TRY_DATATRANS    // NIGHTTIME_MODE_ONLY_SLEEP or NIGHTTIME_MODE_TRY_DATATRANS
// ONLY FOR NIGHTTIME_ENTER = NIGHTTIME_USE_HOURS
#define NIGHTTIME_TURN_ON_HOUR                          5                               // start again logging at this UTC hour
#define NIGHTTIME_TURN_ON_MINUTE                        0                               // start again logging at this UTC minute
#define NIGHTTIME_TURN_OFF_HOUR                         20                              // stop logging at this UTC hour
#define NIGHTTIME_TURN_OFF_MINUTE                       0                               // stop logging at this UTC minute
// ONLY FOR NIGHTTIME_MODE = NIGHTTIME_MODE_TRY_DATATRANS
const uint8_t NIGHT_TIME_DATA_TRANS_DEEP_NIGHT_HOURS[11] = { 19, 20, 21, 22, 23, 0, 1, 2, 3, 4 }; // hours when not trying to establish data connection
#define NIGHTTIME_MODE_TRY_DATATRANS_WAKEUP_SECONDS     60                              // 180, in seconds, MAX 4096, ESPNOW_CUSTOM_RF_CALIBRATION = true and GPS backup current: 900 = every 15min: 28uA, 300 = every 5min: 48.4uA
#define NIGHTTIME_MODE_TRY_DATATRANS_MODULO             2                               // 3, only every X times of wake ups (NIGHTTIME_MODE_TRY_DATATRANS_EVERY_SECONDS) try data transmission, otherwise just check if night time is over

/** Selftest mode parameters */
#define SELFTEST_PARAMETERS                             SELFTEST_VOLTAGE | SELFTEST_LEDS | SELFTEST_HALLSENSOR | SELFTEST_I2C | SELFTEST_RTC | SELFTEST_ACC_GYRO_FOC_CHECK | SELFTEST_ACC_GYRO_FOC_EXECUTE_IF_UNSET /*| SELFTEST_ACC_GYRO_FOC_FORCE_EXECUTION*/ | SELFTEST_BARO | /*SELFTEST_FLASH_BAD_BLOCKS |*/ /*SELFTEST_FLASH_READ_WRITE |*/ SELFTEST_FLASH_FULL_ERASE | SELFTEST_NVS_RESET | /*SELFTEST_CPU_CLOCK_DOWN |*/ /*SELFTEST_WIFI_SCAN |*/ SELFTEST_ESPNOW_BROADCAST
#define SELFTEST_GPS_SET_BAUDRATE_PERMANTENLY           true                            // trying to set baud rate of GPS to 115200 permanently
#define SELFTEST_VOLTAGE_REF                            3750                            // during selftest routine (if voltage check enabled): check if this value (from power supply) is actually measured

/** Undervoltage mode: battery protection */
#define POWER_DOWN_MODE_DATA_TRANS_WHEN_VOLTAGE_HALF_OK true                            // true, try a data transmission if voltage higher than BATT_MIN_VOLTAGE + ((BATT_RESTART_VOLTAGE - BATT_MIN_VOLTAGE) / 2)
#define BATT_MIN_VOLTAGE                                3500                            // hibernate below that voltage and
#define BATT_RESTART_VOLTAGE                            3650                            // wait until voltage above that threshold again
#define FIRST_UNDER_VOLTAGE_SLEEP_TIME                  1*3600                          // 1*3600, first time UV detected -> stop imu fifo and sleep for that time
#define FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME              12*3600                         // 12*3600, second and more times UV detected -> sleep for that time

/** Time over WIFI or GPS Configuration (immediately after wake-up) on channel 6, then 1, then 11, each for 120ms */
#define SKIP_GET_TIME                                   false                           // true, starting without any time (get it maybe later over GPS)
// ONLY IF NOT SKIP_GET_TIME
#define USE_GPS_TO_GET_TIME_AT_BEGINNING                true                            // true, use not only WiFi but also GPS for getting the time at the beginning
#define USE_GPS_TO_GET_TIME_TIMEOUT_SECONDS             300                             // 300, timeout to only get time at very first start
const uint8_t TIME_WIFI_LIST_SIZE =                     3;                              // should include time wifi
const char* TIME_WIFI_SSIDS[TIME_WIFI_LIST_SIZE] =      { FORCE_DOWNLOAD_WIFI_SSID, "mpitime", "RodelbahnSoelden" }; // wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* TIME_WIFI_PASSWORDS[TIME_WIFI_LIST_SIZE] =  { FORCE_DOWNLOAD_WIFI_PASSWORD, "87654321", "xxxxxx" }; // wifi password for activation/timestamp                  
#define TIME_WIFI_OUTPUT_POWER                          RADIO_MAX_TX_11_DBM             // 19.5dBm will brown out on REV3
#define TIME_BETWEEN_GET_TIME_RETRIES_SECONDS           60                              // 3600, 120 = 660uA, if not activated: sleep for that time, wake up, check if wifi there and try to activate, 120 = 234uA average

/** Activation */
#define ACTIVATION_MODE                                 ACTIVATION_MODE_SKIP            // ACTIVATION_MODE_SKIP or ACTIVATION_MODE_STORE_PERMANENTLY or ACTIVATION_MODE_ON_EVERY_START: how shall the tag be activated
#define ACTIVATION_SOURCE                               ACTIVATION_BY_ESPNOW            // ACTIVATION_BY_ESPNOW or ACTIVATION_BY_WIFI
// ONLY FOR ACTIVATION_BY_WIFI (only start if this wifi has been seen) -> every 60min = 35.3uA (channels 1, 6, 11 each 120ms), every 60min = 35.1uA (channels 1-11 each 30ms), every 60min = 26.6uA (only one channel)
const char* ACTIVATION_WIFI_SSIDS[1] =                  { "mpistart" };                 // mpistart, wifi name to scan for at beginning -> if found -> stark tracking (NO PASSWORD NEEDED)
#define ACTIVATION_WIFI_OUTPUT_POWER                    RADIO_MAX_TX_11_DBM             // RADIO_MAX_TX_11_DBM, 19.5dBm will brown out on REV3
#define ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE           0                               // on which minute shall be checked (40 = 14:40, 15:40, ..)
#define ACTIVATION_WAKE_UP_ON_MODULO_HOURS              1                               // 1 = every hour, 2 = on even hours (14, 16, ..), 3 = on all hours that are dividable by 3, ..
// ONLY FOR ACTIVATION_BY_ESPNOW
#define ACTIVATION_BY_ESPNOW_RETRY_TIME_SECONDS         1200                            // ESPNOW_CUSTOM_RF_CALIBRATION = true: 300 = every 5min: 44.6uA, 900 = every 15min: 27.5uA -> ESPNOW_CUSTOM_RF_CALIBRATION = false: 300 = every 5min: 109uA, 900 = every 15min: 48.6uA

/** Configuration Data Transmission: Common */
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY       1*60                            // 12*60 = 00:00 and 12:00 (UTC), 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission nothing transmitted -> re-try more seldomly
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY     1                               // 1 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission something transmitted -> re-try more often

/** Memory Full Configuration */
#define ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES       2048*64*60                      // 2048*64*60 = 60 blocks transmitted, only exiting state when at least that memory is free -> 2048*64*2048 = until memory totally free except the one block that is not completely full (2048*64*2048 will never be reached actually - minBlocksToTransmit will be 0)
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY       1*60                            // 1*60, 12*60 = 00:00 and 12:00 (UTC), 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission nothing transmitted -> re-try more seldomly
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_LESS_FREQU     2                               // last data transmission something transmitted, current data transmission NOTHING -> use this minute interval for one time
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY     1                               // 1, 12hrs = 60*12 = 720 = 00:00 and 12:00, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission something transmitted -> re-try more often

/** IMU data temporary in RAM (DEPENDING ON ACC FREQUENCY!) */
#define ACC_INTERRUPT_WATERMARK                         960                             // 960 fine for <= 50Hz, 918 for 200Hz (IMU only), other stuff not tested, imu is generating interrupt when fifo is at that fill level

/** IMU data temporary in RAM (not for continous tracking 1Hz mode in light sleep) */
#define ACC_RAM_SIZE_1                                  996                             // ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2 = RTC memory for storing fifo data in wake stub
#define ACC_RAM_SIZE_2                                  6                               // see above

/** Configuration Data Transmission: ESP NOW */
#define ESPNOW_CUSTOM_RF_CALIBRATION                    true                            // WARNING: only works if ACC_RAM_SIZE_1 + ACC_RAM_SIZE_2 is small enough! only 4kByte of RTC RAM
#define ESPNOW_CUSTOM_RF_FULL_CALIB_EVERY_X_TIMES       50                              // 50, max. 255, full RF calibration (around 150ms) every X times when trying to do anything with ESP NOW
#define ESPNOW_GATEWAY_AROUND_LISTENING_TIME            50                              // 50, also for data transmission, gateway broadcasts every 20ms

#define COMMAND_BYTE_FORCE_TRACKING_DURATION_SECONDS    900                             // 1800, around 88 seconds @12.5Hz until this threshold will be checked
#define COMMAND_BYTE_FORCE_TRACKING_MAX                 20                              // this is the maximum amount of times a forced tracking can be issued per day

#if (TRANSMISSION_METHOD == TRANSMISSION_METHOD_ESP_NOW) || (ACTIVATION_SOURCE == ACTIVATION_BY_ESPNOW)
#define ESPNOW_OUTPUT_POWER                             RADIO_MAX_TX_19_5_DBM           // RADIO_MAX_TX_19_5_DBM, 19.5dBm will brown out on REV3
#define ESPNOW_DATA_RATE                                WIFI_PHY_RATE_18M               // WIFI_PHY_RATE_18M, well tested with 18M
#define ESPNOW_LONG_RANGE                               0                               // 0, long range only with 1MBPS -> ESPNOW_DATA_RATE should be 1MBPS!
#define ESPNOW_MIN_BATT_VOLTAGE                         3500                            // 3500, don't start scan if battery voltage too low
#define ESPNOW_MIN_BATT_VOLTAGE_DURING_TRANSM           3200                            // 3200, don't continue transmission if voltage dropped below that threshold after some transmission
#define ESPNOW_MIN_BYTES_TO_TRANSMIT                    1                               // while tracking: only start scan (and data transmission) if there are enough data bytes to send
#define ESPNOW_MAX_BYTES_TO_TRANSMIT                    ESP_NOW_FLASH_STREAM_NO_LIMIT   // maximum bytes to transmit in one go, ESP_NOW_FLASH_STREAM_NO_LIMIT = no limit
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
const char* REST_URL =                                  RESTDB_ENDPOINT2;
const char* REST_CONTENT_TYPE =                         "application/json";
const char* REST_ADD_HEADER_KEY =                       "x-apikey";
const char* REST_ADD_HEADER_VALUE =                     RESTDB_APIKEY1;
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
uint8_t *additionalDataAfterPrefixPointer = NULL;
#endif

#endif