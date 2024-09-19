#ifndef configFor_h
#define configFor_h

#include "PlatformWildFiTagREV6.h"


// OLD: 0.78 Hz
// 24uA average power consumption in active and productive (no data transmission)
// 0.78Hz = FIFO full after 3min 30s, buffer full after 24min 47s (7 x read-out)
// Timestamp, pressure, temperature, humidity every 24min 47s
// Activity check to inactive every 24min 47s (average ODBA of last 3min 30s)
// Average ODBA after 3min 30s no movement: 3-4 of 120
// -> 22.2 uA average power consumption when inactive, so no big advantage to not record -> record continuously

// NEW: 3.12 Hz
// 49uA average power consumption in active and productive (no data transmission)
// 3.12Hz = FIFO full after 53s, buffer full after 6min 12s (7 x read-out)
// Timestamp, pressure, temperature, humidity every 6min 12s
// Activity check to inactive every 6min 12s (average ODBA of last 3min 30s)
// Data estimation: 3.12*6*3600*24 = 1.54 MByte per day -> 165 days until memory full

// Activation every hour (stored permanently)
// In active: data transmission every 24 hours
// In inactive: data transmission every 6 hours -> decided to turn activation enabling OFF!
// In night time (below 3.70V): every 24 hours (wake-up every hour)

// TODOS:
// - NOTHING -

#define WILDFI_CONFIG_NAME                              "Development"
#define WILDFI_CONFIG_VERSION                           99

/** --- CURRENT TRACKER MODE: MODE_TESTRUN, MODE_PRODUCTIVE, MODE_SELFTEST, MODE_MOCK_FLASH_STATE ... --- */
#define DO_THE_BLINK                                    true                            // [CONFIG]
#define TRACKER_MODE                                    MODE_TESTRUN                    // [CONFIG]
#define TRACKING_DATA_MODE                              TRACKING_DATA_MODE_1HZ_GPS_AND_IMU // [CONFIG] select tracking firmware: TRACKING_DATA_MODE_1HZ_GPS_AND_IMU or TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP or TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP or TRACKING_DATA_MODE_DEAD_RECKONING

/** Data transmission to use */
#define TRANSMISSION_METHOD                             TRANSMISSION_METHOD_ESP_NOW     // [CONFIG] how to transmit data, TRANSMISSION_METHOD_ESP_NOW or TRANSMISSION_METHOD_WIFI

/** IMU data temporary in RAM (not for continous tracking 1Hz mode in light sleep) */
#define RTC_RAM_BUFFER                                  6000                            // RTC memory for storing fifo data for deepSleepTracking and deadReckoning

/** TRACKING_DATA_MODE_DEAD_RECKONING settings */
#define TRACKING_DATA_MODE_DR_GPS_MAX_TIME              300                             // 300, stop GPS fix attempt after that time
#define TRACKING_DATA_MODE_DR_GPS_NOT_RESPONDING        3600                            // 3600, if GPS detached during operation this will define the time until tag tries again to start GPS
#define TRACKING_DATA_MODE_DR_GPS_BLINK                 true                            // true, only valid if DO_THE_BLINK = true, blink shortly (red = no fix, green = got fix) if GPS is active
#define TRACKING_DATA_MODE_DR_MIN_HDOP_X_TEN            30                              // [CONFIG] 30, minimum HDOP when to stop fix attempt
#define TRACKING_DATA_MODE_DR_HDOP_WAIT_SECONDS         8                               // 8, how many seconds shall be waited to achieve TRACKING_DATA_MODE_DR_MIN_HDOP_X_TEN
#define TRACKING_DATA_MODE_DR_IMU_SECONDS               1800                            // [CONFIG] 60, after fix: how many seconds shall IMU run standalone (GPS off)
#define RTC_RAM_BUFFER_DR                               RTC_RAM_BUFFER                  // RTC_RAM_BUFFER, can be smaller than RTC RAM (NEVER bigger), if during IMU sampling more frequent header data is better (e.g., environmental data) 

/** TRACKING_DATA_MODE_1HZ_GPS_AND_IMU settings */
#define TRACKING_DATA_MODE_1HZ_NO_TIME_TIMEOUT_SECONDS  300                             // 400, when skip getting time: timeout when first trying to get time (before that night time not possible)
#define TRACKING_DATA_MODE_1HZ_NO_TIME_SLEEP_SECONDS    300                              // 1800, did not get very first time -> sleep for that amount until trying it again to not drain the battery
#define TRACKING_DATA_MODE_1HZ_GPS_NOT_RESPONDING       300                              // 3600, if GPS detached during operation this will define the time until tag tries again to start GPS
#define TRACKING_DATA_MODE_1HZ_GPS_BLINK                true                            // true, only valid if DO_THE_BLINK = true, blink shortly (red = no fix, green = got fix) if GPS is active
#define TRACKING_DATA_MODE_1HZ_GPS_FITNESS_LOW_POWER    false                           // false, WARNING: DOES NOT WORK, set fitness low power mode at start
#define TRACKING_DATA_MODE_1HZ_INTERRUPT_AFTER_SECONDS  60                             // [CONFIG] 0, interrupt logging after X seconds and try a data transmission (0 = do not interrupt)

/** IMU configuration setting (acc can't be turned off) */
#define ACC_FREQUENCY                                   BMX160_ACCEL_ODR_50HZ         // [CONFIG] BMX160_ACCEL_ODR_50HZ, acc frequency
#define ACC_AVG                                         BMX160_ACCEL_BW_RES_AVG8        // [CONFIG] BMX160_ACCEL_BW_RES_AVG8, acc averaging cycles
#define ACC_RANGE                                       BMX160_ACCEL_RANGE_8G           // [CONFIG] BMX160_ACCEL_RANGE_8G, acc range (WARNING: changes meaning of LSB value in data)

#define MAG_FREQUENCY                                   BMX160_MAG_ODR_50HZ             // [CONFIG]
#define MAG_ACCURACY                                    BMX160_MAG_ACCURACY_ENHANCED    // [CONFIG]

#define GYRO_FREQUENCY                                  BMX160_GYRO_ODR_50HZ            // [CONFIG]
#define GYRO_RANGE                                      BMX160_GYRO_RANGE_500_DPS       // [CONFIG]
#define GYRO_MODE                                       BMX160_GYRO_BW_OSR4_MODE        // [CONFIG]

#define USE_MAGNETOMETER                                false                            // [CONFIG]
#define USE_GYRO                                        false                            // [CONFIG]

/** Night time mode (UTC TIME!) */
#define NIGHTTIME_ENTER                                 NIGHTTIME_ONLY_BELOW_370V       // [CONFIG] NIGHTTIME_ALWAYS_NIGHT or NIGHTTIME_USE_HOURS or NIGHTTIME_DISABLED or NIGHTTIME_DURATION_BASED_ON_VOLTAGE -> when activated then logger stops logging at certain time of day
#define NIGHTTIME_MODE                                  NIGHTTIME_MODE_TRY_DATATRANS    // [CONFIG] NIGHTTIME_MODE_ONLY_SLEEP or NIGHTTIME_MODE_TRY_DATATRANS
// ONLY FOR NIGHTTIME_ENTER = NIGHTTIME_USE_HOURS
#define NIGHTTIME_TURN_ON_HOUR                          1                               // [CONFIG] start again logging at this UTC hour
#define NIGHTTIME_TURN_ON_MINUTE                        0                               // [CONFIG] start again logging at this UTC minute
#define NIGHTTIME_TURN_OFF_HOUR                         23                              // [CONFIG] stop logging at this UTC hour
#define NIGHTTIME_TURN_OFF_MINUTE                       0                               // [CONFIG] stop logging at this UTC minute
// ONLY FOR NIGHTTIME_MODE = NIGHTTIME_MODE_TRY_DATATRANS
#define NIGHT_TIME_DATA_TRANS_DEEP_NIGHT_HOURS          0                               // [CONFIG] hours when not trying to establish data connection
#define NIGHTTIME_MODE_TRY_DATATRANS_WAKEUP_SECONDS     3600                            // [CONFIG] 180, in seconds, MAX 4096, ESPNOW_CUSTOM_RF_CALIBRATION = true and GPS backup current: 900 = every 15min: 28uA, 300 = every 5min: 48.4uA
#define NIGHTTIME_MODE_TRY_DATATRANS_MODULO             24                              // 3, only every X times of wake ups (NIGHTTIME_MODE_TRY_DATATRANS_EVERY_SECONDS) try data transmission, otherwise just check if night time is over
// ONLY FOR NIGHTTIME_ENTER = NIGHTTIME_DURATION_BASED_ON_VOLTAGE_xxV
#define NIGHTTIME_DURATION_BASED_ON_V_RETURN_DEBOUNCE   40                              // 40, when in night time due to V < xx (e.g., NIGHTTIME_DURATION_BASED_ON_VOLTAGE_37V), then only returning when voltage > 3.7 + 0.040V

/** Activity detection (tag goes in low power mode when not time to sleep but animal not moving) */
#define ACTIVITY_ACTIVATION_ENABLED                     false                           // [CONFIG] false, when not night time and tag is not moving, then special activity activation mode is activated
#define ACTIVITY_THRESHOLD_ACTIVE_TO_INACTIVE_AVG       7                               // [CONFIG] 120, average ODBA in mg that needs to be exceeded within one FIFO window to go into inactive state
#define ACTIVITY_THRESHOLD_INACTIVE_TO_ACTIVE_THR       4                               // [CONFIG] 4, acceleration that needs to be exceeded to go back into tracking, times 15.64mg (3 = 3 * 15.64mg = 46.92mg), 8G range
#define ACTIVITY_THRESHOLD_INACTIVE_TO_ACTIVE_SKIP      BMX160_INT_SIGMOT_SKIP_2_56S    // BMX160_INT_SIGMOT_SKIP_2_56S, wait for that time after first time exceeded
#define ACTIVITY_THRESHOLD_INACTIVE_TO_ACTIVE_PROOF     BMX160_INT_SIGMOT_PROOF_0_96S   // BMX160_INT_SIGMOT_PROOF_0_96S, wait for that time if again movement detected
#define ACTIVITY_TRANSMISSION_INTERVAL                  60*6                            // [CONFIG] NEW: IN MINUTES, 10, 10 = every 10min = 42.6uA, wake up interval when waiting on activity to check for gateways

/** Undervoltage mode: battery protection */
#define POWER_DOWN_MODE_DATA_TRANS_WHEN_VOLTAGE_HALF_OK true                            // true, try a data transmission if voltage higher than BATT_MIN_VOLTAGE + ((BATT_RESTART_VOLTAGE - BATT_MIN_VOLTAGE) / 2)
#define BATT_MIN_VOLTAGE                                3500                            // [CONFIG] 3500, hibernate below that voltage and
#define BATT_RESTART_VOLTAGE                            3550                            // [CONFIG] 3600, wait until voltage above that threshold again
#define FIRST_UNDER_VOLTAGE_SLEEP_TIME                  1800                            // 1800, first time UV detected -> stop imu fifo and sleep for that time
#define FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME              12*3600                         // 6*3600, second and more times UV detected -> sleep for that time
#define DATATRANS_MIN_BATT_VOLTAGE                      3500                            // [CONFIG] 3500, don't start scan if battery voltage too low
#define DATATRANS_MIN_BATT_VOLTAGE_DURING_TRANSM        3200                            // [CONFIG] 3200, don't continue transmission if voltage dropped below that threshold after some transmission

/** Time over WIFI or GPS Configuration (immediately after wake-up) on channel 6, then 1, then 11, each for 120ms */
#define SKIP_GET_TIME                                   false                           // [CONFIG] false, starting without any time (get it maybe later over GPS)
// ONLY IF NOT SKIP_GET_TIME
#define USE_GPS_TO_GET_TIME_AT_BEGINNING                false                           // true, use not only WiFi but also GPS for getting the time at the beginning
#define USE_GPS_TO_GET_TIME_TIMEOUT_SECONDS             300                             // 300, timeout to only get time at very first start               
#define TIME_WIFI_OUTPUT_POWER                          RADIO_MAX_TX_11_DBM             // RADIO_MAX_TX_11_DBM, 19.5dBm will brown out on REV3
#define TIME_BETWEEN_GET_TIME_RETRIES_SECONDS           1800                            // [CONFIG] 3600, 120 = 660uA, if not activated: sleep for that time, wake up, check if wifi there and try to activate, 120 = 234uA average

/** Activation */
#define ACTIVATION_MODE                                 ACTIVATION_MODE_STORE_PERMANENTLY   // [CONFIG] ACTIVATION_MODE_SKIP or ACTIVATION_MODE_STORE_PERMANENTLY or ACTIVATION_MODE_ON_EVERY_START: how shall the tag be activated
#define ACTIVATION_SOURCE                               ACTIVATION_BY_ESPNOW            // [CONFIG] ACTIVATION_BY_ESPNOW or ACTIVATION_BY_WIFI
// ONLY FOR ACTIVATION_BY_WIFI (only start if this wifi has been seen) -> every 60min = 35.3uA (channels 1, 6, 11 each 120ms), every 60min = 35.1uA (channels 1-11 each 30ms), every 60min = 26.6uA (only one channel)
#define ACTIVATION_WIFI_OUTPUT_POWER                    RADIO_MAX_TX_11_DBM             // RADIO_MAX_TX_11_DBM, 19.5dBm will brown out on REV3
#define ACTIVATION_HOURLY_INTERRUPT_ON_MINUTE           0                               // 0, on which minute shall be checked (40 = 14:40, 15:40, ..)
#define ACTIVATION_WAKE_UP_ON_MODULO_HOURS              1                               // 1, 1 = every hour, 2 = on even hours (14, 16, ..), 3 = on all hours that are dividable by 3, ..
// ONLY FOR ACTIVATION_BY_ESPNOW
#define ACTIVATION_BY_ESPNOW_RETRY_TIME_SECONDS         30                             // [CONFIG] ESPNOW_CUSTOM_RF_CALIBRATION = true: 300 = every 5min: 44.6uA, 900 = every 15min: 27.5uA -> ESPNOW_CUSTOM_RF_CALIBRATION = false: 300 = every 5min: 109uA, 900 = every 15min: 48.6uA
#define ACTIVATION_BY_ESPNOW_RETRY_NO_MAG_TIME_ENABLED  false                            // false, mag calibration all zero, then sleep different time
#define ACTIVATION_BY_ESPNOW_RETRY_NO_MAG_TIME_SEC      30                             // 180, mag calibration all zero, then sleep for that time

/** Configuration Data Transmission: Common */
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY       24*60                            // [CONFIG] 12*60 = 00:00 and 12:00 (UTC), 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission nothing transmitted -> re-try more seldomly
#define DATATRANSM_TRY_EVERY_FULL_MINUTE_FREQUENTLY     24*60                            // 1 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission something transmitted -> re-try more often

/** Memory Full Configuration */
#define ST_MEMFULL_RESTART_WHEN_MEMORY_FREE_BYTES       2048*64*60                      // 2048*64*60 = 60 blocks transmitted, only exiting state when at least that memory is free -> 2048*64*2048 = until memory totally free except the one block that is not completely full (2048*64*2048 will never be reached actually - minBlocksToTransmit will be 0)
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY       24*60                           // [CONFIG] 1*60, 12*60 = 00:00 and 12:00 (UTC), 2 = on every even minute, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission nothing transmitted -> re-try more seldomly
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_LESS_FREQU     24*60                           // last data transmission something transmitted, current data transmission NOTHING -> use this minute interval for one time
#define ST_MEMFULL_TRY_EVERY_FULL_MINUTE_FREQUENTLY     24*60                           // 1, 12hrs = 60*12 = 720 = 00:00 and 12:00, 15 = 14:00, 14:15, .., only IF DATA TO TRANSMIT (see MIN_BLOCKS_TO_TRANSMIT): last data transmission something transmitted -> re-try more often

/** IMU data temporary in RAM (DEPENDING ON ACC FREQUENCY!) */
#define ACC_INTERRUPT_WATERMARK                         960                             // [CONFIG] 960 fine for <= 50Hz, 918 for 200Hz (IMU only), other stuff not tested, imu is generating interrupt when fifo is at that fill level

/** Configuration Data Transmission: General */
#define DATATRANS_OUTPUT_POWER                          RADIO_MAX_TX_19_5_DBM           // [CONFIG] RADIO_MAX_TX_19_5_DBM, 19.5dBm will brown out on REV3

/** Configuration Data Transmission: ESP NOW */
#define ESPNOW_CUSTOM_RF_CALIBRATION                    true                            // [CONFIG] WARNING: only works if ACC_RAM_SIZE_1 + ACC_RAM_SIZE_2 is small enough! only 4kByte of RTC RAM
#define ESPNOW_CUSTOM_RF_FULL_CALIB_EVERY_X_TIMES       50                              // 50, max. 255, full RF calibration (around 150ms) every X times when trying to do anything with ESP NOW
#define ESPNOW_GATEWAY_AROUND_LISTENING_TIME            50                              // 50, also for data transmission, gateway broadcasts every 20ms

#define COMMAND_BYTE_FORCE_TRACKING_DURATION_SECONDS    900                             // [CONFIG] 900, around 88 seconds @12.5Hz until this threshold will be checked

#define ESPNOW_DATA_RATE                                WIFI_PHY_RATE_18M               // [CONFIG] WIFI_PHY_RATE_18M, well tested with 18M
#define ESPNOW_LONG_RANGE                               0                               // [CONFIG] 0, long range only with 1MBPS -> ESPNOW_DATA_RATE should be 1MBPS!
#define ESPNOW_MIN_BYTES_TO_TRANSMIT                    1                               // [CONFIG] while tracking: only start scan (and data transmission) if there are enough data bytes to send
#define ESPNOW_MAX_BYTES_TO_TRANSMIT                    ESP_NOW_FLASH_STREAM_NO_LIMIT   // [CONFIG] maximum bytes to transmit in one go, ESP_NOW_FLASH_STREAM_NO_LIMIT = no limit

/** Configuration Data Transmission: WIFI */
#define WIFI_MAX_POST_TASK_TIME_SECONDS                 45                              // 45, max time for sending one successfully transmitted block
#define WIFI_MIN_BLOCKS_TO_TRANSMIT                     1                               // [CONFIG] 1, only start scan (and data transmission) if there is enough data to send
#define WIFI_MAX_BLOCKS_TO_TRANSMIT                     200                             // [CONFIG] 200, during one connection, burst maximum this amount of blocks (maximum memory blocks = 2048)

/** Configuration Mag Calibration */
#define MAG_CALIBRATION_MODE_DURATION_SECONDS           120                             // 120, when mag calibration is commanded via the gateway, this is the calibration duration

/** Configuration Data Transmission: REST endpoint */
#define WIFI_ENDPOINT_SETTINGS                          3                               // 3, see below for details

#if (WIFI_ENDPOINT_SETTINGS == 1)
    // SETTING FOR DROPBOX
    #define REST_URL                                    "https://api-content.dropbox.com/2/files/upload"
    #define REST_CONTENT_TYPE                           "application/octet-stream"
    #define REST_ADD_HEADER_KEY1                        "Authorization"
    #define REST_ADD_HEADER_VALUE1                      "Bearer " DROPBOX_BEARER
    #define REST_ADD_HEADER2_SPRINTF                    true
    #define REST_ADD_HEADER_KEY2                        "Dropbox-API-Arg"
    #define REST_ADD_HEADER_VALUE2                      "{\"path\":\"/%02X%02X%02X%02X%02X%02X/%u_WildFiData.wfi\",\"mode\":\"add\",\"autorename\":true,\"mute\":false,\"strict_conflict\":false}"
    #define REST_USE_BASE64_ENCODING                    false
    #define REST_SEND_FULL_BLOCKS                       true
    #define REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX        true
    #define REST_PAYLOAD_PREFIX                         "MMMMMM:PPPPPPPP:VVVV:" // M = MAC (last 3 byte), P = page address, V = battery voltage
    #define REST_PAYLOAD_POSTFIX                        "THEEND"
#elif (WIFI_ENDPOINT_SETTINGS == 2)
    // SETTING FOR RESTDB.IO
    #define REST_URL                                    RESTDB_ENDPOINT2
    #define REST_CONTENT_TYPE                           "application/json"
    #define REST_ADD_HEADER_KEY1                        "x-apikey"
    #define REST_ADD_HEADER_VALUE1                      RESTDB_APIKEY1
    #define REST_ADD_HEADER2_SPRINTF                    false
    #define REST_ADD_HEADER_KEY2                        NULL
    #define REST_ADD_HEADER_VALUE2                      NULL
    #define REST_USE_BASE64_ENCODING                    true
    #define REST_SEND_FULL_BLOCKS                       false
    #define REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX        false                        
    #define REST_PAYLOAD_PREFIX                         "[{\"tag\":\"" "WildFi" "\",\"data\":\""
    #define REST_PAYLOAD_POSTFIX                        "\"}]"
#elif (WIFI_ENDPOINT_SETTINGS == 3)
    // SETTING FOR LOCAL GATEWAY (PHONE OR ESP32 CAM)
    #define REST_URL                                    "http://192.168.43.1:8080/store"
    #define REST_CONTENT_TYPE                           "application/octet-stream"
    #define REST_ADD_HEADER_KEY1                        "Content-Length"
    #define REST_ADD_HEADER_VALUE1                      "131099" // 131099 for full blocks, 65563 for half blocks, strlen(PREFIX_CONSTRUCTED: "ABCDEF:PPPPPPPP:VVVV:") 21 + strlen(taskParams.postfix) 6 + (PAGES_IN_ONE_GO * 2048 = 65536)
    #define REST_ADD_HEADER2_SPRINTF                    false
    #define REST_ADD_HEADER_KEY2                        NULL
    #define REST_ADD_HEADER_VALUE2                      NULL
    #define REST_USE_BASE64_ENCODING                    false
    #define REST_SEND_FULL_BLOCKS                       true
    #define REST_PAYLOAD_CONSTRUCT_CUSTOM_PREFIX        true
    #define REST_PAYLOAD_PREFIX                         "MMMMMM:PPPPPPPP:VVVV:" // M = MAC (last 3 byte), P = page address, V = battery voltage
    #define REST_PAYLOAD_POSTFIX                        "THEEND"
#endif

#endif