#ifndef config_h
#define config_h

#include "PlatformWildFiTagREV6.h"
#include "definesForProximity.h"

#define WILDFI_SOFTWARE_VERSION                         26
#define WILDFI_CONFIG_NAME                              "PROXIMITY"
#define WILDFI_CONFIG_VERSION                           1

/** -------- DEFAULT CONFIGURATION -------- */

/** IMPORTANT settings */                      
#define DEF_TRACKER_MODE                                MODE_TESTRUN                    // MODE_ONE_TIME_PROGRAMMING or MODE_TESTRUN or MODE_PRODUCTIVE or MODE_SELFTEST
#define DEF_ACTIVATION_MODE                             ACTIVATION_MODE_SKIP            // ACTIVATION_MODE_SKIP or ACTIVATION_MODE_STORE_PERMANENTLY or ACTIVATION_MODE_ON_EVERY_START: how shall the tag be activated
#define DEF_GET_FIRST_TIME_OVER_WIFI                    true
#define DEF_GET_FIRST_TIME_OVER_GPS                     true
#define DEF_TAG_ID_SOURCE                               TAG_ID_USE_MAC_LAST_TWO_BYTES   // TAG_ID_USE_MAC_LAST_TWO_BYTES or TAG_ID_USE_VALUE_IN_NVS
#define DEF_IMU_MODE                                    IMU_ACC_ONLY                    // IMU_DEACTIVATED, IMU_ACC_ONLY or IMU_ACC_MAG_GYRO             
#define DEF_IMU_BURST_MILLIS                            0                               // 0 = only for one second during proximity detection
#define DEF_ENVIRONMENT_ACTIVATED                       true
#define DEF_USE_LEDS                                    true
#define DEF_TIME_CORRECTION_BETWEEN_TAGS                true
#define DEF_TIME_CORRECTION_DIFF_MS                     50
#define DEF_FREE_MEMORY_IF_FULL                         true

/** IMU settings */                                 
#define DEF_ACC_FREQUENCY                               BMX160_ACCEL_ODR_25HZ
#define DEF_ACC_AVG                                     BMX160_ACCEL_BW_RES_AVG8
#define DEF_ACC_RANGE                                   BMX160_ACCEL_RANGE_8G
#define DEF_MAG_FREQUENCY                               BMX160_MAG_ODR_25HZ
#define DEF_MAG_ACCURACY                                BMX160_MAG_ACCURACY_REGULAR
#define DEF_GYRO_FREQUENCY                              BMX160_GYRO_ODR_25HZ
#define DEF_GYRO_RANGE                                  BMX160_GYRO_RANGE_125_DPS
#define DEF_GYRO_MODE                                   BMX160_GYRO_BW_NORMAL_MODE

/** Night time mode (UTC TIME!) */
#define DEF_NIGHTTIME_ENTER                             NIGHTTIME_DISABLED              // NIGHTTIME_ALWAYS_NIGHT or NIGHTTIME_USE_HOURS or NIGHTTIME_DISABLED -> when activated then logger stops logging at certain time of day
#define DEF_NIGHTTIME_TURN_ON_HOUR                      12                              // ONLY FOR NIGHTTIME_ENTER = NIGHTTIME_USE_HOURS, start again logging at this UTC hour
#define DEF_NIGHTTIME_TURN_ON_MINUTE                    10                              // ONLY FOR NIGHTTIME_ENTER = NIGHTTIME_USE_HOURS, start again logging at this UTC minute (INCLUDED)
#define DEF_NIGHTTIME_TURN_OFF_HOUR                     12                              // ONLY FOR NIGHTTIME_ENTER = NIGHTTIME_USE_HOURS, stop logging at this UTC hour
#define DEF_NIGHTTIME_TURN_OFF_MINUTE                   20                              // ONLY FOR NIGHTTIME_ENTER = NIGHTTIME_USE_HOURS, stop logging at this UTC minute (EXCLUDED)

/** GPS settings */
#define DEF_GPS_FIX_HOUR_BITS                           0                               // 0 = no GPS, GPS_FIX_HOUR_8 | GPS_FIX_HOUR_9 | GPS_FIX_HOUR_10 | GPS_FIX_HOUR_11 | GPS_FIX_HOUR_12 | GPS_FIX_HOUR_13 | GPS_FIX_HOUR_14 | GPS_FIX_HOUR_15 | GPS_FIX_HOUR_16
#define DEF_GPS_RANDOMIZE_FIXES                         true                            // will override gpsFixHours from above
#define DEF_GPS_RANDOMIZE_FIXES_PER_DAY                 24                              // will override gpsFixHours from above
#define DEF_GPS_MIN_HDOP_X10                            30                              // wait after first fix until hdop like that or time below expired
#define DEF_GPS_FIRST_FIX_COLLECT_ORBIT_DATA_SECONDS    60                              // in seconds, for getting time over GPS: collect orbit data after getting very first fix
#define DEF_GPS_FORCED_AFTER_EVERY_PROXIMITY            true                            // ALWAYS execute GPS after proximity detection IF within gpsFixHours (or randomized fixes)
#define DEF_GPS_SYNC_RTC_FREQUENCY                      0                               // how often shall RTC be synchronized over GPS (0 = always, 1 = every second GPS fix)

/** Proximity settings */
#define DEF_PROXIMITY_FREQUENCY_MINUTE                  1                               // WARNING: 60 / DEF_PROXIMITY_FREQUENCY_IN_MINUTES without REST!
#define DEF_PROXIMITY_FREQUENCY_MINUTE_SEEN_SOMEONE     1                               // WARNING: 60 / DEF_PROXIMITY_FREQUENCY_MINUTE_SEEN_SOMEONE without REST!
#define DEF_PROXIMITY_LISTENING_INTERVAL_MS             1000                             // 1000
#define DEF_PROXIMITY_DBM                               RADIO_MAX_TX_19_5_DBM
#define DEF_PROXIMITY_DATARATE                          WIFI_PHY_RATE_1M_L              // WIFI_PHY_RATE_1M_L
#define DEF_PROXIMITY_LONGRANGE                         false                           // 0.25 to 0.5MBPS
#define DEF_PROXIMITY_AIR_TIME_US                       5200                            // 5200, this is measured, CHANGE FOR DIFFERENT DATARATE, calculated would be 1907 = (PROXIMITY_DATA_LEN * 8 * 1000 * 1000) / (1 * 1024 * 1024)

/** Activation settings */
#define DEF_ACTIVATION_BY_GATEWAY_LISTENING_TIME        120                             // gateway broadcasts every 100ms
#define DEF_ACTIVATION_BY_GATEWAY_SLEEP_SECONDS         60                              // (if 120ms listening) every 5min = 65.7uA, every 10min = 38.4uA, every 15min = 29.3uA

/** Battery protection */
#define DEF_BATT_MIN_VOLTAGE                            3550                            // hibernate below that voltage and
#define DEF_BATT_RESTART_VOLTAGE                        3650                            // wait until voltage above that threshold again
#define DEF_BATT_MIN_VOLTAGE_DURING_TRANSM              3300                            // will be checked every x messages during data transmission

/** Get time */          
#define DEF_TIME_WIFI_OUTPUT_POWER                      RADIO_MAX_TX_11_DBM             // 11dBm should be fine
#define DEF_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS       600                             // wait time after trying to get time (first 3 times: FIRST_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS)

/** -------- FIXED CONFIGURATION (NORMALLY NOT CHANGED BETWEEN PROJECTS, NO CONFIGURATION PARAMETERS) -------- */

/** Unsorted */
#define TIME_CORRECTION_WAKE_STUB_FEED_INTERVAL         400                             // so that no watchdog reset happens

/** Connect to WIFI Configuration */
#define WIFI_MAX_CONNECT_TIME_SECONDS                   8                               // (used for get time) connect should take <1s

/** Get time */
#define FIRST_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS     15                              // failing to get time -> first 3 times this is the sleep time

/** Mocking for data transmission */
#define MOCK_FLASH_WRITES                               0                               // 1 = flash will not be written, but all pointers will be updated and data transmitted
#define MOCK_FLASH_READ                                 0                               // (ESP NOW ONLY) 1 = during transmissions dummy data is sent

/** Battery protection */
#define FIRST_UNDER_VOLTAGE_SLEEP_TIME                  120                             // 1*3600, first time UV detected -> stop tracking and sleep for that time
#define FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME              6*3600                          // 12*3600, second and more times UV detected -> sleep for that time
#define BOOT_DELAY_SECONDS                              5                               // 5, after initial start, wait this amount of seconds before entering next state (to give time for putting tag into housing and stuff)

/** GPS */
#define GPS_TIMEOUT_SECONDS                             300                             // for normal fixes but also for getting time
#define GPS_TIMEOUT_NOT_EVEN_TIME_SECONDS               120                             // timeout when year is still 1980 (didn't even get time)
#define GPS_AFTER_FIX_MAX_WAIT_ON_HDOP                  7                               // in seconds, maximum wait time after getting first fix

/** Proximity Group ID */
#define PROXIMITY_OWN_GROUP_0                           0x12
#define PROXIMITY_OWN_GROUP_1                           0x34
#define PROXIMITY_OWN_GROUP_2                           0x56
#define PROXIMITY_OWN_GROUP_3                           0x78

/** Selftest mode parameters */
#define SELFTEST_PARAMETERS                             SELFTEST_VOLTAGE | SELFTEST_LEDS | SELFTEST_HALLSENSOR | SELFTEST_I2C | SELFTEST_RTC | SELFTEST_ACC_GYRO_FOC_CHECK /*| SELFTEST_ACC_GYRO_FOC_EXECUTE_IF_UNSET*/ /*| SELFTEST_ACC_GYRO_FOC_FORCE_EXECUTION*/ | SELFTEST_BARO /*| SELFTEST_FLASH_BAD_BLOCKS | SELFTEST_FLASH_READ_WRITE*/ | SELFTEST_FLASH_FULL_ERASE | SELFTEST_NVS_RESET | SELFTEST_CPU_CLOCK_DOWN | /*SELFTEST_WIFI_SCAN |*/ SELFTEST_ESPNOW_BROADCAST 

/** Proximity settings */
#define PROXIMITY_MAX_TAGS_IN_PARALLEL                  150                             // previous LIMITATION: max. 512 byte in one NVS write cycle = 9 + (3 * X) -> max. 167 tags
#define PROXIMITY_FULL_RF_CALIB_EVERY                   100                             // perform full RF calibration after X times proximity detection

/** Functions */
void printConfigurationHash(tag_config_t *c);
bool configIsPlausible(tag_config_t *config, uint8_t *errorIdIn);
void printConfiguration(tag_config_t *c);
bool checkIfReconfigNeeded(WildFiTagREV6 *device, tag_config_t *currentConfig, uint8_t *nearestGatewayConfiguration, uint8_t nearestGatewayCommand, uint8_t nearestGatewayRssi, bool *changedSomething, bool debug);
bool readConfigFromNVS(WildFiTagREV6 *device, tag_config_t *config, bool printConfig);

#endif