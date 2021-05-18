#ifndef configFor_h
#define configFor_h

#include <stdio.h>
#include "configGeneralProximity.h"

#define WILDFI_CONFIG_NAME                              "TEST"
#define WILDFI_CONFIG_VERSION                           12

/** IMPORTANT settings */                      
#define TRACKER_MODE                                    MODE_TESTRUN                    // MODE_ONE_TIME_PROGRAMMING or MODE_TESTRUN or MODE_PRODUCTIVE or MODE_SELFTEST
#define ACTIVATION_MODE                                 ACTIVATION_MODE_SKIP            // ACTIVATION_MODE_SKIP or ACTIVATION_MODE_STORE_PERMANENTLY or ACTIVATION_MODE_ON_EVERY_START: how shall the tag be activated
#define GET_FIRST_TIME_OVER_WIFI                        true
#define GET_FIRST_TIME_OVER_GPS                         true
#define TAG_ID_SOURCE                                   TAG_ID_USE_MAC_LAST_TWO_BYTES   // TAG_ID_USE_MAC_LAST_TWO_BYTES or TAG_ID_USE_VALUE_IN_NVS
#define USE_IMU                                         true
#define USE_LEDS                                        true

/** GPS settings */
RTC_DATA_ATTR uint8_t gpsFixHours[] =                   { 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21 };
#define GPS_TIMEOUT_SECONDS                             300                             // for normal fixes but also for getting time
#define GPS_MIN_HDOP                                    3.0f                            // wait after first fix until hdop like that or time below expired
#define GPS_AFTER_FIX_MAX_WAIT_ON_HDOP                  7                               // in seconds, maximum wait time after getting first fix
#define GPS_FORCED_AFTER_EVERY_PROXIMITY                true                            // ALWAYS execute GPS after proximity detection, gpsFixHours still used

/** Proximity settings */
#define PROXIMITY_FREQUENCY_IN_MINUTES                  1                               // WARNING: 60 / PROXIMITY_FREQUENCY_IN_MINUTES without REST!
#define PROXIMITY_LISTENING_INTERVAL_MS                 1000
#define PROXIMITY_DBM                                   RADIO_MAX_TX_19_5_DBM
#define PROXIMITY_DATARATE                              WIFI_PHY_RATE_1M_L
#define PROXIMITY_LONGRANGE                             false // 0.25 to 0.5MBPS

#define PROXIMITY_AIR_TIME_US                           (PROXIMITY_DATA_LEN * 8 * 1000 * 1000) / (1 * 1024 * 1024) // FOR 1MBPS and BROADCAST, WARNING: change when using different PHY_RATE, in MICROSECONDS
#define PROXIMITY_DATA_LEN                              250
#define PROXIMITY_MAX_TAGS_IN_PARALLEL                  150                             // LIMITATION: max. 512 byte in one NVS write cycle = 9 + (3 * X) -> max. 167 tags
#define PROXIMITY_FULL_RF_CALIB_EVERY                   100                             // perform full RF calibration after X times proximity detection

/** Activation settings */
#define ACTIVATION_BY_GATEWAY_LISTENING_TIME            120                             // gateway broadcasts every 100ms
#define ACTIVATION_BY_GATEWAY_INTERVAL_SECONDS          60                              // (if 120ms listening) every 5min = 65.7uA, every 10min = 38.4uA, every 15min = 29.3uA

/** Proximity Group ID */
#define PROXIMITY_OWN_GROUP_0                           0x12
#define PROXIMITY_OWN_GROUP_1                           0x34
#define PROXIMITY_OWN_GROUP_2                           0x56
#define PROXIMITY_OWN_GROUP_3                           0x78

/** Battery protection */
#define BATT_MIN_VOLTAGE                                3550                            // hibernate below that voltage and
#define BATT_RESTART_VOLTAGE                            3650                            // wait until voltage above that threshold again

/** Configuration Data Transmission: ESP NOW */
#define ESPNOW_MIN_BATT_VOLTAGE_DURING_TRANSM           3300                            // will be checked every x messages during data transmission


/** Get time */
const uint8_t TIME_WIFI_LIST_SIZE =                     3;                                  // should include time wifi
const char* TIME_WIFI_SSIDS[TIME_WIFI_LIST_SIZE] =      { "RodelbahnSoelden", "wildfi", "guest" };   // wifi name to scan for at beginning -> if found -> connect, get time, stark tracking
const char* TIME_WIFI_PASSWORDS[TIME_WIFI_LIST_SIZE] =  { "xxxxxxx", "wildfi01", "xxxxxxx" };        // wifi password for activation/timestamp                  
#define TIME_WIFI_OUTPUT_POWER                          RADIO_MAX_TX_11_DBM                 // 19.5dBm will brown out on REV3
#define TIME_BETWEEN_GET_TIME_RETRIES_SECONDS           60                                  // 120 = 660uA, if not activated: sleep for that time, wake up, check if wifi there and try to activate, 120 = 234uA average
#define TIME_OVER_GPS_BLINK_LED                         true                                // blink the led when trying to get the time

/** Selftest mode parameters */
#define SELFTEST_PARAMETERS                             SELFTEST_VOLTAGE | SELFTEST_LEDS | SELFTEST_HALLSENSOR | SELFTEST_I2C | SELFTEST_RTC | SELFTEST_ACC_GYRO_FOC_CHECK /*| SELFTEST_ACC_GYRO_FOC_EXECUTE_IF_UNSET*/ /*| SELFTEST_ACC_GYRO_FOC_FORCE_EXECUTION*/ | SELFTEST_BARO /*| SELFTEST_FLASH_BAD_BLOCKS | SELFTEST_FLASH_READ_WRITE*/ | SELFTEST_FLASH_FULL_ERASE | SELFTEST_NVS_RESET | SELFTEST_CPU_CLOCK_DOWN | /*SELFTEST_WIFI_SCAN |*/ SELFTEST_ESPNOW_BROADCAST 

#endif