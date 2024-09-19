#ifndef definesForProximity_h
#define definesForProximity_h

#include <stdio.h>
#include "esp_now.h" // for ESP_NOW_ETH_ALEN

// proximity detection for 1 second (WIFI_PHY_RATE_1M_L, SHORT RANGE): every 2min = 1.18mA, every 3min = 790uA, every 5min = 480uA, every 10min = 245uA, every 30min (calculated) = around 90uA
    // missing in power estimation: storing data to flash, downloading data via esp now, GPS twice per day
    // NEW: every 2min = 953uA
// AVERAGE POWER CONSUMPTION DURING 1 SECOND PROX DETECTION -> DFS = NO EFFECT, customPhyInit removed = NO EFFECT
// INCLUDING GREEN LED ON FOR SHORT TIME
// RX CURRENT WITHOUT SPIKES ALWAYS AROUND 115mA (active task) & 101mA (vTaskDelay 10ms accuracy) -> old (AP mode) = 123mA
// SHORT RANGE (LONG RANGE = 0)
    // 19.5 DBM
        // WIFI_PHY_RATE_1M_L:      136mA, NEW: 119mA (with vTaskDelay and Stationary Mode) <- seems to be best compromise, SUPERNEW: 105mA (112uWh @3.75V)
        // WIFI_PHY_RATE_11M_L:     126mA
        // WIFI_PHY_RATE_18M:       124mA
    // 15 DBM
        // WIFI_PHY_RATE_1M_L:      132mA
    // 11 DBM
        // WIFI_PHY_RATE_1M_L:      128mA
        // WIFI_PHY_RATE_18M:       124mA
    // 2 DBM
        // WIFI_PHY_RATE_1M_L:      125mA, NEW: 108mA, SUPERNEW: 100mA (107uWh @3.75V)
    // -2 DBM
        // WIFI_PHY_RATE_1M_L:      NOT WORKING -> resets to 20dBm -> RETEST, WAS UINT BUG!
// LONG RANGE
    // 19.5 DBM
        // WIFI_PHY_RATE_1M_L:      168mA (default setting, eventually will be 1/2 or 1/4 M), NEW: 133mA
        // WIFI_PHY_RATE_18M:       125mA (really long range?)
    // 11 DBM
        // WIFI_PHY_RATE_1M_L:      144mA

typedef struct {
    bool useLeds;
    uint8_t trackerMode;
    uint8_t activationMode;
    bool getFirstTimeOverWiFi;
    bool getFirstTimeOverGPS;
    uint8_t tagIdSource;
    uint8_t imuMode;
    uint32_t imuBurstMillis;
    bool environmentActivated;
    bool timeCorrectionBetweenTags;
    uint16_t timeCorrectionDiffMs;
    bool freeMemoryIfFull;
    uint8_t accFrequency;
    uint8_t accAvg;
    uint8_t accRange;
    uint8_t magFrequency;
    uint8_t magAccuracy;
    uint8_t gyroFrequency;
    uint8_t gyroRange;
    uint8_t gyroMode;
    uint8_t nightTimeEnter;
    uint8_t nightTimeTurnOnHour;
    uint8_t nightTimeTurnOnMinute;
    uint8_t nightTimeTurnOffHour;
    uint8_t nightTimeTurnOffMinute;
    uint32_t gpsFixHourBits;
    bool gpsRandomizeFixes;
    uint8_t gpsRandomizeFixesPerDay;
    uint8_t gpsMinHdopTimesTen;
    uint8_t gpsFirstFixCollectOrbitDataSeconds;
    bool gpsForcedAfterEveryProximity;
    uint8_t gpsSyncRTCFrequency;
    uint8_t proximityFrequencyMinute;
    uint8_t proximityFrequencyMinuteSeenSomeone;
    uint16_t proximityListeningIntervalMs;
    uint8_t proximityDbm;
    uint8_t proximityDatarate;
    bool proximityLongRange;
    uint16_t proximityAirTimeUs;
    uint16_t activationByGatewayListeningTime;
    uint16_t activationByGatewaySleepSeconds;
    uint16_t battMinVoltage;
    uint16_t battRestartVoltage;
    uint16_t battMinVoltageDuringTransmission;
    uint8_t timeWifiOutputPower;
    uint16_t timeBetweenGetTimeRetriesSeconds;
} tag_config_t;

/** Default NVS storage names */

#define NVS_OWN_ID                                      "ownid"
#define NVS_CONF_USE_LEDS                               "c00" 
#define NVS_CONF_TRACKER_MODE                           "c01" 
#define NVS_CONF_ACTIVATION_MODE                        "c02" 
#define NVS_CONF_GET_FIRST_TIME_OVER_WIFI               "c03"
#define NVS_CONF_GET_FIRST_TIME_OVER_GPS                "c04"
#define NVS_CONF_TAG_ID_SOURCE                          "c05"
#define NVS_CONF_IMU_MODE                               "c06"
#define NVS_CONF_IMU_BURST_MILLIS                       "c07"
#define NVS_CONF_ENVIRONMENT_ACTIVATED                  "c08"
#define NVS_CONF_TIME_CORRECTION_BETWEEN_TAGS           "c09"
#define NVS_CONF_TIME_CORRECTION_DIFF_MS                "c10"
#define NVS_CONF_FREE_MEMORY_IF_FULL                    "c11"
#define NVS_CONF_ACC_FREQUENCY                          "c12"
#define NVS_CONF_ACC_AVG                                "c13"
#define NVS_CONF_ACC_RANGE                              "c14"
#define NVS_CONF_MAG_FREQUENCY                          "c15"
#define NVS_CONF_MAG_ACCURACY                           "c16"
#define NVS_CONF_GYRO_FREQUENCY                         "c17"
#define NVS_CONF_GYRO_RANGE                             "c18"
#define NVS_CONF_GYRO_MODE                              "c19"
#define NVS_CONF_NIGHTTIME_ENTER                        "c20"
#define NVS_CONF_NIGHTTIME_TURN_ON_HOUR                 "c21"
#define NVS_CONF_NIGHTTIME_TURN_ON_MINUTE               "c22"
#define NVS_CONF_NIGHTTIME_TURN_OFF_HOUR                "c23"
#define NVS_CONF_NIGHTTIME_TURN_OFF_MINUTE              "c24"
#define NVS_CONF_GPS_FIX_HOUR_BITS                      "c25"
#define NVS_CONF_GPS_RANDOMIZE_FIXES                    "c26"
#define NVS_CONF_GPS_RANDOMIZE_FIXES_PER_DAY            "c27"
#define NVS_CONF_GPS_MIN_HDOP_X10                       "c28"
#define NVS_CONF_GPS_FIRST_FIX_COLLECT_ORBIT_SECONDS    "c29"
#define NVS_CONF_GPS_FORCED_AFTER_EVERY_PROXIMITY       "c30"
#define NVS_CONF_PROXIMITY_FREQUENCY_MINUTE             "c31"
#define NVS_CONF_PROXIMITY_FREQUENCY_MINUTE_SEENSOMEONE "c32"
#define NVS_CONF_PROXIMITY_LISTENING_INTERVAL_MS        "c33"
#define NVS_CONF_PROXIMITY_DBM                          "c34"
#define NVS_CONF_PROXIMITY_DATARATE                     "c35"
#define NVS_CONF_PROXIMITY_LONGRANGE                    "c36"
#define NVS_CONF_PROXIMITY_AIR_TIME_US                  "c37"
#define NVS_CONF_ACTIVATION_BY_GATEWAY_LISTENING_TIME   "c38"
#define NVS_CONF_ACTIVATION_BY_GATEWAY_SLEEP_SECONDS    "c39"
#define NVS_CONF_BATT_MIN_VOLTAGE                       "c40"
#define NVS_CONF_BATT_RESTART_VOLTAGE                   "c41"
#define NVS_CONF_BATT_MIN_VOLTAGE_DURING_TRANSM         "c42"
#define NVS_CONF_TIME_WIFI_OUTPUT_POWER                 "c43"
#define NVS_CONF_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS  "c44"
#define NVS_CONF_GPS_SYNC_RTC_FREQUENCY                 "c45"

/** Data NVS storage names */
#define NVS_FLASH_BUFFER_SIZE                           512
#define NVS_FLASH_BUFFER_NAME                           "flashbuff"                     // NVS_FLASH_BUFFER_SIZE byte blob buffer in NVS for flash memory
#define NVS_FLASH_WRITE_PAGE_POINTER                    "flashpgpnt"                    // data writing: pointing on current flash page 0 .. 131071
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffpnt"                   // data writing: pointing on current byte in flash page 0 .. 2047
#define NVS_FLASH_SEND_PAGE_POINTER                     "sendpgpnt"                     // data sending: pointing on current flash page 0 .. 131071
#define NVS_FLASH_SEND_PAGE_OFFSET_POINTER              "sendoffpnt"                    // data sending: pointing on current byte in flash page 0 .. 2047
#define NVS_FLASH_BLOCK_DELETED_POINTER                 "flashbldel"                    // deleted flash blocks (re-usable for new data)
#define NVS_FLASH_TAG_ACTIVATED_BY_WIFI                 "activated"                     // tag has seen activation wifi
#define NVS_FLASH_SEND_BLOB_POINTER                     "blobpnt"                       // data sending: pointing on current blob pointer
#define NVS_FLASH_SELFTEST_DONE                         "selftestd"                     // selftest has been executed at least once (failed or not)

/** -------- DEFINES (NEVER CHANGE) --------  */

/** Defines for modes */
#define MODE_TESTRUN                                    0                               // with debug output, not writing into flash memory, not incrementing NVS_FLASH_POINTER
#define MODE_PRODUCTIVE                                 1                               // normal tracker mode, fully operational, flash memory should be empty before and NVS_FLASH_POINTER reset to 0
#define MODE_SELFTEST                                   2                               // runs selftest on flash memory
#define MODE_GPS_TIME_SERVER                            3                               // only runs GPS time server

/** Defines for activation modes */
#define ACTIVATION_MODE_SKIP                            0                               // activation mode is skipped
#define ACTIVATION_MODE_STORE_PERMANENTLY               1                               // activation information is stored permanently in NVS (still valid after resets)
#define ACTIVATION_MODE_ON_EVERY_START                  2                               // activation information is stored only during a power cycle (lost after battery disconnected)

/** Tag id source */
#define TAG_ID_USE_MAC_LAST_TWO_BYTES                   1
#define TAG_ID_USE_VALUE_IN_NVS                         2

/** Defines for entering night time mode */
#define NIGHTTIME_ALWAYS_NIGHT                          0
#define NIGHTTIME_USE_HOURS                             1
#define NIGHTTIME_DISABLED                              2

/** Defines for IMU mode */
#define IMU_DEACTIVATED                                 0
#define IMU_ACC_ONLY                                    1
#define IMU_ACC_MAG_GYRO                                2

/** Selftest mode parameters */
#define SELFTEST_VOLTAGE_REF                            3750                            // during selftest routine (if voltage check enabled): check if this value (from power supply) is actually measured

/** All messages payload data types */
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY          0xAA
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND         0xCC
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND     0xDD
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED      0xEE

/** All messages message type (first byte of all message) */
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE                0

/** Proximity payload data structure (250 bytes max!) */
#define PROXIMITY_DATA_LEN                              250
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_0                 1
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_1                 2
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_2                 3
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_3                 4
#define PROX_PAYLOAD_OFFSET_OWN_ID_0                    5
#define PROX_PAYLOAD_OFFSET_OWN_ID_1                    6
#define PROX_PAYLOAD_OFFSET_SENDOFFSET_0                7
#define PROX_PAYLOAD_OFFSET_SENDOFFSET_1                8
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_0                9
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_1                10
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_2                11
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_3                12
#define PROX_PAYLOAD_OFFSET_TSLASTSYNCTYPE              13
#define PROX_PAYLOAD_OFFSET_VOLTAGE_0                   14
#define PROX_PAYLOAD_OFFSET_VOLTAGE_1                   15
#define PROX_PAYLOAD_OFFSET_LASTERRORID                 16
#define PROX_PAYLOAD_OFFSET_ERRORCNT_0                  17
#define PROX_PAYLOAD_OFFSET_ERRORCNT_1                  18
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_0                 19
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_1                 20
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_2                 21
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_3                 22
#define PROX_PAYLOAD_OFFSET_SW_VERSION                  23
#define PROX_PAYLOAD_OFFSET_CONF_VERSION                24
#define PROX_PAYLOAD_OFFSET_SYNCCOUNTER_0               25
#define PROX_PAYLOAD_OFFSET_SYNCCOUNTER_1               26
#define PROX_PAYLOAD_OFFSET_SYNCCOUNTER_2               27
#define PROX_PAYLOAD_OFFSET_SYNCCOUNTER_3               28
#define PROX_PAYLOAD_OFFSET_FREEMEMORY_0                29
#define PROX_PAYLOAD_OFFSET_FREEMEMORY_1                30
#define PROX_PAYLOAD_OFFSET_FREEMEMORY_2                31
#define PROX_PAYLOAD_OFFSET_FREEMEMORY_3                32
#define PROX_PAYLOAD_OFFSET_STARTCNT_0                  33
#define PROX_PAYLOAD_OFFSET_STARTCNT_1                  34
#define PROX_PAYLOAD_OFFSET_STARTCNT_2                  35
#define PROX_PAYLOAD_OFFSET_STARTCNT_3                  36

/** Gateway around payload data structure */
#define GATEWAY_AROUND_LEN                              70
#define GATEWAY_AROUND_OFFSET_OWN_GROUP_0               1
#define GATEWAY_AROUND_OFFSET_OWN_GROUP_1               2
#define GATEWAY_AROUND_OFFSET_OWN_GROUP_2               3
#define GATEWAY_AROUND_OFFSET_OWN_GROUP_3               4
#define GATEWAY_AROUND_OFFSET_OWN_ID_0                  5
#define GATEWAY_AROUND_OFFSET_OWN_ID_1                  6
#define GATEWAY_AROUND_OFFSET_COMMAND                   7 
#define GATEWAY_AROUND_OFFSET_START_CONFIG              8

/** Gateway commands */
#define PROXIMITY_COMMAND_NOTHING                       0x00
#define PROXIMITY_COMMAND_DO_NOT_SEND                   0x1F
#define PROXIMITY_COMMAND_ACTIVATE                      0x2F
#define PROXIMITY_COMMAND_DEACTIVATE                    0x3F
#define PROXIMITY_COMMAND_CHANGE_CONFIG                 0x4F
#define PROXIMITY_COMMAND_FULL_RESET                    0x5F
#define PROXIMITY_COMMAND_MAG_CALIB                     0x6F
#define PROXIMITY_COMMAND_RESYNC_TIME_BY_WIFI           0x7F
#define PROXIMITY_COMMAND_ACTIVATE_AT_06_00             0x8F
#define PROXIMITY_COMMAND_ACTIVATE_AT_12_00             0x9F
#define PROXIMITY_COMMAND_ACTIVATE_AT_15_00             0xAF
#define PROXIMITY_COMMAND_ACTIVATE_AT_20_00             0xBF
#define PROXIMITY_COMMAND_FIRST_SYNC_TIME_IN_ACTIVATION 0xCF

/** Tag around payload data structure */
#define TAG_AROUND_OFFSET_OWN_GROUP_0                   1
#define TAG_AROUND_OFFSET_OWN_GROUP_1                   2
#define TAG_AROUND_OFFSET_OWN_GROUP_2                   3
#define TAG_AROUND_OFFSET_OWN_GROUP_3                   4
#define TAG_AROUND_OFFSET_OWN_ID_0                      5      
#define TAG_AROUND_OFFSET_OWN_ID_1                      6
#define TAG_AROUND_OFFSET_VOLTAGE_0                     7
#define TAG_AROUND_OFFSET_VOLTAGE_1                     8
#define TAG_AROUND_OFFSET_LASTERRORID                   9
#define TAG_AROUND_OFFSET_ERRORCNT_0                    10
#define TAG_AROUND_OFFSET_ERRORCNT_1                    11
#define TAG_AROUND_OFFSET_CMD_BYTE_MIRRORED             12
#define TAG_AROUND_OFFSET_STATE                         13
#define TAG_AROUND_OFFSET_IS_ACTIVATED                  14
#define TAG_AROUND_OFFSET_HAS_VALID_TIMESTAMP           15

/** Got activated payload data structure */
#define GOT_ACTIVATED_OFFSET_OWN_GROUP_0                1
#define GOT_ACTIVATED_OFFSET_OWN_GROUP_1                2
#define GOT_ACTIVATED_OFFSET_OWN_GROUP_2                3
#define GOT_ACTIVATED_OFFSET_OWN_GROUP_3                4
#define GOT_ACTIVATED_OFFSET_OWN_ID_0                   5      
#define GOT_ACTIVATED_OFFSET_OWN_ID_1                   6

/** GPS hours for activation */
#define GPS_FIX_HOURS_9_TO_5				            0b000000111111111000000000
#define GPS_FIX_HOURS_0_TO_1				            0b000000000000000000000011
#define GPS_FIX_HOUR_23						            (1UL << 23)
#define GPS_FIX_HOUR_22			            			(1UL << 22)
#define GPS_FIX_HOUR_21						            (1UL << 21)
#define GPS_FIX_HOUR_20	            					(1UL << 20)
#define GPS_FIX_HOUR_19	            					(1UL << 19)
#define GPS_FIX_HOUR_18	            					(1UL << 18)
#define GPS_FIX_HOUR_17	            					(1UL << 17)
#define GPS_FIX_HOUR_16	            					(1UL << 16)
#define GPS_FIX_HOUR_15		            				(1UL << 15)
#define GPS_FIX_HOUR_14		            				(1UL << 14)
#define GPS_FIX_HOUR_13		            				(1UL << 13)
#define GPS_FIX_HOUR_12		            				(1UL << 12)
#define GPS_FIX_HOUR_11		            				(1UL << 11)
#define GPS_FIX_HOUR_10		            				(1UL << 10)
#define GPS_FIX_HOUR_9		            				(1UL << 9)
#define GPS_FIX_HOUR_8		            				(1UL << 8)
#define GPS_FIX_HOUR_7			            			(1UL << 7)
#define GPS_FIX_HOUR_6			            			(1UL << 6)
#define GPS_FIX_HOUR_5			               			(1UL << 5)
#define GPS_FIX_HOUR_4			            			(1UL << 4)
#define GPS_FIX_HOUR_3			            			(1UL << 3)
#define GPS_FIX_HOUR_2				            		(1UL << 2)
#define GPS_FIX_HOUR_1				            		(1UL << 1)
#define GPS_FIX_HOUR_0					            	(1UL)

/** State enums */
typedef enum { ST_BOOT = 0, ST_SERIAL_MENUE, ST_FIRST_START, ST_GET_TIME, ST_ACTIVATION, ST_START, ST_TRACK, ST_PWRDWN, ST_RF_FULL_CALIB, ST_FULL_RESET, ST_MAG_CALIBRATION, ST_WIFI_SYNC, ST_GPS_TIME_SERVER, ST_WIFI_SYNC_IN_ACTIVATION, ST_GET_TIME_BACK_TO_ACTIVATION, ST_NIGHTTIME_DATATRANS } tracker_state_t;

typedef enum {
    TAG_TYPE_TAG = 0,
    TAG_TYPE_GATEWAY = 1,
    TAG_TYPE_GATEWAY_NO_DATA = 2
} tag_type_t;

typedef enum {
    SYNC_TYPE_NONE = 0,
    SYNC_TYPE_GPS,
    SYNC_TYPE_WIFI,
    SYNC_TYPE_NEIGHBOR
} last_sync_type_t;

/** Structs / Enums */
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint16_t id;
    uint8_t receivedMessages;
    uint8_t rssiMin;
    uint8_t rssiMax;
    uint16_t rssiSum;
    int16_t timeDifferenceSumMs;
    uint32_t timestampLastSync;
    tag_type_t tagType;
} proximity_entry_t;

typedef enum {
    PUSH_DATA_SUCCESS = 0,
    PUSH_DATA_PARAM_ERROR,
	PUSH_DATA_NVS_ERROR,
    PUSH_DATA_NVS_GET_BLOB_ERROR,
    PUSH_DATA_NVS_GET_BLOB_ERROR2,
    PUSH_DATA_NVS_SET_BLOB_ERROR,
    PUSH_DATA_NVS_COMMIT_ERROR,
    PUSH_DATA_NVS_SET_BLOB_ERROR2,
    PUSH_DATA_NVS_COMMIT_ERROR2,
    PUSH_DATA_NVS_WRITE_ERROR,
    PUSH_DATA_MT29_SEQ_WRITE_STATUS_BUFFER_ERROR,
    PUSH_DATA_MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR,
    PUSH_DATA_MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR1,
    PUSH_DATA_MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR2,
    PUSH_DATA_MT29_SEQ_WRITE_STATUS_MEM_FULL_FATAL
} push_data_result_t;

#endif