#ifndef configGeneralProximity_h
#define configGeneralProximity_h

#include <stdio.h>
#include "esp_now.h" // for ESP_NOW_ETH_ALEN

#define WILDFI_SOFTWARE_VERSION                         102

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

/** -------- ACTUAL CONFIGURATION (NORMALLY NOT CHANGED BETWEEN PROJECTS) --------  */

/** Unsorted */
#define TIME_CORRECTION_WAKE_STUB_FEED_INTERVAL         400                             // so that no watchdog reset happens

/** Connect to WIFI Configuration */
#define WIFI_MAX_CONNECT_TIME_SECONDS                   8                               // (used for get time) connect should take <1s

/** Get Time */
#define TIME_SLEEP_AFTER_GOT_TIME                       1                               // sleep for that time until re-starting the system

/** NVS Flash Buffer */
#define NVS_FLASH_BUFFER_NAME                           "flashbuff"                 
#define NVS_FLASH_BUFFER_SIZE                           512

/** Mocking for data transmission */
#define MOCK_FLASH_WRITES                               0                               // 1 = flash will not be written, but all pointers will be updated and data transmitted
#define MOCK_FLASH_READ                                 0                               // (ESP NOW ONLY) 1 = during transmissions dummy data is sent

/** Battery protection */
#define FIRST_UNDER_VOLTAGE_SLEEP_TIME                  20                              // 1*3600, first time UV detected -> stop tracking and sleep for that time
#define FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME              20                              // 12*3600, second and more times UV detected -> sleep for that time
#define SLEEP_TIME_AFTER_START                          3                               // 10, after initial start, wait this amount of seconds before entering next state (to give time for putting tag into housing and stuff)

/** -------- DEFINES (NEVER CHANGE) --------  */
/** Defines for modes */
#define MODE_TESTRUN                                    0                               // with debug output, not writing into flash memory, not incrementing NVS_FLASH_POINTER
#define MODE_PRODUCTIVE                                 1                               // normal tracker mode, fully operational, flash memory should be empty before and NVS_FLASH_POINTER reset to 0
#define MODE_SELFTEST                                   2                               // runs selftest on flash memory
//#define MODE_READFLASH                                  3                               // reads out full flash (10.000 pages = 30min) and then goes to sleep
//#define MODE_MOCK_FLASH_STATE                           4                               // bring flash pointers at certain position to test stuff like memory full state
#define MODE_ONE_TIME_PROGRAMMING                       5                               // for id writing and so on

/** Defines for activation modes */
#define ACTIVATION_MODE_SKIP                            0                               // activation mode is skipped
#define ACTIVATION_MODE_STORE_PERMANENTLY               1                               // activation information is stored permanently in NVS (still valid after resets)
#define ACTIVATION_MODE_ON_EVERY_START                  2                               // activation information is stored only during a power cycle (lost after battery disconnected)

/** NVS storage names */
#define NVS_FLASH_WRITE_PAGE_POINTER                    "flashpgpnt"                    // data writing: pointing on current flash page 0 .. 131071
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffpnt"                   // data writing: pointing on current byte in flash page 0 .. 2047

#define NVS_FLASH_SEND_PAGE_POINTER                     "sendpgpnt"                     // data sending: pointing on current flash page 0 .. 131071
#define NVS_FLASH_SEND_PAGE_OFFSET_POINTER              "sendoffpnt"                    // data sending: pointing on current byte in flash page 0 .. 2047

#define NVS_FLASH_TAG_ACTIVATED_BY_WIFI                 "activated"                     // tag has seen activation wifi
#define NVS_OWN_ID                                      "ownid"                         // ID in NVS, only needs one time programming

#define NVS_FLASH_SEND_BLOB_POINTER                     "blobpnt"                       // data sending: pointing on current blob pointer

/** Tag id source */
#define TAG_ID_USE_MAC_LAST_TWO_BYTES                   1
#define TAG_ID_USE_VALUE_IN_NVS                         2

/** Selftest mode parameters */
#define SELFTEST_VOLTAGE_REF                            4200                            // during selftest routine (if voltage check enabled): check if this value (from power supply) is actually measured

/** All messages payload data types */
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY          0xAA
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND         0xCC
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND     0xDD
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED      0xEE

/** All messages message type (first byte of all message) */
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE                0

/** Proximity payload data structure (250 bytes max!) */
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_0                 1
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_1                 2
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_2                 3
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_3                 4
#define PROX_PAYLOAD_OFFSET_OWN_ID_0                    5
#define PROX_PAYLOAD_OFFSET_OWN_ID_1                    6
#define PROX_PAYLOAD_OFFSET_SENDOFFSET_0                7
#define PROX_PAYLOAD_OFFSET_SENDOFFSET_1                8
#define PROX_PAYLOAD_OFFSET_LASTSYNC_0                  9
#define PROX_PAYLOAD_OFFSET_LASTSYNC_1                  10
#define PROX_PAYLOAD_OFFSET_VOLTAGE_0                   11
#define PROX_PAYLOAD_OFFSET_VOLTAGE_1                   12
#define PROX_PAYLOAD_OFFSET_LASTERRORID                 13
#define PROX_PAYLOAD_OFFSET_ERRORCNT_0                  14
#define PROX_PAYLOAD_OFFSET_ERRORCNT_1                  15

/** Gateway around payload data structure */
#define GATEWAY_AROUND_OFFSET_OWN_GROUP_0               1
#define GATEWAY_AROUND_OFFSET_OWN_GROUP_1               2
#define GATEWAY_AROUND_OFFSET_OWN_GROUP_2               3
#define GATEWAY_AROUND_OFFSET_OWN_GROUP_3               4
#define GATEWAY_AROUND_OFFSET_OWN_ID_0                  5           // always 0
#define GATEWAY_AROUND_OFFSET_OWN_ID_1                  6           // always 0
#define GATEWAY_AROUND_OFFSET_COMMAND                   7  

#define GATEWAY_AROUND_OFFSET_COMMAND_NOTHING           0x00
#define GATEWAY_AROUND_OFFSET_COMMAND_ACTIVATE          0x33

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

/** Flash memory storage structure */
/*
#define FLASH_TYPE_PROXIMITY                            0x01
#define FLASH_TYPE_GPS                                  0x02
// Proximity NEW: 1 byte data length (MAX 243, including this byte) + 1 byte type + timestamp 4 bytes + X * (per seen tag: 2 byte ID, 1 byte RSSI))
#define FLASH_PROX_DATA_LENGTH_LEN                      1
#define FLASH_PROX_TYPE_LEN                             1 // = FLASH_TYPE_PROXIMITY
#define FLASH_PROX_TIMESTAMP_LEN                        4
#define FLASH_PROX_ID_LEN                               2
#define FLASH_PROX_RSSI_LEN                             1
// GPS: 1 byte data length (MAX 243, including this byte) + 1 byte type + timestamp 4 bytes + 4 byte LAT + 4 byte LON + 1 byte HDOP*10 + 1 byte TTF in secs
#define FLASH_GPS_DATA_LENGTH_LEN                       1
#define FLASH_GPS_TYPE_LEN                              1 // = FLASH_TYPE_GPS
#define FLASH_GPS_TIMESTAMP_LENGTH                      4
#define FLASH_GPS_LAT_LENGTH                            4
#define FLASH_GPS_LON_LENGTH                            4
#define FLASH_GPS_HDOP_LENGTH                           1
#define FLASH_GPS_TTF_LENGTH                            1
*/

/** State enums */
typedef enum { ST_FIRST_START_HARD_RESET = 0, ST_GET_TIME, ST_WAIT_FOR_ACTIVATION, ST_START, ST_TRACK, ST_PWRDWN, ST_MEMFULL, ST_NIGHT_TIME, ST_RF_FULL_CALIB } tracker_state_t;

/** Structs */
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint16_t id;
    uint8_t receivedMessages;
    uint8_t rssiMin;
    uint8_t rssiMax;
    uint16_t rssiSum;
    int16_t timeDifferenceSumMs;
    uint16_t lastSync;
    bool isGateway;
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
    PUSH_DATA_MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR,
} push_data_result_t;

#endif