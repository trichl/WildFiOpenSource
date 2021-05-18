#ifndef configGeneralSettings_h
#define configGeneralSettings_h

/** -------- ACTUAL CONFIGURATION (NORMALLY NOT CHANGED BETWEEN PROJECTS) --------  */
#define WILDFI_SOFTWARE_VERSION                         31

/** Force download WIFI Configuration (immediately after wake-up, during time wifi scan, enter special mode to download all data and then sleep forever) */
const char* FORCE_DOWNLOAD_WIFI_SSID =                  "mpiforce";                     // mpidownload, enter special mode during get time state when this wifi is seen
const char* FORCE_DOWNLOAD_WIFI_PASSWORD =              "ABCDEFGH";                     // ABCDEFGH, enter special mode during get time state when this wifi is seen

/** Connect to WIFI Configuration */
#define WIFI_MAX_CONNECT_TIME_SECONDS                   8                               // (used for get time, activation and data transmission) connect should take <1s

/** Time over WIFI Configuration (immediately after wake-up) on channel 6, then 1, then 11, each for 120ms */
#define TIME_SLEEP_AFTER_GOT_TIME                       5                               // sleep for that time until re-starting the system

/** Mocking for data transmission */
#define MOCK_FLASH_WRITES                               0                               // 1 = flash will not be written, but all pointers will be updated and data transmitted
#define MOCK_FLASH_READ                                 0                               // (ESP NOW ONLY) 1 = during transmissions dummy data is sent
#define MOCK_FLASH_DELETE                               0                               // 1 = flash block will not be deleted after successful data transmission, but all pointers are updated

#define SLEEP_TIME_AFTER_START                          3                               // 10, after initial start, wait this amount of seconds before entering next state (to give time for putting tag into housing and stuff)

/** -------- DEFINES (NEVER CHANGE) --------  */
/** Defines for modes */
#define MODE_TESTRUN                                    0                               // with debug output, not writing into flash memory, not incrementing NVS_FLASH_POINTER
#define MODE_PRODUCTIVE                                 1                               // normal tracker mode, fully operational, flash memory should be empty before and NVS_FLASH_POINTER reset to 0
#define MODE_SELFTEST                                   2                               // runs selftest on flash memory
#define MODE_READFLASH                                  3                               // reads out full flash (10.000 pages = 30min) and then goes to sleep
#define MODE_MOCK_FLASH_STATE                           4                               // bring flash pointers at certain position to test stuff like memory full state

/** Defines for night time mode */
#define NIGHTTIME_MODE_ONLY_SLEEP                       1
#define NIGHTTIME_MODE_TRY_DATATRANS                    2

/** Defines for entering night time mode */
#define NIGHTTIME_ALWAYS_NIGHT                          0
#define NIGHTTIME_USE_HOURS                             1
#define NIGHTTIME_DISABLED                              2

/** Defines for activation modes */
#define ACTIVATION_BY_WIFI                              0                               
#define ACTIVATION_BY_ESPNOW                            1                               

#define ACTIVATION_MODE_SKIP                            0                               // activation mode is skipped
#define ACTIVATION_MODE_STORE_PERMANENTLY               1                               // activation information is stored permanently in NVS (still valid after resets)
#define ACTIVATION_MODE_ON_EVERY_START                  2                               // activation information is stored only during a power cycle (lost after battery disconnected)

/** ESP NOW message */
#define ESPNOW_META_MSG_TAG_AROUND                      0x77
#define ESPNOW_META_MSG_TAG_AROUND_LEN                  10
#define ESPNOW_META_MSG_GOT_ACTIVATED                   0x55
#define ESPNOW_META_MSG_GOT_ACTIVATED_LEN               1           
#define ESPNOW_META_MSG_GATEWAY_AROUND                  0x66
#define ESPNOW_META_MSG_GATEWAY_AROUND_LEN              2

/** NVS storage names */
#define NVS_FLASH_WRITE_POINTER                         "flashpnt"                      // data writing: pointing on current flash page
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffsetpnt"                // data writing: pointing on current byte in flash page
#define NVS_FLASH_TAG_ACTIVATED_BY_WIFI                 "activated"                     // tag has seen activation wifi
// ONLY FOR ESP NOW DATA TRANSMISSION
    #define NVS_FLASH_SEND_POINTER                      "sendpnt"                       // data transmission: pointing on next page within block to transmit
    #define NVS_FLASH_SEND_PAGE_OFFSET_POINTER          "sendoffpnt"                    // data transmission: pointing on next subpage within page to transmit
// ONLY FOR WIFI DATA TRANSMISSION
    #define NVS_FLASH_SEND_NEXT_BLOCK_POINTER           "blockpntsent"                  // data transmission: pointing on next block to transmit
    #define NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER      "halfblocksent"                 // data transmission: sometimes only half a block transmitted -> save and do not retransmit

/** State enums */
typedef enum { ST_FIRST_START_HARD_RESET = 0, ST_GET_TIME = 1, ST_WAIT_FOR_ACTIVATION = 2, ST_START = 3, ST_TRACK = 4, ST_PWRDWN = 5, ST_MEMFULL = 6, ST_NIGHT_TIME = 7, ST_FORCE_DOWNLOAD = 8 } tracker_state_t;

/** Transmission methods */
#define TRANSMISSION_METHOD_ESP_NOW                     0                               // transmit data over esp now
#define TRANSMISSION_METHOD_WIFI                        1                               // transmit data over wifi

#define TRACKING_DATA_MODE_1HZ_GPS_AND_IMU              0
#define TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP            1
#define TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP           2

/** Defines ESP NOW command bytes */
#define COMMAND_BYTE_FORCE_TRACKING                     0x23                            // start tracking independent of normal run times
#define COMMAND_BYTE_ACTIVATE                           0x33                            // activate the tag
#define COMMAND_BYTE_DEACTIVATE                         0x43                            // put the tag back into wait for activation state

#endif