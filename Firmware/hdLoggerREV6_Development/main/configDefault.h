#ifndef config_h
#define config_h

#include "PlatformWildFiTagREV6.h"
#include "passwords.h"
#include "configDevelopment.h"

typedef struct {
    bool doTheBlink;
    uint8_t trackerMode;
    uint8_t trackingDataMode;
    uint8_t transmissionMethod;
    uint8_t drMinHdopXTen;
    uint16_t drImuSeconds;
    uint16_t gps1HzInterruptAfterSeconds;
    uint8_t accFrequency;
    uint8_t accAvg;
    uint8_t accRange;
    uint8_t magFrequency;
    uint8_t magAccuracy;
    uint8_t gyroFrequency;
    uint8_t gyroRange;
    uint8_t gyroMode;
    bool useMagnetometer;
    bool useGyro;
    uint8_t nightTimeEnter;
    uint8_t nightTimeMode;
    uint8_t nightTimeTurnOnHour;
    uint8_t nightTimeTurnOnMinute;
    uint8_t nightTimeTurnOffHour;
    uint8_t nightTimeTurnOffMinute;
    uint32_t nightTimeDataTransDeepestNightHours;
    uint16_t nightTimeModeTryDataTransWakeupSeconds;
    uint16_t battMinVoltage;
    uint16_t battRestartVoltage;
    uint16_t dataTransBattMinVoltage;
    uint16_t dataTransBattMinVoltageDuringTrans;
    bool skipGetTime;
    uint16_t timeBetweenGetTimeRetriesSeconds;
    uint8_t activationMode;
    uint8_t activationSource;
    uint16_t activationByEspNowRetrySeconds;
    uint16_t dataTransTryEveryFullMinSeldomly;
    uint16_t memFullTryEveryFullMinSeldomly;
    uint16_t accInterruptWatermark;
    uint8_t dataTransOutputPower;
    bool espNowCustomRFCalibration;
    uint16_t commandByteForceTrackingDurationSeconds;
    uint8_t espNowDataRate;
    bool espNowLongRange;
    uint32_t espNowMinBytesToTransmit;
    uint32_t espNowMaxBytesToTransmit;
    uint16_t wifiMinBlocksToTransmit;
    uint16_t wifiMaxBlocksToTransmit;
    bool activityActivationEnabled;
    uint16_t activityThresholdActiveToInactiveAvg;
    uint8_t activityThresholdInactiveToActiveThr;
    uint16_t activityTransmissionInterval;
} tag_config_t;

/** Default NVS storage names */
#define NVS_CONF_DO_THE_BLINK                           "h00" 
#define NVS_CONF_TRACKER_MODE                           "h01" 
#define NVS_CONF_TRACKING_DATA_MODE                     "h02"
#define NVS_CONF_TRANSMISSION_METHOD                    "h03"
#define NVS_CONF_DR_MIN_HDOP_X_TEN                      "h04"
#define NVS_CONF_DR_IMU_SECONDS                         "h05"
#define NVS_CONF_1HZ_INTERRUPT_AFTER_SECONDS            "h06"
#define NVS_CONF_ACC_FREQUENCY                          "h07"
#define NVS_CONF_ACC_AVG                                "h08"
#define NVS_CONF_ACC_RANGE                              "h09"
#define NVS_CONF_MAG_FREQUENCY                          "h10"
#define NVS_CONF_MAG_ACCURACY                           "h11"
#define NVS_CONF_GYRO_FREQUENCY                         "h12"
#define NVS_CONF_GYRO_RANGE                             "h13"
#define NVS_CONF_GYRO_MODE                              "h14"
#define NVS_CONF_USE_MAG                                "h15"
#define NVS_CONF_USE_GYRO                               "h16"
#define NVS_CONF_NIGHT_TIME_ENTER                       "h17"
#define NVS_CONF_NIGHT_TIME_MODE                        "h18"
#define NVS_CONF_NIGHTTIME_TURN_ON_HOUR                 "h19"
#define NVS_CONF_NIGHTTIME_TURN_ON_MINUTE               "h20"
#define NVS_CONF_NIGHTTIME_TURN_OFF_HOUR                "h21"
#define NVS_CONF_NIGHTTIME_TURN_OFF_MINUTE              "h22"
#define NVS_CONF_NIGHTTIME_DATA_TRANS_DEEP_NIGHT_HOURS  "h23"
#define NVS_CONF_NIGHTTIME_DATATRANS_WAKEUP_SECONDS     "h24"
#define NVS_CONF_BATT_MIN_VOLTAGE                       "h25"
#define NVS_CONF_BATT_RESTART_VOLTAGE                   "h26"
#define NVS_CONF_DATATRANS_MIN_BATT_VOLTAGE             "h27"
#define NVS_CONF_DATATRANS_MIN_BATT_VOLTAGE_DURING_TR   "h28"
#define NVS_CONF_SKIP_GET_TIME                          "h29"
#define NVS_CONF_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS  "h30"
#define NVS_CONF_ACTIVATION_MODE                        "h31"
#define NVS_CONF_ACTIVATION_SOURCE                      "h32"
#define NVS_CONF_ACTIVATION_BY_ESPNOW_RETRY_TIME_SECS   "h33"
#define NVS_CONF_DATATRANSM_TRY_EVERY_FULL_MINUTE_SELD  "h34"
#define NVS_CONF_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY "h35"
#define NVS_CONF_ACC_INTERRUPT_WATERMARK                "h36"
#define NVS_CONF_DATATRANS_OUTPUT_POWER                 "h37"
#define NVS_CONF_ESPNOW_CUSTOM_RF_CALIBRATION           "h38"
#define NVS_CONF_COMMAND_BYTE_FORCE_TR_DURATION_SECS    "h39"
#define NVS_CONF_ESPNOW_DATA_RATE                       "h40"
#define NVS_CONF_ESPNOW_LONG_RANGE                      "h41"
#define NVS_CONF_ESPNOW_MIN_BYTES_TO_TRANSMIT           "h42"
#define NVS_CONF_ESPNOW_MAX_BYTES_TO_TRANSMIT           "h43"
#define NVS_CONF_WIFI_MIN_BLOCKS_TO_TRANSMIT            "h44"
#define NVS_CONF_WIFI_MAX_BLOCKS_TO_TRANSMIT            "h45"
#define NVS_CONF_ACTIVITY_ACTIVATION_ENABLED            "h46"
#define NVS_CONF_ACTIVITY_THRESHOLD_ACTIVE_TO_INACT_AVG "h47"
#define NVS_CONF_ACTIVITY_THRESHOLD_INACTIVE_TO_ACT_THR "h48"
#define NVS_CONF_ACTIVITY_TRANSMISSION_INTERVAL         "h49"

/** -------- ACTUAL CONFIGURATION (NORMALLY NOT CHANGED BETWEEN PROJECTS) --------  */
#define WILDFI_SOFTWARE_VERSION                         75
#define BOOT_DELAY_SECONDS                              5                               // 5, after initial start, wait this amount of seconds before entering next state (to give time for putting tag into housing and stuff)

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
//#define MODE_SELFTEST                                 2                               // runs selftest on flash memory
//#define MODE_READFLASH                                3                               // reads out full flash (10.000 pages = 30min) and then goes to sleep
//#define MODE_MOCK_FLASH_STATE                         4                               // bring flash pointers at certain position to test stuff like memory full state

/** Defines for night time mode */
#define NIGHTTIME_MODE_ONLY_SLEEP                       1
#define NIGHTTIME_MODE_TRY_DATATRANS                    2

/** Defines for entering night time mode */
#define NIGHTTIME_ALWAYS_NIGHT                          0
#define NIGHTTIME_USE_HOURS                             1
#define NIGHTTIME_DISABLED                              2
#define NIGHTTIME_DURATION_BASED_ON_VOLTAGE_37V         3
#define NIGHTTIME_DURATION_BASED_ON_VOLTAGE_38V         4
#define NIGHTTIME_DURATION_BASED_ON_VOLTAGE_39V         5
#define NIGHTTIME_DURATION_BASED_ON_VOLTAGE_40V         6
#define NIGHTTIME_ONLY_BELOW_360V                       7
#define NIGHTTIME_ONLY_BELOW_365V                       8
#define NIGHTTIME_ONLY_BELOW_370V                       9
#define NIGHTTIME_ONLY_BELOW_375V                       10
#define NIGHTTIME_ONLY_BELOW_380V                       11

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
#define ESPNOW_META_MSG_GATEWAY_AROUND_V2               0x88
#define ESPNOW_META_MSG_GATEWAY_AROUND_V2_LEN           250
#define ESPNOW_META_MSG_TAG_AROUND_V2                   0x99
#define ESPNOW_META_MSG_TAG_AROUND_V2_LEN               30

/** FIFO */
#define IMU_FIFO_DEFINITELY_FULL                        990                             // valid for ALL sampling combintations (acc, mag, gyro), if fifo >= this, then it's full

/** NVS storage names */
#define NVS_FLASH_WRITE_POINTER                         "flashpnt"                      // data writing: pointing on current flash page
#define NVS_FLASH_WRITE_PAGE_OFFSET_POINTER             "flashoffsetpnt"                // data writing: pointing on current byte in flash page
#define NVS_FLASH_TAG_ACTIVATED_BY_WIFI                 "activated"                     // tag has seen activation wifi
#define NVS_FLASH_SELFTEST_DONE                         "selftestd"                     // self-test routine was executed
// ONLY FOR ESP NOW DATA TRANSMISSION
    #define NVS_FLASH_SEND_POINTER                      "sendpnt"                       // data transmission: pointing on next page within block to transmit
    #define NVS_FLASH_SEND_PAGE_OFFSET_POINTER          "sendoffpnt"                    // data transmission: pointing on next subpage within page to transmit
// ONLY FOR WIFI DATA TRANSMISSION
    #define NVS_FLASH_SEND_NEXT_BLOCK_POINTER           "blockpntsent"                  // data transmission: pointing on next block to transmit
    #define NVS_FLASH_SEND_NEXT_HALF_BLOCK_POINTER      "halfblocksent"                 // data transmission: sometimes only half a block transmitted -> save and do not retransmit

/** State enums */
typedef enum { ST_BOOT = 0, ST_SERIAL_MENUE, ST_FIRST_START, ST_GET_TIME, ST_WAIT_FOR_ACTIVATION, ST_START, ST_TRACK, ST_PWRDWN, ST_MEMFULL, ST_NIGHT_TIME, ST_FORCE_DOWNLOAD, ST_FULL_RF_CALIB, ST_MAG_CALIBRATION, ST_WAIT_FOR_ACTIVITY, ST_TIME_RESYNC } tracker_state_t;

/** Transmission methods */
#define TRANSMISSION_METHOD_ESP_NOW                     0                               // transmit data over esp now
#define TRANSMISSION_METHOD_WIFI                        1                               // transmit data over wifi

#define TRACKING_DATA_MODE_1HZ_GPS_AND_IMU              0
#define TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP            1
#define TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP           2
#define TRACKING_DATA_MODE_DEAD_RECKONING               3

/** Defines ESP NOW command bytes */
#define COMMAND_BYTE_NOTHING                            0x00                            // no command
#define COMMAND_BYTE_FORCE_TRACKING                     0x23                            // start tracking independent of normal run times
#define COMMAND_BYTE_ACTIVATE                           0x33                            // activate the tag
#define COMMAND_BYTE_DEACTIVATE                         0x43                            // put the tag back into wait for activation state
#define COMMAND_BYTE_CHANGE_CONFIG                      0x53                            // change configuration of tag
#define COMMAND_BYTE_MAG_CALIBRATION                    0x63                            // enter mag calibration mode
#define COMMAND_BYTE_DO_NOT_SEND                        0x73                            // do not send data messages
#define COMMAND_BYTE_TIME_RESYNC                        0x83                            // enter time resync mode
#define COMMAND_BYTE_TIME_SYNC_ACTIVATION               0x93                            // enter time resync mode, but only in activation and only when not already got time
#define COMMAND_BYTE_ACTIVATE_WHEN_NO_GW                0xA3                            // NEW: then this command is received in activation mode: from now on: start as soon as no gateway is seen anymore, NO DATA IS SENT

/** Defines for Dead Reckoning */
#define DR_STATE_GPS                                    0
#define DR_STATE_IMU                                    1

/** Hours for deep night */
#define HOURS_9_TO_17				                    0x7FC0 // 0b000000000111111111000000
#define HOURS_23_TO_3				                    0xE00001 // 0b111000000000000000000001
#define HOUR_0						                    (1UL << 23)
#define HOUR_1			            			        (1UL << 22)
#define HOUR_2						                    (1UL << 21)
#define HOUR_3	            					        (1UL << 20)
#define HOUR_4	            					        (1UL << 19)
#define HOUR_5	            					        (1UL << 18)
#define HOUR_6	            					        (1UL << 17)
#define HOUR_7	            					        (1UL << 16)
#define HOUR_8		            				        (1UL << 15)
#define HOUR_9		            				        (1UL << 14)
#define HOUR_10		            				        (1UL << 13)
#define HOUR_11		            				        (1UL << 12)
#define HOUR_12		            				        (1UL << 11)
#define HOUR_13		            				        (1UL << 10)
#define HOUR_14		            				        (1UL << 9)
#define HOUR_15		            				        (1UL << 8)
#define HOUR_16			            			        (1UL << 7)
#define HOUR_17			            			        (1UL << 6)
#define HOUR_18			               			        (1UL << 5)
#define HOUR_19			            			        (1UL << 4)
#define HOUR_20			            	        		(1UL << 3)
#define HOUR_21				            		        (1UL << 2)
#define HOUR_22				            	        	(1UL << 1)
#define HOUR_23					                    	(1UL)

/** Functions */
void printConfigurationHash(tag_config_t *c);
bool configIsPlausible(tag_config_t *config, uint8_t *errorIdIn);
void printConfiguration(tag_config_t *c);
bool checkIfReconfigNeeded(WildFiTagREV6 *device, tag_config_t *currentConfig, uint8_t *nearestGatewayConfiguration, uint8_t nearestGatewayCommand, uint8_t nearestGatewayRssi, bool *changedSomething, bool debug);
bool readConfigFromNVS(WildFiTagREV6 *device, tag_config_t *config, bool printConfig);

#endif