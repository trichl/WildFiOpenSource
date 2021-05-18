#ifndef PlatformWildFiTagREV6_h
#define PlatformWildFiTagREV6_h

// TODO: read temperature
// TODO: tryout disabling ADC0 and see if smaller current during active
// TODO: Interrupts -> both RTC and WAKEUP to EXT1? (because both need pull-up)

// TEST 29.06.2020: ACTIVATED DYNAMIC FREQUENCY SCALING AND TICKLESS IDLE (FreeRTOS)! (don't know side effects) -> DEACTIVATED, higher current consumption with BLE

// ESP-IDF
#include <stdio.h>

#include "InterfaceSoftwareI2C.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp32-hal-cpu.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/uart.h" // for setting baudrate
#include "driver/adc.h" // for voltage measurement
#include "esp_adc_cal.h" // for calibrated voltage measurement

// ESP-IDF for SPIFFS
#include "esp_spiffs.h"

// ESP-IDF for ESPNOW
#include "esp_now.h"
#include "esp_private/wifi.h" // for modifying default transmission speed of 1Mbps
#include "esp_phy_init.h" // for modifying custom phy calibration

// ESP-IDF for WIFI
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

// ESP-IDF for HTTP
#include "esp_http_client.h"
#include "esp_tls.h"
#include "mbedtls/base64.h"

// ESP-IDF for NTP Time Sync
#include "esp_sntp.h"

// ESP-IDF for BLE
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "esp_bt.h"

// ESP-IDF for Watchdog
#include "esp_task_wdt.h"

// Interfaces
#include "InterfaceTiming.h"

// Helper
#include "HelperBits.h"

// Integrated module libs
#include "ModuleRTC_RV8803C7.h"
//#include "ModuleBARO_BSEC_BME680.h"
#include "ModuleBARO_BME680.h"
#include "ModuleIMU_BMX160.h"
#include "ModuleIMU_BMX160_WAKESTUB.h"
#include "ModuleFLASH_MT29.h"

// Wake stub
#include "rom/rtc.h"  // WARNING: rom/rtc.h marked as deprecated, when using esp32/rom/rtc.h -> BROWNOUTS + HIGHER DEEP SLEEP CURRENT
#include "soc/timer_group_reg.h"
#include "soc/rtc.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"

// Software I2C
#include "esp_pm.h"

class WildFiTagREV6; // forward declaration for post_task_stream_flash_parameters_t

/** PIN definitions (equals to GPIOxy) */
#define PIN_WAKEUP						GPIO_NUM_38		// REV4 CHANGED: input
#define PIN_GPIO_A						GPIO_NUM_27		// REV4 CHANGED: EXTENSION PORT
#define PIN_BATT_VOLT					GPIO_NUM_35		// input
#define PIN_I2S_WS						GPIO_NUM_32		// EXTENSION PORT
#define PIN_I2S_SDO						GPIO_NUM_33		// EXTENSION PORT
#define PIN_RTC_INT						GPIO_NUM_25		// input (int. pull-up)
#define PIN_I2S_SCK						GPIO_NUM_26		// EXTENSION PORT
#define PIN_ACC_INT						GPIO_NUM_37		// REV4 CHANGED: input
#define PIN_SPI_SCLK					GPIO_NUM_14		// EXTENSION PORT
#define PIN_SPI_MISO					GPIO_NUM_12		// EXTENSION PORT
#define PIN_SPI_MOSI					GPIO_NUM_13		// EXTENSION PORT
#define PIN_SPI_CS						GPIO_NUM_15		// EXTENSION PORT
#define PIN_LED_GREEN					GPIO_NUM_2		// output
#define PIN_GPIO_B						GPIO_NUM_4		// EXTENSION PORT
#define PIN_TXD2						GPIO_NUM_10		// REV4 SWITCHED: EXTENSION PORT
#define PIN_RXD2						GPIO_NUM_9		// REV4 SWITCHED: EXTENSION PORT
#define PIN_POWER2						GPIO_NUM_18		// output
#define PIN_LED_RED						GPIO_NUM_23		// output
#define PIN_POWER						GPIO_NUM_19		// output

#define PIN_WAKEUP_BITMASK				( (1ULL<<38) )	// REV4 CHANGED: for wake-up
#define PIN_ACC_INT_BITMASK				( (1ULL<<37) )	// REV4 CHANGED: for wake-up
#define ADC_CHANNEL_BATT_VOLT			ADC1_CHANNEL_7

/** Battery voltage calculation */
#define BATT_VOLT_RES_1					47 			// in MOhm * 10 (47 = 4.7MOhm)
#define BATT_VOLT_RES_2					100			// in MOhm * 10
#define DEFAULT_VREF    				1100		// for characteristic calculation

/** CPU speeds */
#define ESP32_10MHZ						10			// with delay, new LDO, sensor power on, I2C initialized = 9.0mA (old LDO: 8.79mA)
#define ESP32_80MHZ						80			// with delay, new LDO, sensor power on, I2C initialized = 26.9mA (old LDO: 25.4mA)
#define ESP32_SERIAL_BAUD_RATE			115200

/** Data NVS */
#define NVS_DATA_PARTITION				"nvs2"

/** Blinky blink */
#define BLINKY_ON_DELAY					80 // in ms
#define BLINKY_PAUSE_DELAY				100 // in ms
typedef enum {
    B_RED = 0,
	B_GREEN = 1,
	B_BOTH = 2,
	B_NONE = 3
} blink_t;

/** Wake-up */
typedef enum {
	BY_RESET = 0,
	BY_EXT0 = 1,
	BY_EXT1 = 2,
	BY_TIMER = 3
} wake_up_reason_t;

// RTC FAST MEM: 8kByte (wake stub, data storage, ONLY CPU1 = PRO_CPU (my application) has access)
	// -> RTC_IRAM_ATTR, (RTC_DATA_ATTR when CONFIG_ESP32_RTCDATA_IN_FAST_MEM set)
	// -> NOT WORKING WHEN CONFIG_FREERTOS_UNICORE IS SET!!! -> SOFTWARE WILL COMPLETELY RUN ON CPU0 (wifi core)
// RTC SLOW MEM: 8kByte [upper 4kByte reserverd for future WiFi use] (ULP programs during deep sleep)
	// -> RTC_DATA_ATTR

/** RADIO TX POWER, x 0.25 = dBm, with HT7833 sometimes brownout @56 = 14dBm and always @78 = 19.5dBm, 20dBm = legal limit */
#define RADIO_MAX_TX_MINUS_2_DBM		-4		// does not work, result is 78 -> BECAUSE I PASSED uint8_t -> DO A RE-TEST
#define RADIO_MAX_TX_2_DBM				8
#define RADIO_MAX_TX_5_DBM				20
#define RADIO_MAX_TX_8_DBM				34
#define RADIO_MAX_TX_11_DBM				44
#define RADIO_MAX_TX_13_DBM				52
#define RADIO_MAX_TX_14_DBM				56
#define RADIO_MAX_TX_15_DBM				60
#define RADIO_MAX_TX_16_5_DBM			66
#define RADIO_MAX_TX_18_DBM				72
#define RADIO_MAX_TX_19_5_DBM			78

/** ESPNOW Frame Format for Sniffing */
#define ESPNOW_FRAME_OFFSET_FRAME_CTRL              0       // 2 bytes, D0 00
#define ESPNOW_FRAME_OFFSET_DURATION_ID             2       // 2 bytes, normally 00 00
#define ESPNOW_FRAME_OFFSET_RECEIVE_MAC             4       // 6 bytes, all FF if broadcast
#define ESPNOW_FRAME_OFFSET_SENDER_MAC              10      // 6 bytes
#define ESPNOW_FRAME_OFFSET_FILTER_MAC              16      // 6 bytes, all FF if broadcast
#define ESPNOW_FRAME_OFFSET_SEQ_CTRL                22      // 2 bytes, counting messages up but weirdly
#define ESPNOW_FRAME_OFFSET_CATEGORY_CODE           24      // 1 byte, ALWAYS 7F
#define ESPNOW_FRAME_OFFSET_ORG_ID                  25      // 3 bytes, ALWAYS 18 FE 34 (espressif)
#define ESPNOW_FRAME_OFFSET_RANDOM                  28      // 4 bytes RANDOM

#define ESPNOW_FRAME_OFFSET_VENDOR_SPEC_ELEMENT_ID  32      // 1 byte, ALWAYS DD
#define ESPNOW_FRAME_OFFSET_VENDOR_SPEC_LENGTH      33      // 1 byte, FF when 250 bytes transmission
#define ESPNOW_FRAME_OFFSET_VENDOR_SPEC_ORD_ID      34      // 3 bytes, ALWAYS 18 FE 34 (espressif), redundant with ESPNOW_FRAME_OFFSET_ORG_ID
#define ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE        37      // 1 byte, IMPORTANT, 0x04 = ESP NOW!
#define ESPNOW_FRAME_OFFSET_VENDOR_SPEC_VERSION     38      // 1 byte, esp now version, saw 01
#define ESPNOW_FRAME_OFFSET_PAYLOAD                 39
#define ESPNOW_FRAME_OFFSET_FCS_CHECKSUM_250_BYTES  289     // 4 bytes, ONLY VALID IF PAYLOAD = 250 bytes

#define ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD         43      // including checksum at end, add payload length to get total frame length for promiscous scan

/** ESPNOW Stream */
typedef enum {
	ESP_NOW_NOT_FINISHED = 0,
	ESP_NOW_FINISHED_BUT_FAILED = 1,
	ESP_NOW_FINISHED_AND_RECEIVED = 2
} esp_now_sending_status_t;
typedef enum {
	ESP_NOW_STREAM_DATA_FINISHED = 0,
	ESP_NOW_STREAM_DATA_WRONG_PARAMS = 1,
	ESP_NOW_STREAM_DATA_DMA_BUFFER_ERROR = 2,
	ESP_NOW_STREAM_DATA_FLASH_READ_ERROR = 3,
	ESP_NOW_STREAM_DATA_ESP_NOW_SEND_ERROR = 4,
	ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR = 5,
	ESP_NOW_STREAM_DATA_NO_ACK_DURING_TRANSMISSION_ERROR = 6,
	ESP_NOW_STREAM_DATA_FLASH_DELETE_ERROR = 7,
	ESP_NOW_STREAM_DATA_VOLTAGE_DURING_TRANSM_LOW = 8,
	ESP_NOW_STREAM_DATA_TOO_LONG_FATAL = 9,
	ESP_NOW_STREAM_DATA_ACK_TIMEOUT_FATAL = 10,
	ESP_NOW_STREAM_NO_DATA_TO_SEND = 11,
	ESP_NOW_STREAM_SEVERE_SOFTWARE_ERROR = 12 // NEW
} esp_now_stream_status_t;

#define ESP_NOW_FLASH_STREAM_NO_LIMIT							0xFFFFFFFF
#define ESP_NOW_FLASH_STREAM_HEADER_LEN							5
#define ESP_NOW_FLASH_STREAM_PAYLOAD_LEN						(250 - ESP_NOW_FLASH_STREAM_HEADER_LEN)
#define ESP_NOW_FLASH_STREAM_FIRST_BYTE							0xAB
#define ESP_NOW_FLASH_STREAM_VOLTAGE_CHECK_EVERY_X_MESSAGES		100

/** WiFi */
#define DEFAULT_SCAN_LIST_SIZE 			15 // maximum aps that can be found
#define WIFI_ALL_14_CHANNELS			0
#define WIFI_CHANNELS_1_TO_11			255
typedef enum {
	WIFI_CONNECT_RUNNING = 0,
	WIFI_CONNECT_SUCCESS = 1,
	WIFI_CONNECT_FAIL_AP_NOT_FOUND = 2,
	WIFI_CONNECT_NEVER_STARTED = 3
} wifi_connect_status_t;

/** REST Webservices */
typedef enum {
	HTTP_POST_DATA_RUNNING = 0,
	HTTP_POST_DATA_FINISHED_ALL_GOOD = 1,
	HTTP_POST_DATA_ERROR_TASK_CREATE = 2,
	HTTP_POST_DATA_ERROR_MALLOC_ENCODING = 3,
	HTTP_POST_DATA_ERROR_HTTP_EVENT = 4,
	HTTP_POST_DATA_ERROR_TLS = 5, 						// saw this 2 times during test run on dog
	HTTP_POST_DATA_ERROR_FLASH_READ = 6,
	HTTP_POST_DATA_ERROR_HTTP_OPEN = 7,					// happens when connected to hotspot, but no internet at all
	HTTP_POST_DATA_ERROR_CLIENT_WRITE = 8, 				// happens also when disabling hotspot OR internet gone DURING transmission
	HTTP_POST_DATA_ERROR_NO_BLOCKS_TO_UPLOAD = 9,
	HTTP_POST_DATA_ERROR_VOLTAGE_LOW = 10,
	HTTP_POST_DATA_ERROR_TIMEOUT_TASK_KILLED = 11,
	HTTP_POST_DATA_ERROR_WIFI_NOT_CONNECTED = 12,
	HTTP_POST_DATA_ERROR_FETCH_HEADER = 13,				
	HTTP_POST_DATA_ERROR_RET_CODE_MIN1 = 14,
	HTTP_POST_DATA_ERROR_RET_CODE_100 = 15,
	HTTP_POST_DATA_ERROR_RET_CODE_200 = 16,
	HTTP_POST_DATA_ERROR_RET_CODE_400 = 17,
	HTTP_POST_DATA_ERROR_RET_CODE_500 = 18,				// happens if database does not exist
	HTTP_POST_DATA_ERROR_RET_CODE_UNKNOWN = 19
} wifi_post_data_status_t;
typedef struct {
	const char *url;
	const char *contentType;
	const char *additionalHeaderKey;
	const char *additionalHeaderValue;
    const char *prefix;
    const char *postfix;
    uint8_t *data;
    uint32_t dataLen;
} post_task_parameters_t;
typedef struct {
	const char *url;
	const char *contentType;
	const char *additionalHeaderKey;
	const char *additionalHeaderValue;
    const char *prefix;
	bool constructCustomPrefix;
    const char *postfix;
	FLASH_MT29 *flashObject; 						// needed for callback functions (read flash memory)
	uint16_t flashBlockToSendNextPointer;			// start sending from this flash block
    uint16_t flashHalfBlockToSendNextPointer;		// start sending from this half of block
	uint16_t flashMaxNumberOfBlocksToTransmit;		// maximum number of blocks to transmit, starting from flashStartBlock, will be less if voltage dropped too low
	WildFiTagREV6 *deviceObject;					// needed for callback to readSupplyVoltage() function -> evaluating if we should continue to send data
	uint16_t minBatteryVoltageToContinue;			// after transmitting block: if voltage dropped below that value, then stop sending data!
	bool useBase64Encoding;
	bool debug;
} post_task_stream_flash_parameters_t;

esp_err_t httpEventHandler(esp_http_client_event_t *evt);
uint32_t mallocBASE64Encoding(uint8_t *dataIn, uint32_t dataInLen, char **dataOut);
void BASE64Encoding(uint8_t *dataIn, uint32_t dataInLen, char **dataOut, uint32_t dataOutLenBASE64);
bool createPOSTData(const char *prefixIn, uint8_t *dataIn, uint32_t dataInLen, const char *postfixIn, char **dataOut); // CALLED FROM TASK, for posting raw data with JSON prefix/postfix
void freeMemoryPOSTData(char *dataOut); // CALLED FROM TASK

/** NTP Time Sync */
void time_sync_notification_cb(struct timeval *tv);

/** BLE */
#define BLE_BEACON_MAX_DATA						16
#define BLE_BEACON_MAX_DEVICES					16
#define BLE_BEACON_FILL_VAL						0xFF

#define BIOLOGGER_BLE_BEACON_ADV_MAYOR			0xABCD		// 2 byte, general identifying BLE application
#define BIOLOGGER_BLE_BEACON_ADV_MINOR			0x0124		// 2 byte = id
#define BIOLOGGER_BLE_TX_POWER					-60			// FOR DISTANCE CALCULATION, measured RSSI when 1m away!!!

typedef enum {
    BLE_ADVERTISE_AND_SCAN = 0,
	BLE_ADVERTISE_ONLY = 1
} ble_mode_t;

typedef enum {
	BLE_MSG_DONT_CARE = 0,
    BLE_MSG_FROM_ESP = 1,
	BLE_MSG_FROM_ANDROID = 2
} ble_msg_type_t;

struct {
	uint16_t biologgerFound = 0;
	int32_t rssiSumArr[BLE_BEACON_MAX_DEVICES] = { 0 };
	uint16_t beaconCnt[BLE_BEACON_MAX_DEVICES] = { 0 };
	//uint64_t macAddressArr[BLE_BEACON_MAX_DEVICES] = { 0 }; // 6 * 8 = 48 bits needed
	uint16_t biologgerId[BLE_BEACON_MAX_DEVICES] = { 0 };
	uint8_t payload[BLE_BEACON_MAX_DEVICES][BLE_BEACON_MAX_DATA]; // only filled if isBioLoggerBeacon = true
	uint8_t payloadLen[BLE_BEACON_MAX_DEVICES];
	//bool isESP[BLE_BEACON_MAX_DEVICES] = { 0 };
} bleRXBuffer;

void bleMain(void *param); 												// main task executed in separate thread

/** Wake stub */
#define LIN_COEFF_A_SCALE_ESP               65536						// for V_BATT calculation in wake stub
#define LIN_COEFF_A_ROUND_ESP               (LIN_COEFF_A_SCALE_ESP/2)	// for V_BATT calculation in wake stub
#define DEEP_SLEEP_TIME_OVERHEAD_US 		(250 + 100 * 240 / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ) // using the internal RTC RC (150KHz) oscillator.
#define STRANGE_VBATT_OFFSET				32							// seems to be constant offset
#define V_BATT_MINIMUM_FOR_BOOT				3500						// do not boot if V_BATT below that voltage/1000!
#define USE_EXT0_IF_WAKE_UP_REJECTED		0							// wake stub decides not to wake up system -> will wake up again on EXT0 interrupt
#define USE_EXT1_IF_WAKE_UP_REJECTED		1							// wake stub decides not to wake up system -> will wake up again on EXT1 interrupt
RTC_IRAM_ATTR void deepsleep_for_us(uint64_t duration_us);
RTC_IRAM_ATTR void enter_deepsleep_wait_for_ext0();
void RTC_IRAM_ATTR adc1_get_raw_ram(adc1_channel_t channel);
void RTC_IRAM_ATTR esp_wake_deep_sleep(void); 							// 10ms without serial output, overwrites default wake stub

/** Self test */
#define SELFTEST_VOLTAGE						1 << 0
#define SELFTEST_LEDS							1 << 1
#define SELFTEST_HALLSENSOR						1 << 2
#define SELFTEST_I2C							1 << 3
#define SELFTEST_RTC							1 << 4
#define SELFTEST_ACC_GYRO_FOC_CHECK				1 << 5
#define SELFTEST_ACC_GYRO_FOC_EXECUTE_IF_UNSET	1 << 6
#define SELFTEST_ACC_GYRO_FOC_FORCE_EXECUTION	1 << 7
#define SELFTEST_BARO							1 << 8
#define SELFTEST_FLASH_BAD_BLOCKS				1 << 9
#define SELFTEST_FLASH_READ_WRITE				1 << 10
#define SELFTEST_FLASH_FULL_ERASE				1 << 11
#define SELFTEST_NVS_RESET						1 << 12
#define SELFTEST_CPU_CLOCK_DOWN					1 << 13
#define SELFTEST_WIFI_SCAN						1 << 14
#define SELFTEST_ESPNOW_BROADCAST				1 << 15

/** UART 2 */
#define UART2_PORT_NUMBER   				UART_NUM_1
#define UART2_RX_BUFFER 					2048
#define UART2_EVENT_QUEUE_SIZE 				20

/** Magnetometer calibration */
struct mag_calibration_t {
	int16_t xMin;
	int16_t xMax;
	int16_t yMin;
	int16_t yMax;
	int16_t zMin;
	int16_t zMax;
};

class WildFiTagREV6 {
	public:
		/** Basic functions */
		WildFiTagREV6();
		~WildFiTagREV6();
		bool selfTest(uint16_t voltageSupplied, uint32_t testBits);
		void initPins();							// takes 1ms
		// menuconfig: RELATED to WAKEUP TIME:
			// CONFIG_ESP32_DEEP_SLEEP_WAKEUP_DELAY = 500 (default 2000)
			// WARNING RECENTLY CHANGED: CONFIG_ESP32_RTC_CLK_CAL_CYCLES = 512 (default 1024)
			// WARNING RECENTLY CHANGED: CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP = y (default n)
			// WARNING RECENTLY CHANGED: CONFIG_FREERTOS_HZ = 1000 (default 100) -> ROLLBACK: actually WORSE than before
		void deepSleep(bool leavePowerOn = false); 	// OVERRIDE, start-up = 33ms, with RTC stuff and new LDO 12.7uA
		void lightSleep();							// NEW: 485-495uA singlecore (@80MHz or 10MHz, independent), 912uA singlecore, 1.11mA (new LDO), continues immediately (<3ms), but wakeup depends on RTC clock (might be inaccurate)
		void shortLightSleep(uint32_t millseconds);	// NEW 485-495uA singlecore (@80MHz or 10MHz, independent)
		void wasteOn();
		void wasteOff();
		void delay(uint16_t d);						// uses RTOS ticks -> not accurate, only +/-10ms accuracy, for accurate delays better use timer or preciseDelay
		void preciseDelay(uint32_t us);				// uses busy waiting
		void setCPUSpeed(uint8_t speed);			// start-up ALWAYS with 80MHz, @10MHz: 8.62mA (new: 9.02mA, see CPU speed defines above)
		esp_reset_reason_t getLastResetReason();	// ESP_RST_POWERON (normal), ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT (IMPORTANT!!)
		bool enableDynamicFrequencyScaling();

		/** Wake-up from deep sleep */
		void enableInternalTimerInterruptInDeepSleep(uint32_t seconds); // measured: 3.92h in deep sleep = 123s drift (too short), according to internet: drift should be around 1 minute per day
		void enableRTCInterruptInDeepSleep();		// using EXT0 (because pull-up supported), call before calling deepSleep()	
		void enableWakeUpPinInDeepSleep();			// using EXT1, only ALL pins LOW or one pin HIGH, call before calling deepSleep()
		void enableAccInterruptInDeepSleep();		// using EXT1 -> ACC or Wakeup pin wake up, both together not possible
		void enableUart2InterruptInLightSleep();
		wake_up_reason_t getWakeUpReason();

		/** Wake-stub */
		void customWakeStubFunction(bool (*functionPointer)());		// SEEMS TO RUN @40MHz CPU SPEED, call like device.customWakeStubFunction(wakeStub), needs to be RTC_DATA_ATTR
		void disableWakeStubNoBootIfVoltageLow();
		void setWakeStubRejectionInterruptSrc(uint8_t interruptSrc);	// use EXT0 (USE_EXT0_IF_WAKE_UP_REJECTED) or EXT1 (USE_EXT1_IF_WAKE_UP_REJECTED) to wake up if wake stub rejected wake up
		
		/** ADC battery voltage */
		uint16_t readSupplyVoltage();				// OVERRIDE, in V*1000 (2700 = 2.7V)
		uint32_t readSupplyVoltageFromWakeStub();	// Test result with real lipo (compared with multimeter): SHOULD: 4.11V, IS: 4.182V (TAG1) / 4.138V (TAG2) -> long time experiment: 4.14V measured with multimeter, measurement function: 4.217V 
		int32_t readHallSensor(uint16_t iterations);// might return shitty values due to PIN_WAKEUP = GPIO36: reading from it uses channels 0 and 3 of ADC1 (GPIO 36 and 39) -> "do not connect anything else to these pins and do not change their configuration, otherwise it may affect the measurement of low value signal from the sensor"
		uint32_t readSupplyVoltageFromADC();		// only for internal use, returns real pin voltage

		/** Built-in LEDs */
		void ledRedOn();
		void ledRedOff();
		void ledGreenOn();
		void ledGreenOff();
		void blinkTimes(uint8_t howOften, blink_t color, bool addDelayAtEnd = false);
		void blink(blink_t l1, blink_t l2 = B_NONE, blink_t l3 = B_NONE);

		/** Internal NVS memory (NEW: using partition "nvs2" instead of "nvs" to reduce initDataNVS time!) */
		bool initNVS();											// WARNING: takes 157ms (@10MHz) and 16ms-87ms (@80MHz), used for WiFi and BLE, using partition "nvs"
		bool initDataNVS();										// 9ms @80MHz, less pages = faster initialization but more wear levelling
		bool resetDataNVS();									// full erase
		void printDataNVSStats();
		uint32_t nvsReadUINT32(const char *key);				// 1ms
		uint16_t nvsReadUINT16(const char *key);				// 1ms
		uint8_t nvsReadUINT8(const char *key);					// 1ms
		bool nvsWriteUINT32(const char *key, uint32_t val); 	// 7ms
		bool nvsWriteUINT16(const char *key, uint16_t val); 	// 7ms
		bool nvsWriteUINT8(const char *key, uint8_t val); 		// 7ms
		bool nvsWriteUINT32x2(const char *key1, uint32_t val1, const char *key2, uint32_t val2);
		bool nvsWriteUINT32andUINT16(const char *key1, uint32_t val1, const char *key2, uint16_t val2);
		uint32_t nvsReadThenPlusOneUINT32(const char *key);

		/** Internal SPIFFS memory */
		bool initSpiffs(uint8_t maxFiles);
		bool printSpiffs(const char* fileName);
		void resetSpiffs(const char* fileName);
		uint32_t getFreeSpaceSpiffs();

		/** Power pins */
		void sensorPowerOn(); 						// +30uA during active mode, WARNING: BMX160 needs 38ms wake-up time! (really?)
		void sensorPowerOff();
		bool flashPowerOn(bool withDelay = true);	// +150uA in active mode (10MHz)
		bool flashPowerOff(bool withDelay);
		void keepSensorPowerOnInDeepSleep();		// +1uA in deep sleep (imu and baro on), ACC can fill FiFo while ESP32 sleeps

		/** GPIO B */
		void gpioBOn();
		void gpioBOff();

		/** UART 2 (TXD2, RXD2) */
		QueueHandle_t* uart2GetQueue();
		bool uart2Init(uint32_t baudRate);
		void uart2UpdateBaudrate(uint32_t baudRate);
		void uart2EnablePatternInterrupt(char pattern);
		void uart2InterruptPrint(uint16_t stopAfter);

		/** PHY/RF */
		//static bool customPhyInit(); // 155ms after reset, 6ms after deep sleep, WARNING: first function call in routine, otherwise wifi connect fails, reduces ESP NOW init time significantly
		bool fullRFCalibration(); // takes 145 - 160ms, WARNING: first function call in routine, otherwise wifi connect fails, reduces ESP NOW init time significantly
		bool onlyLoadRFCalibration(); // takes 5ms BUT ~81mA spike, WARNING: first function call in routine, otherwise wifi connect fails, reduces ESP NOW init time significantly
		// if none of the functions are called before initESPNOWStationary WITHOUT NVS, init will take around 160ms
		
		void printTxPower(); // call after initEspNow, in general AFTER esp_wifi_start() 

		/** ESP NOW, WARNING: needs 80MHz or more! can broadcast or send data to specific peers (one or more), but needs to know the MACs in advance */
		// also long preambles possible, check: https://blogs.ntu.edu.sg/ps9888-2020-g16/2020/07/11/esp-now-data-transfer/
		bool initESPNOWStationary(bool longrange, int8_t maxTxPower, bool withoutNVS, wifi_phy_rate_t phyRate = WIFI_PHY_RATE_1M_L);
		bool initESPNOW(bool longrange, int8_t maxTxPower, bool withoutNVS, wifi_phy_rate_t phyRate = WIFI_PHY_RATE_1M_L); // @80MHz: nowadays 325ms (withoutNVS = 166ms), was: 136ms -> might be less without NVS
		void stopESPNOW();
		bool addESPNOWBroadcastReceiver(); // 1ms
		bool addESPNOWBroadcastReceiverStationary(); // 1ms
		bool addESPNOWReceiver(uint8_t *mac);
		bool addESPNOWReceiverStationary(uint8_t *mac);
		bool broadcastESPNOWData(uint8_t *data, uint8_t len); // call initESPNOW and addESPNOWBroadcastReceiver before, 250 bytes of data: 0 - 1ms (!!)
		bool sendESPNOWData(uint8_t *data, uint8_t len); // call initESPNOW and addESPNOWReceiver before
		bool sendESPNOWDataToMac(uint8_t *mac, uint8_t *data, uint8_t len); // call initESPNOW and addESPNOWReceiver before
		esp_now_sending_status_t getESPNOWSendingStatus(); // for polling, not for broadcasting, 330ms in case receiver not answering (timeout), 11ms in case receiver answered
		//esp_now_stream_status_t doESPNOWFlashStream(uint8_t *macAddress, uint32_t millisWaitIfSendFailed, uint16_t millisBetweenBlocks, uint32_t flashPointer, uint16_t &flashBlockPointer, uint8_t &flashPageInBlockPointer, uint8_t &flashSubPagePointer, uint16_t flashMaxNumberOfBlocksToTransmit, uint16_t minBatteryVoltageToContinue, bool debug, bool mockFlashRead, bool mockSending, bool mockFlashDelete); // MONSTER FUNCTION, activate flash before
		esp_now_stream_status_t doESPNOWFlashStreamNew(uint8_t *macAddress, uint8_t *customPrefixAdd, uint8_t customPrefixAddLength, uint32_t *sendPagePointer, uint16_t *sendPageOffsetPointer, uint32_t flashPageWritePointer, uint16_t flashPageWriteOffsetPointer, uint32_t maxBytesToSend, uint32_t millisWaitIfSendFailed, uint16_t maxSendRetries, uint16_t minBatteryVoltageToContinue, uint8_t debugLvl, bool mockFlashRead, bool mockSending); // MONSTER FUNCTION, activate flash before
		//esp_now_stream_status_t doESPNOWFlashStreamProximityData(uint8_t *macAddress, uint8_t prefix0, uint8_t prefix1, uint8_t prefix2, uint32_t *sendPagePointer, uint16_t *sendPageOffsetPointer, uint32_t flashPageWritePointer, uint16_t flashPageWriteOffsetPointer, uint32_t limitMessagesMax, uint16_t minBatteryVoltageToContinue, uint8_t debugLvl, bool mockFlashRead, bool mockSending); // MONSTER FUNCTION TIMES 2, activate flash before
	
		/** WiFi, WARNING: needs 80MHz or more! */
		// RELATED MENUCONFIG: CONFIG_ESP32_WIFI_NVS_ENABLED=y (this makes sense -> connectedToWiFi() will take over 2 seconds @118mA avg first time, afterwards only 780ms, even after power-on reset! nice!)
		// RELATED MENUCONFIG: # CONFIG_ESP32_PHY_CALIBRATION_AND_DATA_STORAGE is not set (full PHY calibration on every system reset (not deepsleep), recommended, increases boot time by around 100ms)
		bool initWiFi(); // @80MHz: 109-154ms -> around 15ms if CONFIG_ESP32_WIFI_NVS_ENABLED not enabled, but then 118mA for 2 more seconds during connect
		bool scanForWiFis(bool blocking, int8_t maxTxPower, uint32_t scanTimePerChannel = 120, uint8_t channel = WIFI_ALL_14_CHANNELS); // scanTimePerChannel = 30 worked always, WARNING! brownout possible, @80MHz: 2210ms if blocking (14 channels, 12-14 passive scans = each 350ms), 1331ms (channels 1-11), 118ms if not blocking, channel = 0 = scan all 14 channels
		bool scanForWiFisOn1and6and11(const char** ssids, const uint8_t wifiListSize, uint8_t *wifiArrayId, uint8_t *foundOnChannel, int8_t maxTxPower, uint32_t scanTimePerChannel, uint16_t timeoutScanMs);
		bool scanForWiFisOn1and6and11WithPriority(bool debug, const char** ssids, const uint8_t wifiListSize, uint8_t *wifiArrayId, uint8_t *foundOnChannel, int8_t maxTxPower, uint32_t scanTimePerChannel, uint16_t timeoutScanMs);
		bool scanForWiFisOn1and6and11and13WithPriority(bool debug, const char** ssids, const uint8_t wifiListSize, uint8_t *wifiArrayId, uint8_t *foundOnChannel, int8_t maxTxPower, uint32_t scanTimePerChannel, uint16_t timeoutScanMs);
		bool wiFiScanCompleted(); // for polling if scanForWiFis shall not block, default (all 14 channels): 2051ms (120ms scanTimePerChannel * 14 channels + some additional time), 1611ms if scanTimePerChannel = 80ms, 951ms if scanTimePerChannel = 20ms -> ONLY ONE CHANNEL SCAN = 311ms (!)
		bool wiFiScanIncludes(const char* ssid, uint8_t *foundChannel);
		bool wiFiScanIncludesArray(const char** ssids, const uint8_t wifiListSize, uint8_t *wifiArrayId, uint8_t *foundChannel);
		//bool wiFiScanIncludesArray(const char** ssid);
		bool connectToWiFiAfterScan(const char* ssid, const char* password, uint8_t channel); // using maxTxPower from scanForWiFis AND ALSO scanTimePerChannel (last setting)!
		uint8_t printWiFiScanResults();
		bool connectToWiFiDirectly(const char* ssid, const char* password, int8_t maxTxPower, uint8_t channel = WIFI_ALL_14_CHANNELS); // 119ms, needs initWiFi to be called before
		wifi_connect_status_t connectedToWiFi(); // for polling, Korntal needs 836/840/1336ms/3338ms until connected (callback called) / 1515-2012ms if connectToWiFiAfterScan is used, 2052-2224ms if WiFi AP not there -> if stored in NVS only 780ms
		void disconnectAndStopWiFi(); // 9ms

		/** REST Webservices */
		wifi_post_data_status_t getWiFiPOSTCallStatus(); // WORST CASE: 18 seconds (connected to WiFi, but no Internet)
		uint16_t getWiFiPostReturnCode();
		void doWiFiPOSTCall(post_task_parameters_t *globalPostParameters, const uint16_t taskStackSize = 8192); // tested with up to 49.152 Bytes (24*2048) payload, taskStackSize with new malloc implementation not important (no local variable), CREATES OWN TASK, connectedToWiFi needs to be true before!, globalPostParameters NEEDS TO BE GLOBAL VARIABLE, CONTENT OF IT AS WELL
		void doWiFiPOSTStreamCallFlashHalfBlock(post_task_stream_flash_parameters_t *globalPostParameters); // TURN FLASH ON BEFORE THAT, monster function: reading FIFO from flash, transmitting it as a http POST stream
		void doWiFiPOSTStreamCallFlashFullBlock(post_task_stream_flash_parameters_t *globalPostParameters); // TURN FLASH ON BEFORE THAT, monster function: reading FIFO from flash, transmitting it as a http POST stream
		void restPostStreamGetSuccessfullyTransmittedBlocks(uint16_t &wiFiPostStreamBlocksSuccessfullyTransmittedIn, uint16_t &wiFiPostStreamHalfBlocksSuccessfullyTransmittedIn);
		void killPOSTTask(); // LEADS TO RESET, DO NOT USE!!!

		/** NTP Time Synchronisation */
		bool getNTPTimestampUTCAndCompareAccuracy(bool storeInRTC, uint32_t &timestampUTC, uint16_t &milliseconds, int64_t &timestampDiffMs, const uint32_t timeoutMs, const char* serverAddr);
		bool getNTPTimestampUTC(bool storeInRTC, uint32_t &timestampUTC, uint16_t &milliseconds, const uint32_t timeoutMs, const char* serverAddr); // normally serverAddr = "pool.ntp.org"

		/** BLE (NIMBLE) */
		bool startBLE(ble_mode_t mode, uint8_t *data, uint8_t len); // 102mA average if advertising and scanning, 32mA only advertising
		void stopBLE(); // 500ms between start and stop = already received some beacons, <= 300ms = too short, 1000ms seems nice, 1100ms for proximity detection seems good
		static void printOwnBLEAddress();
		uint16_t getBLEBiologgerFound();
		int32_t getBLERSSISum(uint16_t index);
		int8_t getBLERSSIAverage(uint16_t index);
		uint16_t getBLEBeaconCnt(uint16_t index);
		//uint64_t getBLEMacAddress(uint16_t index);
		uint8_t getBLEPayloadLen(uint16_t index);
		uint8_t getBLEPayload(uint16_t deviceIndex, uint8_t byteIndex);
		uint16_t getBLEBiologgerId(uint16_t index);

		/** Helper functions */
		uint32_t measureTime(const char* text, bool dontPrint = false);

		/** Magnetometer calibration routine */
		bool magnetometerCalibrationMode(uint16_t durationSeconds, mag_calibration_t *calibrationData, bmm150_trim_registers *trimDataIn); // before: get trimData and start imu

		/** WildFiTagREV6 integrated modules */
		RTC_RV8803C7 rtc = RTC_RV8803C7();
		//BARO_BSEC_BME680 bsec = BARO_BSEC_BME680();
		BARO_BME680 baro = BARO_BME680();
		IMU_BMX160 imu = IMU_BMX160();
		FLASH_MT29 flash = FLASH_MT29();

	private:
		bool wiFiInitialized; // wifi was initialized
		bool espNowInitialized; // wifi was initialized
		bool BLEinitialized; // BLE was initialized
		bool NVSinitialized; // NVS (for wifi and BLE) was initialized
		bool NVSForDataInitialized; // NVS (for wifi and BLE) was initialized
		bool ADCinitialized; // ADC was initialized
		bool pinsInitialized; // pins are initialized
		bool powerPinInitialized; // power pin is initialized
		bool power2PinInitialized; // power2 pin is initialized
		uint32_t lastTime;

		QueueHandle_t uart2Queue;
		
		bool checkBLEOkay(uint16_t index);
		bool wiFiSetCountryToUseChannel1to11();
};

#endif
