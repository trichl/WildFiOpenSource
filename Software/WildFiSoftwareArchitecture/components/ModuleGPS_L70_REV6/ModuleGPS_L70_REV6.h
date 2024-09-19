#ifndef ModuleGPS_L70_REV6_h
#define ModuleGPS_L70_REV6_h

#include "PlatformWildFiTagREV6.h"

#define NMEA_MAX_STATEMENT_ITEM_LENGTH 			16
#define GPS_MAX_SATELLITES_IN_USE 				12
#define GPS_MAX_SATELLITES_IN_VIEW 				16

#define RTC_UPDATE_THRESHOLD					20  // Don't set RTC if time difference is less than +/-x ms
#define UART_OFFSETS_LENGTH                     8
#define UART_OFFSETS_OFFSET                     4 // How many sats on index 0 of UARToffsets array
constexpr uint32_t UARToffsets[UART_OFFSETS_LENGTH] = {73,77,142,155,167,179,194,225};

typedef struct {
    uint32_t timeoutSeconds;                        // timeout
    uint32_t timeoutNotEvenTimeSeconds;             // timeout (date still 1980)
    float minHDOP;                                  // minimum HDOP that needs to be reached before stopping the fix attempt
    uint8_t afterFixMaxWaitOnHDOP;                  // maximum wait time in seconds
    bool waitAfterFixUntilZeroMs;                   // after getting a fix (including HDOP requirements): additionally wait until messages arrive at .000 ms
    bool usePPSReference;                           // compensate delay introduced by uart communication by using pulse per second (pps) output of GPS module to more precisely determine time
    bool setSYSTime;                                // set ESP system time after fix
    bool setRTCTime;                                // last action when got a fix: update the RTC time
    bool blinkLeds;                                 // red = no fix, green = fix
    bool debug;                                     // debug output
} gps_get_fix_config_t;

typedef enum {
    GPS_FIX_INVALID, /*!< Not fixed */
    GPS_FIX_GPS,     /*!< GPS */
    GPS_FIX_DGPS,    /*!< Differential GPS */
} gps_fix_t;

typedef enum {
    GPS_MODE_INVALID = 1, /*!< Not fixed */
    GPS_MODE_2D,          /*!< 2D GPS */
    GPS_MODE_3D           /*!< 3D GPS */
} gps_fix_mode_t;

typedef struct {
    uint8_t num;       /*!< Satellite number */
    uint8_t elevation; /*!< Satellite elevation */
    uint16_t azimuth;  /*!< Satellite azimuth */
    uint8_t snr;       /*!< Satellite signal noise ratio */
} gps_satellite_t;

typedef struct {
    uint8_t hour;      /*!< Hour */
    uint8_t minute;    /*!< Minute */
    uint8_t second;    /*!< Second */
    uint16_t thousand; /*!< Thousand */
} gps_time_t;

typedef struct {
    uint8_t day;   /*!< Day (start from 1) */
    uint8_t month; /*!< Month (start from 1) */
    uint16_t year; /*!< Year (start from 2000) */
} gps_date_t;

typedef enum {
    STATEMENT_UNKNOWN = 0, /*!< Unknown statement */
    STATEMENT_GGA,         /*!< GGA */
    STATEMENT_GSA,         /*!< GSA */
    STATEMENT_RMC,         /*!< RMC */
    STATEMENT_GSV,         /*!< GSV */
    STATEMENT_GLL,         /*!< GLL */
    STATEMENT_VTG          /*!< VTG */
} nmea_statement_t;

typedef enum {
    GPS_FIX_RESULT_NOT_ANSWERING = 0,
    GPS_FIX_RESULT_SETTINGS_NO_ANSWER,
    GPS_FIX_MALLOC_FAIL,
    GPS_FIX_TIMER_FAIL,
    GPS_FIX_TIMEOUT,
    GPS_FIX_NO_ANSWER_FATAL,
    GPS_FIX_RTC_CONFIG_FAIL,
    GPS_FIX_SUCCESS_NO_RTC_UPDATE,
    GPS_FIX_SUCCESS_AND_RTC_UPDATED
} get_fix_result_t;

typedef enum {
    GPS_DECODE_RESULT_SUCESS = 0,
    GPS_DECODE_RESULT_CRC_ERR,
    GPS_DECODE_RESULT_UNKNOWN_STATEMENT_ERR,
} get_decode_result_t;

typedef enum {
    GPS_UART_RESULT_SUCESS = 0,
    GPS_UART_RESULT_TIMEOUT,
    GPS_UART_RESULT_INIT_ERROR,
    GPS_UART_RESULT_BUFF_OVERFLOW_ERROR,
    GPS_UART_RESULT_DECODE_ERROR,
    GPS_UART_RESULT_ORDER_INCORRECT,
    GPS_UART_RESULT_NOT_VALID_ERROR
} get_uart_result_t;

typedef struct {
    float latitude;                                                /*!< Latitude (degrees) */
    float longitude;                                               /*!< Longitude (degrees) */
    float altitude;                                                /*!< Altitude (meters) */
    gps_fix_t fix;                                                 /*!< Fix status */
    uint8_t sats_in_use;                                           /*!< Number of satellites in use */
    gps_time_t tim;                                                /*!< time in UTC */
    gps_fix_mode_t fix_mode;                                       /*!< Fix mode */
    uint8_t sats_id_in_use[GPS_MAX_SATELLITES_IN_USE];             /*!< ID list of satellite in use */
    float dop_h;                                                   /*!< Horizontal dilution of precision */
    float dop_p;                                                   /*!< Position dilution of precision  */
    float dop_v;                                                   /*!< Vertical dilution of precision  */
    uint8_t sats_in_view;                                          /*!< Number of satellites in view */
    gps_satellite_t sats_desc_in_view[GPS_MAX_SATELLITES_IN_VIEW]; /*!< Information of satellites in view */
    gps_date_t date;                                               /*!< Fix date */
    bool valid;                                                    /*!< GPS validity */
    float speed;                                                   /*!< Ground speed, unit: m/s */
    float cog;                                                     /*!< Course over ground */
    float variation;                                               /*!< Magnetic variation */
    uint32_t ttfMilliseconds;                                      /*!< Time to fix in milliseconds */
    uint32_t utcTimestamp;                                         /*!< UTC Timestamp calculated by date and tim */
} gps_t;

typedef struct {
    uint8_t item_pos;                              /*!< Current position in item */
    uint8_t item_num;                              /*!< Current item number */
    uint8_t asterisk;                              /*!< Asterisk detected flag */
    uint8_t crc;                                   /*!< Calculated CRC value */
    uint8_t parsed_statement;                      /*!< OR'd of statements that have been parsed */
    uint8_t sat_num;                               /*!< Satellite number */
    uint8_t sat_count;                             /*!< Satellite count */
    uint8_t cur_statement;                         /*!< Current statement ID */
    uint32_t all_statements;                       /*!< All statements mask */
    char item_str[NMEA_MAX_STATEMENT_ITEM_LENGTH]; /*!< Current item */
    gps_t parent;                                  /*!< Parent class */
} esp_gps_t;

class GPS_L70_REV6 {
	public:
		GPS_L70_REV6();
		void init(QueueHandle_t* queueHandleIn);
        bool isStarted();
		bool setBaudrate115200();
		void permanentlySetBaudrate115200();
        bool setFLPMode(bool permanently);
        bool setFLPModeOnce();
        bool setNormalMode(bool permanently);
		bool setNMEAMessagesMinimum();
        bool setNMEAMessagesMinimum1Hz();
        bool setNMEAMessagesMinimum1HzWithZDA();
		bool waitForAnswer(const char* answer, uint16_t timeousMs);
        get_uart_result_t afterLightSleepWaitForGPRMCandGPGGA(esp_gps_t *gpsData, bool *hasValidTimestamp, uint16_t *uartMillisWait, bool debug);
        get_fix_result_t tryToGetFix(esp_gps_t *gpsData, gps_get_fix_config_t *config, WildFiTagREV6 *device); // uses afterLightSleepWaitForGPRMCandGPGGA
        get_fix_result_t tryToGetFixV2(esp_gps_t *gpsData, gps_get_fix_config_t *config, WildFiTagREV6 *device);
		get_decode_result_t gpsDecodeLine(esp_gps_t *esp_gps, char *d, uint32_t len);
        bool gpsDecodeCorruptedGPRMCLine(esp_gps_t *esp_gps, const char* message);
        uint32_t estimateUARTSendTimeMs(uint32_t messageLength);
        bool updateUTCTimestamp(esp_gps_t *gpsData);
        void checkAliveAndMaybeChangeBaudrate(WildFiTagREV6 *device, bool blinkIfError, bool debug);

	private:
    	float parseLatLong(esp_gps_t *esp_gps);
		uint8_t convertTwoDigit2Number(const char *digit_char);
		void parseUTCTime(esp_gps_t *esp_gps);
		void parseGGA(esp_gps_t *esp_gps);
		void parseGSA(esp_gps_t *esp_gps);
		void parseGSV(esp_gps_t *esp_gps);
		void parseRMC(esp_gps_t *esp_gps);
		void parseGLL(esp_gps_t *esp_gps);
		void parseVTG(esp_gps_t *esp_gps);
		bool gpsDecodeParse(esp_gps_t *esp_gps);
		QueueHandle_t* queueHandle;
};

#endif
