#ifndef ModuleRTC_RV3028C7_h
#define ModuleRTC_RV3028C7_h

// TODO: bringing BREAKTIME function from TimeLib.h in here

#include <stdint.h>
#include "HelperTime.h" // for setTimeWithUTCCompileTime
#include "HelperBits.h"
#include "InterfaceI2C.h"
#include "InterfaceTiming.h"
	
#define RTC_RV3028C7_ADDRESS		0x52

#define REG_TIMESTAMP				0x1B

#define REG_SECONDS					0x00
#define REG_MINUTES					0x01
#define REG_HOURS					0x02

#define REG_WEEKDAY					0x03
#define REG_DATE					0x04
#define REG_MONTH					0x05
#define REG_YEAR					0x06

#define REG_TIMER_VAL0				0x0A
#define REG_TIMER_VAL1				0x0B

#define REG_STATUS					0x0E
#define REG_STATUS_AF				2
#define REG_STATUS_TF				3
#define REG_STATUS_UF				4

#define REG_CONTROL1				0x0F
#define REG_CONTROL1_TD0			0
#define REG_CONTROL1_TD1			1
#define REG_CONTROL1_TE				2
#define REG_CONTROL1_USEL			4
#define REG_CONTROL1_WADA			5
#define REG_CONTROL1_TRPT			7

#define REG_CONTROL2				0x10
#define REG_CONTROL2_RESET			0
#define REG_CONTROL2_AIE			3
#define REG_CONTROL2_TIE			4
#define REG_CONTROL2_UIE			5

#define REG_ALARM_MINUTE			0x07
#define REG_ALARM_MINUTE_AE_M		7

#define REG_ALARM_HOUR				0x08
#define REG_ALARM_HOUR_AE_H			7
	
#define REG_ALARM_WEEKDAY			0x09
#define REG_ALARM_WEEKDAY_AE_WD		7

#define REG_EEPROM_CLKOUT			0x35

#define WEEKDAY_MON					0x00
#define WEEKDAY_TUE					0x01
#define WEEKDAY_WED					0x02
#define WEEKDAY_THU					0x03
#define WEEKDAY_FRI					0x04
#define WEEKDAY_SAT					0x05
#define WEEKDAY_SUN					0x06

#define INTERRUPT_DAILY				0x00
#define INTERRUPT_HOURLY			0x01
#define NO_INTERRUPT				0x02

#define INT_REASON_CALENDAR			0x00
#define INT_REASON_REGULAR			0x01

#define SET_UTC_COMP_TIME_OFFSET    		16      // seconds offset for time setting during compilation (depends on your computer)
#define SET_UTC_COMP_TIME_GMT2_OFFSET		7200    // in Germany: 2 hours in seconds to get UTC time

class RTC_RV3028C7 {
	public:
		RTC_RV3028C7();
		
		bool disableClockOut(); // SAVES 0.6uA in deep sleep! could also be stored in EEPROM

		uint32_t getTimestamp(bool &error);
		bool setTimestamp(uint32_t timestamp);
		
		uint8_t setTimeWithUTCCompileTime(bool resetIssuedByUPDI); // needs to be called as soon as possible in setup method
		
		// calendar time running independently of timestamp!
		uint8_t getSeconds(bool &error);
		uint8_t getMinutes(bool &error);
		uint8_t getHours(bool &error);
		uint8_t getWeekday(bool &error);
		uint8_t getDate(bool &error);
		uint8_t getMonth(bool &error);
		uint16_t getYear(bool &error);
	
		bool set(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t weekday, uint8_t date, uint8_t month, uint16_t year);

		bool setRegularInterrupt(uint16_t seconds); // pin goes low for 7.813ms, first period has time uncertainty of 15.625ms (1Hz timer)
		bool disableRegularInterrupt();

		bool setTimeUpdateInterruptMinuteChange(); // on every minute change, use set for time setting before

		bool setDailyInterrupt(uint8_t hour, uint8_t minute); // NO AUTOMATIC CLEAR OF INT PIN, DOES NOT USE TIMESTAMP, only set function
		bool setHourlyInterrupt(uint8_t minute); // NO AUTOMATIC CLEAR OF INT PIN, DOES NOT USE TIMESTAMP, only set function
		bool disableHourlyDailyInterrupt(); 
		
		bool dailyOrHourlyInterruptHappened(bool &error);
		bool regularInterruptHappened(bool &error);
		
		bool resetInterruptFlags();
		
	private:
		bool setCalendarInterrupt(uint8_t mode, uint8_t hours, uint8_t minutes);
};

#endif
