#ifndef ModuleRTC_RV8803C7_h
#define ModuleRTC_RV8803C7_h

// TODO: bringing BREAKTIME function from TimeLib.h in here

#include <stdint.h>
#include "HelperTime.h" // for setTimeWithUTCCompileTime
#include "HelperBits.h"
#include "InterfaceI2C.h"
#include "InterfaceTiming.h"
	
#define RTC_RV8803C7_ADDRESS			0x32

#define REG8803_SECONDS					0x00
#define REG8803_MINUTES					0x01
#define REG8803_HOURS					0x02

#define REG8803_WEEKDAY					0x03 // WARNING: different content than RV3028C7
#define REG8803_DATE					0x04
#define REG8803_MONTH					0x05
#define REG8803_YEAR					0x06

#define REG8803_EXTENSION				0x0D
#define REG8803_EXTENSION_TD0			0
#define REG8803_EXTENSION_TD1			1
#define REG8803_EXTENSION_TE			4
#define REG8803_EXTENSION_USEL			5
#define REG8803_EXTENSION_WADA			6

#define REG8803_FLAG					0x0E
#define REG8803_FLAG_V1F				0
#define REG8803_FLAG_V2F				1
#define REG8803_FLAG_AF					3
#define REG8803_FLAG_TF					4
#define REG8803_FLAG_UF					5

#define REG8803_CONTROL					0x0F
#define REG8803_CONTROL_RESET			0
#define REG8803_CONTROL_AIE				3
#define REG8803_CONTROL_TIE				4
#define REG8803_CONTROL_UIE				5

#define REG8803_100TH_SECONDS			0x10

#define REG8803_TIMER_CNT0				0x0B
#define REG8803_TIMER_CNT1				0x0C

#define INTERRUPT_DAILY					0x00
#define INTERRUPT_HOURLY				0x01
#define NO_INTERRUPT					0x02

#define REG8803_ALARM_MINUTE			0x08
#define REG8803_ALARM_MINUTE_AE_M		7

#define REG8803_ALARM_HOUR				0x09
#define REG8803_ALARM_HOUR_AE_H			7
	
#define REG8803_ALARM_WEEKDAY			0x0A
#define REG8803_ALARM_WEEKDAY_AE_WD		7

class RTC_RV8803C7 {
	public:
		RTC_RV8803C7();
		bool timeIsValidNoUndervoltage(bool &error);
		bool set(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t weekday, uint8_t date, uint8_t month, uint16_t year);
		uint32_t getTimestampInternal(bool &error);
		bool getTimestamp(uint32_t *timestamp, uint8_t *hundreds);
		uint8_t getSeconds(bool &error);
		uint8_t getMinutes(bool &error);
		uint8_t getHours(bool &error);
		uint8_t getDate(bool &error);
		uint8_t getMonth(bool &error);
		uint16_t getYear(bool &error);
		uint8_t get100thOfSeconds(bool &error);
		bool setSeconds(uint8_t seconds);

		// interrupt on every full minute
		bool setTimeUpdateInterruptMinuteChange(); // UIE, UF, USEL -> on every minute change, use set function for time setting before
		bool disableTimeUpdateInterruptMinuteChange(); // UIE, UF, USEL
		bool reenableTimeUpdateInterruptMinuteChange(); 
		bool timeUpdateInterruptMinuteChangeHappened(bool &error);

		// interrupt every 1 .. 4096 seconds
		bool setRegularInterrupt(uint16_t seconds); // TIE, TF, TE -> pin goes low for 7.813ms, first period has time uncertainty of 15.625ms (1Hz timer)
		bool disableRegularInterrupt(); // TIE, TF, TE
		bool regularInterruptHappened(bool &error); // TIE, TF, TE

		// interrupt every day or every hour (WARNING: NO AUTOMATIC CLEAR OF INT PIN)
		bool setDailyInterrupt(uint8_t hour, uint8_t minute); // AIE, AF, AE_M, AE_H, AE_WD -> NO AUTOMATIC CLEAR OF INT PIN
		bool setHourlyInterrupt(uint8_t minute); // AIE, AF, AE_M, AE_H, AE_WD -> NO AUTOMATIC CLEAR OF INT PIN
		bool disableHourlyDailyInterrupt(); // AIE, AF, AE_M, AE_H, AE_WD 
		bool dailyOrHourlyInterruptHappened(bool &error); // AIE, AF, AE_M, AE_H, AE_WD
		
		// resetting of all interrupts (important for daily and hourly interrupts)
		bool resetInterruptFlags(); // UF, TF, AF
	private:
		bool setCalendarInterrupt(uint8_t mode, uint8_t hours, uint8_t minutes);
};

#endif
