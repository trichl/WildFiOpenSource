#include "ModuleRTC_RV8803C7.h"

RTC_RV8803C7::RTC_RV8803C7() {

}

bool RTC_RV8803C7::timeIsValidNoUndervoltage(bool &error) {
	bool timeValid = true;
	uint8_t reg_flag = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, error);
	if(error) { return false; }

	if((HelperBits::getBit(reg_flag, REG8803_FLAG_V1F) == 1) || (HelperBits::getBit(reg_flag, REG8803_FLAG_V2F) == 1)) {
		timeValid = false;
	
		// reset the flag (is always set after POR)
		reg_flag = HelperBits::setBit(reg_flag, REG8803_FLAG_V1F, 0);
		reg_flag = HelperBits::setBit(reg_flag, REG8803_FLAG_V2F, 0);
		error |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, reg_flag);

		// new: set time to "0", otherwise after restart might clear under voltage but when resetting it is okay again
		set(0, 0, 0, 0, 1, 1, 2004);
	}
	return timeValid;
}

bool RTC_RV8803C7::setSeconds(uint8_t seconds) {
	return i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_SECONDS, HelperBits::intToBCD(seconds));
}

bool RTC_RV8803C7::set(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t weekday, uint8_t date, uint8_t month, uint16_t year) {
	const uint8_t LEN = 7;
	uint8_t vals[LEN];

	if((seconds > 59) || (minutes > 59) || (hours > 23) || (weekday > 6) || (date > 31) || (date == 0) || (month > 12) || (month == 0) || (year > 2099) || (year < 2000)) {
		return false;
	}
	
	vals[0] = HelperBits::intToBCD(seconds);
	vals[1] = HelperBits::intToBCD(minutes);
	vals[2] = HelperBits::intToBCD(hours);
	if(weekday == 0) { vals[3] = 0b00000001; }
	else if(weekday == 1) { vals[3] = 0b00000010; }
	else if(weekday == 2) { vals[3] = 0b00000100; }
	else if(weekday == 3) { vals[3] = 0b00001000; }
	else if(weekday == 4) { vals[3] = 0b00010000; }
	else if(weekday == 5) { vals[3] = 0b00100000; }
	else if(weekday == 6) { vals[3] = 0b01000000; }
	vals[4] = HelperBits::intToBCD(date);
	vals[5] = HelperBits::intToBCD(month);
	uint16_t yearWithout2000 = year - 2000;
	vals[6] = HelperBits::intToBCD((uint8_t) yearWithout2000);
	
	return i2c.writeRegisterBase(RTC_RV8803C7_ADDRESS, REG8803_SECONDS, vals, LEN);
}

bool RTC_RV8803C7::getTimestamp(uint32_t *timestamp, uint8_t *hundreds) {
	bool error = false;
	if(timestamp == NULL) { return false; }
	*timestamp = getTimestampInternal(error); // will re-read second time when seconds = 59
	if(error) {
		*timestamp = 0;
		*hundreds = 0;
		return false;
	}
	if(hundreds != NULL) {
		*hundreds = get100thOfSeconds(error);
		if(error) {
			*timestamp = 0;
			*hundreds = 0;
			return false;
		}
		// happens sometimes: millis already 0, but rest not updated -> re-read timestamp
		if(*hundreds == 0) { 
			*timestamp = getTimestampInternal(error); // will re-read second time when seconds = 59
			if(error) {
				*timestamp = 0;
				*hundreds = 0;
				return false;
			}
		}
	}	
	return true;
}

uint32_t RTC_RV8803C7::getTimestampInternal(bool &error) {
	uint8_t seconds, minutes, hours, day, month, yearWithout2000;
	uint16_t yearWith2000;
	const uint8_t LEN = 7;
	uint8_t vals[LEN];
	if(!i2c.readRegisterPageArray(RTC_RV8803C7_ADDRESS, REG8803_SECONDS, vals, LEN)) {
		error = true;
		return 0;
	}
	seconds = HelperBits::BCDToInt(vals[0]);
	minutes = HelperBits::BCDToInt(vals[1]);
	hours = HelperBits::BCDToInt(vals[2]);
	//uint8_t weekday = HelperBits::BCDToInt(vals[3]); // not used
	day = HelperBits::BCDToInt(vals[4]);
	month = HelperBits::BCDToInt(vals[5]);
	yearWithout2000 = HelperBits::BCDToInt(vals[6]);
	yearWith2000 = 2000 + yearWithout2000;

	if(seconds == 59) { // according to datasheet: read a second time to avoid inconsistent data
		if(!i2c.readRegisterPageArray(RTC_RV8803C7_ADDRESS, REG8803_SECONDS, vals, LEN)) {
			error = true;
			return 0;
		}
		if(HelperBits::BCDToInt(vals[0]) != seconds) { // there was a wrap around in case first second read isn't the same like second read
			// second data read is valid -> update data
			seconds = HelperBits::BCDToInt(vals[0]);
			minutes = HelperBits::BCDToInt(vals[1]);
			hours = HelperBits::BCDToInt(vals[2]);
			//uint8_t weekday = HelperBits::BCDToInt(vals[3]); // not used
			day = HelperBits::BCDToInt(vals[4]);
			month = HelperBits::BCDToInt(vals[5]);
			yearWithout2000 = HelperBits::BCDToInt(vals[6]);
			yearWith2000 = 2000 + yearWithout2000;
		}
		// otherwise: still 59 -> first data is valid
	}
	if(day == 0) { day = 1; } // RESET value is 0 (but month is 1)
	//if(yearWith2000 == 2000) { yearWith2000 = 1970; } // RESET value is 0 + 2000 = 2000 -> timestamp starts in 2000, better start in 1970
	uint32_t timestamp = _UNIX_TIMESTAMP(yearWith2000, month, day, hours, minutes, seconds);
	return timestamp;
}

uint8_t RTC_RV8803C7::getSeconds(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_SECONDS, error));
}

uint8_t RTC_RV8803C7::getMinutes(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_MINUTES, error));
}

uint8_t RTC_RV8803C7::getHours(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_HOURS, error));
}

uint8_t RTC_RV8803C7::getDate(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_DATE, error));
}

uint8_t RTC_RV8803C7::getMonth(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_MONTH, error));
}

uint16_t RTC_RV8803C7::getYear(bool &error) {
	uint8_t yearWithout2000 = HelperBits::BCDToInt(i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_YEAR, error));
	uint16_t yearWith2000 = 2000 + yearWithout2000;
	return yearWith2000;
}

uint8_t RTC_RV8803C7::get100thOfSeconds(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_100TH_SECONDS, error));
}

bool RTC_RV8803C7::timeUpdateInterruptMinuteChangeHappened(bool &error) {
	bool happened = i2c.readRegisterBit(RTC_RV8803C7_ADDRESS, REG8803_FLAG, REG8803_FLAG_UF, error);
	if(happened) { // reset when happened
		uint8_t reg_flag = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, error);
		if(error) { return false; }
		reg_flag = HelperBits::setBit(reg_flag, REG8803_FLAG_UF, 0); // UF = 0 (clear setTimeUpdateInterruptMinuteChange interrupt)
		return i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, reg_flag);
	}
	return happened;
}

bool RTC_RV8803C7::setTimeUpdateInterruptMinuteChange() {
	bool readError = false;
	bool writeError = false;

	// read relevant registers
	uint8_t reg_control = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, readError);
	if(readError) { return false; }
	uint8_t reg_flag = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, readError);
	if(readError) { return false; }
	uint8_t reg_extension = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_EXTENSION, readError);
	if(readError) { return false; }
	
	// initialize UIE and UF to 0
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_UIE, 0)); // 0x0F: UIE = 0 (disable interrupts shortly)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, HelperBits::setBit(reg_flag, REG8803_FLAG_UF, 0)); // 0x0E: UF = 0 (clear timer interrupt event)
	
	// choose timer source
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_EXTENSION, HelperBits::setBit(reg_extension, REG8803_EXTENSION_USEL, 1)); // 0x0D: USEL = 1 (update on minute, auto reset time = 7.813ms, 0 would be on second)

	// enable interrupt pin
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_UIE, 1));
	
	return !writeError;
}

bool RTC_RV8803C7::disableTimeUpdateInterruptMinuteChange() {
	bool readError = false;
	bool writeError = false;

	// read relevant registers
	uint8_t reg_control = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, readError);
	if(readError) { return false; }
	uint8_t reg_flag = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, readError);
	if(readError) { return false; }
	
	// initialize UIE and UF to 0
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_UIE, 0)); // 0x0F: UIE = 0 (disable interrupt)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, HelperBits::setBit(reg_flag, REG8803_FLAG_UF, 0)); // 0x0E: UF = 0 (clear timer interrupt event)

	return !writeError;
}

bool RTC_RV8803C7::reenableTimeUpdateInterruptMinuteChange() {
	bool readError = false;
	bool writeError = false;

	// read relevant registers
	uint8_t reg_control = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, readError);
	if(readError) { return false; }
	
	// initialize UIE to 1 again (interrupts happening)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_UIE, 1)); // 0x0F: UIE = 1 (enable interrupt)

	return !writeError;
}

bool RTC_RV8803C7::setRegularInterrupt(uint16_t seconds) {
	uint8_t sec_lwr = ((seconds >> 0) & 0xff);
	uint8_t sec_upr = ((seconds >> 8) & 0xff);
	
	bool readError = false;
	bool writeError = false;
	if((seconds > 4096) || (seconds == 0)) { // 12 bit maximum size, max ~1 hour (can be higher if clock frequency TD is set lower)
		return false;
	}
	
	// read relevant registers
	uint8_t reg_control = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, readError);
	if(readError) { return false; }
	uint8_t reg_flag = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, readError);
	if(readError) { return false; }
	uint8_t reg_extension = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_EXTENSION, readError);
	if(readError) { return false; }
	
	// write relevant registers
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_TIE, 0)); // 0x0F: TIE = 0 (disable interrupts shortly)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, HelperBits::setBit(reg_flag, REG8803_FLAG_TF, 0)); // 0x0E: TF = 0 (clear timer interrupt event)
	
	reg_extension = HelperBits::setBit(reg_extension, REG8803_EXTENSION_TE, 0); // 0x0D: TE = 0 (disable timer)
	reg_extension = HelperBits::setBit(reg_extension, REG8803_EXTENSION_TD0, 0); // 0x0D: TD = 10 (countdown clock frequency = 1Hz = 1s)
	reg_extension = HelperBits::setBit(reg_extension, REG8803_EXTENSION_TD1, 1); // 0x0D: TD = 10 (countdown clock frequency = 1Hz = 1s)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_EXTENSION, reg_extension);
	
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_TIMER_CNT0, sec_lwr); // 0x0B: Countdown value, lower byte (seconds)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_TIMER_CNT1, sec_upr); // 0x0C: Countdown value, upper byte (seconds)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_TIE, 1)); // 0x0F: TIE = 1 (get INT pin interrupts)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_EXTENSION, HelperBits::setBit(reg_extension, REG8803_EXTENSION_TE, 1)); // 0x0D: TE = 1 (enable countdown) -> NOW STARTING!!!
	
	return !writeError;
}

bool RTC_RV8803C7::disableRegularInterrupt() {
	bool readError = false;
	bool writeError = false;

	// read relevant registers
	uint8_t reg_control = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, readError);
	if(readError) { return false; }
	uint8_t reg_flag = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, readError);
	if(readError) { return false; }
	uint8_t reg_extension = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_EXTENSION, readError);
	if(readError) { return false; }
	
	// write relevant registers
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_TIE, 0)); // 0x0F: TIE = 0 (disable interrupts shortly)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, HelperBits::setBit(reg_flag, REG8803_FLAG_TF, 0)); // 0x0E: TF = 0 (clear timer interrupt event)
	
	reg_extension = HelperBits::setBit(reg_extension, REG8803_EXTENSION_TE, 0); // 0x0D: TE = 0 (disable timer)
	reg_extension = HelperBits::setBit(reg_extension, REG8803_EXTENSION_TD0, 0); // 0x0D: TD = 10 (countdown clock frequency = 1Hz = 1s)
	reg_extension = HelperBits::setBit(reg_extension, REG8803_EXTENSION_TD1, 1); // 0x0D: TD = 10 (countdown clock frequency = 1Hz = 1s)
	writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_EXTENSION, reg_extension);
	
	return !writeError;
}

bool RTC_RV8803C7::regularInterruptHappened(bool &error) {
	return i2c.readRegisterBit(RTC_RV8803C7_ADDRESS, REG8803_FLAG, REG8803_FLAG_TF, error);
}

bool RTC_RV8803C7::setCalendarInterrupt(uint8_t mode, uint8_t hour, uint8_t minute) {
	bool readError = false;
	bool writeError = false;
	if(minute > 59 || hour > 23) {
		return false;
	}
	
	// read relevant registers
	uint8_t reg_control = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, readError);
	if(readError) { return false; }
	uint8_t reg_flag = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, readError);
	if(readError) { return false; }
	uint8_t reg_extension = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_EXTENSION, readError);
	if(readError) { return false; }

	bool AE_M = false, AE_H = false, AE_WD = false;
	if(mode == INTERRUPT_DAILY) {
		AE_M = false;
		AE_H = false;
		AE_WD = true;
	}
	else if(mode == INTERRUPT_HOURLY) {
		AE_M = false;
		AE_H = true;
		AE_WD = true;		
	}
	else { // NO_INTERRUPT, 1 = disabled
		AE_M = true;
		AE_H = true;
		AE_WD = true;		
	}
	
	// write relevant registers
	if((mode == INTERRUPT_DAILY) || (mode == INTERRUPT_HOURLY)) { // enable interrupts
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_AIE, 0)); // AIE = 0
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, HelperBits::setBit(reg_flag, REG8803_FLAG_AF, 0)); // AF = 0
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_EXTENSION, HelperBits::setBit(reg_extension, REG8803_EXTENSION_WADA, 1)); // WADA = 1 (weekday alarm disabled)
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_ALARM_MINUTE, HelperBits::setBit(HelperBits::intToBCD(minute), REG8803_ALARM_MINUTE_AE_M, AE_M)); // Minutes alarm register, AE_M = 0 (enabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_ALARM_HOUR, HelperBits::setBit(HelperBits::intToBCD(hour), REG8803_ALARM_HOUR_AE_H, AE_H)); // Hours alarm register, AE_H = 0 (enabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_ALARM_WEEKDAY, HelperBits::setBit(0x0, REG8803_ALARM_WEEKDAY_AE_WD, AE_WD)); // Weekday alarm register, AE_WD = 1 (disabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_AIE, 1)); // AIE = 1 (get INT pin interrupts)
	}
	else { // disable interrupts
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_CONTROL, HelperBits::setBit(reg_control, REG8803_CONTROL_AIE, false)); // AIE = 0
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, HelperBits::setBit(reg_flag, REG8803_FLAG_AF, false)); // AF = 0
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_ALARM_MINUTE, HelperBits::setBit(0x0, REG8803_ALARM_MINUTE_AE_M, AE_M)); // Minutes alarm register, AE_M = 0 (enabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_ALARM_HOUR, HelperBits::setBit(0x0, REG8803_ALARM_HOUR_AE_H, AE_H)); // Hours alarm register, AE_H = 0 (enabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_ALARM_WEEKDAY, HelperBits::setBit(0x0, REG8803_ALARM_WEEKDAY_AE_WD, AE_WD)); // Weekday alarm register, AE_WD = 1 (disabled, when hours + minutes match, once per day)
	}

	return !writeError;
}

bool RTC_RV8803C7::setDailyInterrupt(uint8_t hour, uint8_t minute) {
	return setCalendarInterrupt(INTERRUPT_DAILY, hour, minute);
}

bool RTC_RV8803C7::setHourlyInterrupt(uint8_t minute) {
	return setCalendarInterrupt(INTERRUPT_HOURLY, 0x0, minute);
}

bool RTC_RV8803C7::dailyOrHourlyInterruptHappened(bool &error) {
	return i2c.readRegisterBit(RTC_RV8803C7_ADDRESS, REG8803_FLAG, REG8803_FLAG_AF, error);
}

bool RTC_RV8803C7::disableHourlyDailyInterrupt() {
	return setCalendarInterrupt(NO_INTERRUPT, 0, 0);
}

bool RTC_RV8803C7::resetInterruptFlags() {
	bool readError = false;
	uint8_t reg_flag = i2c.readRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, readError);
	if(readError) { return false; }
	reg_flag = HelperBits::setBit(reg_flag, REG8803_FLAG_AF, 0); // AF = 0 (clear daily or hourly interrupt -> mandatory to reset INT pin)
	reg_flag = HelperBits::setBit(reg_flag, REG8803_FLAG_TF, 0); // TF = 0 (clear regular interrupt)
	reg_flag = HelperBits::setBit(reg_flag, REG8803_FLAG_UF, 0); // UF = 0 (clear setTimeUpdateInterruptMinuteChange interrupt)
	return i2c.writeRegister(RTC_RV8803C7_ADDRESS, REG8803_FLAG, reg_flag);
}