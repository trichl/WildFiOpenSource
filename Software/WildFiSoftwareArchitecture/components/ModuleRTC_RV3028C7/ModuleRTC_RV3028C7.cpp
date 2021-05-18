#include "ModuleRTC_RV3028C7.h"

RTC_RV3028C7::RTC_RV3028C7() {

}

bool RTC_RV3028C7::disableClockOut() {
	bool readError = false;
	uint8_t reg;
	reg = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_EEPROM_CLKOUT, readError);
	if(readError) {
		return false;
	}
	if(reg == 0x00) { // CLKOUT already disabled -> 0b 1100 0000 would be enabled
		return true;
	}
	else {
		if(!i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_EEPROM_CLKOUT, 0x00)) { // disable CLKOUT
			return false;
		}
		reg = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_EEPROM_CLKOUT, readError); // read again to double check
		if(readError) {
			return false;
		}
		if(reg != 0x00) {
			return false;
		}
	}
	return true;
}

uint8_t RTC_RV3028C7::setTimeWithUTCCompileTime(bool resetIssuedByUPDI) {
	bool error = false;
	tmElements_t compileTimeStruct;
	uint32_t compileTimestamp = UNIX_TIMESTAMP + SET_UTC_COMP_TIME_OFFSET - SET_UTC_COMP_TIME_GMT2_OFFSET; // UTC time with small deviation due to compilation time offset
	breakTime(compileTimestamp, compileTimeStruct); // create hour, minute, ... from timestamp

	if(resetIssuedByUPDI) { // only set after UPDI programming (in case MCU resets but time still okay) -> power loss = needs reprogramming!
		error |= (!set(compileTimeStruct.Hour, compileTimeStruct.Minute, compileTimeStruct.Second, 0, compileTimeStruct.Day, compileTimeStruct.Month, compileTimeStruct.Year+1970)) << 0;
		error |= (!setTimestamp(compileTimestamp)); 
	}
	else {
		// do not set time
	}
	return !error;	
}

uint32_t RTC_RV3028C7::getTimestamp(bool &error) {
	return i2c.readRegisterPageInt32(RTC_RV3028C7_ADDRESS, REG_TIMESTAMP, error); // no bcd coding
}

bool RTC_RV3028C7::setTimestamp(uint32_t timestamp) {
	bool readError = false;
	bool writeError = false;
	// reset of prescaler
	uint8_t reg_control2 = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, readError);
	if(readError) {
		return false;
	}
	writeError = !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, HelperBits::setBit(reg_control2, REG_CONTROL2_RESET, true)); 		// 0x10: TIE = 0 (disable interrupts shortly)
	if(writeError) {
		return false;
	}
	return i2c.writeRegisterPageInt32(RTC_RV3028C7_ADDRESS, REG_TIMESTAMP, timestamp); // no bcd coding
}

uint8_t RTC_RV3028C7::getSeconds(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_SECONDS, error));
}

uint8_t RTC_RV3028C7::getMinutes(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_MINUTES, error));
}

uint8_t RTC_RV3028C7::getHours(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_HOURS, error));
}

uint8_t RTC_RV3028C7::getWeekday(bool &error) {
	return i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_WEEKDAY, error); // no bcd coding
}

uint8_t RTC_RV3028C7::getDate(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_DATE, error));
}

uint8_t RTC_RV3028C7::getMonth(bool &error) {
	return HelperBits::BCDToInt(i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_MONTH, error));
}

uint16_t RTC_RV3028C7::getYear(bool &error) {
	uint8_t yearWithout2000 = HelperBits::BCDToInt(i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_YEAR, error));
	uint16_t yearWith2000 = 2000 + yearWithout2000;
	return yearWith2000;
}

bool RTC_RV3028C7::set(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t weekday, uint8_t date, uint8_t month, uint16_t year) {
	const uint8_t LEN = 7;
	uint8_t vals[LEN];

	if((seconds > 59) || (minutes > 59) || (hours > 23) || (weekday > 6) || (date > 31) || (date == 0) || (month > 12) || (month == 0) || (year > 2099) || (year < 2000)) {
		return false;
	}
	
	vals[0] = HelperBits::intToBCD(seconds);
	vals[1] = HelperBits::intToBCD(minutes);
	vals[2] = HelperBits::intToBCD(hours);
	vals[3] = weekday;
	vals[4] = HelperBits::intToBCD(date);
	vals[5] = HelperBits::intToBCD(month);
	uint16_t yearWithout2000 = year - 2000;
	vals[6] = HelperBits::intToBCD((uint8_t) yearWithout2000);
	
	return i2c.writeRegisterBase(RTC_RV3028C7_ADDRESS, REG_SECONDS, vals, LEN);
}

bool RTC_RV3028C7::setTimeUpdateInterruptMinuteChange() {
	bool readError = false;
	bool writeError = false;

	// read relevant registers
	uint8_t reg_control2 = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, readError);
	if(readError) {
		return false;
	}
	uint8_t reg_status = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, readError);
	if(readError) {
		return false;
	}
	uint8_t reg_control1 = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL1, readError);
	if(readError) {
		return false;
	}
	
	// initialize UIE and UF to 0
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, HelperBits::setBit(reg_control2, REG_CONTROL2_UIE, 0)); 			// 0x10: UIE = 0 (disable interrupts shortly)
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, HelperBits::setBit(reg_status, REG_STATUS_UF, 0));					// 0x0E: UF = 0 (clear timer interrupt event)
	
	// choose timer source
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL1, HelperBits::setBit(reg_control1, REG_CONTROL1_USEL, 1));			// 0x0F: USEL = 1 (update on minute, auto reset time = 7.813ms, 0 would be on second)

	// enable interrupt pin
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, HelperBits::setBit(reg_control2, REG_CONTROL2_UIE, 1));
	
	return !writeError;
}

bool RTC_RV3028C7::setRegularInterrupt(uint16_t seconds) {
	uint8_t sec_lwr = ((seconds >> 0) & 0xff);
	uint8_t sec_upr = ((seconds >> 8) & 0xff);
	
	bool readError = false;
	bool writeError = false;
	if(seconds > 4096) { // 12 bit maximum size, max ~1 hour (can be higher if clock frequency TD is set lower)
		return false;
	}
	
	// read relevant registers
	uint8_t reg_control2 = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, readError);
	if(readError) {
		return false;
	}
	uint8_t reg_status = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, readError);
	if(readError) {
		return false;
	}
	uint8_t reg_control1 = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL1, readError);
	if(readError) {
		return false;
	}
	
	// write relevant registers
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, HelperBits::setBit(reg_control2, REG_CONTROL2_TIE, false)); 		// 0x10: TIE = 0 (disable interrupts shortly)
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, HelperBits::setBit(reg_status, REG_STATUS_TF, false));				// 0x0E: TF = 0 (clear timer interrupt event)
	
	reg_control1 = HelperBits::setBit(reg_control1, REG_CONTROL1_TRPT, true);									// 0x0F: TRPT = 1 (repeat)
	reg_control1 = HelperBits::setBit(reg_control1, REG_CONTROL1_TE, false);									// 0x0F: TE = 0 (disable timer)
	reg_control1 = HelperBits::setBit(reg_control1, REG_CONTROL1_TD0, false);									// 0x0F: TD = 10 (countdown clock frequency = 1Hz = 1s)
	reg_control1 = HelperBits::setBit( reg_control1, REG_CONTROL1_TD1, true);									// 0x0F: TD = 10 (countdown clock frequency = 1Hz = 1s)
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL1, reg_control1);
	
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_TIMER_VAL0, sec_lwr);													// 0x0A: Countdown value, lower byte (seconds)
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_TIMER_VAL1, sec_upr);													// 0x0B: Countdown value, upper byte (seconds)
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, HelperBits::setBit(reg_control2, REG_CONTROL2_TIE, true)); 		// 0x10: TIE = 1 (get INT pin interrupts)
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL1, HelperBits::setBit(reg_control1, REG_CONTROL1_TE, true)); 		// 0x0F: TE = 1 (enable countdown) -> NOW STARTING!!!
	
	return !writeError;
}

bool RTC_RV3028C7::disableRegularInterrupt() {
	bool readError = false;
	bool writeError = false;

	// read relevant registers
	uint8_t reg_control2 = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, readError);
	if(readError) {
		return false;
	}
	uint8_t reg_status = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, readError);
	if(readError) {
		return false;
	}
	uint8_t reg_control1 = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL1, readError);
	if(readError) {
		return false;
	}
	
	// write relevant registers
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, HelperBits::setBit(reg_control2, REG_CONTROL2_TIE, false)); 		// 0x10: TIE = 0 (disable interrupts)
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, HelperBits::setBit(reg_status, REG_STATUS_TF, false));				// 0x0E: TF = 0 (clear timer interrupt event)
	
	reg_control1 = HelperBits::setBit(reg_control1, REG_CONTROL1_TRPT, true);									// 0x0F: TRPT = 1 (repeat)
	reg_control1 = HelperBits::setBit(reg_control1, REG_CONTROL1_TE, false);									// 0x0F: TE = 0 (disable timer)
	reg_control1 = HelperBits::setBit(reg_control1, REG_CONTROL1_TD0, false);									// 0x0F: TD = 10 (countdown clock frequency = 1Hz = 1s)
	reg_control1 = HelperBits::setBit( reg_control1, REG_CONTROL1_TD1, true);									// 0x0F: TD = 10 (countdown clock frequency = 1Hz = 1s)
	writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL1, reg_control1);
	
	return !writeError;
}

bool RTC_RV3028C7::setDailyInterrupt(uint8_t hour, uint8_t minute) {
	return setCalendarInterrupt(INTERRUPT_DAILY, hour, minute);
}

bool RTC_RV3028C7::setHourlyInterrupt(uint8_t minute) {
	return setCalendarInterrupt(INTERRUPT_HOURLY, 0x0, minute);
}

bool RTC_RV3028C7::setCalendarInterrupt(uint8_t mode, uint8_t hour, uint8_t minute) {
	bool readError = false;
	bool writeError = false;
	if(minute > 59 || hour > 23) {
		return false;
	}
	
	// read relevant registers
	uint8_t reg_control2 = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, readError);
	if(readError) {
		return false;
	}
	uint8_t reg_status = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, readError);
	if(readError) {
		return false;
	}
	uint8_t reg_control1 = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL1, readError);
	if(readError) {
		return false;
	}
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
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, HelperBits::setBit(reg_control2, REG_CONTROL2_AIE, false)); 						// 0x10: AIE = 0
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, HelperBits::setBit(reg_status, REG_STATUS_AF, false));								// 0x0E: AF = 0
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL1, HelperBits::setBit(reg_control1, REG_CONTROL1_WADA, true));						// 0x0F: WADA = 1 (weekday alarm disabled)
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_ALARM_MINUTE, HelperBits::setBit(HelperBits::intToBCD(minute), REG_ALARM_MINUTE_AE_M, AE_M));	// 0x07: Minutes alarm register, AE_M = 0 (enabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_ALARM_HOUR, HelperBits::setBit(HelperBits::intToBCD(hour), REG_ALARM_HOUR_AE_H, AE_H));		// 0x08: Hours alarm register, AE_H = 0 (enabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_ALARM_WEEKDAY, HelperBits::setBit(0x0, REG_ALARM_WEEKDAY_AE_WD, AE_WD));					// 0x09: Weekday alarm register, AE_WD = 1 (disabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, HelperBits::setBit(reg_control2, REG_CONTROL2_AIE, true)); 						// 0x10: AIE = 1 (get INT pin interrupts)
	}
	else { // disable interrupts
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_CONTROL2, HelperBits::setBit(reg_control2, REG_CONTROL2_AIE, false)); 						// 0x10: AIE = 0
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, HelperBits::setBit(reg_status, REG_STATUS_AF, false));								// 0x0E: AF = 0
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_ALARM_MINUTE, HelperBits::setBit(0x0, REG_ALARM_MINUTE_AE_M, AE_M));	// 0x07: Minutes alarm register, AE_M = 0 (enabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_ALARM_HOUR, HelperBits::setBit(0x0, REG_ALARM_HOUR_AE_H, AE_H));		// 0x08: Hours alarm register, AE_H = 0 (enabled, when hours + minutes match, once per day)
		writeError |= !i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_ALARM_WEEKDAY, HelperBits::setBit(0x0, REG_ALARM_WEEKDAY_AE_WD, AE_WD));					// 0x09: Weekday alarm register, AE_WD = 1 (disabled, when hours + minutes match, once per day)
	}

	return !writeError;
}

bool RTC_RV3028C7::dailyOrHourlyInterruptHappened(bool &error) {
	return i2c.readRegisterBit(RTC_RV3028C7_ADDRESS, REG_STATUS, REG_STATUS_AF, error);
}

bool RTC_RV3028C7::regularInterruptHappened(bool &error) {
	return i2c.readRegisterBit(RTC_RV3028C7_ADDRESS, REG_STATUS, REG_STATUS_TF, error);
}

bool RTC_RV3028C7::disableHourlyDailyInterrupt() {
	return setCalendarInterrupt(NO_INTERRUPT, 0, 0);
}

bool RTC_RV3028C7::resetInterruptFlags() {
	bool readError = false;
	uint8_t reg_status = i2c.readRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, readError);
	if(readError) {
		return false;
	}
	reg_status = HelperBits::setBit(reg_status, REG_STATUS_AF, false);	// 0x0E: AF = 0 (clear daily or hourly interrupt -> mandatory to reset INT pin)
	reg_status = HelperBits::setBit(reg_status, REG_STATUS_TF, false);	// 0x0E: TF = 0 (clear regular interrupt)
	reg_status = HelperBits::setBit(reg_status, REG_STATUS_UF, false);	// 0x0E: UF = 0 (clear setTimeUpdateInterruptMinuteChange interrupt)
	return i2c.writeRegister(RTC_RV3028C7_ADDRESS, REG_STATUS, reg_status);
}
