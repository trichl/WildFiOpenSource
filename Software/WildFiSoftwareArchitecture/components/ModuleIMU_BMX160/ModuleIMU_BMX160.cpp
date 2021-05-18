#include "ModuleIMU_BMX160.h"

IMU_BMX160::IMU_BMX160() {

}

bool IMU_BMX160::initFIFO(uint8_t forWhat) {
	bool error = false;
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_FIFO_CONFIG_1_ADDR, forWhat); // only store ACC data, fifo tagging DISABLED, headerless format
	return !error;
}

bool IMU_BMX160::enableFIFOInterrupt(uint16_t watermark) {
	bool error = false;
	uint8_t temp;
	
	watermark = watermark / 4; // stored in 1 byte register, +1 = 4 bytes
	if(watermark > 255) {
		return false;
	}
	temp = watermark; // store in 1 byte variable
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_FIFO_CONFIG_0_ADDR, temp); // interrupt if FIFO watermark = 128 * 4Byte = 512 (half full)

	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_1_ADDR, error);
	temp |= 0b01000000; // enable fifo watermark interrupt
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_1_ADDR, temp);
	
	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_1_ADDR, error);
	temp |= 0b01000000; // map INT1 to watermark interrupt
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_1_ADDR, temp);
	
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_OUT_CTRL_ADDR, 0b00001010); // INT1 output enable, push-pull, active-high
	
	return !error;
}

uint8_t IMU_BMX160::countAccFIFOLogs(uint16_t len) {
	if(len % 6 != 0) { // check if can be evenly divided by 6 (bytes)
		return 0;
	}
	len = len / 6;
	return (uint8_t) len;
}

int16_t IMU_BMX160::getAccFIFOLog(uint8_t fifoData[], uint16_t len, uint16_t logNumber, uint8_t xyz) {
	uint16_t value;
	uint16_t pointer;
	pointer = xyz;
	pointer = (logNumber * 6) + (pointer * 2);
	if((pointer+1) > len) {
		return 0;
	}
	value = (int16_t) ((fifoData[pointer+1] << 8) | fifoData[pointer]);
	return value;
}

bool IMU_BMX160::readAccFIFO(uint8_t fifoData[], uint16_t len) {
	bool error = false;
	bool finished = false;
	uint16_t writeLen;
	uint16_t fifoPointer = 0;

	error |= !accSetMode(BMX160_ACCEL_NORMAL_MODE); // datasheet recommends NORMAL mode when reading FIFO!
	//error |= !readRegisterPageArray(BMX160_FIFO_DATA_ADDR, fifoData, len);
	
	i2c.beginTransmission(IMU_BMX160_ADDRESS);
	i2c.write(BMX160_FIFO_DATA_ADDR);
	i2c.endTransmission();
	
	while(!finished) { // repeat until all data is read, remember: if burst-reading less than frame size (here 6 byte), it will be repeated! -> so max. 30 bytes per I2C transmission in burst
		if(len <= BMX160_FIFO_BURST_READ_FOR_6_BYTE) {
			writeLen = len;
			finished = true;
		}
		else {
			writeLen = BMX160_FIFO_BURST_READ_FOR_6_BYTE;
			len -= writeLen;
		}
		i2c.requestFrom(IMU_BMX160_ADDRESS, writeLen);
		for(uint16_t i = 0; i < writeLen; i++) {
			fifoData[fifoPointer] = i2c.read();
			fifoPointer++;
		}
		//i2c.endTransmission();
	}
	Timing::delay(5); // NEW!
	error |= !accSetMode(BMX160_ACCEL_LOWPOWER_MODE); // back to power saving mode
	return !error;
}

bool IMU_BMX160::readGeneralFIFOInOneGoFast(bool accOn, bool magOn, bool gyroOn, uint8_t fifoData[], uint16_t currentFifoLen, bool waitUntilBackInLowPower) {
	bool error = false;
	if(currentFifoLen == 0) {
		return false;
	}

	if(magOn) {
		error |= !magSetMode(BMX160_MAGN_NORMAL_MODE); // datasheet recommends NORMAL mode when reading FIFO!
	}
	if(accOn) {
		error |= !accSetMode(BMX160_ACCEL_NORMAL_MODE); // datasheet recommends NORMAL mode when reading FIFO!
	}
	if(gyroOn) {
		// gyro does not have low power mode -> not needed
	}

	i2c.beginTransmission(IMU_BMX160_ADDRESS);
	i2c.write(BMX160_FIFO_DATA_ADDR);
	i2c.endTransmission();

	i2c.requestFrom(IMU_BMX160_ADDRESS, currentFifoLen);
	for(uint16_t i = 0; i < currentFifoLen; i++) {
		fifoData[i] = i2c.read();
	}

	if(waitUntilBackInLowPower) {
		if(magOn) {
			error |= !magSetMode(BMX160_MAGN_LOWPOWER_MODE); // back to power saving mode
		}
		if(accOn) {
			error |= !accSetMode(BMX160_ACCEL_LOWPOWER_MODE); // back to power saving mode
		}
	}
	else {
		if(magOn) {
			error |= !magSetMode(BMX160_MAGN_LOWPOWER_MODE); // should be faster to power down
		}
		if(accOn) {
			if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_ACCEL_LOWPOWER_MODE)) { error = true; }
		}
	}

	if(error) {
		return false;
	}
	return true;
}

bool IMU_BMX160::readAccFIFOInOneGoFast(uint8_t fifoData[], uint16_t currentFifoLen, bool waitUntilBackInLowPower) {
	bool error = false;
	if(currentFifoLen == 0) {
		return false;
	}
	//Timing::delay(5); // NEW!

	error |= !accSetMode(BMX160_ACCEL_NORMAL_MODE); // datasheet recommends NORMAL mode when reading FIFO!

	i2c.beginTransmission(IMU_BMX160_ADDRESS);
	i2c.write(BMX160_FIFO_DATA_ADDR);
	i2c.endTransmission();

	i2c.requestFrom(IMU_BMX160_ADDRESS, currentFifoLen);
	for(uint16_t i = 0; i < currentFifoLen; i++) {
		fifoData[i] = i2c.read();
	}
	//i2c.endTransmission();
	//Timing::delay(5); // NEW!
	if(waitUntilBackInLowPower) {
		error |= !accSetMode(BMX160_ACCEL_LOWPOWER_MODE); // back to power saving mode
	}
	else {
		if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_ACCEL_LOWPOWER_MODE)) { error = true; }
	}
	if(error) {
		return false;
	}
	return true;
}

uint16_t IMU_BMX160::readAccFIFOInOneGo(uint8_t fifoData[]) {
	bool error = false;
	uint16_t currentFifoLen;
	
	currentFifoLen = getFIFOLength();
	if(currentFifoLen == 0) {
		return 0;
	}
	Timing::delay(5); // NEW!

	error |= !accSetMode(BMX160_ACCEL_NORMAL_MODE); // datasheet recommends NORMAL mode when reading FIFO!

	i2c.beginTransmission(IMU_BMX160_ADDRESS);
	i2c.write(BMX160_FIFO_DATA_ADDR);
	i2c.endTransmission();

	i2c.requestFrom(IMU_BMX160_ADDRESS, currentFifoLen);
	for(uint16_t i = 0; i < currentFifoLen; i++) {
		fifoData[i] = i2c.read();
	}
	//i2c.endTransmission();
	Timing::delay(5); // NEW!
	error |= !accSetMode(BMX160_ACCEL_LOWPOWER_MODE); // back to power saving mode

	if(error) {
		return 0;
	}
	return currentFifoLen;
}

uint16_t IMU_BMX160::getFIFOLength() {
	uint8_t data[2] = {0};
	uint16_t result = 0;
	if(!i2c.readRegisterPageArray(IMU_BMX160_ADDRESS, BMX160_FIFO_LENGTH_ADDR, data, 2)) {
		return 0;
	}
	result = (uint16_t) ((data[1] << 8) | data[0]);
	return result;
}

bool IMU_BMX160::resetFIFO() {
	return i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, 0xB0);
}

bool IMU_BMX160::accSetMode(uint8_t mode) {
	bool error = false;
	uint8_t timeout = 0;
	uint8_t regMode = 0;
	if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, mode)) {
		return false;
	}
	while(true) {
		Timing::delay(3);
		regMode = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_PMU_STATUS_ADDR, error); // get current acc status
		if(error) { return false; }
		regMode = (regMode >> 4) & 0b11; // bit 4 + 5
		if((mode == BMX160_ACCEL_NORMAL_MODE) && (regMode == 0b01)) {
			break;
		}
		if((mode == BMX160_ACCEL_LOWPOWER_MODE) && (regMode == 0b10)) {
			break;
		}
		if((mode == BMX160_ACCEL_SUSPEND_MODE) && (regMode == 0b00)) {
			break;
		}
		timeout++;
		if(timeout > 200) { // 200 * 3 = 600ms
			return false;
		}
	}
	return true;
}

bool IMU_BMX160::gyroSetMode(uint8_t mode) {
	bool error = false;
	uint8_t timeout = 0;
	uint8_t regMode = 0;
	if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, mode)) {
		return false;
	}
	while(true) {
		Timing::delay(3);
		regMode = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_PMU_STATUS_ADDR, error); // get current acc status
		if(error) { return false; }
		regMode = (regMode >> 2) & 0b11; // bit 2 + 3
		if((mode == BMX160_GYRO_NORMAL_MODE) && (regMode == 0b01)) {
			break;
		}
		if((mode == BMX160_GYRO_FASTSTARTUP_MODE) && (regMode == 0b11)) {
			break;
		}
		if((mode == BMX160_GYRO_SUSPEND_MODE) && (regMode == 0b00)) {
			break;
		}
		timeout++;
		if(timeout > 200) { // 200 * 3 = 600ms
			return false;
		}
	}
	return true;
}

bool IMU_BMX160::magSetMode(uint8_t mode) {
	bool error = false;
	uint8_t timeout = 0;
	uint8_t regMode = 0;
	if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, mode)) {
		return false;
	}
	while(true) {
		Timing::delay(3);
		regMode = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_PMU_STATUS_ADDR, error); // get current mag status
		if(error) { return false; }
		regMode = regMode & 0b11; // bit 0 + 1
		if((mode == BMX160_MAGN_NORMAL_MODE) && (regMode == 0b01)) {
			break;
		}
		if((mode == BMX160_MAGN_LOWPOWER_MODE) && (regMode == 0b10)) {
			break;
		}
		if((mode == BMX160_MAGN_SUSPEND_MODE) && (regMode == 0b00)) {
			break;
		}
		timeout++;
		if(timeout > 200) { // 200 * 3 = 600ms
			return false;
		}
	}
	return true;
}

/*bool IMU_BMX160::startGyroFOC() { // very OLD, mixes Gyro and Mag -> FOC only for ACC and Gyro possible
	bool error = false;
	uint8_t reg = 0b01000000; // config: FOC for gyro enabled, FOC for ACC on all axes disabled
	error |= !writeRegister(BMX160_FOC_CONF_ADDR, reg); // config for FOC
	error |= !magSetMode(BMX160_MAGN_NORMAL_MODE); // datasheet recommends NORMAL mode when executing FOC
	error |= !writeRegister(BMX160_COMMAND_REG_ADDR, BMX160_START_FOC_CMD); // start FOC
	while(true) { // should take max. 250ms
		Timing::delay(20);
		reg = readRegister(BMX160_STATUS_ADDR, error); // get FOC status
		if(reg & 0x8) { // bit 3 = foc_rdy
			break;
		}
	}
	error |= !magSetMode(BMX160_MAGN_LOWPOWER_MODE); // LOW POWER mode again
	return !error;	
}*/

/*bool IMU_BMX160::accFOCAlreadyDoneAndStoredInNVM() {
	uint8_t reg = 0;
	bool error = false;

	reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_OFFSET_6, error); // read current state of register
	if(error) {
		return true; // don't trigger NVM-writing again in case of a communication issue
	}
	else {
		if(reg & (0b01000000)) {  // bit in "use FOC offset compensation" (bit 6) is set
			return true;
		}
	}
	return false;
}*/

bool IMU_BMX160::accGyroFOCAlreadyDoneAndStoredInNVM() {
	uint8_t reg = 0;
	bool error = false;

	reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_OFFSET_6, error); // read current state of register
	if(error) {
		return true; // don't trigger NVM-writing again in case of a communication issue
	}
	else {
		if((reg & (0b11000000)) == 0b11000000) {  // bit in "use FOC offset compensation" (bit 6 + 7) is set
			return true;
		}
	}
	return false;
}

bool IMU_BMX160::startAccGyroFOCAndStoreInNVM(bool forceMode) {
	bool error = false;
	uint16_t timeout = 0;
	uint8_t reg = 0b01111101; // config: FOC for gyro ENABLED, FOC for ACC x axis 0g, y axis 0g, z axis +1g (device on table flat, looking up?)

	if((!forceMode) && accGyroFOCAlreadyDoneAndStoredInNVM()) {
		return false; // already done! don't do it again because NVM can only be written 14 times or less
	}

	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_FOC_CONF_ADDR, reg); // config for FOC
	error |= !accSetMode(BMX160_ACCEL_NORMAL_MODE); // datasheet recommends NORMAL mode when executing FOC
	error |= !gyroSetMode(BMX160_GYRO_NORMAL_MODE); // datasheet recommends NORMAL mode when executing FOC
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_START_FOC_CMD); // start FOC
	if(error) { // return here already
		return false;
	}
	while(true) { // should take max. 250ms
		Timing::delay(20);
		reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_STATUS_ADDR, error); // get FOC status
		if(reg & 0x8) { // bit 3 = foc_rdy
			break;
		}
		timeout++;
		if(timeout > 200) { // 4 seconds
			return false;
		}
	}
	if(error) { // return here already to not write into NVM too many times
		return false;
	}
	
	reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_OFFSET_6, error); // read current state of register
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_OFFSET_6, reg | (0b11000000)); // IMPORTANT: activate FOC offset compensation (bit 6 = acc, bit 7 = gyro)

	if(error) { // return here already to not write into NVM too many times
		return false;
	}

	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_CONF, 0b10); // write 1 to nvm_prog_en

	if(error) { // return here already to not write into NVM too many times
		return false;
	}

	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_START_NVM_WRITE_CMD); // write data into NVM

	timeout = 0;
	while(true) { // don't know how long it takes
		Timing::delay(20);
		reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_STATUS_ADDR, error); // get FOC status
		if(reg & 0b10000) { // bit 4 = nvm_rdy
			break;
		}
		timeout++;
		if(timeout > 200) { // 4 seconds
			return false;
		}
	}
	error |= !softReset(false); // instead of setting acc and gyro to low power -> perform a soft reset
	return !error;
}

/*bool IMU_BMX160::startAccFOCAndStoreInNVM() {
	bool error = false;
	uint16_t timeout = 0;
	uint8_t reg = 0b00111101; // config: FOC for gyro disabled, FOC for ACC x axis 0g, y axis 0g, z axis +1g (device on table flat, looking up?)

	if(accFOCAlreadyDoneAndStoredInNVM()) {
		return false; // already done! don't do it again because NVM can only be written 14 times or less
	}

	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_FOC_CONF_ADDR, reg); // config for FOC
	error |= !accSetMode(BMX160_ACCEL_NORMAL_MODE); // datasheet recommends NORMAL mode when executing FOC
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_START_FOC_CMD); // start FOC
	if(error) { // return here already
		return false;
	}
	while(true) { // should take max. 250ms
		Timing::delay(20);
		reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_STATUS_ADDR, error); // get FOC status
		if(reg & 0x8) { // bit 3 = foc_rdy
			break;
		}
		timeout++;
		if(timeout > 200) { // 4 seconds
			return false;
		}
	}
	if(error) { // return here already to not write into NVM too many times
		return false;
	}
	
	reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_OFFSET_6, error); // read current state of register
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_OFFSET_6, reg | (0b01000000)); // IMPORTANT: activate FOC offset compensation (bit 6)

	if(error) { // return here already to not write into NVM too many times
		return false;
	}

	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_CONF, 0b10); // write 1 to nvm_prog_en

	if(error) { // return here already to not write into NVM too many times
		return false;
	}

	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_START_NVM_WRITE_CMD); // write data into NVM

	timeout = 0;
	while(true) { // don't know how long it takes
		Timing::delay(20);
		reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_STATUS_ADDR, error); // get FOC status
		if(reg & 0b10000) { // bit 4 = nvm_rdy
			break;
		}
		timeout++;
		if(timeout > 200) { // 4 seconds
			return false;
		}
	}

	error |= !accSetMode(BMX160_ACCEL_LOWPOWER_MODE); // LOW POWER mode again
	return !error;
}*/

bool IMU_BMX160::enableAccNoMotionInterruptXYZ(uint8_t duration, uint8_t threshold) {
	// duration:
	// 00xxxx = (xxxx+1) * 1.28s (range: 1.28 - 20.48s)
	// 01xxxx = (xxxx+5) * 5.12s (range: 25.6 - 102.4s)
	// 1xxxxx = (xxxxx+11) * 10.24s (range: 112.64 - 430.08s)
	
	bool error = false;
	uint8_t temp;
	if(duration > 63) { // 6 bit
		return false;
	}
	
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MOTION_0_ADDR, duration); // set duration (only last 2 bits, others are overwritten with 0)
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MOTION_1_ADDR, threshold); // set treshold, depends on range, @2G: treshold*3.91mg

	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_MOTION_3_ADDR, error);
	temp |= 0b00000001; // enable no motion interrupt and not slo motion interrupt
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MOTION_3_ADDR, temp);
	
	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_MOTION_0_ADDR, error);
	temp |= (duration << 2); // set duration for no motion interrupt being fired
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MOTION_0_ADDR, temp);
	
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MOTION_2_ADDR, threshold); // set treshold, depends on range, @2G: treshold*3.91mg
	
	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_2_ADDR, error);
	temp |= 0b00000111; // enable nomotion interrupt on x, y, z
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_2_ADDR, temp);
	
	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_0_ADDR, error);
	temp |= 0b00001000; // bit 3: map INT1 to nomotion interrupt
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_0_ADDR, temp);
	
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_OUT_CTRL_ADDR, 0b00001010); // INT1 output enable, push-pull, active-high
	
	return !error;
}

bool IMU_BMX160::enableAccOrientationInterrupt(uint8_t orientHysteresis, uint8_t orientBlocking) {
	// TODO: int_orient_theta not configurable, don't know if important
	bool error = false;
	uint8_t temp;
	if((orientHysteresis > 15) // 4 bit
		|| (orientBlocking > 3)) { // 2 bit, 0 = no blocking of interrupt, 1 = theta blocking or acc > 1.5mg on any axis, ...
		return false;
	}
	temp = ((orientHysteresis << 4) | (orientBlocking << 2) | 0b11); // 0b11 = symmetrical mode
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_ORIENT_0_ADDR, temp);
	
	// keep second config register with default values (int_orient_theta = 8 = certain blocking angle)
	//error |= !writeRegister(BMX160_INT_ORIENT_1_ADDR, whatever);
	
	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_0_ADDR, error);
	temp |= 0b01000000; // bit 6: enable orientation interrupt
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_0_ADDR, temp);
	
	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_0_ADDR, error);
	temp |= 0b01000000; // bit 6: map INT1 to orientation interrupt
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_0_ADDR, temp);
	
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_OUT_CTRL_ADDR, 0b00001010); // INT1 output enable, push-pull, active-high
	
	return !error;	
}

uint8_t IMU_BMX160::getOrientation() {
	// only if orientation interrupt is enabled
	bool error;
	uint8_t reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_STATUS_ADDR+3, error);
	return ((reg & 0x30) >> 4); // bit 4+5
}


bool IMU_BMX160::enableAccFlatDetectionInterrupt(uint8_t flatTheta, uint8_t flatHoldTime, uint8_t flatHysteresis) {
	bool error = false;
	uint8_t temp;
	if((flatTheta > 63) // 6 bit
		|| (flatHoldTime > 3) // 2 bit, 0 = 0ms, 1 = 640ms, 2 = 2x640ms, ...
		|| (flatHysteresis > 7)) { // 3 bit
		return false;
	}
	temp = ((flatHoldTime << 4) | flatHysteresis);

	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_FLAT_0_ADDR, flatTheta);
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_FLAT_1_ADDR, temp);
	
	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_0_ADDR, error);
	temp |= 0b10000000; // bit 7: enable flat interrupt
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_0_ADDR, temp);
	
	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_0_ADDR, error);
	temp |= 0b10000000; // bit 7: map INT1 to flat interrupt
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_0_ADDR, temp);
	
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_OUT_CTRL_ADDR, 0b00001010); // INT1 output enable, push-pull, active-high
	
	return !error;	
}

uint16_t IMU_BMX160::getInterruptReasons() {
	uint8_t data[2] = {0};
	uint16_t result = 0;
	if(!i2c.readRegisterPageArray(IMU_BMX160_ADDRESS, BMX160_INT_STATUS_ADDR, data, 2)) {
		return 0;
	}
	result = (uint16_t) ((data[1] << 8) | data[0]);
	return result;
}

bool IMU_BMX160::containsInterrupt(uint16_t reasons, uint16_t interruptMask) {
	return (reasons & interruptMask);
}

bool IMU_BMX160::isFlat() {
	// only if flat interrupt is enabled
	bool error;
	uint8_t reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_STATUS_ADDR+3, error);
	return (reg & 0x80); // bit 7
}

bool IMU_BMX160::isOnDownside() {
	// only if flat interrupt or orientation interrupt is enabled
	bool error;
	uint8_t reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_STATUS_ADDR+3, error);
	return (reg & 0x40); // bit 6
}

bool IMU_BMX160::enableAccAnyMotionInterruptXYZ(uint8_t duration, uint8_t threshold) {
	bool error = false;
	uint8_t temp;
	if(duration > 0b11) {
		return false;
	}
	
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MOTION_0_ADDR, duration); // set duration (only last 2 bits, others are overwritten with 0)
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MOTION_1_ADDR, threshold); // set threshold, depends on range, @2G: threshold*3.91mg

	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_0_ADDR, error);
	temp |= 0b00000111; // enable anymotion interrupt on x, y, z
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_ENABLE_0_ADDR, temp);
	
	temp = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_0_ADDR, error);
	temp |= 0b00000100; // bit 2: map INT1 to anymotion interrupt
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_MAP_0_ADDR, temp);
	
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_OUT_CTRL_ADDR, 0b00001010); // INT1 output enable, push-pull, active-high
	
	return !error;
}

uint8_t IMU_BMX160::anyMotionInterruptAxis() {
	// TODO (maybe): could also get slope direction (negativ or positive) = int_anym_sign
	bool error;
	uint8_t reg;
	reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_INT_STATUS_ADDR+2, error);
	if(error) {
		return 0x0;
	}
	reg = reg & BMX160_INT_STATUS_ANYMOT_AXIS_MSK;
	return reg;
}

bool IMU_BMX160::enableAccStepCounter(uint8_t minThreshold, uint8_t minStepTime, uint8_t minBuffer) {
	// minThreshold = peak needs to exceed that, minStepTime = minimum delay between two peaks
	bool error = false;
	uint8_t conf0;
	uint8_t conf1;
	
	conf0 = minThreshold | minStepTime;
	conf1 = minBuffer | 0x8; // enable step counter (bit 3)

	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_STEP_CONF_0_ADDR, conf0);
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_STEP_CONF_1_ADDR, conf1);

	return !error;
}

bool IMU_BMX160::enableAccStepCounterNormalMode() {
	// CONF0 = 0x15, CONF1 = 0x03
	return enableAccStepCounter(BMX160_STEP_CONF_0_MIN_THRESH_2, BMX160_STEP_CONF_0_STEPTIME_MIN_5, BMX160_STEP_CONF_1_MIN_BUF_3);
}

bool IMU_BMX160::enableAccStepCounterSensitiveMode() {
	// CONF0 = 0x2D, CONF1 = 0x00
	// BMX160_STEP_CONF_0_MIN_THRESH_5 suddenly has 3 bits here, maybe it means 1 not 5, would make more sense!
	return enableAccStepCounter(BMX160_STEP_CONF_0_MIN_THRESH_5, BMX160_STEP_CONF_0_STEPTIME_MIN_5, BMX160_STEP_CONF_1_MIN_BUF_0);
}

bool IMU_BMX160::enableAccStepCounterRobustMode() {
	// CONF0 = 0x1D, CONF1 = 0x07
	return enableAccStepCounter(BMX160_STEP_CONF_0_MIN_THRESH_3, BMX160_STEP_CONF_0_STEPTIME_MIN_5, BMX160_STEP_CONF_1_MIN_BUF_7);
}

uint16_t IMU_BMX160::getStepCounter() {
	uint8_t data[2] = {0};
	uint16_t steps;
	bool error = false;
	
	// datasheet recommends NORMAL mode for ACC when reading out step counter (for most recent value)
	error = !accSetMode(BMX160_ACCEL_NORMAL_MODE);
	if(error) {
		return 0;
	}
    if(!i2c.readRegisterPageArray(IMU_BMX160_ADDRESS, BMX160_STEP_CNT_0_ADDR, data, 2)) {
		return 0;
	}
	steps = (uint16_t) ((data[1] << 8) | data[0]);
	// LOW POWER mode again
	error = !accSetMode(BMX160_ACCEL_LOWPOWER_MODE);
	if(error) {
		return 0;
	}
	
	return steps;
}

bool IMU_BMX160::resetStepCounter() {
	bool error = false;
	error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, 0xB2); // send reset CMD
	return !error;
}

bool IMU_BMX160::setupMag(uint8_t odr, uint8_t accuracy) {
	/* *     settings->xy_rep     |  nXY(XY Repetitions)
	* -------------------------|-----------------------
	*   0x00                   |   1
	*   0x01                   |   3
	*   0x02                   |   5
	*    .                     |   .
	*    .                     |   .
	*   0xFF                   |   511

	*     settings->z_rep      |  nZ(Z Repetitions)
	* -------------------------|-----------------------
	*   0x00                   |   1
	*   0x01                   |   2
	*   0x02                   |   3
	*    .                     |   .
	*    .                     |   .
	*   0xFF                   |   256*/
	bool error = false;
	// 0x7E: put MAG_IF into normal mode
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_MAGN_NORMAL_MODE);
	Timing::delay(2); // datasheet recommends 650us
	// 0x4C: MAG_IF enter setup mode, mag_manual_en = 1 (start register access), mag_offset = 0b0000 (max offset)
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_0_ADDR, 0x80);
	Timing::delay(20); // really necessary?!, DFRobot library = 50ms delay
    // indirect write to MAG register 0x4B: coming out of suspend mode
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, 0x01);
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_2_ADDR, 0x4B);
	if(!waitOnIndirectMagRead()) { return false; } // IMPORTANT: BUG before, if this is not here: XY repetition is never executed (because MAG not out of suspend mode!)
	Timing::delay(10); // just to be sure!!!
    // indirect write to MAG register 0x51: REPXY = low power preset (number of repetitions on X and Y)
    if(accuracy == BMX160_MAG_ACCURACY_LOW_POWER) { error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, BMX160_MAG_ACCURACY_LOW_POWER_XY); }  // 0x01 = 3 repetitions
	else if(accuracy == BMX160_MAG_ACCURACY_REGULAR) { error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, BMX160_MAG_ACCURACY_REGULAR_XY); }
	else if(accuracy == BMX160_MAG_ACCURACY_ENHANCED) { error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, BMX160_MAG_ACCURACY_ENHANCED_XY); }
	else { error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, BMX160_MAG_ACCURACY_HIGH_XY); }
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_2_ADDR, 0x51);
	if(!waitOnIndirectMagRead()) { return false; } // wait until write was confirmed
    // indirect write to MAG register 0x52: REPZ = low power preset (number of repetitions on Z)
    if(accuracy == BMX160_MAG_ACCURACY_LOW_POWER) { error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, BMX160_MAG_ACCURACY_LOW_POWER_Z); } // 0x02 = 3 repetitions
	else if(accuracy == BMX160_MAG_ACCURACY_REGULAR) { error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, BMX160_MAG_ACCURACY_REGULAR_Z); }
	else if(accuracy == BMX160_MAG_ACCURACY_ENHANCED) { error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, BMX160_MAG_ACCURACY_ENHANCED_Z); }
	else { error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, BMX160_MAG_ACCURACY_HIGH_Z); }
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_2_ADDR, 0x52);
	if(!waitOnIndirectMagRead()) { return false; } // wait until write was confirmed
    // prepare interface for data mode
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, 0x02);
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_2_ADDR, 0x4C);
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_1_ADDR, 0x42);
	if(!waitOnIndirectMagRead()) { return false; } // wait until write was confirmed
	// 0x44: mag_odr = 0b0110 = 25Hz
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_CONFIG_ADDR, odr);
	// mag_manual_en = 0 (end register access), mag_offset = 0b0000 (max offset), mag_rd_burst = 0 (1 Byte for burst read)
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_0_ADDR, 0x0);
    Timing::delay(3); // really necessary?!, DFRobot library = 50ms
	
	return !error;
}

bool IMU_BMX160::stopMag() {
	bool error = false;
	// put MAG_IF into normal mode
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_MAGN_NORMAL_MODE);
	Timing::delay(2); // datasheet recommends 350us
	// mag_manual_en = 1 (start register access), mag_offset = 0b0000 (max offset)
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_0_ADDR, 0x80);
    // indirect write to MAG register 0x4B: suspend mode
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, 0x00);
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_2_ADDR, 0x4B);
	// put MAG_IF into suspend mode
    error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_MAGN_SUSPEND_MODE);
	return !error;
}

bool IMU_BMX160::stop() {
	return start(NULL, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC);
}

bool IMU_BMX160::softReset(bool magWasRunning) {
	if(magWasRunning) {
		if(!stopMag()) {
			return false;
		}
	}
	if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_COMMAND_REG_ADDR, BMX160_SOFT_RESET_CMD)) {
		return false;
	}
	Timing::delay(BMX160_SOFT_RESET_DELAY_MS); // from DFRobot library
	return true;
}

bool IMU_BMX160::start(acc_config_t *accConf, mag_config_t *magConf, gyro_config_t *gyroConf, uint8_t interruptLatch) {
	bool error = false;
	uint8_t mode;

	// MAG config, configure first because Bosch recommends it
	if(magConf != NULL) {
		// low power mode
		mode = BMX160_MAGN_LOWPOWER_MODE;
		error |= !setupMag(magConf->frequency, magConf->accuracy);
	}
	else {
		// TODO: datasheet recommends procedure to put MAG into suspend, this only puts interface into suspend -> perform a soft reset maybe when stopping?
		mode = BMX160_MAGN_SUSPEND_MODE;
	}
	error |= !magSetMode(mode); // set MAG mode, interface is always running in low power mode (while the mag itself can run in normal or low power mode)
	
	// ACC config
	if(accConf != NULL) {
		// low power mode
		mode = BMX160_ACCEL_LOWPOWER_MODE;
		// config ACC for undersampling enabled (MANDATORY for low power mode), 25Hz + 8 AVG cycles = 32uA power consumption (2mg RMS noise)
		uint8_t accConfigRegister = BMX160_ACCEL_UNDERSAMPLING_MASK
			| (accConf->averaging << 4) // would recommend BMX160_ACCEL_BW_RES_AVG8
			| accConf->frequency;
		error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_ACCEL_CONFIG_ADDR, accConfigRegister);
		if(accConf->range != BMX160_ACCEL_RANGE_2G) { // normally using default ACC range of 2G maximum (BMX160_ACCEL_RANGE_ADDR = 0b11) = maximum 0 - 70km/h in 1 second, should be okay
			error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_ACCEL_RANGE_ADDR, accConf->range);
		}
	}
	else {
		mode = BMX160_ACCEL_SUSPEND_MODE;
	}
	error |= !accSetMode(mode); // set ACC mode (own function because used frequently)
	
	// GYRO CONFIG
	if(gyroConf != NULL) {
		mode = BMX160_GYRO_NORMAL_MODE;
		// config ACC for undersampling enabled (MANDATORY for low power mode), 25Hz + 8 AVG cycles = 32uA power consumption (2mg RMS noise)
		uint8_t gyroConfigRegister = (gyroConf->bw << 4) // would recommend BMX160_ACCEL_BW_RES_AVG8
			| gyroConf->frequency;
		error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_GYRO_CONFIG_ADDR, gyroConfigRegister);
		error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_GYRO_RANGE_ADDR, gyroConf->range);
	}
	else {
		mode = BMX160_GYRO_SUSPEND_MODE;
	}
	error |= !gyroSetMode(mode); // set ACC mode (own function because used frequently)
	Timing::delay(10);
	
	// LATCH for interrupts (CHANGE triggers interrupt two times)
	if((magConf != NULL) || (gyroConf != NULL) || (magConf != NULL)) { // only change if one of the sensors is actually turned on
		error |= !i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_INT_LATCH_ADDR, interruptLatch);
	}
	
	return !error;
}

/*int16_t IMU_BMX160::toMS2x1000(int16_t rawData, uint16_t range) {
	// warning: calculation depends on ACC range (here 2G = maximum)
	int32_t temp = rawData; // to 32 bit (e.g. -16500 ~= -9.81m/s^2)
	temp *= range; // = -1007077500 (MAX = 2G = -32768 * 6103 = -199983104 -> fits into 32 bit signed)
	temp /= 10000; // MAX = -19998 -> fits into 16 bit signed
	return ((int16_t) temp); // cast to 16 bit
}*/

/*int16_t IMU_BMX160::toMicroTeslaXY(int16_t rawData) {
	// NOT ACCURATE ENOUGH: x0.3 equals uT, according to DFRobot library
	// NOT ACCURATE ENOUGH: magnetic field resolution of ~0.3uT according to BMM150 datasheet
	// X and Y axis magnetic field range = +/-1300μT -> 13 bits = -4096 to 4095 -> -4096 * X = -1300uT -> 0.31738
	// NOT CORRECT according to Bosch, no linear formular for raw data to uT
	int64_t temp = rawData;
	temp *= 30000; // 31738
	temp /= 100000; // 100000
	return ((int16_t) temp);
}

int16_t IMU_BMX160::toMicroTeslaZ(int16_t rawData) {
	// NOT WORKING: correct calculation: Z axis magnetic field range = +/-2500μT -> 15 bits = -16384 to 16383 -> -16384 * X = -2500uT -> 0.15258
	//return rawData;
	int64_t temp = rawData;
	temp *= 30000; // 15258
	temp /= 100000; // 100000
	return ((int16_t) temp);
}*/

int16_t IMU_BMX160::toCelsiusx100(uint16_t tempRaw) {
	float temp = tempRaw;
	if(tempRaw > 0x8000) {
		// f(0xFFFF) = 23 - (0.5^9)
		// f(0x8001) = -41 + (0.5^9)
		float m = (-41. + pow(0.5,9)) - (23. - pow(0.5,9));
		m = m / (0x8001 - 0xFFFF);
		float b = (-41. + pow(0.5,9)) - (m*0x8001);
		temp = (m * temp) + b;
		temp *= 100;
	}
	else {
		temp *= 2.;
		temp /= 10.;
		temp += 2300.;
	}
    return (int16_t) temp;
}

bool IMU_BMX160::getTemperatureRaw(uint16_t &tempRaw) {
	uint8_t data[2] = {0};
	if(!i2c.readRegisterPageArray(IMU_BMX160_ADDRESS, BMX160_TEMP_0_ADDR, data, 2)) {
		return false;
	}
	tempRaw = (uint16_t) ((data[1] << 8) | data[0]);
	return true;
}

bool IMU_BMX160::accDataReady() {
	uint8_t reg = 0;
	bool error = false;
	reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_STATUS_ADDR, error); // get current acc status
	if(error) { return false; }
	if(reg & (1 << BMX160_STATUS_DATARDY_ACC_BITPOS)) { return true; }
	return false;
}

bool IMU_BMX160::magDataReady() {
	uint8_t reg = 0;
	bool error = false;
	reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_STATUS_ADDR, error); // get current acc status
	if(error) { return false; }
	if(reg & (1 << BMX160_STATUS_DATARDY_MAG_BITPOS)) { return true; }
	return false;
}
bool IMU_BMX160::gyroDataReady() {
	uint8_t reg = 0;
	bool error = false;
	reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_STATUS_ADDR, error); // get current acc status
	if(error) { return false; }
	if(reg & (1 << BMX160_STATUS_DATARDY_GYRO_BITPOS)) { return true; }
	return false;
}

bool IMU_BMX160::getData(struct BMX160Data *accel, struct BMX160Data *magn, struct BMX160Data *gyro, uint16_t *hall) {
    uint8_t data[23] = { 0 };
	int16_t msbData = 0;

    if(!i2c.readRegisterPageArray(IMU_BMX160_ADDRESS, BMX160_MAG_DATA_ADDR, data, 23)) { return false; }
	if(magn != NULL) {
		// OLD implementation
    	/*magn->x = (int16_t) ((data[1] << 8) | data[0]);
		magn->y = (int16_t) ((data[3] << 8) | data[2]);
		magn->z = (int16_t) ((data[5] << 8) | data[4]);*/

        data[0] = BMM150_GET_BITS(data[0], BMM150_DATA_X); // Mag X axis data
        msbData = ((int16_t)((int8_t)data[1])) * 32; // Shift the MSB data to left by 5 bits, Multiply by 32 to get the shift left by 5 value
        magn->x = (int16_t)(msbData | data[0]); // Raw mag X axis data

        data[2] = BMM150_GET_BITS(data[2], BMM150_DATA_Y); // Mag Y axis data
        msbData = ((int16_t)((int8_t)data[3])) * 32; // Shift the MSB data to left by 5 bits, Multiply by 32 to get the shift left by 5 value
        magn->y = (int16_t)(msbData | data[2]); // Raw mag Y axis data

        data[4] = BMM150_GET_BITS(data[4], BMM150_DATA_Z); // Mag Z axis data
        msbData = ((int16_t)((int8_t)data[5])) * 128; // Shift the MSB data to left by 7 bits, Multiply by 128 to get the shift left by 7 value
       	magn->z = (int16_t)(msbData | data[4]); // Raw mag Z axis data
	}
	
	//data[6] = (data[6] & 0xFC) >> 2; // OLD implementation, hall sensor data is not 14 bit long, take here only 6 bits
	if(hall != NULL) { // this seems to be used for temperature compensation!
		//*hall = (uint16_t)(((uint16_t)data[7] << 6) | data[6]);
        data[6] = BMM150_GET_BITS(data[6], BMM150_DATA_RHALL);
        *hall = (uint16_t)(((uint16_t)data[7] << 6) | data[6]);
	}

	if(gyro != NULL) {
		gyro->x = (int16_t) ((data[9] << 8) | data[8]);
		gyro->y = (int16_t) ((data[11] << 8) | data[10]);
		gyro->z = (int16_t) ((data[13] << 8) | data[12]);
	}

	if(accel != NULL) {
		accel->x = (int16_t) ((data[15] << 8) | data[14]);
		accel->y = (int16_t) ((data[17] << 8) | data[16]);
		accel->z = (int16_t) ((data[19] << 8) | data[18]);
	}

	return true;	
}

float IMU_BMX160::accRawToG(int16_t input, uint8_t range) {
	float multiplicator = 0.0;
	if(range == BMX160_ACCEL_RANGE_2G) { multiplicator = 0.00006103515; }
	else if(range == BMX160_ACCEL_RANGE_4G) { multiplicator = 0.00012207031; }
	else if(range == BMX160_ACCEL_RANGE_8G) { multiplicator = 0.00024414062; }
	else if(range == BMX160_ACCEL_RANGE_16G) { multiplicator = 0.00048828125; }
	return (input * multiplicator);
}

float IMU_BMX160::gyroRawToDegreePerSecond(int16_t input, uint8_t range) {
	float multiplicator = 0.0;
	if(range == BMX160_GYRO_RANGE_2000_DPS) { multiplicator = 0.061; }
	else if(range == BMX160_GYRO_RANGE_1000_DPS) { multiplicator = 0.0305; }
	else if(range == BMX160_GYRO_RANGE_500_DPS) { multiplicator = 0.0153; }
	else if(range == BMX160_GYRO_RANGE_250_DPS) { multiplicator = 0.0076; }
	else if(range == BMX160_GYRO_RANGE_125_DPS) { multiplicator = 0.0038; }
	return (input * multiplicator);
}

bool IMU_BMX160::printFifoData(uint8_t *fifoData, uint16_t fifoLen, bmx160_fifo_dataset_len_t datasetLen, uint8_t accelRange, uint8_t gyroRange) {
	uint16_t iterator = 0;
	int16_t accX = 0, accY = 0, accZ = 0, magXComp = 0, magYComp = 0, magZComp = 0, gyroX = 0, gyroY = 0, gyroZ = 0;
	float accXConv = 0.0; float accYConv = 0.0; float accZConv = 0.0;
	float gyroXConv = 0.0; float gyroYConv = 0.0; float gyroZConv = 0.0;
	uint16_t hall = 0;
	if(fifoLen < datasetLen) { return false; }
    if(fifoLen % datasetLen != 0) { return false; }
	while(true) {
		if(iterator + datasetLen > fifoLen) { break; }
		if(datasetLen == BMX160_FIFO_DATASET_LEN_ACC_AND_MAG) {
			magXComp = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			magYComp = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			magZComp = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			hall = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			accX = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			accY = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			accZ = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
		}
		else if(datasetLen == BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO) {
			magXComp = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			magYComp = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			magZComp = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			hall = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			gyroX = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			gyroY = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			gyroZ = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			accX = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			accY = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
			accZ = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
		}
		else { // unknown format
			iterator += datasetLen;
		}
		accXConv = accRawToG(accX, accelRange);
		accYConv = accRawToG(accY, accelRange);
		accZConv = accRawToG(accZ, accelRange);
		gyroXConv = gyroRawToDegreePerSecond(gyroX, gyroRange);
		gyroYConv = gyroRawToDegreePerSecond(gyroY, gyroRange);
		gyroZConv = gyroRawToDegreePerSecond(gyroZ, gyroRange);
		printf("acc %f, %f, %f, mag %d %d %d, hall %d, gyro: %f %f %f\n", accXConv, accYConv, accZConv, magXComp, magYComp, magZComp, hall, gyroXConv, gyroYConv, gyroZConv);
	}
	return true;
}

bool IMU_BMX160::magCompensateFifoData(uint8_t *fifoData, uint16_t fifoLen, bmx160_fifo_dataset_len_t datasetLen, bmm150_trim_registers *trimDataIn) {
    uint16_t iterator = 0;
    int16_t magX, magY, magZ;
    uint16_t hall;
    int16_t temp;
    if(fifoLen < datasetLen) { return false; }
    if(fifoLen % datasetLen != 0) { return false; }
    while(true) {
        if(iterator + datasetLen > fifoLen) { break; } // BUG before: >= -> last dataset was not compensated
        fifoData[iterator] = BMM150_GET_BITS(fifoData[iterator], BMM150_DATA_X); // Mag X axis data
        temp = ((int16_t)((int8_t)fifoData[iterator+1])) * 32; // Shift the MSB data to left by 5 bits, Multiply by 32 to get the shift left by 5 value
        magX = (int16_t)(temp | fifoData[iterator]); // Raw mag X axis data
        iterator += 2;

        fifoData[iterator] = BMM150_GET_BITS(fifoData[iterator], BMM150_DATA_Y); // Mag Y axis data
        temp = ((int16_t)((int8_t)fifoData[iterator+1])) * 32; // Shift the MSB data to left by 5 bits, Multiply by 32 to get the shift left by 5 value
        magY = (int16_t)(temp | fifoData[iterator]); // Raw mag Y axis data
        iterator += 2;

        fifoData[iterator] = BMM150_GET_BITS(fifoData[iterator], BMM150_DATA_Z); // Mag Z axis data
        temp = ((int16_t)((int8_t)fifoData[iterator+1])) * 128; // Shift the MSB data to left by 5 bits, Multiply by 32 to get the shift left by 5 value
        magZ = (int16_t)(temp | fifoData[iterator]); // Raw mag Z axis data
        iterator += 2;

        fifoData[iterator] = BMM150_GET_BITS(fifoData[iterator], BMM150_DATA_RHALL);
        hall = (uint16_t)(((uint16_t)fifoData[iterator+1]<< 6) | fifoData[iterator]);
        iterator += 2;

        magX = magCompensateXandConvertToMicroTesla(magX, hall, trimDataIn);
        magY = magCompensateYandConvertToMicroTesla(magY, hall, trimDataIn);
        magZ = magCompensateZandConvertToMicroTesla(magZ, hall, trimDataIn);

        iterator -= 8; // go back to beginning for updating fifo
        fifoData[iterator] = magX;
        fifoData[iterator+1] = magX >> 8;
        fifoData[iterator+2] = magY;
        fifoData[iterator+3] = magY >> 8;
        fifoData[iterator+4] = magZ;
        fifoData[iterator+5] = magZ >> 8;
        fifoData[iterator+6] = hall; // unchanged
        fifoData[iterator+7] = hall >> 8; // unchanged

        iterator += datasetLen; // skip other data
    }

    return true;
}

void IMU_BMX160::magCompensatePrintTrimData(bmm150_trim_registers *trimData) {
    printf(" dig_x1 (int8): %d\n", trimData->dig_x1);
    printf(" dig_y1 (int8): %d\n", trimData->dig_y1);
    printf(" dig_x2 (int8): %d\n", trimData->dig_x2);
    printf(" dig_y2 (int8): %d\n", trimData->dig_y2);
    printf(" dig_z1 (uint16): %d\n", trimData->dig_z1);
    printf(" dig_z2 (int16): %d\n", trimData->dig_z2);
    printf(" dig_z3 (int16): %d\n", trimData->dig_z3);
    printf(" dig_z4 (int16): %d\n", trimData->dig_z4);
    printf(" dig_xy1 (uint8): %d\n", trimData->dig_xy1);
    printf(" dig_xy2 (int8): %d\n", trimData->dig_xy2);
    printf(" dig_xyz1 (uint16): %d\n", trimData->dig_xyz1);
}

bool IMU_BMX160::waitOnIndirectMagRead() {
    uint8_t reg = 0;
    uint8_t timeout = 0;
    bool error = false;
    while(true) { // wait for STATUS mag_man_op = 0
        Timing::delay(10); // 10ms is fixed delay in BMI160 driver for indirect read (not polling BMX160_STATUS_ADDR)
		reg = i2c.readRegister(IMU_BMX160_ADDRESS, BMX160_STATUS_ADDR, error); // get current acc status
		if(error) { return false; }
		if((reg & 0b100) == 0) { // mag_man_op = 0
            break;
        }
		timeout++;
		if(timeout > 50) { // 50 * 10 = 500ms
			return false;
		}
	}
    return true;
}

bool IMU_BMX160::magCompensateReadTrimData(bmm150_trim_registers *trimData) {
    uint8_t trim_x1y1[2] = { 0 };
    uint8_t trim_xyz_data[4] = { 0 };
    uint8_t trim_xy1xy2[10] = { 0 };
    uint16_t temp_msb = 0;

    // put magnetometer in normal mode
    if(!magSetMode(BMX160_MAGN_NORMAL_MODE)) { return false; }
	Timing::delay(2); // datasheet recommends 650us

    // mag_manual_en = 1 (start register access), mag_offset = 0b0000 (max offset), mag_rd_burst = 2 byte (0b00 = 1 byte, 0b01 = 2 byte, 0b10 = 6 byte, 0b11 = 8 byte)
    if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_0_ADDR, 0x80 | 0b01)) { return false; } // 2 bytes burst read
    Timing::delay(2); // not really needed, but maybe better

    // indirect write to BMM to wake it up
    if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, 0x01)) {return false; } // set power control bit to 1 -> wake the BMM up! (important)
    if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_2_ADDR, 0x4B)) {return false; } // power control register
    if(!waitOnIndirectMagRead()) { return false; } // wait on result
    
    // indirect read 1: trim_x1y1 (2 bytes)
    if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_1_ADDR, BMM150_DIG_X1)) { return false; } // indirect read register
    if(!waitOnIndirectMagRead()) { return false; } // wait on result
    if(!i2c.readRegisterPageArray(IMU_BMX160_ADDRESS, BMX160_MAG_DATA_ADDR, trim_x1y1, 2)) { return false; } // reading data
    Timing::delay(2); // not really needed, but maybe better

    // indirect read 2: trim_xyz_data (4 bytes)
    for(uint8_t i=0; i<2; i++) {
        if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_1_ADDR, BMM150_DIG_Z4_LSB + (i*2))) { return false; } // indirect read register
        if(!waitOnIndirectMagRead()) { return false; } // wait on result
        if(!i2c.readRegisterPageArray(IMU_BMX160_ADDRESS, BMX160_MAG_DATA_ADDR, trim_xyz_data + (i*2), 2)) { return false; } // reading data
        Timing::delay(2); // not really needed, but maybe better
    }

    // indirect read 3: trim_xy1xy2 (10 bytes)
    for(uint8_t i=0; i<5; i++) {
        if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_1_ADDR, BMM150_DIG_Z2_LSB + (i*2))) { return false; } // indirect read register
        if(!waitOnIndirectMagRead()) { return false; } // wait on result
        if(!i2c.readRegisterPageArray(IMU_BMX160_ADDRESS, BMX160_MAG_DATA_ADDR, trim_xy1xy2 + (i*2), 2)) { return false; } // reading data
        Timing::delay(2); // not really needed, but maybe better
    }

    // copy data into struct
    trimData->dig_x1 = (int8_t)trim_x1y1[0];
    trimData->dig_y1 = (int8_t)trim_x1y1[1];
    trimData->dig_x2 = (int8_t)trim_xyz_data[2];
    trimData->dig_y2 = (int8_t)trim_xyz_data[3];
    temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
    trimData->dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);
    temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
    trimData->dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);
    temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
    trimData->dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);
    temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
    trimData->dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);
    trimData->dig_xy1 = trim_xy1xy2[9];
    trimData->dig_xy2 = (int8_t)trim_xy1xy2[8];
    temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
    trimData->dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);

    // indirect write to MAG register 0x4B: suspend mode
    if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_3_ADDR, 0x00)) { return false; } // set power control bit to 0 -> send MAG to suspend mode
    if(!i2c.writeRegister(IMU_BMX160_ADDRESS, BMX160_MAGN_IF_2_ADDR, 0x4B)) { return false; }
    if(!waitOnIndirectMagRead()) { return false; } // wait on result

	// put MAG_IF into suspend mode
    if(!magSetMode(BMX160_MAGN_SUSPEND_MODE)) { return false; } // send INTERFACE to suspend mode as well

    // perform a soft reset of the whole IMU
    if(!softReset(false)) { return false; } // MAG already put into suspend mode, just reset the rest as well

    return true;
}

int16_t IMU_BMX160::magCompensateXandConvertToMicroTesla(int16_t mag_data_x, uint16_t data_rhall, bmm150_trim_registers *trimData) {
    int16_t retval;
    uint16_t process_comp_x0 = 0;
    int32_t process_comp_x1;
    uint16_t process_comp_x2;
    int32_t process_comp_x3;
    int32_t process_comp_x4;
    int32_t process_comp_x5;
    int32_t process_comp_x6;
    int32_t process_comp_x7;
    int32_t process_comp_x8;
    int32_t process_comp_x9;
    int32_t process_comp_x10;
    /* Overflow condition check */
    if(mag_data_x != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP) {
        if(data_rhall != 0) {
            /* Availability of valid data */
            process_comp_x0 = data_rhall;
        }
        else if(trimData->dig_xyz1 != 0) {
            process_comp_x0 = trimData->dig_xyz1;
        }
        else {
            process_comp_x0 = 0;
        }

        if(process_comp_x0 != 0) {
            /* Processing compensation equations */
            process_comp_x1 = ((int32_t)trimData->dig_xyz1) * 16384;
            process_comp_x2 = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_x2);
            process_comp_x3 = (((int32_t)retval) * ((int32_t)retval));
            process_comp_x4 = (((int32_t)trimData->dig_xy2) * (process_comp_x3 / 128));
            process_comp_x5 = (int32_t)(((int16_t)trimData->dig_xy1) * 128);
            process_comp_x6 = ((int32_t)retval) * process_comp_x5;
            process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
            process_comp_x8 = ((int32_t)(((int16_t)trimData->dig_x2) + ((int16_t)0xA0)));
            process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
            process_comp_x10 = ((int32_t)mag_data_x) * process_comp_x9;
            retval = ((int16_t)(process_comp_x10 / 8192));
            retval = (retval + (((int16_t)trimData->dig_x1) * 8)) / 16;
        }
        else {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    }
    else {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }
    return retval;
}

int16_t IMU_BMX160::magCompensateYandConvertToMicroTesla(int16_t mag_data_y, uint16_t data_rhall, bmm150_trim_registers *trimData) {
    int16_t retval;
    uint16_t process_comp_y0 = 0;
    int32_t process_comp_y1;
    uint16_t process_comp_y2;
    int32_t process_comp_y3;
    int32_t process_comp_y4;
    int32_t process_comp_y5;
    int32_t process_comp_y6;
    int32_t process_comp_y7;
    int32_t process_comp_y8;
    int32_t process_comp_y9;
    /* Overflow condition check */
    if(mag_data_y != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP) {
        if(data_rhall != 0) {
            /* Availability of valid data */
            process_comp_y0 = data_rhall;
        }
        else if(trimData->dig_xyz1 != 0) {
            process_comp_y0 = trimData->dig_xyz1;
        }
        else {
            process_comp_y0 = 0;
        }

        if(process_comp_y0 != 0) {
            /* Processing compensation equations */
            process_comp_y1 = (((int32_t)trimData->dig_xyz1) * 16384) / process_comp_y0;
            process_comp_y2 = ((uint16_t)process_comp_y1) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_y2);
            process_comp_y3 = ((int32_t) retval) * ((int32_t)retval);
            process_comp_y4 = ((int32_t)trimData->dig_xy2) * (process_comp_y3 / 128);
            process_comp_y5 = ((int32_t)(((int16_t)trimData->dig_xy1) * 128));
            process_comp_y6 = ((process_comp_y4 + (((int32_t)retval) * process_comp_y5)) / 512);
            process_comp_y7 = ((int32_t)(((int16_t)trimData->dig_y2) + ((int16_t)0xA0)));
            process_comp_y8 = (((process_comp_y6 + ((int32_t)0x100000)) * process_comp_y7) / 4096);
            process_comp_y9 = (((int32_t)mag_data_y) * process_comp_y8);
            retval = (int16_t)(process_comp_y9 / 8192);
            retval = (retval + (((int16_t)trimData->dig_y1) * 8)) / 16;
        }
        else {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    }
    else {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }
    return retval;
}

int16_t IMU_BMX160::magCompensateZandConvertToMicroTesla(int16_t mag_data_z, uint16_t data_rhall, bmm150_trim_registers *trimData) {
    int32_t retval;
    int16_t process_comp_z0;
    int32_t process_comp_z1;
    int32_t process_comp_z2;
    int32_t process_comp_z3;
    int16_t process_comp_z4;
    if(mag_data_z != BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL) {
        if((trimData->dig_z2 != 0) && (trimData->dig_z1 != 0) && (data_rhall != 0) && (trimData->dig_xyz1 != 0)) {
            /*Processing compensation equations */
            process_comp_z0 = ((int16_t)data_rhall) - ((int16_t) trimData->dig_xyz1);
            process_comp_z1 = (((int32_t)trimData->dig_z3) * ((int32_t)(process_comp_z0))) / 4;
            process_comp_z2 = (((int32_t)(mag_data_z - trimData->dig_z4)) * 32768);
            process_comp_z3 = ((int32_t)trimData->dig_z1) * (((int16_t)data_rhall) * 2);
            process_comp_z4 = (int16_t)((process_comp_z3 + (32768)) / 65536);
            retval = ((process_comp_z2 - process_comp_z1) / (trimData->dig_z2 + process_comp_z4));

            /* Saturate result to +/- 2 micro-tesla */
            if(retval > BMM150_POSITIVE_SATURATION_Z) {
                retval = BMM150_POSITIVE_SATURATION_Z;
            }
            else if(retval < BMM150_NEGATIVE_SATURATION_Z) {
                retval = BMM150_NEGATIVE_SATURATION_Z;
            }
            /* Conversion of LSB to micro-tesla */
            retval = retval / 16;
        }
        else {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    }
    else {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }
    return (int16_t)retval;
}

int16_t IMU_BMX160::magXConvertToMicroTesla(int16_t mag_data_x, bmm150_trim_registers *trimData) {
	// according to Bosch: pass this argument as default r_hall value to deactivate temperature compensation
	return magCompensateXandConvertToMicroTesla(mag_data_x, trimData->dig_xyz1, trimData);
}

int16_t IMU_BMX160::magYConvertToMicroTesla(int16_t mag_data_y, bmm150_trim_registers *trimData) {
	// according to Bosch: pass this argument as default r_hall value to deactivate temperature compensation
	return magCompensateYandConvertToMicroTesla(mag_data_y, trimData->dig_xyz1, trimData);
}

int16_t IMU_BMX160::magZConvertToMicroTesla(int16_t mag_data_z, bmm150_trim_registers *trimData) {
	// according to Bosch: pass this argument as default r_hall value to deactivate temperature compensation
	return magCompensateZandConvertToMicroTesla(mag_data_z, trimData->dig_xyz1, trimData);
}	
