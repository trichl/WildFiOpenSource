#include "ModuleIMU_BMX160_WAKESTUB.h"

RTC_IRAM_ATTR bool waitForModeChange(uint8_t mode) {
	uint8_t regMode = 0;
	uint8_t timeout = 0;
	while(true) {
		ets_delay_us(2*1000);

		beginTransmissionSw(IMU_BMX160_ADDRESS);
		writeSw(BMX160_PMU_STATUS_ADDR);
		endTransmissionSw();
		requestFromSw(IMU_BMX160_ADDRESS, 1, &regMode);

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
		if(timeout > 250) {
			return false;
		}
	}
	return true;
}

RTC_IRAM_ATTR uint16_t getAccFIFOLength() {
    // READ FIFO LENGTH
    uint8_t data[2] = {0};
    beginTransmissionSw(IMU_BMX160_ADDRESS);
	writeSw(BMX160_FIFO_LENGTH_ADDR);
	endTransmissionSw();
	requestFromSw(IMU_BMX160_ADDRESS, 2, &data[0]);
	return ((uint16_t) ((data[1] << 8) | data[0]));
}

RTC_IRAM_ATTR bool readAccFIFO(uint8_t *fifoBuffer, uint16_t length, bool waitUntilBackInLowPower) {
    // PUT IN NORMAL MODE
    beginTransmissionSw(IMU_BMX160_ADDRESS);
	writeSw(BMX160_COMMAND_REG_ADDR);
	writeSw(BMX160_ACCEL_NORMAL_MODE);
	endTransmissionSw();

	// WAIT UNTIL IN NORMAL MODE, according to datasheet: 3.2ms for ACC (!) low power -> normal
	if(!waitForModeChange(BMX160_ACCEL_NORMAL_MODE)) {
		return false;
	}

    // GET DATA
    beginTransmissionSw(IMU_BMX160_ADDRESS);
	writeSw(BMX160_FIFO_DATA_ADDR);
	endTransmissionSw();
	requestFromSw(IMU_BMX160_ADDRESS, length, fifoBuffer);

    // PUT IN LOW POWER MODE
    beginTransmissionSw(IMU_BMX160_ADDRESS);
	writeSw(BMX160_COMMAND_REG_ADDR);
	writeSw(BMX160_ACCEL_LOWPOWER_MODE);
	endTransmissionSw();

	// WAIT UNTIL IN LOW POWER MODE (if not going to sleep directly afterwards)
	if(waitUntilBackInLowPower) {
		if(!waitForModeChange(BMX160_ACCEL_LOWPOWER_MODE)) {
			return false;
		}
	}

	return true;
}
