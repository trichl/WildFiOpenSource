#include "InterfaceI2C.h"

I2C i2c = I2C();

/** I2C Wrapper */
I2C::I2C() {
	cmd = NULL;
	cmdValid = false;
	readQuantity = 0;
	bufferPointer = 0;
}

bool I2C::available() {
	// mocked for BSEC library, might need to be implemented in multi-master-environments
	return true;
}

void I2C::begin(i2c_clock_speed_t clockSpeed) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_21;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_io_num = GPIO_NUM_22;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = (uint32_t) clockSpeed;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0)); // TODO: evaluate return value
}

bool I2C::changeClockSpeed(i2c_clock_speed_t clockNew) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_21;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_io_num = GPIO_NUM_22;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = (uint32_t) clockNew;
	if(i2c_param_config(I2C_NUM_0, &conf) != ESP_OK) {
		return false;
	}
	return true;
}

/*void I2C::setClock(uint32_t clock) {
	//Wire.setClock(clock);
	// mocked, done in begin()
}*/

void I2C::beginTransmission(uint8_t address) {
	//Wire.beginTransmission(address);
	cmd = i2c_cmd_link_create();
	cmdValid = true;
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // send 1 byte with slave address
}

void I2C::write(uint8_t data) {
	//Wire.write(data);
	if(cmdValid) {
		i2c_master_write_byte(cmd, data, ACK_CHECK_EN); // send 1 byte
	}
}

uint8_t I2C::endTransmission() {
	//return Wire.endTransmission();
	esp_err_t i2c_ret = ESP_OK;
	if(!cmdValid) {
		return 1;
	}
	i2c_master_stop(cmd); // stop sequence
	i2c_ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, (I2C_TIMEOUT / portTICK_RATE_MS)); // I2C_TIMEOUT = max wait time (task blocked, not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
	i2c_cmd_link_delete(cmd);
	cmdValid = false;
	if (i2c_ret == ESP_OK) {
		return 0; // 0 = okay!
    }
	return 1; // 1 = not okay
}

void I2C::requestFrom(uint8_t address, uint16_t quantity) {
	//Wire.requestFrom(address, quantity);
	cmd = i2c_cmd_link_create();
	cmdValid = true;
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_READ, ACK_CHECK_EN); // send 1 byte with slave address
	readQuantity = quantity;
	if(readQuantity > I2C_BUFFER) {
		readQuantity = I2C_BUFFER;
	}
	bufferPointer = 0;
	
    if(readQuantity > 1) { // NEW AND UNTESTED!!!
        i2c_master_read(cmd, buffer, readQuantity - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &buffer[readQuantity - 1], I2C_MASTER_NACK);
	/*for(uint16_t i = 0; i < readQuantity; i++) {
		if(i == readQuantity-1) { // last byte
			i2c_master_read_byte(cmd, &buffer[i], I2C_MASTER_NACK);
		}
		else {
			i2c_master_read_byte(cmd, &buffer[i], I2C_MASTER_ACK);
		}
	}*/
	endTransmission(); // get the data!
}

uint8_t I2C::read() {
	//return Wire.read();
	bufferPointer++;
	return buffer[bufferPointer-1];
}

bool I2C::isAlive(uint8_t address) {
	beginTransmission(address);
	if(endTransmission() == 0) {
		return true;
	}
	return false;
}

bool I2C::writeRegister(uint8_t address, uint8_t reg, uint8_t val) {
	return writeRegisterBase(address, reg, &val, 1);
}

bool I2C::writeRegisterPageInt32(uint8_t address, uint8_t reg, uint32_t val) {
	// sensor needs to support page mode (automatic address increment)
	uint8_t const LEN = 4;
	uint8_t val_array[LEN];

	for(uint8_t i = 0; i < LEN; i++) {
		val_array[i] = (val >> (i*8)) & 0xff;
	}

	return writeRegisterBase(address, reg, val_array, LEN);
}

bool I2C::writeRegisterBase(uint8_t address, uint8_t reg, uint8_t vals[], uint16_t len) {
	// sensor needs to support page mode (automatic address increment)
	beginTransmission(address);
	write(reg);

	for(uint16_t i = 0; i < len; i++) {
		write(vals[i]);
	}

	if(endTransmission() != 0) {
		return false;
	}
	return true;
}

uint8_t I2C::readRegister(uint8_t address, uint8_t reg, bool &error) {
	uint8_t const LEN = 1;
	uint8_t val_array[LEN];

	// important: do not reset error here, e.g. IMU_BMX160 works under this assumption
	if(!readRegisterBase(address, &reg, 1, val_array, LEN)) {
		error = true;
		return 0x0;
	}
	return val_array[0];
}

uint32_t I2C::readRegisterPageInt32(uint8_t address, uint8_t reg, bool &error) {
	// sensor needs to support page mode (automatic address increment)
	uint8_t const LEN = 4;
	uint8_t vals[LEN];
	uint32_t result = 0;
	uint32_t temp = 0;
	error = false;

	if(!readRegisterBase(address, &reg, 1, vals, LEN)) {
		error = true;
		return 0x0;
	}
	for(uint8_t i = 0; i < LEN; i++) {
		temp = (uint32_t) vals[i];
		result |= temp << (i*8);
	}
	return result;
}

bool I2C::readRegisterPageArray(uint8_t address, uint8_t reg, uint8_t vals[], uint16_t len) {
	// sensor needs to support page mode (automatic address increment)
	return readRegisterBase(address, &reg, 1, vals, len);
}

bool I2C::readRegisterBase(uint8_t address, uint8_t regs[], uint8_t reg_len, uint8_t vals[], uint16_t len) {
	beginTransmission(address);
	for(uint8_t i = 0; i < reg_len; i++) {
		write(regs[i]);
	}
	if(endTransmission() != 0) {
		return false;
	}
	requestFrom(address, len);
	for(uint16_t i = 0; i < len; i++) {
		vals[i] = read();
	}
	// NOT NEEDED! after removal not tested with ATTINY and stuff
	/*if(endTransmission() != 0) {
		return false;
	}*/
	return true;
}

bool I2C::readRegisterBit(uint8_t address, uint8_t reg, uint8_t bit, bool &error) {
	uint8_t reg_result = readRegister(address, reg, error);
	return (reg_result >> bit) & 1UL;
}
