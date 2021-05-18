#ifndef InterfaceI2C_h
#define InterfaceI2C_h

// TODO: test I2C with long messages (like BMX160 data)

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
						
#define I2C_MASTER_TX_BUF_DISABLE 	0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 	0                           /*!< I2C master doesn't need buffer */
#define ACK_CHECK_EN 				0x1
#define I2C_TIMEOUT					1000
#define I2C_BUFFER					1024						// was 32
#define I2C_ARDUINO_BUFFER_LIMIT	32

typedef enum {
	I2C_FREQ_HZ_400KHZ = 400000,								// 400kHz (fast mode)
    I2C_FREQ_HZ_1MHZ = 1000000									// 1MHz (not supported by every device)
} i2c_clock_speed_t;

class I2C {
	public:
		I2C();
		void begin(i2c_clock_speed_t clockSpeed);
		bool changeClockSpeed(i2c_clock_speed_t clockNew);
		//void setClock(uint32_t clock);
		void beginTransmission(uint8_t address);
		void write(uint8_t data);
		uint8_t read();
		uint8_t endTransmission();
		bool available();
		void requestFrom(uint8_t address, uint16_t quantity);

		bool isAlive(uint8_t address);

		bool writeRegister(uint8_t address, uint8_t reg, uint8_t val); // uses writeRegisterBase
		bool writeRegisterPageInt32(uint8_t address, uint8_t reg, uint32_t val); // uses writeRegisterBase
		bool writeRegisterBase(uint8_t address, uint8_t reg, uint8_t vals[], uint16_t len);

		uint8_t readRegister(uint8_t address, uint8_t reg, bool &error); // uses readRegisterBase
		uint32_t readRegisterPageInt32(uint8_t address, uint8_t reg, bool &error); // uses readRegisterBase
		bool readRegisterPageArray(uint8_t address, uint8_t reg, uint8_t vals[], uint16_t len); // uses readRegisterBase
		bool readRegisterBase(uint8_t address, uint8_t regs[], uint8_t reg_len, uint8_t vals[], uint16_t len);

		bool readRegisterBit(uint8_t address, uint8_t reg, uint8_t bit, bool &error);
	private:
		i2c_cmd_handle_t cmd;
		bool cmdValid;
		uint16_t readQuantity;
		uint8_t buffer[I2C_BUFFER];
		uint16_t bufferPointer;
};

extern I2C i2c;

#endif
