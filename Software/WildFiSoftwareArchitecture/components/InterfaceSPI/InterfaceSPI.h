#ifndef InterfaceSPI_h
#define InterfaceSPI_h

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"

#define SPI_SPEED_MHZ				10

class SPI {
	public:
		SPI();
		bool init();
		bool deinit();
		bool read(uint16_t cmd, uint8_t cmdLengthByte, uint64_t addr, uint8_t addrLengthByte, uint8_t *data, uint16_t dataLength); // data should be allocated in DMA memory!
		bool write(uint16_t cmd, uint8_t cmdLengthByte, uint64_t addr, uint8_t addrLengthByte, const uint8_t *data, uint16_t dataLength); // data should be allocated in DMA memory!
		uint32_t readMax4Byte(uint16_t cmd, uint8_t cmdLengthByte, uint64_t addr, uint8_t addrLengthByte, size_t dataLengthByte);
	private:
		bool spiInitialized;
		spi_device_handle_t spiHandle;
};

extern SPI spi;

#endif
