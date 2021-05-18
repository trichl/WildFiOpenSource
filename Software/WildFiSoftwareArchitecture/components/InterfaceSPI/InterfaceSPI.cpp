#include "InterfaceSPI.h"

SPI spi = SPI();

/** SPI Wrapper */
SPI::SPI() {
	spiHandle = NULL;
	spiInitialized = false;
}

bool SPI::init() {
	if(spiInitialized) { 
		return true;
	}
	esp_err_t ret;
	spi_bus_config_t buscfg = {
		.mosi_io_num=GPIO_NUM_13,
		.miso_io_num=GPIO_NUM_12,
		.sclk_io_num=GPIO_NUM_14,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1
	};
	spi_device_interface_config_t devcfg = {
		.command_bits = 8,
		.address_bits = 8,									// sometimes more, sometimes less
		//.dummy_bits = 8,
		.mode = 0,											// SPI mode 0: CPOL = 0, CPHA = 0
		//.duty_cycle_pos = 128,
		//.cs_ena_pretrans = 0,
		//.cs_ena_posttrans = 3								// keep the CS low 3 cycles after transaction, to stop the master from missing the last bit when CS has less propagation delay than CLK
		.clock_speed_hz = SPI_SPEED_MHZ * 1000 * 1000,		// flash supports 133MHz max
		.spics_io_num = GPIO_NUM_15,						// CS pin
		.flags = 0,
		.queue_size = 1,									// we want to be able to queue 1 transactions at a time
		.pre_cb = NULL,
		.post_cb = NULL,
	};
	ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1); // initialize the SPI bus, DMA IS ENABLED!!!!
	if(ret != ESP_OK) {
		return false;
	}
	ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spiHandle); // attach the memory to the SPI bus
	if(ret != ESP_OK) {
		return false;
	}
	spiInitialized = true;
	return true;
}

bool SPI::deinit() {
	if(spiInitialized) {
		if(spi_bus_remove_device(spiHandle) != ESP_OK) {
			return false;
		}	
		if(spi_bus_free(HSPI_HOST) != ESP_OK) {
			return false;
		}
		spiHandle = NULL;
		spiInitialized = false;
		return true;
	}
	return false;
}

/*uint32_t SPI::readMax4Byte(uint16_t cmd, uint64_t addr, size_t length) {
	if((!spiInitialized) || (length > 4)) {
		return 0xFFFFFFFF;
	}
	spi_transaction_t t = {
		.flags = SPI_TRANS_USE_RXDATA,
		.cmd = cmd,
		.addr = addr,
		.length = 8 * length,
		.rxlength = 8 * length
	};
	esp_err_t err = spi_device_polling_transmit(spiHandle, &t);
	if(err != ESP_OK) {
		return 0xFFFFFFFF;
    }
	uint32_t result = 0;
	result = *(uint32_t*)t.rx_data;
	
	return result;
}*/

bool SPI::read(uint16_t cmd, uint8_t cmdLengthByte, uint64_t addr, uint8_t addrLengthByte, uint8_t *data, uint16_t dataLength) {
	if( (!spiInitialized) || (cmdLengthByte > 2) || (addrLengthByte > 8) ) {
		return false;
	}
	cmdLengthByte *= 8; // byte to bit
	addrLengthByte *= 8; // byte to bit
	//printf("ADDR: %llu (%d)\n", addr, addrLengthByte); // DEBUG
	spi_transaction_ext_t t = (spi_transaction_ext_t) {
		.base = {
			.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
			.cmd = cmd,
			.addr = addr,
			.length = 8 * ((size_t) dataLength),
			.tx_buffer = NULL,
			.rx_buffer = data,
            },
		.command_bits = cmdLengthByte,
		.address_bits = addrLengthByte,  
	};
	
	esp_err_t err = spi_device_polling_transmit(spiHandle, (spi_transaction_t*)&t);
	if(err != ESP_OK) {
		return false;
    }
	
	return true;
}

bool SPI::write(uint16_t cmd, uint8_t cmdLengthByte, uint64_t addr, uint8_t addrLengthByte, const uint8_t *data, uint16_t dataLength) {
	if(!spiInitialized) {
		return false;
	}
	cmdLengthByte *= 8; // byte to bit
	addrLengthByte *= 8; // byte to bit
	//printf("ADDR: %llu (%d)\n", addr, addrLengthByte); // DEBUG
	spi_transaction_ext_t t = (spi_transaction_ext_t) {
		.base = {
			.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
			.cmd = cmd,
			.addr = addr,
			.length = 8 * ((size_t) dataLength),
			.tx_buffer = data,
			.rx_buffer = NULL,
            },
		.command_bits = cmdLengthByte,
		.address_bits = addrLengthByte,  
	};
	
	esp_err_t err = spi_device_polling_transmit(spiHandle, (spi_transaction_t*)&t);
	if(err != ESP_OK) {
		return false;
    }
	return true;
}

uint32_t SPI::readMax4Byte(uint16_t cmd, uint8_t cmdLengthByte, uint64_t addr, uint8_t addrLengthByte, size_t dataLengthByte) {
	if( (!spiInitialized) || (dataLengthByte > 4) || (cmdLengthByte > 2) || (addrLengthByte > 8) ) {
		return 0xFFFFFFFF;
	}
	cmdLengthByte *= 8; // byte to bit
	addrLengthByte *= 8; // byte to bit
	//printf("ADDR: %llu (%d)\n", addr, addrLengthByte); // DEBUG
	spi_transaction_ext_t t = (spi_transaction_ext_t) {
		.base = {
			.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
			.cmd = cmd,
			.addr = addr,
			.length = 8 * dataLengthByte,
			.rxlength = 8 * dataLengthByte,
            },
		.command_bits = cmdLengthByte,
		.address_bits = addrLengthByte,  
	};
	if(dataLengthByte == 0) {
		t.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
	}
	
	esp_err_t err = spi_device_polling_transmit(spiHandle, (spi_transaction_t*)&t);
	if(err != ESP_OK) {
		return 0xFFFFFFFF;
    }

	uint32_t result = 0;
	
	if(dataLengthByte > 0) {
		result = *(uint32_t*)t.base.rx_data;
	}
	return result;
}
