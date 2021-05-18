#ifndef InterfaceTiming_h
#define InterfaceTiming_h

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"

class Timing {
	public:
		static void delay(uint16_t d);
		static uint64_t millis();
};

#endif
