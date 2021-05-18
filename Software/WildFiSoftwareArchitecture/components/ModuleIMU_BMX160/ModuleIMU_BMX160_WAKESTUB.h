#ifndef ModuleIMU_BMX160_WAKESTUB_h
#define ModuleIMU_BMX160_WAKESTUB_h

#include <stdint.h>

#include "InterfaceSoftwareI2C.h" // needed for spi object and SPI class definition
#include "ModuleIMU_BMX160.h"

RTC_IRAM_ATTR bool waitForModeChange(uint8_t mode);
RTC_IRAM_ATTR uint16_t getAccFIFOLength();
RTC_IRAM_ATTR bool readAccFIFO(uint8_t *fifoBuffer, uint16_t length, bool waitUntilBackInLowPower); // CALL beginSw(); BEFORE!!!, 150ms with 966 bytes in fifo

#endif
