#ifndef ModuleBARO_BSEC_BME680_h
#define ModuleBARO_BSEC_BME680_h

#include "InterfaceI2C.h" // for I2C
#include <stdint.h>
#include "bsec.h"

// for BSEC offline: gas sampling rate needs to be 1/3Hz or 1/300Hz
	
#define BARO_BSEC_BME680_ADDRESS					0x76

class BARO_BSEC_BME680 {
	public:
		BARO_BSEC_BME680();
		bool init();
		int8_t checkStatus(); // 0 = okay, negative = error, positive = warning
		bool run(); // true = new data available
		
		Bsec bsec;
};

#endif
