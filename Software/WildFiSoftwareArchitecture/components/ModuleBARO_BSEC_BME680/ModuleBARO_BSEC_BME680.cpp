#include "ModuleBARO_BSEC_BME680.h"

// TODO: save raw data instead of calculated values -> calculation offline with parameters, seems constant? own program to download calibration via serial
// TODO: bme680_set_sensor_mode has while loop to check if in sleep -> remove?

BARO_BSEC_BME680::BARO_BSEC_BME680() {

}

bool BARO_BSEC_BME680::init() {
	bsec.begin(BARO_BSEC_BME680_ADDRESS, i2c);
	
	if(checkStatus() != 0) {
		return false;
	}

	bsec_virtual_sensor_t sensorList[10] = {
		BSEC_OUTPUT_RAW_TEMPERATURE,
		BSEC_OUTPUT_RAW_PRESSURE,
		BSEC_OUTPUT_RAW_HUMIDITY,
		BSEC_OUTPUT_RAW_GAS,
		BSEC_OUTPUT_IAQ,
		BSEC_OUTPUT_STATIC_IAQ,
		BSEC_OUTPUT_CO2_EQUIVALENT,
		BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
	};
		
	bsec.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP); // TODO: ULP!!!
		
	if(checkStatus() != 0) {
		return false;
	}
	return true;
}

int8_t BARO_BSEC_BME680::checkStatus() {
	return (int8_t) bsec.status;
}

bool BARO_BSEC_BME680::run() {
	return bsec.run();
}
