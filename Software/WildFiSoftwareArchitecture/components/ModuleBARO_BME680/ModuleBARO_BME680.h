#ifndef ModuleBARO_BME680_h
#define ModuleBARO_BME680_h

#include "ModuleBARO_BME680_defs.h"
#include "InterfaceI2C.h"
#include "InterfaceTiming.h"
#include <stdint.h>

// for BSEC offline: gas sampling rate needs to be 1/3Hz or 1/300Hz
	
#define BARO_BME680_ADDRESS					0x76

class BARO_BME680 {
	public:
		BARO_BME680();
		bool init(uint8_t overTemp, uint8_t overHum, uint8_t overPress, uint8_t filterSize, uint16_t heaterTemp, uint16_t heaterTime);
		bool performMeasurement();
		bool measurementFinished();
		bool getResults();
		
		int16_t getTemperature(bool &error); // temperature in degree celsius x100
		uint32_t getPressure(bool &error); // pressure in Pascal
		uint32_t getHumidity(bool &error); // humidity in % x1000
		uint32_t getGasResistance(bool &error); // gas resistance in Ohms
	private:
		struct bme680_dev dev; // configuration struct
		struct bme680_field_data data; // data storage
		bool tempEnabled, humEnabled, pressEnabled, filterEnabled, gasEnabled;
		
		// Bosch functions
		int8_t bme680_init();
		int8_t bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len);
		int8_t bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
		int8_t bme680_set_sensor_mode();
		int8_t bme680_get_sensor_mode();
		int8_t bme680_get_sensor_data();
		int8_t bme680_set_sensor_settings(uint16_t desired_settings);
		int8_t bme680_get_sensor_settings(uint16_t desired_settings);

		int8_t get_calib_data();
		int8_t set_gas_config();
		int8_t get_gas_config();
		uint8_t calc_heater_dur(uint16_t dur);
		int16_t calc_temperature(uint32_t temp_adc);
		uint32_t calc_pressure(uint32_t pres_adc);
		uint32_t calc_humidity(uint16_t hum_adc);
		uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range);
		uint8_t calc_heater_res(uint16_t temp);
		int8_t read_field_data();	
};

#endif
