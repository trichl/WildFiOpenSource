#include "ModuleBARO_BME680.h"

// TODO: save raw data instead of calculated values -> calculation offline with parameters, seems constant? own program to download calibration via serial
// TODO: bme680_set_sensor_mode has while loop to check if in sleep -> remove?

BARO_BME680::BARO_BME680() {
	tempEnabled = false;
	humEnabled = false;
	pressEnabled = false;
	filterEnabled = false;
	gasEnabled = false;
}

bool BARO_BME680::init(uint8_t overTemp, uint8_t overHum, uint8_t overPress, uint8_t filterSize, uint16_t heaterTemp, uint16_t heaterTime) {
	dev.amb_temp = 25;
	
	// check if component is okay
	if(bme680_init() != BME680_OK) {
		return false;
	}
	
	// oversampling settings
	tempEnabled = !(overTemp == BME680_OS_NONE);
	dev.tph_sett.os_temp = overTemp;
	
	humEnabled = !(overHum == BME680_OS_NONE);
	dev.tph_sett.os_hum = overHum;
	   
	pressEnabled = !(overPress == BME680_OS_NONE);
	dev.tph_sett.os_pres = overPress;
	
	// IIR filter settings
	filterEnabled = !(filterSize == BME680_FILTER_SIZE_0);
	dev.tph_sett.filter = filterSize;
	
	// gas sensor settings
	dev.gas_sett.heatr_temp = heaterTemp;
	dev.gas_sett.heatr_dur = heaterTime;
	if((heaterTemp == 0) || (heaterTime == 0)) {
		dev.gas_sett.heatr_ctrl = BME680_DISABLE_HEATER;
		dev.gas_sett.run_gas = BME680_DISABLE_GAS_MEAS;
		gasEnabled = false;
	}
	else {
		dev.gas_sett.heatr_ctrl = BME680_ENABLE_HEATER;
		dev.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
		gasEnabled = true;
	}
	
	// activate forced mode
	dev.power_mode = BME680_FORCED_MODE;

	return true;
}

bool BARO_BME680::performMeasurement() {
	uint8_t settings = 0;
  
	// TODO: check if measurement finished? = Sleep mode?
	dev.power_mode = BME680_FORCED_MODE; // select power mode
	
	if(tempEnabled) {
		settings |= BME680_OST_SEL;
	}
	if(humEnabled) {
		settings |= BME680_OSH_SEL;
	}
	if(pressEnabled) {
		settings |= BME680_OSP_SEL;
	}
	if(filterEnabled) {
		settings |= BME680_FILTER_SEL;
	}
	if(gasEnabled) {
		settings |= BME680_GAS_SENSOR_SEL;
	}
	
	// set sensor settings
	if(bme680_set_sensor_settings(settings) != BME680_OK) {
		return false;
	}
	
	// trigger the measurement
	if(bme680_set_sensor_mode() != BME680_OK) {
		return false;
	}
	
	return true;
}

bool BARO_BME680::measurementFinished() {
	bool error = false;
	uint8_t mode, modeTemp; // STRANGE: modeTemp necessary, otherwise bits are not set correctly

	modeTemp = i2c.readRegister(BARO_BME680_ADDRESS, BME680_CONF_T_P_MODE_ADDR, error);
	if(!error) {
		mode = (modeTemp & BME680_MODE_MSK);
		if(mode == BME680_SLEEP_MODE) {
			return true;
		}
		Timing::delay(BME680_POLL_PERIOD_MS); // wait 10ms before next polling
	}
	return false;
}

bool BARO_BME680::getResults() {
	return (bme680_get_sensor_data() == BME680_OK);
}

int16_t BARO_BME680::getTemperature(bool &error) {
	error = false;
	if(tempEnabled) {
		return data.temperature;
	}
	error = true;
	return INT16_MAX;
}

uint32_t BARO_BME680::getPressure(bool &error) {
	error = false;
	if(pressEnabled) {
		return data.pressure;
	}
	error = true;
	return UINT32_MAX;	
}

uint32_t BARO_BME680::getHumidity(bool &error) {
	error = false;
	if(humEnabled) {
		return data.humidity;
	}
	error = true;
	return UINT32_MAX;		
}

uint32_t BARO_BME680::getGasResistance(bool &error) {
	error = !gasEnabled || (!(data.status & BME680_HEAT_STAB_MSK)); // error if gas measurement was unstable
	return data.gas_resistance;		
}


/* ----- BOSCH FUNCTIONS ----- */

/* Modified by Timm
 * - Removed floating functions
 * - Removed SPI support
 * - #define for removing gas sensor functions
 * - Removed SoftReset function at beginning (commented out), as device always starts from hard reset
 * - Removed checking chip id -> isAlive function sufficient
 * - Removed null_ptr_check
 * - Removed boundary_check
 * - Removed get_mem_page
 * - Removed bme680_set_profile_dur
 * - Removed dev pointers
 */

int8_t BARO_BME680::bme680_init()
{
	return get_calib_data();
}

int8_t BARO_BME680::bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = BME680_OK;
	dev.com_rslt = (!i2c.readRegisterBase(BARO_BME680_ADDRESS, &reg_addr, 1, reg_data, len));
	if (dev.com_rslt != 0)
		rslt = BME680_E_COM_FAIL;

	return rslt;
}

int8_t BARO_BME680::bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len)
{
	int8_t rslt = BME680_OK;
	/* Length of the temporary buffer is 2*(length of register)*/
	uint8_t tmp_buff[BME680_TMP_BUFFER_LENGTH] = { 0 };
	uint16_t index;

	if ((len > 0) && (len < BME680_TMP_BUFFER_LENGTH / 2)) {
		/* Interleave the 2 arrays */
		for (index = 0; index < len; index++) {
			tmp_buff[(2 * index)] = reg_addr[index];
			tmp_buff[(2 * index) + 1] = reg_data[index];
		}
		/* Write the interleaved array */
		if (rslt == BME680_OK) {
			dev.com_rslt = (!i2c.writeRegisterBase(BARO_BME680_ADDRESS, tmp_buff[0], &tmp_buff[1], (2 * len) - 1));
			if (dev.com_rslt != 0)
				rslt = BME680_E_COM_FAIL;
		}
	} else {
		rslt = BME680_E_INVALID_LENGTH;
	}
	return rslt;
}

int8_t BARO_BME680::bme680_set_sensor_settings(uint16_t desired_settings)
{
	int8_t rslt = BME680_OK;
	uint8_t reg_addr;
	uint8_t data = 0;
	uint8_t count = 0;
	uint8_t reg_array[BME680_REG_BUFFER_LENGTH] = { 0 };
	uint8_t data_array[BME680_REG_BUFFER_LENGTH] = { 0 };
	uint8_t intended_power_mode = dev.power_mode; /* Save intended power mode */

	#ifndef NO_GAS_MEASUREMENTS
	if (desired_settings & BME680_GAS_MEAS_SEL)
		rslt = set_gas_config();
	#endif

	dev.power_mode = BME680_SLEEP_MODE;
	if (rslt == BME680_OK)
		rslt = bme680_set_sensor_mode();

	/* Selecting the filter */
	if (desired_settings & BME680_FILTER_SEL) {
		reg_addr = BME680_CONF_ODR_FILT_ADDR;
		rslt = bme680_get_regs(reg_addr, &data, 1);

		if (desired_settings & BME680_FILTER_SEL)
			data = BME680_SET_BITS(data, BME680_FILTER, dev.tph_sett.filter);

		reg_array[count] = reg_addr; /* Append configuration */
		data_array[count] = data;
		count++;
	}

	/* Selecting heater control for the sensor */
	if (desired_settings & BME680_HCNTRL_SEL) {
		reg_addr = BME680_CONF_HEAT_CTRL_ADDR;

		rslt = bme680_get_regs(reg_addr, &data, 1);
		data = BME680_SET_BITS_POS_0(data, BME680_HCTRL, dev.gas_sett.heatr_ctrl);

		reg_array[count] = reg_addr; /* Append configuration */
		data_array[count] = data;
		count++;
	}

	/* Selecting heater T,P oversampling for the sensor */
	if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL)) {
		reg_addr = BME680_CONF_T_P_MODE_ADDR;

		rslt = bme680_get_regs(reg_addr, &data, 1);

		if (desired_settings & BME680_OST_SEL)
			data = BME680_SET_BITS(data, BME680_OST, dev.tph_sett.os_temp);

		if (desired_settings & BME680_OSP_SEL)
			data = BME680_SET_BITS(data, BME680_OSP, dev.tph_sett.os_pres);

		reg_array[count] = reg_addr;
		data_array[count] = data;
		count++;
	}

	/* Selecting humidity oversampling for the sensor */
	if (desired_settings & BME680_OSH_SEL) {
		reg_addr = BME680_CONF_OS_H_ADDR;

		rslt = bme680_get_regs(reg_addr, &data, 1);
		data = BME680_SET_BITS_POS_0(data, BME680_OSH, dev.tph_sett.os_hum);

		reg_array[count] = reg_addr; /* Append configuration */
		data_array[count] = data;
		count++;
	}

	/* Selecting the runGas and NB conversion settings for the sensor */
	if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)) {
		reg_addr = BME680_CONF_ODR_RUN_GAS_NBC_ADDR;

		rslt = bme680_get_regs(reg_addr, &data, 1);

		if (desired_settings & BME680_RUN_GAS_SEL)
			data = BME680_SET_BITS(data, BME680_RUN_GAS, dev.gas_sett.run_gas);

		if (desired_settings & BME680_NBCONV_SEL)
			data = BME680_SET_BITS_POS_0(data, BME680_NBCONV, dev.gas_sett.nb_conv);

		reg_array[count] = reg_addr; /* Append configuration */
		data_array[count] = data;
		count++;
	}

	if (rslt == BME680_OK)
		rslt = bme680_set_regs(reg_array, data_array, count);

	/* Restore previous intended power mode */
	dev.power_mode = intended_power_mode;

	return rslt;
}

int8_t BARO_BME680::bme680_get_sensor_settings(uint16_t desired_settings)
{
	int8_t rslt;
	/* starting address of the register array for burst read*/
	uint8_t reg_addr = BME680_CONF_HEAT_CTRL_ADDR;
	uint8_t data_array[BME680_REG_BUFFER_LENGTH] = { 0 };

	rslt = bme680_get_regs(reg_addr, data_array, BME680_REG_BUFFER_LENGTH);

	if (rslt == BME680_OK) {
		if (desired_settings & BME680_GAS_MEAS_SEL)
			rslt = get_gas_config();

		/* get the T,P,H ,Filter,ODR settings here */
		if (desired_settings & BME680_FILTER_SEL)
			dev.tph_sett.filter = BME680_GET_BITS(data_array[BME680_REG_FILTER_INDEX],
				BME680_FILTER);

		if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL)) {
			dev.tph_sett.os_temp = BME680_GET_BITS(data_array[BME680_REG_TEMP_INDEX], BME680_OST);
			dev.tph_sett.os_pres = BME680_GET_BITS(data_array[BME680_REG_PRES_INDEX], BME680_OSP);
		}

		if (desired_settings & BME680_OSH_SEL)
			dev.tph_sett.os_hum = BME680_GET_BITS_POS_0(data_array[BME680_REG_HUM_INDEX],
				BME680_OSH);

		/* get the gas related settings */
		if (desired_settings & BME680_HCNTRL_SEL)
			dev.gas_sett.heatr_ctrl = BME680_GET_BITS_POS_0(data_array[BME680_REG_HCTRL_INDEX],
				BME680_HCTRL);

		if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)) {
			dev.gas_sett.nb_conv = BME680_GET_BITS_POS_0(data_array[BME680_REG_NBCONV_INDEX],
				BME680_NBCONV);
			dev.gas_sett.run_gas = BME680_GET_BITS(data_array[BME680_REG_RUN_GAS_INDEX],
				BME680_RUN_GAS);
		}
	}

	return rslt;
}

int8_t BARO_BME680::bme680_set_sensor_mode()
{
	int8_t rslt = BME680_OK;
	uint8_t tmp_pow_mode;
	uint8_t pow_mode = 0;
	uint8_t reg_addr = BME680_CONF_T_P_MODE_ADDR;

	/* Call repeatedly until in sleep */
	do {
		rslt = bme680_get_regs(BME680_CONF_T_P_MODE_ADDR, &tmp_pow_mode, 1);
		if (rslt == BME680_OK) {
			/* Put to sleep before changing mode */
			pow_mode = (tmp_pow_mode & BME680_MODE_MSK);

			if (pow_mode != BME680_SLEEP_MODE) {
				tmp_pow_mode = tmp_pow_mode & (~BME680_MODE_MSK); /* Set to sleep */
				rslt = bme680_set_regs(&reg_addr, &tmp_pow_mode, 1);
				Timing::delay(BME680_POLL_PERIOD_MS);
			}
		}
	} while (pow_mode != BME680_SLEEP_MODE);

	/* Already in sleep */
	if (dev.power_mode != BME680_SLEEP_MODE) {
		tmp_pow_mode = (tmp_pow_mode & ~BME680_MODE_MSK) | (dev.power_mode & BME680_MODE_MSK);
		if (rslt == BME680_OK)
			rslt = bme680_set_regs(&reg_addr, &tmp_pow_mode, 1);
	}

	return rslt;
}

int8_t BARO_BME680::bme680_get_sensor_mode()
{
	int8_t rslt;
	uint8_t mode;

	rslt = bme680_get_regs(BME680_CONF_T_P_MODE_ADDR, &mode, 1);
	/* Masking the other register bit info*/
	dev.power_mode = mode & BME680_MODE_MSK;

	return rslt;
}

int8_t BARO_BME680::bme680_get_sensor_data()
{
	int8_t rslt;
	
	/* Reading the sensor data in forced mode only */
	rslt = read_field_data();
	if (rslt == BME680_OK) {
		if (data.status & BME680_NEW_DATA_MSK)
			dev.new_fields = 1;
		else
			dev.new_fields = 0;
	}

	return rslt;
}

int8_t BARO_BME680::get_calib_data()
{
	int8_t rslt = BME680_OK;
	uint8_t coeff_array[BME680_COEFF_SIZE] = { 0 };
	uint8_t temp_var = 0; /* Temporary variable */

	rslt = bme680_get_regs(BME680_COEFF_ADDR1, coeff_array, BME680_COEFF_ADDR1_LEN);
	/* Append the second half in the same array */
	if (rslt == BME680_OK)
		rslt = bme680_get_regs(BME680_COEFF_ADDR2, &coeff_array[BME680_COEFF_ADDR1_LEN], BME680_COEFF_ADDR2_LEN);

	/* Temperature related coefficients */
	dev.calib.par_t1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T1_MSB_REG],
		coeff_array[BME680_T1_LSB_REG]));
	dev.calib.par_t2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T2_MSB_REG],
		coeff_array[BME680_T2_LSB_REG]));
	dev.calib.par_t3 = (int8_t) (coeff_array[BME680_T3_REG]);

	/* Pressure related coefficients */
	dev.calib.par_p1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P1_MSB_REG],
		coeff_array[BME680_P1_LSB_REG]));
	dev.calib.par_p2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P2_MSB_REG],
		coeff_array[BME680_P2_LSB_REG]));
	dev.calib.par_p3 = (int8_t) coeff_array[BME680_P3_REG];
	dev.calib.par_p4 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P4_MSB_REG],
		coeff_array[BME680_P4_LSB_REG]));
	dev.calib.par_p5 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P5_MSB_REG],
		coeff_array[BME680_P5_LSB_REG]));
	dev.calib.par_p6 = (int8_t) (coeff_array[BME680_P6_REG]);
	dev.calib.par_p7 = (int8_t) (coeff_array[BME680_P7_REG]);
	dev.calib.par_p8 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P8_MSB_REG],
		coeff_array[BME680_P8_LSB_REG]));
	dev.calib.par_p9 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P9_MSB_REG],
		coeff_array[BME680_P9_LSB_REG]));
	dev.calib.par_p10 = (uint8_t) (coeff_array[BME680_P10_REG]);

	/* Humidity related coefficients */
	dev.calib.par_h1 = (uint16_t) (((uint16_t) coeff_array[BME680_H1_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
		| (coeff_array[BME680_H1_LSB_REG] & BME680_BIT_H1_DATA_MSK));
	dev.calib.par_h2 = (uint16_t) (((uint16_t) coeff_array[BME680_H2_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
		| ((coeff_array[BME680_H2_LSB_REG]) >> BME680_HUM_REG_SHIFT_VAL));
	dev.calib.par_h3 = (int8_t) coeff_array[BME680_H3_REG];
	dev.calib.par_h4 = (int8_t) coeff_array[BME680_H4_REG];
	dev.calib.par_h5 = (int8_t) coeff_array[BME680_H5_REG];
	dev.calib.par_h6 = (uint8_t) coeff_array[BME680_H6_REG];
	dev.calib.par_h7 = (int8_t) coeff_array[BME680_H7_REG];

	/* Gas heater related coefficients */
	
	#ifndef NO_GAS_MEASUREMENTS
	dev.calib.par_gh1 = (int8_t) coeff_array[BME680_GH1_REG];
	dev.calib.par_gh2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_GH2_MSB_REG],
		coeff_array[BME680_GH2_LSB_REG]));
	dev.calib.par_gh3 = (int8_t) coeff_array[BME680_GH3_REG];
	#endif
	
	/* Other coefficients */
	if (rslt == BME680_OK) {
		rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_RANGE_ADDR, &temp_var, 1);

		dev.calib.res_heat_range = ((temp_var & BME680_RHRANGE_MSK) / 16);
		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_VAL_ADDR, &temp_var, 1);

			dev.calib.res_heat_val = (int8_t) temp_var;
			if (rslt == BME680_OK)
				rslt = bme680_get_regs(BME680_ADDR_RANGE_SW_ERR_ADDR, &temp_var, 1);
		}
	}
	dev.calib.range_sw_err = ((int8_t) temp_var & (int8_t) BME680_RSERROR_MSK) / 16;

	return rslt;
}

int8_t BARO_BME680::set_gas_config()
{
	int8_t rslt = BME680_OK;
	uint8_t reg_addr[2] = {0};
	uint8_t reg_data[2] = {0};

	if (dev.power_mode == BME680_FORCED_MODE) {
		reg_addr[0] = BME680_RES_HEAT0_ADDR;
		reg_data[0] = calc_heater_res(dev.gas_sett.heatr_temp);
		reg_addr[1] = BME680_GAS_WAIT0_ADDR;
		reg_data[1] = calc_heater_dur(dev.gas_sett.heatr_dur);
		dev.gas_sett.nb_conv = 0;
	} else {
		rslt = BME680_W_DEFINE_PWR_MODE;
	}
	if (rslt == BME680_OK)
		rslt = bme680_set_regs(reg_addr, reg_data, 2);
	return rslt;
}

int8_t BARO_BME680::get_gas_config()
{
	int8_t rslt = BME680_OK;
	/* starting address of the register array for burst read*/
	uint8_t reg_addr1 = BME680_ADDR_SENS_CONF_START;
	uint8_t reg_addr2 = BME680_ADDR_GAS_CONF_START;
	uint8_t reg_data = 0;

	rslt = bme680_get_regs(reg_addr1, &reg_data, 1);
	if (rslt == BME680_OK) {
		dev.gas_sett.heatr_temp = reg_data;
		rslt = bme680_get_regs(reg_addr2, &reg_data, 1);
		if (rslt == BME680_OK) {
			/* Heating duration register value */
			dev.gas_sett.heatr_dur = reg_data;
		}
	}
			
	return rslt;
}

int16_t BARO_BME680::calc_temperature(uint32_t temp_adc)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int16_t calc_temp;

	var1 = ((int32_t) temp_adc >> 3) - ((int32_t) dev.calib.par_t1 << 1);
	var2 = (var1 * (int32_t) dev.calib.par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t) dev.calib.par_t3 << 4)) >> 14;
	dev.calib.t_fine = (int32_t) (var2 + var3);
	calc_temp = (int16_t) (((dev.calib.t_fine * 5) + 128) >> 8);

	return calc_temp;
}

uint32_t BARO_BME680::calc_pressure(uint32_t pres_adc)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t pressure_comp;

	var1 = (((int32_t)dev.calib.t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
		(int32_t)dev.calib.par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)dev.calib.par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)dev.calib.par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
		((int32_t)dev.calib.par_p3 << 5)) >> 3) +
		(((int32_t)dev.calib.par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)dev.calib.par_p1) >> 15;
	pressure_comp = 1048576 - pres_adc;
	pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
	if (pressure_comp >= BME680_MAX_OVERFLOW_VAL)
		pressure_comp = ((pressure_comp / var1) << 1);
	else
		pressure_comp = ((pressure_comp << 1) / var1);
	var1 = ((int32_t)dev.calib.par_p9 * (int32_t)(((pressure_comp >> 3) *
		(pressure_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(pressure_comp >> 2) *
		(int32_t)dev.calib.par_p8) >> 13;
	var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
		(int32_t)(pressure_comp >> 8) *
		(int32_t)dev.calib.par_p10) >> 17;

	pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
		((int32_t)dev.calib.par_p7 << 7)) >> 4);

	return (uint32_t)pressure_comp;
}

uint32_t BARO_BME680::calc_humidity(uint16_t hum_adc)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t var6;
	int32_t temp_scaled;
	int32_t calc_hum;

	temp_scaled = (((int32_t) dev.calib.t_fine * 5) + 128) >> 8;
	var1 = (int32_t) (hum_adc - ((int32_t) ((int32_t) dev.calib.par_h1 * 16)))
		- (((temp_scaled * (int32_t) dev.calib.par_h3) / ((int32_t) 100)) >> 1);
	var2 = ((int32_t) dev.calib.par_h2
		* (((temp_scaled * (int32_t) dev.calib.par_h4) / ((int32_t) 100))
			+ (((temp_scaled * ((temp_scaled * (int32_t) dev.calib.par_h5) / ((int32_t) 100))) >> 6)
				/ ((int32_t) 100)) + (int32_t) (1 << 14))) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t) dev.calib.par_h6 << 7;
	var4 = ((var4) + ((temp_scaled * (int32_t) dev.calib.par_h7) / ((int32_t) 100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

	if (calc_hum > 100000) /* Cap at 100%rH */
		calc_hum = 100000;
	else if (calc_hum < 0)
		calc_hum = 0;

	return (uint32_t) calc_hum;
}

uint32_t BARO_BME680::calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range)
{
	#ifndef NO_GAS_MEASUREMENTS
		int64_t var1;
		uint64_t var2;
		int64_t var3;
		uint32_t calc_gas_res;
		/**Look up table 1 for the possible gas range values */
		uint32_t lookupTable1[16] = { UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
			UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
			UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
			UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647) };
		/**Look up table 2 for the possible gas range values */
		uint32_t lookupTable2[16] = { UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
			UINT32_C(255744255), UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016),
			UINT32_C(8000000), UINT32_C(4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000),
			UINT32_C(250000), UINT32_C(125000) };

		var1 = (int64_t) ((1340 + (5 * (int64_t) dev.calib.range_sw_err)) *
			((int64_t) lookupTable1[gas_range])) >> 16;
		var2 = (((int64_t) ((int64_t) gas_res_adc << 15) - (int64_t) (16777216)) + var1);
		var3 = (((int64_t) lookupTable2[gas_range] * (int64_t) var1) >> 9);
		calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);

		return calc_gas_res;
	#else
		return 0;
	#endif
}

uint8_t BARO_BME680::calc_heater_res(uint16_t temp)
{
	#ifndef NO_GAS_MEASUREMENTS
		uint8_t heatr_res;
		int32_t var1;
		int32_t var2;
		int32_t var3;
		int32_t var4;
		int32_t var5;
		int32_t heatr_res_x100;

		if (temp > 400) /* Cap temperature */
			temp = 400;

		var1 = (((int32_t) dev.amb_temp * dev.calib.par_gh3) / 1000) * 256;
		var2 = (dev.calib.par_gh1 + 784) * (((((dev.calib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
		var3 = var1 + (var2 / 2);
		var4 = (var3 / (dev.calib.res_heat_range + 4));
		var5 = (131 * dev.calib.res_heat_val) + 65536;
		heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
		heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

		return heatr_res;
	#else
		return 0;
	#endif
}

uint8_t BARO_BME680::calc_heater_dur(uint16_t dur)
{
	#ifndef NO_GAS_MEASUREMENTS
		uint8_t factor = 0;
		uint8_t durval;

		if (dur >= 0xfc0) {
			durval = 0xff; /* Max duration*/
		} else {
			while (dur > 0x3F) {
				dur = dur / 4;
				factor += 1;
			}
			durval = (uint8_t) (dur + (factor * 64));
		}

		return durval;
	#else
		return 0;
	#endif
}

int8_t BARO_BME680::read_field_data()
{
	int8_t rslt = BME680_OK;
	uint8_t buff[BME680_FIELD_LENGTH] = { 0 };
	uint8_t gas_range;
	uint32_t adc_temp;
	uint32_t adc_pres;
	uint16_t adc_hum;
	uint16_t adc_gas_res;
	uint8_t tries = 10;

	do {
		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(((uint8_t) (BME680_FIELD0_ADDR)), buff, (uint16_t) BME680_FIELD_LENGTH);

			data.status = buff[0] & BME680_NEW_DATA_MSK;
			//data.gas_index = buff[0] & BME680_GAS_INDEX_MSK;
			//data.meas_index = buff[1];

			/* read the raw data from the sensor */
			adc_pres = (uint32_t) (((uint32_t) buff[2] * 4096) | ((uint32_t) buff[3] * 16)
				| ((uint32_t) buff[4] / 16));
			adc_temp = (uint32_t) (((uint32_t) buff[5] * 4096) | ((uint32_t) buff[6] * 16)
				| ((uint32_t) buff[7] / 16));
			adc_hum = (uint16_t) (((uint32_t) buff[8] * 256) | (uint32_t) buff[9]);
			adc_gas_res = (uint16_t) ((uint32_t) buff[13] * 4 | (((uint32_t) buff[14]) / 64));
			gas_range = buff[14] & BME680_GAS_RANGE_MSK;

			data.status |= buff[14] & BME680_GASM_VALID_MSK;
			data.status |= buff[14] & BME680_HEAT_STAB_MSK;

			if (data.status & BME680_NEW_DATA_MSK) {
				data.temperature = calc_temperature(adc_temp);
				data.pressure = calc_pressure(adc_pres);
				data.humidity = calc_humidity(adc_hum);
				data.gas_resistance = calc_gas_resistance(adc_gas_res, gas_range);
				break;
			}
			/* Delay to poll the data */
			Timing::delay(BME680_POLL_PERIOD_MS);
		}
		tries--;
	} while (tries);

	if (!tries)
		rslt = BME680_W_NO_NEW_DATA;

	return rslt;
}



