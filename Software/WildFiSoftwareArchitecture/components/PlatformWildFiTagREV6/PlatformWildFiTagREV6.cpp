#include "PlatformWildFiTagREV6.h"

/** ----- STATIC FUNCTIONS & VARIABLES ----- */

// ESP NOW
static void dataSentESPNOW(const uint8_t *mac_addr, esp_now_send_status_t status);
static esp_now_sending_status_t espNowSendingStatus;
static uint8_t espNowBroadcastAddress[6] = {0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF};

// REST
static wifi_post_data_status_t wiFiPostDataStatus;
static uint16_t wiFiPostStreamBlocksSuccessfullyTransmitted;
static uint16_t wiFiPostStreamHalfBlocksSuccessfullyTransmitted;
static uint16_t wiFiPostReturnCode;
static void restPostTask(void *pvParameters);
static void restPostStreamFlashTaskHalfBlocks(void *pvParameters);
static void restPostStreamFlashTaskFullBlocks(void *pvParameters);
static TaskHandle_t RTOStaskHandle = NULL;

// WIFI
static bool wiFiScanRunning = false;	// needed for callback function, in order to call esp_wifi_connect or not
static bool wiFiScanDone = false;
static wifi_connect_status_t wiFiConnected = WIFI_CONNECT_NEVER_STARTED;
static void eventHandlerWiFi(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

// NTP
static bool ntpSuccess = false;
static bool ntpTimerFinished = false;

// BLE
static ble_mode_t BLEmode;
static uint8_t bleTXBuffer[BLE_BEACON_MAX_DATA]; 									// data that shall be advertised (as UUID)
static uint8_t ownAddressType; 														// BLE address type
static uint64_t myBLEAddress; 														// own BLE mac address
static void bleOnSync(); 															// executed once BLE driver is synced
static void bleAdvertise(); 														// advertise 16 bytes of data
static void bleScan(); 																// scanning of nearby BLE beacons
static int bleOnGAPEvent(struct ble_gap_event *event, void *arg); 					// found a beacon
//static uint64_t decodeBLEmacAddress(struct ble_gap_event *e);						// extract mac address from beacon
static void decodeBLEBeacon(struct ble_gap_event *event); 							// decode beacon and save to bleRXBuffer
static ble_msg_type_t checkIfBiologgerBeacon(uint8_t *dataBeacon, uint8_t len, uint16_t &biologgerId); 	// check if beacon content is a biologger participant (tag or reader phone etc.)

// BATTERY VOLTAGE CALC
static esp_adc_cal_characteristics_t *adc_chars; 									// characteristics for calibrated voltage measurement

/** ----- WAKE STUB VARIABLES ----- */

RTC_DATA_ATTR uint32_t adcValue = 0;
RTC_DATA_ATTR uint32_t adcCoeffA = 0;
RTC_DATA_ATTR uint32_t adcCoeffB = 0;
RTC_DATA_ATTR bool gpioHoldEnabled = false;

/** ----- CUSTOM PHY DATA ----- */

static RTC_DATA_ATTR esp_phy_calibration_data_t phyCalDataInRTC;

/** ----- MAIN STUFF ----- */

WildFiTagREV6::WildFiTagREV6() {
	wiFiInitialized = false;
	NVSinitialized = false;
	NVSForDataInitialized = false;
	BLEinitialized = false;
	ADCinitialized = false;
	pinsInitialized = false;
	powerPinInitialized = false;
	power2PinInitialized = false;
	myBLEAddress = 0;
	lastTime = 0;
	initPins();
}

WildFiTagREV6::~WildFiTagREV6() { }

bool WildFiTagREV6::selfTest(uint16_t voltageSupplied, uint32_t testBits) {
	bool isAlive = false;
	bool error = false;
	const uint16_t startDelay = 10;
	esp_task_wdt_init(120, false); // disable watchdog

	uint8_t myMac[6] = { 0 };
	esp_efuse_mac_get_default(myMac);
	printf("Selftest: MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);

	printf("Selftest: starting in %d seconds..\n", startDelay);
	delay(100); // finish printf
	shortLightSleep(startDelay * 1000);

	/** Voltage measurement */
	printf("Selftest: 01: voltage measurement..");
	if((testBits & SELFTEST_VOLTAGE) == 0) { printf("..SKIPPED\n"); }
	else {
		uint16_t voltageMeasured = readSupplyVoltage();
		uint16_t voltageOffset = 0;
		if(voltageMeasured > voltageSupplied) {
			voltageOffset = voltageMeasured - voltageSupplied;
			printf("..(too high by %dmV, %dmV)..", voltageOffset, voltageMeasured);
		}
		else {
			voltageOffset = voltageSupplied - voltageMeasured;
			printf("..too low by %dmV, %dmV)..", voltageOffset, voltageMeasured);		
		}
		if(voltageOffset >= 100) { printf("..FAILED (off too much)\n"); return false; }
		printf("..OKAY\n");
	}

	/** LEDs */
	printf("Selftest: 02: LEDs (green, red, both), please check..");
	if((testBits & SELFTEST_LEDS) == 0) { printf("..SKIPPED\n"); }
	else {
		delay(1000);
		ledGreenOn();
		delay(2000);
		ledGreenOff();
		ledRedOn();
		delay(2000);
		ledRedOff();
		ledGreenOn(); ledRedOn();
		delay(2000);
		ledGreenOff(); ledRedOff();
		printf("..OKAY\n");
	}

	/** Hall sensor */
	printf("Selftest: 03: hall sensor measurement..");
	if((testBits & SELFTEST_HALLSENSOR) == 0) { printf("..SKIPPED\n"); }
	else {
		int32_t hallValue = readHallSensor(1);
		if((hallValue) >= 80 || (hallValue <= -50)) { printf("..FAILED (out of bounds: %d)\n", hallValue); return false; }
		printf("..(%d)..OKAY\n", hallValue);
	}

	/** SENSOR POWER ON AND I2C INIT */
	if((testBits & SELFTEST_I2C) || (testBits & SELFTEST_RTC) || (testBits & SELFTEST_ACC_GYRO_FOC_CHECK) || (testBits & SELFTEST_BARO)) { // I2C related tests
		printf("Selftest: - sensor power on -\n");
		sensorPowerOn();
		i2c.begin(I2C_FREQ_HZ_400KHZ);
		delay(200);
	}

	/** I2C Ping */
	printf("Selftest: 04: I2C bus..");
	if((testBits & SELFTEST_I2C) == 0) { printf("..SKIPPED\n"); }
	else {
		isAlive = false;
		isAlive = i2c.isAlive(IMU_BMX160_ADDRESS);
		if(!isAlive) { printf("..FAILED (IMU)\n"); sensorPowerOff(); return false; }
		isAlive = false;
		isAlive = i2c.isAlive(BARO_BME680_ADDRESS);
		if(!isAlive) { printf("..FAILED (Baro)\n"); sensorPowerOff(); return false; }
		isAlive = false;
		isAlive = i2c.isAlive(RTC_RV8803C7_ADDRESS);
		if(!isAlive) { printf("..FAILED (RTC)\n"); sensorPowerOff(); return false; }
		printf("..OKAY\n");
	}

	/** RTC */
	printf("Selftest: 05: RTC set/get time..");
	if((testBits & SELFTEST_RTC) == 0) { printf("..SKIPPED\n"); }
	else {
		uint32_t expectedTimestamp = 1615213937;
		if(!rtc.set(14, 32, 17, 6, 8, 3, 2021)) { printf("..FAILED (set)\n"); sensorPowerOff(); return false; }
		uint32_t timestamp = rtc.getTimestamp(error);
		if(error) { printf("..FAILED (get)\n"); return false; }
		if(timestamp != expectedTimestamp) { printf("..FAILED (%d not equal %d)\n", timestamp, expectedTimestamp); sensorPowerOff(); return false; }
		delay(1200);
		timestamp = rtc.getTimestamp(error);
		if(timestamp != expectedTimestamp + 1) { printf("..FAILED (not +1)\n"); sensorPowerOff(); return false; }
		printf("..OKAY\n");
	}

	/** Acc and Gyro calibration */
	printf("Selftest: 06: ACC and GYRO FOC calibration done?..");
	if((testBits & SELFTEST_ACC_GYRO_FOC_CHECK) == 0) { printf("..SKIPPED\n"); }
	else {
		acc_config_t accConfigForFOC = { BMX160_ACCEL_ODR_25HZ, BMX160_ACCEL_BW_RES_AVG8, BMX160_ACCEL_RANGE_2G };
		gyro_config_t gyroConfigForFOC = { BMX160_GYRO_ODR_25HZ, BMX160_GYRO_RANGE_2000_DPS, BMX160_GYRO_BW_NORMAL_MODE };
		if(!imu.start(&accConfigForFOC, NULL, &gyroConfigForFOC, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("..FAILED (start)\n"); sensorPowerOff(); return false; }
		delay(100);
		bool doneAlready = imu.accGyroFOCAlreadyDoneAndStoredInNVM();
		bool doTheFOC = false;
		bool forceMode = false;
		if(((testBits & SELFTEST_ACC_GYRO_FOC_EXECUTE_IF_UNSET) != 0) && (!doneAlready)) {
			printf("NOT DONE and requested..");
			doTheFOC = true;
		}
		else if((testBits & SELFTEST_ACC_GYRO_FOC_FORCE_EXECUTION) != 0) {
			printf("FORCING EXECUTION (done: %d)..", doneAlready);
			doTheFOC = true;
			forceMode = true;
		}
		if(doTheFOC) {
			printf("-> PERFORMING IN 10 SECONDS -> DEVICE FLAT ON TABLE???..");
			delay(10000);
			if(!imu.startAccGyroFOCAndStoreInNVM(forceMode)) { printf("..TRIED BUT FAILED\n"); sensorPowerOff(); return false; }
			printf("..DONE!\n");
		}
		else {
			printf("..NOT doing it (done: %d, requested: %d, forced: %d)\n", doneAlready, ((testBits & SELFTEST_ACC_GYRO_FOC_EXECUTE_IF_UNSET) != 0), ((testBits & SELFTEST_ACC_GYRO_FOC_FORCE_EXECUTION) != 0));
		}
		if(!imu.stop()) { printf("..FAILED (stop)\n"); sensorPowerOff(); return false; }
	}

	/** Baro get temperature */
	printf("Selftest: 07: Baro get temperature..");
	if((testBits & SELFTEST_BARO) == 0) { printf("..SKIPPED\n"); }
	else {
		if(!baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) { printf("..FAILED (init)\n"); sensorPowerOff(); return false; }
        if(!baro.performMeasurement()) { printf("..FAILED (perform measurement)\n"); sensorPowerOff(); return false; }
		if(!baro.getResults()) { printf("..FAILED (get results)\n"); sensorPowerOff(); return false;  }
		int16_t temperature = baro.getTemperature(error);
		if(error) { printf("..FAILED (get temperature)\n"); sensorPowerOff(); return false;  }
		if((temperature < 1000) || (temperature > 3500)) { printf("..FAILED (temperature strange: %d)\n", temperature); sensorPowerOff(); return false;  }
		printf("..OKAY %d degree (/100)\n", temperature);
	}

	/** SENSOR POWER OFF */
	if((testBits & SELFTEST_I2C) || (testBits & SELFTEST_RTC) || (testBits & SELFTEST_ACC_GYRO_FOC_CHECK) || (testBits & SELFTEST_BARO)) { // I2C related tests
		printf("Selftest: - sensor power off -\n");
		sensorPowerOff(); // WARNING: keep on in case of adding BME680 data test!
	}

	/** FLASH POWER ON */
	if((testBits & SELFTEST_FLASH_BAD_BLOCKS) || (testBits & SELFTEST_FLASH_READ_WRITE) || (testBits & SELFTEST_FLASH_FULL_ERASE)) { // flash related tests
		printf("Selftest: - flash power on -\n");
		if(!flashPowerOn(true)) { printf("..FAILED (power on)\n"); return false; }
	}

	/** Flash bad blocks */
	printf("Selftest: 08: Flash bad blocks..");
	if((testBits & SELFTEST_FLASH_BAD_BLOCKS) == 0) { printf("..SKIPPED\n"); }
	else {
		printf("\n");
		bool badBlockResult = flash.selfTestBadBlocks();
		if(!badBlockResult) { printf("..FAILED (bad blocks found)\n"); flashPowerOff(true); return false; }
		delay(100);
		printf("..OKAY (no bad blocks)\n");
	}

	/** Flash read/write */
	printf("Selftest: 09: Flash write/read page 0 and 32..");
	if((testBits & SELFTEST_FLASH_READ_WRITE) == 0) { printf("..SKIPPED\n"); }
	else {
		uint8_t *testData = NULL;
		if(!flash.createBuffer(&testData, MT29_CACHE_SIZE)) { printf("..FAILED (buffer create)\n"); flashPowerOff(true); return false; } 
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) {
			testData[i] = i;
		}
		// ERASE
		if(!flash.erase(0)) { printf("..FAILED (erase block 0)\n"); flashPowerOff(true); return false; }
		// WRITE
		if(!flash.write(0, testData, MT29_CACHE_SIZE)) { printf("..FAILED (write page 0)\n"); flashPowerOff(true); return false; }
		if(!flash.write(32, testData, MT29_CACHE_SIZE)) { printf("..FAILED (write page 32)\n"); flashPowerOff(true); return false; }
		// READ 1 (0, 1, 2, 3, ..)
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) { testData[i] = 0; } // reset array
		if(!flash.read(0, 0, testData, MT29_CACHE_SIZE)) { printf("..FAILED (read block 0)\n"); flashPowerOff(true); return false; }
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) {
			uint8_t reference = i;
			if(testData[i] != reference) { printf("..FAILED (page 0: position %d: %d != %d)\n", i, testData[i], reference); flashPowerOff(true); return false; }
		}
		// READ 21 (FF, FF, FF, ..)
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) { testData[i] = 0; } // reset array
		if(!flash.read(1, 0, testData, MT29_CACHE_SIZE)) { printf("..FAILED (read block 1)\n"); flashPowerOff(true); return false; }
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) {
			if(testData[i] != 0xFF) { printf("..FAILED (page 1: position %d: %d != %d)\n", i, testData[i], 0xFF); flashPowerOff(true); return false; }
		}
		// READ 31 (0, 1, 2, 3, ..)
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) { testData[i] = 0; } // reset array
		if(!flash.read(32, 0, testData, MT29_CACHE_SIZE)) { printf("..FAILED (read block 32)\n"); flashPowerOff(true); return false; }
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) {
			uint8_t reference = i;
			if(testData[i] != reference) { printf("..FAILED (page 32: position %d: %d != %d)\n", i, testData[i], reference); flashPowerOff(true); return false; }
		}
		// ERASE AGAIN
		if(!flash.erase(0)) { printf("..FAILED (erase block 0 second time)\n"); flashPowerOff(true); return false; }
		// READ ALL 0xFF 1
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) { testData[i] = 0; } // reset array
		if(!flash.read(0, 0, testData, MT29_CACHE_SIZE)) { printf("..FAILED (read block 0)\n"); flashPowerOff(true); return false; }
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) {
			if(testData[i] != 0xFF) { printf("..FAILED (page 0: position %d: %d != %d)\n", i, testData[i], 0xFF); flashPowerOff(true); return false; }
		}
		// READ ALL 0xFF 2
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) { testData[i] = 0; } // reset array
		if(!flash.read(32, 0, testData, MT29_CACHE_SIZE)) { printf("..FAILED (read block 32)\n"); flashPowerOff(true); return false; }
		for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) {
			if(testData[i] != 0xFF) { printf("..FAILED (page 32: position %d: %d != %d)\n", i, testData[i], 0xFF); flashPowerOff(true); return false; }
		}
		printf("..OKAY\n");
	}

	/** Flash full erase */
	printf("Selftest: 10: Flash FULL ERASE (starts in 1s)..");
	if((testBits & SELFTEST_FLASH_FULL_ERASE) == 0) { printf("..SKIPPED\n"); }
	else {
		delay(1000);
		if(!flash.fullErase()) { printf("..FAILED\n"); flashPowerOff(true); return false; }
		printf("..DONE\n");
	}

	/** FLASH POWER OFF */
	if((testBits & SELFTEST_FLASH_BAD_BLOCKS) || (testBits & SELFTEST_FLASH_READ_WRITE) || (testBits & SELFTEST_FLASH_FULL_ERASE)) { // flash related tests
		printf("Selftest: - flash power off -\n");
		delay(200);
		if(!flashPowerOff(true)) { printf("ERROR FLASH OFF\n"); return false; } // DONE WITH FLASH HERE!
	}

	/** NVS full reset */
	printf("Selftest: 11: NVS full reset..");
	if((testBits & SELFTEST_NVS_RESET) == 0) { printf("..SKIPPED\n"); }
	else {
		if(!resetDataNVS()) { printf("..FAILED (reset)\n"); return false; }
    	if(!initDataNVS()) { printf("..FAILED (re-init)\n"); return false; }
		printf("..DONE\n");
	}

	/** Clock down */
	printf("Selftest: 12: CPU clock down to 10MHz..");
	if((testBits & SELFTEST_CPU_CLOCK_DOWN) == 0) { printf("..SKIPPED\n"); }
	else {
		setCPUSpeed(ESP32_10MHZ);
		printf("can you read me (10MHz)?..");
		delay(500);
		printf("can you read me (10MHz)?..");
		delay(500);
		setCPUSpeed(ESP32_80MHZ);
		printf("can you read me (80MHz)?..");
		delay(500);
		printf("can you read me (80MHz)?..\n");
	}

	/** WiFi scan @19.5dBm */
	printf("Selftest: 13: WiFi scan..");
	if((testBits & SELFTEST_WIFI_SCAN) == 0) { printf("..SKIPPED\n"); }
	else {
		if(!initWiFi()) { printf("..FAILED (init)\n"); disconnectAndStopWiFi(); return false; }
		if(!scanForWiFis(true, RADIO_MAX_TX_19_5_DBM, 60, WIFI_ALL_14_CHANNELS))  { printf("..FAILED (scan)\n"); disconnectAndStopWiFi(); return false; }
		int8_t txPwr = 0;
		esp_wifi_get_max_tx_power(&txPwr);
		if(txPwr != RADIO_MAX_TX_19_5_DBM) { printf("WARNING TX POWER WRONG (%d vs %d)..", txPwr, RADIO_MAX_TX_19_5_DBM); }
		printf("..DONE (tx power: %d)\n", txPwr);
		printWiFiScanResults();
		disconnectAndStopWiFi();
	}

	/** ESP NOW long range messages @19.5dBm */
	printf("Selftest: 14: ESP NOW..");
	if((testBits & SELFTEST_ESPNOW_BROADCAST) == 0) { printf("..SKIPPED\n"); }
	else {
		delay(500);
		if(!initESPNOWStationary(true, RADIO_MAX_TX_19_5_DBM, true, WIFI_PHY_RATE_1M_L)) { printf("..FAILED (init)\n"); stopESPNOW(); return false; }
		if(!addESPNOWBroadcastReceiverStationary()) { printf("..FAILED (add broadcast receiver)\n"); stopESPNOW(); return false; }
		uint8_t data[250] = { 0 };
		for(uint8_t i=0; i<250; i++) { data[i] = i; }
		for(uint8_t i=0; i<10; i++) {
			delay(100);
			if(!broadcastESPNOWData(&data[0], 250)) { printf("..FAILED (send broadcast)\n"); stopESPNOW(); return false; }
		}
		stopESPNOW();
		printf("..OKAY\n");
	}
	printf("Selftest: FINISHED WITHOUT PROBLEMS!\n");

	return true;
}

uint32_t WildFiTagREV6::measureTime(const char* text, bool dontPrint) {
	uint32_t duration;
    uint32_t timeNow = ((uint32_t) Timing::millis());
    lastTime = timeNow - lastTime;
	duration = lastTime;
	if(!dontPrint) {
    	printf("Time: %s took %d ms\n", text, lastTime);
	}
    lastTime = (uint32_t) Timing::millis();
	return duration;
}

/*bool WildFiTagREV6::customPhyInit() {
    const esp_phy_init_data_t* init_data = esp_phy_get_init_data();
    if(init_data == NULL) {
        return false;
    }
    if(esp_reset_reason() != ESP_RST_DEEPSLEEP) { // full calibration on reset and all other wake up reasons
        if(esp_phy_rf_init(init_data, PHY_RF_CAL_FULL, &phyCalDataInRTC, PHY_WIFI_MODULE) != ESP_OK) {
            return false;
        }
    }
    else { // no calibration on deep sleep wakeup, just set phy data
        if(esp_phy_rf_init(init_data, PHY_RF_CAL_NONE, &phyCalDataInRTC, PHY_WIFI_MODULE) != ESP_OK) {
            return false;
        }
    }
	// DO NOT FULLY UNDERSTAND THIS: deinit is called, but phyCalDataInRTC will be used after this, esp now init takes way less time
	if(esp_phy_rf_deinit(PHY_WIFI_MODULE) != ESP_OK) { // otherwise +1.4mA in deep sleep
		return false;
	}
	//adc_power_off(); // alternative to deinit, but I think it's rather strange
    return true;
}*/

bool WildFiTagREV6::fullRFCalibration() {
    // takes 145 - 160ms
    const esp_phy_init_data_t* init_data = esp_phy_get_init_data();
    if(init_data == NULL) {
        return false;
    }
    if(esp_phy_rf_init(init_data, PHY_RF_CAL_FULL, &phyCalDataInRTC, PHY_WIFI_MODULE) != ESP_OK) {
        return false;
    }
	// DO NOT FULLY UNDERSTAND THIS: deinit is called, but phyCalDataInRTC will be used after this, esp now init takes way less time
	if(esp_phy_rf_deinit(PHY_WIFI_MODULE) != ESP_OK) { // otherwise +1.4mA in deep sleep
		return false;
	}
	//adc_power_off(); // alternative to deinit, but I think it's rather strange
    return true;    
}

bool WildFiTagREV6::onlyLoadRFCalibration() {
    // takes 5ms
    const esp_phy_init_data_t* init_data = esp_phy_get_init_data();
    if(init_data == NULL) {
        return false;
    }
    if(esp_phy_rf_init(init_data, PHY_RF_CAL_NONE, &phyCalDataInRTC, PHY_WIFI_MODULE) != ESP_OK) {
        return false;
    }
	// DO NOT FULLY UNDERSTAND THIS: deinit is called, but phyCalDataInRTC will be used after this, esp now init takes way less time
	if(esp_phy_rf_deinit(PHY_WIFI_MODULE) != ESP_OK) { // otherwise +1.4mA in deep sleep
		return false;
	}
	//adc_power_off(); // alternative to deinit, but I think it's rather strange
    return true;       
}

esp_reset_reason_t WildFiTagREV6::getLastResetReason() {
	return esp_reset_reason();
}

bool WildFiTagREV6::enableDynamicFrequencyScaling() {
	esp_pm_config_esp32_t pmConfig = {
		.max_freq_mhz = 80,
		.min_freq_mhz = 10,
		.light_sleep_enable = true
	};
	if(esp_pm_configure(&pmConfig) != ESP_OK) { return false; };
	return true;
}

void WildFiTagREV6::initPins() {
	// default pin config after boot: gpio function, enable pullup, disable input and output (open drain with output = 1? then totally disconnected from circuit)
	gpio_config_t io_conf;
	
	if(pinsInitialized) { // do not initialize again
		return;
	}
	
	/** Configure OUTPUT pins */
	const uint64_t outputPins = ( (1ULL<<PIN_LED_RED) | (1ULL<<PIN_LED_GREEN) ); 
    io_conf.intr_type = GPIO_INTR_DISABLE; // no interrupt on OUTPUT
    io_conf.mode = GPIO_MODE_OUTPUT; // set as output
    io_conf.pin_bit_mask = outputPins; // use bitmask
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE ; // disable pull-down
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE ; // disable pull-up
    gpio_config(&io_conf); // configure
	
	/** Configure INPUT pins */
	rtc_gpio_pullup_en(PIN_RTC_INT); // enable pull-up for RTC interrupt pin
	
	/** Initialize ADC for V_BATT measurements (EVERY time because adc_chars is not in RTC memory, only adcCoeffA and adcCoeffB) */
	adc1_config_width(ADC_WIDTH_BIT_12); // 12 bit width
	adc1_config_channel_atten(ADC_CHANNEL_BATT_VOLT, ADC_ATTEN_11db); // 11db attenuation for full-scale measurements up to 3.9V
	adc_chars = (esp_adc_cal_characteristics_t *) calloc(1, sizeof(esp_adc_cal_characteristics_t)); // get characteristics for calibrated voltage measurements
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars); // get characteristics for calibrated voltage measurements 
	adcCoeffA = adc_chars->coeff_a; // for wake stub ADC voltage calc
	adcCoeffB = adc_chars->coeff_b; // for wake stub ADC voltage calc
	ADCinitialized = true;
    
	pinsInitialized = true;
	
	// PIN_BATT_VOLT does not need initialization
	// PIN_WAKEUP does not need initialization
	// PIN_POWER not initialized as output pin because consumes ~1mA if set to 0 (leakage through I2C pull-ups, assume by BMX160)
}

bool WildFiTagREV6::flashPowerOn(bool withDelay) {
	if(!power2PinInitialized) {
		gpio_set_drive_capability(PIN_POWER2, GPIO_DRIVE_CAP_3); // important, otherwise no access to SPI memory sometimes
        gpio_config_t io_conf;
        const uint64_t outputPins = ( (1ULL<<PIN_POWER2) | (1ULL<<PIN_SPI_CS) ); 
		io_conf.intr_type = GPIO_INTR_DISABLE; // no interrupt on OUTPUT
		io_conf.mode = GPIO_MODE_OUTPUT; // set as output
		io_conf.pin_bit_mask = outputPins; // use bitmask
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE ; // disable pull-down
		io_conf.pull_up_en = GPIO_PULLUP_DISABLE ; // disable pull-up
		gpio_config(&io_conf); // configure
		power2PinInitialized = true;
	}
	// IMPORTANT: CS shall follow VCC of SPI flash memory (CS = 1 = SELECTED)
	gpio_set_level(PIN_POWER2, 1);
	gpio_set_level(PIN_SPI_CS, 1);
	
	if(withDelay) {
		delay(10); // according to datasheet of MT29: at least 1.25ms shall elapse before doing stuff
	}
	bool res = spi.init(); // init SPI for MT29
	return res;
}

bool WildFiTagREV6::flashPowerOff(bool withDelay) {
	bool ret = true;
	ret = spi.deinit(); // DO NOT CALL, increases sleep current by +300uA -> only if not all SPI pins below are set to be disabled!
	if(withDelay) {
		delay(10); // give flash a bit time before shutting down
	}
	gpio_config_t io_conf;
    const uint64_t pins = ( (1ULL<<PIN_POWER2) | (1ULL<<PIN_SPI_CS) | (1ULL<<PIN_SPI_SCLK) | (1ULL<<PIN_SPI_MISO) | (1ULL<<PIN_SPI_MOSI) ); // leakage if SCLK, MISO, MOSI not disabled as well
	io_conf.intr_type = GPIO_INTR_DISABLE; // no interrupt
	io_conf.mode = GPIO_MODE_OUTPUT; // NEW: set to output (fully disable before)
	io_conf.pin_bit_mask = pins; // use bitmask
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pull-down
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // disable pull-up -> IMPORTANT, otherwise +5uA leakage @PIN_POWER2
	gpio_config(&io_conf); // configure

	// NEW: set output to 0 otherwise some strange power consumption for 200ms immediately when entering deep sleep
	gpio_set_level(PIN_POWER2, 0);
	gpio_set_level(PIN_SPI_CS, 0);
    gpio_set_level(PIN_SPI_SCLK, 0);
    gpio_set_level(PIN_SPI_MISO, 0);
    gpio_set_level(PIN_SPI_MOSI, 0);

	power2PinInitialized = false;
	return ret;
}

void WildFiTagREV6::sensorPowerOn() {
	if(gpioHoldEnabled) { gpio_hold_dis(PIN_POWER); } // when keepSensorPowerOnInDeepSleep was called

	gpio_config_t io_conf;
	if(!powerPinInitialized) {
		// set PIN_POWER as output right BEFORE powering
		const uint64_t outputPins = ( (1ULL<<PIN_POWER) ); 
		io_conf.intr_type = GPIO_INTR_DISABLE; // no interrupt on OUTPUT
		io_conf.mode = GPIO_MODE_OUTPUT; // set as output
		io_conf.pin_bit_mask = outputPins; // use bitmask
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE ; // disable pull-down
		io_conf.pull_up_en = GPIO_PULLUP_DISABLE ; // disable pull-up
		gpio_config(&io_conf); // configure
		powerPinInitialized = true;
	}
	gpio_set_level(PIN_POWER, 1); // power it up!
	if(gpioHoldEnabled) { gpio_hold_en(PIN_POWER); } // when keepSensorPowerOnInDeepSleep was called
}

void WildFiTagREV6::sensorPowerOff() {
	if(gpioHoldEnabled) { gpio_hold_dis(PIN_POWER); } // when keepSensorPowerOnInDeepSleep was called, otherwise set level has no effect

	gpio_set_level(PIN_POWER, 0); // turn imu off

	// set PIN_POWER as input
	gpio_config_t io_conf;
	const uint64_t outputPins = ( (1ULL<<PIN_POWER) ); 
	io_conf.intr_type = GPIO_INTR_DISABLE; // no interrupt on OUTPUT
	io_conf.mode = GPIO_MODE_INPUT; // set as input
	io_conf.pin_bit_mask = outputPins; // use bitmask
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE ; // disable pull-down
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE ; // disable pull-up
	gpio_config(&io_conf); // configure
	powerPinInitialized = false;

	if(gpioHoldEnabled) { gpio_hold_en(PIN_POWER); } // when keepSensorPowerOnInDeepSleep was called
}

void WildFiTagREV6::keepSensorPowerOnInDeepSleep() {
	gpio_hold_en(PIN_POWER); // EXTREMELY IMPORTANT! one night of bug search
	//gpio_hold_en(PIN_POWER2); // otherwise +5uA in deep sleep
	gpio_deep_sleep_hold_en();
	gpio_hold_en(PIN_LED_RED); // CAREFUL: PIN_LED_RED is also GPIO only (not in RTC) and configured in initPins as OUTPUT -> Wake stub refuses boot, then leakage of +140uA through PIN_LED_RED during deep sleep
	gpioHoldEnabled = true;
}

void WildFiTagREV6::delay(uint16_t d) {
	Timing::delay(d);
}

void WildFiTagREV6::preciseDelay(uint32_t us) {
	ets_delay_us(us);
}

void WildFiTagREV6::deepSleep(bool leavePowerOn) {
    // all off = 9uA, all on = 10.7uA
	if(wiFiInitialized) {
		esp_wifi_stop();
		wiFiInitialized = false;
		wiFiConnected = WIFI_CONNECT_NEVER_STARTED;
	}
	stopESPNOW(); // in case if started
	
	esp_deep_sleep_disable_rom_logging(); // no more boot messages -> saves couple of ms
	
	//rtc_gpio_isolate(GPIO_NUM_12); // NO, increase by 40uA!
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); // +1uA, internal pull-ups and pull-downs and ULP and RTC gpio settings
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON); // +0.4uA, RTC_DATA_ATTR
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON); // +0.3uA, wake stub code, RTC_IRAM_ATTR
    esp_deep_sleep_start(); 
}

void WildFiTagREV6::lightSleep() {
	// not turning off wifi?
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); // +1uA, internal pull-ups and pull-downs and ULP
	esp_light_sleep_start();
}

void WildFiTagREV6::shortLightSleep(uint32_t millseconds) {
	uint64_t sleepTimeInUs = millseconds; // IMPORTANT: had overflow before due to not using ULL
	sleepTimeInUs *= 1000ULL;
	esp_sleep_enable_timer_wakeup(sleepTimeInUs);
	lightSleep();
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER); // IMPORTANT: disable after wake up, otherwise the wakeup source stays active!
}


uint32_t WildFiTagREV6::readSupplyVoltageFromADC() {
	if(!ADCinitialized) { // requires initPins before read
		return 0;
	}
	// measure
    uint32_t raw = adc1_get_raw(ADC_CHANNEL_BATT_VOLT); // after this call ADC is ON (~1.6mA)
	// disable ADC again
	adc_power_off();
	// get calibrated value (actual voltage)
    uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, adc_chars);
	return voltage;	
}

uint32_t WildFiTagREV6::readSupplyVoltageFromWakeStub() {
	return adcValue;
}

uint16_t WildFiTagREV6::readSupplyVoltage() {
	uint32_t voltage;
	if(!ADCinitialized) { // requires initPins before read
		return 0;
	}
	voltage = readSupplyVoltageFromADC();
	voltage = (voltage * (BATT_VOLT_RES_1 + BATT_VOLT_RES_2)) / BATT_VOLT_RES_2; // converting measured voltage to battery voltage (voltage divider), e.g. (3300 * (100+47)) / 100 = 4851 = 4.81V
	return (uint16_t) voltage;
}

int32_t WildFiTagREV6::readHallSensor(uint16_t iterations) {
	adc1_config_width(ADC_WIDTH_BIT_12); // 12 bit width
    int32_t val = 0;
	if(iterations == 0) { return 0; }
	for(uint16_t i=0; i<iterations; i++) {
		val += hall_sensor_read();
	}
	adc_power_off();
	val = val / iterations;
	return val;
}

void WildFiTagREV6::setCPUSpeed(uint8_t speed) {
	setCpuFrequencyMhz(speed);
	uart_set_baudrate(UART_NUM_0, ESP32_SERIAL_BAUD_RATE); // to make printf work again
}

bool WildFiTagREV6::initSpiffs(uint8_t maxFiles) {
    esp_vfs_spiffs_conf_t conf = {
    	.base_path = "/spiffs",
    	.partition_label = NULL,
    	.max_files = maxFiles,
    	.format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf); // 30ms
    if(ret != ESP_OK) {
        if(ret == ESP_FAIL) {
            //printf("Failed to mount or format filesystem\n");
        }
        else if(ret == ESP_ERR_NOT_FOUND) {
            //printf("Failed to find SPIFFS partition\n");
        }
        else {
            //printf("Failed to initialize SPIFFS (%s)\n", esp_err_to_name(ret));
        }
        return false;
    }
    return true;
}

bool WildFiTagREV6::printSpiffs(const char* fileName) {
	uint16_t cntLines = 0;
    FILE* f = fopen(fileName, "r");
    if(f == NULL) {
        //printf("Failed to open file for reading (not existing?)\n");
        return false;
    }
    for(char buffer[256], *line; (line = fgets(buffer, sizeof(buffer), f));) {
        char* pos = strchr(line, '\n');
        if(pos) { *pos = '\0'; }
        printf("%s\n", line);
        cntLines++;
    }
    fclose(f);
	return true;
}

void WildFiTagREV6::resetSpiffs(const char* fileName) {
    struct stat st;
    if(stat(fileName, &st) == 0) { // delete it if it exists
        unlink(fileName);
    }
}

uint32_t WildFiTagREV6::getFreeSpaceSpiffs() {
    uint32_t total = 0, used = 0;
    if(esp_spiffs_info(NULL, &total, &used) != ESP_OK) {
        return 0;
    }
	return (total - used);
}

void WildFiTagREV6::wasteOn() {
	ledGreenOn();
	ledRedOn();
}

void WildFiTagREV6::wasteOff() {
	ledGreenOff();
	ledRedOff();
}

void WildFiTagREV6::ledRedOn() {
	if(gpioHoldEnabled) { gpio_hold_dis(PIN_LED_RED); } // when keepSensorPowerOnInDeepSleep was called
	gpio_set_level(PIN_LED_RED, 1);
	if(gpioHoldEnabled) { gpio_hold_en(PIN_LED_RED); } // when keepSensorPowerOnInDeepSleep was called
}

void WildFiTagREV6::ledRedOff() {
	if(gpioHoldEnabled) { gpio_hold_dis(PIN_LED_RED); } // when keepSensorPowerOnInDeepSleep was called
	gpio_set_level(PIN_LED_RED, 0);
	if(gpioHoldEnabled) { gpio_hold_en(PIN_LED_RED); } // when keepSensorPowerOnInDeepSleep was called
}

void WildFiTagREV6::ledGreenOn() {
	gpio_set_level(PIN_LED_GREEN, 1);
}

void WildFiTagREV6::ledGreenOff() {
	gpio_set_level(PIN_LED_GREEN, 0);
}

void WildFiTagREV6::blinkTimes(uint8_t howOften, blink_t color, bool addDelayAtEnd) {
	if((howOften == 0) || (color == B_NONE)) { return; }
	ledRedOff();
	ledGreenOff();
	for(uint8_t i=0; i<howOften; i++) {
		if(color == B_RED) { ledRedOn(); }
		else if(color == B_GREEN) { ledGreenOn(); }
		else if(color == B_BOTH) { ledGreenOn(); ledRedOn(); }
		delay(BLINKY_ON_DELAY);
		if(color == B_RED) { ledRedOff(); }
		else if(color == B_GREEN) { ledGreenOff(); }
		else if(color == B_BOTH) { ledGreenOff(); ledRedOff(); }
		if(addDelayAtEnd) { delay(BLINKY_PAUSE_DELAY); }
		else {
			if(i < (howOften - 1)) { delay(BLINKY_PAUSE_DELAY); } // last iteration -> do not add delay
		}
	}
}

void WildFiTagREV6::blink(blink_t l1, blink_t l2, blink_t l3) {
	// off
	ledRedOff();
	ledGreenOff();
	// first blink
	if(l1 == B_RED) {
		ledRedOn();
	}
	else if(l1 == B_GREEN) {
		ledGreenOn();
	}
	else if(l1 == B_BOTH) {
		ledGreenOn();
		ledRedOn();
	}
	delay(BLINKY_ON_DELAY);
	// off
	ledRedOff();
	ledGreenOff();
	if(l2 != B_NONE) {
		delay(BLINKY_PAUSE_DELAY);
		// second blink
		if(l2 == B_RED) {
			ledRedOn();
		}
		else if(l2 == B_GREEN) {
			ledGreenOn();
		}
		else if(l2 == B_BOTH) {
			ledGreenOn();
			ledRedOn();
		}
		delay(BLINKY_ON_DELAY);
		// off
		ledRedOff();
		ledGreenOff();
	}
	
	if(l3 != B_NONE) {
		delay(BLINKY_PAUSE_DELAY);
		// third blink
		if(l3 == B_RED) {
			ledRedOn();
		}
		else if(l3 == B_GREEN) {
			ledGreenOn();
		}
		else if(l3 == B_BOTH) {
			ledGreenOn();
			ledRedOn();
		}
		delay(BLINKY_ON_DELAY);
		// off
		ledRedOff();
		ledGreenOff();
	}
}

wake_up_reason_t WildFiTagREV6::getWakeUpReason() {
	/*
	ESP_SLEEP_WAKEUP_UNDEFINED    //!< (NORMAL RESET or other, e.g. brownout) In case of deep sleep, reset was not caused by exit from deep sleep
    ESP_SLEEP_WAKEUP_ALL          //!< Not a wakeup cause, used to disable all wakeup sources with esp_sleep_disable_wakeup_source
    ESP_SLEEP_WAKEUP_EXT0         //!< Wakeup caused by external signal using RTC_IO
    ESP_SLEEP_WAKEUP_EXT1         //!< Wakeup caused by external signal using RTC_CNTL
    ESP_SLEEP_WAKEUP_TIMER        //!< Wakeup caused by timer
    ESP_SLEEP_WAKEUP_TOUCHPAD     //!< Wakeup caused by touchpad
    ESP_SLEEP_WAKEUP_ULP          //!< Wakeup caused by ULP program
    ESP_SLEEP_WAKEUP_GPIO         //!< Wakeup caused by GPIO (light sleep only)
    ESP_SLEEP_WAKEUP_UART         //!< Wakeup caused by UART (light sleep only)
	*/
	esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
	
	if(reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
		return BY_RESET;
	}
	else if(reason == ESP_SLEEP_WAKEUP_EXT0) {
		return BY_EXT0;
	}
	else if(reason == ESP_SLEEP_WAKEUP_EXT1) {
		return BY_EXT1;
	}
	else if(reason == ESP_SLEEP_WAKEUP_TIMER) {
		return BY_TIMER;
	}
	else return BY_RESET;
}

void WildFiTagREV6::enableInternalTimerInterruptInDeepSleep(uint32_t seconds) {
	// max. 400 days
	uint64_t sleepTimeInUs = seconds; // IMPORTANT: had overflow before due to not using ULL
	sleepTimeInUs *= 1000000ULL;
	esp_sleep_enable_timer_wakeup(sleepTimeInUs);
}

void WildFiTagREV6::enableWakeUpPinInDeepSleep() {
	esp_sleep_enable_ext1_wakeup(PIN_WAKEUP_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW); // EXT1 requires bitmask of pins
}

void WildFiTagREV6::enableAccInterruptInDeepSleep() {
	esp_sleep_enable_ext1_wakeup(PIN_ACC_INT_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); // EXT1 requires bitmask of pins
}

void WildFiTagREV6::enableRTCInterruptInDeepSleep() {
	esp_sleep_enable_ext0_wakeup(PIN_RTC_INT, 0); // EXT0 (only one pin)
}

void WildFiTagREV6::enableUart2InterruptInLightSleep() {
	// unfortunately "real" UART interrupt does not work, means first characters are bullshit
	/*PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA2_U, FUNC_SD_DATA2_U1RXD); // GPIO9 should be configured as function_5
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA3_U, FUNC_SD_DATA3_U1TXD);
	if(uart_set_wakeup_threshold(UART2_PORT_NUMBER, 3) != ESP_OK) { printf("ERROR10\n"); }
	if(esp_sleep_enable_uart_wakeup(UART2_PORT_NUMBER) != ESP_OK) { printf("ERROR11\n"); }*/
    gpio_wakeup_enable(PIN_RXD2, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
}

void WildFiTagREV6::printTxPower() {
    int8_t txPwr = 0;
	esp_wifi_get_max_tx_power(&txPwr);
	printf("TX POWER: %d\n", txPwr);
}

void WildFiTagREV6::gpioBOn() {
	if(!rtc_gpio_is_valid_gpio(PIN_GPIO_B)) { return; }
    rtc_gpio_init(PIN_GPIO_B);
    rtc_gpio_set_direction(PIN_GPIO_B, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(PIN_GPIO_B, HIGH);
}

void WildFiTagREV6::gpioBOff() {
	if(!rtc_gpio_is_valid_gpio(PIN_GPIO_B)) { return; }
    //rtc_gpio_init(PIN_GPIO_B);
    //rtc_gpio_set_direction(PIN_GPIO_B, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(PIN_GPIO_B, LOW);
}

QueueHandle_t* WildFiTagREV6::uart2GetQueue() {
	return &uart2Queue;
}

bool WildFiTagREV6::uart2Init(uint32_t baudRate) {
	uart_config_t uart_config = { };
	uart_config.baud_rate = baudRate;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uart_config.source_clk = UART_SCLK_REF_TICK; // UART_SCLK_REF_TICK or UART_SCLK_APB
	if(uart_driver_install(UART2_PORT_NUMBER, UART2_RX_BUFFER, 0, UART2_EVENT_QUEUE_SIZE, &uart2Queue, 0) != ESP_OK) { return false; };
	if(uart_param_config(UART2_PORT_NUMBER, &uart_config) != ESP_OK) { return false; };
	if(uart_set_pin(UART2_PORT_NUMBER, PIN_TXD2, PIN_RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) { return false; };
	return true;
}

void WildFiTagREV6::uart2UpdateBaudrate(uint32_t baudRate) {
	uart_set_baudrate(UART2_PORT_NUMBER, baudRate);
	uart_flush(UART2_PORT_NUMBER);
}

void WildFiTagREV6::uart2EnablePatternInterrupt(char pattern) {
	uart_enable_pattern_det_baud_intr(UART2_PORT_NUMBER, pattern, 1, 9, 0, 0);
	uart_pattern_queue_reset(UART2_PORT_NUMBER, UART2_EVENT_QUEUE_SIZE);
	uart_flush(UART2_PORT_NUMBER);
}

void WildFiTagREV6::uart2InterruptPrint(uint16_t stopAfter) {
	uint16_t listenIteration = 0;
	uart_event_t event;
	size_t bufferedSize;
	uint8_t *uart2Data = (uint8_t*) malloc(UART2_RX_BUFFER);
	while(true) {
		if(xQueueReceive(uart2Queue, (void * )&event, (portTickType) portMAX_DELAY)) {
			bzero(uart2Data, UART2_RX_BUFFER);
			if(event.type == UART_PATTERN_DET) {
				uart_get_buffered_data_len(UART2_PORT_NUMBER, &bufferedSize);
				int pos = uart_pattern_pop_pos(UART2_PORT_NUMBER);
				printf("[UART PATTERN DETECTED] pos: %d, buffer len: %d\n", pos, bufferedSize);
				if(pos == -1) { // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not record the position. We should set a larger queue size, as an example, we directly flush the rx buffer here
					printf("ERROR TOO SLOW\n");
					uart_flush_input(UART2_PORT_NUMBER);
				} else {
					int read_len = uart_read_bytes(UART2_PORT_NUMBER, uart2Data, pos + 1, 100 / portTICK_PERIOD_MS); // pos+1 to also read the pattern (\n)
					uart2Data[read_len] = '\0'; // make sure the line is a standard string
					//if(pos > 0) { uart2Data[pos-1] = 0; } // terminate string more early because \r sucks
					//uint8_t pattern = 0;
					//uart_read_bytes(UART2_PORT_NUMBER, &pattern, 1, 100 / portTICK_PERIOD_MS);
					printf("%d Read (strlen: %d): %s \n", (uint32_t)(Timing::millis()), strlen((char *) uart2Data), uart2Data);
				}
				listenIteration++;
				if(listenIteration >= stopAfter) { break; }
			}
		}
	}
}

/** ----- NVS ----- */

bool WildFiTagREV6::initNVS() {
	if(NVSinitialized == false) {
		esp_err_t ret = nvs_flash_init();
		if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
			if(nvs_flash_erase() != ESP_OK) {
				return false;
			}
			ret = nvs_flash_init();
		}
		if(ret != ESP_OK) {
			return false;
		}
		NVSinitialized = true;
	}
	return true;
}

bool WildFiTagREV6::initDataNVS() {
	if(NVSForDataInitialized == false) {
		esp_err_t ret = nvs_flash_init_partition(NVS_DATA_PARTITION);
		if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
			if(nvs_flash_erase_partition(NVS_DATA_PARTITION) != ESP_OK) {
				return false;
			}
			ret = nvs_flash_init_partition(NVS_DATA_PARTITION);
		}
		if(ret != ESP_OK) {
			return false;
		}
		NVSForDataInitialized = true;
	}
	return true;
}

bool WildFiTagREV6::resetDataNVS() {
	if(nvs_flash_erase_partition(NVS_DATA_PARTITION) != ESP_OK) {
		return false;
	}
	return true;
}

void WildFiTagREV6::printDataNVSStats() {
	if(!NVSForDataInitialized) {
		return;
	}
	nvs_stats_t nvs_stats;
    nvs_get_stats(NVS_DATA_PARTITION, &nvs_stats);
    printf("NVS Data: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)\n", nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
}

uint8_t WildFiTagREV6::nvsReadUINT8(const char *key) {
	esp_err_t err;
	uint8_t value = 0; // default 0 if not existing
	if(!NVSForDataInitialized) {
		return 0;
	}
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return 0;
    }
	// read
    err = nvs_get_u8(handle, key, &value);
	nvs_close(handle);
    switch (err) {
        case ESP_OK:
			return value;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            // value not initialized
			return 0;
            break;
        default :
            return 0;
    }
	return 0;
}

uint16_t WildFiTagREV6::nvsReadUINT16(const char *key) {
	esp_err_t err;
	uint16_t value = 0; // default 0 if not existing
	if(!NVSForDataInitialized) {
		return 0;
	}
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return 0;
    }
	// read
    err = nvs_get_u16(handle, key, &value);
	nvs_close(handle);
    switch (err) {
        case ESP_OK:
			return value;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            // value not initialized
			return 0;
            break;
        default :
            return 0;
    }
	return 0;
}

uint32_t WildFiTagREV6::nvsReadUINT32(const char *key) {
	esp_err_t err;
	uint32_t value = 0; // default 0 if not existing
	if(!NVSForDataInitialized) {
		return 0;
	}
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return 0;
    }
	// read
    err = nvs_get_u32(handle, key, &value);
	nvs_close(handle);
    switch (err) {
        case ESP_OK:
			return value;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            // value not initialized
			return 0;
            break;
        default :
            return 0;
    }
	return 0;
}

bool WildFiTagREV6::nvsWriteUINT8(const char *key, uint8_t val) {
	esp_err_t err;
	if(!NVSForDataInitialized) {
		return false;
	}
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return false;
    }
	// write
	err = nvs_set_u8(handle, key, val); // 6-7ms (!)
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    err = nvs_commit(handle);
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    nvs_close(handle);
	return true;
}

bool WildFiTagREV6::nvsWriteUINT16(const char *key, uint16_t val) {
	esp_err_t err;
	if(!NVSForDataInitialized) {
		return false;
	}
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return false;
    }
	// write
	err = nvs_set_u16(handle, key, val); // 6-7ms (!)
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    err = nvs_commit(handle);
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    nvs_close(handle);
	return true;
}

bool WildFiTagREV6::nvsWriteUINT32(const char *key, uint32_t val) {
	esp_err_t err;
	if(!NVSForDataInitialized) {
		return false;
	}
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return false;
    }
	// write
	err = nvs_set_u32(handle, key, val); // 6-7ms (!)
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    err = nvs_commit(handle);
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    nvs_close(handle);
	return true;
}

bool WildFiTagREV6::nvsWriteUINT32x2(const char *key1, uint32_t val1, const char *key2, uint32_t val2) {
	esp_err_t err;
	if(!NVSForDataInitialized) {
		return false;
	}
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return false;
    }
	// write 1
	err = nvs_set_u32(handle, key1, val1); // 6-7ms (!)
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
	// write 2
	err = nvs_set_u32(handle, key2, val2); // 6-7ms (!)
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
	// commit
    err = nvs_commit(handle);
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    nvs_close(handle);
	return true;
}

bool WildFiTagREV6::nvsWriteUINT32andUINT16(const char *key1, uint32_t val1, const char *key2, uint16_t val2) {
	esp_err_t err;
	if(!NVSForDataInitialized) {
		return false;
	}
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return false;
    }
	// write 1
	err = nvs_set_u32(handle, key1, val1); // 6-7ms (!)
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
	// write 2
	err = nvs_set_u16(handle, key2, val2); // 6-7ms (!)
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
	// commit
    err = nvs_commit(handle);
    if(err != ESP_OK) {
		nvs_close(handle);
		return false;
	}
    nvs_close(handle);
	return true;
}

uint32_t WildFiTagREV6::nvsReadThenPlusOneUINT32(const char *key) {
	esp_err_t err;
	uint32_t value = 0;
	if(!NVSForDataInitialized) {
		return false;
	}
	// open
    nvs_handle_t handle;
    err = nvs_open_from_partition(NVS_DATA_PARTITION, "storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return 0;
    }
	// read
    err = nvs_get_u32(handle, key, &value);
	if(err != ESP_OK) { // for example not initalized
		value = 0;
	}
	// increment
	value++;
	// write
	err = nvs_set_u32(handle, key, value);
    if(err != ESP_OK) {
		nvs_close(handle);
		return 0;
	}
    err = nvs_commit(handle);
    if(err != ESP_OK) {
		nvs_close(handle);
		return 0;
	}
    nvs_close(handle);

	return (value-1); // previous value
}

/** ----- ESP NOW ----- */

bool WildFiTagREV6::initESPNOWStationary(bool longrange, int8_t maxTxPower, bool withoutNVS, wifi_phy_rate_t phyRate) {
	esp_log_level_set("ESPNOW", ESP_LOG_NONE); // saves 1ms
	// init NVS (in case not initialized) -> I THINK NOT NEEDED, UNTESTED! -> NO, is needed for PHY calibration in NVS -> NO, works if RF is calibrated before
	if(!withoutNVS) {
		if(!initNVS()) { // 90ms
			return false;
		}
	}
    // init wifi
    esp_netif_init(); // 1ms, CHANGED in ESP IDF 4.1: was tcpip_adapter_init();
    if(esp_event_loop_create_default() != ESP_OK) { // 1ms
		return false;
	}
	esp_netif_create_default_wifi_sta(); // 1ms
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	if(phyRate != WIFI_PHY_RATE_1M_L) {
		cfg.ampdu_tx_enable = 0;
	}
	if(withoutNVS) {
		cfg.nvs_enable = false; // overrides CONFIG_ESP32_WIFI_NVS_ENABLED
	}
	if(esp_wifi_init(&cfg) != ESP_OK) { // 58ms, fails if NVS not initialized
		return false;
	}
	if(esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) { // 0ms, different from normal wifi initialization, default is flash (reduces flash lifetime)
		return false;
	}
	if(esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) { // 0ms, ACCESS POINT, not STATIONARY, different from normal wifi initialization, soft access point = ESP as hotspot, I guess STATIONARY would also be possible?!
		return false;
	}
	if(esp_wifi_start() != ESP_OK) { // 152ms, start the wifi (I think takes more time because CONFIG_ESP32_PHY_CALIBRATION_AND_DATA_STORAGE = false = full calibration on every start?!? don't know)
		return false;
	}
	// reduce TX power due to brownout issue
	esp_wifi_set_max_tx_power(maxTxPower); // 0ms, ONLY WORKS AFTER esp_wifi_start, default = 78 (browns out) -> 52 * 0.25 = 13dBm works always
	/*int8_t txPwr = 0;
	esp_wifi_get_max_tx_power(&txPwr);
	printf("TX POWER: %d\n", txPwr);*/

	// eventually activate long range mode
    if(longrange) { // the LR rate has very limited throughput because the raw PHY data rate LR is 1/2 Mbits and 1/4 Mbits
		if(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR) != ESP_OK) { // CHANGED, was WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR
			return false;
		}
	}
	if(phyRate != WIFI_PHY_RATE_1M_L) {
		if(esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, phyRate) != ESP_OK) { // default is 1M (except long range, then it is 0.25 - 0.5M)
			return false;
		}
	}
	// init ESP NOW
    if(esp_now_init() != ESP_OK) { // 1ms, HERE BROWNOUT POSSIBLE
		return false;
	}
	espNowInitialized = true;
	return true;
}

bool WildFiTagREV6::initESPNOW(bool longrange, int8_t maxTxPower, bool withoutNVS, wifi_phy_rate_t phyRate) {
	esp_log_level_set("ESPNOW", ESP_LOG_NONE); // saves 1ms
	// init NVS (in case not initialized) -> I THINK NOT NEEDED, UNTESTED! -> NO, is needed for PHY calibration in NVS
	if(!withoutNVS) {
		if(!initNVS()) { // 90ms
			return false;
		}
	}
    // init wifi
    esp_netif_init(); // 1ms, CHANGED in ESP IDF 4.1: was tcpip_adapter_init();
    if(esp_event_loop_create_default() != ESP_OK) { // 1ms
		return false;
	}
	esp_netif_create_default_wifi_sta(); // 1ms
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	if(phyRate != WIFI_PHY_RATE_1M_L) {
		cfg.ampdu_tx_enable = 0;
	}
	if(withoutNVS) {
		cfg.nvs_enable = false; // overrides CONFIG_ESP32_WIFI_NVS_ENABLED
	}
	if(esp_wifi_init(&cfg) != ESP_OK) { // 58ms, fails if NVS not initialized
		return false;
	}
	if(esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) { // 0ms, different from normal wifi initialization, default is flash (reduces flash lifetime)
		return false;
	}
	if(esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK) { // 0ms, ACCESS POINT, not STATIONARY, different from normal wifi initialization, soft access point = ESP as hotspot, I guess STATIONARY would also be possible?!
		return false;
	}
	if(esp_wifi_start() != ESP_OK) { // 152ms, start the wifi (I think takes more time because CONFIG_ESP32_PHY_CALIBRATION_AND_DATA_STORAGE = false = full calibration on every start?!? don't know)
		return false;
	}
	// reduce TX power due to brownout issue
	esp_wifi_set_max_tx_power(maxTxPower); // 0ms, ONLY WORKS AFTER esp_wifi_start, default = 78 (browns out) -> 52 * 0.25 = 13dBm works always
	/*int8_t txPwr = 0;
	esp_wifi_get_max_tx_power(&txPwr);
	printf("TX POWER: %d\n", txPwr);*/

	// eventually activate long range mode
    if(longrange) { // the LR rate has very limited throughput because the raw PHY data rate LR is 1/2 Mbits and 1/4 Mbits
		if(esp_wifi_set_protocol(ESP_IF_WIFI_AP, WIFI_PROTOCOL_LR) != ESP_OK) { // CHANGED, was WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR
			return false;
		}
	}
	if(phyRate != WIFI_PHY_RATE_1M_L) {
		if(esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_AP, true, phyRate) != ESP_OK) { // default is 1M (except long range, then it is 0.25 - 0.5M)
			return false;
		}
	}
	// init ESP NOW
    if(esp_now_init() != ESP_OK) { // 1ms, HERE BROWNOUT POSSIBLE
		return false;
	}
	espNowInitialized = true;
	return true;
}

void WildFiTagREV6::stopESPNOW() {
	if(espNowInitialized) {
		esp_now_deinit(); // deletes all information on paired devices
		esp_wifi_stop();
		espNowInitialized = false;
	}
}

bool WildFiTagREV6::addESPNOWBroadcastReceiver() {
	esp_now_peer_info_t peerInfo;
	if(!espNowInitialized) {
		return false;
	}
	memcpy(peerInfo.peer_addr, espNowBroadcastAddress, 6);
	peerInfo.channel = 0;  
	peerInfo.encrypt = false;
	peerInfo.ifidx = ESP_IF_WIFI_AP;                
    if(esp_now_add_peer(&peerInfo) != ESP_OK){
        return false;
	}
	return true;
}

bool WildFiTagREV6::addESPNOWBroadcastReceiverStationary() {
	esp_now_peer_info_t peerInfo;
	if(!espNowInitialized) {
		return false;
	}
	memcpy(peerInfo.peer_addr, espNowBroadcastAddress, 6);
	peerInfo.channel = 0;  
	peerInfo.encrypt = false;
	peerInfo.ifidx = ESP_IF_WIFI_STA;                
    if(esp_now_add_peer(&peerInfo) != ESP_OK){
        return false;
	}
	return true;
}

static void dataSentESPNOW(const uint8_t *mac_addr, esp_now_send_status_t status) {
	if(status == ESP_NOW_SEND_SUCCESS) { espNowSendingStatus = ESP_NOW_FINISHED_AND_RECEIVED; }
	else { espNowSendingStatus = ESP_NOW_FINISHED_BUT_FAILED; }
}

bool WildFiTagREV6::addESPNOWReceiver(uint8_t *mac) {
	esp_now_peer_info_t peerInfo;
	if(!espNowInitialized) {
		return false;
	}
	memcpy(peerInfo.peer_addr, mac, 6);
	peerInfo.channel = 0;  
	peerInfo.encrypt = false;
	peerInfo.ifidx = ESP_IF_WIFI_AP;                
    if(esp_now_add_peer(&peerInfo) != ESP_OK){
        return false;
	}
	esp_now_register_send_cb(dataSentESPNOW);

	return true;	
}

bool WildFiTagREV6::addESPNOWReceiverStationary(uint8_t *mac) {
	esp_now_peer_info_t peerInfo;
	if(!espNowInitialized) {
		return false;
	}
	memcpy(peerInfo.peer_addr, mac, 6);
	peerInfo.channel = 0;  
	peerInfo.encrypt = false;
	peerInfo.ifidx = ESP_IF_WIFI_STA;                
    if(esp_now_add_peer(&peerInfo) != ESP_OK){
        return false;
	}
	esp_now_register_send_cb(dataSentESPNOW);

	return true;	
}

esp_now_sending_status_t WildFiTagREV6::getESPNOWSendingStatus() {
	return espNowSendingStatus;
}

bool WildFiTagREV6::broadcastESPNOWData(uint8_t *data, uint8_t len) {
	if(len > ESP_NOW_MAX_DATA_LEN) {
		return false;
	}
    if(esp_now_send(espNowBroadcastAddress, data, len) == ESP_OK) {
        return true;
    }
    return false;
}

bool WildFiTagREV6::sendESPNOWData(uint8_t *data, uint8_t len) {
	if(len > ESP_NOW_MAX_DATA_LEN) {
		return false;
	}
	espNowSendingStatus = ESP_NOW_NOT_FINISHED; // for ACK
    if(esp_now_send(NULL, data, len) == ESP_OK) { // NULL = send to all peers in list
        return true;
    }
    return false;
}

bool WildFiTagREV6::sendESPNOWDataToMac(uint8_t *mac, uint8_t *data, uint8_t len) {
	if(len > ESP_NOW_MAX_DATA_LEN) {
		return false;
	}
	espNowSendingStatus = ESP_NOW_NOT_FINISHED; // for ACK
    if(esp_now_send(mac, data, len) == ESP_OK) {
        return true;
    }
    return false;
}

/** ----- WIFI ----- */
bool WildFiTagREV6::initWiFi() {
	// init NVS
	if(!initNVS()) { // use NVS here (menuconfig CONFIG_ESP32_WIFI_NVS_ENABLED) to significantly decrease connection time if connected to wifi before
		return false;
	}
	// init wifi
	esp_netif_init(); // CHANGED in ESP IDF 4.1, was tcpip_adapter_init();
    if(esp_event_loop_create_default() != ESP_OK) {
		return false;
	}
	esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if(esp_wifi_init(&cfg) != ESP_OK) {
		return false;
	}
	// commented out below means: use NVS flash as storage (default)
	/*if(!useNVSAsStorage) {
		if(esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) { // different from normal wifi initialization, default is flash (reduces flash lifetime)
			return false;
		}
	}*/
    if(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandlerWiFi, NULL) != ESP_OK) { // register function for events of type WIFI_EVENT (for scan)
		return false;
	}
	if(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandlerWiFi, NULL) != ESP_OK) { // register function for event if connected to certain wifi
		return false;
	}
    if(esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) { // station mode = connects to soft access point (router or phone), soft access point = ESP as hotspot
		return false;
	}
	wiFiInitialized = true;
	return true;
}

static void eventHandlerWiFi(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
	if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) { // HAPPENS WHEN CALLING esp_wifi_start, EVENT STA STARTED AND WANT TO CONNECT TO WIFI OR SCANNING WIFIS
		//printf("REMOVE ME: WIFI STARTED!\n");
		if(!wiFiScanRunning) { // event is also called when only scanning for wifis -> connect attempt will not work, also called when AP not found
			//printf("REMOVE ME: TRY TO CONNECT TO WIFI!\n");
			esp_wifi_connect(); // try to connect to wifi
		}
    }
    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) { // EVENT WIFI SCAN DONE
        wiFiScanDone = true;
		wiFiScanRunning = false;
		//printf("REMOVE ME: WIFI SCAN DONE!\n");
    }
	else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) { // DISCONNECTED FROM WIFI, also called before sleep, ALSO CALLED WHEN AP NOT FOUND!!!
        /*if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }*/
        //printf("REMOVE ME: DISCONNECTED\n");
		wiFiConnected = WIFI_CONNECT_FAIL_AP_NOT_FOUND;
    } else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { // CONNECTED TO WIFI
        //ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		// printf("GOT IP");
		wiFiConnected = WIFI_CONNECT_SUCCESS;
        //s_retry_num = 0;
        //xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

bool WildFiTagREV6::connectToWiFiAfterScan(const char* ssid, const char* password, uint8_t channel) {
	// esp_wifi_start() already called in scanForWiFis
	if(!wiFiScanDone) {
		return false;
	}
	if(channel == WIFI_CHANNELS_1_TO_11) { // should not happen
		channel = 0;
	}
	wiFiConnected = WIFI_CONNECT_RUNNING;
	wifi_config_t wifi_config = { }; // zero initialization
	wifi_config.sta.scan_method = WIFI_FAST_SCAN; // NEW!!! -> maybe problem if no specific channel defined?
	wifi_config.sta.bssid_set = false; // don't use MAC address from wifi
	wifi_config.sta.channel = channel;
	wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
	strcpy((char *)wifi_config.sta.ssid, (char *)ssid);
	strcpy((char *)wifi_config.sta.password, (char *)password);
    if(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) != ESP_OK) {
		return false;
	}
	esp_wifi_connect(); // try to connect
	return true;
}

bool WildFiTagREV6::wiFiSetCountryToUseChannel1to11() {
	wifi_country_t wifiCountry = { };
	strcpy(wifiCountry.cc, "EU");
	wifiCountry.schan = 1;
	wifiCountry.nchan = 11;
	//wifiCountry.maxTxPower = ???;
	wifiCountry.policy = WIFI_COUNTRY_POLICY_MANUAL;
	if(esp_wifi_set_country(&wifiCountry) != ESP_OK) {
		return false;
	}
	return true;
}

bool WildFiTagREV6::connectToWiFiDirectly(const char* ssid, const char* password, int8_t maxTxPower, uint8_t channel) {
	if(!wiFiInitialized) {
		return false;
	}
	if(channel == WIFI_CHANNELS_1_TO_11) {
		channel = 0; // scan on all channels
		if(!wiFiSetCountryToUseChannel1to11()) { // but set country code to only use channels 1 to 11
			return false;
		}
	}
	wiFiConnected = WIFI_CONNECT_RUNNING;
	wifi_config_t wifi_config = { }; // zero initialization
	wifi_config.sta.scan_method = WIFI_FAST_SCAN; // NEW!!! -> maybe problem if no specific channel defined?
	wifi_config.sta.bssid_set = false; // don't use MAC address from wifi
	wifi_config.sta.channel = channel;
	wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
	strcpy((char *)wifi_config.sta.ssid, (char *)ssid);
	strcpy((char *)wifi_config.sta.password, (char *)password);
    if(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) != ESP_OK) {
		return false;
	}
    if(esp_wifi_start() != ESP_OK) {
		return false;
	}
	esp_wifi_set_max_tx_power(maxTxPower); // ONLY WORKS AFTER esp_wifi_start, default = 78 (browns out) -> 52 * 0.25 = 13dBm works always
	return true;
}

void WildFiTagREV6::disconnectAndStopWiFi() {
	esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandlerWiFi);
	esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandlerWiFi);
	esp_wifi_disconnect();
	esp_wifi_stop(); // also stops scanning
	esp_wifi_deinit();
	wiFiInitialized = false;
}


bool WildFiTagREV6::scanForWiFisOn1and6and11WithPriority(bool debug, const char** ssids, const uint8_t wifiListSize, uint8_t *wifiArrayId, uint8_t *foundOnChannel, int8_t maxTxPower, uint32_t scanTimePerChannel1and6, uint16_t timeoutScanMs) {
	// PROBLEM: if scanTime != 120 and wifi on channel 6 or 1 -> will not be changed back to 120s!!!
	const uint8_t CHANNEL_LIST[] = { 6, 1, 11 }; // scan first on channel 6 because highest probability
	uint8_t CHANNEL_LIST_SIZE = sizeof(CHANNEL_LIST) / sizeof(CHANNEL_LIST[0]);

	uint8_t foundOnChannelUnused = 0;
	bool foundAtLeastOneInList = false;
	uint8_t minimumWifiArrayId = 0;
	uint8_t minimumChannel = 0;
	uint8_t foundArrayIdTemp = 0;
	*foundOnChannel = 0;
	if(!wiFiInitialized) { return false; }
	wiFiScanRunning = true; // IMPORTANT: set this BEFORE esp_wifi_start -> because there it will be checked (trys to connect if false)
    if(esp_wifi_start() != ESP_OK) { return false; }
	esp_wifi_set_max_tx_power(maxTxPower); // ONLY WORKS AFTER esp_wifi_start, default = 78 (browns out) -> 52 * 0.25 = 13dBm works always

	/** INIT */
	wifi_active_scan_time_t scanTimeActive = {
		.min = 0, // in ms per channel
		.max = scanTimePerChannel1and6 // in ms per channel
	};
	wifi_scan_time_t scanTime = { };
	scanTime.active = scanTimeActive;
	wifi_scan_config_t scanConf = {
        .ssid = NULL, // NULL = don't scan for specific SSID
        .bssid = NULL, // NULL = don't scan for specific BSSID
        .channel = 0, // 0 = all channel scan
        .show_hidden = true, // show hidden wifis
		.scan_type = WIFI_SCAN_TYPE_ACTIVE, // default = active scan (sends out beacon instead of passively waiting for one)
		.scan_time = scanTime
    };

	/** SCAN CHANNELS */
	for(uint8_t i = 0; i < CHANNEL_LIST_SIZE; i++) {
		scanConf.channel = CHANNEL_LIST[i]; // modify channel
		if(i == (CHANNEL_LIST_SIZE - 1)) { // IMPORTANT: last scan again 120ms long, because otherwise scan_time will be used also during connect (STUPID but TRUE!)
			scanTime.active.max = 120; 
			scanConf.scan_time = scanTime;
		}
		wiFiScanDone = false; // will be set to true in esp_wifi_scan_start (in interrupt routine)
		wiFiScanRunning = true;  // will be set to false in esp_wifi_scan_start (in interrupt routine)
		if(esp_wifi_scan_start(&scanConf, true) != ESP_OK) { return false; } // BLOCKING, HERE BROWNOUT with tx power = 78 (*0.25 = 19.5dBm)
		if(wiFiScanIncludesArray(ssids, wifiListSize, &foundArrayIdTemp, &foundOnChannelUnused)) { // this already returns highest priority (= lowest index)
			if(debug && (foundArrayIdTemp == 0)) { printf("SCAN (ch %d): found 0 -> DIRECTLY RETURN\n", scanConf.channel); }
			if(foundArrayIdTemp == 0) {	*foundOnChannel = scanConf.channel; *wifiArrayId = foundArrayIdTemp; return true; } // already found highest priority network, no need to continue
			else {
				if(!foundAtLeastOneInList) { // first time any wifi from the list was found -> this is best result so far
					minimumWifiArrayId = foundArrayIdTemp;
					minimumChannel = scanConf.channel;
					foundAtLeastOneInList = true;
					if(debug) { printf("SCAN (ch %d): found first id %d\n", scanConf.channel, minimumWifiArrayId); }
				}
				else { // another wifi from the list has already been found -> check if the priority of this here is higher
					if(foundArrayIdTemp < minimumWifiArrayId) { // has higher priority, then use this
						minimumWifiArrayId = foundArrayIdTemp;
						minimumChannel = scanConf.channel;
						if(debug) { printf("SCAN (ch %d): found id %d -> better than %d\n", scanConf.channel, foundArrayIdTemp, minimumWifiArrayId); }				
					}
					else if(debug) { printf("SCAN (ch %d): found id %d, but %d is better\n", scanConf.channel, foundArrayIdTemp, minimumWifiArrayId); }
				}
			}
		}
	}
	/** FINISHED SCANNING (id = 0 = highest priority not found, but maybe another one) */
	if(foundAtLeastOneInList) {
		*foundOnChannel = minimumChannel;
		*wifiArrayId = minimumWifiArrayId;
	}
	return true; // maybe not found but also no error
}

bool WildFiTagREV6::scanForWiFisOn1and6and11and13WithPriority(bool debug, const char** ssids, const uint8_t wifiListSize, uint8_t *wifiArrayId, uint8_t *foundOnChannel, int8_t maxTxPower, uint32_t scanTimePerChannel1and6, uint16_t timeoutScanMs) {
	// PROBLEM: if scanTime != 120 and wifi on channel 6 or 1 -> will not be changed back to 120s!!!
	const uint8_t CHANNEL_LIST[] = { 6, 1, 11, 13 }; // scan first on channel 6 because highest probability
	uint8_t CHANNEL_LIST_SIZE = sizeof(CHANNEL_LIST) / sizeof(CHANNEL_LIST[0]);

	uint8_t foundOnChannelUnused = 0;
	bool foundAtLeastOneInList = false;
	uint8_t minimumWifiArrayId = 0;
	uint8_t minimumChannel = 0;
	uint8_t foundArrayIdTemp = 0;
	*foundOnChannel = 0;
	if(!wiFiInitialized) { return false; }
	wiFiScanRunning = true; // IMPORTANT: set this BEFORE esp_wifi_start -> because there it will be checked (trys to connect if false)
    if(esp_wifi_start() != ESP_OK) { return false; }
	esp_wifi_set_max_tx_power(maxTxPower); // ONLY WORKS AFTER esp_wifi_start, default = 78 (browns out) -> 52 * 0.25 = 13dBm works always

	/** INIT */
	wifi_active_scan_time_t scanTimeActive = {
		.min = 0, // in ms per channel
		.max = scanTimePerChannel1and6 // in ms per channel
	};
	wifi_scan_time_t scanTime = { };
	scanTime.active = scanTimeActive;
	wifi_scan_config_t scanConf = {
        .ssid = NULL, // NULL = don't scan for specific SSID
        .bssid = NULL, // NULL = don't scan for specific BSSID
        .channel = 0, // 0 = all channel scan
        .show_hidden = true, // show hidden wifis
		.scan_type = WIFI_SCAN_TYPE_ACTIVE, // default = active scan (sends out beacon instead of passively waiting for one)
		.scan_time = scanTime
    };

	/** SCAN CHANNELS */
	for(uint8_t i = 0; i < CHANNEL_LIST_SIZE; i++) {
		scanConf.channel = CHANNEL_LIST[i]; // modify channel
		if(i == (CHANNEL_LIST_SIZE - 1)) { // IMPORTANT: last scan again 120ms long, because otherwise scan_time will be used also during connect (STUPID but TRUE!)
			scanTime.active.max = 120; 
			scanConf.scan_time = scanTime;
		}
		wiFiScanDone = false; // will be set to true in esp_wifi_scan_start (in interrupt routine)
		wiFiScanRunning = true;  // will be set to false in esp_wifi_scan_start (in interrupt routine)
		if(esp_wifi_scan_start(&scanConf, true) != ESP_OK) { return false; } // BLOCKING, HERE BROWNOUT with tx power = 78 (*0.25 = 19.5dBm)
		if(wiFiScanIncludesArray(ssids, wifiListSize, &foundArrayIdTemp, &foundOnChannelUnused)) { // this already returns highest priority (= lowest index)
			if(debug && (foundArrayIdTemp == 0)) { printf("SCAN (ch %d): found 0 -> DIRECTLY RETURN\n", scanConf.channel); }
			if(foundArrayIdTemp == 0) {	*foundOnChannel = scanConf.channel; *wifiArrayId = foundArrayIdTemp; return true; } // already found highest priority network, no need to continue
			else {
				if(!foundAtLeastOneInList) { // first time any wifi from the list was found -> this is best result so far
					minimumWifiArrayId = foundArrayIdTemp;
					minimumChannel = scanConf.channel;
					foundAtLeastOneInList = true;
					if(debug) { printf("SCAN (ch %d): found first id %d\n", scanConf.channel, minimumWifiArrayId); }
				}
				else { // another wifi from the list has already been found -> check if the priority of this here is higher
					if(foundArrayIdTemp < minimumWifiArrayId) { // has higher priority, then use this
						minimumWifiArrayId = foundArrayIdTemp;
						minimumChannel = scanConf.channel;
						if(debug) { printf("SCAN (ch %d): found id %d -> better than %d\n", scanConf.channel, foundArrayIdTemp, minimumWifiArrayId); }				
					}
					else if(debug) { printf("SCAN (ch %d): found id %d, but %d is better\n", scanConf.channel, foundArrayIdTemp, minimumWifiArrayId); }
				}
			}
		}
	}
	/** FINISHED SCANNING (id = 0 = highest priority not found, but maybe another one) */
	if(foundAtLeastOneInList) {
		*foundOnChannel = minimumChannel;
		*wifiArrayId = minimumWifiArrayId;
	}
	return true; // maybe not found but also no error
}

bool WildFiTagREV6::scanForWiFisOn1and6and11(const char** ssids, const uint8_t wifiListSize, uint8_t *wifiArrayId, uint8_t *foundOnChannel, int8_t maxTxPower, uint32_t scanTimePerChannel1and6, uint16_t timeoutScanMs) {
	// PROBLEM: if scanTime != 120 and wifi on channel 6 or 1 -> will not be changed back to 120s!!!
	const uint8_t CHANNEL_LIST[] = { 6, 1, 11 }; // scan first on channel 6 because highest probability
	uint8_t CHANNEL_LIST_SIZE = sizeof(CHANNEL_LIST) / sizeof(CHANNEL_LIST[0]);

	uint8_t foundOnChannelUnused = 0;
	*foundOnChannel = 0;
	if(!wiFiInitialized) { return false; }
	wiFiScanRunning = true; // IMPORTANT: set this BEFORE esp_wifi_start -> because there it will be checked (trys to connect if false)
    if(esp_wifi_start() != ESP_OK) { return false; }
	esp_wifi_set_max_tx_power(maxTxPower); // ONLY WORKS AFTER esp_wifi_start, default = 78 (browns out) -> 52 * 0.25 = 13dBm works always

	/** INIT */
	wifi_active_scan_time_t scanTimeActive = {
		.min = 0, // in ms per channel
		.max = scanTimePerChannel1and6 // in ms per channel
	};
	wifi_scan_time_t scanTime = { };
	scanTime.active = scanTimeActive;
	wifi_scan_config_t scanConf = {
        .ssid = NULL, // NULL = don't scan for specific SSID
        .bssid = NULL, // NULL = don't scan for specific BSSID
        .channel = 0, // 0 = all channel scan
        .show_hidden = true, // show hidden wifis
		.scan_type = WIFI_SCAN_TYPE_ACTIVE, // default = active scan (sends out beacon instead of passively waiting for one)
		.scan_time = scanTime
    };

	/** SCAN CHANNELS */
	for(uint8_t i = 0; i < CHANNEL_LIST_SIZE; i++) {
		scanConf.channel = CHANNEL_LIST[i]; // modify channel
		if(i == (CHANNEL_LIST_SIZE - 1)) { // IMPORTANT: last scan again 120ms long, because otherwise scan_time will be used also during connect (STUPID but TRUE!)
			scanTime.active.max = 120; 
			scanConf.scan_time = scanTime;
		}
		wiFiScanDone = false; // will be set to true in esp_wifi_scan_start (in interrupt routine)
		wiFiScanRunning = true;  // will be set to false in esp_wifi_scan_start (in interrupt routine)
		if(esp_wifi_scan_start(&scanConf, true) != ESP_OK) { return false; } // BLOCKING, HERE BROWNOUT with tx power = 78 (*0.25 = 19.5dBm)
		if(wiFiScanIncludesArray(ssids, wifiListSize, wifiArrayId, &foundOnChannelUnused)) { *foundOnChannel = scanConf.channel; return true; }  // found
	}
	return true; // not found but also no error
}

bool WildFiTagREV6::scanForWiFis(bool blocking, int8_t maxTxPower, uint32_t scanTimePerChannel, uint8_t channel) {
	if(!wiFiInitialized) {
		return false;
	}
	if(channel == WIFI_CHANNELS_1_TO_11) {
		channel = 0; // scan on all channels
		if(!wiFiSetCountryToUseChannel1to11()) {
			return false;
		}
	}
	wiFiScanRunning = true; // IMPORTANT: set before esp_wifi_start, otherwise trys to connect
	wiFiScanDone = false;
    if(esp_wifi_start() != ESP_OK) {
		return false;
	}

	esp_wifi_set_max_tx_power(maxTxPower); // ONLY WORKS AFTER esp_wifi_start, default = 78 (browns out) -> 52 * 0.25 = 13dBm works always
	
	/*int8_t txPwr = 0;
	esp_wifi_get_max_tx_power(&txPwr);
	printf("TX POWER: %d\n", txPwr);*/

	wifi_active_scan_time_t scanTimeActive = {
		.min = 0, // in ms per channel
		.max = scanTimePerChannel // in ms per channel (14 channels in total, default = 120ms per channel = 1680ms scan time)
	};
	wifi_scan_time_t scanTime = { };
	scanTime.active = scanTimeActive;
	wifi_scan_config_t scanConf = {
        .ssid = NULL, // NULL = don't scan for specific SSID
        .bssid = NULL, // NULL = don't scan for specific BSSID
        .channel = channel, // 0 = all channel scan
        .show_hidden = true, // show hidden wifis
		.scan_type = WIFI_SCAN_TYPE_ACTIVE, // default = active scan (sends out beacon instead of passively waiting for one)
		.scan_time = scanTime
    };
    if(esp_wifi_scan_start(&scanConf, blocking) != ESP_OK) { // HERE BROWNOUT with tx power = 78 (*0.25 = 19.5dBm)
		return false;
	}
	return true;
}

bool WildFiTagREV6::wiFiScanCompleted() {
	return wiFiScanDone;
}

wifi_connect_status_t WildFiTagREV6::connectedToWiFi() {
	return wiFiConnected;
}

bool WildFiTagREV6::wiFiScanIncludesArray(const char** ssids, const uint8_t wifiListSize, uint8_t *wifiArrayId, uint8_t *foundChannel) {
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
	*wifiArrayId = 0xFF;
    memset(ap_info, 0, sizeof(ap_info));
	*foundChannel = 0; // reset channel
    if(esp_wifi_scan_get_ap_records(&number, ap_info) != ESP_OK) { // important, frees up memory from scanning
		return false;
	}
    if(esp_wifi_scan_get_ap_num(&ap_count) != ESP_OK) {
		if(ap_count == 0) {
			return false; // no results
		}
	}
	// FIRST iterate over known wifi list (for priority), THEN over found wifis
	for(uint8_t wifiList=0; wifiList<wifiListSize; wifiList++) { // iterate over known wifi list
		const char* wifiFromList = ssids[wifiList];
    	for(uint16_t i=0; (i<DEFAULT_SCAN_LIST_SIZE) && (i<ap_count); i++) { // iterate over scanned wifis
			//printf("Scan Found: %s (RSSI: %d)\n", ap_info[i].ssid, ap_info[i].rssi);
			if(strcmp(wifiFromList, (const char*) (ap_info[i].ssid)) == 0) { // 0 = strings are equal
				*wifiArrayId = wifiList;
				*foundChannel = ap_info[i].primary;
				return true; // wifi found!
			}
		}		
    }
	return false;
}

bool WildFiTagREV6::wiFiScanIncludes(const char* ssid, uint8_t *foundChannel) {
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));
	*foundChannel = 0; // reset channel
    if(esp_wifi_scan_get_ap_records(&number, ap_info) != ESP_OK) { // important, frees up memory from scanning
		return false;
	}
    if(esp_wifi_scan_get_ap_num(&ap_count) != ESP_OK) {
		if(ap_count == 0) {
			return false; // no results
		}
	}
    for(uint16_t i=0; (i<DEFAULT_SCAN_LIST_SIZE) && (i<ap_count); i++) {
		if(strcmp(ssid, (const char*) (ap_info[i].ssid)) == 0) { // 0 = strings are equal
			*foundChannel = ap_info[i].primary;
			return true; // wifi found!
		}
    }
	return false;
}

uint8_t WildFiTagREV6::printWiFiScanResults() {
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));
    if(esp_wifi_scan_get_ap_records(&number, ap_info) != ESP_OK) { // important, frees up memory from scanning
		return 255;
	}
    if(esp_wifi_scan_get_ap_num(&ap_count) != ESP_OK) {
		return 255;
	}
    printf("Total APs = %u\n", ap_count);
    for(int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
        printf("SSID %s, ", ap_info[i].ssid);
        printf("RSSI %d, ", ap_info[i].rssi);
        //print_auth_mode(ap_info[i].authmode);
        if (ap_info[i].authmode != WIFI_AUTH_WEP) {
            //print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
        }
        printf("Channel %d\n", ap_info[i].primary);
    }
	return ap_count;
}

/** ----- REST WEBSERVICE ----- */

esp_err_t httpEventHandler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            wiFiPostDataStatus = HTTP_POST_DATA_ERROR_HTTP_EVENT;
            break;
        case HTTP_EVENT_ON_CONNECTED: // connected
            break;
        case HTTP_EVENT_HEADER_SENT: // header sent
            break;
        case HTTP_EVENT_ON_HEADER: // got header
            //printf("HTTP_EVENT_ON_HEADER, key=%s, value=%s\n", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA: // got data
            /*printf("HTTP_EVENT_ON_DATA, len=%d\n", evt->data_len);
            if(!esp_http_client_is_chunked_response(evt->client)) {
                printf("%.*s", evt->data_len, (char*)evt->data); // write out data
            }*/
            break;
        case HTTP_EVENT_ON_FINISH: // finished
            break;
        case HTTP_EVENT_DISCONNECTED: // disconnected (happens normally)
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t) evt->data, &mbedtls_err, NULL);
            if(err != 0) {
                //printf("Last esp error code: 0x%x\n", err);
                //printf("Last mbedtls failure: 0x%x\n", mbedtls_err);
				if(wiFiPostDataStatus == HTTP_POST_DATA_RUNNING) { // don't override other error codes
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_TLS; // called if connected to hotspot, but data not sent through, also happens if data too big
				}
            }
            break;
    }
    return ESP_OK;
}

/*esp_now_stream_status_t WildFiTagREV6::doESPNOWFlashStreamProximityData(uint8_t *macAddress, uint8_t prefix0, uint8_t prefix1, uint8_t prefix2, uint32_t *sendPagePointer, uint16_t *sendPageOffsetPointer, uint32_t flashPageWritePointer, uint16_t flashPageWriteOffsetPointer, uint32_t limitMessagesMax, uint16_t minBatteryVoltageToContinue, uint8_t debugLvl, bool mockFlashRead, bool mockSending) {
	// Data structure see https://docs.google.com/spreadsheets/d/1sjSp5M9RCDqLs_74mrNyFUHEmGBtZ9cjOV2RSFFfckc/edit#gid=243107087
	// Name		SEND_TYPE	OWN_ID	FLASH_PNT	|	PROXIMITY	GPS		PROXIMITY	PROXIMITY	...
	// Bytes	1			2		4	
	// Value	0xBB			
	// Max											243
	uint8_t *dmaBuffer2048Bytes;
	uint8_t data[250] = { 0 };
	bool receivedAck;
	bool haveSeenAnyAck = false;
	uint32_t bytesToSend;
	uint32_t sendBytePointer, writeBytePointer;
	uint32_t messageCounter = 0;

	// do initial range checks
	if((macAddress == NULL)
		|| (*sendPagePointer >= MT29_NUMBER_PAGES)
		|| (*sendPageOffsetPointer >= MT29_CACHE_SIZE)
		|| (flashPageWritePointer >= MT29_NUMBER_PAGES)
		|| (flashPageWriteOffsetPointer >= MT29_CACHE_SIZE)) {
		if(debugLvl > 1) { printf("NOW-STREAM: error, wrong params!\n"); }
		return ESP_NOW_STREAM_DATA_WRONG_PARAMS;
	}
	
	// check how many bytes are needed to send out
	sendBytePointer = ((*sendPagePointer) * MT29_CACHE_SIZE) + (*sendPageOffsetPointer);
	writeBytePointer = (flashPageWritePointer * MT29_CACHE_SIZE) + flashPageWriteOffsetPointer;
	bytesToSend = flash.fifoGetNumberOfPopableBytes(sendBytePointer, writeBytePointer);
	if(limitMessagesMax == ESP_NOW_FLASH_STREAM_PROXIMITY_UNLIMITED) {
		if(debugLvl > 1) { printf("NOW-STREAM: send max messages\n"); }
	}
	if(debugLvl > 0) { printf("NOW-STREAM: in total %d bytes to send\n", bytesToSend); }
	if(bytesToSend == 0) { return ESP_NOW_STREAM_NO_DATA_TO_SEND; }
	
	// create DMA buffer for flash reading
	if(!flash.createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) { // create buffer for flash reading
		if(debugLvl > 1) { printf("NOW-STREAM: dma buffer error!\n"); }
		return ESP_NOW_STREAM_DATA_DMA_BUFFER_ERROR;
	}
	
	// create temporary pointer and state variables
	uint32_t sendPagePointerTemp = (*sendPagePointer);
	uint16_t sendPageOffsetPointerTemp = (*sendPageOffsetPointer);
	uint8_t stateMachine = 0; // 0 = search for data length, 1 = fill esp now message, 2 = send
	uint8_t flashPacketLength = 0;
	uint8_t sendDataPointer = 0;
	uint8_t flashPacketsInEspNowMsg = 0;
	bool headerWritten = false;
	
	while(true) { // while bytesToSend > 0 basically
		// read one single flash page (2048 bytes), read complete page without offset (more easy)
		if(debugLvl > 0) { printf("NOW-STREAM: -- page %d (state %d, missing %d)\n", sendPagePointerTemp, stateMachine, flashPacketLength); }
		if(mockFlashRead) {
			uint8_t cnt = 1;
			for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) {
				//dmaBuffer2048Bytes[i] = cnt;
				//cnt++;
				//if(cnt == 245) { cnt = 1; }
				dmaBuffer2048Bytes[i] = 100;
				//if(sendPagePointerTemp == 1) dmaBuffer2048Bytes[i] = 99;
			}
			//dmaBuffer2048Bytes[MT29_CACHE_SIZE-1] = 1;
		}
		else {
			if(!flash.read(sendPagePointerTemp, 0, dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
				if(debugLvl > 1) { printf("NOW-STREAM: error, flash read failed\n"); }
				free(dmaBuffer2048Bytes);
				return ESP_NOW_STREAM_DATA_FLASH_READ_ERROR;
			}
		}
		
		// iterate over the flash page and put single flash messages into esp now messages
		uint16_t iterator = sendPageOffsetPointerTemp;
		while(true) {
			// STATE 0 - searching for data length
			if(stateMachine == 0) { 
				flashPacketLength = dmaBuffer2048Bytes[iterator]; // first byte of a flash data packet is the length (including this byte)
				if(debugLvl > 1) { printf("NOW-STREAM: %d byte msg, %d bytes free", flashPacketLength, ESP_NOW_FLASH_STREAM_PROXIMITY_PAYLOAD_LEN - sendDataPointer); }
				if((flashPacketLength == 0) || (flashPacketLength > ESP_NOW_FLASH_STREAM_PROXIMITY_PAYLOAD_LEN)) { // violates frame format, length needs to be between 1 and 244
					if(debugLvl > 1) { printf(" -> fatal error packet len = %d\n", flashPacketLength); }
					free(dmaBuffer2048Bytes);
					return ESP_NOW_STREAM_DATA_TOO_LONG_FATAL;
				}
				if(flashPacketLength + sendDataPointer > ESP_NOW_FLASH_STREAM_PROXIMITY_PAYLOAD_LEN) {
					if(debugLvl > 1) { printf(" -> doesn't fit\n"); }
					stateMachine = 2; 
				}
				else {
					if(debugLvl > 1) { printf(" -> fits\n"); }
					stateMachine = 1;
					
					// fill header
					if(!headerWritten) { // first message
						data[0] = prefix0;
						data[1] = prefix1;
						data[2] = prefix2;
						data[3] = sendPagePointerTemp >> 9; // upper 8 bits of 17 bits
						data[4] = (sendPagePointerTemp >> 1) & 0xFF; // middle 8 bits of 17 bits
						data[5] = ((sendPagePointerTemp & 0b1) << 7) | ((iterator >> 4) & 0b1111111); // 1 bit of sendPagePointer, upper 7 bits of sendPageOffsetPointerTemp = i
						data[6] = ((iterator & 0b1111) << 4) | 0b1111;
						headerWritten = true;
						if(debugLvl > 1) { printf("HEADER (%d,%d) "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\n", sendPagePointerTemp, iterator, BYTE_TO_BINARY(data[3]), BYTE_TO_BINARY(data[4]), BYTE_TO_BINARY(data[5]), BYTE_TO_BINARY(data[6])); }
					}
				}
			}
			// STATE 1 - fill message byte by byte
			else if(stateMachine == 1) {
				if(bytesToSend == 0) { // no more bytes to send at all, very last message
					if(flashPacketLength != 0) {
						if(debugLvl > 0) { printf("NOW-STREAM: WARNING pointers inconsistent, %d bytes left\n", flashPacketLength); }
					}
					else {
						if(debugLvl > 1) { printf("NOW-STREAM: no more bytes to send\n"); }
					}
					stateMachine = 2; // send out very last message
					flashPacketsInEspNowMsg++;	
				}
				else if(iterator >= MT29_CACHE_SIZE) { // the 2048 byte page is finished, but still some bytes to send -> read next page
					if(debugLvl > 1) { printf("NOW-STREAM: end of page\n"); }
					break;
				}
				else if(flashPacketLength == 0) { // current data message in flash is finished, but still some bytes to send and page not finished yet
					if(debugLvl > 1) { printf("NOW-STREAM: filled msg\n"); }
					stateMachine = 0; // next message in flash
					flashPacketsInEspNowMsg++;
				}
				else { // just add the byte to the payload
					data[ESP_NOW_FLASH_STREAM_PROXIMITY_HEADER_LEN + sendDataPointer] = dmaBuffer2048Bytes[iterator];
					sendDataPointer++;
					bytesToSend--;
					flashPacketLength--;
					iterator++; // only here bytes are digested (iterator incremented)
					//if(debugLvl > 1) { printf("+"); }
				}	
			}
			// STATE 2 - send message
			else if(stateMachine == 2) { 
				if(debugLvl > 0) { printf("NOW-STREAM: sending (%d + %d bytes with %d packets)\n", sendDataPointer, ESP_NOW_FLASH_STREAM_PROXIMITY_HEADER_LEN, flashPacketsInEspNowMsg); }
				
				// send
				uint8_t sendRetryCounter = 0;
				while(true) {
					sendRetryCounter++;
					if(mockSending) {
						//if(debug) { printf("NOW-STREAM: mock sending %d Bytes: %02X %02X %02X %02X\n", dataLen, data[3], data[4], data[5], data[6]); }
						receivedAck = true;
					}
					else {
						if(!sendESPNOWDataToMac(macAddress, &data[0], ESP_NOW_FLASH_STREAM_PROXIMITY_HEADER_LEN + sendDataPointer)) { // 1ms
							if(debugLvl > 1) { printf("NOW-STREAM: error, sendESPNOWData\n"); }
							free(dmaBuffer2048Bytes);
							return ESP_NOW_STREAM_DATA_ESP_NOW_SEND_ERROR;
						}
						receivedAck = false;
						uint32_t startSendingTime = ((uint32_t) Timing::millis());
						while(1) { // wait for ACK
							esp_now_sending_status_t statusSend = getESPNOWSendingStatus();
							if(statusSend == ESP_NOW_FINISHED_BUT_FAILED) { // 330ms (long range), 88ms in normal mode
								if(debugLvl > 1) { printf("NOW-STREAM: no ack received\n"); }
								break;
							}
							else if(statusSend == ESP_NOW_FINISHED_AND_RECEIVED) { // 10-12ms (long range), 1-4ms in normal mode
								receivedAck = true;
								if(!haveSeenAnyAck) { haveSeenAnyAck = true; } // to see if gateway was in reach at all
								break;
							}
							if(((uint32_t) Timing::millis()) - startSendingTime > 500) {
								if(debugLvl > 1) { printf("NOW-STREAM: fatal ack timeout\n"); }
								free(dmaBuffer2048Bytes);
								return ESP_NOW_STREAM_DATA_ACK_TIMEOUT_FATAL;
							}
						}
					}
					if(receivedAck) { // successfully transmitted message
						// update pointer (call by reference)
						if(debugLvl > 0) { printf("NOW-STREAM: SUCCESS, pointer update %d %d -> ", (*sendPagePointer), (*sendPageOffsetPointer)); }
						*sendPagePointer = sendPagePointerTemp;
						*sendPageOffsetPointer = iterator;
						if(debugLvl > 0) { printf("%d %d\n", (*sendPagePointer), (*sendPageOffsetPointer)); }
						break;
					}
					else {
						if(haveSeenAnyAck) { // got at least one ack before
							if(sendRetryCounter >= ESP_NOW_FLASH_STREAM_RETRIES_AFTER_SUCCESSFUL_TRANSM) { // after 3 tries failed
								free(dmaBuffer2048Bytes);
								return ESP_NOW_STREAM_DATA_NO_ACK_DURING_TRANSMISSION_ERROR;
							}
							else { // had some successful transmission before - retry!
								Timing::delay(ESP_NOW_FLASH_STREAM_RETRIES_WAIT_BETWEEN_MS); // wait a tiny bit
								if(debugLvl > 0) { printf("NOW-STREAM: retry send no %d (had ack before)\n", sendRetryCounter); }	
							}
						} 
						else { free(dmaBuffer2048Bytes); return ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR; } // no ack at all -> gateway not there, immediately stop
					}
				}

				// finished sending one ESP NOW message with maybe several flash packets included
				stateMachine = 0; // start searching for the next flash packet length
				sendDataPointer = 0; // reset pointer to esp now message
				flashPacketLength = 0; // also flash packet was fully transmitted
				messageCounter++; // one more esp now message came through
				headerWritten = false; // next time in state 0: write the header
				flashPacketsInEspNowMsg = 0; // reset number of flash packets in esp now msg
				
				if(bytesToSend == 0) {
					if(debugLvl > 1) { printf("NOW-STREAM: done sending all bytes\n"); }
					break;
				}
				if(messageCounter >= limitMessagesMax) {
					if(debugLvl > 1) { printf("NOW-STREAM: HITTING MESSAGE LIMIT\n"); }
					break;
				}
				// check voltage
				if(((messageCounter - 1) % ESP_NOW_FLASH_STREAM_VOLTAGE_CHECK_EVERY_X_MESSAGES) == (ESP_NOW_FLASH_STREAM_VOLTAGE_CHECK_EVERY_X_MESSAGES - 1)) {
					uint16_t currentVoltage = readSupplyVoltage();
					if(debugLvl > 1) { printf("NOW-STREAM: voltage check %d (min: %d)\n", currentVoltage, minBatteryVoltageToContinue); }
					if(currentVoltage < minBatteryVoltageToContinue) { // check current voltage
						if(debugLvl > 0) { printf("NOW-STREAM: voltage low, stop\n"); }
						free(dmaBuffer2048Bytes);
						return ESP_NOW_STREAM_DATA_VOLTAGE_DURING_TRANSM_LOW;
					}
				}
			}
		}
		
		// finished one flash page
		if((bytesToSend == 0) || (messageCounter >= limitMessagesMax)) { // done with sending
			if(iterator >= MT29_CACHE_SIZE) { // data ends exactly with page
				sendPagePointerTemp++;
				if(sendPagePointerTemp >= MT29_NUMBER_PAGES) { sendPagePointerTemp = 0; } // wrap around
				sendPageOffsetPointerTemp = 0; // start at offset 0 when reading next page
			}
			else { sendPageOffsetPointerTemp = iterator; } // remember offset
			break; // no need to run while loop again			
		}
		else { // prepare reading next flash page
			sendPagePointerTemp++;
			if(sendPagePointerTemp >= MT29_NUMBER_PAGES) { sendPagePointerTemp = 0; } // wrap around
			sendPageOffsetPointerTemp = 0; // start at offset 0 when reading next page				
		}
	}
	free(dmaBuffer2048Bytes);
	if(debugLvl > 0) { printf("NOW-STREAM: transmitted %d messages\n", messageCounter); }
	return ESP_NOW_STREAM_DATA_FINISHED;
}*/

/*
esp_now_stream_status_t WildFiTagREV6::doESPNOWFlashStream(uint8_t *macAddress, uint32_t millisWaitIfSendFailed, uint16_t millisBetweenBlocks, uint32_t flashPointer, uint16_t &flashBlockPointer, uint8_t &flashPageInBlockPointer, uint8_t &flashSubPagePointer, uint16_t flashMaxNumberOfBlocksToTransmit, uint16_t minBatteryVoltageToContinue, bool debug, bool mockFlashRead, bool mockSending, bool mockFlashDelete) {
	// flashSubPagePointer = 0 .. 8 (0 .. 7 = 247 Bytes each, 8 = 72 Bytes)
	uint8_t data[250] = { 0 };
	uint8_t *dmaBuffer2048Bytes;
	bool receivedAck;
	bool haveSeenAnyAck = false;
	uint16_t actuallyTransmittedBlocks = 0;
	uint32_t timeNeededForBlockTransmission;

	// TODO: TIME MEASUREMENT: SQUEEZE OUT AS MUCH DATA AS POSSIBLE IN X SECONDS, THEN STOP! -> instead of flashMaxNumberOfBlocksToTransmit?!?
	// TODO: MAYBE READ MORE THAN 2048 BYTES AT ONCE FROM FLASH MEMORY?? WILL THIS BE FASTER??

	// do initial range checks
	if(flashMaxNumberOfBlocksToTransmit == 0) { // might be intentional to force a gateway scan!
		if(debug) { printf("NOW-STREAM: returning, no data to send!\n"); }
		return ESP_NOW_STREAM_NO_DATA_TO_SEND;
	}
	if((flashBlockPointer >= MT29_NUMBER_BLOCKS) || (flashPageInBlockPointer >= MT29_PAGES_PER_BLOCK) || (flashSubPagePointer >= 9)) {
		if(debug) { printf("NOW-STREAM: error, wrong params!\n"); }
		return ESP_NOW_STREAM_DATA_WRONG_PARAMS;
	}
	// create DMA buffer for flash reading
	if(!flash.createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) { // create buffer for flash reading
		if(debug) { printf("NOW-STREAM: dma buffer error!\n"); }
		return ESP_NOW_STREAM_DATA_DMA_BUFFER_ERROR;
	}

	timeNeededForBlockTransmission = ((uint32_t) Timing::millis()); // start time counter
	
	while(true) { // loop over multiple pages and blocks
		// calculate current page
		uint32_t currentFlashPage = (flashBlockPointer * MT29_PAGES_PER_BLOCK) + flashPageInBlockPointer;
		if(debug) { printf("NOW-STREAM: transmitting flash page %d, flashSubPagePointer %d\n", currentFlashPage, flashSubPagePointer); }

		// read one single flash page (2048 bytes)
		if(mockFlashRead) {
			uint8_t cnt = 0;
			for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) {
				dmaBuffer2048Bytes[i] = cnt;
				cnt++;
				if(cnt == 255) { cnt = 0; }
			}
		}
		else {
			if(!flash.read(currentFlashPage, 0, dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
				if(debug) { printf("NOW-STREAM: error, flash read failed\n"); }
				free(dmaBuffer2048Bytes);
				return ESP_NOW_STREAM_DATA_FLASH_READ_ERROR;
			}
		}
		// transmit one single flash page in 9 ESP NOW bursts
		while(flashSubPagePointer < 9) {
			// 3 BYTES PREAMBLE: [ 0000 0000 ] [ 000 | 00000 ] [ 0 | 0000 | 111 ] (11 bits for block, 6 bits for page in block, 4 bits for page part, 3 dummy bits)
			data[0] = flashBlockPointer >> 3;
			data[1] = ((flashBlockPointer & 0b111) << 5) | (flashPageInBlockPointer >> 1);
			data[2] = ((flashPageInBlockPointer & 0b1) << 7) | ((flashSubPagePointer & 0b1111) << 3) | 0b111;
			//if(debug) { printf("NOW-STREAM: preamble %02X | %02X | %02X\n", data[0], data[1], data[2]); }

			uint8_t dataLen = ESP_NOW_STREAM_PAYLOAD_LEN;
			if(flashSubPagePointer == 8) { // last message in page is a bit shorter
				dataLen = ESP_NOW_STREAM_PAYLOAD_LAST_MESSAGE_LEN;
			}
			uint16_t dmaOffset = flashSubPagePointer * ESP_NOW_STREAM_PAYLOAD_LEN; // flashSubPagePointer max = 8 -> 8 * 247 = 1976, then dataLen = 47
			memcpy(data + 3, dmaBuffer2048Bytes + dmaOffset, dataLen);
			// send the data out
			uint8_t sendRetryCounter = 0;
			while(true) {
				sendRetryCounter++;			
				if(mockSending) {
					//if(debug) { printf("NOW-STREAM: mock sending %d Bytes: %02X %02X %02X %02X\n", dataLen, data[3], data[4], data[5], data[6]); }
					receivedAck = true;
				}
				else {
					// TODO: SEND TO MAC ADDRESS!!
					if(!sendESPNOWDataToMac(macAddress, &data[0], dataLen + 3)) { // 1ms, dataLen is normally 247 -> but send preamble as well, so +3 Byte
						if(debug) { printf("NOW-STREAM: error, sendESPNOWData\n"); }
						free(dmaBuffer2048Bytes);
						return ESP_NOW_STREAM_DATA_ESP_NOW_SEND_ERROR;
					}
					receivedAck = false;
					uint32_t startSendingTime = ((uint32_t) Timing::millis());
					while(1) { // wait for ACK
						esp_now_sending_status_t statusSend = getESPNOWSendingStatus();
						if(statusSend == ESP_NOW_FINISHED_BUT_FAILED) { // 330ms (long range), 88ms in normal mode
							if(debug) { printf("NOW-STREAM: no ack received (flashBlockPointer %d, flashPageInBlockPointer %d, flashSubPagePointer %d)\n", flashBlockPointer, flashPageInBlockPointer, flashSubPagePointer); }
							break;
						}
						else if(statusSend == ESP_NOW_FINISHED_AND_RECEIVED) { // 10-12ms (long range), 1-4ms in normal mode
							receivedAck = true;
							if(!haveSeenAnyAck) { haveSeenAnyAck = true; } // to see if gateway was in reach at all
							break;
						}
						if(((uint32_t) Timing::millis()) - startSendingTime > 500) {
							if(debug) { printf("NOW-STREAM: fatal ack timeout\n"); }
							free(dmaBuffer2048Bytes);
							return ESP_NOW_STREAM_DATA_ACK_TIMEOUT_FATAL;
						}
					}
				}
				if(receivedAck) { // successfully transmitted one of nine parts of a flash page
					flashSubPagePointer++; // MODIFY RETURN VALUE: one part of a page successfully transmitted
					break;
				}
				else {
					if(haveSeenAnyAck) { // got at least one ack before
						if(sendRetryCounter >= ESP_NOW_FLASH_STREAM_RETRIES_AFTER_SUCCESSFUL_TRANSM) { // after 3 tries failed
							free(dmaBuffer2048Bytes);
							return ESP_NOW_STREAM_DATA_NO_ACK_DURING_TRANSMISSION_ERROR;
						}
						else { // had some successful transmission before - retry!
							Timing::delay(millisWaitIfSendFailed); // wait a tiny bit
							if(debug) { printf("NOW-STREAM: retry send no %d (had ack before)\n", sendRetryCounter); }	
						}
					} 
					else { free(dmaBuffer2048Bytes); return ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR; } // no ack at all -> gateway not there, immediately stop
				}
			}
		}
		// fully transmitted a page
		//delay(10); // between pages
		if(flashSubPagePointer >= 9) { // wrap around -> should always happen on very last iteration
			flashSubPagePointer = 0; // MODIFY RETURN VALUE: next page
		}
		flashPageInBlockPointer++; // MODIFY RETURN VALUE: next page
		if(flashPageInBlockPointer >= MT29_PAGES_PER_BLOCK) { // one block fully transmitted (normally 9 * 64 = 576 messages)
			flashPageInBlockPointer = 0; // MODIFY RETURN VALUE: wrap around the flash page counter
			//flashBlockPointer = flash.fifoIncrementBlocksErasedPointer(flashBlockPointer, MT29_NUMBER_PAGES); // MODIFY RETURN VALUE: WARNING: assuming that FIFO has size of full flash memory

			timeNeededForBlockTransmission = ((uint32_t) Timing::millis()) - timeNeededForBlockTransmission;
			if(debug) { printf("NOW-STREAM: 1 block transmitted in %dms, deleting block now from flash\n", timeNeededForBlockTransmission); }

			// delete fully transmitted block
			if(!flash.fifoPopDelete(flashBlockPointer, flashPointer, MT29_NUMBER_PAGES, mockFlashDelete)) { // automatically increments
				if(debug) { printf("NOW-STREAM: ERROR block delete\n"); }
				return ESP_NOW_STREAM_DATA_FLASH_DELETE_ERROR;
			}
			
			actuallyTransmittedBlocks++; // one more block finished!
			if(actuallyTransmittedBlocks >= flashMaxNumberOfBlocksToTransmit) {
				break; // done, jump out of endless loop
			}
			else {
				delay(millisBetweenBlocks); // give everybody, especially the flash, a bit time to breath
				uint16_t currentVoltage = readSupplyVoltage();
				if(debug) { printf("NOW-STREAM: current voltage %d (min: %d)\n", currentVoltage, minBatteryVoltageToContinue); }
				if(currentVoltage < minBatteryVoltageToContinue) { // check current voltage
					if(debug) { printf("NOW-STREAM: voltage low, stop\n"); }
					return ESP_NOW_STREAM_DATA_VOLTAGE_DURING_TRANSM_LOW;
				}
				if(debug) { printf("NOW-STREAM: transmit next block!\n"); }
				timeNeededForBlockTransmission = ((uint32_t) Timing::millis());
			}
		}
	}
	return ESP_NOW_STREAM_DATA_FINISHED;
}
*/

esp_now_stream_status_t WildFiTagREV6::doESPNOWFlashStreamNew(uint8_t *macAddress, uint8_t *customPrefixAdd, uint8_t customPrefixAddLength, uint32_t *sendPagePointer, uint16_t *sendPageOffsetPointer, uint32_t flashPageWritePointer, uint16_t flashPageWriteOffsetPointer, uint32_t maxBytesToSend, uint32_t millisWaitIfSendFailed, uint16_t maxSendRetries, uint16_t minBatteryVoltageToContinue, uint8_t debugLvl, bool mockFlashRead, bool mockSending) {
	// IDEA 1: send data from flash memory on byte level
	// IDEA 2: send data even when block is not full
	// IDEA 3: deleting block is possible after sending data (better in own function?)
	// IDEA 4: put maximum bytes into one ESP NOW message 

	// Data structure
	// Name		SEND_TYPE	FLASH_PNT	|	(customPrefix)		DATA
	// Bytes	1			4	
	// Value	0xAB			
	// Max														245 - customPrefixAddLength
		
	uint8_t *dmaBuffer2048Bytes;
	uint8_t data[250] = { 0 };
	bool receivedAck;
	bool haveSeenAnyAck = false;
	uint32_t bytesToSend;
	uint32_t sendBytePointer, writeBytePointer;
	uint32_t messageCounter = 0;

	// do initial range checks
	if((macAddress == NULL)
		|| (*sendPagePointer >= MT29_NUMBER_PAGES)
		|| (*sendPageOffsetPointer >= MT29_CACHE_SIZE)
		|| (flashPageWritePointer >= MT29_NUMBER_PAGES)
		|| (flashPageWriteOffsetPointer >= MT29_CACHE_SIZE)) {
		if(debugLvl > 1) { printf("NOW-STREAM: error, wrong params!\n"); }
		return ESP_NOW_STREAM_DATA_WRONG_PARAMS;
	}
	
	// check how many bytes are needed to send out
	sendBytePointer = ((*sendPagePointer) * MT29_CACHE_SIZE) + (*sendPageOffsetPointer);
	writeBytePointer = (flashPageWritePointer * MT29_CACHE_SIZE) + flashPageWriteOffsetPointer;
	bytesToSend = flash.fifoGetNumberOfPopableBytes(sendBytePointer, writeBytePointer);
	if(maxBytesToSend != ESP_NOW_FLASH_STREAM_NO_LIMIT)  {
		if(bytesToSend > maxBytesToSend) { bytesToSend = maxBytesToSend; }
		if(debugLvl > 0) { printf("NOW-STREAM: byte limit active\n"); }
	}
	if(debugLvl > 0) { printf("NOW-STREAM: in total %d bytes to send\n", bytesToSend); }
	if(bytesToSend == 0) { return ESP_NOW_STREAM_NO_DATA_TO_SEND; }
	
	// create DMA buffer for flash reading
	if(!flash.createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) { // create buffer for flash reading
		if(debugLvl > 1) { printf("NOW-STREAM: dma buffer error!\n"); }
		return ESP_NOW_STREAM_DATA_DMA_BUFFER_ERROR;
	}
	
	// create temporary pointer and state variables
	uint32_t sendPagePointerTemp = (*sendPagePointer); // use temp variable to later manipulate "real" pointer
	uint16_t sendPageOffsetPointerTemp = (*sendPageOffsetPointer); // use temp variable to later manipulate "real" pointer
	uint8_t stateMachine = 0;
	uint8_t espNowMessagePointer = 0;
	uint8_t bytesFromLastPage = 0;
	
	while(true) { // while bytesToSend > 0 basically
		// STATE 0 -  read one single flash page (2048 bytes), read complete page without offset (more easy)
		if(stateMachine == 0) {
			if(debugLvl > 0) { printf("NOW-STREAM: -- read pg %d\n", sendPagePointerTemp); }
			if(mockFlashRead) {
				for(uint16_t i=0; i<MT29_CACHE_SIZE; i++) {
					dmaBuffer2048Bytes[i] = sendPagePointerTemp;
				}
				dmaBuffer2048Bytes[2047] = 0xAA;
			}
			else {
				if(!flash.read(sendPagePointerTemp, 0, dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
					if(debugLvl > 1) { printf("NOW-STREAM: error, flash read failed\n"); }
					free(dmaBuffer2048Bytes);
					return ESP_NOW_STREAM_DATA_FLASH_READ_ERROR;
				}
			}
			stateMachine = 1;
		}
		// STATE 1 - try fill a message
		else if(stateMachine == 1) {
			// fill header if this is a new message
			if(espNowMessagePointer == 0) {
				data[0] = ESP_NOW_FLASH_STREAM_FIRST_BYTE;
				data[1] = sendPagePointerTemp >> 9; // upper 8 bits of 17 bits
				data[2] = (sendPagePointerTemp >> 1) & 0xFF; // middle 8 bits of 17 bits
				data[3] = ((sendPagePointerTemp & 0b1) << 7) | ((sendPageOffsetPointerTemp >> 4) & 0b1111111); // 1 bit of sendPagePointer, upper 7 bits of sendPageOffsetPointerTemp = i
				data[4] = ((sendPageOffsetPointerTemp & 0b1111) << 4) | 0b1111;
				for(uint8_t i=0; i<customPrefixAddLength; i++) { data[ESP_NOW_FLASH_STREAM_HEADER_LEN+i] = customPrefixAdd[i]; }
				espNowMessagePointer = ESP_NOW_FLASH_STREAM_HEADER_LEN + customPrefixAddLength;
				if(debugLvl > 1) {
					printf("NOW-STREAM: new msg, %d/%d HEAD %02X%02X%02X%02X%02X", sendPagePointerTemp, sendPageOffsetPointerTemp, data[0], data[1], data[2], data[3], data[4]);
					for(uint8_t i=0; i<customPrefixAddLength; i++) { printf("(%02X)", data[ESP_NOW_FLASH_STREAM_HEADER_LEN+i]); }
					printf("\n");
				}
			}
			
			if(espNowMessagePointer > 250) { free(dmaBuffer2048Bytes); return ESP_NOW_STREAM_SEVERE_SOFTWARE_ERROR; }

			// got 3 parameters: bytes left in current page, space left in esp now message, bytes to send
			uint16_t bytesLeftToSendInPage = MT29_CACHE_SIZE - sendPageOffsetPointerTemp; // anything between 0 and 2048 bytes
			uint8_t spaceLeftInEspNowMessage = 250 - espNowMessagePointer; // how many bytes can be squeezed into ESP NOW message
			
			// ASSUMPTION: try to create a full ESP NOW message (250 byte)
			uint8_t bytesToAddToEspNowMessage = spaceLeftInEspNowMessage;
			
			// LIMITER 1: remaining bytes to send (minus what already was added eventually from last page) smaller than what still fits into ESP NOW message
			if((bytesToSend - bytesFromLastPage) < bytesToAddToEspNowMessage) { // message still fits e.g. 245 byte but remaining bytes to send is less than that -> only add that (VERY LAST MESSAGE)
				bytesToAddToEspNowMessage = bytesToSend - bytesFromLastPage;
				if(debugLvl > 1) { printf("NOW-STREAM: -> LAST MSG: only %d bytes left to send (from last page %d)\n", bytesToAddToEspNowMessage, bytesFromLastPage); }
			}
			
			// LIMITER 2: remaining bytes in page are smaller than what still fits into ESP NOW message
			// WARNING: bug before: bytesLeftToSendInPage < bytesToAddToEspNowMessage -> in case EQUAL -> won't load new flash page
			if(bytesLeftToSendInPage <= bytesToAddToEspNowMessage) { // message could still fit e.g. 230 byte but remaining data in page is only 200 bytes
				if((debugLvl > 1) && (bytesLeftToSendInPage < bytesToAddToEspNowMessage)) { printf("NOW-STREAM: -> message over two pages\n"); }
				else if((debugLvl > 1) && (bytesLeftToSendInPage == bytesToAddToEspNowMessage)) { printf("NOW-STREAM: msg ends perfectly with page\n"); }
				bytesToAddToEspNowMessage = bytesLeftToSendInPage;
				// next state reading out next flash page
				stateMachine = 0;
				bytesFromLastPage = bytesToAddToEspNowMessage; // consider this as bytes that already were sent out
			}
			else {
				// next state send data out (do not need to fill data from other page)
				stateMachine = 2;
			}
	
			// copy stuff into current message
			if(bytesToAddToEspNowMessage > 0) {
				memcpy(data + espNowMessagePointer, dmaBuffer2048Bytes + sendPageOffsetPointerTemp, bytesToAddToEspNowMessage);
				espNowMessagePointer += bytesToAddToEspNowMessage;
				if(debugLvl > 1) { printf("NOW-STREAM: adding %d bytes to msg\n", bytesToAddToEspNowMessage); }
				
				// update temporary send pointer
				sendPageOffsetPointerTemp += bytesToAddToEspNowMessage;
				if(sendPageOffsetPointerTemp > MT29_CACHE_SIZE) {
					free(dmaBuffer2048Bytes);
					return ESP_NOW_STREAM_SEVERE_SOFTWARE_ERROR;
				}
				if(sendPageOffsetPointerTemp == MT29_CACHE_SIZE) { sendPageOffsetPointerTemp = 0; sendPagePointerTemp++; }
				if(sendPagePointerTemp >= MT29_NUMBER_PAGES) { sendPagePointerTemp = 0; } // wrap around
			}		
		}
		// STATE 2 - send
		else if(stateMachine == 2) {
			if(debugLvl > 1) { printf("NOW-STREAM: sending %d + %d byte (left: %d)\n", espNowMessagePointer-ESP_NOW_FLASH_STREAM_HEADER_LEN-customPrefixAddLength, ESP_NOW_FLASH_STREAM_HEADER_LEN+customPrefixAddLength, bytesToSend); }
			
			// send
			uint16_t sendRetryCounter = 0;
			while(true) {
				sendRetryCounter++;
				if(mockSending) {
					//if(debug) { printf("NOW-STREAM: mock sending %d Bytes: %02X %02X %02X %02X\n", dataLen, data[3], data[4], data[5], data[6]); }
					receivedAck = true;
				}
				else {
					if(!sendESPNOWDataToMac(macAddress, &data[0], espNowMessagePointer)) { // 1ms
						if(debugLvl > 1) { printf("NOW-STREAM: error, sendESPNOWData\n"); }
						free(dmaBuffer2048Bytes);
						return ESP_NOW_STREAM_DATA_ESP_NOW_SEND_ERROR;
					}
					receivedAck = false;
					uint32_t startSendingTime = ((uint32_t) Timing::millis());
					while(true) { // wait for ACK
						esp_now_sending_status_t statusSend = getESPNOWSendingStatus();
						if(statusSend == ESP_NOW_FINISHED_BUT_FAILED) { // 330ms (long range), 88ms in normal mode
							if(debugLvl > 1) { printf("NOW-STREAM: no ack received\n"); }
							break;
						}
						else if(statusSend == ESP_NOW_FINISHED_AND_RECEIVED) { // 10-12ms (long range), 1-4ms in normal mode
							receivedAck = true;
							if(!haveSeenAnyAck) { haveSeenAnyAck = true; } // to see if gateway was in reach at all
							break;
						}
						if(((uint32_t) Timing::millis()) - startSendingTime > 500) {
							if(debugLvl > 1) { printf("NOW-STREAM: fatal ack timeout\n"); }
							free(dmaBuffer2048Bytes);
							return ESP_NOW_STREAM_DATA_ACK_TIMEOUT_FATAL;
						}
					}
				}
				if(receivedAck) { // successfully transmitted message
					// update pointer (call by reference)
					if(debugLvl > 1) { printf("NOW-STREAM: SUCCESS, pointer update %d %d -> ", (*sendPagePointer), (*sendPageOffsetPointer)); }
					*sendPagePointer = sendPagePointerTemp;
					*sendPageOffsetPointer = sendPageOffsetPointerTemp;
					if(debugLvl > 1) { printf("%d %d\n", (*sendPagePointer), (*sendPageOffsetPointer)); }
					break;
				}
				else {
					if(haveSeenAnyAck) { // got at least one ack before
						if(sendRetryCounter >= maxSendRetries) { // after 3 tries failed
							free(dmaBuffer2048Bytes);
							if(debugLvl > 0) { printf("NOW-STREAM: %d.%d retry %d -> STOP\n", *sendPagePointer, *sendPageOffsetPointer, sendRetryCounter); }	
							return ESP_NOW_STREAM_DATA_NO_ACK_DURING_TRANSMISSION_ERROR;
						}
						else { // had some successful transmission before - retry!
							Timing::delay(millisWaitIfSendFailed); // wait a tiny bit
							if(debugLvl > 0) { printf("NOW-STREAM: %d.%d retry %d\n", *sendPagePointer, *sendPageOffsetPointer, sendRetryCounter); }	
						}
					} 
					else { free(dmaBuffer2048Bytes); return ESP_NOW_STREAM_DATA_NEVER_ACK_ERROR; } // no ack at all -> gateway not there, immediately stop
				}
			}
			
			messageCounter++;
			
			// finished sending one ESP NOW message
			if(bytesToSend < (espNowMessagePointer - ESP_NOW_FLASH_STREAM_HEADER_LEN - customPrefixAddLength)) {
				free(dmaBuffer2048Bytes);
				return ESP_NOW_STREAM_SEVERE_SOFTWARE_ERROR;
			}
			bytesToSend -= (espNowMessagePointer - ESP_NOW_FLASH_STREAM_HEADER_LEN - customPrefixAddLength);
			if(bytesToSend == 0) {
				if(debugLvl > 1) { printf("NOW-STREAM: done sending all bytes\n"); }
				break;
			}
			
			// check voltage
			if(((messageCounter - 1) % ESP_NOW_FLASH_STREAM_VOLTAGE_CHECK_EVERY_X_MESSAGES) == (ESP_NOW_FLASH_STREAM_VOLTAGE_CHECK_EVERY_X_MESSAGES - 1)) {
				uint16_t currentVoltage = readSupplyVoltage();
				if(debugLvl > 1) { printf("NOW-STREAM: voltage check %d (min: %d)\n", currentVoltage, minBatteryVoltageToContinue); }
				if(currentVoltage < minBatteryVoltageToContinue) { // check current voltage
					if(debugLvl > 0) { printf("NOW-STREAM: voltage low, stop\n"); }
					free(dmaBuffer2048Bytes);
					return ESP_NOW_STREAM_DATA_VOLTAGE_DURING_TRANSM_LOW;
				}
			}
			
			espNowMessagePointer = 0;
			stateMachine = 1;
			bytesFromLastPage = 0;
		}
	}
	free(dmaBuffer2048Bytes);
	if(debugLvl > 0) { printf("NOW-STREAM: transmitted %d messages\n", messageCounter); }
	return ESP_NOW_STREAM_DATA_FINISHED;
}

void WildFiTagREV6::restPostStreamGetSuccessfullyTransmittedBlocks(uint16_t &wiFiPostStreamBlocksSuccessfullyTransmittedIn, uint16_t &wiFiPostStreamHalfBlocksSuccessfullyTransmittedIn) {
	wiFiPostStreamBlocksSuccessfullyTransmittedIn = wiFiPostStreamBlocksSuccessfullyTransmitted;
	wiFiPostStreamHalfBlocksSuccessfullyTransmittedIn = wiFiPostStreamHalfBlocksSuccessfullyTransmitted;
}

static void restPostStreamFlashTaskFullBlocks(void *pvParameters) {
	uint32_t PAGES_IN_ONE_REQUEST;
	uint32_t PAGES_IN_DMA_BUFFER;
	uint32_t base64SpaceNeeded = 0; // ONLY IF useBase64Encoding = 1
	char *payloadData = NULL; // ONLY IF useBase64Encoding = 1
	int32_t writeLen = 0;
	uint32_t currentFlashPage = 0;
	uint32_t currentBlock;
	uint32_t streamTransmissionLen = 0;
	bool useBase64Encoding = false; 
	char *restPrefixPointer = NULL;
	uint16_t currentVoltage = 0;
	uint8_t *localBuffer = NULL;

	// read parameters
	post_task_stream_flash_parameters_t taskParams = *(post_task_stream_flash_parameters_t *) pvParameters; // get parameters for the task
	esp_http_client_config_t config = { };
	config.url = taskParams.url;
	//config.timeout_ms = 9000; // for every smaller section of data transfer, never happened?!
    config.event_handler = httpEventHandler; // does not get called if not using perform function?!? -> YES, GETS CALLED!
	esp_http_client_handle_t client;
	if(taskParams.flashMaxNumberOfBlocksToTransmit < 1) { // no blocks to transfer -> stop
		if(taskParams.debug) { printf("POST: error, nothing to upload\n"); }
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_NO_BLOCKS_TO_UPLOAD;
		goto exit;
	}
	wiFiPostStreamBlocksSuccessfullyTransmitted = 0; // no blocks successfully transmitted
	currentBlock = taskParams.flashBlockToSendNextPointer; // block to start reading out fifo
	useBase64Encoding = taskParams.useBase64Encoding;
	currentFlashPage = currentBlock * MT29_PAGES_PER_BLOCK; // blocks to page, starting at beginning of block

	// separate transmission scheme
	if(useBase64Encoding) {
		PAGES_IN_ONE_REQUEST = 64;
		PAGES_IN_DMA_BUFFER = 1;
	}
	else {
		PAGES_IN_ONE_REQUEST = 8;
		PAGES_IN_DMA_BUFFER = 8;
	}

	// client init
    client = esp_http_client_init(&config);
	if(taskParams.debug) { printf("POST: prefixlen %d, postfixlen %d, blockstart %d, maxblocks %d\n", strlen(taskParams.prefix), strlen(taskParams.postfix), currentBlock, taskParams.flashMaxNumberOfBlocksToTransmit); }
	// setup post call
    esp_http_client_set_url(client, taskParams.url);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", taskParams.contentType);
	if((taskParams.additionalHeaderKey != NULL) && (taskParams.additionalHeaderValue != NULL)) {
		esp_http_client_set_header(client, taskParams.additionalHeaderKey, taskParams.additionalHeaderValue);
	}

	// sample first time current voltage:
	currentVoltage = (taskParams.deviceObject)->readSupplyVoltage();
	// reserve memory for PREFIX (copy into separate )
	restPrefixPointer = (char *) malloc(strlen(taskParams.prefix) + 1); // +1 for terminating zero! otherwise system crash
	strcpy(restPrefixPointer, taskParams.prefix);

	// create local buffer instead of taskParams.dataDMA2048Bytes (DMA buffer doesn't make a difference)
	localBuffer = (uint8_t *) malloc(PAGES_IN_DMA_BUFFER * MT29_CACHE_SIZE);
	if(localBuffer == NULL) {
		if(taskParams.debug) { printf("POST: error, local malloc failed\n"); }
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_MALLOC_ENCODING;
		esp_http_client_cleanup(client);
		free(restPrefixPointer);
		goto exit;
	}

	// setup encoding (or not)
	if(useBase64Encoding) { // malloc memory for encoded data
		base64SpaceNeeded = mallocBASE64Encoding(localBuffer, PAGES_IN_DMA_BUFFER * MT29_CACHE_SIZE, &payloadData); // length includes 0-termination
		if(taskParams.debug) { printf("POST: allocated payload space needed: %d\n", base64SpaceNeeded); }
		if((base64SpaceNeeded == 0) || (restPrefixPointer == NULL)) { // also check prefix malloc here
			if(taskParams.debug) { printf("POST: error, malloc failed\n"); }
			wiFiPostDataStatus = HTTP_POST_DATA_ERROR_MALLOC_ENCODING;
			esp_http_client_cleanup(client);
			free(restPrefixPointer);
			free(localBuffer);
			goto exit;
		}
		if(taskParams.debug) { printf("POST: BASE64 after malloc FREE HEAP: %d, LARGEST HEAP BLOCK: %d\n", xPortGetFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); }
	}
	else { // when transmitting raw data -> no additional malloc necessary!
		if(taskParams.debug) { printf("POST: RAW after malloc FREE HEAP: %d, LARGEST HEAP BLOCK: %d\n", xPortGetFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); }
	}

	// loop over multiple blocks
	for(uint16_t multipleBlockIterator=0; multipleBlockIterator<(taskParams.flashMaxNumberOfBlocksToTransmit); multipleBlockIterator++) {
		// open http connection
		if(useBase64Encoding) {
			streamTransmissionLen = strlen(restPrefixPointer) + strlen(taskParams.postfix) + ((PAGES_IN_ONE_REQUEST * PAGES_IN_DMA_BUFFER) * (base64SpaceNeeded - 1)); // must match EXACTLY to written data in total without 0 termination, -1 because base64SpaceNeeded includes 0 termination
		}
		else {
			streamTransmissionLen = strlen(restPrefixPointer) + strlen(taskParams.postfix) + ((PAGES_IN_ONE_REQUEST * PAGES_IN_DMA_BUFFER) * MT29_CACHE_SIZE); // must match EXACTLY to written data in total
		}
		if(taskParams.debug) { printf("POST: HTTP data: %d\n", streamTransmissionLen); }
		if(esp_http_client_open(client, streamTransmissionLen) != ESP_OK) { /** BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME */
			if(taskParams.debug) { printf("POST: error, http open failed\n"); }
			wiFiPostDataStatus = HTTP_POST_DATA_ERROR_HTTP_OPEN;
			if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
			esp_http_client_cleanup(client);
			free(restPrefixPointer);
			goto exit;
		}
		// modify prefix data (if configured)
		if(taskParams.constructCustomPrefix) { // requires following format: ABCDEF:PPPPPPPP:VVVV: (strlen = 21 Bytes, pagepointer index 7, voltage index )
			if(strlen(restPrefixPointer) >= 21) {
				HelperBits::addInt32AsHexToCharArray(restPrefixPointer, 7, currentFlashPage);
				HelperBits::addInt16AsHexToCharArray(restPrefixPointer, 16, currentVoltage);
			}
		}
		// write prefix
		writeLen = esp_http_client_write(client, restPrefixPointer, strlen(restPrefixPointer)); /** BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME */
		if(writeLen <= 0)  {
			if(taskParams.debug) { printf("POST: error, client write failed\n"); }
			wiFiPostDataStatus = HTTP_POST_DATA_ERROR_CLIENT_WRITE;
			esp_http_client_close(client);
			if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
			esp_http_client_cleanup(client);
			free(restPrefixPointer);
			free(localBuffer);
			goto exit;
		}
		// loop over 64 flash pages
		if(taskParams.debug) { printf("POST: flash start page: %d\n", currentFlashPage); }
		uint32_t clientWriteDuration = (uint32_t) Timing::millis();
		for(uint32_t singleBlockIterator=0; singleBlockIterator<PAGES_IN_ONE_REQUEST; singleBlockIterator++) {
			// read  flash page (2048 bytes) -> store in taskParams.dataDMA2048Bytes
			for(uint32_t bufIterator=0; bufIterator<PAGES_IN_DMA_BUFFER; bufIterator++) {
				if(!((taskParams.flashObject)->read(currentFlashPage, 0, localBuffer + (bufIterator * MT29_CACHE_SIZE), MT29_CACHE_SIZE))) {
					if(taskParams.debug) { printf("POST: error, flash read failed\n"); }
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_FLASH_READ;
					esp_http_client_close(client);
					freeMemoryPOSTData(payloadData);
					esp_http_client_cleanup(client);
					free(restPrefixPointer);
					free(localBuffer);
					goto exit;
				}
				currentFlashPage++;
			}
			//if(taskParams.debug) { printf("POST: - http writing page %d, first bytes %02X %02X %02X %02X\n", currentFlashPage, taskParams.dataDMA2048Bytes[0], taskParams.dataDMA2048Bytes[1], taskParams.dataDMA2048Bytes[2], taskParams.dataDMA2048Bytes[3]); }
			if(useBase64Encoding) { 
				BASE64Encoding(localBuffer, MT29_CACHE_SIZE * PAGES_IN_DMA_BUFFER, &payloadData, base64SpaceNeeded); // encode flash data
				writeLen = esp_http_client_write(client, payloadData, strlen(payloadData)); // write data, BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME
			}
			else {
				writeLen = esp_http_client_write(client, (char *) (localBuffer), MT29_CACHE_SIZE * PAGES_IN_DMA_BUFFER); // write data, BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME
			}
			if(writeLen <= 0)  {
				if(taskParams.debug) { printf("POST: error, client write failed\n"); }
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_CLIENT_WRITE;
				esp_http_client_close(client);
				if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
				esp_http_client_cleanup(client);
				free(restPrefixPointer);
				free(localBuffer);
				goto exit;
			}
		}
		clientWriteDuration = ((uint32_t) Timing::millis()) - clientWriteDuration;
		if(taskParams.debug) { printf("POST: client write data took %d ms\n", clientWriteDuration); }
		// write postfix
		writeLen = esp_http_client_write(client, taskParams.postfix, strlen(taskParams.postfix)); /** BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME */
		if(writeLen <= 0)  {
			if(taskParams.debug) { printf("POST: error, client write failed\n"); }
			wiFiPostDataStatus = HTTP_POST_DATA_ERROR_CLIENT_WRITE;
			esp_http_client_close(client);
			if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
			esp_http_client_cleanup(client);
			free(restPrefixPointer);
			free(localBuffer);
			goto exit;
		}
		// read response
		int contentLen = esp_http_client_fetch_headers(client); // DO NOT PERFORM READ! just look at return status codes in response header(s)
		if(contentLen < 0) {
			if(taskParams.debug) { printf("POST: error, fetch header failed\n"); }
			wiFiPostDataStatus = HTTP_POST_DATA_ERROR_FETCH_HEADER;
			esp_http_client_close(client);
			if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
			esp_http_client_cleanup(client);
			free(restPrefixPointer);
			free(localBuffer);
			goto exit;				
		}
		if(taskParams.debug) { printf("POST: header response with %d bytes\n", contentLen); }
		// close connection
		esp_http_client_close(client);
		// check response
		int statusCode = esp_http_client_get_status_code(client);
		if(taskParams.debug) { printf("POST: status = %d\n", statusCode); }
		if(statusCode != 201) { // if one POST call didn't go through -> cancel whole operation (but wiFiPostStreamBlocksSuccessfullyTransmitted might transmitted successfully partly)
			if(taskParams.debug) { printf("POST: error, status code != 201\n"); }
			if(statusCode == -1) {
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_MIN1;
			}
			else if((statusCode >= 100) && (statusCode < 200)) {
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_100;
			}
			else if((statusCode >= 200) && (statusCode < 300)) {
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_200; // 200-something but not 201
			}
			else if((statusCode >= 400) && (statusCode < 400)) {
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_400; // 200-something but not 201
			}
			else if((statusCode >= 500) && (statusCode < 500)) {
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_500; // 200-something but not 201
			}
			else {
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_UNKNOWN; // 200-something but not 201
			}
			if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
			esp_http_client_cleanup(client);
			free(restPrefixPointer);
			free(localBuffer);
			goto exit;
		}
		if(taskParams.debug) { printf("POST: 1 block done\n"); }
		wiFiPostStreamBlocksSuccessfullyTransmitted++; // IMPORTANT: check this even if operation was cancelled with error code, maybe some stuff successfully transmitted before
		currentBlock = (taskParams.flashObject)->fifoIncrementBlocksErasedPointer(currentBlock, MT29_NUMBER_PAGES); // WARNING: assuming that FIFO has size of full flash memory
		currentFlashPage = currentBlock * MT29_PAGES_PER_BLOCK; // start with new page (currentBlock could have been wrapped around)

		bool doneSendingAllBlocks = (multipleBlockIterator == (taskParams.flashMaxNumberOfBlocksToTransmit - 1));
        if(!doneSendingAllBlocks) {
			Timing::delay(100); // add delay to give battery voltage some time to breath (don't delay if last block)
			currentVoltage = (taskParams.deviceObject)->readSupplyVoltage();
			if(taskParams.debug) { printf("POST: current voltage %d (min: %d)\n", currentVoltage, taskParams.minBatteryVoltageToContinue); }
			if(currentVoltage < taskParams.minBatteryVoltageToContinue) {
				if(taskParams.debug) { printf("POST: error, voltage too low to continue sending more blocks\n"); }
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_VOLTAGE_LOW;
				if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
				esp_http_client_cleanup(client);
				free(restPrefixPointer);
				free(localBuffer);
				goto exit;
			}
		} 
		else {
			if(taskParams.debug) { printf("POST: FINISHED sending all blocks\n"); }
		}
	}
	// end of service
	if(useBase64Encoding) { freeMemoryPOSTData(payloadData); } // free up the BASE64 buffer
	free(restPrefixPointer); // free up prefix memory
	free(localBuffer);
    esp_http_client_cleanup(client);
	wiFiPostDataStatus = HTTP_POST_DATA_FINISHED_ALL_GOOD; // success

exit:
    { vTaskDelete(NULL); }
}

static void restPostStreamFlashTaskHalfBlocks(void *pvParameters) {
	const uint32_t PAGES_IN_ONE_GO = 32; // 32 * 2048 = 65536 -> encoded = 87382 Bytes in one transmission -> max 64, then transmitting one full block
	uint32_t base64SpaceNeeded = 0; // ONLY IF useBase64Encoding = 1
	char *payloadData = NULL; // ONLY IF useBase64Encoding = 1
	int32_t writeLen = 0;
	uint32_t currentFlashPage = 0;
	uint32_t currentBlock;
	uint16_t currentHalfBlock;
	uint32_t streamTransmissionLen = 0;
	bool useBase64Encoding = false; 
	char *restPrefixPointer = NULL;
	uint16_t currentVoltage = 0;
	uint8_t *localBuffer = NULL;

	post_task_stream_flash_parameters_t taskParams = *(post_task_stream_flash_parameters_t *) pvParameters; // get parameters for the task
	esp_http_client_config_t config = { };
	config.url = taskParams.url;
	//config.timeout_ms = 9000; // for every smaller section of data transfer, never happened?!
    config.event_handler = httpEventHandler; // does not get called if not using perform function?!? -> YES, GETS CALLED!
	esp_http_client_handle_t client;
	if(taskParams.flashMaxNumberOfBlocksToTransmit < 1) { // no blocks to transfer -> fuck it
		if(taskParams.debug) { printf("POST: error, nothing to upload\n"); }
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_NO_BLOCKS_TO_UPLOAD;
		goto exit;
	}
	wiFiPostStreamBlocksSuccessfullyTransmitted = 0; // no blocks successfully transmitted
	wiFiPostStreamHalfBlocksSuccessfullyTransmitted = 0; // also no half blocks
	currentBlock = taskParams.flashBlockToSendNextPointer; // block to start reading out fifo
	currentHalfBlock = taskParams.flashHalfBlockToSendNextPointer;
	useBase64Encoding = taskParams.useBase64Encoding;
	if(currentHalfBlock > 1) { currentHalfBlock = 1; } // for now only support 0 or 1 as value
	currentFlashPage = (currentBlock * MT29_PAGES_PER_BLOCK) + (currentHalfBlock * PAGES_IN_ONE_GO); // blocks to page, starting in middle of block or at beginning of block
	
	// client init
    client = esp_http_client_init(&config);
	if(taskParams.debug) { printf("POST: prefixlen %d, postfixlen %d, blockstart %d, halfblock %d, maxblocks %d\n", strlen(taskParams.prefix), strlen(taskParams.postfix), currentBlock, currentHalfBlock, taskParams.flashMaxNumberOfBlocksToTransmit); }
	
	// setup post call
    esp_http_client_set_url(client, taskParams.url);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", taskParams.contentType);
	if((taskParams.additionalHeaderKey != NULL) && (taskParams.additionalHeaderValue != NULL)) {
		esp_http_client_set_header(client, taskParams.additionalHeaderKey, taskParams.additionalHeaderValue);
	}
	
	// sample first time current voltage:
	currentVoltage = (taskParams.deviceObject)->readSupplyVoltage();
	
	// reserve memory for PREFIX (copy into separate )
	restPrefixPointer = (char *) malloc(strlen(taskParams.prefix) + 1); // +1 for terminating zero! otherwise system crash
	strcpy(restPrefixPointer, taskParams.prefix);

	// create local buffer instead of taskParams.dataDMA2048Bytes (DMA buffer doesn't make a difference)
	localBuffer = (uint8_t *) malloc(MT29_CACHE_SIZE);
	if(localBuffer == NULL) {
		if(taskParams.debug) { printf("POST: error, local malloc failed\n"); }
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_MALLOC_ENCODING;
		esp_http_client_cleanup(client);
		free(restPrefixPointer);
		goto exit;
	}

	// setup encoding (or not)
	if(useBase64Encoding) { // malloc memory for encoded data
		base64SpaceNeeded = mallocBASE64Encoding(localBuffer, MT29_CACHE_SIZE, &payloadData); // length includes 0-termination
		if(taskParams.debug) { printf("POST: allocated payload space needed: %d\n", base64SpaceNeeded); }
		if((base64SpaceNeeded == 0) || (restPrefixPointer == NULL)) { // also check prefix malloc here
			if(taskParams.debug) { printf("POST: error, malloc failed\n"); }
			wiFiPostDataStatus = HTTP_POST_DATA_ERROR_MALLOC_ENCODING;
			esp_http_client_cleanup(client);
			free(restPrefixPointer);
			free(localBuffer);
			goto exit;
		}
		if(taskParams.debug) { printf("POST: BASE64 after malloc FREE HEAP: %d, LARGEST HEAP BLOCK: %d\n", xPortGetFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); }
	}
	else { // when transmitting raw data -> no additional malloc necessary!
		if(taskParams.debug) { printf("POST: TRANSMIT RAW BINARY DATA!\n"); }
	}
	// loop over multiple blocks
	for(uint16_t multipleBlockIterator=0; multipleBlockIterator<(taskParams.flashMaxNumberOfBlocksToTransmit); multipleBlockIterator++) {
		// loop over a full http request (might start from half block offset)
		for(uint16_t fullBlockIterator=currentHalfBlock; fullBlockIterator<(MT29_PAGES_PER_BLOCK/PAGES_IN_ONE_GO); fullBlockIterator++) { // normally two times until block is full, in case half block transmission: only 1 time
			// open http connection
			if(useBase64Encoding) {
				streamTransmissionLen = strlen(restPrefixPointer) + strlen(taskParams.postfix) + (PAGES_IN_ONE_GO * (base64SpaceNeeded - 1)); // must match EXACTLY to written data in total without 0 termination, -1 because base64SpaceNeeded includes 0 termination
			}
			else {
				streamTransmissionLen = strlen(restPrefixPointer) + strlen(taskParams.postfix) + (PAGES_IN_ONE_GO * MT29_CACHE_SIZE); // must match EXACTLY to written data in total
			}
			if(taskParams.debug) { printf("POST: HTTP data in total: %d\n", streamTransmissionLen); }
			if(esp_http_client_open(client, streamTransmissionLen) != ESP_OK) { /** BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME */
				if(taskParams.debug) { printf("POST: error, http open failed\n"); }
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_HTTP_OPEN;
				if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
				esp_http_client_cleanup(client);
				free(restPrefixPointer);
				free(localBuffer);
				goto exit;
			}
			// modify prefix data (if configured)
			if(taskParams.constructCustomPrefix) { // requires following format: ABCDEF:PPPPPPPP:VVVV: (strlen = 21 Bytes, pagepointer index 7, voltage index )
				if(strlen(restPrefixPointer) >= 21) {
                    HelperBits::addInt32AsHexToCharArray(restPrefixPointer, 7, currentFlashPage);
                    HelperBits::addInt16AsHexToCharArray(restPrefixPointer, 16, currentVoltage);
                }
			}
			// write prefix
			if(taskParams.debug) { printf("POST: - http writing prefix\n"); }
			writeLen = esp_http_client_write(client, restPrefixPointer, strlen(restPrefixPointer)); /** BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME */
			if(writeLen <= 0)  {
				if(taskParams.debug) { printf("POST: error, client write failed\n"); }
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_CLIENT_WRITE;
				esp_http_client_close(client);
				if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
				esp_http_client_cleanup(client);
				free(restPrefixPointer);
				free(localBuffer);
				goto exit;
			}
			// loop over 32 flash pages
			if(taskParams.debug) { printf("POST: flash start page: %d, reading %d times\n", currentFlashPage, PAGES_IN_ONE_GO); }
			for(uint32_t halfBlockIterator=0; halfBlockIterator<PAGES_IN_ONE_GO; halfBlockIterator++) {
				// read one flash page (2048 bytes) -> store in taskParams.dataDMA2048Bytes
				if(!((taskParams.flashObject)->read(currentFlashPage, 0, localBuffer, MT29_CACHE_SIZE))) {
					if(taskParams.debug) { printf("POST: error, flash read failed\n"); }
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_FLASH_READ;
					esp_http_client_close(client);
					freeMemoryPOSTData(payloadData);
					esp_http_client_cleanup(client);
					free(restPrefixPointer);
					free(localBuffer);
					goto exit;
				}
				//if(taskParams.debug) { printf("POST: - http writing page %d, first bytes %02X %02X %02X %02X\n", currentFlashPage, taskParams.dataDMA2048Bytes[0], taskParams.dataDMA2048Bytes[1], taskParams.dataDMA2048Bytes[2], taskParams.dataDMA2048Bytes[3]); }
				currentFlashPage++;
				if(useBase64Encoding) { 
					BASE64Encoding(localBuffer, MT29_CACHE_SIZE, &payloadData, base64SpaceNeeded); // encode flash data
					writeLen = esp_http_client_write(client, payloadData, strlen(payloadData)); // write data, BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME
				}
				else {
					writeLen = esp_http_client_write(client, (char *) (localBuffer), MT29_CACHE_SIZE); // write data, BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME
				}
				if(writeLen <= 0)  {
					if(taskParams.debug) { printf("POST: error, client write failed\n"); }
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_CLIENT_WRITE;
					esp_http_client_close(client);
					if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
					esp_http_client_cleanup(client);
					free(restPrefixPointer);
					free(localBuffer);
					goto exit;
				}
			}
			// write postfix
			if(taskParams.debug) { printf("POST: - http writing postfix\n"); }
			writeLen = esp_http_client_write(client, taskParams.postfix, strlen(taskParams.postfix)); /** BAD INTERNET (Edge only) -> WILL HANG HERE LONG TIME */
			if(writeLen <= 0)  {
				if(taskParams.debug) { printf("POST: error, client write failed\n"); }
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_CLIENT_WRITE;
				esp_http_client_close(client);
				if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
				esp_http_client_cleanup(client);
				free(restPrefixPointer);
				free(localBuffer);
				goto exit;
			}
			// read response
			int contentLen = esp_http_client_fetch_headers(client); // DO NOT PERFORM READ! just look at return status codes in response header(s)
			if(contentLen < 0) {
				if(taskParams.debug) { printf("POST: error, fetch header failed\n"); }
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_FETCH_HEADER;
				esp_http_client_close(client);
				if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
				esp_http_client_cleanup(client);
				free(restPrefixPointer);
				free(localBuffer);
				goto exit;				
			}
			if(taskParams.debug) { printf("POST: header: got response with %d bytes\n", contentLen); }
			// close connection
			esp_http_client_close(client);
			// check response
			int statusCode = esp_http_client_get_status_code(client);
			if(taskParams.debug) { printf("POST: status code = %d\n", statusCode); }
			if(statusCode != 201) { // if one POST call didn't go through -> cancel whole operation (but wiFiPostStreamBlocksSuccessfullyTransmitted might transmitted successfully partly)
				if(taskParams.debug) { printf("POST: error, status code != 201\n"); }
				if(statusCode == -1) {
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_MIN1;
				}
				else if((statusCode >= 100) && (statusCode < 200)) {
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_100;
				}
				else if((statusCode >= 200) && (statusCode < 300)) {
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_200; // 200-something but not 201
				}
				else if((statusCode >= 400) && (statusCode < 400)) {
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_400; // 200-something but not 201
				}
				else if((statusCode >= 500) && (statusCode < 500)) {
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_500; // 200-something but not 201
				}
				else {
					wiFiPostDataStatus = HTTP_POST_DATA_ERROR_RET_CODE_UNKNOWN; // 200-something but not 201
				}
				if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
				esp_http_client_cleanup(client);
				free(restPrefixPointer);
				free(localBuffer);
				goto exit;
			}
			if(fullBlockIterator == 0) { // first half of block has been transmitted successfully at this point
				wiFiPostStreamHalfBlocksSuccessfullyTransmitted = 1; // REMEMBER FOR NEXT SENDING: HALF of the block was transmitted -> in case next iteration fails we already know that
				if(taskParams.debug && (currentHalfBlock == 1)) { printf("POST: REMEMBER! half block already sent out\n"); }
			}
		} // end of sending 2x half a block or only 1x half a block
		wiFiPostStreamHalfBlocksSuccessfullyTransmitted = 0; // a full block was completed -> no need to remember half block transmissions for next sending
		if(taskParams.debug && (currentHalfBlock == 0)) { printf("POST: SUCCESSFULLY TRANSMITTED one block in two transmissions\n"); }
		if(taskParams.debug && (currentHalfBlock == 1)) { printf("POST: SUCCESSFULLY TRANSMITTED missing half block\n"); }
		if(currentHalfBlock == 1) { // successfully transmitted a missing half block
			currentHalfBlock = 0; // when started from half block offset -> now no offset anymore, next iteration transmitting a full block
			wiFiPostStreamBlocksSuccessfullyTransmitted++; // also increment here, because the full block is now transmitted
		} 
		else { // transmitted a full block
			wiFiPostStreamBlocksSuccessfullyTransmitted++; // IMPORTANT: check this even if operation was cancelled with error code, maybe some stuff successfully transmitted before
		}
		currentBlock = (taskParams.flashObject)->fifoIncrementBlocksErasedPointer(currentBlock, MT29_NUMBER_PAGES); // WARNING: assuming that FIFO has size of full flash memory
		currentFlashPage = currentBlock * MT29_PAGES_PER_BLOCK; // start with new page (currentBlock could have been wrapped around, don't use half blocks here)

		bool doneSendingAllBlocks = (multipleBlockIterator == (taskParams.flashMaxNumberOfBlocksToTransmit - 1));
        if(!doneSendingAllBlocks) {
        	Timing::delay(100); // add delay to give battery voltage some time to breath (don't delay if last block)
			currentVoltage = (taskParams.deviceObject)->readSupplyVoltage();
			if(taskParams.debug) { printf("POST: current voltage %d (min: %d)\n", currentVoltage, taskParams.minBatteryVoltageToContinue); }
			if(currentVoltage < taskParams.minBatteryVoltageToContinue) {
				if(taskParams.debug) { printf("POST: error, voltage too low to continue sending more blocks\n"); }
				wiFiPostDataStatus = HTTP_POST_DATA_ERROR_VOLTAGE_LOW;
				if(useBase64Encoding) { freeMemoryPOSTData(payloadData); }
				esp_http_client_cleanup(client);
				free(restPrefixPointer);
				free(localBuffer);
				goto exit;
			}
		} 
		else {
			if(taskParams.debug) { printf("POST: FINISHED sending all blocks\n"); }
		}
	}
	// end of service
	freeMemoryPOSTData(payloadData); // free up the BASE64 buffer
	free(restPrefixPointer); // free up prefix memory
	free(localBuffer);
    esp_http_client_cleanup(client);
	wiFiPostDataStatus = HTTP_POST_DATA_FINISHED_ALL_GOOD; // success
exit:
    { vTaskDelete(NULL); }
}

static void restPostTask(void *pvParameters) {
    esp_err_t err;
	post_task_parameters_t taskParams = *(post_task_parameters_t *) pvParameters; // get parameters for the task
    esp_http_client_config_t config = { }; // zero initialize 
	config.url = taskParams.url;
	config.event_handler = httpEventHandler;
    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    printf("TASK PARAM LEN %d, %d, %d\n", strlen(taskParams.prefix), strlen(taskParams.postfix), taskParams.dataLen);

    char *payloadData; // pointer to malloc of payload data
    if(!createPOSTData(taskParams.prefix, taskParams.data, taskParams.dataLen, taskParams.postfix, &payloadData)) { // mallocing and BASE64 encoding
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_MALLOC_ENCODING;
    	esp_http_client_cleanup(client);
    	vTaskDelete(NULL);
	}
    printf("Payload Length: %d\n", strlen(payloadData));

    esp_http_client_set_url(client, taskParams.url);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(client, payloadData, strlen(payloadData));
    esp_http_client_set_header(client, "Content-Type", taskParams.contentType);
	if(taskParams.additionalHeaderKey != NULL) {
		esp_http_client_set_header(client, taskParams.additionalHeaderKey, taskParams.additionalHeaderValue);
	}
    err = esp_http_client_perform(client); // BLOCKING! UP TO 18 SECONDS IF CONNECTED TO WIFI BUT NOT ONLINE
    if(err == ESP_OK) {
		wiFiPostReturnCode = esp_http_client_get_status_code(client);
        //printf("HTTP POST Status = %d, content_length = %d\n", esp_http_client_get_status_code(client), esp_http_client_get_content_length(client));
    } else {
        //printf("HTTP POST request failed: %s\n", esp_err_to_name(err));
		wiFiPostReturnCode = 0;
    }
    /*
    // GET
    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        printf("HTTP GET Status = %d, content_length = %d\n",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        printf("HTTP GET request failed: %s\n", esp_err_to_name(err));
    }
    */
    freeMemoryPOSTData(payloadData);
    esp_http_client_cleanup(client);
	wiFiPostDataStatus = HTTP_POST_DATA_FINISHED_ALL_GOOD;
    vTaskDelete(NULL);
}

wifi_post_data_status_t WildFiTagREV6::getWiFiPOSTCallStatus() {
	return wiFiPostDataStatus;
}

uint16_t WildFiTagREV6::getWiFiPostReturnCode() {
	return wiFiPostReturnCode;
}

void WildFiTagREV6::doWiFiPOSTCall(post_task_parameters_t *globalPostParameters, const uint16_t taskStackSize) {
	if(wiFiConnected != WIFI_CONNECT_SUCCESS) {
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_WIFI_NOT_CONNECTED;
		return;
	}
	wiFiPostDataStatus = HTTP_POST_DATA_RUNNING;
	wiFiPostReturnCode = 0;
	if(xTaskCreate(&restPostTask, "restPostTask", taskStackSize, globalPostParameters, 5, &RTOStaskHandle) != pdPASS) { // IMPORTANT: globalPostParameters and all data within NEEDS TO BE GLOBAL!
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_TASK_CREATE;
	}
}

void WildFiTagREV6::doWiFiPOSTStreamCallFlashHalfBlock(post_task_stream_flash_parameters_t *globalPostParameters) {
	const uint16_t taskStackSize = 8192;
	if(wiFiConnected != WIFI_CONNECT_SUCCESS) {
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_WIFI_NOT_CONNECTED;
		return;
	}
	wiFiPostDataStatus = HTTP_POST_DATA_RUNNING;
	if(xTaskCreate(&restPostStreamFlashTaskHalfBlocks, "restPostStreamFlashTaskHalfBlocks", taskStackSize, globalPostParameters, 5, &RTOStaskHandle) != pdPASS) { // IMPORTANT: globalPostParameters and all data within NEEDS TO BE GLOBAL!
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_TASK_CREATE;
	}
}

void WildFiTagREV6::doWiFiPOSTStreamCallFlashFullBlock(post_task_stream_flash_parameters_t *globalPostParameters) {
	const uint32_t taskStackSize = 32768;
	if(wiFiConnected != WIFI_CONNECT_SUCCESS) {
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_WIFI_NOT_CONNECTED;
		return;
	}
	wiFiPostDataStatus = HTTP_POST_DATA_RUNNING;
	if(xTaskCreate(&restPostStreamFlashTaskFullBlocks, "restPostStreamFlashTaskFullBlocks", taskStackSize, globalPostParameters, 5, &RTOStaskHandle) != pdPASS) { // IMPORTANT: globalPostParameters and all data within NEEDS TO BE GLOBAL!
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_TASK_CREATE;
	}
}

void WildFiTagREV6::killPOSTTask() {
	if(RTOStaskHandle != NULL) {
		vTaskDelete(RTOStaskHandle);
		wiFiPostDataStatus = HTTP_POST_DATA_ERROR_TIMEOUT_TASK_KILLED; // be careful, this variable is modified in different task (which should be deleted here)
	}
}

bool createPOSTData(const char *prefixIn, uint8_t *dataIn, uint32_t dataInLen, const char *postfixIn, char **dataOut) {
    size_t lenOfBASE64EncodedData;
    mbedtls_base64_encode(NULL, 0, &lenOfBASE64EncodedData, (unsigned char*) dataIn, dataInLen); // check how long base64 encoded data would be
    printf("STRLEN: %d, ENCODELEN: %d, PRE+POSTFIXLEN: %d, FREE HEAP: %d, LARGEST HEAP BLOCK: %d\n", dataInLen, lenOfBASE64EncodedData, strlen(prefixIn) + strlen(postfixIn), xPortGetFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
	*dataOut = (char *) malloc(1 + strlen(prefixIn) + lenOfBASE64EncodedData + strlen(postfixIn)); // malloc for whole encoded payload including prefix and postfix and 0 termination of char array
	if(*dataOut == NULL) {
		return false;
	}
	printf("AFTER MALLOC: FREE HEAP: %d, LARGEST HEAP BLOCK: %d\n", xPortGetFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    mbedtls_base64_encode((unsigned char*) ((*dataOut)+strlen(prefixIn)), lenOfBASE64EncodedData, &lenOfBASE64EncodedData, (unsigned char*) dataIn, dataInLen); // encode data and store in dataOut at position after prefix
	for(uint32_t i=0; i<strlen(prefixIn); i++) { (*dataOut)[i] = prefixIn[i]; } // copy prefix into output array (at beginning)
	strcat(*dataOut, postfixIn); // copy postfix into output array
	return true;
}

uint32_t mallocBASE64Encoding(uint8_t *dataIn, uint32_t dataInLen, char **dataOut) {
    size_t lenOfBASE64EncodedData;
    mbedtls_base64_encode(NULL, 0, &lenOfBASE64EncodedData, (unsigned char*) dataIn, dataInLen); // check how long base64 encoded data would be
	*dataOut = (char *) malloc(lenOfBASE64EncodedData); // malloc for whole encoded payload, 0 termination included in lenOfBase64EncodedData
	if(*dataOut == NULL) {
		return 0;
	}
	return ((uint32_t) (lenOfBASE64EncodedData));
}

void BASE64Encoding(uint8_t *dataIn, uint32_t dataInLen, char **dataOut, uint32_t dataOutLenBASE64) {
    mbedtls_base64_encode((unsigned char*) (*dataOut), dataOutLenBASE64, &dataOutLenBASE64, (unsigned char*) dataIn, dataInLen); // encode data and store in dataOut
}

void freeMemoryPOSTData(char *dataOut) {
	free(dataOut);
}

/** ----- NTP Time Synchronisation ----- */

void time_sync_notification_cb(struct timeval *tv) {
    //printf("Notification of a time synchronization event\n");
	ntpSuccess = true;
}

static void ntpTimerCallback(void* arg) {
    ntpTimerFinished = true;
}

bool WildFiTagREV6::getNTPTimestampUTCAndCompareAccuracy(bool storeInRTC, uint32_t &timestampUTC, uint16_t &milliseconds, int64_t &timestampDiffMs, const uint32_t timeoutMs, const char* serverAddr) {
	const uint32_t WAIT_TIME_NTP_DELAY = 10;
	ntpSuccess = false;
	if(wiFiConnected != WIFI_CONNECT_SUCCESS) {
		return false;
	}
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, serverAddr);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

    time_t now = 0;
    struct tm timeinfo = { };
    uint32_t retryCnt = 0;
    const uint32_t retryMax = timeoutMs / WAIT_TIME_NTP_DELAY;

	// for direct storing onto RTC
	tmElements_t timeStruct;
	uint32_t waitTimeUs = 0;
    ntpTimerFinished = false;
    const esp_timer_create_args_t timerArgs = { .callback = &ntpTimerCallback };
    esp_timer_handle_t timer;
	if(storeInRTC) {
		if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { return false; }
	}
    while(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retryCnt < retryMax) {
        delay(WAIT_TIME_NTP_DELAY);
    }
	if(ntpSuccess) {
		time(&now);
		localtime_r(&now, &timeinfo);
		timestampUTC = (uint32_t) now; // now = time_t = 4 Bytes

		// milliseconds
		struct timeval tv;
		gettimeofday(&tv, NULL);
		milliseconds = (uint16_t) (tv.tv_usec / 1000LL);
		
		if(!storeInRTC) {
			return true; // done here already
		}
		else {
			if((timestampUTC > 1600000000) && (milliseconds < 1000)) { // looks like a valid timestamp from NTP
				// start timer
				waitTimeUs = (1000 - milliseconds) * 1000;
				if(waitTimeUs == 0) { waitTimeUs = 1; }
				if(esp_timer_start_once(timer, waitTimeUs) != ESP_OK) { return false; }
				while(!ntpTimerFinished) { ; } // busy waiting

				// BUG before: time api has weekdays from 1 to 7, now changed to 0 to 6, should not happen again (otherwise rtc.set fails, but setTimestamp is executed correctly)
				if(timeStruct.Wday > 6) { timeStruct.Wday = 6; }

				// read time diff
				bool error = false;
				uint32_t oldTimestamp = rtc.getTimestamp(error);
				uint32_t oldMilliseconds = (uint32_t) (rtc.get100thOfSeconds(error));
				oldMilliseconds *= 10;
				// PROBLEM: sometimes millis already 0, but rest not updated
				if(oldMilliseconds == 0) {
					printf("-> WRAP %u <-\n", oldTimestamp);
					oldTimestamp = rtc.getTimestamp(error);
					printf("-> WRAP NEW %u <-\n", oldTimestamp);
				}
				int64_t timestampBeforeMs = oldTimestamp;
				timestampBeforeMs = (timestampBeforeMs * 1000) + oldMilliseconds;
				int64_t timestampNowMs = (timestampUTC + 1);
				timestampNowMs *= 1000;
				
				timestampDiffMs = timestampNowMs - timestampBeforeMs;

				// -- NOW BE FAST, TIME IS CORRECT AT THIS MOMENT --
				breakTime(timestampUTC + 1, timeStruct);
				if(!rtc.set(timeStruct.Hour, timeStruct.Minute, timeStruct.Second, timeStruct.Wday, timeStruct.Day, timeStruct.Month, timeStruct.Year)) { return false; }

				printf("Timestamp: %lld -> %lld\n", timestampBeforeMs, timestampNowMs);
				
				return true;
			}
		}
	}
	return false;
}

bool WildFiTagREV6::getNTPTimestampUTC(bool storeInRTC, uint32_t &timestampUTC, uint16_t &milliseconds, const uint32_t timeoutMs, const char* serverAddr) {
	const uint32_t WAIT_TIME_NTP_DELAY = 10;
	ntpSuccess = false;
	if(wiFiConnected != WIFI_CONNECT_SUCCESS) {
		return false;
	}
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, serverAddr);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

    time_t now = 0;
    struct tm timeinfo = { };
    uint32_t retryCnt = 0;
    const uint32_t retryMax = timeoutMs / WAIT_TIME_NTP_DELAY;

	// for direct storing onto RTC
	tmElements_t timeStruct;
	uint32_t waitTimeUs = 0;
    ntpTimerFinished = false;
    const esp_timer_create_args_t timerArgs = { .callback = &ntpTimerCallback };
    esp_timer_handle_t timer;
	if(storeInRTC) {
		if(esp_timer_create(&timerArgs, &timer) != ESP_OK) { return false; }
	}
    while(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retryCnt < retryMax) {
        delay(WAIT_TIME_NTP_DELAY);
    }
	if(ntpSuccess) {
		time(&now);
		localtime_r(&now, &timeinfo);
		timestampUTC = (uint32_t) now; // now = time_t = 4 Bytes

		// milliseconds
		struct timeval tv;
		gettimeofday(&tv, NULL);
		milliseconds = (uint16_t) (tv.tv_usec / 1000LL);
		
		if(!storeInRTC) {
			return true; // done here already
		}
		else {
			if((timestampUTC > 1600000000) && (milliseconds < 1000)) { // looks like a valid timestamp from NTP
				// start timer
				waitTimeUs = (1000 - milliseconds) * 1000;
				if(waitTimeUs == 0) { waitTimeUs = 1; }
				if(esp_timer_start_once(timer, waitTimeUs) != ESP_OK) { return false; }
				while(!ntpTimerFinished) { ; } // busy waiting

				// BUG before: time api has weekdays from 1 to 7, now changed to 0 to 6, should not happen again (otherwise rtc.set fails, but setTimestamp is executed correctly)
				if(timeStruct.Wday > 6) { timeStruct.Wday = 6; }

				// -- NOW BE FAST, TIME IS CORRECT AT THIS MOMENT --
				breakTime(timestampUTC + 1, timeStruct);
				if(!rtc.set(timeStruct.Hour, timeStruct.Minute, timeStruct.Second, timeStruct.Wday, timeStruct.Day, timeStruct.Month, timeStruct.Year)) { return false; }
				return true;
			}
		}
	}
	return false;
}

/** ----- BLE ----- */

void WildFiTagREV6::printOwnBLEAddress() {
	printf("BLE ADDR: %llX\n", myBLEAddress);
}

static ble_msg_type_t checkIfBiologgerBeacon(uint8_t *dataBeacon, uint8_t len, uint16_t &biologgerId) {
	// --> Example (Android App): 1A FF 4C 00 02 15 - 00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF - 00 02 00 0A BF = 27 Bytes
	// --> Example (ESP32): (02 01 06) 1A FF 4C 00 02 15 - 00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF - 00 02 00 0A BF = 30 Bytes
	// (ESP32) Flags: 02 01 06 (3 Bytes)
	// Adv Length: 1A (1 Byte) = 26 (always with iBeacon)
	// Manufact. Spec. Data: FF (1 Byte)
	// Company ID: 4C 00 (2 Byte) -> little endian = 00 4C (Apple) -> can be changed
	// iBeacon Type: 02 15 (2 Byte) = Apple specific, 02 = iBeacon, 0x15 = 21 = 21 more bytes
	// UUID (Payload, 16 Byte)
	// Major: 00 BIOLOGGER_BLE_BEACON_ADV_MAYOR (2 Byte)
	// Minor: 00 BIOLOGGER_BLE_BEACON_ADV_MINOR (2 Byte)
	// TX Power: BF (1 Byte)
	uint16_t temp;
	if((len == 27)
		&& (dataBeacon[22] == ((BIOLOGGER_BLE_BEACON_ADV_MAYOR >> 8) & 0xFF))
		&& (dataBeacon[23] == ((BIOLOGGER_BLE_BEACON_ADV_MAYOR) & 0xFF))) {
		temp = dataBeacon[24];
		biologgerId = (temp << 8) | dataBeacon[25];
		return BLE_MSG_FROM_ANDROID;
	}
	else if((len == 30)
		&& (dataBeacon[22+3] == ((BIOLOGGER_BLE_BEACON_ADV_MAYOR >> 8) & 0xFF))
		&& (dataBeacon[23+3] == ((BIOLOGGER_BLE_BEACON_ADV_MAYOR) & 0xFF))) {
		temp = dataBeacon[24+3];
		biologgerId = (temp << 8) | dataBeacon[25+3];
		return BLE_MSG_FROM_ESP;
	}
	biologgerId = 0xFFFF;
	return BLE_MSG_DONT_CARE;
}

/*static uint64_t decodeBLEmacAddress(struct ble_gap_event *e) {
	uint64_t temp;
	uint64_t result = 0;
	for(uint8_t i=0; i < 6; i++) { // 6 byte of address, store in uint64_t variable
		temp = e->disc.addr.val[i];
		temp = temp << (i*8);
		result = result | temp;
	}
	return result;
}*/

static void decodeBLEBeacon(struct ble_gap_event *e) {
	// BLE Header (2 Byte)
	// BLE MAC Address (6 Byte) = e->disc.addr.val
	// Flags (3 Byte)
	// CHECK IF BIOLOGGER: Data iBeacon = 27 Byte = e->disc.data[i] (length = e->disc.length_data)
		// --> Example (Android App): 1A FF 4C 00 02 15 - 00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF - 00 02 00 0A BF
		// --> Example (ESP32): (02 01 06) 1A FF 4C 00 02 15 - 00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF - 00 02 00 0A BF
		// { Flags: 02 01 06 (3 Bytes) }
		// Adv Length: 1A (1 Byte) = 26 (always with iBeacon)
		// Manufact. Spec. Data: FF (1 Byte)
		// Company ID: 4C 00 (2 Byte) -> little endian = 00 4C (Apple) -> could be changed in ble_ibeacon.c
		// iBeacon Type: 02 15 (2 Byte) = Apple specific (is iBeacon), 02 = iBeacon, 0x15 = 21 = 21 more bytes
		// UUID (Payload, 16 Byte)
		// Major: 00 02 (2 Byte)
		// Minor: 00 0A (2 Byte)
		// TX Power: BF (1 Byte)
	
	// check if it is a beacon we are interested in
	uint8_t BLE_BEACON_PAYLOAD_OFFSET = 0;
	uint16_t biologgerId = 0;
	ble_msg_type_t msgType = checkIfBiologgerBeacon(e->disc.data, e->disc.length_data, biologgerId);
	if(msgType == BLE_MSG_DONT_CARE) { // if it is not a beacon we are interested in -> skip it!!!
		return;
	}
	else if(msgType == BLE_MSG_FROM_ANDROID) {
		BLE_BEACON_PAYLOAD_OFFSET = 6;
	}
	else if(msgType == BLE_MSG_FROM_ESP) {
		BLE_BEACON_PAYLOAD_OFFSET = 9; // flags are part of payload
	}
	
	// check if already received a beacon from this biologger
	bool found = false;
	uint16_t foundIndex;
	for(uint16_t i = 0; i < bleRXBuffer.biologgerFound; i++) {
		if(bleRXBuffer.biologgerId[i] == biologgerId) {
			found = true;
			foundIndex = i;
			break;
		}
	}
	
	// if already found only increment counter and add RSSI sum 
	if(found) {
		int32_t rssi = e->disc.rssi;
		bleRXBuffer.rssiSumArr[foundIndex] = bleRXBuffer.rssiSumArr[foundIndex] + rssi;
		bleRXBuffer.beaconCnt[foundIndex]++;
	}
	// if first time seen (not found before), add it including payload -> ASSUMING PAYLOAD IS ALWAYS THE SAME!!!
	else {
		if(bleRXBuffer.biologgerFound >= BLE_BEACON_MAX_DEVICES) { // stop adding to RX buffer if received more than 16 biologger beacons
			return;
		}
		bleRXBuffer.rssiSumArr[bleRXBuffer.biologgerFound] = e->disc.rssi; // set rssi of first message (others are added up)
		//bleRXBuffer.macAddressArr[bleRXBuffer.biologgerFound] = mac; // set mac
		bleRXBuffer.beaconCnt[bleRXBuffer.biologgerFound] = 1; // discovered the first beacon
		bleRXBuffer.payloadLen[bleRXBuffer.biologgerFound] = e->disc.length_data;
		bleRXBuffer.biologgerId[bleRXBuffer.biologgerFound] = biologgerId;
		
		// save the actual payload
		for(uint8_t i = 0; i < BLE_BEACON_MAX_DATA; i++) { // extracting 16 byte payload
			bleRXBuffer.payload[bleRXBuffer.biologgerFound][i] = e->disc.data[BLE_BEACON_PAYLOAD_OFFSET + i];
		}

		// increase counter
		bleRXBuffer.biologgerFound++;		
	}
}

static int bleOnGAPEvent(struct ble_gap_event *event, void *arg) {
	// this function gets called each time a beacon is received -> keep it simple!!!
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: // an advertisment report was received during GAP discovery
		decodeBLEBeacon(event);
		//blecent_connect_if_interesting(&event->disc); // try to connect to the advertiser if it looks interesting	
    }
    return 0;
}

static void bleScan() {
    struct ble_gap_disc_params disc_params;
    uint8_t rc;

    disc_params.filter_duplicates = 0; // DO NOT filter duplicates -> we can count beacons and do RSSI averaging
    disc_params.passive = 1; // passive scanning, do not send follow-up scan requests to each advertiser
    disc_params.itvl = 0; // default, scan interval *0.625ms
    disc_params.window = 0; // default, scan window *0.625ms
    disc_params.filter_policy = 0; // default
    disc_params.limited = 0; // default

    rc = ble_gap_disc(ownAddressType, BLE_HS_FOREVER, &disc_params, bleOnGAPEvent, NULL);
    if (rc != 0) {
        printf("Error initiating GAP discovery procedure; rc=%d\n", rc);
    }
}

static void bleAdvertise() {
    struct ble_gap_adv_params adv_params;
    uint8_t rc;
    rc = ble_ibeacon_set_adv_data(bleTXBuffer, BIOLOGGER_BLE_BEACON_ADV_MAYOR, BIOLOGGER_BLE_BEACON_ADV_MINOR, BIOLOGGER_BLE_TX_POWER);
    assert(rc == 0);
	memset(&adv_params, 0, sizeof adv_params); // all 0 = default, no special parameters
    
	adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; // not connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_NON; // not discoverable
	adv_params.itvl_min = 0; // minimum advertising interval, if 0 stack use sane defaults = BLE_GAP_ADV_FAST_INTERVAL2_MIN = 100ms (could be set to BLE_GAP_ADV_FAST_INTERVAL1_MIN = 30ms)
	adv_params.itvl_max = 0; // maximum advertising interval, if 0 stack use sane defaults = BLE_GAP_ADV_FAST_INTERVAL2_MAX = 150ms (could be set to BLE_GAP_ADV_FAST_INTERVAL1_MAX = 60ms)
	adv_params.channel_map = 0; // advertising channel map , if 0 stack use sane defaults
	
    rc = ble_gap_adv_start(ownAddressType, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL); // start advertising forever, no direct address, no event callback because unconnectable
    assert(rc == 0);
}

static void bleOnSync() {
    uint8_t rc;
    rc = ble_hs_id_infer_auto(0, &ownAddressType); // get BLE device address (unique for certain ESP32)
    if (rc != 0) { // error getting address type
        return;
    }
	
	// find out own mac address
	uint8_t ownAddressBuffer[6] = {0};
    ble_hs_id_copy_addr(ownAddressType, ownAddressBuffer, NULL); // copy mac address in array
	myBLEAddress = 0;
	uint64_t temp;
	for(uint8_t i=0; i < 6; i++) { // 6 byte of address, store in uint64_t variable, little endian
		temp = ownAddressBuffer[i];
		temp = temp << (i*8);
		myBLEAddress = myBLEAddress | temp;
	}
	
	// start tasks
    bleAdvertise(); // start advertising (no end)
	if(BLEmode == BLE_ADVERTISE_AND_SCAN) {
		bleScan(); // start scanning at same time (no end)
	}
}

void bleMain(void *param) {
    nimble_port_run(); // this function will return only when nimble_port_stop() is executed
    nimble_port_freertos_deinit();
}

bool WildFiTagREV6::startBLE(ble_mode_t mode, uint8_t *data, uint8_t len) {
	if(len != BLE_BEACON_MAX_DATA) { // data requires always 16 byte
		return false;
	}
	if(!initNVS()) {
		return false;
	}
	for(uint8_t i=0; i < BLE_BEACON_MAX_DATA; i++) { // fill send buffer for beacon
		if(i >= len) {
			bleTXBuffer[i] = BLE_BEACON_FILL_VAL;
		}
		else {
			bleTXBuffer[i] = data[i];
		}
	}
	
	// reset RX buffer
	bleRXBuffer.biologgerFound = 0; // did not found anything yet
	
	uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save WatchDog register
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

    esp_nimble_hci_and_controller_init(); // BROWNOUT if V_BATT < 3.66V -> disabling brownout detector, esp_nimble_hci_and_controller_init needs 500ms!!!

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector again
    
	nimble_port_init();

    ble_hs_cfg.sync_cb = bleOnSync; // callback after sync
	//ble_hs_cfg.reset_cb = blecent_on_reset; // callback on reset
	//ble_hs_cfg.store_status_cb = ble_store_util_status_rr; // callback on ??
	nimble_port_freertos_init(bleMain); // start in own thread
	
	BLEinitialized = true;
	BLEmode = mode;
	return true;
}

void WildFiTagREV6::stopBLE() {
	if(BLEinitialized) {
		uint8_t ret = nimble_port_stop();
        if(ret == 0) {
			nimble_port_deinit();
            ret = esp_nimble_hci_and_controller_deinit();
            if (ret != ESP_OK) {
                printf("DEINIT FAIL\n"); // DEBUG
            }
        }
		BLEinitialized = false;
	}
}

bool WildFiTagREV6::checkBLEOkay(uint16_t index) {
	if((index < bleRXBuffer.biologgerFound) && (BLEmode == BLE_ADVERTISE_AND_SCAN)) {
		return true;
	}
	return false;
}

uint16_t WildFiTagREV6::getBLEBiologgerFound() {
	return bleRXBuffer.biologgerFound;
}

int32_t WildFiTagREV6::getBLERSSISum(uint16_t index) {
	if(checkBLEOkay(index)) {
		return bleRXBuffer.rssiSumArr[index];
	}
	return 0;
}

int8_t WildFiTagREV6::getBLERSSIAverage(uint16_t index) {
	if(checkBLEOkay(index)) {
		int32_t sum = bleRXBuffer.rssiSumArr[index];
		int8_t ret = 0;
		sum = sum / bleRXBuffer.beaconCnt[index];
		ret = (int8_t) sum;
		return ret;
	}
	return 0;
}

uint16_t WildFiTagREV6::getBLEBeaconCnt(uint16_t index) {
	if(checkBLEOkay(index)) {
		return bleRXBuffer.beaconCnt[index];
	}
	return 0;
}

/*uint64_t WildFiTagREV6::getBLEMacAddress(uint16_t index) {
	if(checkBLEOkay(index)) {
		return bleRXBuffer.macAddressArr[index];
	}
	return 0;
}*/

uint8_t WildFiTagREV6::getBLEPayloadLen(uint16_t index) {
	if(checkBLEOkay(index)) {
		return bleRXBuffer.payloadLen[index];
	}
	return 0;
}

uint8_t WildFiTagREV6::getBLEPayload(uint16_t deviceIndex, uint8_t byteIndex) {
	if(checkBLEOkay(deviceIndex)) {
		if(byteIndex < BLE_BEACON_MAX_DATA) {
			return bleRXBuffer.payload[deviceIndex][byteIndex];
		}
	}
	return 0;
}

uint16_t WildFiTagREV6::getBLEBiologgerId(uint16_t index) {
	if(checkBLEOkay(index)) {
		return bleRXBuffer.biologgerId[index];
	}
	return 0;
}

/** ----- MAGNETOMETER CALIBRATION ----- */
bool WildFiTagREV6::magnetometerCalibrationMode(uint16_t durationSeconds, mag_calibration_t *calibrationData, bmm150_trim_registers *trimDataIn) {
	mag_config_t magConf = { BMX160_MAG_ODR_12_5HZ, BMX160_MAG_ACCURACY_REGULAR };
	struct BMX160Data magData;
	int16_t x, y, z;
	uint16_t hallData = 0;
	uint64_t startTime;
	uint64_t limitTime = durationSeconds;
	limitTime *= 1000;
	bool firstSample = true;
	uint64_t sampleCnt = 0;

	if(!imu.start(NULL, &magConf, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { return false; }
	ledGreenOn();
	//setCPUSpeed(ESP32_10MHZ);
	delay(500); // IMPORTANT: otherwise reading schmu data
	startTime = Timing::millis();

	while((Timing::millis() - startTime) < limitTime) {
		while(!imu.magDataReady()) { delay(10); }
		if(!imu.getData(NULL, &magData, NULL, &hallData)) { break; }

		x = imu.magCompensateXandConvertToMicroTesla(magData.x, hallData, trimDataIn);
		y = imu.magCompensateYandConvertToMicroTesla(magData.y, hallData, trimDataIn);
		z = imu.magCompensateZandConvertToMicroTesla(magData.z, hallData, trimDataIn);

		if(firstSample) {
			calibrationData->xMin = x;
			calibrationData->xMax = x;
			calibrationData->yMin = y;
			calibrationData->yMax = y;
			calibrationData->zMin = z;
			calibrationData->zMax = z;
			firstSample = false;
		}
		else {
			if(x < calibrationData->xMin) { calibrationData->xMin = x; }
			if(x > calibrationData->xMax) { calibrationData->xMax = x; }
			if(y < calibrationData->yMin) { calibrationData->yMin = y; }
			if(y > calibrationData->yMax) { calibrationData->yMax = y; }
			if(z < calibrationData->zMin) { calibrationData->zMin = z; }
			if(z > calibrationData->zMax) { calibrationData->zMax = z; }
		}
		sampleCnt++;
		shortLightSleep(100);
	}
	printf("Sample cnt: %llu in %d seconds\n", sampleCnt, durationSeconds);
	//setCPUSpeed(ESP32_80MHZ);
	//delay(100);
	ledGreenOff();
	return true;
}

/** ----- WAKE STUB ----- */
RTC_DATA_ATTR bool (*wakeStubFunctionPointer)();
RTC_DATA_ATTR bool useCustomWakeStubFunction = false;
RTC_DATA_ATTR bool wakeStubNoBootIfVoltageLow = true;
RTC_DATA_ATTR uint8_t interruptToWakeUpAgainWakeStub = USE_EXT0_IF_WAKE_UP_REJECTED;

void WildFiTagREV6::customWakeStubFunction(bool (*f)()) {
	wakeStubFunctionPointer = f;
	useCustomWakeStubFunction = true;
}

void WildFiTagREV6::disableWakeStubNoBootIfVoltageLow() {
	wakeStubNoBootIfVoltageLow = false;
}

void WildFiTagREV6::setWakeStubRejectionInterruptSrc(uint8_t interruptSrc) {
	interruptToWakeUpAgainWakeStub = interruptSrc;
}

RTC_IRAM_ATTR void deepsleep_for_us(uint64_t duration_us) {
    REG_WRITE(TIMG_WDTFEED_REG(0), 1); // feed the watchdog
    uint32_t period = REG_READ(RTC_SLOW_CLK_CAL_REG); // Get RTC calibration
    int64_t sleep_duration = (int64_t)duration_us - (int64_t)DEEP_SLEEP_TIME_OVERHEAD_US; // Calculate sleep duration in microseconds
    if(sleep_duration < 0) {
        sleep_duration = 0;
    }
    int64_t rtc_count_delta = (sleep_duration << RTC_CLK_CAL_FRACT) / period; // Convert microseconds to RTC clock cycles
    REG_WRITE(TIMG_WDTFEED_REG(0), 1); // Feed watchdog
    SET_PERI_REG_MASK(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_UPDATE); // Get current RTC time
    while(GET_PERI_REG_MASK(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_VALID) == 0) {
        ets_delay_us(1);
    }
    SET_PERI_REG_MASK(RTC_CNTL_INT_CLR_REG, RTC_CNTL_TIME_VALID_INT_CLR);
    uint64_t now = READ_PERI_REG(RTC_CNTL_TIME0_REG);
    now |= ((uint64_t) READ_PERI_REG(RTC_CNTL_TIME1_REG)) << 32;
    
    uint64_t future = now + rtc_count_delta; // Set wakeup time
    WRITE_PERI_REG(RTC_CNTL_SLP_TIMER0_REG, future & UINT32_MAX); // Set wakeup time
    WRITE_PERI_REG(RTC_CNTL_SLP_TIMER1_REG, future >> 32); // Set wakeup time
	
	// could also jump to other function here!
	REG_WRITE(RTC_ENTRY_ADDR_REG, (uint32_t)&esp_wake_deep_sleep); // jump into wake up stub again after sleeping (otherwise normal boot WITHOUT stub)
	
    // Start RTC deepsleep timer
    REG_SET_FIELD(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA, RTC_TIMER_TRIG_EN); // Wake up on timer
    WRITE_PERI_REG(RTC_CNTL_SLP_REJECT_CONF_REG, 0); // Clear sleep rejection cause
    // Go to sleep
    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);
    SET_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);
}

void RTC_IRAM_ATTR adc1_get_raw_ram(adc1_channel_t channel) {
    SENS.sar_read_ctrl.sar1_dig_force = 0; // switch SARADC into RTC channel 
    SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PU; // adc_power_on
    RTCIO.hall_sens.xpd_hall = false; //disable other peripherals
    SENS.sar_meas_wait2.force_xpd_amp = SENS_FORCE_XPD_AMP_PD; // channel is set in the convert function
    
	// disable FSM, it's only used by the LNA.
    SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0; 
    SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;
    SENS.sar_meas_wait1.sar_amp_wait1 = 1;
    SENS.sar_meas_wait1.sar_amp_wait2 = 1;
    SENS.sar_meas_wait2.sar_amp_wait3 = 1; 

    //set controller
    SENS.sar_read_ctrl.sar1_dig_force = false; //RTC controller controls the ADC, not digital controller
    SENS.sar_meas_start1.meas1_start_force = true; //RTC controller controls the ADC,not ulp coprocessor
    SENS.sar_meas_start1.sar1_en_pad_force = true; //RTC controller controls the data port, not ulp coprocessor
    SENS.sar_touch_ctrl1.xpd_hall_force = true; // RTC controller controls the hall sensor power,not ulp coprocessor
    SENS.sar_touch_ctrl1.hall_phase_force = true; // RTC controller controls the hall sensor phase,not ulp coprocessor
    
    //start conversion
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); //only one channel is selected.
    while (SENS.sar_slave_addr1.meas_status != 0); // start measurement
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0); // wait until measurement done
    adcValue = SENS.sar_meas_start1.meas1_data_sar; // set adc value!
	
	adcValue = (((adcCoeffA * adcValue) + LIN_COEFF_A_ROUND_ESP) / LIN_COEFF_A_SCALE_ESP) + adcCoeffB; // calculate real V_BATT (with coefficients calculated by compensation algo before deep sleep)
	adcValue = (adcValue * (BATT_VOLT_RES_1 + BATT_VOLT_RES_2)) / BATT_VOLT_RES_2; // scale to V_BATT
	adcValue -= STRANGE_VBATT_OFFSET;

    SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PD; // adc power off
}

void RTC_IRAM_ATTR enter_deepsleep_wait_for_ext0(void) {
	// could also jump to other function here!
	REG_WRITE(RTC_ENTRY_ADDR_REG, (uint32_t)&esp_wake_deep_sleep); // jump into wake up stub again after sleeping
	REG_SET_FIELD(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA, RTC_EXT0_TRIG_EN); // wake up on EXT0 = RTC
	WRITE_PERI_REG(RTC_CNTL_SLP_REJECT_CONF_REG, 0); // clear sleep rejection cause

	// TEST:
	/*
	CLEAR_PERI_REG_MASK(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_DG_PAD_FORCE_ISO | RTC_CNTL_DG_PAD_FORCE_NOISO);
    SET_PERI_REG_MASK(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_DG_WRAP_PD_EN);
    CLEAR_PERI_REG_MASK(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_DG_WRAP_FORCE_PU | RTC_CNTL_DG_WRAP_FORCE_PD);
    CLEAR_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_FORCE_NOSLEEP);
    // Shut down parts of RTC which may have been left enabled by the wireless drivers
    CLEAR_PERI_REG_MASK(RTC_CNTL_ANA_CONF_REG,
	RTC_CNTL_CKGEN_I2C_PU | RTC_CNTL_PLL_I2C_PU | RTC_CNTL_RFRX_PBUS_PU | RTC_CNTL_TXRF_I2C_PU);
	*/
	/*
    PIN_INPUT_ENABLE(GPIO_PIN_REG_2); // enable input
    GPIO.enable_w1ts = (0x1 << 2); // enable output
    GPIO.pin[2].pad_driver = 1; // open drain
    REG_CLR_BIT(GPIO_PIN_REG_2, FUN_PD); // pull down disable
    REG_SET_BIT(GPIO_PIN_REG_2, FUN_PU); // pull up enable
	*/
	//SET_PERI_REG_MASK(RTC_IO_DIG_PAD_HOLD_REG, BIT(12));

	// Go to sleep
	CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);
	SET_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);		
}

void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
	// usable ROM functions: https://github.com/espressif/esp-idf/tree/master/components/esp_rom/include/esp32/rom
    //static RTC_RODATA_ATTR const char str1[] = "WAKE STUB NO: %d\n";
    //ets_printf(str1, wakeCnt++);
	ets_update_cpu_frequency_rom(ets_get_detected_xtal_freq() / 1000000); // IMPORTANT! NECESSARY FOR ACCURATE ets_delay_us, default is 13 = delays are around 3.3 times faster than intended
	adc1_get_raw_ram(ADC1_CHANNEL_7); // calculate raw adc
	bool do_not_boot = false;
	
	// check conditions that might prevent boot
	if((wakeStubNoBootIfVoltageLow)
		&& (adcValue < V_BATT_MINIMUM_FOR_BOOT)) {
		do_not_boot = true;	
	}
	else if(useCustomWakeStubFunction) {
		if(!((*wakeStubFunctionPointer)())) { // call function, see if returns true
			do_not_boot = true;
		}	
	}

	// boot or don't boot
	if(do_not_boot) {
		/*while (REG_GET_FIELD(UART_STATUS_REG(0), UART_ST_UTX_OUT)) { // wait for UART to end transmitting
			;
		}*/
		if(interruptToWakeUpAgainWakeStub == USE_EXT0_IF_WAKE_UP_REJECTED) {
			enter_deepsleep_wait_for_ext0();
		}
		else if(interruptToWakeUpAgainWakeStub == USE_EXT1_IF_WAKE_UP_REJECTED) {
			REG_WRITE(RTC_ENTRY_ADDR_REG, (uint32_t)&esp_wake_deep_sleep); // jump into wake up stub again after sleeping
			REG_SET_FIELD(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA, RTC_EXT1_TRIG_EN); // wake up on EXT0 = RTC
			WRITE_PERI_REG(RTC_CNTL_SLP_REJECT_CONF_REG, 0); // clear sleep rejection cause
			// Go to sleep
			CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);
			SET_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);
		}
		//deepsleep_for_us(10 * 1000 * 1000); // 10 seconds
		// CODE CONTINUES HERE AFTERWARDS! (after sleep, if RTC_ENTRY_ADDR_REG not set)
	}
	else {
		esp_default_wake_deep_sleep(); // wake up, little susie, wake up!
		return;	
	}
	ets_delay_us(10000); // debounce, 10ms, guess not needed
}

