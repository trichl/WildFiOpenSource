#ifndef ModuleIMU_BMX160_h
#define ModuleIMU_BMX160_h

// TODO: Absolute orientation berechnen -> NICHT MACHBAR MIT ATTINY!
// TODO: Test new implementation enableAccStepCounter
// TODO: Mutterklasse BioLoggerIMU mit Logik-Ops auf Daten? -> ne, in Application Layer ziehen (z.B. Absolutwert IMU Daten berechnen)
// (TODO: tap interrupt)
// (TODO: Freefall interrupt)
// (TODO: High-G interrupt)
// TODO: magSetMode: CHECK PMU_STATUS REGISTER LIKE IN accSetMode
// TODO: test accSetMode
// TODO: current mode change (e.g. FIFO readout): always waiting at the end until back in low power (bool variable makes sense like in IMU_BMX160_WAKESTUB)

#include <stdint.h>
#include <math.h> // for pow
#include "InterfaceI2C.h"
#include "InterfaceTiming.h"
	
#define IMU_BMX160_ADDRESS                       0x69

/** bmx160 Register map */
#define BMX160_ACCEL_CONFIG_ADDR                 0x40
#define BMX160_ACCEL_RANGE_ADDR                  0x41
#define BMX160_COMMAND_REG_ADDR                  0x7E
#define BMX160_MAG_DATA_ADDR                     0x04
#define BMX160_GYRO_DATA_ADDR                    0x0C
#define BMX160_ACCEL_DATA_ADDR                   0x12
#define BMX160_MAGN_CONFIG_ADDR                  0x44
#define BMX160_MAGN_RANGE_ADDR                   0x4B
#define BMX160_MAGN_IF_0_ADDR                    0x4C
#define BMX160_MAGN_IF_1_ADDR                    0x4D
#define BMX160_MAGN_IF_2_ADDR                    0x4E
#define BMX160_MAGN_IF_3_ADDR                    0x4F
#define BMX160_TEMP_0_ADDR                       0x20
#define BMX160_TEMP_1_ADDR                       0x21
#define BMX160_INT_ENABLE_0_ADDR                 0x50
#define BMX160_INT_ENABLE_1_ADDR                 0x51
#define BMX160_INT_ENABLE_2_ADDR                 0x52
#define BMX160_INT_MAP_0_ADDR                    0x55
#define BMX160_INT_MAP_1_ADDR                    0x56
#define BMX160_INT_OUT_CTRL_ADDR                 0x53
#define BMX160_INT_MOTION_0_ADDR                 0x5F
#define BMX160_INT_MOTION_1_ADDR                 0x60
#define BMX160_INT_MOTION_2_ADDR                 0x61
#define BMX160_INT_MOTION_3_ADDR                 0x62
#define BMX160_STEP_CONF_0_ADDR                  0x7A
#define BMX160_STEP_CONF_1_ADDR                  0x7B
#define BMX160_STEP_CNT_0_ADDR                   0x78
#define BMX160_STEP_CNT_1_ADDR                   0x79
#define BMX160_INT_STATUS_ADDR                   0x1C
#define BMX160_INT_FLAT_0_ADDR                   0x67
#define BMX160_INT_FLAT_1_ADDR                   0x68
#define BMX160_INT_LATCH_ADDR                    0x54
#define BMX160_INT_ORIENT_0_ADDR                 0x65
#define BMX160_INT_ORIENT_1_ADDR                 0x66
#define BMX160_FOC_CONF_ADDR                     0x69
#define BMX160_STATUS_ADDR                       0x1B
#define BMX160_FIFO_LENGTH_ADDR                  0x22
#define BMX160_FIFO_DATA_ADDR                    0x24
#define BMX160_FIFO_DOWN_ADDR                    0x45
#define BMX160_FIFO_CONFIG_0_ADDR                0x46
#define BMX160_FIFO_CONFIG_1_ADDR                0x47
#define BMX160_PMU_STATUS_ADDR                   0x03
#define BMX160_OFFSET_6			                 0x77
#define BMX160_CONF								 0x6A
#define BMX160_GYRO_CONFIG_ADDR            		 0x42
#define BMX160_GYRO_RANGE_ADDR           		 0x43

/** Status */
#define BMX160_STATUS_DATARDY_ACC_BITPOS       	 7
#define BMX160_STATUS_DATARDY_GYRO_BITPOS        6
#define BMX160_STATUS_DATARDY_MAG_BITPOS       	 5

/** FOC */
#define BMX160_START_FOC_CMD                     0x03

/** NVM */
#define BMX160_START_NVM_WRITE_CMD               0xA0

/** FIFO init */
#define BMX160_INIT_FIFO_FOR_ACC           		0b01000000
#define BMX160_INIT_FIFO_FOR_MAG           		0b00100000
#define BMX160_INIT_FIFO_FOR_GYRO           	0b10000000

/** Orientation interrupt configuration */
#define BMX160_INT_STATUS_ORIENTATION_PORT_UP    0x00
#define BMX160_INT_STATUS_ORIENTATION_PORT_DOWN   0x01
#define BMX160_INT_STATUS_ORIENTATION_LAND_LEFT  0x02
#define BMX160_INT_STATUS_ORIENTATION_LAND_RIGHT 0x03

/** AnyMotion interrupt configuration */
#define BMX160_INT_STATUS_ANYMOT_AXIS_MSK        0x07
#define BMX160_INT_STATUS_ANYMOT_WAS_X           0x01
#define BMX160_INT_STATUS_ANYMOT_WAS_Y           0x02
#define BMX160_INT_STATUS_ANYMOT_WAS_Z           0x04

/** StepCounter configuration (from Internet) */
#define BMX160_STEP_CONF_0_MIN_THRESH_0        0x00
#define BMX160_STEP_CONF_0_MIN_THRESH_1        0x08
#define BMX160_STEP_CONF_0_MIN_THRESH_2        0x10
#define BMX160_STEP_CONF_0_MIN_THRESH_3        0x18
#define BMX160_STEP_CONF_0_MIN_THRESH_3        0x18
#define BMX160_STEP_CONF_0_MIN_THRESH_5        0x28

#define BMX160_STEP_CONF_0_STEPTIME_MIN_0      0x00
#define BMX160_STEP_CONF_0_STEPTIME_MIN_1      0x01
#define BMX160_STEP_CONF_0_STEPTIME_MIN_2      0x02
#define BMX160_STEP_CONF_0_STEPTIME_MIN_3      0x03
#define BMX160_STEP_CONF_0_STEPTIME_MIN_4      0x04
#define BMX160_STEP_CONF_0_STEPTIME_MIN_5      0x05
#define BMX160_STEP_CONF_0_STEPTIME_MIN_6      0x06
#define BMX160_STEP_CONF_0_STEPTIME_MIN_7      0x07

#define BMX160_STEP_CONF_1_MIN_BUF_0           0x00
#define BMX160_STEP_CONF_1_MIN_BUF_1           0x01
#define BMX160_STEP_CONF_1_MIN_BUF_2           0x02
#define BMX160_STEP_CONF_1_MIN_BUF_3           0x03
#define BMX160_STEP_CONF_1_MIN_BUF_4           0x04
#define BMX160_STEP_CONF_1_MIN_BUF_5           0x05
#define BMX160_STEP_CONF_1_MIN_BUF_6           0x06
#define BMX160_STEP_CONF_1_MIN_BUF_7           0x07

/** ACC configuration */
typedef struct {
	uint8_t frequency;
	uint8_t averaging; // recommendation: BMX160_ACCEL_BW_RES_AVG8
	uint8_t range; // default: BMX160_ACCEL_RANGE_2G
} acc_config_t;

#define BMX160_ACCEL_UNDERSAMPLING_MASK     	 0x80

#define BMX160_ACCEL_BW_RES_AVG1          		 0x00
#define BMX160_ACCEL_BW_RES_AVG2          		 0x01
#define BMX160_ACCEL_BW_RES_AVG4          		 0x02
#define BMX160_ACCEL_BW_RES_AVG8          		 0x03
#define BMX160_ACCEL_BW_RES_AVG16          		 0x04

#define BMX160_ACCEL_RANGE_2G                  	 0b0011
#define BMX160_ACCEL_RANGE_4G                  	 0b0101
#define BMX160_ACCEL_RANGE_8G                  	 0b1000
#define BMX160_ACCEL_RANGE_16G                   0b1100

#define BMX160_ACCEL_ODR_0_78HZ                  0x01 // 6.1uA average in sleep mode, writing in FIFO (only ACC)
#define BMX160_ACCEL_ODR_1_56HZ                  0x02 // 6.9uA average in sleep mode, writing in FIFO (only ACC)
#define BMX160_ACCEL_ODR_3_12HZ                  0x03
#define BMX160_ACCEL_ODR_6_25HZ                  0x04
#define BMX160_ACCEL_ODR_12_5HZ                  0x05
#define BMX160_ACCEL_ODR_25HZ                    0x06
#define BMX160_ACCEL_ODR_50HZ                    0x07
#define BMX160_ACCEL_ODR_100HZ                   0x08
#define BMX160_ACCEL_ODR_200HZ                   0x09
#define BMX160_ACCEL_ODR_400HZ                   0x0A
#define BMX160_ACCEL_ODR_800HZ                   0x0B
#define BMX160_ACCEL_ODR_1600HZ                  0x0C

#define BMX160_LATCH_DUR_NONE                    0x00
#define BMX160_LATCH_DUR_312_5_MICRO_SEC         0x01
#define BMX160_LATCH_DUR_625_MICRO_SEC           0x02
#define BMX160_LATCH_DUR_1_25_MILLI_SEC          0x03
#define BMX160_LATCH_DUR_2_5_MILLI_SEC           0x04
#define BMX160_LATCH_DUR_5_MILLI_SEC             0x05
#define BMX160_LATCH_DUR_10_MILLI_SEC            0x06
#define BMX160_LATCH_DUR_20_MILLI_SEC            0x07
#define BMX160_LATCH_DUR_40_MILLI_SEC            0x08
#define BMX160_LATCH_DUR_80_MILLI_SEC            0x09
#define BMX160_LATCH_DUR_160_MILLI_SEC           0x0A
#define BMX160_LATCH_DUR_320_MILLI_SEC           0x0B
#define BMX160_LATCH_DUR_640_MILLI_SEC           0x0C
#define BMX160_LATCH_DUR_1_28_SEC                0x0D
#define BMX160_LATCH_DUR_2_56_SEC                0x0E
#define BMX160_LATCHED                           0x0F

/** MAG configuration */
typedef struct {
	uint8_t frequency;
	uint8_t accuracy; // default: BMX160_MAG_ACCURACY_LOW_POWER
} mag_config_t;											   

#define BMX160_MAG_ODR_0_78HZ                  	 0x01
#define BMX160_MAG_ODR_1_56HZ                    0x02
#define BMX160_MAG_ODR_3_12HZ                    0x03
#define BMX160_MAG_ODR_6_25HZ                    0x04
#define BMX160_MAG_ODR_12_5HZ                    0x05
#define BMX160_MAG_ODR_25HZ                      0x06
#define BMX160_MAG_ODR_50HZ                      0x07

#define BMX160_MAG_ACCURACY_LOW_POWER         	 0x00 // noise: 1.0uT on xy, 1.5uT on z
#define BMX160_MAG_ACCURACY_REGULAR            	 0x01 // noise: 0.6uT on xy, 0.6uT on z
#define BMX160_MAG_ACCURACY_ENHANCED           	 0x02 // less noise
#define BMX160_MAG_ACCURACY_HIGH               	 0x03 // even less noise

#define BMX160_MAG_ACCURACY_LOW_POWER_XY		 0x01 // 3 repetitions
#define BMX160_MAG_ACCURACY_LOW_POWER_Z			 0x02 // 3 repetitions
#define BMX160_MAG_ACCURACY_REGULAR_XY			 0x04 // 9 repetitions
#define BMX160_MAG_ACCURACY_REGULAR_Z			 0x0E // 15 repetitions
#define BMX160_MAG_ACCURACY_ENHANCED_XY			 0x07 // even more repetitions
#define BMX160_MAG_ACCURACY_ENHANCED_Z			 0x1A // even more repetitions
#define BMX160_MAG_ACCURACY_HIGH_XY				 0x17 // even more repetitions
#define BMX160_MAG_ACCURACY_HIGH_Z				 0x52 // even more repetitions

/** GYRO configuration */
typedef struct {
	uint8_t frequency;
	uint8_t range;
	uint8_t bw;
} gyro_config_t;

#define BMX160_GYRO_ODR_RESERVED             	 0x00
#define BMX160_GYRO_ODR_25HZ                 	 0x06
#define BMX160_GYRO_ODR_50HZ                 	 0x07
#define BMX160_GYRO_ODR_100HZ                	 0x08
#define BMX160_GYRO_ODR_200HZ                	 0x09
#define BMX160_GYRO_ODR_400HZ                	 0x0A
#define BMX160_GYRO_ODR_800HZ                	 0x0B
#define BMX160_GYRO_ODR_1600HZ               	 0x0C
#define BMX160_GYRO_ODR_3200HZ               	 0x0D

#define BMX160_GYRO_RANGE_2000_DPS           	 0x00
#define BMX160_GYRO_RANGE_1000_DPS          	 0x01
#define BMX160_GYRO_RANGE_500_DPS            	 0x02
#define BMX160_GYRO_RANGE_250_DPS            	 0x03
#define BMX160_GYRO_RANGE_125_DPS            	 0x04

#define BMX160_GYRO_BW_OSR4_MODE             	 0x00
#define BMX160_GYRO_BW_OSR2_MODE             	 0x01
#define BMX160_GYRO_BW_NORMAL_MODE           	 0x02

/** Power mode settings */
/* Accel power mode */
#define BMX160_ACCEL_NORMAL_MODE                 0x11
#define BMX160_ACCEL_LOWPOWER_MODE               0x12
#define BMX160_ACCEL_SUSPEND_MODE                0x10

/* Gyro power mode */
#define BMX160_GYRO_SUSPEND_MODE                 0x14
#define BMX160_GYRO_NORMAL_MODE                  0x15
#define BMX160_GYRO_FASTSTARTUP_MODE             0x17

/* Magn power mode */
#define BMX160_MAGN_SUSPEND_MODE                 0x18
#define BMX160_MAGN_NORMAL_MODE                  0x19
#define BMX160_MAGN_LOWPOWER_MODE                0x1A

/** Possible interrupts for containsInterrupt() */
#define BMX160_INT_ANYMOT                        0x0004
#define BMX160_INT_FLAT                          0x0080
#define BMX160_INT_ORIENT                        0x0040
#define BMX160_INT_NOMOT                         0x8000

/** FIFO elements */
#define BMX160_FIFO_BURST_READ_FOR_6_BYTE        (I2C_ARDUINO_BUFFER_LIMIT - (I2C_ARDUINO_BUFFER_LIMIT % 6)) // needs to fit into FIFO frame of 6 bytes, otherwise retransmission
#define BMX160_FIFO_ACC_ELEMENT_X                0
#define BMX160_FIFO_ACC_ELEMENT_Y                1
#define BMX160_FIFO_ACC_ELEMENT_Z                2

/** Soft reset command */
#define BMX160_SOFT_RESET_CMD                    0xB6
#define BMX160_SOFT_RESET_DELAY_MS               100 // DFRobot: 15ms, be more safe here

/* For indirect read of integrated MAG: Trim Extended Registers */
#define BMM150_DIG_X1                             0x5D
#define BMM150_DIG_Y1                             0x5E
#define BMM150_DIG_Z4_LSB                         0x62
#define BMM150_DIG_Z4_MSB                         0x63
#define BMM150_DIG_X2                             0x64
#define BMM150_DIG_Y2                             0x65
#define BMM150_DIG_Z2_LSB                         0x68
#define BMM150_DIG_Z2_MSB                         0x69
#define BMM150_DIG_Z1_LSB                         0x6A
#define BMM150_DIG_Z1_MSB                         0x6B
#define BMM150_DIG_XYZ1_LSB                       0x6C
#define BMM150_DIG_XYZ1_MSB                       0x6D
#define BMM150_DIG_Z3_LSB                         0x6E
#define BMM150_DIG_Z3_MSB                         0x6F
#define BMM150_DIG_XY2                            0x70
#define BMM150_DIG_XY1                            0x71

/* Mag masks */
#define BMM150_GET_BITS(reg_data, bitname)    	  ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))	
#define BMM150_DATA_X_MSK                         UINT8_C(0xF8)
#define BMM150_DATA_X_POS                         UINT8_C(0x03)
#define BMM150_DATA_Y_MSK                         UINT8_C(0xF8)
#define BMM150_DATA_Y_POS                         UINT8_C(0x03)
#define BMM150_DATA_Z_MSK                         UINT8_C(0xFE)
#define BMM150_DATA_Z_POS                         UINT8_C(0x01)
#define BMM150_DATA_RHALL_MSK                     UINT8_C(0xFC)
#define BMM150_DATA_RHALL_POS                     UINT8_C(0x02)

/* MAG: compensation */
#define BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP        -4096
#define BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL         -16384
#define BMM150_OVERFLOW_OUTPUT                    -32768
#define BMM150_NEGATIVE_SATURATION_Z              -32767
#define BMM150_POSITIVE_SATURATION_Z              32767

struct bmm150_trim_registers {
    int8_t dig_x1;
    int8_t dig_y1;
    int8_t dig_x2;
    int8_t dig_y2;
    uint16_t dig_z1;
    int16_t dig_z2;
    int16_t dig_z3;
    int16_t dig_z4;
    uint8_t dig_xy1;
    int8_t dig_xy2;
    uint16_t dig_xyz1;
}; // structure to store the trimming data for compensation of mag

struct BMX160Data {
	int16_t x;
	int16_t y;
	int16_t z;
};

typedef enum {
	BMX160_FIFO_DATASET_LEN_ACC_AND_MAG = 14,        // 6 byte mag, 2 byte rhall, 6 byte acc
	BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO = 20        // 6 byte mag, 2 byte rhall, 6 byte acc, 6 byte gyro
} bmx160_fifo_dataset_len_t;

class IMU_BMX160 {
	public:
		IMU_BMX160();
		bool start(acc_config_t *accConf, mag_config_t *magConf, gyro_config_t *gyroConf, uint8_t interruptLatch); // BEFORE CALLING: WAIT SOME TIME AFTER POWER UP UNTIL BMX IS READY!
		bool stop(); // WARNING: does not stop MAG if it was activated before! -> should use softReset
		bool softReset(bool magWasRunning); // for stopping everything and start from scratch
		bool getData(struct BMX160Data *accel, struct BMX160Data *magn, struct BMX160Data *gyro, uint16_t *hall);
		bool getTemperatureRaw(uint16_t &tempRaw);
		float accRawToG(int16_t input, uint8_t range);
		float gyroRawToDegreePerSecond(int16_t input, uint8_t range);
		//static int16_t toMS2x1000(int16_t rawData, uint16_t range = BMX160_ACCEL_MG_AT_2G); // for acc data, returns m/s^2 x1000 (9.81 = 9800) -> loosing resolution! better keep original 16 bit
		//static int16_t toMicroTeslaXY(int16_t rawData); // NOT CORRECT according to Bosch, no linear formular for raw data to uT
		//static int16_t toMicroTeslaZ(int16_t rawData); // NOT CORRECT according to Bosch, no linear formular for raw data to uT
		static int16_t toCelsiusx100(uint16_t rawData); // for temperature data, C x100 (21° = 2100)
		
		// Polling for getData (not really necessary, driver from Bosch doesn't do it)
		bool accDataReady();
		bool magDataReady();
		bool gyroDataReady();

		// Calibration
		bool accGyroFOCAlreadyDoneAndStoredInNVM(); // check this always before writing!
		bool startAccGyroFOCAndStoreInNVM(bool forceMode);
		//bool startAccFOCAndStoreInNVM(); // WARNING: max 14 times! -> DEVICE NEEDS TO BE FLAT ON TABLE! Z-Axis = +1g
		//bool accFOCAlreadyDoneAndStoredInNVM(); // check this always before writing!
		

		// Mag read trim registers (for temperature compensated AND not temperature compensated values)
		bool magCompensateReadTrimData(bmm150_trim_registers *trimData); // do this BEFORE calling start, but AFTER some time after powering device up! only once, store trimData in RTC or memory
		void magCompensatePrintTrimData(bmm150_trim_registers *trimData);

		// Mag NOT temperature compensated data (according to Bosch: raw values cannot be converted straight to uT - these functions are needed)
		int16_t magXConvertToMicroTesla(int16_t mag_data_x, bmm150_trim_registers *trimData);
		int16_t magYConvertToMicroTesla(int16_t mag_data_y, bmm150_trim_registers *trimData);
		int16_t magZConvertToMicroTesla(int16_t mag_data_z, bmm150_trim_registers *trimData);		

		// Mag temperature compensation (based on the measured RESISTANCE of the hall sensor = rhall)
		int16_t magCompensateXandConvertToMicroTesla(int16_t mag_data_x, uint16_t data_rhall, bmm150_trim_registers *trimData);
		int16_t magCompensateYandConvertToMicroTesla(int16_t mag_data_y, uint16_t data_rhall, bmm150_trim_registers *trimData);
		int16_t magCompensateZandConvertToMicroTesla(int16_t mag_data_z, uint16_t data_rhall, bmm150_trim_registers *trimData);

		bool magCompensateFifoData(uint8_t *fifoData, uint16_t fifoLen, bmx160_fifo_dataset_len_t datasetLen, bmm150_trim_registers *trimDataIn);
		bool printFifoData(uint8_t *fifoData, uint16_t fifoLen, bmx160_fifo_dataset_len_t datasetLen, uint8_t accelRange, uint8_t gyroRange); // call magCompensateFifoData before if mag data included
		
		// FIFO
		bool initFIFO(uint8_t forWhat);
		bool resetFIFO();
		uint16_t getFIFOLength();
		bool readAccFIFO(uint8_t fifoData[], uint16_t len); // MAX 32 BYTE I2C MESSAGES!!! around 350ms if FIFO is full, 0x80 if end is reached
		uint16_t readAccFIFOInOneGo(uint8_t fifoData[]); // 0x80 if end is reached, pass 1024 byte array!
		bool readGeneralFIFOInOneGoFast(bool accOn, bool magOn, bool gyroOn, uint8_t fifoData[], uint16_t currentFifoLen, bool waitUntilBackInLowPower); 
		bool readAccFIFOInOneGoFast(uint8_t fifoData[], uint16_t currentFifoLen, bool waitUntilBackInLowPower); // no delays, option to not wait until IMU back in low power mode, pass current fifo length directly
		static uint8_t countAccFIFOLogs(uint16_t len); // was static, but ESP32 compiler doesn't like it!
		static int16_t getAccFIFOLog(uint8_t fifoData[], uint16_t len, uint16_t logNumber, uint8_t xyz); // was static, but ESP32 compiler doesn't like it!
		
		// FIFO interrupt
		bool enableFIFOInterrupt(uint16_t watermark);

		// Step counter (35uA@25Hz if MCU sleeps, only works with 25Hz or I guess higher)
		bool enableAccStepCounter(uint8_t minThreshold, uint8_t minStepTime, uint8_t minBuffer); // doesn't work with 6.25Hz, but with 25Hz
		bool enableAccStepCounterNormalMode();
		bool enableAccStepCounterSensitiveMode();
		bool enableAccStepCounterRobustMode();
		uint16_t getStepCounter();
		bool resetStepCounter();
		
		// AnyMotion interrupt (35uA@25Hz if MCU sleeps, 14.5uA@6.25Hz)
		bool enableAccAnyMotionInterruptXYZ(uint8_t duration, uint8_t threshold); // TRESHOLD DEPENDS ON RANGE (2G, 4G, ..), works in ACC = low power, rest in suspend
		uint8_t anyMotionInterruptAxis(); // returns 0 for no or either BMX160_INT_STATUS_ANYMOT_WAS_X, BMX160_INT_STATUS_ANYMOT_WAS_Y or BMX160_INT_STATUS_ANYMOT_WAS_Z
	
		// Flat interrupt
		bool enableAccFlatDetectionInterrupt(uint8_t flatTheta = 8, uint8_t flatHoldTime = 1, uint8_t flatHysteresis = 1); // works in ACC = low power, rest in suspend
		bool isFlat(); // only works if flat detection interrupt is enabled
		bool isOnDownside(); // only works if flat detection interrupt is enabled
		
		// Orientation interrupt
		bool enableAccOrientationInterrupt(uint8_t orientHysteresis = 1, uint8_t orientBlocking = 2);
		uint8_t getOrientation(); // BMX160_INT_STATUS_ORIENTATION_PORT_UP, BMX160_INT_STATUS_ORIENTATION_PORT_DWN, BMX160_INT_STATUS_ORIENTATION_LAND_LEFT, BMX160_INT_STATUS_ORIENTATION_LAND_RIGHT
		
		// NoMotion interrupt, 12.5uA@6.25Hz
		bool enableAccNoMotionInterruptXYZ(uint8_t duration, uint8_t threshold); // TRESHOLD DEPENDS ON RANGE (2G, 4G, ..)
		
		// Interrupt reasons
		uint16_t getInterruptReasons();
		bool containsInterrupt(uint16_t reasons, uint16_t interruptMask);
	
		// internal functions
		bool magSetMode(uint8_t mode);
		bool setupMag(uint8_t odr, uint8_t accuracy);
		bool stopMag();
		bool waitOnIndirectMagRead();
		bool accSetMode(uint8_t mode);
		bool gyroSetMode(uint8_t mode);	
};

#endif
