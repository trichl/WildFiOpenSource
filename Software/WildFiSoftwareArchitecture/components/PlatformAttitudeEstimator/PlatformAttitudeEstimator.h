#ifndef PlatformAttitudeEstimator_h
#define PlatformAttitudeEstimator_h

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "attitude_estimator.h"
#include "ModuleIMU_BMX160.h"

class PlatformAttitudeEstimator {
	public:
		stateestimation::AttitudeEstimator est = stateestimation::AttitudeEstimator();
		PlatformAttitudeEstimator();

		double accRawToG(int16_t input, uint8_t range);
		double gyroRawToDegreePerSecond(int16_t input, uint8_t range);
		double uTToGauss(double in);
		double degPerSecondToRadPerSecond(double in);
		double radToDeg(double in);
		double GiToMeterPerSecondSquare(double in);

		bool feed(uint8_t *fifoData, uint16_t fifoLen, uint16_t imuFrequency, uint8_t accelRange, uint8_t gyroRange, bool setMagnetometerValuesToZero, int16_t magHardIronOffsetX, int16_t magHardIronOffsetY, int16_t magHardIronOffsetZ, bool resetAll, uint8_t debugLevel); // needs fifo with 9 axis data

};

#endif
