#include "PlatformAttitudeEstimator.h"

PlatformAttitudeEstimator::PlatformAttitudeEstimator() {

}

double PlatformAttitudeEstimator::accRawToG(int16_t input, uint8_t range) {
	double multiplicator = 0.0;
	if(range == BMX160_ACCEL_RANGE_2G) { multiplicator = 0.00006103515; }
	else if(range == BMX160_ACCEL_RANGE_4G) { multiplicator = 0.00012207031; }
	else if(range == BMX160_ACCEL_RANGE_8G) { multiplicator = 0.00024414062; }
	else if(range == BMX160_ACCEL_RANGE_16G) { multiplicator = 0.00048828125; }
	return (input * multiplicator);
}

double PlatformAttitudeEstimator::gyroRawToDegreePerSecond(int16_t input, uint8_t range) {
	double multiplicator = 0.0;
	if(range == BMX160_GYRO_RANGE_2000_DPS) { multiplicator = 0.061; }
	else if(range == BMX160_GYRO_RANGE_1000_DPS) { multiplicator = 0.0305; }
	else if(range == BMX160_GYRO_RANGE_500_DPS) { multiplicator = 0.0153; }
	else if(range == BMX160_GYRO_RANGE_250_DPS) { multiplicator = 0.0076; }
	else if(range == BMX160_GYRO_RANGE_125_DPS) { multiplicator = 0.0038; }
	return (input * multiplicator);
}

double PlatformAttitudeEstimator::uTToGauss(double in) {
	return in / 100.;
}

double PlatformAttitudeEstimator::degPerSecondToRadPerSecond(double in) {
	return in * 0.01745329252; // pi / 180°
}

double PlatformAttitudeEstimator::radToDeg(double in) {
	return in * (180. / M_PI); // pi / 180°
}

double PlatformAttitudeEstimator::GiToMeterPerSecondSquare(double in) {
	return in * 9.80664999999998;
}

bool PlatformAttitudeEstimator::feed(uint8_t *fifoData, uint16_t fifoLen, uint16_t imuFrequency, uint8_t accelRange, uint8_t gyroRange, bool setMagnetometerValuesToZero, int16_t magHardIronOffsetX, int16_t magHardIronOffsetY, int16_t magHardIronOffsetZ) {
    bmx160_fifo_dataset_len_t datasetLen = BMX160_FIFO_DATASET_LEN_ACC_AND_MAG_AND_GYRO;
	uint16_t iterator = 0;
	int16_t accX = 0, accY = 0, accZ = 0, magXComp = 0, magYComp = 0, magZComp = 0, gyroX = 0, gyroY = 0, gyroZ = 0;
    double fAccX, fAccY, fAccZ, fMagX, fMagY, fMagZ, fGyroX, fGyroY, fGyroZ;
	uint16_t hall = 0;
    double timeDiff = imuFrequency;
    uint16_t datasetCounter = 0;
    timeDiff = 1. / timeDiff;
	if(fifoLen < datasetLen) { return false; }
    if(fifoLen % datasetLen != 0) { return false; }
	while(true) {
		if(iterator + datasetLen > fifoLen) { break; }

        magXComp = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
        magYComp = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
        magZComp = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
        hall = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
        gyroX = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
        gyroY = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
        gyroZ = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
        accX = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
        accY = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;
        accZ = (int16_t) ((fifoData[iterator+1] << 8) | fifoData[iterator]); iterator += 2;

        // substract hard iron offset if not zero
        magXComp -= magHardIronOffsetX;
        magYComp -= magHardIronOffsetY;
        magZComp -= magHardIronOffsetZ;

        // normally done in post processing (by decoder software)
		fAccX = accRawToG(accX, accelRange);
		fAccY = accRawToG(accY, accelRange);
		fAccZ = accRawToG(accZ, accelRange);
		fGyroX = gyroRawToDegreePerSecond(gyroX, gyroRange);
		fGyroY = gyroRawToDegreePerSecond(gyroY, gyroRange);
		fGyroZ = gyroRawToDegreePerSecond(gyroZ, gyroRange);
        fMagX = magXComp;
        fMagY = magYComp;
        fMagZ = magZComp;

        printf("%d MAG uT: %.1f %.1f %.1f\n", datasetCounter, fMagX, fMagY, fMagZ);

        // important data translation for library
        fAccX = GiToMeterPerSecondSquare(fAccX);
        fAccY = GiToMeterPerSecondSquare(fAccY);
        fAccZ = GiToMeterPerSecondSquare(fAccZ);
        fMagX = uTToGauss(fMagX);
        fMagY = uTToGauss(fMagY);
        fMagZ = uTToGauss(fMagZ);
        fGyroX = degPerSecondToRadPerSecond(fGyroX);
        fGyroY = degPerSecondToRadPerSecond(fGyroY);
        fGyroZ = degPerSecondToRadPerSecond(fGyroZ);

        if(setMagnetometerValuesToZero) {
            fMagX = 0.;
            fMagY = 0.;
            fMagZ = 0.;
        }

        est.update(timeDiff, fGyroX, fGyroY, fGyroZ, fAccX, fAccY, fAccZ, fMagX, fMagY, fMagZ);

        // for printing
        double q[4];
        est.getAttitude(q);
        //printf("My attitude is (quaternion): %f, %f, %f\n", q[0], q[1], q[2], q[3]);
        //printf("My attitude is (ZYX Euler rad): y %f, p %f, r %f\n", est.eulerYaw(), est.eulerPitch(), est.eulerRoll());
        printf("%d ZYX Euler deg: y %.2f°, p %.2f°, r %.2f°\n", datasetCounter, radToDeg(est.eulerYaw()), radToDeg(est.eulerPitch()), radToDeg(est.eulerRoll()));
        datasetCounter++;
	}


    return true;
}