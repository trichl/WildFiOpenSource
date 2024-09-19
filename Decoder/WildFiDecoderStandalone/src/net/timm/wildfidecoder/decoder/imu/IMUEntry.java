package net.timm.wildfidecoder.decoder.imu;

import java.util.Formatter;
import java.util.Locale;

public class IMUEntry {
    public long calculatedTimestampEstimation, consecutiveNumber;

    public double accXinG, accYinG, accZinG;
    public double magXinUT, magYinUT, magZinUT;
    public long hall;
    public double gyroXinDPS, gyroYinDPS, gyroZinDPS;
		public double accConvFact, magConvFact, gyroConvFact;

    public boolean hasAccData = false, hasMagData = false, hasGyroData = false;

    public double accCalculatedMagnitudeWithoutEarthGravitation;

    public IMUEntry(boolean hasAccDataIn, boolean hasMagDataIn, boolean hasGyroDataIn) {
        hasAccData = hasAccDataIn;
        hasMagData = hasMagDataIn;
        hasGyroData = hasGyroDataIn;
    }

    public static String serializeHeadline(boolean hasAccData, boolean hasMagData, boolean hasGyroData, boolean useBurstForm) {
		if (useBurstForm) {
			if (hasAccData && !hasMagData && !hasGyroData)
				return "consecutiveNumberBurst,accInGBurst,accConvFact,imuFreqHz,axes";
			else if (hasAccData && hasMagData && !hasGyroData)
				return "consecutiveNumberBurst,accInGBurst,magInUTBurst,hallBurst,accConvFact,magConvFact,imuFreqHz,axes";
			else if (hasAccData && !hasMagData && hasGyroData)
				return "consecutiveNumberBurst,accInGBurst,gyroInDPSBurst,accConvFact,gyroConvFact,imuFreqHz,axes";
			else if (hasAccData && hasMagData && hasGyroData)
				return "consecutiveNumberBurst,accInGBurst,magInUTBurst,hallBurst,gyroInDPSBurst,accConvFact,magConvFact,gyroConvFact,imuFreqHz,axes";
			return "ERROR";
		} else {
			if(hasAccData && !hasMagData && !hasGyroData)
				return "consecutiveNumber,accXinG,accYinG,accZinG,accConvFact";
			else if(hasAccData && hasMagData && !hasGyroData)
				return "consecutiveNumber,accXinG,accYinG,accZinG,magXinUT,magYinUT,magZinUT,hall,accConvFact,magConvFact";
			else if(hasAccData && !hasMagData && hasGyroData)
				return "consecutiveNumber,accXinG,accYinG,accZinG,gyroXinDPS,gyroYinDPS,gyroZinDPS,accConvFact,gyroConvFact";
			else if(hasAccData && hasMagData && hasGyroData)
				return "consecutiveNumber,accXinG,accYinG,accZinG,magXinUT,magYinUT,magZinUT,hall,gyroXinDPS,gyroYinDPS,gyroZinDPS,accConvFact,magConvFact,gyroConvFact";
			return "ERROR";
		}
    }

    public String serialize() {
        if(hasAccData && !hasMagData && !hasGyroData) {
            return consecutiveNumber + ","
                    + (new Formatter(Locale.US).format("%.11f", accXinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accYinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accZinG)) + ","
										+ (new Formatter(Locale.US).format("%.11f", accConvFact));
        }
        else if(hasAccData && hasMagData && !hasGyroData) {
            return consecutiveNumber + ","
                    + (new Formatter(Locale.US).format("%.11f", accXinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accYinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accZinG)) + ","
                    + (new Formatter(Locale.US).format("%.4f", magXinUT)) + ","
                    + (new Formatter(Locale.US).format("%.4f", magYinUT)) + ","
                    + (new Formatter(Locale.US).format("%.4f", magZinUT)) + ","
                    + hall + ","
										+ (new Formatter(Locale.US).format("%.11f", accConvFact)) + ","
										+ (new Formatter(Locale.US).format("%.11f", magConvFact));
        }
        else if(hasAccData && !hasMagData && hasGyroData) {
            return consecutiveNumber + ","
                    + (new Formatter(Locale.US).format("%.11f", accXinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accYinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accZinG)) + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroXinDPS)) + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroYinDPS)) + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroZinDPS)) + ","
										+ (new Formatter(Locale.US).format("%.11f", accConvFact)) + ","
										+ (new Formatter(Locale.US).format("%.11f", gyroConvFact));
        }
        else if(hasAccData && hasMagData && hasGyroData) {
            return consecutiveNumber + ","
                    + (new Formatter(Locale.US).format("%.11f", accXinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accYinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accZinG)) + ","
                    + (new Formatter(Locale.US).format("%.4f", magXinUT)) + ","
                    + (new Formatter(Locale.US).format("%.4f", magYinUT)) + ","
                    + (new Formatter(Locale.US).format("%.4f", magZinUT)) + ","
                    + hall + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroXinDPS)) + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroYinDPS)) + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroZinDPS)) + ","
										+ (new Formatter(Locale.US).format("%.11f", accConvFact)) + ","
										+ (new Formatter(Locale.US).format("%.11f", magConvFact)) + ","
										+ (new Formatter(Locale.US).format("%.11f", gyroConvFact));
        }
        else return "ERROR";
    }

	public String serializeConsecutiveNumber() {
		if (hasAccData)
			return Long.toString(consecutiveNumber);
		return "ERROR";
	}

	public String serializeAccelerometerData() {
		if (hasAccData)
			return	  (new Formatter(Locale.US).format("%.11f", accXinG)) + " "
					+ (new Formatter(Locale.US).format("%.11f", accYinG)) + " "
					+ (new Formatter(Locale.US).format("%.11f", accZinG));
		return "ERROR";
	}

	public String serializeMagnetometerData() {
		if (hasAccData) {
			if (hasMagData)
				return	  (new Formatter(Locale.US).format("%.4f", magXinUT)) + " "
						+ (new Formatter(Locale.US).format("%.4f", magYinUT)) + " "
						+ (new Formatter(Locale.US).format("%.4f", magZinUT));
			return "";
		}
		return "ERROR";
	}

	public String serializeHallSensorData() {
		if (hasAccData) {
			if (hasMagData)
				return Long.toString(hall);
			return "";
		}
		return "ERROR";
	}

	public String serializeGyroscopeData() {
		if (hasAccData) {
			if (hasGyroData)
				return	  (new Formatter(Locale.US).format("%.4f", gyroXinDPS)) + " "
						+ (new Formatter(Locale.US).format("%.4f", gyroYinDPS)) + " "
						+ (new Formatter(Locale.US).format("%.4f", gyroZinDPS));
			return "";
		}
		return "ERROR";
	}

	public String serializeConversionFactor() {
		if (hasAccData && !hasMagData && !hasGyroData)
			return "" + (new Formatter(Locale.US).format("%.11f", accConvFact));
		else if (hasAccData && hasMagData && !hasGyroData)
			return   (new Formatter(Locale.US).format("%.11f", accConvFact)) + ","
						 + (new Formatter(Locale.US).format("%.11f", magConvFact));
		else if (hasAccData && !hasMagData && hasGyroData)
			return   (new Formatter(Locale.US).format("%.11f", accConvFact)) + ","
						 + (new Formatter(Locale.US).format("%.11f", gyroConvFact));
		else if (hasAccData && hasMagData && hasGyroData)
			return   (new Formatter(Locale.US).format("%.11f", accConvFact)) + ","
						 + (new Formatter(Locale.US).format("%.11f", magConvFact)) + ","
						 + (new Formatter(Locale.US).format("%.11f", gyroConvFact));
		return "ERROR";
	}

	public static String serializeEmpty(boolean hasAccData, boolean hasMagData, boolean hasGyroData) {
		if (hasAccData && !hasMagData && !hasGyroData)
			return "-,-,-,-";
		else if (hasAccData && hasMagData && !hasGyroData)
			return "-,-,-,-,-,-,-,-";
		else if (hasAccData && !hasMagData && hasGyroData)
			return "-,-,-,-,-,-,-";
		else if (hasAccData && hasMagData && hasGyroData)
			return "-,-,-,-,-,-,-,-,-,-,-";
		else
			return "ERROR";
	}
}
