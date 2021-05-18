package net.timm.wildfidecoder.decoder.imu;

import java.util.Formatter;
import java.util.Locale;

public class IMUEntry {
    public long calculatedTimestampEstimation, consecutiveNumber;

    public double accXinG, accYinG, accZinG;
    public long magXinUT, magYinUT, magZinUT, hall;
    public double gyroXinDPS, gyroYinDPS, gyroZinDPS;

    public boolean hasAccData = false, hasMagData = false, hasGyroData = false;

    public double accCalculatedMagnitudeWithoutEarthGravitation;

    public IMUEntry(boolean hasAccDataIn, boolean hasMagDataIn, boolean hasGyroDataIn) {
        hasAccData = hasAccDataIn;
        hasMagData = hasMagDataIn;
        hasGyroData = hasGyroDataIn;
    }

    public static String serializeHeadline(boolean hasAccData, boolean hasMagData, boolean hasGyroData) {
        if(hasAccData && !hasMagData && !hasGyroData) {
            return "consecutiveNumber,accXinG,accYinG,accZinG";
        }
        else if(hasAccData && hasMagData && !hasGyroData) {
            return "consecutiveNumber,accXinG,accYinG,accZinG,magXinUT,magYinUT,magZinUT,hall";
        }
        else if(hasAccData && !hasMagData && hasGyroData) {
            return "consecutiveNumber,accXinG,accYinG,accZinG,gyroXinDPS,gyroYinDPS,gyroZinDPS";
        }
        else if(hasAccData && hasMagData && hasGyroData) {
            return "consecutiveNumber,accXinG,accYinG,accZinG,magXinUT,magYinUT,magZinUT,hall,gyroXinDPS,gyroYinDPS,gyroZinDPS";
        }
        else return "ERROR";
    }

    public String serialize() {
        if(hasAccData && !hasMagData && !hasGyroData) {
            return consecutiveNumber + ","
                    + (new Formatter(Locale.US).format("%.11f", accXinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accYinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accZinG));
        }
        else if(hasAccData && hasMagData && !hasGyroData) {
            return consecutiveNumber + ","
                    + (new Formatter(Locale.US).format("%.11f", accXinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accYinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accZinG)) + ","
                    + magXinUT + ","
                    + magYinUT + ","
                    + magZinUT + ","
                    + hall;
        }
        else if(hasAccData && !hasMagData && hasGyroData) {
            return consecutiveNumber + ","
                    + (new Formatter(Locale.US).format("%.11f", accXinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accYinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accZinG)) + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroXinDPS)) + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroYinDPS)) + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroZinDPS));
        }
        else if(hasAccData && hasMagData && hasGyroData) {
            return consecutiveNumber + ","
                    + (new Formatter(Locale.US).format("%.11f", accXinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accYinG)) + ","
                    + (new Formatter(Locale.US).format("%.11f", accZinG)) + ","
                    + magXinUT + ","
                    + magYinUT + ","
                    + magZinUT + ","
                    + hall + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroXinDPS))  + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroYinDPS))  + ","
                    + (new Formatter(Locale.US).format("%.4f", gyroZinDPS)) ;
        }
        else return "ERROR";
    }

}
