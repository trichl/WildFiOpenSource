package net.timm.wildfidecoder.decoder.imu;

import java.util.ArrayList;

public class IMUDecoder {

    public static double calculateAccMagnitudeSumWithoutEarthGravitation(ArrayList<IMUEntry> accEntries) {
        double accMagnitudeSumWithoutEarthGravitation = 0;
        for(IMUEntry a : accEntries) {
            accMagnitudeSumWithoutEarthGravitation += a.accCalculatedMagnitudeWithoutEarthGravitation;
        }
        return accMagnitudeSumWithoutEarthGravitation;
    }

    public static double accGetAvgX(ArrayList<IMUEntry> imuEntries) {
        double sum = 0;
        if(imuEntries.size() == 0) return 0;
        for(IMUEntry a : imuEntries) {
            sum += a.accXinG;
        }
        sum /= imuEntries.size();
        return sum;
    }

    public static double accGetAvgY(ArrayList<IMUEntry> imuEntries) {
        double sum = 0;
        if(imuEntries.size() == 0) return 0;
        for(IMUEntry a : imuEntries) {
            sum += a.accYinG;
        }
        sum /= imuEntries.size();
        return sum;
    }

    public static double accGetAvgZ(ArrayList<IMUEntry> imuEntries) {
        double sum = 0;
        if(imuEntries.size() == 0) return 0;
        for(IMUEntry a : imuEntries) {
            sum += a.accZinG;
        }
        sum /= imuEntries.size();
        return sum;
    }

    public static double getAverageMagnitudeWithEarthGravitation(ArrayList<IMUEntry> imuEntries) {
        return Math.sqrt(Math.pow(accGetAvgX(imuEntries), 2) + Math.pow(accGetAvgY(imuEntries), 2) + Math.pow(accGetAvgZ(imuEntries), 2));
    }

    public static long calculateTimestampFraction(IMUEntry a, int pos, int listSize, long lastTimestamp, double accFrequency) {
        double fraction = 1 / accFrequency; // time between acc logs
        fraction = (listSize - 1 - pos) * fraction;
        fraction *= 1000;
        return (lastTimestamp * 1000) - (long) fraction;
    }

    public static int createIMUData(boolean hasAccData, boolean hasMagData, boolean hasGyroData, String fifo, ArrayList<IMUEntry> imuEntries, double accConversionFactorToG, double gyroConversionFactorToDPS) {
        int len = fifo.length();
        int countIMUEntries = 0;
        int DATASET_LENGTH;

        if(hasAccData && !hasMagData && !hasGyroData) DATASET_LENGTH = 6;
        else if(hasAccData && hasMagData && !hasGyroData) DATASET_LENGTH = 14;
        else if(hasAccData && !hasMagData && hasGyroData) DATASET_LENGTH = 12;
        else if(hasAccData && hasMagData && hasGyroData) DATASET_LENGTH = 20;
        else return 0;

        DATASET_LENGTH *= 2; // 2 chars = 1 byte

        for(int i=0; i<(len/DATASET_LENGTH); i++) {
            IMUEntry a = new IMUEntry(hasAccData, hasMagData, hasGyroData);
            String dataset = fifo.substring((i*DATASET_LENGTH),(i*DATASET_LENGTH)+DATASET_LENGTH);
            short x,y,z,hall;

            if(hasAccData && !hasMagData && !hasGyroData) {
                x = (short) Integer.parseInt(dataset.substring(2, 4)+dataset.substring(0, 2),16);
                y = (short) Integer.parseInt(dataset.substring(6, 8)+dataset.substring(4, 6),16);
                z = (short) Integer.parseInt(dataset.substring(10, 12)+dataset.substring(8, 10),16);
                a.accXinG = x * accConversionFactorToG;
                a.accYinG = y * accConversionFactorToG;
                a.accZinG = z * accConversionFactorToG;
            }
            else if(hasAccData && hasMagData && !hasGyroData) {
                x = (short) Integer.parseInt(dataset.substring(2, 4)+dataset.substring(0, 2),16);
                y = (short) Integer.parseInt(dataset.substring(6, 8)+dataset.substring(4, 6),16);
                z = (short) Integer.parseInt(dataset.substring(10, 12)+dataset.substring(8, 10),16);
                hall = (short) Integer.parseInt(dataset.substring(14, 16)+dataset.substring(12, 14),16);
                a.magXinUT = x;
                a.magYinUT = y;
                a.magZinUT = z;
                a.hall = hall;

                x = (short) Integer.parseInt(dataset.substring(18, 20)+dataset.substring(16, 18),16);
                y = (short) Integer.parseInt(dataset.substring(22, 24)+dataset.substring(20, 22),16);
                z = (short) Integer.parseInt(dataset.substring(26, 28)+dataset.substring(24, 26),16);
                a.accXinG = x * accConversionFactorToG;
                a.accYinG = y * accConversionFactorToG;
                a.accZinG = z * accConversionFactorToG;
            }
            else if(hasAccData && !hasMagData && hasGyroData) {
                x = (short) Integer.parseInt(dataset.substring(2, 4)+dataset.substring(0, 2),16);
                y = (short) Integer.parseInt(dataset.substring(6, 8)+dataset.substring(4, 6),16);
                z = (short) Integer.parseInt(dataset.substring(10, 12)+dataset.substring(8, 10),16);
                a.gyroXinDPS = x * gyroConversionFactorToDPS;
                a.gyroYinDPS = y * gyroConversionFactorToDPS;
                a.gyroZinDPS = z * gyroConversionFactorToDPS;

                x = (short) Integer.parseInt(dataset.substring(14, 16)+dataset.substring(12, 14),16);
                y = (short) Integer.parseInt(dataset.substring(18, 20)+dataset.substring(16, 18),16);
                z = (short) Integer.parseInt(dataset.substring(22, 24)+dataset.substring(20, 22),16);
                a.accXinG = x * accConversionFactorToG;
                a.accYinG = y * accConversionFactorToG;
                a.accZinG = z * accConversionFactorToG;
            }
            else if(hasAccData && hasMagData && hasGyroData) {
                x = (short) Integer.parseInt(dataset.substring(2, 4)+dataset.substring(0, 2),16);
                y = (short) Integer.parseInt(dataset.substring(6, 8)+dataset.substring(4, 6),16);
                z = (short) Integer.parseInt(dataset.substring(10, 12)+dataset.substring(8, 10),16);
                hall = (short) Integer.parseInt(dataset.substring(14, 16)+dataset.substring(12, 14),16);
                a.magXinUT = x;
                a.magYinUT = y;
                a.magZinUT = z;
                a.hall = hall;

                x = (short) Integer.parseInt(dataset.substring(18, 20)+dataset.substring(16, 18),16);
                y = (short) Integer.parseInt(dataset.substring(22, 24)+dataset.substring(20, 22),16);
                z = (short) Integer.parseInt(dataset.substring(26, 28)+dataset.substring(24, 26),16);
                a.gyroXinDPS = x * gyroConversionFactorToDPS;
                a.gyroYinDPS = y * gyroConversionFactorToDPS;
                a.gyroZinDPS = z * gyroConversionFactorToDPS;

                x = (short) Integer.parseInt(dataset.substring(30, 32)+dataset.substring(28, 30),16);
                y = (short) Integer.parseInt(dataset.substring(34, 36)+dataset.substring(32, 34),16);
                z = (short) Integer.parseInt(dataset.substring(38, 40)+dataset.substring(36, 38),16);
                a.accXinG = x * accConversionFactorToG;
                a.accYinG = y * accConversionFactorToG;
                a.accZinG = z * accConversionFactorToG;
            }

            /*
            a.accCalculatedMagnitudeWithoutEarthGravitation = Math.sqrt(Math.pow(a.accXinG, 2) + Math.pow(a.accYinG, 2) + Math.pow(a.accZinG, 2)); // between 0 and 3.464 G
            a.accCalculatedMagnitudeWithoutEarthGravitation -= 1; // remove 1G = earth gravitation
            a.accCalculatedMagnitudeWithoutEarthGravitation = Math.abs(a.accCalculatedMagnitudeWithoutEarthGravitation); // absolute value because if result negative then there is still a force
            */
            a.consecutiveNumber = countIMUEntries;
            //if(accFrequency != 0) a.calculatedTimestampEstimation = calculateTimestampFraction(a, i, len/12, timestamp, accFrequency);
            a.calculatedTimestampEstimation = 0;
            imuEntries.add(a);
            countIMUEntries++;
        }
        return countIMUEntries;
    }

}
