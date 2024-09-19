package net.timm.wildfidecoder.decoder.entries;

import net.timm.wildfidecoder.decoder.imu.IMUSettings;

public abstract class LogEntry {

    public String dataMessageCustomPrefix;
    public String prefix;
    public int entryLengthInBytes;
    public boolean plausibilityCheckOkay = true;
    public long utcTimestamp;
    public int imuFrequency;

    public static final int INDEX(int in) {
        return in * 2;
    }

    public String serializeHeaderEmpty() {
        String returnVal = prefix;
        int numberOfDataElementsWithoutVarData = (int) (headlineHeader().chars().filter(ch -> ch == ',').count() + 1);
        for(int i = 0; i < numberOfDataElementsWithoutVarData; i++) {
            returnVal += ",";
        }
        if (returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
        return returnVal;
    }

    public abstract LogEntry copyMe(String dataMessageCustomPrefixIn);
    public abstract void decode(String name, String dataset, boolean debug, IMUSettings imuSettings, int imuFrequency);

    public abstract int getVarDataLength();

    public abstract String headlineHeader();
    public abstract String headlineHeaderAndVarData(boolean useBurstForm);
    public abstract String serializeHeader();
    public abstract String serializeHeaderAndVarData(boolean useBurstForm);


}
