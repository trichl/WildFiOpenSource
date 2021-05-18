package net.timm.wildfidecoder.decoder.entries;

import net.timm.wildfidecoder.decoder.imu.IMUSettings;

public abstract class LogEntry {

    public String dataMessageCustomPrefix;
    public String prefix;
    public int entryLengthInBytes;
    public boolean plausibilityCheckOkay = true;
    public boolean decodeOnlyHeader;
    public long utcTimestamp;

    public static final int INDEX(int in) {
        return in * 2;
    }

    public String serializeHeaderEmpty() {
        String returnVal = prefix;
        int numberOfDataElementsWithoutVarData = numberOfDataElementsWithoutVarData = (int) (headlineHeader().chars().filter(ch -> ch == ',').count() + 1);
        for(int i = 0; i< numberOfDataElementsWithoutVarData; i++) {
            returnVal += ",";
        }
        if (returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
        return returnVal;
    }

    public abstract LogEntry copyMe(String dataMessageCustomPrefixIn);
    public abstract void decode(String dataset, boolean decodeOnlyHeaderIn, boolean debug, IMUSettings imuSettings);

    public abstract int getVarDataLength();

    public abstract String headlineHeader();
    public abstract String headlineHeaderAndVarData();
    public abstract String serializeHeader();
    public abstract String serializeHeaderAndVarData();


}
