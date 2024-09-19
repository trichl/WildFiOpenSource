package net.timm.wildfidecoder.decoder.entries;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.LogEntryManager;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.util.Formatter;
import java.util.Locale;

import static java.lang.Math.toIntExact;

public class LogEntryGPSOnly extends LogEntry {
    public long ttf;
    double lat, lon, hdop;
    String name;

    public int minLength = INDEX(17);
    public LogEntryGPSOnly() {
        prefix = "123458";
    }

    public void decode(String name, String dataset, boolean debug, IMUSettings imuSettings, int imuFrequency) {
        if(dataset.length() < minLength) {
            if (debug) Log.d("decoder-plausibility", "length not plausible " + dataset.length());
            plausibilityCheckOkay = false;
            return;
        }

        this.name = name;
        this.imuFrequency = imuFrequency;

        utcTimestamp = Long.parseLong(dataset.substring(INDEX(3), INDEX(7)), 16);
        lat = ((int) Long.parseLong(dataset.substring(INDEX(7), INDEX(11)), 16)) / 1000000.;
        lon = ((int) Long.parseLong(dataset.substring(INDEX(11), INDEX(15)), 16)) / 1000000.;
        hdop = (Long.parseLong(dataset.substring(INDEX(15), INDEX(16)), 16)) / 10.;
        ttf = (Long.parseLong(dataset.substring(INDEX(16), INDEX(17)), 16));

        if (!LogEntryManager.timestampPlausible(utcTimestamp, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.gpsTTFPlausible(ttf, debug)) plausibilityCheckOkay = false;

        entryLengthInBytes = toIntExact(minLength / 2);
    }

    public String headlineHeader() {
        return "id," +
                "prefixDataType," +
								"tagId," +
                "utcTimestamp," +
                "utcDate," +
                "lat," +
                "lon," +
                "hdop," +
                "ttfSeconds";
    }

    public String serializeHeader() {
        return dataMessageCustomPrefix + ","
                + prefix + ","
								+ name + ","
                + utcTimestamp + ","
                + LogEntryManager.utcTimestampToStringWithoutWeekday(utcTimestamp) + ","
                + LogEntryManager.gpsLatLngToText(lat) + ","
                + LogEntryManager.gpsLatLngToText(lon) + ","
                + LogEntryManager.gpsHdopToText(hdop) + ","
                + ttf;
    }

    public int getVarDataLength() {
        return 0;
    }

    public String headlineHeaderAndVarData(boolean useBurstForm) {
        return headlineHeader();
    }

    public String serializeHeaderAndVarData(boolean useBurstForm) {
        String returnVal = "";
        returnVal += serializeHeader() + "\n";
        if (returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
        return returnVal;
    }

    public LogEntry copyMe(String dataMessageCustomPrefixIn) {
        LogEntryGPSOnly e = new LogEntryGPSOnly();
        e.dataMessageCustomPrefix = dataMessageCustomPrefixIn;
        return e;
    }
}
