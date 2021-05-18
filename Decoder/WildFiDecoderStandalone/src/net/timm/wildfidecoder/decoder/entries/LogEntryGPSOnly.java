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

    public int minLength = INDEX(17);
    public LogEntryGPSOnly() {
        prefix = "123458";
    }

    public void decode(String dataset, boolean decodeOnlyHeaderIn, boolean debug, IMUSettings imuSettings) {
        decodeOnlyHeader = decodeOnlyHeaderIn;

        if(dataset.length() < minLength) {
            if (debug) Log.d("decoder-plausibility", "length not plausible " + dataset.length());
            plausibilityCheckOkay = false;
            return;
        }

        utcTimestamp = Long.parseLong(dataset.substring(INDEX(3), INDEX(7)), 16);
        lat = (Long.parseLong(dataset.substring(INDEX(7), INDEX(11)), 16)) / 1000000.;
        lon = (Long.parseLong(dataset.substring(INDEX(11), INDEX(15)), 16)) / 1000000.;
        hdop = (Long.parseLong(dataset.substring(INDEX(15), INDEX(16)), 16)) / 10.;
        ttf = (Long.parseLong(dataset.substring(INDEX(15), INDEX(16)), 16));

        if (!LogEntryManager.timestampPlausible(utcTimestamp, debug)) plausibilityCheckOkay = false;

        entryLengthInBytes = toIntExact(minLength / 2);
    }

    public String headlineHeader() {
        return "id," +
                "prefixDataType," +
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
                + utcTimestamp + ","
                + LogEntryManager.utcTimestampToStringWithoutWeekday(utcTimestamp) + ","
                + (new Formatter(Locale.US).format("%.7f", lat)) + ","
                + (new Formatter(Locale.US).format("%.7f", lon)) + ","
                + (new Formatter(Locale.US).format("%.1f", hdop)) + ","
                + ttf;
    }

    public int getVarDataLength() {
        return 0;
    }

    public String headlineHeaderAndVarData() {
        return headlineHeader();
    }

    public String serializeHeaderAndVarData() {
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
