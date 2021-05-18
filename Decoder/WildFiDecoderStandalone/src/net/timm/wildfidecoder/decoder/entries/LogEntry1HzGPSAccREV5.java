package net.timm.wildfidecoder.decoder.entries;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.LogEntryManager;
import net.timm.wildfidecoder.decoder.imu.IMUEntry;
import net.timm.wildfidecoder.decoder.imu.IMUDecoder;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.util.ArrayList;
import java.util.Formatter;
import java.util.Locale;

import static java.lang.Math.toIntExact;

public class LogEntry1HzGPSAccREV5 extends LogEntry {
    public long lastErrorId, fifoLen;
    double lat, lon, hdop;

    public ArrayList<IMUEntry> imuEntries = new ArrayList<>();

    public LogEntry1HzGPSAccREV5() {
        prefix = "123456";
    }
    public int minLength = INDEX(19);

    public void decode(String dataset, boolean decodeOnlyHeaderIn, boolean debug, IMUSettings imuSettings) {
        decodeOnlyHeader = decodeOnlyHeaderIn;

        if(dataset.length() < minLength) {
            if (debug) Log.d("decoder-plausibility", "length not plausible " + dataset.length());
            plausibilityCheckOkay = false;
            return;
        }

        utcTimestamp = Long.parseLong(dataset.substring(INDEX(3), INDEX(7)), 16);
        lastErrorId = Long.parseLong(dataset.substring(INDEX(7), INDEX(8)), 16);
        lat = (Long.parseLong(dataset.substring(INDEX(8), INDEX(12)), 16)) / 1000000.;
        lon = (Long.parseLong(dataset.substring(INDEX(12), INDEX(16)), 16)) / 1000000.;
        hdop = (Long.parseLong(dataset.substring(INDEX(16), INDEX(17)), 16)) / 10.;
        fifoLen = Long.parseLong(dataset.substring(INDEX(17), INDEX(19)), 16);

        if (!LogEntryManager.timestampPlausible(utcTimestamp, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.fifoLenPlausible(fifoLen, debug)) plausibilityCheckOkay = false;

        String fifoDataExtracted = "";
        try {
            fifoDataExtracted = dataset.substring(minLength, minLength + INDEX(toIntExact(fifoLen)));
        }
        catch(Exception e) {
            plausibilityCheckOkay = false;
            if (debug) Log.d("decoder-plausibility", "fifo length does not match string length (partially transmitted fifo, maybe last received message)");
        }

        if(plausibilityCheckOkay) {
            IMUDecoder.createIMUData(true, false, false, fifoDataExtracted, imuEntries, imuSettings.accConversionFactor, imuSettings.gyroConversionFactor);
        }
        entryLengthInBytes = toIntExact((minLength / 2) + fifoLen);
    }

    public String headlineHeader() {
        return "prefixDataType," +
                "utcTimestamp," +
                "utcDate," +
                "lastErrorId," +
                "fifoLen," +
                "lat," +
                "lon," +
                "hdop," +
                "accAvgXinG," +
                "accAvgYinG," +
                "accAvgZinG," +
                "accAvgMagnitudeInG," +
                "accMagnitudeSumWithoutEarthGravitation";
    }

    public String serializeHeader() {
        return prefix + ","
                + utcTimestamp + ","
                + LogEntryManager.utcTimestampToStringWithoutWeekday(utcTimestamp) + ","
                + lastErrorId + ","
                + (new Formatter(Locale.US).format("%.7f", lat)) + ","
                + (new Formatter(Locale.US).format("%.7f", lon)) + ","
                + (new Formatter(Locale.US).format("%.1f", hdop)) + ","
                + fifoLen + ","
                + (new Formatter(Locale.US).format("%.7f", IMUDecoder.accGetAvgX(imuEntries))) + ","
                + (new Formatter(Locale.US).format("%.7f", IMUDecoder.accGetAvgY(imuEntries))) + ","
                + (new Formatter(Locale.US).format("%.7f", IMUDecoder.accGetAvgZ(imuEntries))) + ","
                + (new Formatter(Locale.US).format("%.7f", IMUDecoder.getAverageMagnitudeWithEarthGravitation(imuEntries))) + ","
                + (new Formatter(Locale.US).format("%.3f", IMUDecoder.calculateAccMagnitudeSumWithoutEarthGravitation(imuEntries)));
    }

    public int getVarDataLength() {
        return imuEntries.size();
    }

    public String headlineHeaderAndVarData() {
        return headlineHeader() + "," + IMUEntry.serializeHeadline(true, false, false);
    }

    public String serializeHeaderAndVarData() {
        String returnVal = "";
        int i = 0;
        for (IMUEntry a : imuEntries) {
            if(i++ == imuEntries.size() - 1) returnVal += serializeHeader() + "," + a.serialize() + "\n";
            else returnVal += serializeHeaderEmpty() + "," + a.serialize() + "\n";
        }
        if (returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
        return returnVal;
    }

    public LogEntry copyMe(String dataMessageCustomPrefixIn) {
        LogEntry1HzGPSAccREV5 e = new LogEntry1HzGPSAccREV5();
        e.dataMessageCustomPrefix = dataMessageCustomPrefixIn;
        return e;
    }
}
