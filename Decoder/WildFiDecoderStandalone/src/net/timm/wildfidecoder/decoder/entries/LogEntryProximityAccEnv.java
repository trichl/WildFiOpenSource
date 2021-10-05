package net.timm.wildfidecoder.decoder.entries;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.LogEntryManager;
import net.timm.wildfidecoder.decoder.imu.IMUDecoder;
import net.timm.wildfidecoder.decoder.imu.IMUEntry;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;
import net.timm.wildfidecoder.decoder.prox.ProxEntry;

import java.util.ArrayList;
import java.util.Formatter;
import java.util.Locale;

import static java.lang.Math.toIntExact;

public class LogEntryProximityAccEnv extends LogEntry {
    public long proxLen, fifoLen;
    public ArrayList<ProxEntry> proxEntries = new ArrayList<>();
    public ArrayList<IMUEntry> imuEntries = new ArrayList<>();
    double temperature, humidity, pressure;

    public int minLength = INDEX(21);
    public LogEntryProximityAccEnv() {
        prefix = "123464";
    }

    public void decode(String dataset, boolean decodeOnlyHeaderIn, boolean debug, IMUSettings imuSettings) {
        decodeOnlyHeader = decodeOnlyHeaderIn;

        if(dataset.length() < minLength) {
            if (debug) Log.d("decoder-plausibility", "length not plausible " + dataset.length());
            plausibilityCheckOkay = false;
            return;
        }

        utcTimestamp = Long.parseLong(dataset.substring(INDEX(3), INDEX(7)), 16);
        temperature = Long.parseLong(dataset.substring(INDEX(7), INDEX(9)), 16) / 100.;
        humidity = Long.parseLong(dataset.substring(INDEX(9), INDEX(13)), 16) / 1000.;
        pressure = Long.parseLong(dataset.substring(INDEX(13), INDEX(17)), 16) / 100.;
        proxLen = Long.parseLong(dataset.substring(INDEX(17), INDEX(19)), 16);
        fifoLen = Long.parseLong(dataset.substring(INDEX(19), INDEX(21)), 16);

        if (!LogEntryManager.timestampPlausible(utcTimestamp, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.proxLenPlausible(proxLen, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.fifoLenPlausible(fifoLen, debug)) plausibilityCheckOkay = false;

        // PROX DATA
        String proxDataExtracted = "";
        try {
            proxDataExtracted = dataset.substring(minLength, minLength + INDEX(toIntExact(proxLen)));
        }
        catch(Exception e) {
            plausibilityCheckOkay = false;
            if (debug) Log.d("decoder-plausibility", "prox length does not match string length (partially transmitted fifo, maybe last received message)");
        }
        if(plausibilityCheckOkay) {
            ProxEntry.createProxData(proxDataExtracted, utcTimestamp, proxEntries);
        }

        // FIFO DATA
        String fifoDataExtracted = "";
        try {
            fifoDataExtracted = dataset.substring(minLength + INDEX(toIntExact(proxLen)), minLength + INDEX(toIntExact(proxLen)) + INDEX(toIntExact(fifoLen)));
        }
        catch(Exception e) {
            plausibilityCheckOkay = false;
            if (debug) Log.d("decoder-plausibility", "fifo length does not match string length (partially transmitted fifo, maybe last received message)");
        }

        if(plausibilityCheckOkay) {
            IMUDecoder.createIMUData(true, false, false, fifoDataExtracted, imuEntries, imuSettings.accConversionFactor, imuSettings.gyroConversionFactor);
        }

        entryLengthInBytes = toIntExact((minLength / 2) + fifoLen + proxLen);
    }

    public String headlineHeader() {
        return "id," +
                "prefixDataType," +
                "utcTimestamp," +
                "utcDate," +
                "temperatureInDegCel," +
                "humidityInPercent," +
                "pressureInHPA," +
                "proxLen," +
                "fifoLen";
    }

    public String serializeHeader() {
        return dataMessageCustomPrefix + ","
                + prefix + ","
                + utcTimestamp + ","
                + LogEntryManager.utcTimestampToStringWithoutWeekday(utcTimestamp) + ","
                + (new Formatter(Locale.US).format("%.3f", temperature)) + ","
                + (new Formatter(Locale.US).format("%.3f", humidity)) + ","
                + (new Formatter(Locale.US).format("%.3f", pressure)) + ","
                + proxLen + ","
                + fifoLen;
    }

    public int getVarDataLength() {
        return proxEntries.size() + imuEntries.size();
    }

    public String headlineHeaderAndVarData() {
        return headlineHeader() + "," + ProxEntry.serializeHeadline() + "\n" + IMUEntry.serializeHeadline(true, false, false);
    }

    public String serializeHeaderAndVarData() {
        String returnVal = "";
        int i = 0;
        for (IMUEntry a : imuEntries) {
            returnVal += a.serialize() + "\n";
        }
        if(proxEntries.size() == 0) {
            returnVal += serializeHeader() + "," + ProxEntry.serializeNobodySeen() + "\n";
        }
        for (ProxEntry p : proxEntries) {
            returnVal += serializeHeader() + "," + p.serialize() + "\n";
        }
        if (returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
        return returnVal;
    }

    public LogEntry copyMe(String dataMessageCustomPrefixIn) {
        LogEntryProximityAccEnv e = new LogEntryProximityAccEnv();
        e.dataMessageCustomPrefix = dataMessageCustomPrefixIn;
        return e;
    }
}
