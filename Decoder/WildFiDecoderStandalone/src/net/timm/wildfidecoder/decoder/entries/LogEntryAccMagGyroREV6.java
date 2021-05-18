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

public class LogEntryAccMagGyroREV6 extends LogEntry {
    public long milliseconds, lastErrorId, errorCnt, fifoLen;
    double batteryVoltage, temperature, humidity, pressure, temperatureBmx;

    public ArrayList<IMUEntry> imuEntries = new ArrayList<>();

    public LogEntryAccMagGyroREV6() {
        prefix = "12345E";
    }
    public int minLength = INDEX(27);

    public void decode(String dataset, boolean decodeOnlyHeaderIn, boolean debug, IMUSettings imuSettings) {
        decodeOnlyHeader = decodeOnlyHeaderIn;

        if(dataset.length() < minLength) {
            if (debug) Log.d("decoder-plausibility", "length not plausible " + dataset.length());
            plausibilityCheckOkay = false;
            return;
        }

        utcTimestamp = Long.parseLong(dataset.substring(INDEX(3), INDEX(7)), 16);
        milliseconds = 10 * Long.parseLong(dataset.substring(INDEX(7), INDEX(8)), 16);
        lastErrorId = Long.parseLong(dataset.substring(INDEX(8), INDEX(9)), 16);
        errorCnt = Long.parseLong(dataset.substring(INDEX(9), INDEX(11)), 16);
        batteryVoltage = (Long.parseLong(dataset.substring(INDEX(11), INDEX(13)), 16)) / 1000.;

        temperature = Long.parseLong(dataset.substring(INDEX(13), INDEX(15)), 16) / 100.;
        humidity = Long.parseLong(dataset.substring(INDEX(15), INDEX(19)), 16) / 1000.;
        pressure = Long.parseLong(dataset.substring(INDEX(19), INDEX(23)), 16) / 100.;
        temperatureBmx = Long.parseLong(dataset.substring(INDEX(23), INDEX(25)), 16) / 100.;

        fifoLen = Long.parseLong(dataset.substring(INDEX(25), INDEX(27)), 16);

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
            IMUDecoder.createIMUData(true, true, true, fifoDataExtracted, imuEntries, imuSettings.accConversionFactor, imuSettings.gyroConversionFactor);
        }

        entryLengthInBytes = toIntExact((minLength / 2) + fifoLen);
    }

    public String headlineHeader() {
        return "prefixDataType," +
                "utcTimestamp," +
                "utcDate," +
                "milliseconds," +
                "lastErrorId," +
                "errorCount," +
                "batteryVoltage," +
                "temperatureInDegCel," +
                "humidityInPercent," +
                "pressureInHPA," +
                "temperatureBmxInDegCel," +
                "fifoLen";
                /*
                "accAvgXinG," +
                "accAvgYinG," +
                "accAvgZinG," +
                "accAvgMagnitudeInG," +
                "accMagnitudeSumWithoutEarthGravitation";*/
    }

    public String serializeHeader() {
        return prefix + ","
                + utcTimestamp + ","
                + LogEntryManager.utcTimestampToStringWithoutWeekday(utcTimestamp) + ","
                + milliseconds + ","
                + lastErrorId + ","
                + errorCnt + ","
                + (new Formatter(Locale.US).format("%.3f", batteryVoltage)) + ","
                + (new Formatter(Locale.US).format("%.3f", temperature)) + ","
                + (new Formatter(Locale.US).format("%.3f", humidity)) + ","
                + (new Formatter(Locale.US).format("%.3f", pressure)) + ","
                + (new Formatter(Locale.US).format("%.3f", temperatureBmx)) + ","
                + fifoLen;
                /*+ (new Formatter(Locale.US).format("%.7f", IMUDecoder.accGetAvgX(imuEntries))) + ","
                + (new Formatter(Locale.US).format("%.7f", IMUDecoder.accGetAvgY(imuEntries))) + ","
                + (new Formatter(Locale.US).format("%.7f", IMUDecoder.accGetAvgZ(imuEntries))) + ","
                + (new Formatter(Locale.US).format("%.7f", IMUDecoder.getAverageMagnitudeWithEarthGravitation(imuEntries))) + ","
                + (new Formatter(Locale.US).format("%.3f", IMUDecoder.calculateAccMagnitudeSumWithoutEarthGravitation(imuEntries)));*/
    }

    public int getVarDataLength() {
        return imuEntries.size();
    }

    public String headlineHeaderAndVarData() {
        return headlineHeader() + "," + IMUEntry.serializeHeadline(true, true, true);
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
        LogEntryAccMagGyroREV6 e = new LogEntryAccMagGyroREV6();
        e.dataMessageCustomPrefix = dataMessageCustomPrefixIn;
        return e;
    }
}
