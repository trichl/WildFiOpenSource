package net.timm.wildfidecoder.decoder.entries;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.LogEntryManager;
import net.timm.wildfidecoder.decoder.imu.IMUDecoder;
import net.timm.wildfidecoder.decoder.imu.IMUEntry;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.util.ArrayList;
import java.util.Formatter;
import java.util.Locale;

import static java.lang.Math.toIntExact;

public class LogEntry1HzGPSAccMagREV6 extends LogEntry {
    public long milliseconds, lastErrorId, fifoLen;
    double temperature, humidity, pressure, temperatureBmx;
    double lat, lon, hdop;
		String name;

    public ArrayList<IMUEntry> imuEntries = new ArrayList<>();

    public LogEntry1HzGPSAccMagREV6() {
        prefix = "12345C";
    }
    public int minLength = INDEX(32);

    public void decode(String name, String dataset, boolean debug, IMUSettings imuSettings, int imuFrequency) {
        if(dataset.length() < minLength) {
            if (debug) Log.d("decoder-plausibility", "length not plausible " + dataset.length());
            plausibilityCheckOkay = false;
            return;
        }

				this.name = name;
        this.imuFrequency = imuFrequency;

        utcTimestamp = Long.parseLong(dataset.substring(INDEX(3), INDEX(7)), 16);
        milliseconds = 10 * Long.parseLong(dataset.substring(INDEX(7), INDEX(8)), 16);
        lastErrorId = Long.parseLong(dataset.substring(INDEX(8), INDEX(9)), 16);

        lat = ((int) Long.parseLong(dataset.substring(INDEX(9), INDEX(13)), 16)) / 1000000.;
        lon = ((int) Long.parseLong(dataset.substring(INDEX(13), INDEX(17)), 16)) / 1000000.;
        hdop = (Long.parseLong(dataset.substring(INDEX(17), INDEX(18)), 16)) / 10.;

        temperature = ((short) Integer.parseInt(dataset.substring(INDEX(18), INDEX(20)), 16)) / 100.;
        humidity = Long.parseLong(dataset.substring(INDEX(20), INDEX(24)), 16) / 1000.;
        pressure = Long.parseLong(dataset.substring(INDEX(24), INDEX(28)), 16) / 100.;
        temperatureBmx = ((short) Integer.parseInt(dataset.substring(INDEX(28), INDEX(30)), 16)) / 100.;

        fifoLen = Long.parseLong(dataset.substring(INDEX(30), INDEX(32)), 16);

        //if (!LogEntryManager.timestampPlausible(utcTimestamp, debug)) plausibilityCheckOkay = false; // NO, because maybe starting without time
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
            IMUDecoder.createIMUData(true, true, false, fifoDataExtracted, imuEntries, imuSettings.accConversionFactor, imuSettings.gyroConversionFactor, imuSettings.magConversionFactor);
        }
        entryLengthInBytes = toIntExact((minLength / 2) + fifoLen);
    }

    public String headlineHeader() {
        return "prefixDataType," +
								"tagId," +
                "utcTimestamp," +
                "utcDate," +
                "milliseconds," +
                "lastErrorId," +
                "lat," +
                "lon," +
                "hdop," +
                "temperatureInDegCel," +
                "humidityInPercent," +
                "pressureInHPA," +
                "temperatureBmxInDegCel," +
                "fifoLen";
    }

    public String serializeHeader() {
        return prefix + ","
								+ name + ","
                + utcTimestamp + ","
                + LogEntryManager.utcTimestampToStringWithoutWeekday(utcTimestamp) + ","
                + milliseconds + ","
                + lastErrorId + ","
                + LogEntryManager.gpsLatLngToText(lat) + ","
                + LogEntryManager.gpsLatLngToText(lon) + ","
                + LogEntryManager.gpsHdopToText(hdop) + ","
                + (new Formatter(Locale.US).format("%.3f", temperature)) + ","
                + (new Formatter(Locale.US).format("%.3f", humidity)) + ","
                + (new Formatter(Locale.US).format("%.3f", pressure)) + ","
                + (new Formatter(Locale.US).format("%.3f", temperatureBmx)) + ","
                + fifoLen;
    }

    public int getVarDataLength() {
        return imuEntries.size();
    }

	public String headlineHeaderAndVarData(boolean useBurstForm) {
		return headlineHeader() + "," + IMUEntry.serializeHeadline(true, true, false, useBurstForm);
	}

	public String serializeHeaderAndVarData (boolean useBurstForm) {
		String returnVal = "";
		if (useBurstForm) {
			returnVal = serializeHeader() + ",";
			for (int i = 0; i < imuEntries.size(); i++)
				returnVal += imuEntries.get(i).serializeConsecutiveNumber() + (i == imuEntries.size() - 1 ? "," : " ");
			for (int i = 0; i < imuEntries.size(); i++)
				returnVal += imuEntries.get(i).serializeAccelerometerData() + (i == imuEntries.size() - 1 ? "," : " ");
			for (int i = 0; i < imuEntries.size(); i++)
				returnVal += imuEntries.get(i).serializeMagnetometerData() + (i == imuEntries.size() - 1 ? "," : " ");
			for (int i = 0; i < imuEntries.size(); i++)
				returnVal += imuEntries.get(i).serializeHallSensorData() + (i == imuEntries.size() - 1 ? "," : " ");
            if(imuEntries.size() > 0) returnVal += imuEntries.get(0).serializeConversionFactor() + ",";
            else returnVal += ",";
            returnVal += imuFrequency + ",XYZ,";
		} else {
			for (int i = 0; i < imuEntries.size(); i++)
				returnVal += (i == imuEntries.size() - 1 ? serializeHeader() : serializeHeaderEmpty()) + "," + imuEntries.get(i).serialize() + "\n";
		}
		if (returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
		return returnVal;
	}

    public LogEntry copyMe(String dataMessageCustomPrefixIn) {
        LogEntry1HzGPSAccMagREV6 e = new LogEntry1HzGPSAccMagREV6();
        e.dataMessageCustomPrefix = dataMessageCustomPrefixIn;
        return e;
    }
}
