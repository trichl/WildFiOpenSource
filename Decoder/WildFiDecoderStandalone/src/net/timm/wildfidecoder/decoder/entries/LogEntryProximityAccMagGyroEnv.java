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

public class LogEntryProximityAccMagGyroEnv extends LogEntry {
    public long proxLen, fifoLen;
    public ArrayList<ProxEntry> proxEntries = new ArrayList<>();
    public ArrayList<IMUEntry> imuEntries = new ArrayList<>();
    double temperature, humidity, pressure;
		String name;

    public int minLength = INDEX(21);
    public LogEntryProximityAccMagGyroEnv() {
        prefix = "123466";
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
        temperature = ((short) Integer.parseInt(dataset.substring(INDEX(7), INDEX(9)), 16)) / 100.;
        humidity = Long.parseLong(dataset.substring(INDEX(9), INDEX(13)), 16) / 1000.;
        pressure = Long.parseLong(dataset.substring(INDEX(13), INDEX(17)), 16) / 100.;
        proxLen = Long.parseLong(dataset.substring(INDEX(17), INDEX(19)), 16);
        fifoLen = Long.parseLong(dataset.substring(INDEX(19), INDEX(21)), 16);

        if (!LogEntryManager.timestampPlausible(utcTimestamp, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.proxLenPlausible(proxLen, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.fifoLenPlausible(fifoLen, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.utcProximityTimePlausible(utcTimestamp, debug)) plausibilityCheckOkay = false;

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
        if (!LogEntryManager.proximityRSSIsMakeSense(proxEntries, debug)) plausibilityCheckOkay = false;

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
            IMUDecoder.createIMUData(true, true, true, fifoDataExtracted, imuEntries, imuSettings.accConversionFactor, imuSettings.gyroConversionFactor, imuSettings.magConversionFactor);
        }

        entryLengthInBytes = toIntExact((minLength / 2) + fifoLen + proxLen);
    }

    public String headlineHeader() {
        return "id," +
                "prefixDataType," +
								"tagId," +
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
								+ name + ","
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

	public String headlineHeaderAndVarData(boolean useBurstForm) {
		if (useBurstForm)
			return headlineHeader() + "," + ProxEntry.serializeHeadline(true) + "," + IMUEntry.serializeHeadline(true, true, true, true);
		else
			return headlineHeader() + "," + ProxEntry.serializeHeadline(false) + "\n" + IMUEntry.serializeHeadline(true, true, true, false);
	}

	public String serializeHeaderAndVarData(boolean useBurstForm) {
		String returnVal = "";
		if (useBurstForm) {
			returnVal += serializeHeader() + ",";
			if (proxEntries.size() > 0) {
				for (int i = 0; i < proxEntries.size(); i++)
					returnVal += proxEntries.get(i).serializeId() + (i == proxEntries.size() - 1 ? "," : " ");
				for (int i = 0; i < proxEntries.size(); i++)
					returnVal += proxEntries.get(i).serializeRssi() + (i == proxEntries.size() - 1 ? "," : " ");
			} else
				returnVal += ProxEntry.serializeNobodySeen() + ",";
			if(imuEntries.size() > 0) {
                for (int i = 0; i < imuEntries.size(); i++)
                    returnVal += imuEntries.get(i).serializeConsecutiveNumber() + (i == imuEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < imuEntries.size(); i++)
                    returnVal += imuEntries.get(i).serializeAccelerometerData() + (i == imuEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < imuEntries.size(); i++)
                    returnVal += imuEntries.get(i).serializeMagnetometerData() + (i == imuEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < imuEntries.size(); i++)
                    returnVal += imuEntries.get(i).serializeHallSensorData() + (i == imuEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < imuEntries.size(); i++)
                    returnVal += imuEntries.get(i).serializeGyroscopeData() + (i == imuEntries.size() - 1 ? "," : " ");
                returnVal += imuEntries.get(0).serializeConversionFactor() + ",";
            }
            else {
                returnVal += ",,,";
            }
            returnVal += imuFrequency + ",XYZ,";
		} else {
			for (IMUEntry a : imuEntries)
				returnVal += a.serialize() + "\n";
			if (proxEntries.size() == 0)
				returnVal += serializeHeader() + "," + ProxEntry.serializeNobodySeen() + "\n";
			for (ProxEntry p : proxEntries)
				returnVal += serializeHeader() + "," + p.serialize() + "\n";
		}
		if (returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
		return returnVal;
	}

    public LogEntry copyMe(String dataMessageCustomPrefixIn) {
        LogEntryProximityAccMagGyroEnv e = new LogEntryProximityAccMagGyroEnv();
        e.dataMessageCustomPrefix = dataMessageCustomPrefixIn;
        return e;
    }
}
