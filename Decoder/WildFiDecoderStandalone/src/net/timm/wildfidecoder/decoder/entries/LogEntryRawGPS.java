package net.timm.wildfidecoder.decoder.entries;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.LogEntryManager;
import net.timm.wildfidecoder.decoder.rawgps.RawGPSEntry;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.util.ArrayList;
import java.util.Formatter;
import java.util.Locale;

import static java.lang.Math.toIntExact;

public class LogEntryRawGPS extends LogEntry {

    String name;
    public long measurementLen;
    public ArrayList<RawGPSEntry> rawGPSEntries = new ArrayList<>();
    float receiverTimeOfWeekInSec;
    int gpsWeekNumber, gpsLeapSeconds, receiverTrackingStatusBitField, observationLen;

    public int minLength = INDEX(17);
    public LogEntryRawGPS() {
        prefix = "123472";
    }

    public void decode(String name, String dataset, boolean debug, IMUSettings imuSettings, int imuFrequency) {
        Long temp;
        if(dataset.length() < minLength) {
            if (debug) Log.d("decoder-plausibility", "length not plausible " + dataset.length());
            plausibilityCheckOkay = false;
            return;
        }

        this.name = name;
        this.imuFrequency = imuFrequency;

        utcTimestamp = Long.parseLong(dataset.substring(INDEX(3), INDEX(7)), 16);
        temp = Long.parseLong(dataset.substring(INDEX(10), INDEX(11)) + dataset.substring(INDEX(9), INDEX(10)) + dataset.substring(INDEX(8), INDEX(9)) + dataset.substring(INDEX(7), INDEX(8)), 16);
        receiverTimeOfWeekInSec = Float.intBitsToFloat(temp.intValue());
        gpsWeekNumber = Integer.parseInt(dataset.substring(INDEX(12), INDEX(13)) + dataset.substring(INDEX(11), INDEX(12)), 16);
        gpsLeapSeconds = Integer.parseInt(dataset.substring(INDEX(13), INDEX(14)), 16);
        receiverTrackingStatusBitField = Integer.parseInt(dataset.substring(INDEX(14), INDEX(15)), 16);
        measurementLen = Integer.parseInt(dataset.substring(INDEX(15), INDEX(16)), 16);
        observationLen = Integer.parseInt(dataset.substring(INDEX(16), INDEX(17)), 16);

        //if (!LogEntryManager.timestampPlausible(utcTimestamp, debug)) plausibilityCheckOkay = false; // timestamp is zero at the moment
        if (!LogEntryManager.rawGPSMeasurementsPlausible(measurementLen, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.observationLenPlausible(observationLen, debug)) plausibilityCheckOkay = false;

        String measurementDataExtracted = "";
        try {
            measurementDataExtracted = dataset.substring(minLength, minLength + INDEX(toIntExact(measurementLen)*observationLen));
        }
        catch(Exception e) {
            plausibilityCheckOkay = false;
            if (debug) Log.d("decoder-plausibility", "measurement length does not match string length (partially transmitted fifo, maybe last received message)");
        }

        if(plausibilityCheckOkay) {
            RawGPSEntry.createMeasurementData(measurementDataExtracted, utcTimestamp, rawGPSEntries, observationLen);
        }
        //if (!LogEntryManager.proximityRSSIsMakeSense(proxEntries, debug)) plausibilityCheckOkay = false;
        entryLengthInBytes = toIntExact((minLength / 2) + (observationLen*toIntExact(measurementLen)));
    }

    public String headlineHeader() {
        return "id," +
                "prefixDataType," +
                "tagId," +
                "utcTimestamp," +
                "utcDate," +
                "receiverTimeOfWeekInSec," +
                "gpsWeekNumber," +
                "gpsLeapSeconds," +
                "receiverTrackingStatusBitField," +
                "measurementLen," +
                "observationLen";
    }

    public String serializeHeader() {
        return dataMessageCustomPrefix + ","
                + prefix + ","
                + name + ","
                + utcTimestamp + ","
                + LogEntryManager.utcTimestampToStringWithoutWeekday(utcTimestamp) + ","
                + (new Formatter(Locale.US).format("%.10f", receiverTimeOfWeekInSec)) + ","
                + gpsWeekNumber + ","
                + gpsLeapSeconds + ","
                + receiverTrackingStatusBitField + ","
                + measurementLen + ","
                + observationLen;
    }

    public int getVarDataLength() {
        return rawGPSEntries.size();
    }

    public String headlineHeaderAndVarData(boolean useBurstForm) {
        return headlineHeader() + "," + RawGPSEntry.serializeHeadline(useBurstForm);
    }

    public String serializeHeaderAndVarData(boolean useBurstForm) {
		String returnVal = "";
		if (useBurstForm) {
			returnVal += serializeHeader() + ",";
			if (rawGPSEntries.size() > 0) {
				for (int i = 0; i < rawGPSEntries.size(); i++)
					returnVal += rawGPSEntries.get(i).serializePseudorange() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeCarrierphase() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeDopplershift() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeCarriertonoiseratio() + (i == rawGPSEntries.size() - 1 ? "," : " ");

                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeGnssId() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeSatId() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeSignalId() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeFrequencySlot() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeCarrierPhaseLockTimeCounter() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializePseudoRangeStdDev() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeCarrierPhaseStdDev() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeDopplerShiftStdDev() + (i == rawGPSEntries.size() - 1 ? "," : " ");
                for (int i = 0; i < rawGPSEntries.size(); i++)
                    returnVal += rawGPSEntries.get(i).serializeTrackingStatus() + (i == rawGPSEntries.size() - 1 ? "," : " ");

                // TODO
			} else
				returnVal += RawGPSEntry.serializeEmpty() + ",";
		} else {
			if (rawGPSEntries.size() == 0)
				returnVal += serializeHeader() + "," + RawGPSEntry.serializeEmpty() + "\n";
			for (RawGPSEntry p : rawGPSEntries)
				returnVal += serializeHeader() + "," + p.serialize() + "\n";
		}
		if (returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
		return returnVal;
	}

    public LogEntry copyMe(String dataMessageCustomPrefixIn) {
        LogEntryRawGPS e = new LogEntryRawGPS();
        e.dataMessageCustomPrefix = dataMessageCustomPrefixIn;
        return e;
    }
}
