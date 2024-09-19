package net.timm.wildfidecoder.decoder.entries;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.LogEntryManager;
import net.timm.wildfidecoder.decoder.imu.IMUEntry;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;
import net.timm.wildfidecoder.decoder.prox.ProxEntry;

import java.util.ArrayList;

import static java.lang.Math.toIntExact;

public class LogEntryProximity extends LogEntry {
    public long proxLen;
    public ArrayList<ProxEntry> proxEntries = new ArrayList<>();
		String name;

    public int minLength = INDEX(9);
    public LogEntryProximity() {
        prefix = "123457";
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
        proxLen = Long.parseLong(dataset.substring(INDEX(7), INDEX(9)), 16);

        if (!LogEntryManager.timestampPlausible(utcTimestamp, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.proxLenPlausible(proxLen, debug)) plausibilityCheckOkay = false;
        if (!LogEntryManager.utcProximityTimePlausible(utcTimestamp, debug)) plausibilityCheckOkay = false;

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
        entryLengthInBytes = toIntExact((minLength / 2) + proxLen);
    }

    public String headlineHeader() {
        return "id," +
                "prefixDataType," +
                "tagId," +
                "utcTimestamp," +
                "utcDate," +
                "proxLen";
    }

    public String serializeHeader() {
        return dataMessageCustomPrefix + ","
                + prefix + ","
                + name + ","
                + utcTimestamp + ","
                + LogEntryManager.utcTimestampToStringWithoutWeekday(utcTimestamp) + ","
                + proxLen;
    }

    public int getVarDataLength() {
        return proxEntries.size();
    }

    public String headlineHeaderAndVarData(boolean useBurstForm) {
        return headlineHeader() + "," + ProxEntry.serializeHeadline(useBurstForm);
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
		} else {
			if (proxEntries.size() == 0)
				returnVal += serializeHeader() + "," + ProxEntry.serializeNobodySeen() + "\n";
			for (ProxEntry p : proxEntries)
				returnVal += serializeHeader() + "," + p.serialize() + "\n";
		}
		if (returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
		return returnVal;
	}

    public LogEntry copyMe(String dataMessageCustomPrefixIn) {
        LogEntryProximity e = new LogEntryProximity();
        e.dataMessageCustomPrefix = dataMessageCustomPrefixIn;
        return e;
    }
}
