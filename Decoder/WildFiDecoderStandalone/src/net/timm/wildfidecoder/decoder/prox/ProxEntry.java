package net.timm.wildfidecoder.decoder.prox;

import java.util.ArrayList;

public class ProxEntry {

    public String id = "";
    public int rssi = 0;

    public static String serializeHeadline(boolean useBurstForm) {
		if (useBurstForm)
			return "proxIdBurst,proxRssiBurst";
		else
        	return "proxId,proxRssi";
    }

    public String serialize() {
        return id + "," + rssi;
    }

	public String serializeId() {
		return id;
	}

	public String serializeRssi() {
		return Integer.toString(rssi);
	}

    public static String serializeNobodySeen() {
        //return "nobody,nobody";
        return ",";
    }

    public static String serializeEmpty() {
        return "-,-";
    }

    public static void createProxData(String vardata, long utcTimestamp, ArrayList<ProxEntry> proxEntries) {
        int len = vardata.length();
        for(int i=0; i<(len/6); i++) {
            ProxEntry p = new ProxEntry();
            String dataset = vardata.substring((i*6),(i*6)+6);
            p.id = dataset.substring(0, 2) + dataset.substring(2, 4);
            p.rssi = -Integer.parseInt(dataset.substring(4, 6), 16);
            proxEntries.add(p);
        }
    }
}
