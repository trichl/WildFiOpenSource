package net.timm.wildfidecoder.decoder.entries;

import java.util.Formatter;
import java.util.Locale;

import static java.lang.Math.floor;

public class MetaDataEntry {
    public double receiveTimeDouble;
    public String group;
    public String ownId;
    public long sendOffset;
    public long timestampLastSync;
    public long lastSyncType;
    public long voltage;
    public long lastErrorId;
    public long errorCnt;
    public long timestamp;
    public long swVersion;
    public long confVersion;
    public long syncCounter;

    public static String serializeHeader() {
        return "receiveTimeDouble,calculatedTimeOnTags,group,ownId,sendOffset,timestampLastSync,lastSyncType,voltage,lastErrorId,errorCnt,timestamp,swVersion,confVersion,syncCounter\n";
    }

    public String serialize() {
        return receiveTimeDouble
                + "," /* + (new Formatter(Locale.US).format("%.3f", calculatedTimeOnTags)) */
                //+ "," + (new Formatter(Locale.US).format("%03.0f", floor(receiveTimeDouble / 3600)))
                //+ ":" + (new Formatter(Locale.US).format("%02.0f", floor((receiveTimeDouble / 60) % 3600)))
                //+ ":" + (new Formatter(Locale.US).format("%02.0f", floor(receiveTimeDouble % 60)))
                //+ "," + mac + "," + messageLengthLong + "," + messageType
                + "," + group
                + "," + ownId
                + "," + sendOffset
                + "," + timestampLastSync
                + "," + lastSyncType
                + "," + voltage
                + "," + lastErrorId
                + "," + errorCnt
                + "," + timestamp
                + "," + swVersion
                + "," + confVersion
                + "," + syncCounter
                + "\n";
    }
}
