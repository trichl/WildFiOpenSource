package net.timm.wildfidecoder.decoder;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.entries.*;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.TimeZone;

public class LogEntryManager {
    ArrayList<LogEntry> logEntryTypes = new ArrayList<>();

    public LogEntryManager() {
        logEntryTypes.add(new LogEntry1HzGPSAccREV5());
        logEntryTypes.add(new LogEntryProximity());
        logEntryTypes.add(new LogEntryGPSOnly());
        logEntryTypes.add(new LogEntryAccMagGyroREV6());
        logEntryTypes.add(new LogEntry1HzGPSAccMagGyroREV6());
        logEntryTypes.add(new LogEntryAccREV6());
        logEntryTypes.add(new LogEntryProximityAccEnv());
        logEntryTypes.add(new LogEntryProximityAccMagGyroEnv());
    }

    public LogEntry createEntry(String line, String dataMessageCustomPrefix) {
        LogEntry eReturn;
        String startPrefix = line.substring(0, 6);
        for(LogEntry eType : logEntryTypes) {
            if(startPrefix.equals(eType.prefix)) {
                eReturn = eType.copyMe(dataMessageCustomPrefix);
                return eReturn;
            }
        }
        return null;
    }

    public int estimateDataOffsetFromFirstStreamBytes(String firstStreamData) {
        int firstBytesMax = 120;
        if(firstStreamData.length() < 120) firstBytesMax = firstStreamData.length();
        Log.d("decoder-estimate-offset", "first bytes: " + firstStreamData.substring(0, firstBytesMax));
        int foundStartIndex = -1;
        String foundPrefix = "";
        for(LogEntry e : logEntryTypes) {
            int temp = firstStreamData.indexOf(e.prefix);
            //System.out.println(("TEST: " + temp + "," +e.prefix));
            if(temp != -1) {
                if(foundStartIndex == -1) { // first prefix found
                    foundStartIndex = temp;
                    foundPrefix = e.prefix;
                }
                else if(temp < foundStartIndex) {
                    foundStartIndex = temp;
                    foundPrefix = e.prefix;
                }
            }
        }
        if (foundStartIndex == -1) return -1;

        Log.d("decoder-estimate-offset", "plausible offset @" + foundStartIndex + ", Prefix: " + foundPrefix);
        return foundStartIndex / 2;

        /*
        //System.out.println()
        if (foundStartIndex + (DataStructure1Hz.currentFifoLenIndex) + DataStructure1Hz.currentFifoLenLength > firstStreamData.length())
            return -1;
        long fifoLength = Long.parseLong(firstStreamData.substring(foundStartIndex + DataStructure1Hz.currentFifoLenIndex, foundStartIndex + DataStructure1Hz.currentFifoLenIndex + DataStructure1Hz.currentFifoLenLength), 16);
        if ((fifoLength < DataStructure1Hz.maxFifoLength) && (fifoLength > 0)) {

        }
        firstStreamData = firstStreamData.substring(foundStartIndex + 6);
        System.out.println(firstStreamData.length());
        */
    }

    public static String utcTimestampToString(long utcTimestampIn) {
        utcTimestampIn *= 1000;
        SimpleDateFormat dateFormat = new SimpleDateFormat("E dd.MM.yy HH:mm:ss");
        dateFormat.setTimeZone(TimeZone.getTimeZone("UTC"));
        return dateFormat.format(utcTimestampIn);
    }

    public static String utcTimestampToStringWithoutWeekday(long utcTimestampIn) {
        utcTimestampIn *= 1000;
        SimpleDateFormat dateFormat = new SimpleDateFormat("dd.MM.yy HH:mm:ss");
        dateFormat.setTimeZone(TimeZone.getTimeZone("UTC"));
        return dateFormat.format(utcTimestampIn);
    }

    public static String utcTimestampToStringForFileName(long utcTimestampIn) {
        utcTimestampIn *= 1000;
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyMMdd_HHmmss");
        dateFormat.setTimeZone(TimeZone.getTimeZone("UTC"));
        return dateFormat.format(utcTimestampIn);
    }

    public static boolean timestampPlausible(long utcTimestampIn, boolean debug) {
        long timestampNowPlusOneDay = (System.currentTimeMillis() / 1000L) + (24 * 60 * 60);
        if ((utcTimestampIn < 1546300800) && (utcTimestampIn > 1000000)) {
            if (debug) Log.d("decoder-plausibility", "timestamp not plausible " + utcTimestampIn);
            return false;
        }
        if (utcTimestampIn > timestampNowPlusOneDay) {
            if (debug) Log.d("decoder-plausibility", "date not plausible " + utcTimestampIn);
            return false;
        }
        return true;
    }

    public static boolean fifoLenPlausible(long fifoLen, boolean debug) {
        if (fifoLen == 0) {
            if (debug) Log.d("decoder-plausibility", "fifoLen not plausible " + fifoLen);
            return false;
        }
        if (fifoLen > (1024 * 8)) {
            if (debug) Log.d("decoder-plausibility", "fifoLen not plausible " + fifoLen);
            return false;
        }
        return true;
    }

    public static boolean proxLenPlausible(long proxLen, boolean debug) {
        if (proxLen > 512) {
            if (debug) Log.d("decoder-plausibility", "proxLen not plausible " + proxLen);
            return false;
        }
        return true;
    }

}
