package net.timm.wildfidecoder.decoder;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.entries.*;
import net.timm.wildfidecoder.decoder.prox.ProxEntry;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Formatter;
import java.util.Locale;
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
        logEntryTypes.add(new LogEntryProximityAcc());
        logEntryTypes.add(new LogEntryProximityAccMagGyro());
        logEntryTypes.add(new LogEntry1HzGPSAccMagREV6());
        logEntryTypes.add(new LogEntryAccMagREV6());
        logEntryTypes.add(new LogEntryProximityEnv());
        logEntryTypes.add(new LogEntryRawGPS());
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
        Log.d("decoder-estimate-offset", "first 120/"+firstStreamData.length()/2+" byte : " + firstStreamData.substring(0, firstBytesMax));
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

    public static String gpsLatLngToText(double in) {
        if(in == 0.0f) { return "NA"; }
        return (new Formatter(Locale.US).format("%.7f", in)).toString();
    }

    public static String gpsHdopToText(double in) {
        if(in == 0.0f) { return "NA"; }
        return (new Formatter(Locale.US).format("%.1f", in)).toString();
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

    public static boolean gpsTTFPlausible(long ttf, boolean debug) {
        if (ttf == 0) {
            if (debug) Log.d("decoder-plausibility", "TTF not plausible " + ttf);
            return false;
        }
        if (ttf > 600) {
            if (debug) Log.d("decoder-plausibility", "fifoLen not plausible " + ttf);
            return false;
        }
        return true;
    }

    public static boolean fifoLenPlausible(long fifoLen, boolean debug) {
        if (fifoLen == 0) {
            if (debug) Log.d("decoder-plausibility", "WARNING fifoLen = 0");
            //return false; // happened sometimes, removed so that other messages still get decoded
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

    public static boolean rawGPSMeasurementsPlausible(long measurementLen, boolean debug) {
        if (measurementLen > 50) {
            if (debug) Log.d("decoder-plausibility", "measurementLen not plausible " + measurementLen);
            return false;
        }
        return true;
    }

    public static boolean observationLenPlausible(int observationLen, boolean debug) {
        if (observationLen != 34) {
            if (debug) Log.d("decoder-plausibility", "observationLen not plausible " + observationLen);
            return false;
        }
        return true;
    }

    public static boolean utcProximityTimePlausible(long utcTimestamp, boolean debug) {
        if((utcTimestamp % 60) != 0) {
            if (debug) Log.d("decoder-plausibility", "utcTimestamp not plausible " + utcTimestamp);
            return false;
        }
        return true;
    }

    public static boolean proximityRSSIsMakeSense(ArrayList<ProxEntry> proxEntries, boolean debug) {
        for(ProxEntry p : proxEntries) {
            if(p.rssi < -100) {
                if (debug) Log.d("decoder-plausibility", "proximity RSSI not plausible " + p.rssi);
                return false;
            }
        }
        return true;
    }
}