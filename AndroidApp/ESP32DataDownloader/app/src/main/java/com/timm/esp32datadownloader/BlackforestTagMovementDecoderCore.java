package com.timm.esp32datadownloader;

import android.util.Log;

import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.*;

import static java.lang.Math.toIntExact;

class DecoderParameters {
    public final static String PREFIX = "NNNNNN:PPPPPPPP:VVVV:"; // "NNNNNN:BBBB:HHHH:VVVV:"
    public final static String POSTFIX = "NNNNNN";
    public final static int PAYLOAD_LEN_PAGES = 64; // 32 for old Software (Pangolin), 64 for new Software
    public final static int PAYLOAD_LEN = PAYLOAD_LEN_PAGES * 2048;
    public final static int PAYLOAD_LEN_COMPLETE = (PREFIX.length() + PAYLOAD_LEN + POSTFIX.length());
}

class BlackforestTrackerSetting {
    public static final double CONVERSION_FACTOR_MAX_2G = 0.000061035;
    public static final int HEADER_LEN = 29;
    public static final double ACC_DATA_TO_G_CONVERSION_FACTOR = CONVERSION_FACTOR_MAX_2G; // converts raw data to mg
}

class FlashBlock {
    public String tagName;
    public long pagePointer;
    public long voltage;
    public byte[] payload;

    public static boolean check(ArrayList<FlashBlock> flashBlocks) {
        boolean first = true;
        long startPageAddress = 0;
        for(FlashBlock h : flashBlocks) {
            if(first) {
                first = false;
                startPageAddress = h.pagePointer;
                Log.d("decoder-check", "starting at page address " + startPageAddress);
            }
            else {
                if(h.pagePointer != startPageAddress + DecoderParameters.PAYLOAD_LEN_PAGES) {
                    Log.d("decoder-check", "error at page address " + h.pagePointer + " vs. " + startPageAddress + " + 32");
                    return false;
                }
                startPageAddress += DecoderParameters.PAYLOAD_LEN_PAGES;
            }
        }
        Log.d("decoder-check", "all good, page pointers are consecutive");
        return true;
    }

    public void fillPrefixData(String prefixString, boolean debug) {
        tagName = prefixString.substring(0, 6);
        pagePointer = Long.parseLong(prefixString.substring(7, 15), 16);
        voltage = Long.parseLong(prefixString.substring(16, 20), 16);
        if(debug) Log.d("decoder-fillprefixdata", "tagName: " + tagName + ", pagePointer: " + pagePointer + ", voltage: " + voltage);
    }

    public FlashBlock(byte[] data, boolean debug) {
        byte[] prefix = Arrays.copyOfRange(data, 0, DecoderParameters.PREFIX.length());
        String prefixString = new String(prefix, StandardCharsets.UTF_8);
        byte[] postfix = Arrays.copyOfRange(data, data.length - DecoderParameters.POSTFIX.length(), data.length);
        String postfixString = new String(postfix, StandardCharsets.UTF_8);
        payload = Arrays.copyOfRange(data, DecoderParameters.PREFIX.length(), data.length - DecoderParameters.POSTFIX.length());
        //Log.d("decoder", "Prefix: " + prefixString + ", Postfix: " + postfixString + ", PayloadLen: " + payload.length);
        fillPrefixData(prefixString, debug);
    }
}

class AccEntry {
    public long accCalculatedTimestampEstimation, accConsecutiveNumber;
    public double accX, accY, accZ; // in g
    public double accCalculatedMagnitudeWithoutEarthGravitation;

    public static String serializeHeadline() {
        return "accCalculatedTimestampEstimation,accConsecutiveNumber,accX,accY,accZ,accCalculatedMagnitudeWithoutEarthGravitation";
    }

    public String serialize() {
        return accCalculatedTimestampEstimation+","
                +accConsecutiveNumber+","
                +accX+","
                +accY+","
                +accZ+","
                +accCalculatedMagnitudeWithoutEarthGravitation;
    }

    public static long calculateTimestampFraction(AccEntry a, int pos, int listSize, long lastTimestamp, double accFrequency) {
        double fraction = 1 / accFrequency; // time between acc logs
        fraction = (listSize - 1 - pos) * fraction;
        fraction *= 1000;
        return (lastTimestamp * 1000) - (long) fraction;
    }

    public static int createAccData(String fifo, long timestamp, ArrayList<AccEntry> accEntries, double accFrequency) {
        int len = fifo.length();
        int countAccEntries = 0;
        for(int i=0; i<(len/12); i++) {
            AccEntry a = new AccEntry();
            String dataset = fifo.substring((i*12),(i*12)+12);
            short x,y,z;
            x = (short) Integer.parseInt(dataset.substring(2, 4)+dataset.substring(0, 2),16);
            y = (short) Integer.parseInt(dataset.substring(6, 8)+dataset.substring(4, 6),16);
            z = (short) Integer.parseInt(dataset.substring(10, 12)+dataset.substring(8, 10),16);
            a.accX = x * BlackforestTrackerSetting.ACC_DATA_TO_G_CONVERSION_FACTOR;
            a.accY = y * BlackforestTrackerSetting.ACC_DATA_TO_G_CONVERSION_FACTOR;
            a.accZ = z * BlackforestTrackerSetting.ACC_DATA_TO_G_CONVERSION_FACTOR;
            a.accCalculatedMagnitudeWithoutEarthGravitation = Math.sqrt(Math.pow(a.accX, 2) + Math.pow(a.accY, 2) + Math.pow(a.accZ, 2)); // between 0 and 3.464 G
            a.accCalculatedMagnitudeWithoutEarthGravitation -= 1; // remove 1G = earth gravitation
            a.accCalculatedMagnitudeWithoutEarthGravitation = Math.abs(a.accCalculatedMagnitudeWithoutEarthGravitation); // absolute value because if result negative then there is still a force
            a.accConsecutiveNumber = countAccEntries;
            a.accCalculatedTimestampEstimation = calculateTimestampFraction(a, i, len/12, timestamp, accFrequency);
            accEntries.add(a);
            countAccEntries++;
        }
        return countAccEntries;
    }
}

class LogEntry {
    public long utcTimestamp, startCnt, voltage, temperature, humidity, pressure, temperatureBmx, fifoLen, lastErrorId, errorCnt;
    public double voltageDouble, temperatureDouble, humidityDouble;
    private boolean plausibilityCheckOkay;
    public ArrayList<AccEntry> accEntries = new ArrayList<>();

    public boolean isPlausible() { return plausibilityCheckOkay; }

    public LogEntry(String dataset, boolean onlyHeader, double accFrequency, boolean debug) {
        if(dataset.length() < (BlackforestTrackerSetting.HEADER_LEN * 2)) {
            plausibilityCheckOkay = false;
        }
        else {
            utcTimestamp = Long.parseLong(dataset.substring(0, 8), 16);
            startCnt = Long.parseLong(dataset.substring(8, 16), 16);
            voltage = Long.parseLong(dataset.substring(16, 24), 16);
            temperature = Long.parseLong(dataset.substring(24, 28), 16);
            humidity = Long.parseLong(dataset.substring(28, 36), 16);
            pressure = Long.parseLong(dataset.substring(36, 44), 16);
            temperatureBmx = Long.parseLong(dataset.substring(44, 48), 16);
            fifoLen = Long.parseLong(dataset.substring(48, 52), 16);
            lastErrorId = Long.parseLong(dataset.substring(52, 54), 16);
            errorCnt = Long.parseLong(dataset.substring(54, 58), 16);
            voltageDouble = voltage / 1000.;
            temperatureDouble = temperature / 100.;
            humidityDouble = humidity / 1000.;
            plausibilityCheckOkay = plausibilityCheck(this, debug);
            if(!onlyHeader) {
                if(dataset.length() < 58 + toIntExact(2 * fifoLen)) plausibilityCheckOkay = false;
                else AccEntry.createAccData(dataset.substring(58, 58 + toIntExact(2 * fifoLen)), utcTimestamp, accEntries, accFrequency);
            }
        }
    }

    public static int getFifoLengthInBytesFromHeaderString(String line) {
        int fifoLen = (int) Long.parseLong(line.substring(48,52), 16);
        return fifoLen;
    }

    public static boolean plausibilityCheck(LogEntry e, boolean debug) {
        //long timestampNowPlusOneDay = Instant.now().getEpochSecond() + (24*60*60);
        long timestampNowPlusOneDay = (System.currentTimeMillis() / 1000L) + (24*60*60);

        if(e.utcTimestamp < 1546300800) {
            if(debug) Log.d("decoder-plausibility", "timestamp not plausible "+e.utcTimestamp);
            return false;
        }
        if(e.utcTimestamp > timestampNowPlusOneDay) {
            if(debug) Log.d("decoder-plausibility", "date not plausible "+e.utcTimestamp);
            return false;
        }
        if(e.voltageDouble < 2.0) {
            if(debug) Log.d("decoder-plausibility", "voltage not plausible "+e.voltageDouble);
            return false;
        }
        if(e.voltageDouble > 5.0) {
            if(debug) Log.d("decoder-plausibility", "voltage not plausible "+e.voltageDouble);
            return false;
        }
        if(e.temperatureDouble < -50.0) {
            if(debug) Log.d("decoder-plausibility", "temperature not plausible "+e.temperatureDouble);
            return false;
        }
        if(e.temperatureDouble > 120.0) {
            if(debug) Log.d("decoder-plausibility", "temperature not plausible "+e.temperatureDouble);
            return false;
        }
        if(e.humidityDouble < 0.0) {
            if(debug) Log.d("decoder-plausibility", "humidity not plausible "+e.humidityDouble);
            return false;
        }
        if(e.humidityDouble > 110.0) {
            if(debug) Log.d("decoder-plausibility", "humidity not plausible "+e.humidityDouble);
            return false;
        }
        if(e.fifoLen < 100) {
            if(debug) Log.d("decoder-plausibility", "fifoLen not plausible "+e.fifoLen);
            return false;
        }
        if(e.fifoLen > 8000) {
            if(debug) Log.d("decoder-plausibility", "fifoLen not plausible "+e.fifoLen);
            return false;
        }
        return true;
    }

    public static boolean headerIsPlausible(String line) {
        if(line.length() != (BlackforestTrackerSetting.HEADER_LEN * 2)) return false;
        LogEntry e = new LogEntry(line, true, 1, false);
        return e.plausibilityCheckOkay;
    }

    public static int estimateDataOffsetFromHeaderString(String line) {
        String header = "";
        for(int i = 0; i < line.length() - (BlackforestTrackerSetting.HEADER_LEN * 2); i += 2) {
            header = line.substring(i, i + (BlackforestTrackerSetting.HEADER_LEN * 2));
            //System.out.println("Header: " + header);
            if(headerIsPlausible(header)) {
                //System.out.println("-> is plausible @" + (i / 2));
                return (i / 2);
            }
        }
        return -1;
    }

    public double bmxTempToCelsius() {
        double temp = temperatureBmx;
        if(temperatureBmx > 0x8000) {
            // f(0xFFFF) = 23 - (0.5^9)
            // f(0x8001) = -41 + (0.5^9)
            double m = (-41. + Math.pow(0.5,9)) - (23. - Math.pow(0.5,9));
            m = m / (0x8001 - 0xFFFF);
            double b = (-41. + Math.pow(0.5,9)) - (m*0x8001);
            temp = (m * temp) + b;
        }
        else {
            temp *= 2.;
            temp /= 10.;
            temp += 2300.;
            temp /= 100;
        }
        return temp;
    }

    public static String utcTimestampToString(long utcTimestampIn) {
        /*Instant instant = Instant.ofEpochSecond(utcTimestampIn);;
        final DateTimeFormatter formatter = DateTimeFormatter.ofPattern("E dd.MM.yy HH:mm:ss");
        final String utcDateFormatted = instant.atZone(ZoneId.of("GMT")).format(formatter);
        return utcDateFormatted;*/
        utcTimestampIn *= 1000;
        SimpleDateFormat dateFormat = new SimpleDateFormat("E dd.MM.yy HH:mm:ss");
        return dateFormat.format(utcTimestampIn);
    }

    public static String utcTimestampToStringWithoutWeekday(long utcTimestampIn) {
        utcTimestampIn *= 1000;
        SimpleDateFormat dateFormat = new SimpleDateFormat("dd.MM.yy HH:mm:ss");
        return dateFormat.format(utcTimestampIn);
    }

    public static String serializeHeadlineOnlyHeader() {
        return "utcTimestamp,utcDate,startCnt,voltage,temperature,humidity,pressure,temperatureBmx,fifoLen,lastErrorId,errorCnt,voltageDouble,tempBmxDouble,temperatureDouble,heightMeter,humidityDouble";
    }

    public String serializeOnlyHeader() {
        return utcTimestamp
                +","+utcTimestampToString(utcTimestamp)+","
                +startCnt+","
                +voltage+","
                +temperature+","
                +humidity+","
                +pressure+","
                +temperatureBmx+","
                +fifoLen+","
                +lastErrorId+","
                +errorCnt+","
                +(new Formatter(Locale.US).format("%.3f", voltageDouble))+","
                +(new Formatter(Locale.US).format("%.3f", bmxTempToCelsius()))+","
                +(new Formatter(Locale.US).format("%.2f", temperatureDouble))+","
                +(new Formatter(Locale.US).format("%.4f", pressureToHeight()))+","
                +(new Formatter(Locale.US).format("%.3f", humidityDouble));
    }

    public static String serializeHeadlineWithAcc() {
        return serializeHeadlineOnlyHeader()+","+AccEntry.serializeHeadline();
    }

    public String serializeWithAcc() {
        String returnVal = "";
        for(AccEntry a : accEntries) {
            returnVal += serializeOnlyHeader() + "," + a.serialize() + "\n";
        }
        if(returnVal.length() > 0) returnVal = returnVal.substring(0, returnVal.length() - 1);
        return returnVal;
    }

    public double pressureToHeight() {
        double height;
        height = (1 - Math.pow((pressure / 100.) / 1013.25, 0.190284)) * 145366.45 * 0.3048;
        return height;
    }
}

class DataStream {
    byte[] stream;
    long firstPageAddress = 0; // 0 .. (2048*64 - 1)
    int estimatedDataOffset = 0;

    /*public long estimateDataOffsetOld() {
        long bytesBeforeStream = firstPageAddress * 2048;
        long offset = BlackforestTrackerSetting.getAccDataLen() - (bytesBeforeStream % BlackforestTrackerSetting.getAccDataLen());
        System.out.println("Estimated offset old: " + offset);
        return offset;
    }*/

    private int estimateDataOffset() {
        String line = "";
        for(int i = 0; i < 16000; i++) {
            line += String.format("%02X", stream[i]);
        }
        int offset = LogEntry.estimateDataOffsetFromHeaderString(line);
        return offset;
    }

    public LogEntry extractFirstLogEntry(double accFrequency, boolean debug) {
        String line = "";
        LogEntry e;
        if(debug) Log.d("decoder-first-log", "First log entry starts at " + estimatedDataOffset);
        for(int i = estimatedDataOffset; i < estimatedDataOffset + BlackforestTrackerSetting.HEADER_LEN; i++) {
            line += String.format("%02X", stream[i]);
        }
        if(debug) Log.d("decoder-first-log", "Header: " + line);
        int currentFifoLen = LogEntry.getFifoLengthInBytesFromHeaderString(line);
        if(currentFifoLen == 0) {
            if(debug) Log.d("decoder-first-log", "Could not determine fifo length");
            return null;
        }
        else if(debug) Log.d("decoder-first-log", "Fifo length: " + currentFifoLen);
        line = "";
        for(int i = estimatedDataOffset; i < estimatedDataOffset + BlackforestTrackerSetting.HEADER_LEN + currentFifoLen; i++) {
            line += String.format("%02X", stream[i]);
        }
        e = new LogEntry(line, false, accFrequency, debug);
        if(!e.isPlausible()){
            if(debug) Log.d("decoder-first-log", "Log entry dataset error");
            return null;
        }
        else {
            if(debug) Log.d("decoder-first-log", "first log entry " + e.serializeOnlyHeader() + " with " + e.accEntries.size() + " acc entries");
        }
        return e;
    }

    public LogEntry extractLastLogEntry(double accFrequency, boolean debug) {
        String line = "";
        LogEntry e;
        int offset = estimatedDataOffset;
        int currentFifoLen = 0;

        while(true) {
            for(int i = offset; i < offset + BlackforestTrackerSetting.HEADER_LEN; i++) {
                line += String.format("%02X", stream[i]);
            }
            currentFifoLen = LogEntry.getFifoLengthInBytesFromHeaderString(line);
            line = "";
            offset += BlackforestTrackerSetting.HEADER_LEN + currentFifoLen;
            if(offset > stream.length) {
                offset -= 2 * (BlackforestTrackerSetting.HEADER_LEN + currentFifoLen);
                break;
            }
        }
        if(debug) Log.d("decoder-last-log", "Last log entry at " + offset + " (total length " + stream.length + ")");
        line = "";
        for(int i = offset; i < offset + BlackforestTrackerSetting.HEADER_LEN + currentFifoLen; i++) {
            line += String.format("%02X", stream[i]);
        }
        if(debug) Log.d("decoder-last-log", "Line " + line);
        e = new LogEntry(line, false, accFrequency, debug);
        if(!e.isPlausible()){
            if(debug) Log.d("decoder-last-log", "Log entry dataset error"); // happens at first dataset
            return null;
        }
        else {
            if(debug) Log.d("decoder-last-log", "last log entry " + e.serializeOnlyHeader() + " with " + e.accEntries.size() + " acc entries");
        }
        return e;
    }

    public boolean testFunctionForTimestampEstimation(double accFrequency) {
        String line = "";
        int fifoLenFromCurrentLogEntry = 0;
        boolean foundFifoLen = false;
        LogEntry e = null;
        long timestampBefore = 0;
        long cnt = 0;
        long idDiff = 0;
        for(int i = estimatedDataOffset; i < stream.length; i++) {
            line += String.format("%02X", stream[i]);
            if(line.length() == (BlackforestTrackerSetting.HEADER_LEN * 2)) {
                fifoLenFromCurrentLogEntry = LogEntry.getFifoLengthInBytesFromHeaderString(line);
                foundFifoLen = true;
            }
            if(foundFifoLen && (line.length() == (BlackforestTrackerSetting.HEADER_LEN * 2) + (fifoLenFromCurrentLogEntry * 2))) {
                if(e != null) {
                    timestampBefore = e.utcTimestamp;
                }
                e = new LogEntry(line, true, accFrequency, true);
                if(timestampBefore != 0) {
                    long timestampDiff = e.utcTimestamp - timestampBefore;
                    if(timestampDiff != 4) {
                        long tempIdDiff = cnt - idDiff;
                        Log.d("testFunctionForTimestampEstimation", cnt + ": diff " + timestampDiff + " (+" + tempIdDiff + ")");
                        idDiff = cnt;
                    }
                }
                line = "";
                foundFifoLen = false;
                cnt++;
            }
        }
        return true;
    }

    public boolean loadIntoFile(BufferedWriter writer, long selectedStartTimestamp, long selectedEndTimestamp, long limit, boolean onlyHeader, double accFrequency) throws IOException {
        String line = "";
        int fifoLenFromCurrentLogEntry = 0;
        boolean foundFifoLen = false;
        LogEntry e;
        if(accFrequency <= 0) return false;
        if(onlyHeader) writer.write(LogEntry.serializeHeadlineOnlyHeader()+"\n");
        else writer.write(LogEntry.serializeHeadlineWithAcc()+"\n");
        boolean useLimit = (limit > 0);
        for(int i = estimatedDataOffset; i < stream.length; i++) {
            line += String.format("%02X", stream[i]);
            if(line.length() == (BlackforestTrackerSetting.HEADER_LEN * 2)) {
                fifoLenFromCurrentLogEntry = LogEntry.getFifoLengthInBytesFromHeaderString(line);
                foundFifoLen = true;
            }
            if(foundFifoLen && (line.length() == (BlackforestTrackerSetting.HEADER_LEN * 2) + (fifoLenFromCurrentLogEntry * 2))) {
                e = new LogEntry(line, onlyHeader, accFrequency, true);
                if(!e.isPlausible()){
                    Log.d("decoder", "log entry not plausible"); // happens at first dataset
                }
                else {
                    boolean doNotAddEntry = false;
                    if(selectedStartTimestamp > 0) {
                        if(e.utcTimestamp < selectedStartTimestamp) doNotAddEntry = true;
                    }
                    if(selectedEndTimestamp > 0) {
                        if(e.utcTimestamp > selectedEndTimestamp) {
                            return false;
                        }
                    }
                    if(doNotAddEntry) {
                        Log.d("decoder", "(skipped)");
                    }
                    else {
                        if(onlyHeader) writer.write(e.serializeOnlyHeader() + "\n");
                        else writer.write(e.serializeWithAcc() + "\n");
                        Log.d("decoder", "added " + e.serializeOnlyHeader() + " with " + e.accEntries.size() + " acc entries");
                    }
                }
                line = "";
                foundFifoLen = false;
                if(useLimit) {
                    limit--;
                    if(limit == 0) break;
                }
            }
        }
        return true;
    }

    public DataStream(ArrayList<FlashBlock> flashBlocks) {
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream( );
        boolean first = true;
        for(FlashBlock h : flashBlocks) {
            if(first) {
                first = false;
                firstPageAddress = h.pagePointer;
            }
            try {
                outputStream.write(h.payload);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        stream = outputStream.toByteArray();
        estimatedDataOffset = estimateDataOffset();
        if(estimatedDataOffset < 0) {
            Log.d("decoder", "WARNING: data offset could not be estimated (-1), using 0 instead");
            estimatedDataOffset = 0;
        }
        Log.d("decoder", "stream is " + stream.length + " Bytes long, starting at page address " + firstPageAddress + ", estimated data offset " + estimatedDataOffset);
    }
}

public class BlackforestTagMovementDecoderCore {
    private int fileLength = 0;
    private ArrayList<FlashBlock> flashBlocks = new ArrayList<>();
    private DataStream dataStream = null;

    public ArrayList<FlashBlock> getFlashBlocks() { return flashBlocks; }
    public DataStream getDataStream() { return dataStream; }

    public boolean loadSelectedFlashBlocksIntoDataStream(String selectedTagName) {
        Log.d("decoder", "removing everything except " + selectedTagName);
        ArrayList<FlashBlock> flashBlocksToRemove = new ArrayList<>();
        for(FlashBlock h : flashBlocks) {
            if(!h.tagName.equals(selectedTagName)) {
                flashBlocksToRemove.add(h);
            }
        }
        flashBlocks.removeAll(flashBlocksToRemove);
        if(!FlashBlock.check(flashBlocks)) {
            Log.d("decoder", "block error");
            return false;
        }
        dataStream = new DataStream(flashBlocks);
        return true;
    }

    public void debugFileBytes(byte[] fileData) {
        long numberFlashBlocks = fileData.length / DecoderParameters.PAYLOAD_LEN_COMPLETE;
        Log.d("decoder", "blocks = " + numberFlashBlocks + " (REST = " + (fileData.length % DecoderParameters.PAYLOAD_LEN_COMPLETE) + ")");
        for(int i=0; i<numberFlashBlocks; i++) {
            int index = i * DecoderParameters.PAYLOAD_LEN_COMPLETE;
            byte[] subArray = Arrays.copyOfRange(fileData, index, index + DecoderParameters.PAYLOAD_LEN_COMPLETE);
            FlashBlock h = new FlashBlock(subArray, true);
            Log.d("decoder", "block " + i + ": " + subArray.length + " bytes: " + h.tagName);
        }
    }

    public boolean loadFileBytesIntoFlashBlocks(byte[] fileData, ArrayList<String> tagNamesOut, boolean debug) {
        fileLength = fileData.length;
        Log.d("decoder", "file length: " + fileData.length + " bytes");
        if(fileLength % DecoderParameters.PAYLOAD_LEN_COMPLETE != 0) {
            Log.d("decoder", "file corrupt, cannot divide by " + DecoderParameters.PAYLOAD_LEN_COMPLETE + " bytes");
            return false;
        }
        else {
            long numberFlashBlocks = fileLength / DecoderParameters.PAYLOAD_LEN_COMPLETE;
            Log.d("decoder", "file okay, blocks = " + numberFlashBlocks);
            for(int i=0; i<numberFlashBlocks; i++) {
                int index = i * DecoderParameters.PAYLOAD_LEN_COMPLETE;
                byte[] subArray = Arrays.copyOfRange(fileData, index, index + DecoderParameters.PAYLOAD_LEN_COMPLETE);
                if(debug) Log.d("decoder", "block " + i + ": " + subArray.length + " bytes");
                FlashBlock h = new FlashBlock(subArray, debug);
                flashBlocks.add(h);
                if(!tagNamesOut.contains(h.tagName)) tagNamesOut.add(h.tagName);
            }
            Log.d("decoder", "loaded data stream of " + numberFlashBlocks + " blocks");
            Log.d("decoder", "found " + tagNamesOut.size() + " tag name(s) in data");
            for(String tagName : tagNamesOut) {
                //Log.d("decoder", "found tag name: " + tagName);
            }
        }
        return true;
    }
}
