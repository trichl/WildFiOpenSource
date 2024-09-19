package net.timm.wildfidecoder.decoder;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.entries.LogEntry;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.io.File;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.text.NumberFormat;
import java.util.*;

public class EspNowDecoder {
    // 8000 IS NOT LARGE ENOUGH!!! CORRUPTED FFs CAN BE LIKE 6873, THEN NOT POSSIBLE TO EXTRACT REMAINING MSG!
    private static int LINE_BUFFER_SIZE = 8000;

    LogEntryManager logEntryManager = new LogEntryManager();
    public ArrayList<EspNowMessage> espNowMessagesAll = new ArrayList<>();
    public ArrayList<EspNowMessage> espNowMessagesSingleTag = new ArrayList<>();
    boolean stepTwoExecuted = false;
    String dataMessageCustomPrefix = "";
    byte[] stream;
    int estimatedDataOffset = 0;
    String name = "";

    public class FillByteResults {
        public int iterator = 0;
        public String line = "";
    }

    public FillByteResults fillBytesAgain(String line, int iterator) {
        int filledBytes = 0;
        int newIterator = iterator;
        while (newIterator < stream.length) {
            line += String.format("%02X", stream[newIterator]);
            newIterator++;
            filledBytes++;
            if ((line.length() == LogEntry.INDEX(LINE_BUFFER_SIZE)) || (newIterator == stream.length)) {
                break;
            }
        }
        Log.d("decoder", "filled bytes: " + filledBytes + "/" + line.length()/2);

        FillByteResults r = new FillByteResults();
        r.iterator = newIterator;
        r.line = line;
        return r;
    }

    public boolean stepFiveWriteResultToFile(String fileNameCsvComplete, long selectedStartTimestamp, long selectedEndTimestamp, IMUSettings imuSettings, int outputModulo, boolean useBurstForm, int imuFrequency) throws IOException {
        BufferedWriter writerComplete = null;
        String line = "";
        int corruptedFlashCounter = 0;
        long corruptedFlashBytes = 0;
        int unknownPrefixCounter = 0;
        int plausibilityCheckFailedCounter = 0;
        int lineCnt = 0;
        int entryCnt = 0;
        LogEntry e;
        int iterator = estimatedDataOffset;
        String previousPrefix = "";
        ArrayList<String> listOfWrittenFiles = new ArrayList<>();
        boolean entireLineError = false;

        while (iterator < stream.length) {
            line += String.format("%02X", stream[iterator]);
            iterator++;

            if ((line.length() == LogEntry.INDEX(LINE_BUFFER_SIZE)) || (iterator == stream.length)) {
                entireLineError = false;
                entryCnt++;
                // NEW: fixing flash FFFF errors
                if(line.startsWith("FFFFFFFFFFFFFFFFFFFFFFFF")) {
                    //System.out.println("\n\n" + line + "\n\n");
                    Log.d("decoder", entryCnt + ": CORRUPTED FLASH MESSAGE! line starts with FFFF..., trying to repair");
                    int countFFs = 0;
                    for(int i = 0; i < line.length(); i++) {
                        if(line.charAt(i) == 'F') { countFFs++; }
                        else { break; }
                    }
                    countFFs /= 2;
                    Log.d("decoder", entryCnt + ": FFs counted: " + countFFs);
                    corruptedFlashCounter++;
                    corruptedFlashBytes += countFFs;
                    int pos = logEntryManager.estimateDataOffsetFromFirstStreamBytes(line);
                    if(pos > 0) {
                        Log.d("decoder", entryCnt + ": got next start at index " + pos + "/" + line.length()/2);
                        line = line.substring(pos*2);
                        FillByteResults r = fillBytesAgain(line, iterator); // BRAND NEW
                        line = r.line;
                        iterator = r.iterator;
                        String lineTruncated = line;
                        if(line.length() > 60) lineTruncated = lineTruncated.substring(0, 60) + "(..)";
                        Log.d("decoder", entryCnt + ": new line: " + lineTruncated);
                    }
                    else {
                        Log.d("decoder", entryCnt + ": FATAL_A, could not repair FFs (no new prefix found), but continue");
                    }
                }
                // END NEW

                e = logEntryManager.createEntry(line, dataMessageCustomPrefix);
                if(e == null) {
                    Log.d("decoder", entryCnt + ": unknown prefix, decode ERROR at line " + line);
                    unknownPrefixCounter++;

                    // NEW: fixing wrong prefix errors
                    int pos = logEntryManager.estimateDataOffsetFromFirstStreamBytes(line);
                    if(pos > 0) {
                        Log.d("decoder", entryCnt + ": got next start at index " + pos + "/" + line.length()/2);
                        line = line.substring(pos*2);
                        FillByteResults r = fillBytesAgain(line, iterator); // BRAND NEW
                        line = r.line;
                        iterator = r.iterator;
                        String lineTruncated = line;
                        if(line.length() > 60) lineTruncated = lineTruncated.substring(0, 60) + "(..)";
                        Log.d("decoder", entryCnt + ": new line: " + lineTruncated);

                        e = logEntryManager.createEntry(line, dataMessageCustomPrefix);
                        if(e == null) {
                            Log.d("decoder", entryCnt + ": FATAL_B, could not repair, GET NEXT LINE");
                            entireLineError = true;
                        }
                    }
                    else {
                        Log.d("decoder", entryCnt + ": FATAL_C, could not repair, GET NEXT LINE");
                        entireLineError = true;
                    }
                    // END NEW
                }
                if(!entireLineError) {
                    boolean doNotAddEntry = false;
                    e.decode(name, line, true, imuSettings, imuFrequency);
                    if (e.plausibilityCheckOkay == false) {
                        Log.d("decoder", entryCnt + ": decode plausibility ERROR at line " + line);
                        plausibilityCheckFailedCounter++;

                        // NEW: try to fix in case implausible message: remove prefix, try to find new prefix
                        Log.d("decoder", entryCnt + ": skip message, try to find next");
                        line = line.substring(2); // remove first byte to start finding new prefix
                        FillByteResults r = fillBytesAgain(line, iterator); // BRAND NEW
                        line = r.line;
                        iterator = r.iterator;
                        int pos = logEntryManager.estimateDataOffsetFromFirstStreamBytes(line);
                        if (pos > 0) {
                            Log.d("decoder", entryCnt + ": got next start at index " + pos + "/" + line.length() / 2);
                            String lineTruncated = line.substring(pos * 2);
                            if (lineTruncated.length() > 60) lineTruncated = lineTruncated.substring(0, 60) + "(..)";
                            Log.d("decoder", entryCnt + ": new line: " + lineTruncated);

                            e.entryLengthInBytes = pos; // mock entry length for next iteration
                            doNotAddEntry = true; // skip this entry, take next one
                        } else {
                            Log.d("decoder", entryCnt + ": FATAL_D, could not repair, NEXT LINE");
                            entireLineError = true;
                        }
                        // END NEW: try to fix in case implausible message: remove prefix, try to find new prefix
                    }

                    if(!entireLineError) {
                        if (selectedStartTimestamp > 0) {
                            if (e.utcTimestamp < selectedStartTimestamp) doNotAddEntry = true;
                        }
                        if (selectedEndTimestamp > 0) {
                            if (e.utcTimestamp > selectedEndTimestamp) {
                                return false;
                            }
                        }
                        if (doNotAddEntry) {
                            Log.d("decoder", entryCnt + ": (skipped)");
                        } else {
                            long writeStartTime = System.nanoTime();
                            if (!e.prefix.equals(previousPrefix)) {
                                if (writerComplete != null)
                                    writerComplete.close();

                                if (new File(fileNameCsvComplete + "_" + e.prefix + ".csv").isFile()) {
                                    // file exists already
                                    if (listOfWrittenFiles.contains(fileNameCsvComplete + "_" + e.prefix + ".csv")) {
                                        // file was created within this function -> append
                                        writerComplete = new BufferedWriter(new FileWriter(fileNameCsvComplete + "_" + e.prefix + ".csv", true));
                                    } else {
                                        // file was there before -> overwrite
                                        Log.d("decoder", entryCnt + ": File " + fileNameCsvComplete + "_" + e.prefix + ".csv exists already -> overwrite");
                                        listOfWrittenFiles.add(fileNameCsvComplete + "_" + e.prefix + ".csv");
                                        writerComplete = new BufferedWriter(new FileWriter(fileNameCsvComplete + "_" + e.prefix + ".csv"));
                                        writerComplete.write(e.headlineHeaderAndVarData(useBurstForm) + "\n");
                                        writerComplete.flush();
                                    }
                                } else {
                                    // file does not exist
                                    listOfWrittenFiles.add(fileNameCsvComplete + "_" + e.prefix + ".csv");
                                    writerComplete = new BufferedWriter(new FileWriter(fileNameCsvComplete + "_" + e.prefix + ".csv"));
                                    writerComplete.write(e.headlineHeaderAndVarData(useBurstForm) + "\n");
                                    writerComplete.flush();
                                }
                                //Log.d("decoder","[" + lineCnt + "]  switching data type " + previousPrefix + " -> " + e.prefix);
                            }
                            String dataToWrite = e.serializeHeaderAndVarData(useBurstForm);
                            writerComplete.write(dataToWrite + "\n");
                            writerComplete.flush(); // IMPORTANT: NEEDED! otherwise writerComplete sometimes randomly stops writing into file!!! gonna hate Java

                            //System.out.println("[" + lineCnt + "] " + dataToWrite); // REMOVE

                            long writeTime = (System.nanoTime() - writeStartTime) / 1000L;
                            if (lineCnt % outputModulo == 0) {
                                Log.d("decoder", "[" + lineCnt + "] added " + e.serializeHeader() + " with " + e.getVarDataLength() + " vardata in " + writeTime + " ms");
                            }
                            lineCnt++;
                            previousPrefix = e.prefix;
                        }
                    }
                }

                if(!entireLineError) {
                    iterator = (iterator - (line.length() / 2)) + e.entryLengthInBytes;
                    line = "";
                }
                else { // skip entire line, move to next line, iterator is pointing to entire new line
                    line = "";
                }
            }
        }
        writerComplete.close();
        Log.d("decoder", entryCnt + ": *** DECODING OF TAG DATA DONE ***\n\tcorrupted flash error count: " + corruptedFlashCounter + "\n\tcorrupted flash bytes: " + corruptedFlashBytes + "\n\tunknown prefix counter: " + unknownPrefixCounter + "\n\tplausibility check failed counter: " + plausibilityCheckFailedCounter);
        return true;
    }

    public boolean stepFourSelectedEspNowMessagesToDataStream() {
        if(!stepTwoExecuted) {
            Log.d("decoder", "error - did not choose a tag yet!");
            return false;
        }
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        int firstSendPagePointer = 0, firstSendPageOffsetPointer = 0;
        boolean first = true;
        for (EspNowMessage e : espNowMessagesSingleTag) {
            if (e.isDataMessage) {
                if (first) {
                    first = false;
                    firstSendPagePointer = e.sendPagePointer;
                    firstSendPageOffsetPointer = e.sendPageOffsetPointer;
                }
                try {
                    outputStream.write(e.dataAsByteArray());
                } catch (IOException err) {
                    err.printStackTrace();
                }
            }
        }
        stream = outputStream.toByteArray();

        // extract first part
        String firstBytes = "";
        int firstPartLength = 16000;
        if(stream.length < firstPartLength) firstPartLength = stream.length;
        for (int i = 0; i < firstPartLength; i++) {
            firstBytes += String.format("%02X", stream[i]);
        }
        estimatedDataOffset = logEntryManager.estimateDataOffsetFromFirstStreamBytes(firstBytes);

        if (estimatedDataOffset < 0) {
            Log.d("decoder", "WARNING: data offset could not be estimated (-1), using 0 instead");
            estimatedDataOffset = 0;
        }
        Log.d("decoder", "stream is " + stream.length + " Bytes long, starting at " + firstSendPagePointer + "/" + firstSendPageOffsetPointer + ", estimated data offset " + estimatedDataOffset);
        return true;
    }

    public boolean stepThreeCheckIfPointersAreConsecutive() {
        if(!stepTwoExecuted) {
            Log.d("decoder", "error - did not choose a tag yet!");
            return false;
        }
        boolean isFirst = true;
        int temp1 = 0, temp2 = 0, temp3 = 0;
        int estimatedNextAddress = 0;
        int cntRemovedDuplicates = 0;
        int firstSendPagePointer = 0, firstSendPageOffsetPointer = 0, firstLength = 0;
        int lastSendPagePointer = 0, lastSendPageOffsetPointer = 0, lastLength = 0;
        ArrayList<EspNowMessage> messagesToRemove = new ArrayList<>();
        for(EspNowMessage e : espNowMessagesSingleTag) {
            if(e.isDataMessage) {
                if (isFirst) {
                    isFirst = false;
                    temp1 = e.sendPagePointer;
                    temp2 = e.sendPageOffsetPointer;
                    temp3 = e.receivedLength;
                    dataMessageCustomPrefix = e.dataMessageCustomPrefix; // assuming that won't change for other data messages of ONE tag
                    firstSendPagePointer = e.sendPagePointer;
                    firstSendPageOffsetPointer = e.sendPageOffsetPointer;
                    firstLength = e.receivedLength;
                } else {
                    estimatedNextAddress = (temp1 * 2048) + temp2;
                    estimatedNextAddress += temp3;
                    if ((e.sendPagePointer * 2048) + e.sendPageOffsetPointer != estimatedNextAddress) {
                        Log.d("decoder-check", "subpage error at sendPagePointer " + e.sendPagePointer + " (is) vs " + temp1 + " (should), sendPageOffsetPointer " + e.sendPageOffsetPointer + " (is) vs. " + temp2 + " (should)");
                        if ((e.sendPagePointer * 2048) + e.sendPageOffsetPointer == (temp1 * 2048) + temp2) {
                            Log.d("decoder-check", "DUPLICATE -> REMOVE");
                            messagesToRemove.add(e);
                            cntRemovedDuplicates++;
                        }
                        else {
                            Log.d("decoder-check", "fatal error, not a duplicate");
                            return false;
                        }
                    }
                    temp1 = e.sendPagePointer;
                    temp2 = e.sendPageOffsetPointer;
                    temp3 = e.receivedLength;
                }
            }
        }
        lastSendPagePointer = temp1;
        lastSendPageOffsetPointer = temp2;
        lastLength = temp3;
        espNowMessagesSingleTag.removeAll(messagesToRemove);
        Log.d("decoder-check", "found pages from " + firstSendPagePointer + "." + firstSendPageOffsetPointer + "(" + firstLength + "B) to " + lastSendPagePointer + "." + lastSendPageOffsetPointer + "(" + lastLength + "B) to ");
        Log.d("decoder-check", "removed " + cntRemovedDuplicates +  " messages (duplicates)");
        Log.d("decoder-check", "all messages are consecutive!");
        return true;
    }

    public void stepTwoAndAHalfSort() {
        Collections.sort(espNowMessagesSingleTag);
        /*for(EspNowMessage e : espNowMessagesSingleTag) {
            System.out.println(e.getByteNumberInFlashMemory());
        }*/
    }

    public void stepTwoReduceDataToOneTag(String selectedTagName) {
        int cntAdded = 0;
        espNowMessagesSingleTag.removeAll(espNowMessagesSingleTag);
        for(EspNowMessage e : espNowMessagesAll) {
            if(e.mac.equals(selectedTagName)) {
                espNowMessagesSingleTag.add(e);
                cntAdded++;
            }
        }
        Log.d("decoder", "found " + cntAdded +  " messages from " + selectedTagName);
				name = selectedTagName.substring(12, 14) + selectedTagName.substring(15, 17);
        stepTwoExecuted = true;
    }

    public boolean stepOneBytesToEspNowMessages(byte[] fileData, ArrayList<String> macs, boolean debug, IMUSettings imuSettings) {
        Log.d("decoder", "file length: " + fileData.length + " bytes");

        long decodeErrors = 0;
        long decodeErrorBytes = 0;
        int decodeErrorStartPage = 0;
        int decodeErrorStartPageOffset = 0;

        int mode = 1;
        int modeCnt = 0;
        int msgLength = 0;
        long msgCnt = 0;

        EspNowMessage e = new EspNowMessage();

        for(int i = 0; i < fileData.length; i++) {
            /*if(mode == 0) { // receive time
                e.receiveTime += String.format("%02X", fileData[i]);
                modeCnt++;
                if(modeCnt >= 4) {
                    mode = 1;
                    modeCnt = 0;
                }
            }*/
            if(mode == 1) { // MAC
                if(modeCnt < 5) e.mac += String.format("%02X", fileData[i]) + ":";
                else e.mac += String.format("%02X", fileData[i]);
                modeCnt++;
                if(modeCnt >= 6) {
                    mode = 2;
                    modeCnt = 0;
                }
            }
            else if(mode == 2) { // LENGTH
                msgLength = Byte.toUnsignedInt(fileData[i]);
                modeCnt = 0;
                mode = 3;
            }
            else if(mode == 3) { // DATA
                //String val = String.format("%02X", fileData[i]);
                e.addData(fileData[i]);
                modeCnt++;
                if(modeCnt >= msgLength) {
                    modeCnt = 0;
                    mode = 1;
                    if(!e.decodeMessage(imuSettings)) {
                        if(decodeErrors == 0) { decodeErrorStartPage = e.sendPagePointer; decodeErrorStartPageOffset = e.sendPageOffsetPointer; }
                        decodeErrors++;
                        decodeErrorBytes += e.data.size();
                    }

                    espNowMessagesAll.add(e);

                    if(debug) {
                        if(msgCnt % 2000 == 0) {
                            e.printMe(msgCnt, i, fileData.length);
                        }
                    }
                    msgCnt++;
                    if(!macs.contains(e.mac)) macs.add(e.mac);
                    // ONLY FOR TESTING!!
                    //if(e.sendPagePointer > 60000) { return true; }
                    e = new EspNowMessage();
                }
            }
        }
        Log.d("decoder", "decode errors: " + decodeErrors);
        if(decodeErrors > 0) {
            Log.d("decoder", "decode errors start at " + decodeErrorStartPage + "/" + decodeErrorStartPageOffset);
            Log.d("decoder", "decode errors are " + decodeErrorBytes + " Byte long!");
        }
        return true;
    }

    public static boolean reduceBytesToCertainMac(byte[] fileDataIn, HashMap<String, ByteArrayOutputStream> macsAndBytes) {
        int mode = 1;
        int modeCnt = 0;
        int msgLength = 0;

        String currentMac = "";
        ByteArrayOutputStream currentMessage = new ByteArrayOutputStream();

        for(int i = 0; i < fileDataIn.length; i++) {
            if(mode == 1) { // MAC
                currentMessage.write(fileDataIn[i]);
                if(modeCnt < 5) currentMac += String.format("%02X", fileDataIn[i]) + ":";
                else currentMac += String.format("%02X", fileDataIn[i]);
                modeCnt++;
                if(modeCnt >= 6) {
                    mode = 2;
                    modeCnt = 0;
                }
            }
            else if(mode == 2) { // LENGTH
                currentMessage.write(fileDataIn[i]);
                msgLength = Byte.toUnsignedInt(fileDataIn[i]);
                modeCnt = 0;
                mode = 3;
            }
            else if(mode == 3) { // DATA
                currentMessage.write(fileDataIn[i]);
                modeCnt++;
                if(modeCnt >= msgLength) {
                    modeCnt = 0;
                    mode = 1;

                    if(macsAndBytes.containsKey(currentMac)) {
                        try {
                            macsAndBytes.get(currentMac).write(currentMessage.toByteArray());
                        } catch (IOException e) {
                            e.printStackTrace();
                            return false;
                        }
                    }
                    else {
                        macsAndBytes.put(currentMac, currentMessage);
                    }

                    currentMac = "";
                    currentMessage = new ByteArrayOutputStream();
                }
            }
        }
        for (Map.Entry<String, ByteArrayOutputStream> entry : macsAndBytes.entrySet()) {
            String key = entry.getKey();
            ByteArrayOutputStream value = entry.getValue();
            Log.d("decoder", key + ": " + NumberFormat.getNumberInstance(Locale.US).format(value.size()) + " B");
        }
        //Log.d("decoder", "decode errors: " + decodeErrors);
        return true;
    }
}
