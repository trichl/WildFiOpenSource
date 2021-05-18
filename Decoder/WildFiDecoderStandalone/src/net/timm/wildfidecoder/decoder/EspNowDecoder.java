package net.timm.wildfidecoder.decoder;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.entries.LogEntry;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.ArrayList;

public class EspNowDecoder {
    LogEntryManager logEntryManager = new LogEntryManager();
    private ArrayList<EspNowMessage> espNowMessages = new ArrayList<>();
    boolean stepTwoExecuted = false;
    String dataMessageCustomPrefix = "";
    byte[] stream;
    int estimatedDataOffset = 0;

    public boolean stepFiveWriteResultToFile(BufferedWriter writer, long selectedStartTimestamp, long selectedEndTimestamp, boolean onlyHeader, IMUSettings imuSettings) throws IOException {
        String line = "";
        int fifoLenFromCurrentLogEntry = 0;
        boolean foundFifoLen = false;
        LogEntry e;
        int iterator = estimatedDataOffset;
        String previousPrefix = "";
        while (iterator < stream.length) {
            line += String.format("%02X", stream[iterator]);
            iterator++;

            if ((line.length() == LogEntry.INDEX(1500)) || (iterator == stream.length)) { // 1500 bytes
                e = logEntryManager.createEntry(line, dataMessageCustomPrefix);
                if(e == null) {
                    Log.d("decoder", "unknown prefix, decode ERROR at line " + line);
                    return false;
                }
                e.decode(line, onlyHeader, true, imuSettings);
                if(e.plausibilityCheckOkay == false) {
                    Log.d("decoder", "decode plausibility ERROR at line " + line);
                    return false;
                }

                boolean doNotAddEntry = false;
                if (selectedStartTimestamp > 0) {
                    if (e.utcTimestamp < selectedStartTimestamp) doNotAddEntry = true;
                }
                if (selectedEndTimestamp > 0) {
                    if (e.utcTimestamp > selectedEndTimestamp) {
                        return false;
                    }
                }
                if (doNotAddEntry) {
                    Log.d("decoder", "(skipped)");
                } else {
                    if(!e.prefix.equals(previousPrefix)) {
                        if (onlyHeader) writer.write("---\n" + e.headlineHeader() + "\n");
                        else writer.write("--- NEW DATA TYPE ---\n" + e.headlineHeaderAndVarData() + "\n");
                    }
                    if (onlyHeader) writer.write(e.serializeHeader() + "\n");
                    else writer.write(e.serializeHeaderAndVarData() + "\n");
                    Log.d("decoder", "added " + e.serializeHeader() + " with " + e.getVarDataLength() + " * vardata");
                    previousPrefix = e.prefix;
                }
                iterator = (iterator - (line.length() / 2)) + e.entryLengthInBytes;
                line = "";
            }
        }
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
        for (EspNowMessage e : espNowMessages) {
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
        for(EspNowMessage e : espNowMessages) {
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
        espNowMessages.removeAll(messagesToRemove);
        Log.d("decoder-check", "found pages from " + firstSendPagePointer + "." + firstSendPageOffsetPointer + "(" + firstLength + "B) to " + lastSendPagePointer + "." + lastSendPageOffsetPointer + "(" + lastLength + "B) to ");
        Log.d("decoder-check", "removed " + cntRemovedDuplicates +  " messages (duplicates)");
        Log.d("decoder-check", "all messages are consecutive!");
        return true;
    }

    public void stepTwoReduceDataToOneTag(String selectedTagName) {
        int cntRemoved = 0;
        ArrayList<EspNowMessage> messagesToRemove = new ArrayList<>();
        for(EspNowMessage e : espNowMessages) {
            if(!e.mac.equals(selectedTagName)) {
                messagesToRemove.add(e);
                cntRemoved++;
            }
        }
        espNowMessages.removeAll(messagesToRemove);
        Log.d("decoder", "removed " + cntRemoved +  " messages (everything except from " + selectedTagName + " data)");
        stepTwoExecuted = true;
    }

    public boolean stepOneBytesToEspNowMessages(byte[] fileData, ArrayList<String> macs, boolean debug, IMUSettings imuSettings) {
        Log.d("decoder", "file length: " + fileData.length + " bytes");

        int mode = 1;
        int modeCnt = 0;
        int msgLength = 0;

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
                    e.decodeMessage(imuSettings);
                    espNowMessages.add(e);
                    if(debug) e.printMe();
                    if(!macs.contains(e.mac)) macs.add(e.mac);
                    // ONLY FOR TESTING!!
                    //if(e.sendPagePointer > 60000) { return true; }
                    e = new EspNowMessage();
                }
            }
        }
        return true;
    }
}
