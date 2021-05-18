package net.timm.wildfidecoder;

import net.timm.wildfidecoder.decoder.EspNowDecoder;
import net.timm.wildfidecoder.decoder.LogEntryManager;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.io.*;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;
import java.util.*;

public class Main {
    //static BlackforestTagMovementDecoderCore decoderForWifi = new BlackforestTagMovementDecoderCore();
    static EspNowDecoder decoderEspNow = new EspNowDecoder();

    public static int modeSelect() {
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        System.out.println("\t[ 0 ] Decode binary data received by phone or WIFI gateway (.bin)");
        System.out.println("\t[ 1 ] Decode WildFi proxLogger binary data received by ESPNOW gateway (..PROX_DATA.bin)");
        System.out.println("\t[ 2 ] Decode WildFi hdLogger binary data received by ESPNOW gateway (..MOVE_DATA.bin)");

        System.out.print("--> Enter selection: ");
        int i = 0;
        try {
            i = Integer.parseInt(br.readLine());
        } catch(Exception e) {
            Log.d("Error", "Invalid Format! Assuming you want to decode data.");
            return 0;
        }
        if((i < 0) || (i > 2)) { Log.d("Error", "Invalid Selection! Assuming you want to decode data."); return 0; }
        return i;
    }

    public static boolean onlyHeaderSelect() {
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        System.out.print("--> Do you want to decode full acceleration data (not only header data) (y or n)?: ");
        String yesOrNo = "";
        try {
            yesOrNo = br.readLine();
        } catch(Exception e) {
            Log.d("Error", "Invalid Format! Assuming YES.");
            return false;
        }
        if(yesOrNo.equals("y")) return false;
        else if(yesOrNo.equals("n")) return true;
        else {
            Log.d("Error", "Invalid Format! Assuming YES.");
            return false;
        }
    }

    public static long timeSelect(String header, long startDate, long endDate) {
        SimpleDateFormat parser = new SimpleDateFormat("dd.MM.yy HH:mm:ss");
        parser.setTimeZone(TimeZone.getTimeZone("UTC"));   // This line converts the given date into UTC time zone
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        System.out.print("--> Enter " + header + " time in format 'dd.mm.yy hh:mm:ss' (between " + LogEntryManager.utcTimestampToString(startDate) + " and " + LogEntryManager.utcTimestampToString(endDate) + ", 0 = all): ");
        try {
            Date selectedStartDate = parser.parse(br.readLine());
            Timestamp timestamp = new Timestamp(selectedStartDate.getTime());
            //System.out.println("Date is " + (timestamp.getTime() / 1000));
            System.out.println("Date is okay!");
            return (timestamp.getTime() / 1000);
        } catch (Exception e) {
            Log.d("Warning", "Invalid date - using all data!");
            return 0;
        }
    }

    public static String tagSelect(ArrayList<String> tagNamesIn) {
        int cnt = 0;
        for(String s : tagNamesIn){
            System.out.println("\t[ " + cnt + " ] " + s);
            cnt++;
        }
        if(cnt == 0) return "";
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        System.out.print("--> Enter selection (0 .. " + (cnt-1) + "): ");
        int i = 0;
        try {
            i = Integer.parseInt(br.readLine());
        } catch(Exception e) {
            Log.d("Error", "Invalid Format!");
            return "";
        }
        if(i < 0) { Log.d("Error", "Invalid Selection!"); return ""; }
        if(i >= cnt) { Log.d("Error", "Invalid Selection!"); return ""; }
        String selected = tagNamesIn.get(i);
        return selected;
    }

    public static String binaryFileSelect() {
        File curDir = new File(".");
        File[] filesList = curDir.listFiles();
        ArrayList<String> relevantFileNames = new ArrayList<String>();
        for(File f : filesList){
            if(f.isFile()){
                String extension = f.getName().substring(f.getName().indexOf("."));
                if(extension.equals(".bin")) {
                    System.out.println("\t[ " + relevantFileNames.size() + " ] " + f.getName());
                    relevantFileNames.add(f.getName());
                }
            }
        }
        if(relevantFileNames.size() == 0) {
            Log.d("Error", "No files found to convert, please copy binary files into the folder of this .exe (should end with .bin)!");
            return "";
        }

        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        System.out.print("--> Enter selection (0 .. " + (relevantFileNames.size() - 1) + "): ");
        int i = 0;
        try {
            i = Integer.parseInt(br.readLine());
        } catch(Exception e) {
            Log.d("Error", "Invalid Format!");
            return "";
        }
        if(i < 0) { Log.d("Error", "Invalid Selection!"); return ""; }
        if(i >= relevantFileNames.size()) { Log.d("Error", "Invalid Selection!"); return ""; }
        String selectedFileToConvert = relevantFileNames.get(i);
        return selectedFileToConvert;
    }

    public static String createFileName(String inputFileName, String selectedTag, boolean onlyHeader, long startTime, long endTime) {
        String fileNameCsv = "";
        selectedTag = selectedTag.replace(':', '_');
        if((startTime == 0) && (endTime == 0)) {
            fileNameCsv = "ALL_" + inputFileName + "_" + selectedTag;
        }
        else fileNameCsv = LogEntryManager.utcTimestampToStringForFileName(startTime) + "_to_" + LogEntryManager.utcTimestampToStringForFileName(endTime) + "_" + inputFileName + "_" + selectedTag;
        if (onlyHeader)
            fileNameCsv += "_onlyHeader.csv";
        else
            fileNameCsv += ".csv";

        return fileNameCsv;
    }

    static void readConfigFile(IMUSettings imuSettings) {
        String configFileName = "config.txt";
        BufferedReader br = null;
        try {
            br = new BufferedReader(new FileReader(configFileName));
        } catch (FileNotFoundException e) {
            System.out.println(" - WARNING: config file '" + configFileName + "' not found, using default settings for IMU decoding");
            return;
        }
        String line = null;
        int lineNumber = 0;
        try {
            line = br.readLine();
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        while (line != null) {
            if (lineNumber == 1) {
                try {
                    double readValue = Double.parseDouble(line);
                    imuSettings.accConversionFactor = readValue;
                } catch (Exception e) {
                    System.out.println(" - ERROR READING CONFIG FILE, LINE " + lineNumber);
                }
            }
            if (lineNumber == 3) {
                try {
                    double readValue = Double.parseDouble(line);
                    imuSettings.gyroConversionFactor = readValue;
                } catch (Exception e) {
                    System.out.println(" - ERROR READING CONFIG FILE, LINE " + lineNumber);
                }
            }
            try {
                line = br.readLine();
            } catch (IOException e) {
                e.printStackTrace();
                return;
            }
            lineNumber++;
        }
    }

    public static void main(String[] args) throws IOException {
        IMUSettings imuSettings = new IMUSettings();
        ArrayList<String> tagNames = new ArrayList<>();
        System.out.println("-------------------------------------------------");
        System.out.println("--- BINARY DATA CONVERTER FOR WILDFI TAG V2.0 ---");
        System.out.println("-------------------------------------------------");
        System.out.println("Application started!");
        readConfigFile(imuSettings);
        System.out.println("Settings: " + imuSettings.settingsToString());

        int mode = modeSelect();
        if(mode == 1) imuSettings.customPrefixLength = 2;
        else if(mode == 2) imuSettings.customPrefixLength = 0;

        String selectedFileToConvert = binaryFileSelect();
        if (!selectedFileToConvert.equals("")) {
            System.out.println("Selected: " + selectedFileToConvert);
            File file = new File(selectedFileToConvert);
            byte[] fileData = new byte[(int) file.length()];
            boolean error = false;
            System.out.println("Opening file..");
            try {
                DataInputStream dis = new DataInputStream(new FileInputStream(file));
                dis.readFully(fileData);
                dis.close();
            } catch (IOException e) {
                error = true;
            }
            if (error) {
                Log.d("Error", "Could not read file!");
            } else {
                System.out.println("Read in total " + fileData.length + " Bytes");
                //decoder.debugFileBytes(fileData);
                if(mode == 1 || mode == 2) {
                    System.out.println("WORK IN PROGRESS");
                    if (!decoderEspNow.stepOneBytesToEspNowMessages(fileData, tagNames, true, imuSettings)) {
                        Log.d("Error", "Could not load file!");
                    } else {
                        System.out.println("File loaded, please select tag name to convert from..");
                        String selectedTag = tagSelect(tagNames);
                        if (selectedTag.equals("")) {
                            Log.d("Error", "Could not load tag data!");
                        } else {
                            boolean onlyHeader = onlyHeaderSelect();
                            String inputFileName = selectedFileToConvert.substring(0, selectedFileToConvert.lastIndexOf('.'));
                            decoderEspNow.stepTwoReduceDataToOneTag(selectedTag);
                            if (!decoderEspNow.stepThreeCheckIfPointersAreConsecutive()) {
                                Log.d("Error", "Esp now data messages are not consecutive (maybe due to some re-transmission)");
                            }
                            if (!decoderEspNow.stepFourSelectedEspNowMessagesToDataStream()) {
                                Log.d("Error", "Could not load esp now messages into stream!");
                            } else {
                                long selectedStartTime = 0;
                                long selectedEndTime = 0;
                                String fileNameCsv = createFileName(inputFileName, selectedTag, onlyHeader, selectedStartTime, selectedEndTime);
                                BufferedWriter writer = new BufferedWriter(new FileWriter(fileNameCsv));
                                System.out.println("Start converting into file " + fileNameCsv + " (please wait)..");
                                decoderEspNow.stepFiveWriteResultToFile(writer, selectedStartTime, selectedEndTime, onlyHeader, imuSettings);
                                System.out.println("Finished conversion");
                                writer.close();
                            }
                        }
                    }
                }
                if(mode == 0) {
                    Log.d("Error", "Not yet implemented!");
                    /*if (!decoderForWifi.loadFileBytesIntoFlashBlocks(fileData, tagNames, false)) {
                        Log.d("Error", "Could not load file!");
                    } else {
                        System.out.println("File loaded, please select tag name to convert from..");
                        String selectedTag = tagSelect(tagNames);
                        if (selectedTag.equals("")) {
                            Log.d("Error", "Could not load tag data!");
                        } else {
                            boolean onlyHeader = onlyHeaderSelect();
                            String inputFileName = selectedFileToConvert.substring(0, selectedFileToConvert.lastIndexOf('.'));
                            decoderForWifi.removeSelectedFlashBlocks(selectedTag);
                            if (!decoderForWifi.checkIfFlashBlocksAreConsecutive(false)) {
                                Log.d("Error", "Flash blocks are not consecutive (maybe due to some retransmission)");
                                if (!decoderForWifi.tryToFixFlashBlocksNonConsecutiveError()) {
                                    Log.d("Error", "Tried to fix it, but wasn't possible!");
                                }
                                // TODO: REMOVE THAT LINE!!!!
                                //decoder.removeFlashBlocksAtEnd(188); // TEMPORARY, ONLY FOR PANGOLIN DATA!!!
                            }

                            if (!decoderForWifi.loadSelectedFlashBlocksIntoDataStream()) {
                                Log.d("Error", "Could not load blocks!");
                            } else {
                                double accFrequencySelected = frequencySelect();
                                LogEntry firstEntry = decoderForWifi.getDataStream().extractFirstLogEntry(accFrequencySelected, false);
                                LogEntry lastEntry = decoderForWifi.getDataStream().extractLastLogEntry(accFrequencySelected, false);

                                if (firstEntry != null)
                                    System.out.println("Data starts:\t" + LogEntry.utcTimestampToString(firstEntry.utcTimestamp) + " (UTC)");
                                if (lastEntry != null)
                                    System.out.println("Data ends:\t\t" + LogEntry.utcTimestampToString(lastEntry.utcTimestamp) + " (UTC)");
                                if ((firstEntry == null) || (lastEntry == null)) {
                                    Log.d("Error", "Could not determine first and last entry!");
                                } else {
                                    long selectedStartTime = timeSelect("start", firstEntry.utcTimestamp, lastEntry.utcTimestamp);
                                    long selectedEndTime = timeSelect("end", firstEntry.utcTimestamp, lastEntry.utcTimestamp);

                                    if (selectedStartTime == 0) selectedStartTime = firstEntry.utcTimestamp;
                                    if (selectedEndTime == 0) selectedEndTime = lastEntry.utcTimestamp;
                                    String fileNameCsv = createFileName(inputFileName, selectedTag, onlyHeader, selectedStartTime, selectedEndTime);
                                    BufferedWriter writer = new BufferedWriter(new FileWriter(fileNameCsv));

                                    System.out.println("Start converting into file " + fileNameCsv + " (please wait)..");

                                    //decoder.getDataStream().testFunctionForTimestampEstimation(200);
                                    decoderForWifi.getDataStream().loadIntoFile(writer, selectedStartTime, selectedEndTime, 0, onlyHeader, accFrequencySelected);
                                    System.out.println("Finished conversion");
                                    writer.close();
                                }
                            }
                        }
                    }*/
                }
            }
        }
        System.out.println("-------------------------------------------------");
        System.out.println("------------------- FINISHED --------------------");
        System.out.println("-------------------------------------------------");
        Scanner scanner = new Scanner(System.in);
        scanner.nextLine();
    }

}
