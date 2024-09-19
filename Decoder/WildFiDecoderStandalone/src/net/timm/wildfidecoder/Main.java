package net.timm.wildfidecoder;

import net.timm.wildfidecoder.decoder.*;
import net.timm.wildfidecoder.decoder.entries.MetaDataEntry;
import net.timm.wildfidecoder.decoder.graphs.BinaryGraph;
import net.timm.wildfidecoder.decoder.graphs.LogEntryGraph;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.sql.Timestamp;
import java.text.NumberFormat;
import java.text.SimpleDateFormat;
import java.util.*;

public class Main {
    public static String SOFTWARE_VERSION ="7.2";
    public static int modeSelect() {
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        Log.dNoLog("main","\t[ 0 ] Decode binary data received by phone or WIFI gateway (.bin)");
        Log.dNoLog("main","\t[ 1 ] Decode WildFi proxLogger binary data received by ESPNOW gateway (..PROX_DATA.bin)");
        Log.dNoLog("main","\t[ 2 ] Decode WildFi hdLogger binary data received by ESPNOW gateway (..MOVE_DATA.bin)");
        Log.dNoLog("main","\t[ 3 ] Decode ESP32 CAM gateway meta data (METADATA.txt)");
        Log.dNoLog("main","\t[ 4 ] Merge all binary files in current folder into one");
        Log.dNoLog("main","\t[ 5 ] Merge all binary files in sub folders into one");
        Log.dNoLog("main","\t[ 6 ] Merge all binary files in sub folders into tag-separated files");
        Log.dNoLog("main","\t[ 7 ] Merge all binary files in sub sub folders into one");
        Log.dNoLog("main","\t[ 8 ] Merge csv files in current directory");
        Log.dNoLog("main","\t[ 9 ] Movebank upload of csv files");
        Log.dNoLog("main","\t[ 99 ] Exit");

        System.out.print("--> Enter selection: ");
        int i = 0;
        try {
            i = Integer.parseInt(br.readLine());
        } catch(Exception e) {
            Log.dNoLog("Error", "Invalid Format! Assuming you want to decode data.");
            return 0;
        }
        if(i == 99) return i;
        if((i < 0) || (i > 10)) { Log.dNoLog("Error", "Invalid Selection! Assuming you want to decode data."); return 0; }
        return i;
    }

    public static boolean burstFormSelect() {
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        Log.d("main","\t[ 0 ] Write entire IMU bursts in single lines");
        Log.d("main","\t[ 1 ] Split IMU bursts over multiple lines");

        System.out.print("--> Enter selection: ");
        int i = 0;
        try {
            i = Integer.parseInt(br.readLine());
        } catch(Exception e) {
            Log.d("Error", "Invalid Format! Assuming you want to write bursts in single lines.");
            return true;
        }
        if((i < 0) || (i > 1)) { Log.d("Error", "Invalid Selection! Assuming you want to write bursts in single lines."); return true; }

        if (i == 0)
            return true;
        else
            return false;
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
            //Log.d("main","Date is " + (timestamp.getTime() / 1000));
            Log.d("main","Date is okay!");
            return (timestamp.getTime() / 1000);
        } catch (Exception e) {
            Log.d("Warning", "Invalid date - using all data!");
            return 0;
        }
    }

    public static String tagSelect(ArrayList<String> tagNamesIn) {
        int cnt = 0;
        Log.d("main","\t[ " + cnt + " ] decode all");
        cnt++;
        for(String s : tagNamesIn){
            Log.d("main","\t[ " + cnt + " ] " + s);
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
        if(i == 0) { return "all"; }
        if(i < 0) { Log.d("Error", "Invalid Selection!"); return ""; }
        if(i >= cnt) { Log.d("Error", "Invalid Selection!"); return ""; }
        String selected = tagNamesIn.get(i-1);
        return selected;
    }

    public static int imuFrequencySelect() {
        Log.d("main","Please enter the frequency of IMU recordings (e.g., 100 for 100 Hz): ");
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        int i = 0;
        try {
            i = Integer.parseInt(br.readLine());
        } catch(Exception e) {
            Log.d("Error", "Invalid Format!");
            return 0;
        }
        return i;
    }

    public static String fileSelect(String fileExtensionIn) {
        File curDir = new File(".");
        File[] filesList = curDir.listFiles();
        ArrayList<String> relevantFileNames = new ArrayList<String>();
        for(File f : filesList){
            if(f.isFile()){
                String extension = f.getName().substring(f.getName().indexOf("."));
                if(extension.equals(fileExtensionIn)) {
                    Log.d("main","\t[ " + relevantFileNames.size() + " ] " + f.getName() + " (" + NumberFormat.getNumberInstance(Locale.US).format(f.length()) + " B)");
                    relevantFileNames.add(f.getName());
                }
            }
        }
        if(relevantFileNames.size() == 0) {
            Log.d("Error", "No files found to convert, please copy binary files into the folder of this .exe (should end with " + fileExtensionIn + ")!");
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

    public static void mergeBinaryFiles(String fileExtensionIn) {
        String selectedFileToConvert = "";
        long i = 0;
        File curDir = new File(".");
        File[] filesList = curDir.listFiles();

        try {
            Files.deleteIfExists(Paths.get("merged.bin"));
        } catch (IOException e) {
            Log.d("Error", "Could not delete file!");
            return;
        }

        for(File f : filesList){
            if(f.isFile()){
                String extension = f.getName().substring(f.getName().indexOf("."));
                if(extension.equals(fileExtensionIn) && !f.getName().equals("merged.bin")) {
                    Log.d("main","\tMerging [ " + i + " ] " + f.getName());
                    i++;
                    selectedFileToConvert = f.getName();
                    File file = new File(selectedFileToConvert);
                    byte[] fileData = new byte[(int) file.length()];
                    try {
                        DataInputStream dis = new DataInputStream(new FileInputStream(file));
                        dis.readFully(fileData);
                        dis.close();
                    } catch (IOException e) {
                        Log.d("Error", "Could not read file!");
                        return;
                    }
                    try {
                        Files.write(Paths.get("merged.bin"), fileData, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                    } catch (IOException e) {
                        Log.d("Error", "Could not write file!");
                        return;
                    }
                }
            }
        }
        if(i == 0) {
            Log.d("Error", "No files found to convert, please copy binary files into the folder of this .exe (should end with " + fileExtensionIn + ")!");
            return;
        }
    }

    public static void mergeBinaryFilesInSubFolders(String fileExtensionIn) {
        String selectedFileToConvert = "";
        long i = 0;
        File curDir = new File(".");
        File[] filesList = curDir.listFiles();

        try {
            Files.deleteIfExists(Paths.get("mergedsubs.bin"));
        } catch (IOException e) {
            Log.d("Error", "Could not delete file!");
            return;
        }

        for(File fparent : filesList){
            if(fparent.isDirectory()){
                File[] filesListSub = fparent.listFiles();
                for(File f : filesListSub) {
                    if(f.isFile()) {
                        String extension = f.getName().substring(f.getName().indexOf("."));
                        if (extension.equals(fileExtensionIn) && !f.getName().equals("mergedsubs.bin")) {
                            selectedFileToConvert = fparent.getName() + "/" + f.getName();
                            Log.d("main","\tMerging [ " + i + " ] " + selectedFileToConvert);
                            i++;
                            File file = new File(selectedFileToConvert);
                            byte[] fileData = new byte[(int) file.length()];
                            try {
                                DataInputStream dis = new DataInputStream(new FileInputStream(file));
                                dis.readFully(fileData);
                                dis.close();
                            } catch (IOException e) {
                                Log.d("Error", "Could not read file!");
                                return;
                            }
                            try {
                                Files.write(Paths.get("mergedsubs.bin"), fileData, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                            } catch (IOException e) {
                                Log.d("Error", "Could not write file!");
                                return;
                            }
                        }
                    }
                }
            }
        }
        if(i == 0) {
            Log.d("Error", "No files found to convert, please copy binary files into the sub folder of this .exe (should end with " + fileExtensionIn + ")!");
            return;
        }
    }

    public static void mergeBinaryFilesInSubSubFolders(String fileExtensionIn) {
        String selectedFileToConvert = "";
        long i = 0;
        File curDir = new File(".");
        File[] filesList = curDir.listFiles();

        try {
            Files.deleteIfExists(Paths.get("mergedsubs.bin"));
        } catch (IOException e) {
            Log.d("Error", "Could not delete file!");
            return;
        }

        for(File fparent : filesList){
            if(fparent.isDirectory()){
                File[] filesListSub = fparent.listFiles();
                for(File f : filesListSub) {
                    if(f.isDirectory()) {
                        File[] filesListSubSub = f.listFiles();
                        for (File f2 : filesListSubSub) {
                            if (f2.isFile()) {
                                String extension = f2.getName().substring(f2.getName().indexOf("."));
                                if (extension.equals(fileExtensionIn) && !f2.getName().equals("mergedsubs.bin")) {
                                    selectedFileToConvert = fparent.getName() + "/" + f.getName() + "/" + f2.getName();
                                    Log.d("main","\tMerging [ " + i + " ] " + selectedFileToConvert);
                                    i++;
                                    File file = new File(selectedFileToConvert);
                                    byte[] fileData = new byte[(int) file.length()];
                                    try {
                                        DataInputStream dis = new DataInputStream(new FileInputStream(file));
                                        dis.readFully(fileData);
                                        dis.close();
                                    } catch (IOException e) {
                                        Log.d("Error", "Could not read file!");
                                        return;
                                    }
                                    try {
                                        Files.write(Paths.get("mergedsubs.bin"), fileData, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                                    } catch (IOException e) {
                                        Log.d("Error", "Could not write file!");
                                        return;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if(i == 0) {
            Log.d("Error", "No files found to convert, please copy binary files into the sub folder of this .exe (should end with " + fileExtensionIn + ")!");
            return;
        }
    }

    public static void mergeBinaryFilesInSubFoldersHdLoggerSplittedMacs(String fileExtensionIn) {
        String selectedFileToConvert = "";
        long i = 0;
        File curDir = new File(".");
        File[] filesList = curDir.listFiles();

        // delete all existing _DATA_MERGED.bin
        for(File f : filesList) {
            if(f.isFile()) {
                String fileName = f.getName();
                if(fileName.endsWith("_DATA_MERGED.bin")) {
                    Log.d("main","Deleting " + fileName);
                    try {
                        Files.deleteIfExists(Paths.get(fileName));
                    } catch (IOException e) {
                        Log.d("Error", "Could not delete file!");
                        return;
                    }
                }
            }
        }

        for(File fparent : filesList){
            if(fparent.isDirectory()){
                File[] filesListSub = fparent.listFiles();
                for(File f : filesListSub) {
                    if(f.isFile()) {
                        String extension = f.getName().substring(f.getName().indexOf("."));
                        if (extension.equals(fileExtensionIn) /*&& !f.getName().equals("mergedsubs.bin")*/) {
                            selectedFileToConvert = fparent.getName() + "/" + f.getName();
                            Log.d("main","\tMerging [ " + i + " ] " + selectedFileToConvert + " (" + NumberFormat.getNumberInstance(Locale.US).format(f.length()) + " B)");
                            i++;
                            File file = new File(selectedFileToConvert);
                            byte[] fileData = new byte[(int) file.length()];

                            try {
                                DataInputStream dis = new DataInputStream(new FileInputStream(file));
                                dis.readFully(fileData);
                                dis.close();
                            } catch (IOException e) {
                                Log.d("Error", "Could not read file!");
                                return;
                            }

                            HashMap<String, ByteArrayOutputStream> macsAndBytes = new HashMap<String, ByteArrayOutputStream>();
                            EspNowDecoder.reduceBytesToCertainMac(fileData, macsAndBytes);

                            for (Map.Entry<String, ByteArrayOutputStream> entry : macsAndBytes.entrySet()) {
                                String mac = entry.getKey();
                                ByteArrayOutputStream macBytes = entry.getValue();
                                try {
                                    String macFormatted = mac.replace(':', '_');
                                    Files.write(Paths.get(macFormatted + "_DATA_MERGED.bin"), macBytes.toByteArray(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                                } catch (IOException e) {
                                    Log.d("Error", "Could not write file!");
                                    return;
                                }
                            }
                        }
                    }
                }
            }
        }
        if(i == 0) {
            Log.d("Error", "No files found to convert, please copy binary files into the sub folder of this .exe (should end with " + fileExtensionIn + ")!");
            return;
        }
    }

    public static void mergeCSVFiles() throws Exception {
        File curDir = new File(".");
        File[] filesList = curDir.listFiles();

        Log.d("main","*** ONLY MERGING 123458 and 123464 MSGS ***");
        Files.deleteIfExists(Paths.get("123458_ALL.csv"));
        Files.deleteIfExists(Paths.get("123464_ALL.csv"));

        BufferedWriter writer123458 = new BufferedWriter(new FileWriter("123458_ALL.csv", false), 8*1024);
        BufferedWriter writer123464 = new BufferedWriter(new FileWriter("123464_ALL.csv", false), 8*1024);

        long counter123458 = 0;
        long counter123464 = 0;

        for(File f : filesList) {
            if (!f.isDirectory()) {
                String filename = f.getName();
                if(filename.endsWith("_123458.csv")) {
                    Log.d("main","MERGE: " + filename);
                    BufferedReader reader = new BufferedReader(new FileReader(filename));
                    String line = reader.readLine();
                    long lineCnt = 0;
                    while (line != null) {
                        if(lineCnt == 0) {
                            if(counter123458 == 0) writer123458.write(line+ "\n"); // write header line just once
                        }
                        else writer123458.write(line+ "\n");
                        line = reader.readLine();
                        lineCnt++;
                    }
                    reader.close();
                    counter123458++;
                }
                if(filename.endsWith("_123464.csv")) {
                    Log.d("main","MERGE: " + filename);
                    BufferedReader reader = new BufferedReader(new FileReader(filename));
                    String line = reader.readLine();
                    long lineCnt = 0;
                    while (line != null) {
                        if(lineCnt == 0) {
                            if(counter123464 == 0) writer123464.write(line+ "\n"); // write header line just once
                        }
                        else writer123464.write(line+ "\n");
                        line = reader.readLine();
                        lineCnt++;
                    }
                    reader.close();
                    counter123464++;
                }
            }
        }

        writer123458.close();
        writer123464.close();
    }

    public static void generateMovebankUploadBatchFile() throws Exception {
        File curDir = new File(".");
        File[] filesList = curDir.listFiles();

        Log.d("main","*** GENERATING BATCH FILE ***");
        Files.deleteIfExists(Paths.get("uploadCSVFilesToMovebank.bat"));

        BufferedWriter writer = new BufferedWriter(new FileWriter("uploadCSVFilesToMovebank.bat", false), 8*1024);

        writer.write("@ECHO OFF\n" +
                "ECHO Hello! This script uploads all compatible decoded csv files to Movebank. Please wait until script reports FINISHED.\n" +
                "TIMEOUT 2\n");

        for(File f : filesList) {
            if (!f.isDirectory()) {
                String filename = f.getName();
                if(filename.endsWith(".csv") && filename.contains("burstFormat")) {
                    Log.d("main","UPLOAD SCRIPT FOR: " + filename);
                    BufferedReader reader = new BufferedReader(new FileReader(filename));
                    String line = reader.readLine();
                    long lineCnt = 0;
                    int foundIDindex = -1;
                    String extractedId = "";
                    while (line != null) {
                        if(lineCnt == 0) {
                            String[] split = line.split(",");
                            int i=0;
                            for (String s: split) {
                                if(s.equals("id") || s.equals("tagId")) {
                                    foundIDindex = i;
                                }
                                i++;
                            }
                        }
                        if(lineCnt == 1) {
                            if(foundIDindex >= 0) {
                                String[] split = line.split(",");
                                extractedId = split[foundIDindex];
                                Log.d("main","ID: " + extractedId);
                            }
                            break;
                        }
                        line = reader.readLine();
                        lineCnt++;
                    }
                    reader.close();

                    if(!extractedId.equals("")) {
                        writer.write("ECHO ***\n");
                        writer.write("ECHO *** Uploading file " + filename + " (tagId: "+extractedId+")\n");
                        writer.write("ECHO ***\n");
                        writer.write("curl --user \"wildfi_hd_admin:Uloonoo3Ej\" -F \"operation=add-data\" -F \"feed=wildfi-hd/multi\" -F \"tag=" + extractedId + "\" -F \"data=@" + filename + "\" \"https://www.movebank.org/movebank/service/external-feed\" --ssl-no-revoke --progress-bar --retry-all-errors --verbose\n");
                    }
                }
            }
        }
        writer.write("ECHO FINISHED.\n");
        writer.write("PAUSE\n");

        writer.close();

        Log.d("main","*** PLEASE RUN uploadCSVFilesToMovebank.bat FOR UPLOAD! ***");
    }

    public static String createFileName(String inputFileName, String selectedTag, boolean onlyHeader, boolean errorHappened, long startTime, long endTime, boolean useBurstForm) {
        String fileNameCsv = "";
        selectedTag = selectedTag.replace(':', '_');
        if((startTime == 0) && (endTime == 0))
            fileNameCsv = inputFileName + "_" + selectedTag;
        else fileNameCsv = LogEntryManager.utcTimestampToStringForFileName(startTime) + "_to_" + LogEntryManager.utcTimestampToStringForFileName(endTime) + "_" + inputFileName + "_" + selectedTag;
        if (useBurstForm)
            fileNameCsv += "_burstFormat";
        if (onlyHeader) {
            if (errorHappened)
                fileNameCsv += "_onlyHeader_INCOMPLETE";
            else
                fileNameCsv += "_onlyHeader";
        }
        else {
            if (errorHappened)
                fileNameCsv += "_INCOMPLETE";
            else
                fileNameCsv += "";
        }

        return fileNameCsv;
    }

    static void readConfigFile(IMUSettings imuSettings) {
        String configFileName = "WildFiDecoderConfig.txt";
        BufferedReader br = null;
        try {
            br = new BufferedReader(new FileReader(configFileName));
        } catch (FileNotFoundException e) {
            Log.dNoLog("main"," - WARNING: config file '" + configFileName + "' not found, using default settings for IMU decoding");
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
                    Log.dNoLog("main"," - ERROR READING CONFIG FILE, LINE " + lineNumber);
                }
            }
            if (lineNumber == 3) {
                try {
                    double readValue = Double.parseDouble(line);
                    imuSettings.gyroConversionFactor = readValue;
                } catch (Exception e) {
                    Log.dNoLog("main"," - ERROR READING CONFIG FILE, LINE " + lineNumber);
                }
            }
            if (lineNumber == 5) {
                try {
                    double readValue = Double.parseDouble(line);
                    imuSettings.magConversionFactor = readValue;
                } catch (Exception e) {
                    Log.dNoLog("main"," - ERROR READING CONFIG FILE, LINE " + lineNumber);
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
        Log.dNoLog("main","-------------------------------------------------");
        Log.dNoLog("main","--- BINARY DATA CONVERTER FOR WILDFI TAG V"+SOFTWARE_VERSION+" ---");
        Log.dNoLog("main","-------------------------------------------------");
        Log.dNoLog("main","Application started!");
        Log.dNoLog("main","Heap size: " + Runtime.getRuntime().totalMemory() + " Byte");
        readConfigFile(imuSettings);
        Log.dNoLog("main","Settings: " + imuSettings.settingsToString());

        while(true) {
            Log.init();
            EspNowDecoder decoderEspNow = new EspNowDecoder();
            boolean useBurstForm = false;
            ArrayList<String> tagNames = new ArrayList<>();
            int mode = modeSelect();

            if (mode == 1 || mode == 2)
                useBurstForm = burstFormSelect();
            if (mode == 3) {
                String fileNameMetaDataProx = "METADATA_PROX.csv";
                Files.deleteIfExists(Paths.get(fileNameMetaDataProx));
                BufferedWriter writer = null;
                try {
                    writer = new BufferedWriter(new FileWriter(fileNameMetaDataProx, true), 8*1024);
                    writer.write(MetaDataEntry.serializeHeader());
                    writer.close();
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
                MetaDataDecoder metaDataDecoder = new MetaDataDecoder();
                //metaDataDecoder.addHeadline("proxTimeData.csv");
                //metaDataDecoder.addHeadline("proxTimeDataGWTime.csv");
                //metaDataDecoder.addTimeDiffHeadline("proxTimeDiffs.csv");
                metaDataDecoder.serializeHeadlineAllPairTimeDifferences("proxTimeDiffsGWTime.csv");
                metaDataDecoder.runInAllSubFolders(".", "METADATA.txt", fileNameMetaDataProx);
                //metaDataDecoder.evaluateProximityMessagesByTagTimestamps("proxTimeData.csv", "proxTimeDiffs.csv");
                metaDataDecoder.evaluateProximityMessagesByGWTime("proxTimeDiffsGWTime.csv", 2.0);
            } else if (mode == 4) {
                mergeBinaryFiles(".bin");
            } else if (mode == 5) {
                mergeBinaryFilesInSubFolders(".bin");
            } else if (mode == 6) {
                mergeBinaryFilesInSubFoldersHdLoggerSplittedMacs(".bin");
            } else if (mode == 7) {
                mergeBinaryFilesInSubSubFolders(".bin");
            } else if (mode == 8) {
                try {
                    mergeCSVFiles();
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            } else if (mode == 9) {
                try {
                    generateMovebankUploadBatchFile();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            } else if (mode == 99) {
                break;
            } else {
                if (mode == 1) imuSettings.customPrefixLength = 2;
                else if (mode == 2) imuSettings.customPrefixLength = 0;

                String selectedFileToConvert = fileSelect(".bin");
                if (!selectedFileToConvert.equals("")) {
                    Log.d("main","Selected: " + selectedFileToConvert);
                    File file = new File(selectedFileToConvert);
                    byte[] fileData = new byte[(int) file.length()];
                    boolean error = false;
                    Log.d("main","Opening file..");
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
                        Log.d("main","Read in total " + fileData.length + " Bytes");
                        //decoder.debugFileBytes(fileData);
                        if (mode == 1 || mode == 2) {
                            if (!decoderEspNow.stepOneBytesToEspNowMessages(fileData, tagNames, true, imuSettings)) {
                                Log.d("Error", "Could not load file!");
                            } else {
                                Log.d("main","File loaded, please select tag name to convert from..");
                                String selectedTag = tagSelect(tagNames);
                                int imuFrequency = imuFrequencySelect();
                                if (selectedTag.equals("")) {
                                    Log.d("Error", "Could not load tag data!");
                                } else {
                                    //boolean onlyHeader = onlyHeaderSelect();
                                    int tagCnt = 0;
                                    if (!selectedTag.equals("all")) {
                                        tagNames.removeAll(tagNames);
                                        tagNames.add(selectedTag);
                                    }
                                    int fatalErrors = 0;
                                    for (String tagIterator : tagNames) {
                                        Log.d("main","*** Start decoding tag " + tagIterator + " ***");
                                        String inputFileName = selectedFileToConvert.substring(0, selectedFileToConvert.lastIndexOf('.'));
                                        decoderEspNow.stepTwoReduceDataToOneTag(tagIterator);

                                        if(mode == 1) {
                                            // sort messages here! only when proximity data, otherwise pointers might wrap around
                                            decoderEspNow.stepTwoAndAHalfSort();
                                        }
                                        if (!decoderEspNow.stepThreeCheckIfPointersAreConsecutive()) {
                                            Log.d("Error", "Esp now data messages are not consecutive (maybe due to some re-transmission)");
                                        }
                                        if (!decoderEspNow.stepFourSelectedEspNowMessagesToDataStream()) {
                                            Log.d("Error", "Could not load esp now messages into stream!");
                                        } else {
                                            long selectedStartTime = 0;
                                            long selectedEndTime = 0;
                                            String fileNameCsvComplete = createFileName(inputFileName, tagIterator, false, false, selectedStartTime, selectedEndTime, useBurstForm);
                                            //String fileNameCsvOnlyHeader = createFileName(inputFileName, tagIterator, true, false, selectedStartTime, selectedEndTime, useBurstForm);
                                            //BufferedWriter writerComplete = new BufferedWriter(new FileWriter(fileNameCsvComplete));
                                            //BufferedWriter writerOnlyHeader = new BufferedWriter(new FileWriter(fileNameCsvOnlyHeader));
                                            Log.d("main","Start converting into file (please wait)..");
                                            boolean success = decoderEspNow.stepFiveWriteResultToFile(fileNameCsvComplete, selectedStartTime, selectedEndTime, imuSettings, 10, useBurstForm, imuFrequency);
                                            //writerComplete.close();
                                            //writerOnlyHeader.close();
                                            if (success) {
                                                Log.d("main","SUCCESS: Finished conversion!");
                                            } else {
                                                Log.d("main","FATAL ERROR HAPPENED! Continue with next tag.");
                                                fatalErrors++;
                                                // rename file
                                            /*String fileNameCsvCompleteNew = createFileName(inputFileName, tagIterator, false, true, selectedStartTime, selectedEndTime, useBurstForm);
                                            String fileNameCsvOnlyHeaderNew = createFileName(inputFileName, tagIterator, true, true, selectedStartTime, selectedEndTime, useBurstForm);
                                            File fileToRename = new File(fileNameCsvComplete);
                                            if(!fileToRename.renameTo(new File(fileNameCsvCompleteNew))) {
                                                Log.d("main","FATAL: Could not rename file!");
                                            }
                                            File fileToRename2 = new File(fileNameCsvOnlyHeader);
                                            if(!fileToRename2.renameTo(new File(fileNameCsvOnlyHeaderNew))) {
                                                Log.d("main","FATAL: Could not rename file!");
                                            }*/
                                            }
                                        }
                                        if (tagCnt++ != tagNames.size() - 1) {
                                            Log.d("main","*** NEXT TAG ***");
                                            /*try {
                                                TimeUnit.SECONDS.sleep(1);
                                            } catch (InterruptedException interruptedException) {
                                                interruptedException.printStackTrace();
                                            }*/
                                        }
                                    }
                                    Log.d("main","TOTAL EXTREMELY FATAL ERRORS (decoder stopped): " + fatalErrors);
                                }
                            }
                        }
                        if (mode == 0) {
                            Log.d("Error", "Not yet implemented!");
                            /*if (!decoderForWifi.loadFileBytesIntoFlashBlocks(fileData, tagNames, false)) {
                                Log.d("Error", "Could not load file!");
                            } else {
                                Log.d("main","File loaded, please select tag name to convert from..");
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
                                            Log.d("main","Data starts:\t" + LogEntry.utcTimestampToString(firstEntry.utcTimestamp) + " (UTC)");
                                        if (lastEntry != null)
                                            Log.d("main","Data ends:\t\t" + LogEntry.utcTimestampToString(lastEntry.utcTimestamp) + " (UTC)");
                                        if ((firstEntry == null) || (lastEntry == null)) {
                                            Log.d("Error", "Could not determine first and last entry!");
                                        } else {
                                            long selectedStartTime = timeSelect("start", firstEntry.utcTimestamp, lastEntry.utcTimestamp);
                                            long selectedEndTime = timeSelect("end", firstEntry.utcTimestamp, lastEntry.utcTimestamp);

                                            if (selectedStartTime == 0) selectedStartTime = firstEntry.utcTimestamp;
                                            if (selectedEndTime == 0) selectedEndTime = lastEntry.utcTimestamp;
                                            String fileNameCsv = createFileName(inputFileName, selectedTag, onlyHeader, selectedStartTime, selectedEndTime);
                                            BufferedWriter writer = new BufferedWriter(new FileWriter(fileNameCsv));

                                            Log.d("main","Start converting into file " + fileNameCsv + " (please wait)..");

                                            //decoder.getDataStream().testFunctionForTimestampEstimation(200);
                                            decoderForWifi.getDataStream().loadIntoFile(writer, selectedStartTime, selectedEndTime, 0, onlyHeader, accFrequencySelected);
                                            Log.d("main","Finished conversion");
                                            writer.close();
                                        }
                                    }
                                }
                            }*/
                        }
                    }
                }
            }
            Log.deinit();
        }
        Log.deinit();
        Log.dNoLog("main","-------------------------------------------------");
        Log.dNoLog("main","------------------- FINISHED --------------------");
        Log.dNoLog("main","-------------------------------------------------");
        Scanner scanner = new Scanner(System.in);
        scanner.nextLine();
    }

}
