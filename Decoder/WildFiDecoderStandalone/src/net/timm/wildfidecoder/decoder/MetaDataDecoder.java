package net.timm.wildfidecoder.decoder;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.entries.MetaDataEntry;

import java.io.*;
import java.lang.reflect.Array;
import java.util.*;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

public class MetaDataDecoder {

    public static final int INDEX(int in) {
        return in * 2;
    }

    public ArrayList<MetaDataEntry> metaEntries = new ArrayList<>();

    public static double getMAD(ArrayList<Double> values, double median) {
        ArrayList<Double> mads = new ArrayList<>();
        for (double accuracy : values) {
            double mad = accuracy - median;
            if (mad < 0) mad *= -1.0;
            mads.add(mad);
        }
        Collections.sort(mads);
        double mad = 0.0;
        if (mads.size() % 2 == 1)
            mad = (double) mads.get((mads.size() + 1) / 2 - 1);
        else {
            double lower = (double) mads.get(mads.size() / 2 - 1);
            double upper = (double) mads.get(mads.size() / 2);
            mad = (lower + upper) / 2.0;
        }
        return mad;
    }

    public static double getMedian(ArrayList<Double> values) {
        Collections.sort(values);
        double median = 0.0;
        if (values.size() % 2 == 1)
            median = (double) values.get((values.size() + 1) / 2 - 1);
        else {
            double lower = (double) values.get(values.size() / 2 - 1);
            double upper = (double) values.get(values.size() / 2);
            median = (lower + upper) / 2.0;
        }
        return median;
    }

    /*static double getMedianOfPairTimeDifferencesBetweenTags(ArrayList<Double> originalList) {
        // for one proximity event: if for example 4 tags communicated, then look at all relative time deviations between them -> (N*(N-1))/2, then build median of that value!
        // calculate differences between all possible pairs
        double median;
        ArrayList<Double> differencesList = new ArrayList<>();
        for (int i = 0; i < originalList.size(); i++) { // iterate through each number (4 times)
            for (int j = i + 1; j < originalList.size(); j++) {
                differencesList.add(Math.abs(originalList.get(j) - originalList.get(i)));
            }
        }
        if(differencesList.size() == 0) { return -1; }
        // calculate median of that values
        Collections.sort(differencesList);
        if (differencesList.size() % 2 == 1)
            median = differencesList.get((differencesList.size() + 1) / 2 - 1);
        else {
            double lower = differencesList.get(differencesList.size() / 2 - 1);
            double upper = differencesList.get(differencesList.size() / 2);

            median = (lower + upper) / 2.0;
        }
        return median;
    }*/

    public void serializeHeadlineAllPairTimeDifferences(String filenameTimeDiff) {
        BufferedWriter writer = null;
        try {
            String headline = "groupCounter,mostFrequentTimestampInGroup,tagA,tagB,timeDiffMs,timeDiffBMinusAMs,tagCntInGroup\n";
            writer = new BufferedWriter(new FileWriter(filenameTimeDiff, false), 8*1024);
            System.out.print(headline);
            writer.write(headline);
            writer.close();

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    static String serializeAllPairTimeDifferences(long groupCounter, ArrayList<String> tagIds, long timestamp, ArrayList<Double> originalList) {
        // for one proximity event: if for example 4 tags communicated, then look at all relative time deviations between them -> (N*(N-1))/2, then build median of that value!
        // Calculate differences between all possible pairs
        ArrayList<Double> differencesList = new ArrayList<>();
        ArrayList<Double> differencesListWithNegatives = new ArrayList<>();
        ArrayList<String> tagAs = new ArrayList<>();
        ArrayList<String> tagBs = new ArrayList<>();
        for (int i = 0; i < originalList.size(); i++) { // iterate through each number (4 times)
            for (int j = i + 1; j < originalList.size(); j++) {
                differencesList.add(Math.abs(originalList.get(j) - originalList.get(i)));
                differencesListWithNegatives.add(originalList.get(j) - originalList.get(i));
                tagAs.add(tagIds.get(i));
                tagBs.add(tagIds.get(j));
            }
        }
        if(differencesList.size() == 0) { return ""; }

        // add each time difference to list
        String result = "";
        for (int x = 0; x < differencesList.size(); x++) {
            result = result +
                    groupCounter + "," +
                    timestamp + "," +
                    tagAs.get(x) + "," +
                    tagBs.get(x) + "," +
                    (new Formatter(Locale.US).format("%.4f", differencesList.get(x))) + "," +
                    (new Formatter(Locale.US).format("%.4f", differencesListWithNegatives.get(x))) + "," +
                    originalList.size() + '\n';
        }
        return result;
    }

    public static boolean foundStrangeLargeTimeDiffs(ArrayList<Double> originalList) {
        ArrayList<Double> differencesList = new ArrayList<>();
        for (int i = 0; i < originalList.size(); i++) { // iterate through each number (4 times)
            for (int j = i + 1; j < originalList.size(); j++) {
                double diff = Math.abs(originalList.get(j) - originalList.get(i));
                if(diff > 14.0) { return true; }
            }
        }
        return false;
    }

    public static void calculateStatisticsOfProximityEvent(long groupCounter, ArrayList<String> tagIds, ArrayList<Double> gatewayTimesFirstMsg, long timestamp, String filenameAllTimeDifferences) {
        // write all values (non-summarized) in second file
        String allTimeDifferences = serializeAllPairTimeDifferences(groupCounter, tagIds, timestamp, gatewayTimesFirstMsg);
        BufferedWriter writer2 = null;
        try {
            writer2 = new BufferedWriter(new FileWriter(filenameAllTimeDifferences, true), 8*1024);
            writer2.write(allTimeDifferences);
            writer2.close();

        } catch (IOException e) {
            throw new RuntimeException(e);
        }


    }

    /*public void addHeadline(String filenameProxEval) {
        BufferedWriter writer = null;
        try {
            String headline = "TagTimestamp,TagCnt,Median,MAD,MedianPairTimeDifferences,Min,Max,Avg\n";
            writer = new BufferedWriter(new FileWriter(filenameProxEval, false), 8*1024);
            System.out.print(headline);
            writer.write(headline);
            writer.close();

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }*/

    private static long findMostFrequentValue(ArrayList<Long> list) {
        // Create a HashMap to store the frequency of each value
        HashMap<Long, Integer> frequencyMap = new HashMap<>();

        // Iterate through the list and update the frequency map
        for (Long value : list) {
            frequencyMap.put(value, frequencyMap.getOrDefault(value, 0) + 1);
        }

        // Find the value with the highest frequency
        long mostFrequentValue = 0L;
        int maxFrequency = 0;

        for (Map.Entry<Long, Integer> entry : frequencyMap.entrySet()) {
            if (entry.getValue() > maxFrequency) {
                maxFrequency = entry.getValue();
                mostFrequentValue = entry.getKey();
            }
        }

        return mostFrequentValue;
    }

    public void evaluateProximityMessagesByGWTime(String filenameTimeDiffs, double minTimeDiffForNewGroup) {
        boolean DEBUG_GATEWAYTIMES = false;

        System.out.println("*** PROX EVALUATION BY GW TIME ***");
        System.out.println("No msgs: " + metaEntries.size());

        ArrayList<ArrayList<MetaDataEntry>> groups = new ArrayList<>();

        // all metadata messages should be ordered by time already
        // find all messages that belong to one proximity event, based on ESP32 CAM receive time (max. minTimeDiffForNewGroup second windows)
        double lastReceiveTimeDouble = 0.0;
        ArrayList<MetaDataEntry> currentGroup = new ArrayList<>();
        for(MetaDataEntry m : metaEntries) {
            double differenceToLastMsg = abs(m.receiveTimeDouble-lastReceiveTimeDouble);
            if(differenceToLastMsg > minTimeDiffForNewGroup) {
                // new group
                groups.add(currentGroup);
                currentGroup = new ArrayList<>();
            }
            currentGroup.add(m);
            lastReceiveTimeDouble = m.receiveTimeDouble;
        }
        groups.add(currentGroup);
        groups.remove(0);

        // DEBUGGING TEMPORARY
        /*ArrayList<MetaDataEntry> secondGroup = groups.get(groups.size()-1);
        for(MetaDataEntry m : secondGroup) {
            System.out.print(m.serialize());
        }*/

        // now we have multiple groups that contain all proximity messages from one single proximity event
        long groupCounter = 0;
        for(ArrayList<MetaDataEntry> am : groups) {
            ArrayList<String> tagIds = new ArrayList<>();
            ArrayList<Double> gatewayStartTimes = new ArrayList<>();
            ArrayList<Long> timestampsInGroup = new ArrayList<>();
            long mostProbableTimestamp = 0;
            groupCounter++;

            if(DEBUG_GATEWAYTIMES) System.out.println("***** NEW GROUP: " + groupCounter);

            // for each proximity message in one group: add unique tagIds and add calculated time on tags (ONLY FIRST PROXIMITY MSG PER TAG, should be okay)
            // calculatedTimeOnTags is the gateway time in seconds minus the sendOffset from proximity message, so the time estimation of the tag during start of proximity detection in the time system of the gateway!
            for(MetaDataEntry m : am) {
                if (!tagIds.contains(m.ownId)) {
                    tagIds.add(m.ownId);
                    timestampsInGroup.add(m.timestamp);

                    // find median time of gateways (look at all prox messages)
                    ArrayList<Double> calculatedProximityStartTimes = new ArrayList<>();
                    if(DEBUG_GATEWAYTIMES) System.out.print("--- " + m.ownId + ":");
                    for(MetaDataEntry m1 : am) {
                        if(m1.ownId.equals(m.ownId)) {
                            double sendOffsetDouble = m1.sendOffset;
                            sendOffsetDouble /= 1000.;
                            double calculatedTimeOnTags = m1.receiveTimeDouble - sendOffsetDouble;
                            calculatedProximityStartTimes.add(calculatedTimeOnTags);
                            if(DEBUG_GATEWAYTIMES) System.out.print(String.format(Locale.US, "%.4f", calculatedTimeOnTags) + " ");
                        }
                    }
                    // BETTER: try min! messages can only come later due to delay, but not more early!
                    //double medianCalculatedStartTime = getMedian(calculatedProximityStartTimes);
                    double minCalculatedStartTime = Collections.min(calculatedProximityStartTimes);
                    if(DEBUG_GATEWAYTIMES) System.out.println(" -> " + String.format(Locale.US, "%.4f", minCalculatedStartTime));
                    //if(sendOffsetDouble > 1.0) { System.out.print("BIG OFFSET: " + m.serialize()); }
                    gatewayStartTimes.add(minCalculatedStartTime);
                }
            }

            // find the timestamp on tags that appears most often
            // does NOT detect when only one tag is around with wrong timestamp
            mostProbableTimestamp = findMostFrequentValue(timestampsInGroup);

            // check if timestamps from tags within group are the same, if not generate error
            for(MetaDataEntry m : am) {
                if(m.timestamp != mostProbableTimestamp) {
                    System.out.print("WARNING: some timestamps within group not equal ("+mostProbableTimestamp+"): " + m.serialize());
                    break; // output only once, might be more tags with different timestamps in group
                }
            }

            // creates multiple lines in csv file with summary statistics for each pair of tags
            calculateStatisticsOfProximityEvent(groupCounter, tagIds, gatewayStartTimes, mostProbableTimestamp, filenameTimeDiffs);

            if(foundStrangeLargeTimeDiffs(gatewayStartTimes)) {
                System.out.println("--- STRANGE TIME DIFF IN GROUP:");
                System.out.print(MetaDataEntry.serializeHeader());
                for(MetaDataEntry m : am) {
                    System.out.print(m.serialize());
                }
                System.out.println("--- END");
            }

            // DEBUG
            /*if(mostProbableTimestamp == 1675045800) {
                System.out.println("--- SPECIAL TIME DEBUG:");
                System.out.print(MetaDataEntry.serializeHeader());
                for(MetaDataEntry m : am) {
                    System.out.print(m.serialize());
                }
                System.out.println("--- END SPECIAL TIME DEBUG");
            }*/
        }


    }
    /*public void evaluateProximityMessagesByTagTimestamps(String filenameProxEval, String filenameTimeDiffs) {
        ArrayList<Long> timestamps = new ArrayList<>();
        System.out.println("*** PROX EVALUATION BY TAG TIMESTAMPS ***");
        System.out.println("No msgs: " + metaEntries.size());

        // find all timestamps that were sent by tags (time taken from proximity messages)
        for(MetaDataEntry m : metaEntries) {
            if(!timestamps.contains(m.timestamp)) {
                timestamps.add(m.timestamp);
            }
        }

        // sort by timestamps
        Collections.sort(timestamps);

        // for each timestamp: create a csv table entry with Median, MAD, ... etc. (calculateMedian function)
        for(long timestamp : timestamps) {
            ArrayList<String> tagIds = new ArrayList<>();
            ArrayList<Double> timeOfFirstReceivedMsgInWindow = new ArrayList<>();
            for (MetaDataEntry m : metaEntries) {
                if (m.timestamp == timestamp) { // 1674902400 1675260600, 1675008600, 1675044900
                    //System.out.print(m.serialize());
                    if (!tagIds.contains(m.ownId)) {
                        tagIds.add(m.ownId);

                        double sendOffsetDouble = m.sendOffset;
                        sendOffsetDouble /= 1000.;
                        double calculatedTimeOnTags = m.receiveTimeDouble - sendOffsetDouble;

                        timeOfFirstReceivedMsgInWindow.add(calculatedTimeOnTags);
                    }
                }
            }
            //System.out.println("No tags: " + tagIds.size());
            calculateStatisticsOfProximityEvent(timeOfFirstReceivedMsgInWindow, timestamp, filenameProxEval, filenameTimeDiffs);
        }

    }*/

    public void decode(String selectedFileToConvert, boolean debug, String filenameResults) {
        BufferedReader reader;
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter(filenameResults, true), 8*1024);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            reader = new BufferedReader(new FileReader(selectedFileToConvert));
            String line = reader.readLine();

            while (line != null) {
                line = line.replace("_", "");
                String[] lineComponents = line.split(",");
                if(debug) Log.d("meta-decoder", line);
                if(lineComponents.length == 4) {
                    String receiveTime = lineComponents[0];
                    String mac = lineComponents[1];
                    String messageLength = lineComponents[2];
                    String data = lineComponents[3];

                    double receiveTimeDouble = Double.parseDouble(receiveTime);
                    long messageLengthLong = Long.parseLong(messageLength, 16);
                    String messageType = data.substring(INDEX(0), INDEX(1));

                    if(messageType.equals("55")) { // ACTIVATED
                        Log.d("meta-decoder","ACTIVATED: " + mac);
                    }

                    if(messageType.equals("99") && data.length() >= INDEX(26)) { // TAG AROUND V2
                        long voltage = Long.parseLong(data.substring(INDEX(1), INDEX(3)), 16);
                        long lastError = Long.parseLong(data.substring(INDEX(3), INDEX(4)), 16);
                        long errorCnt = Long.parseLong(data.substring(INDEX(4), INDEX(6)), 16);
                        String commandByteMirrored = data.substring(INDEX(6), INDEX(7));
                        long state = Long.parseLong(data.substring(INDEX(7), INDEX(8)), 16);
                        long isActivated = Long.parseLong(data.substring(INDEX(8), INDEX(9)), 16);
                        long hasValidTimestamp = Long.parseLong(data.substring(INDEX(9), INDEX(10)), 16);
                        long wildFiSoftwareVersion = Long.parseLong(data.substring(INDEX(10), INDEX(11)), 16);
                        long wildFiConfigVersion = Long.parseLong(data.substring(INDEX(11), INDEX(12)), 16);
                        int magHardIronOffsetX = Integer.parseInt(data.substring(INDEX(12), INDEX(14)), 16);
                        int magHardIronOffsetY = Integer.parseInt(data.substring(INDEX(14), INDEX(16)), 16);
                        int magHardIronOffsetZ = Integer.parseInt(data.substring(INDEX(16), INDEX(18)), 16);
                        long startCnt = Long.parseLong(data.substring(INDEX(18), INDEX(22)), 16);
                        long bytesToTransmit = Long.parseLong(data.substring(INDEX(22), INDEX(26)), 16);

                        Log.d("meta-decoder",
                                (new Formatter(Locale.US).format("%03.0f", floor(receiveTimeDouble / 3600)))
                                        + ":" + (new Formatter(Locale.US).format("%02.0f", floor((receiveTimeDouble / 60) % 3600)))
                                        + ":" + (new Formatter(Locale.US).format("%02.0f", floor(receiveTimeDouble % 60)))
                                        + " (ID: " + mac + ", Length: " + messageLengthLong + ") Type: " + messageType
                                        + ", Voltage: " + voltage
                                        + ", lastError: " + lastError
                                        + ", errorCnt: " + errorCnt
                                        + ", commandByteMirrored: " + commandByteMirrored
                                        + ", state: " + state
                                        + ", isActivated: " + isActivated
                                        + ", hasValidTimestamp: " + hasValidTimestamp
                                        + ", wildFiSoftware: " + wildFiSoftwareVersion + "/" + wildFiConfigVersion
                                        + ", magOffsets: " + magHardIronOffsetX + "/" + magHardIronOffsetY + "/" + magHardIronOffsetZ
                                        + ", startCnt: " + startCnt
                                        + ", bytesToTransmit: " + bytesToTransmit);
                    }

                    if(messageType.equals("AA") && data.length() >= INDEX(250)) { // PROXIMITY
                        MetaDataEntry m = new MetaDataEntry();
                        m.receiveTimeDouble = receiveTimeDouble;
                        m.group = data.substring(INDEX(1), INDEX(5));
                        m.ownId = data.substring(INDEX(5), INDEX(7));
                        m.sendOffset = Long.parseLong(data.substring(INDEX(7), INDEX(9)), 16);
                        m.timestampLastSync = Long.parseLong(data.substring(INDEX(9), INDEX(13)), 16);
                        m.lastSyncType = Long.parseLong(data.substring(INDEX(13), INDEX(14)), 16);
                        m.voltage = Long.parseLong(data.substring(INDEX(14), INDEX(16)), 16);
                        m.lastErrorId = Long.parseLong(data.substring(INDEX(16), INDEX(17)), 16);
                        m.errorCnt = Long.parseLong(data.substring(INDEX(17), INDEX(19)), 16);
                        m.timestamp = Long.parseLong(data.substring(INDEX(19), INDEX(23)), 16);
                        m.swVersion = Long.parseLong(data.substring(INDEX(23), INDEX(24)), 16);
                        m.confVersion = Long.parseLong(data.substring(INDEX(24), INDEX(25)), 16);
                        m.syncCounter = Long.parseLong(data.substring(INDEX(25), INDEX(29)), 16);

                        writer.write(
                                m.serialize()
                        );
                        metaEntries.add(m);
                    }
                }

                line = reader.readLine();
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            writer.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void runInAllSubFolders(String directory, String filename, String filenameResults) {
        long i = 0;
        File curDir = new File(directory);
        File[] filesList = curDir.listFiles();

        for(File fparent : filesList){
            if(fparent.isDirectory()) {
                runInAllSubFolders(fparent.getAbsolutePath(), filename, filenameResults);
            }
            else {
                if(fparent.getName().equals(filename)) {
                    Log.d("meta-decoder","DECODING: " + fparent.getAbsolutePath());
                    decode(fparent.getAbsolutePath(), false, filenameResults);
                }
                /*File[] filesListSub = fparent.listFiles();
                for(File f : filesListSub) {
                    if(f.isFile()) {
                        String extension = f.getName().substring(f.getName().indexOf("."));
                        if (extension.equals(fileExtensionIn) && !f.getName().equals("mergedsubs.bin")) {
                            selectedFileToConvert = fparent.getName() + "/" + f.getName();
                            Log.d("meta-decoder","\tMerging [ " + i + " ] " + selectedFileToConvert);
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
                }*/
            }
        }

    }
}
