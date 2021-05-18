package com.timm.esp32datadownloader;

import android.content.Context;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

public class LogDataStorage {

    private static final String LOGFILENAME = "log.txt";
    private File fileToLogTo;
    private String fullFileName = "";

    public void deleteLogFile() {
        if(fileToLogTo.exists()) {
            fileToLogTo.delete();
        }
    }

    public String readLastLogs(int numberOfLogs) {
        FileInputStream is;
        BufferedReader reader;
        ArrayList<String> lines = new ArrayList<>();
        if(fileToLogTo.exists()) {
            try {
                is = new FileInputStream(fileToLogTo);
            } catch (FileNotFoundException e) {
                return "Log file empty exception";
            }
            reader = new BufferedReader(new InputStreamReader(is));
            String line = null;
            try {
                line = reader.readLine();
                while(line != null){
                    lines.add(line);
                    line = reader.readLine();
                }
            } catch (IOException e) {
                return "Log file exception";
            }
            String returnText = "";
            int cnt = 0;
            for(int i = lines.size()-1; i >= 0; i--) {
                returnText = returnText + lines.get(i) + "\n";
                cnt++;
                if(cnt >= numberOfLogs) break;
            }
            return returnText;
        }
        else return "Log file empty";
    }

    public void writeToLog(String text) {
        SimpleDateFormat sdf = new SimpleDateFormat("dd.MM.yyyy HH:mm:ss.SSS", Locale.getDefault());
        String currentTime = sdf.format(new Date());
        text = currentTime + ": " + text + "\n";
        byte[] data = text.getBytes();
        if(!fileToLogTo.exists()) {
            try {
                fileToLogTo.createNewFile();
            } catch (IOException e) {
                return;
            }
        }
        try {
            FileOutputStream stream = new FileOutputStream(fullFileName, true);
            stream.write(data);
            stream.close();
        } catch (IOException e1) {
            return;
        }
    }

    public LogDataStorage(Context context) {
        fullFileName = StorageGeneral.getStorageDirectory(context) + "/" + LOGFILENAME;
        fileToLogTo = new File(fullFileName);
    }
}
