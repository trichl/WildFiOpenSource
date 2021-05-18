package com.timm.esp32datadownloader;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class DecoderActivity extends AppCompatActivity {

    private String selectedFile = "";
    private int selectedFileIndex = 0;
    private TextView tSelectedFile, tStatus;

    ArrayList<String> tagNames = new ArrayList<>();

    private BlackforestTagMovementDecoderCore decoder;

    Thread threadLoadFile;

    String statusText = "";

    public void addStatusLine(String text) {
        if(statusText.equals("")) statusText = text;
        else statusText = statusText + "\n" + text;
        tStatus.setText(statusText);
    }

    public void startLoadFileThread() {
        threadLoadFile = new Thread() {
            @Override
            public void run() {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        if(tSelectedFile.equals("") || selectedFileIndex < 0) {
                            addStatusLine("No file selected");
                        }
                        else {
                            String fullFileName = StorageGeneral.getStorageDirectory(DecoderActivity.this) + "/" + selectedFile;
                            File file = new File(fullFileName);
                            byte[] fileData = new byte[(int) file.length()];
                            boolean error = false;
                            addStatusLine("Opening file..");
                            try {
                                DataInputStream dis = new DataInputStream(new FileInputStream(file));
                                dis.readFully(fileData);
                                dis.close();
                            } catch (IOException e) {
                                error = true;
                            }
                            if (error) {
                                addStatusLine("Error: could not read file");
                            } else {
                                addStatusLine("Read " + fileData.length + " Bytes");
                                if(!decoder.loadFileBytesIntoFlashBlocks(fileData, tagNames, false)) {
                                    addStatusLine("Error: could not load files into flash blocks");
                                }
                                else {
                                    System.out.println("File loaded!");
                                }
                            }
                        }
                    }
                });
            }
        };
        threadLoadFile.start();
    }

    public void showFileSelectorAlert(final Context context) {
        String path = StorageGeneral.getStorageDirectory(context) + "/";
        Log.d("mpi", "Path: " + path);
        File directory = new File(path);

        tSelectedFile.setText("No file selected");

        FilenameFilter textFilter = new FilenameFilter() {
            public boolean accept(File dir, String name) {
                String lowercaseName = name.toLowerCase();
                if(lowercaseName.endsWith(".bin")) {
                    return true;
                } else {
                    return false;
                }
            }
        };

        final File[] files = directory.listFiles(textFilter);
        Arrays.sort(files, Collections.reverseOrder());
        Log.d("mpi", "Size: "+ files.length);
        if(files.length <= 0) {
            Toast.makeText(context, "No data received yet!", Toast.LENGTH_LONG).show();
            selectedFileIndex = -1;
        }
        else {
            AlertDialog.Builder adb = new AlertDialog.Builder(context);
            CharSequence items[] = new CharSequence[files.length];
            selectedFileIndex = 0;

            for (int i = 0; i < files.length; i++) {
                items[i] = files[i].getName();
            }
            adb.setSingleChoiceItems(items, 0, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface d, int n) {
                    selectedFileIndex = n;
                }
            });
            adb.setNegativeButton("Cancel", null);
            adb.setPositiveButton("Okay", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    selectedFile = files[selectedFileIndex].getName();
                    tSelectedFile.setText(selectedFile);
                    startLoadFileThread();
                }
            });
            adb.setTitle("Select file to decode into CSV");
            adb.show();
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.getSupportActionBar().hide();
        setContentView(R.layout.activity_decoder);

        tSelectedFile = findViewById(R.id.tselectedfile);
        tStatus = findViewById(R.id.tstatus);
        decoder = new BlackforestTagMovementDecoderCore();

        showFileSelectorAlert(this);
    }
}
