package com.timm.esp32datadownloader;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;

class NanoHTTPDWebserver extends NanoHTTPD {
    RawDataStorage rawDataStorage;
    LogDataStorage logDataStorage;

    public NanoHTTPDWebserver(int port) {
        super(port);
    }

    public void setDataStorages(RawDataStorage rawDataStorageIn, LogDataStorage logDataStorageIn) {
        rawDataStorage = rawDataStorageIn;
        logDataStorage = logDataStorageIn;
    }

    public static String byteToString(byte b) {
        int val = b & 0xFF;
        return String.format("%02X", val);
    }

    @Override
    public Response serve(IHTTPSession session) {
        if(session.getUri().equalsIgnoreCase("/test")) {
            logDataStorage.writeToLog("Somebody called test function");
            return newFixedLengthResponse(Response.Status.CREATED, MIME_PLAINTEXT, "GATEWAY IS RUNNING!!!");
        }
        else if(session.getUri().equalsIgnoreCase("/store") && session.getMethod() == Method.POST) {
            int streamLength = Integer.parseInt(session.getHeaders().get("content-length"));
            long startTime = System.currentTimeMillis();
            if(!StorageGeneral.enoughFreeMemoryToStoreShit()) {
                logDataStorage.writeToLog("Receiving data but ERROR: NOT ENOUGH MEMORY TO STORE DATA!");
            }
            else {
                byte[] fileContent = new byte[streamLength];
                try {
                    InputStream input = session.getInputStream();
                    int bytesRead = 0;
                    int iterations = 0;
                    while(bytesRead < streamLength) {
                        int numberThisRead = input.read(fileContent, bytesRead, streamLength - bytesRead);
                        bytesRead += numberThisRead;
                        iterations++;
                    }
                    if(rawDataStorage.writeToFile(fileContent)) {
                        long difference = System.currentTimeMillis() - startTime;
                        logDataStorage.writeToLog("Got " + streamLength + " bytes, stored " + bytesRead + " bytes (" + iterations + " it.), "+difference+"ms");
                        return newFixedLengthResponse(Response.Status.CREATED, MIME_PLAINTEXT, "DATA STORED!");
                    } else {
                        logDataStorage.writeToLog("Got bytes but FAILED TO STORE LOCALLY " + bytesRead + " bytes (" + iterations + " it.)");
                    }
                } catch (Exception e) {
                    logDataStorage.writeToLog("Got something but EXCEPTION happened: " + e.toString());
                }
            }
        }
        else if(session.getUri().equalsIgnoreCase("/storeOld") && session.getMethod() == Method.POST) {
            Map<String, String> files = new HashMap<String, String>();
            try {
                session.parseBody(files);
            } catch (IOException e) {
                logDataStorage.writeToLog("Receiving data: ERROR 01");
                return newFixedLengthResponse(Response.Status.BAD_REQUEST, MIME_PLAINTEXT,"ERROR!");
            } catch (ResponseException e) {
                logDataStorage.writeToLog("Receiving data: ERROR 02");
                return newFixedLengthResponse(Response.Status.BAD_REQUEST, MIME_PLAINTEXT, "ERROR!");
            }
            String postBody = files.get("postData");
            int endIndex = 40;
            if(postBody.length() < 40) endIndex = postBody.length();
            ArrayList<Short> firstBytes = new ArrayList<>();
            logDataStorage.writeToLog("Receiving data: " + postBody.substring(0, endIndex) + ".. (" + postBody.length() + " Bytes)");
            return newFixedLengthResponse(Response.Status.CREATED, MIME_PLAINTEXT, "DATA STORED!");
        }
        else if(session.getUri().equalsIgnoreCase("/configuration") && session.getMethod() == Method.GET) {
            Map<String, String> parms = session.getParms();
            if(parms.get("tagname") != null) {
                String tagname = parms.get("tagname");
                logDataStorage.writeToLog("- TAG "+ tagname + " IS DOWNLOADING CONFIGURATION SETTINGS -");
                return newFixedLengthResponse(Response.Status.CREATED, MIME_PLAINTEXT, "SETTINGS ABC");
            }
        }
        return newFixedLengthResponse(Response.Status.BAD_REQUEST, MIME_PLAINTEXT,"DO NOT KNOW YOUR SHIT!");
    }
}