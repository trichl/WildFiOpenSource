package net.timm.wildfidecoder;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class Log {
    public static String logFileName = "DECODER_LOG.txt";
    public static BufferedWriter writer = null;
    public static void init() {
        try {
            writer = new BufferedWriter(new FileWriter(logFileName, true));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public static void d(String a, String b) {
        if(writer != null) {
            try {
                writer.write(a + ": " + b + "\n");
                writer.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        System.out.println(a + ": " + b);
    }
    public static void dNoLog(String a, String b) {
        System.out.println(a + ": " + b);
    }
    public static void deinit() {
        if(writer != null) {
            try {
                writer.close();
                writer = null;
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
