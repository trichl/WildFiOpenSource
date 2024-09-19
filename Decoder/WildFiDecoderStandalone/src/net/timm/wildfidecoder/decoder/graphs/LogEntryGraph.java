package net.timm.wildfidecoder.decoder.graphs;

public class LogEntryGraph {
    public static int TYPE_SKIPPED = 0;
    public static int TYPE_FFFF = 1;
    public static int TYPE_PREFIX_NOT_FOUND = 2;
    public static int TYPE_DECODED_OKAY = 3;
    public static int TYPE_DECODED_NOT_PLAUSIBLE = 4;
    public static int TYPE_DECODED_NOT_PLAUSIBLE_STOPPED = 5;
    public static int TYPE_PREFIX_NOT_FOUND_UNREPAIRABLE = 6;

    public int start = 0;
    public int length = 0;
    public int type = TYPE_FFFF;
    public String text = "";
}
