package net.timm.wildfidecoder.decoder.imu;

public class IMUSettings {

    public double accConversionFactor = 1;
    public double gyroConversionFactor = 1;
    public double magConversionFactor = 1;
    public int customPrefixLength = 0;

    public String settingsToString() {
        return "accConversionFactor: " + accConversionFactor + ", gyroConversionFactor: " + gyroConversionFactor + ", magConversionFactor: " + magConversionFactor + ", customPrefixLength: " + customPrefixLength;
    }


}
