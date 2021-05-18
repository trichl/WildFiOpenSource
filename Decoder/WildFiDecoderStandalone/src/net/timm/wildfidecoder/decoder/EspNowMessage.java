package net.timm.wildfidecoder.decoder;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.util.ArrayList;

public class EspNowMessage {
    //public String receiveTime = "";
    public String mac = "";
    public ArrayList<Byte> data = new ArrayList<>();
    public boolean isDataMessage = false;
    public String dataMessageCustomPrefix = "";
    public int receivedLength = 0;

    public int sendPagePointer = 0;
    public int sendPageOffsetPointer = 0;

    public static String byteArrayToHex(ArrayList<Byte> a) {
        byte[] result = new byte[a.size()];
        for (int i = 0; i < a.size(); i++) {
            result[i] = a.get(i).byteValue();
        }
        StringBuilder sb = new StringBuilder(result.length * 2);
        for (byte b : a)
            sb.append(String.format("%02x", b));
        return sb.toString();
    }

    byte[] dataAsByteArray() {
        byte[] result = new byte[data.size()];
        for (int i = 0; i < data.size(); i++) {
            result[i] = data.get(i).byteValue();
        }
        return result;
    }

    void decodeMessage(IMUSettings imuSettings) {
        // 3 BYTES PREAMBLE: [ 0000 0000 ] [ 000 | 00000 ] [ 0 | 0000 | 111 ] (11 bits for block, 6 bits for page in block, 4 bits for page part, 3 dummy bits)
        if (data == null || data.size() < 3) return;

        if (isDataMessage) {
            int flashPnt = (Byte.toUnsignedInt(data.get(1)) << 24) | (Byte.toUnsignedInt(data.get(2)) << 16) | (Byte.toUnsignedInt(data.get(3)) << 8) | (Byte.toUnsignedInt(data.get(4)));
            sendPagePointer = (flashPnt >> 15) & (0b11111111111111111); // 17 bit sendPagePointer
            sendPageOffsetPointer = (flashPnt >> 4) & (0b11111111111); // 11 bit sendPageOffsetPointer

            data.remove(0);
            data.remove(0);
            data.remove(0);
            data.remove(0);
            data.remove(0);

            if(imuSettings.customPrefixLength > 0) {
                for(int i=0; i< imuSettings.customPrefixLength; i++) {
                    dataMessageCustomPrefix += String.format("%02X", data.get(0));
                    data.remove(0);
                }
            }
            receivedLength = data.size();
        } else {
            receivedLength = data.size();
        }
    }

    public void addData(Byte oneByte) {
        //int oneByteInt = (int) Integer.parseInt(oneByte,16);
        data.add(oneByte);
        if (data.size() == 1) {
            if (oneByte == (byte) 0xAB) isDataMessage = true;
            else isDataMessage = false;
        }
    }

    public void printMe() {
        if (isDataMessage) {
            String firstBytes = byteArrayToHex(data);
            firstBytes = firstBytes.substring(0, Math.min(firstBytes.length(), 16));
            Log.d("decoder", "Msg: " + mac + " (" + receivedLength + " byte, data),(" + sendPagePointer + "/" + sendPageOffsetPointer + ")," + firstBytes + "..");
        } else {
            Log.d("decoder", "Msg: " + mac + " (" + receivedLength + " byte, no data)," + byteArrayToHex(data));
        }
    }
}
