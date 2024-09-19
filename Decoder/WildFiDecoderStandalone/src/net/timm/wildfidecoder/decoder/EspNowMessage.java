package net.timm.wildfidecoder.decoder;

import net.timm.wildfidecoder.Log;
import net.timm.wildfidecoder.decoder.imu.IMUSettings;

import java.util.ArrayList;
import java.util.Locale;

public class EspNowMessage implements Comparable {
    public static String FF_PATTERN = "ffffffffffff";
    public String mac = "";
    public ArrayList<Byte> data = new ArrayList<>();
    public boolean isDataMessage = false;
    public String dataMessageCustomPrefix = "";
    public int receivedLength = 0;

    public int sendPagePointer = 0;
    public int sendPageOffsetPointer = 0;

    public long getByteNumberInFlashMemory() {
        return (2048 * ((long) sendPagePointer)) + ((long) sendPageOffsetPointer);
    }

    @Override
    public int compareTo(Object e) {
        /* For Ascending order*/
        int compResult = (int) (this.getByteNumberInFlashMemory() - ((EspNowMessage) e).getByteNumberInFlashMemory());
        return compResult;
    }

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

    public static boolean onlyFFs(String line) {
        String lineLowerCase = line.toLowerCase();
        for (int i = 0; i < lineLowerCase.length(); i++) {
            if (lineLowerCase.charAt(i) != 'f') {
                return false;
            }
        }
        return true;
    }

    boolean checkForFFFFFFFFError() {
        String asHex = byteArrayToHex(data);
        if(asHex.startsWith(FF_PATTERN) || asHex.endsWith(FF_PATTERN)) {
            Log.d("decoder", "(" + sendPagePointer + "/" + sendPageOffsetPointer + ") detected possible flash error: " + asHex);
            //printMe();
            /*System.out.println("*** Sleep 2 seconds ***");
            try {
                TimeUnit.SECONDS.sleep(2);
            } catch (InterruptedException interruptedException) {
                interruptedException.printStackTrace();
            }*/
            return true;
        }
        return false;
    }

    public boolean decodeMessage(IMUSettings imuSettings) {
        // 3 BYTES PREAMBLE: [ 0000 0000 ] [ 000 | 00000 ] [ 0 | 0000 | 111 ] (11 bits for block, 6 bits for page in block, 4 bits for page part, 3 dummy bits)
        if (data == null || data.size() < 3) return false;

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

        if(checkForFFFFFFFFError()) { return false; }
        return true;
    }

    public void addData(Byte oneByte) {
        //int oneByteInt = (int) Integer.parseInt(oneByte,16);
        data.add(oneByte);
        if (data.size() == 1) {
            if (oneByte == (byte) 0xAB) isDataMessage = true;
            else isDataMessage = false;
        }
    }

    public void printMe(long i, int dataPnt, int dataLen) {
        if (isDataMessage) {
            String firstBytes = byteArrayToHex(data);
            firstBytes = firstBytes.substring(0, Math.min(firstBytes.length(), 16));
            Log.d("decoder", "(" + dataPnt + "/" + dataLen + " byte) Msg: " + mac + " (" + receivedLength + " byte, data),(" + sendPagePointer + "/" + sendPageOffsetPointer + ")," + firstBytes + "..");
        } else {
            Log.d("decoder", "(" + dataPnt + "/" + dataLen + " byte) Msg: " + mac + " (" + receivedLength + " byte, no data)," + byteArrayToHex(data));
        }
    }
}
