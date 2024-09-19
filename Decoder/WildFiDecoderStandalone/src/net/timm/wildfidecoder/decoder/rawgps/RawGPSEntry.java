package net.timm.wildfidecoder.decoder.rawgps;

import java.math.BigDecimal;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Formatter;
import java.util.Locale;
import java.util.function.DoublePredicate;

public class RawGPSEntry {

    double pseudorange = 0.0;
    double carrierphase = 0.0;
    double dopplershift = 0.0;
    double carriertonoiseratio = 0.0;
    int gnssid = 0;
    int satid = 0;
    int signalid = 0;
    int frequencyslot = 0;
    int carrierphaselocktimecounter = 0;
    int pseudorangestddev = 0;
    int carrierphasestddev = 0;
    int dopplershiftstddev = 0;
    int trackingstatus = 0;

    public static String serializeHeadline(boolean useBurstForm) {
		if (useBurstForm)
			return "pseudorange,carrierphase,dopplershift,carriertonoiseratio,gnssid,satid,signalid,frequencyslot,carrierphaselocktimecounter,pseudorangestddev,carrierphasestddev,dopplershiftstddev,trackingstatus";
		else
        	return "pseudorange,carrierphase,dopplershift,carriertonoiseratio,gnssid,satid,signalid,frequencyslot,carrierphaselocktimecounter,pseudorangestddev,carrierphasestddev,dopplershiftstddev,trackingstatus";
    }

    public String serialize() {
        return (new Formatter(Locale.US).format("%.10f", pseudorange)) + ","
                + (new Formatter(Locale.US).format("%.10f", carrierphase)) + ","
                + (new Formatter(Locale.US).format("%.10f", dopplershift)) + ","
                + (new Formatter(Locale.US).format("%.10f", carriertonoiseratio)) + ","
                + gnssid + ","
                + satid + ","
                + signalid + ","
                + frequencyslot + ","
                + carrierphaselocktimecounter + ","
                + pseudorangestddev + ","
                + carrierphasestddev + ","
                + dopplershiftstddev + ","
                + Integer.toBinaryString(trackingstatus);
    }

    public String serializeGnssId() {
        return gnssid+"";
    }

    public String serializeSatId() {
        return satid+"";
    }

    public String serializeSignalId() {
        return signalid+"";
    }

    public String serializeFrequencySlot() {
        return frequencyslot+"";
    }

    public String serializeCarrierPhaseLockTimeCounter() {
        return carrierphaselocktimecounter+"";
    }

    public String serializePseudoRangeStdDev() {
        return pseudorangestddev+"";
    }

    public String serializeCarrierPhaseStdDev() {
        return carrierphasestddev+"";
    }

    public String serializeDopplerShiftStdDev() {
        return dopplershiftstddev+"";
    }

    public String serializeTrackingStatus() {
        return Integer.toBinaryString(trackingstatus)+"";
    }

    public String serializePseudorange() {
        return (new Formatter(Locale.US).format("%.10f", pseudorange))+"";
    }

    public String serializeCarrierphase() {
        return (new Formatter(Locale.US).format("%.10f", carrierphase))+"";
    }

    public String serializeDopplershift() {
        return (new Formatter(Locale.US).format("%.10f", dopplershift))+"";
    }

    public String serializeCarriertonoiseratio() {
        return (new Formatter(Locale.US).format("%.10f", carriertonoiseratio))+"";
    }


    public static String serializeEmpty() {
        return ",,,,,,,,,,,,";
    }

    public static void createMeasurementData(String vardata, long utcTimestamp, ArrayList<RawGPSEntry> rawGPSEntries, int observationLen) {
        int len = vardata.length();
        Long temp;
        for(int i=0; i<(len/(observationLen*2)); i++) {
            RawGPSEntry p = new RawGPSEntry();
            String dataset = vardata.substring((i*(observationLen*2)),(i*(observationLen*2))+(observationLen*2));
            String temp0 = dataset.substring(14,16) + dataset.substring(12,14) + dataset.substring(10,12) + dataset.substring(8, 10) +
                    dataset.substring(6, 8) + dataset.substring(4, 6) + dataset.substring(2, 4) + dataset.substring(0, 2);
            BigInteger bigIntTemp = new BigInteger(temp0,16);
            p.pseudorange = Double.longBitsToDouble(bigIntTemp.longValue());

            temp0 = dataset.substring(30,32) + dataset.substring(28,30) + dataset.substring(26,28) + dataset.substring(24, 26) +
                    dataset.substring(22, 24) + dataset.substring(20, 22) + dataset.substring(18, 20) + dataset.substring(16, 18);
            bigIntTemp = new BigInteger(temp0,16);
            p.carrierphase = Double.longBitsToDouble(bigIntTemp.longValue());

            temp = Long.parseLong(dataset.substring(38, 40) + dataset.substring(36, 38) + dataset.substring(34, 36) + dataset.substring(32, 34), 16);
            p.dopplershift = Float.intBitsToFloat(temp.intValue());
            temp = Long.parseLong(dataset.substring(46, 48) + dataset.substring(44, 46) + dataset.substring(42, 44) + dataset.substring(40, 42), 16);
            p.carriertonoiseratio = Float.intBitsToFloat(temp.intValue());
            p.gnssid = Integer.parseInt(dataset.substring(48, 50), 16);
            p.satid = Integer.parseInt(dataset.substring(50, 52), 16);
            p.signalid = Integer.parseInt(dataset.substring(52, 54), 16);
            p.frequencyslot = Integer.parseInt(dataset.substring(54, 56), 16); // SIGNED!
            p.carrierphaselocktimecounter = ((short) Integer.parseInt(dataset.substring(58, 60) + dataset.substring(56, 58), 16));
            p.pseudorangestddev = Integer.parseInt(dataset.substring(60, 62), 16);
            p.carrierphasestddev = Integer.parseInt(dataset.substring(62, 64), 16);
            p.dopplershiftstddev = Integer.parseInt(dataset.substring(64, 66), 16);
            p.trackingstatus = Integer.parseInt(dataset.substring(66, 68), 16);
            rawGPSEntries.add(p);
        }
    }
}