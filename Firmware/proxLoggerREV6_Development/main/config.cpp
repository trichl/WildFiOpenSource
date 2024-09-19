#include "config.h"

void printConfigurationHash(tag_config_t *c) {
    printf("confHash: ");
    printf("%01X", c->useLeds);
    printf("%01X", c->trackerMode);
    printf("%01X", c->activationMode);
    printf("%01X", c->getFirstTimeOverWiFi);
    printf("%01X", c->getFirstTimeOverGPS);
    printf("%01X", c->tagIdSource);
    printf("%01X", c->imuMode);
    printf("%08X", c->imuBurstMillis);
    printf("%01X", c->environmentActivated);
    printf("%01X", c->timeCorrectionBetweenTags);
    printf("%04X", c->timeCorrectionDiffMs);
    printf("%01X", c->freeMemoryIfFull);
    printf("%02X", c->accFrequency);
    printf("%02X", c->accAvg);
    printf("%02X", c->accRange);
    printf("%02X", c->magFrequency);
    printf("%02X", c->magAccuracy);
    printf("%02X", c->gyroFrequency);
    printf("%02X", c->gyroRange);
    printf("%02X", c->gyroMode);
    printf("%01X", c->nightTimeEnter);
    printf("%02X", c->nightTimeTurnOnHour);
    printf("%02X", c->nightTimeTurnOnMinute);
    printf("%02X", c->nightTimeTurnOffHour);
    printf("%02X", c->nightTimeTurnOffMinute);
    printf("%08X", c->gpsFixHourBits);
    printf("%01X", c->gpsRandomizeFixes);
    printf("%02X", c->gpsRandomizeFixesPerDay);
    printf("%02X", c->gpsMinHdopTimesTen);
    printf("%02X", c->gpsFirstFixCollectOrbitDataSeconds);
    printf("%01X", c->gpsForcedAfterEveryProximity);
    printf("%02X", c->gpsSyncRTCFrequency);
    printf("%02X", c->proximityFrequencyMinute);
    printf("%02X", c->proximityFrequencyMinuteSeenSomeone);
    printf("%04X", c->proximityListeningIntervalMs);
    printf("%02X", c->proximityDbm);
    printf("%02X", c->proximityDatarate);
    printf("%01X", c->proximityLongRange);
    printf("%04X", c->proximityAirTimeUs);
    printf("%04X", c->activationByGatewayListeningTime);
    printf("%04X", c->activationByGatewaySleepSeconds);
    printf("%04X", c->battMinVoltage);
    printf("%04X", c->battRestartVoltage);
    printf("%04X", c->battMinVoltageDuringTransmission);
    printf("%02X", c->timeWifiOutputPower);
    printf("%04X", c->timeBetweenGetTimeRetriesSeconds);
    printf("\n");
}

void printConfiguration(tag_config_t *c) {
    printf("useLeds: %d\n", c->useLeds);
    printf("trackerMode: %d\n", c->trackerMode);
    printf("activationMode: %d\n", c->activationMode);
    printf("getFirstTimeOverWiFi: %d\n", c->getFirstTimeOverWiFi);
    printf("getFirstTimeOverGPS: %d\n", c->getFirstTimeOverGPS);
    printf("tagIdSource: %d\n", c->tagIdSource);
    printf("imuMode: %d\n", c->imuMode);
    printf("imuBurstMillis: %d\n", c->imuBurstMillis);
    printf("environmentActivated: %d\n", c->environmentActivated);
    printf("timeCorrectionBetweenTags: %d\n", c->timeCorrectionBetweenTags);
    printf("timeCorrectionDiffMs: %d\n", c->timeCorrectionDiffMs);
    printf("freeMemoryIfFull: %d\n", c->freeMemoryIfFull);
    printf("accFrequency: %02X\n", c->accFrequency);
    printf("accAvg: %02X\n", c->accAvg);
    printf("accRange: %02X\n", c->accRange);
    printf("magFrequency: %02X\n", c->magFrequency);
    printf("magAccuracy: %02X\n", c->magAccuracy);
    printf("gyroFrequency: %02X\n", c->gyroFrequency);
    printf("gyroRange: %02X\n", c->gyroRange);
    printf("gyroMode: %02X\n", c->gyroMode);
    printf("nightTimeEnter: %d\n", c->nightTimeEnter);
    printf("nightTimeTurnOnHour: %d\n", c->nightTimeTurnOnHour);
    printf("nightTimeTurnOnMinute: %d\n", c->nightTimeTurnOnMinute);
    printf("nightTimeTurnOffHour: %d\n", c->nightTimeTurnOffHour);
    printf("nightTimeTurnOffMinute: %d\n", c->nightTimeTurnOffMinute);
    printf("gpsFixHourBits: %08X\n", c->gpsFixHourBits);
    printf("gpsRandomizeFixes: %d\n", c->gpsRandomizeFixes);
    printf("gpsRandomizeFixesPerDay: %d\n", c->gpsRandomizeFixesPerDay);
    printf("gpsMinHdopTimesTen: %d\n", c->gpsMinHdopTimesTen);
    printf("gpsFirstFixCollectOrbitDataSeconds: %d\n", c->gpsFirstFixCollectOrbitDataSeconds);
    printf("gpsForcedAfterEveryProximity: %d\n", c->gpsForcedAfterEveryProximity);
    printf("gpsSyncRTCFrequency: %d\n", c->gpsSyncRTCFrequency);
    printf("proximityFrequencyMinute: %d\n", c->proximityFrequencyMinute);
    printf("proximityFrequencyMinuteSeenSomeone: %d\n", c->proximityFrequencyMinuteSeenSomeone);
    printf("proximityListeningIntervalMs: %d\n", c->proximityListeningIntervalMs);
    printf("proximityDbm: %02X\n", c->proximityDbm);
    printf("proximityDatarate: %02X\n", c->proximityDatarate);
    printf("proximityLongRange: %d\n", c->proximityLongRange);
    printf("proximityAirTimeUs: %d\n", c->proximityAirTimeUs);
    printf("activationByGatewayListeningTime: %d\n", c->activationByGatewayListeningTime);
    printf("activationByGatewaySleepSeconds: %d\n", c->activationByGatewaySleepSeconds);
    printf("battMinVoltage: %d\n", c->battMinVoltage);
    printf("battRestartVoltage: %d\n", c->battRestartVoltage);
    printf("battMinVoltageDuringTransmission: %d\n", c->battMinVoltageDuringTransmission);
    printf("timeWifiOutputPower: %02X\n", c->timeWifiOutputPower);
    printf("timeBetweenGetTimeRetriesSeconds: %d\n", c->timeBetweenGetTimeRetriesSeconds);
}

bool configIsPlausible(tag_config_t *config, uint8_t *errorIdIn) {
    bool allGood = true;
    uint8_t errorId = 0;
    // WARNING: GPS time server mode cannot be entered via remote configuration
    if((config->trackerMode != MODE_TESTRUN) && (config->trackerMode != MODE_PRODUCTIVE) && (config->trackerMode != MODE_SELFTEST)/* && (config->trackerMode != MODE_GPS_TIME_SERVER)*/) { allGood = false; errorId = 1; }
    if((config->activationMode != ACTIVATION_MODE_SKIP) && (config->activationMode != ACTIVATION_MODE_STORE_PERMANENTLY) && (config->activationMode != ACTIVATION_MODE_ON_EVERY_START)) { allGood = false; errorId = 2; }
    if((config->tagIdSource != TAG_ID_USE_MAC_LAST_TWO_BYTES) && (config->tagIdSource != TAG_ID_USE_VALUE_IN_NVS)) { allGood = false; errorId = 3; }
    if((config->imuMode != IMU_DEACTIVATED) && (config->imuMode != IMU_ACC_ONLY) && (config->imuMode != IMU_ACC_MAG_GYRO)) { allGood = false; errorId = 4; }
    if(config->timeCorrectionDiffMs > 1000) { allGood = false; errorId = 5; }
    if((config->accFrequency < BMX160_ACCEL_ODR_0_78HZ) || (config->accFrequency > BMX160_ACCEL_ODR_1600HZ)) { allGood = false; errorId = 6; }
    if(config->accAvg > BMX160_ACCEL_BW_RES_AVG16) { allGood = false; errorId = 7; }
    if((config->accRange != BMX160_ACCEL_RANGE_2G) && (config->accRange != BMX160_ACCEL_RANGE_4G) && (config->accRange != BMX160_ACCEL_RANGE_8G) && (config->accRange != BMX160_ACCEL_RANGE_16G)) { allGood = false; errorId = 8; }
    if((config->magFrequency < BMX160_MAG_ODR_0_78HZ) || (config->magFrequency > BMX160_MAG_ODR_50HZ)) { allGood = false; errorId = 9; }
    if(config->magAccuracy > BMX160_MAG_ACCURACY_HIGH) { allGood = false; errorId = 10; }
    if((config->gyroFrequency < BMX160_GYRO_ODR_25HZ) || (config->gyroFrequency > BMX160_GYRO_ODR_3200HZ)) { allGood = false; errorId = 11; }
    if(config->gyroRange > BMX160_GYRO_RANGE_125_DPS) { allGood = false; errorId = 12; }
    if(config->gyroMode > BMX160_GYRO_BW_NORMAL_MODE) { allGood = false; errorId = 13; }
    if((config->nightTimeEnter != NIGHTTIME_ALWAYS_NIGHT) && (config->nightTimeEnter != NIGHTTIME_USE_HOURS) && (config->nightTimeEnter != NIGHTTIME_DISABLED)) { allGood = false; errorId = 14; }
    if(config->nightTimeTurnOnHour > 23) { allGood = false; errorId = 15; }
    if(config->nightTimeTurnOnMinute > 59) { allGood = false; errorId = 16; }
    if(config->nightTimeTurnOffHour > 23) { allGood = false; errorId = 17; }
    if(config->nightTimeTurnOffMinute > 59) { allGood = false; errorId = 18; }
    if(config->gpsFixHourBits > (GPS_FIX_HOUR_0 | GPS_FIX_HOUR_1 | GPS_FIX_HOUR_2 | GPS_FIX_HOUR_3 | GPS_FIX_HOUR_4 | GPS_FIX_HOUR_5 | GPS_FIX_HOUR_6 | GPS_FIX_HOUR_7 | GPS_FIX_HOUR_8 | GPS_FIX_HOUR_9 | GPS_FIX_HOUR_10 | GPS_FIX_HOUR_11 | GPS_FIX_HOUR_12 | GPS_FIX_HOUR_13 | GPS_FIX_HOUR_14 | GPS_FIX_HOUR_15 | GPS_FIX_HOUR_16 | GPS_FIX_HOUR_17 | GPS_FIX_HOUR_18 | GPS_FIX_HOUR_19 | GPS_FIX_HOUR_20 | GPS_FIX_HOUR_21 | GPS_FIX_HOUR_22 | GPS_FIX_HOUR_23)) {
        allGood = false; errorId = 19;
    }
    if(config->gpsRandomizeFixesPerDay > 24) { allGood = false; errorId = 20; }
    if(config->proximityListeningIntervalMs > 30000)  { allGood = false; errorId = 21; } // NEW: duration can be zero as well -> no proximity!
    if(config->proximityDbm > RADIO_MAX_TX_19_5_DBM) { allGood = false; errorId = 22; }
    if(config->proximityDatarate > WIFI_PHY_RATE_MAX) { allGood = false; errorId = 23; }
    if(config->activationByGatewayListeningTime == 0) { allGood = false; errorId = 24; }
    if(config->activationByGatewaySleepSeconds == 0) { allGood = false; errorId = 25; }
    if(config->battMinVoltage > 6000) { allGood = false; errorId = 26; }
    if(config->battRestartVoltage > 6000) { allGood = false; errorId = 27; }
    if(config->battMinVoltageDuringTransmission > 6000) { allGood = false; errorId = 28; }
    if(config->timeWifiOutputPower > RADIO_MAX_TX_19_5_DBM) { allGood = false; errorId = 29; }
    if(config->timeBetweenGetTimeRetriesSeconds == 0) { allGood = false; errorId = 30; }
    if(errorIdIn != NULL) { *errorIdIn = errorId; }
    return allGood;
}

bool checkIfReconfigNeeded(WildFiTagREV6 *device, tag_config_t *currentConfig, uint8_t *nearestGatewayConfiguration, uint8_t nearestGatewayCommand, uint8_t nearestGatewayRssi, bool *changedSomething, bool debug) {
    bool noError = true;
    uint16_t changeCnt = 0;
    if(debug) {
        printf("RECONF: (CMD: %02X, RSSI: -%d), CONFIG: ", nearestGatewayCommand, nearestGatewayRssi);
        for(uint8_t i=0; i<GATEWAY_AROUND_LEN; i++) { printf("%02X", nearestGatewayConfiguration[i]); }
        printf("\n");
    }
    if(nearestGatewayCommand == PROXIMITY_COMMAND_CHANGE_CONFIG) {
        // parsing data into config struct
        tag_config_t receivedConfig = { }; // init with only zeros
        uint16_t i = 0;
        receivedConfig.useLeds = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.trackerMode = nearestGatewayConfiguration[i]; i++;
        receivedConfig.activationMode = nearestGatewayConfiguration[i]; i++;
        receivedConfig.getFirstTimeOverWiFi = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.getFirstTimeOverGPS = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.tagIdSource = nearestGatewayConfiguration[i]; i++;
        receivedConfig.imuMode = nearestGatewayConfiguration[i]; i++;
        receivedConfig.imuBurstMillis |= (nearestGatewayConfiguration[i] << 24); i++;
        receivedConfig.imuBurstMillis |= (nearestGatewayConfiguration[i] << 16); i++;
        receivedConfig.imuBurstMillis |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.imuBurstMillis |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.environmentActivated = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.timeCorrectionBetweenTags = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.timeCorrectionDiffMs |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.timeCorrectionDiffMs |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.freeMemoryIfFull = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.accFrequency = nearestGatewayConfiguration[i]; i++;
        receivedConfig.accAvg = nearestGatewayConfiguration[i]; i++;
        receivedConfig.accRange = nearestGatewayConfiguration[i]; i++;
        receivedConfig.magFrequency = nearestGatewayConfiguration[i]; i++;
        receivedConfig.magAccuracy = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gyroFrequency = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gyroRange = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gyroMode = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeEnter = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeTurnOnHour = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeTurnOnMinute = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeTurnOffHour = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeTurnOffMinute = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gpsFixHourBits |= (nearestGatewayConfiguration[i] << 24); i++;
        receivedConfig.gpsFixHourBits |= (nearestGatewayConfiguration[i] << 16); i++;
        receivedConfig.gpsFixHourBits |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.gpsFixHourBits |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.gpsRandomizeFixes = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.gpsRandomizeFixesPerDay = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gpsMinHdopTimesTen = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gpsFirstFixCollectOrbitDataSeconds = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gpsForcedAfterEveryProximity = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.gpsSyncRTCFrequency = nearestGatewayConfiguration[i]; i++; // new
        receivedConfig.proximityFrequencyMinute = nearestGatewayConfiguration[i]; i++;
        receivedConfig.proximityFrequencyMinuteSeenSomeone = nearestGatewayConfiguration[i]; i++;
        receivedConfig.proximityListeningIntervalMs |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.proximityListeningIntervalMs |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.proximityDbm = nearestGatewayConfiguration[i]; i++;
        receivedConfig.proximityDatarate = nearestGatewayConfiguration[i]; i++;
        receivedConfig.proximityLongRange = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.proximityAirTimeUs |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.proximityAirTimeUs |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.activationByGatewayListeningTime |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.activationByGatewayListeningTime |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.activationByGatewaySleepSeconds |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.activationByGatewaySleepSeconds |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.battMinVoltage |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.battMinVoltage |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.battRestartVoltage |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.battRestartVoltage |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.battMinVoltageDuringTransmission |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.battMinVoltageDuringTransmission |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.timeWifiOutputPower = nearestGatewayConfiguration[i]; i++;
        receivedConfig.timeBetweenGetTimeRetriesSeconds |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.timeBetweenGetTimeRetriesSeconds |= (nearestGatewayConfiguration[i] << 0); i++;

        // printing new configuration
        //printConfiguration(&receivedConfig);

        // plausibility check
        uint8_t errorId = 0;
        if(!configIsPlausible(&receivedConfig, &errorId)) {
            if(debug) { printf("RECONF: error config implausible (errorId %d)\n", errorId); }
            return false;
        }

        // check for differences
        if(receivedConfig.useLeds != currentConfig->useLeds) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing useLeds %d -> %d\n", currentConfig->useLeds, receivedConfig.useLeds); }
                currentConfig->useLeds = receivedConfig.useLeds;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_USE_LEDS, receivedConfig.useLeds)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.trackerMode != currentConfig->trackerMode) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing trackerMode %d -> %d\n", currentConfig->trackerMode, receivedConfig.trackerMode); }
                currentConfig->trackerMode = receivedConfig.trackerMode;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_TRACKER_MODE, receivedConfig.trackerMode)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.activationMode != currentConfig->activationMode) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activationMode %d -> %d\n", currentConfig->activationMode, receivedConfig.activationMode); }
                currentConfig->activationMode = receivedConfig.activationMode;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ACTIVATION_MODE, receivedConfig.activationMode)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.getFirstTimeOverWiFi != currentConfig->getFirstTimeOverWiFi) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing getFirstTimeOverWiFi %d -> %d\n", currentConfig->getFirstTimeOverWiFi, receivedConfig.getFirstTimeOverWiFi); }
                currentConfig->getFirstTimeOverWiFi = receivedConfig.getFirstTimeOverWiFi;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GET_FIRST_TIME_OVER_WIFI, receivedConfig.getFirstTimeOverWiFi)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.getFirstTimeOverGPS != currentConfig->getFirstTimeOverGPS) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing getFirstTimeOverGPS %d -> %d\n", currentConfig->getFirstTimeOverGPS, receivedConfig.getFirstTimeOverGPS); }
                currentConfig->getFirstTimeOverGPS = receivedConfig.getFirstTimeOverGPS;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GET_FIRST_TIME_OVER_GPS, receivedConfig.getFirstTimeOverGPS)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.tagIdSource != currentConfig->tagIdSource) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing tagIdSource %d -> %d\n", currentConfig->tagIdSource, receivedConfig.tagIdSource); }
                currentConfig->tagIdSource = receivedConfig.tagIdSource;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_TAG_ID_SOURCE, receivedConfig.tagIdSource)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.imuMode != currentConfig->imuMode) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing imuMode %d -> %d\n", currentConfig->imuMode, receivedConfig.imuMode); }
                currentConfig->imuMode = receivedConfig.imuMode;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_IMU_MODE, receivedConfig.imuMode)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.imuBurstMillis != currentConfig->imuBurstMillis) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing imuBurstMillis %d -> %d\n", currentConfig->imuBurstMillis, receivedConfig.imuBurstMillis); }
                currentConfig->imuBurstMillis = receivedConfig.imuBurstMillis;
                if(!device->defaultNvsWriteUINT32(NVS_CONF_IMU_BURST_MILLIS, receivedConfig.imuBurstMillis)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.environmentActivated != currentConfig->environmentActivated) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing environmentActivated %d -> %d\n", currentConfig->environmentActivated, receivedConfig.environmentActivated); }
                currentConfig->environmentActivated = receivedConfig.environmentActivated;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ENVIRONMENT_ACTIVATED, receivedConfig.environmentActivated)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.timeCorrectionBetweenTags != currentConfig->timeCorrectionBetweenTags) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing timeCorrectionBetweenTags %d -> %d\n", currentConfig->timeCorrectionBetweenTags, receivedConfig.timeCorrectionBetweenTags); }
                currentConfig->timeCorrectionBetweenTags = receivedConfig.timeCorrectionBetweenTags;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_TIME_CORRECTION_BETWEEN_TAGS, receivedConfig.timeCorrectionBetweenTags)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.timeCorrectionDiffMs != currentConfig->timeCorrectionDiffMs) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing timeCorrectionDiffMs %d -> %d\n", currentConfig->timeCorrectionDiffMs, receivedConfig.timeCorrectionDiffMs); }
                currentConfig->timeCorrectionDiffMs = receivedConfig.timeCorrectionDiffMs;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_TIME_CORRECTION_DIFF_MS, receivedConfig.timeCorrectionDiffMs)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.freeMemoryIfFull != currentConfig->freeMemoryIfFull) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing freeMemoryIfFull %d -> %d\n", currentConfig->freeMemoryIfFull, receivedConfig.freeMemoryIfFull); }
                currentConfig->freeMemoryIfFull = receivedConfig.freeMemoryIfFull;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_FREE_MEMORY_IF_FULL, receivedConfig.freeMemoryIfFull)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.accFrequency != currentConfig->accFrequency) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing accFrequency %d -> %d\n", currentConfig->accFrequency, receivedConfig.accFrequency); }
                currentConfig->accFrequency = receivedConfig.accFrequency;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ACC_FREQUENCY, receivedConfig.accFrequency)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.accAvg != currentConfig->accAvg) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing accAvg %d -> %d\n", currentConfig->accAvg, receivedConfig.accAvg); }
                currentConfig->accAvg = receivedConfig.accAvg;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ACC_AVG, receivedConfig.accAvg)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.accRange != currentConfig->accRange) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing accRange %d -> %d\n", currentConfig->accRange, receivedConfig.accRange); }
                currentConfig->accRange = receivedConfig.accRange;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ACC_RANGE, receivedConfig.accRange)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.magFrequency != currentConfig->magFrequency) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing magFrequency %d -> %d\n", currentConfig->magFrequency, receivedConfig.magFrequency); }
                currentConfig->magFrequency = receivedConfig.magFrequency;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_MAG_FREQUENCY, receivedConfig.magFrequency)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.magAccuracy != currentConfig->magAccuracy) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing magAccuracy %d -> %d\n", currentConfig->magAccuracy, receivedConfig.magAccuracy); }
                currentConfig->magAccuracy = receivedConfig.magAccuracy;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_MAG_ACCURACY, receivedConfig.magAccuracy)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gyroFrequency != currentConfig->gyroFrequency) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gyroFrequency %d -> %d\n", currentConfig->gyroFrequency, receivedConfig.gyroFrequency); }
                currentConfig->gyroFrequency = receivedConfig.gyroFrequency;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GYRO_FREQUENCY, receivedConfig.gyroFrequency)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gyroRange != currentConfig->gyroRange) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gyroRange %d -> %d\n", currentConfig->gyroRange, receivedConfig.gyroRange); }
                currentConfig->gyroRange = receivedConfig.gyroRange;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GYRO_RANGE, receivedConfig.gyroRange)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gyroMode != currentConfig->gyroMode) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gyroMode %d -> %d\n", currentConfig->gyroMode, receivedConfig.gyroMode); }
                currentConfig->gyroMode = receivedConfig.gyroMode;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GYRO_MODE, receivedConfig.gyroMode)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.nightTimeEnter != currentConfig->nightTimeEnter) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing nightTimeEnter %d -> %d\n", currentConfig->nightTimeEnter, receivedConfig.nightTimeEnter); }
                currentConfig->nightTimeEnter = receivedConfig.nightTimeEnter;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_NIGHTTIME_ENTER, receivedConfig.nightTimeEnter)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.nightTimeTurnOnHour != currentConfig->nightTimeTurnOnHour) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing nightTimeTurnOnHour %d -> %d\n", currentConfig->nightTimeTurnOnHour, receivedConfig.nightTimeTurnOnHour); }
                currentConfig->nightTimeTurnOnHour = receivedConfig.nightTimeTurnOnHour;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_NIGHTTIME_TURN_ON_HOUR, receivedConfig.nightTimeTurnOnHour)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.nightTimeTurnOnMinute != currentConfig->nightTimeTurnOnMinute) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing nightTimeTurnOnMinute %d -> %d\n", currentConfig->nightTimeTurnOnMinute, receivedConfig.nightTimeTurnOnMinute); }
                currentConfig->nightTimeTurnOnMinute = receivedConfig.nightTimeTurnOnMinute;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_NIGHTTIME_TURN_ON_MINUTE, receivedConfig.nightTimeTurnOnMinute)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.nightTimeTurnOffHour != currentConfig->nightTimeTurnOffHour) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing nightTimeTurnOffHour %d -> %d\n", currentConfig->nightTimeTurnOffHour, receivedConfig.nightTimeTurnOffHour); }
                currentConfig->nightTimeTurnOffHour = receivedConfig.nightTimeTurnOffHour;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_NIGHTTIME_TURN_OFF_HOUR, receivedConfig.nightTimeTurnOffHour)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.nightTimeTurnOffMinute != currentConfig->nightTimeTurnOffMinute) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing nightTimeTurnOffMinute %d -> %d\n", currentConfig->nightTimeTurnOffMinute, receivedConfig.nightTimeTurnOffMinute); }
                currentConfig->nightTimeTurnOffMinute = receivedConfig.nightTimeTurnOffMinute;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_NIGHTTIME_TURN_OFF_MINUTE, receivedConfig.nightTimeTurnOffMinute)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gpsFixHourBits != currentConfig->gpsFixHourBits) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gpsFixHourBits %08X -> %08X\n", currentConfig->gpsFixHourBits, receivedConfig.gpsFixHourBits); }
                currentConfig->gpsFixHourBits = receivedConfig.gpsFixHourBits;
                if(!device->defaultNvsWriteUINT32(NVS_CONF_GPS_FIX_HOUR_BITS, receivedConfig.gpsFixHourBits)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gpsRandomizeFixes != currentConfig->gpsRandomizeFixes) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gpsRandomizeFixes %d -> %d\n", currentConfig->gpsRandomizeFixes, receivedConfig.gpsRandomizeFixes); }
                currentConfig->gpsRandomizeFixes = receivedConfig.gpsRandomizeFixes;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GPS_RANDOMIZE_FIXES, receivedConfig.gpsRandomizeFixes)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gpsRandomizeFixesPerDay != currentConfig->gpsRandomizeFixesPerDay) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gpsRandomizeFixesPerDay %d -> %d\n", currentConfig->gpsRandomizeFixesPerDay, receivedConfig.gpsRandomizeFixesPerDay); }
                currentConfig->gpsRandomizeFixesPerDay = receivedConfig.gpsRandomizeFixesPerDay;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GPS_RANDOMIZE_FIXES_PER_DAY, receivedConfig.gpsRandomizeFixesPerDay)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gpsMinHdopTimesTen != currentConfig->gpsMinHdopTimesTen) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gpsMinHdopTimesTen %d -> %d\n", currentConfig->gpsMinHdopTimesTen, receivedConfig.gpsMinHdopTimesTen); }
                currentConfig->gpsMinHdopTimesTen = receivedConfig.gpsMinHdopTimesTen;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GPS_MIN_HDOP_X10, receivedConfig.gpsMinHdopTimesTen)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gpsFirstFixCollectOrbitDataSeconds != currentConfig->gpsFirstFixCollectOrbitDataSeconds) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gpsFirstFixCollectOrbitDataSeconds %d -> %d\n", currentConfig->gpsFirstFixCollectOrbitDataSeconds, receivedConfig.gpsFirstFixCollectOrbitDataSeconds); }
                currentConfig->gpsFirstFixCollectOrbitDataSeconds = receivedConfig.gpsFirstFixCollectOrbitDataSeconds;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GPS_FIRST_FIX_COLLECT_ORBIT_SECONDS, receivedConfig.gpsFirstFixCollectOrbitDataSeconds)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gpsForcedAfterEveryProximity != currentConfig->gpsForcedAfterEveryProximity) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gpsForcedAfterEveryProximity %d -> %d\n", currentConfig->gpsForcedAfterEveryProximity, receivedConfig.gpsForcedAfterEveryProximity); }
                currentConfig->gpsForcedAfterEveryProximity = receivedConfig.gpsForcedAfterEveryProximity;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GPS_FORCED_AFTER_EVERY_PROXIMITY, receivedConfig.gpsForcedAfterEveryProximity)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gpsSyncRTCFrequency != currentConfig->gpsSyncRTCFrequency) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gpsSyncRTCFrequency %d -> %d\n", currentConfig->gpsSyncRTCFrequency, receivedConfig.gpsSyncRTCFrequency); }
                currentConfig->gpsSyncRTCFrequency = receivedConfig.gpsSyncRTCFrequency;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_GPS_SYNC_RTC_FREQUENCY, receivedConfig.gpsSyncRTCFrequency)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.proximityFrequencyMinute != currentConfig->proximityFrequencyMinute) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing proximityFrequencyMinute %d -> %d\n", currentConfig->proximityFrequencyMinute, receivedConfig.proximityFrequencyMinute); }
                currentConfig->proximityFrequencyMinute = receivedConfig.proximityFrequencyMinute;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_PROXIMITY_FREQUENCY_MINUTE, receivedConfig.proximityFrequencyMinute)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.proximityFrequencyMinuteSeenSomeone != currentConfig->proximityFrequencyMinuteSeenSomeone) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing proximityFrequencyMinuteSeenSomeone %d -> %d\n", currentConfig->proximityFrequencyMinuteSeenSomeone, receivedConfig.proximityFrequencyMinuteSeenSomeone); }
                currentConfig->proximityFrequencyMinuteSeenSomeone = receivedConfig.proximityFrequencyMinuteSeenSomeone;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_PROXIMITY_FREQUENCY_MINUTE_SEENSOMEONE, receivedConfig.proximityFrequencyMinuteSeenSomeone)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.proximityListeningIntervalMs != currentConfig->proximityListeningIntervalMs) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing proximityListeningIntervalMs %d -> %d\n", currentConfig->proximityListeningIntervalMs, receivedConfig.proximityListeningIntervalMs); }
                currentConfig->proximityListeningIntervalMs = receivedConfig.proximityListeningIntervalMs;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_PROXIMITY_LISTENING_INTERVAL_MS, receivedConfig.proximityListeningIntervalMs)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.proximityDbm != currentConfig->proximityDbm) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing proximityDbm %d -> %d\n", currentConfig->proximityDbm, receivedConfig.proximityDbm); }
                currentConfig->proximityDbm = receivedConfig.proximityDbm;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_PROXIMITY_DBM, receivedConfig.proximityDbm)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.proximityDatarate != currentConfig->proximityDatarate) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing proximityDatarate %d -> %d\n", currentConfig->proximityDatarate, receivedConfig.proximityDatarate); }
                currentConfig->proximityDatarate = receivedConfig.proximityDatarate;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_PROXIMITY_DATARATE, receivedConfig.proximityDatarate)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.proximityLongRange != currentConfig->proximityLongRange) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing proximityLongRange %d -> %d\n", currentConfig->proximityLongRange, receivedConfig.proximityLongRange); }
                currentConfig->proximityLongRange = receivedConfig.proximityLongRange;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_PROXIMITY_LONGRANGE, receivedConfig.proximityLongRange)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.proximityAirTimeUs != currentConfig->proximityAirTimeUs) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing proximityAirTimeUs %d -> %d\n", currentConfig->proximityAirTimeUs, receivedConfig.proximityAirTimeUs); }
                currentConfig->proximityAirTimeUs = receivedConfig.proximityAirTimeUs;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_PROXIMITY_AIR_TIME_US, receivedConfig.proximityAirTimeUs)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.activationByGatewayListeningTime != currentConfig->activationByGatewayListeningTime) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activationByGatewayListeningTime %d -> %d\n", currentConfig->activationByGatewayListeningTime, receivedConfig.activationByGatewayListeningTime); }
                currentConfig->activationByGatewayListeningTime = receivedConfig.activationByGatewayListeningTime;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_ACTIVATION_BY_GATEWAY_LISTENING_TIME, receivedConfig.activationByGatewayListeningTime)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.activationByGatewaySleepSeconds != currentConfig->activationByGatewaySleepSeconds) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activationByGatewaySleepSeconds %d -> %d\n", currentConfig->activationByGatewaySleepSeconds, receivedConfig.activationByGatewaySleepSeconds); }
                currentConfig->activationByGatewaySleepSeconds = receivedConfig.activationByGatewaySleepSeconds;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_ACTIVATION_BY_GATEWAY_SLEEP_SECONDS, receivedConfig.activationByGatewaySleepSeconds)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.battMinVoltage != currentConfig->battMinVoltage) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing battMinVoltage %d -> %d\n", currentConfig->battMinVoltage, receivedConfig.battMinVoltage); }
                currentConfig->battMinVoltage = receivedConfig.battMinVoltage;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_BATT_MIN_VOLTAGE, receivedConfig.battMinVoltage)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.battRestartVoltage != currentConfig->battRestartVoltage) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing battRestartVoltage %d -> %d\n", currentConfig->battRestartVoltage, receivedConfig.battRestartVoltage); }
                currentConfig->battRestartVoltage = receivedConfig.battRestartVoltage;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_BATT_RESTART_VOLTAGE, receivedConfig.battRestartVoltage)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.battMinVoltageDuringTransmission != currentConfig->battMinVoltageDuringTransmission) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing battMinVoltageDuringTransmission %d -> %d\n", currentConfig->battMinVoltageDuringTransmission, receivedConfig.battMinVoltageDuringTransmission); }
                currentConfig->battMinVoltageDuringTransmission = receivedConfig.battMinVoltageDuringTransmission;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_BATT_MIN_VOLTAGE_DURING_TRANSM, receivedConfig.battMinVoltageDuringTransmission)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.timeWifiOutputPower != currentConfig->timeWifiOutputPower) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing timeWifiOutputPower %d -> %d\n", currentConfig->timeWifiOutputPower, receivedConfig.timeWifiOutputPower); }
                currentConfig->timeWifiOutputPower = receivedConfig.timeWifiOutputPower;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_TIME_WIFI_OUTPUT_POWER, receivedConfig.timeWifiOutputPower)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.timeBetweenGetTimeRetriesSeconds != currentConfig->timeBetweenGetTimeRetriesSeconds) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing timeBetweenGetTimeRetriesSeconds %d -> %d\n", currentConfig->timeBetweenGetTimeRetriesSeconds, receivedConfig.timeBetweenGetTimeRetriesSeconds); }
                currentConfig->timeBetweenGetTimeRetriesSeconds = receivedConfig.timeBetweenGetTimeRetriesSeconds;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS, receivedConfig.timeBetweenGetTimeRetriesSeconds)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }

        *changedSomething = (changeCnt > 0);

        // plausibility check of NEW configuration
        if(!configIsPlausible(currentConfig, &errorId)) {
            if(debug) { printf("RECONF: error AFTER config implausible (errorId: %d)\n", errorId); }
            noError = false;
        }

        // print results
        if(debug) { printf("RECONF: changed %d params (success: %d)\n", changeCnt, noError); }

    }
    else {
        if(debug) { printf("RECONF: no cmd to change config\n"); }
    }
    return noError;
}

bool readConfigFromNVS(WildFiTagREV6 *device, tag_config_t *config, bool printConfig) {
    bool neverWritten = false;
    uint8_t temp8 = 0;
    uint16_t temp16 = 0;
    uint32_t temp32 = 0;
    uint16_t nonDefaultCounter = 0;
    if(!device->initNVS()) { return false; }

    printf("readConfig:\n");
    printf("** IMPORTANT SETTINGS **\n");
    temp8 = device->defaultNvsReadUINT8(NVS_CONF_USE_LEDS, &neverWritten);
    if(neverWritten) { config->useLeds = DEF_USE_LEDS; }
    else { config->useLeds = (temp8 > 0); }
    if(printConfig) {
        printf(": useLeds: %d", config->useLeds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_TRACKER_MODE, &neverWritten);
    if(neverWritten) { config->trackerMode = DEF_TRACKER_MODE; }
    else { config->trackerMode = temp8; }
    if(printConfig) {
        printf(": trackerMode: %d", config->trackerMode);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACTIVATION_MODE, &neverWritten);
    if(neverWritten) { config->activationMode = DEF_ACTIVATION_MODE; }
    else { config->activationMode = temp8; }
    if(printConfig) {
        printf(": activationMode: %d", config->activationMode);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GET_FIRST_TIME_OVER_WIFI, &neverWritten);
    if(neverWritten) { config->getFirstTimeOverWiFi = DEF_GET_FIRST_TIME_OVER_WIFI; }
    else { config->getFirstTimeOverWiFi = (temp8 > 0); }
    if(printConfig) {
        printf(": getFirstTimeOverWiFi: %d", config->getFirstTimeOverWiFi);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GET_FIRST_TIME_OVER_GPS, &neverWritten);
    if(neverWritten) { config->getFirstTimeOverGPS = DEF_GET_FIRST_TIME_OVER_GPS; }
    else { config->getFirstTimeOverGPS = (temp8 > 0); }
    if(printConfig) {
        printf(": getFirstTimeOverGPS: %d", config->getFirstTimeOverGPS);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_TAG_ID_SOURCE, &neverWritten);
    if(neverWritten) { config->tagIdSource = DEF_TAG_ID_SOURCE; }
    else { config->tagIdSource = temp8; }
    if(printConfig) {
        printf(": tagIdSource: %d", config->tagIdSource);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_IMU_MODE, &neverWritten);
    if(neverWritten) { config->imuMode = DEF_IMU_MODE; }
    else { config->imuMode = temp8; }
    if(printConfig) {
        printf(": imuMode: %d", config->imuMode);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp32 = device->defaultNvsReadUINT32(NVS_CONF_IMU_BURST_MILLIS, &neverWritten);
    if(neverWritten) { config->imuBurstMillis = DEF_IMU_BURST_MILLIS; }
    else { config->imuBurstMillis = temp32; }
    if(printConfig) {
        printf(": imuBurstMillis: %d", config->imuBurstMillis);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ENVIRONMENT_ACTIVATED, &neverWritten);
    if(neverWritten) { config->environmentActivated = DEF_ENVIRONMENT_ACTIVATED; }
    else { config->environmentActivated = (temp8 > 0); }
    if(printConfig) {
        printf(": environmentActivated: %d", config->environmentActivated);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_TIME_CORRECTION_BETWEEN_TAGS, &neverWritten);
    if(neverWritten) { config->timeCorrectionBetweenTags = DEF_TIME_CORRECTION_BETWEEN_TAGS; }
    else { config->timeCorrectionBetweenTags = (temp8 > 0); }
    if(printConfig) {
        printf(": timeCorrectionBetweenTags: %d", config->timeCorrectionBetweenTags);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_TIME_CORRECTION_DIFF_MS, &neverWritten);
    if(neverWritten) { config->timeCorrectionDiffMs = DEF_TIME_CORRECTION_DIFF_MS; }
    else { config->timeCorrectionDiffMs = temp16; }
    if(printConfig) {
        printf(": timeCorrectionDiffMs: %d", config->timeCorrectionDiffMs);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_FREE_MEMORY_IF_FULL, &neverWritten);
    if(neverWritten) { config->freeMemoryIfFull = DEF_FREE_MEMORY_IF_FULL; }
    else { config->freeMemoryIfFull = (temp8 > 0); }
    if(printConfig) {
        printf(": freeMemoryIfFull: %d", config->freeMemoryIfFull);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** IMU SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACC_FREQUENCY, &neverWritten);
    if(neverWritten) { config->accFrequency = DEF_ACC_FREQUENCY; }
    else { config->accFrequency = temp8; }
    if(printConfig) {
        printf(": accFrequency: %d", config->accFrequency);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACC_AVG, &neverWritten);
    if(neverWritten) { config->accAvg = DEF_ACC_AVG; }
    else { config->accAvg = temp8; }
    if(printConfig) {
        printf(": accAvg: %d", config->accAvg);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACC_RANGE, &neverWritten);
    if(neverWritten) { config->accRange = DEF_ACC_RANGE; }
    else { config->accRange = temp8; }
    if(printConfig) {
        printf(": accRange: %d", config->accRange);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_MAG_FREQUENCY, &neverWritten);
    if(neverWritten) { config->magFrequency = DEF_MAG_FREQUENCY; }
    else { config->magFrequency = temp8; }
    if(printConfig) {
        printf(": magFrequency: %d", config->magFrequency);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_MAG_ACCURACY, &neverWritten);
    if(neverWritten) { config->magAccuracy = DEF_MAG_ACCURACY; }
    else { config->magAccuracy = temp8; }
    if(printConfig) {
        printf(": magAccuracy: %d", config->magAccuracy);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GYRO_FREQUENCY, &neverWritten);
    if(neverWritten) { config->gyroFrequency = DEF_GYRO_FREQUENCY; }
    else { config->gyroFrequency = temp8; }
    if(printConfig) {
        printf(": gyroFrequency: %d", config->gyroFrequency);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GYRO_RANGE, &neverWritten);
    if(neverWritten) { config->gyroRange = DEF_GYRO_RANGE; }
    else { config->gyroRange = temp8; }
    if(printConfig) {
        printf(": gyroRange: %d", config->gyroRange);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GYRO_MODE, &neverWritten);
    if(neverWritten) { config->gyroMode = DEF_GYRO_MODE; }
    else { config->gyroMode = temp8; }
    if(printConfig) {
        printf(": gyroMode: %d", config->gyroMode);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** NIGHT SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHTTIME_ENTER, &neverWritten);
    if(neverWritten) { config->nightTimeEnter = DEF_NIGHTTIME_ENTER; }
    else { config->nightTimeEnter = temp8; }
    if(printConfig) {
        printf(": nightTimeEnter: %d", config->nightTimeEnter);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHTTIME_TURN_ON_HOUR, &neverWritten);
    if(neverWritten) { config->nightTimeTurnOnHour = DEF_NIGHTTIME_TURN_ON_HOUR; }
    else { config->nightTimeTurnOnHour = temp8; }
    if(printConfig) {
        printf(": nightTimeTurnOnHour: %d", config->nightTimeTurnOnHour);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHTTIME_TURN_ON_MINUTE, &neverWritten);
    if(neverWritten) { config->nightTimeTurnOnMinute = DEF_NIGHTTIME_TURN_ON_MINUTE; }
    else { config->nightTimeTurnOnMinute = temp8; }
    if(printConfig) {
        printf(": nightTimeTurnOnMinute: %d", config->nightTimeTurnOnMinute);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHTTIME_TURN_OFF_HOUR, &neverWritten);
    if(neverWritten) { config->nightTimeTurnOffHour = DEF_NIGHTTIME_TURN_OFF_HOUR; }
    else { config->nightTimeTurnOffHour = temp8; }
    if(printConfig) {
        printf(": nightTimeTurnOffHour: %d", config->nightTimeTurnOffHour);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHTTIME_TURN_OFF_MINUTE, &neverWritten);
    if(neverWritten) { config->nightTimeTurnOffMinute = DEF_NIGHTTIME_TURN_OFF_MINUTE; }
    else { config->nightTimeTurnOffMinute = temp8; }
    if(printConfig) {
        printf(": nightTimeTurnOffMinute: %d", config->nightTimeTurnOffMinute);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** GPS SETTINGS **\n");

    temp32 = device->defaultNvsReadUINT32(NVS_CONF_GPS_FIX_HOUR_BITS, &neverWritten);
    if(neverWritten) { config->gpsFixHourBits = DEF_GPS_FIX_HOUR_BITS; }
    else { config->gpsFixHourBits = temp32; }
    if(printConfig) {
        printf(": gpsFixHourBits: %d", config->gpsFixHourBits);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GPS_RANDOMIZE_FIXES, &neverWritten);
    if(neverWritten) { config->gpsRandomizeFixes = DEF_GPS_RANDOMIZE_FIXES; }
    else { config->gpsRandomizeFixes = (temp8 > 0); }
    if(printConfig) {
        printf(": gpsRandomizeFixes: %d", config->gpsRandomizeFixes);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GPS_RANDOMIZE_FIXES_PER_DAY, &neverWritten);
    if(neverWritten) { config->gpsRandomizeFixesPerDay = DEF_GPS_RANDOMIZE_FIXES_PER_DAY; }
    else { config->gpsRandomizeFixesPerDay = temp8; }
    if(printConfig) {
        printf(": gpsRandomizeFixesPerDay: %d", config->gpsRandomizeFixesPerDay);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GPS_MIN_HDOP_X10, &neverWritten);
    if(neverWritten) { config->gpsMinHdopTimesTen = DEF_GPS_MIN_HDOP_X10; }
    else { config->gpsMinHdopTimesTen = temp8; }
    if(printConfig) {
        printf(": gpsMinHdopTimesTen: %d", config->gpsMinHdopTimesTen);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GPS_FIRST_FIX_COLLECT_ORBIT_SECONDS, &neverWritten);
    if(neverWritten) { config->gpsFirstFixCollectOrbitDataSeconds = DEF_GPS_FIRST_FIX_COLLECT_ORBIT_DATA_SECONDS; }
    else { config->gpsFirstFixCollectOrbitDataSeconds = temp8; }
    if(printConfig) {
        printf(": gpsFirstFixCollectOrbitDataSeconds: %d", config->gpsFirstFixCollectOrbitDataSeconds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GPS_FORCED_AFTER_EVERY_PROXIMITY, &neverWritten);
    if(neverWritten) { config->gpsForcedAfterEveryProximity = DEF_GPS_FORCED_AFTER_EVERY_PROXIMITY; }
    else { config->gpsForcedAfterEveryProximity = (temp8 > 0); }
    if(printConfig) {
        printf(": gpsForcedAfterEveryProximity: %d", config->gpsForcedAfterEveryProximity);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GPS_SYNC_RTC_FREQUENCY, &neverWritten);
    if(neverWritten) { config->gpsSyncRTCFrequency = DEF_GPS_SYNC_RTC_FREQUENCY; }
    else { config->gpsSyncRTCFrequency = temp8; }
    if(printConfig) {
        printf(": gpsSyncRTCFrequency: %d", config->gpsSyncRTCFrequency);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** PROXIMITY SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_PROXIMITY_FREQUENCY_MINUTE, &neverWritten);
    if(neverWritten) { config->proximityFrequencyMinute = DEF_PROXIMITY_FREQUENCY_MINUTE; }
    else { config->proximityFrequencyMinute = temp8; }
    if(printConfig) {
        printf(": proximityFrequencyMinute: %d", config->proximityFrequencyMinute);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_PROXIMITY_FREQUENCY_MINUTE_SEENSOMEONE, &neverWritten);
    if(neverWritten) { config->proximityFrequencyMinuteSeenSomeone = DEF_PROXIMITY_FREQUENCY_MINUTE_SEEN_SOMEONE; }
    else { config->proximityFrequencyMinuteSeenSomeone = temp8; }
    if(printConfig) {
        printf(": proximityFrequencyMinuteSeenSomeone: %d", config->proximityFrequencyMinuteSeenSomeone);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_PROXIMITY_LISTENING_INTERVAL_MS, &neverWritten);
    if(neverWritten) { config->proximityListeningIntervalMs = DEF_PROXIMITY_LISTENING_INTERVAL_MS; }
    else { config->proximityListeningIntervalMs = temp16; }
    if(printConfig) {
        printf(": proximityListeningIntervalMs: %d", config->proximityListeningIntervalMs);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_PROXIMITY_DBM, &neverWritten);
    if(neverWritten) { config->proximityDbm = DEF_PROXIMITY_DBM; }
    else { config->proximityDbm = temp8; }
    if(printConfig) {
        printf(": proximityDbm: %d", config->proximityDbm);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_PROXIMITY_DATARATE, &neverWritten);
    if(neverWritten) { config->proximityDatarate = DEF_PROXIMITY_DATARATE; }
    else { config->proximityDatarate = temp8; }
    if(printConfig) {
        printf(": proximityDatarate: %d", config->proximityDatarate);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_PROXIMITY_LONGRANGE, &neverWritten);
    if(neverWritten) { config->proximityLongRange = DEF_PROXIMITY_LONGRANGE; }
    else { config->proximityLongRange = (temp8 > 0); }
    if(printConfig) {
        printf(": proximityLongRange: %d", config->proximityLongRange);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_PROXIMITY_AIR_TIME_US, &neverWritten);
    if(neverWritten) { config->proximityAirTimeUs = DEF_PROXIMITY_AIR_TIME_US; }
    else { config->proximityAirTimeUs = temp16; }
    if(printConfig) {
        printf(": proximityAirTimeUs: %d", config->proximityAirTimeUs);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** ACTIVATION SETTINGS **\n");

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_ACTIVATION_BY_GATEWAY_LISTENING_TIME, &neverWritten);
    if(neverWritten) { config->activationByGatewayListeningTime = DEF_ACTIVATION_BY_GATEWAY_LISTENING_TIME; }
    else { config->activationByGatewayListeningTime = temp16; }
    if(printConfig) {
        printf(": activationByGatewayListeningTime: %d", config->activationByGatewayListeningTime);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_ACTIVATION_BY_GATEWAY_SLEEP_SECONDS, &neverWritten);
    if(neverWritten) { config->activationByGatewaySleepSeconds = DEF_ACTIVATION_BY_GATEWAY_SLEEP_SECONDS; }
    else { config->activationByGatewaySleepSeconds = temp16; }
    if(printConfig) {
        printf(": activationByGatewaySleepSeconds: %d", config->activationByGatewaySleepSeconds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** BATTERY SETTINGS **\n");
    
    temp16 = device->defaultNvsReadUINT16(NVS_CONF_BATT_MIN_VOLTAGE, &neverWritten);
    if(neverWritten) { config->battMinVoltage = DEF_BATT_MIN_VOLTAGE; }
    else { config->battMinVoltage = temp16; }
    if(printConfig) {
        printf(": battMinVoltage: %d", config->battMinVoltage);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_BATT_RESTART_VOLTAGE, &neverWritten);
    if(neverWritten) { config->battRestartVoltage = DEF_BATT_RESTART_VOLTAGE; }
    else { config->battRestartVoltage = temp16; }
    if(printConfig) {
        printf(": battRestartVoltage: %d", config->battRestartVoltage);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_BATT_MIN_VOLTAGE_DURING_TRANSM, &neverWritten);
    if(neverWritten) { config->battMinVoltageDuringTransmission = DEF_BATT_MIN_VOLTAGE_DURING_TRANSM; }
    else { config->battMinVoltageDuringTransmission = temp16; }
    if(printConfig) {
        printf(": battMinVoltageDuringTransmission: %d", config->battMinVoltageDuringTransmission);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** GET TIME SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_TIME_WIFI_OUTPUT_POWER, &neverWritten);
    if(neverWritten) { config->timeWifiOutputPower = DEF_TIME_WIFI_OUTPUT_POWER; }
    else { config->timeWifiOutputPower = temp8; }
    if(printConfig) {
        printf(": timeWifiOutputPower: %d", config->timeWifiOutputPower);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS, &neverWritten);
    if(neverWritten) { config->timeBetweenGetTimeRetriesSeconds = DEF_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS; }
    else { config->timeBetweenGetTimeRetriesSeconds = temp16; }
    if(printConfig) {
        printf(": timeBetweenGetTimeRetriesSeconds: %d", config->timeBetweenGetTimeRetriesSeconds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    if(printConfig) {
        printf("** NON-DEFAULT SETTINGS: %d **\n", nonDefaultCounter);
    }
    return true;
}