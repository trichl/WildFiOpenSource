#include "configDefault.h"

void printConfigurationHash(tag_config_t *c) {
    printf("confHash: ");
    printf("%01X", c->doTheBlink);
    printf("%01X", c->trackerMode);
    printf("%01X", c->trackingDataMode);
    printf("%01X", c->transmissionMethod);
    printf("%02X", c->drMinHdopXTen);
    printf("%04X", c->drImuSeconds);
    printf("%04X", c->gps1HzInterruptAfterSeconds);
    printf("%02X", c->accFrequency);
    printf("%02X", c->accAvg);
    printf("%02X", c->accRange);
    printf("%02X", c->magFrequency);
    printf("%02X", c->magAccuracy);
    printf("%02X", c->gyroFrequency);
    printf("%02X", c->gyroRange);
    printf("%02X", c->gyroMode);
    printf("%01X", c->useMagnetometer);
    printf("%01X", c->useGyro);
    printf("%01X", c->nightTimeEnter);
    printf("%01X", c->nightTimeMode);
    printf("%02X", c->nightTimeTurnOnHour);
    printf("%02X", c->nightTimeTurnOnMinute);
    printf("%02X", c->nightTimeTurnOffHour);
    printf("%02X", c->nightTimeTurnOffMinute);
    printf("%08X", c->nightTimeDataTransDeepestNightHours);
    printf("%04X", c->nightTimeModeTryDataTransWakeupSeconds);
    printf("%04X", c->battMinVoltage);
    printf("%04X", c->battRestartVoltage);
    printf("%04X", c->dataTransBattMinVoltage);
    printf("%04X", c->dataTransBattMinVoltageDuringTrans);
    printf("%01X", c->skipGetTime);
    printf("%04X", c->timeBetweenGetTimeRetriesSeconds);
    printf("%01X", c->activationMode);
    printf("%01X", c->activationSource);
    printf("%04X", c->activationByEspNowRetrySeconds);
    printf("%04X", c->dataTransTryEveryFullMinSeldomly);
    printf("%04X", c->memFullTryEveryFullMinSeldomly);
    printf("%04X", c->accInterruptWatermark);
    printf("%02X", c->dataTransOutputPower);
    printf("%01X", c->espNowCustomRFCalibration);
    printf("%04X", c->commandByteForceTrackingDurationSeconds);
    printf("%02X", c->espNowDataRate);
    printf("%01X", c->espNowLongRange);
    printf("%08X", c->espNowMinBytesToTransmit);
    printf("%08X", c->espNowMaxBytesToTransmit);
    printf("%04X", c->wifiMinBlocksToTransmit);
    printf("%04X", c->wifiMaxBlocksToTransmit);
    printf("%01X", c->activityActivationEnabled);
    printf("%04X", c->activityThresholdActiveToInactiveAvg);
    printf("%02X", c->activityThresholdInactiveToActiveThr);
    printf("%04X", c->activityTransmissionInterval);
    printf("\n");
}

void printConfiguration(tag_config_t *c) {
    printf("doTheBlink: %d\n", c->doTheBlink);
    printf("trackerMode: %d\n", c->trackerMode);
    printf("trackingDataMode: %d\n", c->trackingDataMode);
    printf("transmissionMethod: %d\n", c->transmissionMethod);
    printf("drMinHdopXTen: %d\n", c->drMinHdopXTen);
    printf("drImuSeconds: %d\n", c->drImuSeconds);
    printf("gps1HzInterruptAfterSeconds: %d\n", c->gps1HzInterruptAfterSeconds);
    printf("accFrequency: %02X\n", c->accFrequency);
    printf("accAvg: %02X\n", c->accAvg);
    printf("accRange: %02X\n", c->accRange);
    printf("magFrequency: %02X\n", c->magFrequency);
    printf("magAccuracy: %02X\n", c->magAccuracy);
    printf("gyroFrequency: %02X\n", c->gyroFrequency);
    printf("gyroRange: %02X\n", c->gyroRange);
    printf("gyroMode: %02X\n", c->gyroMode);
    printf("useMagnetometer: %02X\n", c->useMagnetometer);
    printf("useGyro: %02X\n", c->useGyro);
    printf("nightTimeEnter: %d\n", c->nightTimeEnter);
    printf("nightTimeMode: %d\n", c->nightTimeMode);
    printf("nightTimeTurnOnHour: %d\n", c->nightTimeTurnOnHour);
    printf("nightTimeTurnOnMinute: %d\n", c->nightTimeTurnOnMinute);
    printf("nightTimeTurnOffHour: %d\n", c->nightTimeTurnOffHour);
    printf("nightTimeTurnOffMinute: %d\n", c->nightTimeTurnOffMinute);
    printf("nightTimeDataTransDeepestNightHours: %08X\n", c->nightTimeDataTransDeepestNightHours);
    printf("nightTimeModeTryDataTransWakeupSeconds: %d\n", c->nightTimeModeTryDataTransWakeupSeconds);
    printf("battMinVoltage: %d\n", c->battMinVoltage);
    printf("battRestartVoltage: %d\n", c->battRestartVoltage);
    printf("dataTransBattMinVoltage: %d\n", c->dataTransBattMinVoltage);
    printf("dataTransBattMinVoltageDuringTrans: %d\n", c->dataTransBattMinVoltageDuringTrans);
    printf("skipGetTime: %d\n", c->skipGetTime);
    printf("timeBetweenGetTimeRetriesSeconds: %d\n", c->timeBetweenGetTimeRetriesSeconds);
    printf("activationMode: %d\n", c->activationMode);
    printf("activationSource: %d\n", c->activationSource);
    printf("activationByEspNowRetrySeconds: %d\n", c->activationByEspNowRetrySeconds);
    printf("dataTransTryEveryFullMinSeldomly: %d\n", c->dataTransTryEveryFullMinSeldomly);
    printf("memFullTryEveryFullMinSeldomly: %d\n", c->memFullTryEveryFullMinSeldomly);
    printf("accInterruptWatermark: %d\n", c->accInterruptWatermark);
    printf("dataTransOutputPower: %d\n", c->dataTransOutputPower);
    printf("espNowCustomRFCalibration: %d\n", c->espNowCustomRFCalibration);
    printf("commandByteForceTrackingDurationSeconds: %d\n", c->commandByteForceTrackingDurationSeconds);
    printf("espNowDataRate: %d\n", c->espNowDataRate);
    printf("espNowLongRange: %d\n", c->espNowLongRange);
    printf("espNowMinBytesToTransmit: %d\n", c->espNowMinBytesToTransmit);
    printf("espNowMaxBytesToTransmit: %d\n", c->espNowMaxBytesToTransmit);
    printf("wifiMinBlocksToTransmit: %d\n", c->wifiMinBlocksToTransmit);
    printf("wifiMaxBlocksToTransmit: %d\n", c->wifiMaxBlocksToTransmit);
    printf("activityActivationEnabled: %d\n", c->activityActivationEnabled);
    printf("activityThresholdActiveToInactiveAvg: %d\n", c->activityThresholdActiveToInactiveAvg);
    printf("activityThresholdInactiveToActiveThr: %d\n", c->activityThresholdInactiveToActiveThr);
    printf("activityTransmissionInterval: %d\n", c->activityTransmissionInterval);
}

bool configIsPlausible(tag_config_t *config, uint8_t *errorIdIn) {
    bool allGood = true;
    uint8_t errorId = 0;
    if((config->trackerMode != MODE_TESTRUN) && (config->trackerMode != MODE_PRODUCTIVE)) { allGood = false; errorId = 1; } // REFUSE:  && (config->trackerMode != MODE_SELFTEST) && (config->trackerMode != MODE_READFLASH) && (config->trackerMode != MODE_MOCK_FLASH_STATE)
    if((config->trackingDataMode != TRACKING_DATA_MODE_1HZ_GPS_AND_IMU) && (config->trackingDataMode != TRACKING_DATA_MODE_IMU_SD_DEEP_SLEEP) && (config->trackingDataMode != TRACKING_DATA_MODE_IMU_HD_LIGHT_SLEEP) && (config->trackingDataMode != TRACKING_DATA_MODE_DEAD_RECKONING)) { allGood = false; errorId = 2; }
    if((config->transmissionMethod != TRANSMISSION_METHOD_ESP_NOW) && (config->transmissionMethod != TRANSMISSION_METHOD_WIFI)) { allGood = false; errorId = 3; }
    if(config->drImuSeconds == 0) { allGood = false; errorId = 4; }
    if((config->accFrequency < BMX160_ACCEL_ODR_0_78HZ) || (config->accFrequency > BMX160_ACCEL_ODR_1600HZ)) { allGood = false; errorId = 5; }
    if(config->accAvg > BMX160_ACCEL_BW_RES_AVG16) { allGood = false; errorId = 6; }
    if((config->accRange != BMX160_ACCEL_RANGE_2G) && (config->accRange != BMX160_ACCEL_RANGE_4G) && (config->accRange != BMX160_ACCEL_RANGE_8G) && (config->accRange != BMX160_ACCEL_RANGE_16G)) { allGood = false; errorId = 7; }
    if((config->magFrequency < BMX160_MAG_ODR_0_78HZ) || (config->magFrequency > BMX160_MAG_ODR_50HZ)) { allGood = false; errorId = 8; }
    if(config->magAccuracy > BMX160_MAG_ACCURACY_HIGH) { allGood = false; errorId = 9; }
    if((config->gyroFrequency < BMX160_GYRO_ODR_25HZ) || (config->gyroFrequency > BMX160_GYRO_ODR_3200HZ)) { allGood = false; errorId = 10; }
    if(config->gyroRange > BMX160_GYRO_RANGE_125_DPS) { allGood = false; errorId = 11; }
    if(config->gyroMode > BMX160_GYRO_BW_NORMAL_MODE) { allGood = false; errorId = 12; }
    if(config->nightTimeEnter > NIGHTTIME_ONLY_BELOW_380V) { allGood = false; errorId = 13; }
    if((config->nightTimeMode != NIGHTTIME_MODE_ONLY_SLEEP) && (config->nightTimeMode != NIGHTTIME_MODE_TRY_DATATRANS)) { allGood = false; errorId = 14; }
    if(config->nightTimeTurnOnHour > 23) { allGood = false; errorId = 15; }
    if(config->nightTimeTurnOnMinute > 59) { allGood = false; errorId = 16; }
    if(config->nightTimeTurnOffHour > 23) { allGood = false; errorId = 17; }
    if(config->nightTimeTurnOffMinute > 59) { allGood = false; errorId = 18; }
    if(config->nightTimeDataTransDeepestNightHours > (HOUR_0 | HOUR_1 | HOUR_2 | HOUR_3 | HOUR_4 | HOUR_5 | HOUR_6 | HOUR_7 | HOUR_8 | HOUR_9 | HOUR_10 | HOUR_11 | HOUR_12 | HOUR_13 | HOUR_14 | HOUR_15 | HOUR_16 | HOUR_17 | HOUR_18 | HOUR_19 | HOUR_20 | HOUR_21 | HOUR_22 | HOUR_23)) {
        allGood = false; errorId = 19;
    }
    if(config->nightTimeModeTryDataTransWakeupSeconds >= 4096) { allGood = false; errorId = 20; }
    if(config->battMinVoltage > 6000) { allGood = false; errorId = 21; }
    if(config->battRestartVoltage > 6000) { allGood = false; errorId = 22; }
    if(config->dataTransBattMinVoltage > 6000) { allGood = false; errorId = 23; }
    if(config->dataTransBattMinVoltageDuringTrans > 6000) { allGood = false; errorId = 24; }
    if(config->timeBetweenGetTimeRetriesSeconds == 0) { allGood = false; errorId = 25; }
    if((config->activationMode != ACTIVATION_MODE_SKIP) && (config->activationMode != ACTIVATION_MODE_STORE_PERMANENTLY) && (config->activationMode != ACTIVATION_MODE_ON_EVERY_START)) { allGood = false; errorId = 26; }
    if((config->activationSource != ACTIVATION_BY_WIFI) && (config->activationSource != ACTIVATION_BY_ESPNOW)) { allGood = false; errorId = 27; }
    if(config->activationByEspNowRetrySeconds == 0) { allGood = false; errorId = 28; }
    if(config->dataTransTryEveryFullMinSeldomly == 0) { allGood = false; errorId = 29; }
    if(config->memFullTryEveryFullMinSeldomly == 0) { allGood = false; errorId = 30; }
    if((config->accInterruptWatermark == 0) || (config->accInterruptWatermark > 1024)) { allGood = false; errorId = 31; }
    if(config->dataTransOutputPower > RADIO_MAX_TX_19_5_DBM) { allGood = false; errorId = 32; }
    if(config->commandByteForceTrackingDurationSeconds == 0) { allGood = false; errorId = 33; }
    if(config->espNowDataRate > WIFI_PHY_RATE_MAX) { allGood = false; errorId = 34; }
    if(config->espNowMinBytesToTransmit == 0) { allGood = false; errorId = 35; }
    if(config->wifiMinBlocksToTransmit == 0) { allGood = false; errorId = 36; }
    if(config->activityTransmissionInterval >= 4096) { allGood = false; errorId = 37; }
    if(errorIdIn != NULL) { *errorIdIn = errorId; }
    return allGood;
}

bool checkIfReconfigNeeded(WildFiTagREV6 *device, tag_config_t *currentConfig, uint8_t *nearestGatewayConfiguration, uint8_t nearestGatewayCommand, uint8_t nearestGatewayRssi, bool *changedSomething, bool debug) {
    bool noError = true;
    uint16_t changeCnt = 0;
    if(debug) {
        printf("RECONF: (CMD: %02X, RSSI: -%d), CONFIG: ", nearestGatewayCommand, nearestGatewayRssi);
        for(uint8_t i=0; i<ESPNOW_META_MSG_GATEWAY_AROUND_V2_LEN; i++) { printf("%02X", nearestGatewayConfiguration[i]); }
        printf("\n");
    }
    if(nearestGatewayCommand == COMMAND_BYTE_CHANGE_CONFIG) {
        // parsing data into config struct
        tag_config_t receivedConfig = { }; // init with only zeros
        uint16_t i = 0;
        receivedConfig.doTheBlink = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.trackerMode = nearestGatewayConfiguration[i]; i++;
        receivedConfig.trackingDataMode = nearestGatewayConfiguration[i]; i++;
        receivedConfig.transmissionMethod = nearestGatewayConfiguration[i]; i++;
        receivedConfig.drMinHdopXTen = nearestGatewayConfiguration[i]; i++;
        receivedConfig.drImuSeconds |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.drImuSeconds |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.gps1HzInterruptAfterSeconds |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.gps1HzInterruptAfterSeconds |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.accFrequency = nearestGatewayConfiguration[i]; i++;
        receivedConfig.accAvg = nearestGatewayConfiguration[i]; i++;
        receivedConfig.accRange = nearestGatewayConfiguration[i]; i++;
        receivedConfig.magFrequency = nearestGatewayConfiguration[i]; i++;
        receivedConfig.magAccuracy = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gyroFrequency = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gyroRange = nearestGatewayConfiguration[i]; i++;
        receivedConfig.gyroMode = nearestGatewayConfiguration[i]; i++;
        receivedConfig.useMagnetometer = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.useGyro = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.nightTimeEnter = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeMode = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeTurnOnHour = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeTurnOnMinute = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeTurnOffHour = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeTurnOffMinute = nearestGatewayConfiguration[i]; i++;
        receivedConfig.nightTimeDataTransDeepestNightHours |= (nearestGatewayConfiguration[i] << 24); i++;
        receivedConfig.nightTimeDataTransDeepestNightHours |= (nearestGatewayConfiguration[i] << 16); i++;
        receivedConfig.nightTimeDataTransDeepestNightHours |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.nightTimeDataTransDeepestNightHours |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.nightTimeModeTryDataTransWakeupSeconds |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.nightTimeModeTryDataTransWakeupSeconds |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.battMinVoltage |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.battMinVoltage |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.battRestartVoltage |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.battRestartVoltage |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.dataTransBattMinVoltage |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.dataTransBattMinVoltage |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.dataTransBattMinVoltageDuringTrans |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.dataTransBattMinVoltageDuringTrans |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.skipGetTime = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.timeBetweenGetTimeRetriesSeconds |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.timeBetweenGetTimeRetriesSeconds |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.activationMode = nearestGatewayConfiguration[i]; i++;
        receivedConfig.activationSource = nearestGatewayConfiguration[i]; i++;
        receivedConfig.activationByEspNowRetrySeconds |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.activationByEspNowRetrySeconds |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.dataTransTryEveryFullMinSeldomly |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.dataTransTryEveryFullMinSeldomly |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.memFullTryEveryFullMinSeldomly |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.memFullTryEveryFullMinSeldomly |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.accInterruptWatermark |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.accInterruptWatermark |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.dataTransOutputPower = nearestGatewayConfiguration[i]; i++;
        receivedConfig.espNowCustomRFCalibration = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.commandByteForceTrackingDurationSeconds |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.commandByteForceTrackingDurationSeconds |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.espNowDataRate = nearestGatewayConfiguration[i]; i++;
        receivedConfig.espNowLongRange = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.espNowMinBytesToTransmit |= (nearestGatewayConfiguration[i] << 24); i++;
        receivedConfig.espNowMinBytesToTransmit |= (nearestGatewayConfiguration[i] << 16); i++;
        receivedConfig.espNowMinBytesToTransmit |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.espNowMinBytesToTransmit |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.espNowMaxBytesToTransmit |= (nearestGatewayConfiguration[i] << 24); i++;
        receivedConfig.espNowMaxBytesToTransmit |= (nearestGatewayConfiguration[i] << 16); i++;
        receivedConfig.espNowMaxBytesToTransmit |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.espNowMaxBytesToTransmit |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.wifiMinBlocksToTransmit |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.wifiMinBlocksToTransmit |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.wifiMaxBlocksToTransmit |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.wifiMaxBlocksToTransmit |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.activityActivationEnabled = (nearestGatewayConfiguration[i] > 0); i++;
        receivedConfig.activityThresholdActiveToInactiveAvg |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.activityThresholdActiveToInactiveAvg |= (nearestGatewayConfiguration[i] << 0); i++;
        receivedConfig.activityThresholdInactiveToActiveThr = nearestGatewayConfiguration[i]; i++;
        receivedConfig.activityTransmissionInterval |= (nearestGatewayConfiguration[i] << 8); i++;
        receivedConfig.activityTransmissionInterval |= (nearestGatewayConfiguration[i] << 0); i++;

        // printing new configuration
        //printConfiguration(&receivedConfig);

        // plausibility check
        uint8_t errorId = 0;
        if(!configIsPlausible(&receivedConfig, &errorId)) {
            if(debug) { printf("RECONF: error config implausible (errorId %d)\n", errorId); }
            return false;
        }

        // check for differences
        if(receivedConfig.doTheBlink != currentConfig->doTheBlink) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing doTheBlink %d -> %d\n", currentConfig->doTheBlink, receivedConfig.doTheBlink); }
                currentConfig->doTheBlink = receivedConfig.doTheBlink;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_DO_THE_BLINK, receivedConfig.doTheBlink)) { noError = false; }
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
        if(receivedConfig.trackingDataMode != currentConfig->trackingDataMode) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing trackingDataMode %d -> %d\n", currentConfig->trackingDataMode, receivedConfig.trackingDataMode); }
                currentConfig->trackingDataMode = receivedConfig.trackingDataMode;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_TRACKING_DATA_MODE, receivedConfig.trackingDataMode)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.transmissionMethod != currentConfig->transmissionMethod) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing transmissionMethod %d -> %d\n", currentConfig->transmissionMethod, receivedConfig.transmissionMethod); }
                currentConfig->transmissionMethod = receivedConfig.transmissionMethod;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_TRANSMISSION_METHOD, receivedConfig.transmissionMethod)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.drMinHdopXTen != currentConfig->drMinHdopXTen) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing drMinHdopXTen %d -> %d\n", currentConfig->drMinHdopXTen, receivedConfig.drMinHdopXTen); }
                currentConfig->drMinHdopXTen = receivedConfig.drMinHdopXTen;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_DR_MIN_HDOP_X_TEN, receivedConfig.drMinHdopXTen)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.drImuSeconds != currentConfig->drImuSeconds) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing drImuSeconds %d -> %d\n", currentConfig->drImuSeconds, receivedConfig.drImuSeconds); }
                currentConfig->drImuSeconds = receivedConfig.drImuSeconds;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_DR_IMU_SECONDS, receivedConfig.drImuSeconds)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.gps1HzInterruptAfterSeconds != currentConfig->gps1HzInterruptAfterSeconds) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing gps1HzInterruptAfterSeconds %d -> %d\n", currentConfig->gps1HzInterruptAfterSeconds, receivedConfig.gps1HzInterruptAfterSeconds); }
                currentConfig->gps1HzInterruptAfterSeconds = receivedConfig.gps1HzInterruptAfterSeconds;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_1HZ_INTERRUPT_AFTER_SECONDS, receivedConfig.gps1HzInterruptAfterSeconds)) { noError = false; }
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
        if(receivedConfig.useMagnetometer != currentConfig->useMagnetometer) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing useMagnetometer %d -> %d\n", currentConfig->useMagnetometer, receivedConfig.useMagnetometer); }
                currentConfig->useMagnetometer = receivedConfig.useMagnetometer;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_USE_MAG, receivedConfig.useMagnetometer)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.useGyro != currentConfig->useGyro) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing useGyro %d -> %d\n", currentConfig->useGyro, receivedConfig.useGyro); }
                currentConfig->useGyro = receivedConfig.useGyro;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_USE_GYRO, receivedConfig.useGyro)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.nightTimeEnter != currentConfig->nightTimeEnter) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing nightTimeEnter %d -> %d\n", currentConfig->nightTimeEnter, receivedConfig.nightTimeEnter); }
                currentConfig->nightTimeEnter = receivedConfig.nightTimeEnter;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_NIGHT_TIME_ENTER, receivedConfig.nightTimeEnter)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.nightTimeMode != currentConfig->nightTimeMode) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing nightTimeMode %d -> %d\n", currentConfig->nightTimeMode, receivedConfig.nightTimeMode); }
                currentConfig->nightTimeMode = receivedConfig.nightTimeMode;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_NIGHT_TIME_MODE, receivedConfig.nightTimeMode)) { noError = false; }
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
        if(receivedConfig.nightTimeDataTransDeepestNightHours != currentConfig->nightTimeDataTransDeepestNightHours) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing nightTimeDataTransDeepestNightHours %08X -> %08X\n", currentConfig->nightTimeDataTransDeepestNightHours, receivedConfig.nightTimeDataTransDeepestNightHours); }
                currentConfig->nightTimeDataTransDeepestNightHours = receivedConfig.nightTimeDataTransDeepestNightHours;
                if(!device->defaultNvsWriteUINT32(NVS_CONF_NIGHTTIME_DATA_TRANS_DEEP_NIGHT_HOURS, receivedConfig.nightTimeDataTransDeepestNightHours)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.nightTimeModeTryDataTransWakeupSeconds != currentConfig->nightTimeModeTryDataTransWakeupSeconds) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing nightTimeModeTryDataTransWakeupSeconds %d -> %d\n", currentConfig->nightTimeModeTryDataTransWakeupSeconds, receivedConfig.nightTimeModeTryDataTransWakeupSeconds); }
                currentConfig->nightTimeModeTryDataTransWakeupSeconds = receivedConfig.nightTimeModeTryDataTransWakeupSeconds;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_NIGHTTIME_DATATRANS_WAKEUP_SECONDS, receivedConfig.nightTimeModeTryDataTransWakeupSeconds)) { noError = false; }
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
        if(receivedConfig.dataTransBattMinVoltage != currentConfig->dataTransBattMinVoltage) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing dataTransBattMinVoltage %d -> %d\n", currentConfig->dataTransBattMinVoltage, receivedConfig.dataTransBattMinVoltage); }
                currentConfig->dataTransBattMinVoltage = receivedConfig.dataTransBattMinVoltage;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_DATATRANS_MIN_BATT_VOLTAGE, receivedConfig.dataTransBattMinVoltage)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.dataTransBattMinVoltageDuringTrans != currentConfig->dataTransBattMinVoltageDuringTrans) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing dataTransBattMinVoltageDuringTrans %d -> %d\n", currentConfig->dataTransBattMinVoltageDuringTrans, receivedConfig.dataTransBattMinVoltageDuringTrans); }
                currentConfig->dataTransBattMinVoltageDuringTrans = receivedConfig.dataTransBattMinVoltageDuringTrans;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_DATATRANS_MIN_BATT_VOLTAGE_DURING_TR, receivedConfig.dataTransBattMinVoltageDuringTrans)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.skipGetTime != currentConfig->skipGetTime) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing skipGetTime %d -> %d\n", currentConfig->skipGetTime, receivedConfig.skipGetTime); }
                currentConfig->skipGetTime = receivedConfig.skipGetTime;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_SKIP_GET_TIME, receivedConfig.skipGetTime)) { noError = false; }
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
        if(receivedConfig.activationMode != currentConfig->activationMode) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activationMode %d -> %d\n", currentConfig->activationMode, receivedConfig.activationMode); }
                currentConfig->activationMode = receivedConfig.activationMode;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ACTIVATION_MODE, receivedConfig.activationMode)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.activationSource != currentConfig->activationSource) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activationSource %d -> %d\n", currentConfig->activationSource, receivedConfig.activationSource); }
                currentConfig->activationSource = receivedConfig.activationSource;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ACTIVATION_SOURCE, receivedConfig.activationSource)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.activationByEspNowRetrySeconds != currentConfig->activationByEspNowRetrySeconds) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activationByEspNowRetrySeconds %d -> %d\n", currentConfig->activationByEspNowRetrySeconds, receivedConfig.activationByEspNowRetrySeconds); }
                currentConfig->activationByEspNowRetrySeconds = receivedConfig.activationByEspNowRetrySeconds;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_ACTIVATION_BY_ESPNOW_RETRY_TIME_SECS, receivedConfig.activationByEspNowRetrySeconds)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.dataTransTryEveryFullMinSeldomly != currentConfig->dataTransTryEveryFullMinSeldomly) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing dataTransTryEveryFullMinSeldomly %d -> %d\n", currentConfig->dataTransTryEveryFullMinSeldomly, receivedConfig.dataTransTryEveryFullMinSeldomly); }
                currentConfig->dataTransTryEveryFullMinSeldomly = receivedConfig.dataTransTryEveryFullMinSeldomly;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_DATATRANSM_TRY_EVERY_FULL_MINUTE_SELD, receivedConfig.dataTransTryEveryFullMinSeldomly)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.memFullTryEveryFullMinSeldomly != currentConfig->memFullTryEveryFullMinSeldomly) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing memFullTryEveryFullMinSeldomly %d -> %d\n", currentConfig->memFullTryEveryFullMinSeldomly, receivedConfig.memFullTryEveryFullMinSeldomly); }
                currentConfig->memFullTryEveryFullMinSeldomly = receivedConfig.memFullTryEveryFullMinSeldomly;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY, receivedConfig.memFullTryEveryFullMinSeldomly)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.accInterruptWatermark != currentConfig->accInterruptWatermark) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing accInterruptWatermark %d -> %d\n", currentConfig->accInterruptWatermark, receivedConfig.accInterruptWatermark); }
                currentConfig->accInterruptWatermark = receivedConfig.accInterruptWatermark;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_ACC_INTERRUPT_WATERMARK, receivedConfig.accInterruptWatermark)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.dataTransOutputPower != currentConfig->dataTransOutputPower) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing dataTransOutputPower %d -> %d\n", currentConfig->dataTransOutputPower, receivedConfig.dataTransOutputPower); }
                currentConfig->dataTransOutputPower = receivedConfig.dataTransOutputPower;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_DATATRANS_OUTPUT_POWER, receivedConfig.dataTransOutputPower)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.espNowCustomRFCalibration != currentConfig->espNowCustomRFCalibration) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing espNowCustomRFCalibration %d -> %d\n", currentConfig->espNowCustomRFCalibration, receivedConfig.espNowCustomRFCalibration); }
                currentConfig->espNowCustomRFCalibration = receivedConfig.espNowCustomRFCalibration;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ESPNOW_CUSTOM_RF_CALIBRATION, receivedConfig.espNowCustomRFCalibration)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.commandByteForceTrackingDurationSeconds != currentConfig->commandByteForceTrackingDurationSeconds) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing commandByteForceTrackingDurationSeconds %d -> %d\n", currentConfig->commandByteForceTrackingDurationSeconds, receivedConfig.commandByteForceTrackingDurationSeconds); }
                currentConfig->commandByteForceTrackingDurationSeconds = receivedConfig.commandByteForceTrackingDurationSeconds;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_COMMAND_BYTE_FORCE_TR_DURATION_SECS, receivedConfig.commandByteForceTrackingDurationSeconds)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.espNowDataRate != currentConfig->espNowDataRate) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing espNowDataRate %d -> %d\n", currentConfig->espNowDataRate, receivedConfig.espNowDataRate); }
                currentConfig->espNowDataRate = receivedConfig.espNowDataRate;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ESPNOW_DATA_RATE, receivedConfig.espNowDataRate)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.espNowLongRange != currentConfig->espNowLongRange) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing espNowLongRange %d -> %d\n", currentConfig->espNowLongRange, receivedConfig.espNowLongRange); }
                currentConfig->espNowLongRange = receivedConfig.espNowLongRange;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ESPNOW_LONG_RANGE, receivedConfig.espNowLongRange)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.espNowMinBytesToTransmit != currentConfig->espNowMinBytesToTransmit) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing espNowMinBytesToTransmit %08X -> %08X\n", currentConfig->espNowMinBytesToTransmit, receivedConfig.espNowMinBytesToTransmit); }
                currentConfig->espNowMinBytesToTransmit = receivedConfig.espNowMinBytesToTransmit;
                if(!device->defaultNvsWriteUINT32(NVS_CONF_ESPNOW_MIN_BYTES_TO_TRANSMIT, receivedConfig.espNowMinBytesToTransmit)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.espNowMaxBytesToTransmit != currentConfig->espNowMaxBytesToTransmit) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing espNowMaxBytesToTransmit %08X -> %08X\n", currentConfig->espNowMaxBytesToTransmit, receivedConfig.espNowMaxBytesToTransmit); }
                currentConfig->espNowMaxBytesToTransmit = receivedConfig.espNowMaxBytesToTransmit;
                if(!device->defaultNvsWriteUINT32(NVS_CONF_ESPNOW_MAX_BYTES_TO_TRANSMIT, receivedConfig.espNowMaxBytesToTransmit)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.wifiMinBlocksToTransmit != currentConfig->wifiMinBlocksToTransmit) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing wifiMinBlocksToTransmit %d -> %d\n", currentConfig->wifiMinBlocksToTransmit, receivedConfig.wifiMinBlocksToTransmit); }
                currentConfig->wifiMinBlocksToTransmit = receivedConfig.wifiMinBlocksToTransmit;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_WIFI_MIN_BLOCKS_TO_TRANSMIT, receivedConfig.wifiMinBlocksToTransmit)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.wifiMaxBlocksToTransmit != currentConfig->wifiMaxBlocksToTransmit) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing wifiMaxBlocksToTransmit %d -> %d\n", currentConfig->wifiMaxBlocksToTransmit, receivedConfig.wifiMaxBlocksToTransmit); }
                currentConfig->wifiMaxBlocksToTransmit = receivedConfig.wifiMaxBlocksToTransmit;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_WIFI_MAX_BLOCKS_TO_TRANSMIT, receivedConfig.wifiMaxBlocksToTransmit)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.activityActivationEnabled != currentConfig->activityActivationEnabled) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activityActivationEnabled %d -> %d\n", currentConfig->activityActivationEnabled, receivedConfig.activityActivationEnabled); }
                currentConfig->activityActivationEnabled = receivedConfig.activityActivationEnabled;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ACTIVITY_ACTIVATION_ENABLED, receivedConfig.activityActivationEnabled)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.activityThresholdActiveToInactiveAvg != currentConfig->activityThresholdActiveToInactiveAvg) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activityThresholdActiveToInactiveAvg %d -> %d\n", currentConfig->activityThresholdActiveToInactiveAvg, receivedConfig.activityThresholdActiveToInactiveAvg); }
                currentConfig->activityThresholdActiveToInactiveAvg = receivedConfig.activityThresholdActiveToInactiveAvg;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_ACTIVITY_THRESHOLD_ACTIVE_TO_INACT_AVG, receivedConfig.activityThresholdActiveToInactiveAvg)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.activityThresholdInactiveToActiveThr != currentConfig->activityThresholdInactiveToActiveThr) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activityThresholdInactiveToActiveThr %d -> %d\n", currentConfig->activityThresholdInactiveToActiveThr, receivedConfig.activityThresholdInactiveToActiveThr); }
                currentConfig->activityThresholdInactiveToActiveThr = receivedConfig.activityThresholdInactiveToActiveThr;
                if(!device->defaultNvsWriteUINT8(NVS_CONF_ACTIVITY_THRESHOLD_INACTIVE_TO_ACT_THR, receivedConfig.activityThresholdInactiveToActiveThr)) { noError = false; }
                else { changeCnt++; }
            }
            else { noError = false; }
        }
        if(receivedConfig.activityTransmissionInterval != currentConfig->activityTransmissionInterval) {
            if(device->initNVS()) {
                if(debug) { printf("RECONF: changing activityTransmissionInterval %d -> %d\n", currentConfig->activityTransmissionInterval, receivedConfig.activityTransmissionInterval); }
                currentConfig->activityTransmissionInterval = receivedConfig.activityTransmissionInterval;
                if(!device->defaultNvsWriteUINT16(NVS_CONF_ACTIVITY_TRANSMISSION_INTERVAL, receivedConfig.activityTransmissionInterval)) { noError = false; }
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

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_DO_THE_BLINK, &neverWritten);
    if(neverWritten) { config->doTheBlink = DO_THE_BLINK; }
    else { config->doTheBlink = (temp8 > 0); }
    if(printConfig) {
        printf(": doTheBlink: %d", config->doTheBlink);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_TRACKER_MODE, &neverWritten);
    if(neverWritten) { config->trackerMode = TRACKER_MODE; }
    else { config->trackerMode = temp8; }
    if(printConfig) {
        printf(": trackerMode: %d", config->trackerMode);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_TRACKING_DATA_MODE, &neverWritten);
    if(neverWritten) { config->trackingDataMode = TRACKING_DATA_MODE; }
    else { config->trackingDataMode = temp8; }
    if(printConfig) {
        printf(": trackingDataMode: %d", config->trackingDataMode);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_TRANSMISSION_METHOD, &neverWritten);
    if(neverWritten) { config->transmissionMethod = TRANSMISSION_METHOD; }
    else { config->transmissionMethod = temp8; }
    if(printConfig) {
        printf(": transmissionMethod: %d", config->transmissionMethod);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** DEAD RECKONING SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_DR_MIN_HDOP_X_TEN, &neverWritten);
    if(neverWritten) { config->drMinHdopXTen = TRACKING_DATA_MODE_DR_MIN_HDOP_X_TEN; }
    else { config->drMinHdopXTen = temp8; }
    if(printConfig) {
        printf(": drMinHdopXTen: %d", config->drMinHdopXTen);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_DR_IMU_SECONDS, &neverWritten);
    if(neverWritten) { config->drImuSeconds = TRACKING_DATA_MODE_DR_IMU_SECONDS; }
    else { config->drImuSeconds = temp16; }
    if(printConfig) {
        printf(": drImuSeconds: %d", config->drImuSeconds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** 1HZ GPS AND IMU SETTINGS **\n");

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_1HZ_INTERRUPT_AFTER_SECONDS, &neverWritten);
    if(neverWritten) { config->gps1HzInterruptAfterSeconds = TRACKING_DATA_MODE_1HZ_INTERRUPT_AFTER_SECONDS; }
    else { config->gps1HzInterruptAfterSeconds = temp16; }
    if(printConfig) {
        printf(": gps1HzInterruptAfterSeconds: %d", config->gps1HzInterruptAfterSeconds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** IMU SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACC_FREQUENCY, &neverWritten);
    if(neverWritten) { config->accFrequency = ACC_FREQUENCY; }
    else { config->accFrequency = temp8; }
    if(printConfig) {
        printf(": accFrequency: %d", config->accFrequency);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACC_AVG, &neverWritten);
    if(neverWritten) { config->accAvg = ACC_AVG; }
    else { config->accAvg = temp8; }
    if(printConfig) {
        printf(": accAvg: %d", config->accAvg);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACC_RANGE, &neverWritten);
    if(neverWritten) { config->accRange = ACC_RANGE; }
    else { config->accRange = temp8; }
    if(printConfig) {
        printf(": accRange: %d", config->accRange);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_MAG_FREQUENCY, &neverWritten);
    if(neverWritten) { config->magFrequency = MAG_FREQUENCY; }
    else { config->magFrequency = temp8; }
    if(printConfig) {
        printf(": magFrequency: %d", config->magFrequency);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_MAG_ACCURACY, &neverWritten);
    if(neverWritten) { config->magAccuracy = MAG_ACCURACY; }
    else { config->magAccuracy = temp8; }
    if(printConfig) {
        printf(": magAccuracy: %d", config->magAccuracy);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GYRO_FREQUENCY, &neverWritten);
    if(neverWritten) { config->gyroFrequency = GYRO_FREQUENCY; }
    else { config->gyroFrequency = temp8; }
    if(printConfig) {
        printf(": gyroFrequency: %d", config->gyroFrequency);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GYRO_RANGE, &neverWritten);
    if(neverWritten) { config->gyroRange = GYRO_RANGE; }
    else { config->gyroRange = temp8; }
    if(printConfig) {
        printf(": gyroRange: %d", config->gyroRange);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_GYRO_MODE, &neverWritten);
    if(neverWritten) { config->gyroMode = GYRO_MODE; }
    else { config->gyroMode = temp8; }
    if(printConfig) {
        printf(": gyroMode: %d", config->gyroMode);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_USE_MAG, &neverWritten);
    if(neverWritten) { config->useMagnetometer = USE_MAGNETOMETER; }
    else { config->useMagnetometer = (temp8 > 0); }
    if(printConfig) {
        printf(": useMagnetometer: %d", config->useMagnetometer);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_USE_GYRO, &neverWritten);
    if(neverWritten) { config->useGyro = USE_GYRO; }
    else { config->useGyro = (temp8 > 0); }
    if(printConfig) {
        printf(": useGyro: %d", config->useGyro);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_ACC_INTERRUPT_WATERMARK, &neverWritten);
    if(neverWritten) { config->accInterruptWatermark = ACC_INTERRUPT_WATERMARK; }
    else { config->accInterruptWatermark = temp16; }
    if(printConfig) {
        printf(": accInterruptWatermark: %d", config->accInterruptWatermark);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** NIGHT SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHT_TIME_ENTER, &neverWritten);
    if(neverWritten) { config->nightTimeEnter = NIGHTTIME_ENTER; }
    else { config->nightTimeEnter = temp8; }
    if(printConfig) {
        printf(": nightTimeEnter: %d", config->nightTimeEnter);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHT_TIME_MODE, &neverWritten);
    if(neverWritten) { config->nightTimeMode = NIGHTTIME_MODE; }
    else { config->nightTimeMode = temp8; }
    if(printConfig) {
        printf(": nightTimeMode: %d", config->nightTimeMode);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHTTIME_TURN_ON_HOUR, &neverWritten);
    if(neverWritten) { config->nightTimeTurnOnHour = NIGHTTIME_TURN_ON_HOUR; }
    else { config->nightTimeTurnOnHour = temp8; }
    if(printConfig) {
        printf(": nightTimeTurnOnHour: %d", config->nightTimeTurnOnHour);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHTTIME_TURN_ON_MINUTE, &neverWritten);
    if(neverWritten) { config->nightTimeTurnOnMinute = NIGHTTIME_TURN_ON_MINUTE; }
    else { config->nightTimeTurnOnMinute = temp8; }
    if(printConfig) {
        printf(": nightTimeTurnOnMinute: %d", config->nightTimeTurnOnMinute);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHTTIME_TURN_OFF_HOUR, &neverWritten);
    if(neverWritten) { config->nightTimeTurnOffHour = NIGHTTIME_TURN_OFF_HOUR; }
    else { config->nightTimeTurnOffHour = temp8; }
    if(printConfig) {
        printf(": nightTimeTurnOffHour: %d", config->nightTimeTurnOffHour);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_NIGHTTIME_TURN_OFF_MINUTE, &neverWritten);
    if(neverWritten) { config->nightTimeTurnOffMinute = NIGHTTIME_TURN_OFF_MINUTE; }
    else { config->nightTimeTurnOffMinute = temp8; }
    if(printConfig) {
        printf(": nightTimeTurnOffMinute: %d", config->nightTimeTurnOffMinute);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp32 = device->defaultNvsReadUINT32(NVS_CONF_NIGHTTIME_DATA_TRANS_DEEP_NIGHT_HOURS, &neverWritten);
    if(neverWritten) { config->nightTimeDataTransDeepestNightHours = NIGHT_TIME_DATA_TRANS_DEEP_NIGHT_HOURS; }
    else { config->nightTimeDataTransDeepestNightHours = temp32; }
    if(printConfig) {
        printf(": nightTimeDataTransDeepestNightHours: %d", config->nightTimeDataTransDeepestNightHours);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_NIGHTTIME_DATATRANS_WAKEUP_SECONDS, &neverWritten);
    if(neverWritten) { config->nightTimeModeTryDataTransWakeupSeconds = NIGHTTIME_MODE_TRY_DATATRANS_WAKEUP_SECONDS; }
    else { config->nightTimeModeTryDataTransWakeupSeconds = temp16; }
    if(printConfig) {
        printf(": nightTimeModeTryDataTransWakeupSeconds: %d", config->nightTimeModeTryDataTransWakeupSeconds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** BATTERY SETTINGS **\n");
    
    temp16 = device->defaultNvsReadUINT16(NVS_CONF_BATT_MIN_VOLTAGE, &neverWritten);
    if(neverWritten) { config->battMinVoltage = BATT_MIN_VOLTAGE; }
    else { config->battMinVoltage = temp16; }
    if(printConfig) {
        printf(": battMinVoltage: %d", config->battMinVoltage);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_BATT_RESTART_VOLTAGE, &neverWritten);
    if(neverWritten) { config->battRestartVoltage = BATT_RESTART_VOLTAGE; }
    else { config->battRestartVoltage = temp16; }
    if(printConfig) {
        printf(": battRestartVoltage: %d", config->battRestartVoltage);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_DATATRANS_MIN_BATT_VOLTAGE, &neverWritten);
    if(neverWritten) { config->dataTransBattMinVoltage = DATATRANS_MIN_BATT_VOLTAGE; }
    else { config->dataTransBattMinVoltage = temp16; }
    if(printConfig) {
        printf(": dataTransBattMinVoltage: %d", config->dataTransBattMinVoltage);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_DATATRANS_MIN_BATT_VOLTAGE_DURING_TR, &neverWritten);
    if(neverWritten) { config->dataTransBattMinVoltageDuringTrans = DATATRANS_MIN_BATT_VOLTAGE_DURING_TRANSM; }
    else { config->dataTransBattMinVoltageDuringTrans = temp16; }
    if(printConfig) {
        printf(": dataTransBattMinVoltageDuringTrans: %d", config->dataTransBattMinVoltageDuringTrans);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** GETTING TIME SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_SKIP_GET_TIME, &neverWritten);
    if(neverWritten) { config->skipGetTime = SKIP_GET_TIME; }
    else { config->skipGetTime = (temp8 > 0); }
    if(printConfig) {
        printf(": skipGetTime: %d", config->skipGetTime);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_TIME_BETWEEN_GET_TIME_RETRIES_SECONDS, &neverWritten);
    if(neverWritten) { config->timeBetweenGetTimeRetriesSeconds = TIME_BETWEEN_GET_TIME_RETRIES_SECONDS; }
    else { config->timeBetweenGetTimeRetriesSeconds = temp16; }
    if(printConfig) {
        printf(": timeBetweenGetTimeRetriesSeconds: %d", config->timeBetweenGetTimeRetriesSeconds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** ACTIVATION SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACTIVATION_MODE, &neverWritten);
    if(neverWritten) { config->activationMode = ACTIVATION_MODE; }
    else { config->activationMode = temp8; }
    if(printConfig) {
        printf(": activationMode: %d", config->activationMode);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACTIVATION_SOURCE, &neverWritten);
    if(neverWritten) { config->activationSource = ACTIVATION_SOURCE; }
    else { config->activationSource = temp8; }
    if(printConfig) {
        printf(": activationSource: %d", config->activationSource);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_ACTIVATION_BY_ESPNOW_RETRY_TIME_SECS, &neverWritten);
    if(neverWritten) { config->activationByEspNowRetrySeconds = ACTIVATION_BY_ESPNOW_RETRY_TIME_SECONDS; }
    else { config->activationByEspNowRetrySeconds = temp16; }
    if(printConfig) {
        printf(": activationByEspNowRetrySeconds: %d", config->activationByEspNowRetrySeconds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** DATA TRANSMISSION SETTINGS **\n");

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_DATATRANSM_TRY_EVERY_FULL_MINUTE_SELD, &neverWritten);
    if(neverWritten) { config->dataTransTryEveryFullMinSeldomly = DATATRANSM_TRY_EVERY_FULL_MINUTE_SELDOMLY; }
    else { config->dataTransTryEveryFullMinSeldomly = temp16; }
    if(printConfig) {
        printf(": dataTransTryEveryFullMinSeldomly: %d", config->dataTransTryEveryFullMinSeldomly);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_DATATRANS_OUTPUT_POWER, &neverWritten);
    if(neverWritten) { config->dataTransOutputPower = DATATRANS_OUTPUT_POWER; }
    else { config->dataTransOutputPower = temp8; }
    if(printConfig) {
        printf(": dataTransOutputPower: %d", config->dataTransOutputPower);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ESPNOW_CUSTOM_RF_CALIBRATION, &neverWritten);
    if(neverWritten) { config->espNowCustomRFCalibration = ESPNOW_CUSTOM_RF_CALIBRATION; }
    else { config->espNowCustomRFCalibration = (temp8 > 0); }
    if(printConfig) {
        printf(": espNowCustomRFCalibration: %d", config->espNowCustomRFCalibration);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_COMMAND_BYTE_FORCE_TR_DURATION_SECS, &neverWritten);
    if(neverWritten) { config->commandByteForceTrackingDurationSeconds = COMMAND_BYTE_FORCE_TRACKING_DURATION_SECONDS; }
    else { config->commandByteForceTrackingDurationSeconds = temp16; }
    if(printConfig) {
        printf(": commandByteForceTrackingDurationSeconds: %d", config->commandByteForceTrackingDurationSeconds);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ESPNOW_DATA_RATE, &neverWritten);
    if(neverWritten) { config->espNowDataRate = ESPNOW_DATA_RATE; }
    else { config->espNowDataRate = temp8; }
    if(printConfig) {
        printf(": espNowDataRate: %d", config->espNowDataRate);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ESPNOW_LONG_RANGE, &neverWritten);
    if(neverWritten) { config->espNowLongRange = ESPNOW_LONG_RANGE; }
    else { config->espNowLongRange = (temp8 > 0); }
    if(printConfig) {
        printf(": espNowLongRange: %d", config->espNowLongRange);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp32 = device->defaultNvsReadUINT32(NVS_CONF_ESPNOW_MIN_BYTES_TO_TRANSMIT, &neverWritten);
    if(neverWritten) { config->espNowMinBytesToTransmit = ESPNOW_MIN_BYTES_TO_TRANSMIT; }
    else { config->espNowMinBytesToTransmit = temp32; }
    if(printConfig) {
        printf(": espNowMinBytesToTransmit: %d", config->espNowMinBytesToTransmit);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp32 = device->defaultNvsReadUINT32(NVS_CONF_ESPNOW_MAX_BYTES_TO_TRANSMIT, &neverWritten);
    if(neverWritten) { config->espNowMaxBytesToTransmit = ESPNOW_MAX_BYTES_TO_TRANSMIT; }
    else { config->espNowMaxBytesToTransmit = temp32; }
    if(printConfig) {
        printf(": espNowMaxBytesToTransmit: %d", config->espNowMaxBytesToTransmit);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_WIFI_MIN_BLOCKS_TO_TRANSMIT, &neverWritten);
    if(neverWritten) { config->wifiMinBlocksToTransmit = WIFI_MIN_BLOCKS_TO_TRANSMIT; }
    else { config->wifiMinBlocksToTransmit = temp16; }
    if(printConfig) {
        printf(": wifiMinBlocksToTransmit: %d", config->wifiMinBlocksToTransmit);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_WIFI_MAX_BLOCKS_TO_TRANSMIT, &neverWritten);
    if(neverWritten) { config->wifiMaxBlocksToTransmit = WIFI_MAX_BLOCKS_TO_TRANSMIT; }
    else { config->wifiMaxBlocksToTransmit = temp16; }
    if(printConfig) {
        printf(": wifiMaxBlocksToTransmit: %d", config->wifiMaxBlocksToTransmit);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** MEMORY FULL SETTINGS **\n");

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY, &neverWritten);
    if(neverWritten) { config->memFullTryEveryFullMinSeldomly = ST_MEMFULL_TRY_EVERY_FULL_MINUTE_SELDOMLY; }
    else { config->memFullTryEveryFullMinSeldomly = temp16; }
    if(printConfig) {
        printf(": memFullTryEveryFullMinSeldomly: %d", config->memFullTryEveryFullMinSeldomly);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    printf("** ACTIVITY ACTIVATION SETTINGS **\n");

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACTIVITY_ACTIVATION_ENABLED, &neverWritten);
    if(neverWritten) { config->activityActivationEnabled = ACTIVITY_ACTIVATION_ENABLED; }
    else { config->activityActivationEnabled = (temp8 > 0); }
    if(printConfig) {
        printf(": activityActivationEnabled: %d", config->activityActivationEnabled);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_ACTIVITY_THRESHOLD_ACTIVE_TO_INACT_AVG, &neverWritten);
    if(neverWritten) { config->activityThresholdActiveToInactiveAvg = ACTIVITY_THRESHOLD_ACTIVE_TO_INACTIVE_AVG; }
    else { config->activityThresholdActiveToInactiveAvg = temp16; }
    if(printConfig) {
        printf(": activityThresholdActiveToInactiveAvg: %d", config->activityThresholdActiveToInactiveAvg);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp8 = device->defaultNvsReadUINT8(NVS_CONF_ACTIVITY_THRESHOLD_INACTIVE_TO_ACT_THR, &neverWritten);
    if(neverWritten) { config->activityThresholdInactiveToActiveThr = ACTIVITY_THRESHOLD_INACTIVE_TO_ACTIVE_THR; }
    else { config->activityThresholdInactiveToActiveThr = temp8; }
    if(printConfig) {
        printf(": activityThresholdInactiveToActiveThr: %d", config->activityThresholdInactiveToActiveThr);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    temp16 = device->defaultNvsReadUINT16(NVS_CONF_ACTIVITY_TRANSMISSION_INTERVAL, &neverWritten);
    if(neverWritten) { config->activityTransmissionInterval = ACTIVITY_TRANSMISSION_INTERVAL; }
    else { config->activityTransmissionInterval = temp16; }
    if(printConfig) {
        printf(": activityTransmissionInterval: %d", config->activityTransmissionInterval);
        if(neverWritten) { printf(" (DEFAULT)\n"); }
        else { printf("\n"); nonDefaultCounter++; }
    }

    if(printConfig) {
        printf("** NON-DEFAULT SETTINGS: %d **\n", nonDefaultCounter);
    }
    return true;
}