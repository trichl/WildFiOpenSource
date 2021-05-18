#include "PlatformWildFiTagREV5.h"
#include <math.h>

WildFiTagREV5 device = WildFiTagREV5();

#define MAG_ITERATIONS                                  1
#define MAG_SAMPLE_RATE_SECONDS                         15                  
#define MAG_START_SAVING_AFTER_SAMPLES_COUNT            17280ULL          // every 15 seconds -> 24h = 86400s / 15 = 5760ULL, 72h = 259200s / 15 = 17280ULL
#define STORE_DATA                                      true

#define RESET_SPIFFS_AT_START                           false

#define MAG_CALIBRATION_AFTER_START                     false
#define MAG_CALIBRATION_TIME_SECONDS                    20

// BMX160_MAG_ACCURACY_LOW_POWER                        205uA -> new after bugfix: 244uA
// BMX160_MAG_ACCURACY_REGULAR                          505uA -> new after bugfix: 655uA
// BMX160_MAG_ACCURACY_ENHANCED                         811uA -> new after bugfix: 1.09mA
// BMX160_MAG_ACCURACY_HIGH                             2300uA -> new after bugfix: 3.17mA

mag_config_t magConf =                                  { BMX160_MAG_ODR_12_5HZ, BMX160_MAG_ACCURACY_REGULAR };

const uint8_t WIFI_LIST_SIZE =                          1;
const char* WIFI_SSIDS[WIFI_LIST_SIZE] =                { "RodelbahnSoelden" };        
const char* WIFI_PASSWORDS[WIFI_LIST_SIZE] =            { "xxxxxx" };

const char* SPIFFS_FILE =                               "/spiffs/mag.txt";

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint64_t bootCnt = 0;
RTC_DATA_ATTR float averageMagnitudeSum = 0;
RTC_DATA_ATTR uint64_t sampleCnt = 0;

RTC_DATA_ATTR bmm150_trim_registers trimData = {};
RTC_DATA_ATTR mag_calibration_t magCalibration = {};
struct BMX160Data mag;
uint16_t hall = 0;

// Wiki: in Mitteleuropa sind es etwa 48 µT, nämlich etwa 20 µT in der horizontalen und 44 µT in der vertikalen Richtung.
// TODO: test multiple samples with new algorithm
// TODO: store offset calibration function data in NVS
// TODO: offset calibration wifi (when seen -> enter calibration mode)

float calculateMagnitude(int16_t xInt, int16_t yInt, int16_t zInt) {
    float x = xInt;
    float y = yInt;
    float z = zInt;
    float magnitude = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    return magnitude;
}

bool getTimeOverWiFi() {
    bool success = false;
    uint8_t foundArrayId = 0;
    uint8_t foundOnChannel = 0;

    if(device.initWiFi()) {
        device.ledRedOn(); // visual feedback during activation scan
        printf("getTimeOverWiFi: done init!\n");
        if(device.scanForWiFisOn1and6and11(WIFI_SSIDS, WIFI_LIST_SIZE, &foundArrayId, &foundOnChannel, RADIO_MAX_TX_11_DBM, 120, 1500)) {
            printf("getTimeOverWiFi: done scan!\n");
            if(foundOnChannel > 0) {
                // TODO: if special wifi seen -> calibration mode!
                printf("getTimeOverWiFi: found wifi!\n");
                if(device.connectToWiFiAfterScan(WIFI_SSIDS[foundArrayId], WIFI_PASSWORDS[foundArrayId], foundOnChannel)) {
                    printf("getTimeOverWiFi: connecting!\n");
                    uint32_t connectStartedTime = ((uint32_t) Timing::millis());
                    bool connectionTimeout = false;
                    while(device.connectedToWiFi() == WIFI_CONNECT_RUNNING) {
                        device.delay(20);
                        if(((uint32_t) Timing::millis()) - connectStartedTime > (12 * 1000)) { // e.g. password wrong
                            printf("getTimeOverWiFi: timeout!\n");
                            connectionTimeout = true;
                            break;
                        }
                    }
                    if(connectionTimeout || (device.connectedToWiFi() == WIFI_CONNECT_FAIL_AP_NOT_FOUND) || (device.connectedToWiFi() == WIFI_CONNECT_NEVER_STARTED)) { // should not happen because wifi already seen in scan
                        printf("getTimeOverWiFi: connect failed!\n");
                    }
                    else { // connected to wifi
                    printf("getTimeOverWiFi: connected!\n");
                        uint32_t timestampUTC = 0;
                        uint16_t millisecondsUTC = 0;
                        if(!device.getNTPTimestampUTC(true, timestampUTC, millisecondsUTC, 12000, "pool.ntp.org")) { // will block, will set RTC time
                            printf("getTimeOverWiFi: get time failed!\n");
                        }
                        else {
                            printf("getTimeOverWiFi: got time: %d!\n", timestampUTC);
                            success = true;
                        }
                    }
                }
            }
        }
        device.ledRedOff();
    }
    device.disconnectAndStopWiFi();
    return success;
}

bool printFreeSpace() {
    uint32_t total = 0, used = 0;
    if(esp_spiffs_info(NULL, &total, &used) != ESP_OK) {
        return false;
    }
    printf("SPIFFS total: %d, used: %d, free: %d\n", total, used, (total - used));
    return true;
}

extern "C" void app_main() {
    while(1) {
        device.initSpiffs(1); // might perform format for 11 seconds
        if(firstStart) {
            bool someError = false;
            device.disableWakeStubNoBootIfVoltageLow();
            
            device.delay(6000);
            //device.shortLightSleep(4000);
            
            if(RESET_SPIFFS_AT_START) {
                device.resetSpiffs(SPIFFS_FILE);
            }
            printFreeSpace();
            if(!device.printSpiffs(SPIFFS_FILE)) { printf("Spiffs: Nothing to read!\n"); }

            firstStart = false;

            i2c.begin(I2C_FREQ_HZ_400KHZ);

            // init rtc
            if(!device.rtc.disableClockOut()) { printf("ERROR disable clock\n"); someError = true; }

            if(getTimeOverWiFi()) {
                device.blinkTimes(5, B_BOTH);
                device.sensorPowerOn(false);
                device.keepSensorPowerOnInDeepSleep();
                device.delay(100); // IMPORTANT: wait some time before starting sensor

                // init mag
                if(!device.imu.magCompensateReadTrimData(&trimData)) { printf("ERROR READ TRIM REGISTERS\n"); someError = true; }
                device.imu.magCompensatePrintTrimData(&trimData);

                if(MAG_CALIBRATION_AFTER_START) {
                    device.measureTime("Start");
                    if(!device.magnetometerCalibrationMode(MAG_CALIBRATION_TIME_SECONDS, &magCalibration, &trimData)) { printf("ERROR MAG calib\n"); someError = true; }
                    device.measureTime("magnetometerCalibrationMode");
                    printf("Result Calib: xmin %d, xmax %d, ymin %d, ymax %d, zmin %d, zmax %d\n", magCalibration.xMin, magCalibration.xMax, magCalibration.yMin, magCalibration.yMax, magCalibration.zMin, magCalibration.zMax);
                }
                else {
                    if(!device.imu.start(NULL, &magConf, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("ERROR IMU Start\n"); someError = true; }
	                device.delay(500); // IMPORTANT: otherwise reading schmu data
                }
                device.delay(100);
            }
            else { printf("ERROR WIFI Time\n"); someError = true; }

            if(someError) {
                device.blinkTimes(20, B_RED);
                printf("ERROR - eternal sleep!\n");
            }
            else {
                device.rtc.setRegularInterrupt(MAG_SAMPLE_RATE_SECONDS);
                device.enableRTCInterruptInDeepSleep();
            }
        }
        else {
            int16_t temperatureBmx = 0, temperature = 0;
            bool error = false;
            i2c.begin(I2C_FREQ_HZ_400KHZ);
            uint64_t cnt;

            sampleCnt++;
            if(sampleCnt > MAG_START_SAVING_AFTER_SAMPLES_COUNT) {
                // get timestamp
                uint32_t timestamp = device.rtc.getTimestamp(error);

                // get data: temperature BMX
                device.imu.getTemperature(temperatureBmx);
                temperatureBmx = device.imu.toCelsiusx100(temperatureBmx);

                // get data: temperature baro
                if(device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) {
                    if(device.baro.performMeasurement()) {
                        while(!device.baro.measurementFinished()) { ; }
                        if(device.baro.getResults()) {
                            temperature = device.baro.getTemperature(error);
                        }
                        else { printf("Baro err 03\n"); }
                    }
                    else { printf("Baro err 02\n"); }
                }
                else { printf("Baro err 01\n"); }

                //printf("counter,timestampUTC,temperatureOnIMU,temperatureOnCHIP,xUncompensatedInUT,yUncompensatedInUT,zUncompensatedInUT,xCompensatedInUT,yCompensatedInUT,zCompensatedInUT\n");

                for(uint8_t i=0; i<MAG_ITERATIONS; i++) {
                    cnt = 0;

                    while(!device.imu.magDataReady()) { cnt++; device.delay(10); }
                    if(!device.imu.getData(NULL, &mag, NULL, &hall)) { printf("ACC: error2\n"); }

                    int16_t xUncomp = device.imu.magXConvertToMicroTesla(mag.x, &trimData);
                    int16_t yUncomp = device.imu.magYConvertToMicroTesla(mag.y, &trimData);
                    int16_t zUncomp = device.imu.magZConvertToMicroTesla(mag.z, &trimData);

                    int16_t xComp = device.imu.magCompensateXandConvertToMicroTesla(mag.x, hall, &trimData);
                    int16_t yComp = device.imu.magCompensateYandConvertToMicroTesla(mag.y, hall, &trimData);
                    int16_t zComp = device.imu.magCompensateZandConvertToMicroTesla(mag.z, hall, &trimData);

                    float magnitudeNotCompensated = calculateMagnitude(xUncomp, yUncomp, zUncomp);
                    float magnitudeCompensated = calculateMagnitude(xComp, yComp, zComp);
                    averageMagnitudeSum += magnitudeCompensated;
                    
                    // store data
                    #if STORE_DATA
                        if(device.getFreeSpaceSpiffs() > 1000) {
                            FILE* f = fopen(SPIFFS_FILE, "a"); // a = append, w = (over)write
                            if(f == NULL) {
                                printf("Failed to open file for writing\n");
                            }
                            else {
                                fprintf(f, "%llu,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", bootCnt - MAG_START_SAVING_AFTER_SAMPLES_COUNT, timestamp, temperatureBmx, temperature, xUncomp, yUncomp, zUncomp, xComp, yComp, zComp);
                                printf("Stored: %llu,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", bootCnt - MAG_START_SAVING_AFTER_SAMPLES_COUNT, timestamp, temperatureBmx, temperature, xUncomp, yUncomp, zUncomp, xComp, yComp, zComp);
                            }
                            fclose(f);
                        }
                    #else
                        printf("Unstored: %llu,%d,%d,%d,%d,\n%d,%d,%d,\n%d,%d,%d\n", bootCnt, timestamp, temperatureBmx, temperature, hall, xUncomp, yUncomp, zUncomp, xComp, yComp, zComp);
                    #endif
                    
                    //printf("%.2f,%.2f\n\n", magnitudeCompensated, magnitudeNotCompensated);

                    /*int16_t xOffset = (magCalibration.xMin + magCalibration.xMax) / 2;
                    int16_t yOffset = (magCalibration.yMin + magCalibration.yMax) / 2;
                    int16_t zOffset = (magCalibration.zMin + magCalibration.zMax) / 2;
                    printf("Offs: %d,%d,%d\n", xComp - xOffset, yComp - yOffset, zComp - zOffset);*/
                }
                //printf("Total Avg Mag: %.2f\n", (averageMagnitudeSum / (MAG_ITERATIONS * bootCnt)));

                printf("\n");
                device.blinkTimes(1,B_GREEN);
            }
            else {
                printf("%llu <= %llu\n", sampleCnt, MAG_START_SAVING_AFTER_SAMPLES_COUNT);
            }
            device.enableRTCInterruptInDeepSleep();
        }
        bootCnt++;
        device.deepSleep();
    }
}
