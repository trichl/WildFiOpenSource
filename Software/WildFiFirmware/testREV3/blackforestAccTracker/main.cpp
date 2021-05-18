#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

// POWER CONSUMPTION AVERAGE: 433uA @80MHz (445uA @10MHz -> NVS Init takes 10 times longer) -> NEW! 160uA!!!
// TODO: partial page programming flash memory

#define MODE_TESTRUN                            0       // with debug output, not writing into flash memory, not incrementing flashpnt
#define MODE_PRODUCTIVE                         1       // normal tracker mode, fully operational, flash memory should be empty before and flashpnt reset to 0
#define MODE_READFLASH                          2       // reads out full flash (10.000 pages = 30min) and then goes to sleep
#define MODE_RESETALL                           3       // deletes complete flash and flashpnt
#define MODE_SELFTEST                           4       // runs selftest on flash memory

/** --- CURRENT TRACKER MODE --- */
#define TRACKER_MODE                            MODE_PRODUCTIVE

#define BYTES_BEFORE_FIFO                       28
#define BATT_MIN_VOLTAGE                        3550
#define BATT_RESTART_VOLTAGE                    3700
#define FIRST_UNDER_VOLTAGE_SLEEP_TIME          2*60*60
#define FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME      4*60*60
#define BLINK_HOW_OFTEN                         2
#define ACC_INTERRUPT_WATERMARK                 960
#define ACC_RAM_SIZE_1                          (ACC_INTERRUPT_WATERMARK + 12)
#define ACC_RAM_SIZE_2                          4

// enums
typedef enum {
    ST_START = 0, ST_TRACK = 1,	ST_PWRDWN = 2
} tracker_state_t;

// RTC variables
RTC_DATA_ATTR uint16_t fifoLenRam = 0;
RTC_DATA_ATTR uint8_t fifoDataRam[ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2] = { 0 };
RTC_DATA_ATTR uint16_t fifoDataPointerRam = 0;

RTC_DATA_ATTR tracker_state_t state = ST_START;
RTC_DATA_ATTR uint32_t startCnt = 0;
RTC_DATA_ATTR uint16_t errorCnt = 0;

// normal variables
bool error = false;
uint64_t t = 0;
uint8_t *dataInDMA = NULL;
uint16_t dataPointer = 0;

void addData4(uint32_t d) {
    if(dataInDMA == NULL) {
        return;
    }
    dataInDMA[dataPointer] = d >> 24;
    dataInDMA[dataPointer+1] = d >> 16;
    dataInDMA[dataPointer+2] = d >>  8;
    dataInDMA[dataPointer+3] = d;
    dataPointer += 4;
}

void addData2(int16_t d) {
    if(dataInDMA == NULL) {
        return;
    }
    dataInDMA[dataPointer] = d >> 8;
    dataInDMA[dataPointer+1] = d;
    dataPointer += 2;
}

void readFlash(uint32_t addressStart, uint16_t times, uint16_t len) {
    uint8_t *dataTemp = NULL;
    if(!device.flash.createBuffer(&dataTemp, MT29_CACHE_SIZE)) { printf("ERROR BUFF\n"); }
    for(uint16_t j=0; j<times; j++) {
        if(!device.flash.read(addressStart+j, 0, dataTemp, len)) {
            printf("ERROR READ\n");
            return;
        }
        printf("READ %08X: ", addressStart+j);
        for(uint16_t i=0; i<len; i++) {
            printf("%02X", dataTemp[i]);
        }
        printf("\n");
    }
    heap_caps_free(dataTemp);
    printf("\n");
}

void selftest() {
    printf("V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub());
    device.disableWakeStubNoBootIfVoltageLow();

    if(!device.initNVS()) { printf("ERROR NVS\n"); }
    uint32_t flashPointer = device.nvsReadUINT32("flashpnt");
    printf("FLASH POINTER: %d\n", flashPointer);

    device.delay(8000);

    if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
    if(!device.flash.selfTestBadBlocks()) { printf("FOUND BAD BLOCKS!\n"); }

    uint8_t *testData = NULL;
    if(!device.flash.createBuffer(&testData, MT29_CACHE_SIZE)) { printf("ERROR BUFF\n"); }
    for(uint16_t i=0; i<200; i++) {
        testData[i] = i;
    }
    if(!device.flash.write(0, testData, 200)) { printf("ERROR\n"); }
    if(!device.flash.write(2, testData, 200)) { printf("ERROR\n"); }
    readFlash(0, 4, MT29_CACHE_SIZE);
    if(!device.flash.erase(0)) { printf("ERROR FLASH DELETE\n"); }
    readFlash(0, 10, MT29_CACHE_SIZE);
    
    if(!device.flashPowerOff()) { printf("ERROR FLASH OFF\n"); }

    device.enableInternalTimerInterruptInDeepSleep(3000);
    device.deepSleep();
}

void resetAll() {
    printf("V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub());
    device.disableWakeStubNoBootIfVoltageLow();

    // reset flash pointer
    if(!device.initNVS()) { printf("ERROR NVS\n"); }
    if(!device.nvsWriteUINT32("flashpnt", 0)) { errorCnt += 1; }
    uint32_t flashPointer = device.nvsReadUINT32("flashpnt");
    printf("FLASH POINTER AFTER DELETE: %d\n", flashPointer);

    // wait long so that serial monitor reset does not happen while already deleting the flash
    device.delay(8000);

    // delete flash
    if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
    if(!device.flash.fullErase()) { printf("FULL ERASE ERROR\n"); }
    //if(!device.flash.erase(0)) { printf("ERROR FLASH DELETE BLOCK 0\n"); }
    readFlash(0, 10, MT29_CACHE_SIZE); // read first 10 pages
    if(!device.flashPowerOff()) { printf("ERROR FLASH OFF\n"); }

    device.enableInternalTimerInterruptInDeepSleep(3000);
    device.deepSleep();    
}

void readFullFlash() {
    const uint32_t START_PAGES = 0; // 0
    const uint32_t READ_PAGES = 42058; // 42058
    const uint16_t READ_BYTES_PER_PAGE = (1024+BYTES_BEFORE_FIFO);

    if(!device.flashPowerOn()) { printf("ERROR FLASH\n"); }
    device.delay(8000);
    printf("START READING..\n");
    device.initNVS();
    uint32_t flashPointer = device.nvsReadUINT32("flashpnt");
    printf("FLASH POINTER: %d\n", flashPointer);
    
    uint8_t *dataTemp = NULL;
    if(!device.flash.createBuffer(&dataTemp, MT29_CACHE_SIZE)) { printf("ERROR BUFF\n"); }
    for(uint16_t j=0; j<READ_PAGES; j++) {
        if(!device.flash.read(START_PAGES+j, 0, dataTemp, 1024+BYTES_BEFORE_FIFO)) {
            printf("ERROR READ\n");
            return;
        }
        for(uint16_t i=0; i<READ_BYTES_PER_PAGE; i++) {
            printf("%02X", dataTemp[i]);
        }
        printf("\n");
        device.delay(10); // otherwise watchdog kicks in
    }
    heap_caps_free(dataTemp);
    printf("\n");

    device.enableInternalTimerInterruptInDeepSleep(6000);
    device.deepSleep();
}

RTC_DATA_ATTR bool wakeStub() {
    beginSw(); // start i2c
    fifoLenRam = getAccFIFOLength();
    if((fifoDataPointerRam + fifoLenRam) > (ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2)) { // storing fifo would exceed ACC array in RTC memory
        return true; // FIFO FULL, start the ESP32
    }
    readAccFIFO(fifoDataRam+fifoDataPointerRam, fifoLenRam, false);
    fifoDataPointerRam += fifoLenRam;
    return false;
}

extern "C" void app_main() {
    while(1) {
        //device.setCPUSpeed(ESP32_10MHZ); // CPU runs @80MHz is more energy efficient than @10MHz (about 5%) if NVS is used
        if(TRACKER_MODE != MODE_PRODUCTIVE) {
            if(TRACKER_MODE == MODE_READFLASH) { printf("MODE READ FLASH\n"); readFullFlash(); }
            else if(TRACKER_MODE == MODE_RESETALL) { printf("MODE RESET ALL\n"); resetAll(); }
            else if(TRACKER_MODE == MODE_SELFTEST) { printf("MODE SELFTEST\n"); selftest(); }
            else if(TRACKER_MODE == MODE_TESTRUN) { printf("MODE TESTRUN\n"); }
        }

        if(state == ST_START) {
            // disable no boot if voltage low (do here)
            device.disableWakeStubNoBootIfVoltageLow();

            // init BMX
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            device.delay(100); // wait until BMX is ready
            if(!device.imu.init(true, false, false, BMX160_ACCEL_ODR_6_25HZ, BMX160_LATCH_DUR_160_MILLI_SEC)) { errorCnt += 1; }
            if(!device.imu.startAccFOC()) { errorCnt += 1; }
            if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { errorCnt += 1; }
            if(!device.imu.initFIFOForAcc()) { errorCnt += 1; }

            // init RTC
            if(!device.rtc.setTimestamp(UNIX_TIMESTAMP+30)) { errorCnt += 1; }
            printf("TIME SET TO: %lu\n", (UNIX_TIMESTAMP+30));

            // print flash pointer (also in productive)
            if(!device.initNVS()) { errorCnt += 1; }
            uint32_t flashPointer = device.nvsReadUINT32("flashpnt");
            printf("FLASH POINTER: %d\n", flashPointer);

            // custom wake stub function
            device.customWakeStubFunction(wakeStub);
            device.setWakeStubRejectionInterruptSrc(USE_EXT1_IF_WAKE_UP_REJECTED);

            // move to next state
            device.blink(B_GREEN, B_GREEN, B_GREEN);
            state = ST_TRACK;
            device.enableAccInterruptInDeepSleep();
        }
        else if(state == ST_PWRDWN) {
            //printf("(PWR_DWN) V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub());
            if(device.readSupplyVoltageFromWakeStub() <= BATT_RESTART_VOLTAGE) {
                device.enableInternalTimerInterruptInDeepSleep(FOLLOWING_UNDER_VOLTAGE_SLEEP_TIME); // sleep for longer time 
            }
            else {
                //printf("RESTART!\n");
                i2c.begin(); // start i2c
                if(!device.imu.resetFIFO()) { // empty fifo, do not read it
                    errorCnt += 1;
                }
                state = ST_TRACK;
                device.enableAccInterruptInDeepSleep(); // wake up from acc interrupt
            }
        }
        else if(state == ST_TRACK) {
            if(TRACKER_MODE == MODE_TESTRUN) {
                printf("WAKE NUMBER: %d\n", startCnt);
                printf("V_BATT WAKE STUB: %d\n", device.readSupplyVoltageFromWakeStub());
            }

            if(device.readSupplyVoltageFromWakeStub() < BATT_MIN_VOLTAGE) {
                if(TRACKER_MODE == MODE_TESTRUN) {
                    printf("POWER DOWN\n");
                }
                state = ST_PWRDWN;
                device.enableInternalTimerInterruptInDeepSleep(FIRST_UNDER_VOLTAGE_SLEEP_TIME); // sleep for some time
            }
            else {
                // I2C start (1ms)
                i2c.begin();

                // reserve some memory (0ms)
                if(!device.flash.createBuffer(&dataInDMA, MT29_CACHE_SIZE)) { errorCnt += 1; }

                // start bme measurement (7ms)
                bool bmeOk = false;
                int16_t temperature = 0;
                int16_t temperatureBmx = 0;
                uint32_t pressure = 0;
                uint32_t humidity = 0;
                if(device.baro.init(BME680_OS_8X, BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 0, 0)) {
                    if(device.baro.performMeasurement()) {
                        bmeOk = true;
                    }
                }

                // get fifo data (101ms -> NEW: 15ms)
                t = Arduino::millisWrapper();
                uint16_t fifoReadout = device.imu.readAccFIFOInOneGo(dataInDMA+BYTES_BEFORE_FIFO);
                if(TRACKER_MODE == MODE_TESTRUN) {
                    printf("RAM FIFO DATA POINTER: %d, LAST FIFO LEN: %d\n", fifoDataPointerRam, fifoLenRam);
                    printf("FIFO READ TOOK: %lld ms\n", (Arduino::millisWrapper() - t));
                    printf("READOUT LENGTH: %d\n", fifoReadout);
                    device.delay(1);
                    uint16_t fifoLenNew = device.imu.getFIFOLength();
                    printf("BMX FIFO AFTER READ: %d\n", fifoLenNew);
                }

                // get bmx temperature
                device.imu.getTemperature(temperatureBmx);

                // get rtc timestamp (1ms)
                uint32_t timestamp = device.rtc.getTimestamp(error);
                if(error) { errorCnt += 1; }
                if(TRACKER_MODE == MODE_TESTRUN) {
                    printf("TIMESTAMP: %d\n", timestamp);
                }

                // get bme data (1ms)
                if(bmeOk) {
                    if(device.baro.getResults()) {
                        temperature = device.baro.getTemperature(error);
                        if(error) { errorCnt += 1; }
                        pressure = device.baro.getPressure(error);
                        if(error) { errorCnt += 1; }
                        humidity = device.baro.getHumidity(error);
                        if(error) { errorCnt += 1; }
                        if(TRACKER_MODE == MODE_TESTRUN) {
                            printf("TEMP: %d\nTEMP BMX: %d\nPRESS: %d\nHUMI: %d\n", temperature, temperatureBmx, pressure, humidity);
                        }
                    }
                }
                else { errorCnt += 1; }

                // add rest of data
                addData4(timestamp);
                addData4(startCnt);
                addData4(device.readSupplyVoltageFromWakeStub());
                addData2(temperature);
                addData4(humidity);
                addData4(pressure);
                addData2(temperatureBmx);
                addData2(fifoReadout);
                addData2(errorCnt);

                // print data
                if(TRACKER_MODE == MODE_TESTRUN) {
                    for(uint16_t i=0; i<(BYTES_BEFORE_FIFO+fifoReadout); i++) {
                        printf("%02X ", dataInDMA[i]);
                        if(i==(BYTES_BEFORE_FIFO-1)) {
                            printf("\n");
                        }
                    }
                    printf("\n");

                    printf("RAM: ");
                    for(uint16_t i=0; i<fifoDataPointerRam; i++) {
                        printf("%02X ", fifoDataRam[i]);
                    }
                    printf("\n");
                }

                // write data to flash
                if(TRACKER_MODE == MODE_PRODUCTIVE) { // ONLY WRITE IN NVS AND FLASH IF PRODUCTIVE START!!!
                    // turn on flash power already (5ms)
                    if(!device.flashPowerOn(false)) { errorCnt += 1; }
                    
                    // get flash pointer from NVS (15ms)
                    if(!device.initNVS()) { errorCnt += 1; }
                    uint32_t flashPointer = device.nvsReadThenPlusOneUINT32("flashpnt");

                    // write into flash memory (2ms)
                    if(!device.flash.write(flashPointer, dataInDMA, BYTES_BEFORE_FIFO+fifoReadout)) { errorCnt += 1; }

                    // turn off flash power (3ms)
                    if(!device.flashPowerOff()) { errorCnt += 1; }
                }

                // fifo data in ram
                fifoDataPointerRam = 0; // reset pointer

                // print error count
                if(TRACKER_MODE == MODE_TESTRUN) {
                    printf("ERROR CNT: %d\n", errorCnt);
                }

                // blink first times to see if tag is operating
                if(startCnt < BLINK_HOW_OFTEN) {
                    device.blink(B_GREEN, B_GREEN, B_GREEN);
                }

                // free memory
                heap_caps_free(dataInDMA);
                device.enableAccInterruptInDeepSleep();
            }
        }
        startCnt++;
        device.deepSleep();
    }
}