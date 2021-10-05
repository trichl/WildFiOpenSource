#include "PlatformWildFiTagREV6.h"
#include "ModuleGPS_L70_REV6.h"
#include <math.h>

WildFiTagREV6 device = WildFiTagREV6();
GPS_L70_REV6 gps = GPS_L70_REV6();

RTC_DATA_ATTR bool firstStart = true;

void GPSNew2() {
    esp_gps_t gpsData = { };
    device.gpioBOn();
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');

    gps_get_fix_config_t gpsConfig = {
        .timeoutSeconds = 300,
        .timeoutNotEvenTimeSeconds = 120,
        .minHDOP = 2.5f, // not important, stopping after getting time
        .afterFixMaxWaitOnHDOP = 12, // not important, stopping after getting time
        .setRTCTime = true,
        .blinkLeds = true,
        .debug = true };
    get_fix_result_t fixResult = gps.tryToGetFix(&gpsData, &gpsConfig, &device);

    printf("%d Res: %d.%d.%d %d:%d:%d.%03u, LAT: %f, LON: %f, SATS: %d, HDOP: %.2f, FIX: %d\n\n", (uint8_t) fixResult, gpsData.parent.date.day, gpsData.parent.date.month, gpsData.parent.date.year, gpsData.parent.tim.hour, gpsData.parent.tim.minute, gpsData.parent.tim.second, gpsData.parent.tim.thousand, gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.sats_in_use, gpsData.parent.dop_h, gpsData.parent.fix);
    
    device.gpioBOff();

    if(fixResult != GPS_FIX_SUCCESS_AND_RTC_UPDATED) {
        printf("ERROR -> STOP\n");
        while(1) { ; }
    }
}

void GPSNew() {
    //uint32_t waitCnt = 0;
    bool errorDuringInit = false;
    esp_gps_t gpsData = { };

    device.gpioBOn();
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');

    if(!gps.isStarted()) { errorDuringInit = true; printf("E1"); }
    if(!gps.setFLPMode(false)) { errorDuringInit = true; printf("E2"); }
    if(!gps.setNMEAMessagesMinimum1HzWithZDA()) { errorDuringInit = true; printf("E3"); }
    uart_flush(UART2_PORT_NUMBER);

    if(errorDuringInit) {
        printf("ERROR\n");
        while(1) { ; }
    }
    uint32_t errorCnt = 0;
    uint16_t millisWait = 0;
    //bool hasValidTimestamp = false;
    while(true) {
        uart_flush(UART2_PORT_NUMBER);
        device.enableUart2InterruptInLightSleep();
        device.lightSleep();
        bool hasValidTimestamp = false;
        device.ledRedOn();
        get_uart_result_t res = gps.afterLightSleepWaitForGPRMCandGPGGA(&gpsData, &hasValidTimestamp, &millisWait, true);
        device.ledRedOff();
        if(res != GPS_UART_RESULT_SUCESS) { errorCnt++; }
        printf("Res: %d (%d), TS: %d\n", (uint8_t) res, errorCnt, hasValidTimestamp);
        printf("Res: %d.%d.%d %d:%d:%d.%03u, LAT: %f, LON: %f, SATS: %d, HDOP: %.2f, FIX: %d\n\n", gpsData.parent.date.day, gpsData.parent.date.month, gpsData.parent.date.year, gpsData.parent.tim.hour, gpsData.parent.tim.minute, gpsData.parent.tim.second, gpsData.parent.tim.thousand, gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.sats_in_use, gpsData.parent.dop_h, gpsData.parent.fix);
        
        //printf("stack: %d\n\n", uxTaskGetStackHighWaterMark(NULL));
        //device.delay(30);
    }
}

void GPSDebug() {
    uart_event_t event;
    size_t bufferedSize;
    //uint32_t waitCnt = 0;
    uint8_t *uart2Data = (uint8_t*) malloc(UART2_RX_BUFFER);
    bool errorDuringInit = false;

    device.gpioBOn();
    device.uart2Init(115200);
    gps.init(device.uart2GetQueue());
    device.uart2EnablePatternInterrupt('\n');

    if(!gps.isStarted()) { errorDuringInit = true; }
    if(!gps.setFLPMode(false)) { errorDuringInit = true; }
    if(!gps.setNMEAMessagesMinimum1HzWithZDA()) { errorDuringInit = true; }
    uart_flush(UART2_PORT_NUMBER);

    bzero(uart2Data, UART2_RX_BUFFER); // write all zeros into buffer

    if(errorDuringInit) {
        printf("ERROR\n");
        while(1) { ; }
    }

    bool receiveTimeout = false;

    while(true) {
        receiveTimeout = false;
        uart_flush(UART2_PORT_NUMBER);
        device.enableUart2InterruptInLightSleep();
        device.lightSleep();
        while(true) {
            if(!xQueueReceive(*(device.uart2GetQueue()), (void * )&event, (50 / portTICK_PERIOD_MS))) {
                //nmeaMessageCounter = 0;
                //printf(".");
                if(!receiveTimeout) {
                    receiveTimeout = true;
                    printf("\n");
                    break;
                }
            }
            else {
                receiveTimeout = false;
                if(event.type == UART_PATTERN_DET) { // '\n' detected

                }
                else if(event.type == UART_DATA) {
                    uart_get_buffered_data_len(UART2_PORT_NUMBER, &bufferedSize); // get length of message in UART buffer
                    if(bufferedSize > 0) {
                        bool lineBreak = false;
                        bzero(uart2Data, UART2_RX_BUFFER); // write all zeros into buffer
                        int readLen = uart_read_bytes(UART2_PORT_NUMBER, uart2Data, bufferedSize, 100 / portTICK_PERIOD_MS);
                        if(readLen > 0) {
                            uart2Data[readLen] = '\0'; // make sure the line is a standard string
                            for(uint16_t a = 0; a < readLen; a++) {
                                if(uart2Data[a] == '\n') {  uart2Data[a] = 'n'; lineBreak = true; }
                                if(uart2Data[a] == '\r') {  uart2Data[a] = 'r'; }
                                if(uart2Data[a] == '\0') { uart2Data[a] = 'x'; } // make sure that rubbish at start does not include string termination
                            }
                            char *uart2DataChar = (char *) uart2Data;
                            printf("(%d.%03d|%s|%d)", ((uint32_t) Timing::millis()) / 1000, ((uint32_t) Timing::millis()) % 1000, uart2DataChar, strlen(uart2DataChar));
                            if(lineBreak) printf("\n");
                            bufferedSize = 0;
                        }
                    }
                    //printf("%d.%03d: D: %d, %d, Buff: %d\n", event.type, event.size, bufferedSize);
                }
            }
        }
    }
}

extern "C" void app_main() {
    while(1) {
        i2c.begin(I2C_FREQ_HZ_400KHZ);
        //GPSDebug();
        //GPSNew();
        GPSNew2();
        device.enableInternalTimerInterruptInDeepSleep(10);
        device.deepSleep();
    }
}
