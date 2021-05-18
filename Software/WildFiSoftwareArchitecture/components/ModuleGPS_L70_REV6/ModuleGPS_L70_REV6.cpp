#include "ModuleGPS_L70_REV6.h"

static bool validTimeTimerFinished = false;
static void validTimeCallback(void* arg) {
    validTimeTimerFinished = true;
}

GPS_L70_REV6::GPS_L70_REV6() {
	queueHandle = NULL;
}

void GPS_L70_REV6::init(QueueHandle_t* queueHandleIn) {
	queueHandle = queueHandleIn;
}

uint32_t GPS_L70_REV6::estimateUARTSendTimeMs(uint32_t messageLength) {
    // delay from UART: (max. 2 * 80 Byte*(8+2) * (1 / 115200 = 0.00868ms/bit) = 13.88ms)
    float delay = messageLength;
    delay = messageLength * (8 + 2); // total number of BITS that were transmitted (UART: one byte needs two more bits for transmission)
    delay = delay * (1000. / 115200.); // divide by ms per bit

    if((delay - ((uint32_t) (delay))) < 0.5) { return ((uint32_t) delay); } // floor
    return (((int) (delay)) + 1); // ceil
}

bool GPS_L70_REV6::getTimeOnly(esp_gps_t *gpsData, uint32_t timeoutSeconds, WildFiTagREV6 *device, bool blinkRedLed, bool debug) {
    // L70 stages: first time in GPGGA, then a bit later date in GPRMC (GOT TIMESTAMP then), then fix
    const uint8_t RUBBISH_MINIMUM_LENGTH_NMEA = 10; // 17
    uint16_t addExtraDelay = 0;
    uint8_t nmeaMessageCounter = 0;
    int64_t ttffStartUs = 0;

    if(!isStarted()) { return false; }
    ttffStartUs = esp_timer_get_time(); // start timer
    if(!setNMEAMessagesMinimum1HzWithZDA()) { return false; }
    uart_flush(UART2_PORT_NUMBER);

    const esp_timer_create_args_t validTimeArgs = { .callback = &validTimeCallback };
    esp_timer_handle_t validTimeTimer;
	if(esp_timer_create(&validTimeArgs, &validTimeTimer) != ESP_OK) { return false; }

    uint8_t *uart2Data = (uint8_t*) malloc(UART2_RX_BUFFER);
    if(uart2Data == NULL) { return false; }

    device->enableUart2InterruptInLightSleep();
    esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering (disconnected)
    device->lightSleep();

    while(true) {
        //uart_flush(UART2_PORT_NUMBER); // WHY?
        nmeaMessageCounter = 0;
        uart_event_t event;
        size_t bufferedSize;

        // turn on green LED
        if(blinkRedLed) { device->ledRedOn(); }

        // check if wakeup due to timeout
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
            free(uart2Data);
            if(debug) { printf("getTimeOnly: fatal GPS not responding!\n"); }
            if(blinkRedLed) { device->ledRedOff(); }
            return false;
        }

        // check timeout
        int64_t ttffTemp = (esp_timer_get_time() - ttffStartUs) / 1000000ULL;
        uint32_t ttffCompareSeconds = (uint32_t) ttffTemp;
        if(ttffCompareSeconds >= timeoutSeconds) {
            free(uart2Data);
            if(debug) { printf("getTimeOnly: timeout!\n"); }
            if(blinkRedLed) { device->ledRedOff(); }
            return false;
        }

        while(true) {
            if(xQueueReceive(*(device->uart2GetQueue()), (void * )&event, 0)) {
                if(event.type == UART_PATTERN_DET) { // '\n' detected
                    nmeaMessageCounter++;
                    uart_get_buffered_data_len(UART2_PORT_NUMBER, &bufferedSize); // get length of message in UART buffer
                    if((nmeaMessageCounter == 1) && (bufferedSize < RUBBISH_MINIMUM_LENGTH_NMEA)) { nmeaMessageCounter--; } // maybe \n in first 20 bytes of rubbish detected (happened) -> do not count as NMEA message

                    if(nmeaMessageCounter >= 3) {
                        bzero(uart2Data, UART2_RX_BUFFER); // write all zeros into buffer
                        int readLen = uart_read_bytes(UART2_PORT_NUMBER, uart2Data, bufferedSize, 100 / portTICK_PERIOD_MS); // pos + 1 to also read the pattern itself (\n)
                        if(readLen > 0) { // string looks like this: "???????,,,,0.00,0.00,050180,,,N*4C\r\n$GPGGA,000225.800,,,,,0,0,,,M,,M,,*45\r\n" -> ~20 characters lost at start (@115200)
                            uart2Data[readLen] = '\0'; // make sure the line is a standard string
                            for(uint16_t a = 0; a < readLen; a++) {
                                if(uart2Data[a] == '\0') { uart2Data[a] = 'X'; } // make sure that rubbish at start does not include string termination
                            }
                            //for(uint8_t rubbish = 0; rubbish < RUBBISH_LENGTH_AFTER_LIGHT_SLEEP; rubbish++) { uart2Data[rubbish] = 'X'; } // instead of rubbish that might include string terminations or line breaks, just add Xs
                            char *uart2DataChar = (char *) uart2Data;
                            
                            // extract complete $GP messages
                            bool foundRealStart = false;
                            char *validPart = strchr(uart2DataChar, '$'); // substring: first start of $
                            while(!foundRealStart) {
                                if(validPart == NULL) { break; } // no more '$' in string
                                else {
                                    if(strlen(validPart) < 5) { break; } // should be an error
                                    else {
                                        if(strncmp("$GP", validPart, 3) == 0) { foundRealStart = true; break; } // first time that after $ comes valid GPS message -> stop
                                        else { validPart = strchr(validPart + 1, '$'); } // + 1 because pointing on next char value
                                    }
                                }
                            }
                            if(validPart != NULL) {
                                if(foundRealStart) {
                                    uint16_t numberFullNMEAs = 0;
                                    char *temp = validPart;
                                    for(numberFullNMEAs=0; temp[numberFullNMEAs]; temp[numberFullNMEAs]=='$' ? numberFullNMEAs++ : *temp++);
                                    if(numberFullNMEAs != 2) {
                                        if(debug) { printf("getTimeOnly: WARNING: received %d msgs!\n", numberFullNMEAs); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                    }
                                    if(strncmp("$GPRMC", validPart, 6) != 0) { // first message (after ZDA) should be GPRMC
                                        if(debug) { printf("getTimeOnly: WARNING: rmc not first msg -> wait 500ms!\n"); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                        addExtraDelay = 500;
                                    }
                                    //if(debug) { printf("getTimeOnly: valid: %s", validPart); }  // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                    if(gpsDecodeLine(gpsData, validPart, readLen + 1) != GPS_DECODE_RESULT_SUCESS) {
                                        if(debug) { printf("getTimeOnly: decode line error\n"); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                    }
                                }
                            }
                            //if(updateUTCTimestamp(gpsData) && (gpsData->parent.fix > 0)) { // NEW!!!
                            if(updateUTCTimestamp(gpsData)) { // first time getting a valid GPS time!
                                // --- BE QUICK HERE ---
                                // delay from UART: (max. 2 * 80 Byte*(8+2) * (1 / 115200 = 0.00868ms/bit) = 13.88ms)
                                validTimeTimerFinished = false;

                                // start timer
                                uint32_t waitTimeUs = gpsData->parent.tim.thousand;
                                // TODO: add estimateUARTSendTimeMs
                                waitTimeUs = (1000 - waitTimeUs) * 1000;
                                if(waitTimeUs == 0) { waitTimeUs = 1; }

                                tmElements_t timeStruct;
                                if(waitTimeUs == 1000000) { // 1 full second to wait (milliseconds of GPS = 0) -> don't wait
                                    breakTime(gpsData->parent.utcTimestamp, timeStruct); // use THIS second
                                    waitTimeUs = 0;
                                }
                                else {
                                    if(esp_timer_start_once(validTimeTimer, waitTimeUs) != ESP_OK) {
                                        if(blinkRedLed) { device->ledRedOff(); }
                                        uart_flush(UART2_PORT_NUMBER); free(uart2Data); return false;
                                    }
                                    while(!validTimeTimerFinished) { ; } // busy waiting until full second
                                    breakTime(gpsData->parent.utcTimestamp + 1, timeStruct);
                                    gpsData->parent.utcTimestamp += 1;
                                }

                                /*if(debug) { 
                                    // calculate difference to current time
                                    bool error = false;
                                    uint32_t oldTimestamp = device->rtc.getTimestamp(error);
                                    uint32_t oldMilliseconds = (uint32_t) (device->rtc.get100thOfSeconds(error));
                                    oldMilliseconds *= 10;
                                    // PROBLEM: sometimes millis already 0, but rest not updated
                                    if(oldMilliseconds == 0) {
                                        printf("getTimeOnly: 0ms -> repeat %u ->", oldTimestamp);
                                        oldTimestamp = device->rtc.getTimestamp(error);
                                        printf("%u\n", oldTimestamp);
                                    }
                                    int64_t timestampBeforeMs = oldTimestamp;
                                    timestampBeforeMs = (timestampBeforeMs * 1000) + oldMilliseconds;
                                    int64_t timestampNowMs = gpsData->parent.utcTimestamp;
                                    timestampNowMs *= 1000;
                                    int64_t timestampDiffMs = timestampNowMs - timestampBeforeMs;
                                    printf("getTimeOnly: %lld -> %lld, diff: %lld\n", timestampBeforeMs, timestampNowMs, timestampDiffMs);  
                                }*/

                                if(!device->rtc.set(timeStruct.Hour, timeStruct.Minute, timeStruct.Second, timeStruct.Wday, timeStruct.Day, timeStruct.Month, timeStruct.Year)) {
                                    if(blinkRedLed) { device->ledRedOff(); }
                                    uart_flush(UART2_PORT_NUMBER); free(uart2Data); return false;
                                }
                                if(debug) { printf("getTimeOnly: updated RTC to: %02d:%02d:%02d (wait: %u ms)\n", timeStruct.Hour, timeStruct.Minute, timeStruct.Second, (waitTimeUs/1000)); }
                                if(debug) { printf("getTimeOnly: %llds: UTC %d, %d.%d.%d %d:%d:%d.%03u, LAT: %f, LON: %f, SATS: %d, HDOP: %.2f, FIX: %d\n", (esp_timer_get_time()-ttffStartUs)/1000000ULL, gpsData->parent.utcTimestamp, gpsData->parent.date.day, gpsData->parent.date.month, gpsData->parent.date.year, gpsData->parent.tim.hour, gpsData->parent.tim.minute, gpsData->parent.tim.second, gpsData->parent.tim.thousand, gpsData->parent.latitude, gpsData->parent.longitude, gpsData->parent.sats_in_use, gpsData->parent.dop_h, gpsData->parent.fix); }
                        
                                if(blinkRedLed) { device->ledRedOff(); }
                                uart_flush(UART2_PORT_NUMBER);
                                free(uart2Data);
                                return true;
                            }
                            if(debug) { printf("getTimeOnly: %llds: UTC %d, %d.%d.%d %d:%d:%d.%03u, LAT: %f, LON: %f, SATS: %d, HDOP: %.2f, FIX: %d\n", (esp_timer_get_time()-ttffStartUs)/1000000ULL, gpsData->parent.utcTimestamp, gpsData->parent.date.day, gpsData->parent.date.month, gpsData->parent.date.year, gpsData->parent.tim.hour, gpsData->parent.tim.minute, gpsData->parent.tim.second, gpsData->parent.tim.thousand, gpsData->parent.latitude, gpsData->parent.longitude, gpsData->parent.sats_in_use, gpsData->parent.dop_h, gpsData->parent.fix); }
                        }
                        break; // go into light sleep
                    }
                }
            }        
        } // finished with getting gps data (but didn't get time yet)
        if(addExtraDelay > 0) { device->delay(addExtraDelay); addExtraDelay = 0; } // in case of unsynced GPS messages
        if(blinkRedLed) { device->ledRedOff(); }
        uart_flush(UART2_PORT_NUMBER);
        device->enableUart2InterruptInLightSleep();
        esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
        device->lightSleep();
    }
    // should never be reached
    free(uart2Data);
    if(blinkRedLed) { device->ledRedOff(); }
    return false;
}

get_fix_result_t GPS_L70_REV6::tryToGetFix(esp_gps_t *gpsData, gps_get_fix_config_t *config, WildFiTagREV6 *device) {
    // UNTESTED
    // WARNING: >900ms wait time might fuck up receiving order?!? -> add separate delay??
    // L70 stages: first time in GPGGA, then a bit later date in GPRMC (GOT TIMESTAMP then), then fix
    const uint8_t RUBBISH_MINIMUM_LENGTH_NMEA = 10;
    uint16_t addExtraDelay = 0;
    bool gotTimeAfterFix = false;
    bool gotFix = false;
    bool updatedRTC = false;
    bool couldUpdateTime = false;
    uint8_t nmeaMessageCounter = 0;
    int64_t ttffStartUs = 0;
    int64_t ttffWaitAfterFixUs = 0;
    const int64_t ttffWaitAfterFixLimitUs = config->afterFixMaxWaitOnHDOP * 1000ULL * 1000ULL;

    if(!isStarted()) { return GPS_FIX_RESULT_NOT_ANSWERING; }
    ttffStartUs = esp_timer_get_time(); // start timer
    gpsData->parent.ttfMilliseconds = 0;
    if(!setNMEAMessagesMinimum1HzWithZDA()) { return GPS_FIX_RESULT_SETTINGS_NO_ANSWER; }
    uart_flush(UART2_PORT_NUMBER);

    const esp_timer_create_args_t validTimeArgs = { .callback = &validTimeCallback };
    esp_timer_handle_t validTimeTimer;
	if(config->setRTCTime) {
		if(esp_timer_create(&validTimeArgs, &validTimeTimer) != ESP_OK) { return GPS_FIX_TIMER_FAIL; }
	}

    uint8_t *uart2Data = (uint8_t*) malloc(UART2_RX_BUFFER);
    if(uart2Data == NULL) { return GPS_FIX_MALLOC_FAIL; }

    device->enableUart2InterruptInLightSleep();
    esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering (disconnected)
    device->lightSleep();

    while(!gotFix) {
        //uart_flush(UART2_PORT_NUMBER); // WHY?
        nmeaMessageCounter = 0;
        uart_event_t event;
        size_t bufferedSize;

        // turn on LED
        if(config->blinkLeds) {
            if(couldUpdateTime) { device->ledGreenOn(); } // or gpsData->parent.fix > 0
            else { device->ledRedOn(); }
        }

        // check if wakeup due to timeout
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
            free(uart2Data);
            if(config->debug) { printf("tryToGetFix: fatal GPS not responding!\n"); }
            if(config->blinkLeds) { device->ledGreenOff(); device->ledRedOff(); }
            return GPS_FIX_NO_ANSWER_FATAL;
        }

        // check timeout
        int64_t ttffTemp = (esp_timer_get_time() - ttffStartUs) / 1000000ULL;
        uint32_t ttffCompareSeconds = (uint32_t) ttffTemp;
        if(ttffCompareSeconds >= config->timeoutSeconds) {
            free(uart2Data);
            if(config->debug) { printf("tryToGetFix: timeout!\n"); }
            if(config->blinkLeds) { device->ledGreenOff(); device->ledRedOff(); }
            return GPS_FIX_TIMEOUT;
        }

        while(true) {
            if(xQueueReceive(*(device->uart2GetQueue()), (void * )&event, 0)) {
                if(event.type == UART_PATTERN_DET) { // '\n' detected
                    nmeaMessageCounter++;
                    uart_get_buffered_data_len(UART2_PORT_NUMBER, &bufferedSize); // get length of message in UART buffer
                    if((nmeaMessageCounter == 1) && (bufferedSize < RUBBISH_MINIMUM_LENGTH_NMEA)) { nmeaMessageCounter--; } // maybe \n in first 20 bytes of rubbish detected (happened) -> do not count as NMEA message

                    if(nmeaMessageCounter >= 3) {
                        bzero(uart2Data, UART2_RX_BUFFER); // write all zeros into buffer
                        // uart_get_buffered_data_len(UART2_PORT_NUMBER, &bufferedSize);
                        int readLen = uart_read_bytes(UART2_PORT_NUMBER, uart2Data, bufferedSize, 100 / portTICK_PERIOD_MS); // pos + 1 to also read the pattern itself (\n)
                        if(readLen > 0) { // string looks like this: "???????,,,,0.00,0.00,050180,,,N*4C\r\n$GPGGA,000225.800,,,,,0,0,,,M,,M,,*45\r\n" -> ~20 characters lost at start (@115200)
                            uart2Data[readLen] = '\0'; // make sure the line is a standard string
                            for(uint16_t a = 0; a < readLen; a++) {
                                if(uart2Data[a] == '\0') { uart2Data[a] = 'X'; } // make sure that rubbish at start does not include string termination
                            }
                            char *uart2DataChar = (char *) uart2Data;
                            
                            // extract complete $GP messages
                            bool foundRealStart = false;
                            char *validPart = strchr(uart2DataChar, '$'); // substring: first start of $
                            while(!foundRealStart) {
                                if(validPart == NULL) { break; } // no more '$' in string
                                else {
                                    if(strlen(validPart) < 5) { break; } // should be an error
                                    else {
                                        if(strncmp("$GP", validPart, 3) == 0) { foundRealStart = true; break; } // first time that after $ comes valid GPS message -> stop
                                        else { validPart = strchr(validPart + 1, '$'); } // + 1 because pointing on next char value
                                    }
                                }
                            }
                            if(validPart != NULL) {
                                if(foundRealStart) {
                                    uint16_t numberFullNMEAs = 0;
                                    char *temp = validPart;
                                    for(numberFullNMEAs=0; temp[numberFullNMEAs]; temp[numberFullNMEAs]=='$' ? numberFullNMEAs++ : *temp++);
                                    if(numberFullNMEAs != 2) {
                                        if(config->debug) { printf("tryToGetFix: WARNING: received %d msgs!\n", numberFullNMEAs); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                    }
                                    if(strncmp("$GPRMC", validPart, 6) != 0) { // first message (after ZDA) should be GPRMC
                                        if(config->debug) { printf("tryToGetFix: WARNING: rmc not first msg -> wait 500ms!\n"); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                        addExtraDelay = 500;
                                    }
                                    //if(config->debug) { printf("tryToGetFix: valid: %s", validPart); }  // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                    if(gpsDecodeLine(gpsData, validPart, readLen + 1) != GPS_DECODE_RESULT_SUCESS) {
                                        if(config->debug) { printf("tryToGetFix: decode line error\n"); } // WARNING: CREATES TIME DELAY WHEN SYNCHRONIZING RTC
                                    }
                                }
                            }

                            // calculate timestamp
                            couldUpdateTime = updateUTCTimestamp(gpsData); // returns true if time could be set (means timestamp is valid)

                            // check if got a valid time for the first time
                            if(couldUpdateTime && (!gotTimeAfterFix) && (gpsData->parent.fix > 0)) { // first time getting a valid GPS time! (only executed once) -> ONLY AFTER GETTING A FIX
                                gotTimeAfterFix = true;
                                if(config->setRTCTime) { // ASSUMING I2C HAS BEEN STARTED! update RTC time
                                    // --- BE QUICK HERE ---
                                    // delay from UART: (max. 2 * 80 Byte*(8+2) * (1 / 115200 = 0.00868ms/bit) = 13.88ms)
                                    validTimeTimerFinished = false;

                                    // if debugging: get current milliseconds to compare
                                    uint8_t seconds100thOld = 0;
                                    uint16_t millisecondsOld = 0;
                                    if(config->debug) { 
                                        bool error = false;
                                        seconds100thOld = device->rtc.get100thOfSeconds(error);
                                        millisecondsOld = seconds100thOld * 10;
                                    }

                                    // start timer
                                    uint32_t waitTimeUs = gpsData->parent.tim.thousand;
                                    // TODO: add estimateUARTSendTimeMs
                                    waitTimeUs = (1000 - waitTimeUs) * 1000;
                                    if(waitTimeUs == 0) { waitTimeUs = 1; } // should not happen, but just in case

                                    tmElements_t timeStruct;
                                    if(waitTimeUs == 1000000) { // 1 full second to wait (milliseconds of GPS = 0) -> don't wait
                                        breakTime(gpsData->parent.utcTimestamp, timeStruct); // use THIS second
                                        waitTimeUs = 0;
                                    }
                                    else {
                                        if(esp_timer_start_once(validTimeTimer, waitTimeUs) != ESP_OK) {
                                            if(config->blinkLeds) { device->ledGreenOff(); device->ledRedOff(); }
                                            uart_flush(UART2_PORT_NUMBER); free(uart2Data); return GPS_FIX_TIMER_FAIL;
                                        }
                                        while(!validTimeTimerFinished) { ; } // busy waiting until full second
                                        breakTime(gpsData->parent.utcTimestamp + 1, timeStruct);
                                    }

                                    if(!device->rtc.set(timeStruct.Hour, timeStruct.Minute, timeStruct.Second, timeStruct.Wday, timeStruct.Day, timeStruct.Month, timeStruct.Year)) {
                                        if(config->blinkLeds) { device->ledGreenOff(); device->ledRedOff(); }
                                        uart_flush(UART2_PORT_NUMBER); free(uart2Data); return GPS_FIX_RTC_CONFIG_FAIL;
                                    }

                                    updatedRTC = true;
                                    if(config->debug) {
                                        printf("tryToGetFix: RTC milli compare before sync: %u -> %u\n", millisecondsOld, gpsData->parent.tim.thousand);
                                        printf("tryToGetFix: updated RTC to: %d:%d:%d (wait: %u ms)\n", timeStruct.Hour, timeStruct.Minute, timeStruct.Second, (waitTimeUs/1000));
                                    }
                                }
                                if(config->debug) { printf("tryToGetFix: ---- GOT 1ST VALID TIME: %u.%u ----\n", gpsData->parent.utcTimestamp, gpsData->parent.tim.thousand); }
                            }

                            // check if we shall stop
                            if(config->debug) { printf("tryToGetFix: %llds: %d.%d.%d %d:%d:%d.%03u, LAT: %f, LON: %f, SATS: %d, HDOP: %.2f, FIX: %d\n", (esp_timer_get_time()-ttffStartUs)/1000000ULL, gpsData->parent.date.day, gpsData->parent.date.month, gpsData->parent.date.year, gpsData->parent.tim.hour, gpsData->parent.tim.minute, gpsData->parent.tim.second, gpsData->parent.tim.thousand, gpsData->parent.latitude, gpsData->parent.longitude, gpsData->parent.sats_in_use, gpsData->parent.dop_h, gpsData->parent.fix); }
                            if(gpsData->parent.fix > 0) { // valid gps fix?
                                if(ttffWaitAfterFixUs == 0) {
                                    if(config->debug) { printf("tryToGetFix: ---- FIX AFTER %lld SECONDS, BUT WAIT ----\n", (esp_timer_get_time()-ttffStartUs)/1000000ULL); }
                                    ttffWaitAfterFixUs = esp_timer_get_time(); // first time got fix - remember time
                                }
                                // wait maximum x seconds or until HDOP is already good enough
                                if((gpsData->parent.dop_h < config->minHDOP) || ((esp_timer_get_time() - ttffWaitAfterFixUs) > ttffWaitAfterFixLimitUs)) {
                                    if(config->debug) { printf("tryToGetFix: ---- FIX AFTER %lld SECONDS ----\n", (esp_timer_get_time()-ttffStartUs)/1000000ULL); }
                                    gpsData->parent.ttfMilliseconds = (uint32_t) ((esp_timer_get_time() - ttffStartUs) / 1000ULL);
                                    gotFix = true;
                                }
                            }
                            //if(config->debug) { printf("\n"); }
                            //if(config->debug) { device->delay(20); } // for printf to finish before light sleep
                        }
                        break; // go into light sleep
                    }
                }
            }        
        }
        if(!gotFix) {
            if(addExtraDelay > 0) { device->delay(addExtraDelay); addExtraDelay = 0; } // in case of unsynced GPS messages
            uart_flush(UART2_PORT_NUMBER);
            if(config->blinkLeds) { device->ledGreenOff(); device->ledRedOff(); }
            device->enableUart2InterruptInLightSleep();
            esp_sleep_enable_timer_wakeup(15000000UL); // 15 seconds security for light sleep in case GPS is not answering
            device->lightSleep();
        }
    }
    uart_flush(UART2_PORT_NUMBER);
    free(uart2Data);
    if(config->blinkLeds) { device->ledGreenOff(); device->ledRedOff(); }
    if(updatedRTC) { return GPS_FIX_SUCCESS_AND_RTC_UPDATED; }
    return GPS_FIX_SUCCESS_NO_RTC_UPDATE;
}

bool GPS_L70_REV6::updateUTCTimestamp(esp_gps_t *gpsData) {
	if(gpsData == NULL) { gpsData->parent.utcTimestamp = 0; return false; }
	if(gpsData->parent.date.year == 80) { // could not get time yet (still 1980)
        gpsData->parent.utcTimestamp = 0;
        return false;
    }

    uint16_t year = gpsData->parent.date.year;
    year += 2000;
    gpsData->parent.utcTimestamp = _UNIX_TIMESTAMP(year, gpsData->parent.date.month, gpsData->parent.date.day, gpsData->parent.tim.hour, gpsData->parent.tim.minute, gpsData->parent.tim.second);
    return true;
}

bool GPS_L70_REV6::waitForAnswer(const char* answer, uint16_t timeousMs) {
    // WARNING: DO NOT DO ANY PRINTFs HERE!
	if(queueHandle == NULL) { return false; }
	bool foundAnswer = false;
	uart_event_t event;
	size_t bufferedSize;
	uint8_t *uart2Data = (uint8_t*) malloc(UART2_RX_BUFFER);
    if(uart2Data == NULL) { return false; }
	uint32_t timeStart = (uint32_t) (Timing::millis());
	while(true) {
		if(xQueueReceive(*queueHandle, (void * )&event, pdMS_TO_TICKS(100))) { // wait max. 100ms
			if(event.type == UART_PATTERN_DET) { // '\n' detected
                bzero(uart2Data, UART2_RX_BUFFER);
				uart_get_buffered_data_len(UART2_PORT_NUMBER, &bufferedSize);
				int pos = uart_pattern_pop_pos(UART2_PORT_NUMBER);
				//printf("[UART PATTERN DETECTED] pos: %d, buffered size: %d\n", pos, bufferedSize);
				if(pos == -1) { // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not record the position. We should set a larger queue size, as an example, we directly flush the rx buffer here
					//printf("ERROR TOO SLOW\n");
					uart_flush_input(UART2_PORT_NUMBER);
					break;
				}
                else {
					int read_len = uart_read_bytes(UART2_PORT_NUMBER, uart2Data, pos + 1, 100 / portTICK_PERIOD_MS); // pos+1 to also read the pattern (\n)
					uart2Data[read_len] = '\0'; // make sure the line is a standard string
					char *uart2DataChar = (char *) uart2Data;
					//printf("Read (strlen: %d): %s \n", strlen(uart2DataChar), uart2DataChar);
					//printf("Compare to (strlen: %d): %s \n", strlen(answer), answer);
					if(strcmp(uart2DataChar, answer) == 0) { // 0 = strings are equal
						foundAnswer = true;
                        break;
					}
				}
			}
		}
        if(((uint32_t) (Timing::millis())) - timeStart > timeousMs) { break; }
	}
	free(uart2Data);
    uart_flush(UART2_PORT_NUMBER); // empty the RX buffer
	return foundAnswer;
}

bool GPS_L70_REV6::isStarted() {
	if(!waitForAnswer("$PMTK010,001*2E\r\n", 2000)) { return false; } // system startup!
	return true;    
}

bool GPS_L70_REV6::setBaudrate115200() {
	const char* command = "$PMTK251,115200*1F\r\n";
	uart_write_bytes(UART2_PORT_NUMBER, command, strlen(command));
	// TODO: wait for answer
	return true;
}

void GPS_L70_REV6::permanentlySetBaudrate115200() {
	const char* command = "$PQBAUD,W,115200*43\r\n";
	uart_write_bytes(UART2_PORT_NUMBER, command, strlen(command));
    Timing::delay(2000); // only waiting because waitForAnswer doesn't work here (better turn off/on)
	//uart_set_baudrate(UART2_PORT_NUMBER, 115200);
	//uart_flush(UART2_PORT_NUMBER);
	//if(!waitForAnswer("$PQBAUD,W,OK*40\r\n", 2000)) { return false; }
}

bool GPS_L70_REV6::setFLPMode(bool permanently) {
	const char* commandPermanently = "$PQFLP,W,1,1*20\r\n";
    const char* commandOnce = "$PQFLP,W,1,0*21\r\n";
    if(permanently) {
        uart_write_bytes(UART2_PORT_NUMBER, commandPermanently, strlen(commandPermanently));
    }
    else {
        uart_write_bytes(UART2_PORT_NUMBER, commandOnce, strlen(commandOnce));
    }
    if(!waitForAnswer("$PQFLP,W,OK*08\r\n", 2000)) { return false; }
    return true;
}

bool GPS_L70_REV6::setFLPMode2() {
	const char* command = "$PMTK262,1*29\r\n";
    uart_write_bytes(UART2_PORT_NUMBER, command, strlen(command));
    if(!waitForAnswer("$PMTK001,262,3,1*2B\r\n", 2000)) { return false; }
    return true;
}

bool GPS_L70_REV6::setNormalMode(bool permanently) {
	const char* commandPermanently = "$PQFLP,W,0,1*21\r\n";
    const char* commandOnce = "$PQFLP,W,0,0*20\r\n";
    if(permanently) {
        uart_write_bytes(UART2_PORT_NUMBER, commandPermanently, strlen(commandPermanently));
    }
    else {
        uart_write_bytes(UART2_PORT_NUMBER, commandOnce, strlen(commandOnce));
    }
    if(!waitForAnswer("$PQFLP,W,OK*08\r\n", 2000)) { return false; }
    return true;
}

bool GPS_L70_REV6::setNMEAMessagesMinimum() {
	const char* command = "$PMTK314,0,2,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; // GPRMC, GPGGA
	uart_write_bytes(UART2_PORT_NUMBER, command, strlen(command));
    if(!waitForAnswer("$PMTK001,314,3*36\r\n", 2000)) { return false; }
	return true;
}

bool GPS_L70_REV6::setNMEAMessagesMinimum1Hz() {
	const char* command = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; // GPRMC, GPGGA
	uart_write_bytes(UART2_PORT_NUMBER, command, strlen(command));
    if(!waitForAnswer("$PMTK001,314,3*36\r\n", 2000)) { return false; }
	return true;
}

bool GPS_L70_REV6::setNMEAMessagesMinimum1HzWithZDA() {
	const char* command = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*29\r\n"; // GPRMC, GPGGA, ZDA
	uart_write_bytes(UART2_PORT_NUMBER, command, strlen(command));
    if(!waitForAnswer("$PMTK001,314,3*36\r\n", 2000)) { return false; }
	return true;
}

float GPS_L70_REV6::parseLatLong(esp_gps_t *esp_gps) {
    float ll = strtof(esp_gps->item_str, NULL);
    int deg = ((int)ll) / 100;
    float min = ll - (deg * 100);
    ll = deg + min / 60.0f;
    return ll;
}

uint8_t GPS_L70_REV6::convertTwoDigit2Number(const char *digit_char) {
    return 10 * (digit_char[0] - '0') + (digit_char[1] - '0');
}

void GPS_L70_REV6::parseUTCTime(esp_gps_t *esp_gps) {
    esp_gps->parent.tim.hour = convertTwoDigit2Number(esp_gps->item_str + 0);
    esp_gps->parent.tim.minute = convertTwoDigit2Number(esp_gps->item_str + 2);
    esp_gps->parent.tim.second = convertTwoDigit2Number(esp_gps->item_str + 4);
    if(esp_gps->item_str[6] == '.') {
        uint16_t tmp = 0;
        uint8_t i = 7;
        while (esp_gps->item_str[i]) {
            tmp = 10 * tmp + esp_gps->item_str[i] - '0';
            i++;
        }
        esp_gps->parent.tim.thousand = tmp;
    }
}

void GPS_L70_REV6::parseGGA(esp_gps_t *esp_gps) {
    /* Process GGA statement */
    switch (esp_gps->item_num) {
    case 1: /* Process UTC time */
        parseUTCTime(esp_gps);
        break;
    case 2: /* Latitude */
        esp_gps->parent.latitude = parseLatLong(esp_gps);
        break;
    case 3: /* Latitude north(1)/south(-1) information */
        if(esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's') {
            esp_gps->parent.latitude *= -1;
        }
        break;
    case 4: /* Longitude */
        esp_gps->parent.longitude = parseLatLong(esp_gps);
        break;
    case 5: /* Longitude east(1)/west(-1) information */
        if(esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w') {
            esp_gps->parent.longitude *= -1;
        }
        break;
    case 6: /* Fix status */
        esp_gps->parent.fix = (gps_fix_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 7: /* Satellites in use */
        esp_gps->parent.sats_in_use = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 8: /* HDOP */
        esp_gps->parent.dop_h = strtof(esp_gps->item_str, NULL);
        break;
    case 9: /* Altitude */
        esp_gps->parent.altitude = strtof(esp_gps->item_str, NULL);
        break;
    case 11: /* Altitude above ellipsoid */
        esp_gps->parent.altitude += strtof(esp_gps->item_str, NULL);
        break;
    default:
        break;
    }
}

void GPS_L70_REV6::parseGSA(esp_gps_t *esp_gps) {
    /* Process GSA statement */
    switch (esp_gps->item_num) {
    case 2: /* Process fix mode */
        esp_gps->parent.fix_mode = (gps_fix_mode_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 15: /* Process PDOP */
        esp_gps->parent.dop_p = strtof(esp_gps->item_str, NULL);
        break;
    case 16: /* Process HDOP */
        esp_gps->parent.dop_h = strtof(esp_gps->item_str, NULL);
        break;
    case 17: /* Process VDOP */
        esp_gps->parent.dop_v = strtof(esp_gps->item_str, NULL);
        break;
    default:
        /* Parse satellite IDs */
        if(esp_gps->item_num >= 3 && esp_gps->item_num <= 14) {
            esp_gps->parent.sats_id_in_use[esp_gps->item_num - 3] = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        }
        break;
    }
}

void GPS_L70_REV6::parseGSV(esp_gps_t *esp_gps) {
    /* Process GSV statement */
    switch (esp_gps->item_num) {
    case 1: /* total GSV numbers */
        esp_gps->sat_count = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 2: /* Current GSV statement number */
        esp_gps->sat_num = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 3: /* Process satellites in view */
        esp_gps->parent.sats_in_view = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    default:
        if(esp_gps->item_num >= 4 && esp_gps->item_num <= 19) {
            uint8_t item_num = esp_gps->item_num - 4; /* Normalize item number from 4-19 to 0-15 */
            uint8_t index;
            uint32_t value;
            index = 4 * (esp_gps->sat_num - 1) + item_num / 4; /* Get array index */
            if(index < GPS_MAX_SATELLITES_IN_VIEW) {
                value = strtol(esp_gps->item_str, NULL, 10);
                switch (item_num % 4) {
                case 0:
                    esp_gps->parent.sats_desc_in_view[index].num = (uint8_t)value;
                    break;
                case 1:
                    esp_gps->parent.sats_desc_in_view[index].elevation = (uint8_t)value;
                    break;
                case 2:
                    esp_gps->parent.sats_desc_in_view[index].azimuth = (uint16_t)value;
                    break;
                case 3:
                    esp_gps->parent.sats_desc_in_view[index].snr = (uint8_t)value;
                    break;
                default:
                    break;
                }
            }
        }
        break;
    }
}

void GPS_L70_REV6::parseRMC(esp_gps_t *esp_gps) {
    /* Process GPRMC statement */
    switch (esp_gps->item_num) {
    case 1:/* Process UTC time */
        parseUTCTime(esp_gps);
        break;
    case 2: /* Process valid status */
        esp_gps->parent.valid = (esp_gps->item_str[0] == 'A');
        break;
    case 3:/* Latitude */
        esp_gps->parent.latitude = parseLatLong(esp_gps);
        break;
    case 4: /* Latitude north(1)/south(-1) information */
        if(esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's') {
            esp_gps->parent.latitude *= -1;
        }
        break;
    case 5: /* Longitude */
        esp_gps->parent.longitude = parseLatLong(esp_gps);
        break;
    case 6: /* Longitude east(1)/west(-1) information */
        if(esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w') {
            esp_gps->parent.longitude *= -1;
        }
        break;
    case 7: /* Process ground speed in unit m/s */
        esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) * 1.852;
        break;
    case 8: /* Process true course over ground */
        esp_gps->parent.cog = strtof(esp_gps->item_str, NULL);
        break;
    case 9: /* Process date */
        esp_gps->parent.date.day = convertTwoDigit2Number(esp_gps->item_str + 0);
        esp_gps->parent.date.month = convertTwoDigit2Number(esp_gps->item_str + 2);
        esp_gps->parent.date.year = convertTwoDigit2Number(esp_gps->item_str + 4);
        break;
    case 10: /* Process magnetic variation */
        esp_gps->parent.variation = strtof(esp_gps->item_str, NULL);
        break;
    default:
        break;
    }
}

void GPS_L70_REV6::parseGLL(esp_gps_t *esp_gps) {
    /* Process GPGLL statement */
    switch (esp_gps->item_num) {
    case 1:/* Latitude */
        esp_gps->parent.latitude = parseLatLong(esp_gps);
        break;
    case 2: /* Latitude north(1)/south(-1) information */
        if(esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's') {
            esp_gps->parent.latitude *= -1;
        }
        break;
    case 3: /* Longitude */
        esp_gps->parent.longitude = parseLatLong(esp_gps);
        break;
    case 4: /* Longitude east(1)/west(-1) information */
        if(esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w') {
            esp_gps->parent.longitude *= -1;
        }
        break;
    case 5:/* Process UTC time */
        parseUTCTime(esp_gps);
        break;
    case 6: /* Process valid status */
        esp_gps->parent.valid = (esp_gps->item_str[0] == 'A');
        break;
    default:
        break;
    }
}

void GPS_L70_REV6::parseVTG(esp_gps_t *esp_gps) {
    /* Process GPVGT statement */
    switch (esp_gps->item_num) {
    case 1: /* Process true course over ground */
        esp_gps->parent.cog = strtof(esp_gps->item_str, NULL);
        break;
    case 3:/* Process magnetic variation */
        esp_gps->parent.variation = strtof(esp_gps->item_str, NULL);
        break;
    case 5:/* Process ground speed in unit m/s */
        esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) * 1.852;//knots to m/s
        break;
    case 7:/* Process ground speed in unit m/s */
        esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) / 3.6;//km/h to m/s
        break;
    default:
        break;
    }
}

bool GPS_L70_REV6::gpsDecodeParse(esp_gps_t *esp_gps) {
    /* start of a statement */
	bool error = false;
    if(esp_gps->item_num == 0 && esp_gps->item_str[0] == '$') {
        if(strstr(esp_gps->item_str, "GGA")) { esp_gps->cur_statement = STATEMENT_GGA; }
        else if(strstr(esp_gps->item_str, "GSA")) { esp_gps->cur_statement = STATEMENT_GSA; }
        else if(strstr(esp_gps->item_str, "RMC")) { esp_gps->cur_statement = STATEMENT_RMC; }
        else if(strstr(esp_gps->item_str, "GSV")) { esp_gps->cur_statement = STATEMENT_GSV; }
        else if(strstr(esp_gps->item_str, "GLL")) { esp_gps->cur_statement = STATEMENT_GLL; }
        else if(strstr(esp_gps->item_str, "VTG")) { esp_gps->cur_statement = STATEMENT_VTG; }
        else { esp_gps->cur_statement = STATEMENT_UNKNOWN; }
        goto out;
    }
    /* Parse each item, depend on the type of the statement */
    if(esp_gps->cur_statement == STATEMENT_UNKNOWN) {
        goto out;
    }
    else if(esp_gps->cur_statement == STATEMENT_GGA) {
        parseGGA(esp_gps);
    }
    else if(esp_gps->cur_statement == STATEMENT_GSA) {
        parseGSA(esp_gps);
    }
    else if(esp_gps->cur_statement == STATEMENT_GSV) {
        parseGSV(esp_gps);
    }
    else if(esp_gps->cur_statement == STATEMENT_RMC) {
        parseRMC(esp_gps);
    }
    else if(esp_gps->cur_statement == STATEMENT_GLL) {
        parseGLL(esp_gps);
    }
    else if(esp_gps->cur_statement == STATEMENT_VTG) {
        parseVTG(esp_gps);
    }
    else {
        error =  true;
    }
out:
    return !error;
}

get_decode_result_t GPS_L70_REV6::gpsDecodeLine(esp_gps_t *esp_gps, char *d, uint32_t len) {
    while(*d) {
        /* Start of a statement */
        if(*d == '$') {
            /* Reset runtime information */
            esp_gps->asterisk = 0;
            esp_gps->item_num = 0;
            esp_gps->item_pos = 0;
            esp_gps->cur_statement = 0;
            esp_gps->crc = 0;
            esp_gps->sat_count = 0;
            esp_gps->sat_num = 0;
            /* Add character to item */
            esp_gps->item_str[esp_gps->item_pos++] = *d;
            esp_gps->item_str[esp_gps->item_pos] = '\0';
        }
        /* Detect item separator character */
        else if(*d == ',') {
            /* Parse current item */
            gpsDecodeParse(esp_gps);
            /* Add character to CRC computation */
            esp_gps->crc ^= (uint8_t)(*d);
            /* Start with next item */
            esp_gps->item_pos = 0;
            esp_gps->item_str[0] = '\0';
            esp_gps->item_num++;
        }
        /* End of CRC computation */
        else if(*d == '*') {
            /* Parse current item */
            gpsDecodeParse(esp_gps);
            /* Asterisk detected */
            esp_gps->asterisk = 1;
            /* Start with next item */
            esp_gps->item_pos = 0;
            esp_gps->item_str[0] = '\0';
            esp_gps->item_num++;
        }
        /* End of statement */
        else if(*d == '\r') {
            /* Convert received CRC from string (hex) to number */
            uint8_t crc = (uint8_t) strtol(esp_gps->item_str, NULL, 16);
            /* CRC passed */
            if(esp_gps->crc == crc) {
                switch (esp_gps->cur_statement) {
                case STATEMENT_GGA:
                    esp_gps->parsed_statement |= 1 << STATEMENT_GGA;
                    break;
                case STATEMENT_GSA:
                    esp_gps->parsed_statement |= 1 << STATEMENT_GSA;
                    break;
                case STATEMENT_RMC:
                    esp_gps->parsed_statement |= 1 << STATEMENT_RMC;
                    break;
                case STATEMENT_GSV:
                    if(esp_gps->sat_num == esp_gps->sat_count) {
                        esp_gps->parsed_statement |= 1 << STATEMENT_GSV;
                    }
                    break;
                case STATEMENT_GLL:
                    esp_gps->parsed_statement |= 1 << STATEMENT_GLL;
                    break;
                case STATEMENT_VTG:
                    esp_gps->parsed_statement |= 1 << STATEMENT_VTG;
                    break;
                default:
                    break;
                }
                /* Check if all statements have been parsed */
                if(((esp_gps->parsed_statement) & esp_gps->all_statements) == esp_gps->all_statements) {
                    esp_gps->parsed_statement = 0;
                }
            }
			else { // CRC error
				return GPS_DECODE_RESULT_CRC_ERR;
            }
            if(esp_gps->cur_statement == STATEMENT_UNKNOWN) {
				return GPS_DECODE_RESULT_UNKNOWN_STATEMENT_ERR;
            }
        }
        /* Other non-space character */
        else {
            if(!(esp_gps->asterisk)) {
                /* Add to CRC */
                esp_gps->crc ^= (uint8_t)(*d);
            }
            /* Add character to item */
            esp_gps->item_str[esp_gps->item_pos++] = *d;
            esp_gps->item_str[esp_gps->item_pos] = '\0';
        }
        /* Process next character */
        d++;
    }
    return GPS_DECODE_RESULT_SUCESS;
}

bool GPS_L70_REV6::gpsDecodeCorruptedGPRMCLine(esp_gps_t *esp_gps, const char* message) {
	//???????,,,,0.00,0.00,050180,,,N*4C\r\n (should: $GPRMC,235942.800,V,,,,,0.00,0.00,050180,,,N*42) -> around 20 characters lost at start
	char buffer[12];
	uint8_t bufferPnt = 0;
	memset(buffer, 0, 12);
	uint8_t cntStars = 0;
	uint8_t cntCommas = 0;
	for(int32_t i = strlen(message) - 1; i >= 0; i--) { // iterate in reverse order
	    if(message[i] == '*') { cntStars++; }
		else if(message[i] == ',') { cntCommas++; }
		if((cntStars == 1) && (cntCommas == 3) && (message[i] != ',')) { // only then start writing to buffer
			if(bufferPnt < 6) { buffer[bufferPnt++] = message[i]; }
		}
		if((cntCommas > 3) || (cntStars > 1)) { break; } // don't look any further, stop when next comma reached
	}
	if(strlen(buffer) == 6) {
		char temp;
		temp = buffer[0]; buffer[0] = buffer[5]; buffer[5] = temp; // reverse again
		temp = buffer[1]; buffer[1] = buffer[4]; buffer[4] = temp; // reverse again
		temp = buffer[2]; buffer[2] = buffer[3]; buffer[3] = temp; // reverse again
		//printf("Len: %d, StrLen: %d, Str: %s\n", bufferPnt, strlen(buffer), buffer);
        esp_gps->parent.date.day = convertTwoDigit2Number(&buffer[0]);
        esp_gps->parent.date.month = convertTwoDigit2Number(&buffer[2]);
        esp_gps->parent.date.year = convertTwoDigit2Number(&buffer[4]);
		//printf("%d.%d.%d\n", esp_gps->parent.date.day, esp_gps->parent.date.month, esp_gps->parent.date.year);
		return true;
	}
	return false;	
}
