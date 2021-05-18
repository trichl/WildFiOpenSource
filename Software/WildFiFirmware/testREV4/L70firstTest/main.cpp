#include "ESP32TrackerREV4.h"
#include "GPS_L70.h"

ESP32TrackerREV4 device = ESP32TrackerREV4();
GPS_L70 gps = GPS_L70();

RTC_DATA_ATTR bool firstStart = true;
RTC_DATA_ATTR uint64_t bootCnt = 0;
uint32_t ttffStart = 0;

uint32_t currentTime() {
    int64_t temp = esp_timer_get_time();
    temp = temp / 1000; // ms
    uint32_t timeNow = (uint32_t) temp;
    return timeNow;
}

uint32_t ttffTimeRunning() {
    return (currentTime() - ttffStart);
}
 
extern "C" void app_main() {
    while(1) {
        //device.setCPUSpeed(ESP32_10MHZ);
        if(firstStart) {
            device.disableWakeStubNoBootIfVoltageLow();
            firstStart = false;
            printf("FIRST START\n");
            device.delay(6000);

            device.gpioBOn();
            device.ledRedOn();

            ttffStart = currentTime();

            printf("%d PREPARE UART..\n", ttffTimeRunning());
            device.uart2Init(115200);
            gps.init(device.uart2GetQueue());
            device.uart2EnablePatternInterrupt('\n');

            //device.uart2InterruptPrint(20);
            if(!gps.isStarted()) { printf("ERROR NOT STARTED\n"); }
            else { printf("%d GPS RECEIVED: STARTED\n", ttffTimeRunning()); }

            if(!gps.setNMEAMessagesMinimum()) { printf("ERROR DID NOT RECEIVE FEEDBACK\n"); }
            else { printf("%d GPS RECEIVED: CONFIRMED NMEAs\n", ttffTimeRunning()); }
            //device.uart2InterruptPrint(200); // TEST

            uart_flush(UART2_PORT_NUMBER);
            printf("SLEEP\n");
            device.delay(20);
        }
        else {
            //printf("%d AWAKE\n", ttffTimeRunning());
            bool stopListening = false;
            uart_event_t event;
            size_t bufferedSize;
            uint8_t *uart2Data = (uint8_t*) malloc(UART2_RX_BUFFER);
            while(!stopListening) {
                if(xQueueReceive(*(device.uart2GetQueue()), (void * )&event, 0)) { // TEST
                    bzero(uart2Data, UART2_RX_BUFFER);
                    if(event.type == UART_PATTERN_DET) {
                        uart_get_buffered_data_len(UART2_PORT_NUMBER, &bufferedSize);
                        int pos = uart_pattern_pop_pos(UART2_PORT_NUMBER);
                        //printf("[UART PATTERN DETECTED] pos: %d, buffered size: %d\n", pos, bufferedSize);
                        if(pos == -1) {
                            printf("ERROR TOO SLOW\n");
                            uart_flush_input(UART2_PORT_NUMBER);
                        }
                        else {
                            int read_len = uart_read_bytes(UART2_PORT_NUMBER, uart2Data, pos + 1, 100 / portTICK_PERIOD_MS); // pos+1 to also read the pattern (\n)
					        uart2Data[read_len] = '\0'; // make sure the line is a standard string
					        char *uart2DataChar = (char *) uart2Data;
                            
                            //gps_decode(esp_gps, read_len + 1)
                            if((uart2DataChar[0] == '$') && (uart2DataChar[1] == 'G') && (uart2DataChar[2] == 'P') && (uart2DataChar[3] == 'G')) {
                                stopListening = true;
                                esp_gps_t gpsData = { };
                                if(!gps.gpsDecodeLine(&gpsData, uart2Data, read_len)) { printf("decode error\n"); }
                                else {
                                    //printf("%s", uart2DataChar);
                                    printf("%ds: %d:%d:%d, LAT: %f, LON: %f, SATS: %d, FIX: %d\n", ttffTimeRunning()/1000, gpsData.parent.tim.hour, gpsData.parent.tim.minute, gpsData.parent.tim.second, gpsData.parent.latitude, gpsData.parent.longitude, gpsData.parent.sats_in_use, gpsData.parent.fix);
                                    if((gpsData.parent.latitude > 1.0) || (gpsData.parent.latitude < -1.0)) {
                                        printf("---- FIX AFTER %d SECONDS ----\n", ttffTimeRunning()/1000);
                                        device.ledRedOff();
                                        device.ledGreenOn();
                                    }
                                    else if(gpsData.parent.sats_in_use > 0) {
                                        device.ledRedOn();
                                        device.ledGreenOn();
                                    }
                                }
                            }
                        }
                    }
                }
            }
            free(uart2Data);
            device.delay(20);
            uart_flush(UART2_PORT_NUMBER);
        }
        bootCnt++;

        /*PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA2_U, FUNC_SD_DATA2_U1RXD); // GPIO9 should be configured as function_5
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA3_U, FUNC_SD_DATA3_U1TXD);
        if(uart_set_wakeup_threshold(UART2_PORT_NUMBER, 3) != ESP_OK) { printf("ERROR10\n"); }
        if(esp_sleep_enable_uart_wakeup(UART2_PORT_NUMBER) != ESP_OK) { printf("ERROR11\n"); }*/
        gpio_wakeup_enable(PIN_RXD2, GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
        //esp_sleep_enable_timer_wakeup(1100000ULL); // reduces light sleep current
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); // +1uA, internal pull-ups and pull-downs and ULP
        device.lightSleep();
    }
}
