#include "ESP32TrackerREV4.h"

ESP32TrackerREV4 device = ESP32TrackerREV4();

// 200Hz, interrupt @900:
    // wake stub 2.31uWh
    // after boot @10MHZ: 2.46uWh (948)
    // after boot @80MHz: 1.67uWh (936)
        // with readAccFIFOInOneGoFast: 1.59uWh
            // on unicore: 1.38uWh

// readAccFIFOInOneGoFast: 14ms instead of 16ms

#define ACC_INTERRUPT_WATERMARK                         900
#define ACC_RAM_SIZE_1                                  996  // (ACC_INTERRUPT_WATERMARK + 12), ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2 = RTC memory for storing fifo data in wake stub
#define ACC_RAM_SIZE_2                                  4                               // see above

RTC_DATA_ATTR uint8_t fifoDataRam[ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2] = { 0 }; // RTC_FAST_ATTR = PRO CPU ONLY! -> NO SINGLE CORE MODE POSSIBLE
RTC_DATA_ATTR uint16_t fifoDataPointerRam = 0; // RTC_FAST_ATTR = PRO CPU ONLY! -> NO SINGLE CORE MODE POSSIBLE
uint16_t fifoLen = 0;

RTC_DATA_ATTR uint16_t startCnt = 0;

acc_config_t accConfig = {
    BMX160_ACCEL_ODR_200HZ,
    BMX160_ACCEL_BW_RES_AVG2,
    BMX160_ACCEL_RANGE_2G
};

static void eventHandlerWiFi2(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) { // EVENT WIFI SCAN DONE
        //wiFiScanDone = true;
		//wiFiScanRunning = false;
		printf("WIFI SCAN DONE!\n");
    }
	else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) { // EVENT STA STARTED AND WANT TO CONNECT TO WIFI OR SCANNING WIFIS
		printf("WIFI STARTED!\n");
		/*if(!wiFiScanRunning) { // event is also called when only scanning for wifis -> connect attempt will not work, also called when AP not found
			//printf("TRY TO CONNECT TO WIFI!\n");
			esp_wifi_connect(); // try to connect to wifi
		}*/
    }
	else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) { // DISCONNECTED FROM WIFI, also called before sleep, ALSO CALLED WHEN AP NOT FOUND!!!
        printf("DISCONNECTED\n");
    } else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { // CONNECTED TO WIFI
		printf("GOT IP");
    }
}

static void wifi_scan(void) {
    // initWiFi
    // INIT NVS
	if(!device.initNVS()) {
		printf("ERROR\n");
	}

    ESP_ERROR_CHECK(esp_netif_init()); // SAME
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // SAME
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta(); // SAME
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // SAME
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // SAME
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // SAME
    
    if(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandlerWiFi2, NULL) != ESP_OK) { // register function for events of type WIFI_EVENT (for scan)
		printf("ERRORX\n");
	}
	if(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandlerWiFi2, NULL) != ESP_OK) { // register function for event if connected to certain wifi
		printf("ERRORY\n");
	}


    // scanForWiFis
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    printf("esp_wifi_start\n");
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("esp_wifi_scan_start\n");
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
    printf("esp_wifi_scan_get_ap_records\n");
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    printf("esp_wifi_scan_get_ap_num\n");
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    printf("Total APs scanned = %u\n", ap_count);
    for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
        printf("SSID \t\t%s", ap_info[i].ssid);
        printf("RSSI \t\t%d", ap_info[i].rssi);
        printf("Channel \t\t%d\n", ap_info[i].primary);
    }

    device.disconnectAndStopWiFi();

}

extern "C" void app_main() {
    while(1) {
        //device.setCPUSpeed(ESP32_10MHZ);
        if(startCnt == 0) {
            device.disableWakeStubNoBootIfVoltageLow();


            if(!device.initWiFi()) { printf("..FAILED (init)\n"); }
            if(!device.scanForWiFis(true, RADIO_MAX_TX_19_5_DBM, 60, WIFI_ALL_14_CHANNELS))  { printf("..FAILED (scan)\n"); }
            printf("..DONE\n");
            device.printWiFiScanResults();
            device.disconnectAndStopWiFi();
            //wifi_scan();


            /*
            device.sensorPowerOn(true);
            device.keepSensorPowerOnInDeepSleep();
            device.delay(100);
            if(!device.imu.start(&accConfig, NULL, NULL, BMX160_LATCH_DUR_5_MILLI_SEC)) { printf("ERROR1\n"); }
            if(!device.imu.enableFIFOInterrupt(ACC_INTERRUPT_WATERMARK)) { printf("ERROR2\n"); }
            if(!device.imu.initFIFOForAcc()) { printf("ERROR3\n"); }
            if(!device.imu.resetFIFO()) { printf("ERROR4\n"); }
            if(!device.rtc.disableClockOut()) { printf("ERROR5\n"); }
            */  
        }
        else {
            /*i2c.begin(I2C_FREQ_HZ_1MHZ);
            
            fifoLen = device.imu.getFIFOLength();
            if((fifoDataPointerRam + fifoLen) <= (ACC_RAM_SIZE_1 * ACC_RAM_SIZE_2)) { // storing fifo would exceed ACC array in RTC memory
                //uint64_t timeMeas = Arduino::millisWrapper();
                if(!device.imu.readAccFIFOInOneGoFast(fifoDataRam+fifoDataPointerRam, fifoLen, false)) { printf("ERROR"); }
                fifoDataPointerRam += fifoLen;
                //timeMeas = Arduino::millisWrapper() - timeMeas;
                //printf("Fifo read in %llu ms, length %d\n", timeMeas, fifoLen);
            }
            else {
                printf("STOP READING HERE, fifoDataPointerRam = %d\n", fifoDataPointerRam);
                device.imu.resetFIFO();
                device.imu.stop();
                for(uint16_t i=0; i<fifoDataPointerRam; i++) {
                    printf("%02X ", fifoDataRam[i]);
                    if(i % 6 == 5) { printf("\n"); }
                }
            }*/
        }
    
        startCnt++;
        device.enableAccInterruptInDeepSleep();
        device.deepSleep();
    }
}