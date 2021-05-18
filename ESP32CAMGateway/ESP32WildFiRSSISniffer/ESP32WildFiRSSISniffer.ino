// SELECT ESP32 WROVER MODULE, HUGE APP NO OTA
#define GATEWAY_FOR_MOVEMENT_LOGGER                     0
#define GATEWAY_FOR_PROXIMITY_DETECTION                 1
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <WiFi.h>
#include "time.h"
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include <esp_wifi_internal.h> // for modifying default transmission speed of 1Mbps
#include "SPIFFS.h"
#include "soc/soc.h"           // disable brownout problems
#include "soc/rtc_cntl_reg.h"  // disable brownout problems
#include "driver/rtc_io.h"

/** ------ SETTINGS ------ */

const char* ssid                                        = "RodelbahnSoelden";
const char* password                                    = "xxxxxxxx";
const char* ntpServer                                   = "pool.ntp.org";

/** ------ DEFINES & STRUCTS ------ */

#define SEND_FAKE_DATA_MESSAGES                         true

#define PIN_LED_RED                                     33
#define RX_QUEUE_SIZE                                   1000 // 576 = 1 memory block of esp32

/** ------ MESSAGE STRUCTURE FOR MOVEMENT LOGGER ------ */

#define ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE            37  // 1 byte, IMPORTANT, 0x04 = ESP NOW!
#define ESPNOW_META_MSG_GATEWAY_AROUND                  0x66
#define ESPNOW_FRAME_OFFSET_SENDER_MAC                  10 // 6 bytes
#define ESPNOW_FRAME_OFFSET_PAYLOAD                     39

#define PHY_RATE                                        WIFI_PHY_RATE_18M
#define LONG_RANGE                                      0


typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    uint8_t data_len;
    int32_t rssi;
    uint32_t phy_rate;
} queue_entry_t;

/** ------ GLOBAL VARIABLES ------ */

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long lastMillisReceived = 0;
unsigned long lastMillisPrintStatus = 0;
unsigned long lastMillisGatewayMsg = 0;
int32_t lastRssi = 0;
uint32_t lastPhyRate = 0;
uint32_t errorCounter = 0;
uint64_t promCounter = 0;
uint64_t sendCounter = 0;

uint8_t ownMac[6] = { 0 };
long lastMsgMillis = 0;
bool receiveRunning = false;
static xQueueHandle rxQueue;
bool wifiStarted = true;
RTC_DATA_ATTR uint8_t stateGetTime = 0;
uint64_t totalAmountReceivedData = 0;
uint32_t newDataCounter = 0;

bool receivedGatewayAroundMsg = false;
uint8_t gatewayAroundEspNowMac[6] = { 0 };
uint8_t espNowMsgSuccess = 0;

/** ------ VARIOUS FUNCTIONS ------ */

void printLogo() {
  Serial.println("----------------------------");
  Serial.println("---WILDFI-TAG-SNIFFER-V1---");
  Serial.println("---SW: by Timm Richlick---");
  Serial.println("----------------------------");
}

void blinkRedLed(int cnt) {
  for(int i=0; i<cnt; i++) {
    digitalWrite(PIN_LED_RED, LOW); // on
    delay(200);
    digitalWrite(PIN_LED_RED, HIGH); // off
    delay(200);   
  }
}

void printQueueEntry(queue_entry_t *queueEntry) {
  struct tm timeinfo;
  getLocalTime(&timeinfo);      
  Serial.printf("%02d:%02d:%02d: %02X:%02X:%02X:%02X:%02X:%02X %dB, RSSI %d, PHY %d\n",
    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
    queueEntry->mac[0], queueEntry->mac[1], queueEntry->mac[2], queueEntry->mac[3], queueEntry->mac[4], queueEntry->mac[5],
    queueEntry->data_len, queueEntry->rssi, queueEntry->phy_rate);
}

/** ------ GET TIME OVER WIFI ------ */

bool printTime() {
  struct tm timeinfo;
  bool success = true;
  if(!getLocalTime(&timeinfo)){
    Serial.println("TIME: failed to obtain time");
    success = false;
  }
  timeinfo.tm_mon++; // starts at 0
  Serial.printf("TIME: %d.%d.%d %d:%d:%d\n", timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year + 1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  return success;
}

bool getTimeOverWifi() {
  Serial.println("TIME: started");
  WiFi.begin(ssid, password);
  uint16_t timeout = 0;
  while(WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
      timeout++;
      if(timeout > 8) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        Serial.println("");
        return false;
      }
  }
  Serial.println("\nTIME: connected to wifi");
  
  configTime(0, 0, ntpServer);

  bool gotTime = printTime();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  return gotTime;
}

/** ------ ESP NOW ------ */

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.println("SENDING");
  if(status == ESP_NOW_SEND_SUCCESS) { espNowMsgSuccess = 1; }
  else { espNowMsgSuccess = 2; }   
}

void sendTestMessage() {
    uint8_t data[250] = { 0 }; // all zero
    data[0] = 0xFF;
    data[1] = 0xBB;
    data[2] = 0xCC;
    if(esp_now_send(gatewayAroundEspNowMac, data, 250) != ESP_OK) { printf("SEND ERROR\n"); errorCounter++; }
}

void wifiPromiscuous(void* buffer, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
  if(type == WIFI_PKT_MGMT) { // ESP NOW uses management frame
    if(p->payload[ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE] == 0x04) { // is ESP NOW frame
      lastRssi = p->rx_ctrl.rssi;
      lastPhyRate = p->rx_ctrl.rate;
      promCounter++;
      //Serial.printf("%d\n", millis());

      if(SEND_FAKE_DATA_MESSAGES) {
          if((p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + 0] == ESPNOW_META_MSG_GATEWAY_AROUND)) { // gateway around message
              if(!receivedGatewayAroundMsg) {
                receivedGatewayAroundMsg = true;
                esp_now_peer_info_t peer;
                memset(&peer, 0, sizeof(esp_now_peer_info_t));
                gatewayAroundEspNowMac[0] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+0];
                gatewayAroundEspNowMac[1] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+1];
                gatewayAroundEspNowMac[2] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+2];
                gatewayAroundEspNowMac[3] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+3];
                gatewayAroundEspNowMac[4] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+4];
                gatewayAroundEspNowMac[5] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+5];
                memcpy(peer.peer_addr, gatewayAroundEspNowMac, sizeof(uint8_t[6]));
                peer.ifidx = ESP_IF_WIFI_STA;
                peer.channel = 0;
                peer.encrypt = false;
                if(esp_now_add_peer(&peer) != ESP_OK){
                  Serial.println("GW SEEN -> ERROR PEER ADD");
                }
                else {
                  Serial.println("GW SEEN -> transmit mode");
                }
              }
          } 
      }
    }
  }
}

void onReceiveData(const uint8_t *mac, const uint8_t *data, int len) { // be quick here!
  if(receivedGatewayAroundMsg) { return; }
  
  queue_entry_t queueEntry;
  queueEntry.rssi = lastRssi;
  queueEntry.phy_rate = lastPhyRate;
  if(mac == NULL || data == NULL || len <= 0) {
    Serial.println("E01");
    errorCounter++;
    return;
  }
  
  newDataCounter++;
  lastMsgMillis = millis();
  
  memcpy(queueEntry.mac, mac, ESP_NOW_ETH_ALEN);
  queueEntry.data = (uint8_t *) ps_malloc(len);
  if(queueEntry.data == NULL) {
    Serial.println("E02");
    errorCounter++;
    return;
  }
  memcpy(queueEntry.data, data, len);
  queueEntry.data_len = (uint8_t) len;
  if(uxQueueSpacesAvailable(rxQueue) == 0) {
    Serial.println("E03"); // HAPPENS if too many incoming messages
    errorCounter++;
    free(queueEntry.data);
  }
  else {
    if(xQueueSend(rxQueue, &queueEntry, 0) != pdTRUE) { // portMAX_DELAY = wait forever if queue is full
      Serial.println("E04");
      errorCounter++;
      free(queueEntry.data);
    }
  }
}

static esp_err_t wifiEventHandler(void *ctx, system_event_t *event){
  switch(event->event_id) {
  case SYSTEM_EVENT_STA_START:
    Serial.println("EVENTHANDLER: START");
    wifiStarted = true;
    break;
  case SYSTEM_EVENT_STA_STOP:
    Serial.println("EVENTHANDLER: STOP");
    wifiStarted = false;
  default:
    break;
  }
  return ESP_OK;
}

bool initEspNowCustom() { // code for ESP IDF 3.3, modified code from ESP32-Tracker (using AP mode, not Stationary, means is bursting out every 100ms strange messages)
  esp_log_level_set("ESPNOW", ESP_LOG_NONE); // saves 1ms
  
  tcpip_adapter_init();
  if(esp_event_loop_init(wifiEventHandler, NULL) != ESP_OK) {
    Serial.println("ERROR INIT1");
    return false;
  }
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  cfg.ampdu_tx_enable = 0;
  cfg.nvs_enable = false; // overrides CONFIG_ESP32_WIFI_NVS_ENABLED
  if(esp_wifi_init(&cfg) != ESP_OK) { // 58ms, fails if NVS not initialized
    Serial.println("ERROR INIT2");
    return false;
  }
  if(esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) { // 0ms, different from normal wifi initialization, default is flash (reduces flash lifetime)
    Serial.println("ERROR INIT3");
    return false;
  }
  if(esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) { // 0ms, ACCESS POINT, not STATIONARY, different from normal wifi initialization, soft access point = ESP as hotspot, I guess STATIONARY would also be possible?!
    Serial.println("ERROR INIT100");
    return false;
  }
  if(esp_wifi_start() != ESP_OK) { // 152ms, start the wifi (I think takes more time because CONFIG_ESP32_PHY_CALIBRATION_AND_DATA_STORAGE = false = full calibration on every start?!? don't know)
    Serial.println("ERROR INIT4");
    return false;
  }
  // eventually activate long range mode
  if(LONG_RANGE) { // the LR rate has very limited throughput because the raw PHY data rate LR is 1/2 Mbits and 1/4 Mbits
    if(esp_wifi_set_protocol(ESP_IF_WIFI_AP, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) != ESP_OK) { // here again access point
      return false;
    }
  }
  if(esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, PHY_RATE) != ESP_OK) { // default is 1M
    Serial.println("ERROR INIT5");
    return false;
  }
  // init ESP NOW
  if(esp_now_init() != ESP_OK) { // 1ms, HERE BROWNOUT POSSIBLE
    Serial.println("ERROR INIT6");
    return false;
  }
  while(!wifiStarted) { ; }
  delay(150); // for printf

  esp_now_register_recv_cb(onReceiveData);
  esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuous);
  esp_wifi_set_promiscuous(true);

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  memcpy(peer.peer_addr, broadcastAddress, sizeof(uint8_t[6]));
  peer.ifidx = ESP_IF_WIFI_STA;
  peer.channel = 0;
  peer.encrypt = false;
  
  if(esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("ERROR INIT7");
    return false;
  }
  esp_now_register_send_cb(OnDataSent);

  return true;
}

/** ------ LOOPS & TASKS ------ */

void errorState() {
  Serial.println("ERROR STATE: ERROR HAPPENED -> DO NOT CONTINUE");
  while(1) {
    blinkRedLed(10);
    delay(500);   
  }    
}

void restart() {
  SD_MMC.end(); // important, eject SDHC
  esp_sleep_enable_timer_wakeup(1 * 1000 * 1000);
  esp_deep_sleep_disable_rom_logging(); // no more boot messages -> saves couple of ms
  esp_deep_sleep_start();  
}

void mainTask(void* parameter) {
  bool writeSuccess;
  while(1) {
    if(uxQueueMessagesWaiting(rxQueue) >= 1) {
      receiveRunning = true;
      uint32_t savedMessages = 0;
      long t = millis();
      queue_entry_t queueEntry;
      while(xQueueReceive(rxQueue, &queueEntry, 0) == pdTRUE) {
        printQueueEntry(&queueEntry);
        totalAmountReceivedData += queueEntry.data_len;
        free(queueEntry.data);
        savedMessages++;
      }
      uint32_t timeNeeded = millis() - t;
      receiveRunning = false;
    }
    else delay(10); // otherwise watchdog sometimes triggers

    if(receivedGatewayAroundMsg) {
      uint8_t dataFake[250];
      dataFake[0] = 0xAB; // data
      for(uint8_t i=1; i<250; i++) {
        dataFake[i] = 0xEE;
      }
      Serial.println("-> START ENDLESS TRANSMISSION");
      while(true) {
        espNowMsgSuccess = 0;
        if(esp_now_send(gatewayAroundEspNowMac, dataFake, 250) != ESP_OK) { printf("Send error\n"); break; }
        while(espNowMsgSuccess == 0) {
          delay(1);
        }
        if(espNowMsgSuccess == 2) {
          break;
        }
        else { sendCounter++; }
      }
      receivedGatewayAroundMsg = false;
      // remove peer
      esp_now_del_peer(gatewayAroundEspNowMac);
      Serial.println("-> END OF ENDLESS TRANSMISSION (HAHA)");
    }
  }
}

void setup() {
  // get wake up reason
  esp_sleep_wakeup_cause_t wakeupReason;
  wakeupReason = esp_sleep_get_wakeup_cause();

  // init pins and serial
  Serial.begin(115200);
  pinMode(PIN_LED_RED, OUTPUT);
  digitalWrite(PIN_LED_RED, HIGH); // off
  delay(500);

  // print the logo
  printLogo();
  
  // get time over wifi
  if(wakeupReason == ESP_SLEEP_WAKEUP_TIMER) { Serial.println("INIT: WOKE UP FROM DEEP SLEEP"); } // second start
  else {
    if(getTimeOverWifi()) { stateGetTime = 1; } 
    else { stateGetTime = 2; }
    restart(); // restarts the system (setup starting again)
  }

  // create file name to write to
  if(stateGetTime == 1) { // got time over wifi
    Serial.println("INIT: using WiFi time");
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("INIT: failed to obtain time");
      errorState();
    }
  }
  else if(stateGetTime == 2) { // wifi failed, use incremental file names
    Serial.println("INIT: WiFi not found / no Internet, using consecutive file name"); // AFTER sdhc init
  }
  else {
    Serial.println("INIT: failed to obtain time, strange error");
    errorState();
  }
  
  // check PSRAM
  Serial.printf("INIT: total PSRAM: %d, free PSRAM: %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  if(ESP.getFreePsram() < 200000) { Serial.println("INIT: ERROR PSRAM too low!"); errorState(); }

  // create receiving queue
  rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(queue_entry_t)); // WARNING: already reserves memory for everything except the payload data itself (malloc in PSRAM)
  if(rxQueue == NULL) { Serial.println("INIT: queue failed!"); errorState(); }

  // init ESP NOW
  if(!initEspNowCustom()) { Serial.println("INIT: error ESP NOW init"); errorState(); }
  Serial.print("INIT: gateway MAC: "); Serial.println(WiFi.macAddress());
  esp_efuse_mac_get_default(ownMac);
  int8_t txPwr = 0;
  esp_wifi_get_max_tx_power(&txPwr);
  Serial.printf("INIT: tx power: %d\n", txPwr);

  // check heap
  Serial.printf("INIT: free heap: %d\n",ESP.getFreeHeap()); // 259272

  // start listening
  Serial.printf("INIT: all done, START LISTENING\n");
  xTaskCreate(mainTask, "mainTask", 8000, NULL, 1, NULL);

  // main loop
  struct tm timeinfo;
  while(1) {
    if((millis() - lastMillisPrintStatus > 3000) && (!receiveRunning) && (millis() - lastMsgMillis > 10000)) { // every 15 seconds, only if no receive running and only if lastMsg is at least 10 seconds ago
      lastMillisPrintStatus = millis();
      getLocalTime(&timeinfo);
        Serial.printf("%02d:%02d:%02d: Messages (new: %d, total rcv: %llu, total sent: %llu) | Bytes rcvd: %llu, Heap: %d, Psram: %d, ErrorCnt: %d, LastMsg: %ds\n",
          timeinfo.tm_hour,
          timeinfo.tm_min,
          timeinfo.tm_sec,
          newDataCounter,
          promCounter,
          sendCounter,
          totalAmountReceivedData,
          ESP.getFreeHeap(),
          ESP.getFreePsram(),
          errorCounter,
          ((uint32_t)(millis() - lastMsgMillis)) / 1000);
      newDataCounter = 0;
    }
    delay(1000); // THIS IS NEW
  }
}

void loop() { }
