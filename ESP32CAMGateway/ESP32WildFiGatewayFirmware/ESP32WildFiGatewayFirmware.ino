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

// TODO: only sniff/participate but don't act as gateway?

// WARNING: changed storage on SDHC: not storing receive timestamp (needs too much additional time)

#define GATEWAY_FOR                                           GATEWAY_FOR_MOVEMENT_LOGGER
#define RECEVING_FROM_MULTIPLE_TAGS_ENABLED                   false // WARNING: PSRAM might not be sufficient
#define CHANGE_FILENAME_AFTER_TRANSMISSION                    true
#define GW_VERSION                                            11

String fileExtension                                          = ".bin"; // if STORE_AS_TEXT_FILE = true -> change to .txt
const char* SSID1                                             = "RodelbahnSoelden";
const char* PASSWORD1                                         = "xxxxxxxx";
const char* SSID2                                             = "wildfi";
const char* PASSWORD2                                         = "wildfi01";
const char* ntpServer                                         = "pool.ntp.org";
const String configFilename                                   = "/config.txt";

/** ------ DEFINES & STRUCTS ------ */

#define PIN_LED_RED                                           33
#define RESTART_WHEN_FILE_TOO_BIG_BYTE                        20000000UL // 20MB
#define SDHC_MINIMUM_FREE_MBYTES                              256
#define RX_QUEUE_SIZE                                         15000 // one msg = 245 + 7 = 252 byte -> 15000 * 252 = 3780000 = 3.7MB, sufficient for 4 MByte PSRAM
#define SDHC_BUFFER_SIZE                                      65536

#define ESP_NOW_FLASH_STREAM_FIRST_BYTE                       0xAB

#define COMMAND_BYTE_FORCE_TRACKING                           0x23
#define COMMAND_BYTE_ACTIVATE                                 0x33
#define COMMAND_BYTE_DEACTIVATE                               0x43

/** ------ MESSAGE STRUCTURE FOR MOVEMENT LOGGER ------ */

#if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER
  #define PHY_RATE                                            WIFI_PHY_RATE_18M
  #define LONG_RANGE                                          0
  const String filenameStart                                  = "_MOVE_DATA";
  
  #define GATEWAY_MSG_EVERY_MS                                20 // changed from 30 to 20

  #define ESPNOW_META_MSG_TAG_AROUND                          0x77
  #define ESPNOW_META_MSG_TAG_AROUND_LEN                      10
  #define ESPNOW_META_MSG_GOT_ACTIVATED                       0x55
  #define ESPNOW_META_MSG_GOT_ACTIVATED_LEN                   1           
  #define ESPNOW_META_MSG_GATEWAY_AROUND                      0x66
  #define ESPNOW_META_MSG_GATEWAY_AROUND_LEN                  2
#endif

/** ------ MESSAGE STRUCTURE FOR PROXIMITY DETECTION ------ */

#if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION
  #define PHY_RATE                                            WIFI_PHY_RATE_1M_L
  #define LONG_RANGE                                          0
  const String filenameStart                                  = "_PROX_DATA";
  
  #define GATEWAY_MSG_EVERY_MS                                80

  #define PROXIMITY_OWN_GROUP_0                               0x12
  #define PROXIMITY_OWN_GROUP_1                               0x34
  #define PROXIMITY_OWN_GROUP_2                               0x56
  #define PROXIMITY_OWN_GROUP_3                               0x78
  
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY          0xAA
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY_LEN      250
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND         0xCC
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND_LEN     16
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND     0xDD
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND_LEN 8
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED      0xEE
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED_LEN  7
#endif

typedef struct {
  uint8_t *message;
  //uint8_t mac[ESP_NOW_ETH_ALEN]; // allocated in PSRAM, pointer needs 4 byte
  //uint8_t *data; // 4 byte pointer
  //uint8_t data_len; // 4 byte pointer
  //int32_t rssi;
  //uint32_t phy_rate;
} queue_entry_t;

/** ------ GLOBAL VARIABLES ------ */

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long lastMillisReceived = 0;
unsigned long lastMillisPrintStatus = 0;
unsigned long lastMillisGatewayMsg = 0; // unused at the moment (old implementation)
int32_t lastRssi;
RTC_DATA_ATTR uint32_t errorCounter = 0;
uint32_t promCounter = 0;
bool errorWithQueue = false;
bool errorWithPSRAMLow = false;

uint8_t ownMac[6] = { 0 };
long lastMsgMillis = 0;
long lastDataMsgMillis = 0;
File file;
//File fileMeta;
bool receiveRunning = false;
static xQueueHandle rxQueue;
bool wifiStarted = true;
String currentFoldername = "";
String currentFilename = "";
RTC_DATA_ATTR uint8_t stateGetTime = 0;
uint64_t totalAmountReceivedData = 0;
uint32_t newDataCounter = 0;
uint64_t msgDataTransCnt = 0;
uint64_t msgGotActivatedCnt = 0;
uint64_t msgTagAroundCnt = 0;
uint64_t msgRefusedCnt = 0;
uint64_t msgProxCnt = 0;
uint8_t configCommandToSend = 0;

uint8_t sdhcBuffer[SDHC_BUFFER_SIZE] = { 0 };
uint32_t sdhcBufferPointer = 0;

/** ------ VARIOUS FUNCTIONS ------ */

void printLogo() {
  Serial.println("----------------------------");
  Serial.printf("---WILDFI-TAG-GATEWAY-V%d---\n", GW_VERSION);
  Serial.println("---SW: by Timm Alexander Wild---");
  Serial.println("----------------------------");
  #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER 
    Serial.println("CONFIGURED FOR MOVEMENT LOGGING");
  #endif
  #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION 
    Serial.println("CONFIGURED FOR PROXIMITY DETECTION");
  #endif
  Serial.println("----------------------------");
  Serial.printf("RTOS running @%dHz (should: 1000)\n", CONFIG_FREERTOS_HZ);
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
  uint16_t flashBlockPointer = 0;
  uint8_t flashPageInBlockPointer = 0, flashSubPagePointer = 0;
  decodeFlashPointer(queueEntry->message[7], queueEntry->message[8], queueEntry->message[9], &flashBlockPointer, &flashPageInBlockPointer, &flashSubPagePointer);
  
  Serial.print(millis());
  Serial.print(": ");
  for(int i = 0; i < 6; i++) {
    Serial.printf("%02X", queueEntry->message[i]);
    if(i < 5) Serial.print(":");
  }
  //Serial.print(", RSSI: ");
  //Serial.print(queueEntry->rssi);
  Serial.print(", ");
  Serial.printf(" LEN: %d, ", queueEntry->message[6]); 
  Serial.printf(" B: %d, P: %d, SUB: %d, ", flashBlockPointer, flashPageInBlockPointer, flashSubPagePointer); 
  /*for(int i = 3; i < 6; i++) {
    Serial.printf("%02X ", queueEntry->message[i + 7]);
  }*/
  uint8_t dataLen = queueEntry->message[6];
  Serial.printf(".. %02X %02X ", queueEntry->message[dataLen + 7 - 2], queueEntry->message[dataLen + 7 - 1]);
  Serial.printf(" (HeapFree: %d)", ESP.getFreeHeap());
  Serial.println("");
}

void decodeFlashPointer(uint8_t data0, uint8_t data1, uint8_t data2, uint16_t *flashBlockPointer, uint8_t *flashPageInBlockPointer, uint8_t *flashSubPagePointer) {
  // 3 BYTES PREAMBLE: [ 0000 0000 ] [ 000 | 00000 ] [ 0 | 0000 | 111 ] (11 bits for block, 6 bits for page in block, 4 bits for page part, 3 dummy bits)
  uint16_t temp = data0;
  *flashBlockPointer = (temp << 3) | ((data1 & 0b11100000) >> 5);
  *flashPageInBlockPointer = ((data1 & 0b00011111) << 1) | ((data2 & 0b10000000) >> 7);
  *flashSubPagePointer = (data2 & 0b01111000) >> 3;
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
  bool foundWifi = false;
  bool gotTime = false;
  String ssidToConnectTo = "";
  String passwordToConnectTo = "";
  Serial.println("TIME: started");
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
      if(WiFi.SSID(i) == SSID1) {
        ssidToConnectTo = SSID1;
        passwordToConnectTo = PASSWORD1;
        foundWifi = true;
      }
      else if(WiFi.SSID(i) == SSID2) {
        ssidToConnectTo = SSID2;
        passwordToConnectTo = PASSWORD2;
        foundWifi = true;
      }
  }
  if(foundWifi) {
    Serial.print("TIME: connect to ");
    Serial.println(ssidToConnectTo);
    WiFi.begin(ssidToConnectTo.c_str(), passwordToConnectTo.c_str());
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
    gotTime = printTime();
  }
  else {
    Serial.println("TIME: no suitable WiFi found!");
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  return gotTime;
}

/** ------ ESP NOW ------ */

/*void wifiPromiscuous(void* buffer, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
  if(type == WIFI_PKT_MGMT) { // ESP NOW uses management frame
    if((p->rx_ctrl.sig_len == 293) || (p->rx_ctrl.sig_len == 118)) { // 250 or 75 bytes
      if(p->rx_ctrl.rate == PHY_RATE) { // 14 = WIFI_PHY_RATE_18M
        lastRssi = p->rx_ctrl.rssi;
        promCounter++;        
      }
    }
  }
}*/

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { } // not doing anything here because only broadcasting

void sendGatewayNearMessage() {
  #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER 
    uint8_t data[ESPNOW_META_MSG_GATEWAY_AROUND_LEN] = { 0 }; // all zero
    data[0] = ESPNOW_META_MSG_GATEWAY_AROUND;
    data[1] = configCommandToSend;
    if(esp_now_send(broadcastAddress, data, ESPNOW_META_MSG_GATEWAY_AROUND_LEN) != ESP_OK) { printf("SEND ERROR\n"); errorCounter++; }
  #endif

  #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION 
    uint8_t data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND_LEN] = { 0 }; // all zero
    data[0] = ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND;
    data[1] = PROXIMITY_OWN_GROUP_0;
    data[2] = PROXIMITY_OWN_GROUP_1;
    data[3] = PROXIMITY_OWN_GROUP_2;
    data[4] = PROXIMITY_OWN_GROUP_3;
    data[5] = ownMac[4];
    data[6] = ownMac[5];
    data[7] = configCommandToSend;
    if(esp_now_send(broadcastAddress, data, ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND_LEN) != ESP_OK) { printf("SEND ERROR\n"); errorCounter++; }
  #endif
}

void onReceiveData(const uint8_t *mac, const uint8_t *data, int len) { // be quick here!
  queue_entry_t queueEntry;
  //queueEntry.rssi = lastRssi;
  if(mac == NULL || data == NULL || len <= 0) {
    Serial.println("E01");
    errorCounter++;
    errorWithQueue = true; // leads to restart
    return;
  }

  long currentTime = millis();

  #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER
    if((len == ESPNOW_META_MSG_GOT_ACTIVATED_LEN) && (data[0] == ESPNOW_META_MSG_GOT_ACTIVATED)) {
      msgGotActivatedCnt++;
      Serial.printf("ACTIVATED %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]); 
    }
    else if((len == ESPNOW_META_MSG_TAG_AROUND_LEN) && (data[0] == ESPNOW_META_MSG_TAG_AROUND)) {
      msgTagAroundCnt++;
      uint16_t voltage = (data[1] << 8) | data[2];
      uint8_t lastErrorId = data[3];
      uint16_t errorCnt = (data[4] << 8) | data[5];
      uint8_t commandByteMirrored = data[6];
      uint8_t state = data[7];
      uint8_t isActivated = data[8];
      uint8_t hasValidTimestamp = data[9];
      Serial.printf("TAG AROUND %02X:%02X:%02X:%02X:%02X:%02X: %d mV, lastErrorId: %d, errorCnt: %d, mirroredCmdByte: 0x%02X, state: %d/A%d/T%d\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], voltage, lastErrorId, errorCnt, commandByteMirrored, state, isActivated, hasValidTimestamp);
    }
    else if(data[0] == ESP_NOW_FLASH_STREAM_FIRST_BYTE) {
      msgDataTransCnt++;
      lastDataMsgMillis = currentTime;
    }
    else {
      msgRefusedCnt++;
      return; // do not store
    }
  #endif
  
  #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION 
    if((data[0] == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED) && (len == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED_LEN)) {
      msgGotActivatedCnt++;
      Serial.printf("ACTIVATED %02X:%02X:%02X:%02X:%02X:%02X: group: 0x%02X%02X%02X%02X, id: %02X%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], data[1], data[2], data[3], data[4], data[5], data[6]); 
    }
    else if((data[0] == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND) && (len == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND_LEN)) {
      msgTagAroundCnt++;
      uint16_t voltage = (data[7] << 8) | data[8];
      uint8_t lastErrorId = data[9];
      uint16_t errorCnt = (data[10] << 8) | data[11];
      uint8_t commandByteMirrored = data[12];
      uint8_t state = data[13];
      uint8_t isActivated = data[14];
      uint8_t hasValidTimestamp = data[15];
      Serial.printf("TAG AROUND %02X:%02X:%02X:%02X:%02X:%02X: group: 0x%02X%02X%02X%02X, id: %02X%02X, %d mV, lastErrorId: %d, errorCnt: %d, mirroredCmdByte: 0x%02X, state: %d/A%d/T%d\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], data[1], data[2], data[3], data[4], data[5], data[6], voltage, lastErrorId, errorCnt, commandByteMirrored, state, isActivated, hasValidTimestamp);
    }
    else if((data[0] == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY) && (len == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY_LEN)) {
      msgProxCnt++;
    }
    else if(data[0] == ESP_NOW_FLASH_STREAM_FIRST_BYTE) {
      msgDataTransCnt++;
      lastDataMsgMillis = currentTime;
    }
    else {
      msgRefusedCnt++;
      return; // do not store
    }
  #endif
  
  newDataCounter++;
  lastMsgMillis = currentTime;
  
  queueEntry.message = (uint8_t *) ps_malloc(len + 6 + 1); // new!
  if(queueEntry.message == NULL) {
    Serial.println("E02");
    errorCounter++;
    errorWithQueue = true; // leads to restart
    return;
  }
  memcpy(queueEntry.message, mac, ESP_NOW_ETH_ALEN); // 6 byte mac
  queueEntry.message[6] = (uint8_t) len; // 1 byte len
  memcpy(queueEntry.message + 7, data, len); // data
  //Serial.printf("SPACE: %d\n", uxQueueSpacesAvailable(rxQueue));
  if(uxQueueSpacesAvailable(rxQueue) == 0) {
    Serial.println("E03"); // HAPPENS if too many incoming messages
    errorCounter++;
    errorWithQueue = true; // leads to restart
    free(queueEntry.message);
  }
  else {
    if(xQueueSend(rxQueue, &queueEntry, 0) != pdTRUE) { // portMAX_DELAY = wait forever if queue is full
      Serial.println("E04");
      errorCounter++;
      errorWithQueue = true; // leads to restart
      free(queueEntry.message);
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
  //esp_wifi_set_promiscuous_rx_cb(&wifiPromiscuous);
  //esp_wifi_set_promiscuous(true);

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
  esp_now_register_send_cb(onDataSent);

  return true;
}

/** ------ CAMERA ------ */

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

bool initCamera() { // FRAMESIZE_SVGA = 100kByte RAM, FRAMESIZE_QVGA = 26kByte
  camera_config_t config;
  if(!psramFound()){
    return false;
  }
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  esp_err_t err = esp_camera_init(&config);
  if(err != ESP_OK) {
    Serial.printf("CAM: camera init failed with error 0x%x", err);
    return false;
  }
  return true;
}

bool deinitCamera() {
  if(esp_camera_deinit() != ESP_OK) {
    Serial.println("CAM: deinit failed");
    return false;
  }
  gpio_uninstall_isr_service();
  return true;
}

bool takePhotoAndStore(String filename) {
  if(!initCamera()) {
    Serial.println("CAM: init failed!");
    return false;
  }
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("CAM: capture failed!");
    return false;
  }
  String path = "/" + filename +".jpg";
  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("CAM: failed to open file in writing mode");
    return false;
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
  }
  file.close();
  esp_camera_fb_return(fb); 
  
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  /*pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);*/

  if(!deinitCamera()) {
    Serial.println("CAM: deinit failed!");
    return false;
  }

  return true;
}

/** ------ SDHC ------ */

bool initSDHC() {
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return false;
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return false;
  }
  return true;
}

void deleteEmptyFolders() {
  fs::FS &fs = SD_MMC;
  File root = fs.open("/");
  if(!root){ return; }
  if(!root.isDirectory()){ return; }
  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      File sub = fs.open(file.name());
      if(sub) {
        File subFile = sub.openNextFile();
        int cntFiles = 0;
        bool isEmpty = true;
        while(subFile) {
          subFile = sub.openNextFile();
          cntFiles++;
          if(cntFiles > 1) { isEmpty = false; break; }
        }
        if(subFile) { subFile.close(); }
        sub.close();
        if(isEmpty) {
          if(fs.rmdir(file.name())) { Serial.print("INIT: removed empty dir: "); Serial.println(file.name()); }
        }
      }
    }
    file = root.openNextFile();
  }
  if(file) { file.close(); }
  root.close();
}
void sdhcGetConfig() {
  String dataLine = "";
  char dataByte;
  fs::FS &fs = SD_MMC;
  File configFile = fs.open(configFilename);
  if(!configFile){
    Serial.println("CONFIG: Failed to open config file, use default CMD = 0x00");
    configCommandToSend = 0;
    return;
  }
  while(configFile.available()) {
    dataByte = configFile.read();
    if(dataByte == '\n') {
      break;
    }
    else if(dataByte == '\r') { }
    else if(dataByte == ' ') { }
    else {
      dataLine += dataByte;
    }
  }
  if(dataLine == "CMD_NOTHING") { configCommandToSend = 0; }
  else if(dataLine == "CMD_FORCE") { configCommandToSend = COMMAND_BYTE_FORCE_TRACKING; }
  else if(dataLine == "CMD_ACTIVATE") { configCommandToSend = COMMAND_BYTE_ACTIVATE; }
  else if(dataLine == "CMD_DEACTIVATE") { configCommandToSend = COMMAND_BYTE_DEACTIVATE; }
  else { configCommandToSend = 0; }
  Serial.print("CONFIG: Reading: ");
  Serial.print(dataLine);
  Serial.printf(" (%d Byte) -> Resulting Command Byte: ", dataLine.length());
  Serial.printf("0x%02X\n", configCommandToSend);
}

void sdhcGetNextFreeFilenameNumbers() {
  int i = 0;
  String appendix = "";
  String filenameToTest = "";
  while(true) {
    appendix = "";
    if(i<10000) appendix += "0";
    if(i<1000) appendix += "0";
    if(i<100) appendix += "0";
    if(i<10) appendix += "0";
    filenameToTest = appendix + String(i) + filenameStart;
    if(!SD_MMC.exists("/" + currentFoldername + "/" + filenameToTest + fileExtension)){
      Serial.println(filenameToTest + " FREE!");
      currentFilename = filenameToTest;
      break;
    }
    i++;
  }
}

void sdhcGetNextFreeFoldernameNumbers() {
  int i = 0;
  String appendix = "";
  String foldernameToTest = "";
  while(true) {
    appendix = "";
    if(i<10000) appendix += "0";
    if(i<1000) appendix += "0";
    if(i<100) appendix += "0";
    if(i<10) appendix += "0";
    foldernameToTest = appendix + String(i) + "_DATA";
    if(!SD_MMC.exists("/" + foldernameToTest)){
      currentFoldername = foldernameToTest;
      break;
    }
    i++;
  }
}

void sdhcOpen(String path) {
  fs::FS &fs = SD_MMC;
  file = fs.open(path.c_str(), FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file in appending mode");
  } 
}

void sdhcClose() {
  file.close();
}

/*bool storeDataOnSDHC(queue_entry_t *queueEntry) { // OLD IMPLEMENTATION
  int writtenBytes = 0;
  if(queueEntry == NULL) { return false; }

  //uint8_t recTimeArray[4] = { 0 };
  //uint32_t recTimeSeconds = (uint32_t) millis();
  //recTimeArray[0] = recTimeSeconds >> 24; recTimeArray[1] = recTimeSeconds >> 16; recTimeArray[2] = recTimeSeconds >>  8; recTimeArray[3] = recTimeSeconds;
  //writtenBytes = file.write(recTimeArray, 4); // receive time in seconds from gateway
  //if(writtenBytes == 0) { return false; } // SDHC ejected

  uint8_t dataLen = queueEntry->message[6];
  writtenBytes = file.write(queueEntry->message, dataLen + 7); // payload (image), payload length
  if(writtenBytes == 0) { return false; } // SDHC ejected

  return true;
}*/

bool writeRestInBufferOnSDHC() {
  if(sdhcBufferPointer == 0) { return true; }
  //sdhcOpen(currentFilenameWithExtension);
  int writtenBytes = file.write(sdhcBuffer, sdhcBufferPointer); // receive time in seconds from gateway
  if(writtenBytes == 0) { return false; } // SDHC ejected
  //sdhcClose();
  Serial.printf("SDHC: wrote rest (%d bytes)\n", sdhcBufferPointer);
  sdhcBufferPointer = 0;
  return true;
}

bool storeDataOnBuffer(queue_entry_t *queueEntry) { // new function: using buffer
  uint32_t dataLenWithMacAndLength = queueEntry->message[6];
  dataLenWithMacAndLength += 7;

  if(sdhcBufferPointer + dataLenWithMacAndLength <= SDHC_BUFFER_SIZE) {
    memcpy(sdhcBuffer + sdhcBufferPointer, queueEntry->message, dataLenWithMacAndLength); // just add data to buffer
    sdhcBufferPointer += dataLenWithMacAndLength;
  }
  else {
    long t = millis();
    uint32_t partOneLength = SDHC_BUFFER_SIZE - sdhcBufferPointer;
    memcpy(sdhcBuffer + sdhcBufferPointer, queueEntry->message, partOneLength); // copy to fill buffer completely
    uint32_t partTwoLength = dataLenWithMacAndLength - partOneLength;

    // write full buffer on SDHC
    //sdhcOpen(currentFilenameWithExtension);
    int writtenBytes = file.write(sdhcBuffer, SDHC_BUFFER_SIZE); // receive time in seconds from gateway
    if(writtenBytes == 0) { return false; } // SDHC ejected
    //sdhcClose();

    sdhcBufferPointer = 0;
    if(partTwoLength > 0) {
      memcpy(sdhcBuffer + sdhcBufferPointer, queueEntry->message + partOneLength, partTwoLength); // copy to add rest of message
      sdhcBufferPointer += partTwoLength;
    }
    uint32_t timeNeeded = millis() - t;
    Serial.printf("SDHC: %dB (rest: %d, q: %d) in %dms\n", SDHC_BUFFER_SIZE, sdhcBufferPointer, uxQueueMessagesWaiting(rxQueue), timeNeeded);
  }
  return true;
}

bool storeMetaData(queue_entry_t *queueEntry) { // new function: using buffer
  //long t = millis();
  char metaBuffer[1024];
  uint16_t metaBufferPnt = 0;
  const char *hex = "0123456789ABCDEF";
  uint32_t dataLenWithMacAndLength = queueEntry->message[6];
  dataLenWithMacAndLength += 7;
  String path = "/" + currentFoldername + "/METADATA.txt";
  fs::FS &fs = SD_MMC;
  File metaFile = fs.open(path.c_str(), FILE_APPEND);
  if(!metaFile){
    Serial.println("Failed to open meta file in appending mode");
    return false;
  }

  uint32_t recTimeSeconds = (uint32_t) (millis() / 1000);
  if(recTimeSeconds > 9999999) { recTimeSeconds = 9999999; }
  sprintf(metaBuffer, "%07d", recTimeSeconds);
  metaBuffer[7] = ':';
  metaBufferPnt += 8;

  for(uint16_t i=0; i<dataLenWithMacAndLength; i++) {
    metaBuffer[metaBufferPnt] = hex[(queueEntry->message[i] >> 4) & 0xF];
    metaBufferPnt++;
    metaBuffer[metaBufferPnt] = hex[(queueEntry->message[i]) & 0xF];
    metaBufferPnt++;
    if(i == 5) {
      metaBuffer[metaBufferPnt] = ' ';
      metaBufferPnt++;
    }
    if(i == 6) {
      metaBuffer[metaBufferPnt] = ' ';
      metaBufferPnt++;
    }
  }
  metaBuffer[metaBufferPnt] = '\n';
  metaBufferPnt++;
  metaBuffer[metaBufferPnt] = '\0';
  metaBufferPnt++;
  int writtenBytes = metaFile.print(metaBuffer);
  if(writtenBytes == 0) { return false; } // SDHC ejected
  metaFile.close();
  //uint32_t timeNeeded = millis() - t;
  //Serial.printf("MAIN: meta stored in %dms\n", timeNeeded);
  return true;
}

/*bool storeDataOnSDHCText(queue_entry_t *queueEntry) { // storing as hex
  int writtenBytes = 0;
  if(queueEntry == NULL) { return false; }
  for(uint8_t i=0; i<6; i++) {
    writtenBytes = file.print(String(queueEntry->message[i], HEX));
    if(writtenBytes == 0) { return false; } // SDHC ejected
    if(i != 5) { file.print(":"); }
  }
  writtenBytes = file.print(" ");
  if(writtenBytes == 0) { return false; } // SDHC ejected
  
  uint8_t data_len = (uint8_t) queueEntry->message[6];
  for(uint8_t i=0; i<data_len; i++) {
    if(queueEntry->message[i + 7] < 0x10) { 
      writtenBytes = file.print("0");
      if(writtenBytes == 0) { return false; } // SDHC ejected
    }
    writtenBytes = file.print(String(queueEntry->message[i + 7], HEX));
    if(writtenBytes == 0) { return false; } // SDHC ejected
    if(i != (data_len-1)) {
      writtenBytes = file.print(" ");
      if(writtenBytes == 0) { return false; } // SDHC ejected
    }
  }
  writtenBytes = file.println(""); 
  if(writtenBytes == 0) { return false; } // SDHC ejected
  return true;
}*/

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

void storeRestMessagesAfterError() {
  queue_entry_t queueEntry;
  uint64_t restData = 0;
  uint64_t restMsgs = 0;
  while(xQueueReceive(rxQueue, &queueEntry, 0) == pdTRUE) { 
    storeDataOnBuffer(&queueEntry);
    restData += queueEntry.message[6];
    restMsgs++;
    free(queueEntry.message);
  }
  if(!writeRestInBufferOnSDHC()) {
    Serial.println(" -> ERROR storing rest in buffer\n");
  }
  Serial.printf(" -> Stored rest in queue: %llu byte (%llu msgs)\n", restData, restMsgs);
}

void mainTask(void* parameter) {
  bool writeSuccess;
  long currentMillis;
  while(1) {
    if(uxQueueMessagesWaiting(rxQueue) >= 1) {
      receiveRunning = true;
      uint32_t savedMessages = 0;
      long t = millis();
      queue_entry_t queueEntry;
      while(xQueueReceive(rxQueue, &queueEntry, 0) == pdTRUE) {
        if(queueEntry.message[7] == ESP_NOW_FLASH_STREAM_FIRST_BYTE) { // flash stream data
          writeSuccess = storeDataOnBuffer(&queueEntry);
        }
        else { // metadata
          writeSuccess = storeMetaData(&queueEntry);
        }
        totalAmountReceivedData += queueEntry.message[6];
        free(queueEntry.message);
        savedMessages++;

        // error possibility: SDCARD ejected
        if(!writeSuccess) {
          esp_now_deinit();
          esp_wifi_stop();
          errorCounter++;
          Serial.println("MAIN: SEVERE ERROR - SDHC write failed (ejected?) - RESTARTING");
          restart();
        }
        
        // error possibility: PSRAM gets low
        if(savedMessages % 100 == 0) {
          if(ESP.getFreePsram() < 500000) {
            errorWithPSRAMLow = true;
            esp_now_deinit();
            esp_wifi_stop();
            errorCounter++;
            Serial.println("MAIN: SEVERE ERROR - PSRAM low -> storing rest of msgs then - RESTARTING");
            storeRestMessagesAfterError();
            sdhcClose();
            restart();
          }
        }

        // error possibility: some queue error
        if(errorWithQueue) {
          esp_now_deinit(); // deletes all information on paired devices
          esp_wifi_stop();
          Serial.println("MAIN: SEVERE ERROR - queue failure - RESTARTING");
          storeRestMessagesAfterError();
          sdhcClose();
          restart();
        }
      }
      uint32_t timeNeeded = millis() - t;
      //Serial.printf("MAIN: %d msgs in %dms\n", savedMessages, timeNeeded);
      receiveRunning = false;
    }
    
    delay(5); // otherwise watchdog sometimes triggers, 1ms possible because 1000Hz RTOS

    currentMillis = millis();
    if((sdhcBufferPointer > 0) && ((currentMillis - lastMsgMillis) > 10000)) { // no active transmission: then store rest of data in buffer on SDHC
      long t = millis();
      if(!writeRestInBufferOnSDHC()) {
        errorWithQueue = true;
        esp_now_deinit();
        esp_wifi_stop();
        errorCounter++;
        Serial.println("MAIN: SEVERE ERROR - SDHC write rest data failed (ejected?) - RESTARTING");
        // do not close SDHC (as not responding)
        restart();
      }
      sdhcClose(); // close the session
      uint32_t timeNeeded = millis() - t;
      Serial.printf("MAIN: buffer rest stored in %dms\n", timeNeeded);

      if(CHANGE_FILENAME_AFTER_TRANSMISSION) {
        // create file name (AFTER SDHC init)
        if(stateGetTime == 1) { // got time over wifi
          char currentFilenameChar[100];
          struct tm timeinfo;
          if(!getLocalTime(&timeinfo)){ Serial.println("MAIN: failed to obtain time"); }
          timeinfo.tm_mon++; // starts at 0
          sprintf(currentFilenameChar, "%04d%02d%02d_%02d%02d%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon,timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);  // 20201011_193020xxxxx   
          currentFilename = currentFilenameChar;
          currentFilename = currentFilename + filenameStart;
        }
        else { // wifi failed, use incremental file names
          sdhcGetNextFreeFilenameNumbers();
        }
        Serial.println("SDHC: write to file: /" + currentFoldername + "/" + currentFilename + fileExtension);
      }
      sdhcOpen("/" + currentFoldername + "/" + currentFilename + fileExtension); // open again 
    }

    // previously gateway messages sent here
    /*if(millis() - lastMillisGatewayMsg > GATEWAY_MSG_EVERY_MS) { // send out gateway message (I'm here)
      lastMillisGatewayMsg = millis();
      sendGatewayNearMessage();
    }*/
  }
}

void setup() {
  struct tm timeinfo;
  
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
  
  // SDHC
  if(!initSDHC()) { Serial.println("INIT: SDHC card mounting failed!"); errorState(); }
  else { Serial.printf("INIT: SDCARD size: %lld MByte, total MBytes: %lld, used MBytes: %lld (%lld Bytes)\n", SD_MMC.cardSize() / (1024 * 1024), SD_MMC.totalBytes() / (1024 * 1024), SD_MMC.usedBytes() / (1024 * 1024), SD_MMC.usedBytes()); }

  // check if enough free space
  uint64_t sdhcFreeBytes = SD_MMC.totalBytes() - SD_MMC.usedBytes();
  uint64_t sdhcFreeMegaBytes = sdhcFreeBytes / (1024 * 1024);
  if(sdhcFreeMegaBytes < SDHC_MINIMUM_FREE_MBYTES) {
    Serial.printf("INIT: ERROR not enough free space on SDHC: %d MBytes (Minimum: %d)\n", sdhcFreeMegaBytes, SDHC_MINIMUM_FREE_MBYTES);
    errorState();
  }

  // create folder to write to (AFTER SDHC init)
  deleteEmptyFolders();
  if(stateGetTime == 1) { // got time over wifi
    Serial.println("INIT: using WiFi time");
    if(!getLocalTime(&timeinfo)){
      Serial.println("INIT: failed to obtain time");
      errorState();
    }
    timeinfo.tm_mon++; // starts at 0
    char currentFoldernameChar[100];
    sprintf(currentFoldernameChar, "%04d%02d%02d_%02d%02d%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon,timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);  // 20201011_193020xxxxx   
    currentFoldername = currentFoldernameChar;
    currentFoldername = currentFoldername + "_DATA";
  }
  else { // wifi failed, use incremental file names
    Serial.println("INIT: WiFi not found / no Internet, using consecutive file name"); // AFTER sdhc init
    sdhcGetNextFreeFoldernameNumbers();
  }
  fs::FS &fs = SD_MMC;
  if(fs.mkdir("/" + currentFoldername)){ Serial.println("INIT: created dir: " + currentFoldername); }
  else {
    Serial.println("INIT: failed to create dir: " + currentFoldername);
   errorState();
  }

  // create file name (AFTER SDHC init)
  if(stateGetTime == 1) { // got time over wifi
    char currentFilenameChar[100];
    sprintf(currentFilenameChar, "%04d%02d%02d_%02d%02d%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon,timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);  // 20201011_193020xxxxx   
    currentFilename = currentFilenameChar;
    currentFilename = currentFilename + filenameStart;
  }
  else { // wifi failed, use incremental file names
    sdhcGetNextFreeFilenameNumbers();
  }

  Serial.println("SDHC: write to file: /" + currentFoldername + "/" + currentFilename + fileExtension);

  // list root directory
  //sdhcListDir("/", 0);
  //sdhcDecode();

  // camera test -> TAKES UP A LOT OF RAM
  //Serial.printf("INIT: 1 free heap: %d\n",ESP.getFreeHeap());
  //if(!takePhotoAndStore(currentFilename)) { Serial.println("INIT: camera photo failed!"); errorState(); }
  //Serial.printf("INIT: 2 free heap: %d\n",ESP.getFreeHeap());

  // get configuration
  sdhcGetConfig();
  
  // check PSRAM
  Serial.printf("INIT: total PSRAM: %d, free PSRAM: %d, queue entry size: %d\n", ESP.getPsramSize(), ESP.getFreePsram(), sizeof(queue_entry_t));
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

  // NEW: open file already
  sdhcOpen("/" + currentFoldername + "/" + currentFilename + fileExtension);

  // start listening
  Serial.printf("INIT: all done, START LISTENING (broadcast every %d ms)\n", GATEWAY_MSG_EVERY_MS);
  xTaskCreate(mainTask, "mainTask", 8000, NULL, 1, NULL); // priority 1 = low

  // main loop
  while(1) {
    long currentTime = millis();
    if((currentTime - lastMillisPrintStatus > 15000) && (!receiveRunning) && (currentTime - lastMsgMillis > 10000)) { // every 15 seconds, only if no receive running and only if lastMsg is at least 10 seconds ago
      lastMillisPrintStatus = millis();
      getLocalTime(&timeinfo);
      #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER 
        Serial.printf("MOV %02d:%02d:%02d: Cmd: 0x%02X, Messages (new: %d): Data: %llu, Activation: %llu, TagAround: %llu, Refused: %llu | Bytes stored: %llu, Heap: %d, Psram: %d, SDHC: %llu, Buffer: %d, ErrorCnt: %d, LastMsg: %ds\n",
          timeinfo.tm_hour,
          timeinfo.tm_min,
          timeinfo.tm_sec,
          configCommandToSend,
          newDataCounter,
          msgDataTransCnt,
          msgGotActivatedCnt,
          msgTagAroundCnt,
          msgRefusedCnt,
          totalAmountReceivedData,
          ESP.getFreeHeap(),
          ESP.getFreePsram(),
          sdhcFreeBytes - totalAmountReceivedData,
          sdhcBufferPointer,
          errorCounter,
          ((uint32_t)(currentTime - lastMsgMillis)) / 1000);
      #endif
      #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION 
        Serial.printf("PROX %02d:%02d:%02d: Cmd: 0x%02X, Messages (new: %d): Data: %llu, Activation: %llu, TagAround: %llu, Proximity: %llu, Refused: %llu | Bytes stored: %llu, Heap: %d, Psram: %d, SDHC: %llu, Buffer: %d, ErrorCnt: %d, LastMsg: %ds\n",
          timeinfo.tm_hour,
          timeinfo.tm_min,
          timeinfo.tm_sec,
          configCommandToSend,
          newDataCounter,
          msgDataTransCnt,
          msgGotActivatedCnt,
          msgTagAroundCnt,
          msgProxCnt,
          msgRefusedCnt,
          totalAmountReceivedData,
          ESP.getFreeHeap(),
          ESP.getFreePsram(),
          sdhcFreeBytes - totalAmountReceivedData,
          sdhcBufferPointer,
          errorCounter,
          ((uint32_t)(currentTime - lastMsgMillis)) / 1000);
      #endif
        
      newDataCounter = 0;
      
      if(sdhcFreeBytes - totalAmountReceivedData < 10000000UL) { // less than 10 MByte
        Serial.println("ERROR: not enough space on SDHC");
        restart();
      }
    }
    // THIS IS NEW
    if((!errorWithQueue) && (!errorWithPSRAMLow)) {
      if(RECEVING_FROM_MULTIPLE_TAGS_ENABLED) { // constantly pumping out gateway near messages
        sendGatewayNearMessage();
      }
      else {
        if(currentTime - lastDataMsgMillis > 5000) { // only send gateway near message if no receive running and only if last DATA MESSAGE is at least 5s ago
          sendGatewayNearMessage();          
        }
        else {
          Serial.printf("-> NO BROADCAST (PSRAM left %d, queue size %d, buffer pointer %d)\n", ESP.getFreePsram(), uxQueueMessagesWaiting(rxQueue), sdhcBufferPointer);
          delay(2000);
        }
      }
    }
    delay(GATEWAY_MSG_EVERY_MS); // THIS IS NEW
  }
}

void loop() { }
