// SELECT ESP32 WROVER MODULE, HUGE APP NO OTA
// WARNING: do not use getLocalTime when never connected via WiFi (auto-disconnecting ESP-NOW every some seconds)
// WARNING: after receiving data (MOVEMENT_LOGGER) wait for WRITING_REST_ON_SDHC_AFTER_MS (10s), to write rest, otherwise full data loss
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
// WARNING: changed storage on SDHC: not storing receive timestamp (needs too much additional time)

#define GATEWAY_FOR                                           GATEWAY_FOR_PROXIMITY_DETECTION
#define GW_VERSION                                            2

/** ------ DEFINES & STRUCTS ------ */

#define PIN_LED_RED                                           33
#define SDHC_MINIMUM_FREE_MBYTES                              256
#define RX_QUEUE_SIZE                                         15000 // one msg = 245 + 7 = 252 byte -> 15000 * 252 = 3780000 = 3.7MB, sufficient for 4 MByte PSRAM
#define SDHC_BUFFER_SIZE                                      65536
#define ESP_NOW_FLASH_STREAM_FIRST_BYTE                       0xAB
#define ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD                   43 // including checksum at end, add payload length to get total frame length for promiscous scan
#define ESPNOW_FRAME_OFFSET_PAYLOAD                           39

/** ------ MESSAGE STRUCTURE FOR MOVEMENT LOGGER ------ */

#if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER
  #define PHY_RATE                                            WIFI_PHY_RATE_18M
  #define LONG_RANGE                                          0
  #define GATEWAY_MSG_EVERY_MS                                20 // changed from 30 to 20

  #define ESPNOW_META_MSG_TAG_AROUND                          0x77
  #define ESPNOW_META_MSG_TAG_AROUND_LEN                      10
  #define ESPNOW_META_MSG_GOT_ACTIVATED                       0x55
  #define ESPNOW_META_MSG_GOT_ACTIVATED_LEN                   1           
  #define ESPNOW_META_MSG_GATEWAY_AROUND                      0x66
  #define ESPNOW_META_MSG_GATEWAY_AROUND_LEN                  2
  #define ESPNOW_META_MSG_GATEWAY_AROUND_V2                   0x88
  #define ESPNOW_META_MSG_GATEWAY_AROUND_V2_LEN               250
  #define ESPNOW_META_MSG_TAG_AROUND_V2                       0x99
  #define ESPNOW_META_MSG_TAG_AROUND_V2_LEN                   30

  #define COMMAND_BYTE_NOTHING                                0x00                            // no command
  #define COMMAND_BYTE_FORCE_TRACKING                         0x23                            // start tracking independent of normal run times
  #define COMMAND_BYTE_ACTIVATE                               0x33                            // activate the tag
  #define COMMAND_BYTE_DEACTIVATE                             0x43                            // put the tag back into wait for activation state
  #define COMMAND_BYTE_CHANGE_CONFIG                          0x53                            // change configuration of tag
  #define COMMAND_BYTE_MAG_CALIBRATION                        0x63                            // enter mag calibration mode
  #define COMMAND_BYTE_DO_NOT_SEND                            0x73                            // do not send data messages
  #define COMMAND_BYTE_TIME_RESYNC                            0x83                            // enter time resync mode

  uint8_t configCommandToSend = COMMAND_BYTE_DO_NOT_SEND;
#endif

/** ------ MESSAGE STRUCTURE FOR PROXIMITY DETECTION ------ */

#if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION
  #define PHY_RATE                                            WIFI_PHY_RATE_1M_L
  #define LONG_RANGE                                          0
  #define GATEWAY_MSG_EVERY_MS                                80

  #define PROXIMITY_OWN_GROUP_0                               0x12 // still sniffing all proximity data (also of other groups), but tags will not send data and won't store proximity information
  #define PROXIMITY_OWN_GROUP_1                               0x34 // still sniffing all proximity data (also of other groups), but tags will not send data and won't store proximity information
  #define PROXIMITY_OWN_GROUP_2                               0x56 // still sniffing all proximity data (also of other groups), but tags will not send data and won't store proximity information
  #define PROXIMITY_OWN_GROUP_3                               0x78 // still sniffing all proximity data (also of other groups), but tags will not send data and won't store proximity information
  
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY          0xAA
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY_LEN      250
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND         0xCC
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND_LEN     16
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND     0xDD
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND_LEN 70
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED      0xEE
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED_LEN  7

  /** Gateway commands */
  #define PROXIMITY_COMMAND_NOTHING                       0x00
  #define PROXIMITY_COMMAND_DO_NOT_SEND                   0x1F
  #define PROXIMITY_COMMAND_ACTIVATE                      0x2F
  #define PROXIMITY_COMMAND_DEACTIVATE                    0x3F
  #define PROXIMITY_COMMAND_CHANGE_CONFIG                 0x4F
  #define PROXIMITY_COMMAND_FULL_RESET                    0x5F
  #define PROXIMITY_COMMAND_MAG_CALIB                     0x6F
  #define PROXIMITY_COMMAND_RESYNC_TIME_BY_WIFI           0x7F
  #define PROXIMITY_COMMAND_ACTIVATE_AT_06_00             0x8F
  #define PROXIMITY_COMMAND_ACTIVATE_AT_12_00             0x9F
  #define PROXIMITY_COMMAND_ACTIVATE_AT_15_00             0xAF
  #define PROXIMITY_COMMAND_ACTIVATE_AT_20_00             0xBF
  #define PROXIMITY_COMMAND_FIRST_SYNC_TIME_IN_ACTIVATION 0xCF

  /** All messages message type (first byte of all message) */
  #define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE                0

  uint8_t configCommandToSend = PROXIMITY_COMMAND_DO_NOT_SEND;
#endif

typedef struct {
  uint8_t *message;
  //uint8_t mac[ESP_NOW_ETH_ALEN]; // allocated in PSRAM, pointer needs 4 byte
  //uint8_t *data; // 4 byte pointer
  //uint8_t data_len; // 4 byte pointer
  int32_t rssi;
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
bool wifiStarted = true;
uint64_t totalAmountReceivedData = 0;
uint32_t newDataCounter = 0;
uint64_t msgDataTransCnt = 0;
uint64_t msgGotActivatedCnt = 0;
uint64_t msgTagAroundCnt = 0;
uint64_t msgRefusedCnt = 0;
uint64_t msgProxCnt = 0;
bool newProximitDataReceived = false;
uint64_t gatewayMsgs = 0;

/** ------ VARIOUS FUNCTIONS ------ */

void printLogo() {
  Serial.println("----------------------------");
  Serial.printf("---WILDFI-TAG-NEARBY-ONLY-V%d---\n", GW_VERSION);
  Serial.println("---SW: by Timm Alexander Wild---");
  Serial.println("----------------------------");
  #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER 
    Serial.println("CONFIGURED FOR MOVEMENT LOGGING");
  #endif
  #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION 
    Serial.println("CONFIGURED FOR PROXIMITY DETECTION");
  #endif
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

/** ------ ESP NOW ------ */

void wifiPromiscuous(void* buffer, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
  if(type == WIFI_PKT_MGMT) { // ESP NOW uses management frame
      if(p->rx_ctrl.rate == PHY_RATE) { // 14 = WIFI_PHY_RATE_18M
        bool isRelevantMessage = false;
        #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER
          if(p->rx_ctrl.sig_len == (ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD+ESPNOW_META_MSG_GOT_ACTIVATED_LEN)) { isRelevantMessage = true; }
          if(p->rx_ctrl.sig_len == (ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD+ESPNOW_META_MSG_TAG_AROUND_LEN)) { isRelevantMessage = true; }
          if(p->rx_ctrl.sig_len == (ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD+ESPNOW_META_MSG_TAG_AROUND_V2_LEN)) { isRelevantMessage = true; }
        #endif
        #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION 
          if(p->rx_ctrl.sig_len == (ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD+ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GOT_ACTIVATED_LEN)) { isRelevantMessage = true; }
          if(p->rx_ctrl.sig_len == (ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD+ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_TAG_AROUND_LEN)) { isRelevantMessage = true; }
        #endif
        if(isRelevantMessage) {
          lastRssi = p->rx_ctrl.rssi;
          promCounter++;
          Serial.printf("(%d %d) ", lastRssi, p->rx_ctrl.sig_len);
        }
      }
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { } // not doing anything here because only broadcasting

void sendGatewayNearMessage() {
  #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER
    uint8_t data[ESPNOW_META_MSG_GATEWAY_AROUND_LEN] = { 0 }; // all zero
    data[0] = ESPNOW_META_MSG_GATEWAY_AROUND;
    data[1] = configCommandToSend;
    if(esp_now_send(broadcastAddress, data, ESPNOW_META_MSG_GATEWAY_AROUND_LEN) != ESP_OK) { printf("SEND ERROR\n"); errorCounter++; }
    else { gatewayMsgs++; }      
  #endif

  #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION 
    uint8_t data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND_LEN] = { 0 }; // all zero
    uint8_t i = 0;
    data[i] = ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND; i++;
    data[i] = PROXIMITY_OWN_GROUP_0; i++;
    data[i] = PROXIMITY_OWN_GROUP_1; i++;
    data[i] = PROXIMITY_OWN_GROUP_2; i++;
    data[i] = PROXIMITY_OWN_GROUP_3; i++;
    data[i] = ownMac[4]; i++;
    data[i] = ownMac[5]; i++;
    data[i] = configCommandToSend; i++;
    if(esp_now_send(broadcastAddress, data, ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND_LEN) != ESP_OK) { printf("SEND ERROR\n"); errorCounter++; }
    else { gatewayMsgs++; }
  #endif
}

void onReceiveData(const uint8_t *mac, const uint8_t *data, int len) { // be quick here!
  queue_entry_t queueEntry;
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
    else if((len == ESPNOW_META_MSG_TAG_AROUND_V2_LEN) && (data[0] == ESPNOW_META_MSG_TAG_AROUND_V2)) {
      msgTagAroundCnt++;
      uint16_t voltage = (data[1] << 8) | data[2];
      uint8_t lastErrorId = data[3];
      uint16_t errorCnt = (data[4] << 8) | data[5];
      uint8_t commandByteMirrored = data[6];
      uint8_t state = data[7];
      uint8_t isActivated = data[8];
      uint8_t hasValidTimestamp = data[9];
      
      uint8_t wildFiSoftwareVersion = data[10];
      uint8_t wildFiConfigVersion = data[11];
      int16_t magHardIronOffsetX = (data[12] << 8) | data[13];
      int16_t magHardIronOffsetY = (data[14] << 8) | data[15];
      int16_t magHardIronOffsetZ = (data[16] << 8) | data[17];
      uint32_t startCnt = (data[18] << 24) | (data[19] << 16) | (data[20] << 8) | data[21];
      uint32_t bytesToTransmit = (data[22] << 24) | (data[23] << 16) | (data[24] << 8) | data[25];
      Serial.printf("TAG AROUND V2 %02X:%02X:%02X:%02X:%02X:%02X: %d mV, error: %d/%d, mirroredCmd: 0x%02X, st: %d/A%d/T%d, V%d.%d, Mag %d/%d/%d, %d, %d\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], voltage, lastErrorId, errorCnt, commandByteMirrored, state, isActivated, hasValidTimestamp,
        wildFiSoftwareVersion, wildFiConfigVersion, magHardIronOffsetX, magHardIronOffsetY, magHardIronOffsetZ, startCnt, bytesToTransmit);
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
  esp_now_register_send_cb(onDataSent);

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

void setup() {
  // init pins and serial
  Serial.begin(115200);
  pinMode(PIN_LED_RED, OUTPUT);
  digitalWrite(PIN_LED_RED, HIGH); // off
  delay(500);

  // print the logo
  printLogo();
  
  // check PSRAM
  Serial.printf("INIT: total PSRAM: %d, free PSRAM: %d, queue entry size: %d\n", ESP.getPsramSize(), ESP.getFreePsram(), sizeof(queue_entry_t));
  if(ESP.getFreePsram() < 200000) { Serial.println("INIT: ERROR PSRAM too low!"); errorState(); }
  
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
  Serial.printf("INIT: all done, START LISTENING (broadcast every %d ms)\n", GATEWAY_MSG_EVERY_MS);
 
  // main loop
  while(1) {
    sendGatewayNearMessage();
    delay(GATEWAY_MSG_EVERY_MS); // THIS IS NEW
  }
}

void loop() { }
