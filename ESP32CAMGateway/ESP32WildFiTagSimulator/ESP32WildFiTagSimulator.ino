// SELECT ESP32 WROVER MODULE, HUGE APP NO OTA
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
#include "HelperTime.h"
#include "WifiCredentials.h"

/** ------ SETTINGS ------ */

#define GW_VERSION                                            10

const char* SSID1                                             = WIFI_SSID1;
const char* PASSWORD1                                         = WIFI_PASSWORD1;
const char* SSID2                                             = WIFI_SSID2;
const char* PASSWORD2                                         = WIFI_PASSWORD2;
const char* ntpServer                                         = "pool.ntp.org";
const String configFilename                                   = "/config.txt";

/** ------ DEFINES & STRUCTS ------ */

#define PIN_LED_RED                                           33
#define ESP_NOW_FLASH_STREAM_FIRST_BYTE                       0xAB

/** ------ MESSAGE STRUCTURE FOR PROXIMITY DETECTION ------ */

#define PHY_RATE                                            WIFI_PHY_RATE_1M_L
#define LONG_RANGE                                          0
#define GATEWAY_MSG_EVERY_MS                                300

#define PROXIMITY_OWN_GROUP_0                               0x12
#define PROXIMITY_OWN_GROUP_1                               0x34
#define PROXIMITY_OWN_GROUP_2                               0x56
#define PROXIMITY_OWN_GROUP_3                               0x78

#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY          0xAA
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY_LEN      250
#define ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE                37      // 1 byte, IMPORTANT, 0x04 = ESP NOW!
#define ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD                 43      // including checksum at end, add payload length to get total frame length for promiscous scan
#define PROXIMITY_DATA_LEN                                  250
#define ESPNOW_FRAME_OFFSET_PAYLOAD                         39
#define ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE                    0

/** Proximity payload data structure (250 bytes max!) */
#define PROXIMITY_DATA_LEN                              250
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_0                 1
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_1                 2
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_2                 3
#define PROX_PAYLOAD_OFFSET_OWN_GROUP_3                 4
#define PROX_PAYLOAD_OFFSET_OWN_ID_0                    5
#define PROX_PAYLOAD_OFFSET_OWN_ID_1                    6
#define PROX_PAYLOAD_OFFSET_SENDOFFSET_0                7
#define PROX_PAYLOAD_OFFSET_SENDOFFSET_1                8
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_0                9
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_1                10
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_2                11
#define PROX_PAYLOAD_OFFSET_TSLASTSYNC_3                12
#define PROX_PAYLOAD_OFFSET_TSLASTSYNCTYPE              13
#define PROX_PAYLOAD_OFFSET_VOLTAGE_0                   14
#define PROX_PAYLOAD_OFFSET_VOLTAGE_1                   15
#define PROX_PAYLOAD_OFFSET_LASTERRORID                 16
#define PROX_PAYLOAD_OFFSET_ERRORCNT_0                  17
#define PROX_PAYLOAD_OFFSET_ERRORCNT_1                  18
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_0                 19
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_1                 20
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_2                 21
#define PROX_PAYLOAD_OFFSET_TIMESTAMP_3                 22
#define PROX_PAYLOAD_OFFSET_SW_VERSION                  23
#define PROX_PAYLOAD_OFFSET_CONF_VERSION                24
#define PROX_PAYLOAD_OFFSET_SYNCCOUNTER_0               25
#define PROX_PAYLOAD_OFFSET_SYNCCOUNTER_1               26
#define PROX_PAYLOAD_OFFSET_SYNCCOUNTER_2               27
#define PROX_PAYLOAD_OFFSET_SYNCCOUNTER_3               28
#define PROX_PAYLOAD_OFFSET_FREEMEMORY_0                29
#define PROX_PAYLOAD_OFFSET_FREEMEMORY_1                30
#define PROX_PAYLOAD_OFFSET_FREEMEMORY_2                31
#define PROX_PAYLOAD_OFFSET_FREEMEMORY_3                32
#define PROX_PAYLOAD_OFFSET_STARTCNT_0                  33
#define PROX_PAYLOAD_OFFSET_STARTCNT_1                  34
#define PROX_PAYLOAD_OFFSET_STARTCNT_2                  35
#define PROX_PAYLOAD_OFFSET_STARTCNT_3                  36

/** ------ GLOBAL VARIABLES ------ */

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long lastMillisPrintStatus = 0;
int32_t lastRssi;
RTC_DATA_ATTR uint32_t errorCounter = 0;
uint64_t promCounter = 0;

uint8_t ownMac[6] = { 0 };
bool wifiStarted = true;
RTC_DATA_ATTR uint8_t stateGetTime = 0;
uint32_t newDataCounter = 0;

typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint16_t id;
    uint8_t receivedMessages;
    uint8_t rssiMin;
    uint8_t rssiMax;
    uint16_t rssiSum;
    int16_t timeDifferenceSumMs;
    uint32_t timestampLastSync;
    uint8_t lastSyncType;
    uint16_t voltage;
    uint8_t lastErrorId;
    uint16_t errorCnt;
    uint32_t timestamp;
    uint8_t swVersion;
    uint8_t confVersion;
    uint32_t syncCounter;
    uint32_t freeMemory;
    uint32_t startCnt;
    bool isGateway;
} proximity_entry_t;

#define PROXIMITY_MAX_TAGS_IN_PARALLEL                  150
#define ESPNOW_FRAME_OFFSET_SENDER_MAC                  10      // 6 bytes
proximity_entry_t scanResults[PROXIMITY_MAX_TAGS_IN_PARALLEL];
uint16_t scanResultsPointer = 0;

/** ------ VARIOUS FUNCTIONS ------ */

void printLogo() {
  Serial.println("----------------------------");
  Serial.printf("---WILDFI-TAG-SIMULATOR-V%d---\n", GW_VERSION);
  Serial.println("---SW: by Timm Alexander Wild---");
  Serial.println("----------------------------");
  Serial.println("CONFIGURED FOR SIMULATING TAG");
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

void printScanResult() {
    //printf("SCAN RESULT: found %d tag(s)\n", scanResultsPointer);
    struct tm timeinfo;
    bool success = true;
    if(!getLocalTime(&timeinfo)){
      Serial.printf("00:00:00\n");
      success = false;
    }
    else {
      timeinfo.tm_mon++; // starts at 0
      Serial.printf("%02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }
  
    for(uint16_t i=0; i<scanResultsPointer; i++) {
        printf(" %d: id: %04X, gw: %d, mac: %02X%02X%02X%02X%02X%02X, recMsgs: %d, rssiMin %d, rssiMax %d, rssiSum %d, rssiAvg %d, timeDiffSum %d, timeDiffAvg %d, lastSync %d (type: %d, cnt: %d), Vbatt %d, lastErrorId: %d, errorCnt: %d, ts: %d, version: %d.%d\n", i, scanResults[i].id, scanResults[i].isGateway, scanResults[i].mac[0], scanResults[i].mac[1], scanResults[i].mac[2], scanResults[i].mac[3], scanResults[i].mac[4], scanResults[i].mac[5],
            scanResults[i].receivedMessages, scanResults[i].rssiMin, scanResults[i].rssiMax, scanResults[i].rssiSum, (scanResults[i].rssiSum / scanResults[i].receivedMessages), scanResults[i].timeDifferenceSumMs, (scanResults[i].timeDifferenceSumMs / scanResults[i].receivedMessages), scanResults[i].timestampLastSync, scanResults[i].lastSyncType, scanResults[i].syncCounter, scanResults[i].voltage, scanResults[i].lastErrorId, scanResults[i].errorCnt, scanResults[i].timestamp, scanResults[i].swVersion, scanResults[i].confVersion);
    }
}

bool macsAreSame(uint8_t *mac1, uint8_t *mac2) {
    return ((mac1[0] == mac2[0]) && (mac1[1] == mac2[1]) && (mac1[2] == mac2[2]) && (mac1[3] == mac2[3]) && (mac1[4] == mac2[4]) && (mac1[5] == mac2[5]));
}

void addProximityMessageToScanResultImmediatePrint(wifi_promiscuous_pkt_t* p) {
    uint8_t rssiAbs;
    int8_t rssiTemp;

    proximity_entry_t result = { 0 };

    uint16_t id = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_ID_0];
    result.id = (id << 8) | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_ID_1];
    result.mac[0] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+0];
    result.mac[1] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+1];
    result.mac[2] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+2];
    result.mac[3] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+3];
    result.mac[4] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+4];
    result.mac[5] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+5];

    uint32_t timestampLastSyncOther = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_0] << 24)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_1] << 16)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_2] << 8)
        | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_3];
    result.timestampLastSync = timestampLastSyncOther; // only add lastSync from FIRST message, assuming that this will not change -> also faster
    result.lastSyncType = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNCTYPE];
    result.voltage = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_VOLTAGE_0] << 8)
        | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_VOLTAGE_1];
    result.lastErrorId = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_LASTERRORID];
    result.errorCnt = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_ERRORCNT_0] << 8)
        | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_ERRORCNT_1];
    result.timestamp = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TIMESTAMP_0] << 24)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TIMESTAMP_1] << 16)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TIMESTAMP_2] << 8)
        | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TIMESTAMP_3];            
    result.swVersion = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SW_VERSION];
    result.confVersion = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_CONF_VERSION];
    result.syncCounter = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SYNCCOUNTER_0] << 24)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SYNCCOUNTER_1] << 16)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SYNCCOUNTER_2] << 8)
        | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SYNCCOUNTER_3];
    result.freeMemory = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_FREEMEMORY_0] << 24)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_FREEMEMORY_1] << 16)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_FREEMEMORY_2] << 8)
        | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_FREEMEMORY_3];
    result.startCnt = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_STARTCNT_0] << 24)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_STARTCNT_1] << 16)
        | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_STARTCNT_2] << 8)
        | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_STARTCNT_3];

    if(p->rx_ctrl.rssi < 0) { rssiTemp = -(p->rx_ctrl.rssi); } // calculate absolute rssi value
    else { rssiTemp = p->rx_ctrl.rssi; }
    rssiAbs = rssiTemp;

    uint16_t sendOffset = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SENDOFFSET_0] << 8)
        | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SENDOFFSET_1];

    struct tm timeinfo;
    getLocalTime(&timeinfo);
    uint32_t currentMillis = (uint32_t) millis();

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint16_t millisecondsTime = (uint16_t) (tv.tv_usec / 1000LL);

    tmElements_t timestampStruct;
    breakTime(result.timestamp, timestampStruct);

    tmElements_t timestampLastSyncStruct;
    breakTime(result.timestampLastSync, timestampLastSyncStruct);
    
    printf("%02d:%02d:%02d.%03d (%d.%03d) id: %04X (%02X%02X%02X%02X%02X%02X), offset: %03d, rssi: -%d, lastSync: %02d:%02d:%02d (%02d.%02d.%04d, type: %d, cnt: %d), %d.%dV, err: %d.%d, ts: %02d:%02d:%02d (%02d.%02d.%04d), v: %d.%d, freeMem: %u, startCnt: %u\n",
        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, millisecondsTime, currentMillis / 1000, currentMillis % 1000,
        result.id, result.mac[0], result.mac[1], result.mac[2], result.mac[3], result.mac[4], result.mac[5],
        sendOffset, rssiAbs,
        timestampLastSyncStruct.Hour, timestampLastSyncStruct.Minute, timestampLastSyncStruct.Second, timestampLastSyncStruct.Day, timestampLastSyncStruct.Month, timestampLastSyncStruct.Year,
        result.lastSyncType, result.syncCounter, result.voltage / 1000, result.voltage % 1000, result.lastErrorId, result.errorCnt,
        timestampStruct.Hour, timestampStruct.Minute, timestampStruct.Second, timestampStruct.Day, timestampStruct.Month, timestampStruct.Year,
        result.swVersion, result.confVersion,
        result.freeMemory, result.startCnt);
}

void addProximityMessageToScanResult(wifi_promiscuous_pkt_t* p) {
    uint8_t rssiAbs;
    int8_t rssiTemp;
    if(scanResultsPointer >= PROXIMITY_MAX_TAGS_IN_PARALLEL) { /*lastErrorId = 66; errorCnt++;*/ } // should not happen -> if so, stop adding
    else {
        // check if already in list
        for(uint16_t i=0; i<scanResultsPointer; i++) {
            if(macsAreSame(&scanResults[i].mac[0], &p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC])) { // WARNING: compares real MAC not ID
                scanResults[i].timeDifferenceSumMs = 0; // be fast here!
                scanResults[i].receivedMessages = scanResults[i].receivedMessages + 1; // increment message counter
                if(p->rx_ctrl.rssi < 0) { rssiTemp = -(p->rx_ctrl.rssi); } // calculate absolute rssi value
                else { rssiTemp = p->rx_ctrl.rssi; }
                rssiAbs = rssiTemp;
                if(rssiAbs > scanResults[i].rssiMax) { scanResults[i].rssiMax = rssiAbs; }
                if(rssiAbs < scanResults[i].rssiMin) { scanResults[i].rssiMin = rssiAbs; }
                scanResults[i].rssiSum = scanResults[i].rssiSum + rssiAbs;
                return;
            }
        }
        // not in list -> add 
        uint16_t id = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_ID_0];
        scanResults[scanResultsPointer].timeDifferenceSumMs = 0; // be fast here!
        scanResults[scanResultsPointer].id = (id << 8) | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_OWN_ID_1];
        scanResults[scanResultsPointer].mac[0] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+0];
        scanResults[scanResultsPointer].mac[1] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+1];
        scanResults[scanResultsPointer].mac[2] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+2];
        scanResults[scanResultsPointer].mac[3] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+3];
        scanResults[scanResultsPointer].mac[4] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+4];
        scanResults[scanResultsPointer].mac[5] = p->payload[ESPNOW_FRAME_OFFSET_SENDER_MAC+5];
        scanResults[scanResultsPointer].receivedMessages = 1;
        scanResults[scanResultsPointer].isGateway = false;

        uint32_t timestampLastSyncOther = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_0] << 24)
            | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_1] << 16)
            | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_2] << 8)
            | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNC_3];
        scanResults[scanResultsPointer].timestampLastSync = timestampLastSyncOther; // only add lastSync from FIRST message, assuming that this will not change -> also faster
        scanResults[scanResultsPointer].lastSyncType = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TSLASTSYNCTYPE];
        scanResults[scanResultsPointer].voltage = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_VOLTAGE_0] << 8)
            | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_VOLTAGE_1];
        scanResults[scanResultsPointer].lastErrorId = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_LASTERRORID];
        scanResults[scanResultsPointer].errorCnt = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_ERRORCNT_0] << 8)
            | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_ERRORCNT_1];
        scanResults[scanResultsPointer].timestamp = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TIMESTAMP_0] << 24)
            | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TIMESTAMP_1] << 16)
            | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TIMESTAMP_2] << 8)
            | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_TIMESTAMP_3];            
        scanResults[scanResultsPointer].swVersion = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SW_VERSION];
        scanResults[scanResultsPointer].confVersion = p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_CONF_VERSION];
        scanResults[scanResultsPointer].syncCounter = (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SYNCCOUNTER_0] << 24)
            | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SYNCCOUNTER_1] << 16)
            | (p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SYNCCOUNTER_2] << 8)
            | p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + PROX_PAYLOAD_OFFSET_SYNCCOUNTER_3];

        if(p->rx_ctrl.rssi < 0) { rssiTemp = -(p->rx_ctrl.rssi); } // calculate absolute rssi value
        else { rssiTemp = p->rx_ctrl.rssi; }
        rssiAbs = rssiTemp;

        scanResults[scanResultsPointer].rssiMax = rssiAbs;
        scanResults[scanResultsPointer].rssiMin = rssiAbs;
        scanResults[scanResultsPointer].rssiSum = rssiAbs;
        
        scanResultsPointer++;
        //lastSeenSomeone = 0;
    }
}

void wifiPromiscuous(void* buffer, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*) (buffer);
  if(type == WIFI_PKT_MGMT) { // ESP NOW uses management frame
    if(p->rx_ctrl.rate == PHY_RATE) {
      if(p->payload[ESPNOW_FRAME_OFFSET_VENDOR_SPEC_TYPE] == 0x04) { // is ESP NOW frame
        if(p->rx_ctrl.sig_len == ESPNOW_FRAME_LENGTH_WITHOUT_PAYLOAD + PROXIMITY_DATA_LEN) { // normally 43 bytes additionally + 250 bytes payload
          if(p->payload[ESPNOW_FRAME_OFFSET_PAYLOAD + ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE] == ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY) { // proximity message
            lastRssi = p->rx_ctrl.rssi;
            promCounter++;
            //addProximityMessageToScanResult(p);
            addProximityMessageToScanResultImmediatePrint(p);
          }
        }
      }
    }
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { } // not doing anything here because only broadcasting

void sendProximityMessage() {
  uint8_t data[ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY_LEN] = { 0 }; // all zero

  struct timeval tv = { 0 };
  gettimeofday(&tv, NULL);
  uint16_t millisecondsTime = (uint16_t) (tv.tv_usec / 1000LL);
    
  data[0] = ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY;
  data[PROX_PAYLOAD_OFFSET_OWN_GROUP_0] = PROXIMITY_OWN_GROUP_0;
  data[PROX_PAYLOAD_OFFSET_OWN_GROUP_1] = PROXIMITY_OWN_GROUP_1;
  data[PROX_PAYLOAD_OFFSET_OWN_GROUP_2] = PROXIMITY_OWN_GROUP_2;
  data[PROX_PAYLOAD_OFFSET_OWN_GROUP_3] = PROXIMITY_OWN_GROUP_3;
  data[PROX_PAYLOAD_OFFSET_OWN_ID_0] = ownMac[4];
  data[PROX_PAYLOAD_OFFSET_OWN_ID_1] = ownMac[5];
  data[PROX_PAYLOAD_OFFSET_SENDOFFSET_0] = millisecondsTime >> 8; // BOOT TIME of ESP32 is not considered!
  data[PROX_PAYLOAD_OFFSET_SENDOFFSET_1] = millisecondsTime & 0xFF; // BOOT TIME of ESP32 is not considered!
  data[PROX_PAYLOAD_OFFSET_TSLASTSYNC_0] = 0; // never use time from tag simulator
  data[PROX_PAYLOAD_OFFSET_TSLASTSYNC_1] = 0; // never use time from tag simulator
  data[PROX_PAYLOAD_OFFSET_TSLASTSYNC_2] = 0; // never use time from tag simulator
  data[PROX_PAYLOAD_OFFSET_TSLASTSYNC_3] = 0; // never use time from tag simulator
  data[PROX_PAYLOAD_OFFSET_TSLASTSYNCTYPE] = 0;
  data[PROX_PAYLOAD_OFFSET_VOLTAGE_0] = 0x00;
  data[PROX_PAYLOAD_OFFSET_VOLTAGE_1] = 0x01;
  data[PROX_PAYLOAD_OFFSET_LASTERRORID] = 0;
  data[PROX_PAYLOAD_OFFSET_ERRORCNT_0] = 0;
  data[PROX_PAYLOAD_OFFSET_ERRORCNT_1] = 0;
  data[PROX_PAYLOAD_OFFSET_TIMESTAMP_0] = 0;
  data[PROX_PAYLOAD_OFFSET_TIMESTAMP_1] = 0;
  data[PROX_PAYLOAD_OFFSET_TIMESTAMP_2] = 0;
  data[PROX_PAYLOAD_OFFSET_TIMESTAMP_3] = 0;
  
  if(esp_now_send(broadcastAddress, data, ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_PROXIMITY_LEN) != ESP_OK) { printf("SEND ERROR\n"); errorCounter++; }
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

  //esp_now_register_recv_cb(onReceiveData);
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

void restart() {
  SD_MMC.end(); // important, eject SDHC
  esp_sleep_enable_timer_wakeup(1 * 1000 * 1000);
  esp_deep_sleep_disable_rom_logging(); // no more boot messages -> saves couple of ms
  esp_deep_sleep_start();  
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
  
  // check PSRAM
  Serial.printf("INIT: total PSRAM: %d, free PSRAM: %d\n", ESP.getPsramSize(), ESP.getFreePsram());
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

  long proximityTimeStart = 0;

  getLocalTime(&timeinfo);
  Serial.printf("TAG %02d:%02d:%02d: %llu msgs, %d scanResults, %d lastRssi\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, promCounter, scanResultsPointer, lastRssi);
  newDataCounter = 0;

  // main loop
  while(1) {
    long currentTime = millis();
    /*if(currentTime - lastMillisPrintStatus > 20000) {
      lastMillisPrintStatus = millis();
      getLocalTime(&timeinfo);
      Serial.printf("TAG %02d:%02d:%02d: %llu msgs, %d scanResults, %d lastRssi\n",
        timeinfo.tm_hour,
        timeinfo.tm_min,
        timeinfo.tm_sec,
        promCounter,
        scanResultsPointer,
        lastRssi);
      newDataCounter = 0;
    }*/
    if(scanResultsPointer > 0) {
      if(proximityTimeStart == 0) {
        proximityTimeStart = millis();
      }
      else if(currentTime - proximityTimeStart > 1100) {
        printScanResult();
        scanResultsPointer = 0;
        proximityTimeStart = 0;  
      }  
    }

    sendProximityMessage(); 
    delay(GATEWAY_MSG_EVERY_MS); // THIS IS NEW
  }
}

void loop() { }
