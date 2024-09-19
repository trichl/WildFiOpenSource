// SELECT ESP32 WROVER MODULE, HUGE APP NO OTA
// COMPILES WITH ARDUINO 1.8.19 (esp32 1.0.4)
// WARNING: do not use getLocalTime when never connected via WiFi (auto-disconnecting ESP-NOW every some seconds)
// WARNING: after receiving data (MOVEMENT_LOGGER) wait for WRITING_REST_ON_SDHC_AFTER_MS (10s), to write rest, otherwise full data loss
// WARNING: changed storage on SDHC: not storing receive timestamp (needs too much additional time)
// USE WINDOWS STORE VERSION OF ARDUINO (VERSION 1.8.19), then esp32 package is located here: C:\Users\[User]\Documents\ArduinoData\packages\esp32\hardware\esp32
// ADD ADDITIONAL BOARD MANAGEMENT URL: https://github.com/espressif/arduino-esp32/releases/download/1.0.4/package_esp32_index.json and select and install in board manager "esp32" version 1.0.4
#define GATEWAY_FOR_MOVEMENT_LOGGER                     0
#define GATEWAY_FOR_PROXIMITY_DETECTION                 1
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "time.h"
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include <esp_wifi_internal.h> // for modifying default transmission speed of 1Mbps
#include "SPIFFS.h"
#include "soc/soc.h" // disable brownout problems
#include "soc/rtc_cntl_reg.h" // disable brownout problems
#include "driver/rtc_io.h"
#include "WifiCredentials.h"

/** ------ SETTINGS ------ */

#define GATEWAY_FOR                                           GATEWAY_FOR_MOVEMENT_LOGGER
#define GW_VERSION                                            28

/** ------ ONLY FOR MOVEMENT LOGGER ------ */

#define WILDFI_SOFTWARE_VERSION                               0 // 0 = all versions
#define WILDFI_CONFIG_VERSION                                 0 // 0 = all versions

String fileExtension                                          = ".bin";
const char* ntpServer                                         = "pool.ntp.org";

/** ------ DEFINES & STRUCTS ------ */

#define PIN_LED_RED                                           33
#define SDHC_MINIMUM_FREE_MBYTES                              256
#define RX_QUEUE_SIZE                                         15000 // one msg = 245 + 7 = 252 byte -> 15000 * 252 = 3780000 = 3.7MB, sufficient for 4 MByte PSRAM
#define SDHC_BUFFER_SIZE                                      65536
#define ESP_NOW_FLASH_STREAM_FIRST_BYTE                       0xAB
#define STATE_MACHINE_FIRST_START                             0
#define STATE_MACHINE_RUN                                     1
#define AUTO_RESTART_SECONDS                                  (3600UL*12UL)
#define AUTO_RESTART_SLEEP_SECONDS                            15
#define ERROR_STATE_AUTO_RESET_SECONDS                        3600UL

/** ------ WIFI UPLOAD ------ */

bool uploadEnabled = true;
bool uploadRenameFiles = true;
String uploadServer = "api-content.dropbox.com";
String uploadPath = "/2/files/upload";
const int uploadPort = 443;
WiFiClientSecure client;

/** ------ MESSAGE STRUCTURE FOR MOVEMENT LOGGER ------ */

#if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER
  #define RECEVING_FROM_MULTIPLE_TAGS_ENABLED                 false // WARNING: PSRAM might not be sufficient, will stop broadcasting when getting DATA messages, up to 5 seconds
  #define RECEVING_FROM_MULTIPLE_TAGS_NO_SEND_MS              5000 // if RECEVING_FROM_MULTIPLE_TAGS_ENABLED = false: do not send a gatewayAround message X ms after last message received
  #define CHANGE_FILENAME_AFTER_TRANSMISSION                  true // WRITING_REST_ON_SDHC_AFTER_MS seconds after receiving last transmission: start a new file
  #define WRITING_REST_ON_SDHC_AFTER_MS                       6000 // wait for this time until finally writing data on SDHC

  const String configFilename                                 = "/configMove.txt";
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
  #define COMMAND_BYTE_TIME_SYNC_ACTIVATION                   0x93                            // enter time resync mode, but only in activation and only when not already got time
  #define COMMAND_BYTE_ACTIVATE_WHEN_NO_GW                    0xA3                            // NEW: then this command is received in activation mode: from now on: start as soon as no gateway is seen anymore


  typedef struct {
      bool doTheBlink;
      uint8_t trackerMode;
      uint8_t trackingDataMode;
      uint8_t transmissionMethod;
      uint8_t drMinHdopXTen;
      uint16_t drImuSeconds;
      uint16_t gps1HzInterruptAfterSeconds;
      uint8_t accFrequency;
      uint8_t accAvg;
      uint8_t accRange;
      uint8_t magFrequency;
      uint8_t magAccuracy;
      uint8_t gyroFrequency;
      uint8_t gyroRange;
      uint8_t gyroMode;
      bool useMagnetometer;
      bool useGyro;
      uint8_t nightTimeEnter;
      uint8_t nightTimeMode;
      uint8_t nightTimeTurnOnHour;
      uint8_t nightTimeTurnOnMinute;
      uint8_t nightTimeTurnOffHour;
      uint8_t nightTimeTurnOffMinute;
      uint32_t nightTimeDataTransDeepestNightHours;
      uint16_t nightTimeModeTryDataTransWakeupSeconds;
      uint16_t battMinVoltage;
      uint16_t battRestartVoltage;
      uint16_t dataTransBattMinVoltage;
      uint16_t dataTransBattMinVoltageDuringTrans;
      bool skipGetTime;
      uint16_t timeBetweenGetTimeRetriesSeconds;
      uint8_t activationMode;
      uint8_t activationSource;
      uint16_t activationByEspNowRetrySeconds;
      uint16_t dataTransTryEveryFullMinSeldomly;
      uint16_t memFullTryEveryFullMinSeldomly;
      uint16_t accInterruptWatermark;
      uint8_t dataTransOutputPower;
      bool espNowCustomRFCalibration;
      uint16_t commandByteForceTrackingDurationSeconds;
      uint8_t espNowDataRate;
      bool espNowLongRange;
      uint32_t espNowMinBytesToTransmit;
      uint32_t espNowMaxBytesToTransmit;
      uint16_t wifiMinBlocksToTransmit;
      uint16_t wifiMaxBlocksToTransmit;
      bool activityActivationEnabled;
      uint16_t activityThresholdActiveToInactiveAvg;
      uint8_t activityThresholdInactiveToActiveThr;
      uint16_t activityTransmissionInterval;
  } tag_config_t;
  tag_config_t configSend = { };
#endif

/** ------ MESSAGE STRUCTURE FOR PROXIMITY DETECTION ------ */

#if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION
  #define RECEVING_FROM_MULTIPLE_TAGS_ENABLED                 true // WARNING: PSRAM might not be sufficient, will stop broadcasting when getting DATA messages (which means not effective for proximity detection), up to 5 seconds
  #define RECEVING_FROM_MULTIPLE_TAGS_NO_SEND_MS              5000
  #define CHANGE_FILENAME_AFTER_TRANSMISSION                  false // WRITING_REST_ON_SDHC_AFTER_MS seconds after receiving last transmission: start a new file
  #define WRITING_REST_ON_SDHC_AFTER_MS                       1500 // wait for this time until finally writing data on SDHC

  const String configFilename                                 = "/configProx.txt";
  #define PHY_RATE                                            WIFI_PHY_RATE_1M_L
  #define LONG_RANGE                                          0
  const String filenameStart                                  = "_PROX_DATA";
  
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

  typedef struct {
      bool useLeds;
      uint8_t trackerMode;
      uint8_t activationMode;
      bool getFirstTimeOverWiFi;
      bool getFirstTimeOverGPS;
      uint8_t tagIdSource;
      uint8_t imuMode;
      uint32_t imuBurstMillis;
      bool environmentActivated;
      bool timeCorrectionBetweenTags;
      uint16_t timeCorrectionDiffMs;
      bool freeMemoryIfFull;
      uint8_t accFrequency;
      uint8_t accAvg;
      uint8_t accRange;
      uint8_t magFrequency;
      uint8_t magAccuracy;
      uint8_t gyroFrequency;
      uint8_t gyroRange;
      uint8_t gyroMode;
      uint8_t nightTimeEnter;
      uint8_t nightTimeTurnOnHour;
      uint8_t nightTimeTurnOnMinute;
      uint8_t nightTimeTurnOffHour;
      uint8_t nightTimeTurnOffMinute;
      uint32_t gpsFixHourBits;
      bool gpsRandomizeFixes;
      uint8_t gpsRandomizeFixesPerDay;
      uint8_t gpsMinHdopTimesTen;
      uint8_t gpsFirstFixCollectOrbitDataSeconds;
      bool gpsForcedAfterEveryProximity;
      uint8_t gpsSyncRTCFrequency;
      uint8_t proximityFrequencyMinute;
      uint8_t proximityFrequencyMinuteSeenSomeone;
      uint16_t proximityListeningIntervalMs;
      uint8_t proximityDbm;
      uint8_t proximityDatarate;
      bool proximityLongRange;
      uint16_t proximityAirTimeUs;
      uint16_t activationByGatewayListeningTime;
      uint16_t activationByGatewaySleepSeconds;
      uint16_t battMinVoltage;
      uint16_t battRestartVoltage;
      uint16_t battMinVoltageDuringTransmission;
      uint8_t timeWifiOutputPower;
      uint16_t timeBetweenGetTimeRetriesSeconds;
  } tag_config_t;
  tag_config_t configSend = { };
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

RTC_DATA_ATTR uint8_t stateMachine = STATE_MACHINE_FIRST_START;
RTC_DATA_ATTR bool timeAvailable = false;
RTC_DATA_ATTR uint32_t errorCounter = 0;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long lastMillisReceived = 0;
unsigned long lastMillisPrintStatus = 0;
bool errorWithQueue = false;
bool errorWithPSRAMLow = false;
uint8_t ownMac[6] = { 0 };
long lastMsgMillis = 0;
long lastDataMsgMillis = 0;
File file;
File fileMeta;
bool receiveRunning = false;
static xQueueHandle rxQueue;
bool wifiStarted = true;
String currentFoldername = "";
String currentFilename = "";
uint64_t totalAmountReceivedData = 0;
uint32_t newDataCounter = 0;
uint64_t msgDataTransCnt = 0;
uint64_t msgGotActivatedCnt = 0;
uint64_t msgTagAroundCnt = 0;
uint64_t msgRefusedCnt = 0;
uint64_t msgProxCnt = 0;
uint8_t configCommandToSend = 0;
bool newProximityDataReceived = false;
uint64_t gatewayMsgs = 0;
uint8_t sdhcBuffer[SDHC_BUFFER_SIZE] = { 0 };
uint32_t sdhcBufferPointer = 0;
bool autoCommandChange = false;
bool silent = false;

/** ------ COMMANDS ------ */
String commandToText(uint8_t cmd) {
  String cmdText = "-";
  #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER
    if(cmd == COMMAND_BYTE_NOTHING) { cmdText = "NOTHING"; }
    else if(cmd == COMMAND_BYTE_FORCE_TRACKING) { cmdText = "FORCE"; }
    else if(cmd == COMMAND_BYTE_ACTIVATE) { cmdText = "ACTIVATE"; }
    else if(cmd == COMMAND_BYTE_DEACTIVATE) { cmdText = "DEACTIVATE"; }
    else if(cmd == COMMAND_BYTE_CHANGE_CONFIG) { cmdText = "CHANGECONF"; }
    else if(cmd == COMMAND_BYTE_MAG_CALIBRATION) { cmdText = "MAGCALIB"; }
    else if(cmd == COMMAND_BYTE_DO_NOT_SEND) { cmdText = "DONOTSEND"; }
    else if(cmd == COMMAND_BYTE_TIME_RESYNC) { cmdText = "TIMERESYNC"; }
    else if(cmd == COMMAND_BYTE_TIME_SYNC_ACTIVATION) { cmdText = "TIMESYNCACTIVATION"; }
    else if(cmd == COMMAND_BYTE_ACTIVATE_WHEN_NO_GW) { cmdText = "ACTIVATEWHENNOGW"; }
  #endif
  #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION
    if(cmd == PROXIMITY_COMMAND_NOTHING) { cmdText = "NOTHING"; }
    else if(cmd == PROXIMITY_COMMAND_DO_NOT_SEND) { cmdText = "DONOTSEND"; }
    else if(cmd == PROXIMITY_COMMAND_ACTIVATE) { cmdText = "ACTIVATE"; }
    else if(cmd == PROXIMITY_COMMAND_DEACTIVATE) { cmdText = "DEACTIVATE"; }
    else if(cmd == PROXIMITY_COMMAND_CHANGE_CONFIG) { cmdText = "CHANGECONF"; }
    else if(cmd == PROXIMITY_COMMAND_FULL_RESET) { cmdText = "FULLRESET"; }
    else if(cmd == PROXIMITY_COMMAND_MAG_CALIB) { cmdText = "MAGCALIB"; }
    else if(cmd == PROXIMITY_COMMAND_RESYNC_TIME_BY_WIFI) { cmdText = "WIFIRESYNC"; }
    else if(cmd == PROXIMITY_COMMAND_ACTIVATE_AT_06_00) { cmdText = "ACTIVATE_06"; }
    else if(cmd == PROXIMITY_COMMAND_ACTIVATE_AT_12_00) { cmdText = "ACTIVATE_12"; }
    else if(cmd == PROXIMITY_COMMAND_ACTIVATE_AT_15_00) { cmdText = "ACTIVATE_15"; }
    else if(cmd == PROXIMITY_COMMAND_ACTIVATE_AT_20_00) { cmdText = "ACTIVATE_20"; }
    else if(cmd == PROXIMITY_COMMAND_FIRST_SYNC_TIME_IN_ACTIVATION) { cmdText = "FIRSTSYNCACTIVATION"; }  
  #endif
  return cmdText;
}
/** ------ WIFI UPLOAD ------ */

void uploadClientHeader(String filename, uint32_t chunkSize) {
  client.println("POST " + uploadPath + " HTTP/1.1");
  client.println("Host: " + uploadServer);
  client.println("Authorization: Bearer " + String(DROPBOX_BEARER));
  client.println("Dropbox-API-Arg: {\"path\":\"/GW_" + macAsString() + "_V" + String(GW_VERSION) + filename + "\",\"mode\":\"add\",\"autorename\":true,\"mute\":false,\"strict_conflict\":false}");
  client.println("Content-Type: application/octet-stream");
  client.println("Content-Length: " + String(chunkSize));
  client.println();
}

bool uploadSingleFile(String filename) {
  fs::FS &fs = SD_MMC;
  const uint32_t BUFF_SIZE = 1000;
  uint8_t fileBuffer[BUFF_SIZE];

  File uploadFile = fs.open(filename);
  if(!uploadFile) {
    Serial.println("UPLOAD: failed, could not open " + filename);
    return false;
  }
  Serial.print("UPLOAD: uploading " + filename + " (" + uploadFile.size() + "b) to " + uploadServer);

  if(client.connect(uploadServer.c_str(), uploadPort)) {
    //Serial.println("UPLOAD: connected!");
    uint32_t chunkSize = uploadFile.size();
  
    uploadClientHeader(filename, chunkSize);

    while(uploadFile.available()) {
      int cc = uploadFile.read(fileBuffer, BUFF_SIZE);
      size_t written = client.write(fileBuffer, cc);
      if(written == 0) { client.stop(); return false; }
    }
    
    long startTimer = millis();
    boolean state = false;
    bool success = false;
    String getAll;
    String getBody;
    
    while(true) {
      //Serial.print(".");
      delay(100);      
      while(client.available()) {
        char c = client.read();
        if (c == '\n') {
          if(getAll.length() == 0) { state = true; }
          getAll = "";
        }
        else if(c != '\r') { getAll += String(c); }
        if(state == true) { getBody += String(c); }
        startTimer = millis();
      }
      if(getBody.length() > 0) {
        if(getBody.indexOf("{\"name\": \"") != -1) {
          success = true;
        }
        break;
      }
      if((startTimer + 20000) <= millis()) { break; }
    }
    
    client.stop();
    if(uploadFile) { uploadFile.close(); }
    //Serial.println();
    //Serial.println("BODY: " + getBody);
    if(success) {
      Serial.println(" -> SUCCESS!");
      return true;
    }
    else { Serial.println(" -> FAILED1"); }
  }
  else {
    Serial.println(" -> FAILED2");
  }
  return false;
}

void sdhcGoThroughFilesForUpload() {
  fs::FS &fs = SD_MMC;
  File root = fs.open("/");
  if(!root){ return; }
  if(!root.isDirectory()){ return; }
  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){ // look into all sub dirs
      // only consider files in sub dirs
      File sub = fs.open(file.name());
      //Serial.println("Dir " + String(file.name()));
      if(sub) {
        File subFile;
        uint32_t cntFiles = 0;
        while(true) {
          subFile = sub.openNextFile();
          if(subFile) {
            if(String(subFile.name()).endsWith(fileExtension) || String(subFile.name()).endsWith(".txt")) { // only consider bin and txt files
              if(subFile.size() > 0) {
                cntFiles++;
                //Serial.println(String(cntFiles) + ": " + String(subFile.name()));
                if(uploadSingleFile(String(subFile.name()))) { // uploaded successfully
                  if(uploadRenameFiles) {
                    fs.rename(String(subFile.name()), String(subFile.name())+".up");
                  }
                }
                else { // upload failed, do not try with all other files in directory
                  break;
                }
              }
            }
          }
          else { break; } // no more files
        }
        if(subFile) { subFile.close(); }
        sub.close();
      }
    }
    file = root.openNextFile();
  }
  if(file) { file.close(); }
  root.close();
}

/** ------ VARIOUS FUNCTIONS ------ */

String macAsString() {
  String s;
  for(uint8_t i = 0; i < 6; ++i) {
    char buf[3];
    sprintf(buf, "%02X", ownMac[i]); // J-M-L: slight modification, added the 0 in the format for padding 
    s += buf;
  }
  return s;
}

void printLogo() {
  Serial.println("----------------------------");
  Serial.printf("---WILDFI-TAG-GATEWAY-ONLINE-V%d---\n", GW_VERSION);
  Serial.println("---SW: by Wild Lab---");
  Serial.println("---MAC: " + macAsString() + "---");
  if(timeAvailable) { Serial.println("---TIME: YES---"); }
  else { Serial.println("---TIME: NO---"); }
  Serial.println("----------------------------");
  #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER 
    Serial.println("CONFIGURED FOR MOVEMENT LOGGING");
    if((WILDFI_SOFTWARE_VERSION == 0) && (WILDFI_CONFIG_VERSION == 0)) { Serial.println("CONFIGURED FOR ALL SOFTWARE VERSIONS"); }
    else { Serial.println("CONFIGURED FOR SOFTWARE VERSIONS V" + String(WILDFI_SOFTWARE_VERSION) + "." + String(WILDFI_CONFIG_VERSION)); }
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

/** ------ GET TIME OVER WIFI AND UPLOAD ------ */

bool getTimeOverWifiAndUpload() {
  struct tm timeinfo;
  bool foundWifi = false;
  bool gotTime = false;
  String ssidToConnectTo = "";
  String passwordToConnectTo = "";
  Serial.println("WIFI: started");
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
      if(WiFi.SSID(i) == String(WIFI_SSID1)) {
        ssidToConnectTo = String(WIFI_SSID1);
        passwordToConnectTo = String(WIFI_PASSWORD1);
        foundWifi = true;
      }
      else if(WiFi.SSID(i) == String(WIFI_SSID2)) {
        ssidToConnectTo = String(WIFI_SSID2);
        passwordToConnectTo = String(WIFI_PASSWORD2);
        foundWifi = true;
      }
      else if(WiFi.SSID(i) == String(WIFI_SSID3)) {
        ssidToConnectTo = String(WIFI_SSID3);
        passwordToConnectTo = String(WIFI_PASSWORD3);
        foundWifi = true;
      }
      else if(WiFi.SSID(i) == String(WIFI_SSID4)) {
        ssidToConnectTo = String(WIFI_SSID4);
        passwordToConnectTo = String(WIFI_PASSWORD4);
        foundWifi = true;
      }
      else if(WiFi.SSID(i) == String(WIFI_SSID5)) {
        ssidToConnectTo = String(WIFI_SSID5);
        passwordToConnectTo = String(WIFI_PASSWORD5);
        foundWifi = true;
      }
      else if(WiFi.SSID(i) == String(WIFI_SSID6)) {
        ssidToConnectTo = String(WIFI_SSID6);
        passwordToConnectTo = String(WIFI_PASSWORD6);
        foundWifi = true;
      }
  }
  if(foundWifi) {
    Serial.print("WIFI: connect to ");
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
    Serial.println("WIFI: connected to wifi");

    // get time
    configTime(0, 0, ntpServer);
    if(!getLocalTime(&timeinfo)){ Serial.println("WIFI: failed to obtain time!"); }
    else { gotTime = true; }
    timeinfo.tm_mon++; // starts at 0
    Serial.printf("WIFI: got time: %d.%d.%d %d:%d:%d\n", timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year + 1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    // upload data
    if(uploadEnabled) {
      sdhcGoThroughFilesForUpload();
    }
  }
  else {
    Serial.println("WIFI: no suitable WiFi found!");
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
    // NEW: always sending 250 byte message, because otherwise SOFTWARE_VERSION and CONFIG_VERSION not considered for all commands
    /*if(configCommandToSend != COMMAND_BYTE_CHANGE_CONFIG) {
      uint8_t data[ESPNOW_META_MSG_GATEWAY_AROUND_LEN] = { 0 }; // all zero
      data[0] = ESPNOW_META_MSG_GATEWAY_AROUND;
      data[1] = configCommandToSend;
      if(esp_now_send(broadcastAddress, data, ESPNOW_META_MSG_GATEWAY_AROUND_LEN) != ESP_OK) { printf("SEND ERROR\n"); errorCounter++; }
      else { gatewayMsgs++; }      
    }
    else {*/
      uint8_t data[ESPNOW_META_MSG_GATEWAY_AROUND_V2_LEN] = { 0 }; // all zero
      uint8_t i = 0;
      data[i] = ESPNOW_META_MSG_GATEWAY_AROUND_V2; i++;
      data[i] = configCommandToSend; i++;
      data[i] = configSend.doTheBlink; i++;
      data[i] = configSend.trackerMode; i++;
      data[i] = configSend.trackingDataMode; i++;
      data[i] = configSend.transmissionMethod; i++;
      data[i] = configSend.drMinHdopXTen; i++;
      data[i] = configSend.drImuSeconds >> 8; i++;
      data[i] = configSend.drImuSeconds & 0xFF; i++;
      data[i] = configSend.gps1HzInterruptAfterSeconds >> 8; i++;
      data[i] = configSend.gps1HzInterruptAfterSeconds & 0xFF; i++;
      data[i] = configSend.accFrequency; i++;
      data[i] = configSend.accAvg; i++;
      data[i] = configSend.accRange; i++;
      data[i] = configSend.magFrequency; i++;
      data[i] = configSend.magAccuracy; i++;
      data[i] = configSend.gyroFrequency; i++;
      data[i] = configSend.gyroRange; i++;
      data[i] = configSend.gyroMode; i++;
      data[i] = configSend.useMagnetometer; i++;
      data[i] = configSend.useGyro; i++;
      data[i] = configSend.nightTimeEnter; i++;
      data[i] = configSend.nightTimeMode; i++;
      data[i] = configSend.nightTimeTurnOnHour; i++;
      data[i] = configSend.nightTimeTurnOnMinute; i++;
      data[i] = configSend.nightTimeTurnOffHour; i++;
      data[i] = configSend.nightTimeTurnOffMinute; i++;
      data[i] = configSend.nightTimeDataTransDeepestNightHours >> 24; i++;
      data[i] = configSend.nightTimeDataTransDeepestNightHours >> 16; i++;
      data[i] = configSend.nightTimeDataTransDeepestNightHours >> 8; i++;
      data[i] = configSend.nightTimeDataTransDeepestNightHours; i++;
      data[i] = configSend.nightTimeModeTryDataTransWakeupSeconds >> 8; i++;
      data[i] = configSend.nightTimeModeTryDataTransWakeupSeconds & 0xFF; i++;
      data[i] = configSend.battMinVoltage >> 8; i++;
      data[i] = configSend.battMinVoltage & 0xFF; i++;
      data[i] = configSend.battRestartVoltage >> 8; i++;
      data[i] = configSend.battRestartVoltage & 0xFF; i++;
      data[i] = configSend.dataTransBattMinVoltage >> 8; i++;
      data[i] = configSend.dataTransBattMinVoltage & 0xFF; i++;
      data[i] = configSend.dataTransBattMinVoltageDuringTrans >> 8; i++;
      data[i] = configSend.dataTransBattMinVoltageDuringTrans & 0xFF; i++;
      data[i] = configSend.skipGetTime; i++;
      data[i] = configSend.timeBetweenGetTimeRetriesSeconds >> 8; i++;
      data[i] = configSend.timeBetweenGetTimeRetriesSeconds & 0xFF; i++;
      data[i] = configSend.activationMode; i++;
      data[i] = configSend.activationSource; i++;
      data[i] = configSend.activationByEspNowRetrySeconds >> 8; i++;
      data[i] = configSend.activationByEspNowRetrySeconds & 0xFF; i++;
      data[i] = configSend.dataTransTryEveryFullMinSeldomly >> 8; i++;
      data[i] = configSend.dataTransTryEveryFullMinSeldomly & 0xFF; i++;
      data[i] = configSend.memFullTryEveryFullMinSeldomly >> 8; i++;
      data[i] = configSend.memFullTryEveryFullMinSeldomly & 0xFF; i++;
      data[i] = configSend.accInterruptWatermark >> 8; i++;
      data[i] = configSend.accInterruptWatermark & 0xFF; i++;
      data[i] = configSend.dataTransOutputPower; i++;
      data[i] = configSend.espNowCustomRFCalibration; i++;
      data[i] = configSend.commandByteForceTrackingDurationSeconds >> 8; i++;
      data[i] = configSend.commandByteForceTrackingDurationSeconds & 0xFF; i++;
      data[i] = configSend.espNowDataRate; i++;
      data[i] = configSend.espNowLongRange; i++;
      data[i] = configSend.espNowMinBytesToTransmit >> 24; i++;
      data[i] = configSend.espNowMinBytesToTransmit >> 16; i++;
      data[i] = configSend.espNowMinBytesToTransmit >> 8; i++;
      data[i] = configSend.espNowMinBytesToTransmit; i++;
      data[i] = configSend.espNowMaxBytesToTransmit >> 24; i++;
      data[i] = configSend.espNowMaxBytesToTransmit >> 16; i++;
      data[i] = configSend.espNowMaxBytesToTransmit >> 8; i++;
      data[i] = configSend.espNowMaxBytesToTransmit; i++;
      data[i] = configSend.wifiMinBlocksToTransmit >> 8; i++;
      data[i] = configSend.wifiMinBlocksToTransmit & 0xFF; i++;
      data[i] = configSend.wifiMaxBlocksToTransmit >> 8; i++;
      data[i] = configSend.wifiMaxBlocksToTransmit & 0xFF; i++;
      data[i] = configSend.activityActivationEnabled; i++;
      data[i] = configSend.activityThresholdActiveToInactiveAvg >> 8; i++;
      data[i] = configSend.activityThresholdActiveToInactiveAvg & 0xFF; i++;
      data[i] = configSend.activityThresholdInactiveToActiveThr; i++;
      data[i] = configSend.activityTransmissionInterval >> 8; i++;
      data[i] = configSend.activityTransmissionInterval & 0xFF; i++;
      data[248] = WILDFI_SOFTWARE_VERSION;
      data[249] = WILDFI_CONFIG_VERSION;
      
      if(esp_now_send(broadcastAddress, data, ESPNOW_META_MSG_GATEWAY_AROUND_V2_LEN) != ESP_OK) { printf("SEND ERROR\n"); errorCounter++; }
      else { gatewayMsgs++; }       
    //}
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

    data[i] = configSend.useLeds; i++;
    data[i] = configSend.trackerMode; i++;
    data[i] = configSend.activationMode; i++;
    data[i] = configSend.getFirstTimeOverWiFi; i++;
    data[i] = configSend.getFirstTimeOverGPS; i++;
    data[i] = configSend.tagIdSource; i++;
    data[i] = configSend.imuMode; i++;
    data[i] = configSend.imuBurstMillis >> 24; i++;
    data[i] = configSend.imuBurstMillis >> 16; i++;
    data[i] = configSend.imuBurstMillis >> 8; i++;
    data[i] = configSend.imuBurstMillis; i++;
    data[i] = configSend.environmentActivated; i++;
    data[i] = configSend.timeCorrectionBetweenTags; i++;
    data[i] = configSend.timeCorrectionDiffMs >> 8; i++;
    data[i] = configSend.timeCorrectionDiffMs & 0xFF; i++;
    data[i] = configSend.freeMemoryIfFull; i++;
    data[i] = configSend.accFrequency; i++;
    data[i] = configSend.accAvg; i++;
    data[i] = configSend.accRange; i++;
    data[i] = configSend.magFrequency; i++;
    data[i] = configSend.magAccuracy; i++;
    data[i] = configSend.gyroFrequency; i++;
    data[i] = configSend.gyroRange; i++;
    data[i] = configSend.gyroMode; i++;
    data[i] = configSend.nightTimeEnter; i++;
    data[i] = configSend.nightTimeTurnOnHour; i++;
    data[i] = configSend.nightTimeTurnOnMinute; i++;
    data[i] = configSend.nightTimeTurnOffHour; i++;
    data[i] = configSend.nightTimeTurnOffMinute; i++;
    data[i] = configSend.gpsFixHourBits >> 24; i++;
    data[i] = configSend.gpsFixHourBits >> 16; i++;
    data[i] = configSend.gpsFixHourBits >> 8; i++;
    data[i] = configSend.gpsFixHourBits; i++;
    data[i] = configSend.gpsRandomizeFixes; i++;
    data[i] = configSend.gpsRandomizeFixesPerDay; i++;
    data[i] = configSend.gpsMinHdopTimesTen; i++;
    data[i] = configSend.gpsFirstFixCollectOrbitDataSeconds; i++;
    data[i] = configSend.gpsForcedAfterEveryProximity; i++;
    data[i] = configSend.gpsSyncRTCFrequency; i++;
    data[i] = configSend.proximityFrequencyMinute; i++;
    data[i] = configSend.proximityFrequencyMinuteSeenSomeone; i++;
    data[i] = configSend.proximityListeningIntervalMs >> 8; i++;
    data[i] = configSend.proximityListeningIntervalMs & 0xFF; i++;
    data[i] = configSend.proximityDbm; i++;
    data[i] = configSend.proximityDatarate; i++;
    data[i] = configSend.proximityLongRange; i++;
    data[i] = configSend.proximityAirTimeUs >> 8; i++;
    data[i] = configSend.proximityAirTimeUs & 0xFF; i++;
    data[i] = configSend.activationByGatewayListeningTime >> 8; i++;
    data[i] = configSend.activationByGatewayListeningTime & 0xFF; i++;
    data[i] = configSend.activationByGatewaySleepSeconds >> 8; i++;
    data[i] = configSend.activationByGatewaySleepSeconds & 0xFF; i++;
    data[i] = configSend.battMinVoltage >> 8; i++;
    data[i] = configSend.battMinVoltage & 0xFF; i++;
    data[i] = configSend.battRestartVoltage >> 8; i++;
    data[i] = configSend.battRestartVoltage & 0xFF; i++;
    data[i] = configSend.battMinVoltageDuringTransmission >> 8; i++;
    data[i] = configSend.battMinVoltageDuringTransmission & 0xFF; i++;
    data[i] = configSend.timeWifiOutputPower; i++;
    data[i] = configSend.timeBetweenGetTimeRetriesSeconds >> 8; i++;
    data[i] = configSend.timeBetweenGetTimeRetriesSeconds & 0xFF; i++;
    
    if(esp_now_send(broadcastAddress, data, ALL_MSGS_PAYLOAD_OFFSET_MSG_TYPE_GATEWAY_AROUND_LEN) != ESP_OK) { printf("SEND ERROR\n"); errorCounter++; }
    else { gatewayMsgs++; }
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
    if(autoCommandChange) {
      autoCommandChange = false;
      configCommandToSend = COMMAND_BYTE_NOTHING;
      Serial.printf("AUTO CMD CHANGE: %s\n", commandToText(configCommandToSend).c_str());
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
    if(autoCommandChange) {
      autoCommandChange = false;
      configCommandToSend = PROXIMITY_COMMAND_NOTHING;
      Serial.printf("AUTO CMD CHANGE: %s\n", commandToText(configCommandToSend).c_str());
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

#if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER
bool sdhcGetConfig() {
  char line[1024] = { 0 };
  uint16_t linePointer = 0;
  char lineByte;
  uint16_t configLine = 0;
  fs::FS &fs = SD_MMC;
  File configFile = fs.open(configFilename);
  if(!configFile){
    Serial.println("CONFIG: Failed to open config file, use default CMD = 0x00");
    configCommandToSend = 0;
    return true;
  }

  while(true) {
    linePointer = 0;
    memset(line, 0, 1024);
    while(configFile.available()) {
      lineByte = configFile.read();
      if(lineByte == '\n') { break; }
      else if(lineByte == '\r') { }
      else if(lineByte == ' ') { }
      else { line[linePointer] = lineByte; linePointer++; }
    }
    if(configLine == 1) {
      if(strcmp(line,  "COMMAND_BYTE_NOTHING") == 0) { configCommandToSend = COMMAND_BYTE_NOTHING; }
      else if(strcmp(line, "COMMAND_BYTE_FORCE_TRACKING") == 0) { configCommandToSend = COMMAND_BYTE_FORCE_TRACKING; }
      else if(strcmp(line, "COMMAND_BYTE_ACTIVATE") == 0) { configCommandToSend = COMMAND_BYTE_ACTIVATE; }
      else if(strcmp(line, "COMMAND_BYTE_DEACTIVATE") == 0) { configCommandToSend = COMMAND_BYTE_DEACTIVATE; }
      else if(strcmp(line, "COMMAND_BYTE_CHANGE_CONFIG") == 0) { configCommandToSend = COMMAND_BYTE_CHANGE_CONFIG; }
      else if(strcmp(line, "COMMAND_BYTE_MAG_CALIBRATION") == 0) { configCommandToSend = COMMAND_BYTE_MAG_CALIBRATION; }
      else if(strcmp(line, "COMMAND_BYTE_DO_NOT_SEND") == 0) { configCommandToSend = COMMAND_BYTE_DO_NOT_SEND; }
      else if(strcmp(line, "COMMAND_BYTE_TIME_RESYNC") == 0) { configCommandToSend = COMMAND_BYTE_TIME_RESYNC; }
      else if(strcmp(line, "COMMAND_BYTE_TIME_SYNC_ACTIVATION") == 0) { configCommandToSend = COMMAND_BYTE_TIME_SYNC_ACTIVATION; }
      else if(strcmp(line, "COMMAND_BYTE_ACTIVATE_WHEN_NO_GW") == 0) { configCommandToSend = COMMAND_BYTE_ACTIVATE_WHEN_NO_GW; }
      else { Serial.printf("CONFIG: error %s (%d)\n", line, strlen(line)); return false; }
      Serial.printf("CONFIG: commandByte: 0x%02X (%s)\n", configCommandToSend, line);
    }
    else if(configLine == 3) {
      if(strcmp(line,  "false") == 0) { configSend.doTheBlink = false; }
      else if(strcmp(line,  "true") == 0) { configSend.doTheBlink = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: doTheBlink: %d (%s)\n", configSend.doTheBlink, line);
    }
    else if(configLine == 5) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.trackerMode = val;
      Serial.printf("CONFIG: trackerMode: %d (%s)\n", configSend.trackerMode, line);
    }
    else if(configLine == 7) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.trackingDataMode = val;
      Serial.printf("CONFIG: trackingDataMode: %d (%s)\n", configSend.trackingDataMode, line);
    }
    else if(configLine == 9) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.transmissionMethod = val;
      Serial.printf("CONFIG: transmissionMethod: %d (%s)\n", configSend.transmissionMethod, line);
    }
    else if(configLine == 11) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.drMinHdopXTen = val;
      Serial.printf("CONFIG: drMinHdopXTen: %d (%s)\n", configSend.drMinHdopXTen, line);
    }
    else if(configLine == 13) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.drImuSeconds = val;
      Serial.printf("CONFIG: drImuSeconds: %d (%s)\n", configSend.drImuSeconds, line);
    }
    else if(configLine == 15) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gps1HzInterruptAfterSeconds = val;
      Serial.printf("CONFIG: gps1HzInterruptAfterSeconds: %d (%s)\n", configSend.gps1HzInterruptAfterSeconds, line);
    }
    else if(configLine == 17) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.accFrequency = val;
      Serial.printf("CONFIG: accFrequency: %02X (%s)\n", configSend.accFrequency, line);
    }
    else if(configLine == 19) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.accAvg = val;
      Serial.printf("CONFIG: accAvg: %02X (%s)\n", configSend.accAvg, line);
    }
    else if(configLine == 21) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.accRange = val;
      Serial.printf("CONFIG: accRange: %02X (%s)\n", configSend.accRange, line);
    }
    else if(configLine == 23) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.magFrequency = val;
      Serial.printf("CONFIG: magFrequency: %02X (%s)\n", configSend.magFrequency, line);
    }
    else if(configLine == 25) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.magAccuracy = val;
      Serial.printf("CONFIG: magAccuracy: %02X (%s)\n", configSend.magAccuracy, line);
    }
    else if(configLine == 27) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gyroFrequency = val;
      Serial.printf("CONFIG: gyroFrequency: %02X (%s)\n", configSend.gyroFrequency, line);
    }
    else if(configLine == 29) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gyroRange = val;
      Serial.printf("CONFIG: gyroRange: %02X (%s)\n", configSend.gyroRange, line);
    }
    else if(configLine == 31) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gyroMode = val;
      Serial.printf("CONFIG: gyroMode: %02X (%s)\n", configSend.gyroMode, line);
    }
    else if(configLine == 33) {
      if(strcmp(line,  "false") == 0) { configSend.useMagnetometer = false; }
      else if(strcmp(line,  "true") == 0) { configSend.useMagnetometer = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: useMagnetometer: %d (%s)\n", configSend.useMagnetometer, line);
    }
    else if(configLine == 35) {
      if(strcmp(line,  "false") == 0) { configSend.useGyro = false; }
      else if(strcmp(line,  "true") == 0) { configSend.useGyro = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: useGyro: %d (%s)\n", configSend.useGyro, line);
    }
    else if(configLine == 37) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeEnter = val;
      Serial.printf("CONFIG: nightTimeEnter: %d (%s)\n", configSend.nightTimeEnter, line);
    }
    else if(configLine == 39) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeMode = val;
      Serial.printf("CONFIG: nightTimeMode: %d (%s)\n", configSend.nightTimeMode, line);
    }
    else if(configLine == 41) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeTurnOnHour = val;
      Serial.printf("CONFIG: nightTimeTurnOnHour: %d (%s)\n", configSend.nightTimeTurnOnHour, line);
    }
    else if(configLine == 43) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeTurnOnMinute = val;
      Serial.printf("CONFIG: nightTimeTurnOnMinute: %d (%s)\n", configSend.nightTimeTurnOnMinute, line);
    }
    else if(configLine == 45) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeTurnOffHour = val;
      Serial.printf("CONFIG: nightTimeTurnOffHour: %d (%s)\n", configSend.nightTimeTurnOffHour, line);
    }
    else if(configLine == 47) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeTurnOffMinute = val;
      Serial.printf("CONFIG: nightTimeTurnOffMinute: %d (%s)\n", configSend.nightTimeTurnOffMinute, line);
    }
    else if(configLine == 49) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeDataTransDeepestNightHours = val;
      Serial.printf("CONFIG: nightTimeDataTransDeepestNightHours: %d (%s)\n", configSend.nightTimeDataTransDeepestNightHours, line);
    }
    else if(configLine == 51) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeModeTryDataTransWakeupSeconds = val;
      Serial.printf("CONFIG: nightTimeModeTryDataTransWakeupSeconds: %d (%s)\n", configSend.nightTimeModeTryDataTransWakeupSeconds, line);
    }
    else if(configLine == 53) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.battMinVoltage = val;
      Serial.printf("CONFIG: battMinVoltage: %d (%s)\n", configSend.battMinVoltage, line);
    }
    else if(configLine == 55) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.battRestartVoltage = val;
      Serial.printf("CONFIG: battRestartVoltage: %d (%s)\n", configSend.battRestartVoltage, line);
    }
    else if(configLine == 57) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.dataTransBattMinVoltage = val;
      Serial.printf("CONFIG: dataTransBattMinVoltage: %d (%s)\n", configSend.dataTransBattMinVoltage, line);
    }
    else if(configLine == 59) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.dataTransBattMinVoltageDuringTrans = val;
      Serial.printf("CONFIG: dataTransBattMinVoltageDuringTrans: %d (%s)\n", configSend.dataTransBattMinVoltageDuringTrans, line);
    }
    else if(configLine == 61) {
      if(strcmp(line,  "false") == 0) { configSend.skipGetTime = false; }
      else if(strcmp(line,  "true") == 0) { configSend.skipGetTime = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: skipGetTime: %d (%s)\n", configSend.skipGetTime, line);
    }
    else if(configLine == 63) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.timeBetweenGetTimeRetriesSeconds = val;
      Serial.printf("CONFIG: timeBetweenGetTimeRetriesSeconds: %d (%s)\n", configSend.timeBetweenGetTimeRetriesSeconds, line);
    }
    else if(configLine == 65) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.activationMode = val;
      Serial.printf("CONFIG: activationMode: %d (%s)\n", configSend.activationMode, line);
    }
    else if(configLine == 67) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.activationSource = val;
      Serial.printf("CONFIG: activationSource: %d (%s)\n", configSend.activationSource, line);
    }
    else if(configLine == 69) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.activationByEspNowRetrySeconds = val;
      Serial.printf("CONFIG: activationByEspNowRetrySeconds: %d (%s)\n", configSend.activationByEspNowRetrySeconds, line);
    }
    else if(configLine == 71) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.dataTransTryEveryFullMinSeldomly = val;
      Serial.printf("CONFIG: dataTransTryEveryFullMinSeldomly: %d (%s)\n", configSend.dataTransTryEveryFullMinSeldomly, line);
    }
    else if(configLine == 73) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.memFullTryEveryFullMinSeldomly = val;
      Serial.printf("CONFIG: memFullTryEveryFullMinSeldomly: %d (%s)\n", configSend.memFullTryEveryFullMinSeldomly, line);
    }
    else if(configLine == 75) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.accInterruptWatermark = val;
      Serial.printf("CONFIG: accInterruptWatermark: %d (%s)\n", configSend.accInterruptWatermark, line);
    }
    else if(configLine == 77) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.dataTransOutputPower = val;
      Serial.printf("CONFIG: dataTransOutputPower: %d (%s)\n", configSend.dataTransOutputPower, line);
    }
    else if(configLine == 79) {
      if(strcmp(line,  "false") == 0) { configSend.espNowCustomRFCalibration = false; }
      else if(strcmp(line,  "true") == 0) { configSend.espNowCustomRFCalibration = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: espNowCustomRFCalibration: %d (%s)\n", configSend.espNowCustomRFCalibration, line);
    }
    else if(configLine == 81) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.commandByteForceTrackingDurationSeconds = val;
      Serial.printf("CONFIG: commandByteForceTrackingDurationSeconds: %d (%s)\n", configSend.commandByteForceTrackingDurationSeconds, line);
    }
    else if(configLine == 83) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.espNowDataRate = val;
      Serial.printf("CONFIG: espNowDataRate: %d (%s)\n", configSend.espNowDataRate, line);
    }
    else if(configLine == 85) {
      if(strcmp(line,  "false") == 0) { configSend.espNowLongRange = false; }
      else if(strcmp(line,  "true") == 0) { configSend.espNowLongRange = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: espNowLongRange: %d (%s)\n", configSend.espNowLongRange, line);
    }
    else if(configLine == 87) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.espNowMinBytesToTransmit = val;
      Serial.printf("CONFIG: espNowMinBytesToTransmit: %d (%s)\n", configSend.espNowMinBytesToTransmit, line);
    }
    else if(configLine == 89) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.espNowMaxBytesToTransmit = val;
      Serial.printf("CONFIG: espNowMaxBytesToTransmit: %08X (%s)\n", configSend.espNowMaxBytesToTransmit, line);
    }
    else if(configLine == 91) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.wifiMinBlocksToTransmit = val;
      Serial.printf("CONFIG: wifiMinBlocksToTransmit: %d (%s)\n", configSend.wifiMinBlocksToTransmit, line);
    }
    else if(configLine == 93) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.wifiMaxBlocksToTransmit = val;
      Serial.printf("CONFIG: wifiMaxBlocksToTransmit: %d (%s)\n", configSend.wifiMaxBlocksToTransmit, line);
    }
    else if(configLine == 95) {
      if(strcmp(line,  "false") == 0) { configSend.activityActivationEnabled = false; }
      else if(strcmp(line,  "true") == 0) { configSend.activityActivationEnabled = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: activityActivationEnabled: %d (%s)\n", configSend.activityActivationEnabled, line);
    }
    else if(configLine == 97) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.activityThresholdActiveToInactiveAvg = val;
      Serial.printf("CONFIG: activityThresholdActiveToInactiveAvg: %d (%s)\n", configSend.activityThresholdActiveToInactiveAvg, line);
    }
    else if(configLine == 99) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.activityThresholdInactiveToActiveThr = val;
      Serial.printf("CONFIG: activityThresholdInactiveToActiveThr: %d (%s)\n", configSend.activityThresholdInactiveToActiveThr, line);
    }
    else if(configLine == 101) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.activityTransmissionInterval = val;
      Serial.printf("CONFIG: activityTransmissionInterval: %d (%s)\n", configSend.activityTransmissionInterval, line);
    }
    else {
      //Serial.printf("(CONFIG: %s)\n", line); 
    }

    configLine++;
    if(strlen(line) == 0) { break; }
  }
  Serial.printf("CONFIG: FINAL COMMAND: 0x%02X (%s)\n", configCommandToSend, commandToText(configCommandToSend).c_str());

  return true;
}
#elif GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION
bool sdhcGetConfig() {
  char line[1024] = { 0 };
  uint16_t linePointer = 0;
  char lineByte;
  uint16_t configLine = 0;
  fs::FS &fs = SD_MMC;
  File configFile = fs.open(configFilename);
  if(!configFile){
    Serial.println("CONFIG: Failed to open config file, use default CMD = 0x00");
    configCommandToSend = 0;
    return true;
  }

  while(true) {
    linePointer = 0;
    memset(line, 0, 1024);
    while(configFile.available()) {
      lineByte = configFile.read();
      if(lineByte == '\n') { break; }
      else if(lineByte == '\r') { }
      else if(lineByte == ' ') { }
      else { line[linePointer] = lineByte; linePointer++; }
    }
    if(configLine == 1) {
      if(strcmp(line,  "PROXIMITY_COMMAND_NOTHING") == 0) { configCommandToSend = PROXIMITY_COMMAND_NOTHING; }
      else if(strcmp(line, "PROXIMITY_COMMAND_DO_NOT_SEND") == 0) { configCommandToSend = PROXIMITY_COMMAND_DO_NOT_SEND; }
      else if(strcmp(line, "PROXIMITY_COMMAND_ACTIVATE") == 0) { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE; }
      else if(strcmp(line, "PROXIMITY_COMMAND_DEACTIVATE") == 0) { configCommandToSend = PROXIMITY_COMMAND_DEACTIVATE; }
      else if(strcmp(line, "PROXIMITY_COMMAND_CHANGE_CONFIG") == 0) { configCommandToSend = PROXIMITY_COMMAND_CHANGE_CONFIG; }
      else if(strcmp(line, "PROXIMITY_COMMAND_FULL_RESET") == 0) { configCommandToSend = PROXIMITY_COMMAND_FULL_RESET; }
      else if(strcmp(line, "PROXIMITY_COMMAND_MAG_CALIB") == 0) { configCommandToSend = PROXIMITY_COMMAND_MAG_CALIB; }
      else if(strcmp(line, "PROXIMITY_COMMAND_RESYNC_TIME_BY_WIFI") == 0) { configCommandToSend = PROXIMITY_COMMAND_RESYNC_TIME_BY_WIFI; }
      else if(strcmp(line, "PROXIMITY_COMMAND_ACTIVATE_AT_06_00") == 0) { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE_AT_06_00; }
      else if(strcmp(line, "PROXIMITY_COMMAND_ACTIVATE_AT_12_00") == 0) { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE_AT_12_00; }
      else if(strcmp(line, "PROXIMITY_COMMAND_ACTIVATE_AT_15_00") == 0) { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE_AT_15_00; }
      else if(strcmp(line, "PROXIMITY_COMMAND_ACTIVATE_AT_20_00") == 0) { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE_AT_20_00; }
      else if(strcmp(line, "PROXIMITY_COMMAND_FIRST_SYNC_TIME_IN_ACTIVATION") == 0) { configCommandToSend = PROXIMITY_COMMAND_FIRST_SYNC_TIME_IN_ACTIVATION; }
      else { Serial.printf("CONFIG: error %s (%d)\n", line, strlen(line)); return false; }
      Serial.printf("CONFIG: commandByte: 0x%02X (%s)\n", configCommandToSend, line);
    }
    else if(configLine == 3) {
      if(strcmp(line,  "false") == 0) { configSend.useLeds = false; }
      else if(strcmp(line,  "true") == 0) { configSend.useLeds = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: useLeds: %d (%s)\n", configSend.useLeds, line);
    }
    else if(configLine == 5) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.trackerMode = val;
      Serial.printf("CONFIG: trackerMode: %d (%s)\n", configSend.trackerMode, line);
    }
    else if(configLine == 7) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.activationMode = val;
      Serial.printf("CONFIG: activationMode: %d (%s)\n", configSend.activationMode, line);
    }
    else if(configLine == 9) {
      if(strcmp(line,  "false") == 0) { configSend.getFirstTimeOverWiFi = false; }
      else if(strcmp(line,  "true") == 0) { configSend.getFirstTimeOverWiFi = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: getFirstTimeOverWiFi: %d (%s)\n", configSend.getFirstTimeOverWiFi, line);
    }
    else if(configLine == 11) {
      if(strcmp(line,  "false") == 0) { configSend.getFirstTimeOverGPS = false; }
      else if(strcmp(line,  "true") == 0) { configSend.getFirstTimeOverGPS = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: getFirstTimeOverGPS: %d (%s)\n", configSend.getFirstTimeOverGPS, line);
    }
    else if(configLine == 13) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.tagIdSource = val;
      Serial.printf("CONFIG: tagIdSource: %d (%s)\n", configSend.tagIdSource, line);
    }
    else if(configLine == 15) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.imuMode = val;
      Serial.printf("CONFIG: imuMode: %d (%s)\n", configSend.imuMode, line);
    }
    else if(configLine == 17) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.imuBurstMillis = val;
      Serial.printf("CONFIG: imuBurstMillis: %d (%s)\n", configSend.imuBurstMillis, line);
    }
    else if(configLine == 19) {
      if(strcmp(line,  "false") == 0) { configSend.environmentActivated = false; }
      else if(strcmp(line,  "true") == 0) { configSend.environmentActivated = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: environmentActivated: %d (%s)\n", configSend.environmentActivated, line);
    }
    else if(configLine == 21) {
      if(strcmp(line,  "false") == 0) { configSend.timeCorrectionBetweenTags = false; }
      else if(strcmp(line,  "true") == 0) { configSend.timeCorrectionBetweenTags = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: timeCorrectionBetweenTags: %d (%s)\n", configSend.timeCorrectionBetweenTags, line);
    }
    else if(configLine == 23) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.timeCorrectionDiffMs = val;
      Serial.printf("CONFIG: timeCorrectionDiffMs: %d (%s)\n", configSend.timeCorrectionDiffMs, line);
    }
    else if(configLine == 25) {
      if(strcmp(line,  "false") == 0) { configSend.freeMemoryIfFull = false; }
      else if(strcmp(line,  "true") == 0) { configSend.freeMemoryIfFull = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: freeMemoryIfFull: %d (%s)\n", configSend.freeMemoryIfFull, line);
    }
    else if(configLine == 27) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.accFrequency = val;
      Serial.printf("CONFIG: accFrequency: %d (%s)\n", configSend.accFrequency, line);
    }
    else if(configLine == 29) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.accAvg = val;
      Serial.printf("CONFIG: accAvg: %d (%s)\n", configSend.accAvg, line);
    }
    else if(configLine == 31) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.accRange = val;
      Serial.printf("CONFIG: accRange: %d (%s)\n", configSend.accRange, line);
    }
    else if(configLine == 33) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.magFrequency = val;
      Serial.printf("CONFIG: magFrequency: %d (%s)\n", configSend.magFrequency, line);
    }
    else if(configLine == 35) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.magAccuracy = val;
      Serial.printf("CONFIG: magAccuracy: %d (%s)\n", configSend.magAccuracy, line);
    }
    else if(configLine == 37) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gyroFrequency = val;
      Serial.printf("CONFIG: gyroFrequency: %d (%s)\n", configSend.gyroFrequency, line);
    }
    else if(configLine == 39) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gyroRange = val;
      Serial.printf("CONFIG: gyroRange: %d (%s)\n", configSend.gyroRange, line);
    }
    else if(configLine == 41) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gyroMode = val;
      Serial.printf("CONFIG: gyroMode: %d (%s)\n", configSend.gyroMode, line);
    }
    else if(configLine == 43) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeEnter = val;
      Serial.printf("CONFIG: nightTimeEnter: %d (%s)\n", configSend.nightTimeEnter, line);
    }
    else if(configLine == 45) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeTurnOnHour = val;
      Serial.printf("CONFIG: nightTimeTurnOnHour: %d (%s)\n", configSend.nightTimeTurnOnHour, line);
    }
    else if(configLine == 47) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeTurnOnMinute = val;
      Serial.printf("CONFIG: nightTimeTurnOnMinute: %d (%s)\n", configSend.nightTimeTurnOnMinute, line);
    }
    else if(configLine == 49) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeTurnOffHour = val;
      Serial.printf("CONFIG: nightTimeTurnOffHour: %d (%s)\n", configSend.nightTimeTurnOffHour, line);
    }
    else if(configLine == 51) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.nightTimeTurnOffMinute = val;
      Serial.printf("CONFIG: nightTimeTurnOffMinute: %d (%s)\n", configSend.nightTimeTurnOffMinute, line);
    }
    else if(configLine == 53) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gpsFixHourBits = val;
      Serial.printf("CONFIG: gpsFixHourBits: %d (%s)\n", configSend.gpsFixHourBits, line);
    }
    else if(configLine == 55) {
      if(strcmp(line,  "false") == 0) { configSend.gpsRandomizeFixes = false; }
      else if(strcmp(line,  "true") == 0) { configSend.gpsRandomizeFixes = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: gpsRandomizeFixes: %d (%s)\n", configSend.gpsRandomizeFixes, line);
    }
    else if(configLine == 57) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gpsRandomizeFixesPerDay = val;
      Serial.printf("CONFIG: gpsRandomizeFixesPerDay: %d (%s)\n", configSend.gpsRandomizeFixesPerDay, line);
    }
    else if(configLine == 59) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gpsMinHdopTimesTen = val;
      Serial.printf("CONFIG: gpsMinHdopTimesTen: %d (%s)\n", configSend.gpsMinHdopTimesTen, line);
    }
    else if(configLine == 61) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gpsFirstFixCollectOrbitDataSeconds = val;
      Serial.printf("CONFIG: gpsFirstFixCollectOrbitDataSeconds: %d (%s)\n", configSend.gpsFirstFixCollectOrbitDataSeconds, line);
    }
    else if(configLine == 63) {
      if(strcmp(line,  "false") == 0) { configSend.gpsForcedAfterEveryProximity = false; }
      else if(strcmp(line,  "true") == 0) { configSend.gpsForcedAfterEveryProximity = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: gpsForcedAfterEveryProximity: %d (%s)\n", configSend.gpsForcedAfterEveryProximity, line);
    }
    else if(configLine == 65) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.gpsSyncRTCFrequency = val;
      Serial.printf("CONFIG: gpsSyncRTCFrequency: %d (%s)\n", configSend.gpsSyncRTCFrequency, line);
    }
    else if(configLine == 67) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.proximityFrequencyMinute = val;
      Serial.printf("CONFIG: proximityFrequencyMinute: %d (%s)\n", configSend.proximityFrequencyMinute, line);
    }
    else if(configLine == 69) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.proximityFrequencyMinuteSeenSomeone = val;
      Serial.printf("CONFIG: proximityFrequencyMinuteSeenSomeone: %d (%s)\n", configSend.proximityFrequencyMinuteSeenSomeone, line);
    }
    else if(configLine == 71) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.proximityListeningIntervalMs = val;
      Serial.printf("CONFIG: proximityListeningIntervalMs: %d (%s)\n", configSend.proximityListeningIntervalMs, line);
    }
    else if(configLine == 73) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.proximityDbm = val;
      Serial.printf("CONFIG: proximityDbm: %d (%s)\n", configSend.proximityDbm, line);
    }
    else if(configLine == 75) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.proximityDatarate = val;
      Serial.printf("CONFIG: proximityDatarate: %d (%s)\n", configSend.proximityDatarate, line);
    }
    else if(configLine == 77) {
      if(strcmp(line,  "false") == 0) { configSend.proximityLongRange = false; }
      else if(strcmp(line,  "true") == 0) { configSend.proximityLongRange = true; }
      else { Serial.printf("CONFIG: error %s\n", line); return false; }
      Serial.printf("CONFIG: proximityLongRange: %d (%s)\n", configSend.proximityLongRange, line);
    }
    else if(configLine == 79) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.proximityAirTimeUs = val;
      Serial.printf("CONFIG: proximityAirTimeUs: %d (%s)\n", configSend.proximityAirTimeUs, line);
    }
    else if(configLine == 81) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.activationByGatewayListeningTime = val;
      Serial.printf("CONFIG: activationByGatewayListeningTime: %d (%s)\n", configSend.activationByGatewayListeningTime, line);
    }
    else if(configLine == 83) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.activationByGatewaySleepSeconds = val;
      Serial.printf("CONFIG: activationByGatewaySleepSeconds: %d (%s)\n", configSend.activationByGatewaySleepSeconds, line);
    }
    else if(configLine == 85) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.battMinVoltage = val;
      Serial.printf("CONFIG: battMinVoltage: %d (%s)\n", configSend.battMinVoltage, line);
    }
    else if(configLine == 87) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.battRestartVoltage = val;
      Serial.printf("CONFIG: battRestartVoltage: %d (%s)\n", configSend.battRestartVoltage, line);
    }
    else if(configLine == 89) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.battMinVoltageDuringTransmission = val;
      Serial.printf("CONFIG: battMinVoltageDuringTransmission: %d (%s)\n", configSend.battMinVoltageDuringTransmission, line);
    }
    else if(configLine == 91) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.timeWifiOutputPower = val;
      Serial.printf("CONFIG: timeWifiOutputPower: %d (%s)\n", configSend.timeWifiOutputPower, line);
    }
    else if(configLine == 93) {
      if(strlen(line) == 0) { Serial.printf("CONFIG: error %s\n", line); return false; }
      int val = atoi(line);
      configSend.timeBetweenGetTimeRetriesSeconds = val;
      Serial.printf("CONFIG: timeBetweenGetTimeRetriesSeconds: %d (%s)\n", configSend.timeBetweenGetTimeRetriesSeconds, line);
    }
    else {
      //Serial.printf("(CONFIG: %s)\n", line); 
    }

    configLine++;
    if(strlen(line) == 0) { break; }
  }
  Serial.printf("CONFIG: FINAL COMMAND: 0x%02X (%s)\n", configCommandToSend, commandToText(configCommandToSend).c_str());

  return true;
}
#endif

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
    filenameToTest =  currentFoldername + "_" + appendix + String(i) + filenameStart;
    if(!SD_MMC.exists("/" + currentFoldername + "/" + filenameToTest + fileExtension)) {
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
  int writtenBytes = file.write(sdhcBuffer, sdhcBufferPointer); // receive time in seconds from gateway
  if(writtenBytes == 0) { return false; } // SDHC ejected
  Serial.printf("SDHC: wrote rest (%d bytes)\n", sdhcBufferPointer);
  sdhcBufferPointer = 0;
  return true;
}

bool storeDataOnBuffer(queue_entry_t *queueEntry) { // new function: using buffer, faster if writing data as a multiple of 2
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
    int writtenBytes = file.write(sdhcBuffer, SDHC_BUFFER_SIZE); // receive time in seconds from gateway
    if(writtenBytes == 0) { return false; } // SDHC ejected

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

void sdhcOpenMeta() {
  String path = "/" + currentFoldername + "/METADATA.txt";
  fs::FS &fs = SD_MMC;
  fileMeta = fs.open(path.c_str(), FILE_APPEND);
  if(!fileMeta){
    Serial.println("Failed to open meta file in appending mode");
  } 
}

void sdhcCloseMeta() {
  fileMeta.close();
}

bool storeMetaData(queue_entry_t *queueEntry) { // new function: using buffer
  //long t = millis();
  const uint16_t META_BUFFER_SIZE = 1024 + 1;
  char metaBuffer[META_BUFFER_SIZE];
  uint16_t metaBufferPnt = 0;
  const char *hex = "0123456789ABCDEF";
  uint32_t dataLenWithMacAndLength = queueEntry->message[6];
  dataLenWithMacAndLength += 7;

  uint32_t recTimeSeconds = (uint32_t) (millis() / 1000);
  uint32_t recTimeMilliseconds = millis() % 1000UL;
  if(recTimeSeconds > 9999999) { recTimeSeconds = 9999999; }
  sprintf(metaBuffer, "%07d.%03d", recTimeSeconds, recTimeMilliseconds);
  metaBufferPnt += (7 + 4);
  metaBuffer[metaBufferPnt] = ',';
  metaBufferPnt++;

  for(uint16_t i=0; i<dataLenWithMacAndLength; i++) {
    if(i == 6) {
      uint32_t dataLength = queueEntry->message[i];
      sprintf(metaBuffer + metaBufferPnt, "%03d", queueEntry->message[i]);
      metaBufferPnt += 3;      
    }
    else {
      metaBuffer[metaBufferPnt] = hex[(queueEntry->message[i] >> 4) & 0xF];
      metaBufferPnt++;
      metaBuffer[metaBufferPnt] = hex[(queueEntry->message[i]) & 0xF];
      metaBufferPnt++;
    }
    
    if(i < 5) {
      metaBuffer[metaBufferPnt] = ':';
      metaBufferPnt++;
    }
    if(i == 5) {
      metaBuffer[metaBufferPnt] = ',';
      metaBufferPnt++;
    }
    if(i == 6) {
      metaBuffer[metaBufferPnt] = ',';
      metaBufferPnt++;
    }
  }
  for(uint16_t i=metaBufferPnt; i<(META_BUFFER_SIZE-2); i++) {
    metaBuffer[metaBufferPnt] = '_';
    metaBufferPnt++;    
  }
  metaBuffer[metaBufferPnt] = '\n';
  metaBufferPnt++;
  metaBuffer[metaBufferPnt] = '\0';
  metaBufferPnt++;
  
  int writtenBytes = fileMeta.print(metaBuffer);
  if(writtenBytes == 0) { return false; } // SDHC ejected
  
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
  long tEnterErrorState = millis();
  Serial.println("ERROR STATE: ERROR HAPPENED -> DO NOT CONTINUE");
  while(1) {
    blinkRedLed(10);
    delay(500);
    
    // auto restart after some time
    if(millis() - tEnterErrorState > (ERROR_STATE_AUTO_RESET_SECONDS * 1000UL)) {
      Serial.println("ERROR STATE: AUTO RESET");
      restart(1, true);
    }
    
    // manual restart
    if(Serial.available() > 0) {
      char c = Serial.read();
      if(c != '\n') {
        if(c == '0') {
          Serial.println("ERROR STATE: MANUAL RESET");
          restart(1, true);
        }
      }
    }
  }    
}

void restart(uint64_t seconds, bool stateTransitionToFirstStart) {
  if(stateTransitionToFirstStart) { stateMachine = STATE_MACHINE_FIRST_START; } // go back to very first state
  SD_MMC.end(); // important, eject SDHC
  esp_sleep_enable_timer_wakeup(seconds * 1000UL * 1000UL);
  esp_deep_sleep_disable_rom_logging(); // no more boot messages -> saves couple of ms
  esp_deep_sleep_start();  
}

void storeRestMessagesAfterError() {
  queue_entry_t queueEntry;
  uint64_t restData = 0;
  uint64_t restMsgs = 0;
  bool writeSuccess = false;
  while(xQueueReceive(rxQueue, &queueEntry, 0) == pdTRUE) {
    if(queueEntry.message[7] == ESP_NOW_FLASH_STREAM_FIRST_BYTE) { // flash stream data
      writeSuccess = storeDataOnBuffer(&queueEntry);
    }
    else { // metadata or proximity data
      writeSuccess = storeMetaData(&queueEntry);
    }
    restData += queueEntry.message[6];
    restMsgs++;
    free(queueEntry.message);
  }
  if(!writeRestInBufferOnSDHC()) {
    Serial.println(" -> ERROR storing rest in buffer\n");
  }
  if(!writeSuccess) {
    Serial.println(" -> ERROR writing rest in buffer\n");
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
        else { // metadata or proximity data
          writeSuccess = storeMetaData(&queueEntry);
          newProximityDataReceived = true;
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
          restart(1, true);
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
            sdhcCloseMeta();
            restart(1, true);
          }
        }

        // error possibility: some queue error
        if(errorWithQueue) {
          esp_now_deinit(); // deletes all information on paired devices
          esp_wifi_stop();
          Serial.println("MAIN: SEVERE ERROR - queue failure - RESTARTING");
          storeRestMessagesAfterError();
          sdhcClose();
          sdhcCloseMeta();
          restart(1, true);
        }
      }
      uint32_t timeNeeded = millis() - t;
      //Serial.printf("MAIN: %d msgs in %dms\n", savedMessages, timeNeeded);
      receiveRunning = false;
    }
    
    delay(5); // otherwise watchdog sometimes triggers, 1ms possible because 1000Hz RTOS

    currentMillis = millis();
    if((sdhcBufferPointer > 0) && ((currentMillis - lastMsgMillis) > WRITING_REST_ON_SDHC_AFTER_MS)) { // no active transmission: then store rest of data in buffer on SDHC
      long t = millis();
      if(!writeRestInBufferOnSDHC()) {
        errorWithQueue = true;
        esp_now_deinit();
        esp_wifi_stop();
        errorCounter++;
        Serial.println("MAIN: SEVERE ERROR - SDHC write rest data failed (ejected?) - RESTARTING");
        // do not close SDHC (as not responding)
        restart(1, true);
      }
      sdhcClose(); // close the session
      uint32_t timeNeeded = millis() - t;
      Serial.printf("MAIN: buffer rest stored in %dms\n", timeNeeded);

      if(CHANGE_FILENAME_AFTER_TRANSMISSION) {
        // create file name (AFTER SDHC init)
        if(timeAvailable) { // got time over wifi
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

    if((newProximityDataReceived) && ((currentMillis - lastMsgMillis) > WRITING_REST_ON_SDHC_AFTER_MS)) { // no active transmission: then commit meta data to SDHC (calling close), otherwise not stored on SDHC
      newProximityDataReceived = false;
      long t = millis();
      sdhcCloseMeta(); // also close meta file
      sdhcOpenMeta(); // open again
      uint32_t timeNeeded = millis() - t;
      if(!silent) { Serial.printf("MAIN: committed meta data in %dms\n", timeNeeded); }
    }

    if((sdhcBufferPointer == 0) && (!newProximityDataReceived) && (!uxQueueMessagesWaiting(rxQueue)) && (millis() > (AUTO_RESTART_SECONDS * 1000UL))) {
      esp_now_deinit(); // deletes all information on paired devices
      esp_wifi_stop();
      Serial.println("MAIN: auto restarting, sleeping for " + String(AUTO_RESTART_SLEEP_SECONDS));
      sdhcClose();
      sdhcCloseMeta();
      restart(AUTO_RESTART_SLEEP_SECONDS, true);
    }
  }
}

void setup() {
  struct tm timeinfo = {};

  // init pins and serial
  Serial.begin(115200);
  pinMode(PIN_LED_RED, OUTPUT);
  digitalWrite(PIN_LED_RED, HIGH); // off
  delay(500);

  // get own mac address
  esp_efuse_mac_get_default(ownMac);

  // print the logo
  printLogo();

  // SDHC
  if(!initSDHC()) { Serial.println("INIT: SDHC card mounting failed!"); errorState(); }
  else { Serial.printf("INIT: SDCARD size: %lld MByte, total MBytes: %lld, used MBytes: %lld (%lld Bytes)\n", SD_MMC.cardSize() / (1024 * 1024), SD_MMC.totalBytes() / (1024 * 1024), SD_MMC.usedBytes() / (1024 * 1024), SD_MMC.usedBytes()); }

  if(stateMachine == STATE_MACHINE_FIRST_START) {
    // get time over wifi and upload data
    Serial.println("INIT: STATE_MACHINE_FIRST_START");
    if(getTimeOverWifiAndUpload()) { timeAvailable = true; } // returns true if time was updated
    else { timeAvailable = false; } // WARNING: resetting to false even when got time before
    stateMachine = STATE_MACHINE_RUN;
    restart(1, false); // restarts the system (setup starting again), but not modifying stateMachine
  }
  else if(stateMachine == STATE_MACHINE_RUN) {
    Serial.println("INIT: STATE_MACHINE_RUN");

    // check if enough free space
    uint64_t sdhcFreeBytes = SD_MMC.totalBytes() - SD_MMC.usedBytes();
    uint64_t sdhcFreeMegaBytes = sdhcFreeBytes / (1024 * 1024);
    if(sdhcFreeMegaBytes < SDHC_MINIMUM_FREE_MBYTES) {
      Serial.printf("INIT: ERROR not enough free space on SDHC: %d MBytes (Minimum: %d)\n", sdhcFreeMegaBytes, SDHC_MINIMUM_FREE_MBYTES);
      errorState();
    }
  
    // create folder to write to (AFTER SDHC init)
    deleteEmptyFolders();
    if(timeAvailable) { // got time over wifi
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
    if(timeAvailable) { // got time over wifi
      char currentFilenameChar[100];
      sprintf(currentFilenameChar, "%04d%02d%02d_%02d%02d%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon,timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);  // 20201011_193020xxxxx   
      currentFilename = currentFilenameChar;
      currentFilename = currentFilename + filenameStart;
    }
    else { // wifi failed, use incremental file names
      sdhcGetNextFreeFilenameNumbers();
    }
  
    Serial.println("SDHC: write to file: /" + currentFoldername + "/" + currentFilename + fileExtension);
  
    // camera test -> TAKES UP A LOT OF RAM
    //Serial.printf("INIT: 1 free heap: %d\n",ESP.getFreeHeap());
    //if(!takePhotoAndStore(currentFilename)) { Serial.println("INIT: camera photo failed!"); errorState(); }
    //Serial.printf("INIT: 2 free heap: %d\n",ESP.getFreeHeap());
  
    // get configuration
    if(!sdhcGetConfig()) {
      Serial.println("INIT: config file not OK!");
      errorState();    
    }
    
    // check PSRAM
    Serial.printf("INIT: total PSRAM: %d, free PSRAM: %d, queue entry size: %d\n", ESP.getPsramSize(), ESP.getFreePsram(), sizeof(queue_entry_t));
    if(ESP.getFreePsram() < 200000) { Serial.println("INIT: ERROR PSRAM too low!"); errorState(); }
    
    // create receiving queue
    rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(queue_entry_t)); // WARNING: already reserves memory for everything except the payload data itself (malloc in PSRAM)
    if(rxQueue == NULL) { Serial.println("INIT: queue failed!"); errorState(); }
  
    // init ESP NOW
    if(!initEspNowCustom()) { Serial.println("INIT: error ESP NOW init"); errorState(); }
    int8_t txPwr = 0;
    esp_wifi_get_max_tx_power(&txPwr);
    Serial.printf("INIT: tx power: %d\n", txPwr);
  
    // check heap
    Serial.printf("INIT: free heap: %d\n", ESP.getFreeHeap()); // 259272
  
    // NEW: open file already
    sdhcOpen("/" + currentFoldername + "/" + currentFilename + fileExtension);
    sdhcOpenMeta();
  
    // start listening
    Serial.printf("INIT: all done, START LISTENING (broadcast every %d ms)\n", GATEWAY_MSG_EVERY_MS);
    xTaskCreate(mainTask, "mainTask", 8000, NULL, 1, NULL); // priority 1 = low
  
    // main loop
    while(1) {
      long currentTime = millis();
      // send status text
      if((currentTime - lastMillisPrintStatus > 15000) && (!receiveRunning) && (currentTime - lastMsgMillis > 10000)) { // every 15 seconds, only if no receive running and only if lastMsg is at least 10 seconds ago
        lastMillisPrintStatus = millis();
        if(timeAvailable) {
          getLocalTime(&timeinfo);
        }
        else {
          timeinfo.tm_sec = ((uint32_t) (millis() / 1000)) % 60;
          timeinfo.tm_min = ((uint32_t) ((millis() / 1000) / 60)) % 60;
          timeinfo.tm_hour = ((uint32_t) ((millis() / 1000) / 3600)) % 24;
        }
        if(!silent) {
        #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER 
          Serial.printf("MOV %02d:%02d:%02d: Cmd: %s (0x%02X), Messages (new: %d): Data: %llu, Activation: %llu, TagAround: %llu, Refused: %llu | Bytes stored: %llu, Heap: %d, Psram: %d, SDHC: %llu, Buffer: %d, ErrorCnt: %d, LastMsg: %ds, gwMsgs: %llu\n",
            timeinfo.tm_hour,
            timeinfo.tm_min,
            timeinfo.tm_sec,
            commandToText(configCommandToSend).c_str(),
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
            ((uint32_t)(currentTime - lastMsgMillis)) / 1000,
            gatewayMsgs);
        #endif
        #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION 
          Serial.printf("ID %02X%02X, PROX %02d:%02d:%02d: Cmd: %s (0x%02X), Messages (new: %d): Data: %llu, Activation: %llu, TagAround: %llu, Proximity: %llu, Refused: %llu | Bytes stored: %llu, Heap: %d, Psram: %d, SDHC: %llu, Buffer: %d, ErrorCnt: %d, LastMsg: %ds, gwMsgs: %llu\n",
            ownMac[4],
            ownMac[5],
            timeinfo.tm_hour,
            timeinfo.tm_min,
            timeinfo.tm_sec,
            commandToText(configCommandToSend).c_str(),
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
            ((uint32_t)(currentTime - lastMsgMillis)) / 1000,
            gatewayMsgs);
        #endif
        }
        
        newDataCounter = 0;
        
        if(sdhcFreeBytes - totalAmountReceivedData < 10000000UL) { // less than 10 MByte
          Serial.println("ERROR: not enough space on SDHC");
          restart(1, true);
        }
      }
  
      // send gateway message
      if((!errorWithQueue) && (!errorWithPSRAMLow)) {
        if(RECEVING_FROM_MULTIPLE_TAGS_ENABLED) { // constantly pumping out gateway near messages
          sendGatewayNearMessage();
        }
        else {
          if(currentTime - lastDataMsgMillis > RECEVING_FROM_MULTIPLE_TAGS_NO_SEND_MS) { // only send gateway near message if no receive running and only if last DATA MESSAGE is at least 5s ago
            sendGatewayNearMessage();          
          }
          else {
            Serial.printf("-> NO BROADCAST (PSRAM left %d, queue size %d, buffer pointer %d)\n", ESP.getFreePsram(), uxQueueMessagesWaiting(rxQueue), sdhcBufferPointer);
            delay(1000);
          }
        }
      }
  
      // serial interface
      if(Serial.available() > 0) {
          char c = Serial.read();
          if(c != '\n') {
            Serial.printf("RECEIVED: %c\n", c);
            #if GATEWAY_FOR == GATEWAY_FOR_MOVEMENT_LOGGER
              if(c == 'n') { configCommandToSend = COMMAND_BYTE_NOTHING; }
              else if(c == 'f') { configCommandToSend = COMMAND_BYTE_FORCE_TRACKING; }
              else if(c == 'a') { configCommandToSend = COMMAND_BYTE_ACTIVATE; }
              else if(c == 'd') { configCommandToSend = COMMAND_BYTE_DEACTIVATE; }
              else if(c == 'c') { configCommandToSend = COMMAND_BYTE_CHANGE_CONFIG; }
              else if(c == 'm') { configCommandToSend = COMMAND_BYTE_MAG_CALIBRATION; }
              else if(c == 's') { configCommandToSend = COMMAND_BYTE_DO_NOT_SEND; }
              else if(c == 't') { configCommandToSend = COMMAND_BYTE_TIME_RESYNC; }
              else if(c == 'w') { configCommandToSend = COMMAND_BYTE_TIME_SYNC_ACTIVATION; }
              else if(c == 'x') { configCommandToSend = COMMAND_BYTE_ACTIVATE_WHEN_NO_GW; }
              Serial.printf("CURRENT COMMAND: %s (0x%02X)\n", commandToText(configCommandToSend).c_str(), configCommandToSend);
            #endif
            #if GATEWAY_FOR == GATEWAY_FOR_PROXIMITY_DETECTION
              if(c == 'n') { configCommandToSend = PROXIMITY_COMMAND_NOTHING; }
              else if(c == 's') { configCommandToSend = PROXIMITY_COMMAND_DO_NOT_SEND; }
              else if(c == 'a') { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE; }
              else if(c == 'd') { configCommandToSend = PROXIMITY_COMMAND_DEACTIVATE; }
              else if(c == 'c') { configCommandToSend = PROXIMITY_COMMAND_CHANGE_CONFIG; }
              else if(c == 'r') { configCommandToSend = PROXIMITY_COMMAND_FULL_RESET; }
              else if(c == 'm') { configCommandToSend = PROXIMITY_COMMAND_MAG_CALIB; }
              else if(c == 'w') { configCommandToSend = PROXIMITY_COMMAND_RESYNC_TIME_BY_WIFI; }
              else if(c == '2') { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE_AT_06_00; }
              else if(c == '3') { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE_AT_12_00; }
              else if(c == '4') { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE_AT_15_00; }
              else if(c == '5') { configCommandToSend = PROXIMITY_COMMAND_ACTIVATE_AT_20_00; }
              else if(c == 'f') { configCommandToSend = PROXIMITY_COMMAND_FIRST_SYNC_TIME_IN_ACTIVATION; }
              Serial.printf("CURRENT COMMAND: %s (0x%02X)\n", commandToText(configCommandToSend).c_str(), configCommandToSend);
            #endif
            if(c == '0') {
              Serial.printf("-> RESET COMMANDED\n");
              if((sdhcBufferPointer == 0) && (!newProximityDataReceived) && (!uxQueueMessagesWaiting(rxQueue))) {
                esp_now_deinit(); // deletes all information on paired devices
                esp_wifi_stop();
                Serial.printf("-> RESETTING\n");
                sdhcClose();
                sdhcCloseMeta();
                restart(1, true);
              }
              else { Serial.printf("-> REFUSED\n"); }
            }
            else if(c == 'q') {
              Serial.printf("-> STOP COMMANDED\n");
              if((sdhcBufferPointer == 0) && (!newProximityDataReceived) && (!uxQueueMessagesWaiting(rxQueue))) {
                esp_now_deinit(); // deletes all information on paired devices
                esp_wifi_stop();
                sdhcClose();
                sdhcCloseMeta();
                Serial.printf("-> STOPPED\n");
                while(1) {
                  blinkRedLed(2);
                  delay(500);
                }
              }
              else { Serial.printf("-> REFUSED\n"); }
            }
            else if(c == '1') {
              Serial.printf("-> AUTO COMMAND CHANGE ACTIVE\n");
              autoCommandChange = true;
            }
            else if(c == '9') {
              Serial.printf("SILENT MODE\n");
              if(silent) { silent = false; }
              else { silent = true; }
            }
          }
      }
      delay(GATEWAY_MSG_EVERY_MS); // THIS IS NEW
    }
  }
  else { errorState(); }
}

void loop() { }
