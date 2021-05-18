#include "ESP32TrackerREV2.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

extern "C" void app_main() {
    while(1) {
        bool error = false;
        printf("AWAKE!\n");
        device.initPins();
        printf("START WIFI\n");
        if(!device.initWiFi()) {
            error = true;
        }
        else {
            if(!device.scanForWiFis()) { // DANGER, BROWNOUTS!
                error = true;
            }
        }

        if(error) {
            device.blink(B_RED, B_RED, B_RED);
            printf("ERROR\n");
        }
        else {
            while (1) {
                if(device.wiFiScanCompleted()) {
                    printf("SCAN DONE\n");
                    uint8_t numWiFis = device.printWiFiScanResults();
                    if(numWiFis == 0) {
                        device.blink(B_RED);
                    }
                    else if(numWiFis == 1) {
                        device.blink(B_GREEN);
                    }
                    else if(numWiFis == 2) {
                        device.blink(B_GREEN, B_GREEN);
                    }
                    else {
                        device.blink(B_GREEN, B_GREEN, B_GREEN);
                    }
                    break;
                } else { 
                    device.delay(100);
                    continue;
                }
            }
        }

        device.enableInternalRTCInterruptInDeepSleep(10);
        device.deepSleep();
    }
}