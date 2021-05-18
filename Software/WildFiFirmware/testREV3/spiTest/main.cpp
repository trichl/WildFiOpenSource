#include "ESP32TrackerREV3.h"

ESP32TrackerREV3 device = ESP32TrackerREV3();

uint8_t *dataInDMA = NULL;

void readFeatures() {
    uint32_t res = 0;
    res = device.flash.getFeatures(GET_FEATURES_BLOCK_LOCK);
    printf("BLOCK LOCK: %d\n", res); // 124 after reset = everything locked
    res = device.flash.getFeatures(GET_FEATURES_CONF);
    printf("CONF: %d\n", res); // 16 after reset = ECC protection active after boot
    res = device.flash.getFeatures(GET_FEATURES_STATUS);
    printf("STATUS: %d\n", res); // 0 after reset (normally, also seen 32 = ECCS1 status = >8 bit-errors, not corrected)
    res = device.flash.getFeatures(GET_FEATURES_DIE_SEL);
    printf("DIE SELECT: %d\n\n", res);
}

uint8_t readSomething(uint32_t address, uint16_t len) {
    uint8_t firstByte = 0;
    if(!device.flash.read(address, 0, dataInDMA, len)) {
        printf("ERROR READ\n");
        return 0;
    }
    printf("READ %08X: ", address);
    firstByte = dataInDMA[0];
    for(uint16_t i=0; i<len; i++) {
        printf("%02X", dataInDMA[i]);
    }
    printf("\n\n");
    return firstByte;
}

void writeSomething(uint32_t address, uint16_t len, uint8_t start) {
    for(uint16_t i=0; i<len; i++) {
        dataInDMA[i] = start + i;
    }
    if(!device.flash.write(address, dataInDMA, len)) {
        printf("ERROR WRITE\n");
    }
    else {
        printf("WRITE %08X DONE\n\n", address);
    }
}

void test1() {
    readFeatures();
    if(!device.flash.erase(2046)) { // delete PRE-LAST BLOCK = 2046 = page 130944 .. 131007
        printf("ERROR!\n");
    }
    if(!device.flash.erase(2047)) { // delete VERY LAST BLOCK = 2047 = page 131008 .. 131071
        printf("ERROR!\n");
    }

    readSomething(131006, MT29_CACHE_SIZE_EXTENDED); // FF (deleted) - OK
    readSomething(131007, MT29_CACHE_SIZE_EXTENDED); // FF (deleted) - OK

    readSomething(131008, MT29_CACHE_SIZE_EXTENDED); // FF (deleted) - OK
    readSomething(131009, MT29_CACHE_SIZE_EXTENDED); // FF (deleted) - OK
    readSomething(131070, MT29_CACHE_SIZE_EXTENDED); // FF (deleted) - OK
    readSomething(131071, MT29_CACHE_SIZE_EXTENDED); // FF (deleted) - OK

    writeSomething(131007, MT29_CACHE_SIZE, 5); // write page 63 of pre-last block
    writeSomething(131008, MT29_CACHE_SIZE, 4); // write page 0 of last block
    writeSomething(131009, MT29_CACHE_SIZE, 3); // write page 1 of last block
    writeSomething(131071, MT29_CACHE_SIZE, 2); // write page 63 of last block

    readSomething(131006, MT29_CACHE_SIZE_EXTENDED); // FF (never written) - OK
    readSomething(131007, MT29_CACHE_SIZE_EXTENDED); // values - starts with 5 ok
    readSomething(131008, MT29_CACHE_SIZE_EXTENDED); // values - ALSO STARTS WITH 5 NOK
    readSomething(131009, MT29_CACHE_SIZE_EXTENDED); // values - ALSO STARTS WITH 5 NOK
    readSomething(131070, MT29_CACHE_SIZE_EXTENDED); // FF (never written) - ALSO STARTS WITH 5 NOK
    readSomething(131071, MT29_CACHE_SIZE_EXTENDED); // values  - ALSO STARTS WITH 5 NOK

    printf("---------\n");

    if(!device.flash.erase(2047)) { // delete ONLY very last block
        printf("ERROR!\n");
    }

    readSomething(131006, MT29_CACHE_SIZE_EXTENDED); // FF (never written)
    readSomething(131007, MT29_CACHE_SIZE_EXTENDED); // should still be there
    readSomething(131008, MT29_CACHE_SIZE_EXTENDED); // FF (deleted)
    readSomething(131009, MT29_CACHE_SIZE_EXTENDED); // FF (deleted)
    readSomething(131070, MT29_CACHE_SIZE_EXTENDED); // FF (never written)
    readSomething(131071, MT29_CACHE_SIZE_EXTENDED); // FF (deleted)
    readFeatures();
}

void testBadBlocks() {
    if(!device.flash.selfTestBadBlocks()) {
        printf("ERROR SELFTEST");
    }
}

void test2() {
    readFeatures();
    device.flash.erase(0);

    readSomething(0, 50);
    readSomething(1, 50); 
    readSomething(2, 50); 
    readSomething(3, 50); 

    writeSomething(1, MT29_CACHE_SIZE, 1);
    writeSomething(2, MT29_CACHE_SIZE, 2);
    writeSomething(7, MT29_CACHE_SIZE, 7);

    readSomething(0, 50);
    readSomething(1, 50); 
    readSomething(2, 50);
    readSomething(3, 50);
    readSomething(4, 50);
    readSomething(5, 50);
    readSomething(6, 50);
    readSomething(7, 50);

    readFeatures();
}

void test3() { // FAILS
    readFeatures();
    device.flash.erase(MT29_NUMBER_BLOCKS-1);
    readSomething(0x0000, 50);
    readSomething(0x0001, 50);
    readSomething(0xFFFD, 50);
    readSomething(0xFFFE, 50);
    readSomething(0xFFFF, 50);

    writeSomething(0xFFFE, MT29_CACHE_SIZE, 5);
    readSomething(0x0000, 50);
    readSomething(0x0001, 50);
    readSomething(0xFFFD, 50);
    readSomething(0xFFFE, 50);
    readSomething(0xFFFF, 50);

    readFeatures();
}

void test4() { // PASS
    const uint16_t NUMBER_OF_PAGES_IN_TEST = (64*3);

    readFeatures();
    printf("START DELETE ALL\n");
    if(!device.flash.fullErase()) {
        printf("ERROR FULL ERASE!\n");
        readFeatures();
        return;
    }
    printf("END DELETE ALL\n");
    readFeatures();

    uint8_t startByte = 0;
    uint8_t readStartByte = 0;

    for(uint32_t i=0; i<NUMBER_OF_PAGES_IN_TEST; i++) {
        printf("START ITERATION %d\n", i);
        writeSomething(i, MT29_CACHE_SIZE, startByte);

        if(i>0) {
            readStartByte = readSomething(i-1, 10);
            if(startByte == 0) {
                if(readStartByte != 0xFF) {
                    printf("ERROR, STOP1\n");
                }
            }
            else {
                if(readStartByte != (startByte-1)) {
                    printf("ERROR, STOP2\n");
                    break;
                }
            }
        }
        readStartByte = readSomething(i, 10);
        if(readStartByte != startByte) {
            printf("ERROR, STOP3\n");
            break;
        }
        readStartByte = readSomething(i+1, 10);
        if(readStartByte != 0xFF) {
            printf("ERROR, STOP4\n");
            break;
        }
        
        printf("NEXT ITERATION\n");

        if(startByte == 255) {
            startByte = 0;
        }
        else {
            startByte++;
        }
        device.delay(10);
    }
    readFeatures();
}

void test5() {
    device.flash.erase(0);

    readSomething(0, 10);
    readSomething(1, 10);
    readSomething(63, 10);

    readSomething(64, 10);
    readSomething(65, 10);
    readSomething(127, 10);

    readSomething(128, 10);
    readSomething(129, 10);

    writeSomething(0, MT29_CACHE_SIZE, 11);
    writeSomething(1, MT29_CACHE_SIZE, 12);
    writeSomething(64, MT29_CACHE_SIZE, 13);
    writeSomething(128, MT29_CACHE_SIZE, 9);

    readSomething(0, 10);
    readSomething(1, 10);
    readSomething(2, 10);
    readSomething(63, 10);

    readSomething(64, 10);
    readSomething(65, 10);
    readSomething(127, 10);

    readSomething(128, 10);
    readSomething(129, 10);

    printf("ERASE\n");
    device.flash.erase(0);

    readSomething(0, 10);
    readSomething(1, 10);
    readSomething(2, 10);
    readSomething(63, 10);

    readSomething(64, 10);
    readSomething(65, 10);
    readSomething(127, 10);

    readSomething(128, 10);
    readSomething(129, 10);

    printf("ERASE\n");
    device.flash.erase(1);

    readSomething(0, 10);
    readSomething(1, 10);
    readSomething(2, 10);
    readSomething(63, 10);

    readSomething(64, 10);
    readSomething(65, 10);
    readSomething(127, 10);

    readSomething(128, 10);
    readSomething(129, 10);

    printf("ERASE\n");
    device.flash.erase(2);

    readSomething(0, 10);
    readSomething(1, 10);
    readSomething(2, 10);
    readSomething(63, 10);

    readSomething(64, 10);
    readSomething(65, 10);
    readSomething(127, 10);

    readSomething(128, 10);
    readSomething(129, 10);
}

void test6() {
    readSomething(0, 10);
    readSomething(64, 10);
    readSomething(128, 10);
    readSomething(129, 10);
    readSomething(MT29_NUMBER_PAGES-1, 10);
    readSomething(MT29_NUMBER_PAGES, 10);
}

extern "C" void app_main() {
    while(1) {
        device.setCPUSpeed(ESP32_10MHZ);
        printf("WAKE FROM DEEP SLEEP: %d\n", device.getWakeUpReason());
        device.delay(7000);
        if(!device.flashPowerOn()) {
            printf("ERROR\n");
        }
        gpio_drive_cap_t strength;
        gpio_get_drive_capability(PIN_POWER2, &strength);
        printf("POWER2 STRENGTH: %d\n\n", strength); // 2 = 40mA (default), 3 = 80mA

        if(!device.flash.createBuffer(&dataInDMA, MT29_CACHE_SIZE)) {
            printf("ERROR BUFF\n");
        }
        else {
            test3();
        }
        

        printf("SLEEPY TIME\n");
        device.deepSleep();
    }
}