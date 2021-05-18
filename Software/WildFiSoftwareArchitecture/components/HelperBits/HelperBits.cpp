#include "HelperBits.h"

void HelperBits::addData4_AndIncrementPointer(uint32_t d, uint8_t *array, uint16_t *arrayPointer) {
    if(array == NULL) { return; }
    array[(*arrayPointer)] = d >> 24; array[(*arrayPointer)+1] = d >> 16; array[(*arrayPointer)+2] = d >>  8; array[(*arrayPointer)+3] = d; (*arrayPointer) += 4;
}

void HelperBits::addData2Signed_AndIncrementPointer(int16_t d, uint8_t *array, uint16_t *arrayPointer) { // WARNING: signed!
    if(array == NULL) { return; }
    array[(*arrayPointer)] = d >> 8; array[(*arrayPointer)+1] = d; (*arrayPointer) += 2;
}

void HelperBits::addData2_AndIncrementPointer(uint16_t d, uint8_t *array, uint16_t *arrayPointer) {
    if(array == NULL) { return; }
    array[(*arrayPointer)] = d >> 8; array[(*arrayPointer)+1] = d; (*arrayPointer) += 2;
}

void HelperBits::addData1_AndIncrementPointer(uint8_t d, uint8_t *array, uint16_t *arrayPointer) {
    if(array == NULL) { return; }
    array[(*arrayPointer)] = d; (*arrayPointer)++;
}

void HelperBits::addData4(uint32_t d, uint8_t *pointer) {
    if(pointer == NULL) { return; }
    pointer[0] = d >> 24; pointer[1] = d >> 16; pointer[2] = d >>  8; pointer[3] = d;
}

void HelperBits::addData4Signed(int32_t d, uint8_t *pointer) {
    if(pointer == NULL) { return; }
    pointer[0] = d >> 24; pointer[1] = d >> 16; pointer[2] = d >>  8; pointer[3] = d;
}

void HelperBits::addData2(uint16_t d, uint8_t *pointer) {
    if(pointer == NULL) { return; }
    pointer[0] = d >> 8; pointer[1] = d;
}

void HelperBits::addData2Signed(int16_t d, uint8_t *pointer) {
    if(pointer == NULL) { return; }
    pointer[0] = d >> 8; pointer[1] = d;
}

void HelperBits::addData1(uint8_t d, uint8_t *pointer) {
    if(pointer == NULL) { return; }
    pointer[0] = d;
}

uint8_t HelperBits::intToBCD(uint8_t num) {
	uint8_t lw, up;
	lw = num % 10;
	up = num / 10;
	return lw | (up << 4);
}

uint8_t HelperBits::BCDToInt(uint8_t bcd) {
	uint8_t lw, up;
	lw = bcd & 0x0f;
	up = bcd >> 4;
	return (up * 10) + lw;
}

uint8_t HelperBits::setBit(uint8_t reg, uint8_t pos, bool val) {
	if(val) {
		return reg |= 1UL << pos;
	}
	return reg &= ~(1UL << pos);
}

bool HelperBits::getBit(uint8_t reg, uint8_t pos) {
	return (reg >> pos) & 1UL;
}

uint32_t HelperBits::to32KHz(uint32_t ms) {
	return ms / 31UL;
}

void HelperBits::addInt16AsHexToCharArray(char *array, uint16_t startPosition, uint16_t value) {
    const char hexchars[17] = "0123456789ABCDEF";
    /*if(strlen(array) < startPosition + 4) {
        return;
    }*/
    array[startPosition+0] = hexchars[(value >> 12) & 0xF];
    array[startPosition+1] = hexchars[(value >> 8) & 0xF];
    array[startPosition+2] = hexchars[(value >> 4) & 0xF];
    array[startPosition+3] = hexchars[(value >> 0) & 0xF];
}

void HelperBits::addInt32AsHexToCharArray(char *array, uint16_t startPosition, uint32_t value) {
    const char hexchars[17] = "0123456789ABCDEF";
    /*if(strlen(array) < startPosition + 4) {
        return;
    }*/
	array[startPosition+0] = hexchars[(value >> 28) & 0xF];
    array[startPosition+1] = hexchars[(value >> 24) & 0xF];
    array[startPosition+2] = hexchars[(value >> 20) & 0xF];
    array[startPosition+3] = hexchars[(value >> 16) & 0xF];
	
    array[startPosition+4] = hexchars[(value >> 12) & 0xF];
    array[startPosition+5] = hexchars[(value >> 8) & 0xF];
    array[startPosition+6] = hexchars[(value >> 4) & 0xF];
    array[startPosition+7] = hexchars[(value >> 0) & 0xF];
}
