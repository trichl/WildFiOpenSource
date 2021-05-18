#ifndef HelperBits_h
#define HelperBits_h

#include <stdint.h>
#include <stdio.h>

// for printing bits in printf, example: printf("TEST "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(var));
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

class HelperBits {
	public:
    static void addData4_AndIncrementPointer(uint32_t d, uint8_t *array, uint16_t *arrayPointer);
    static void addData2Signed_AndIncrementPointer(int16_t d, uint8_t *array, uint16_t *arrayPointer);
    static void addData2_AndIncrementPointer(uint16_t d, uint8_t *array, uint16_t *arrayPointer);
    static void addData1_AndIncrementPointer(uint8_t d, uint8_t *array, uint16_t *arrayPointer);

    static void addData4(uint32_t d, uint8_t *pointer);
    static void addData4Signed(int32_t d, uint8_t *pointer);
    static void addData2(uint16_t d, uint8_t *pointer);
    static void addData2Signed(int16_t d, uint8_t *pointer);
    static void addData1(uint8_t d, uint8_t *pointer);

		static uint8_t intToBCD(uint8_t num);
		static uint8_t BCDToInt(uint8_t num);
		static uint8_t setBit(uint8_t reg, uint8_t pos, bool val);
		static bool getBit(uint8_t reg, uint8_t pos);
		static uint32_t to32KHz(uint32_t ms);
		static void addInt16AsHexToCharArray(char *array, uint16_t startPosition, uint16_t value); // char array needs to be at least 4 bytes long (+ terminating zero)
		static void addInt32AsHexToCharArray(char *array, uint16_t startPosition, uint32_t value); // char array needs to be at least 8 bytes long (+ terminating zero)
};

#endif
