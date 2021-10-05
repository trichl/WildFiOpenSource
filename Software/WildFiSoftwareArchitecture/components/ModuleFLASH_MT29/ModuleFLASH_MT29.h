#ifndef FLASH_MT29_h
#define FLASH_MT29_h

#include <stdint.h>
#include "InterfaceSPI.h" // needed for spi object and SPI class definition
#include "InterfaceTiming.h" // needed delay

// IMPORTANT: pages can be partially programmed, but only max. 8 times! only 0s can be set, 1s not, means no overwriting is possible after (partial) programming (otherwise erase of whole block = 64 pages necessary)
	// datasheet says: 4 times
	// TESTED: ECC only valid when using 512 or 1024 partial programming blocks!
		// 4 x 512 -> OK
		// 256 + 256 + 512 (1024 empty) -> ECC error
		// 1 x 512 (rest empty) -> OK
		// 1 x 256 (rest empty) -> OK
		// 2 x 256 (rest empty) -> ECC error
		// 2 x 512 (rest empty) -> OK
// IMPORTANT: partial programming should be avoided! means 2048 bytes should be flashed as smallest unit (or less, but not written more)
// IMPORTANT: ease command deletes only blocks = 64 pages, smaller not possible
// LAYOUT: 1 page = 2048 byte + 128 byte (ECC), 64 pages = 1 block, device has 2048 blocks (1024 in plane 0, 1024 in plane1) = 2048 * 64 * 2048 = 256MByte
// IMPORTANT: BLOCKS WITH UNEVEN NUMBER ARE LOCATED IN SECOND PLANE -> ADJUST THE OFFSFET!!!
// BAD BLOCK AFTER SHIPPING CHECK: 1st page of block, byte 2048 = 0x00 = bad block

// TODO: CREATE MAP FOR BAD BLOCKS AFTER SELF-TEST -> DON'T USE THESE PAGES THEN, STORE IN ESP MEMORY


#define MT29_CMD_GET_FEATURES					0x0F
typedef enum {
    GET_FEATURES_BLOCK_LOCK = 0xA0,
	GET_FEATURES_CONF = 0xB0,
	GET_FEATURES_STATUS = 0xC0,
	GET_FEATURES_DIE_SEL = 0xD0
} addr_get_features_t;
typedef enum {
    MT29_SEQ_WRITE_STATUS_MEMORY_FULL = 0,
	MT29_SEQ_WRITE_STATUS_SUCCESS = 1,
	MT29_SEQ_WRITE_STATUS_BUFFER_ERROR = 2,
	MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR = 3,
	MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR = 4
} sequential_write_status_t;
typedef enum {
    MT29_COPY_DATA_1  = 0,
	MT29_COPY_DATA_2 = 1,
	MT29_COPY_DATA_3 = 2,
	MT29_COPY_DONE = 3
} sequential_write_state_internal_t;

#define MT29_CMD_SET_FEATURES					0x1F
#define MT29_CMD_WRITE_ENABLE					0x06
#define MT29_CMD_PROGRAM_LOAD					0x02
#define MT29_CMD_PROGRAM_EXECUTE				0x10
#define MT29_CMD_PAGE_READ						0x13
#define MT29_CMD_READ_CACHE_X1					0x03
#define MT29_CMD_BLOCK_ERASE					0xD8

#define MT29_CACHE_SIZE							2048		// equals size of a single page, don't touch 128 bytes for ECC at end
#define MT29_CACHE_SIZE_EXTENDED				2176		// with 128 bytes for ECC
#define MT29_NUMBER_PAGES						131072 		// each with MT29_CACHE_SIZE bytes
#define MT29_PAGES_PER_BLOCK					64
#define MT29_NUMBER_BLOCKS						2048
#define MT29_NUMBER_BYTES						MT29_NUMBER_PAGES * MT29_CACHE_SIZE

class FLASH_MT29 {
	public:
		FLASH_MT29();
		/** user functions */
		bool write(uint32_t pageAddress, uint8_t *dataResult, uint16_t dataLen); // dataResult should be DMA allocated memory, pageAddress = 0 - 131071, byteOffset = 0, dataLen = 1 - 2048
		bool partialWrite(uint32_t pageAddress, uint16_t byteOffset, uint8_t *dataResult, uint16_t dataLen); // dataResult should be DMA allocated memory, pageAddress = 0 - 131071, byteOffset = 0 - 2047, dataLen = 1 - 2048
		bool partialWriteMock(uint32_t pageAddress, uint16_t byteOffset, uint8_t *dataResult, uint16_t dataLen); // for testing
		bool read(uint32_t pageAddress, uint16_t byteOffset, uint8_t *dataResult, uint16_t dataLen); // dataResult should be DMA allocated memory, pageAddress = 0 - 131071, byteOffset = 0 - 2047, dataLen = 1 - 2048
		bool readWithECC(uint32_t pageAddress, uint16_t byteOffset, uint8_t *dataResult, uint16_t dataLen); // ONLY FOR 512 OR 1024 OR 2048 BYTE PARTIAL WRITES, dataResult should be DMA allocated memory, pageAddress = 0 - 131071, byteOffset = 0 - 2047, dataLen = 1 - 2048
		bool erase(uint16_t block);
		bool eraseMock(uint16_t block); // for testing
		bool fullErase(); // takes 2 seconds
		bool createBuffer(uint8_t **data, uint16_t len); // allocate memory in DMA
		bool selfTestBadBlocks(); // needs 364ms
		uint32_t getFeatures(addr_get_features_t addr);
		bool fitsIntoMemory(uint32_t currentPageAddress, uint16_t currentByteOffset, uint32_t dataLen);
		//sequential_write_status_t sequentialWrite(uint32_t &pageAddressStart, uint16_t &byteOffsetStart, uint8_t *data1, uint32_t dataLen1, uint8_t *data2, uint32_t dataLen2, uint8_t *data3, uint32_t dataLen3, bool debug = false, uint16_t maxIterations = 20); // ~20ms @80MHz
		bool printFlash(uint32_t addressStart, uint32_t numberPages, uint16_t lenPerPage = MT29_CACHE_SIZE, bool onlyData = false);

		/** FIFO memory functions */
		uint32_t fifoGetFreeSpace(uint16_t blocksErasedPointer, uint32_t currentPageAddress, uint16_t currentByteOffset, uint32_t fifoSizePages = MT29_NUMBER_PAGES);
		sequential_write_status_t fifoPushSimple(uint16_t blocksErasedPointer, uint32_t &pageAddressStart, uint16_t &byteOffsetStart, uint8_t *data, uint32_t dataLen, bool readBack, bool debug, uint16_t maxIterations = 128, uint32_t fifoSizePages = MT29_NUMBER_PAGES);
		sequential_write_status_t fifoPush(uint16_t blocksErasedPointer, uint32_t &pageAddressStart, uint16_t &byteOffsetStart, uint8_t *data1, uint32_t dataLen1, uint8_t *data2, uint32_t dataLen2, uint8_t *data3, uint32_t dataLen3, bool debug = false, uint16_t maxIterations = 128, uint32_t fifoSizePages = MT29_NUMBER_PAGES);
		uint16_t fifoGetNumberOfPopableBlocks(uint16_t blocksErasedPointer, uint32_t currentPageAddress, uint32_t fifoSizePages = MT29_NUMBER_PAGES);
		uint32_t fifoGetNumberOfPopableBytes(uint32_t sendBytePointer, uint32_t writeBytePointer);
		uint16_t fifoIncrementBlocksErasedPointer(uint16_t blocksErasedPointer, uint32_t fifoSizePages = MT29_NUMBER_PAGES);
		bool fifoPopDelete(uint16_t &blocksErasedPointer, uint32_t currentPageAddress, uint32_t fifoSizePages = MT29_NUMBER_PAGES, bool debug = false);

	private:
		/** driver internal functions */
		bool waitUntilFinished();
		bool writeEnable();
		bool unlockAllBlocks();
		bool loadProgram(bool plane, uint16_t offset, const uint8_t *data, uint16_t len);
		bool executeProgram(uint32_t address);
		bool readPage(uint32_t address);
		bool readFromCache(bool plane, uint16_t offset, uint8_t *data, uint16_t len);
		bool blockEraseInternal(uint32_t address);
		bool getPlane(uint32_t pageAddress);
};

#endif
