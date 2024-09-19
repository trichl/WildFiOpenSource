#include "ModuleFLASH_MT29.h"

FLASH_MT29::FLASH_MT29() {
	
}

uint16_t FLASH_MT29::readID() {
	// 0 address bytes, 1 dummy, 2 byte data
	// simulate dummy byte by using 1 adress bytes
	uint16_t id = (uint16_t) spi.readMax4Byte(MT29_CMD_READ_ID, 1, 0, 1, 2);
	return id;
}

bool FLASH_MT29::waitOnID() {
    uint16_t flashID = 0;
    const uint16_t MAX_CHECKS = 500; // normally after 34 checks when flashPowerOn without delay
    uint16_t i = 0;
    while(true) {
        flashID = readID();
        if(flashID == MT29_FLASH_ID) { break; }
        if(i > MAX_CHECKS) { return false; }
        i++;
    }
    return true;  
}

bool FLASH_MT29::reset() {
	// WARNING: not working if flash just started without any delay (because SPI returns 0x00 as default if device is not yet responding)
	const uint8_t WAIT_TIMEOUT = 250;
	uint16_t loopCnt = 0;
	uint32_t status = 0;
	uint32_t result = spi.readMax4Byte(MT29_CMD_RESET, 1, 0, 0, 0);
	if(result != 0) { return false; }

	// polling status register (oip = 1 immediately after reset)
	while(1) {
		status = getFeatures(GET_FEATURES_STATUS);
		if(status == 0) { // complete status register should be zero (including OIP)
			break;
		}
		loopCnt++;
		if(loopCnt > WAIT_TIMEOUT) { // timeout waiting
			return false;
		}
		//Timing::delay(3);
	}
	return true;
}

uint32_t FLASH_MT29::fifoGetFreeSpace(uint16_t blocksErasedPointer, uint32_t currentPageAddress, uint16_t currentByteOffset, uint32_t fifoSizePages) {
	// currentPageAddress = 0 .. 131071, currentByteOffset = 0 - 2047
	// blocksErasedPointer = 0 .. 2047 -> if 0 then nothing erased yet -> if 2048 then ERROR
	uint32_t pagesErasedPointer = blocksErasedPointer * MT29_PAGES_PER_BLOCK;
	uint32_t spaceLeft = 0;
	if(currentPageAddress >= fifoSizePages) {
		//printf("ERROR1\n");
		return 0; // error
	}
	if(pagesErasedPointer >= fifoSizePages) { // 2048 
		//printf("ERROR2\n");
		return 0; // error
	}
	if(pagesErasedPointer > currentPageAddress) { // currentPageAddress already wrapped around
		//    currentPageAddress      pagesErasedPointer     
		// DDD_____|________|________|DDDDDDDD|DDDDDDDD|
		uint32_t spaceInCurrentPageLeft = MT29_CACHE_SIZE - currentByteOffset;
		uint32_t emptyPagesUntilBlocksErasedPointer = pagesErasedPointer - currentPageAddress - 1;
		spaceLeft = spaceInCurrentPageLeft + (MT29_CACHE_SIZE * emptyPagesUntilBlocksErasedPointer);
	}
	else { 
		//          pagesErasedPointer              currentPageAddress
		// ________|DDDDDDDD|DDDDDDDD|DDDDDDDD|DDDDD___|
		uint32_t spaceInCurrentPageLeft = MT29_CACHE_SIZE - currentByteOffset;
		uint32_t emptyPagesUntilEndOfRingBuffer = fifoSizePages - currentPageAddress - 1;
		uint32_t emptyPagesBeforePagesErasedPointer = pagesErasedPointer;
		spaceLeft = spaceInCurrentPageLeft + (MT29_CACHE_SIZE * emptyPagesUntilEndOfRingBuffer) + (MT29_CACHE_SIZE * emptyPagesBeforePagesErasedPointer);
	}
	return spaceLeft;
}

sequential_write_status_t FLASH_MT29::fifoPushSimpleSkipBadBlocks(uint16_t blocksErasedPointer, uint32_t &pageAddressStart, uint16_t &byteOffsetStart, uint8_t *data, uint32_t dataLen, bool readBack, bool readBackCorrectFFFFFF, bool debug, uint16_t maxIterations, uint32_t fifoSizePages) {
	// UNTESTED!!!
	sequential_write_status_t firstResult, followingResult;
	firstResult = fifoPushSimple(blocksErasedPointer, pageAddressStart, byteOffsetStart, data, dataLen, readBack, readBackCorrectFFFFFF, debug, maxIterations, fifoSizePages);
	if((firstResult == MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR) || (firstResult == MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR)) {
		// try to write until writing works (maximum 256 times = 4 blocks when writing one full page)
		for(uint16_t i=0; i<256; i++) {
			Timing::delay(10); // 10 ms between writes and max 256 times = 2.56 seconds max. writing time
			followingResult = fifoPushSimple(blocksErasedPointer, pageAddressStart, byteOffsetStart, data, dataLen, readBack, readBackCorrectFFFFFF, debug, maxIterations, fifoSizePages);
			if((followingResult == MT29_SEQ_WRITE_STATUS_SUCCESS) || (followingResult == MT29_SEQ_WRITE_STATUS_MEMORY_FULL)) {
				break; // worked!
			}
		}
	}
	return firstResult; // always return first result, even if could be corrected
}

sequential_write_status_t FLASH_MT29::fifoPushSimple(uint16_t blocksErasedPointer, uint32_t &pageAddressStart, uint16_t &byteOffsetStart, uint8_t *data, uint32_t dataLen, bool readBack, bool readBackCorrectFFFFFF, bool debug, uint16_t maxIterations, uint32_t fifoSizePages) {
	uint32_t cpyPointer = 0; // current working pointer for data
	uint16_t flashWriteIteration = 0; // for timeout in case of fatal software error (should not happen!)
	uint16_t dmaBufferPointer = 0;
	uint16_t spaceLeftInPage = 0;
	uint8_t errorHappened = 0;
	bool copyDone = false;
	uint8_t *dmaBuffer2048Bytes = NULL;
	uint8_t *dmaReadBackBuffer2048Bytes = NULL;

	// IMPORTANT: if only ">" and just fits in memory -> wrap around of pointers to 0.0, then overwriting existing data! horror!
	if(dataLen >= fifoGetFreeSpace(blocksErasedPointer, pageAddressStart, byteOffsetStart, fifoSizePages)) {
		if(debug) { printf("MEMORY FULL, only %d Bytes left\n", fifoGetFreeSpace(blocksErasedPointer, pageAddressStart, byteOffsetStart, fifoSizePages)); }
		return MT29_SEQ_WRITE_STATUS_MEMORY_FULL;
	}
	if(!createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
		return MT29_SEQ_WRITE_STATUS_BUFFER_ERROR;
	}
	if(readBack) {
		if(!createBuffer(&dmaReadBackBuffer2048Bytes, MT29_CACHE_SIZE)) {
			heap_caps_free(dmaBuffer2048Bytes); // free first DMA memory
			return MT29_SEQ_WRITE_STATUS_BUFFER_ERROR;
		}
	}

	// NEW: wait until flash ID is readable (no error set, will be overwritten most probably)
	if(!waitOnID()) {
		if(debug) { printf("COULD NOT GET ID -> performing reset\n"); }
		Timing::delay(10);
		if(reset()) {
			if(debug) { printf("SUCCESS\n"); }
		}
		Timing::delay(10);
	}

	while(true) { // one iteration = one write into flash memory with maximum 2048 bytes
		spaceLeftInPage = MT29_CACHE_SIZE - byteOffsetStart; // we can squeeze in this amount of data into current page
		dmaBufferPointer = 0; // reset dma pointer (fill with new data)
		if(debug) {
			printf("--- FLASH WRITE ITERATION %d, %d Bytes in page left\n", flashWriteIteration, spaceLeftInPage);
			printf("Page: %d, Offset: %d\n", pageAddressStart, byteOffsetStart);
		}
		// build a package with size spaceLeftInPage
		while(dmaBufferPointer < spaceLeftInPage) { // fill data until current page would be full
			// COPY DATA
			if(cpyPointer < dataLen) {
				dmaBuffer2048Bytes[dmaBufferPointer] = data[cpyPointer];
				//if(debug) { printf("%02X ", dmaBuffer2048Bytes[dmaBufferPointer]); }
				dmaBufferPointer++;
				cpyPointer++;
			}
			else { // everything from  data copied in DMA memory
				if(debug) { printf("FINISHED DATA (%d Bytes)\n", cpyPointer); }
				cpyPointer = 0;
				copyDone = true;
				break;
			}
		}
		if(debug) { printf("WRITING %d Bytes\n", dmaBufferPointer); }

		if(dmaBufferPointer > 0) { // was a BUG before -> when writing so that stuff fits exactly into last page -> partialWrite with length = dmaBufferPointer = 0 is called -> simulated with Java, works with this code
			// write the data into flash
			if(!partialWrite(pageAddressStart, byteOffsetStart, dmaBuffer2048Bytes, dmaBufferPointer)) { // will also return FALSE when trying to write zero bytes (issue before)
				errorHappened = 1; // don't stop here, try to execute next write commands to have consistent pointers at the end
			}
			// read back
			if(readBack) {
				bool errorWithThisPage = false;
				if(!readWithECC(pageAddressStart, byteOffsetStart, dmaReadBackBuffer2048Bytes, dmaBufferPointer)) { errorHappened = 2; errorWithThisPage = true; }
				// compare
				for(uint16_t i=0; i<dmaBufferPointer; i++) {
					if(dmaReadBackBuffer2048Bytes[i] != dmaBuffer2048Bytes[i]) { errorHappened = 2; errorWithThisPage = true; break; }
				}
				// NEW: correction algorithm
				if(readBackCorrectFFFFFF) {
					if(errorWithThisPage) {
						if(debug) { printf("(*) ERROR WITH THIS PAGE -> try correcting\n"); }
						bool aLotOfFFs = true;
						for(uint16_t i=0; i<dmaBufferPointer; i++) {
							if(dmaReadBackBuffer2048Bytes[i] != 0xFF) { aLotOfFFs = false; break; }
						}
						if(aLotOfFFs) {
							bool retryFailed = false;
							if(debug) { printf("(*) only FFs\n"); }
							Timing::delay(10);
							reset(); // perform a reset
							Timing::delay(10);
							// retry
							partialWrite(pageAddressStart, byteOffsetStart, dmaBuffer2048Bytes, dmaBufferPointer);
							if(!readWithECC(pageAddressStart, byteOffsetStart, dmaReadBackBuffer2048Bytes, dmaBufferPointer)) { retryFailed = true; }
							for(uint16_t i=0; i<dmaBufferPointer; i++) {
								if(dmaReadBackBuffer2048Bytes[i] != dmaBuffer2048Bytes[i]) { retryFailed = true; break; }
							}
							if(retryFailed) {
								if(debug) { printf("(*) could not correct\n"); }
							}
							// keep errorHappened = 2, even if error was corrected
						}
						else {
							if(debug) { printf("(*) NOT only FFs: %02X %02x %02X %02x\n", dmaReadBackBuffer2048Bytes[0], dmaReadBackBuffer2048Bytes[1], dmaReadBackBuffer2048Bytes[2], dmaReadBackBuffer2048Bytes[3]); }
						}
					}
				}
			}
		}
		
		//Timing::delay(1); // 1ms delay after writing
		// update the pointer (call by reference)
		byteOffsetStart = byteOffsetStart + dmaBufferPointer;
		if(byteOffsetStart >= MT29_CACHE_SIZE) { // when page would be full after write -> increase pageAddress
			byteOffsetStart = byteOffsetStart - MT29_CACHE_SIZE;
			pageAddressStart++;
			if(pageAddressStart >= fifoSizePages) {
				pageAddressStart = 0; 
			}
		}
		flashWriteIteration++;
		if(copyDone) {
			if(debug) { printf("DONE\n"); }
			break;
		}
		if(flashWriteIteration > maxIterations) {
			errorHappened = 1;
			break;
		}
    }
	heap_caps_free(dmaBuffer2048Bytes); // free DMA memory
	if(readBack) { heap_caps_free(dmaReadBackBuffer2048Bytes); } // free DMA memory
	if(errorHappened == 0) { return MT29_SEQ_WRITE_STATUS_SUCCESS; }
	else if(errorHappened == 1) { return MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR; }
	else { return MT29_SEQ_WRITE_STATUS_READ_BACK_ERROR; }
}

sequential_write_status_t FLASH_MT29::fifoPush(uint16_t blocksErasedPointer, uint32_t &pageAddressStart, uint16_t &byteOffsetStart, uint8_t *data1, uint32_t dataLen1, uint8_t *data2, uint32_t dataLen2, uint8_t *data3, uint32_t dataLen3, bool debug, uint16_t maxIterations, uint32_t fifoSizePages) {
	sequential_write_state_internal_t stateFlashCopy = MT29_COPY_DATA_1; // state, start with copying first dataset
	uint32_t cpyPointer = 0; // current working pointer for data1, data2 or data3
	uint16_t flashWriteIteration = 0; // for timeout in case of fatal software error (should not happen!)
	uint16_t dmaBufferPointer = 0;
	uint16_t spaceLeftInPage = 0;
	bool errorHappened = false;
	uint8_t *dmaBuffer2048Bytes = NULL;

	// IMPORTANT: if only ">" and just fits in memory -> wrap around of pointers to 0.0, then overwriting existing data! horror!
	if((dataLen1 + dataLen2 + dataLen3) >= fifoGetFreeSpace(blocksErasedPointer, pageAddressStart, byteOffsetStart, fifoSizePages)) { 
		if(debug) { printf("MEMORY FULL, only %d Bytes left\n", fifoGetFreeSpace(blocksErasedPointer, pageAddressStart, byteOffsetStart, fifoSizePages)); }
		return MT29_SEQ_WRITE_STATUS_MEMORY_FULL;
	}
	if(!createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
		return MT29_SEQ_WRITE_STATUS_BUFFER_ERROR;
	}
	while(true) { // one iteration = one write into flash memory with maximum 2048 bytes
		spaceLeftInPage = MT29_CACHE_SIZE - byteOffsetStart; // we can squeeze in this amount of data into current page
		dmaBufferPointer = 0; // reset dma pointer (fill with new data)
		if(debug) {
			printf("--- FLASH WRITE ITERATION %d, %d Bytes in page left\n", flashWriteIteration, spaceLeftInPage);
			printf("Page: %d, Offset: %d\n", pageAddressStart, byteOffsetStart);
		}
		// build a package with size spaceLeftInPage, first copy data 1, then data 2, then data 3
		while(dmaBufferPointer < spaceLeftInPage) { // fill data until current page would be full
			// COPY DATA 1
			if(stateFlashCopy == MT29_COPY_DATA_1) {
				if(cpyPointer < dataLen1) {
					dmaBuffer2048Bytes[dmaBufferPointer] = data1[cpyPointer];
					//if(debug) { printf("%02X ", dmaBuffer2048Bytes[dmaBufferPointer]); }
					dmaBufferPointer++;
					cpyPointer++;
				}
				else { // everything from first data copied in DMA memory
					if(debug) { printf("FINISHED DATA 1 (%d Bytes)\n", cpyPointer); }
					stateFlashCopy = MT29_COPY_DATA_2;
					cpyPointer = 0;
				}
			}
			// COPY DATA 2
			else if(stateFlashCopy == MT29_COPY_DATA_2) {
				if(cpyPointer < dataLen2) {
					dmaBuffer2048Bytes[dmaBufferPointer] = data2[cpyPointer];
					//if(debug) { printf("%02X ", dmaBuffer2048Bytes[dmaBufferPointer]); }
					dmaBufferPointer++;
					cpyPointer++;
				}
				else { // everything from second data copied in DMA memory
					if(debug) { printf("FINISHED DATA 2 (%d Bytes)\n", cpyPointer); }
					cpyPointer = 0;
					stateFlashCopy = MT29_COPY_DATA_3;
				}
			}
			// COPY DATA 3
			else if(stateFlashCopy == MT29_COPY_DATA_3) {
				if(cpyPointer < dataLen3) {
					dmaBuffer2048Bytes[dmaBufferPointer] = data3[cpyPointer];
					//if(debug) { printf("%02X ", dmaBuffer2048Bytes[dmaBufferPointer]); }
					dmaBufferPointer++;
					cpyPointer++;
				}
				else { // everything from third data copied in DMA memory
					if(debug) { printf("FINISHED DATA 3 (%d Bytes)\n", cpyPointer); }
					cpyPointer = 0;
					stateFlashCopy = MT29_COPY_DONE;
					break;
				}
			}
		}
		if(debug) { printf("WRITING %d Bytes\n", dmaBufferPointer); }

		if(dmaBufferPointer > 0) { // was a BUG before -> when writing so that stuff fits exactly into last page -> partialWrite with length = dmaBufferPointer = 0 is called -> simulated with Java, works with this code
			// write the data into flash
			if(debug) {
				if(!partialWriteMock(pageAddressStart, byteOffsetStart, dmaBuffer2048Bytes, dmaBufferPointer)) {
					errorHappened = true; // don't stop here, try to execute next write commands to have consistent pointers at the end
				}
			}
			else {
				if(!partialWrite(pageAddressStart, byteOffsetStart, dmaBuffer2048Bytes, dmaBufferPointer)) { // will also return FALSE when trying to write zero bytes (issue before)
					errorHappened = true; // don't stop here, try to execute next write commands to have consistent pointers at the end
				}
			}
		}
		
		//Timing::delay(1); // 1ms delay after writing
		// update the pointer (call by reference)
		byteOffsetStart = byteOffsetStart + dmaBufferPointer;
		if(byteOffsetStart >= MT29_CACHE_SIZE) { // when page would be full after write -> increase pageAddress
			byteOffsetStart = byteOffsetStart - MT29_CACHE_SIZE;
			pageAddressStart++;
			if(pageAddressStart >= fifoSizePages) {
				pageAddressStart = 0; 
			}
		}
		flashWriteIteration++;
		if(stateFlashCopy == MT29_COPY_DONE) {
			if(debug) { printf("DONE\n"); }
			break;
		}
		if(flashWriteIteration > maxIterations) {
			errorHappened = true;
			break;
		}
    }
	heap_caps_free(dmaBuffer2048Bytes); // free DMA memory
	if(errorHappened) {
		return MT29_SEQ_WRITE_STATUS_PARTIAL_WRITE_ERROR;
	}
	return MT29_SEQ_WRITE_STATUS_SUCCESS;
}

uint16_t FLASH_MT29::fifoGetNumberOfPopableBlocks(uint16_t blocksErasedPointer, uint32_t currentPageAddress, uint32_t fifoSizePages) {
	// currentPageAddress = 0 .. 131071, currentByteOffset = 0 - 2047
	// blocksErasedPointer = 0 .. 2047 -> if 0 then nothing erased yet
	uint32_t popableBlocks = 0;
	uint32_t fifoSizeBlocks = fifoSizePages / MT29_PAGES_PER_BLOCK;
	if(currentPageAddress >= fifoSizePages) {
		//printf("ERROR1\n");
		return 0; // error
	}
	if(blocksErasedPointer >= fifoSizeBlocks) {
		//printf("ERROR2\n");
		return 0; // error
	}
	if((blocksErasedPointer * MT29_PAGES_PER_BLOCK) > currentPageAddress) { // currentPageAddress already wrapped around
		//    currentPageAddress      blocksErasedPointer     
		// DDD_____|________|________|DDDDDDDD|DDDDDDDD|
		uint32_t blocksToPopAtEnd = fifoSizeBlocks - blocksErasedPointer;
		uint32_t blocksToPopAtStart = currentPageAddress / MT29_PAGES_PER_BLOCK;
		popableBlocks = blocksToPopAtEnd + blocksToPopAtStart;
	}
	else { 
		//          blocksErasedPointer             currentPageAddress
		// ________|DDDDDDDD|DDDDDDDD|DDDDDDDD|DDDDD___|
		uint32_t blocksUntilCurrentPage = currentPageAddress / MT29_PAGES_PER_BLOCK;
		popableBlocks = blocksUntilCurrentPage - blocksErasedPointer;
	}
	return popableBlocks;	
}

uint32_t FLASH_MT29::fifoGetNumberOfPopableBytes(uint32_t sendBytePointer, uint32_t writeBytePointer) {
	// sendBytePointer = 0 .. 268435456-1, writeBytePointer = 0 .. 268435456-1
	uint32_t popableBytes = 0;
	if(sendBytePointer >= MT29_NUMBER_BYTES) { return 0; }
	if(writeBytePointer >= MT29_NUMBER_BYTES) { return 0; }
	if(sendBytePointer > writeBytePointer) {
		//    writeBytePointer        sendBytePointer     
		// DDD_____|________|________|DDDDDDDD|DDDDDDDD|
		uint32_t bytesToPopAtEnd = MT29_NUMBER_BYTES - sendBytePointer;
		uint32_t bytesToPopAtStart = writeBytePointer;
		popableBytes = bytesToPopAtEnd + bytesToPopAtStart;			
	}
	else {
		//          sendBytePointer                writeBytePointer
		// ________|DDDDDDDD|DDDDDDDD|DDDDDDDD|DDDDD___|
		popableBytes = writeBytePointer - sendBytePointer;			
	}
	return popableBytes;
}

uint16_t FLASH_MT29::fifoIncrementBlocksErasedPointer(uint16_t blocksErasedPointer, uint32_t fifoSizePages) {
	uint16_t ret = blocksErasedPointer;
	ret++;
	if(ret >= (fifoSizePages / MT29_PAGES_PER_BLOCK)) { // wrap around at end of FIFO
		ret = 0;
	}
	return ret;
}

bool FLASH_MT29::fifoPopDelete(uint16_t &blocksErasedPointer, uint32_t currentPageAddress, uint32_t fifoSizePages, bool debug) {
	if(fifoGetNumberOfPopableBlocks(blocksErasedPointer, currentPageAddress, fifoSizePages) == 0) {
		return false; // can't pop because no block fully written
	}
	if(debug) {
		if(!eraseMock(blocksErasedPointer)) {
			return false;
		}
	}
	else {
		if(!erase(blocksErasedPointer)) {
			return false;
		}		
	}
	blocksErasedPointer = fifoIncrementBlocksErasedPointer(blocksErasedPointer, fifoSizePages);
	return true;
}

/*sequential_write_status_t FLASH_MT29::sequentialWrite(uint32_t &pageAddressStart, uint16_t &byteOffsetStart, uint8_t *data1, uint32_t dataLen1, uint8_t *data2, uint32_t dataLen2, uint8_t *data3, uint32_t dataLen3, bool debug, uint16_t maxIterations) {
	sequential_write_state_internal_t stateFlashCopy = MT29_COPY_DATA_1; // state, start with copying first dataset
	uint16_t cpyPointer = 0; // current working pointer for data1, data2 or data3
	uint16_t flashWriteIteration = 0; // for timeout
	uint16_t dmaBufferPointer = 0;
	uint16_t spaceLeftInPage = 0;
	bool errorHappened = false;
	uint8_t *dmaBuffer2048Bytes = NULL;

	if(!fitsIntoMemory(pageAddressStart, byteOffsetStart, dataLen1 + dataLen2 + dataLen3)) {
		return MT29_SEQ_WRITE_STATUS_MEMORY_FULL;
	}
	if(!createBuffer(&dmaBuffer2048Bytes, MT29_CACHE_SIZE)) {
		return MT29_SEQ_WRITE_STATUS_ERROR;
	}
	while(true) { // one iteration = one write into flash memory with maximum 2048 bytes
		spaceLeftInPage = MT29_CACHE_SIZE - byteOffsetStart; // we can squeeze in this amount of data into current page
		dmaBufferPointer = 0; // reset dma pointer (fill with new data)
		if(debug) {
			printf("--- FLASH WRITE ITERATION %d, %d Bytes in page left\n", flashWriteIteration, spaceLeftInPage);
			printf("Page: %d, Offset: %d\n", pageAddressStart, byteOffsetStart);
		}
		// build a package with size spaceLeftInPage, first copy data 1, then data 2, then data 3
		while(dmaBufferPointer < spaceLeftInPage) { // fill data until current page would be full
			// COPY DATA 1
			if(stateFlashCopy == MT29_COPY_DATA_1) {
				if(cpyPointer < dataLen1) {
					dmaBuffer2048Bytes[dmaBufferPointer] = data1[cpyPointer];
					if(debug) { printf("%02X ", dmaBuffer2048Bytes[dmaBufferPointer]); }
					dmaBufferPointer++;
					cpyPointer++;
				}
				else { // everything from first data copied in DMA memory
					if(debug) { printf("\nFINISHED DATA 1 (%d Bytes)\n", cpyPointer); }
					stateFlashCopy = MT29_COPY_DATA_2;
					cpyPointer = 0;
				}
			}
			// COPY DATA 2
			else if(stateFlashCopy == MT29_COPY_DATA_2) {
				if(cpyPointer < dataLen2) {
					dmaBuffer2048Bytes[dmaBufferPointer] = data2[cpyPointer];
					if(debug) { printf("%02X ", dmaBuffer2048Bytes[dmaBufferPointer]); }
					dmaBufferPointer++;
					cpyPointer++;
				}
				else { // everything from second data copied in DMA memory
					if(debug) { printf("\nFINISHED DATA 2 (%d Bytes)\n", cpyPointer); }
					cpyPointer = 0;
					stateFlashCopy = MT29_COPY_DATA_3;
				}
			}
			// COPY DATA 3
			else if(stateFlashCopy == MT29_COPY_DATA_3) {
				if(cpyPointer < dataLen3) {
					dmaBuffer2048Bytes[dmaBufferPointer] = data3[cpyPointer];
					if(debug) { printf("%02X ", dmaBuffer2048Bytes[dmaBufferPointer]); }
					dmaBufferPointer++;
					cpyPointer++;
				}
				else { // everything from third data copied in DMA memory
					if(debug) { printf("\nFINISHED DATA 3 (%d Bytes)\n", cpyPointer); }
					cpyPointer = 0;
					stateFlashCopy = MT29_COPY_DONE;
					break;
				}
			}
		}
		if(debug) { printf("\nWRITING %d Bytes\n", dmaBufferPointer); }

		// write the data into flash
		if(debug) {
			if(!partialWriteMock(pageAddressStart, byteOffsetStart, dmaBuffer2048Bytes, dmaBufferPointer)) {
				errorHappened = true; // don't stop here, try to execute next write commands to have consistent pointers at the end
			}
		}
		else {
			if(!partialWrite(pageAddressStart, byteOffsetStart, dmaBuffer2048Bytes, dmaBufferPointer)) {
				errorHappened = true; // don't stop here, try to execute next write commands to have consistent pointers at the end
			}
		}
		Arduino::delayWrapper(1); // 1ms delay after writing
		// update the pointer (call by reference)
		byteOffsetStart = byteOffsetStart + dmaBufferPointer;
		if(byteOffsetStart >= MT29_CACHE_SIZE) { // when page would be full after write -> increase pageAddress
			byteOffsetStart = byteOffsetStart - MT29_CACHE_SIZE;
			pageAddressStart++;
		}
		flashWriteIteration++;
		if(stateFlashCopy == MT29_COPY_DONE) {
			if(debug) { printf("DONE\n"); }
			break;
		}
		if(flashWriteIteration > maxIterations) {
			errorHappened = true;
			break;
		}
    }
	heap_caps_free(dmaBuffer2048Bytes); // free DMA memory
	if(errorHappened) {
		return MT29_SEQ_WRITE_STATUS_ERROR;
	}
	return MT29_SEQ_WRITE_STATUS_SUCCESS;
}*/

bool FLASH_MT29::fitsIntoMemory(uint32_t currentPageAddress, uint16_t currentByteOffset, uint32_t dataLen) {
	// pageAddress = 0 .. 131071, byteOffset = 0 - 2047
	uint32_t spaceInCurrentPageLeft = MT29_CACHE_SIZE - currentByteOffset;
	uint32_t totallyEmptyPages = MT29_NUMBER_PAGES - currentPageAddress - 1;
	uint32_t spaceLeftInFlash = (totallyEmptyPages * MT29_CACHE_SIZE) + spaceInCurrentPageLeft;
	printf("SPACE LEFT = %d Bytes\n", spaceLeftInFlash); // TODO: REMOVE
	if(spaceLeftInFlash < dataLen) {
		return false;
	}
	return true;
}

bool FLASH_MT29::getPlane(uint32_t pageAddress) {
	bool plane = 0;
	uint32_t blockNumber = 0;
	blockNumber = pageAddress / MT29_PAGES_PER_BLOCK; // calculate block number from page
	//printf("GET PLANE, blockNumber = %d\n", blockNumber); // DEBUG ONLY
	if((blockNumber % 2) == 1) { // uneven 
		//printf("GET PLANE, UNEVEN!\n"); // DEBUG ONLY
		plane = 1;
	}
	return plane;
}

bool FLASH_MT29::partialWriteMock(uint32_t pageAddress, uint16_t byteOffset, uint8_t *dataResult, uint16_t dataLen) {
	if(dataLen > MT29_CACHE_SIZE) { // less can be written, but don't overwrite stuff afterwards! partially page programming not so good
		printf("ERR: DATA LEN TOO LARGE\n");
		return false;
	}
	if(dataLen == 0) { // nothing to write
		printf("ERR: DATALEN ZERO\n");
		return false;
	}
	if(pageAddress >= MT29_NUMBER_PAGES) { // also protected by function executeProgram, but better to cancel everything more early (no action needed)
		printf("ERR: PAGE ADDRESS TOO LARGE\n");
		return false;
	}
	if((byteOffset >= MT29_CACHE_SIZE) // important for partialWrite
		|| (byteOffset + dataLen) > MT29_CACHE_SIZE) {
		printf("ERR: BYTE OFFSET TOO LARGE\n");
		return false;
	}
	printf("Fake-Writing at page %d (Offset: %d): %d bytes\n", pageAddress, byteOffset, dataLen);
	return true;
}

bool FLASH_MT29::partialWrite(uint32_t pageAddress, uint16_t byteOffset, uint8_t *dataResult, uint16_t dataLen) {
	// pageAddress = 0 .. 131071
	uint32_t res = 0;

	if(dataLen > MT29_CACHE_SIZE) { // less can be written, but don't overwrite stuff afterwards! partially page programming not so good
		return false;
	}
	if(dataLen == 0) { // nothing to write
		return false;
	}
	if(pageAddress >= MT29_NUMBER_PAGES) { // also protected by function executeProgram, but better to cancel everything more early (no action needed)
		return false;
	}
	if((byteOffset >= MT29_CACHE_SIZE) // important for partialWrite
		|| (byteOffset + dataLen) > MT29_CACHE_SIZE) { 
		return false;
	}
	if(!writeEnable()) {
        return false;
    }
    if(!unlockAllBlocks()) {
        return false;
    }
    res = getFeatures(GET_FEATURES_BLOCK_LOCK);
	if(res != 0) { // blocks are not unlocked
		return false;
	}
    res = getFeatures(GET_FEATURES_STATUS);
    if((res & (1 << 1)) == 0) { // WEL bit, 1 = write enabled
		return false;
	}
    if(!loadProgram(getPlane(pageAddress), byteOffset, dataResult, dataLen)) { // using byteOffset here
        return false;
    }
    if(!executeProgram(pageAddress)) {
        return false;
    }
    // wait until program executed
	if(!waitUntilFinished()) {
		return false;
	}
	return true;
}

bool FLASH_MT29::write(uint32_t pageAddress, uint8_t *dataResult, uint16_t dataLen) {
	// pageAddress = 0 .. 131071
	uint32_t res = 0;

	if(dataLen > MT29_CACHE_SIZE) { // less can be written, but don't overwrite stuff afterwards! partially page programming not so good
		return false;
	}
	if(dataLen == 0) { // nothing to write
		return false;
	}
	if(pageAddress >= MT29_NUMBER_PAGES) { // also protected by function executeProgram, but better to cancel everything more early (no action needed)
		return false;
	}
	if(!writeEnable()) {
        return false;
    }
    if(!unlockAllBlocks()) {
        return false;
    }
    res = getFeatures(GET_FEATURES_BLOCK_LOCK);
	if(res != 0) { // blocks are not unlocked
		return false;
	}
    res = getFeatures(GET_FEATURES_STATUS);
    if((res & (1 << 1)) == 0) { // WEL bit, 1 = write enabled
		return false;
	}
    if(!loadProgram(getPlane(pageAddress), 0, dataResult, dataLen)) {
        return false;
    }
    if(!executeProgram(pageAddress)) {
        return false;
    }
    // wait until program executed
	if(!waitUntilFinished()) {
		return false;
	}
	return true;
}

bool FLASH_MT29::read(uint32_t pageAddress, uint16_t byteOffset, uint8_t *dataResult, uint16_t dataLen) {
	// pageAddress = 0 .. 131071
	// move page into cache
	if(!readPage(pageAddress)) {
		return false;
	}
	// wait until page loaded
	if(!waitUntilFinished()) {
		return false;
	}
	// get data
	if(!readFromCache(getPlane(pageAddress), byteOffset, dataResult, dataLen)) {
		return false;
	}
	
	return true;
}

bool FLASH_MT29::readWithECC(uint32_t pageAddress, uint16_t byteOffset, uint8_t *dataResult, uint16_t dataLen) {
	// pageAddress = 0 .. 131071
	// move page into cache
	if(!readPage(pageAddress)) {
		return false;
	}
	// wait until page loaded
	if(!waitUntilFinished()) {
		return false;
	}
	// get data
	if(!readFromCache(getPlane(pageAddress), byteOffset, dataResult, dataLen)) {
		return false;
	}
	// check for ECC errors -> NOT WORKING WHEN PARTIAL PAGE PROGRAMMING, I THINK WOULD NEED TO WRITE 2048/4 = 512 BYTE AT ONCE TO KEEP ECC VALID?! -> YES
	uint32_t eccStatus = (getFeatures(GET_FEATURES_STATUS) >> 4) & 0b111; // get ECC0, ECC1, ECC2
	if((eccStatus == 0b010) || (eccStatus == 0b011) || (eccStatus == 0b101)) {
		//printf("ECC ERROR %d\n", eccStatus);
		return false;
	}
	return true;
}

bool FLASH_MT29::eraseMock(uint16_t block) {
	if(block >= MT29_NUMBER_BLOCKS) {
		printf("ERASEMOCK ERROR\n");
		return false;
	}
	uint32_t pageStart = block * MT29_PAGES_PER_BLOCK;
	uint32_t pageEnd = ((block+1) * MT29_PAGES_PER_BLOCK) - 1;
	printf("ERASEMOCK block %d, pages %d - %d\n", block, pageStart, pageEnd);
	return true;
}

bool FLASH_MT29::erase(uint16_t block) {
	// block = 0 .. 2047
	uint32_t res;
	uint32_t address = block;
	address *= MT29_PAGES_PER_BLOCK; // convert block id to page address

	if(block >= MT29_NUMBER_BLOCKS) {
		return false;
	}
	if(!writeEnable()) {
        return false;
    }
    if(!unlockAllBlocks()) {
        return false;
    }
    res = getFeatures(GET_FEATURES_BLOCK_LOCK);
	if(res != 0) { // blocks are not unlocked
		return false;
	}
    res = getFeatures(GET_FEATURES_STATUS);
    if((res & (1 << 1)) == 0) { // WEL bit, 1 = write enabled
		return false;
	}
	if(!blockEraseInternal(address)) {
        return false;
    }
	if(!waitUntilFinished()) {
		return false;
	}
	return true;
}

bool FLASH_MT29::fullErase() {
	for(uint16_t i = 0; i < MT29_NUMBER_BLOCKS; i++) {
        if(!erase(i)) {
			return false;
		}
		Timing::delay(2);
    }
	return true;
}

bool FLASH_MT29::waitUntilFinished() {
	const uint8_t WAIT_TIMEOUT = 200;
	uint16_t loopCnt = 0;
	uint32_t status = 0;
	while(1) {
		status = getFeatures(GET_FEATURES_STATUS);
		if((status & 1) == 0) { // OIP (operation in progress) bit, 0 = done
			break;
		}
		loopCnt++;
		if(loopCnt > WAIT_TIMEOUT) { // timeout waiting
			return false;
		}
		Timing::delay(3);
	}
	return true;
}

uint32_t FLASH_MT29::getFeatures(addr_get_features_t addr) {
	uint64_t addrConv = addr;
	uint32_t result = spi.readMax4Byte(MT29_CMD_GET_FEATURES, 1, addrConv, 1, 1);
	return result;
}

bool FLASH_MT29::writeEnable() {
	uint32_t result = spi.readMax4Byte(MT29_CMD_WRITE_ENABLE, 1, 0, 0, 0);
	if(result == 0) {
		return true;
	}
	return false;
}

bool FLASH_MT29::unlockAllBlocks() {
	// TODO: WRITE USING 4BYTE TX BUFFER!
	uint64_t addrConv = GET_FEATURES_BLOCK_LOCK;
	uint8_t unlockData = 0x0;
	if(spi.write(MT29_CMD_SET_FEATURES, 1, addrConv, 1, &unlockData, 1)) {
		return true;
	}
	return false;
	//The following command sequence unlocks all blocks after power-up: The SET FEATURES
	//register write (1Fh) operation is issued, followed by the feature address (A0h).
	//Then, 00h is issued on data bits to unlock all blocks.
}

bool FLASH_MT29::loadProgram(bool plane, uint16_t offset, const uint8_t *data, uint16_t len) {
	// 2 address bytes (3 bit dummy, 1 bit plane select, 12 bit address), 0 dummy, 1-2176 byte data
	uint16_t offsetWithPlane = offset;
	if(plane) { // use second plane
		offsetWithPlane = offsetWithPlane | 0b1000000000000; // set plane bit (bit 13)
	}
	if((offset >= MT29_CACHE_SIZE)
		|| (offset + len) > MT29_CACHE_SIZE) { 
		return false;
	}
	if(spi.write(MT29_CMD_PROGRAM_LOAD, 1, offsetWithPlane, 2, data, len)) {
		return true;
	}
	return false;
}

bool FLASH_MT29::executeProgram(uint32_t address) {
	// 3 adress bytes (24 bit = 7 dummy bits + 17 bit where to write cache data), no data
	if(address >= MT29_NUMBER_PAGES) {
		return false;
	}
	uint32_t result = spi.readMax4Byte(MT29_CMD_PROGRAM_EXECUTE, 1, address, 3, 0);
	if(result == 0) {
		return true;
	}
	return false;
}

bool FLASH_MT29::readPage(uint32_t address) { 
	// 3 address bytes  (24 bit = 7 dummy bits + 17 bit to select page), no data
	if(address >= MT29_NUMBER_PAGES) {
		return false;
	}
	uint32_t result = spi.readMax4Byte(MT29_CMD_PAGE_READ, 1, address, 3, 0);
	if(result == 0) {
		return true;
	}
	return false;
}

bool FLASH_MT29::readFromCache(bool plane, uint16_t offset, uint8_t *data, uint16_t len) {
	// 2 address bytes (3 bit dummy, 1 bit plane select, 12 bit address), 1 dummy, 1-2176 byte data
	// simulate dummy byte by using 3 adress bytes and byte shifts: | CMD | ADR_MSB | ADR_LSB | 0 | data ... |
	uint32_t offsetTemp = offset;
	if(plane) { // use second plane
		offsetTemp = offsetTemp | 0b1000000000000; // set plane bit (bit 13)
	}
	offsetTemp = offsetTemp << 8; // shift by 8 bits, to create dummy byte
	if((offset + len) > MT29_CACHE_SIZE_EXTENDED) { // extended to also read 128 extra bytes, e.g. for self test
		return false;
	}

	if(spi.read(MT29_CMD_READ_CACHE_X1, 1, offsetTemp, 3, data, len)) {
		return true;
	}
	return false;
}

bool FLASH_MT29::createBuffer(uint8_t **data, uint16_t len) {
    *data = (uint8_t*) heap_caps_malloc(len, MALLOC_CAP_DMA);
	if(*data == NULL) {
		return false;
	}
	return true;
}

bool FLASH_MT29::blockEraseInternal(uint32_t address) {
	// 3 address bytes (24 bit = NORMAL PAGE ADDRESS, but e.g. address 1 deletes pages 0, 1, .., 63!!!), no data
	uint32_t result;
	if(address >= MT29_NUMBER_PAGES) {
		return false;
	}
	result = spi.readMax4Byte(MT29_CMD_BLOCK_ERASE, 1, address, 3, 0);
	if(result == 0) {
		return true;
	}
	return false;
}

bool FLASH_MT29::selfTestBadBlocks() {
	// checks 2048 blocks (first page) for errors
	uint32_t address = 0;
	const uint16_t BUF_LEN = 1;
	bool badBlockFound = false;

	// create buffer
	uint8_t *dataDMA = NULL;
    if(!createBuffer(&dataDMA, BUF_LEN)) {
        return false;
    }

	printf("START SELFTEST BAD BLOCKS\n");

	for(uint16_t i = 0; i < MT29_NUMBER_BLOCKS; i++) {
		// read page
		if(!readPage(address)) {
			printf("FAILED READ PAGE ADDR %d\n", address);
			return false;
		}
		// wait until loaded
		if(!waitUntilFinished()) {
			printf("WAITED TOO LONG, ERROR\n");
			return false;
		}
		// get data
		dataDMA[0] = 0xAA; // set to some dummy value to validate the overwriting
		if(!readFromCache(getPlane(address), 2048, dataDMA, BUF_LEN)) { // check byte 2048 (first spare area location in the first page of each block)
			printf("FAILED READ FROM CACHE\n");
			return false;
		}
		if(dataDMA[0] != 255) { // not okay
			printf("BLOCK %i (addr %d) NOT OK: %d\n", i, address, dataDMA[0]);
			badBlockFound = true;
		}
		address += MT29_PAGES_PER_BLOCK;
		Timing::delay(1); // just wait a bit
	}
	heap_caps_free(dataDMA);
	printf("END SELFTEST BAD BLOCKS\n");
	if(badBlockFound) { return false; }
	return true;
}

bool FLASH_MT29::printFlash(uint32_t addressStart, uint32_t numberPages, uint16_t lenPerPage, bool onlyData) {
    uint8_t *dataTemp = NULL;
    if(!createBuffer(&dataTemp, MT29_CACHE_SIZE)) {
		return false;
	}
    for(uint32_t j=0; j<numberPages; j++) {
        if(!read(addressStart+j, 0, dataTemp, lenPerPage)) {
            return false;
        }
        if(!onlyData) { printf("READ %08X: ", addressStart+j); }
        for(uint16_t i=0; i<lenPerPage; i++) {
            printf("%02X", dataTemp[i]);
        }
        if(!onlyData) { printf("\n"); }
        Timing::delay(10); // otherwise watchdog kicks in
    }
    heap_caps_free(dataTemp);
    printf("\n");
	return true;
}
