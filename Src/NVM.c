/*
 * EEPROM.c
 *
 *  Created on: Oct 6, 2016
 *      Author: dongfang
 */
#include "stm32l0xx.h"
#include "NVM.h"
#include "Calibration.h"
#include "Globals.h"
#include "RTC.h"
#include "APRS.h"

__attribute__((section(".flashnvm")))         const volatile LogRecord_t flashdata[NUM_LOG_RECORDS];
__attribute__((section(".eemem")))            const volatile LogRecordIndex_t storedRecordIndex;
__attribute__((section(".eemem")))            const volatile PLLTrimCalibration_t pllTrimCalibration;
__attribute__((section(".eemem")))            const volatile CalibrationRecord_t calibrationByTemperatureRanges[NUM_TEMPERATURE_RANGES];
__attribute__((section(".eemem")))            const volatile Odometer_t odometer;

__attribute__((section(".noinit"))) static uint32_t rand;

void storeToEEPROM(uint32_t* const target, const uint32_t* const source,
		size_t numWords) {
	uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
	RCC->AHBENR |= RCC_AHBENR_MIFEN;

	// Unlock eeprom
	FLASH->PEKEYR = 0x89ABCDEF;
	FLASH->PEKEYR = 0x02030405;
	uint32_t unlocked = FLASH->PECR & FLASH_PECR_PELOCK;

	FLASH->PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_FIX); // no need to erase
	for (size_t i = 0; i < numWords; i++) {
		// "The software can reset it writing 1 in the status register."
		FLASH->SR |= FLASH_SR_EOP;
		target[i] = source[i];
		while ((FLASH->SR & FLASH_SR_EOP) == 0)
			;
	}

	RCC->AHBENR = (RCC_AHBENR_MIFEN & ~RCC_AHBENR_MIFEN) | alreadyRunning;

	// Lock again.
	FLASH->PECR |= FLASH_PECR_PELOCK;
}

void NVM_writeCalibrationRecord(CalibrationRecord_t* const record,
		uint8_t index) {
	storeToEEPROM((uint32_t*) &calibrationByTemperatureRanges[index],
			(uint32_t*) record, (sizeof(CalibrationRecord_t) + 3) / 4);
}

void NVM_WritePLLTrimCalibration(PLLTrimCalibration_t* cal) {
	storeToEEPROM((uint32_t*) &pllTrimCalibration, (uint32_t*) cal,
			(sizeof(PLLTrimCalibration_t) + 3) / 4);
}

void NVM_WriteOdometer(Odometer_t* odo) {
	storeToEEPROM((uint32_t*) &odometer, (uint32_t*) odo,
			(sizeof(Odometer_t) + 3) / 4);
}

// If we need a function in RAM this is how:
// __attribute__((section(".RAMtext")))
static void NVM_storeLogRecord(uint16_t i, LogRecord_t* const data) {
	uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
	RCC->AHBENR |= RCC_AHBENR_MIFEN;

	// Unlock nv memories
	FLASH->PEKEYR = 0x89ABCDEF;
	FLASH->PEKEYR = 0x02030405;

	// Additional FLASH unlocking
	FLASH->PRGKEYR = 0x8C9DAEBF;
	FLASH->PRGKEYR = 0x13141516;

	// Test if cleared if we need that.
	// FLASH->PECR & FLASH_PECR_PRGLOCK;

	FLASH->SR |= FLASH_SR_EOP;

	/* Set the ERASE bit */
	//SET_BIT(FLASH->PECR, FLASH_PECR_ERASE);
	/* Set PROG bit */
	//SET_BIT(FLASH->PECR, FLASH_PECR_PROG);
	/* this is not needed?!?
	 FLASH->SR |= FLASH_SR_EOP;
	 FLASH->PECR |= FLASH_PECR_ERASE | FLASH_PECR_PROG;
	 *((uint32_t*) &(flashdata[i * 4])) = 0;
	 while ((FLASH->SR & FLASH_SR_EOP) == 0)
	 ;
	 */
	uint32_t* start = (uint32_t*) (flashdata + i);
	uint32_t* from = (uint32_t*) (data);

	//FPRG = 1, PRG = 1.
	FLASH->PECR = (FLASH->PECR & ~FLASH_PECR_ERASE) | FLASH_PECR_PROG
			| FLASH_PECR_FIX;

	// size in words.
	for (size_t j = 0; j < (sizeof(LogRecord_t) + 3) / 4; j++) {
		// "The software can reset it writing 1 in the status register."
		FLASH->SR |= FLASH_SR_EOP;
		start[j] = from[j];
		while ((FLASH->SR & FLASH_SR_EOP) == 0)
			;
	}

	// Lock again.
	FLASH->PECR |= FLASH_PECR_PRGLOCK;
	FLASH->PECR |= FLASH_PECR_PELOCK;
}

void recordFlightLog() {
	uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
	RCC->AHBENR |= RCC_AHBENR_MIFEN;

	Time_t now;
	RTC_read(&now);

	if (now.hours != storedRecordIndex.lastLogTime.hours) {
		LogRecordIndex_t tmpIndex = storedRecordIndex;
		// TODO check if valid and reset if not.

		tmpIndex.lastLogTime = now;
		LogRecord_t record;
		record.lat = lastNonzeroPosition.lat * 1E7;
		record.lon = lastNonzeroPosition.lon * 1E7;
		record.alt = lastNonzeroPosition.alt;
		DateTime_t datetime;
		record.compactedDateTime = compactDateTime(&GPSDateTime);
		record.speed_kts = speed_kts;
		record.course = course * (256/360.0f);
		record.temperature = temperature;
		record.checksum =
				record.lat + record.lon + record.alt + record.compactedDateTime + record.speed_kts + record.temperature + 123;

		NVM_storeLogRecord(tmpIndex.headIdx, &record);

		tmpIndex.headIdx++;
		if (tmpIndex.headIdx == NUM_LOG_RECORDS)
			tmpIndex.headIdx = 0;
		if (tmpIndex.headIdx == tmpIndex.tailIdx) {
			// bang, we are full.
			tmpIndex.tailIdx++;
			if (tmpIndex.tailIdx == NUM_LOG_RECORDS)
				tmpIndex.tailIdx = 0;
		}
		storeToEEPROM((uint32_t*) &storedRecordIndex, (uint32_t*)&tmpIndex, (sizeof(LogRecordIndex_t) + 3) / 4);
	}

	RCC->AHBENR = (RCC_AHBENR_MIFEN & ~RCC_AHBENR_MIFEN) | alreadyRunning;
}

uint32_t random(uint32_t seed) {
	Time_t now;
	RTC_read(&now);
	return now.hours + now.minutes + now.seconds + systime + seed;
}

uint16_t playbackFlightLog(APRS_Band_t band, uint32_t frequency) {
	uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
	RCC->AHBENR |= RCC_AHBENR_MIFEN;
	volatile int16_t n = storedRecordIndex.headIdx - storedRecordIndex.tailIdx;
	if (n) {
		if (n < 0)
			n += NUM_LOG_RECORDS;
		currentTextMessage[0] = '}'; // this is the only allowed char which is not also a base-91 one. Perfect.
		char* buf = currentTextMessage + 1;
		uint16_t linearIdx = 0;
		rand += temperature * VDDa * 1024;
		for (uint8_t i = 0; i < n && linearIdx < n; i++) {
			if (buf >= currentTextMessage + sizeof(currentTextMessage) - 20)
				break;
			uint16_t span = ((n - linearIdx)/2 + 1);
			linearIdx += ((rand = random(rand)) % span);
			uint16_t circularIdx = linearIdx + storedRecordIndex.tailIdx;
			linearIdx++; // ensure SOME progress.
			if (circularIdx >= NUM_LOG_RECORDS)
				circularIdx -= NUM_LOG_RECORDS;
			buf = APRS_marshallFlightLogMessage(flashdata + circularIdx, buf);
		}
		// timer_sleep(500);
		APRS_transmitMessage(band, TEXT_MESSAGE, frequency);
	}
	RCC->AHBENR = (RCC_AHBENR_MIFEN & ~RCC_AHBENR_MIFEN) | alreadyRunning;
	return n;
}
