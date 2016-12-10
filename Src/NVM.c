/*
 * EEPROM.c
 *
 *  Created on: Oct 6, 2016
 *      Author: dongfang
 */
#include "stm32l0xx.h"
#include "Trace.h"
#include "NVM.h"
#include "Calibration.h"

// 0x0808 0000 - 0x0808 07FF
// aka
// #define DATA_EEPROM_BASE       ((uint32_t)0x08080000U) /*!< DATA_EEPROM base address in the alias region */
// #define DATA_EEPROM_END        ((uint32_t)0x080807FFU) /*!< DATA EEPROM end address in the alias region */

__attribute__((section(".flashnvm")))  const LogRecord_t flashdata[NUM_LOG_RECORDS];
__attribute__((section(".eemem")))    const LogRecordIndex_t storedRecordIndex;
__attribute__((section(".eemem")))    const PLLCtrlCalibration_t pllCtrlCalibration;
__attribute__((section(".eemem")))    const FlightLog_t flightLog;
__attribute__((section(".eemem")))    const CalibrationRecord_t calibrationByTemperatureRanges[NUM_TEMPERATURE_RANGES];

// Calibration record.
// System status (blank, calibrated0..calibratedn, error)

/*
 void eepromExperiment() {
 uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
 RCC->AHBENR |= RCC_AHBENR_MIFEN;

 uint32_t old = *((uint32_t*) DATA_EEPROM_BASE);

 // Unlock eeprom
 FLASH->PEKEYR = 0x89ABCDEF;
 FLASH->PEKEYR = 0x02030405;
 uint32_t unlocked = ~(FLASH->PECR & FLASH_PECR_PELOCK);

 FLASH->PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_FIX); // no need to erase
 *((uint32_t*) DATA_EEPROM_BASE) = old + 1;

 RCC->AHBENR = (RCC_AHBENR_MIFEN & ~RCC_AHBENR_MIFEN) | alreadyRunning;

 // Lock again.
 FLASH->PECR |= FLASH_PECR_PELOCK;
 }
 */

void storeToEEPROM(uint32_t* const target, const uint32_t* const source,
		size_t numWords) {
	uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
	RCC->AHBENR |= RCC_AHBENR_MIFEN;

	// Unlock eeprom
	FLASH->PEKEYR = 0x89ABCDEF;
	FLASH->PEKEYR = 0x02030405;
	uint32_t unlocked = ~(FLASH->PECR & FLASH_PECR_PELOCK);

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

/* USER CODE END 0 */

// __attribute__((section(".RAMtext")))
void storeLogRecord(uint16_t i, LogRecord_t* const data) {
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
	uint32_t* start = (uint32_t*) (flashdata) + i;
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

/* not needed, really. Just access directly.
 LogRecord_t* const logRecord(uint16_t i) {
 return &flashdata[i];
 }
 */

/*
 void HALFlashExperiment() {
 FLASH_OBProgramInitTypeDef pOBInit;

 HAL_FLASH_OB_Unlock();
 HAL_FLASHEx_OBGetConfig(&pOBInit);

 trace_printf("Write protect: %d\n", pOBInit.WRPState);
 trace_printf("Write protect sector1: %d\n", pOBInit.WRPSector);
 trace_printf("Write protect:sector2: %d\n", pOBInit.WRPSector2);

 // HAL_FLASHEx_OBProgram(&pOBInit);
 HAL_FLASH_OB_Lock();

 trace_printf("Old flash value was %d\n", flashdata[0]);
 HAL_FLASH_Unlock();
 // FLASH_ErasePage((uint32_t)&flashdata);
 HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&flashdata, 0x12345678);
 trace_printf("Status: %d\n", status);
 trace_printf("New flash value was %d\n", ((volatile uint32_t*)flashdata)[0]);
 HAL_FLASH_Lock();
 }
 */

uint8_t uncompressDateHoursMinutes(LogRecord_t* record, Time_t* time) {
	return 0;
}
uint16_t uncompressBatteryVoltageTomV(LogRecord_t* record) {
	return 0;
}
uint16_t uncompressSolarVoltageTomV(LogRecord_t* record) {
	return 0;
}
