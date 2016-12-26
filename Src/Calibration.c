/*
 * Calibration.c
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */

#include "Calibration.h"
#include "CDCE913.h"
#include "GPS.h"
#include "LED.h"
#include "Globals.h"
#include "NVM.h"
#include "PLL.h"
#include "PrecisionTimer.h"
#include "stdint.h"
#include "Systime.h"
#include "Types.h"
#include "APRS.h"
#include "PLL.h"
#include "WSPRTransmitter.h"
#include <stm32l051xx.h>
#include <string.h>

const int8_t temperatureRanges[] = TEMPERATURE_RANGES;
/*
const CalibrationRecord_t defaultCalibration = {
		.transmitterOscillatorFrequencyAtDefaultTrim = DEFAULT_XTAL_FREQ, };
*/
#define CALIBRATION_DIVISION 7
#define CALIBRATION_NUM_CYCLES 1

static uint32_t selfCalibrateAbsolutePLLFrequency() {
	// TODO, remove from here.
	/*
	GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 6))) | 0 << (2 * 6);

	while ((GPIOA->IDR & (1 << 6)) == 0) {
		GPS_getData();
		trace_printf("Waiting for GPS timelock (%d)\n",
				GPSStatus.numberOfSatellites);
		timer_sleep(1000);
	}
	*/
	PLL_setBypassModeWithDivision(PLL_CENTER_TRIM, CALIBRATION_DIVISION);
	timer_sleep(100);

	// One shot ought to be enough; the +/- 1 error is limited to 0.3ppm
	double rawResult = measurePeriod(PLL_CYCLES_PER_GPS_CYCLE, 1E6, 5000);

	PLL_shutdown();

	return rawResult * CALIBRATION_DIVISION;
}

static uint8_t getIndex() {
	uint8_t index = 0;
	while (index < sizeof(temperatureRanges) - 1
			&& temperatureRanges[index + 1] <= temperature)
		index++;
	return index;
}

static uint16_t calibrationRecord_Checksum(const CalibrationRecord_t* record) {
	uint32_t checksum = 0x1F55F1AA;
	checksum += record->transmitterOscillatorFrequencyAtDefaultTrim * 7;
	checksum += record->temperature * 12345;
	return checksum;
}

void calibratePLLFrequency() {
	uint32_t selfCalibration = selfCalibrateAbsolutePLLFrequency();
	if (selfCalibration) {
		CalibrationRecord_t tempRecord;
		volatile uint8_t index = getIndex();
		tempRecord.transmitterOscillatorFrequencyAtDefaultTrim = selfCalibration;
		tempRecord.temperature = temperature;
		// Commit the new calibration.
		tempRecord.checksum = calibrationRecord_Checksum(&tempRecord);
		NVM_writeCalibrationRecord(&tempRecord, index);
//		calibrationByTemperatureRanges[getIndex(temperature)] = tempRecord;
	}
}

// somehow the eeprom readback is fucked. The eeprom stored data is ok.
uint32_t getCalibratedPLLOscillatorFrequency() {
	uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
	RCC->AHBENR |= RCC_AHBENR_MIFEN;

	volatile CalibrationRecord_t* calrec = calibrationByTemperatureRanges + getIndex();
	uint16_t check = calibrationRecord_Checksum(calrec);
	uint32_t fromRecord = calrec->transmitterOscillatorFrequencyAtDefaultTrim;
	RCC->AHBENR = (RCC_AHBENR_MIFEN & ~RCC_AHBENR_MIFEN) | alreadyRunning;

	if (check != calrec->checksum) {
		return DEFAULT_XTAL_FREQ;	// damn, we had nothing!
	}
	return fromRecord;
}

static uint32_t PLLTrimCalibration_Checksum(const PLLTrimCalibration_t* cal) {
	uint32_t checksum = 0x1F55F1AA;
	for (uint8_t i = 0; i < 24; i++) {
		checksum += cal->trimEffect[i] * (i + 1);
	}
	return checksum;
}

/*
 * Grind a relative trim effect table.
 * Compare PLL to RTC at different trims, then relativize the measurements to one
 * at center trim.
 * Does not need GPS.
 */
static void _selfCalibratePLLTrim(int16_t* result, uint32_t requiredCount, uint8_t numCycles) {
	double values[24];
	for (uint8_t round = 0; round <= numCycles; round++) {
		// Go alternately min to max and max to min.
		uint8_t from = (round % 2) ? PLL_MIN_TRIM : PLL_MAX_TRIM;
		uint8_t to = (round % 2) ? PLL_MAX_TRIM : PLL_MIN_TRIM;
		int8_t step = (round % 2) ? 1 : -1;

		do {
			PLL_setBypassModeWithDivision(from, 1000);
			// Maybe RTC is not that bad at all.
			timer_sleep(100);
			LED_toggle();
			double np = measurePeriod(RTC_CYCLES_PER_PLL_CYCLE, requiredCount, requiredCount/32 + 2000);

			if (round) {
				values[from] += np;
			} else {
				// discard result of first round.
				values[from] = 0;
			}
			from += step;
		} while (from != to + step);
	}

	for (uint8_t i = PLL_MIN_TRIM; i <= PLL_MAX_TRIM; i++) {
		// oscillator faster than center->
		// few RTC counts->
		// value less than center->
		// positive.
		double relativeToCenter = values[PLL_CENTER_TRIM] / values[i] - 1;
		int32_t asPP10M = (int32_t) (relativeToCenter * 1E7 + 0.5);
		int16_t check = asPP10M;
		//if (asPP10M != check) {
		// strange, that is far too much deviation. Should be within +-3000 or so.
		//} else {
		result[i] = asPP10M;
		//}
	}
}

// Does not need GPS.
void selfCalibratePLLTrim() {
	PLLTrimCalibration_t tmpCal;

	uint8_t ledWas = LED_init(1);
	_selfCalibratePLLTrim(tmpCal.trimEffect, 250E3, 5);
	LED_init(ledWas);

	LED_off();
	PLL_shutdown();
	tmpCal.checksum = PLLTrimCalibration_Checksum(&tmpCal);
	NVM_WritePLLTrimCalibration(&tmpCal);

	// send some radio messages with the cal. values.
	for (uint8_t r = 0; r < 2; r++) {
		uint8_t i = PLL_MIN_TRIM;
		while (i <= PLL_MAX_TRIM) {
			strcpy(currentTextMessage, "TRIM ");
			char* buf = currentTextMessage + strlen("TRIM ");
			for (uint8_t j = 0;; j++) {
				buf = printInteger(buf, i);
				*(buf++) = ':';
				buf = printInteger(buf, tmpCal.trimEffect[i]);
				i++;
				if (j == 6 || i > PLL_MAX_TRIM) {
					*(buf++) = 0;
					break;
				} else {
					*(buf++) = ',';
				}
			}
			APRS_transmitMessage(VHF, TEXT_MESSAGE, DIAGNOSTICS_APRS_FREQUENCY);
			timer_sleep(1000);
		}
	}
}

static void testModulate(uint8_t i) {
	if (i)
		GPIOA->BSRR = 1 << 4;
	else
		GPIOA->BRR = 1 << 4;
}

// Simplified version of above.
void verifyWSPRModulation(uint8_t trim) {
	double totalNumPLLCycles[2] = { 0 };
	if (GPIOA->OTYPER & (1 << 4)) {
		GPIOA->OTYPER &= ~(1 << 4);
	}
	GPIOA->MODER = (GPIOA->MODER & ~(3 << (4 * 2))) | (1 << (4 * 2));
	PLL_setBypassModeWithDivision(trim, CALIBRATION_DIVISION);
	timer_sleep(1000);
	while (1) {
		for (uint8_t mod = 0; mod < 2; mod++) {
			LED_toggle();
			//WSPR_modulate(mod);
			testModulate(mod);
			PLL_setBypassModeWithDivision(trim, 7);
			timer_sleep(50);
			// uint32_t thisCycle = directPLLGPScalibration(3); // quarter Hz res.
			double thisCycle = measurePeriod(PLL_CYCLES_PER_GPS_CYCLE, 1E6, 2000);
			if (thisCycle > 0.999 * DEFAULT_XTAL_FREQ / CALIBRATION_DIVISION
					&& thisCycle < 1.001 * DEFAULT_XTAL_FREQ / CALIBRATION_DIVISION) {
				totalNumPLLCycles[mod] = thisCycle;
			}
		}
		if (totalNumPLLCycles[0] && totalNumPLLCycles[1]) {
			double dev = totalNumPLLCycles[1] / totalNumPLLCycles[0] - 1;
			int asPPB = 1E9 * dev;
			char* buf = currentTextMessage + strlen("WSPRMOD ");
			strcpy(currentTextMessage, "WSPRMOD ");
			buf = printInteger(buf, trim);
			*buf++ = ',';
//					buf = printInteger(buf, mod);
			*buf++ = 'x';
			*buf++ = ':';
			printInteger(buf, asPPB);
			APRS_transmitMessage(
					VHF,
					TEXT_MESSAGE,
			DIAGNOSTICS_APRS_FREQUENCY);
			return;
		}
	}
	GPIOA->MODER = GPIOA->MODER | (3 << (4 * 2));
}

void verifyRTCCalibration() {
	double numPeriods = measurePeriod(RTC_CYCLES_PER_GPS_CYCLE, 1E6, 1E6/32+2000);
	strcpy(currentTextMessage, "RTC ");
	char* buf = currentTextMessage + strlen("RTC ");
	// Just doesn't work as claimed.
	// It returns same char* as its argument, not a pointer to end of copy: buf = strcpy(buf, "RTC ");
	buf = print2DigitFloat(buf, (numPeriods / 32768.0 - 1.0) * 1E6);
	strcpy(buf, " ppm");
	APRS_transmitMessage(VHF, TEXT_MESSAGE, DIAGNOSTICS_APRS_FREQUENCY);
}

uint8_t isPLLTrimCalibrationValid() {
	return pllTrimCalibration.checksum
			== PLLTrimCalibration_Checksum(&pllTrimCalibration);
}

static const int16_t DEFAULT_PLL_CONTROL_TRIM[] =
PLL_XTAL_TRIM_PP10M_VALUES;

int16_t getPLLTrimCalibration(uint8_t trim) {
	//if (isPLLTrimCalibrationValid())
	//	return pllTrimCalibration.trimEffect[trim];
	return DEFAULT_PLL_CONTROL_TRIM[trim];
}
