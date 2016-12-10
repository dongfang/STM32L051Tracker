/*
 * Calibration.c
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */

#include "NVM.h"
#include "Globals.h"
#include "Calibration.h"
#include "stm32l0xx.h"

#define DEFAULT_XTAL_FREQ 26000000
const int8_t temperatureRanges[] = TEMPERATURE_RANGES;

const CalibrationRecord_t defaultCalibration = {
		.transmitterOscillatorFrequencyAtDefaultTrim = DEFAULT_XTAL_FREQ, };

static uint8_t isMeasurementPlausible(uint32_t frequencyMeasured) {
	// allow 500 ppm, that is extreeeeme.
	int limit = 0.0005 * DEFAULT_XTAL_FREQ;
	return (frequencyMeasured
			>= DEFAULT_XTAL_FREQ - limit)
			&& (frequencyMeasured
					<= DEFAULT_XTAL_FREQ + limit);
}

#define CALIBRATION_DIVISION 7
#define CALIBRATION_NUM_CYCLES 1
static uint32_t selfCalibrate() {
	// TODO, remove from here.
	GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 6))) | 0 << (2 * 6);

	while ((GPIOA->IDR & (1 << 6)) == 0) {
		GPS_getData();
		trace_printf("Waiting for GPS timelock (%d)\n", GPSStatus.numberOfSatellites);
		timer_sleep(1000);
	}

	PLL_setBypassModeWithDivision(CALIBRATION_DIVISION, 7);
	timer_sleep(1000);

	// One shot ought to be enough; the +/- 1 error is limited to 0.3ppm
	uint32_t rawResult = directPLLGPScalibration(CALIBRATION_NUM_CYCLES);
	PLL_shutdown();

	uint32_t nresult = (rawResult * CALIBRATION_DIVISION) / CALIBRATION_NUM_CYCLES;

	trace_printf("%d\n", nresult);
	return nresult;
}

static uint8_t getIndex(int8_t temperature) {
	uint8_t index = 0;
	while (index < sizeof(temperatureRanges) - 1
			&& temperatureRanges[index + 1] < temperature)
		index++;
	return index;
}

static uint32_t checksum(const CalibrationRecord_t* record) {
	uint32_t checksum = 0x1F55F1AA;
	checksum += record->transmitterOscillatorFrequencyAtDefaultTrim * 7;
	checksum += record->temperature * 12345;
	return checksum;
}

void calibrate(int8_t temperature) {
	uint32_t selfCalibration = selfCalibrate();
	if (isMeasurementPlausible(selfCalibration)) {
		CalibrationRecord_t tempRecord;
		tempRecord.transmitterOscillatorFrequencyAtDefaultTrim = selfCalibration;
		tempRecord.temperature = temperature;
		// Commit the new calibration.
		tempRecord.checksum = checksum(&tempRecord);
		NVM_writeCalibrationRecord(&tempRecord, getIndex(temperature));
//		calibrationByTemperatureRanges[getIndex(temperature)] = tempRecord;
	}
}

const CalibrationRecord_t* getCalibration() {
	uint8_t index = getIndex(simpleTemperature);

	uint32_t check = checksum(calibrationByTemperatureRanges + index);
	if (check != calibrationByTemperatureRanges[index].checksum) {
		trace_printf("No calibration found for temperature %d in range %d\n",
				temperature, index);
		return &defaultCalibration;	// damn, we had nothing!
	}

	return calibrationByTemperatureRanges + index;
}

/*
 uint32_t expectedPLLXtalFrequency(int8_t simpleTemperature) {
 if (isCalibrationPlausible()) {
 return currentCalibration.transmitterOscillatorFrequencyAtDefaultTrim;
 } else {
 return DEFAULT_XTAL_FREQ;
 }
 }
 */
