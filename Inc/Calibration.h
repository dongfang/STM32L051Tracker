/*
 * Calibration.h
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "Types.h"

#define NUM_TEMPERATURE_RANGES 11
#define TEMPERATURE_RANGES {-60,-50,-40,-30,-20,-10,0,10,20,30,40}
extern const int8_t temperatureRanges[];

uint32_t getCalibratedPLLOscillatorFrequency();
uint8_t isPLLConctrolCalibrationValid();

/*
 * Find the frequency of the PLL xtal at default trim by referencing to the GPS.
 * Store in EEPROM temperature based cal. array.
 */
void calibratePLLFrequency();

/*
 * Calibrate PLL trims (find the parts per 10 million effect of each trim setting),
 * comparing to the RTC. Store in EEPROM. Send message with result.
 */
void selfCalibratePLLTrim();

/*
 * Find the ppm deviation of RTC, comparing to GPS. Send message with result.
 */
void verifyRTCCalibration();

/*
 * Check deviation of PLL xtal, using an output with more modulation than the WSPR one.
 * The result in PPB is sent in a message and can be used to calculate the WSPR modulation
 * setting resistors.
 */
void verifyWSPRModulation(uint8_t trim);

int16_t getPLLTrimCalibration(uint8_t trim);

#endif /* CALIBRATION_H_ */
