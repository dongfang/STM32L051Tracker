/*
 * Calibration.h
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#define NUM_TEMPERATURE_RANGES 11
#define TEMPERATURE_RANGES {-60,-50,-40,-30,-20,-10,0,10,20,30,40}
extern const int8_t temperatureRanges[];
extern const CalibrationRecord_t calibrationByTemperatureRanges[];

const CalibrationRecord_t* getCalibration();
//uint32_t expectedPLLXtalFrequency(int8_t simpleTemperature);

#endif /* CALIBRATION_H_ */
