/*
 * PrecisionTimer.h
 *
 *  Created on: Oct 18, 2016
 *      Author: dongfang
 */

#ifndef PRECISIONTIMER_H_
#define PRECISIONTIMER_H_

//#define MAX_NUM_CAPTURE_VALUES 10
//extern volatile uint32_t PT_extendedCaptureValue;

double measurePeriod(TimerMeasurement_t inputSelect, uint32_t numRTCCycles, uint32_t maxTimeMillis);

#endif /* PRECISIONTIMER_H_ */
