/*
 * NVM.h
 *
 *  Created on: Oct 8, 2016
 *      Author: dongfang
 */

#ifndef NVM_H_
#define NVM_H_
#include "Types.h"

// #define MAX_NUM_STORED_RECORDS (16384 / sizeof(LogRecord_t))
#define NUM_LOG_RECORDS (16384 / sizeof(LogRecord_t))

void NVM_toreLogRecord(uint16_t i, LogRecord_t* const data);
// LogRecord_t* const logRecord(uint16_t i);

void NVM_writeCalibrationRecord(CalibrationRecord_t* const record, uint8_t index);

uint8_t uncompressDateHoursMinutes(LogRecord_t* record, Time_t* time);
uint16_t uncompressBatteryVoltageTomV(LogRecord_t* record);
uint16_t uncompressSolarVoltageTomV(LogRecord_t* record);

// extern const LogRecord_t flashdata[];

#endif /* NVM_H_ */
