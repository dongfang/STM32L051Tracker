/*
 * NVM.h
 *
 *  Created on: Oct 8, 2016
 *      Author: dongfang
 */

#ifndef NVM_H_
#define NVM_H_

#include "Types.h"

typedef struct {
	int32_t lat;	//1
	int32_t lon;	//2
	int16_t alt;	//2.5
	uint16_t numVHF;//3
	uint16_t numHF;//3.5
	uint16_t status; //4 words
} LogRecord_t;

#define NUM_LOG_RECORDS (16384 / sizeof(LogRecord_t))

#define MAX_NUM_STORED_RECORDS (16384 / sizeof(LogRecord_t))

void storeLogRecord(uint16_t i, LogRecord_t* const data);
LogRecord_t* const logRecord(uint16_t i);

uint8_t uncompressDateHoursMinutes(LogRecord_t* record, Time_t* time);
uint16_t uncompressBatteryVoltageTomV(LogRecord_t* record);
uint16_t uncompressSolarVoltageTomV(LogRecord_t* record);

// extern const LogRecord_t flashdata[];

#endif /* NVM_H_ */
