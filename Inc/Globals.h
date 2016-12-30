/*
 * Globals.h
 *
 *  Created on: Oct 16, 2016
 *      Author: dongfang
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "Types.h"
#include "Setup.h"
#include "NVM.h"
#include "GPS.h"

extern volatile uint32_t systime;

extern Location_t GPSPosition;
extern Location_t lastNonzeroPosition;
extern Location_t lastNonzero3DPosition;
extern uint16_t lastGPSFixTime;
extern uint16_t lastWSPRWindowWaitTime;
extern boolean latestAPRSRegions[12]; 	 // 12 is sufficently large for the world map...
extern boolean latestAPRSCores[12];	 // 12 is sufficently large for the world map...

extern int8_t temperature;
extern uint8_t MSITrim;
extern uint8_t isADCUsingHSI;

extern float VDDa;
extern float vBattery;
extern float vSolar;
extern float speed_kts;

extern SysState_t sysState;
extern char currentTextMessage[64];

extern volatile const LogRecord_t flashdata[NUM_LOG_RECORDS];
extern volatile const LogRecordIndex_t flightLogIndex;
extern volatile const CalibrationRecord_t calibrationByTemperatureRanges[];
extern volatile const Odometer_t odometer;
extern volatile const PLLTrimCalibration_t pllTrimCalibration;

#endif /* GLOBALS_H_ */
