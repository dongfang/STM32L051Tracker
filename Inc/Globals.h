/*
 * Globals.h
 *
 *  Created on: Oct 16, 2016
 *      Author: dongfang
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "Types.h"
#include "NVM.h"
#include "GPS.h"

//extern NMEA_TimeInfo_t GPSTime;
//extern NMEA_CRS_SPD_Info_t GPSCourseSpeed;
//extern NMEA_StatusInfo_t GPSStatus;

extern Location_t GPSPosition;
extern Location_t lastNonzeroPosition;
extern Location_t lastNonzero3DPosition;
extern uint16_t lastGPSFixTime;
extern uint16_t lastWSPRWindowWaitTime;
extern boolean latestAPRSRegions[12]; 	 // 12 is sufficently large for the world map...
extern boolean latestAPRSCores[12];	 // 12 is sufficently large for the world map...
// extern const CalibrationRecord_t* currentCalibration;
extern char scheduleName;

extern float temperature;
extern int8_t simpleTemperature;
extern float batteryVoltage;
extern float solarVoltage;
extern float speed_kts;

extern const LogRecord_t flashdata[NUM_LOG_RECORDS];
extern const LogRecordIndex_t storedRecordIndex;
extern const PLLCtrlCalibration_t pllCtrlCalibration;
extern const FlightLog_t flightLog;
extern const CalibrationRecord_t calibrationByTemperatureRanges[];


#endif /* GLOBALS_H_ */
