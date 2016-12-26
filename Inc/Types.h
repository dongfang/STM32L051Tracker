/*
 * Types.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

#include <stdint.h>
#include <stddef.h>

typedef uint8_t boolean;
#define true 1
#define false 0

typedef enum {
	FLIGHT,
	CALIBRATION,
	GROUNDTEST
} SysState_t;

typedef enum {
	VHF,
	HF
} APRS_Band_t;

typedef enum {
	COMPRESSED_POSITION_MESSAGE,
	TELEMETRY_MESSAGE,
	TEXT_MESSAGE
} APRS_MessageType_t;

typedef enum {
	THIRTY_M,
	TEN_M
} WSPRBand_t;

typedef struct {
	uint8_t year100;
	uint8_t month;
	uint8_t date;
	boolean valid;
} Date_t;

typedef struct {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	boolean valid;
} Time_t;

// various data in whatever units NMEA has chosen to use.
typedef struct {
	Time_t time;
	Date_t date;
} DateTime_t;

typedef struct {
	double lat; // 1e-7
	double lon; // 1e-7
} Position_t;

typedef struct {
	double lat; // 1e-7
	double lon; // 1e-7
	Time_t fixTime;
	float alt;
	char valid; // The A or V of the position
} Location_t;

typedef enum {
	RTC_CYCLES_PER_GPS_CYCLE, RTC_CYCLES_PER_PLL_CYCLE, PLL_CYCLES_PER_GPS_CYCLE
} TimerMeasurement_t;

// Whether we are in an APRS core zone (where we can expect to be received) or not.
typedef enum {
	UNKNOWN_CORE_ZONE, IN_CORE_ZONE, OUTSIDE_CORE_ZONE
} CoreZoneStatus_t;

// An AX.25 address.
typedef struct {
	const char* callsign;
	uint8_t ssid;
} AX25_Address_t;

// #define APRS_LOG_NUM_RETRANSMISSIONS 3

typedef struct {
	uint16_t tailIdx;
	uint16_t headIdx; //[APRS_LOG_NUM_RETRANSMISSIONS];
//	uint16_t entryIdx;
	Time_t lastLogTime;
	uint16_t checksum;
} LogRecordIndex_t;

typedef struct {
	int16_t trimEffect[24];
	uint32_t checksum;
} PLLTrimCalibration_t;

typedef struct {
	// Transmitter osc frequency measured at default trim
	uint32_t transmitterOscillatorFrequencyAtDefaultTrim;

	uint16_t checksum;

	// Temperature measured at
	int8_t temperature;
} CalibrationRecord_t;

typedef struct {
	// Date and time in one minute resolution.
	uint16_t compactedDateTime;
	int16_t alt;
	//1
	int32_t lat;
	//2
	int32_t lon;
	//3
	uint8_t speed_kts;
	int8_t temperature;
	uint8_t numHF;
	uint8_t checksum;
	//4
} LogRecord_t;

typedef struct {
	Position_t lastOdometeredPosition;
	Time_t lastOdometerTime;
	double odometer_nm;
	uint16_t checksum;
} Odometer_t;

#endif /* INC_TYPES_H_ */
