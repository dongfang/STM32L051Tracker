#ifndef _NMEA_H_
#define _NMEA_H_

#include "Types.h"

//#define UBX_CONF_PORT_MESSAGE {0x06,0x00,1,0x00,0x00,0x00,

#define UBX_POLL_NAV5_MESSAGE {0x06,0x24,0x00,0x00}
/*
#define UBX_INIT_NAV5_MESSAGE {0x06,0x24,0x05,0x00,/*mask allows setting of just the dynModel and fixmode* /\
	6,3,0,0, /*dynModel 6 (airborne < 1g) and 2/3D fixmode* /\
	0,0,0x10,0x27,\
	0,0,/*2D fix alt=0 and 3D alt variance=10000 units* /\
	5,0,/*min elev 5 degrees (we are nice and high up and less might work, but the param seems not settable, drlimit=whatever)* /\
	0xfa,0,0xfa,0,/*Position DoP mask 250 and time DoP 250  .. 19* /\
	0xfa,0,0x2c,1,/* 20.. Position accuracy mask 250m and time accuracy mask 300  23* /\
	0,0,/* 24.. static hold threshold and dgps timeout .. 25 * /\
	0,0,0,0,0,0,0,0,0,0 /*all the rest are just left at zero * /\
	}
*/

// UBX preamble b5 62
#define UBX_INIT_NAV5_MESSAGE {\
 0x06, 0x24 ,0x24 ,0x00 ,0xff ,0xff ,0x06, 0x03 ,0x00 ,0x00 ,0x00 ,0x00 ,0x10 ,0x27 \
,0x00 ,0x00 ,0x05 ,0x00 ,0xfa ,0x00 ,0xfa ,0x00 ,0x64 ,0x00 ,0x2c ,0x01 ,0x00 ,0x3c ,0x00 ,0x00 \
,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x52 ,0xe8 ,0xb5 ,0x62 ,0x06 ,0x24 \
,0x00, 0x00 ,0x2a, 0x84}
// this is checksum from u-center.
//2a 84

typedef struct {
	uint8_t length;
	uint8_t* message;
} UBX_MESSAGE;

typedef enum {
	CONSUMED,
	NEWDATA,
	INVALID,
} MessageState;

typedef struct {
	float groundSpeed;
	float course;
} NMEA_CRS_SPD_Info_t;

typedef struct {
	uint8_t fixMode;
	uint8_t numberOfSatellites;
	float horizontalAccuracy;
} NMEA_StatusInfo_t;

typedef void(GPSStopFunctionInit_t)();
typedef uint8_t(GPSStopFunction_t)(void*);

#define BAUDRATE 9600

//void GPS_start();
//void GPS_stopListening();
//void GPS_shutdown();
uint8_t GPSCycle_voltageLimited();
uint8_t GPSCycle_timeLimited();
uint8_t GPS_waitForTimelock(GPSStopFunctionInit_t* stopInit, GPSStopFunction_t* stopFunction, void* limit);
uint8_t GPS_waitForPosition(GPSStopFunctionInit_t* stopInit, GPSStopFunction_t* stopFunction, void* limit);
uint8_t GPS_waitForPrecisionPosition(GPSStopFunctionInit_t* stopInit, GPSStopFunction_t* stopFunction, void* limit);
void GPS_driver();

// Fake location to be remote from certain stuff
void excludeZones(Location_t* location);

extern DateTime_t GPSDateTime;
extern NMEA_CRS_SPD_Info_t GPSCourseSpeed;
extern Location_t GPSPosition;
extern NMEA_StatusInfo_t GPSStatus;

extern Location_t lastNonzeroPosition; // __attribute__((section (".noinit")));
extern Location_t lastNonzero3DPosition; // __attribute__((section (".noinit")));

extern float climbRate;
extern double odometer_nm;
extern float speed_kts;

#endif // _NMEA_H_
