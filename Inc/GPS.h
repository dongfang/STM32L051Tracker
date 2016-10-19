#ifndef _NMEA_H_
#define _NMEA_H_

#include "Types.h"

//#define UBX_CONF_PORT_MESSAGE {0x06,0x00,1,0x00,0x00,0x00,

#define UBX_POLL_NAV5_MESSAGE {0x06,0x24,0x00,0x00}
#define UBX_INIT_NAV5_MESSAGE {0x06,0x24,0x24,0x00,0x05,0x00,/*mask allows setting of just the dynModel and fixmode*/\
	6,3, /*dynModel 6 (airborne < 1g) and 2/3D fixmode*/\
	0,0,0,0,0x10,0x27,0,0,/*2D fix alt=0 and 3D alt variance=10000 units*/\
	5,0,/*min elev 5 degrees (we are nice and high up and less might work, but the param seems not settable, drlimit=whatever)*/\
	0xfa,0,0xfa,0,/*Position DoP mask 250 and time DoP 250*/\
	0xfa,0,0x2c,1,/*Position accuracy mask 250m and time accuracy mask 300*/\
	0,0x3c,/*static hold threshold and dgps timeout*/\
	0,0,0,0,0,0,0,0,0,0,0,0/*all the rest are just left at zero */\
	}

/* Checksum algo (from the char after 0x62, till just before checksum bytes):
 * CK_A = 0, CK_B = 0
For(I=0;I<N;I++)
{
    CK_A = CK_A + Buffer[I]
    CK_B = CK_B + CK_A
}
 */
typedef struct {
	uint8_t length;
	uint8_t* message;
} UBX_MESSAGE;

typedef enum {
	CONSUMED,
	NEWDATA,
	INVALID,
} MessageState;

// various data in whatever units NMEA has chosen to use.
typedef struct {
	Time_t time;
	Date_t date;
} NMEA_TimeInfo_t;

typedef struct {
	float groundSpeed;
	float course;
} NMEA_CRS_SPD_Info_t;

typedef struct {
	uint8_t fixMode;
	uint8_t numberOfSatellites;
	float horizontalAccuracy;
} NMEA_StatusInfo_t;

#define BAUDRATE 9600

void GPS_start();
void GPS_stopListening();
void GPS_shutdown();

void GPS_powerUpInit();
boolean GPS_isPowered();

uint8_t GPS_waitForTimelock(uint32_t maxTime);
uint8_t GPS_waitForPosition(uint32_t maxTime);
uint8_t GPS_waitForPrecisionPosition(uint32_t maxTime);

// Fake location to be remote from certain stuff
void excludeZones(Location_t* location);

extern NMEA_TimeInfo_t GPSTime;
extern uint8_t GPSDate;
extern NMEA_CRS_SPD_Info_t GPSCourseSpeed;
extern Location_t GPSPosition;
extern NMEA_StatusInfo_t GPSStatus;

extern Location_t lastNonzeroPosition; // __attribute__((section (".noinit")));
extern Location_t lastNonzero3DPosition; // __attribute__((section (".noinit")));

extern float climbRate;
extern double odometer_nm;
extern float speed_kts;

#endif // _NMEA_H_
