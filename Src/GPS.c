#include <inttypes.h>
#include "GPS.h"
#include <math.h>
#include "stm32l0xx.h"
#include "Setup.h"

NMEA_TimeInfo_t GPSTime;
uint8_t GPSDate;
NMEA_CRS_SPD_Info_t GPSCourseSpeed;
Location_t GPSPosition;
NMEA_StatusInfo_t GPSStatus;

uint16_t lastGPSFixTime;

Location_t lastNonzeroPosition; // __attribute__((section (".noinit")));
Location_t lastNonzero3DPosition; // __attribute__((section (".noinit")));

Position_t lastOdometeredPosition __attribute__((section (".noinit")));
double lastOdometeredPositionCheck __attribute__((section (".noinit")));
double odometer_nm __attribute__((section (".noinit")));
float speed_kts;

Time_t lastOdometerTime __attribute__((section (".noinit")));

Time_t lastAltitudeTime;
float lastAltitude;
float climbRate;

void debugTime(const char* text, Time_t* time) {
	trace_printf("%s said %02u:%02u:%02u\n", text, time->hours, time->minutes,
			time->seconds);
}

// Do some data event processing when data has arrived.
void onNewGPSData() {
	if (GPSPosition.lat != 0 && GPSPosition.lon != 0) {
		lastNonzeroPosition = GPSPosition;

		if (GPSPosition.alt != 0) {
			lastNonzero3DPosition = GPSPosition;

			int timeSinceLastAltitude = timeAfter_seconds(&lastAltitudeTime, &GPSTime.time);
			if (timeSinceLastAltitude >= 120) {
				lastAltitudeTime = GPSTime.time;
				float dAltitude = GPSPosition.alt - lastAltitude;
				lastAltitude = GPSPosition.alt;
				climbRate = dAltitude * 60.0 / timeSinceLastAltitude;
//				trace_printf("dAlt is %d, dt is %d, climb rate: %d\n", (int)dAltitude, timeSinceLastAltitude, (int)climbRate);
			}
		}
	}
}

void flashNumSatellites(uint8_t numSatellites) {
	static uint8_t cnt;

	boolean odd = (cnt & 1) != 0;

	if (odd) {
		LED_PORT->BSRR = LED_OFF; // LED
	} else {
		if (cnt <= numSatellites * 2) {
			LED_PORT->BSRR = LED_ON; // LED
		}
	}

	cnt++;
	if (cnt >= 25)
		cnt = 0;
}

extern void GPS_invalidateDateTime();
extern boolean GPS_isDateTimeValid();
extern void GPS_invalidatePosition();
extern boolean GPS_isPositionValid();
extern void GPS_getData();
extern void GPS_invalidateNumSatellites();
extern uint8_t GPS_numberOfSatellites();
extern void GPS_powerOn();
extern void GPS_powerOff();
// Implementation of stopListening is elsewhere. It depends on the IO type to GPS.
extern void GPS_stopListening();
// Implementation of GPS_kill is elsewhere. It depends on the power control mechanism.

uint8_t GPS_waitForTimelock(uint32_t maxTime) {
	timer_mark();
	GPS_invalidateDateTime();

	trace_printf("Waiting for GPS time\n");

	do {
		GPS_getData();
		flashNumSatellites(GPSStatus.numberOfSatellites);
		// trace_printf("now %02d:%02d:%02d tvalid %d, dvalid %d\n", GPSTime.time.hours, GPSTime.time.minutes, GPSTime.time.seconds, GPSTime.time.valid, GPSTime.date.valid);
		timer_sleep(100);
	} while ((!GPS_isDateTimeValid()
			|| (GPSTime.time.hours == 0 && GPSTime.time.minutes == 0
					&& GPSTime.time.seconds == 0)) && !timer_elapsed(maxTime));
	LED_PORT->BSRR = LED_OFF; // LED
	GPS_getData();
	if (GPS_isDateTimeValid()) {
		trace_printf("GPS time success: %02d:%02d:%02d\n", GPSTime.time.hours,
				GPSTime.time.minutes, GPSTime.time.seconds);
		return 1;
	} else {
		trace_printf("GPS wait for timelock: FAIL\n");
		return 0;
	}
}

boolean GPS_waitForPosition(uint32_t maxTime) {
	timer_mark();

	GPS_invalidatePosition();

	do {
		GPS_getData();
		flashNumSatellites(GPSStatus.numberOfSatellites);
		timer_sleep(100);
	} while (!GPS_isPositionValid() && !timer_elapsed(maxTime));
	LED_PORT->BSRR = LED_OFF; // LED
	GPS_getData();
	if (GPS_isPositionValid()) {
		trace_printf("Got GPS position: %d, %d, %d\n",
				(int) (GPSPosition.lat * 1.0E7),
				(int) (GPSPosition.lon * 1.0E7), (int) GPSPosition.alt);
	} else {
		trace_printf(
				"GPS wait for position FAIL: valid:%c, fixMode:%u numSats:%u\n",
				GPSPosition.valid, GPSStatus.fixMode,
				GPSStatus.numberOfSatellites);
	}
	return GPSPosition.valid == 'A';
}

static void GPS_debugGPSPosition() {
#ifdef TRACE_GPS
	trace_printf("GPS pos: lat %d, lon %d, alt %d, valid %c, fix %d, sat %d\n",
			(int) (GPSPosition.lat * 1000), (int) (GPSPosition.lon * 1000),
			(int) (GPSPosition.alt), GPSPosition.valid, GPSStatus.fixMode,
			GPSStatus.numberOfSatellites);
#endif
}

boolean GPS_waitForPrecisionPosition(uint32_t maxTime) {
	timer_mark();
	GPS_invalidateNumSatellites();
	boolean timeout;
	uint8_t debugPrintCnt = 0;
	do {
		GPS_getData();
		flashNumSatellites(GPSStatus.numberOfSatellites);
		timer_sleep(100);
		debugPrintCnt++;
		if (debugPrintCnt == 10) {
			debugPrintCnt = 0;
			GPS_debugGPSPosition();
		}
	} while ((GPSPosition.valid != 'A'
			|| (GPS_numberOfSatellites() < REQUIRE_HIGH_PRECISION_NUM_SATS)
			|| (REQUIRE_HIGH_PRECISION_ALTITUDE && GPSPosition.alt == 0)
			|| GPSStatus.fixMode < REQUIRE_HIGH_PRECISION_FIXLEVEL)
			&& !(timeout = timer_elapsed(maxTime)) && timer_sleep(100));

	LED_PORT->BSRR = LED_OFF; // LED
	GPS_getData();
	GPS_debugGPSPosition();

	if (timeout) {
		trace_printf("GPS wait for precision pos: FAIL\n");
	} else if (GPSPosition.lat != 0 || GPSPosition.lon != 0) {
		if (lastOdometeredPosition.lat + lastOdometeredPosition.lon + 1
				== lastOdometeredPositionCheck) {
			// valid
			double latFactor = cos(GPSPosition.lat * 0.01745329251994); // convert to radians
			double dist = (GPSPosition.lat - lastOdometeredPosition.lat)
					* (GPSPosition.lat - lastOdometeredPosition.lat);
			dist += (GPSPosition.lon - lastOdometeredPosition.lon)
					* (GPSPosition.lon - lastOdometeredPosition.lon)
					* latFactor;
			dist = sqrt(dist);
			// now it is in degrees latitude, each of which is 60nm, or 1852 * 60m

			dist = dist * 60.0; // now it's in nautical miles
			odometer_nm += dist;

			Time_t now;
			RTC_getTime(&now);
			float time_h = timeAfter_seconds(&lastOdometerTime, &now) / 3600.0;
			speed_kts = dist / time_h;
			// trace_printf("Dist, time, speed: %d,%d,%d\n", (int)dist, time_s, (int)speed_m_s);
			lastOdometerTime = now;
		} else {
			odometer_nm = 0;
			speed_kts = 0;
		}

		lastOdometeredPosition.lat = GPSPosition.lat;
		lastOdometeredPosition.lon = GPSPosition.lon;
		lastOdometeredPositionCheck = lastOdometeredPosition.lat
				+ lastOdometeredPosition.lon + 1;
	}

	return !timeout;
}

void GPS_start() {
// GPS on
	// PWR_startDevice(E_DEVICE_GPS);
	GPS_powerOn();
}

void GPS_shutdown() {
// Stop IO
	GPS_stopListening();
// Cut power
	GPS_powerOff();
// And note for the crash log that GPS was off.
	// PWR_stopDevice(E_DEVICE_GPS);
}
