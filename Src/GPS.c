#include "GPS.h"
#include "Types.h"
#include <math.h>
#include "Setup.h"
#include "LED.h"
#include "RTC.h"
#include "ADC.h"
#include "NVM.h"
#include "Globals.h"
#include "Systime.h"
#include "stm32l0xx.h"

DateTime_t GPSDateTime __attribute__((section (".noinit")));
NMEA_CRS_SPD_Info_t GPSCourseSpeed __attribute__((section (".noinit")));
Location_t GPSPosition __attribute__((section (".noinit")));
NMEA_StatusInfo_t GPSStatus __attribute__((section (".noinit")));

uint16_t lastGPSFixTime __attribute__((section (".noinit")));

Location_t lastNonzeroPosition; // __attribute__((section (".noinit")));
Location_t lastNonzero3DPosition; // __attribute__((section (".noinit")));

float speed_kts;
Time_t lastAltitudeTime;
float lastAltitude;
float climbRate;

// Do some data event processing when data has arrived.
void onNewGPSData() {
	if (GPSPosition.lat != 0 && GPSPosition.lon != 0) {
		lastNonzeroPosition = GPSPosition;

		if (GPSPosition.alt != 0) {
			lastNonzero3DPosition = GPSPosition;

			// calculate climb rate.
			int timeSinceLastAltitude = timeAfter_seconds(&lastAltitudeTime,
					&GPSDateTime.time);
			if (timeSinceLastAltitude >= 300) {
				lastAltitudeTime = GPSDateTime.time;
				float dAltitude = GPSPosition.alt - lastAltitude;
				lastAltitude = GPSPosition.alt;
				climbRate = dAltitude * 60.0 / timeSinceLastAltitude;
			}
		}
	}
}

void flashNumSatellites(uint8_t numSatellites) {
	static uint8_t cnt;

	boolean odd = (cnt & 1) != 0;

	if (odd) {
		LED_off();
	} else {
		if (cnt <= numSatellites * 2) {
			LED_on();
		}
	}

	cnt++;
	if (cnt >= 20)
		cnt = 0;
}

extern void GPS_invalidateDateTime();
extern boolean GPS_isDateTimeValid();
extern void GPS_invalidatePosition();
extern boolean GPS_isPositionValid();
extern void GPS_getData();
extern void GPS_invalidateNumSatellites();
extern uint8_t GPS_numberOfSatellites();
extern void GPS_powerOff();
// Implementation of stopListening is elsewhere. It depends on the IO type to GPS.
extern void GPS_stopListening();
// Implementation of GPS_kill is elsewhere. It depends on the power control mechanism.

static uint32_t gpsStartTime;

void GPSTimeoutStopFunctionInit() {
	gpsStartTime = systime;
}

uint8_t GPSTimeoutStopFunction(void* limitptr) {
	uint32_t limit = *((uint32_t*) limitptr);
	return systime > gpsStartTime + limit;
}

void GPSVoltageStopFunctionInit() {
	// we can, at least currently, assume it was already done.
	// ADC_init();
}

uint8_t GPSVoltageStopFunction(void* limitptr) {
	float voltageNow = ADC_measureBatteryVoltage();
	vBattery = voltageNow;
	float limit = *((float*) limitptr);
	return voltageNow < limit;
}

uint8_t GPS_waitForTimelock(GPSStopFunctionInit_t* stopInit,
		GPSStopFunction_t* stopFunction, void* limit) {
	stopInit();
	GPS_invalidateDateTime();

	do {
		GPS_getData();
		// volatile uint32_t uart1ClockSource = RCC->CCIPR & RCC_CCIPR_USART1SEL;
		flashNumSatellites(GPSStatus.numberOfSatellites);
		// trace_printf("now %02d:%02d:%02d tvalid %d, dvalid %d\n", GPSTime.time.hours, GPSTime.time.minutes, GPSTime.time.seconds, GPSTime.time.valid, GPSTime.date.valid);
		timer_sleep(250);
	} while ((!GPS_isDateTimeValid()
			|| (GPSDateTime.time.hours == 0 && GPSDateTime.time.minutes == 0
					&& GPSDateTime.time.seconds == 0)) && !stopFunction(limit));
	LED_off();
	GPS_getData();
	if (GPS_isDateTimeValid()) {
		//trace_printf("GPS time success: %02d:%02d:%02d\n", GPSDateTime.time.hours,
		//		GPSDateTime.time.minutes, GPSDateTime.time.seconds);
		return 1;
	} else {
		// trace_printf("GPS wait for timelock: FAIL\n");
		return 0;
	}
}

boolean GPS_waitForPosition(GPSStopFunctionInit_t* stopInit,
		GPSStopFunction_t* stopFunction, void* limit) {
	stopInit();
	GPS_invalidatePosition();
	do {
		GPS_getData();
		flashNumSatellites(GPSStatus.numberOfSatellites);
		timer_sleep(250);
	} while (!GPS_isPositionValid() && !stopFunction(limit));
	LED_off();
	GPS_getData();
	return GPSPosition.valid == 'A';
}

uint16_t odometer_checksum(Odometer_t* odo) {
	/*
	 Position_t lastOdometeredPosition;
	 Time_t lastOdometerTime;
	 double odometer_nm;
	 uint16_t checksum;
	 */
	return (uint16_t) (12345 + odo->lastOdometeredPosition.lat
			+ 3 * odo->lastOdometeredPosition.lon + 7 * odo->odometer_nm);
}

boolean GPS_waitForPrecisionPosition(GPSStopFunctionInit_t* stopInit,
		GPSStopFunction_t* stopFunction, void* limit) {
	stopInit();
	GPS_invalidateNumSatellites();
	boolean timedout;
	do {
		GPS_getData();
		flashNumSatellites(GPSStatus.numberOfSatellites);
		timer_sleep(250);
	} while ((GPSPosition.valid != 'A'
			|| (GPS_numberOfSatellites() < REQUIRE_HIGH_PRECISION_NUM_SATS)
			|| (REQUIRE_HIGH_PRECISION_ALTITUDE && GPSPosition.alt == 0)
			|| GPSStatus.fixMode < REQUIRE_HIGH_PRECISION_FIXLEVEL)
			&& !(timedout = stopFunction(limit)));

	LED_off();

	GPS_getData();

	if (timedout) {
//		trace_printf("GPS wait for precision pos: FAIL\n");
	} else if (GPSPosition.lat != 0 || GPSPosition.lon != 0) {
		Odometer_t tmpOdo;
		uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
		RCC->AHBENR |= RCC_AHBENR_MIFEN;
		tmpOdo = odometer;
		RCC->AHBENR = (RCC_AHBENR_MIFEN & ~RCC_AHBENR_MIFEN) | alreadyRunning;

		if (odometer_checksum(&tmpOdo) == tmpOdo.checksum) {
			// valid
			double latFactor = cos(GPSPosition.lat * 0.01745329251994); // convert to radians
			double dist = (GPSPosition.lat - tmpOdo.lastOdometeredPosition.lat)
					* (GPSPosition.lat - tmpOdo.lastOdometeredPosition.lat);
			dist += (GPSPosition.lon - tmpOdo.lastOdometeredPosition.lon)
					* (GPSPosition.lon - tmpOdo.lastOdometeredPosition.lon)
					* latFactor;
			dist = sqrt(dist);
			// now it is in degrees latitude, each of which is 60nm, or 1852 * 60m

			dist = dist * 60.0; // now it's in nautical miles
			tmpOdo.odometer_nm += dist;

			Time_t now;
			RTC_read(&now);

			// time in hours
			float time_h = timeAfter_seconds(&tmpOdo.lastOdometerTime, &now)
					/ 3600.0;

			// nm / hours === knots
			speed_kts = dist / time_h;
			tmpOdo.lastOdometerTime = now;
		} else {
			tmpOdo.odometer_nm = 0;
			speed_kts = 0;
		}

		tmpOdo.lastOdometeredPosition.lat = GPSPosition.lat;
		tmpOdo.lastOdometeredPosition.lon = GPSPosition.lon;
		tmpOdo.checksum = odometer_checksum(&tmpOdo);
		NVM_WriteOdometer(&tmpOdo);
	}

	return !timedout;
}

void GPS_shutdown() {
// Stop IO
	GPS_stopListening();
// Cut power
	GPS_powerOff();
// And note for the crash log that GPS was off.
	// PWR_stopDevice(E_DEVICE_GPS);
}

uint8_t GPSCycle_timeLimited() {
	uint32_t gpsTimeout = 100000;
	uint32_t gpsstart = systime;
	uint8_t result = 0;

	GPS_start();

	if (GPS_waitForTimelock(GPSTimeoutStopFunctionInit, GPSTimeoutStopFunction,
			&gpsTimeout)) {
		RTC_set(&GPSDateTime.time);
		result = 1;
	}

	if (GPS_waitForPrecisionPosition(GPSTimeoutStopFunctionInit,
			GPSTimeoutStopFunction, &gpsTimeout)) {
		lastGPSFixTime = (systime - gpsstart) / 1000;
		calibratePLLFrequency();
		result = 2;
	}

	GPS_shutdown();

	if (result == 2) {
		recordFlightLog();
	}

	return result;
}

uint8_t GPSCycle_voltageLimited() {
	float cutoffVoltage = 2.1;
	uint32_t gpsstart = systime;
	uint8_t result = 0;

	switchTo2MHzMSI();
	GPS_start();

	if (GPS_waitForTimelock(GPSVoltageStopFunctionInit, GPSVoltageStopFunction,
			&cutoffVoltage)) {
		RTC_set(&GPSDateTime.time);
		result = 1;
	}

	if (GPS_waitForPrecisionPosition(GPSVoltageStopFunctionInit,
			GPSVoltageStopFunction, &cutoffVoltage)) {
		lastGPSFixTime = (systime - gpsstart) / 1000;
		result = 2;
	}

	if (result == 2) {
		switchTo8MHzHSI();
		calibratePLLFrequency();
	}

	GPS_shutdown();

	if (result == 2) {
		recordFlightLog();
	}

	return result;
}
