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

boolean latestAPRSRegions[12]; // 12 is sufficently large for the world map... and yes these ARE inited to zero.
boolean latestAPRSCores[12]; // 12 is sufficently large for the world map... and yes these ARE inited to zero.

float speed_kts __attribute__((section (".noinit")));
float course __attribute__((section (".noinit")));
//float climbRate __attribute__((section (".noinit")));

// Do some data event processing when data has arrived.
void onNewGPSPosition() {
	if (GPSPosition.lat != 0 && GPSPosition.lon != 0) {
		lastNonzeroPosition = GPSPosition;

		if (GPSPosition.alt != 0) {
			lastNonzero3DPosition = GPSPosition;
		}
	}
}

void flashNumSatellites(uint8_t numSatellites) {
	static uint32_t nextFlashTime;
	static uint8_t cnt;

	if (systime < nextFlashTime)
		return;
	nextFlashTime = systime + 250;

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
		GPS_driver();
		// volatile uint32_t uart1ClockSource = RCC->CCIPR & RCC_CCIPR_USART1SEL;
		flashNumSatellites(GPSStatus.numberOfSatellites);
		// trace_printf("now %02d:%02d:%02d tvalid %d, dvalid %d\n", GPSTime.time.hours, GPSTime.time.minutes, GPSTime.time.seconds, GPSTime.time.valid, GPSTime.date.valid);
		// timer_sleep(250);
	} while ((!GPS_isDateTimeValid()
			|| (GPSDateTime.time.hours == 0 && GPSDateTime.time.minutes == 0
					&& GPSDateTime.time.seconds == 0)) && !stopFunction(limit));
	LED_off();
	GPS_driver();
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
		// timer_sleep(250);
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
		GPS_driver();
		flashNumSatellites(GPSStatus.numberOfSatellites);
		// timer_sleep(250);
	} while ((GPSPosition.valid != 'A'
			|| (GPS_numberOfSatellites() < REQUIRE_HIGH_PRECISION_NUM_SATS)
			|| (REQUIRE_HIGH_PRECISION_ALTITUDE && GPSPosition.alt == 0)
			|| GPSStatus.fixMode < REQUIRE_HIGH_PRECISION_FIXLEVEL)
			&& !(timedout = stopFunction(limit)));

	LED_off();

	GPS_driver();

	if (timedout) {
//		trace_printf("GPS wait for precision pos: FAIL\n");
	} else if (GPSPosition.lat != 0 || GPSPosition.lon != 0) {
		// calculate climb rate.
		Odometer_t tmpOdo;
		uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
		RCC->AHBENR |= RCC_AHBENR_MIFEN;
		tmpOdo = odometer;
		RCC->AHBENR = (RCC_AHBENR_MIFEN & ~RCC_AHBENR_MIFEN) | alreadyRunning;

		if (odometer_checksum(&tmpOdo) == tmpOdo.checksum) {
			int timeSinceLastOdo =
					timeAfter_seconds(&tmpOdo.lastOdometerTime,
					&GPSDateTime.time);

			/*
			float dAltitude = GPSPosition.alt - tmpOdo.altitude;
			tmpOdo.altitude = GPSPosition.alt;
			climbRate = dAltitude * 60.0 / timeSinceLastOdo;
			*/

			// valid
			double latFactor = cos(GPSPosition.lat * 0.01745329251994); // convert to radians
			double lateralDist = GPSPosition.lat - tmpOdo.lastOdometeredPosition.lat;
			double longitudinalDist = (GPSPosition.lon - tmpOdo.lastOdometeredPosition.lon) * latFactor;

			double distsq = lateralDist*lateralDist + longitudinalDist*longitudinalDist;
			double dist = sqrt(distsq);
			// now it is in degrees latitude, each of which is 60nm, or 1852 * 60m

			dist = dist * 60.0; // now it's in nautical miles
			tmpOdo.odometer_nm += dist;

			// time in hours
			float time_h = timeSinceLastOdo / 3600.0;

			// nm / hours === knots
			speed_kts = dist / time_h;
			// dLat=1, dLon=0 should yield 0. atan2(-1,0) = pi = 180d want 0 180-180 = 0
			// dLat=1, dLon=1 should yield 45. atan2(-1,1) = 3pi/4 = 135d want 45 180-135 = 45
			// dLat=0, dLon=1 should yield 90. atan2(0,1) = pi/2 = 90 want 90 180-90 = 90
			// dLat=-1, dLon=1 should yield 135. atan2(1,1) = pi/4 = 45
			// dLat=-1, dLon=0 should yield 180. atan2(1,0) = 0 = 0
			// dLat=-1, dLon=-1 should yield 225. atand(1,-1) = -pi/4 = -45  180 --45 = 225
			// dLat=1, dLon=-0.000001 should yield 359.99... atan2(-1,-0.0001) = -pi+a little = -179.999

			course = 180 - atan2(-lateralDist, longitudinalDist) * 57.2957795130;

			tmpOdo.lastOdometerTime = GPSDateTime.time;
		} else {
			tmpOdo.odometer_nm = 0;
			speed_kts = 0;
			course = 0;
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

	GPS_stopListening();

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
