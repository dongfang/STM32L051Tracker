/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32l0xx.h"

#include "Setup.h"
#include "APRS.h"
#include "AX25.h"
#include "AFSK.h"
#include "Calibration.h"
#include "Types.h"
#include "PLL.h"
#include "Globals.h"
#include "LED.h"
#include "RTC.h"

extern uint16_t numGPSChecksumErrors;

static void APRS_initDirectVHFTransmission(uint32_t frequency,
		uint32_t referenceFrequency);
static void APRS_endDirectTransmission();
static void APRS_initDirectHFTransmission(uint32_t frequency,
		uint32_t referenceFrequency);
char currentTextMessage[64] __attribute__((section (".noinit")));

// JUST to avoid printf's huge memory impact.
char* printInteger(char* out, int32_t number) {
	static const int divs[] = { 1, 10, 100, 1000, 10000, 100000, 1000000,
			10000000, 100000000, 1000000000 };
	int dividx = 0;
	size_t pos = 0;
	if (number < 0) {
		out[pos++] = '-';
		number = -number;
	}
	// 0->0, 9->0, 10->1, 99->1, 100->2
	while (dividx <= 8 && number >= divs[dividx + 1]) {
		dividx++;
	}
	boolean started = false;
	do {
		int digit = number / divs[dividx];
		if (started || digit || dividx == 0) {
			out[pos++] = '0' + digit;
			started = true;
		}
		number -= divs[dividx] * digit;
		dividx--;
	} while (dividx >= 0);
	out[pos] = 0;
	return out + pos;
}

char* print2DigitFloat(char* out, float in) {
	int i_Hundredths = (int) (in * 100);
	int i_Tenths = (int) (in * 10);
	int i_Whole = (int) in;
	int i_hundredthdigit = i_Hundredths % 10;
	if (i_hundredthdigit < 0)
		i_hundredthdigit = -i_hundredthdigit;
	int i_tenthdigit = i_Tenths % 10;
	if (i_tenthdigit < 0)
		i_tenthdigit = -i_tenthdigit;

	size_t pos = 0;
	if (i_Whole < 0) {
		out[pos++] = '-';
		i_Whole = -i_Whole;
	}

	char* buf = printInteger(out + pos, i_Whole);

	buf[pos++] = '.';
	buf[pos++] = i_tenthdigit + '0';
	buf[pos++] = i_hundredthdigit + '0';
	buf[pos] = 0;

	return buf + pos;
}

// Module functions
static float meters_to_feet(float m) {
	return m * 3.28084; // m to ft.
}

/*
 void makeTimestamp(char* timestamp) {
 unsigned long now = millis();
 now += START_DATE * (1000UL * 3600UL * 24UL)
 + START_HOURS * (1000UL * 3600UL) + START_MINUTES * (1000UL * 60UL);
 int days = now / (1000UL * 3600UL * 24UL);
 now -= days * (1000UL * 3600UL * 24UL);
 if (days > 30)
 days -= 30;
 days += 26;
 int hours = now / (1000UL * 3600UL);
 now -= hours * (1000UL * 3600UL);
 int minutes = now / (1000UL * 60UL);
 now -= minutes * (1000UL * 30UL);
 int seconds = now / (1000UL);
 sprintf_P(timestamp, PSTR("%02d%02d%02dh"), hours, minutes, seconds);
 }
 */

static void base91encode2char(uint16_t val, char* out) {
	if (val > 8280)
		val = 8280;
	out[0] = val / 91 + 33;
	out[1] = val % 91 + 33;
}

/*
 * See http://he.fi/doc/aprs-base91-comment-telemetry.txt for specification of this.
 */
static uint8_t compressTelemetry(uint16_t seq, uint8_t nval, uint16_t* vals,
		char* out) {
	out[0] = '|';
	base91encode2char(seq, out + 1);
	uint8_t pos = 3;
	for (uint8_t j = 0; j < nval; j++) {
		base91encode2char(vals[j], out + pos);
		pos += 2;
	}
	// and at end
	out[pos++] = '|';
	return pos;
}

static void base91encode4digit(uint32_t n, char* out) {
	uint8_t q = n % 91;
	out[3] = q + 33;
	n /= 91;

	q = n % 91;
	out[2] = q + 33;
	n /= 91;

	q = n % 91;
	out[1] = q + 33;
	n /= 91;
	q = n % 91;

	out[0] = q + 33;
}

static uint8_t compressPosition(float lat, float lon, float alt, char* out) {
	// /YYYYXXXX$csT
	// YYYY is 380926 x (90 – latitude) [base 91]
	// latitude is positive for north, negative for south, in degrees.
	// XXXX is 190463 x (180 + longitude) [base 91]
	// longitude is positive for east, negative for west, in degrees.
	// $ is Symbol Code
	// cs is course-speed
	// T is compression type
	out[0] = '/';
	int32_t ilat = 380926 * (90 - lat);
	base91encode4digit(ilat, out + 1);

	int32_t ilon = 190463 * (180 + lon);
	base91encode4digit(ilon, out + 5);
	out[9] = 'O'; // this should be ????? really... ah o for balloon.

	alt = meters_to_feet(alt);
	alt = log(alt) / log(1.002);
	base91encode2char(alt, out + 10);
	out[12] = 0b110101 + 33;
	return 13;
}

/*
 static uint8_t compressedTimestamp(uint8_t date, Time_t* time, char* out) {
 return sprintf(out, "%02d%02d%02dz", date, time->hours, time->minutes);
 }
 */

static void aprs_send_header(const AX25_Address_t* destination,
		uint16_t txDelay) {
	ax25_begin(txDelay);

	uint8_t numAddresses = 0;
	const AX25_Address_t* addresses[4];

	addresses[numAddresses++] = destination;
	addresses[numAddresses++] = &MY_ADDRESS;

	if (strlen(APRS_DIGI1.callsign)) {
		addresses[numAddresses++] = &APRS_DIGI1;
	}
	if (strlen(APRS_DIGI2.callsign)) {
		addresses[numAddresses++] = &APRS_DIGI2;
	}

	ax25_send_header(addresses, numAddresses);
}

// This does reset, so we can see # reboots. It is not required for APRS anyway.
static uint16_t statusMessageSequence;

void APRS_marshallTextMessage(uint16_t txDelay) {
	aprs_send_header(&APRS_DEST, txDelay);
	ax25_send_byte('>');
	ax25_send_string(currentTextMessage);
	ax25_end();
}

// Exported functions
void APRS_marshallTelemetryMessage(uint16_t txDelay) {
	uint32_t alreadyRunning = RCC->AHBENR & RCC_AHBENR_MIFEN;
	RCC->AHBENR |= RCC_AHBENR_MIFEN;

	if (statusMessageSequence > 999)
		statusMessageSequence = 0;

	char* buf = currentTextMessage;
	*buf++ = 'q';
	buf = printInteger(buf, statusMessageSequence++);

	*buf++ = ',';
	*buf++ = 'a';
	buf = printInteger(buf, (int) lastNonzero3DPosition.alt);

	*buf++ = ',';
	*buf++ = 'b';
	buf = print2DigitFloat(buf, vBattery);

	*buf++ = ',';
	*buf++ = 's';
	buf = print2DigitFloat(buf, vSolar);

	*buf++ = ',';
	*buf++ = 'o';
	buf = print2DigitFloat(buf, odometer.odometer_nm);

	*buf++ = ',';
	*buf++ = 't';
	buf = printInteger(buf, temperature);

	int32_t pllXtalDiff =
			getCalibratedPLLOscillatorFrequency() - DEFAULT_XTAL_FREQ;
	*buf++ = ',';
	*buf++ = 'x';
	buf = printInteger(buf, pllXtalDiff);

	*buf++ = ',';
	*buf++ = 'm';
	buf = printInteger(buf, MSITrim);

	*buf++ = ',';
	*buf++ = 'e';
	buf = printInteger(buf, numGPSChecksumErrors);

	*buf++ = ',';
	*buf++ = 'r';
	buf = printInteger(buf, RTC_readBackupRegister(RTC_BACKUP_REGISTER_NUM_RESETS_IDX));

	*buf++ = ',';
	*buf++ = 'w';
	buf = printInteger(buf, RTC_readBackupRegister(RTC_BACKUP_REGISTER_NUM_WWDG_RESETS_IDX));

	/*
	 *buf++ = ',';
	 *buf++ = 'c';
	 buf = printInteger(buf, course);

	 *buf++ = ',';
	 *buf++ = 's';
	 buf = printInteger(buf, speed_kts);
	 */
	/*
	 sprintf(temp, ",f%lu", txFrequency / 1000);
	 ax25_send_string(temp);

	 statusMessageValue('g', PHY_batteryAfterGPSVoltage(), temp);
	 ax25_send_string(temp);

	 statusMessageValue('h', PHY_batteryAfterHFVoltage(), temp);
	 ax25_send_string(temp);

	 const CalibrationRecord_t* cal = getCalibration(simpleTemperature);
	 int32_t freqError = cal->transmitterOscillatorFrequencyAtDefaultTrim - PLL_XTAL_DEFAULT_FREQUENCY;
	 sprintf(temp, ",o%ld", freqError);
	 ax25_send_string(temp);

	 char safe = (PWR_isSafeToUseDevice(E_DEVICE_GPS) ? 1 : 0)
	 + (PWR_isSafeToUseDevice(E_DEVICE_HF_TX) ? 2 : 0)
	 + (PWR_isSafeToUseDevice(E_DEVICE_VHF_TX) ? 4 : 0);
	 sprintf(temp, ",G%c,", '0' + safe);
	 ax25_send_string(temp);
	 */

	// Disable memory interface again.
	RCC->AHBENR = (RCC_AHBENR_MIFEN & ~RCC_AHBENR_MIFEN) | alreadyRunning;
	APRS_marshallTextMessage(txDelay);
}

// This does not reset at CPU reset.
static uint16_t telemetrySequence __attribute__((section (".noinit")));

void APRS_marshallPositionMessage(uint16_t txDelay) {
	// We offset temp. by 100 degrees so it is never negative.
	int _temperature = temperature + 100;
	if (_temperature < 0)
		_temperature = 0;

	uint16_t telemetryValues[] = {
			//(uint16_t) (vBattery * 1000.0f),
			(uint16_t) (vSolar * 1000.0f), _temperature, lastGPSFixTime,
			(int16_t) course, (int16_t) (speed_kts * 10) // conversion to 1/10 kts
			};

	aprs_send_header(&APRS_APSTM1_DEST, txDelay);
	char temp[40];                   // Temperature (int/ext)
	ax25_send_byte('!'); // Report w/o timestamp, no APRS messaging.
	uint8_t end = 0;

	// Location_t location = lastNonzeroPosition;
	// excludeZones(&location); no need any more.

	float alt = lastNonzeroPosition.alt;
	if (alt < 0)
		alt = 0;

	end = compressPosition(lastNonzeroPosition.lat, lastNonzeroPosition.lon,
			alt, temp);

	telemetrySequence++;
	if (telemetrySequence > 8280) {
		telemetrySequence = 0;
	}

	end += compressTelemetry(telemetrySequence, 5, telemetryValues, temp + end);

	temp[end] = 0;
	ax25_send_string(temp);
	ax25_end();
}

/* no longer used.
 void APRS_marshallStoredPositionMessage(LogRecord_t* record, uint16_t txDelay) {
 aprs_send_header(&APRS_APSTM1_DEST, txDelay);
 char temp[40];
 ax25_send_byte('/'); // Report w timestamp, no APRS messaging.
 uint8_t end = 0;

 Time_t uncompressedTime;
 uint8_t date = uncompressDateHoursMinutes(record, &uncompressedTime);
 end = compressedTimestamp(date, &uncompressedTime, temp);
 end += compressPosition(record->lat, record->lon, record->alt, temp + end);

 // int16_t speed = (int16_t)(record->speed * 10); // conversion to 1/10 kts

 uint16_t telemetryValues[] = { uncompressBatteryVoltageTomV(record),
 uncompressSolarVoltageTomV(record),
 / * (record->simpleTemperature +100)*10 * /0,	// Will require an offset of 100
 0,
 / *speed* /0 };

 end += compressTelemetry(telemetrySequence, 5, telemetryValues, temp + end);

 telemetrySequence++;
 if (telemetrySequence > 8280) {
 telemetrySequence = 0;
 }

 temp[end] = 0;
 ax25_send_string(temp);
 ax25_end();
 }
 */
static char* binaryMarshallWordByBase91(char* buf, uint32_t word) {
	for (size_t i = 0; i < 5; i++) {
		uint16_t work = word % 91;
		buf[i] = work + 33;
		word = word / 91;
	}
	return buf + 5;
}

char* APRS_marshallFlightLogMessage(const volatile LogRecord_t* message,
		char* buf) {
	// Status message
	//The status text occupies the rest of the Information field, and may be up to 62 characters long
	// (if there is no timestamp in the report) or 55 characters (if there is a timestamp).
	// The text may contain any printable ASCII characters except | or ~.
	// Approach: base-91, 6.5077946401987 bits per char.
	// Each message word fits into 5 chars (32/6.5077946401987 <= 5)
	size_t sizeInWords = (sizeof(LogRecord_t) + 3) / 4;
	uint32_t* unstructuredMessage = (uint32_t*) message;
	for (size_t j = 0; j < sizeInWords; j++) {
		buf = binaryMarshallWordByBase91(buf, unstructuredMessage[j]);
	}
	*buf = 0;
	return buf;
}

const APRSTransmission_t APRS_TRANSMISSIONS[] = { { .modulationMode = AFSK,
		.txDelay = 16, .initTransmitter = APRS_initDirectVHFTransmission,
		.shutdownTransmitter = APRS_endDirectTransmission }, { .modulationMode =
		GFSK, .txDelay = 10, .initTransmitter = APRS_initDirectHFTransmission,
		.shutdownTransmitter = APRS_endDirectTransmission } };

void APRS_makeDirectTransmissionFrequency(uint32_t frequency,
		uint32_t referenceFrequency, CDCE913_OutputMode_t output) {
	PLL_Setting_t pllSetting;
	double maxError = 20E-6; // 20 ppm tolerated, that is 3kHz off(!)
	double error = PLL_bestPLLSetting(referenceFrequency, frequency,
			&pllSetting);
	double absError = error < 0 ? -error : error;
	if (absError <= maxError) {
		// trace_printf("Using N=%d, M=%d, trim=%d\n", pllSetting.N, pllSetting.M, pllSetting.trim);
		setPLL(output, &pllSetting);
	} else {
		// trace_printf("Was not able to find a setting (weird)\n");
		LED_faultCode(LED_FAULT_NO_PLL_SETTING);
	}
}

static void APRS_initDirectHFTransmission(uint32_t frequency,
		uint32_t referenceFrequency) {
	// HF_enableDriver(HF_power());
	APRS_makeDirectTransmissionFrequency(frequency, referenceFrequency,
	HF_30m_HARDWARE_OUTPUT);
	// GFSK_init();
}

static void APRS_initDirectVHFTransmission(uint32_t frequency,
		uint32_t referenceFrequency) {
	APRS_makeDirectTransmissionFrequency(frequency, referenceFrequency,
	DIRECT_2m_HARDWARE_OUTPUT);
	AFSK_init();
}

void APRS_endDirectTransmission() {
	// HF_shutdownDriver();
	AFSK_shutdown();
	PLL_shutdown();
}

void APRS_transmitMessage(APRS_Band_t band, APRS_MessageType_t messageType,
		uint32_t frequency) {
	WWDG_pat();

	const APRSTransmission_t* mode = &APRS_TRANSMISSIONS[band];

	switch (messageType) {
	case COMPRESSED_POSITION_MESSAGE:
		APRS_marshallPositionMessage(mode->txDelay);
		break;
	case TELEMETRY_MESSAGE:
		APRS_marshallTelemetryMessage(mode->txDelay);
		break;
	case TEXT_MESSAGE:
		APRS_marshallTextMessage(mode->txDelay);
		break;
	}

	LED_on();

	// Prepare WFI
	packet_cnt = 0;
	int lastPacketCnt = 0;
	mode->initTransmitter(frequency, getCalibratedPLLOscillatorFrequency());

	while (packet_cnt != packet_size) {
		if (packet_cnt != lastPacketCnt) {
			WWDG_pat(); // if there is progress only.
			lastPacketCnt = packet_cnt;
		}
		// This fucked up modulation after deepsleep! __WFI();
	}

// We are now done transmitting.
	mode->shutdownTransmitter();

	LED_off();
}

