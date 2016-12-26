// For ground test, see the DummyGPS source.
// #if MODE==FLIGHT

#include "GPS.h"
#include <math.h>
#include <string.h>
#include "Systime.h"
#include "GPS.h"
#include "Globals.h"
#include "stm32l0xx.h"

enum {
	STATE_IDLE,
	STATE_READ_ID,
	STATE_DATA,
	STATE_CHECKSUM1,
	STATE_CHECKSUM2,
	STATE_READ_BINARY_UBX_SYNC_2,
	STATE_READ_BINARY_UBX_CLASS_ID,
	STATE_READ_BINARY_UBX_MSG_ID,
	STATE_READ_BINARY_UBX_MSG_LEN1,
	STATE_READ_BINARY_UBX_MSG_LEN2,
	STATE_READ_BINARY_UBX_MSG_BODY,
	STATE_READ_BINARY_UBX_MSG_CHECKA,
	STATE_READ_BINARY_UBX_MSG_CHECKB,
} NMEA_PARSER_STATES;

enum {
	GPVTG,	// Track made good and ground speed
	GPGGA,	// Global Positioning System Fix Data
	GPRMC,	// Recommended minimum specific GPS/Transit data
	GPGSA,	// GPS DOP and active satellites
	GPGSV,	// Detailed Satellite data
	GPGLL	// Lat/Lon data
} messagesParsed;

#define GPS_CONF_RESEND_INTERVAL 5000

static uint8_t dataindex;
static uint8_t commaindex;
static uint8_t state;
static uint8_t commitCheck;

// static uint8_t _UBX_POLL_NAV5_MESSAGE[] = UBX_POLL_NAV5_MESSAGE;
static uint8_t _UBX_INIT_NAV5_MESSAGE[] = UBX_INIT_NAV5_MESSAGE;

//static UBX_MESSAGE POLL_NAV_MESSAGE = { sizeof(_UBX_POLL_NAV5_MESSAGE),
//		_UBX_POLL_NAV5_MESSAGE };

static UBX_MESSAGE INIT_NAV_MESSAGE = { sizeof(_UBX_INIT_NAV5_MESSAGE),
		_UBX_INIT_NAV5_MESSAGE };

// Unsafes are always valid (no parser construction site) but are written to from within
// interrupt handler.
// Interested consumers should disable interrupts, copy the unsafes into a their locally
// managed safe copies and re-enable interrupts.
volatile DateTime_t nmeaTimeInfo_unsafe;
volatile NMEA_CRS_SPD_Info_t nmeaCRSSPDInfo_unsafe;
volatile Location_t nmeaPositionInfo_unsafe;
volatile NMEA_StatusInfo_t nmeaStatusInfo_unsafe;

uint8_t nmea_parse(char c);

static UBX_MESSAGE* currentSendingMessage;
static int8_t currentSendingIndex;
static uint8_t currentChecksumA;
static uint8_t currentChecksumB;

static uint8_t readClassId;
static uint8_t readMessageId;
static uint16_t readBodyLength;
static uint16_t readBodyCnt;

static uint8_t ackedClassId;
static uint8_t ackedMessageId;
static uint32_t lastSendConfigurationTime;
static boolean navSettingsConfirmed;

static void sendToGPS(uint8_t data) {
	USART2->TDR = data;
}

void GPS_transmit() {
	if (currentSendingMessage == NULL) {
		USART2->CR1 &= ~USART_CR1_TXEIE;
		return;
	}

	if (currentSendingIndex == -2) {
		sendToGPS(0xB5);
		currentSendingIndex++;
	} else if (currentSendingIndex == -1) {
		sendToGPS(0x62);
		currentSendingIndex++;
	} else if (currentSendingIndex < currentSendingMessage->length) {
		uint8_t data = currentSendingMessage->message[currentSendingIndex];
		sendToGPS(data);
		currentChecksumA += data;
		currentChecksumB += currentChecksumA;
		currentSendingIndex++;
	} else if (currentSendingIndex == currentSendingMessage->length) {
		sendToGPS(currentChecksumA);
		currentSendingIndex++;
	} else if (currentSendingIndex == currentSendingMessage->length + 1) {
		sendToGPS(currentChecksumB);
		currentSendingMessage = NULL;
	}
}

static void beginSendUBXMessage(UBX_MESSAGE* message) {
	currentSendingMessage = message;
	currentSendingIndex = -2;
	currentChecksumA = 0;
	currentChecksumB = 0;
	// __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_TXE);
	// USART2->CR1 |= USART_CR1_TXEIE;
	// should not be needed to do: 		GPS_transmit();
}

void USART2_IRQHandler() {
	uint32_t stat = USART2->ISR;
	if (stat & USART_ISR_RXNE) {
		uint16_t rxd = USART2->RDR;
		nmea_parse(rxd);
	}
}

/*
 void printNMEA_TimeInfo() {
 uint32_t itod = nmeaTimeInfo_unsafe.itod;
 int millis = itod % 1000;
 itod = itod / 1000;
 int seconds = itod % 60;
 itod = itod / 60;
 int minutes = itod % 60;
 itod = itod / 60;
 int hours = itod % 60;

 trace_printf("NMEA_TimeInfo_t\r\n");
 trace_printf("itod: %u (%0d:%0d:%0d.%000d)\r\n", nmeaTimeInfo_unsafe.itod,
 hours, minutes, seconds, millis);
 }

 void printNMEA_CRS_SPD_Info() {
 printf("NMEA_CRS_SPD_Info\r\n");
 printf("crs: %f spd %f\r\n", nmeaCRSSPDInfo_unsafe.course, nmeaCRSSPDInfo_unsafe.groundSpeed);
 }

 void printNMEA_PositionInfo() {
 uint32_t itod = nmeaPositionInfo_unsafe.fixTimeUTC;
 int millis = itod % 1000;
 itod = itod / 1000;
 int seconds = itod % 60;
 itod = itod / 60;
 int minutes = itod % 60;
 itod = itod / 60;
 int hours = itod % 60;
 printf("FixTime: %u (%0d:%0d:%0d.%000d) ", nmeaPositionInfo_unsafe.fixTimeUTC, hours, minutes, seconds, millis);
 printf("lat %f, lon %f, alt %f, valid %c\r\n", nmeaPositionInfo_unsafe.lat,nmeaPositionInfo_unsafe.lon,nmeaPositionInfo_unsafe.alt, nmeaPositionInfo_unsafe.validity);
 }

 void printNMEA_StatusInfo() {
 printf("NMEA_StatusInfo\r\n");
 printf("FixMode %d, NumSat %d, horizAcc %f\r\n", nmeaStatusInfo_unsafe.fixMode, nmeaStatusInfo_unsafe.numberOfSatellites, nmeaStatusInfo_unsafe.horizontalAccuracy);
 }
 */
MessageState latestGPSState = CONSUMED;

// Parse one character of a NMEA time.
// apparently writes into a millis value at end.
static void parseTime(char c, uint8_t* state, Time_t* value) {
	switch (++(*state)) {
	case 1:
		value->hours = (c - '0') * 10;
		break;
	case 2:
		value->hours += (c - '0');
		break;
	case 3:
		value->minutes = (c - '0') * 10;
		break;
	case 4:
		value->minutes += (c - '0');
		break;
	case 5:
		value->seconds = (c - '0') * 10;
		break;
	case 6:
		value->seconds += (c - '0');
		value->valid = true;
		break;
	default:
		break;
	}

	// trace_printf("at state %d parsed %02d:%02d:%02d\n", *state, value->hours, value->minutes, value->seconds);
}

static void parseDegrees(char c, uint8_t* state, double* value) {
	static uint32_t ivalue;
	uint8_t i;
	double temp;
	switch (++(*state)) {
	case 1:
		ivalue = (c - '0') * 1000;
		break;
	case 2:
		ivalue += (c - '0') * 100;
		break;
	case 3:
		ivalue += (c - '0') * 10;
		break;
	case 4:
		ivalue += (c - '0') * 1;
		break;
	case 5: {
		if (c == '.') {
			*state = 9;
		} else {
			ivalue *= 10;
			ivalue += (c - '0') * 1;
		}
		break;
	}
	case 6: {
		*state = 9;
		break;
	}
	case 10:
		temp = ivalue % 100;
		*value = ivalue / 100 + temp / 60.0;
		// no break;
	default:
		temp = (c - '0') / 60.0;
		for (i = *state - 9; i > 0; i--) {
			temp /= 10;
		}
		*value += temp;
		break;
	}
}

static void parseFloat(char c, uint8_t *state, float* value) {
	uint8_t i;
	if (c == '.') {
		*state = 1;
	} else if (!*state) {
		*value *= 10;
		*value += (c - '0');
	} else {
		float digit = (c - '0');
		for (i = *state; i > 0; i--) {
			digit /= 10;
		}
		*value += digit;
		(*state)++;
	}
}

static void parseDate(char c, uint8_t* state, Date_t* value) {
	switch (++(*state)) {
	case 1:
		value->date = (c - '0') * 10;
		break;
	case 2:
		value->date += (c - '0');
		break;
	case 3:
		value->month = (c - '0') * 10;
		break;
	case 4:
		value->month += (c - '0');
		break;
	case 5:
		value->year100 = (c - '0') * 10;
		break;
	case 6:
		value->year100 += (c - '0');
		value->valid = true;
		break;
	default:
		break;
	}
}

static void parseInt8(char c, uint8_t* value) {
	*value *= 10;
	uint8_t digit = (c - '0');
	*value += digit;
}

static double tempLat;
static double tempLon;
static float tempFloat;
static float tempFloat2;
static Time_t tempTime;
static Date_t tempDate;
static char tempChar;
static uint8_t tempU8;
static uint8_t tempU82;

static void parseGPVTG(char c) {
	static uint8_t state;
	if (c == ',') {
		state = 0;
		if (commaindex == 0) {
			tempFloat = 0;
			tempFloat2 = 0;
		} else if (commaindex == 5) {
			nmeaCRSSPDInfo_unsafe.course = tempFloat;
			nmeaCRSSPDInfo_unsafe.groundSpeed = tempFloat2 * 0.514444; // knots to m/s
			commitCheck |= 1;
		}
		commaindex++;
	} else {
		switch (commaindex) {
		case 1:
			parseFloat(c, &state, &tempFloat);
			break;
		case 5:
			parseFloat(c, &state, &tempFloat2);
			break;
		default:
			break;
		}
	}
}

void parseGPGGA(char c) {
	static uint8_t state;
	// trace_printf("GGA: commaindex %d, state %d\n", commaindex, state);
	if (c == ',') {
		state = 0;
		if (commaindex == 0) {
			tempLat = 0;
			tempLon = 0;
			tempU8 = 0;
			tempU82 = 0;
			tempFloat = 0;
			tempFloat2 = 0;
			tempTime.valid = false;
		} else if (commaindex == 9) {
			// debugTime("GGA", &tempTime);
			nmeaTimeInfo_unsafe.time = tempTime;
			nmeaTimeInfo_unsafe.time.valid = tempTime.valid;
			nmeaPositionInfo_unsafe.lat = tempLat;
			nmeaPositionInfo_unsafe.lon = tempLon;
			nmeaStatusInfo_unsafe.fixMode = tempU8;
			nmeaStatusInfo_unsafe.numberOfSatellites = tempU82;
			nmeaStatusInfo_unsafe.horizontalAccuracy = tempFloat;
			nmeaPositionInfo_unsafe.alt = tempFloat2;
			commitCheck |= 2;
		}
		commaindex++;
	} else {
		switch (commaindex) {
		case 1:
			parseTime(c, &state, &tempTime);
			break;
		case 2:
			parseDegrees(c, &state, &tempLat);
			break;
		case 3:
			if (c == 'S')
				tempLat = -tempLat;
			break;
		case 4:
			parseDegrees(c, &state, &tempLon);
			break;
		case 5:
			if (c == 'W')
				tempLon = -tempLon;
			break;
		case 6:
			parseInt8(c, &tempU8);
			break;
		case 7:
			parseInt8(c, &tempU82);
			break;
		case 8:
			parseFloat(c, &state, &tempFloat);
			break;
		case 9:
			parseFloat(c, &state, &tempFloat2);
			break;
		default:
			break;
		}
	}
}

void parseGPRMC(char c) {
	static uint8_t state;
	if (c == ',') {
		state = 0;
		if (commaindex == 0) {
			tempFloat = 0;
			tempFloat2 = 0;
			tempLat = 0;
			tempLon = 0;
			tempDate.valid = false;
			tempTime.valid = false;
		} else if (commaindex == 9) {
			// debugTime("RMC", &tempTime);
			nmeaPositionInfo_unsafe.fixTime = tempTime;
			nmeaPositionInfo_unsafe.fixTime.valid = tempTime.valid;
			nmeaPositionInfo_unsafe.valid = tempChar;
			nmeaPositionInfo_unsafe.lat = tempLat;
			nmeaPositionInfo_unsafe.lon = tempLon;
			nmeaCRSSPDInfo_unsafe.groundSpeed = tempFloat * 0.514444; // knots to m/s
			nmeaCRSSPDInfo_unsafe.course = tempFloat2;
			nmeaTimeInfo_unsafe.date = tempDate;
			nmeaTimeInfo_unsafe.date.valid = tempDate.valid;
			commitCheck |= 4;
		}
		commaindex++;
	} else {
		switch (commaindex) {
		case 1:
			parseTime(c, &state, &tempTime);
			break;
		case 2:
			tempChar = c;
			break;
		case 3:
			parseDegrees(c, &state, &tempLat);
			break;
		case 4:
			if (c == 'S')
				tempLat = -tempLat;
			break;
		case 5:
			parseDegrees(c, &state, &tempLon);
			break;
		case 6:
			if (c == 'W')
				tempLon = -tempLon;
			break;
		case 7:
			parseFloat(c, &state, &tempFloat);
			break;
		case 8:
			parseFloat(c, &state, &tempFloat2);
			break;
		case 9:
			parseDate(c, &state, &tempDate);
			break;
		default:
			break;
		}
	}
}

void parseGPGSA(char c) {
// Ignore. Not important.
	commitCheck |= 16;
}

void parseGPGSV(char c) {
// Ignore GSV
	if (c == ',') {
		commaindex++;
#ifdef TRACE_GPS_SNR
		if commaindex >= 5 && (commaindex-5) % 4 == 0) {
			// between SV number and elevation data
			trace_putchar(':');
		}
		if (DEBUG_GPS_SNR && commaindex >= 4 && commaindex % 4 == 0) {
			// Before SV number
			trace_putchar('\n');
		}
#endif
	} else {
		switch (commaindex) {
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
		case 12:
		case 16:
		case 20:
			// PRN number;
#ifdef TRACE_GPS_SNR
			trace_putchar(c);
#endif
			break;
		case 5:
		case 13:
		case 17:
		case 21:
			// elevation
			break;
		case 6:
		case 14:
		case 18:
		case 22:
			// azimuth
			break;
		case 7:
		case 15:
		case 19:
		case 23:
			// SNR
#ifdef TRACE_GPS_SNR
			trace_putchar(c);
#endif
			break;
		}
	}
	commitCheck |= 32;
}

void parseGPGLL(char c) {
	static uint8_t state;
	if (c == ',') {
		state = 0;
		if (commaindex++ == 0) {
			tempLat = 0;
			tempLon = 0;
			tempTime.valid = false;
		}
	} else {
		switch (commaindex) {
		case 1:
			parseDegrees(c, &state, &tempLat);
			break;
		case 2:
			if (c == 'S')
				tempLat = -tempLat;
			break;
		case 3:
			parseDegrees(c, &state, &tempLon);
			break;
		case 4:
			if (c == 'W')
				tempLon = -tempLon;
			break;
		case 5:
			parseTime(c, &state, &tempTime);
			break;
		case 6:
			tempChar = c;
			break;
		case 7:
			//debugTime("GLL", &tempTime);
			nmeaPositionInfo_unsafe.lat = tempLat;
			nmeaPositionInfo_unsafe.lon = tempLon;
			nmeaPositionInfo_unsafe.fixTime = tempTime;
			nmeaPositionInfo_unsafe.valid = tempChar;
			nmeaTimeInfo_unsafe.time = tempTime;
			nmeaTimeInfo_unsafe.time.valid = tempTime.valid;
			commitCheck |= 8;
			break;
		default:
			break;
		}
	}
}

uint8_t char2hexdigit(uint8_t c) {
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	return c - '0';
}

static char id[5];
uint8_t nmea_parse(char c) {
	static char sentence;
	static uint8_t checksum;

	// trace_printf("GPS state %d, in %d\n", state, c);
	switch (state) {
	case STATE_IDLE:
		if (c == '$') {
			state = STATE_READ_ID;
			dataindex = 0;
			checksum = 0;
		} else if (c == 0xb5) {
			state = STATE_READ_BINARY_UBX_SYNC_2;
#ifdef TRACE_GPS
			trace_printf("UBX sync2 received\n");
#endif
		}

		// trigger sending a message, until confirmed.
		if (!navSettingsConfirmed && currentSendingMessage == NULL
				&& systime >= lastSendConfigurationTime + GPS_CONF_RESEND_INTERVAL) {
			ackedClassId = 0;
			ackedMessageId = 0; // clear any prior acknowledge.
			beginSendUBXMessage(&INIT_NAV_MESSAGE);
			lastSendConfigurationTime = systime;
#ifdef TRACE_GPS
			trace_printf("Sending GPS conf. message\n");
#endif
		}
		break;
	case STATE_READ_ID:
		id[dataindex++] = c;
		checksum ^= c;
		if (dataindex == 5) {
			dataindex = 0;
			commaindex = 0;
			state = STATE_DATA;
			if (strncmp("GPVTG", id, 5) == 0) {
				sentence = GPVTG;
				//trace_printf("GPVTG\n");
			} else if (strncmp("GPGGA", id, 5) == 0) {
				sentence = GPGGA;
				//trace_printf("GPGGA\n");
			} else if (strncmp("GPRMC", id, 5) == 0) {
				sentence = GPRMC;
				//trace_printf("GPRMC\n");
			} else if (strncmp("GPGSA", id, 5) == 0) {
				sentence = GPGSA;
				//trace_printf("GPGSA\n");
			} else if (strncmp("GPGSV", id, 5) == 0) {
				sentence = GPGSV;
				//trace_printf("GPGSV\n");
			} else if (strncmp("GPGLL", id, 5) == 0) {
				sentence = GPGLL;
				//trace_printf("GPGLL\n");
			} else {
				// printf("Got an unknown message %s\r\n", id);
				state = STATE_IDLE; // We are not listening to other than these messages.
			}
		}
		break;
	case STATE_DATA:
		if (c == '*') {
			state = STATE_CHECKSUM1;
		} else {
			checksum ^= c;
			switch (sentence) {
			case GPVTG:
				parseGPVTG(c); // looks ok
				break;
			case GPGGA:
				parseGPGGA(c);
				break;
			case GPRMC:
				parseGPRMC(c); // looks ok
				break;
			case GPGSA:
				parseGPGSA(c);
				break;
			case GPGSV:
				parseGPGSV(c);
				break;
			case GPGLL:
				parseGPGLL(c);
				break;
			}
			dataindex++;
		}
		break;
	case STATE_CHECKSUM1:
		checksum -= char2hexdigit(c) << 4;
		state = STATE_CHECKSUM2;
		break;
	case STATE_CHECKSUM2:
		checksum -= char2hexdigit(c);
		state = STATE_IDLE;
		if (checksum) {
			latestGPSState = INVALID;
			// trace_printf("*** Bad GPS checksum!\n");
		} else {
			latestGPSState = NEWDATA;
			// trace_printf("parse check %d\n", commitCheck);
			return 1;
		}
		break;
	case STATE_READ_BINARY_UBX_SYNC_2:
		if (c == 0x62)
			state = STATE_READ_BINARY_UBX_CLASS_ID;
		else
			state = STATE_IDLE;
		break;
	case STATE_READ_BINARY_UBX_CLASS_ID:
		readClassId = c;
		state = STATE_READ_BINARY_UBX_MSG_ID;
		break;
	case STATE_READ_BINARY_UBX_MSG_ID:
		readMessageId = c;
		state = STATE_READ_BINARY_UBX_MSG_LEN1;
		break;
	case STATE_READ_BINARY_UBX_MSG_LEN1:
		readBodyLength = c;
		state = STATE_READ_BINARY_UBX_MSG_LEN2;
		break;
	case STATE_READ_BINARY_UBX_MSG_LEN2:
		readBodyLength += (c << 8);
#ifdef TRACE_GPS
		trace_printf("UBX response is class %d msg %d length %d\n", readClassId, readMessageId, readBodyLength);
#endif
		readBodyCnt = 0;
		state = STATE_READ_BINARY_UBX_MSG_BODY;
		break;
	case STATE_READ_BINARY_UBX_MSG_BODY:
		if (readBodyCnt == 0 && readClassId == 5 && readMessageId == 1) {
			ackedClassId = c;
		} else if (readBodyCnt == 1 && readClassId == 5 && readMessageId == 1) {
			ackedMessageId = c;
		}
		readBodyCnt++;
		if (readBodyCnt == readBodyLength)
			state = STATE_READ_BINARY_UBX_MSG_CHECKA;
		break;
	case STATE_READ_BINARY_UBX_MSG_CHECKA:
		// don't bother to check.
		state = STATE_READ_BINARY_UBX_MSG_CHECKB;
		break;
	case STATE_READ_BINARY_UBX_MSG_CHECKB:
		if (ackedClassId == 6 && ackedMessageId == 0x24) {
#ifdef TRACE_GPS
			trace_printf("NAV5 Settings confirmed.\n");
#endif
			navSettingsConfirmed = true;
		}
		state = STATE_IDLE;
		break;
	}
	return 0;
}

extern void onNewGPSData();

void GPS_getData() {
	NVIC_DisableIRQ(USART2_IRQn);
	__DSB();
	__ISB();

	// These are committed by the IRQ handler once messages are complete.
	GPSDateTime = nmeaTimeInfo_unsafe;
	GPSCourseSpeed = nmeaCRSSPDInfo_unsafe;
	GPSPosition = nmeaPositionInfo_unsafe;
	GPSStatus = nmeaStatusInfo_unsafe;

	onNewGPSData();
	NVIC_EnableIRQ(USART2_IRQn);
}

void GPS_start() {
	// Reset the message sending
	lastSendConfigurationTime = systime - GPS_CONF_RESEND_INTERVAL;
	currentSendingIndex = 0;
	currentSendingMessage = NULL;
	navSettingsConfirmed = false;

	GPIOA->BRR = 1<<7; // PA7 low

	// USART clock
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);

	// and GPIO clock too
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);

	// Set to alternate function
	GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 2))) | (2 << (2 * 2));
	GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 3))) | (2 << (2 * 3));

	// USART2 TX is AF4
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(15 << (4 * 2))) | (4 << (4 * 2));
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(15 << (4 * 3))) | (4 << (4 * 3));

	uint32_t brr = 16E6 / 9600;
	USART2->BRR = brr;

	NVIC_SetPriority(USART2_IRQn, 1);
	NVIC_EnableIRQ(USART2_IRQn);

	// Don't care about overrun detect (what can we do anyway)
	USART2->CR3 |= USART_CR3_OVRDIS;
	// Enable transmitter, and the usart itself.
	USART2->CR1 = USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;

	state = STATE_IDLE;
}

void GPS_stopListening() {
	USART2->CR1 &= USART_CR1_UE;
	latestGPSState = CONSUMED;
}

void GPS_powerOff() {
	GPIOA->BSRR = 1<<7; // Set PA7
}

boolean GPS_isGPSRunning() {
	uint32_t result = GPIOA->ODR & (1<<7);
	return !result;
}

void GPS_invalidateDateTime() {
	nmeaTimeInfo_unsafe.time.valid = false;
	nmeaTimeInfo_unsafe.date.valid = false;
	nmeaTimeInfo_unsafe.time.hours = 0;
	nmeaTimeInfo_unsafe.time.minutes = 0;
	nmeaTimeInfo_unsafe.time.seconds = 0;
}

boolean GPS_isDateTimeValid() {
	boolean result = nmeaTimeInfo_unsafe.time.valid
			&& nmeaTimeInfo_unsafe.date.valid;
	// trace_printf("GPS date time valid: %d\n", result);
	return result;
}

void GPS_invalidatePosition() {
	nmeaPositionInfo_unsafe.valid = 'V';
}

boolean GPS_isPositionValid() {
	return nmeaPositionInfo_unsafe.valid == 'A';
}

void GPS_invalidateNumSatellites() {
	nmeaStatusInfo_unsafe.numberOfSatellites = 0;
}

uint8_t GPS_numberOfSatellites() {
	return nmeaStatusInfo_unsafe.numberOfSatellites;
}

//#endif
