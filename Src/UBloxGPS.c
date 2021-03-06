// For ground test, see the DummyGPS source.
// #if MODE==FLIGHT

#include "GPS.h"
#include <math.h>
#include <string.h>
#include "Systime.h"
#include "GPS.h"
#include "LED.h"
#include "Globals.h"
#include "stm32l0xx.h"
#include "Watchdog.h"

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

static uint8_t state; // this should be inited.
uint16_t numGPSChecksumErrors;
static __attribute__((section (".noinit"))) uint8_t dataindex;
static __attribute__((section (".noinit"))) uint8_t commaindex;
#define GPS_INBUF_SIZE 64
static __attribute__((section (".noinit"))) volatile uint8_t inbuf[GPS_INBUF_SIZE];
volatile int16_t maxBufferLoad;

static volatile __attribute__((section (".noinit"))) uint16_t inbuf_in;
static volatile __attribute__((section (".noinit"))) uint16_t inbuf_out;

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

static volatile UBX_MESSAGE* currentSendingMessage;
static __attribute__((section (".noinit"))) int8_t currentSendingIndex;

static __attribute__((section (".noinit"))) uint8_t readClassId;
static __attribute__((section (".noinit"))) uint8_t readMessageId;
static __attribute__((section (".noinit"))) uint16_t readBodyLength;
static __attribute__((section (".noinit"))) uint16_t readBodyCnt;

static __attribute__((section (".noinit"))) uint8_t ackedClassId;
static __attribute__((section (".noinit"))) uint8_t ackedMessageId;
static __attribute__((section (".noinit"))) char id[5];

static uint32_t nextSendConfigurationTime;

extern void onNewGPSPosition();

boolean navSettingsConfirmed;

static void sendToGPS(uint8_t data) {
	USART2->TDR = data;
}

void GPS_transmit() {
	if (currentSendingIndex == -2) {
		sendToGPS(0xB5);
		currentSendingIndex++;
	} else if (currentSendingIndex == -1) {
		sendToGPS(0x62);
		currentSendingIndex++;
	} else if (currentSendingIndex < currentSendingMessage->length) {
		uint8_t data = currentSendingMessage->message[currentSendingIndex];
		currentSendingIndex++;
		if (currentSendingIndex == currentSendingMessage->length) {
			currentSendingMessage = NULL;
			USART2->CR1 &= ~(USART_CR1_TXEIE);
		}
		sendToGPS(data);
		// currentChecksumA += data;
		// currentChecksumB += currentChecksumA;
	} else {
		currentSendingMessage = NULL; // how did we end up here?
	}
}

static void beginSendUBXMessage(UBX_MESSAGE* message) {
	currentSendingMessage = message;
	currentSendingIndex = -2;

	// This should cause an immediate interrupt...
	USART2->CR1 |= USART_CR1_TXEIE;
}

void USART2_IRQHandler() {
	uint32_t stat = USART2->ISR;
	if (stat & USART_ISR_RXNE) {
		volatile uint16_t rxd = USART2->RDR; // make sure to read it even if never used.
		int16_t bufferLoad = inbuf_in - inbuf_out;
		if (bufferLoad < 0) bufferLoad += GPS_INBUF_SIZE;
		if (bufferLoad > maxBufferLoad)
			maxBufferLoad = bufferLoad;

		uint16_t next = (inbuf_in+1) & (GPS_INBUF_SIZE-1);
		if (next == inbuf_out) {
			// shit, buffer is full.
			// LED_faultCode(LED_FAULT_GPS_INBUF_OVERRUN);
		} else {
			inbuf[inbuf_in] = rxd;
			inbuf_in = next;
		}
	} if ((stat & USART_ISR_TXE) && (currentSendingMessage != NULL)) {
		GPS_transmit();
	}
}

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

static __attribute__((section (".noinit"))) double tempLat;
static __attribute__((section (".noinit"))) double tempLon;
static __attribute__((section (".noinit"))) float tempFloat;
static __attribute__((section (".noinit"))) float tempFloat2;
static __attribute__((section (".noinit"))) Time_t tempTime;
static __attribute__((section (".noinit"))) Date_t tempDate;
static __attribute__((section (".noinit"))) char tempChar;
static __attribute__((section (".noinit"))) uint8_t tempU8;
static __attribute__((section (".noinit"))) uint8_t tempU82;

static void parseGPVTG(char c) {
	static uint8_t state;
	if (c == ',') {
		state = 0;
		if (commaindex == 0) {
			tempFloat = 0;
			tempFloat2 = 0;
		} else if (commaindex == 5) {
			GPSCourseSpeed.course = tempFloat;
			GPSCourseSpeed.groundSpeed = tempFloat2 * 0.514444; // knots to m/s
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
			GPSDateTime.time = tempTime;
			GPSDateTime.time.valid = tempTime.valid;
			GPSPosition.lat = tempLat;
			GPSPosition.lon = tempLon;
			GPSStatus.fixMode = tempU8;
			GPSStatus.numberOfSatellites = tempU82;
			GPSStatus.horizontalAccuracy = tempFloat;
			GPSPosition.alt = tempFloat2;
			onNewGPSPosition();
			// getting a GPS position, whether valid or not, should satisfy the watchdog.
			// WWDG_pat();
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
			GPSPosition.fixTime = tempTime;
			GPSPosition.fixTime.valid = tempTime.valid;
			GPSPosition.valid = tempChar;
			GPSPosition.lat = tempLat;
			GPSPosition.lon = tempLon;
			GPSCourseSpeed.groundSpeed = tempFloat * 0.514444; // knots to m/s
			GPSCourseSpeed.course = tempFloat2;
			GPSDateTime.date = tempDate;
			GPSDateTime.date.valid = tempDate.valid;
			onNewGPSPosition();
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
			GPSPosition.lat = tempLat;
			GPSPosition.lon = tempLon;
			GPSPosition.fixTime = tempTime;
			GPSPosition.valid = tempChar;
			GPSDateTime.time = tempTime;
			GPSDateTime.time.valid = tempTime.valid;
			onNewGPSPosition();
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

uint8_t nmea_parse(volatile char c) {
	static char sentence;
	static uint8_t checksum;

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
				&& systime >= nextSendConfigurationTime
				) {
			ackedClassId = 0;
			ackedMessageId = 0; // clear any prior acknowledge.
			beginSendUBXMessage(&INIT_NAV_MESSAGE);
			nextSendConfigurationTime = systime + GPS_CONF_RESEND_INTERVAL;
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
			numGPSChecksumErrors++;
		} else {
			WWDG_pat();
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
		readClassId = c; // should be 06 for echo of command, or 05 for the ack-ack
		state = STATE_READ_BINARY_UBX_MSG_ID;
		break;
	case STATE_READ_BINARY_UBX_MSG_ID:
		readMessageId = c; // should be 24 or 01 for ack-ack
		state = STATE_READ_BINARY_UBX_MSG_LEN1;
		break;
	case STATE_READ_BINARY_UBX_MSG_LEN1:
		readBodyLength = c; // 02
		state = STATE_READ_BINARY_UBX_MSG_LEN2;
		break;
	case STATE_READ_BINARY_UBX_MSG_LEN2:
		readBodyLength += (c << 8); // 0
#ifdef TRACE_GPS
		b5 62
		05 01
		02 00
		06 24
		32 5b
		24 47
		50
		00001a0 52 4d 43 2c 2c 56 2c 2c 2c 2c 2c 2c 2c 2c 2c 2c
		trace_printf("UBX response is class %d msg %d length %d\n", readClassId, readMessageId, readBodyLength);
#endif
		readBodyCnt = 0;
		state = STATE_READ_BINARY_UBX_MSG_BODY;
		break;
	case STATE_READ_BINARY_UBX_MSG_BODY:
		if (readBodyCnt == 0 && readClassId == 5 && readMessageId == 1) {
			ackedClassId = c; // 06
		} else if (readBodyCnt == 1 && readClassId == 5 && readMessageId == 1) {
			ackedMessageId = c; // 24
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

void GPS_driver() {
	while(inbuf_out != inbuf_in) {
		uint8_t data = inbuf[inbuf_out];
		inbuf_out = (inbuf_out+1) & (GPS_INBUF_SIZE-1);
		nmea_parse(data);
	}
	__WFI(); // wait for more data to arrive.
}

void GPS_start() {
		// Reset the message sending
	nextSendConfigurationTime = systime;
	currentSendingIndex = 0;
	currentSendingMessage = NULL;
	navSettingsConfirmed = false;
	inbuf_in = 0; inbuf_out = 0;

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

#ifdef USE_MSI_WHILE_GPS
	calibrateMSI(MSI_WHILE_GPS_SPEED);
	uint32_t brr = MSI_WHILE_GPS_SPEED / (9600 * 2);
#else
	uint32_t brr = 16E6 / 9600;
#endif
	USART2->BRR = brr;

	NVIC_SetPriority(USART2_IRQn, 1);
	NVIC_EnableIRQ(USART2_IRQn);

	// Don't care about overrun detect (what can we do anyway)
	USART2->CR3 |= USART_CR3_OVRDIS;
	// Enable transmitter, and the usart itself.
	USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;

	state = STATE_IDLE;
}

void GPS_stopListening() {
	USART2->CR1 &= USART_CR1_UE;
}

void GPS_powerOff() {
	GPIOA->BSRR = 1<<7; // Set PA7
}

boolean GPS_isGPSRunning() {
	uint32_t result = GPIOA->ODR & (1<<7);
	return !result;
}

void GPS_invalidateDateTime() {
	GPSDateTime.time.valid = false;
	GPSDateTime.date.valid = false;
	GPSDateTime.time.hours = 0;
	GPSDateTime.time.minutes = 0;
	GPSDateTime.time.seconds = 0;
}

boolean GPS_isDateTimeValid() {
	boolean result = GPSDateTime.time.valid
			&& GPSDateTime.date.valid;
	// trace_printf("GPS date time valid: %d\n", result);
	return result;
}

void GPS_invalidatePosition() {
	GPSPosition.valid = 'V';
}

boolean GPS_isPositionValid() {
	return GPSPosition.valid == 'A';
}

void GPS_invalidateNumSatellites() {
	GPSStatus.numberOfSatellites = 0;
}

uint8_t GPS_numberOfSatellites() {
	return GPSStatus.numberOfSatellites;
}

//#endif
