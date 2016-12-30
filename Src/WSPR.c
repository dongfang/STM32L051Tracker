#include <string.h>
#include <math.h>
#include "stm32l0xx.h"
#include "WSPR.h"
#include "GPS.h"
#include "PLL.h"
#include "Globals.h"
#include "Types.h"
#include "Setup.h"
#include "Power.h"

static uint8_t losslessCompressedBuf[11];
uint8_t convolutionalBuf[21]  __attribute__((section (".noinit")));
uint8_t interleavedbuf[21]  __attribute__((section (".noinit")));
uint8_t symbolList[162 / 4 + 1]  __attribute__((section (".noinit")));
uint8_t nextWSPRMessageTypeIndex  __attribute__((section (".noinit")));

const uint32_t WSPR_FREQUENCIES[] = {10140200, 28126100};

const uint8_t WSPR_SCHEDULE[] = WSPR_DEFAULT_SCHEDULE;
const uint8_t WSPR_SCHEDULE_LENGTH = sizeof(WSPR_SCHEDULE)/sizeof(WSPR_MESSAGE_TYPE);

const uint8_t WSPR_LOWALT_SCHEDULE[] = WSPR_LOWALT_SCHEDULE_DEF;
const uint8_t WSPR_LOWALT_SCHEDULE_LENGTH = sizeof(WSPR_LOWALT_SCHEDULE)/sizeof(WSPR_MESSAGE_TYPE);

extern uint16_t hash(const char* call, uint16_t length);

// Must be length 6. Must have a number at 3rd position.
// Must have no numbers at last 3 positions.
// Valid symbols are [A..Z] and [0..9] and space.

static inline uint8_t encode(char c);
static uint8_t encode(char c) {
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'Z')
		return 10 + c - 'A';
	if (c >= 'a' && c <= 'z')
		return 10 + c - 'a';
	if (c == ' ')
		return 36;
	// trace_printf("Error! Unhandled char in callsign '%c'\n", c);
	return -1;
}

static inline uint8_t encodeCharOnly(char c);
static uint8_t encodeCharOnly(char c) {
	if (c >= 'A' && c <= 'Z')
		return c - 'A';
	if (c >= 'a' && c <= 'z')
		return c - 'a';
	if (c == ' ')
		return 26;
	// trace_printf("Error! Unhandled char in callsign '%c'\n", c);
	return -1;
}

static uint32_t encodeCallsign(const char* callsign) {
	uint32_t result = encode(callsign[0]);
	result = result * 36 + encode(callsign[1]);
	result = result * 10 + encode(callsign[2]);
	result = result * 27 + encodeCharOnly(callsign[3]);
	result = result * 27 + encodeCharOnly(callsign[4]);
	result = result * 27 + encodeCharOnly(callsign[5]);
	return result;
}

static uint32_t encode4DigitMaidenhead(const char* mh) {
	uint32_t result = 10 * (mh[0] - 'A') + (mh[2] - '0');
	result = 179 - result;
	result *= 180;
	result += 10 * (mh[1] - 'A') + (mh[3] - '0');
	return result;
}

static uint8_t encodePower(uint8_t power) {
	return power + 64;
}

static void encodeType1Message(const char* maidenhead4, uint8_t power) {
	char myPaddedCall[7];
	strcpy(myPaddedCall, WSPR_CALL);
	for (uint8_t i = strlen(WSPR_CALL); i < 6; i++)
		myPaddedCall[i] = ' ';

	uint32_t N = encodeCallsign(myPaddedCall) << 4;
	uint32_t maiden = encode4DigitMaidenhead(maidenhead4);
	uint32_t M = (maiden * 128 + encodePower(power)) << (32 - 22 - 4);

	uint8_t* Nsplit = (uint8_t*) &N;
	uint8_t* Msplit = (uint8_t*) &M;

	losslessCompressedBuf[0] = (Nsplit[3]);
	losslessCompressedBuf[1] = (Nsplit[2]);
	losslessCompressedBuf[2] = (Nsplit[1]);
	losslessCompressedBuf[3] = (Nsplit[0]);

	losslessCompressedBuf[3] |= Msplit[3];  // 4 used  and 0 free in buffer
	losslessCompressedBuf[4] = Msplit[2];   // 12 used and 0 free in buffer
	losslessCompressedBuf[5] = Msplit[1];   // 20 used and 0 free in buffer
	losslessCompressedBuf[6] = Msplit[0];   // 22 used and 6 free in buffer
	losslessCompressedBuf[7] = 0;   // 14 free in buffer
	losslessCompressedBuf[8] = 0;   // 22 free in buffer
	losslessCompressedBuf[9] = 0;   // 30 free in buffer
	losslessCompressedBuf[10] = 0;  // 31 free in buffer
}

static const int8_t powerSomething[] = { 0, -1, 1, 0, -1, 2, 1, 0, -1, 1 };

static void encodeType3Message(const char* maidenhead6, int8_t power) {

  	char rotatedMaidenhead[7];
	for (uint8_t i = 1; i < 6; i++)
		rotatedMaidenhead[i - 1] = maidenhead6[i];
	rotatedMaidenhead[5] = maidenhead6[0];
	rotatedMaidenhead[6] = 0;

	uint32_t n1 = encodeCallsign(rotatedMaidenhead);
	n1 = n1<<4;

	uint32_t n2 = hash(WSPR_CALL, strlen(WSPR_CALL));

	//trace_printf("Power initially %d\n",power);
	if (power > 60)
		power = 60;

	power += powerSomething[power % 10];
	power = -(power + 1);

	// This is in arithmetic operations, not bitwise!
	n2 = n2 * 128 + power + 64;
	// trace_printf("n2 aka shifted callsign plus power is %u\n", n2);

	// n2 is in bits 0..21. We move that to 6..27
	n2 = n2 << (32 - 22 - 4);

	uint8_t* n1split = (uint8_t*) &n1;
        uint8_t* n2split = (uint8_t*) &n2;

	losslessCompressedBuf[0] = (n1split[3]);
        losslessCompressedBuf[1] = (n1split[2]);
        losslessCompressedBuf[2] = (n1split[1]);
        losslessCompressedBuf[3] = (n1split[0]);

        losslessCompressedBuf[3] |= n2split[3];  // 4 used  and 0 free in buffer
        losslessCompressedBuf[4] = n2split[2];   // 12 used and 0 free in buffer
        losslessCompressedBuf[5] = n2split[1];   // 20 used and 0 free in buffer
        losslessCompressedBuf[6] = n2split[0];   // 22 used and 6 free in buffer
        losslessCompressedBuf[7] = 0;   // 14 free in buffer
        losslessCompressedBuf[8] = 0;   // 22 free in buffer                                                        
        losslessCompressedBuf[9] = 0;   // 30 free in buffer                                                         
        losslessCompressedBuf[10] = 0;  // 31 free in buffer                  	
}

static inline uint8_t getBit(uint8_t* buf, uint8_t index);
static uint8_t getBit(uint8_t* buf, uint8_t index) {
	uint8_t b = buf[index >> 3];
	uint8_t mask = (1 << (7 - (index & 7)));
	uint8_t value = b & mask;
	return value == 0 ? 0 : 1;
}

static inline void setBit(uint8_t* buf, uint8_t index, uint8_t value01);
static void setBit(uint8_t* buf, uint8_t index, uint8_t value) {
	uint8_t idx = index >> 3;
	uint8_t mask = 1 << (7 - (index & 7));
	if (value)
		buf[idx] |= mask;
	else
		buf[idx] &= ~mask;
}

static inline uint8_t bitParity(uint32_t v);
static uint8_t bitParity(uint32_t v) {
	v ^= v >> 16;
	v ^= v >> 8;
	v ^= v >> 4;
	v &= 0xf;
	return (0x6996 >> v) & 1;
}

static void convolutionalEncoding() {
	uint32_t shift1 = 0;
	uint32_t shift2 = 0;
	uint8_t pos = 0;

	uint8_t idx;
	for (idx = 0; idx < 81; idx++) {
		uint8_t bit = getBit(losslessCompressedBuf, idx);
		shift1 = (shift1 << 1) | bit;
		shift2 = (shift2 << 1) | bit;

		uint32_t x = shift1 & 0xF2D05351UL;
	//	uint8_t bitp = stupidBitParity(x);
		uint8_t bitp2 = bitParity(x);

	//	if (bitp != bitp2)
	//		trace_printf("ERROR! bitp\n");

		setBit(convolutionalBuf, pos++, bitp2);

		x = shift2 & 0xE4613C47UL;
		bitp2 = bitParity(x);
		setBit(convolutionalBuf, pos++, bitp2);
	}
}

static uint8_t fasterBitReverse(uint8_t v) {
  uint8_t r = v; // r will be reversed bits of v; first get LSB of v
  int s = 7; // extra shift needed at end
  
  for (v >>= 1; v; v >>= 1)
    {   
      r <<= 1;
      r |= v & 1;
      s--;
    }
  return r << s; // shift when v's highest bits are zero
}

static void interleave() {
  uint8_t p = 0;
  for (uint16_t i = 0; i < 256; i++) {
   //  uint8_t j = bitReverse(i);
      uint8_t k = fasterBitReverse(i);
  //    if (j != k) trace_printf("Bitreverse lort! %d %d\n", j, k);
    if (k < 162) {
      uint8_t value = getBit(convolutionalBuf, p);
      setBit(interleavedbuf, k, value);
      p++;
    }
  }
}

static const uint32_t SYNC_VECTOR_COMPACT[162/32+1] = {
0b11000000100011100010010111100000,
0b00100101000000101100110100011010,
0b00011010101010010010110001101010,
0b00100000100100111011001101000111,
0b00000101001100000001101011000110,
0b00 };

static inline uint8_t readSyncVectorSym(uint8_t idx);
static uint8_t readSyncVectorSym(uint8_t idx) {
    uint8_t index = idx/32;
    uint32_t mask = 1<<(31-(idx%32));
    uint8_t newvalue = (SYNC_VECTOR_COMPACT[index] & mask) != 0 ? 1 : 0;
    return newvalue;
}

static void makeSymbolList() {
	for (uint8_t i = 0; i < 163; i++) {
		// The last (161th in 0-base) symbol is added 2x to the list.
		uint8_t sourceIndex = i==162 ? 161 : i;
		uint8_t idx = i / 4;
		uint8_t shift = i & 3;
		if (shift == 0)
			symbolList[idx] = 0; // clear a byte 1st time we see it.
		uint8_t value = getBit(interleavedbuf, sourceIndex);
		uint8_t sym = readSyncVectorSym(sourceIndex) + (value ? 2 : 0);
		symbolList[idx] |= sym << (6 - shift * 2);
	}
}

void completeMessage() {
	convolutionalEncoding();
	interleave();
	makeSymbolList();
}

void positionAs4DigitMaidenhead(double lat, double lon, char* target) {
	lon = lon + 180;
	lat = lat + 90;
	target[0] = 'A' + (int) lon / 20;
	target[1] = 'A' + (int) lat / 10;
	target[2] = '0' + (int) (fmod(lon, 20) / 2);
	target[3] = '0' + (int) (fmod(lat, 10));
}

void currentPositionAs4DigitMaidenhead(char* target) {
	positionAs4DigitMaidenhead(lastNonzeroPosition.lat, lastNonzeroPosition.lon, target);
	target[4] = 0;
}

void positionAs6DigitMaidenhead(double lat, double lon, char* target) {
	lon = lon + 180;
	lat = lat + 90;
	target[0] = 'A' + (int) lon / 20;
	target[1] = 'A' + (int) lat / 10;
	target[2] = '0' + (int) (fmod(lon, 20) / 2);
	target[3] = '0' + (int) (fmod(lat, 10));
	target[4] = 'a' + (int) ((lon - (int) (lon / 2) * 2) * 12);
	target[5] = 'a' + (int) ((lat - (int) (lat)) * 24);
}

void currentPositionAs6DigitMaidenhead(char* target) {
  positionAs6DigitMaidenhead(lastNonzeroPosition.lat, lastNonzeroPosition.lon, target);
  target[6] = 0;
}

void positionAsMaidenheadSuperfine(double lat, double lon, char* target) {
  lon = lon + 180;
  lat = lat + 90;
  target[0] = 'A' + (int) lon / 20;
  target[1] = 'A' + (int) lat / 10;
  target[2] = '0' + (int) (fmod(lon, 20) / 2);
  target[3] = '0' + (int) (fmod(lat, 10));
  
  // = (lon - (lon fmod 2)) * 12, resolution is 1/12 degree (of which we lose half because faking)
  // target[4] = 'a' + (int) ((lon - (int) (lon / 2) * 2) * 12);
  // = (lon - (lon fmod 1)) * 24, resolution is 1/24 degree (of which we lose half because faking)
  // target[5] = 'a' + (int) ((lat - (int) (lat)) * 24);
  
  // now we must get lon in 1/(6*24) degrees, that is 50 seconds or 0.5 nm at 47N
  target[4] = 'a' + (int) (fmod(lon, 1.0/6) * 24*6);
  // and lat in 1/(12*24) degrees with resolution, that is 25 seconds or 0.42 nm
  target[5] = 'a' + (int) (fmod(lat, 1.0/12) * 24*12);
}

void currentPositionAsMaidenheadSuperfine(char* target) {
  positionAsMaidenheadSuperfine(lastNonzeroPosition.lat, lastNonzeroPosition.lon, target);
}


void prepareType1Transmission(uint8_t power) {
  char maidenhead4[5];
  currentPositionAs4DigitMaidenhead(maidenhead4);
  encodeType1Message(maidenhead4, power);
  completeMessage();
}

void prepareType3Transmission(uint8_t power, WSPR_MESSAGE_TYPE fake) {
  char maidenhead6_fake[7];
  maidenhead6_fake[6] = 0;
  
  // dummy tm
  // uint8_t gpsFixMode = GPSStatus.fixMode; // between 0 and 2
  // gpsAcqTime is allowed to be in the range [0..5]
  // uint8_t gpsAcqTime = ilog2(lastGPSFixTime/2);
  // uint8_t gpsNumSats = GPSStatus.numberOfSatellites;

  switch(fake) {
  case  REAL_EXTENDED_LOCATION:
    currentPositionAs6DigitMaidenhead(maidenhead6_fake);
    // if (maidenhead6_fake[4] & 1) maidenhead6_fake[4]++; // make even
    if (maidenhead6_fake[5] & 1) maidenhead6_fake[5]++; // make even
    break;

    /*
  case SUPERFINE_EXTENDED_LOCATION:
    currentPositionAsMaidenheadSuperfine(maidenhead6_fake);
    if (!(maidenhead6_fake[4] & 1)) maidenhead6_fake[4]--; // make odd
    if (maidenhead6_fake[5] & 1) maidenhead6_fake[5]++;  // make even
    break;
   */
  case ALTITUDE:
    currentPositionAs4DigitMaidenhead(maidenhead6_fake);
    // add data here
    int16_t ialt = (int16_t)((lastNonzero3DPosition.alt+25) / 50);
    if (ialt < 0) ialt = 0;
    // 144 units of 100m each
    maidenhead6_fake[4] = 'a' + (ialt / 12);	// Units of 50m*12m, max. 23*50*12=13800m
    maidenhead6_fake[5] = 'a' + (ialt % 12)*2;	// Units of 50m, max. 11*50m, max. total 13800m + 50m*11 = 14350m
    // if (maidenhead6_fake[4] & 1) maidenhead6_fake[4]++;    // make even
    if (!(maidenhead6_fake[5] & 1)) maidenhead6_fake[5]--; // make odd
    break;

    /*
  case TELEMETRY: 
    currentPositionAs4DigitMaidenhead(maidenhead6_fake);
    // gpsNumSats is translated:
    // 0-->0, 1-->0, 2-->0, 3-->1, 4-->2, 5-->3, 6-->4, 7 and more: 5
    uint8_t trNumSats;
    if (gpsNumSats < 3) trNumSats = 0;
    else trNumSats = gpsNumSats - 2;
    if (trNumSats > 7) trNumSats = 7;
    if (gpsAcqTime > 5) gpsAcqTime = 5;
    maidenhead6_fake[4] = 'a' + ((trNumSats&4)/4 + gpsAcqTime*2)*2;
    maidenhead6_fake[5] = 'a' + ((trNumSats&3) + GPSStatus.fixMode*4)*2;
    if (!(maidenhead6_fake[4] & 1)) maidenhead6_fake[4]--;
    if (!(maidenhead6_fake[5] & 1)) maidenhead6_fake[5]--;
    break;
    */
    
  // default:
    // trace_printf("Unknown fake-type: %u\n", fake);
  }

  encodeType3Message(maidenhead6_fake, power);
  completeMessage();
}

uint8_t WSPR_getSymbol(uint8_t i) {
  uint8_t idx = i / 4;
  uint8_t shift = i & 3;
  uint8_t sym = symbolList[idx] >> (6 - shift * 2);
  sym &= 3;
  return sym;
}

static const uint8_t powerLevels[] =
{0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37};

static uint8_t fake_dBm_batteryVoltage() {
	// All others cause failure to report position or something, in WSPR program.
	// battery range
	// 10 (0.01W)= 3   .. 3.9
	// 13 (0.02W)= 3.9 .. 4.0
	// 17 (0.05W)= 4.0 .. 4.1
	// 20 (0.1W) = 4.1 .. 4.2
	// 23 (0.2W) = 4.2 ..
	int8_t index = (vBattery - 3.8) * 10;
	if (index < 0) return 0;
	if (index >= sizeof(powerLevels)) return 37;
	return powerLevels[index];
}

static uint8_t fake_dBm_speed() {
	// 10 (0.01W)= 0   .. 20
	// 13 (0.02W)= 20 ..  40
	// 17 (0.05W)= 40 ..  60
	// 20 (0.1W) = 60 ..  80
	// 23 (0.2W) = 80 ..
	int8_t index = speed_kts/20;
	if (index < 0) return 0;
	if (index >= sizeof(powerLevels)) return 37;
	return powerLevels[index];
}

void prepareWSPRMessage(WSPR_MESSAGE_TYPE messageType) {
    uint8_t power = fake_dBm_speed();
    // trace_printf("type: %d\n", messageType);
  switch (messageType) {
  case TYPE1:
    prepareType1Transmission(power);
    break;
  default:
    prepareType3Transmission(power, messageType);
    break;
  }
}

void doWSPR(WSPRBand_t band) {
	uint8_t nextWSPRMessageType;

	if (lastNonzero3DPosition.alt < LOWALT_THRESHOLD) {
		if (nextWSPRMessageTypeIndex >= WSPR_LOWALT_SCHEDULE_LENGTH) {
			nextWSPRMessageTypeIndex = 0;
		}
		nextWSPRMessageType = WSPR_LOWALT_SCHEDULE[nextWSPRMessageTypeIndex];
	} else {
		if (nextWSPRMessageTypeIndex >= WSPR_SCHEDULE_LENGTH) {
			nextWSPRMessageTypeIndex = 0;
		}
		nextWSPRMessageType = WSPR_SCHEDULE[nextWSPRMessageTypeIndex];
	}

	prepareWSPRMessage(nextWSPRMessageType);
	nextWSPRMessageTypeIndex++;
	WSPR_Transmit(THIRTY_M);

	// Recurse, do it again if we sent the rather uninformative TYPE1.
	// if (nextWSPRMessageType == TYPE1) {
	// 	doWSPR();
	// }
}

