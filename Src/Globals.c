/*
 * Globals.c
 *
 *  Created on: Oct 16, 2016
 *      Author: dongfang
 */
#include "Globals.h"
#include "Setup.h"

const AX25_Address_t APRS_APSTM1_DEST = {"APSTM1",0};

const AX25_Address_t APRS_DEST = {"APRS",0};
const AX25_Address_t MY_ADDRESS = {APRS_CALL,MY_APRS_SSID};
const AX25_Address_t APRS_DIGI1 = {"",0};
const AX25_Address_t APRS_DIGI2 = {"",0};

const char WSPR_CALLSIGN[] = WSPR_CALL;

Location_t GPSPosition;
Location_t lastNonzeroPosition;
Location_t lastNonzero3DPosition;
uint16_t lastGPSFixTime;
uint16_t lastWSPRWindowWaitTime;
boolean latestAPRSRegions[12]; 	 // 12 is sufficently large for the world map...
boolean latestAPRSCores[12];	 // 12 is sufficently large for the world map...
char scheduleName;

float temperature;
int8_t simpleTemperature;

float batteryVoltage;
float solarVoltage;
float speed_kts;

// const int16_t PLL_XTAL_TRIM_PP10M[] = PLL_XTAL_TRIM_PP10M_VALUES;
