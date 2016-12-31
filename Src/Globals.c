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

// const int16_t PLL_XTAL_TRIM_PP10M[] = PLL_XTAL_TRIM_PP10M_VALUES;
