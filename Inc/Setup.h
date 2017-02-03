/*
 * Setup.h
 *
 *  Created on: Oct 17, 2016
 *      Author: dongfang
 */

#ifndef SETUP_H_
#define SETUP_H_

#define WSPR_CALL "HB9FDK"
#define APRS_CALL "HB9FDK"
#define MY_APRS_SSID 6

#define REQUIRE_HIGH_PRECISION_NUM_SATS 4
#define REQUIRE_HIGH_PRECISION_ALTITUDE 1
#define REQUIRE_HIGH_PRECISION_FIXLEVEL 1

#define LOWALT_THRESHOLD 5000
#define BATT_ADC_FACTOR 2

// old xtal batch
//#define DEFAULT_XTAL_FREQ 26000410

// new xtal batch
#define DEFAULT_XTAL_FREQ (25998945-133)

#define DIAGNOSTICS_APRS_FREQUENCY 144800000

// Default, in case of missing trim.
//#define PLL_XTAL_TRIM_PP10M_VALUES {0,0,0,0,1194,938,743,559,427,275,179,67,0,-103,-151,-231,-279,-335,-386,-426,-474}
// another sample:                          1202,943,751,571,427,299,200,80,0,-79,-151,-210,-274,-321,-362,-418,-466
// new batch xtal:
#define PLL_XTAL_TRIM_PP10M_VALUES {0,0,0,0,1500,1194,959,727,547,387,240,104,0,-104,-170,-282,-330,-402,-466,-534,-574}

#define USE_MSI_WHILE_GPS
#define MSI_WHILE_GPS_SPEED 2250000
// #define MSI_WHILE_GPS_SPEED 1125000
#endif /* SETUP_H_ */
