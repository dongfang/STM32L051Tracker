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
#define MY_APRS_SSID 3

#define REQUIRE_HIGH_PRECISION_NUM_SATS 4
#define REQUIRE_HIGH_PRECISION_ALTITUDE 1
#define REQUIRE_HIGH_PRECISION_FIXLEVEL 1

#define LOWALT_THRESHOLD 5000

#define BATT_ADC_FACTOR 2

// old xtal batch
// #define DEFAULT_XTAL_FREQ 26000750

// new xtal batch
#define DEFAULT_XTAL_FREQ 25998840

#define DIAGNOSTICS_APRS_FREQUENCY 144800000

// Default, in case of missing trim.
// #define PLL_XTAL_TRIM_PP10M_VALUES {0,0,0,0,1184,944,749,565,421,289,177,73,0,-89,-161,-225,-289,-337,-385,-428,-468}
// another sample:                          1202,943,751,571,427,299,200,80,0,-79,-151,-210,-274,-321,-362,-418,-466
// new batch xtal:
#define PLL_XTAL_TRIM_PP10M_VALUES {0,0,0,0,1500,1194,959,727,547,387,240,104,0,-104,-170,-282,-330,-402,-466,-534,-574}

#endif /* SETUP_H_ */
