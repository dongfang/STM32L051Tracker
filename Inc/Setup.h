/*
 * Setup.h
 *
 *  Created on: Oct 17, 2016
 *      Author: dongfang
 */

#ifndef SETUP_H_
#define SETUP_H_

#define WSPR_CALL "OZ1ZOR"
#define APRS_CALL "HB9FDK"
#define MY_APRS_SSID 1

#define REQUIRE_HIGH_PRECISION_NUM_SATS 4
#define REQUIRE_HIGH_PRECISION_ALTITUDE 1
#define REQUIRE_HIGH_PRECISION_FIXLEVEL 1

#define LOWALT_THRESHOLD 5000

#define LED_PORT GPIOB
#define LED_ON (1 << 2)
#define LED_OFF (1 << (2+16))

//#define PLL_XTAL_TRIM_PP10M_VALUES {\
//2851,2160,1677,1284,1004,752,557,373,238,109,0,-103,-178,-259,-321,-385,-436,-491,-537,-580,-621}

#endif /* SETUP_H_ */
