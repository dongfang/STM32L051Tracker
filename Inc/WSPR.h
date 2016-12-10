/*
 * WSPR.h
 *
 *  Created on: Mar 12, 2015
 *      Author: dongfang
 */

#ifndef WSPR_H_
#define WSPR_H_

#include "Types.h"
#include "PLL.h"

extern const uint32_t WSPR_FREQUENCIES[];

// The fake subsubgrid message types for WSPR
typedef enum {
	TYPE1,
	REAL_EXTENDED_LOCATION,
	// SUPERFINE_EXTENDED_LOCATION,		// not necessary
	ALTITUDE,
	// TELEMETRY						// not necessary
} WSPR_MESSAGE_TYPE;

#define WSPR_LOWALT_SCHEDULE_DEF {TYPE1, ALTITUDE}
#define WSPR_DEFAULT_SCHEDULE {TYPE1, ALTITUDE, TYPE1, REAL_EXTENDED_LOCATION}

extern const uint8_t WSPR_SCHEDULE[];
extern const uint8_t WSPR_SCHEDULE_LENGTH;

extern const uint8_t WSPR_LOWALT_SCHEDULE[];
extern const uint8_t WSPR_LOWALT_SCHEDULE_LENGTH;

void prepareWSPRMessage(WSPR_MESSAGE_TYPE messageType, float txVoltage);
void WSPR_Transmit(
		WSPRBand_t band,
		const PLL_Setting_t* setting);
#endif /* WSPR_H_ */
