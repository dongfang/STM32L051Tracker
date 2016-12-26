/*
 * WSPR.h
 *
 *  Created on: Mar 12, 2015
 *      Author: dongfang
 */

#ifndef WSPR_H_
#define WSPR_H_

#include <stdint.h>
#include <Types.h>

extern const uint32_t WSPR_FREQUENCIES[];

// The fake message types for WSPR
typedef enum {
	TYPE1,
	REAL_EXTENDED_LOCATION,
	ALTITUDE,
	QRP_LABS
} WSPR_MESSAGE_TYPE;

#define WSPR_LOWALT_SCHEDULE_DEF {TYPE1, ALTITUDE}
#define WSPR_DEFAULT_SCHEDULE {TYPE1, ALTITUDE, TYPE1, REAL_EXTENDED_LOCATION, QRP_LABS}

extern const uint8_t WSPR_SCHEDULE[];
extern const uint8_t WSPR_SCHEDULE_LENGTH;

extern const uint8_t WSPR_LOWALT_SCHEDULE[];
extern const uint8_t WSPR_LOWALT_SCHEDULE_LENGTH;

void doWSPR(WSPRBand_t band);
void prepareWSPRMessage(WSPR_MESSAGE_TYPE messageType, float txVoltage);
void WSPR_Transmit(WSPRBand_t band);
uint8_t WSPR_getSymbol(uint8_t i);

#endif /* WSPR_H_ */
