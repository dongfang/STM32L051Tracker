/*
 * Power.h
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */

#ifndef POWER_H_
#define POWER_H_

#include "Types.h"

void switchTo8MHzHSI();
void switchTo1MHzMSI();
void calibrateMSI(uint32_t fDesired);
#endif /* POWER_H_ */
