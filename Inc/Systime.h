/*
 * Systime.h
 *
 *  Created on: Oct 9, 2016
 *      Author: dongfang
 */

#ifndef SYSTIME_H_
#define SYSTIME_H_

#include "Types.h"

uint32_t systimeMillis();
void timer_mark();
boolean timer_elapsed(uint32_t millis);

#endif /* SYSTIME_H_ */
