/*
 * RTC.h
 *
 *  Created on: Oct 9, 2016
 *      Author: dongfang
 */

#ifndef RTC_H_
#define RTC_H_

#include "Types.h"

void RTC_init(void);

void RTC_read(Time_t* out);
void RTC_set(Time_t* time);
void RTC_unlock();
void RTC_lock();
void RTC_scheduleWakeup(uint16_t t);
void RTC_scheduleAlarmA(Time_t* time);
void RTC_scheduleAlarmB(Time_t* time);
uint16_t compactDateTime(DateTime_t* datetime);

// As above but considers one past-midnight event
int timeAfter_seconds(Time_t* from, Time_t* to);
//int timeDiffModulo24(Time_t* from, Time_t* to);
void RTC_nextModoloMinutes(Time_t* time, uint8_t modulo, uint8_t offset);

#endif /* RTC_H_ */
