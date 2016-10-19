/*
 * RTC.h
 *
 *  Created on: Oct 9, 2016
 *      Author: dongfang
 */

#ifndef RTC_H_
#define RTC_H_

typedef struct {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	// uint16_t millis;
} RTC_Time_t;

void RTC_init(void);

void RTC_read(RTC_Time_t* out);
void RTC_set(RTC_Time_t* time);
void RTC_unlock();
void RTC_lock();
void RTC_scheduleWakeup(uint16_t t);
void RTC_scheduleAlarmA(RTC_Time_t* time);
void RTC_scheduleAlarmB(RTC_Time_t* time);
#endif /* RTC_H_ */
