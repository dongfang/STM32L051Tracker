/*
 * RTC.h
 *
 *  Created on: Oct 9, 2016
 *      Author: dongfang
 */

#ifndef RTC_H_
#define RTC_H_

#include "Types.h"

#define RTC_BACKUP_REGISTER_TIME_VALID_IDX 0
#define RTC_BACKUP_REGISTER_NUM_RESETS_IDX 1
#define RTC_BACKUP_REGISTER_NUM_WWDG_RESETS_IDX 2

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

uint32_t RTC_readBackupRegister(uint8_t index);
void RTC_writeBackupRegister(uint8_t index, uint32_t data);

#endif /* RTC_H_ */
