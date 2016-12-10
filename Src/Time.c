/*
 * Time.c
 *
 *  Created on: Dec 8, 2016
 *      Author: dongfang
 */

#include "Types.h"

int secondsOfDay(Time_t* time) {
	int result = time->seconds;
	result += time->minutes * 60;
	result += time->hours * 3600;
	return result;
}

int timeDiffSeconds(Time_t* from, Time_t* to) {
	return secondsOfDay(to) - secondsOfDay(from);
}

// From must be before or equal to to.
int timeAfter_seconds(Time_t* from, Time_t* to) {
	int diff = timeDiffSeconds(from, to);
	if (diff < 0)
		diff += 24 * 60 * 60;
	return diff;
}

static const uint8_t DOM[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

static uint8_t daysOfMonth(uint8_t year100, uint8_t oneBasedMonth) {
	if ((year100 % 4) == 0 && oneBasedMonth == 2)
		return 29;
	return DOM[oneBasedMonth - 1];
}

/*
int minutesBetween(DateTime_t* from, DateTime_t* to) {
	int result = 0;
	int maxDaysForTermination = 1000;
	Date_t countDays = from->date;
	while (from->date.year100 != to->date.year100 || from->date.month != to->date.month || from->date.date != to->date.date) {
		if (countDays.date == daysOfMonth(countDays.year100, countDays.month)) {
			// new month
			countDays.date = 1;
			if (countDays.month == 12) {
				// new year too
				countDays.year100++;
				countDays.month = 1;
			} else {
				countDays.month++;
			}
		} else {
			countDays.date++;
		}
		result += 24*60;
		if (--maxDaysForTermination == 0) return -1; // caller fucked up, and passed from after to.
	}
	int minutes = to->time.hours * 60 + to->time.minutes - from->time.hours * 60
			- from->time.minutes;
	return result + minutes;
}
*/

uint16_t compact (DateTime_t* datetime) {
	return (datetime->date.date*24 + datetime->time.hours)*60 + datetime->time.minutes;
}
