/*
 * Systime.c
 *
 *  Created on: Oct 9, 2016
 *      Author: dongfang
 */
#include "stm32l0xx.h"
#include "Systime.h"

// This one, we can leave inited to zero (bss).
volatile uint32_t systime;

void SysTick_Handler(void) {
	systime++;
}

void timer_sleep(uint32_t time) {
	uint32_t timeout = systime + time;
	int32_t diff;

	do {
		// systime = 2^32-11, time=20, timeout=10
		// diff = 2^32-11 - 10 as
		diff = timeout - systime;
		if (diff > 0) {
			__WFI(); // this goes fukt??
		}
	} while (diff > 0);
}

