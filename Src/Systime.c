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

uint32_t systimeMillis() {
	return systime;
}

void timer_sleep(uint32_t time) {
	uint32_t timeout = systime + time;
	int32_t diff;
	PWR->CR = (PWR->CR & ~3) | PWR_CR_ULP | PWR_CR_FWU;
	SCB->SCR &= ~4; // Don't STOP.
	do {
		// systime = 2^32-11, time=20, timeout=10
		// diff = 2^32-11 - 10 as
		diff = timeout - systime;
		if (diff > 0) {
			__WFI();
		}
	} while (diff > 0);
}

static int32_t mark;
void timer_mark() {
	mark = systime;
}

boolean timer_elapsed(uint32_t millis) {
	int32_t diff = (mark + millis) - systime;
	return diff >= 0;
}
