/*
 * Watchdog.c
 *
 *  Created on: Jan 8, 2017
 *      Author: dongfang
 */

#include <stm32l0xx.h>
#include <core_cm0plus.h>
#include "Watchdog.h"

#define WWDG_WINDOW 127
#define WWDG_RELOAD 127
void WWDG_init() {
	/*
 	 *  For 2MHz clock and 1Hz reset rate, we need division by 5. We can only get 4. 4 means we get about 122 Hz
	 *  and must reload several times/sec. Nah. 8 means we get 61 Hz and 1Hz reset is juuuuust enough.
	 */
	RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;

	WWDG->CFR = WWDG_CFR_WDGTB |  // PCLK / 4096 / 8 = 64.1 Hz
	WWDG_CFR_EWI | WWDG_WINDOW; // We need to reset the downcounter before it reaches 63, and we must reset
						// it after it has decremented below 100.
						// The value to be stored in the WWDG_CR register must be between 0xFF and 0xC0.
						// (that is 0x7f and 0x40, when we don't bother to re-assert WDGA)
						// So, maximum frequency when writing 127 is 64.1/27 Hz, and minimum is
						// 64.1/64 Hz.
	// WWDG->CR = WWDG_RELOAD;  // already reset default.
	WWDG->CR |= WWDG_CR_WDGA;
	WWDG->SR = 0; // try prevent untimely interrupts.
	// Interrupts can't really do anything for us (other than let us update some info that the dog barked)
	// In debugging it is very useful to get a stack trace.
	NVIC_EnableIRQ(WWDG_IRQn);
}

void WWDG_pat() {
	volatile uint8_t was = WWDG->CR;
	WWDG->CR = WWDG_RELOAD;
}

// Pre-reset interrupt. We MIGHT cancel the reset from here by reloading the counter.
void WWDG_IRQHandler(void) {
	// volatile int i = WWDG->CR;
	// cancel the reset by doing something legal here:
	// WWDG->CR = 127;
	WWDG->SR = 0; // reset interrupt flag.
	volatile uint32_t someNonemptyNonsenseForABreakpoint = WWDG->CR;
   // WWDG_pat();
	someNonemptyNonsenseForABreakpoint += 0;
}
