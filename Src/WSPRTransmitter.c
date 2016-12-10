/*
 * WSPRTransmitter.c
 *
 *  Created on: Dec 9, 2016
 *      Author: dongfang
 */
#include "stm32l0xx.h"
#include "Trace.h"
#include "WSPR.h"
#include "LED.h"
#include "Power.h"

// flag set by interrupt handler after each WSPR bit time elapsed.
volatile uint8_t TIM22Updated;

static void WSPRModulate(uint8_t index) {
	uint8_t symbol = getWSPRSymbol(index);
	if (symbol & 1) {
		GPIOA->BSRR = 1;		// set PA0
	} else {
		GPIOA->BSRR = 1 << 16;	// reset PA0
	}
	if (symbol & 2) {
		GPIOA->BSRR = 2;		// set PA1
	} else {
		GPIOA->BSRR = 2 << 16;	// reset PA1
	}
}

void WSPRModulationLoop() {
	// Ports connected to WSPR modulator:
	// PA0, PA1, PA4 and PB1
	// Enable TIM22 clock
	// RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	enableGPIOClock(RCC_IOPENR_GPIOAEN);

	RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;

	// PA0 and PA1 are outputs.
	GPIOA->MODER = (GPIOA->MODER & ~0xf) | 0x4 | 0x1;
	// PA4 and PB1 are analog mode
	GPIOA->MODER = GPIOA->MODER | (3 << (4 * 2));
	GPIOB->MODER = GPIOB->MODER | (3 << (1 * 2));
	// don't bother to set OTYPER or OSPEEDR, they are okay by default.

	WSPRModulate(0); // Output 0th symbol now

	// we need 22369.62133410543483 divider ~ 22370 = 10*2370
	TIM22->PSC = 10 - 1;
	TIM22->ARR = 2237 - 1;

	// Supposed to route LSE to ETR
	TIM22->OR |= TIM22_OR_ETR_RMP_0 | TIM22_OR_ETR_RMP_1;
	// Select external clock as source.
	TIM22->SMCR |= TIM_SMCR_ECE;

	TIM22->DIER = TIM_DIER_UIE;
	NVIC_SetPriority(TIM22_IRQn, 0);
	NVIC_EnableIRQ(TIM22_IRQn);

	// Prepare WFI
	PWR->CR = (PWR->CR & ~3) | PWR_CR_ULP | PWR_CR_FWU;
	SCB->SCR &= ~4; // Don't STOP.

	TIM22Updated = 0;

	// Enable Timer22
	TIM22->CR1 |= TIM_CR1_CEN | TIM_CR1_DIR;

	uint8_t numBitsSent = 0;

	// at init: 0th symbol
	// 0->1 : 0th symbol (re)starts
	// 161->162: 161th and last symbol starts
	// 162->163: 161th symbol ends.
	while (numBitsSent < 163) {
		__WFI();
		if (TIM22Updated) {
			WSPRModulate(numBitsSent++);
			// trace_printf("%d\n", numBitsSent);
			TIM22Updated = 0;
			LED_toggle();
		}
	}
}

void WSPR_Transmit(
		WSPRBand_t band,
		const PLL_Setting_t* setting) {

	trace_printf("Waiting for WSPR window\n");
	// RTC_waitTillModuloMinutes(2, 0);
	// Start PLL early to let drift settle
	// Right now we ignore the band parameter and support just this one band.
	setPLL(HF_30m_HARDWARE_OUTPUT, setting);
	WSPRModulationLoop();
	LED_off();
	// WSPR_shutdownHW();
}

