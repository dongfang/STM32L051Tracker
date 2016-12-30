/*
 * WSPRTransmitter.c
 *
 *  Created on: Dec 9, 2016
 *      Author: dongfang
 */
#include "stm32l0xx.h"
#include "WSPR.h"
#include "LED.h"
#include "PLL.h"
#include "Calibration.h"

// flag set by interrupt handler after each WSPR bit time elapsed.
volatile uint8_t TIM22Updated;

void WSPR_modulate(uint8_t symbol) {
	/*
	if (symbol & 1) {
		GPIOA->BSRR = 1;		// set PA0
	} else {
		GPIOA->BRR = 1;			// reset PA0
	}
	if (symbol & 2) {
		GPIOA->BSRR = 2;		// set PA1
	} else {
		GPIOA->BRR = 2;			// reset PA1
	}
	*/
	GPIOA->ODR = (GPIOA->ODR &~3) | (symbol & 3);
	// trace_putchar(symbol + '0');
}

void WSPR_initGPIO() {
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	// PA0 and PA1 are outputs.
	GPIOA->MODER = (GPIOA->MODER & ~0xf) | 0x4 | 0x1;
	// PA4 and PB1 are analog mode
	GPIOA->MODER = GPIOA->MODER | (3 << (4 * 2));
	GPIOB->MODER = GPIOB->MODER | (3 << (1 * 2));
	// don't bother to set OTYPER or OSPEEDR, they are okay by default.
}

void WSPR_shutdownGPIO() {
	GPIOA->MODER = GPIOA->MODER | 0xf; // analog mode.
}

void WSPRModulationLoop() {
	// Ports connected to WSPR modulator:
	// PA0, PA1, PA4 and PB1
	// Enable TIM22 clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;

	RCC->APB2RSTR |= RCC_APB2RSTR_TIM22RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM22RST;

	WSPR_initGPIO();
	WSPR_modulate(WSPR_getSymbol(0)); // Output 0th symbol now

	// we need 22369.62133410543483 divider ~ 22370 = 10*2370
	TIM22->PSC = 10 - 1;
	TIM22->ARR = 2237 - 1;

	// Supposed to route LSE to ETR
	TIM22->OR |= TIM22_OR_ETR_RMP_0 | TIM22_OR_ETR_RMP_1;
	// Select external clock as source.
	TIM22->SMCR |= TIM_SMCR_ECE;

	// This should not really be needed?!?!
	TIM22->SR = ~(TIM_SR_UIF);

	TIM22->DIER = TIM_DIER_UIE;
	NVIC_SetPriority(TIM22_IRQn, 0);
	NVIC_EnableIRQ(TIM22_IRQn);

	// Prepare WFI
	PWR->CR = (PWR->CR & ~3); // no LPRUN and no deepsleep.
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
			WSPR_modulate(WSPR_getSymbol(numBitsSent++));
			// trace_printf("%d\n", numBitsSent);
			TIM22Updated = 0;
			LED_toggle();
		}
	}
}

void WSPR_Transmit(WSPRBand_t band) {
	uint32_t expectedOscillatorFrequency = getCalibratedPLLOscillatorFrequency();

	PLL_Setting_t pllSetting;

	double maxError = 10E-6;

	while (maxError < 100E-6) {
		double error = PLL_bestPLLSetting(
				expectedOscillatorFrequency,
				WSPR_FREQUENCIES[band],
				&pllSetting);
		double absError = error < 0 ? -error : error;
		if (absError <= maxError) {
			break;
		} else {
			maxError += 25E-6;
		}
	}

	// One COULD reduce the clock speed before WSPR, as there are no requirements.
	// sleepSpeedConfig();

	// trace_printf("Waiting for WSPR window\n");
	// RTC_waitTillModuloMinutes(2, 0);
	// Start PLL early to let drift settle
	// Right now we ignore the band parameter and support just this one band.
	setPLL(HF_30m_HARDWARE_OUTPUT, &pllSetting);

	GPIOA->BRR = 1<<15; // voltage divider for driver.
	WSPRModulationLoop();
	GPIOA->BSRR = 1<<15;
	LED_off();
	PLL_shutdown();
	WSPR_shutdownGPIO();
	TIM22->CR1 &= ~TIM_CR1_CEN;
}

