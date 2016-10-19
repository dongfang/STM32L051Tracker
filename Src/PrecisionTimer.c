/*
 * PrecisionTimer.c
 *
 *  Created on: Sep 17, 2016
 *      Author: dongfang
 */

#include "stm32l0xx.h"
#include "Types.h"
#include "Trace.h"

/* Need : PB6 and PC3 for this. Arh damn PC3 is not available. Need to share and mix then?? Fuck. */

/* Idea:
 * 1) Disable RTC calibration (if that has anything to do with anything)
 * 2) Measure GPS 1Hz freq using external trigger on LPTIM
 * 3) Measure PLL lowest possible freq using external trigger on LPTIM
 * 4) Calculate...
 *
 * Can also: Using LSE as input and just the normal modes, generate interrupts for WSPR transmission
 * Can also: Adjust MSI (not really worth a shit)
 * Can also: Use LSE/27 as a 1200 bit/s interrupt source. It will be 1% off but that can be corrected using 27-27-28 division.
 *
 */

volatile uint32_t PT_extendedCaptureValue __attribute__((section (".noinit")));
volatile int32_t PT_captureCount __attribute__((section (".noinit")));
/*
 void TIM21_LSE_Experiment() {
 // Enable LPTIM clock
 RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;

 // Enable external input GPIO crap

 // Use ARR interrupt
 TIM21->DIER |= TIM_DIER_CC2IE;

 // Supposed to route LSE to TI1 (a trigger input)
 TIM21->OR |= (TIM21->OR & ~TIM21_OR_TI1_RMP) | TIM21_OR_TI1_RMP_2;

 // Select the pin to be used by writing CCxS bits in the TIMx_CCMR1 register.
 TIM21->CCMR1 |= 1; // CC1 channel is configured as input, IC1 is mapped on TI1
 // TIM21->CCER = 1; // capture enable huh??

 // Select the timer TIx as the trigger input source by writing TS bits in the TIMx_SMCR register.
 TIM21->SMCR |= 0b100 << 4; // External Clock Mode 1 (the TS thing)
 TIM21->SMCR |= 0b111; // External Clock Mode 1 (the SMS thing)

 TIM21->PSC = 0;
 TIM21->ARR = 60000 - 1;

 TIM21->CR1 |= TIM_CR1_CEN;
 }

 void TIM21_LSE_Experiment_ETR() {
 // Enable GPIOA clock
 SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);

 // Alternate function #0 TIM21_CH1
 GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 2))) | 2 << (2 * 2);
 // GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 2))) | 0 << (2 * 2);
 GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(15 << (4 * 2))) | 0;

 // Enable LPTIM clock
 RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;

 TIM21->PSC = 32767;
 TIM21->ARR = 60000 - 1;

 // Supposed to route LSE to ETR
 TIM21->OR |= TIM21_OR_ETR_RMP_0 | TIM21_OR_ETR_RMP_1;

 // Select external trigger as source.
 TIM21->SMCR |= TIM_SMCR_ECE;

 // Set up capture input (PA2)
 TIM21->DIER |= TIM_DIER_CC1IE;
 TIM21->CCMR1 |= 1; // use TI1 for CC1 input
 TIM21->CCER |= 1;  // enable capture input 1

 TIM21->CR1 |= TIM_CR1_CEN;
 }
 */

void TIM22_LSECycleCount_ETR(TimerMeasurement_t inputSelect) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;

	// Reset the TIM22 periph if we feel so inclined.
	// RCC->APB2RSTR |= RCC_APB2RSTR_TIM22RST;
	// RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM22RST;

	//switch (inputSelect) {
	//case RTC_CYCLES_PER_GPS_CYCLE:
	if (inputSelect == RTC_CYCLES_PER_GPS_CYCLE || inputSelect == PLL_CYCLES_PER_GPS_CYCLE) {
		// Enable GPIOA clock
		SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);

		// Alternate function #5 TIM22_CH1
		GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 6))) | 2 << (2 * 6);
		// GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 2))) | 0 << (2 * 2);
		GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(15 << (4 * 6))) | (5 << (4 * 6));
	}

	//case RTC_CYCLES_PER_PLL_CYCLE:
	if (inputSelect == RTC_CYCLES_PER_PLL_CYCLE) {
		// Enable GPIOB clock
		SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

		// Alternate function #4 TIM22_CH1
		GPIOB->MODER = (GPIOB->MODER & ~(3 << (2 * 4))) | 2 << (2 * 4);
		// GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 2))) | 0 << (2 * 2);
		GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(15 << (4 * 4))) | (4 << (4 * 4));
	}

	else if (inputSelect == PLL_CYCLES_PER_GPS_CYCLE) {
		// Enable GPIOB clock
		SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

		// Alternate function #4 TIM22_CH2
		GPIOB->MODER = (GPIOB->MODER & ~(3 << (2 * 5))) | 2 << (2 * 5);
		// GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 2))) | 0 << (2 * 2);
		GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(15 << (4 * 5))) | (4 << (4 * 5));
	}

	// Enable TIM22 clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;

	TIM22->PSC = 0;
	TIM22->ARR = 60000 - 1;

	// Select external trigger as source.
	TIM22->SMCR |= TIM_SMCR_ECE;

	// Set up capture input (PA6)
	TIM22->CCMR1 = (TIM22->CCMR1 & ~ TIM_CCMR1_CC1S) | TIM_CCMR1_CC1S_0; // use TI1 for CC1 input

	// If measuring PLL, prescale by 8.
	switch (inputSelect) {
	case RTC_CYCLES_PER_PLL_CYCLE:
		// Supposed to route LSE to ETR
		TIM22->OR |= TIM22_OR_ETR_RMP_0 | TIM22_OR_ETR_RMP_1;
		// Divide by 8 prescaler
		TIM22->CCMR1 |= TIM_CCMR1_IC1PSC;
		break;
	case RTC_CYCLES_PER_GPS_CYCLE:
		// Supposed to route LSE to ETR
		TIM22->OR |= TIM22_OR_ETR_RMP_0 | TIM22_OR_ETR_RMP_1;
		// No prescaler
		TIM22->CCMR1 &= ~TIM_CCMR1_IC1PSC;
		break;
	case PLL_CYCLES_PER_GPS_CYCLE:
		// Supposed to route GPIO to ETR
		TIM22->OR &= (TIM22_OR_ETR_RMP);
		// No prescaler
		TIM22->CCMR1 &= ~TIM_CCMR1_IC1PSC;
		break;
	}

	TIM22->CCER |= TIM_CCER_CC1E;  // enable capture input 1

	TIM22->CR1 |= TIM_CR1_CEN;
}

double measurePeriod(TimerMeasurement_t inputSelect, uint32_t numRTCCycles) {
	PT_extendedCaptureValue = 0;
	PT_captureCount = -1;

	TIM22_LSECycleCount_ETR(inputSelect);

	NVIC_SetPriority(TIM22_IRQn, 0);
	NVIC_EnableIRQ(TIM22_IRQn);

	TIM22->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
	TIM22->DIER |= TIM_DIER_CC1IE;

	// Go now.
	PWR->CR = (PWR->CR & ~3) | PWR_CR_ULP | PWR_CR_FWU;
	SCB->SCR &= ~4; // Don't STOP.

	while (PT_extendedCaptureValue < numRTCCycles) {
		// __WFI();
	}

	// trace_printf("Capture value %lu\n", extendedCaptureValue);

	double result;
	switch (inputSelect) {
	case RTC_CYCLES_PER_GPS_CYCLE:
	case PLL_CYCLES_PER_GPS_CYCLE:
		result = (double) PT_extendedCaptureValue / PT_captureCount;
		break;
	case RTC_CYCLES_PER_PLL_CYCLE:
		// PLL signal is fast, and we used a prescaler of 8.
		result = (double) PT_extendedCaptureValue / (PT_captureCount * 8);
		break;
	}

	// Disable capture/compare (not strictly needed)
	TIM22->CCER &= ~TIM_CCER_CC1E;

	// Disable the whole timer (not strictly needed)
	TIM22->CR1 &= ~TIM_CR1_CEN;

	// Unmap the AFs. This is important, or else we might enable both mappings at the same time later.
	// That does not work.
	GPIOA->MODER &= ~(3 << (2 * 6));
	GPIOB->MODER &= ~(3 << (2 * 4));
	return result;
}

