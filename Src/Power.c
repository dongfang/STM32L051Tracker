/*
 * Power.c
 *
 *  Created on: Oct 20, 2016
 *      Author: dongfang
 */

#include "Globals.h"
#include <stdint.h>
#include <stm32l051xx.h>
#include <core_cm0plus.h>
#include "Power.h"

uint8_t MSITrim;

uint8_t isRunningHSI() {
	return (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_0;
}

uint8_t isRunningMSI() {
	return (RCC->CFGR & RCC_CFGR_SWS) == 0;
}

void switchTo8MHzHSI() {
	SysTick_Config(8000);

	if (isRunningHSI())
		return;
	// We should set up at least:
	// An USART clock
	// Probably need to run at 8-16MHz

	// Page 174:
	// In Range 1 (1.8V) we can run any system clock
	// In Range 2 (1.5V) we cannot run any system clock (but HSE is limited in frequency)
	// In Range 3 (1.2V) we can only run MSI 4.2MHz

	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)) {
	}

	// If we want to set MSI to 1M, here it is
	// RCC->ICSCR = 0xC000;

	RCC->CFGR = // 7U << 24 | // Output LSE to MCO
			0 << 11 | // APB2 is divide by 1
					0 << 8 |  // APB1 is divide by 1
					0b1000 << 4 |  // SYSCLK is divide by 2
					RCC_CFGR_SW_0 // use HSI16 as system clk
			;

	// wait for the switch to happen
	while (!isRunningHSI()) {
	}

	// Experimental: Switch off MSI.
	RCC->CR &= ~RCC_CR_MSION;

	// Use system clock for USARTs.
	// RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_USART1SEL) | RCC_CCIPR_USART1SEL_0;
	// RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_USART2SEL) | RCC_CCIPR_USART2SEL_0;

}

void switchTo1MHzMSI() {
	//if (isRunningMSI())
	//	return;
	RCC->CR |= RCC_CR_MSION;

	while (!(RCC->CR & RCC_CR_MSIRDY)) {
	}

	RCC->CFGR = 0 << 11 | // APB2 is divide by 1
			0 << 8 |  // APB1 is divide by 1
			0b1000 << 4 |  // SYSCLK is divide by 2
			RCC_CFGR_SW_MSI; /*| 7U << 24 |*/ // Use MSI

	while (!isRunningMSI()) {
	}

	if (!isADCUsingHSI) {
		RCC->CR &= ~RCC_CR_HSION;
	}

	SysTick_Config(1000);
}

void switchTo2MHzMSI() {

	//if (isRunningMSI())
	//	return;
	RCC->CR |= RCC_CR_MSION;

	while (!(RCC->CR & RCC_CR_MSIRDY)) {
	}

	RCC->CFGR = 0 << 11 | // APB2 is divide by 1
			0 << 8 |  // APB1 is divide by 1
			0b0000 << 4 |  // SYSCLK is divide by 1
			RCC_CFGR_SW_MSI; /*| 7U << 24 |*/ // Use MSI

	while (!isRunningMSI()) {
	}

	if (!isADCUsingHSI) {
		RCC->CR &= ~RCC_CR_HSION;
	}

	SysTick_Config(2000);
}

void setMediumPerformanceCoreVoltage() {
	while (PWR->CSR & PWR_CSR_VOSF) {
	}
	PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_1;
	while (PWR->CSR & PWR_CSR_VOSF) {
	}
}

void setHighPerformanceCoreVoltage() {
	while (PWR->CSR & PWR_CSR_VOSF) {
	}
	PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0;
	while (PWR->CSR & PWR_CSR_VOSF) {
	}
}

int measureFsys() {
	int n = 2;
	int c = 0;
	while (n--) {
		while (!(TIM21->SR & TIM_SR_CC1IF)) {
		}
		c = TIM21->CCR1 - c;
	}

	if (c < 0)
		c += 1 << 16;
	return c;
}

void setMSITrim(int16_t trim) {
	RCC->ICSCR = (RCC->ICSCR & 0x00FFFFFF) | (trim << 24);
	// for (volatile int delay=0; delay<10000; delay++);
}

void calibrateMSI(uint32_t fDesired) {
	fDesired = fDesired * 8 / 32768;

	RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
	TIM21->ARR = 0xffff;

	// Select the active input: TIMx_CCR1 must be linked to the TI1 input,
	// so write the CC1S bits to ‘01’ in the TIMx_CCMR1 register.
	// As soon as CC1S becomes different from ‘00’, the channel is configured
	// in input mode and the TIMx_CCR1 register becomes read- only.
	TIM21->CCMR1 = TIM_CCMR1_CC1S_0; // TIM_CCMR1_CC1S_1 for TI2
	TIM21->CCMR1 |= 0b11 << 2; // prescale o' eight.

	// TIM21->SMCR |= TIM_SMCR_ECE;
	// Supposed to route LSE to ETR (is this needed? Do we use external clock or external trig??)
	TIM21->OR = 0b100 << 2; // LSE to TI1.

	// Enable CC :
	TIM21->CCER |= TIM_CCER_CC1E;
	TIM21->CR1 |= TIM_CR1_CEN;

	int8_t trim = 16;

	setMSITrim(trim);
	int fMeasured = measureFsys();

	volatile int16_t error;
	volatile int16_t error2 = fMeasured - fDesired;
	volatile int8_t bestTrim;

	do {
		error = error2;
		//save
		bestTrim = trim;
		// try a new setting.
		if (fMeasured > fDesired)
			trim--;
		else
			trim++;
		setMSITrim(trim);
		if (error < 0)
			error = -error; // absolute.
		else if (error == 0)
			break;
		fMeasured = measureFsys();
		error2 = fMeasured - fDesired;
		if (error2 < 0)
			error2 = -error2;
	} while (error > error2); // still going the right way.

	setMSITrim(MSITrim = bestTrim);

	TIM21->CR1 &= ~TIM_CR1_CEN;
	RCC->APB2ENR &= ~RCC_APB2ENR_TIM21EN;
}
