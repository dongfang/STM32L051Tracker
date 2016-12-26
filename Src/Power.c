/*
 * Power.c
 *
 *  Created on: Oct 20, 2016
 *      Author: dongfang
 */

#include <Power.h>
#include <stm32l051xx.h>
#include <stm32l0xx.h>

void setRuntimeClocks() {
	// We should set up at least:
	// An USART clock
	// Probably need to run at 8-16MHz

	// Page 174:
	// In Range 1 (1.8V) we can run any system clock
	// In Range 2 (1.5V) we cannot run any system clock (but HSE is limited in frequency)
	// In Range 3 (1.2V) we can only run MSI 4.2MHz

	RCC->CR = RCC_CR_HSION;
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
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_0) {
	}

	// Use system clock for USARTs.
	RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_USART1SEL) | RCC_CCIPR_USART1SEL_0;
	RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_USART2SEL) | RCC_CCIPR_USART2SEL_0;

	SysTick_Config(8000);
}

void sleepSpeedConfig() {
	RCC->CR = 0;
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_MSI; /*| 7U << 24 |*/ // Use MSI
	SysTick_Config(2000);
}

/*
void enableGPIOClock(GPIO_RCC_t which) {
	SET_BIT(RCC->IOPENR, 1 << which);

	/ * Delay after an RCC peripheral clock enabling (sk: Why? Reference manual says nothing about a delay.) * /
	uint32_t tmpreg;
	do {
		tmpreg = READ_BIT(RCC->IOPENR, 1 << which);
		if (tmpreg == 0) {
			trace_printf("waited.\n");
		}
	} while (tmpreg == 0);
}

void disableGPIOClock(GPIO_RCC_t which) {
	CLEAR_BIT(RCC->IOPENR, 1 << which);
}
*/

