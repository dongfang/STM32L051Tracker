/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <NVM.h>
#include <RTC.h>
#include <stdint.h>
#include <stm32l051xx.h>
#include <stm32l0xx.h>
#include <Trace.h>
#include <Types.h>
#include "Systime.h"
#include "APRS.h"
#include "PrecisionTimer.h"
#include <core_cm0plus.h>

void setRuntimeClocks(void) {

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

	RCC->CFGR = 7U << 24 | // Output LSE to MCO
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

static int loopCnt;

/* The only situations definitely needing to be avoided are:
 * 1) Repetitive UVLO at GPS, blasting GPS backup
 * 2) Repetitive UVLO at WSPR, cutting short transmissions
 * 3) Staying in overly conservative settings forever.
 * Maybe simply - there is V_min_device
 * If device attempt goes okay, subtract (v_end - v_uvlo) / 2 from V_min_device
 * else add 0.1 to V_min_device (do that from the start, really).
 */
void mainLoop() {
	if (loopCnt++ < 10)
		return;
}

int main(void) {
#if defined(TRACE)
	setRuntimeClocks();
	trace_initialize();
	trace_printf("Start\n");
#endif
	NVIC_SetPriority(RTC_IRQn, 2);

	RTC_init();

	/*
	 while (1) {
	 APRS_transmitMessage(
	 VHF,
	 COMPRESSED_POSITION_MESSAGE,
	 144700000,
	 26000000);
	 }
	 */
	/*

	 while (true) {
	 RTC_read(&t);
	 //trace_printf("%d.%d:%d \n", t.hours, t.minutes, t.seconds);

	 PWR->CR = (PWR->CR & ~3) | PWR_CR_LPSDSR | PWR_CR_ULP | PWR_CR_FWU;
	 SCB->SCR |= 4;
	 uint32_t isr;
	 do {
	 #ifndef TRACE
	 __WFE();
	 #endif
	 isr = RTC->ISR & (RTC_ISR_WUTF | RTC_ISR_ALRAF | RTC_ISR_ALRBF);
	 } while (!isr);

	 RTC->ISR &= ~isr;

	 #ifdef TRACE
	 setRuntimeClocks();
	 #endif
	 }
	 */

	static int cnt;
	static int dir = 1;
	long total = 0;

	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);

	while (1) {
		//		PLL_printSettings();
		//		PLL_setBypassModeWithDivision(10, 1023);

		PLL_setBypassModeWithDivision(14, 1000);
		double pllTime = measurePeriod(RTC_CYCLES_PER_PLL_CYCLE, 32768 * 5);
		PLL_shutdown();

		double gpsTime = measurePeriod(RTC_CYCLES_PER_GPS_CYCLE, 32768 * 5);

		double fpll = gpsTime / pllTime * 1000.0;

		trace_printf("pll: %lu, gps: %lu, fpll %lu\n",
				(uint32_t) (pllTime * 1000000.0), (uint32_t) (gpsTime * 100.0),
				(uint32_t) fpll);
		// double ratio = measurePeriod(PLL_CYCLES_PER_GPS_CYCLE, 10);
	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}

#endif

typedef enum {
	GPIO_RCC_A = 0,
	GPIO_RCC_B = 1,
	GPIO_RCC_C = 2,
	GPIO_RCC_D = 3,
	GPIO_RCC_E = 4,
	GPIO_RCC_H = 7
} GPIO_RCC_t;

void enableGPIOClock(GPIO_RCC_t which) {
	SET_BIT(RCC->IOPENR, 1 << which);

	/* Delay after an RCC peripheral clock enabling */
	uint32_t tmpreg;
	do {
		tmpreg = READ_BIT(RCC->IOPENR, 1 << which);
	} while (tmpreg == 0);
}

void disableGPIOClock(GPIO_RCC_t which) {
	CLEAR_BIT(RCC->IOPENR, 1 << which);
}

void sleepSpeedConfig(void) {
	RCC->CR = 0;
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_MSI | 7U << 24 | // Use MSI, output LSE to MCO
			0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
