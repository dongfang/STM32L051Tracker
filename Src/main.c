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
#include "Power.h"
#include <core_cm0plus.h>
#include "GPS.h"
#include "PLL.h"

static int loopCnt;

extern NMEA_StatusInfo_t GPSStatus;

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

	GPS_powerOn();

	// GPS_waitForTimelock(1000000);

	/*
	while (1) {
		APRS_transmitMessage(VHF, COMPRESSED_POSITION_MESSAGE, 144800000, 26000000);
	}

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

	// SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
	enableGPIOClock(RCC_IOPENR_GPIOAEN);

	PLL_shutdown();
	// APRS_makeDirectTransmissionFrequency(50000000, 26000900, DIRECT_2m_HARDWARE_OUTPUT);

	doWSPR(THIRTY_M);

	while (1) {
		//PLL_setBypassModeWithDivision(14, 1000);
		//double pllTime = measurePeriod(RTC_CYCLES_PER_PLL_CYCLE, 32768 * 5);
		//PLL_shutdown();
		//double gpsTime = measurePeriod(RTC_CYCLES_PER_GPS_CYCLE, 32768 * 5);
		//double fpll = gpsTime / pllTime * 1000.0;
		//trace_printf("pll: %lu, gps: %lu, fpll %lu\n",
		//(uint32_t) (pllTime * 1000000.0), (uint32_t) (gpsTime * 100.0),
		//(uint32_t) fpll);
		//double ratio = measurePeriod(PLL_CYCLES_PER_GPS_CYCLE, 10);

		// Input.
		GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 6))) | 0 << (2 * 6);
		while ((GPIOA->IDR & (1 << 6)) == 0) {
			GPS_getData();
			trace_printf("Waiting for GPS timelock (%d)\n", GPSStatus.numberOfSatellites);
			timer_sleep(1000);
		}

		trace_printf("tps\n");
		PLL_setBypassModeWithDivision(10, 7);

		timer_sleep(2500);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
