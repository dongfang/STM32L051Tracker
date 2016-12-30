/*
 * RTC.c
 *
 *  Created on: Oct 9, 2016
 *      Author: dongfang
 */
#include "stm32l0xx.h"
#include "Types.h"
#include "LED.h"
#include "RTC.h"

void RTC_init() {
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// Turn off backup domain write protect.
	SET_BIT(PWR->CR, PWR_CR_DBP);

	// Power up RTC.
	RCC->CSR |= RCC_CSR_RTCSEL_0 | RCC_CSR_LSEON | RCC_CSR_LSEDRV_0;
	RCC->CSR |= RCC_CSR_RTCEN;

	// will not work from here - it is still locked.
	// RTC->CR |= RTC_CR_BYPSHAD;

	// Set up MCO if we need that
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);

	//GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 8))) | 2 << (2 * 8);
	//GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(15 << (4 * 0))) | 0;
	// RCC->CFGR |= 7U << 24; // RCC_CFGR_MCO_LSE

	int i = 10000;
	while (i-- > 0) {
		if (RCC->CSR & RCC_CSR_LSERDY)
			break;
	}

	if (i == 0) {
		LED_faultCode(LED_FAULT_LSE_DEAD);
	}
}

void RTC_deinit() {
	// Just set write protect and stop APB clock.
	CLEAR_BIT(PWR->CR, PWR_CR_DBP);
	RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
}

/*
 * After waking up from low-power mode (Stop or Standby), RSF must be cleared by software. The software must then wait until it is set again before reading the RTC_SSR, RTC_TR and RTC_DR registers.
 The RSF bit must be cleared after wakeup and not before entering low-power mode.
 After a system reset, the software must wait until RSF is set before reading the RTC_SSR, RTC_TR and RTC_DR registers. Indeed, a system reset resets the shadow registers to their default values.
 */
void RTC_read(Time_t* out) {
	// Strictly, if BYPSHAD == 0 we should do this check that shadow regs are updated.
	// while((RTC->ISR & RTC_ISR_RSF) == 0);

	// From 6.1.2 in the reference manual. Right now we do this at startup anyway.
	uint32_t tr = RTC->TR;
	out->hours = ((tr & RTC_TR_HT) >> 20) * 10 + ((tr & RTC_TR_HU) >> 16);
	out->minutes = ((tr & RTC_TR_MNT) >> 12) * 10 + ((tr & RTC_TR_MNU) >> 8);
	out->seconds = ((tr & RTC_TR_ST) >> 4) * 10 + ((tr & RTC_TR_SU) >> 0);
	// uint8_t ss = RTC->SSR;
	// And strictly if BYPSHAD == 0, we should clear RSF here.
}

void RTC_unlock() {
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
}

void RTC_lock() {
	RTC->WPR = 0;
	RTC->WPR = 0;
}

void RTC_set(Time_t* time) {
	uint32_t tr = ((time->hours / 10) << 20) | ((time->hours % 10) << 16)
			| ((time->minutes / 10) << 12) | ((time->minutes % 10) << 8)
			| ((time->seconds / 10) << 4) | ((time->seconds % 10) << 0);

	uint32_t dr = ((2016 / 10) << 20) | ((2016 % 10) << 16) | 1 << 13
			| ((12 / 10) << 12) | ((12 % 10) << 8) | ((31 / 10) << 4)
			| ((31 % 10) << 0);

	RTC_unlock();

	//SET_BIT(RTC->ISR, RTC_ISR_INIT);
	RTC->ISR = 0xFFFFFFFF; // Set INIT bit and leave all the flags alone.
	int w = 0;

	while ((RTC->ISR & RTC_ISR_INITF) == 0)
		w++;

	RTC->TR = tr;
	// RTC->DR = dr;
	RTC->CR &= ~RTC_CR_FMT;

	CLEAR_BIT(RTC->ISR, RTC_ISR_INIT);
	while ((RTC->ISR & RTC_ISR_INITF) != 0);
	RTC_lock();
}

void RTC_scheduleWakeup(uint16_t eachNSec) {
	// EXTI Line 20 to be sensitive to rising edges (Interrupt or Event modes)
	// EXTI->PR |= 1<<20; this clearing is done automatically next line.
	// RCC->APB1ENR |= RCC_APB2ENR_
	EXTI->RTSR |= 1 << 20;
	EXTI->EMR |= 1 << 20; // unmask

	// From 26.3.7 in reference manual.
	RTC_unlock();
	RTC->CR &= ~RTC_CR_WUTE;
	while (RTC->ISR & RTC_ISR_WUTWF == 0)
		;
	RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL) | RTC_CR_WUCKSEL_2;
	RTC->WUTR = eachNSec - 1;

	RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;
	RTC_lock();
}

void RTC_scheduleAlarmA(Time_t* time) {
	// EXTI Line 17 to be sensitive to rising edges (Interrupt or Event modes)
	// EXTI->PR |= 1<<17; this clearing is done automatically next line.

	EXTI->RTSR |= 1 << 17; // rising edge selected.
	EXTI->EMR |= 1 << 17; // unmask

	NVIC_EnableIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 2);

	RTC_unlock();

	// From 26.3.7 in reference manual.
	RTC->CR &= ~RTC_CR_ALRAE;
	while (RTC->ISR & RTC_ISR_ALRAWF == 0)
		;
	// RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL) | RTC_CR_WUCKSEL_2;
	RTC->ALRMAR = RTC_ALRMAR_MSK4
			| // ignore date/day
			(time->hours / 10) << 20 | (time->hours % 10) << 16
			| (time->minutes / 10) << 12 | (time->minutes % 10) << 8
			| (time->seconds / 10) << 4 | (time->seconds % 10);

	RTC->CR |= RTC_CR_ALRAE | RTC_CR_ALRAIE;
	RTC_lock();
}

void RTC_scheduleAlarmB(Time_t* time) {
	// EXTI Line 17 to be sensitive to rising edges (Interrupt or Event modes)
	// EXTI->PR |= 1<<17; this clearing is done automatically next line.
	EXTI->RTSR |= 1 << 17;
	EXTI->EMR |= 1 << 17; // unmask

	// Wakeup seems to work great but I never get an IRQ...
	NVIC_EnableIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 2);

	RTC_unlock();
	// From 26.3.7 in reference manual.
	RTC->CR &= ~RTC_CR_ALRBE;
	while (RTC->ISR & RTC_ISR_ALRBWF == 0)
		;
	// RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL) | RTC_CR_WUCKSEL_2;
	RTC->ALRMBR = RTC_ALRMBR_MSK4 | (time->hours / 10) << 20
			| (time->hours % 10) << 16 | (time->minutes / 10) << 12
			| (time->minutes % 10) << 8 | (time->seconds / 10) << 4
			| (time->seconds % 10);

	RTC->CR |= RTC_CR_ALRBE | RTC_CR_ALRBIE;
	RTC_lock();
}
/*
 * RTC auto-wakeup (AWU) from the Stop mode
  To wake up from the Stop mode with an RTC alarm event, it is necessary to:
 a) Configure the EXTI Line 17 to be sensitive to rising edges (Interrupt or Event
 modes)
 b) Enable the RTC Alarm interrupt in the RTC_CR register
 c) Configure the RTC to generate the RTC alarm
  To wake up from the Stop mode with an RTC Tamper or time stamp event, it is necessary to:
 a) Configure the EXTI Line 19 to be sensitive to rising edges (Interrupt or Event modes)
 b) Enable the RTC TimeStamp Interrupt in the RTC_CR register or the RTC Tamper Interrupt in the RTC_TCR register
 c) Configure the RTC to detect the tamper or time stamp event
  To wake up from the Stop mode with an RTC Wakeup event, it is necessary to:
 a) Configure the EXTI Line 20 to be sensitive to rising edges (Interrupt or Event modes)
 b) Enable the RTC Wakeup Interrupt in the RTC_CR register
 c) Configure the RTC to generate the RTC Wakeup event

  
 
 To wake up from the Standby mode with an RTC alarm event, it is necessary to:
 a) Enable the RTC Alarm interrupt in the RTC_CR register
 b) Configure the RTC to generate the RTC alarm
 To wake up from the Stop mode with an RTC Tamper or time stamp event, it is necessary to:
 a) Enable the RTC TimeStamp Interrupt in the RTC_CR register or the RTC Tamper Interrupt in the RTC_TCR register
 b) Configure the RTC to detect the tamper or time stamp event
 To wake up from the Stop mode with an RTC Wakeup event, it is necessary to:
 ￼162/980
 a) b)
 Enable the RTC Wakeup Interrupt in the RTC_CR register Configure the RTC to generate the RTC Wakeup event

 To enable RTC interrupt(s), the following sequence is required:
 1.
 2. 3.
 Configure and enable the EXTI line(s) corresponding to the RTC event(s) in interrupt mode and select the rising edge sensitivity.
 Configure and enable the RTC IRQ channel in the NVIC. Configure the RTC to generate RTC interrupt(s).
 */

void RTC_nextModoloMinutes(Time_t* time, uint8_t modulo, uint8_t offset) {
	time->seconds = 0;
	time->minutes = time->minutes + modulo;
	time->minutes -= time->minutes % modulo;
	time->minutes += offset;
	if (time->minutes >= 60) {
		time->hours++;
		time->minutes -= 60;
		if (time->hours >= 24) {
			time->hours = 0;
		}
	}
}

uint32_t RTC_readBackupRegister(uint8_t index) {
	RTC_unlock();
	uint32_t* ptr = &RTC->BKP0R;
	uint32_t result = *(ptr + index);
	RTC_lock();
	return result;
}

void RTC_writeBackupRegister(uint8_t index, uint32_t data) {
	RTC_unlock();
	uint32_t* ptr = &RTC->BKP0R;
	*(ptr + index) = data;
	RTC_lock();
}

volatile uint8_t RTCHandlerFlag;

void RTC_IRQHandler() {
//	trace_printf("Some RTC IRQ fired %d\n", RTC->ISR);
	RTCHandlerFlag = 1;
}


int secondsOfDay(Time_t* time) {
	int result = time->seconds;
	result += time->minutes * 60;
	result += time->hours * 3600;
	return result;
}

int timeDiffSeconds(Time_t* from, Time_t* to) {
	return secondsOfDay(to) - secondsOfDay(from);
}

// From must be before or equal to to.
int timeAfter_seconds(Time_t* from, Time_t* to) {
	int diff = timeDiffSeconds(from, to);
	if (diff < 0)
		diff += 24 * 60 * 60;
	return diff;
}

uint16_t compactDateTime(DateTime_t* datetime) {
	return (datetime->date.date*24 + datetime->time.hours)*60 + datetime->time.minutes;
}
