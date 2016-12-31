/* Includes ------------------------------------------------------------------*/
#include "Globals.h"
#include "NVM.h"
#include "RTC.h"
#include <stdint.h>
#include <stm32l051xx.h>
#include <stm32l0xx.h>
#include <core_cm0plus.h>
#include <string.h>
#include "Types.h"
#include "Systime.h"
#include "Calibration.h"
#include "APRS.h"
#include "PrecisionTimer.h"
#include "Power.h"
#include "GPS.h"
#include "PLL.h"
#include "WSPR.h"
#include "ADC.h"
#include "LED.h"
#include "OptionPins.h"

SysState_t sysState;

void initGeneralIOPorts() {
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

	GPIOA->BSRR = (1 << 7) | (1 << 15); // Turn off GPS and transmitter in advance.

	// Modulation voltage divider is off.
	GPIOB->BRR = (1 << 8);

	// GPS control and transmitter control are output open drain.
	GPIOA->OTYPER |= (1 << 7) | (1 << 15); // GPS power and HF voltage divider are open drain.
	GPIOA->MODER = (GPIOA->MODER & ~((3 << (7 * 2)) | (3 << (15 * 2))))
			| ((1 << (7 * 2)) | (1 << (15 * 2)));

	// Modulation voltage divider
	// GPIOB->BRR = 1<<8; // default anyway.
	GPIOB->MODER = (GPIOB->MODER & ~(3 << (8 * 2))) | (1 << (8 * 2));

	// All pullups.
	OPTIONS_PORT->PUPDR |= ((1 << (CALIBRATE_PIN * 2))
			| (1 << (GROUNDTEST_PIN * 2)) | (1 << (RESETLOGS_PIN * 2)));
	// All inputs.
	OPTIONS_PORT->MODER &= ~((3 << (CALIBRATE_PIN * 2))
			| (3 << (GROUNDTEST_PIN * 2)) | (3 << (RESETLOGS_PIN * 2)));

}

/* The only situations definitely needing to be avoided are:
 * 1) Repetitive UVLO at GPS, blasting GPS backup
 * 2) Repetitive UVLO at WSPR, cutting short transmissions
 * 3) Staying in overly conservative settings forever.
 * Maybe simply - there is V_min_device
 * If device attempt goes okay, subtract (v_end - v_uvlo) / 2 from V_min_device
 * else add 0.1 to V_min_device (do that from the start, really).
 */

void RTCWakeupExperiment() {
	Time_t time;
	RTC_read(&time);
	RTC_nextModoloMinutes(&time, 2, 0);
	RTC_scheduleAlarmB(&time);

	// EXTI->PR = 1<<17; // does that clear anything? No...
	// RTC->ISR &= ~RTC_ISR_ALRBF;

	// Prepare WFI'
	// ULP: Internal reference is switched off.
	// FWU: At wakeup, there is no wait for the reference to stabilize.
	// To check the reference, there is the VREFINTRDYF flag in the PWR_CSR register.
	PWR->CR = (PWR->CR & ~3) | PWR_CR_LPSDSR | PWR_CR_ULP | PWR_CR_FWU;
	SCB->SCR |= 4; // Do STOP.

	int isr;
	do {
		__WFE();
		isr = RTC->ISR & (RTC_ISR_ALRBF);
	} while (!isr);
}

//extern volatile uint8_t RTCHandlerFlag;

void selfCalibrate() {
	GPS_start();
	LED_on();
	timer_sleep(500);
	LED_off();
	selfCalibratePLLTrim();
	verifyRTCCalibration();
	verifyWSPRModulation(12);
}

int main(void) {
	initGeneralIOPorts();

	//setMediumPerformanceCoreVoltage();
	//setHighPerformanceCoreVoltage();

	PLL_shutdown();

	sysState = isGroundTestOption() ? GROUNDTEST :
				isCalibrateOption() ? CALIBRATION : FLIGHT;

	LED_init(sysState == FLIGHT);

	RTC_init();

	// }
	// if (isCalibrateOption()) {
	//	selfCalibrate();
	// }

	Time_t alarmTime = { .hours = 12, .minutes = 0, .seconds = 0 };
	RTC_scheduleAlarmB(&alarmTime);

	static uint8_t gpsOrWSPR;
	// static uint8_t clockWasSet;

	while (1) {
		ADC_init();
		switchTo1MHzMSI(); // just to get systick going.

		ADC_measureVddAndTemperature();
		ADC_updateVoltages();

	    float energy = vBattery + vSolar/2;

		if (energy >= 2.75) {
			if (gpsOrWSPR) {
				// Do WSPR. First, do some APRS blah blah to kill time. Seriously, that's why.
				switchTo8MHzHSI();
				uint8_t sentOne = 0;
				for (uint8_t i = 0; i < sizeof(latestAPRSRegions); i++) {
					if (latestAPRSRegions[i]) {
						APRS_transmitMessage(VHF, TELEMETRY_MESSAGE,
								APRS_WORLD_MAP[i].frequency);
						sentOne = true;
						break;
					}
				}
				if (!sentOne) {
					APRS_transmitMessage(VHF, TELEMETRY_MESSAGE,
					DIAGNOSTICS_APRS_FREQUENCY);
				}

				if (RTC_readBackupRegister(0)) {
					doWSPR(THIRTY_M);
					ADC_updateVoltages();

					switchTo8MHzHSI();
					for (uint8_t i = 0; i < sizeof(latestAPRSRegions); i++) {
						if (latestAPRSRegions[i]) {
							APRS_transmitMessage(VHF, TELEMETRY_MESSAGE,
									APRS_WORLD_MAP[i].frequency);
							break;
						}
					}
				}
			} else {
				switchTo1MHzMSI();
				if (GPSCycle_voltageLimited()) {
				//  if (1) {
					RTC_writeBackupRegister(0, 1);

					switchTo8MHzHSI();
					APRS_frequenciesFromPosition(&lastNonzeroPosition,
							latestAPRSRegions, latestAPRSCores);

					for (uint8_t i = 0; i < sizeof(latestAPRSRegions); i++) {
						if (latestAPRSRegions[i]) {
							APRS_transmitMessage(VHF,
									COMPRESSED_POSITION_MESSAGE,
									APRS_WORLD_MAP[i].frequency);
						}
					}
				}
			}
		}

		// Log playback irrespective of whether gps or aprs, hm.
		gpsOrWSPR = !gpsOrWSPR;

		switchTo8MHzHSI();
		for (uint8_t i = 0; i < sizeof(latestAPRSCores); i++) {
			if (latestAPRSCores[i]) {
				uint8_t m = 0;
				while (m < 3
						&& m
								< playbackFlightLog(VHF,
										APRS_WORLD_MAP[i].frequency) / 3)
					m++;
			}
		}

		switchTo1MHzMSI();
		ADC_shutdown();

		RTC_read(&alarmTime);
		RTC_nextModoloMinutes(&alarmTime, 2, 0);
		RTC_scheduleAlarmA(&alarmTime);

		/*
		 * SCR:
		 * Bit4: Whether non enabled interrupts can wake up from WFE
		 * Bit2: SLEEPDEEP
		 * Bit1: SLEEPONEXIT (handler-only mode)
		 */

		if (sysState == FLIGHT) {
			PWR->CR = (PWR->CR & ~3) | PWR_CR_LPSDSR | PWR_CR_ULP | PWR_CR_FWU;
			SCB->SCR |= 4; // Deep sleep.
			uint32_t isr;
			do {
				__WFE();
				isr = RTC->ISR & (RTC_ISR_WUTF | RTC_ISR_ALRAF | RTC_ISR_ALRBF);
			} while (!isr);

			RTC->ISR &= ~isr;
		} else {
			PWR->CR = (PWR->CR & ~3) | PWR_CR_LPSDSR | PWR_CR_ULP | PWR_CR_FWU;
			SCB->SCR &= ~4; // do not deepsleep.
			uint32_t isr;
			do {
				isr = RTC->ISR & (RTC_ISR_WUTF | RTC_ISR_ALRAF | RTC_ISR_ALRBF);
			} while (!isr);
		}
	}
}
