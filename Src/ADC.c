/*
 * ADC_DMA.c
 *
 *  Created on: May 14, 2015
 *      Author: dongfang
 */

/* Includes ------------------------------------------------------------------*/

#include "ADC.h"
#include "stm32l0xx.h"
#include "LED.h"
#include "Globals.h"
#include "Setup.h"

float VDDa;
int8_t temperature  __attribute__((section (".noinit")));
float vBattery  __attribute__((section (".noinit")));
float vSolar  __attribute__((section (".noinit")));
uint8_t isADCUsingHSI;

uint16_t ADC_measure(int channel);

void ADC_measureVddAndTemperature() {
	ADC->CCR |= ADC_CCR_TSEN | ADC_CCR_VREFEN;
	for (volatile uint32_t i=0; i<30000; i++) {}

	// we should apparently use 10us sampling and there is a 10us startup time.
	uint16_t n_vrefint = ADC_measure(17);
	uint16_t n_temperature = ADC_measure(18);
	ADC->CCR &= ~(ADC_CCR_TSEN | ADC_CCR_VREFEN);

	uint16_t vrefintCal = *(uint16_t*) 0x1FF80078;
	VDDa = 3.0 * vrefintCal / n_vrefint;

	uint16_t cal30 = *(uint16_t*) 0x1FF8007A;
	uint16_t cal130 = *(uint16_t*) 0x1FF8007E;

	float compensatedTempADCValue = VDDa * n_temperature / 3.0;
	// float compensatedTempADCValue = 3.0 * n_temperature / VDDa;

	temperature = 30
			+ 100.0 / (cal130 - cal30) * (compensatedTempADCValue - cal30);
}

float ADC_measureBatteryVoltage() {
	// Assume ADC init etc. was done already, and VDDa valid.
	uint16_t n_vBatt = ADC_measure(8);
	return (n_vBatt * BATT_ADC_FACTOR * VDDa) / 4096;
}

float ADC_measureSolarVoltage() {
	// Assume ADC init etc. was done already, and VDDa valid.
	uint16_t n_vSolar = ADC_measure(5); // The trace is missing on PCBs!
	return (n_vSolar * VDDa) / 4096;
}

void ADC_updateVoltages() {
	vBattery = ADC_measureBatteryVoltage();
	vSolar = ADC_measureSolarVoltage();
}

/*
 * Should be retained until ADC periph is reset, or STANDBY mode.
 * ADC volt. reg. disabled, STOP mode, ... no prob.
 */
int ADC_init() {
	isADCUsingHSI =1;

	RCC->CR = RCC_CR_HSION;
		while (!(RCC->CR & RCC_CR_HSIRDY)) {
	}
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	//int alreadyEnabled = ADC1->CR & (ADC_CR_ADEN);
	//alreadyEnabled |= ADC1->CFGR1 & ADC_CFGR1_DISCEN;

	ADC1->CR = ADC_CR_ADCAL; // automatically sets ADVREGEN
	int wait = 100;

	while ((ADC1->ISR & ADC_ISR_EOCAL) == 0 && wait) {
		wait--;
	}

	int cfactor = ADC1->CALFACT;

	ADC1->CR |= ADC_CR_ADEN;

	// 2MHz
	ADC->CCR = (ADC->CCR & ~ADC_CCR_PRESC) | ADC_CCR_PRESC_1 | ADC_CCR_LFMEN;

	ADC1->SMPR = 2; // 7.5 cycles sample.

	// we have to wait till ADRDY is set. Maybe we can configure stuff in the meantime?
	// while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
	// }
	// ADC1->CFGR2 &= ~(ADC_CFGR2_CKMODE); // Use async clock. This is default anyway.
	// ADC1->CFGR2 |= ADC_CFGR2_OVSS_2 | ADC_CFGR2_OVSS_0; // 5 bit oversampling shift
	// ADC1->CFGR2 |= 4*4; // 32x oversampling.
	// ADC1->CFGR2 |= ADC_CFGR2_OVSE; // didn't really bring much.
	// OVSE:OversamplerEnable

	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;

	return cfactor;
}

uint16_t ADC_measure(int channel) {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Clear it.
	// ADC1->ISR |= ADC_ISR_ADRDY;
	// ADC1->CFGR1 |= ADC_CFGR1_AUTOFF; // hmm this prevents the ADC from becoming ready.

	// Sample time: We need 10us or more, that is 5 cycles or more.
	ADC1->CHSELR = 1 << channel; // channel number.

	// while (1) {
	ADC1->CR |= ADC_CR_ADSTART;
	int count = 0;

	// wait till done:
	while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
		count++;
	}

	int result = ADC1->DR;

	return result;
}

void ADC_shutdown() {
	// We just assume there are no ongoing conversions.
	ADC1->CR |= ADC_CR_ADDIS;
	while(ADC1->CR & ADC_CR_ADEN) {}
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// This SHOULD not cause the clock to stop if uses as sysclock.
	RCC->CR &= ~RCC_CR_HSION;
	isADCUsingHSI = 0;
}
