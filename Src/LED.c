/*
 * LED.c
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */
#include "stm32l051xx.h"

void LED_on() {
	GPIOB->BSRR = 1<<5; // Set
}

void LED_off() {
	GPIOB->BSRR = 1<<(16+5); // Reset
}

void LED_toggle() {
	if (GPIOB->ODR & (1<<5)) {
		LED_off();
	} else {
		LED_on();
	}
}
