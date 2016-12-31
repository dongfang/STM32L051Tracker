/*
 * LED.c
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */
#include "stm32l0xx.h"
#include "LED.h"
#include "Globals.h"

uint8_t LED_init(uint8_t dim) {
	uint8_t before = (LED_PORT->MODER & (3 << (2 * LED_PIN)))
			!= 1 << (2 * LED_PIN);
	if (dim) {
		// init LED for input (we use the pullup to source it)
		LED_PORT->MODER = LED_PORT->MODER & ~(3 << (2 * LED_PIN));
	} else {
		// init LED for output
		LED_PORT->MODER = (LED_PORT->MODER & ~(3 << (2 * LED_PIN))) | 1 << (2 * LED_PIN);
	}
	return before;
}

void LED_on() {
	LED_PORT->BSRR = 1 << LED_PIN; // Set
	LED_PORT->PUPDR |= 1 << (LED_PIN * 2);
}

void LED_faintOn() {
	LED_PORT->PUPDR |= 1 << (LED_PIN * 2);
}

void LED_off() {
	LED_PORT->BRR = 1 << LED_PIN; // Reset
	LED_PORT->PUPDR &= ~(1 << (LED_PIN * 2));
}

void LED_toggle() {
	if (LED_PORT->ODR & (1 << LED_PIN)) {
		LED_off();
	} else {
		LED_on();
	}
}

const char MORSE_CODE[] =
		".-|-...|-.-.|-..|.|..-.|--.|....|..|.---|-.-|.-..|--|-.|---|.--.|--.-|.-.|...|-|..-|...-|.--|-..-|-.--|--..|.----|..---|...--|....-|.....|-....|--...|---..|----.|-----||";

#define DOT 100000
void LED_faultCode(char c) {
	uint8_t offset = 0;
	uint8_t n = c - 'A';
	while (n > 0) {
		if (MORSE_CODE[offset++] == '|')
			n--;
	}

	uint8_t idx;
	int i = 0;
	int realNumReps = (sysState == FLIGHT ? 1 : 5);
	charloop: while (i < realNumReps) {
		i++;
		idx = offset;
		while (1) {
			switch (MORSE_CODE[idx++]) {
			case '-':
				LED_on();
				for (volatile int j = 0; j < DOT * 3; j++) {
				}
				break;
			case '.':
				LED_on();
				for (volatile int j = 0; j < DOT; j++) {
				}
				break;
			default: // end of char.
				LED_off();
				for (volatile int j = 0; j < DOT * 3; j++) {
				}
				goto charloop;
			}
			// break after dot or dash.
			LED_off();
			for (volatile int j = 0; j < DOT; j++) {
			}
		}
	}
}
