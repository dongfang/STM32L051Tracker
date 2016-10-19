/*
 * Trace.c
 *
 *  Created on: Jun 2, 2016
 *      Author: dongfang
 */

#include <stdio.h>
#include "stm32l0xx.h"

#define TRACE_USART USART1

void trace_initialize() {
#if defined(TRACE)
	// USART clock
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);

	// and GPIO clock too
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);

	// Set to alternate function
	GPIOA->MODER = (GPIOA->MODER & ~(3 << (2 * 9))) | (2 << (2 * 9));

	// Set to moderate speed
	// GPIOB->MODER = (GPIOB->MODER & ~(3<<(2*TRACE_USART_TX_BIT))) | 2<<(2*TRACE_USART_TX_BIT);

	// USART1 TX is AF4
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(15 << (4 * 1))) | (4 << (4 * 1));

	// TRACE_USART->CR2 = 0; // that is default anyway.
	// TRACE_USART->CR3 = 0; // if using DMA: USART_CR3_DMAR |  USART_CR3_DMAT

	uint32_t brr = 16E6 / 57600;
	TRACE_USART->BRR = brr;

	// Enable transmitter, and the usart itself.
	TRACE_USART->CR1 = USART_CR1_TE | USART_CR1_UE;
#endif
}

// ----------------------------------------------------------------------------

// This function is called from _write() for fd==1 or fd==2 and from some
// of the trace_* functions.
#if defined(TRACE)
#endif

inline void doSend(char c);

void doSend(char c) {
	if (c == '\n')
		doSend('\r');
	while ((TRACE_USART->ISR & USART_ISR_TXE) == 0)
		; // Wait for TXE to assert
	TRACE_USART->TDR = c; // Write transmit register
}

ssize_t trace_write(const char* buf /* __attribute__((unused)) */,
		size_t nbyte /*__attribute__((unused))*/) {
#if defined(TRACE)
	ssize_t cnt = 0;
	while (cnt < nbyte) {
		doSend(buf[cnt++]); // Write transmit register
	}
	return (ssize_t) nbyte; // all characters successfully sent
#endif
	return -1;
}
