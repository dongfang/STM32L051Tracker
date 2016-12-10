/*
 * PWM.c
 *
 *  Created on: Sep 12, 2016
 *      Author: dongfang
 */

#include "stm32l0xx.h"

// f = 125kHz sucker: LC = 1.62E-12 HF. eg. C=100nF, L=16.2uH, or C=1uF, L=1.62uH

static const uint8_t SINTAB[] = { 0x20, 0x23, 0x26, 0x29, 0x2c, 0x2e, 0x31,
		0x33, 0x36, 0x38, 0x3a, 0x3b, 0x3d, 0x3e, 0x3e, 0x3f, 0x3f, 0x3f, 0x3e,
		0x3e, 0x3d, 0x3b, 0x3a, 0x38, 0x36, 0x33, 0x31, 0x2e, 0x2c, 0x29, 0x26,
		0x23, 0x20, 0x1c, 0x19, 0x16, 0x13, 0x11, 0xe, 0xc, 0x9, 0x7, 0x5, 0x4,
		0x2, 0x1, 0x1, 0x0, 0x0, 0x0, 0x1, 0x1, 0x2, 0x4, 0x5, 0x7, 0x9, 0xc,
		0xe, 0x11, 0x13, 0x16, 0x19, 0x1c, };

volatile uint8_t AFSKTransmitting = 0;

void AFSK_init() {
	AFSKTransmitting = 1;

	// Use Timer2 CH1 for PWM gen.
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
	// SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);
	enableGPIOClock(RCC_IOPENR_GPIOBEN);
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMAEN);

	// Set PB3 (I think) to alternate function
	GPIOB->MODER = (GPIOB->MODER & ~(3 << (2 * 3))) | 2 << (2 * 3);
	// GPIOB->MODER = (GPIOB->MODER & ~(3 << (2 * 3))) | 1 << (2 * 3);

	// TIM2 Ch2 is AF2
	GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(15 << (4 * 3))) | (2 << (4 * 3));

	// Center-aligned mode 3.
	//TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CMS_1;
	TIM2->CR1 = TIM_CR1_ARPE;
	TIM2->PSC = 0;

	// Div by 64
	TIM2->ARR = 63;
	TIM2->CCR2 = 32;

	TIM2->CCMR1 |= TIM_CCMR1_OC2M; // PWM mode 2
	TIM2->CCER = 0x30;

	TIM2->CR1 |= TIM_CR1_CEN;

	// Use Timer6 for sintab play (a 1200 or 2200 Hz full playback of the wave)
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC = 28; // 28 , 51
	TIM6->ARR = 1;
	TIM6->DIER = TIM_DIER_UDE;
	TIM6->CR1 = TIM_CR1_CEN; // | TIM_CR1_DIR; // or set ARPE ??

	DMA1_Channel2->CNDTR = sizeof(SINTAB);
	DMA1_Channel2->CPAR = (uint32_t) &TIM2->CCR2;
	DMA1_Channel2->CMAR = (uint32_t) SINTAB;
	DMA1_Channel2->CCR = DMA_CCR_PL_1 | DMA_CCR_MINC | DMA_CCR_CIRC
			| DMA_CCR_DIR | DMA_CCR_PSIZE_1;

	DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~DMA_CSELR_C2S) | (9 << 4); // Ch2 is TIM6UP

	// Use Timer22 as driver of playback frequency change, with LSE as input
	// Enable TIM22 clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;

	TIM22->PSC = 0;
	TIM22->ARR = 26;

	// Supposed to route LSE to ETR
	TIM22->OR |= TIM22_OR_ETR_RMP_0 | TIM22_OR_ETR_RMP_1;
	// Select external clock as source.
	TIM22->SMCR |= TIM_SMCR_ECE;

	TIM22->DIER = TIM_DIER_UIE;
	NVIC_SetPriority(TIM22_IRQn, 0);
	// AFSK_subtrim = 0;
	NVIC_EnableIRQ(TIM22_IRQn);

	// Enable DMA
	DMA1_Channel2->CCR |= DMA_CCR_EN;

	// Enable Timer22
	TIM22->CR1 |= TIM_CR1_CEN | TIM_CR1_DIR;
}

void AFSK_shutdown() {
	AFSKTransmitting = 0;

	// TIM22 deinit.
	TIM22->CR1 &= ~TIM_CR1_CEN;

	NVIC_DisableIRQ(TIM22_IRQn);
	TIM22->DIER &= ~TIM_DIER_UIE;
	RCC->APB2ENR &= ~RCC_APB2ENR_TIM22EN;

	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	RCC->AHBENR &= ~RCC_AHBENR_DMAEN;

	TIM6->CR1 &= ~TIM_CR1_CEN;
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;

	TIM2->CR1 &= ~TIM_CR1_CEN;
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
}
