/*
 * TIM22Handler.c
 *
 *  Created on: Oct 18, 2016
 *      Author: dongfang
 */

#include "stm32l051xx.h"

/*
 * Shared handler for both AFSK and PrecisionTimer.
 */

static uint8_t AFSK_subtrim __attribute__((section (".noinit")));
extern volatile uint16_t packet_size;
extern volatile uint16_t packet_cnt;
extern volatile uint8_t packet[];

extern volatile uint32_t PT_extendedCaptureValue;
extern volatile int32_t PT_captureCount;
static uint32_t PT_firstCaptureValue;

extern volatile uint8_t AFSKTransmitting;
extern volatile uint8_t TIM22Updated;

void TIM22_IRQHandler(void) {
	if (TIM22->SR & TIM_SR_CC1IF) { // Capture/Compare 1
		// We can pretty much assume the only other possibility was PrecisionTimer.
		uint32_t captureValue;

		// This should reset the interrupt flag too.
		captureValue = TIM22->CCR1;

		if (PT_captureCount == -1) {
			PT_firstCaptureValue = captureValue;
		} else {
			PT_extendedCaptureValue += (captureValue - PT_firstCaptureValue);
		}

		PT_captureCount++;
	}
	// If the interrupt cause was update, then AFSK did it.
	if (TIM22->SR & TIM_SR_UIF) {
		// Clear the interrupt flag.
		TIM22->SR &= ~TIM_SR_UIF;
		if (AFSKTransmitting) {

			if (packet_cnt < packet_size) {
				// NRZI impl.
				uint8_t mask = 1 << (packet_cnt & 7);
				uint16_t index = packet_cnt >> 3;
				uint8_t bit = packet[index] & mask;
				if (!bit) { // invert on zero.
					TIM6->PSC = (51 + 28) - TIM6->PSC; // if it were 23, make it 51. If it were 51, make it 23.
				}

				packet_cnt++;
			}

			// Adjust the TIM22 period with some phase jitter, to be about 27.333
			if (AFSK_subtrim < 2) {
				TIM22->ARR = 26;
				AFSK_subtrim++;
			} else {
				TIM22->ARR = 27;
				AFSK_subtrim = 0;
			}
		} else if (PT_captureCount == 0) {
			PT_extendedCaptureValue += 60000;
		}
		TIM22Updated = 1;
	}
}
