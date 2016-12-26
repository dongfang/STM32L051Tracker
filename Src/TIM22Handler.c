/*
 * TIM22Handler.c
 *
 *  Created on: Oct 18, 2016
 *      Author: dongfang
 */

#include "stm32l0xx.h"
#include "LED.h"

/*
 * Shared handler for both AFSK and PrecisionTimer.
 */

static uint8_t AFSK_subtrim __attribute__((section (".noinit")));
extern volatile uint16_t packet_size;
extern volatile uint16_t packet_cnt;
extern volatile uint8_t packet[];

extern volatile uint32_t PT_extendedCaptureValue;
extern volatile uint32_t PT_minCaptureResolution;
extern volatile int32_t PT_captureCount;
extern volatile uint8_t PT_captureState;

static uint32_t PT_prevCaptureValue  __attribute__((section (".noinit")));

extern volatile uint8_t AFSKTransmitting;
extern volatile uint8_t TIM22Updated;
//extern volatile uint32_t numCaptureCycles;

void TIM22_IRQHandler(void) {
	static int subcount;
	if (TIM22->SR & TIM_SR_CC1IF) { // Capture/Compare 1
		// We can pretty much assume the only other possibility was PrecisionTimer.
		uint32_t captureValue;

		// This should reset the interrupt flag too.
		captureValue = TIM22->CCR1;

		if (TIM22->SR & TIM_SR_CC1OF) {
			// Overcapture! The previous capture result was not picked up before the next one arrived.
			// This normally never happens, but if somebody puts a breakpoint in the interrupt handler
			// or wait loop, it may happen.
			// LED_faultCode(LED_FAULT_OVERCAPTURE);
			TIM22->SR &= ~TIM_SR_CC1OF;
		}

		if (PT_captureState == 0) {
			// first time capture. Just save PT_prevCaptureValue
			PT_captureState = 1;
		} else if (PT_captureState == 1) {
			// continue capture till good enough.
			int delta = captureValue - PT_prevCaptureValue;
			PT_extendedCaptureValue += delta;
			PT_captureCount++;
			if (PT_extendedCaptureValue >= PT_minCaptureResolution)
				PT_captureState = 2;
		}

		PT_prevCaptureValue = captureValue;
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
					TIM6->PSC = (51 + 28) - TIM6->PSC; // if it were 28, make it 51. If it were 51, make it 28.
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
		}

		// if (PT_captureCount >= 0) PT_extendedCaptureValue += 60000;
		if (PT_captureState == 1)
			PT_extendedCaptureValue += 1 << 16;
		// This is for the WSPR transmitter.
		TIM22Updated = 1;
	}
}
