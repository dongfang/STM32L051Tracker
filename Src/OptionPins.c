/*
 * OptionPins.c
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */

#include "OptionPins.h"
#include "stm32l0xx.h"

 uint8_t isCalibrateOption() {
	return (OPTIONS_PORT->IDR & (1<<CALIBRATE_PIN)) == 0;
}

 uint8_t isGroundTestOption() {
	return (OPTIONS_PORT->IDR & (1<<GROUNDTEST_PIN)) == 0;
}

 uint8_t isResetLogsOption() {
	return (OPTIONS_PORT->IDR & (1<<RESETLOGS_PIN)) == 0;
}
