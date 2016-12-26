/*
 * OptionPins.h
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */

#ifndef OPTIONPINS_H_
#define OPTIONPINS_H_

#include <stdint.h>
#include "stm32l0xx.h"

#define OPTIONS_PORT GPIOA
#define CALIBRATE_PIN 8
#define GROUNDTEST_PIN 11
#define RESETLOGS_PIN 12

void initOptionPins();

uint8_t isCalibrateOption();
uint8_t isGroundTestOption();
uint8_t isResetLogsOption();

#endif /* OPTIONPINS_H_ */
