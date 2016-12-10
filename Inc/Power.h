/*
 * Power.h
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */

#ifndef POWER_H_
#define POWER_H_

typedef enum {
	GPIO_RCC_A = 0,
	GPIO_RCC_B = 1,
	GPIO_RCC_C = 2,
	GPIO_RCC_D = 3,
	GPIO_RCC_E = 4,
	GPIO_RCC_H = 7
} GPIO_RCC_t;

void enableGPIOClock(GPIO_RCC_t which);
void disableGPIOClock(GPIO_RCC_t which);

#endif /* POWER_H_ */
