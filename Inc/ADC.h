/*
 * ADC.h
 *
 *  Created on: May 14, 2015
 *      Author: dongfang
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

int ADC_init();
void ADC_measureVddAndTemperature();
float ADC_measureBatteryVoltage();
float ADC_measureSolarVoltage();
void ADC_updateVoltages();

#endif /* ADC_H_ */
