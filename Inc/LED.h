/*
 * LED.h
 *
 *  Created on: Dec 10, 2016
 *      Author: dongfang
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>

#define LED_PORT GPIOB
#define LED_PIN 2

#define LED_FAULT_HARDFAULT 'H'
#define LED_FAULT_PLL_SELF_CAL 'S'
#define LED_FAULT_NOT_CALIBRATED 'C'
#define LED_FAULT_WSPR_DEVIATION_OFF 'W'
#define LED_FAULT_I2C_DEAD 'I'
#define LED_FAULT_LSE_DEAD 'L'
#define LED_FAULT_OVERCAPTURE 'O'
#define LED_FAULT_NO_PLL_SETTING 'P'
#define LED_FAULT_GPS_INBUF_OVERRUN 'U'

uint8_t LED_init(uint8_t dim);
void LED_on();
// void LED_faintOn();
void LED_off();
void LED_toggle();
void LED_faultCode(char c);
#endif /* LED_H_ */
