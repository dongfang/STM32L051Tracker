/*
 * WSPRTransmitter.h
 *
 *  Created on: Dec 9, 2016
 *      Author: dongfang
 */

#ifndef WSPRTRANSMITTER_H_
#define WSPRTRANSMITTER_H_

void WSPRTransmission();

// These are public,
// because we need them in calibration to verify correct modulation strength.
void WSPR_modulate(uint8_t symbol);
void WSPR_initGPIO();
void WSPR_shutdownGPIO();

#endif /* WSPRTRANSMITTER_H_ */
