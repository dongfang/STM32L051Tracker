/*
 * PLL.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_PLL_H_
#define INC_PLL_H_

#include "Types.h"
#include <stdint.h>

// the implementation PLL type decides what PLLSetting_t actually is.
#include "CDCE913.h"

#define PLL_XTAL_FREQUENCY PLL_XTAL_DEFAULT_FREQUENCY

// Implementation type of PLL setting (well, one of them)
typedef CDCE913_PLL_Setting_t PLL_Setting_t;

/*
 * PLL options for different frequencies.
 */

boolean PLL_bestPLLSetting(
		uint32_t oscillatorFrequency,
		uint32_t desiredFrequency,
		double maxError,
		PLL_Setting_t* result) ;

int8_t PLL_bestTrim(double desiredTrim);

void setPLL(uint8_t output, const PLL_Setting_t* setting);

void PLL_setXOPassthroughMode(uint8_t trim);

void PLL_setDirectModeWithDivision(uint8_t trim, uint16_t pdiv);

// Shut em down
void PLL_shutdown();

int16_t PLL_oscillatorError(uint32_t measuredFrequency);

// Print settings.
void PLL_printSettings();

#define DIRECT_2m_HARDWARE_OUTPUT CDCE913_OutputMode_OUTPUT_13
#define HF_30m_HARDWARE_OUTPUT CDCE913_OutputMode_OUTPUT_12

#endif /* INC_PLL_H_ */
