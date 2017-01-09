#include "stm32l0xx.h"
#include "CDCE913.h"
#include "Types.h"
#include "Calibration.h"
#include "Globals.h"
#include "LED.h"

// 7 bit address not including that stinky R/!W bit.
#define CDCE913_I2C_ADDR 0b1100101
#define CDCE913_I2C_ADDR_PROG (CDCE913_I2C_ADDR & 3)
#define I2C_TIMEOUT 25
extern int generalShit ;
extern void I2C1_GPIO_Config();
extern boolean I2C1_writeByte(uint8_t DeviceAddress, uint8_t registerAddress,
		uint8_t data);
extern uint8_t I2C1_readByte(uint8_t deviceAddress, uint8_t registerAddress);

void CDCE913_initInterfaceIfNecessary() {
	if (!(GPIOB->ODR & (1 << 8))) {
		I2C1_GPIO_Config();
		// Modulation voltage divider is on.
		GPIOB->BSRR = (1 << 8);
	}
}

uint8_t CDCE913_read(uint8_t registerAddress) {
	return I2C1_readByte(CDCE913_I2C_ADDR << 1, registerAddress | (1 << 7));
}

void CDCE913_write(uint8_t registerAddress, uint8_t data) {
	I2C1_writeByte(CDCE913_I2C_ADDR << 1, registerAddress | (1 << 7), data);
}

static void CDCE913_setPLLValue(const CDCE913_PLL_Setting_t* setting) {
	uint8_t pllRange = 0;

	// uint8_t r2 = I2C_read(0x02);
	// I2C_write(0x02, (r2 & 0b11111100) | (setting->pdiv >> 8));
	// I2C_write(0x03, (uint8_t) setting->pdiv);

	CDCE913_write(0x18, (uint8_t) (setting->N >> 4));
	CDCE913_write(0x19, (setting->N << 4) | (setting->R >> 5));
	CDCE913_write(0x1a, (setting->R << 3) | (setting->Q >> 3)); // Q upper bits
	CDCE913_write(0x1b, (setting->Q << 5) | (setting->P << 2) | pllRange);

	CDCE913_write(0x1c, (uint8_t) (setting->N >> 4));
	CDCE913_write(0x1d, (setting->N << 4) | (setting->R >> 5));
	CDCE913_write(0x1e, (setting->R << 3) | (setting->Q >> 3)); // Q upper bits
	CDCE913_write(0x1f, (setting->Q << 5) | (setting->P << 2) | pllRange);
}

// whichOutput:
// 0 : Shut down
// 1 : Use Y1 and ground the other two.
// 2 : Use Y2 and ground the other two.
// 3 : Use Y3 and ground the other two.
// 4 : Feed xtal to Y1 with division.
// The direct mode is not supported from here.

static void CDCE913_enableOutput(CDCE913_OutputMode_t whichOutput,
		uint16_t pdiv) {
	uint8_t r1 = CDCE913_I2C_ADDR_PROG;
	uint8_t r2 = 0;
	uint8_t r0x14 = 0;
	switch (whichOutput) {
	case CDCE913_OutputMode_SHUTDOWN:
		// CDCEL913_disableOutputPower();
		r1 |= 0b00010000;	// power down
		r2 = 0;
		r0x14 = 0;
		break;
	case CDCE913_OutputMode_OUTPUT_1_PLL:
		r1 |= 0b00000100; 	// power on, VCXO and default address
		r2 = 0b10111100 | (pdiv >> 8);
		r0x14 = 0b01101010;	// Disable Y2, Y3 to low
		CDCE913_write(0x03, pdiv);
		break;
	case CDCE913_OutputMode_OUTPUT_12:
		r1 |= 0b00000100; 	// power on, VCXO and default address
		r2 = 0b10101000;	// Disable Y1
		r0x14 = 0b01101111;	// Enable Y2, Y3
		CDCE913_write(0x16, (1 << 7) | pdiv);
		CDCE913_write(0x17, 0);	// Keep Y3 reset.
		break;
	case CDCE913_OutputMode_OUTPUT_13:
		r1 |= 0b00000100; 	// power on, VCXO and default address
		r2 = 0b10101000;	// Disable Y1
		r0x14 = 0b01101111;	// Enable Y2, Y3
		CDCE913_write(0x16, 1 << 7);	// Keep Y2 reset.
		CDCE913_write(0x17, pdiv);
		break;
	case CDCE913_OutputMode_OUTPUT_1_BYPASS:
		r1 |= 0b00000100; 	// power on, VCXO and default address
		r2 = 0b00111100 | (pdiv >> 8);
		r0x14 = 0b11101010;	// Disable Y2, Y3 to low
		CDCE913_write(0x03, (uint8_t) pdiv);
		break;
	case CDCE913_OutputMode_XO_PASSTHROUGH:
		r1 |= 0b00000100; 	// power on, VCXO and default address
		r2 = 0b10111100 | (pdiv >> 8);
		r0x14 = 0b11101010;	// Disable Y2, Y3 to low
		CDCE913_write(0x03, (uint8_t) pdiv);
		break;
	}
	CDCE913_write(1, r1);
	CDCE913_write(0x02, r2);
	CDCE913_write(0x14, r0x14);
}

void PLL_shutdown() {
	CDCE913_initInterfaceIfNecessary();
	CDCE913_enableOutput(CDCE913_OutputMode_SHUTDOWN, 0);

	// Modulation voltage divider is off.
	GPIOB->BRR = (1 << 8);
}

// Always on output #1
void PLL_setBypassModeWithDivision(uint8_t trim, uint16_t pdiv) {
	CDCE913_initInterfaceIfNecessary();
	CDCE913_enableOutput(CDCE913_OutputMode_OUTPUT_1_BYPASS, pdiv);
	CDCE913_write(5, trim << 3); 	// Cap. in pF.
}

// Always on output #1
void PLL_setXOPassthroughMode(uint8_t trim) {
	CDCE913_initInterfaceIfNecessary();
	CDCE913_enableOutput(CDCE913_OutputMode_XO_PASSTHROUGH, 1);
	CDCE913_write(5, trim << 3); 	// Cap. in pF.
}

void setPLL(CDCE913_OutputMode_t output, const CDCE913_PLL_Setting_t* setting) {
	CDCE913_initInterfaceIfNecessary();
	// Turn on Y1 output in all cases and PLL out
	// I2C_write(1, 0b0101);
	// I2C_write(2, (1 << 7) | (0b1111 << 2));
	CDCE913_enableOutput(output, setting->pdiv);
	CDCE913_setPLLValue(setting);
	// trace_printf("Using trim %d\n", setting->trim);
	CDCE913_write(5, setting->trim << 3); 	// Cap. in pF.
	// PLL_printSettings();
}

// 0-->0
// 1-->0	// 1 is wrong, should be 0!
// 2-->1
// 3-->1
// 4-->2
// etc
static uint8_t ilog2(uint16_t N) {
	uint8_t result = 0;
	N >>= 1; // 0->0, 1->0, which both will correctly return 0
	while (N) {
		N >>= 1;
		result++;
	}
	return result;
}

void setPQR(uint16_t N, uint16_t M, CDCE913_PLL_Setting_t* result) {
	int8_t P = 4 - ilog2(N / M);
	if (P < 0)
		P = 0;
	result->P = P;
	uint32_t Nprime = N * (1 << P);
	result->Q = Nprime / M;
	result->R = Nprime - M * result->Q;
}

boolean findM(int16_t N, double desiredMultiplication, uint16_t* M,
		double* signedError) {
	// desiredMultiplication = N/M
	// M = N/desiredMultiplication
	int16_t Mlo = N / desiredMultiplication;
	// yeah Mhi will be wrong in case of an exact division.
	// But in that case, the Mhi result will be discarded anyway as the Mlo
	// one is "infinitely better" so no reason to make a big deal of it.
	int16_t Mhi = Mlo + 1;

	if (Mlo > 0 && Mlo < 512) {
		// Err = (N/M) / desired - 1
		// = N / (M * desired) - 1
		// Mlo is too low (never too high) so the error should be > 0
		double loError = (double) N / (Mlo * desiredMultiplication) - 1;
		// if (loError <0) trace_printf("SURPRISE!! loError < 0!\n");

		if (Mhi < 512) {
			// Mhi is too high (never too low) so the error should be < 0
			double hiError = (double) N / (Mhi * desiredMultiplication) - 1;
			// if (hiError >0) trace_printf("SURPRISE!! hiError > 0!\n");

			if (loError > -hiError) {
				*signedError = hiError;
				*M = Mhi;
				// trace_printf("Took hi M with error %d\n", (int)(hiError*1E7));
			} else {
				*signedError = loError;
				*M = Mlo;
				// trace_printf("Took lo M with error %d\n",  (int)(loError*1E7));
			}
		}

		return true;
	}

	return false;
}

int8_t PLL_bestTrim(double desiredTrim) {
	int32_t desiredPP10M = desiredTrim * 1E7;
	int32_t bestError = 1500;
	int8_t bestIndex = -1;

	for (uint8_t i = PLL_MIN_TRIM; i <= PLL_MAX_TRIM; i++) {
		int32_t test = desiredPP10M - getPLLTrimCalibration(i);
		if (test < 0)
			test = -test;
		if (test < bestError) {
			bestError = test;
			bestIndex = i;
		}
	}

	return bestIndex;
}

double PLL_bestPLLSetting(
		uint32_t oscillatorFrequency,
		uint32_t desiredFrequency,
		// double maxError,
		// uint8_t maxTrimOffCenter,
		CDCE913_PLL_Setting_t* result) {

	// okay the rounding is off if somebody wants 100MHz. But nobody wants that.
	int Pdivmin = 100E6 / (double) desiredFrequency + 1;
	int pdivMax = 200E6 / (double) desiredFrequency;
	double bestError = -1;
	uint8_t bestTrimOffCenter = 100;
	uint8_t goodEnoughTrimOffCenter = 2;

	generalShit = 110;

	for (int Pdiv = Pdivmin;
			Pdiv <= pdivMax && bestTrimOffCenter > goodEnoughTrimOffCenter;
			Pdiv++) {
		generalShit++;

		double desiredMultiplication = ((double) desiredFrequency * Pdiv)
				/ oscillatorFrequency;
		if (desiredMultiplication < 1)
			continue;
		uint16_t Nmax = desiredMultiplication * 511;
		if (Nmax > 4095) {
			Nmax = 4095;
		}

		// trace_printf("Trying Pdiv : %d, N from 1 to %d\n", Pdiv, Nmax);
		uint16_t M;
		// Just cut it short as we get within decent trim range, which is about +-30ppm
		for (uint16_t N = 1; N <= Nmax && bestTrimOffCenter > goodEnoughTrimOffCenter; N++) {
			double signedError;
			double unsignedError;

			WWDG_pat();

			if (findM(N, desiredMultiplication, &M, &signedError)) {
				// If we divided too much (freq too low), signedError is negative. We need add a little speed on the trim.
				// Opposite for dividing too little.
				int8_t trim = PLL_bestTrim(-signedError);
				int8_t signedOffCenter = PLL_CENTER_TRIM - trim;
				uint8_t trimOffCenter =
						signedOffCenter < 0 ?
								-signedOffCenter : signedOffCenter;

				if (trimOffCenter < bestTrimOffCenter) {
					result->pdiv = Pdiv;
					result->N = N;
					result->M = M;
					setPQR(N, M, result);
					result->trim = trim;
					bestError = signedError + getPLLTrimCalibration(trim) / 1E7;
					bestTrimOffCenter = trimOffCenter;
				}
			}
		}
	}

	return bestError;
}

int16_t PLL_oscillatorError(uint32_t measuredFrequency) {
	int32_t result = (int32_t) measuredFrequency
			- (int32_t) PLL_XTAL_NOMINAL_FREQUENCY;
	// trace_printf("Osc error: %d\n", result);
	if (result > 32767)
		result = 32767;
	else if (result < -32768)
		result = -32768;
	return result;
}

/*
void PLL_printSettings() {
	uint16_t r5 = CDCE913_read(0x05);
	uint16_t r18 = CDCE913_read(0x18);
	uint16_t r19 = CDCE913_read(0x19);
	trace_printf("N=%u\n", (r18 << 4) + (r19 >> 4));

	uint16_t r1a = CDCE913_read(0x1a);
	trace_printf("R=%u\n", ((r19 & 0xf) << 5) + (r1a >> 3));

	uint16_t r1b = CDCE913_read(0x1b);
	trace_printf("Q=%u\n", ((r1a & 0x7) << 3) + (r1b >> 5));
	trace_printf("P=%u\n", (r1b & 0b1100) >> 2);

	trace_printf("Trim=%u\n", r5 >> 3);
}
*/

