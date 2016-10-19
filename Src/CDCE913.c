#include "stm32l0xx.h"
#include "CDCE913.h"
#include "Trace.h"
#include "Types.h"

// 7 bit address not including that stinky R/!W bit.
#define CDCE913_I2C_ADDR 0b1100101
#define CDCE913_I2C_ADDR_PROG (CDCE913_I2C_ADDR & 3)

// const int16_t PLL_XTAL_TRIM_PP10M[]; //= PLL_XTAL_TRIM_PP10M_VALUES;

/**
 * Init the H/W for PLL usage. Includes:
 * Timer2 as capture and with external source + GPIO for same
 * I2C
 */
/*
static void CDCE913_initInterfaceIfNecessary() {
	if ((I2C1->CR1) & I2C_CR1_PE) // was it already enabled?
		return;


	/ **I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB7     ------> I2C1_SDA
	* /
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

	// 6 and 7 are set to alternate function mode (10b)
	GPIOB->MODER = (GPIOB->MODER & ~(0b11<<(6*2) | 0b11<<(7*2))) | 0b10<<(6*2) | 0b10<<(7*2);
	// both are OD output type (1)
	GPIOB->OTYPER = GPIOB->OTYPER | 1<<6 | 1<<7;
	// Medium speed
	// GPIOB->OSPEEDR = (GPIOB->OSPEEDR & (0b11<<(10*2) | 0b11<<(9*2))) | GPIO_SPEED_FREQ_MEDIUM<<(10*2) | GPIO_SPEED_FREQ_MEDIUM<<(9*2);
	// Pull up
	GPIOB->PUPDR = (GPIOB->PUPDR & ~ (3<<(6*2) | 3<<(7*2))) | 1<<(6*2) | 1<<(7*2);
	// AF1
	GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0b1111 << (6*4) | 0b1111 << (7*4))) | 1 << (6*4) | 1 << (7*4);

	// 0..7 scll
	// 8..15 sclh
	// 16..19 scadel
	// 20..23 scldel
	// 28..31 presc
	// I2C1->TIMINGR = 0x303D5B; // from MX Cube
	I2C1->TIMINGR = 1<<28 | 0x2 << 16 | 0x4 << 20 | 0xf << 8 | 0x13;

	// Enable periph.
	SET_BIT(I2C1->CR1, I2C_CR1_PE);
}

 int CDCE913_read(uint8_t registerAddress) {
	CDCE913_initInterfaceIfNecessary();

	I2C1->CR2 = (I2C1->CR2 & ~0xff) | CDCE913_I2C_ADDR; //| I2C_CR2_AUTOEND; // read
	I2C1->CR2 = (I2C1->CR2 & ~(0xff << 16)) | 1<<16; // 1 byte.
	I2C1->CR2 |= I2C_CR2_START;	// address

	uint32_t isr;

	while(((isr = I2C1->ISR) & I2C_ISR_TXIS) == 0) {
		// trace_printf("damn, isr is %d\n");
	}

	I2C1->TXDR = registerAddress | 128;

	while((I2C1->ISR & I2C_ISR_TC) == 0) {
	}

	// Restart with read and autoend
	I2C1->CR2 |= I2C_CR2_START | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND;

	while ((I2C1->ISR  & I2C_ISR_RXNE) == 0) {
	}

	int status = I2C1->ISR;

	uint8_t result = I2C1->RXDR;

	while((I2C1->ISR & I2C_ISR_STOPF) == 0) {
	}

	// CLEAR_BIT(I2C1->CR2, I2C_ICR_STOPCF);

	// rather silly, disable the whole thing again.
	// CLEAR_BIT(I2C1->CR1, I2C_CR1_PE);

	return result;
}
*/

#define I2C_TIMEOUT 25

// const int16_t PLL_XTAL_TRIM_PP10M[] = PLL_XTAL_TRIM_PP10M_VALUES;

/*
static void CDCE913_initInterface() {
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	/ * I2C1 clock enable * /
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/ * I2C1 SDA and SCL configuration * /
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// What the fuck is this for?
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	I2C_DeInit(I2C1);

	/ *enable I2C* /
	I2C_Cmd(I2C1, DISABLE);

	/ * I2C1 configuration * /
	I2C_StructInit(&I2C_InitStructure);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xAA;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 20000;
	I2C_Init(I2C1, &I2C_InitStructure);

	/ *enable I2C* /
	I2C_Cmd(I2C1, ENABLE);
}

static void I2C_attemptUnjam() {
	GPIO_InitTypeDef GPIO_InitStructure;
/ *
	/ * I2C1 clock enable * /
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	I2C_Cmd(I2C1, DISABLE);

	/ * I2C1 SDA and SCL configuration * /
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIOB->ODR &= ~GPIO_Pin_8; // SDA low.
	for (int i=0; i<8; i++) {
		GPIOB->ODR &= ~GPIO_Pin_9;
		timer_sleep(1);
		GPIOB->ODR |= GPIO_Pin_9;
		timer_sleep(1);
	}
	// While clock is high raise SDA. This is supposed to be STOP.
	GPIOB->ODR |= GPIO_Pin_8; // SDA high.
	CDCE913_initInterface();
	timer_sleep(1);
	* /
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_SWRST;
	timer_sleep(1);
	I2C1->CR1 &= ~I2C_CR1_SWRST;
	I2C1->CR1 |= I2C_CR1_PE;
	timer_sleep(1);
}

static void CDCE913_initInterfaceIfNecessary() {
	// if (!(I2C1->CR1 & I2C_CR1_PE)) {
	// if  (!(RCC->APB1ENR & RCC_APB1Periph_I2C1)) {
	// if (!(GPIOB->ODR & GPIO_Pin_1)) {
    // GPIOB->ODR |= GPIO_Pin_1; // PLL chip power control, if present.
	//	timer_sleep(4);
	CDCE913_initInterface();
	// }
}

static uint8_t I2C_read(uint8_t reg_addr) {
	int status;
	timer_mark();
	/ * initiate start sequence * /
	I2C_GenerateSTART(I2C1, ENABLE);
	while ((status = !I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending START\n");
		I2C_attemptUnjam();
	}

	/ *send read command to chip* /
	timer_mark();
	I2C_Send7bitAddress(I2C1, CDCE913_I2C_ADDR << 1, I2C_Direction_Transmitter);
	/ *check master is now in Tx mode* /
	while ((status = !I2C_CheckEvent(I2C1,
	I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending address (1)\n");
		I2C_attemptUnjam();
	}

	/ *who am i register address* /
	timer_mark();
	I2C_SendData(I2C1, reg_addr | (1 << 7)); // The 1<<7 is to do a byte read, not a block read.
	/ * wait for byte send to complete* /
	while ((status = !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending RA\n");
		I2C_attemptUnjam();
	}

	// Repeated start
	timer_mark();
	I2C_GenerateSTART(I2C1, ENABLE);
	while ((status = !I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending repeated start\n");
		I2C_attemptUnjam();
	}
	/ *send read command to chip* /
	timer_mark();
	I2C_Send7bitAddress(I2C1, CDCE913_I2C_ADDR << 1, I2C_Direction_Receiver);
	/ *check master is now in Rx mode* /
	while ((status = !I2C_CheckEvent(I2C1,
	I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending address (2)\n");
		I2C_attemptUnjam();
	}

	/ *enable ACK bit * /
	timer_mark();
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	while ((status = !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	uint8_t data = I2C_ReceiveData(I2C1);
	if (status) {
		trace_printf("I2C:Timeout sending data\n");
		I2C_attemptUnjam();
	}
	/ *generate stop* /
	timer_mark();
	I2C_GenerateSTOP(I2C1, ENABLE);
	/ *stop bit flag* /
	while ((status = I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending STOP\n");
		I2C_attemptUnjam();
	}

	return data;
}

static void I2C_write(uint8_t reg_addr, uint8_t data) {
	int status;
	timer_mark();
	/ * initiate start sequence * /
	I2C_GenerateSTART(I2C1, ENABLE);
	while ((status = !I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending START\n");
		I2C_attemptUnjam();
	}

	timer_mark();
	/ *send read command to chip* /
	I2C_Send7bitAddress(I2C1, CDCE913_I2C_ADDR << 1, I2C_Direction_Transmitter);
	/ *check master is now in Tx mode* /
	while ((status = !I2C_CheckEvent(I2C1,
	I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending address\n");
		I2C_attemptUnjam();
	}

	/ *who register address* /
	timer_mark();
	I2C_SendData(I2C1, reg_addr | (1 << 7));
	/ *wait for byte send to complete* /
	while ((status = !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending RA\n");
		I2C_attemptUnjam();
	}

	/ *data byte* /
	timer_mark();
	I2C_SendData(I2C1, data);
	/ *wait for byte send to complete* /
	while ((status = !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending data\n");
		I2C_attemptUnjam();
	}

	/ *generate stop* /
	timer_mark();
	I2C_GenerateSTOP(I2C1, ENABLE);
	/ *stop bit flag* /
	while ((status = I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending STOP\n");
		I2C_attemptUnjam();
	}
}
*/

extern void I2C1_GPIO_Config();
extern boolean I2C1_writeByte(uint8_t DeviceAddress, uint8_t registerAddress, uint8_t data);
extern uint8_t I2C1_readByte(uint8_t deviceAddress, uint8_t registerAddress);

void CDCE913_initInterfaceIfNecessary() {
//	if (!(GPIOA->ODR & GPIO_Pin_2)) {
//		GPIOA->ODR |= GPIO_Pin_2; // turn on 3v3 reg
//		timer_sleep(5);
		I2C1_GPIO_Config();
//	}
}

uint8_t CDCE913_read(uint8_t registerAddress) {
	return I2C1_readByte(CDCE913_I2C_ADDR<<1, registerAddress | (1<<7));
}

void CDCE913_write(uint8_t registerAddress, uint8_t data) {
	I2C1_writeByte(CDCE913_I2C_ADDR<<1, registerAddress | (1<<7), data);
}

static void CDCE913_setPLLValue(const CDCE913_PLL_Setting_t* setting) {
	uint8_t pllRange = 0;

	// uint8_t r2 = I2C_read(0x02);
	// I2C_write(0x02, (r2 & 0b11111100) | (setting->pdiv >> 8));
	// I2C_write(0x03, (uint8_t) setting->pdiv);

	CDCE913_write(0x18, (uint8_t)(setting->N >> 4));
	CDCE913_write(0x19, (setting->N << 4) | (setting->R >> 5));
	CDCE913_write(0x1a, (setting->R << 3) | (setting->Q >> 3)); // Q upper bits
	CDCE913_write(0x1b, (setting->Q << 5) | (setting->P << 2) | pllRange);

	CDCE913_write(0x1c, (uint8_t)(setting->N >> 4));
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

static void CDCE913_enableOutput(CDCE913_OutputMode_t whichOutput, uint16_t pdiv) {
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

	// timer_sleep(1);
	// GPIOA->ODR &= ~GPIO_Pin_2;

	// timer_sleep(1);
	// I2C_Cmd(I2C1, DISABLE);
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
	// PLL chip power control, if present.
	// timer_sleep(2);
	// GPIOB->ODR &= ~GPIO_Pin_1;
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

boolean findM(int16_t N, double desiredMultiplication, uint16_t* M, double* signedError) {
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

	//trace_printf("FindM returns %d and error %d\n", hadResult, (int)(*signedError*1000));
	return false;
}

int8_t PLL_bestTrim(double desiredTrim) {
	int32_t desiredPP10M = desiredTrim * 1E7;
	int32_t bestError = 1500;
	int8_t bestIndex = -1;

	for (uint8_t i = PLL_MIN_TRIM_INDEX_VALUE; i <= PLL_MAX_TRIM_INDEX_VALUE; i++) {
		int32_t test = desiredPP10M - PLL_XTAL_TRIM_PP10M[i];
		if (test < 0)
			test = -test;
		if (test < bestError) {
			bestError = test;
			bestIndex = i;
		}
	}

	return bestIndex;
}

boolean PLL_bestPLLSetting(
		uint32_t oscillatorFrequency,
		uint32_t desiredFrequency,
		double maxError,
		CDCE913_PLL_Setting_t* result) {
	// trace_printf("bestPLLSetting: %d %d\n", oscillatorFrequency, desiredFrequency);

	// okay the rounding is off if somebody wants 100MHz. But nobody wants that.
	int Pdivmin = 100E6 / (double) desiredFrequency + 1;
	int pdivMax = 200E6 / (double) desiredFrequency;
	double bestError = 1;
	boolean feasible = false;

	for (int Pdiv = Pdivmin; Pdiv <= pdivMax && bestError > maxError; Pdiv++) {
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
		for (uint16_t N = 1; N <= Nmax && bestError > maxError; N++) {
			double signedError;
			double unsignedError;
			if (findM(N, desiredMultiplication, &M, &signedError)) {
				unsignedError = signedError < 0 ? -signedError : signedError;
				// trace_printf("N: %d, M: %d, US: %d S: %d\n", N, M, (int)(unsignedError*1E7), (int)(signedError*1E7));
				if (unsignedError < bestError) {
					bestError = unsignedError;
					// trace_printf("Best N:%d,M:%d, error pp10M:%u\n", N, M, (int) (signedError * 1E7));
					result->pdiv = Pdiv;
					result->N = N;
					result->M = M;
					setPQR(N, M, result);

					// If we divided too much (freq too low), signedError is negative. We need add a little speed on the trim.
					// Opposite for dividing too little.
					int8_t trim = PLL_bestTrim(-signedError);
					// trace_printf("SignedError PPM: %d, trim:%d\n", (int)(signedError * 1E6), trim);

					if (trim != -1) {
						 /*trace_printf("desired:%d, real:%d, error:%d, trim: %d\n",
						 (int) (desiredMultiplication * 1E7),
						 (int) (1E7 * (double) N / (double) M),
						 (int) (signedError * 1E7),
						 trim);
*/
						result->trim = trim;
						feasible = true;
					}
				}
			}
		}
	}
	return feasible;
}

int16_t PLL_oscillatorError(uint32_t measuredFrequency) {
	int32_t result = (int32_t)measuredFrequency - (int32_t)PLL_XTAL_NOMINAL_FREQUENCY;
	// trace_printf("Osc error: %d\n", result);
	if (result > 32767) result = 32767;
	else if (result < -32768) result = -32768;
	return result;
}

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


