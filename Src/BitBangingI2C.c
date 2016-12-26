#include "stm32l0xx.h"
#include "Types.h"
#include "LED.h"

// PB6, SCL is D10 = CN5,3 and CN10,17 on the nucleo
// PB7, SDA, is CN7,21 on nucleo (it has no arduino name).

void I2C1_GPIO_Config(void) {
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

	// 6 and 7 are set to plain old output.
	GPIOB->MODER = (GPIOB->MODER & ~(0b11 << (6 * 2) | 0b11 << (7 * 2)))
			| 0b01 << (6 * 2) | 0b01 << (7 * 2);
	// both are OD output type (1)
	GPIOB->OTYPER = GPIOB->OTYPER | 1 << 6 | 1 << 7;
	// Pull up
	GPIOB->PUPDR = (GPIOB->PUPDR & ~(3 << (6 * 2) | 3 << (7 * 2)))
			| 1 << (6 * 2) | 1 << (7 * 2);
}

void I2C_delay(void) {
	volatile int i = 3;
	while (i) {
		i--;
	}
}

#define SDAH (GPIOB->ODR |= (1<<7))
#define SDAL (GPIOB->ODR &= ~(1<<7))
#define SCLH (GPIOB->ODR |= (1<<6))
#define SCLL (GPIOB->ODR &= ~(1<<6))

#define SDAread (GPIOB->IDR & (1<<7))
#define SCLread (GPIOB->IDR & (1<<6))

boolean I2C1_start(void) {
	SDAH;
	SCLH;
	I2C_delay();
	if (!SDAread)
		return false;
	SDAL; // set data low while clock high (normally data changes only during low clk)
	I2C_delay();
	if (SDAread) 	// never went low?
		return false;
	SDAL;
	I2C_delay();
	return true;
}

void I2C1_stop(void) {
	SCLL;
	I2C_delay();
	SDAL;// set data low while clock low (well, at least they are both low now)
	I2C_delay();
	SCLH;
	I2C_delay();
	SDAH;// set data high while clock high (normally data changes only during low clk)
	I2C_delay();
}

void I2C1_ack(void) {
	SCLL;
	I2C_delay();
	SDAL;
	I2C_delay();
	SCLH;
	I2C_delay();
	SCLL;
	I2C_delay();
}

void I2C1_noAck(void) {
	SCLL;
	I2C_delay();
	SDAH;
	I2C_delay();
	SCLH;
	I2C_delay();
	SCLL;
	I2C_delay();
}

boolean I2C1_waitAck(void) {
	SCLL;
	I2C_delay();
	SDAH;
	I2C_delay();
	SCLH;
	I2C_delay();
	if (SDAread) {
		SCLL;
		return false;
	}
	SCLL;
	return true;
}

void I2C1_sendByte(uint8_t sendByte) {
	unsigned char i = 8;
	while (i--) {
		SCLL;
		I2C_delay();
		if (sendByte & 0x80)
			SDAH;
		else
			SDAL;
		sendByte <<= 1;
		I2C_delay();
		SCLH;
		I2C_delay();
	}
	SCLL;
}

uint8_t I2C1_receiveByte(void) {
	uint8_t i = 8;
	uint8_t receiveByte = 0;

	SDAH;
	while (i--) {
		receiveByte <<= 1;
		SCLL;
		I2C_delay();
		SCLH;
		I2C_delay();
		if (SDAread) {
			receiveByte |= 0x01;
		}
	}
	SCLL;
	return receiveByte;
}

uint8_t I2C1_readByte(uint8_t deviceAddress, uint8_t registerAddress) {
	uint8_t temp;
	if (!I2C1_start())
		return false;

	I2C1_sendByte((deviceAddress & 0xFE));
	if (!I2C1_waitAck()) {
		I2C1_stop();
		return false;
	}
	I2C1_sendByte(registerAddress);
	I2C1_waitAck();

	// Restart
	I2C1_start();
	// Now as read
	I2C1_sendByte((deviceAddress & 0xFE) | 0x01);
	I2C1_waitAck();

	temp = I2C1_receiveByte();

	I2C1_noAck();

	I2C1_stop();
	return temp;
}

boolean I2C1_writeByte(uint8_t DeviceAddress, uint8_t registerAddress,
		uint8_t data) {
	uint8_t temp;

	if (!I2C1_start()) {
		LED_faultCode(LED_FAULT_I2C_DEAD);
		//trace_printf("I2CStart fail\n");
		return false;
	}

	I2C1_sendByte(DeviceAddress & 0xFE);

	if (!I2C1_waitAck()) {
		//trace_printf("I2CWriteDeviceAddress fail\n");
		LED_faultCode(LED_FAULT_I2C_DEAD);
		I2C1_stop();
		return false;
	}

	I2C1_sendByte(registerAddress);

	if (!I2C1_waitAck()) {
		// trace_printf("I2CWriteRegAddress fail\n");
		LED_faultCode(LED_FAULT_I2C_DEAD);
		I2C1_stop();
		return false;
	}	// I2C1_start();
	// I2C1_sendByte(DeviceAddress & 0xFE);
	// I2C1_waitAck();

	I2C1_sendByte(data);
	if (!I2C1_waitAck()) {
		//trace_printf("I2CWriteData fail\n");
		LED_faultCode(LED_FAULT_I2C_DEAD);
		I2C1_stop();
		return false;
	}

	I2C1_stop();
	return temp;
}
