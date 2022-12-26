/*
 * 009i2c_master_tx_testing.c
 *
 *  Created on: Dec 25, 2022
 *      Author: fahmad
 */

#include <string.h>
#include <stdio.h>
#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_i2c_driver.h"

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68
#define LOW 0
#define BTN_PRESSED LOW

I2C_Handle_t I2C1Handle;

// receive buffer
uint8_t rcv_buf[32];

void delay();
void SPI2_GPIOInits();
void SPI2_Inits();
void GPIO_ButtonInit();
void I2C1_Inits();
void I2C1_GPIOInits();
void I2C1_EV_IRQHandler();
void I2C1_ER_IRQHandler();

int main()
{
	uint8_t commandCode;
	uint8_t len;

	printf("app running\n");

	/* Initialize button */
	GPIO_ButtonInit();

	// i2c pin inits
	I2C1_GPIOInits();

	// i2c peripheral configuration
	I2C1_Inits();

	// i2c IRQ configurations
	I2C_IRQInterruptConfig(I2C1_EV_IRQn, ENABLE);
	I2C_IRQInterruptConfig(I2C1_ER_IRQn, ENABLE);

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while (1)
	{
		/* Wait till button is pressed */
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != BTN_PRESSED)
		{
		}

		/* 200ms delay */
		delay();

		commandCode = 0x51;

		while (I2C_MasterSendDataIT(&I2C1Handle, &commandCode, 1,
									SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY)
			;

		while (I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1,
									   SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY)
			;

		commandCode = 0x52;

		while (I2C_MasterSendDataIT(&I2C1Handle, &commandCode, 1,
									SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY)
			;

		while (I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, len,
									   SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY)
			;

		rcv_buf[len + 1] = '\0';

		printf("data: %s\n", rcv_buf);
	}

	return 0;
}

void delay()
{
	for (uint32_t i = 0; i < 500000 / 2; i++)
	{
	}
}

/**
 * @brief this function is used to initialize the GPIO
 * PB6 -> SCL
 * PB7 -> SDA
 */
void I2C1_GPIOInits()
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

/**
 * @brief this function is used to initialize the I2C1
 */
void I2C1_Inits()
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOBtn;
	GPIO_Handle_t GpioLed;

	// user blue button - PC13
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	/* LED GPIO Config */
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);
}

void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent)
{
	if (appEvent == I2C_EV_TX_CMPLT)
	{
		printf("tx is completed\n");
	}
	else if (appEvent == I2C_EV_RX_CMPLT)
	{
		printf("rx is completed\n");
	}
	else if (appEvent == I2C_ERROR_AF)
	{
		printf("error: ack failure\n");
		// in master ack failure
		// happens when slave failes to send ack for the byte
		// sent from the master.
		I2C_CloseSendData(pI2CHandle);

		// generate the stop condition to release the bus
		I2C_GenerateStopCondition(I2C1);

		// hang in infinite loop
		while (1)
			;
	}
}
