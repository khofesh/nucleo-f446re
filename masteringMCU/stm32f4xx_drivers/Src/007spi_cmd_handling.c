/*
 * 007spi_cmd_handling.c
 *
 *  Created on: Dec 19, 2022
 *      Author: fahmad
 */

#include <string.h>
#include <stdio.h>
#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

extern void initialise_monitor_handles();

/**
 * find out microcontroller pin to communicate over SPI2
 * see stm32f446re.pdf page 57 (Table 11. Alternate function)
 *
 * PB15
 * SPI2_MOSI/I2S2_SD
 *
 * PB14
 * SPI2_MISO
 *
 * PB13
 * SPI2_SCK
 *
 * PB12
 * SPI2_NSS
 *
 * ALT function mode: AF5
 */

// command codes
#define COMMAND_LED_CTRL 0x50
#define COMMAND_SENSOR_READ 0X51
#define COMMAND_LED_READ 0x52
#define COMMAND_PRINT 0x53
#define COMMAND_ID_READ 0x54

#define LED_ON 1
#define LED_OFF 0

// arduino analog pins
#define ANALOG_PIN0 0
#define ANALOG_PIN1 1
#define ANALOG_PIN2 2
#define ANALOG_PIN3 3
#define ANALOG_PIN4 4

// arduino led
#define LED_PIN 9

#define LOW 0
#define BTN_PRESSED LOW

void delay();
void SPI2_GPIOInits();
void SPI2_Inits();
void GPIO_ButtonInit();
uint8_t SPI_verifyResponse(uint8_t ackByte);

int main()
{
	initialise_monitor_handles();

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	printf("application is running\n");

	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	printf("SPI2 initialized\n");

	/**
	 * making ssoe 1 does NSS output enable.
	 * the NSS pin is automatically managed by the hardwared.
	 * when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while (1)
	{
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != BTN_PRESSED)
			;
		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		/* 1. CMD_LED_CTRL <pin no(1)> <value(1)> */
		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ackByte;
		uint8_t args[2];

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1 byte) to fetch the
		// response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if (SPI_verifyResponse(ackByte))
		{
			// send arguments
			args[0] = LED_PIN;
			args[1] = args[1] == LED_ON ? LED_OFF : LED_ON;

			SPI_SendData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL executed\n");
		}
		// end of COMMAND_LED_CTRL

		/* 2. CMD_SENSOR_READ <analog pin number(1)> */

		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != BTN_PRESSED)
			;
		delay();

		commandCode = COMMAND_SENSOR_READ;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1 byte) to fetch the
		// response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if (SPI_verifyResponse(ackByte))
		{
			args[0] = ANALOG_PIN0;

			// send arguments
			SPI_SendData(SPI2, args, 1);

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// delay so that slave can ready with the data
			delay();

			// send some dummy bits (1 byte) to fetch the
			// response from the slave.
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("COMMAND_SENSOR_READ %d\n", analog_read);
		}

		/* 3.  CMD_LED_READ 	 <pin no(1) > */

		// wait till button is pressed
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != BTN_PRESSED)
			;
		delay();

		commandCode = COMMAND_LED_READ;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if (SPI_verifyResponse(ackByte))
		{
			args[0] = LED_PIN;

			// send arguments
			SPI_SendData(SPI2, args, 1); // sending one byte of

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// insert some delay so that slave can ready with the data
			delay();

			// Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
			printf("COMMAND_READ_LED %d\n", led_status);
		}

		/* 4. CMD_PRINT 		<len(2)>  <message(len) > */

		// wait till button is pressed
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != BTN_PRESSED)
			;
		delay();

		commandCode = COMMAND_PRINT;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		uint8_t message[] = "Hello ! How are you ??";
		if (SPI_verifyResponse(ackByte))
		{
			args[0] = strlen((char *)message);

			// send arguments
			SPI_SendData(SPI2, args, 1); // sending length

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			delay();

			// send message
			for (int i = 0; i < args[0]; i++)
			{
				SPI_SendData(SPI2, &message[i], 1);
				SPI_ReceiveData(SPI2, &dummy_read, 1);
			}

			printf("COMMAND_PRINT Executed \n");
		}

		/* 5. CMD_ID_READ */

		// wait till button is pressed
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != BTN_PRESSED)
			;
		delay();

		commandCode = COMMAND_ID_READ;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		uint8_t id[11];
		uint32_t i = 0;
		if (SPI_verifyResponse(ackByte))
		{
			// read 10 bytes id from the slave
			for (i = 0; i < 10; i++)
			{
				// send dummy byte to fetch data from slave
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			id[11] = '\0';

			printf("COMMAND_ID : %s \n", id);
		}

		// confirm SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG))
			;

		// disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI Communication Closed\n");
	}

	return 0;
}

/**
 * @brief this function is used to initialize the GPIO
 * pins to behave as SPI2 pins
 */
void SPI2_GPIOInits()
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// serial clock
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

/**
 * @brief this function is used to initialize the SPI2
 * peripheral parameters
 */
void SPI2_Inits()
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	// generates serial clock of 8MHz
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	// hardware slave management enabled for NSS pin
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
}

void delay()
{
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOBtn;

	// user blue button - PC13
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

uint8_t SPI_verifyResponse(uint8_t ackByte)
{
	if (ackByte == 0xF5)
	{
		// ack
		return 1;
	}

	// nack
	return 0;
}
