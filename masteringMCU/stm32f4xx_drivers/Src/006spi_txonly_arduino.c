/*
 * 006spi_txonly_arduino.c
 *
 *  Created on: Dec 18, 2022
 *      Author: fahmad
 */

#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

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

#define LOW 0
#define BTN_PRESSED LOW

void delay();
void SPI2_GPIOInits();
void SPI2_Inits();
void GPIO_ButtonInit();

int main()
{
	char user_data[] = "Learn bare metal driver development using Embedded C";

	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	/**
	 * making ssoe 1 does NSS output enable.
	 * the NSS pin is automatically managed by the hardwared.
	 * when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while (1)
	{
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay();

			// enable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			// send length information
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI2, &dataLen, 1);

			// send data
			SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

			// confirm SPI is not busy
			while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG))
				;

			// disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, DISABLE);
		}
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
	//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//	GPIO_Init(&SPIPins);

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
