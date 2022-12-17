/*
 * 005spi_tx_testing.c
 *
 *  Created on: Dec 17, 2022
 *      Author: fahmad
 */
#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

/**
 * test the SPI_SendData API to send the string
 * "hello world" and use the below configurations:
 * 1. SPI-2 master mode
 * 2. SCLK = max possible
 * 3. DFF = 0 and DFF = 1
 *
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

void SPI2_GPIOInits();
void SPI2_Inits();

int main()
{
	char user_data[] = "hola mundo";

	SPI2_GPIOInits();

	SPI2_Inits();

	// this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	// enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

	SPI_PeripheralControl(SPI2, DISABLE);

	while (1)
	{
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
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);
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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	// software slave management enabled for NSS pin
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}
