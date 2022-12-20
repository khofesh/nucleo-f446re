/*
 * 008spi_msg_rcv_interrupt.c
 *
 *  Created on: Dec 20, 2022
 *      Author: fahmad
 */

#include <string.h>
#include <stdio.h>
#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

SPI_Handle_t SPI2Handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];
volatile char ReadByte;
volatile uint8_t rcvStop = 0;
// this flag will be set in the interrupt handler of the
// arduino interrupt GPio
volatile uint8_t dataAvailable = 0;

void delay();
void SPI2_GPIOInits();
void SPI2_Inits();
void Slave_GPIO_InterruptPinInit();
void SPI2_IRQHandler();
void EXTI15_10_IRQHandler();

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

int main()
{
	uint8_t dummy = 0xff;

	uint32_t justOnce = 0;
	uint32_t justOnce2 = 0;
	uint32_t justOnce3 = 0;

	Slave_GPIO_InterruptPinInit();

	// this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1 , NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while (1)
	{
		rcvStop = 0;

		while (!dataAvailable)
		{
			// wait till data available interrupt from transmitter device(slave)

			if (justOnce != 1)
			{
				printf("trapped here\n");
				justOnce++;
			}
		}

		printf("I'm out\n");

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, DISABLE);

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while (!rcvStop)
		{
			printf("send and receive\n");
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while (SPI_SendDataIT(&SPI2Handle, &dummy, 1) == SPI_BUSY_IN_TX)
			{
				if (justOnce2 != 1)
				{
					printf("trapped in SPI_SendDataIT\n");
					justOnce2++;
				}
			}

			while (SPI_ReceiveDataIT(&SPI2Handle, &ReadByte, 1) == SPI_BUSY_IN_RX)
			{
				if (justOnce3 != 1)
				{
					printf("trapped in SPI_ReceiveDataIT\n");
					justOnce3++;
				}
			}
		}

		printf("confirming now\n");
		// confirm SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG))
			;

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Rcvd data = %s\n", RcvBuff);

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
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
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	// generates serial clock of 8MHz
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	// hardware slave management enabled for NSS pin
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);
}

/**
 * @brief This function configures the gpio pin over which SPI
 * peripheral issues data available interrupt
 */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin, 0, sizeof(spiIntPin));

	// PC12 - button
	spiIntPin.pGPIOx = GPIOC;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&spiIntPin);

	// IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI47);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if (appEvent == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if (ReadByte == '\0' || (i == MAX_LEN))
		{
			rcvStop = 1;
			RcvBuff[i - 1] = '\0';
			i = 0;
		}
	}
}

void SPI2_IRQHandler()
{
	printf("inside SPI2_IRQHandler\n");
	SPI_IRQHandler(&SPI2Handle);
}

void EXTI15_10_IRQHandler()
{
	GPIO_IRQHandler(GPIO_PIN_NO_12);
	dataAvailable = 1;
}
