/*
 * 014uart_tx.c
 *
 *  Created on: Dec 27, 2022
 *      Author: fahmad
 */

#include <string.h>
#include <stdio.h>
#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"

char *msg[3] = {"hihihihihihi123", "Hello How are you ?", "Today is Monday !"};

// reply from arduino will be stored here
char rx_buf[1024];

USART_Handle_t usart2Handle;

uint8_t rxCmplt = RESET;
uint8_t g_data = 0;

#define LOW 0
#define BTN_PRESSED LOW

void USART2_Init();
void USART2_GPIOInit();
void GPIO_ButtonInit();
void delay();

int main()
{
	uint32_t cnt = 0;

	GPIO_ButtonInit();

	USART2_GPIOInit();

	USART2_Init();

	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

	USART_PeripheralControl(USART2, ENABLE);

	printf("Application is running\n");

	while (1)
	{
		/* Wait till button is pressed */
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != BTN_PRESSED)
		{
		}

		/* 200ms delay */
		delay();

		// Next message index ; make sure that cnt value doesn't cross 2
		cnt = cnt % 3;

		// First lets enable the reception in interrupt mode
		// this code enables the receive interrupt
		while (USART_ReceiveDataIT(&usart2Handle, (uint8_t *)rx_buf, strlen(msg[cnt])) != USART_READY)
			;

		// Send the msg indexed by cnt in blocking mode
		USART_SendData(&usart2Handle, (uint8_t *)msg[cnt], strlen(msg[cnt]));

		printf("Transmitted : %s\n", msg[cnt]);

		// Now lets wait until all the bytes are received from the arduino .
		// When all the bytes are received rxCmplt will be SET in application callback
		while (rxCmplt != SET)
			;

		// just make sure that last byte should be null otherwise %s fails while printing
		rx_buf[strlen(msg[cnt]) + 1] = '\0';

		// Print what we received from the arduino
		printf("Received    : %s\n", rx_buf);

		// invalidate the flag
		rxCmplt = RESET;

		// move on to next message indexed in msg[]
		cnt++;
	}
}

/**
 * @brief delay
 *
 */
void delay()
{
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

void USART2_Init()
{
	usart2Handle.pUSARTx = USART2;
	usart2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2Handle);
}

void USART2_GPIOInit()
{
	GPIO_Handle_t usartGpios;

	usartGpios.pGPIOx = GPIOA;
	usartGpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usartGpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usartGpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usartGpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usartGpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	// USART2 TX
	usartGpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usartGpios);

	// USART2 RX
	usartGpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usartGpios);
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

void USART2_IRQHandler(void)
{
	USART_IRQHandling(&usart2Handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv)
{
	if (ApEv == USART_EVENT_RX_CMPLT)
	{
		rxCmplt = SET;
	}
	else if (ApEv == USART_EVENT_TX_CMPLT)
	{
		;
	}
}