/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define		ever		;;

void uart2_init();
void uart2_write(uint8_t ch);
uint8_t uart2_read();
void led_init();
void psuedo_delay(volatile int delay);
void led_play(uint8_t value);

uint8_t received;

int main(void)
{
	uart2_init();
	led_init();

    /* Loop forever */
	for(ever)
	{
		received = uart2_read();
		led_play(received);
	}
}

void uart2_init()
{
	/* enable clock  access for UART GPIO pin */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* enable clock access for UART module */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

	/* set mode of UART TX pin to alternate function */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);

	/* set mode of UART RX pin to alternate function */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);

	/* select UART TX alternate function type */
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7);

	/* select UART RX alternate function type */
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_7);

	/* configure UART protocol parameters */
	LL_USART_Disable(USART2);
	LL_USART_SetTransferDirection (USART2, LL_USART_DIRECTION_TX_RX);
	LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B,
			LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
	LL_USART_SetBaudRate (USART2, 16000000,
			LL_USART_OVERSAMPLING_16, 115200);
	LL_USART_Enable(USART2);
}

void uart2_write(uint8_t ch)
{
	/* wait for TXE flag is active */
	while(!LL_USART_IsActiveFlag_TXE(USART2));

	LL_USART_TransmitData8(USART2, ch);
}

uint8_t uart2_read()
{
	while(!LL_USART_IsActiveFlag_RXNE(USART2));

	return LL_USART_ReceiveData8(USART2);
}

void led_init()
{
	/* enable clock for GPIOA */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* set pin to output mode */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

void psuedo_delay(volatile int delay)
{
	for (int i = 0; i < delay; i++);
}

void led_play(uint8_t value)
{
	value %= 16;
	for(;value > 0; value--)
	{
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
		psuedo_delay(90000);
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
	}
}