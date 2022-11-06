/*
 * led.c
 *
 *  Created on: Nov 5, 2022
 *      Author: fahmad
 */

#include <stdint.h>
#include "led.h"

void delay(uint32_t count)
{
	for (uint32_t i = 0; i < count; i++)
		;
}

void led_init_all(void)
{
	// so the led doesn't work on stm32 f446re
	uint32_t *pRccAhb1enr = (uint32_t *)0x40023830;
	uint32_t *pGpiodModeReg = (uint32_t *)0x40020000;

	*pRccAhb1enr |= (1 << 3);
	// configure LED_GREEN
	*pGpiodModeReg |= (1 << (2 * LED_GREEN));
	*pGpiodModeReg |= (1 << (2 * LED_ORANGE));
	*pGpiodModeReg |= (1 << (2 * LED_RED));
	*pGpiodModeReg |= (1 << (2 * LED_BLUE));

#if 0
	//configure the outputtype
	*pGpioOpTypeReg |= ( 1 << (2 * LED_GREEN));
	*pGpioOpTypeReg |= ( 1 << (2 * LED_ORANGE));
	*pGpioOpTypeReg |= ( 1 << (2 * LED_RED));
	*pGpioOpTypeReg |= ( 1 << (2 * LED_BLUE));
#endif

	led_off(LED_GREEN);
	led_off(LED_ORANGE);
	led_off(LED_RED);
	led_off(LED_BLUE);
}

void led_on(uint32_t led_no)
{
	uint32_t *pGpiodDataReg = (uint32_t *)0x40020C14;
	*pGpiodDataReg |= (1 << led_no);
}

void led_off(uint32_t led_no)
{
	uint32_t *pGpiodDataReg = (uint32_t *)0x40020C14;
	*pGpiodDataReg &= ~(1 << led_no);
}
