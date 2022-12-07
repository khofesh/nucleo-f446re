/*
 * 001led_toggle.c
 *
 *  Created on: Dec 7, 2022
 *      Author: fahmad
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void delay();

int main(void)
{
	// toggle led on PA5
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// push pull
//	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	// open drain - not recommended unless for specific reason
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}

	return 0;
}

void delay()
{
	for (uint32_t i = 0; i < 500000/2; i++);
}
