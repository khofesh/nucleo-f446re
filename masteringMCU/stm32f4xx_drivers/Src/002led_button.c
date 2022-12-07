/*
 * 002led_button.c
 *
 *  Created on: Dec 7, 2022
 *      Author: fahmad
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define LOW 0
#define BTN_PRESSED LOW

void delay();

int main(void)
{
	// toggle led on PA5
	GPIO_Handle_t GpioLed, GPIOBtn;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	// push pull
	GpioLed.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	// user blue button - PC13
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay(); // wait until debouncing is over
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}

	return 0;
}

void delay()
{
	for (uint32_t i = 0; i < 500000/2; i++);
}
