/*
 * 002led_button.c
 *
 *  Created on: Dec 7, 2022
 *      Author: fahmad
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define LOW 0
#define HIGH 1
#define BTN_PRESSED LOW

void delay();

int main(void)
{

	GPIO_Handle_t GpioLed, GPIOBtn;

	// PA8 - led
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	// push pull
	GpioLed.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	// PC13 - button
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay(); // wait until debouncing is over
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
	}

	return 0;
}

void delay()
{
	for (uint32_t i = 0; i < 500000/2; i++);
}
