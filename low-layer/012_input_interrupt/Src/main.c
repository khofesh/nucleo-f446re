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
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_system.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define		ever		;;

void gpio_interrupt_init();
void led_init();
void button_callback();

int main(void)
{
	gpio_interrupt_init();
	led_init();

    /* Loop forever */
	for(ever)
	{

	}
}

void gpio_interrupt_init()
{
	/* clock GPIOC */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	/* set PC13 to input pin - push button */
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);

	/* enable clock access to EXTI module */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

	/* set EXTI source - PC13 */
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

	/* enable EXTI line */
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_13);

	/* set the trigger */
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_13);

	/* enable NVIC interrupt */
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* set the interrupt priority */
	NVIC_SetPriority(EXTI15_10_IRQn, 0);
}

void led_init()
{
	/* enable clock for GPIOA */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* set pin to output mode */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

void button_callback()
{
	LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
}

void EXTI15_10_IRQHandler()
{
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
	{
		button_callback();
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
	}
}
