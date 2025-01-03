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
#include "stm32f4xx_ll_tim.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define ever		;;

void tim2_1hz_init();
void led_init();

int main(void)
{
	tim2_1hz_init();
	led_init();

    /* Loop forever */
	for(ever)
	{
		while(!LL_TIM_IsActiveFlag_UPDATE(TIM2));

		LL_TIM_ClearFlag_UPDATE(TIM2);
		LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
	}
}

void tim2_1hz_init()
{
	/* enable clock access to timer module */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	/* set the pre-scaler */
	LL_TIM_SetPrescaler(TIM2, 1600 - 1);

	/* set auto-reload value */
	LL_TIM_SetAutoReload(TIM2, 10000 - 1);

	/* enable timer module */
	LL_TIM_EnableCounter(TIM2);
}

void led_init()
{
	/* enable clock for GPIOA */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* set pin to output mode */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}
