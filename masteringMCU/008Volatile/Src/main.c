/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define SRAM_ADDRESS1 0x20000000U

int main(void)
{
	uint32_t value = 0;
	uint32_t volatile *p;
	p = (uint32_t *) SRAM_ADDRESS1;

	while(1)
	{
		value = *p;
		if (value)
		{
			break;
		}
	}

    /* Loop forever */
	for(;;);
}