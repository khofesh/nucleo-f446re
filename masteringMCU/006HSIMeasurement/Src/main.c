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

#define RCC_BASE_ADDR 			0x40023800UL
#define RCC_APB2ENR_OFFSET 		0x44UL
#define RCC_APB2ENR_ADDR 		(RCC_BASE_ADDR + RCC_APB2ENR_OFFSET)
#define RCC_CFGR_REG_OFFSET 	0x08UL
#define RCC_CFGR_REG_ADDR 		(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)
#define GPIOA_BASE_ADDR         0x40020000UL
#define RCC_AHB1ENR_OFFSET		0x30UL
#define RCC_AHB1ENR_ADDR		(RCC_BASE_ADDR + RCC_AHB1ENR_OFFSET)

int main(void) {
	uint32_t *pRccCfgrReg = (uint32_t*) RCC_CFGR_REG_ADDR;

	// 1. configure the RCC_CFGR MCO1 bit fields to select HSI as clock source
	*pRccCfgrReg &= ~(0x3 << 21); // clear 21 and 22 bit positions

	// configure MCO1 prescaler
//	*pRccCfgrReg |= (1 << 25);
//	*pRccCfgrReg |= (1 << 26);

	// 2. configure PA8 to AF0 mode to behave as MCO1 signal

	uint32_t *pRCCAhb1Enr = (uint32_t*) (RCC_AHB1ENR_ADDR);
	*pRCCAhb1Enr |= (1 << 0); //Enable GPIOA peripheral clock

	//b ) Configure the mode of GPIOA pin 8 as alternate function mode

	uint32_t *pGPIOAModeReg = (uint32_t*) (GPIOA_BASE_ADDR + 00);
	*pGPIOAModeReg &= ~(0x3 << 16); //clear
	*pGPIOAModeReg |= (0x2 << 16);  //set

	//c ) Configure the alternation function register to set the mode 0 for PA8

	uint32_t *pGPIOAAltFunHighReg = (uint32_t*) (GPIOA_BASE_ADDR + 0x24);
	*pGPIOAAltFunHighReg &= ~(0xf << 0);

	/* Loop forever */
	for (;;)
		;
}

