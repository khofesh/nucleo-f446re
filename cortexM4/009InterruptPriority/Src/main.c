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
#include <stdio.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define IRQNO_TIMER2 28
#define IRQNO_I2C1 31

void TIM2_IRQHandler();
void I2C1_EV_IRQHandler();
void configure_priority_for_irqs(uint8_t irq_no, uint8_t priority_value);

uint32_t *pNVIC_IPRBase = (uint32_t*)0xE000E400;
uint32_t *pNVIC_ISERBase = (uint32_t*)0xE000E100;
uint32_t *pNVIC_ISPRBase = (uint32_t*)0XE000E200;

int main(void)
{
	// configure the priority for the peripherals
	configure_priority_for_irqs(IRQNO_TIMER2, 0x60);
	configure_priority_for_irqs(IRQNO_I2C1, 0x80);

	// set the interrupt pending bit in the NVIC PR
	*pNVIC_ISPRBase |= (1 << IRQNO_TIMER2);

	// enable the IRQs in NVIC ISER
	*pNVIC_ISERBase |= (1 << IRQNO_TIMER2);
	*pNVIC_ISERBase |= (1 << IRQNO_I2C1);

    /* Loop forever */
	for(;;);
}

// implement ISRS

void TIM2_IRQHandler()
{
	printf("TIM2_IRQHandler\n");
	*pNVIC_ISPRBase |= (1 << IRQNO_I2C1);

	while(1);
}

void I2C1_EV_IRQHandler()
{
	printf("I2C1_EV_IRQHandler\n");
}

void configure_priority_for_irqs(uint8_t irq_no, uint8_t priority_value)
{
	// find out iprx
	uint8_t iprx = irq_no / 4;
	uint32_t *ipr = pNVIC_IPRBase + iprx;

	// position in iprx
	uint8_t pos = (irq_no % 4) * 8;

	// configure the priority
	// clear
	*ipr &= ~(0xFF << pos);
	*ipr |= (priority_value << pos);
}