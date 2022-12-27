/*
 * stm32f466xx_rcc_driver.h
 *
 *  Created on: Dec 27, 2022
 *      Author: fahmad
 */

#ifndef INC_STM32F466XX_RCC_DRIVER_H_
#define INC_STM32F466XX_RCC_DRIVER_H_

#include <stm32f446xx.h>

/**
 * @brief RCC_GetPCLK1Value
 * returns APB1 clock value
 * @return uint32_t
 */
uint32_t RCC_GetPCLK1Value();

/**
 * @brief RCC_GetPCLK2Value
 * returns APB2 clock value
 * @return uint32_t
 */
uint32_t RCC_GetPCLK2Value();

uint32_t RCC_GetPLLOutputClock();

#endif /* INC_STM32F466XX_RCC_DRIVER_H_ */
