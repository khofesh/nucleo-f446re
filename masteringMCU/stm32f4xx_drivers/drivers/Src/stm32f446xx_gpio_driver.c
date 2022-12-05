/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Dec 5, 2022
 *      Author: fahmad
 */

#include "stm32f446xx_gpio_driver.h"

/**
 * @brief This function enables or disables peripheral clock for the given GPIO port
 *
 * @param pGPIOx base address of the GPIO peripheral
 * @param EnOrDi ENABLE or DISABLE macros
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint32_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {

        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
    }
}

/**
 * @brief Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
 *
 * @param pGPIOHandle
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    /**
     * 1. configure the mode of GPIO pin
     * RM0390-*.pdf page 187
     * These bits are written by software to configure the I/O direction mode.
     * 00: Input (reset state)
     * 01: General purpose output mode
     * 10: Alternate function mode
     * 11: Analog mode
     */

    // 2. configure the speed

    // 3. configure the pull-up pull-down settings

    // 4. configure the optype

    // 5. configure the alt functionality
}

/**
 * @brief
 *
 * @param pGPIOx
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
}

/**
 * @brief
 *
 * @param pGPIOx
 * @param PinNumber
 * @return uint32_t
 */
uint32_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint32_t PinNumber)
{
}

/**
 * @brief
 *
 * @param pGPIOx
 * @return uint16_t
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
}

/**
 * @brief
 *
 * @param pGPIOx
 * @param PinNumber
 * @param Value
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint32_t PinNumber, uint32_t Value)
{
}

/**
 * @brief
 *
 * @param pGPIOx
 * @param Value
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
}

/**
 * @brief
 *
 * @param pGPIOx
 * @param PinNumber
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint32_t PinNumber)
{
}

/**
 * @brief
 *
 * @param IRQNumber
 * @param IRQPriority
 * @param EnOrDi
 */
void GPIO_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint32_t EnOrDi)
{
}

/**
 * @brief
 *
 * @param PinNumber
 */
void GPIO_IRQHandler(uint32_t PinNumber)
{
}
