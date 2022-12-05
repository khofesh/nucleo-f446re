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
    uint32_t temp = 0;
    // 1. configure the mode of GPIO pin

    // if not interrupt mode
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
        pGPIOHandle->pGPIOx->MODER |= temp;                                                 // setting
    }
    else
    {
        /* code */
    }

    temp = 0;

    // 2. configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
    pGPIOHandle->pGPIOx->OSPEEDER |= temp;

    temp = 0;

    // 3. configure the pull-up pull-down settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;

    // 4. configure the optype
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    // 5. configure the alt functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        // configure the alt function registers.
        uint32_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}

/**
 * @brief
 *
 * @param pGPIOx
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
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
