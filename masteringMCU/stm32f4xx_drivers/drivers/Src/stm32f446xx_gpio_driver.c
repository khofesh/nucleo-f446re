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
        /* configure interrupt mode */
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // 1. configure the FTSR
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            // clear the corresponding RTSR bit
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // 1. configure the RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            // clear the corresponding RTSR bit
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // 1. configure both FTSR and RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            // clear the corresponding RTSR bit
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // 2. configure the GPIO post selection in SYSCFG_EXTICR
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portCode = GPIOB_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);

        // 3. enable the EXTI interrupt delivery using IMR (interrupt mask register)
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
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
 * see rm0390-*.pdf page 189
 *
 * @param pGPIOx
 * @param PinNumber
 * @return 0 or 1
 */
uint16_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber)
{
    uint16_t value;
    value = (uint16_t)(pGPIOx->IDR >> PinNumber & 0x00000001);

    return value;
}

/**
 * @brief
 *
 * @param pGPIOx
 * @return uint16_t
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;

    return value;
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
    if (Value == GPIO_PIN_SET)
    {
        /* write 1 to the output data register at the bit field corresponding to the
        pin number */
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // write
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/**
 * @brief
 *
 * @param pGPIOx
 * @param Value
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/**
 * @brief
 *
 * @param pGPIOx
 * @param PinNumber
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint32_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/**
 * @brief
 * see Cortex-M4-devices-generic-user-guide.pdf page 219
 * `4-2 Nested Vectored Interrupt Controller`
 * @param IRQNumber
 * @param EnOrDi enable or disable
 */
void GPIO_IRQInterruptConfig(uint32_t IRQNumber, uint32_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            /* program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            /* program ISER1 register */
            *NVIC_ISER1 |= (1 << IRQNumber % 32);
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* program ISER2 register */
            *NVIC_ISER2 |= (1 << IRQNumber % 64);
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            /* program ICER0 register */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            /* program ICER1 register */
            *NVIC_ICER1 |= (1 << IRQNumber % 32);
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* program ICER2 register */
            *NVIC_ICER2 |= (1 << IRQNumber % 64);
        }
    }
}

/**
 * @brief change IRQ priority
 * see Cortex-M4-devices-generic-user-guide.pdf page 223
 * `4.2.7 Interrupt Priority Registers`
 * @param IRQPriority
 */
void GPIO_IRQPriorityConfig(uint32_t IRQNumber, uint32_t IRQPriority)
{
    // 1. find out the IPR register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx * 4) |= (IRQPriority << (8 * shift_amount));
}

/**
 * @brief
 *
 * @param PinNumber
 */
void GPIO_IRQHandler(uint32_t PinNumber)
{
    // clear the EXTi PR register corresponding to the pin number
    if (EXTI->PR & (1 << PinNumber))
    {
        /* clear */
        EXTI->PR |= (1 << PinNumber);
    }
}
