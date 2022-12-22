/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Dec 22, 2022
 *      Author: fahmad
 */

#include "stm32f446xx_i2c_driver.h"

/**
 * @brief I2C_PeriClockControl
 *
 * @param pI2Cx
 * @param EnOrDi
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {

        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if (pI2Cx == I2C1)
        {
            I2C2_PCLK_DI();
        }
        else if (pI2Cx == I2C1)
        {
            I2C3_PCLK_DI();
        }
    }
}

/**
 * @brief I2C_Init
 * caution: all configuration in this function must
 * be done when the peripheral is disabled in the control register
 * @param pI2CHandle
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    // 1. configure the mode (standard or fast)

    // 2. configure the speed of the serial clock (SCL)

    // 3. configure the device address (applicable when device is slave)

    // 4. enable the Acking

    // 5. configure the rise time for I2C pins
}

/**
 * @brief I2C_DeInit
 *
 * @param pI2Cx
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    if (pI2Cx == I2C1)
    {
        I2C1_REG_RESET();
    }
    else if (pI2Cx == I2C2)
    {
        I2C2_REG_RESET();
    }
    else if (pI2Cx == I2C3)
    {
        I2C3_REG_RESET();
    }
}

/**
 * @brief I2C_IRQInterruptConfig
 *
 * @param IRQNumber
 * @param EnOrDi
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
}

/**
 * @brief I2C_IRQPriorityConfig
 *
 * @param IRQNumber
 * @param IRQPriority
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
}

/**
 * @brief I2C_PeripheralControl
 *
 * @param pI2Cx
 * @param EnOrDi
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << 0);
    }
}

/**
 * @brief I2C_GetFlagStatus
 *
 * @param pI2Cx
 * @param flagName
 * @return true
 * @return false
 */
bool I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName)
{
}

/**
 * @brief I2C_ApplicationEventCallback
 *
 * @param pI2CHandle
 * @param appEvent
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent)
{
}
