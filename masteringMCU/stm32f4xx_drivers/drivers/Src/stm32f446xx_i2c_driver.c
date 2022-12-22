/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Dec 22, 2022
 *      Author: fahmad
 */

#include "stm32f446xx_i2c_driver.h"

uint32_t RCC_GetPCLK1Value();
uint32_t RCC_GetPLLOutputClock();

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

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
    uint32_t tempReg = 0;

    // ACK
    tempReg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;

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
    return 1;
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

uint32_t RCC_GetPCLK1Value()
{
    uint32_t pclk1;
    uint32_t SystemClk;

    uint8_t clksrc;
    uint8_t temp;
    uint8_t ahbp;
    uint8_t apb1p;

    /*
        see RM0390-*.pdf page 133

        Bits 3:2 SWS[1:0]: System clock switch status
        Set and cleared by hardware to indicate which clock source is used as the system clock.
        00: HSI oscillator used as the system clock
        01: HSE oscillator used as the system clock
        10: PLL used as the system clock
        11: PLL_R used as the system clock
    */
    clksrc = ((RCC->CFGR >> RCC_CFGR_SWS_Pos) & 0x3);

    if (clksrc == 0)
    {
        SystemClk = 16000000;
    }
    else if (clksrc == 1)
    {
        SystemClk = 8000000;
    }
    else if (clksrc == 2)
    {
        SystemClk = RCC_GetPLLOutputClock();
    }

    temp = ((RCC->CFGR >> RCC_CFGR_HPRE_Pos) & 0xFUL);

    if (temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    temp = ((RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x7UL);

    if (temp < 4)
    {
        apb1p = 1;
    }
    else
    {
        apb1p = APB1_PreScaler[temp - 4];
    }

    pclk1 = (SystemClk / ahbp) / apb1p;

    return pclk1;
}

uint32_t RCC_GetPLLOutputClock()
{
    return 0;
}
