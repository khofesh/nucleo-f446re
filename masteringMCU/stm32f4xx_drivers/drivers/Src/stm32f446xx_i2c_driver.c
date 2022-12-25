/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Dec 22, 2022
 *      Author: fahmad
 */

#include "stm32f446xx_i2c_driver.h"

uint32_t RCC_GetPCLK1Value();
uint32_t RCC_GetPLLOutputClock();
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

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
    uint8_t trise;

    // enable the clock for the i2cx peripheral
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    // ACK control bit
    tempReg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
    pI2CHandle->pI2Cx->CR1 = tempReg;

    // configure the FREQ field of CR2
    tempReg = 0;
    tempReg |= RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (tempReg & 0x3F);

    // program the device own address
    tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    // bit 14 - should always be kept at 1 by software
    // see RM0390-*.pdf page 784
    tempReg |= (1 << 14);
    pI2CHandle->pI2Cx->OAR1 = tempReg;

    // CCR calculations
    uint16_t ccr_value = 0;
    tempReg = 0;
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        /* standard mode */
        ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        tempReg |= (ccr_value & 0xFFF);
    }
    else
    {
        /* fast mode*/
        tempReg |= (1 << 15);
        tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        else
        {
            ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }

        tempReg |= (ccr_value & 0xFFF);
    }

    pI2CHandle->pI2Cx->CCR = tempReg;

    // TRISE config
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        /* standard mode */
        trise = (RCC_GetPCLK1Value() / 1000000U) + 1;
    }
    else
    {
        /* fast mode */
        trise = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
    }

    pI2CHandle->pI2Cx->TRISE = (trise & 0x3F);
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
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName)
{
    if (pI2Cx->SR1 & flagName)
    {
        return FLAG_SET;
    }

    return FLAG_RESET;
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

    temp = ((RCC->CFGR >> 4) & 0xFUL);

    if (temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    temp = ((RCC->CFGR >> 10) & 0x7UL);

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

/**
 * @brief I2C_MasterSendData
 *
 * @param pI2CHandle
 * @param pTxBuffer
 * @param Len
 * @param SlaveAddr
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
    // 1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2. confirm that start generation is completed by checking the SB flag in the SR1
    //    Note: Until SB is cleared SCL will be stretched (pulled to LOW)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
    {
        /* code */
    }

    // 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
    I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

    // 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
    {
        /* code */
    }

    // 5. clear the ADDR flag according to its software sequence
    //    Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
    I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

    // 6. send the data until len becomes 0
    while (Len > 0)
    {
        // wait until TXE is set
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
            ;
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    // 7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
    //    Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
    //    when BTF=1 SCL will be stretched (pulled to LOW)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
        ;

    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF))
        ;

    // 8. Generate STOP condition and master need not to wait for the completion of stop condition.
    //    Note: generating STOP, automatically clears the BTF
    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    // slave address + r/nw bit = 0
    SlaveAddr &= ~(1);
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
    uint32_t dummyRead = pI2Cx->SR1;
    dummyRead = pI2Cx->SR2;
    (void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
