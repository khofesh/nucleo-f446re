/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Dec 22, 2022
 *      Author: fahmad
 */

#include "stm32f446xx_i2c_driver.h"
#include "stm32f466xx_rcc_driver.h"

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

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
    if (EnOrDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            /* Program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ISER1 register (32 to 63) */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ISER2 register (64 to 95) */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            /* Program ICER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ICER1 register (32 to 63) */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ICER2 register (64 to 95) */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
}

/**
 * @brief I2C_IRQPriorityConfig
 *
 * @param IRQNumber
 * @param IRQPriority
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
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
 * @return __weak
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent)
{
}

/**
 * @brief I2C_MasterSendData
 *
 * @param pI2CHandle
 * @param pTxBuffer
 * @param Len
 * @param SlaveAddr
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                        uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
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
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    // 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
    {
        /* code */
    }

    // 5. clear the ADDR flag according to its software sequence
    //    Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
    I2C_ClearADDRFlag(pI2CHandle);

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
    if (Sr == I2C_DISABLE_SR)
    {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }
}

/**
 * @brief I2C_MasterReceiveData
 *
 * @param pI2CHandle
 * @param pRxBuffer
 * @param Len
 * @param SlaveAddr
 * @param Sr
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                           uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    // 1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2. confirm that start generation is completed by checking the SB flag in the SR1
    //    Note: Until SB is cleared SCL will be stretched (pulled to LOW)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
    {
        /* code */
    }

    // 3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

    // 4. wait until address phase is completed by checking the ADDR flag in teh SR1
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
    {
        /* code */
    }

    // procedure to read only 1 byte from slave
    if (Len == 1)
    {
        // Disable Acking
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

        // clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        // wait until  RXNE becomes 1
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
            ;

        // generate STOP condition
        if (Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        // read data in to buffer
        *pRxBuffer = pI2CHandle->pI2Cx->DR;
    }

    // procedure to read data from slave when Len > 1
    if (Len > 1)
    {
        // clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        // read the data until Len becomes zero
        for (uint32_t i = Len; i > 0; i--)
        {
            // wait until RXNE becomes 1
            while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
                ;

            if (i == 2) // if last 2 bytes are remaining
            {
                // Disable Acking
                I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

                // generate STOP condition
                if (Sr == I2C_DISABLE_SR)
                {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
            }

            // read the data from data register in to buffer
            *pRxBuffer = pI2CHandle->pI2Cx->DR;

            // increment the buffer address
            pRxBuffer++;
        }
    }

    // re-enable ACKing
    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
    }
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == I2C_ACK_ENABLE)
    {
        /* enable the ack */
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    }
    else
    {
        // disable the ack
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                             uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

    uint8_t busyState = pI2CHandle->TxRxState;

    if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        // Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        // Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        // Implement the code to enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        // Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busyState;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                                uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busyState = pI2CHandle->TxRxState;

    if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->RxSize = Len; // Rxsize is used in the ISR code to manage the data reception
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        // Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        // Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        // Implement the code to enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        // Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busyState;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    // Interrupt handling for both master and slave mode of a device
    uint32_t temp1;
    uint32_t temp2;
    uint32_t temp3;

    temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

    // 1. Handle For interrupt generated by SB event
    //	Note : SB flag is only applicable in Master mode
    if (temp1 && temp3)
    {
        /* SB is set
           the interrupt is generated because of SB event
        */
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
    // 2. Handle For interrupt generated by ADDR event
    // Note : When master mode : Address is sent
    //		 When Slave mode   : Address matched with own address
    if (temp1 && temp3)
    {
        /* ADDR is set */
        I2C_ClearADDRFlag(pI2CHandle);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
    // 3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
    if (temp1 && temp3)
    {
        /* BTF is set */
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            // make sure that TXE is also set
            if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
            {
                /* BTF, TXE = 1 */

                if (pI2CHandle->TxLen == 0)
                {
                    // 1. generate the STOP condition
                    if (pI2CHandle->Sr == I2C_DISABLE_SR)
                    {
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    }

                    // 2. reset all the member elements of the handle structure
                    I2C_CloseSendData(pI2CHandle);

                    // 3. notify the application about the submission complete
                    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
                }
            }
        }
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            /* code */
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
    // 4. Handle For interrupt generated by STOPF event
    //  Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
    // The below code block will not be executed by the master since STOPF will not set in master mode
    if (temp1 && temp3)
    {
        /* STOPF is set
         * clear the STOPF:
         * 1- read SR1
         * 2. write to CR1
         */
        pI2CHandle->pI2Cx->CR1 |= 0x0000;

        // notify the application that STOP is detected
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
    // 5. Handle For interrupt generated by TXE event
    if (temp1 && temp2 && temp3)
    {
        // check if the device is master
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            /* TXE is set
             * do the data transmission
             */

            if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                I2C_MasterHandleTXEInterrupt(pI2CHandle);
            }
        }
        else
        {
            /* slave */
            // make sure that the slave is really in transmitter mode
            if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
    // 6. Handle For interrupt generated by RXNE event
    if (temp1 && temp2 && temp3)
    {
        // check if the device is master
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            /* RXNE is set */
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
            }
        }
        else
        {
            /* slave */
            // make sure that the slave is really in transmitter mode
            if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1;
    uint32_t temp2;

    // Know the status of  ITERREN control bit in the CR2
    temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

    /* Check for Bus error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
    if (temp1 && temp2)
    {
        // This is Bus error

        // Implement the code to clear the buss error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    /* Check for arbitration lost error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
    if (temp1 && temp2)
    {
        // This is arbitration lost error

        // Implement the code to clear the arbitration lost error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    /* Check for ACK failure  error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
    if (temp1 && temp2)
    {
        // This is ACK failure error

        // Implement the code to clear the ACK failure error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    /* Check for Overrun/underrun error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
    if (temp1 && temp2)
    {
        // This is Overrun/underrun

        // Implement the code to clear the Overrun/underrun error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    /* Check for Time out error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
    if (temp1 && temp2)
    {
        // This is Time out error

        // Implement the code to clear the Time out error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
    if (pI2CHandle->TxLen > 0)
    {
        // 1. load the data into DR
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

        // 2. decreement the TxLen
        pI2CHandle->TxLen--;

        // 3. increment the buffer address
        pI2CHandle->pTxBuffer++;
    }
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
    // we have to do the data reception
    if (pI2CHandle->RxSize == 1)
    {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxSize > 1)
    {
        if (pI2CHandle->RxLen == 2)
        {
            // clear the ack bit
            I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
        }

        // read DR
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxLen == 0)
    {
        /* close the I2C data reception and notify the application */

        // 1. generate the stop condition
        if (pI2CHandle->Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        // 2. close the I2C rx
        I2C_CloseReceiveData(pI2CHandle);

        // 3. notify the application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
    }
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    // implement the code to disable ITBUFEN control bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    // implement the code to disable ITEVFEN control bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    // slave address + r/nw bit = 0
    SlaveAddr &= ~(1);
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    // slave address + r/nw bit = 1
    SlaveAddr |= 1;
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
    uint32_t dummy_read;
    // check for device mode
    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
    {
        /* master mode */
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            if (pI2CHandle->RxSize == 1)
            {
                /* disable the ack */
                I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

                // clear the ADDR flag
                dummy_read = pI2CHandle->pI2Cx->SR1;
                dummy_read = pI2CHandle->pI2Cx->SR2;
                (void)dummy_read;
            }
        }
        else
        {
            // clear the ADDR flag
            dummy_read = pI2CHandle->pI2Cx->SR1;
            dummy_read = pI2CHandle->pI2Cx->SR2;
            (void)dummy_read;
        }
    }
    else
    {
        /* slave mode */
        // clear the ADDR flag
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void)dummy_read;
    }
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
    pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
    return (uint8_t)pI2C->DR;
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }
    else
    {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
    }
}
