/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Dec 27, 2022
 *      Author: fahmad
 */

#include "stm32f446xx_usart_driver.h"
#include "stm32f466xx_rcc_driver.h"

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pUSARTx == USART1)
        {
            USART1_PCLK_EN();
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_EN();
        }
        else if (pUSARTx == USART3)
        {
            USART3_PCLK_EN();
        }
        else if (pUSARTx == UART4)
        {
            UART4_PCLK_EN();
        }
    }
    else
    {
        if (pUSARTx == USART1)
        {
            USART1_PCLK_DI();
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_DI();
        }
        else if (pUSARTx == USART3)
        {
            USART3_PCLK_DI();
        }
        else if (pUSARTx == UART4)
        {
            UART4_PCLK_DI();
        }
    }
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
    // Temporary variable
    uint32_t tempreg = 0;

    /* Configuration of CR1 */

    // Implement the code to enable the Clock for given USART peripheral
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

    // Enable USART Tx and Rx engines according to the USART_Mode configuration item
    if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        // Implement the code to enable the Receiver bit field
        tempreg |= (1 << USART_CR1_RE);
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        // Implement the code to enable the Transmitter bit field
        tempreg |= (1 << USART_CR1_TE);
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        // Implement the code to enable the both Transmitter and Receiver bit fields
        tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
    }

    // Implement the code to configure the Word length configuration item
    tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    // Configuration of parity control bit fields
    if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        // Implement the code to enale the parity control
        tempreg |= (1 << USART_CR1_PCE);

        // Implement the code to enable EVEN parity
        // Not required because by default EVEN parity will be selected once you enable the parity control
    }
    else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
    {
        // Implement the code to enable the parity control
        tempreg |= (1 << USART_CR1_PCE);

        // Implement the code to enable ODD parity
        tempreg |= (1 << USART_CR1_PS);
    }

    // Program the CR1 register
    pUSARTHandle->pUSARTx->CR1 = tempreg;

    /* Configuration of CR2 */

    tempreg = 0;

    // Implement the code to configure the number of stop bits inserted during USART frame transmission
    tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

    // Program the CR2 register
    pUSARTHandle->pUSARTx->CR2 = tempreg;

    /* Configuration of CR3 */

    tempreg = 0;

    // Configuration of USART hardware flow control
    if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
    {
        // Implement the code to enable CTS flow control
        tempreg |= (1 << USART_CR3_CTSE);
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
    {
        // Implement the code to enable RTS flow control
        tempreg |= (1 << USART_CR3_RTSE);
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        // Implement the code to enable both CTS and RTS Flow control
        tempreg |= (1 << USART_CR3_CTSE);
        tempreg |= (1 << USART_CR3_RTSE);
    }

    pUSARTHandle->pUSARTx->CR3 = tempreg;

    /* Configuration of BRR(Baudrate register) */

    // Implement the code to configure the baud rate
    // We will cover this in the lecture. No action required here
    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
    if (pUSARTx == USART1)
    {
        USART1_REG_RESET();
    }
    else if (pUSARTx == USART2)
    {
        USART2_REG_RESET();
    }
    else if (pUSARTx == USART3)
    {
        USART3_REG_RESET();
    }
    else if (pUSARTx == UART4)
    {
        UART4_REG_RESET();
    }
    else if (pUSARTx == UART5)
    {
        UART5_REG_RESET();
    }
    else if (pUSARTx == USART6)
    {
        USART6_REG_RESET();
    }
}

/**
 * @brief USART_SendData
 *
 * @param pUSARTHandle
 * @param pTxBuffer
 * @param Len
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint16_t *pdata;
    // Loop over until "Len" number of bytes are transferred
    for (uint32_t i = 0; i < Len; i++)
    {
        // Implement the code to wait until TXE flag is set in the SR
        while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE))
            ;

        // Check the USART_WordLength item for 9BIT or 8BIT in a frame
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            // if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
            pdata = (uint16_t *)pTxBuffer;
            pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

            // check for USART_ParityControl
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                // No parity is used in this transfer. so, 9bits of user data will be sent
                // Implement the code to increment pTxBuffer twice
                pTxBuffer++;
                pTxBuffer++;
            }
            else
            {
                // Parity bit is used in this transfer . so , 8bits of user data will be sent
                // The 9th bit will be replaced by parity bit by the hardware
                pTxBuffer++;
            }
        }
        else
        {
            // This is 8bit data transfer
            pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

            // Implement the code to increment the buffer address
            pTxBuffer++;
        }
    }

    // Implement the code to wait till TC flag is set in the SR
    while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC))
        ;
}

/**
 * @brief USART_ReceiveData
 *
 * @param pUSARTHandle
 * @param pRxBuffer
 * @param Len
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    // Loop over until "Len" number of bytes are transferred
    for (uint32_t i = 0; i < Len; i++)
    {
        // Implement the code to wait until RXNE flag is set in the SR
        while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE))
            ;

        // Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            // We are going to receive 9bit data in a frame

            // check are we using USART_ParityControl control or not
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                // No parity is used. so, all 9bits will be of user data

                // read only first 9 bits. so, mask the DR with 0x01FF
                *((uint16_t *)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

                // Now increment the pRxBuffer two times
                pRxBuffer++;
                pRxBuffer++;
            }
            else
            {
                // Parity is used, so, 8bits will be of user data and 1 bit is parity
                *pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);

                // Increment the pRxBuffer
                pRxBuffer++;
            }
        }
        else
        {
            // We are going to receive 8bit data in a frame

            // check are we using USART_ParityControl control or not
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                // No parity is used , so all 8bits will be of user data

                // read 8 bits from DR
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
            }

            else
            {
                // Parity is used, so , 7 bits will be of user data and 1 bit is parity

                // read only 7 bits , hence mask the DR with 0X7F
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
            }

            // increment the pRxBuffer
            pRxBuffer++;
        }
    }
}

/**
 * @brief USART_SendDataIT
 *
 * @param pUSARTHandle
 * @param pTxBuffer
 * @param Len
 * @return uint8_t
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t txState = pUSARTHandle->TxBusyState;

    if (txState != USART_BUSY_IN_TX)
    {
        pUSARTHandle->TxLen = Len;
        pUSARTHandle->pTxBuffer = pTxBuffer;
        pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

        // Implement the code to enable interrupt for TXE
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

        // Implement the code to enable interrupt for TC
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
    }

    return txState;
}

/**
 * @brief USART_ReceiveDataIT
 *
 * @param pUSARTHandle
 * @param pRxBuffer
 * @param Len
 * @return uint8_t
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t rxState = pUSARTHandle->RxBusyState;

    if (rxState != USART_BUSY_IN_RX)
    {
        pUSARTHandle->RxLen = Len;
        pUSARTHandle->pRxBuffer = pRxBuffer;
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

        (void)pUSARTHandle->pUSARTx->DR;

        // Implement the code to enable interrupt for RXNE
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }

    return rxState;
}

/**
 * @brief USART_IRQInterruptConfig
 *
 * @param IRQNumber
 * @param EnOrDi
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * @brief USART_IRQPriorityConfig
 *
 * @param IRQNumber
 * @param IRQPriority
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pUSARTx->CR1 |= (1 << 13);
    }
    else
    {
        pUSARTx->CR1 &= ~(1 << 13);
    }
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
    if (pUSARTx->SR & FlagName)
    {
        return SET;
    }

    return RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName)
{
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    // Variable to hold the APB clock
    uint32_t PCLKx;

    uint32_t usartdiv;

    // variables to hold Mantissa and Fraction values
    uint32_t M_part, F_part;

    uint32_t tempreg = 0;

    // Get the value of APB bus clock in to the variable PCLKx
    if (pUSARTx == USART1 || pUSARTx == USART6)
    {
        // USART1 and USART6 are hanging on APB2 bus
        PCLKx = RCC_GetPCLK2Value();
    }
    else
    {
        PCLKx = RCC_GetPCLK1Value();
    }

    // Check for OVER8 configuration bit
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        // OVER8 = 1 , over sampling by 8
        usartdiv = ((25 * PCLKx) / (2 * BaudRate));
    }
    else
    {
        // over sampling by 16
        usartdiv = ((25 * PCLKx) / (4 * BaudRate));
    }

    // Calculate the Mantissa part
    M_part = usartdiv / 100;

    // Place the Mantissa part in appropriate bit position . refer USART_BRR
    tempreg |= M_part << 4;

    // Extract the fraction part
    F_part = (usartdiv - (M_part * 100));

    // Calculate the final fractional
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        // OVER8 = 1 , over sampling by 8
        F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
    }
    else
    {
        // over sampling by 16
        F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
    }

    // Place the fractional part in appropriate bit position . refer USART_BRR
    tempreg |= F_part;

    // copy the value of tempreg in to BRR register
    pUSARTx->BRR = tempreg;
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{
}
