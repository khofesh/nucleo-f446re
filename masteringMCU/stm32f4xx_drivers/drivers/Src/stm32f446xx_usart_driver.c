/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Dec 27, 2022
 *      Author: fahmad
 */

#include <stm32f446xx_usart_driver.h>

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
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

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length)
{
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Length)
{
}

uint8_t USART_SendDataInterrupt(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length)
{
    return 0;
}

uint8_t USART_ReceiveDataInterrupt(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Length)
{
    return 0;
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
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
    return 0;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName)
{
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{
}
