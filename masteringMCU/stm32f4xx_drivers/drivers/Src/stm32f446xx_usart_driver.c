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
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length)
{
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Length)
{
}

uint8_t USART_SendDataInterrupt(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length)
{
}

uint8_t USART_ReceiveDataInterrupt(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Length)
{
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
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
