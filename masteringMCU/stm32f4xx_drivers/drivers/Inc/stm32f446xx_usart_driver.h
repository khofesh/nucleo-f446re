/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Dec 27, 2022
 *      Author: fahmad
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"

/**
 * @brief configuration structure for USARTx peripheral
 */
typedef struct
{
    uint8_t USART_Mode;
    uint32_t USART_Baud;
    uint8_t USART_NoOfStopBits;
    uint8_t USART_WordLength;
    uint8_t USART_ParityControl;
    uint8_t USART_HWFlowControl;
} USART_Config_t;

/**
 * @brief handle structure for USARTx peripheral
 */
typedef struct
{
    USART_RegDef_t *pUSARTx;
    USART_Config_t USART_Config;
} USART_Handle_t;

/**
 * @brief USART_Mode
 * Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX 2

/**
 * @brief USART_Baud
 * Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200 1200
#define USART_STD_BAUD_2400 400
#define USART_STD_BAUD_9600 9600
#define USART_STD_BAUD_19200 19200
#define USART_STD_BAUD_38400 38400
#define USART_STD_BAUD_57600 57600
#define USART_STD_BAUD_115200 115200
#define USART_STD_BAUD_230400 230400
#define USART_STD_BAUD_460800 460800
#define USART_STD_BAUD_921600 921600
#define USART_STD_BAUD_2M 2000000
#define SUART_STD_BAUD_3M 3000000

/**
 * @brief USART_ParityControl
 * Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD 2
#define USART_PARITY_EN_EVEN 1
#define USART_PARITY_DISABLE 0

/**
 * @brief USART_WordLength
 * Possible options for USART_WordLength
 * (USART_CR1) bit 12
 */
#define USART_WORDLEN_8BITS 0
#define USART_WORDLEN_9BITS 1

/**
 * @brief USART_NoOfStopBits
 * Possible options for USART_NoOfStopBits
 * (USART_CR2) bit 12 and 13
 */
#define USART_STOPBITS_1 0
#define USART_STOPBITS_0_5 1
#define USART_STOPBITS_2 2
#define USART_STOPBITS_1_5 3

/**
 * @brief USART_HWFlowControl
 * Possible options for USART_HWFlowControl
 * (USART_CR3) bit 9 and 8
 */
#define USART_HW_FLOW_CTRL_NONE 0
#define USART_HW_FLOW_CTRL_CTS 1
#define USART_HW_FLOW_CTRL_RTS 2
#define USART_HW_FLOW_CTRL_CTS_RTS 3

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
