/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Dec 17, 2022
 *      Author: fahmad
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stdbool.h"
#include "stm32f446xx.h"

/**
 * @brief configuration structure for SPIx peripheral
 * see RM0390-*.pdf page 857
 */
typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} SPI_Config_t;

/**
 * @brief handle structure for SPIx peripheral
 *
 */
typedef struct
{
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPIConfig;
    /* !<to store the app Tx buffer address> */
    uint8_t *pTxBuffer;
    /* !<to store the app Rx buffer address> */
    uint8_t *pRxBuffer;
    /* !<to store Tx len> */
    uint32_t TxLen;
    /* !<to store Rx len> */
    uint32_t RxLen;
    /* !<to store Tx state> */
    uint8_t TxState;
    /* !<to store Rx state> */
    uint8_t RxState;
} SPI_Handle_t;

/**
 * @brief possible SPI application states
 */
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

/**
 * @brief possible SPI application events
 *
 */
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR 3
#define SPI_EVENT_CRC_ERR 4

/**
 * @brief SPI_DeviceMode
 * slave or master
 */
// SPI_DeviceMode master
#define SPI_DEVICE_MODE_MASTER 1
// SPI_DeviceMode slave
#define SPI_DEVICE_MODE_SLAVE 0

/**
 * @brief SPI_BusConfig
 */
// SPI_BusConfig full-duplex
#define SPI_BUS_CONFIG_FD 1
// SPI_BusConfig half-duplex
#define SPI_BUS_CONFIG_HD 2
// SPI_BusConfig simplex - receive only
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3

/**
 * @brief SPI_SclkSpeed
 * clock speed
 * see rm0390-*.pdf page 887
 * "Bits 5:3 BR[2:0]: Baud rate control"
 */
// SPI_SclkSpeed 000: fPCLK/2
#define SPI_SCLK_SPEED_DIV2 0
// SPI_SclkSpeed 001: fPCLK/4
#define SPI_SCLK_SPEED_DIV4 1
// SPI_SclkSpeed 010: fPCLK/8
#define SPI_SCLK_SPEED_DIV8 2
// SPI_SclkSpeed 011: fPCLK/16
#define SPI_SCLK_SPEED_DIV16 3
// SPI_SclkSpeed 100: fPCLK/32
#define SPI_SCLK_SPEED_DIV32 4
// SPI_SclkSpeed 101: fPCLK/64
#define SPI_SCLK_SPEED_DIV64 5
// SPI_SclkSpeed 110: fPCLK/128
#define SPI_SCLK_SPEED_DIV128 6
// SPI_SclkSpeed 111: fPCLK/256
#define SPI_SCLK_SPEED_DIV256 7

/**
 * @brief SPI_DFF
 */
// SPI_DFF 8 bits
#define SPI_DFF_8BITS 0
// SPI_DFF 16 bits
#define SPI_DFF_16BITS 1

/**
 * @brief SPI_CPOL
 */
// SPI_CPOL high
#define SPI_CPOL_HIGH 1
// SPI_CPOL low
#define SPI_CPOL_LOW 0

/**
 * @brief SPI_CPHA
 */
// SPI_CPHA high
#define SPI_CPHA_HIGH 1
// SPI_CPHA low
#define SPI_CPHA_LOW 0

/**
 * @brief SPI_SSM
 */
// SPI_SSM Software slave management enabled
#define SPI_SSM_EN 1
// SPI_SSM Software slave management disabled
#define SPI_SSM_DI 0

/**
 * @brief SPI related status flags definitions
 */
#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG (1 << SPI_SR_BSY)

/**
 * @brief peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/**
 * @brief init and de-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @brief data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * @brief IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandler(SPI_Handle_t *pHandle);

/**
 * @brief other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
bool SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

// application callback
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
