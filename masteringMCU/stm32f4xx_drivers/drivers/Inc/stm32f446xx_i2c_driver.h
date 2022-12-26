/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Dec 22, 2022
 *      Author: fahmad
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/**
 * @brief configuration structure for I2Cx peripheral
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
} I2C_Config_t;

/**
 * @brief handle structure for I2Cx peripheral
 */
typedef struct
{
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
    /* Stores application Tx buffer address */
    uint8_t *pTxBuffer;
    /* Stores application Rx buffer address */
    uint8_t *pRxBuffer;
    /* Stores Tx length */
    uint32_t TxLen;
    /* Stores Rx length */
    uint32_t RxLen;
    /* Stores communication state */
    uint8_t TxRxState;
    /* Stores slave/device address */
    uint8_t DevAddr;
    /* Stores Rx size */
    uint32_t RxSize;
    /* Stores repeated start value */
    uint8_t Sr;
} I2C_Handle_t;

/**
 * @brief I2C_SCLSpeed
 */

// standard mode 100kHz
#define I2C_SCL_SPEED_SM 100000
// fast mode 400kHz
#define I2C_SCL_SPEED_FM4K 400000
// fast mode 200kHz
#define I2C_SCL_SPEED_FM2K 200000

/**
 * @brief I2C_ACKControl
 * see RM0390*-.pdf page 781
 * Bit 10 ACK: Acknowledge enable
 * This bit is set and cleared by software and cleared by hardware when PE=0.
 * 0: No acknowledge returned
 * 1: Acknowledge returned after a byte is received (matched address or data)
 */

#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

/**
 * @brief I2C_FMDutyCycle
 * see RM0390*-.pdf page 791
 */
#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1

/**
 * @brief I2C related status flags definitions
 */
#define I2C_FLAG_TXE (1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE (1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF (1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR (1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT (1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR RESET
#define I2C_ENABLE_SR SET

/**
 * @brief Possible I2C Application states
 */
#define I2C_READY 0
#define I2C_BUSY_IN_RX 1
#define I2C_BUSY_IN_TX 2

/**
 * @brief I2C application events
 *
 */
#define I2C_EV_TX_CMPLT 0
#define I2C_EV_RX_CMPLT 1
#define I2C_EV_STOP 2
#define I2C_ERROR_BERR 3
#define I2C_ERROR_ARLO 4
#define I2C_ERROR_AF 5
#define I2C_ERROR_OVR 6
#define I2C_ERROR_TIMEOUT 7
#define I2C_EV_DATA_REQ 8
#define I2C_EV_DATA_RCV 9

/**
 * @brief peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/**
 * @brief init and de-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/**
 * @brief data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/**
 * @brief IRQ configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/**
 * @brief other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

// application callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
