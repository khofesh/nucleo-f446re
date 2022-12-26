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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr);

/**
 * @brief IRQ configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

// application callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEvent);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
