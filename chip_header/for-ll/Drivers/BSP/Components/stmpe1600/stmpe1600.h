/**
  ******************************************************************************
  * @file    stmpe1600.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the
  *          stmpe1600.c IO expander driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STMPE1600_H
#define __STMPE1600_H

#ifdef __cplusplus
 extern "C" {
#endif   
   
/* Includes ------------------------------------------------------------------*/
#include "../Common/io.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Component
  * @{
  */
    
/** @defgroup STMPE1600
  * @{
  */    

/* Exported types ------------------------------------------------------------*/

/** @defgroup STMPE1600_Exported_Types
  * @{
  */ 

/* Exported constants --------------------------------------------------------*/
  
/** @defgroup STMPE1600_Exported_Constants
  * @{
  */ 

/** 
  * @brief STMPE1600 chip IDs  
  */ 
#define STMPE1600_ID                     0x1600

/** 
  * @brief  Interrupt enable  
  */ 
#define STMPE1600_IT_ENABLE              0x04

/** 
  * @brief  Identification registers & System Control  
  */ 
#define STMPE1600_REG_CHP_ID             0x00
#define STMPE1600_REG_ID_VERSION         0x02
#define STMPE1600_REG_SYS_CTRL           0x03

/** 
  * @brief  IO Registers  
  */ 

#define STMPE1600_REG_GPMR               0x10
#define STMPE1600_REG_GPSR               0x12
#define STMPE1600_REG_GPDR               0x14
#define STMPE1600_REG_GPPIR              0x16
   
/** 
  * @brief  Interrupt Control registers  
  */ 
#define STMPE1600_REG_IEGPIOR            0x08
#define STMPE1600_REG_ISGPIOR            0x0A

/** 
  * @brief  IO Pins direction  
  */
#define STMPE1600_DIRECTION_IN           0x00
#define STMPE1600_DIRECTION_OUT          0x01

/** 
  * @brief  IO IT polarity  
  */
#define STMPE1600_POLARITY_LOW           0x00
#define STMPE1600_POLARITY_HIGH          0x01

/** 
  * @brief  IO Pins  
  */     
#define STMPE1600_PIN_0                  0x0001
#define STMPE1600_PIN_1                  0x0002
#define STMPE1600_PIN_2                  0x0004
#define STMPE1600_PIN_3                  0x0008
#define STMPE1600_PIN_4                  0x0010
#define STMPE1600_PIN_5                  0x0020
#define STMPE1600_PIN_6                  0x0040
#define STMPE1600_PIN_7                  0x0080
#define STMPE1600_PIN_8                  0x0100
#define STMPE1600_PIN_9                  0x0200
#define STMPE1600_PIN_10                 0x0400
#define STMPE1600_PIN_11                 0x0800
#define STMPE1600_PIN_12                 0x1000
#define STMPE1600_PIN_13                 0x2000
#define STMPE1600_PIN_14                 0x4000
#define STMPE1600_PIN_15                 0x8000
#define STMPE1600_PIN_ALL                0xFFFF    

/**
  * @}
  */ 
  
/* Exported macro ------------------------------------------------------------*/
  
/** @defgroup STMPE1600_Exported_Macros
  * @{
  */ 

/* Exported functions --------------------------------------------------------*/
  
/** @defgroup STMPE1600_Exported_Functions
  * @{
  */
  
/** 
  * @brief STMPE1600 Control functions
  */
void     stmpe1600_Init(uint16_t DeviceAddr);
void     stmpe1600_Reset(uint16_t DeviceAddr);
uint16_t stmpe1600_ReadID(uint16_t DeviceAddr);
void     stmpe1600_SetITPolarity(uint16_t DeviceAddr, uint8_t Polarity);
void     stmpe1600_EnableGlobalIT(uint16_t DeviceAddr);
void     stmpe1600_DisableGlobalIT(uint16_t DeviceAddr);

/** 
  * @brief STMPE1600 IO functionalities functions
  */
void     stmpe1600_IO_InitPin(uint16_t DeviceAddr, uint32_t IO_Pin, uint8_t Direction);
uint8_t  stmpe1600_IO_Config(uint16_t DeviceAddr, uint32_t IO_Pin, IO_ModeTypedef IO_Mode);
void     stmpe1600_IO_PolarityInv_Enable(uint16_t DeviceAddr, uint32_t IO_Pin);
void     stmpe1600_IO_PolarityInv_Disable(uint16_t DeviceAddr, uint32_t IO_Pin);
void     stmpe1600_IO_WritePin(uint16_t DeviceAddr, uint32_t IO_Pin, uint8_t PinState);
uint32_t stmpe1600_IO_ReadPin(uint16_t DeviceAddr, uint32_t IO_Pin);
void     stmpe1600_IO_EnablePinIT(uint16_t DeviceAddr, uint32_t IO_Pin);
void     stmpe1600_IO_DisablePinIT(uint16_t DeviceAddr, uint32_t IO_Pin);
uint32_t stmpe1600_IO_ITStatus(uint16_t DeviceAddr, uint32_t IO_Pin);
uint8_t  stmpe1600_IO_ReadIT(uint16_t DeviceAddr, uint32_t IO_Pin);
void     stmpe1600_IO_ClearIT(uint16_t DeviceAddr, uint32_t IO_Pin);
void     stmpe1600_Start(uint16_t DeviceAddr, uint32_t IO_Pin);

void     IOE_Init(void);
void     IOE_ITConfig (void);
void     IOE_Delay(uint32_t delay);
void     IOE_Write(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t  IOE_Read(uint8_t addr, uint8_t reg);
uint16_t IOE_ReadMultiple(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);
void     IOE_WriteMultiple(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);

/* STMPE1600 driver structure */
extern IO_DrvTypeDef stmpe1600_io_drv;


#ifdef __cplusplus
}
#endif
#endif /* __STMPE1600_H */

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */
