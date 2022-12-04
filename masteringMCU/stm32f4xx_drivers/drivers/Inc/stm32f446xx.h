/*
 * stm32f446xx.h
 *
 *  Created on: Nov 28, 2022
 *      Author: fahmad
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo volatile

/**
 * base addresses of flash and SRAM memories
 */

// rm0390-*.pdf page 63
#define FLASH_BASEADDR 0x00000000UL
// rm0390-*.pdf page 62
#define SRAM1_BASEADDR 0x20000000UL
// rm0390-*.pdf page 62
#define SRAM2_BASEADDR 0x2001C000UL
// rm0390-*.pdf page 62 "System memory"
#define ROM_BASEADDR 0x1FFF0000UL
#define SRAM SRAM1_BASEADDR

/**
 * AHBx and APBx bus peripheral base addresses
 */

// rm0390-*.pdf page 59
#define PERIPH_BASEADDR 0x40000000UL
#define APB1PERIPH_BASEADDR PERIPH_BASE
#define APB2PERIPH_BASEADDR 0x40010000UL
#define AHB1PERIPH_BASEADDR 0x40020000UL
#define AHB2PERIPH_BASEADDR 0x50000000UL

/**
 * base addresses of peripherals which are hanging on
 * AHB1 bus
 */
#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00)

/**
 * base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + 0x4800)

#define UART4_BASEADDR (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASEADDR + 0x5000)

/**
 * base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400)

#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)

/************Peripheral register definition structures*********/

/**
 * RM0390-*.pdf - GPIO register map page 193
 * GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t*) 0x40020000;
 */
typedef struct
{
	// GPIO port mode register; address offset: 0x00
	__vo uint32_t MODER;
	// GPIO port output type register; address offset: 0x04
	__vo uint32_t OTYPER;
	// GPIO port output speed register; address offset: 0x08
	__vo uint32_t OSPEEDER;
	// GPIO port pull-up/pull-down register; address offset: 0x0C
	__vo uint32_t PUPDR;
	// GPIO port input data register; address offset: 0x10
	__vo uint32_t IDR;
	// GPIO port output data register; address offset: 0x14
	__vo uint32_t ODR;
	// GPIO port bit set/reset register; address offset: 0x18
	__vo uint32_t BSRR;
	// GPIO port configuration lock register; address offset: 0x1C
	__vo uint32_t LCKR;
	/**
	 * AFR[0]: GPIO alternate function low register; address offset: AFRL - 0x20
	 * AFR[1]: GPIO alternate function high register; address offset: AFRH - 0x24
	 */
	__vo uint32_t AFR[2];
} GPIO_RegDef_t;

/**
 * RM0390-*.pdf - RCC register map page 172
 */
typedef struct
{
	__vo uint32_t CR;		  /*!< RCC clock control register,                                  Address offset: 0x00 */
	__vo uint32_t PLLCFGR;	  /*!< RCC PLL configuration register,                              Address offset: 0x04 */
	__vo uint32_t CFGR;		  /*!< RCC clock configuration register,                            Address offset: 0x08 */
	__vo uint32_t CIR;		  /*!< RCC clock interrupt register,                                Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;	  /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;	  /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
	__vo uint32_t AHB3RSTR;	  /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
	uint32_t RESERVED0;		  /*!< Reserved, 0x1C                                                                    */
	__vo uint32_t APB1RSTR;	  /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
	__vo uint32_t APB2RSTR;	  /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
	uint32_t RESERVED1[2];	  /*!< Reserved, 0x28-0x2C                                                               */
	__vo uint32_t AHB1ENR;	  /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
	__vo uint32_t AHB2ENR;	  /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
	__vo uint32_t AHB3ENR;	  /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
	uint32_t RESERVED2;		  /*!< Reserved, 0x3C                                                                    */
	__vo uint32_t APB1ENR;	  /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
	__vo uint32_t APB2ENR;	  /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
	uint32_t RESERVED3[2];	  /*!< Reserved, 0x48-0x4C                                                               */
	__vo uint32_t AHB1LPENR;  /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;  /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	__vo uint32_t AHB3LPENR;  /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	uint32_t RESERVED4;		  /*!< Reserved, 0x5C                                                                    */
	__vo uint32_t APB1LPENR;  /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	__vo uint32_t APB2LPENR;  /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	uint32_t RESERVED5[2];	  /*!< Reserved, 0x68-0x6C                                                               */
	__vo uint32_t BDCR;		  /*!< RCC Backup domain control register,                          Address offset: 0x70 */
	__vo uint32_t CSR;		  /*!< RCC clock control & status register,                         Address offset: 0x74 */
	uint32_t RESERVED6[2];	  /*!< Reserved, 0x78-0x7C                                                               */
	__vo uint32_t SSCGR;	  /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR; /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
	__vo uint32_t PLLSAICFGR; /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
	__vo uint32_t DCKCFGR;	  /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
	__vo uint32_t CKGATENR;	  /*!< RCC Clocks Gated ENable Register,                            Address offset: 0x90 */
	__vo uint32_t DCKCFGR2;	  /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */
} RCC_RegDef_t;

/**
 * peripheral definitions
 * (peripheral base addresses type-casted to xxxx_RegDef_t)
 */

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)

#endif /* INC_STM32F446XX_H_ */
