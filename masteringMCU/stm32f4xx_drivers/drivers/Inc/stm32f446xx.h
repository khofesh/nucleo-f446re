/*
 * stm32f446xx.h
 *
 *  Created on: Nov 28, 2022
 *      Author: fahmad
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/* START: PROCESSOR SPECIFIC DETAIL */

/**
 * @brief ARM cortex Mx process NVIC ISERx register addresses
 * see Cortex-M4-devices-generic-user-guide.pdf page 219
 * `4-2 Nested Vectored Interrupt Controller`
 */
#define NVIC_ISER0 ((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t *)0xE000E10C)
#define NVIC_ISER4 ((__vo uint32_t *)0xE000E110)
#define NVIC_ISER5 ((__vo uint32_t *)0xE000E114)
#define NVIC_ISER6 ((__vo uint32_t *)0xE000E118)
#define NVIC_ISER7 ((__vo uint32_t *)0xE000E11C)

/**
 * @brief ARM Cortex Mx processor NVIC ICERx register addresses
 */
#define NVIC_ICER0 ((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1 ((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2 ((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3 ((__vo uint32_t *)0xE000E18C)
#define NVIC_ICER4 ((__vo uint32_t *)0xE000E190)
#define NVIC_ICER5 ((__vo uint32_t *)0xE000E194)
#define NVIC_ICER6 ((__vo uint32_t *)0xE000E198)
#define NVIC_ICER7 ((__vo uint32_t *)0xE000E19C)

/**
 * @brief ARM Cortex Mx processor priority register address calculation
 *
 */
#define NVIC_PR_BASE_ADDR ((__vo uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4

/* END */

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
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
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
#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x3800)

/**
 * @brief IRQ (interrupt request) numbers of STM32F446xx
 * rm0390-*.pdf page 240
 */
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40

#define IRQ_NO_SPI1 35
#define IRQ_NO_SPI2 36
#define IRQ_NO_SPI3 51
#define IRQ_NO_SPI4 84

#define IRQ_NO_USART1 37
#define IRQ_NO_USART2 38
#define IRQ_NO_USART3 39

/**
 * @brief possible priority levels
 *
 */

#define NVIC_IRQ_PRI0 0
#define NVIC_IRQ_PRI15 15
#define NVIC_IRQ_PRI47 47

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
#define SPI4_BASEADDR (APB2PERIPH_BASEADDR + 0x3400)

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
	/*!< RCC clock control register, Address offset: 0x00 */
	__vo uint32_t CR;
	/*!< RCC PLL configuration register, Address offset: 0x04 */
	__vo uint32_t PLLCFGR;
	/*!< RCC clock configuration register, Address offset: 0x08 */
	__vo uint32_t CFGR;
	/*!< RCC clock interrupt register, Address offset: 0x0C */
	__vo uint32_t CIR;
	/*!< RCC AHB1 peripheral reset register, Address offset: 0x10 */
	__vo uint32_t AHB1RSTR;
	/*!< RCC AHB2 peripheral reset register, Address offset: 0x14 */
	__vo uint32_t AHB2RSTR;
	/*!< RCC AHB3 peripheral reset register, Address offset: 0x18 */
	__vo uint32_t AHB3RSTR;
	/*!< Reserved, 0x1C */
	uint32_t RESERVED0;
	/*!< RCC APB1 peripheral reset register, Address offset: 0x20 */
	__vo uint32_t APB1RSTR;
	/*!< RCC APB2 peripheral reset register, Address offset: 0x24 */
	__vo uint32_t APB2RSTR;
	/*!< Reserved, 0x28-0x2C  */
	uint32_t RESERVED1[2];
	/*!< RCC AHB1 peripheral clock register, Address offset: 0x30 */
	__vo uint32_t AHB1ENR;
	/*!< RCC AHB2 peripheral clock register, Address offset: 0x34 */
	__vo uint32_t AHB2ENR;
	/*!< RCC AHB3 peripheral clock register, Address offset: 0x38 */
	__vo uint32_t AHB3ENR;
	/*!< Reserved, 0x3C */
	uint32_t RESERVED2;
	/*!< RCC APB1 peripheral clock enable register, Address offset: 0x40 */
	__vo uint32_t APB1ENR;
	/*!< RCC APB2 peripheral clock enable register, Address offset: 0x44 */
	__vo uint32_t APB2ENR;
	/*!< Reserved, 0x48-0x4C */
	uint32_t RESERVED3[2];
	/*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	__vo uint32_t AHB1LPENR;
	/*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	__vo uint32_t AHB2LPENR;
	/*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	__vo uint32_t AHB3LPENR;
	/*!< Reserved, 0x5C */
	uint32_t RESERVED4;
	/*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	__vo uint32_t APB1LPENR;
	/*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	__vo uint32_t APB2LPENR;
	/*!< Reserved, 0x68-0x6C */
	uint32_t RESERVED5[2];
	/*!< RCC Backup domain control register, Address offset: 0x70 */
	__vo uint32_t BDCR;
	/*!< RCC clock control & status register, Address offset: 0x74 */
	__vo uint32_t CSR;
	/*!< Reserved, 0x78-0x7C */
	uint32_t RESERVED6[2];
	/*!< RCC spread spectrum clock generation register, Address offset: 0x80 */
	__vo uint32_t SSCGR;
	/*!< RCC PLLI2S configuration register, Address offset: 0x84 */
	__vo uint32_t PLLI2SCFGR;
	/*!< RCC PLLSAI configuration register, Address offset: 0x88 */
	__vo uint32_t PLLSAICFGR;
	/*!< RCC Dedicated Clocks configuration register, Address offset: 0x8C */
	__vo uint32_t DCKCFGR;
	/*!< RCC Clocks Gated ENable Register, Address offset: 0x90 */
	__vo uint32_t CKGATENR;
	/*!< RCC Dedicated Clocks configuration register 2, Address offset: 0x94 */
	__vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

/**
 * @brief External Interrupt/Event Controller
 * see RM0390-*.pdf page 247
 */
typedef struct
{
	/*!< EXTI Interrupt mask register,            Address offset: 0x00 */
	__vo uint32_t IMR;
	/*!< EXTI Event mask register,                Address offset: 0x04 */
	__vo uint32_t EMR;
	/*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
	__vo uint32_t RTSR;
	/*!< EXTI Falling trigger selection register, Address offset: 0x0C */
	__vo uint32_t FTSR;
	/*!< EXTI Software interrupt event register,  Address offset: 0x10 */
	__vo uint32_t SWIER;
	/*!< EXTI Pending register,                   Address offset: 0x14 */
	__vo uint32_t PR;
} EXTI_RegDef_t;

/**
 * @brief Serial Peripheral Interface
 * see RM0390-*.pdf page 886
 * "26.7 SPI and I2S registers"
 */
typedef struct
{
	/*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
	__vo uint32_t CR1;
	/*!< SPI control register 2,                             Address offset: 0x04 */
	__vo uint32_t CR2;
	/*!< SPI status register,                                Address offset: 0x08 */
	__vo uint32_t SR;
	/*!< SPI data register,                                  Address offset: 0x0C */
	__vo uint32_t DR;
	/*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
	__vo uint32_t CRCPR;
	/*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
	__vo uint32_t RXCRCR;
	/*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
	__vo uint32_t TXCRCR;
	/*!< SPI_I2S configuration register,                     Address offset: 0x1C */
	__vo uint32_t I2SCFGR;
	/*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

/**
 * @brief System configuration controller
 * see RM0390-*.pdf page 195
 */
typedef struct
{
	/*!< SYSCFG memory remap register,                      Address offset: 0x00      */
	__vo uint32_t MEMRMP;
	/*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
	__vo uint32_t PMC;
	/*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
	__vo uint32_t EXTICR[4];
	/*!< Reserved, 0x18-0x1C                                                          */
	uint32_t RESERVED[2];
	/*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
	__vo uint32_t CMPCR;
	/*!< Reserved, 0x24-0x28                                                          */
	uint32_t RESERVED1[2];
	/*!< SYSCFG Configuration register,                     Address offset: 0x2C      */
	__vo uint32_t CFGR;
} SYSCFG_RegDef_t;

/**
 * @brief Inter-integrated Circuit Interface
 */

typedef struct
{
	/*!< I2C Control register 1,     Address offset: 0x00 */
	__vo uint32_t CR1;
	/*!< I2C Control register 2,     Address offset: 0x04 */
	__vo uint32_t CR2;
	/*!< I2C Own address register 1, Address offset: 0x08 */
	__vo uint32_t OAR1;
	/*!< I2C Own address register 2, Address offset: 0x0C */
	__vo uint32_t OAR2;
	/*!< I2C Data register,          Address offset: 0x10 */
	__vo uint32_t DR;
	/*!< I2C Status register 1,      Address offset: 0x14 */
	__vo uint32_t SR1;
	/*!< I2C Status register 2,      Address offset: 0x18 */
	__vo uint32_t SR2;
	/*!< I2C Clock control register, Address offset: 0x1C */
	__vo uint32_t CCR;
	/*!< I2C TRISE register,         Address offset: 0x20 */
	__vo uint32_t TRISE;
	/*!< I2C FLTR register,          Address offset: 0x24 */
	__vo uint32_t FLTR;
} I2C_RegDef_t;

/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */

typedef struct
{
	__vo uint32_t SR;	/*!< USART Status register,                   Address offset: 0x00 */
	__vo uint32_t DR;	/*!< USART Data register,                     Address offset: 0x04 */
	__vo uint32_t BRR;	/*!< USART Baud rate register,                Address offset: 0x08 */
	__vo uint32_t CR1;	/*!< USART Control register 1,                Address offset: 0x0C */
	__vo uint32_t CR2;	/*!< USART Control register 2,                Address offset: 0x10 */
	__vo uint32_t CR3;	/*!< USART Control register 3,                Address offset: 0x14 */
	__vo uint32_t GTPR; /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_RegDef_t;

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

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)

#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)

#define USART1 ((USART_RegDef_t *)USART1_BASEADDR)
#define USART2 ((USART_RegDef_t *)USART2_BASEADDR)
#define USART3 ((USART_RegDef_t *)USART3_BASEADDR)
#define UART4 ((USART_RegDef_t *)UART4_BASEADDR)
#define UART5 ((USART_RegDef_t *)UART5_BASEADDR)
#define USART6 ((USART_RegDef_t *)USART6_BASEADDR)

/**
 * clock enable macros for GPIOx peripherals
 * see RM0390-*.pdf page 144
 */

// GPIOA peripheral clock enable
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
// GPIOB peripheral clock enable
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
// GPIOC peripheral clock enable
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
// GPIOD peripheral clock enable
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
// GPIOE peripheral clock enable
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
// GPIOF peripheral clock enable
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
// GPIOG peripheral clock enable
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
// GPIOH peripheral clock enable
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))

/**
 * clock enable macros for I2Cx peripherals
 * see RM0390-*.pdf page 147
 */

// I2C1 peripheral clock enable
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/**
 * clock enable macros for SPIx peripherals
 * see RM0390-*.pdf page 149 &
 * page 147
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))

/**
 * clock enable macros for USARTx peripherals
 */
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))

/**
 * clock enable macros for SYSCFG peripherals
 * see RM0390-*.pdf page 149
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/**
 * clock disable macros for GPIOx peripherals
 */
// GPIOA peripheral clock disable
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))

/**
 * clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

/**
 * clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))

/**
 * clock disable macros for USARTx peripherals
 */
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))

/**
 * clock disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

/**
 * macros to reset GPIOx peripherals
 * see RM0390-*.pdf page 136
 * RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
 */
#define GPIOA_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 0));  \
		(RCC->AHB1RSTR &= ~(1 << 0)); \
	} while (0)
#define GPIOB_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 1));  \
		(RCC->AHB1RSTR &= ~(1 << 1)); \
	} while (0)
#define GPIOC_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 2));  \
		(RCC->AHB1RSTR &= ~(1 << 2)); \
	} while (0)
#define GPIOD_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 3));  \
		(RCC->AHB1RSTR &= ~(1 << 3)); \
	} while (0)
#define GPIOE_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 4));  \
		(RCC->AHB1RSTR &= ~(1 << 4)); \
	} while (0)
#define GPIOF_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 5));  \
		(RCC->AHB1RSTR &= ~(1 << 5)); \
	} while (0)
#define GPIOG_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 6));  \
		(RCC->AHB1RSTR &= ~(1 << 6)); \
	} while (0)
#define GPIOH_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 7));  \
		(RCC->AHB1RSTR &= ~(1 << 7)); \
	} while (0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()               \
	do                                 \
	{                                  \
		(RCC->APB2RSTR |= (1 << 12));  \
		(RCC->APB2RSTR &= ~(1 << 12)); \
	} while (0)
#define SPI2_REG_RESET()               \
	do                                 \
	{                                  \
		(RCC->APB1RSTR |= (1 << 14));  \
		(RCC->APB1RSTR &= ~(1 << 14)); \
	} while (0)
#define SPI3_REG_RESET()               \
	do                                 \
	{                                  \
		(RCC->APB1RSTR |= (1 << 15));  \
		(RCC->APB1RSTR &= ~(1 << 15)); \
	} while (0)
#define SPI4_REG_RESET()               \
	do                                 \
	{                                  \
		(RCC->APB2RSTR |= (1 << 13));  \
		(RCC->APB2RSTR &= ~(1 << 13)); \
	} while (0)

/**
 * @brief reset I2Cx peripherals
 * see RM0390-*.pdf page 147
 */

#define I2C1_REG_RESET()               \
	do                                 \
	{                                  \
		(RCC->APB1RSTR |= (1 << 21));  \
		(RCC->APB1RSTR &= ~(1 << 21)); \
	} while (0)
#define I2C2_REG_RESET()               \
	do                                 \
	{                                  \
		(RCC->APB1RSTR |= (1 << 22));  \
		(RCC->APB1RSTR &= ~(1 << 22)); \
	} while (0)
#define I2C3_REG_RESET()               \
	do                                 \
	{                                  \
		(RCC->APB1RSTR |= (1 << 23));  \
		(RCC->APB1RSTR &= ~(1 << 23)); \
	} while (0)

/**
 * @brief returns port code for given GPIOx base address
 */
#define GPIOB_BASEADDR_TO_CODE(x) ((x == GPIOA)	  ? 0 \
								   : (x == GPIOB) ? 1 \
								   : (x == GPIOC) ? 2 \
								   : (x == GPIOD) ? 3 \
								   : (x == GPIOE) ? 4 \
								   : (x == GPIOF) ? 5 \
								   : (x == GPIOG) ? 6 \
								   : (x == GPIOH) ? 7 \
												  : 0)

/**
 * @brief reset USARTx peripherals
 *
 */
#define USART1_REG_RESET()            \
	do                                \
	{                                 \
		(RCC->APB2RSTR |= (1 << 4));  \
		(RCC->APB2RSTR &= ~(1 << 4)); \
	} while (0)
#define USART2_REG_RESET()             \
	do                                 \
	{                                  \
		(RCC->APB1RSTR |= (1 << 17));  \
		(RCC->APB1RSTR &= ~(1 << 17)); \
	} while (0)
#define USART3_REG_RESET()             \
	do                                 \
	{                                  \
		(RCC->APB1RSTR |= (1 << 18));  \
		(RCC->APB1RSTR &= ~(1 << 18)); \
	} while (0)
#define UART4_REG_RESET()              \
	do                                 \
	{                                  \
		(RCC->APB1RSTR |= (1 << 19));  \
		(RCC->APB1RSTR &= ~(1 << 19)); \
	} while (0)
#define UART5_REG_RESET()              \
	do                                 \
	{                                  \
		(RCC->APB1RSTR |= (1 << 20));  \
		(RCC->APB1RSTR &= ~(1 << 20)); \
	} while (0)
#define USART6_REG_RESET()            \
	do                                \
	{                                 \
		(RCC->APB2RSTR |= (1 << 5));  \
		(RCC->APB2RSTR &= ~(1 << 5)); \
	} while (0)

/**
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 * see RM0390-*.pdf page 239 vector table
 */
typedef enum
{
	/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
	NonMaskableInt_IRQn = -14,	 /*!< 2 Non Maskable Interrupt                                          */
	MemoryManagement_IRQn = -12, /*!< 4 Cortex-M4 Memory Management Interrupt                           */
	BusFault_IRQn = -11,		 /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
	UsageFault_IRQn = -10,		 /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
	SVCall_IRQn = -5,			 /*!< 11 Cortex-M4 SV Call Interrupt                                    */
	DebugMonitor_IRQn = -4,		 /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
	PendSV_IRQn = -2,			 /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
	SysTick_IRQn = -1,			 /*!< 15 Cortex-M4 System Tick Interrupt                                */
	/******  STM32 specific Interrupt Numbers **********************************************************************/
	WWDG_IRQn = 0,				  /*!< Window WatchDog Interrupt                                         */
	PVD_IRQn = 1,				  /*!< PVD through EXTI Line detection Interrupt                         */
	TAMP_STAMP_IRQn = 2,		  /*!< Tamper and TimeStamp interrupts through the EXTI line             */
	RTC_WKUP_IRQn = 3,			  /*!< RTC Wakeup interrupt through the EXTI line                        */
	FLASH_IRQn = 4,				  /*!< FLASH global Interrupt                                            */
	RCC_IRQn = 5,				  /*!< RCC global Interrupt                                              */
	EXTI0_IRQn = 6,				  /*!< EXTI Line0 Interrupt                                              */
	EXTI1_IRQn = 7,				  /*!< EXTI Line1 Interrupt                                              */
	EXTI2_IRQn = 8,				  /*!< EXTI Line2 Interrupt                                              */
	EXTI3_IRQn = 9,				  /*!< EXTI Line3 Interrupt                                              */
	EXTI4_IRQn = 10,			  /*!< EXTI Line4 Interrupt                                              */
	DMA1_Stream0_IRQn = 11,		  /*!< DMA1 Stream 0 global Interrupt                                    */
	DMA1_Stream1_IRQn = 12,		  /*!< DMA1 Stream 1 global Interrupt                                    */
	DMA1_Stream2_IRQn = 13,		  /*!< DMA1 Stream 2 global Interrupt                                    */
	DMA1_Stream3_IRQn = 14,		  /*!< DMA1 Stream 3 global Interrupt                                    */
	DMA1_Stream4_IRQn = 15,		  /*!< DMA1 Stream 4 global Interrupt                                    */
	DMA1_Stream5_IRQn = 16,		  /*!< DMA1 Stream 5 global Interrupt                                    */
	DMA1_Stream6_IRQn = 17,		  /*!< DMA1 Stream 6 global Interrupt                                    */
	ADC_IRQn = 18,				  /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
	CAN1_TX_IRQn = 19,			  /*!< CAN1 TX Interrupt                                                 */
	CAN1_RX0_IRQn = 20,			  /*!< CAN1 RX0 Interrupt                                                */
	CAN1_RX1_IRQn = 21,			  /*!< CAN1 RX1 Interrupt                                                */
	CAN1_SCE_IRQn = 22,			  /*!< CAN1 SCE Interrupt                                                */
	EXTI9_5_IRQn = 23,			  /*!< External Line[9:5] Interrupts                                     */
	TIM1_BRK_TIM9_IRQn = 24,	  /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
	TIM1_UP_TIM10_IRQn = 25,	  /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
	TIM1_TRG_COM_TIM11_IRQn = 26, /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
	TIM1_CC_IRQn = 27,			  /*!< TIM1 Capture Compare Interrupt                                    */
	TIM2_IRQn = 28,				  /*!< TIM2 global Interrupt                                             */
	TIM3_IRQn = 29,				  /*!< TIM3 global Interrupt                                             */
	TIM4_IRQn = 30,				  /*!< TIM4 global Interrupt                                             */
	I2C1_EV_IRQn = 31,			  /*!< I2C1 Event Interrupt                                              */
	I2C1_ER_IRQn = 32,			  /*!< I2C1 Error Interrupt                                              */
	I2C2_EV_IRQn = 33,			  /*!< I2C2 Event Interrupt                                              */
	I2C2_ER_IRQn = 34,			  /*!< I2C2 Error Interrupt                                              */
	SPI1_IRQn = 35,				  /*!< SPI1 global Interrupt                                             */
	SPI2_IRQn = 36,				  /*!< SPI2 global Interrupt                                             */
	USART1_IRQn = 37,			  /*!< USART1 global Interrupt                                           */
	USART2_IRQn = 38,			  /*!< USART2 global Interrupt                                           */
	USART3_IRQn = 39,			  /*!< USART3 global Interrupt                                           */
	EXTI15_10_IRQn = 40,		  /*!< External Line[15:10] Interrupts                                   */
	RTC_Alarm_IRQn = 41,		  /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
	OTG_FS_WKUP_IRQn = 42,		  /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
	TIM8_BRK_TIM12_IRQn = 43,	  /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
	TIM8_UP_TIM13_IRQn = 44,	  /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
	TIM8_TRG_COM_TIM14_IRQn = 45, /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
	TIM8_CC_IRQn = 46,			  /*!< TIM8 Capture Compare global interrupt                             */
	DMA1_Stream7_IRQn = 47,		  /*!< DMA1 Stream7 Interrupt                                            */
	FMC_IRQn = 48,				  /*!< FMC global Interrupt                                              */
	SDIO_IRQn = 49,				  /*!< SDIO global Interrupt                                             */
	TIM5_IRQn = 50,				  /*!< TIM5 global Interrupt                                             */
	SPI3_IRQn = 51,				  /*!< SPI3 global Interrupt                                             */
	UART4_IRQn = 52,			  /*!< UART4 global Interrupt                                            */
	UART5_IRQn = 53,			  /*!< UART5 global Interrupt                                            */
	TIM6_DAC_IRQn = 54,			  /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
	TIM7_IRQn = 55,				  /*!< TIM7 global interrupt                                             */
	DMA2_Stream0_IRQn = 56,		  /*!< DMA2 Stream 0 global Interrupt                                    */
	DMA2_Stream1_IRQn = 57,		  /*!< DMA2 Stream 1 global Interrupt                                    */
	DMA2_Stream2_IRQn = 58,		  /*!< DMA2 Stream 2 global Interrupt                                    */
	DMA2_Stream3_IRQn = 59,		  /*!< DMA2 Stream 3 global Interrupt                                    */
	DMA2_Stream4_IRQn = 60,		  /*!< DMA2 Stream 4 global Interrupt                                    */
	CAN2_TX_IRQn = 63,			  /*!< CAN2 TX Interrupt                                                 */
	CAN2_RX0_IRQn = 64,			  /*!< CAN2 RX0 Interrupt                                                */
	CAN2_RX1_IRQn = 65,			  /*!< CAN2 RX1 Interrupt                                                */
	CAN2_SCE_IRQn = 66,			  /*!< CAN2 SCE Interrupt                                                */
	OTG_FS_IRQn = 67,			  /*!< USB OTG FS global Interrupt                                       */
	DMA2_Stream5_IRQn = 68,		  /*!< DMA2 Stream 5 global interrupt                                    */
	DMA2_Stream6_IRQn = 69,		  /*!< DMA2 Stream 6 global interrupt                                    */
	DMA2_Stream7_IRQn = 70,		  /*!< DMA2 Stream 7 global interrupt                                    */
	USART6_IRQn = 71,			  /*!< USART6 global interrupt                                           */
	I2C3_EV_IRQn = 72,			  /*!< I2C3 event interrupt                                              */
	I2C3_ER_IRQn = 73,			  /*!< I2C3 error interrupt                                              */
	OTG_HS_EP1_OUT_IRQn = 74,	  /*!< USB OTG HS End Point 1 Out global interrupt                       */
	OTG_HS_EP1_IN_IRQn = 75,	  /*!< USB OTG HS End Point 1 In global interrupt                        */
	OTG_HS_WKUP_IRQn = 76,		  /*!< USB OTG HS Wakeup through EXTI interrupt                          */
	OTG_HS_IRQn = 77,			  /*!< USB OTG HS global interrupt                                       */
	DCMI_IRQn = 78,				  /*!< DCMI global interrupt                                             */
	FPU_IRQn = 81,				  /*!< FPU global interrupt                                              */
	SPI4_IRQn = 84,				  /*!< SPI4 global Interrupt                                             */
	SAI1_IRQn = 87,				  /*!< SAI1 global Interrupt                                             */
	SAI2_IRQn = 91,				  /*!< SAI2 global Interrupt                                             */
	QUADSPI_IRQn = 92,			  /*!< QuadSPI global Interrupt                                          */
	CEC_IRQn = 93,				  /*!< CEC global Interrupt                                              */
	SPDIF_RX_IRQn = 94,			  /*!< SPDIF-RX global Interrupt                                          */
	FMPI2C1_EV_IRQn = 95,		  /*!< FMPI2C1 Event Interrupt                                           */
	FMPI2C1_ER_IRQn = 96		  /*!< FMPI2C1 Error Interrupt                                           */
} IRQn_Type;

/**
 * some generic macros
 */
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET RESET
#define FLAG_SET SET

/**********************************************
 * Bit position definitions of SPI peripheral *
 * see rm0390-*.pdf page 886                  *
 **********************************************/

// SPI_CR1 BIDIMODE bit position
#define SPI_CR1_BIDIMODE 15
// SPI_CR1 BIDIOE bit position
#define SPI_CR1_BIDIOE 14
// SPI_CR1 CRCEN bit position
#define SPI_CR1_CRCEN 13
// SPI_CR1 CRCNEXT bit position
#define SPI_CR1_CRCNEXT 12
// SPI_CR1 DFF bit position
#define SPI_CR1_DFF 11
// SPI_CR1 RXONLY bit position
#define SPI_CR1_RXONLY 10
// SPI_CR1 SSM bit position
#define SPI_CR1_SSM 9
// SPI_CR1 SSI bit position
#define SPI_CR1_SSI 8
// SPI_CR1 LSBFIRST bit position
#define SPI_CR1_LSBFIRST 7
// SPI_CR1 SPE bit position
#define SPI_CR1_SPE 6
// SPI_CR1 BR2 bit position
#define SPI_CR1_BR2 5
// SPI_CR1 BR1 bit position
#define SPI_CR1_BR1 4
// SPI_CR1 BR0 bit position
#define SPI_CR1_BR0 3
// SPI_CR1 MSTR bit position
#define SPI_CR1_MSTR 2
// SPI_CR1 CPOL bit position
#define SPI_CR1_CPOL 1
// SPI_CR1 CPHA bit position
#define SPI_CR1_CPHA 0

// SPI_CR2 TXEIE bit position
#define SPI_CR2_TXEIE 7
// SPI_CR2 RXNEIE bit position
#define SPI_CR2_RXNEIE 6
// SPI_CR2 ERRIE bit position
#define SPI_CR2_ERRIE 5
// SPI_CR2 FRF bit position
#define SPI_CR2_FRF 4
// SPI_CR2 SSOE bit position
#define SPI_CR2_SSOE 2
// SPI_CR2 TXDMAEN bit position
#define SPI_CR2_TXDMAEN 1
// SPI_CR2 RXDMAEN bit position
#define SPI_CR2_RXDMAEN 0

// SPI_SR FRE bit position
#define SPI_SR_FRE 8
// SPI_SR BSY bit position
#define SPI_SR_BSY 7
// SPI_SR OVR bit position
#define SPI_SR_OVR 6
// SPI_SR MODF bit position
#define SPI_SR_MODF 5
// SPI_SR CRCERR bit position
#define SPI_SR_CRCERR 4
// SPI_SR UDR bit position
#define SPI_SR_UDR 3
// SPI_SR CHSIDE bit position
#define SPI_SR_CHSIDE 2
// SPI_SR TXE bit position
#define SPI_SR_TXE 1
// SPI_SR RXNE bit position
#define SPI_SR_RXNE 0

/**********************************************
 * Bit position definitions of I2C peripheral *
 * see rm0390-*.pdf page 780                  *
 **********************************************/

/*******************  Bit definition for I2C_CR1 register  ********************/
#define I2C_CR1_PE 0
#define I2C_CR1_SMBUS 1
#define I2C_CR1_SMBTYPE 3
#define I2C_CR1_ENARP 4
#define I2C_CR1_ENPEC 5
#define I2C_CR1_ENGC 6
#define I2C_CR1_NOSTRECH 7
#define I2C_CR1_START 8
#define I2C_CR1_STOP 9
#define I2C_CR1_ACK 10
#define I2C_CR1_POS 11
#define I2C_CR1_PEC 12
#define I2C_CR1_ALERT 13
#define I2C_CR1_RESET 15

/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_FREQ 0
#define I2C_CR2_ITERREN 8
#define I2C_CR2_ITEVTEN 9
#define I2C_CR2_ITBUFEN 10
#define I2C_CR2_DMAEN 11
#define I2C_CR2_LAST 12

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define I2C_OAR1_ADD0 0
#define I2C_OAR1_ADD7_1 1
#define I2C_OAR1_ADD9_8 8
#define I2C_OAR1_ADDMODE 15

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define I2C_OAR2_ENDUAL 0
#define I2C_OAR2_ADD2 1

/********************  Bit definition for I2C_DR register  ********************/
#define I2C_DR_DR0 0
#define I2C_DR_DR1 1
#define I2C_DR_DR2 2
#define I2C_DR_DR3 3
#define I2C_DR_DR4 4
#define I2C_DR_DR5 5
#define I2C_DR_DR6 6
#define I2C_DR_DR7 7

/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_SR1_SB 0
#define I2C_SR1_ADDR 1
#define I2C_SR1_BTF 2
#define I2C_SR1_ADD10 3
#define I2C_SR1_STOPF 4
#define I2C_SR1_RXNE 6
#define I2C_SR1_TXE 7
#define I2C_SR1_BERR 8
#define I2C_SR1_ARLO 9
#define I2C_SR1_AF 10
#define I2C_SR1_OVR 11
#define I2C_SR1_PECERR 12
#define I2C_SR1_TIMEOUT 14
#define I2C_SR1_SMBALERT 15

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_SR2_MSL 0
#define I2C_SR2_BUSY 1
#define I2C_SR2_TRA 2
#define I2C_SR2_GENCALL 4
#define I2C_SR2_SMBDEFAULT 5
#define I2C_SR2_SMBHOST 6
#define I2C_SR2_DUALF 7
#define I2C_SR2_PEC 8

/*******************  Bit definition for I2C_CCR register  ********************/
#define I2C_CCR_CCR0 (0U)
#define I2C_CCR_CCR1 (1U)
#define I2C_CCR_CCR2 (2U)
#define I2C_CCR_CCR3 (3U)
#define I2C_CCR_CCR4 (4U)
#define I2C_CCR_CCR5 (5U)
#define I2C_CCR_CCR6 (6U)
#define I2C_CCR_CCR7 (7U)
#define I2C_CCR_CCR8 (8U)
#define I2C_CCR_CCR9 (9U)
#define I2C_CCR_CCR10 (10U)
#define I2C_CCR_CCR11 (11U)
#define I2C_CCR_DUTY (14U)
#define I2C_CCR_FS (15U)

/******************  Bit definition for I2C_TRISE register  *******************/
#define I2C_TRISE_TRISE0 (0U)
#define I2C_TRISE_TRISE1 (1U)
#define I2C_TRISE_TRISE2 (2U)
#define I2C_TRISE_TRISE3 (3U)
#define I2C_TRISE_TRISE4 (4U)
#define I2C_TRISE_TRISE5 (5U)

/******************  Bit definition for I2C_FLTR register  *******************/
#define I2C_FLTR_DNF0 (0U)
#define I2C_FLTR_DNF1 (1U)
#define I2C_FLTR_DNF2 (2U)
#define I2C_FLTR_DNF3 (3U)
#define I2C_FLTR_ANOFF (4U)

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION_Pos (0U)
#define RCC_CR_HSION_Msk (0x1UL << RCC_CR_HSION_Pos) /*!< 0x00000001 */
#define RCC_CR_HSION RCC_CR_HSION_Msk
#define RCC_CR_HSIRDY_Pos (1U)
#define RCC_CR_HSIRDY_Msk (0x1UL << RCC_CR_HSIRDY_Pos) /*!< 0x00000002 */
#define RCC_CR_HSIRDY RCC_CR_HSIRDY_Msk

#define RCC_CR_HSITRIM_Pos (3U)
#define RCC_CR_HSITRIM_Msk (0x1FUL << RCC_CR_HSITRIM_Pos) /*!< 0x000000F8 */
#define RCC_CR_HSITRIM RCC_CR_HSITRIM_Msk
#define RCC_CR_HSITRIM_0 (0x01UL << RCC_CR_HSITRIM_Pos) /*!< 0x00000008 */
#define RCC_CR_HSITRIM_1 (0x02UL << RCC_CR_HSITRIM_Pos) /*!< 0x00000010 */
#define RCC_CR_HSITRIM_2 (0x04UL << RCC_CR_HSITRIM_Pos) /*!< 0x00000020 */
#define RCC_CR_HSITRIM_3 (0x08UL << RCC_CR_HSITRIM_Pos) /*!< 0x00000040 */
#define RCC_CR_HSITRIM_4 (0x10UL << RCC_CR_HSITRIM_Pos) /*!< 0x00000080 */

#define RCC_CR_HSICAL_Pos (8U)
#define RCC_CR_HSICAL_Msk (0xFFUL << RCC_CR_HSICAL_Pos) /*!< 0x0000FF00 */
#define RCC_CR_HSICAL RCC_CR_HSICAL_Msk
#define RCC_CR_HSICAL_0 (0x01UL << RCC_CR_HSICAL_Pos) /*!< 0x00000100 */
#define RCC_CR_HSICAL_1 (0x02UL << RCC_CR_HSICAL_Pos) /*!< 0x00000200 */
#define RCC_CR_HSICAL_2 (0x04UL << RCC_CR_HSICAL_Pos) /*!< 0x00000400 */
#define RCC_CR_HSICAL_3 (0x08UL << RCC_CR_HSICAL_Pos) /*!< 0x00000800 */
#define RCC_CR_HSICAL_4 (0x10UL << RCC_CR_HSICAL_Pos) /*!< 0x00001000 */
#define RCC_CR_HSICAL_5 (0x20UL << RCC_CR_HSICAL_Pos) /*!< 0x00002000 */
#define RCC_CR_HSICAL_6 (0x40UL << RCC_CR_HSICAL_Pos) /*!< 0x00004000 */
#define RCC_CR_HSICAL_7 (0x80UL << RCC_CR_HSICAL_Pos) /*!< 0x00008000 */

#define RCC_CR_HSEON_Pos (16U)
#define RCC_CR_HSEON_Msk (0x1UL << RCC_CR_HSEON_Pos) /*!< 0x00010000 */
#define RCC_CR_HSEON RCC_CR_HSEON_Msk
#define RCC_CR_HSERDY_Pos (17U)
#define RCC_CR_HSERDY_Msk (0x1UL << RCC_CR_HSERDY_Pos) /*!< 0x00020000 */
#define RCC_CR_HSERDY RCC_CR_HSERDY_Msk
#define RCC_CR_HSEBYP_Pos (18U)
#define RCC_CR_HSEBYP_Msk (0x1UL << RCC_CR_HSEBYP_Pos) /*!< 0x00040000 */
#define RCC_CR_HSEBYP RCC_CR_HSEBYP_Msk
#define RCC_CR_CSSON_Pos (19U)
#define RCC_CR_CSSON_Msk (0x1UL << RCC_CR_CSSON_Pos) /*!< 0x00080000 */
#define RCC_CR_CSSON RCC_CR_CSSON_Msk
#define RCC_CR_PLLON_Pos (24U)
#define RCC_CR_PLLON_Msk (0x1UL << RCC_CR_PLLON_Pos) /*!< 0x01000000 */
#define RCC_CR_PLLON RCC_CR_PLLON_Msk
#define RCC_CR_PLLRDY_Pos (25U)
#define RCC_CR_PLLRDY_Msk (0x1UL << RCC_CR_PLLRDY_Pos) /*!< 0x02000000 */
#define RCC_CR_PLLRDY RCC_CR_PLLRDY_Msk
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_PLLI2S_SUPPORT /*!< Support PLLI2S oscillator */

#define RCC_CR_PLLI2SON_Pos (26U)
#define RCC_CR_PLLI2SON_Msk (0x1UL << RCC_CR_PLLI2SON_Pos) /*!< 0x04000000 */
#define RCC_CR_PLLI2SON RCC_CR_PLLI2SON_Msk
#define RCC_CR_PLLI2SRDY_Pos (27U)
#define RCC_CR_PLLI2SRDY_Msk (0x1UL << RCC_CR_PLLI2SRDY_Pos) /*!< 0x08000000 */
#define RCC_CR_PLLI2SRDY RCC_CR_PLLI2SRDY_Msk
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_PLLSAI_SUPPORT /*!< Support PLLSAI oscillator */

#define RCC_CR_PLLSAION_Pos (28U)
#define RCC_CR_PLLSAION_Msk (0x1UL << RCC_CR_PLLSAION_Pos) /*!< 0x10000000 */
#define RCC_CR_PLLSAION RCC_CR_PLLSAION_Msk
#define RCC_CR_PLLSAIRDY_Pos (29U)
#define RCC_CR_PLLSAIRDY_Msk (0x1UL << RCC_CR_PLLSAIRDY_Pos) /*!< 0x20000000 */
#define RCC_CR_PLLSAIRDY RCC_CR_PLLSAIRDY_Msk

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLM_Pos (0U)
#define RCC_PLLCFGR_PLLM_Msk (0x3FUL << RCC_PLLCFGR_PLLM_Pos) /*!< 0x0000003F */
#define RCC_PLLCFGR_PLLM RCC_PLLCFGR_PLLM_Msk
#define RCC_PLLCFGR_PLLM_0 (0x01UL << RCC_PLLCFGR_PLLM_Pos) /*!< 0x00000001 */
#define RCC_PLLCFGR_PLLM_1 (0x02UL << RCC_PLLCFGR_PLLM_Pos) /*!< 0x00000002 */
#define RCC_PLLCFGR_PLLM_2 (0x04UL << RCC_PLLCFGR_PLLM_Pos) /*!< 0x00000004 */
#define RCC_PLLCFGR_PLLM_3 (0x08UL << RCC_PLLCFGR_PLLM_Pos) /*!< 0x00000008 */
#define RCC_PLLCFGR_PLLM_4 (0x10UL << RCC_PLLCFGR_PLLM_Pos) /*!< 0x00000010 */
#define RCC_PLLCFGR_PLLM_5 (0x20UL << RCC_PLLCFGR_PLLM_Pos) /*!< 0x00000020 */

#define RCC_PLLCFGR_PLLN_Pos (6U)
#define RCC_PLLCFGR_PLLN_Msk (0x1FFUL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00007FC0 */
#define RCC_PLLCFGR_PLLN RCC_PLLCFGR_PLLN_Msk
#define RCC_PLLCFGR_PLLN_0 (0x001UL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00000040 */
#define RCC_PLLCFGR_PLLN_1 (0x002UL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00000080 */
#define RCC_PLLCFGR_PLLN_2 (0x004UL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00000100 */
#define RCC_PLLCFGR_PLLN_3 (0x008UL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00000200 */
#define RCC_PLLCFGR_PLLN_4 (0x010UL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00000400 */
#define RCC_PLLCFGR_PLLN_5 (0x020UL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00000800 */
#define RCC_PLLCFGR_PLLN_6 (0x040UL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00001000 */
#define RCC_PLLCFGR_PLLN_7 (0x080UL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00002000 */
#define RCC_PLLCFGR_PLLN_8 (0x100UL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00004000 */

#define RCC_PLLCFGR_PLLP_Pos (16U)
#define RCC_PLLCFGR_PLLP_Msk (0x3UL << RCC_PLLCFGR_PLLP_Pos) /*!< 0x00030000 */
#define RCC_PLLCFGR_PLLP RCC_PLLCFGR_PLLP_Msk
#define RCC_PLLCFGR_PLLP_0 (0x1UL << RCC_PLLCFGR_PLLP_Pos) /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLP_1 (0x2UL << RCC_PLLCFGR_PLLP_Pos) /*!< 0x00020000 */

#define RCC_PLLCFGR_PLLSRC_Pos (22U)
#define RCC_PLLCFGR_PLLSRC_Msk (0x1UL << RCC_PLLCFGR_PLLSRC_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC RCC_PLLCFGR_PLLSRC_Msk
#define RCC_PLLCFGR_PLLSRC_HSE_Pos (22U)
#define RCC_PLLCFGR_PLLSRC_HSE_Msk (0x1UL << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE RCC_PLLCFGR_PLLSRC_HSE_Msk
#define RCC_PLLCFGR_PLLSRC_HSI 0x00000000U

#define RCC_PLLCFGR_PLLQ_Pos (24U)
#define RCC_PLLCFGR_PLLQ_Msk (0xFUL << RCC_PLLCFGR_PLLQ_Pos) /*!< 0x0F000000 */
#define RCC_PLLCFGR_PLLQ RCC_PLLCFGR_PLLQ_Msk
#define RCC_PLLCFGR_PLLQ_0 (0x1UL << RCC_PLLCFGR_PLLQ_Pos) /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLQ_1 (0x2UL << RCC_PLLCFGR_PLLQ_Pos) /*!< 0x02000000 */
#define RCC_PLLCFGR_PLLQ_2 (0x4UL << RCC_PLLCFGR_PLLQ_Pos) /*!< 0x04000000 */
#define RCC_PLLCFGR_PLLQ_3 (0x8UL << RCC_PLLCFGR_PLLQ_Pos) /*!< 0x08000000 */
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_PLLR_SYSCLK_SUPPORT		   /*!< Support PLLR as system clock */
#define RCC_PLLR_I2S_CLKSOURCE_SUPPORT /*!< Support PLLR clock as I2S clock source */

#define RCC_PLLCFGR_PLLR_Pos (28U)
#define RCC_PLLCFGR_PLLR_Msk (0x7UL << RCC_PLLCFGR_PLLR_Pos) /*!< 0x70000000 */
#define RCC_PLLCFGR_PLLR RCC_PLLCFGR_PLLR_Msk
#define RCC_PLLCFGR_PLLR_0 (0x1UL << RCC_PLLCFGR_PLLR_Pos) /*!< 0x10000000 */
#define RCC_PLLCFGR_PLLR_1 (0x2UL << RCC_PLLCFGR_PLLR_Pos) /*!< 0x20000000 */
#define RCC_PLLCFGR_PLLR_2 (0x4UL << RCC_PLLCFGR_PLLR_Pos) /*!< 0x40000000 */

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos (0U)
#define RCC_CFGR_SW_Msk (0x3UL << RCC_CFGR_SW_Pos) /*!< 0x00000003 */
#define RCC_CFGR_SW RCC_CFGR_SW_Msk				   /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0 (0x1UL << RCC_CFGR_SW_Pos)   /*!< 0x00000001 */
#define RCC_CFGR_SW_1 (0x2UL << RCC_CFGR_SW_Pos)   /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI 0x00000000U	 /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE 0x00000001U	 /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL 0x00000002U	 /*!< PLL selected as system clock */
#define RCC_CFGR_SW_PLLR 0x00000003U /*!< PLL/PLLR selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos (2U)
#define RCC_CFGR_SWS_Msk (0x3UL << RCC_CFGR_SWS_Pos) /*!< 0x0000000C */
#define RCC_CFGR_SWS RCC_CFGR_SWS_Msk				 /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0 (0x1UL << RCC_CFGR_SWS_Pos)	 /*!< 0x00000004 */
#define RCC_CFGR_SWS_1 (0x2UL << RCC_CFGR_SWS_Pos)	 /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI 0x00000000U  /*!< HSI oscillator used as system clock        */
#define RCC_CFGR_SWS_HSE 0x00000004U  /*!< HSE oscillator used as system clock        */
#define RCC_CFGR_SWS_PLL 0x00000008U  /*!< PLL used as system clock                   */
#define RCC_CFGR_SWS_PLLR 0x0000000CU /*!< PLL/PLLR used as system clock       */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos (4U)
#define RCC_CFGR_HPRE_Msk (0xFUL << RCC_CFGR_HPRE_Pos) /*!< 0x000000F0 */
#define RCC_CFGR_HPRE RCC_CFGR_HPRE_Msk				   /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0 (0x1UL << RCC_CFGR_HPRE_Pos)   /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1 (0x2UL << RCC_CFGR_HPRE_Pos)   /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2 (0x4UL << RCC_CFGR_HPRE_Pos)   /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3 (0x8UL << RCC_CFGR_HPRE_Pos)   /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1 0x00000000U	 /*!< SYSCLK not divided    */
#define RCC_CFGR_HPRE_DIV2 0x00000080U	 /*!< SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_DIV4 0x00000090U	 /*!< SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_DIV8 0x000000A0U	 /*!< SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_DIV16 0x000000B0U	 /*!< SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_DIV64 0x000000C0U	 /*!< SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_DIV128 0x000000D0U /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256 0x000000E0U /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512 0x000000F0U /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos (10U)
#define RCC_CFGR_PPRE1_Msk (0x7UL << RCC_CFGR_PPRE1_Pos) /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1 RCC_CFGR_PPRE1_Msk				 /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0 (0x1UL << RCC_CFGR_PPRE1_Pos)	 /*!< 0x00000400 */
#define RCC_CFGR_PPRE1_1 (0x2UL << RCC_CFGR_PPRE1_Pos)	 /*!< 0x00000800 */
#define RCC_CFGR_PPRE1_2 (0x4UL << RCC_CFGR_PPRE1_Pos)	 /*!< 0x00001000 */

#define RCC_CFGR_PPRE1_DIV1 0x00000000U	 /*!< HCLK not divided   */
#define RCC_CFGR_PPRE1_DIV2 0x00001000U	 /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE1_DIV4 0x00001400U	 /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE1_DIV8 0x00001800U	 /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE1_DIV16 0x00001C00U /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos (13U)
#define RCC_CFGR_PPRE2_Msk (0x7UL << RCC_CFGR_PPRE2_Pos) /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2 RCC_CFGR_PPRE2_Msk				 /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0 (0x1UL << RCC_CFGR_PPRE2_Pos)	 /*!< 0x00002000 */
#define RCC_CFGR_PPRE2_1 (0x2UL << RCC_CFGR_PPRE2_Pos)	 /*!< 0x00004000 */
#define RCC_CFGR_PPRE2_2 (0x4UL << RCC_CFGR_PPRE2_Pos)	 /*!< 0x00008000 */

#define RCC_CFGR_PPRE2_DIV1 0x00000000U	 /*!< HCLK not divided   */
#define RCC_CFGR_PPRE2_DIV2 0x00008000U	 /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE2_DIV4 0x0000A000U	 /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE2_DIV8 0x0000C000U	 /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE2_DIV16 0x0000E000U /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define RCC_CFGR_RTCPRE_Pos (16U)
#define RCC_CFGR_RTCPRE_Msk (0x1FUL << RCC_CFGR_RTCPRE_Pos) /*!< 0x001F0000 */
#define RCC_CFGR_RTCPRE RCC_CFGR_RTCPRE_Msk
#define RCC_CFGR_RTCPRE_0 (0x01UL << RCC_CFGR_RTCPRE_Pos) /*!< 0x00010000 */
#define RCC_CFGR_RTCPRE_1 (0x02UL << RCC_CFGR_RTCPRE_Pos) /*!< 0x00020000 */
#define RCC_CFGR_RTCPRE_2 (0x04UL << RCC_CFGR_RTCPRE_Pos) /*!< 0x00040000 */
#define RCC_CFGR_RTCPRE_3 (0x08UL << RCC_CFGR_RTCPRE_Pos) /*!< 0x00080000 */
#define RCC_CFGR_RTCPRE_4 (0x10UL << RCC_CFGR_RTCPRE_Pos) /*!< 0x00100000 */

/*!< MCO1 configuration */
#define RCC_CFGR_MCO1_Pos (21U)
#define RCC_CFGR_MCO1_Msk (0x3UL << RCC_CFGR_MCO1_Pos) /*!< 0x00600000 */
#define RCC_CFGR_MCO1 RCC_CFGR_MCO1_Msk
#define RCC_CFGR_MCO1_0 (0x1UL << RCC_CFGR_MCO1_Pos) /*!< 0x00200000 */
#define RCC_CFGR_MCO1_1 (0x2UL << RCC_CFGR_MCO1_Pos) /*!< 0x00400000 */

#define RCC_CFGR_MCO1PRE_Pos (24U)
#define RCC_CFGR_MCO1PRE_Msk (0x7UL << RCC_CFGR_MCO1PRE_Pos) /*!< 0x07000000 */
#define RCC_CFGR_MCO1PRE RCC_CFGR_MCO1PRE_Msk
#define RCC_CFGR_MCO1PRE_0 (0x1UL << RCC_CFGR_MCO1PRE_Pos) /*!< 0x01000000 */
#define RCC_CFGR_MCO1PRE_1 (0x2UL << RCC_CFGR_MCO1PRE_Pos) /*!< 0x02000000 */
#define RCC_CFGR_MCO1PRE_2 (0x4UL << RCC_CFGR_MCO1PRE_Pos) /*!< 0x04000000 */

#define RCC_CFGR_MCO2PRE_Pos (27U)
#define RCC_CFGR_MCO2PRE_Msk (0x7UL << RCC_CFGR_MCO2PRE_Pos) /*!< 0x38000000 */
#define RCC_CFGR_MCO2PRE RCC_CFGR_MCO2PRE_Msk
#define RCC_CFGR_MCO2PRE_0 (0x1UL << RCC_CFGR_MCO2PRE_Pos) /*!< 0x08000000 */
#define RCC_CFGR_MCO2PRE_1 (0x2UL << RCC_CFGR_MCO2PRE_Pos) /*!< 0x10000000 */
#define RCC_CFGR_MCO2PRE_2 (0x4UL << RCC_CFGR_MCO2PRE_Pos) /*!< 0x20000000 */

#define RCC_CFGR_MCO2_Pos (30U)
#define RCC_CFGR_MCO2_Msk (0x3UL << RCC_CFGR_MCO2_Pos) /*!< 0xC0000000 */
#define RCC_CFGR_MCO2 RCC_CFGR_MCO2_Msk
#define RCC_CFGR_MCO2_0 (0x1UL << RCC_CFGR_MCO2_Pos) /*!< 0x40000000 */
#define RCC_CFGR_MCO2_1 (0x2UL << RCC_CFGR_MCO2_Pos) /*!< 0x80000000 */

/********************  Bit definition for RCC_CIR register  *******************/
#define RCC_CIR_LSIRDYF_Pos (0U)
#define RCC_CIR_LSIRDYF_Msk (0x1UL << RCC_CIR_LSIRDYF_Pos) /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF RCC_CIR_LSIRDYF_Msk
#define RCC_CIR_LSERDYF_Pos (1U)
#define RCC_CIR_LSERDYF_Msk (0x1UL << RCC_CIR_LSERDYF_Pos) /*!< 0x00000002 */
#define RCC_CIR_LSERDYF RCC_CIR_LSERDYF_Msk
#define RCC_CIR_HSIRDYF_Pos (2U)
#define RCC_CIR_HSIRDYF_Msk (0x1UL << RCC_CIR_HSIRDYF_Pos) /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF RCC_CIR_HSIRDYF_Msk
#define RCC_CIR_HSERDYF_Pos (3U)
#define RCC_CIR_HSERDYF_Msk (0x1UL << RCC_CIR_HSERDYF_Pos) /*!< 0x00000008 */
#define RCC_CIR_HSERDYF RCC_CIR_HSERDYF_Msk
#define RCC_CIR_PLLRDYF_Pos (4U)
#define RCC_CIR_PLLRDYF_Msk (0x1UL << RCC_CIR_PLLRDYF_Pos) /*!< 0x00000010 */
#define RCC_CIR_PLLRDYF RCC_CIR_PLLRDYF_Msk
#define RCC_CIR_PLLI2SRDYF_Pos (5U)
#define RCC_CIR_PLLI2SRDYF_Msk (0x1UL << RCC_CIR_PLLI2SRDYF_Pos) /*!< 0x00000020 */
#define RCC_CIR_PLLI2SRDYF RCC_CIR_PLLI2SRDYF_Msk

#define RCC_CIR_PLLSAIRDYF_Pos (6U)
#define RCC_CIR_PLLSAIRDYF_Msk (0x1UL << RCC_CIR_PLLSAIRDYF_Pos) /*!< 0x00000040 */
#define RCC_CIR_PLLSAIRDYF RCC_CIR_PLLSAIRDYF_Msk
#define RCC_CIR_CSSF_Pos (7U)
#define RCC_CIR_CSSF_Msk (0x1UL << RCC_CIR_CSSF_Pos) /*!< 0x00000080 */
#define RCC_CIR_CSSF RCC_CIR_CSSF_Msk
#define RCC_CIR_LSIRDYIE_Pos (8U)
#define RCC_CIR_LSIRDYIE_Msk (0x1UL << RCC_CIR_LSIRDYIE_Pos) /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE RCC_CIR_LSIRDYIE_Msk
#define RCC_CIR_LSERDYIE_Pos (9U)
#define RCC_CIR_LSERDYIE_Msk (0x1UL << RCC_CIR_LSERDYIE_Pos) /*!< 0x00000200 */
#define RCC_CIR_LSERDYIE RCC_CIR_LSERDYIE_Msk
#define RCC_CIR_HSIRDYIE_Pos (10U)
#define RCC_CIR_HSIRDYIE_Msk (0x1UL << RCC_CIR_HSIRDYIE_Pos) /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE RCC_CIR_HSIRDYIE_Msk
#define RCC_CIR_HSERDYIE_Pos (11U)
#define RCC_CIR_HSERDYIE_Msk (0x1UL << RCC_CIR_HSERDYIE_Pos) /*!< 0x00000800 */
#define RCC_CIR_HSERDYIE RCC_CIR_HSERDYIE_Msk
#define RCC_CIR_PLLRDYIE_Pos (12U)
#define RCC_CIR_PLLRDYIE_Msk (0x1UL << RCC_CIR_PLLRDYIE_Pos) /*!< 0x00001000 */
#define RCC_CIR_PLLRDYIE RCC_CIR_PLLRDYIE_Msk
#define RCC_CIR_PLLI2SRDYIE_Pos (13U)
#define RCC_CIR_PLLI2SRDYIE_Msk (0x1UL << RCC_CIR_PLLI2SRDYIE_Pos) /*!< 0x00002000 */
#define RCC_CIR_PLLI2SRDYIE RCC_CIR_PLLI2SRDYIE_Msk

#define RCC_CIR_PLLSAIRDYIE_Pos (14U)
#define RCC_CIR_PLLSAIRDYIE_Msk (0x1UL << RCC_CIR_PLLSAIRDYIE_Pos) /*!< 0x00004000 */
#define RCC_CIR_PLLSAIRDYIE RCC_CIR_PLLSAIRDYIE_Msk
#define RCC_CIR_LSIRDYC_Pos (16U)
#define RCC_CIR_LSIRDYC_Msk (0x1UL << RCC_CIR_LSIRDYC_Pos) /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC RCC_CIR_LSIRDYC_Msk
#define RCC_CIR_LSERDYC_Pos (17U)
#define RCC_CIR_LSERDYC_Msk (0x1UL << RCC_CIR_LSERDYC_Pos) /*!< 0x00020000 */
#define RCC_CIR_LSERDYC RCC_CIR_LSERDYC_Msk
#define RCC_CIR_HSIRDYC_Pos (18U)
#define RCC_CIR_HSIRDYC_Msk (0x1UL << RCC_CIR_HSIRDYC_Pos) /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC RCC_CIR_HSIRDYC_Msk
#define RCC_CIR_HSERDYC_Pos (19U)
#define RCC_CIR_HSERDYC_Msk (0x1UL << RCC_CIR_HSERDYC_Pos) /*!< 0x00080000 */
#define RCC_CIR_HSERDYC RCC_CIR_HSERDYC_Msk
#define RCC_CIR_PLLRDYC_Pos (20U)
#define RCC_CIR_PLLRDYC_Msk (0x1UL << RCC_CIR_PLLRDYC_Pos) /*!< 0x00100000 */
#define RCC_CIR_PLLRDYC RCC_CIR_PLLRDYC_Msk
#define RCC_CIR_PLLI2SRDYC_Pos (21U)
#define RCC_CIR_PLLI2SRDYC_Msk (0x1UL << RCC_CIR_PLLI2SRDYC_Pos) /*!< 0x00200000 */
#define RCC_CIR_PLLI2SRDYC RCC_CIR_PLLI2SRDYC_Msk
#define RCC_CIR_PLLSAIRDYC_Pos (22U)
#define RCC_CIR_PLLSAIRDYC_Msk (0x1UL << RCC_CIR_PLLSAIRDYC_Pos) /*!< 0x00400000 */
#define RCC_CIR_PLLSAIRDYC RCC_CIR_PLLSAIRDYC_Msk

#define RCC_CIR_CSSC_Pos (23U)
#define RCC_CIR_CSSC_Msk (0x1UL << RCC_CIR_CSSC_Pos) /*!< 0x00800000 */
#define RCC_CIR_CSSC RCC_CIR_CSSC_Msk

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define RCC_AHB1RSTR_GPIOARST_Pos (0U)
#define RCC_AHB1RSTR_GPIOARST_Msk (0x1UL << RCC_AHB1RSTR_GPIOARST_Pos) /*!< 0x00000001 */
#define RCC_AHB1RSTR_GPIOARST RCC_AHB1RSTR_GPIOARST_Msk
#define RCC_AHB1RSTR_GPIOBRST_Pos (1U)
#define RCC_AHB1RSTR_GPIOBRST_Msk (0x1UL << RCC_AHB1RSTR_GPIOBRST_Pos) /*!< 0x00000002 */
#define RCC_AHB1RSTR_GPIOBRST RCC_AHB1RSTR_GPIOBRST_Msk
#define RCC_AHB1RSTR_GPIOCRST_Pos (2U)
#define RCC_AHB1RSTR_GPIOCRST_Msk (0x1UL << RCC_AHB1RSTR_GPIOCRST_Pos) /*!< 0x00000004 */
#define RCC_AHB1RSTR_GPIOCRST RCC_AHB1RSTR_GPIOCRST_Msk
#define RCC_AHB1RSTR_GPIODRST_Pos (3U)
#define RCC_AHB1RSTR_GPIODRST_Msk (0x1UL << RCC_AHB1RSTR_GPIODRST_Pos) /*!< 0x00000008 */
#define RCC_AHB1RSTR_GPIODRST RCC_AHB1RSTR_GPIODRST_Msk
#define RCC_AHB1RSTR_GPIOERST_Pos (4U)
#define RCC_AHB1RSTR_GPIOERST_Msk (0x1UL << RCC_AHB1RSTR_GPIOERST_Pos) /*!< 0x00000010 */
#define RCC_AHB1RSTR_GPIOERST RCC_AHB1RSTR_GPIOERST_Msk
#define RCC_AHB1RSTR_GPIOFRST_Pos (5U)
#define RCC_AHB1RSTR_GPIOFRST_Msk (0x1UL << RCC_AHB1RSTR_GPIOFRST_Pos) /*!< 0x00000020 */
#define RCC_AHB1RSTR_GPIOFRST RCC_AHB1RSTR_GPIOFRST_Msk
#define RCC_AHB1RSTR_GPIOGRST_Pos (6U)
#define RCC_AHB1RSTR_GPIOGRST_Msk (0x1UL << RCC_AHB1RSTR_GPIOGRST_Pos) /*!< 0x00000040 */
#define RCC_AHB1RSTR_GPIOGRST RCC_AHB1RSTR_GPIOGRST_Msk
#define RCC_AHB1RSTR_GPIOHRST_Pos (7U)
#define RCC_AHB1RSTR_GPIOHRST_Msk (0x1UL << RCC_AHB1RSTR_GPIOHRST_Pos) /*!< 0x00000080 */
#define RCC_AHB1RSTR_GPIOHRST RCC_AHB1RSTR_GPIOHRST_Msk
#define RCC_AHB1RSTR_CRCRST_Pos (12U)
#define RCC_AHB1RSTR_CRCRST_Msk (0x1UL << RCC_AHB1RSTR_CRCRST_Pos) /*!< 0x00001000 */
#define RCC_AHB1RSTR_CRCRST RCC_AHB1RSTR_CRCRST_Msk
#define RCC_AHB1RSTR_DMA1RST_Pos (21U)
#define RCC_AHB1RSTR_DMA1RST_Msk (0x1UL << RCC_AHB1RSTR_DMA1RST_Pos) /*!< 0x00200000 */
#define RCC_AHB1RSTR_DMA1RST RCC_AHB1RSTR_DMA1RST_Msk
#define RCC_AHB1RSTR_DMA2RST_Pos (22U)
#define RCC_AHB1RSTR_DMA2RST_Msk (0x1UL << RCC_AHB1RSTR_DMA2RST_Pos) /*!< 0x00400000 */
#define RCC_AHB1RSTR_DMA2RST RCC_AHB1RSTR_DMA2RST_Msk
#define RCC_AHB1RSTR_OTGHRST_Pos (29U)
#define RCC_AHB1RSTR_OTGHRST_Msk (0x1UL << RCC_AHB1RSTR_OTGHRST_Pos) /*!< 0x20000000 */
#define RCC_AHB1RSTR_OTGHRST RCC_AHB1RSTR_OTGHRST_Msk

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define RCC_AHB2RSTR_DCMIRST_Pos (0U)
#define RCC_AHB2RSTR_DCMIRST_Msk (0x1UL << RCC_AHB2RSTR_DCMIRST_Pos) /*!< 0x00000001 */
#define RCC_AHB2RSTR_DCMIRST RCC_AHB2RSTR_DCMIRST_Msk
#define RCC_AHB2RSTR_OTGFSRST_Pos (7U)
#define RCC_AHB2RSTR_OTGFSRST_Msk (0x1UL << RCC_AHB2RSTR_OTGFSRST_Pos) /*!< 0x00000080 */
#define RCC_AHB2RSTR_OTGFSRST RCC_AHB2RSTR_OTGFSRST_Msk
/********************  Bit definition for RCC_AHB3RSTR register  **************/
#define RCC_AHB3RSTR_FMCRST_Pos (0U)
#define RCC_AHB3RSTR_FMCRST_Msk (0x1UL << RCC_AHB3RSTR_FMCRST_Pos) /*!< 0x00000001 */
#define RCC_AHB3RSTR_FMCRST RCC_AHB3RSTR_FMCRST_Msk
#define RCC_AHB3RSTR_QSPIRST_Pos (1U)
#define RCC_AHB3RSTR_QSPIRST_Msk (0x1UL << RCC_AHB3RSTR_QSPIRST_Pos) /*!< 0x00000002 */
#define RCC_AHB3RSTR_QSPIRST RCC_AHB3RSTR_QSPIRST_Msk

/********************  Bit definition for RCC_APB1RSTR register  **************/
#define RCC_APB1RSTR_TIM2RST_Pos (0U)
#define RCC_APB1RSTR_TIM2RST_Msk (0x1UL << RCC_APB1RSTR_TIM2RST_Pos) /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST RCC_APB1RSTR_TIM2RST_Msk
#define RCC_APB1RSTR_TIM3RST_Pos (1U)
#define RCC_APB1RSTR_TIM3RST_Msk (0x1UL << RCC_APB1RSTR_TIM3RST_Pos) /*!< 0x00000002 */
#define RCC_APB1RSTR_TIM3RST RCC_APB1RSTR_TIM3RST_Msk
#define RCC_APB1RSTR_TIM4RST_Pos (2U)
#define RCC_APB1RSTR_TIM4RST_Msk (0x1UL << RCC_APB1RSTR_TIM4RST_Pos) /*!< 0x00000004 */
#define RCC_APB1RSTR_TIM4RST RCC_APB1RSTR_TIM4RST_Msk
#define RCC_APB1RSTR_TIM5RST_Pos (3U)
#define RCC_APB1RSTR_TIM5RST_Msk (0x1UL << RCC_APB1RSTR_TIM5RST_Pos) /*!< 0x00000008 */
#define RCC_APB1RSTR_TIM5RST RCC_APB1RSTR_TIM5RST_Msk
#define RCC_APB1RSTR_TIM6RST_Pos (4U)
#define RCC_APB1RSTR_TIM6RST_Msk (0x1UL << RCC_APB1RSTR_TIM6RST_Pos) /*!< 0x00000010 */
#define RCC_APB1RSTR_TIM6RST RCC_APB1RSTR_TIM6RST_Msk
#define RCC_APB1RSTR_TIM7RST_Pos (5U)
#define RCC_APB1RSTR_TIM7RST_Msk (0x1UL << RCC_APB1RSTR_TIM7RST_Pos) /*!< 0x00000020 */
#define RCC_APB1RSTR_TIM7RST RCC_APB1RSTR_TIM7RST_Msk
#define RCC_APB1RSTR_TIM12RST_Pos (6U)
#define RCC_APB1RSTR_TIM12RST_Msk (0x1UL << RCC_APB1RSTR_TIM12RST_Pos) /*!< 0x00000040 */
#define RCC_APB1RSTR_TIM12RST RCC_APB1RSTR_TIM12RST_Msk
#define RCC_APB1RSTR_TIM13RST_Pos (7U)
#define RCC_APB1RSTR_TIM13RST_Msk (0x1UL << RCC_APB1RSTR_TIM13RST_Pos) /*!< 0x00000080 */
#define RCC_APB1RSTR_TIM13RST RCC_APB1RSTR_TIM13RST_Msk
#define RCC_APB1RSTR_TIM14RST_Pos (8U)
#define RCC_APB1RSTR_TIM14RST_Msk (0x1UL << RCC_APB1RSTR_TIM14RST_Pos) /*!< 0x00000100 */
#define RCC_APB1RSTR_TIM14RST RCC_APB1RSTR_TIM14RST_Msk
#define RCC_APB1RSTR_WWDGRST_Pos (11U)
#define RCC_APB1RSTR_WWDGRST_Msk (0x1UL << RCC_APB1RSTR_WWDGRST_Pos) /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST RCC_APB1RSTR_WWDGRST_Msk
#define RCC_APB1RSTR_SPI2RST_Pos (14U)
#define RCC_APB1RSTR_SPI2RST_Msk (0x1UL << RCC_APB1RSTR_SPI2RST_Pos) /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI2RST RCC_APB1RSTR_SPI2RST_Msk
#define RCC_APB1RSTR_SPI3RST_Pos (15U)
#define RCC_APB1RSTR_SPI3RST_Msk (0x1UL << RCC_APB1RSTR_SPI3RST_Pos) /*!< 0x00008000 */
#define RCC_APB1RSTR_SPI3RST RCC_APB1RSTR_SPI3RST_Msk
#define RCC_APB1RSTR_SPDIFRXRST_Pos (16U)
#define RCC_APB1RSTR_SPDIFRXRST_Msk (0x1UL << RCC_APB1RSTR_SPDIFRXRST_Pos) /*!< 0x00010000 */
#define RCC_APB1RSTR_SPDIFRXRST RCC_APB1RSTR_SPDIFRXRST_Msk
#define RCC_APB1RSTR_USART2RST_Pos (17U)
#define RCC_APB1RSTR_USART2RST_Msk (0x1UL << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_USART2RST RCC_APB1RSTR_USART2RST_Msk
#define RCC_APB1RSTR_USART3RST_Pos (18U)
#define RCC_APB1RSTR_USART3RST_Msk (0x1UL << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */
#define RCC_APB1RSTR_USART3RST RCC_APB1RSTR_USART3RST_Msk
#define RCC_APB1RSTR_UART4RST_Pos (19U)
#define RCC_APB1RSTR_UART4RST_Msk (0x1UL << RCC_APB1RSTR_UART4RST_Pos) /*!< 0x00080000 */
#define RCC_APB1RSTR_UART4RST RCC_APB1RSTR_UART4RST_Msk
#define RCC_APB1RSTR_UART5RST_Pos (20U)
#define RCC_APB1RSTR_UART5RST_Msk (0x1UL << RCC_APB1RSTR_UART5RST_Pos) /*!< 0x00100000 */
#define RCC_APB1RSTR_UART5RST RCC_APB1RSTR_UART5RST_Msk
#define RCC_APB1RSTR_I2C1RST_Pos (21U)
#define RCC_APB1RSTR_I2C1RST_Msk (0x1UL << RCC_APB1RSTR_I2C1RST_Pos) /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST RCC_APB1RSTR_I2C1RST_Msk
#define RCC_APB1RSTR_I2C2RST_Pos (22U)
#define RCC_APB1RSTR_I2C2RST_Msk (0x1UL << RCC_APB1RSTR_I2C2RST_Pos) /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST RCC_APB1RSTR_I2C2RST_Msk
#define RCC_APB1RSTR_I2C3RST_Pos (23U)
#define RCC_APB1RSTR_I2C3RST_Msk (0x1UL << RCC_APB1RSTR_I2C3RST_Pos) /*!< 0x00800000 */
#define RCC_APB1RSTR_I2C3RST RCC_APB1RSTR_I2C3RST_Msk
#define RCC_APB1RSTR_FMPI2C1RST_Pos (24U)
#define RCC_APB1RSTR_FMPI2C1RST_Msk (0x1UL << RCC_APB1RSTR_FMPI2C1RST_Pos) /*!< 0x01000000 */
#define RCC_APB1RSTR_FMPI2C1RST RCC_APB1RSTR_FMPI2C1RST_Msk
#define RCC_APB1RSTR_CAN1RST_Pos (25U)
#define RCC_APB1RSTR_CAN1RST_Msk (0x1UL << RCC_APB1RSTR_CAN1RST_Pos) /*!< 0x02000000 */
#define RCC_APB1RSTR_CAN1RST RCC_APB1RSTR_CAN1RST_Msk
#define RCC_APB1RSTR_CAN2RST_Pos (26U)
#define RCC_APB1RSTR_CAN2RST_Msk (0x1UL << RCC_APB1RSTR_CAN2RST_Pos) /*!< 0x04000000 */
#define RCC_APB1RSTR_CAN2RST RCC_APB1RSTR_CAN2RST_Msk
#define RCC_APB1RSTR_CECRST_Pos (27U)
#define RCC_APB1RSTR_CECRST_Msk (0x1UL << RCC_APB1RSTR_CECRST_Pos) /*!< 0x08000000 */
#define RCC_APB1RSTR_CECRST RCC_APB1RSTR_CECRST_Msk
#define RCC_APB1RSTR_PWRRST_Pos (28U)
#define RCC_APB1RSTR_PWRRST_Msk (0x1UL << RCC_APB1RSTR_PWRRST_Pos) /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST RCC_APB1RSTR_PWRRST_Msk
#define RCC_APB1RSTR_DACRST_Pos (29U)
#define RCC_APB1RSTR_DACRST_Msk (0x1UL << RCC_APB1RSTR_DACRST_Pos) /*!< 0x20000000 */
#define RCC_APB1RSTR_DACRST RCC_APB1RSTR_DACRST_Msk

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define RCC_APB2RSTR_TIM1RST_Pos (0U)
#define RCC_APB2RSTR_TIM1RST_Msk (0x1UL << RCC_APB2RSTR_TIM1RST_Pos) /*!< 0x00000001 */
#define RCC_APB2RSTR_TIM1RST RCC_APB2RSTR_TIM1RST_Msk
#define RCC_APB2RSTR_TIM8RST_Pos (1U)
#define RCC_APB2RSTR_TIM8RST_Msk (0x1UL << RCC_APB2RSTR_TIM8RST_Pos) /*!< 0x00000002 */
#define RCC_APB2RSTR_TIM8RST RCC_APB2RSTR_TIM8RST_Msk
#define RCC_APB2RSTR_USART1RST_Pos (4U)
#define RCC_APB2RSTR_USART1RST_Msk (0x1UL << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00000010 */
#define RCC_APB2RSTR_USART1RST RCC_APB2RSTR_USART1RST_Msk
#define RCC_APB2RSTR_USART6RST_Pos (5U)
#define RCC_APB2RSTR_USART6RST_Msk (0x1UL << RCC_APB2RSTR_USART6RST_Pos) /*!< 0x00000020 */
#define RCC_APB2RSTR_USART6RST RCC_APB2RSTR_USART6RST_Msk
#define RCC_APB2RSTR_ADCRST_Pos (8U)
#define RCC_APB2RSTR_ADCRST_Msk (0x1UL << RCC_APB2RSTR_ADCRST_Pos) /*!< 0x00000100 */
#define RCC_APB2RSTR_ADCRST RCC_APB2RSTR_ADCRST_Msk
#define RCC_APB2RSTR_SDIORST_Pos (11U)
#define RCC_APB2RSTR_SDIORST_Msk (0x1UL << RCC_APB2RSTR_SDIORST_Pos) /*!< 0x00000800 */
#define RCC_APB2RSTR_SDIORST RCC_APB2RSTR_SDIORST_Msk
#define RCC_APB2RSTR_SPI1RST_Pos (12U)
#define RCC_APB2RSTR_SPI1RST_Msk (0x1UL << RCC_APB2RSTR_SPI1RST_Pos) /*!< 0x00001000 */
#define RCC_APB2RSTR_SPI1RST RCC_APB2RSTR_SPI1RST_Msk
#define RCC_APB2RSTR_SPI4RST_Pos (13U)
#define RCC_APB2RSTR_SPI4RST_Msk (0x1UL << RCC_APB2RSTR_SPI4RST_Pos) /*!< 0x00002000 */
#define RCC_APB2RSTR_SPI4RST RCC_APB2RSTR_SPI4RST_Msk
#define RCC_APB2RSTR_SYSCFGRST_Pos (14U)
#define RCC_APB2RSTR_SYSCFGRST_Msk (0x1UL << RCC_APB2RSTR_SYSCFGRST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_SYSCFGRST RCC_APB2RSTR_SYSCFGRST_Msk
#define RCC_APB2RSTR_TIM9RST_Pos (16U)
#define RCC_APB2RSTR_TIM9RST_Msk (0x1UL << RCC_APB2RSTR_TIM9RST_Pos) /*!< 0x00010000 */
#define RCC_APB2RSTR_TIM9RST RCC_APB2RSTR_TIM9RST_Msk
#define RCC_APB2RSTR_TIM10RST_Pos (17U)
#define RCC_APB2RSTR_TIM10RST_Msk (0x1UL << RCC_APB2RSTR_TIM10RST_Pos) /*!< 0x00020000 */
#define RCC_APB2RSTR_TIM10RST RCC_APB2RSTR_TIM10RST_Msk
#define RCC_APB2RSTR_TIM11RST_Pos (18U)
#define RCC_APB2RSTR_TIM11RST_Msk (0x1UL << RCC_APB2RSTR_TIM11RST_Pos) /*!< 0x00040000 */
#define RCC_APB2RSTR_TIM11RST RCC_APB2RSTR_TIM11RST_Msk
#define RCC_APB2RSTR_SAI1RST_Pos (22U)
#define RCC_APB2RSTR_SAI1RST_Msk (0x1UL << RCC_APB2RSTR_SAI1RST_Pos) /*!< 0x00400000 */
#define RCC_APB2RSTR_SAI1RST RCC_APB2RSTR_SAI1RST_Msk
#define RCC_APB2RSTR_SAI2RST_Pos (23U)
#define RCC_APB2RSTR_SAI2RST_Msk (0x1UL << RCC_APB2RSTR_SAI2RST_Pos) /*!< 0x00800000 */
#define RCC_APB2RSTR_SAI2RST RCC_APB2RSTR_SAI2RST_Msk

/* Old SPI1RST bit definition, maintained for legacy purpose */
#define RCC_APB2RSTR_SPI1 RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define RCC_AHB1ENR_GPIOAEN_Pos (0U)
#define RCC_AHB1ENR_GPIOAEN_Msk (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos) /*!< 0x00000001 */
#define RCC_AHB1ENR_GPIOAEN RCC_AHB1ENR_GPIOAEN_Msk
#define RCC_AHB1ENR_GPIOBEN_Pos (1U)
#define RCC_AHB1ENR_GPIOBEN_Msk (0x1UL << RCC_AHB1ENR_GPIOBEN_Pos) /*!< 0x00000002 */
#define RCC_AHB1ENR_GPIOBEN RCC_AHB1ENR_GPIOBEN_Msk
#define RCC_AHB1ENR_GPIOCEN_Pos (2U)
#define RCC_AHB1ENR_GPIOCEN_Msk (0x1UL << RCC_AHB1ENR_GPIOCEN_Pos) /*!< 0x00000004 */
#define RCC_AHB1ENR_GPIOCEN RCC_AHB1ENR_GPIOCEN_Msk
#define RCC_AHB1ENR_GPIODEN_Pos (3U)
#define RCC_AHB1ENR_GPIODEN_Msk (0x1UL << RCC_AHB1ENR_GPIODEN_Pos) /*!< 0x00000008 */
#define RCC_AHB1ENR_GPIODEN RCC_AHB1ENR_GPIODEN_Msk
#define RCC_AHB1ENR_GPIOEEN_Pos (4U)
#define RCC_AHB1ENR_GPIOEEN_Msk (0x1UL << RCC_AHB1ENR_GPIOEEN_Pos) /*!< 0x00000010 */
#define RCC_AHB1ENR_GPIOEEN RCC_AHB1ENR_GPIOEEN_Msk
#define RCC_AHB1ENR_GPIOFEN_Pos (5U)
#define RCC_AHB1ENR_GPIOFEN_Msk (0x1UL << RCC_AHB1ENR_GPIOFEN_Pos) /*!< 0x00000020 */
#define RCC_AHB1ENR_GPIOFEN RCC_AHB1ENR_GPIOFEN_Msk
#define RCC_AHB1ENR_GPIOGEN_Pos (6U)
#define RCC_AHB1ENR_GPIOGEN_Msk (0x1UL << RCC_AHB1ENR_GPIOGEN_Pos) /*!< 0x00000040 */
#define RCC_AHB1ENR_GPIOGEN RCC_AHB1ENR_GPIOGEN_Msk
#define RCC_AHB1ENR_GPIOHEN_Pos (7U)
#define RCC_AHB1ENR_GPIOHEN_Msk (0x1UL << RCC_AHB1ENR_GPIOHEN_Pos) /*!< 0x00000080 */
#define RCC_AHB1ENR_GPIOHEN RCC_AHB1ENR_GPIOHEN_Msk
#define RCC_AHB1ENR_CRCEN_Pos (12U)
#define RCC_AHB1ENR_CRCEN_Msk (0x1UL << RCC_AHB1ENR_CRCEN_Pos) /*!< 0x00001000 */
#define RCC_AHB1ENR_CRCEN RCC_AHB1ENR_CRCEN_Msk
#define RCC_AHB1ENR_BKPSRAMEN_Pos (18U)
#define RCC_AHB1ENR_BKPSRAMEN_Msk (0x1UL << RCC_AHB1ENR_BKPSRAMEN_Pos) /*!< 0x00040000 */
#define RCC_AHB1ENR_BKPSRAMEN RCC_AHB1ENR_BKPSRAMEN_Msk
#define RCC_AHB1ENR_DMA1EN_Pos (21U)
#define RCC_AHB1ENR_DMA1EN_Msk (0x1UL << RCC_AHB1ENR_DMA1EN_Pos) /*!< 0x00200000 */
#define RCC_AHB1ENR_DMA1EN RCC_AHB1ENR_DMA1EN_Msk
#define RCC_AHB1ENR_DMA2EN_Pos (22U)
#define RCC_AHB1ENR_DMA2EN_Msk (0x1UL << RCC_AHB1ENR_DMA2EN_Pos) /*!< 0x00400000 */
#define RCC_AHB1ENR_DMA2EN RCC_AHB1ENR_DMA2EN_Msk
#define RCC_AHB1ENR_OTGHSEN_Pos (29U)
#define RCC_AHB1ENR_OTGHSEN_Msk (0x1UL << RCC_AHB1ENR_OTGHSEN_Pos) /*!< 0x20000000 */
#define RCC_AHB1ENR_OTGHSEN RCC_AHB1ENR_OTGHSEN_Msk
#define RCC_AHB1ENR_OTGHSULPIEN_Pos (30U)
#define RCC_AHB1ENR_OTGHSULPIEN_Msk (0x1UL << RCC_AHB1ENR_OTGHSULPIEN_Pos) /*!< 0x40000000 */
#define RCC_AHB1ENR_OTGHSULPIEN RCC_AHB1ENR_OTGHSULPIEN_Msk
/********************  Bit definition for RCC_AHB2ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_AHB2_SUPPORT /*!< AHB2 Bus is supported */

#define RCC_AHB2ENR_DCMIEN_Pos (0U)
#define RCC_AHB2ENR_DCMIEN_Msk (0x1UL << RCC_AHB2ENR_DCMIEN_Pos) /*!< 0x00000001 */
#define RCC_AHB2ENR_DCMIEN RCC_AHB2ENR_DCMIEN_Msk
#define RCC_AHB2ENR_OTGFSEN_Pos (7U)
#define RCC_AHB2ENR_OTGFSEN_Msk (0x1UL << RCC_AHB2ENR_OTGFSEN_Pos) /*!< 0x00000080 */
#define RCC_AHB2ENR_OTGFSEN RCC_AHB2ENR_OTGFSEN_Msk

/********************  Bit definition for RCC_AHB3ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_AHB3_SUPPORT /*!< AHB3 Bus is supported */

#define RCC_AHB3ENR_FMCEN_Pos (0U)
#define RCC_AHB3ENR_FMCEN_Msk (0x1UL << RCC_AHB3ENR_FMCEN_Pos) /*!< 0x00000001 */
#define RCC_AHB3ENR_FMCEN RCC_AHB3ENR_FMCEN_Msk
#define RCC_AHB3ENR_QSPIEN_Pos (1U)
#define RCC_AHB3ENR_QSPIEN_Msk (0x1UL << RCC_AHB3ENR_QSPIEN_Pos) /*!< 0x00000002 */
#define RCC_AHB3ENR_QSPIEN RCC_AHB3ENR_QSPIEN_Msk

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define RCC_APB1ENR_TIM2EN_Pos (0U)
#define RCC_APB1ENR_TIM2EN_Msk (0x1UL << RCC_APB1ENR_TIM2EN_Pos) /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN RCC_APB1ENR_TIM2EN_Msk
#define RCC_APB1ENR_TIM3EN_Pos (1U)
#define RCC_APB1ENR_TIM3EN_Msk (0x1UL << RCC_APB1ENR_TIM3EN_Pos) /*!< 0x00000002 */
#define RCC_APB1ENR_TIM3EN RCC_APB1ENR_TIM3EN_Msk
#define RCC_APB1ENR_TIM4EN_Pos (2U)
#define RCC_APB1ENR_TIM4EN_Msk (0x1UL << RCC_APB1ENR_TIM4EN_Pos) /*!< 0x00000004 */
#define RCC_APB1ENR_TIM4EN RCC_APB1ENR_TIM4EN_Msk
#define RCC_APB1ENR_TIM5EN_Pos (3U)
#define RCC_APB1ENR_TIM5EN_Msk (0x1UL << RCC_APB1ENR_TIM5EN_Pos) /*!< 0x00000008 */
#define RCC_APB1ENR_TIM5EN RCC_APB1ENR_TIM5EN_Msk
#define RCC_APB1ENR_TIM6EN_Pos (4U)
#define RCC_APB1ENR_TIM6EN_Msk (0x1UL << RCC_APB1ENR_TIM6EN_Pos) /*!< 0x00000010 */
#define RCC_APB1ENR_TIM6EN RCC_APB1ENR_TIM6EN_Msk
#define RCC_APB1ENR_TIM7EN_Pos (5U)
#define RCC_APB1ENR_TIM7EN_Msk (0x1UL << RCC_APB1ENR_TIM7EN_Pos) /*!< 0x00000020 */
#define RCC_APB1ENR_TIM7EN RCC_APB1ENR_TIM7EN_Msk
#define RCC_APB1ENR_TIM12EN_Pos (6U)
#define RCC_APB1ENR_TIM12EN_Msk (0x1UL << RCC_APB1ENR_TIM12EN_Pos) /*!< 0x00000040 */
#define RCC_APB1ENR_TIM12EN RCC_APB1ENR_TIM12EN_Msk
#define RCC_APB1ENR_TIM13EN_Pos (7U)
#define RCC_APB1ENR_TIM13EN_Msk (0x1UL << RCC_APB1ENR_TIM13EN_Pos) /*!< 0x00000080 */
#define RCC_APB1ENR_TIM13EN RCC_APB1ENR_TIM13EN_Msk
#define RCC_APB1ENR_TIM14EN_Pos (8U)
#define RCC_APB1ENR_TIM14EN_Msk (0x1UL << RCC_APB1ENR_TIM14EN_Pos) /*!< 0x00000100 */
#define RCC_APB1ENR_TIM14EN RCC_APB1ENR_TIM14EN_Msk
#define RCC_APB1ENR_WWDGEN_Pos (11U)
#define RCC_APB1ENR_WWDGEN_Msk (0x1UL << RCC_APB1ENR_WWDGEN_Pos) /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN RCC_APB1ENR_WWDGEN_Msk
#define RCC_APB1ENR_SPI2EN_Pos (14U)
#define RCC_APB1ENR_SPI2EN_Msk (0x1UL << RCC_APB1ENR_SPI2EN_Pos) /*!< 0x00004000 */
#define RCC_APB1ENR_SPI2EN RCC_APB1ENR_SPI2EN_Msk
#define RCC_APB1ENR_SPI3EN_Pos (15U)
#define RCC_APB1ENR_SPI3EN_Msk (0x1UL << RCC_APB1ENR_SPI3EN_Pos) /*!< 0x00008000 */
#define RCC_APB1ENR_SPI3EN RCC_APB1ENR_SPI3EN_Msk
#define RCC_APB1ENR_SPDIFRXEN_Pos (16U)
#define RCC_APB1ENR_SPDIFRXEN_Msk (0x1UL << RCC_APB1ENR_SPDIFRXEN_Pos) /*!< 0x00010000 */
#define RCC_APB1ENR_SPDIFRXEN RCC_APB1ENR_SPDIFRXEN_Msk
#define RCC_APB1ENR_USART2EN_Pos (17U)
#define RCC_APB1ENR_USART2EN_Msk (0x1UL << RCC_APB1ENR_USART2EN_Pos) /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN RCC_APB1ENR_USART2EN_Msk
#define RCC_APB1ENR_USART3EN_Pos (18U)
#define RCC_APB1ENR_USART3EN_Msk (0x1UL << RCC_APB1ENR_USART3EN_Pos) /*!< 0x00040000 */
#define RCC_APB1ENR_USART3EN RCC_APB1ENR_USART3EN_Msk
#define RCC_APB1ENR_UART4EN_Pos (19U)
#define RCC_APB1ENR_UART4EN_Msk (0x1UL << RCC_APB1ENR_UART4EN_Pos) /*!< 0x00080000 */
#define RCC_APB1ENR_UART4EN RCC_APB1ENR_UART4EN_Msk
#define RCC_APB1ENR_UART5EN_Pos (20U)
#define RCC_APB1ENR_UART5EN_Msk (0x1UL << RCC_APB1ENR_UART5EN_Pos) /*!< 0x00100000 */
#define RCC_APB1ENR_UART5EN RCC_APB1ENR_UART5EN_Msk
#define RCC_APB1ENR_I2C1EN_Pos (21U)
#define RCC_APB1ENR_I2C1EN_Msk (0x1UL << RCC_APB1ENR_I2C1EN_Pos) /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN RCC_APB1ENR_I2C1EN_Msk
#define RCC_APB1ENR_I2C2EN_Pos (22U)
#define RCC_APB1ENR_I2C2EN_Msk (0x1UL << RCC_APB1ENR_I2C2EN_Pos) /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN RCC_APB1ENR_I2C2EN_Msk
#define RCC_APB1ENR_I2C3EN_Pos (23U)
#define RCC_APB1ENR_I2C3EN_Msk (0x1UL << RCC_APB1ENR_I2C3EN_Pos) /*!< 0x00800000 */
#define RCC_APB1ENR_I2C3EN RCC_APB1ENR_I2C3EN_Msk
#define RCC_APB1ENR_FMPI2C1EN_Pos (24U)
#define RCC_APB1ENR_FMPI2C1EN_Msk (0x1UL << RCC_APB1ENR_FMPI2C1EN_Pos) /*!< 0x01000000 */
#define RCC_APB1ENR_FMPI2C1EN RCC_APB1ENR_FMPI2C1EN_Msk
#define RCC_APB1ENR_CAN1EN_Pos (25U)
#define RCC_APB1ENR_CAN1EN_Msk (0x1UL << RCC_APB1ENR_CAN1EN_Pos) /*!< 0x02000000 */
#define RCC_APB1ENR_CAN1EN RCC_APB1ENR_CAN1EN_Msk
#define RCC_APB1ENR_CAN2EN_Pos (26U)
#define RCC_APB1ENR_CAN2EN_Msk (0x1UL << RCC_APB1ENR_CAN2EN_Pos) /*!< 0x04000000 */
#define RCC_APB1ENR_CAN2EN RCC_APB1ENR_CAN2EN_Msk
#define RCC_APB1ENR_CECEN_Pos (27U)
#define RCC_APB1ENR_CECEN_Msk (0x1UL << RCC_APB1ENR_CECEN_Pos) /*!< 0x08000000 */
#define RCC_APB1ENR_CECEN RCC_APB1ENR_CECEN_Msk
#define RCC_APB1ENR_PWREN_Pos (28U)
#define RCC_APB1ENR_PWREN_Msk (0x1UL << RCC_APB1ENR_PWREN_Pos) /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN RCC_APB1ENR_PWREN_Msk
#define RCC_APB1ENR_DACEN_Pos (29U)
#define RCC_APB1ENR_DACEN_Msk (0x1UL << RCC_APB1ENR_DACEN_Pos) /*!< 0x20000000 */
#define RCC_APB1ENR_DACEN RCC_APB1ENR_DACEN_Msk

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define RCC_APB2ENR_TIM1EN_Pos (0U)
#define RCC_APB2ENR_TIM1EN_Msk (0x1UL << RCC_APB2ENR_TIM1EN_Pos) /*!< 0x00000001 */
#define RCC_APB2ENR_TIM1EN RCC_APB2ENR_TIM1EN_Msk
#define RCC_APB2ENR_TIM8EN_Pos (1U)
#define RCC_APB2ENR_TIM8EN_Msk (0x1UL << RCC_APB2ENR_TIM8EN_Pos) /*!< 0x00000002 */
#define RCC_APB2ENR_TIM8EN RCC_APB2ENR_TIM8EN_Msk
#define RCC_APB2ENR_USART1EN_Pos (4U)
#define RCC_APB2ENR_USART1EN_Msk (0x1UL << RCC_APB2ENR_USART1EN_Pos) /*!< 0x00000010 */
#define RCC_APB2ENR_USART1EN RCC_APB2ENR_USART1EN_Msk
#define RCC_APB2ENR_USART6EN_Pos (5U)
#define RCC_APB2ENR_USART6EN_Msk (0x1UL << RCC_APB2ENR_USART6EN_Pos) /*!< 0x00000020 */
#define RCC_APB2ENR_USART6EN RCC_APB2ENR_USART6EN_Msk
#define RCC_APB2ENR_ADC1EN_Pos (8U)
#define RCC_APB2ENR_ADC1EN_Msk (0x1UL << RCC_APB2ENR_ADC1EN_Pos) /*!< 0x00000100 */
#define RCC_APB2ENR_ADC1EN RCC_APB2ENR_ADC1EN_Msk
#define RCC_APB2ENR_ADC2EN_Pos (9U)
#define RCC_APB2ENR_ADC2EN_Msk (0x1UL << RCC_APB2ENR_ADC2EN_Pos) /*!< 0x00000200 */
#define RCC_APB2ENR_ADC2EN RCC_APB2ENR_ADC2EN_Msk
#define RCC_APB2ENR_ADC3EN_Pos (10U)
#define RCC_APB2ENR_ADC3EN_Msk (0x1UL << RCC_APB2ENR_ADC3EN_Pos) /*!< 0x00000400 */
#define RCC_APB2ENR_ADC3EN RCC_APB2ENR_ADC3EN_Msk
#define RCC_APB2ENR_SDIOEN_Pos (11U)
#define RCC_APB2ENR_SDIOEN_Msk (0x1UL << RCC_APB2ENR_SDIOEN_Pos) /*!< 0x00000800 */
#define RCC_APB2ENR_SDIOEN RCC_APB2ENR_SDIOEN_Msk
#define RCC_APB2ENR_SPI1EN_Pos (12U)
#define RCC_APB2ENR_SPI1EN_Msk (0x1UL << RCC_APB2ENR_SPI1EN_Pos) /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN RCC_APB2ENR_SPI1EN_Msk
#define RCC_APB2ENR_SPI4EN_Pos (13U)
#define RCC_APB2ENR_SPI4EN_Msk (0x1UL << RCC_APB2ENR_SPI4EN_Pos) /*!< 0x00002000 */
#define RCC_APB2ENR_SPI4EN RCC_APB2ENR_SPI4EN_Msk
#define RCC_APB2ENR_SYSCFGEN_Pos (14U)
#define RCC_APB2ENR_SYSCFGEN_Msk (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos) /*!< 0x00004000 */
#define RCC_APB2ENR_SYSCFGEN RCC_APB2ENR_SYSCFGEN_Msk
#define RCC_APB2ENR_TIM9EN_Pos (16U)
#define RCC_APB2ENR_TIM9EN_Msk (0x1UL << RCC_APB2ENR_TIM9EN_Pos) /*!< 0x00010000 */
#define RCC_APB2ENR_TIM9EN RCC_APB2ENR_TIM9EN_Msk
#define RCC_APB2ENR_TIM10EN_Pos (17U)
#define RCC_APB2ENR_TIM10EN_Msk (0x1UL << RCC_APB2ENR_TIM10EN_Pos) /*!< 0x00020000 */
#define RCC_APB2ENR_TIM10EN RCC_APB2ENR_TIM10EN_Msk
#define RCC_APB2ENR_TIM11EN_Pos (18U)
#define RCC_APB2ENR_TIM11EN_Msk (0x1UL << RCC_APB2ENR_TIM11EN_Pos) /*!< 0x00040000 */
#define RCC_APB2ENR_TIM11EN RCC_APB2ENR_TIM11EN_Msk
#define RCC_APB2ENR_SAI1EN_Pos (22U)
#define RCC_APB2ENR_SAI1EN_Msk (0x1UL << RCC_APB2ENR_SAI1EN_Pos) /*!< 0x00400000 */
#define RCC_APB2ENR_SAI1EN RCC_APB2ENR_SAI1EN_Msk
#define RCC_APB2ENR_SAI2EN_Pos (23U)
#define RCC_APB2ENR_SAI2EN_Msk (0x1UL << RCC_APB2ENR_SAI2EN_Pos) /*!< 0x00800000 */
#define RCC_APB2ENR_SAI2EN RCC_APB2ENR_SAI2EN_Msk

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define RCC_AHB1LPENR_GPIOALPEN_Pos (0U)
#define RCC_AHB1LPENR_GPIOALPEN_Msk (0x1UL << RCC_AHB1LPENR_GPIOALPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB1LPENR_GPIOALPEN RCC_AHB1LPENR_GPIOALPEN_Msk
#define RCC_AHB1LPENR_GPIOBLPEN_Pos (1U)
#define RCC_AHB1LPENR_GPIOBLPEN_Msk (0x1UL << RCC_AHB1LPENR_GPIOBLPEN_Pos) /*!< 0x00000002 */
#define RCC_AHB1LPENR_GPIOBLPEN RCC_AHB1LPENR_GPIOBLPEN_Msk
#define RCC_AHB1LPENR_GPIOCLPEN_Pos (2U)
#define RCC_AHB1LPENR_GPIOCLPEN_Msk (0x1UL << RCC_AHB1LPENR_GPIOCLPEN_Pos) /*!< 0x00000004 */
#define RCC_AHB1LPENR_GPIOCLPEN RCC_AHB1LPENR_GPIOCLPEN_Msk
#define RCC_AHB1LPENR_GPIODLPEN_Pos (3U)
#define RCC_AHB1LPENR_GPIODLPEN_Msk (0x1UL << RCC_AHB1LPENR_GPIODLPEN_Pos) /*!< 0x00000008 */
#define RCC_AHB1LPENR_GPIODLPEN RCC_AHB1LPENR_GPIODLPEN_Msk
#define RCC_AHB1LPENR_GPIOELPEN_Pos (4U)
#define RCC_AHB1LPENR_GPIOELPEN_Msk (0x1UL << RCC_AHB1LPENR_GPIOELPEN_Pos) /*!< 0x00000010 */
#define RCC_AHB1LPENR_GPIOELPEN RCC_AHB1LPENR_GPIOELPEN_Msk
#define RCC_AHB1LPENR_GPIOFLPEN_Pos (5U)
#define RCC_AHB1LPENR_GPIOFLPEN_Msk (0x1UL << RCC_AHB1LPENR_GPIOFLPEN_Pos) /*!< 0x00000020 */
#define RCC_AHB1LPENR_GPIOFLPEN RCC_AHB1LPENR_GPIOFLPEN_Msk
#define RCC_AHB1LPENR_GPIOGLPEN_Pos (6U)
#define RCC_AHB1LPENR_GPIOGLPEN_Msk (0x1UL << RCC_AHB1LPENR_GPIOGLPEN_Pos) /*!< 0x00000040 */
#define RCC_AHB1LPENR_GPIOGLPEN RCC_AHB1LPENR_GPIOGLPEN_Msk
#define RCC_AHB1LPENR_GPIOHLPEN_Pos (7U)
#define RCC_AHB1LPENR_GPIOHLPEN_Msk (0x1UL << RCC_AHB1LPENR_GPIOHLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB1LPENR_GPIOHLPEN RCC_AHB1LPENR_GPIOHLPEN_Msk
#define RCC_AHB1LPENR_CRCLPEN_Pos (12U)
#define RCC_AHB1LPENR_CRCLPEN_Msk (0x1UL << RCC_AHB1LPENR_CRCLPEN_Pos) /*!< 0x00001000 */
#define RCC_AHB1LPENR_CRCLPEN RCC_AHB1LPENR_CRCLPEN_Msk
#define RCC_AHB1LPENR_FLITFLPEN_Pos (15U)
#define RCC_AHB1LPENR_FLITFLPEN_Msk (0x1UL << RCC_AHB1LPENR_FLITFLPEN_Pos) /*!< 0x00008000 */
#define RCC_AHB1LPENR_FLITFLPEN RCC_AHB1LPENR_FLITFLPEN_Msk
#define RCC_AHB1LPENR_SRAM1LPEN_Pos (16U)
#define RCC_AHB1LPENR_SRAM1LPEN_Msk (0x1UL << RCC_AHB1LPENR_SRAM1LPEN_Pos) /*!< 0x00010000 */
#define RCC_AHB1LPENR_SRAM1LPEN RCC_AHB1LPENR_SRAM1LPEN_Msk
#define RCC_AHB1LPENR_SRAM2LPEN_Pos (17U)
#define RCC_AHB1LPENR_SRAM2LPEN_Msk (0x1UL << RCC_AHB1LPENR_SRAM2LPEN_Pos) /*!< 0x00020000 */
#define RCC_AHB1LPENR_SRAM2LPEN RCC_AHB1LPENR_SRAM2LPEN_Msk
#define RCC_AHB1LPENR_BKPSRAMLPEN_Pos (18U)
#define RCC_AHB1LPENR_BKPSRAMLPEN_Msk (0x1UL << RCC_AHB1LPENR_BKPSRAMLPEN_Pos) /*!< 0x00040000 */
#define RCC_AHB1LPENR_BKPSRAMLPEN RCC_AHB1LPENR_BKPSRAMLPEN_Msk
#define RCC_AHB1LPENR_DMA1LPEN_Pos (21U)
#define RCC_AHB1LPENR_DMA1LPEN_Msk (0x1UL << RCC_AHB1LPENR_DMA1LPEN_Pos) /*!< 0x00200000 */
#define RCC_AHB1LPENR_DMA1LPEN RCC_AHB1LPENR_DMA1LPEN_Msk
#define RCC_AHB1LPENR_DMA2LPEN_Pos (22U)
#define RCC_AHB1LPENR_DMA2LPEN_Msk (0x1UL << RCC_AHB1LPENR_DMA2LPEN_Pos) /*!< 0x00400000 */
#define RCC_AHB1LPENR_DMA2LPEN RCC_AHB1LPENR_DMA2LPEN_Msk

#define RCC_AHB1LPENR_OTGHSLPEN_Pos (29U)
#define RCC_AHB1LPENR_OTGHSLPEN_Msk (0x1UL << RCC_AHB1LPENR_OTGHSLPEN_Pos) /*!< 0x20000000 */
#define RCC_AHB1LPENR_OTGHSLPEN RCC_AHB1LPENR_OTGHSLPEN_Msk
#define RCC_AHB1LPENR_OTGHSULPILPEN_Pos (30U)
#define RCC_AHB1LPENR_OTGHSULPILPEN_Msk (0x1UL << RCC_AHB1LPENR_OTGHSULPILPEN_Pos) /*!< 0x40000000 */
#define RCC_AHB1LPENR_OTGHSULPILPEN RCC_AHB1LPENR_OTGHSULPILPEN_Msk

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define RCC_AHB2LPENR_DCMILPEN_Pos (0U)
#define RCC_AHB2LPENR_DCMILPEN_Msk (0x1UL << RCC_AHB2LPENR_DCMILPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB2LPENR_DCMILPEN RCC_AHB2LPENR_DCMILPEN_Msk
#define RCC_AHB2LPENR_OTGFSLPEN_Pos (7U)
#define RCC_AHB2LPENR_OTGFSLPEN_Msk (0x1UL << RCC_AHB2LPENR_OTGFSLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB2LPENR_OTGFSLPEN RCC_AHB2LPENR_OTGFSLPEN_Msk

/********************  Bit definition for RCC_AHB3LPENR register  *************/
#define RCC_AHB3LPENR_FMCLPEN_Pos (0U)
#define RCC_AHB3LPENR_FMCLPEN_Msk (0x1UL << RCC_AHB3LPENR_FMCLPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB3LPENR_FMCLPEN RCC_AHB3LPENR_FMCLPEN_Msk
#define RCC_AHB3LPENR_QSPILPEN_Pos (1U)
#define RCC_AHB3LPENR_QSPILPEN_Msk (0x1UL << RCC_AHB3LPENR_QSPILPEN_Pos) /*!< 0x00000002 */
#define RCC_AHB3LPENR_QSPILPEN RCC_AHB3LPENR_QSPILPEN_Msk

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define RCC_APB1LPENR_TIM2LPEN_Pos (0U)
#define RCC_APB1LPENR_TIM2LPEN_Msk (0x1UL << RCC_APB1LPENR_TIM2LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB1LPENR_TIM2LPEN RCC_APB1LPENR_TIM2LPEN_Msk
#define RCC_APB1LPENR_TIM3LPEN_Pos (1U)
#define RCC_APB1LPENR_TIM3LPEN_Msk (0x1UL << RCC_APB1LPENR_TIM3LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB1LPENR_TIM3LPEN RCC_APB1LPENR_TIM3LPEN_Msk
#define RCC_APB1LPENR_TIM4LPEN_Pos (2U)
#define RCC_APB1LPENR_TIM4LPEN_Msk (0x1UL << RCC_APB1LPENR_TIM4LPEN_Pos) /*!< 0x00000004 */
#define RCC_APB1LPENR_TIM4LPEN RCC_APB1LPENR_TIM4LPEN_Msk
#define RCC_APB1LPENR_TIM5LPEN_Pos (3U)
#define RCC_APB1LPENR_TIM5LPEN_Msk (0x1UL << RCC_APB1LPENR_TIM5LPEN_Pos) /*!< 0x00000008 */
#define RCC_APB1LPENR_TIM5LPEN RCC_APB1LPENR_TIM5LPEN_Msk
#define RCC_APB1LPENR_TIM6LPEN_Pos (4U)
#define RCC_APB1LPENR_TIM6LPEN_Msk (0x1UL << RCC_APB1LPENR_TIM6LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB1LPENR_TIM6LPEN RCC_APB1LPENR_TIM6LPEN_Msk
#define RCC_APB1LPENR_TIM7LPEN_Pos (5U)
#define RCC_APB1LPENR_TIM7LPEN_Msk (0x1UL << RCC_APB1LPENR_TIM7LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB1LPENR_TIM7LPEN RCC_APB1LPENR_TIM7LPEN_Msk
#define RCC_APB1LPENR_TIM12LPEN_Pos (6U)
#define RCC_APB1LPENR_TIM12LPEN_Msk (0x1UL << RCC_APB1LPENR_TIM12LPEN_Pos) /*!< 0x00000040 */
#define RCC_APB1LPENR_TIM12LPEN RCC_APB1LPENR_TIM12LPEN_Msk
#define RCC_APB1LPENR_TIM13LPEN_Pos (7U)
#define RCC_APB1LPENR_TIM13LPEN_Msk (0x1UL << RCC_APB1LPENR_TIM13LPEN_Pos) /*!< 0x00000080 */
#define RCC_APB1LPENR_TIM13LPEN RCC_APB1LPENR_TIM13LPEN_Msk
#define RCC_APB1LPENR_TIM14LPEN_Pos (8U)
#define RCC_APB1LPENR_TIM14LPEN_Msk (0x1UL << RCC_APB1LPENR_TIM14LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB1LPENR_TIM14LPEN RCC_APB1LPENR_TIM14LPEN_Msk
#define RCC_APB1LPENR_WWDGLPEN_Pos (11U)
#define RCC_APB1LPENR_WWDGLPEN_Msk (0x1UL << RCC_APB1LPENR_WWDGLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB1LPENR_WWDGLPEN RCC_APB1LPENR_WWDGLPEN_Msk
#define RCC_APB1LPENR_SPI2LPEN_Pos (14U)
#define RCC_APB1LPENR_SPI2LPEN_Msk (0x1UL << RCC_APB1LPENR_SPI2LPEN_Pos) /*!< 0x00004000 */
#define RCC_APB1LPENR_SPI2LPEN RCC_APB1LPENR_SPI2LPEN_Msk
#define RCC_APB1LPENR_SPI3LPEN_Pos (15U)
#define RCC_APB1LPENR_SPI3LPEN_Msk (0x1UL << RCC_APB1LPENR_SPI3LPEN_Pos) /*!< 0x00008000 */
#define RCC_APB1LPENR_SPI3LPEN RCC_APB1LPENR_SPI3LPEN_Msk
#define RCC_APB1LPENR_SPDIFRXLPEN_Pos (16U)
#define RCC_APB1LPENR_SPDIFRXLPEN_Msk (0x1UL << RCC_APB1LPENR_SPDIFRXLPEN_Pos) /*!< 0x00010000 */
#define RCC_APB1LPENR_SPDIFRXLPEN RCC_APB1LPENR_SPDIFRXLPEN_Msk
#define RCC_APB1LPENR_USART2LPEN_Pos (17U)
#define RCC_APB1LPENR_USART2LPEN_Msk (0x1UL << RCC_APB1LPENR_USART2LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB1LPENR_USART2LPEN RCC_APB1LPENR_USART2LPEN_Msk
#define RCC_APB1LPENR_USART3LPEN_Pos (18U)
#define RCC_APB1LPENR_USART3LPEN_Msk (0x1UL << RCC_APB1LPENR_USART3LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB1LPENR_USART3LPEN RCC_APB1LPENR_USART3LPEN_Msk
#define RCC_APB1LPENR_UART4LPEN_Pos (19U)
#define RCC_APB1LPENR_UART4LPEN_Msk (0x1UL << RCC_APB1LPENR_UART4LPEN_Pos) /*!< 0x00080000 */
#define RCC_APB1LPENR_UART4LPEN RCC_APB1LPENR_UART4LPEN_Msk
#define RCC_APB1LPENR_UART5LPEN_Pos (20U)
#define RCC_APB1LPENR_UART5LPEN_Msk (0x1UL << RCC_APB1LPENR_UART5LPEN_Pos) /*!< 0x00100000 */
#define RCC_APB1LPENR_UART5LPEN RCC_APB1LPENR_UART5LPEN_Msk
#define RCC_APB1LPENR_I2C1LPEN_Pos (21U)
#define RCC_APB1LPENR_I2C1LPEN_Msk (0x1UL << RCC_APB1LPENR_I2C1LPEN_Pos) /*!< 0x00200000 */
#define RCC_APB1LPENR_I2C1LPEN RCC_APB1LPENR_I2C1LPEN_Msk
#define RCC_APB1LPENR_I2C2LPEN_Pos (22U)
#define RCC_APB1LPENR_I2C2LPEN_Msk (0x1UL << RCC_APB1LPENR_I2C2LPEN_Pos) /*!< 0x00400000 */
#define RCC_APB1LPENR_I2C2LPEN RCC_APB1LPENR_I2C2LPEN_Msk
#define RCC_APB1LPENR_I2C3LPEN_Pos (23U)
#define RCC_APB1LPENR_I2C3LPEN_Msk (0x1UL << RCC_APB1LPENR_I2C3LPEN_Pos) /*!< 0x00800000 */
#define RCC_APB1LPENR_I2C3LPEN RCC_APB1LPENR_I2C3LPEN_Msk
#define RCC_APB1LPENR_FMPI2C1LPEN_Pos (24U)
#define RCC_APB1LPENR_FMPI2C1LPEN_Msk (0x1UL << RCC_APB1LPENR_FMPI2C1LPEN_Pos) /*!< 0x01000000 */
#define RCC_APB1LPENR_FMPI2C1LPEN RCC_APB1LPENR_FMPI2C1LPEN_Msk
#define RCC_APB1LPENR_CAN1LPEN_Pos (25U)
#define RCC_APB1LPENR_CAN1LPEN_Msk (0x1UL << RCC_APB1LPENR_CAN1LPEN_Pos) /*!< 0x02000000 */
#define RCC_APB1LPENR_CAN1LPEN RCC_APB1LPENR_CAN1LPEN_Msk
#define RCC_APB1LPENR_CAN2LPEN_Pos (26U)
#define RCC_APB1LPENR_CAN2LPEN_Msk (0x1UL << RCC_APB1LPENR_CAN2LPEN_Pos) /*!< 0x04000000 */
#define RCC_APB1LPENR_CAN2LPEN RCC_APB1LPENR_CAN2LPEN_Msk
#define RCC_APB1LPENR_CECLPEN_Pos (27U)
#define RCC_APB1LPENR_CECLPEN_Msk (0x1UL << RCC_APB1LPENR_CECLPEN_Pos) /*!< 0x08000000 */
#define RCC_APB1LPENR_CECLPEN RCC_APB1LPENR_CECLPEN_Msk
#define RCC_APB1LPENR_PWRLPEN_Pos (28U)
#define RCC_APB1LPENR_PWRLPEN_Msk (0x1UL << RCC_APB1LPENR_PWRLPEN_Pos) /*!< 0x10000000 */
#define RCC_APB1LPENR_PWRLPEN RCC_APB1LPENR_PWRLPEN_Msk
#define RCC_APB1LPENR_DACLPEN_Pos (29U)
#define RCC_APB1LPENR_DACLPEN_Msk (0x1UL << RCC_APB1LPENR_DACLPEN_Pos) /*!< 0x20000000 */
#define RCC_APB1LPENR_DACLPEN RCC_APB1LPENR_DACLPEN_Msk

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define RCC_APB2LPENR_TIM1LPEN_Pos (0U)
#define RCC_APB2LPENR_TIM1LPEN_Msk (0x1UL << RCC_APB2LPENR_TIM1LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB2LPENR_TIM1LPEN RCC_APB2LPENR_TIM1LPEN_Msk
#define RCC_APB2LPENR_TIM8LPEN_Pos (1U)
#define RCC_APB2LPENR_TIM8LPEN_Msk (0x1UL << RCC_APB2LPENR_TIM8LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB2LPENR_TIM8LPEN RCC_APB2LPENR_TIM8LPEN_Msk
#define RCC_APB2LPENR_USART1LPEN_Pos (4U)
#define RCC_APB2LPENR_USART1LPEN_Msk (0x1UL << RCC_APB2LPENR_USART1LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB2LPENR_USART1LPEN RCC_APB2LPENR_USART1LPEN_Msk
#define RCC_APB2LPENR_USART6LPEN_Pos (5U)
#define RCC_APB2LPENR_USART6LPEN_Msk (0x1UL << RCC_APB2LPENR_USART6LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB2LPENR_USART6LPEN RCC_APB2LPENR_USART6LPEN_Msk
#define RCC_APB2LPENR_ADC1LPEN_Pos (8U)
#define RCC_APB2LPENR_ADC1LPEN_Msk (0x1UL << RCC_APB2LPENR_ADC1LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB2LPENR_ADC1LPEN RCC_APB2LPENR_ADC1LPEN_Msk
#define RCC_APB2LPENR_ADC2LPEN_Pos (9U)
#define RCC_APB2LPENR_ADC2LPEN_Msk (0x1UL << RCC_APB2LPENR_ADC2LPEN_Pos) /*!< 0x00000200 */
#define RCC_APB2LPENR_ADC2LPEN RCC_APB2LPENR_ADC2LPEN_Msk
#define RCC_APB2LPENR_ADC3LPEN_Pos (10U)
#define RCC_APB2LPENR_ADC3LPEN_Msk (0x1UL << RCC_APB2LPENR_ADC3LPEN_Pos) /*!< 0x00000400 */
#define RCC_APB2LPENR_ADC3LPEN RCC_APB2LPENR_ADC3LPEN_Msk
#define RCC_APB2LPENR_SDIOLPEN_Pos (11U)
#define RCC_APB2LPENR_SDIOLPEN_Msk (0x1UL << RCC_APB2LPENR_SDIOLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB2LPENR_SDIOLPEN RCC_APB2LPENR_SDIOLPEN_Msk
#define RCC_APB2LPENR_SPI1LPEN_Pos (12U)
#define RCC_APB2LPENR_SPI1LPEN_Msk (0x1UL << RCC_APB2LPENR_SPI1LPEN_Pos) /*!< 0x00001000 */
#define RCC_APB2LPENR_SPI1LPEN RCC_APB2LPENR_SPI1LPEN_Msk
#define RCC_APB2LPENR_SPI4LPEN_Pos (13U)
#define RCC_APB2LPENR_SPI4LPEN_Msk (0x1UL << RCC_APB2LPENR_SPI4LPEN_Pos) /*!< 0x00002000 */
#define RCC_APB2LPENR_SPI4LPEN RCC_APB2LPENR_SPI4LPEN_Msk
#define RCC_APB2LPENR_SYSCFGLPEN_Pos (14U)
#define RCC_APB2LPENR_SYSCFGLPEN_Msk (0x1UL << RCC_APB2LPENR_SYSCFGLPEN_Pos) /*!< 0x00004000 */
#define RCC_APB2LPENR_SYSCFGLPEN RCC_APB2LPENR_SYSCFGLPEN_Msk
#define RCC_APB2LPENR_TIM9LPEN_Pos (16U)
#define RCC_APB2LPENR_TIM9LPEN_Msk (0x1UL << RCC_APB2LPENR_TIM9LPEN_Pos) /*!< 0x00010000 */
#define RCC_APB2LPENR_TIM9LPEN RCC_APB2LPENR_TIM9LPEN_Msk
#define RCC_APB2LPENR_TIM10LPEN_Pos (17U)
#define RCC_APB2LPENR_TIM10LPEN_Msk (0x1UL << RCC_APB2LPENR_TIM10LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB2LPENR_TIM10LPEN RCC_APB2LPENR_TIM10LPEN_Msk
#define RCC_APB2LPENR_TIM11LPEN_Pos (18U)
#define RCC_APB2LPENR_TIM11LPEN_Msk (0x1UL << RCC_APB2LPENR_TIM11LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB2LPENR_TIM11LPEN RCC_APB2LPENR_TIM11LPEN_Msk
#define RCC_APB2LPENR_SAI1LPEN_Pos (22U)
#define RCC_APB2LPENR_SAI1LPEN_Msk (0x1UL << RCC_APB2LPENR_SAI1LPEN_Pos) /*!< 0x00400000 */
#define RCC_APB2LPENR_SAI1LPEN RCC_APB2LPENR_SAI1LPEN_Msk
#define RCC_APB2LPENR_SAI2LPEN_Pos (23U)
#define RCC_APB2LPENR_SAI2LPEN_Msk (0x1UL << RCC_APB2LPENR_SAI2LPEN_Pos) /*!< 0x00800000 */
#define RCC_APB2LPENR_SAI2LPEN RCC_APB2LPENR_SAI2LPEN_Msk

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSEON_Pos (0U)
#define RCC_BDCR_LSEON_Msk (0x1UL << RCC_BDCR_LSEON_Pos) /*!< 0x00000001 */
#define RCC_BDCR_LSEON RCC_BDCR_LSEON_Msk
#define RCC_BDCR_LSERDY_Pos (1U)
#define RCC_BDCR_LSERDY_Msk (0x1UL << RCC_BDCR_LSERDY_Pos) /*!< 0x00000002 */
#define RCC_BDCR_LSERDY RCC_BDCR_LSERDY_Msk
#define RCC_BDCR_LSEBYP_Pos (2U)
#define RCC_BDCR_LSEBYP_Msk (0x1UL << RCC_BDCR_LSEBYP_Pos) /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP RCC_BDCR_LSEBYP_Msk
#define RCC_BDCR_LSEMOD_Pos (3U)
#define RCC_BDCR_LSEMOD_Msk (0x1UL << RCC_BDCR_LSEMOD_Pos) /*!< 0x00000008 */
#define RCC_BDCR_LSEMOD RCC_BDCR_LSEMOD_Msk

#define RCC_BDCR_RTCSEL_Pos (8U)
#define RCC_BDCR_RTCSEL_Msk (0x3UL << RCC_BDCR_RTCSEL_Pos) /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL RCC_BDCR_RTCSEL_Msk
#define RCC_BDCR_RTCSEL_0 (0x1UL << RCC_BDCR_RTCSEL_Pos) /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1 (0x2UL << RCC_BDCR_RTCSEL_Pos) /*!< 0x00000200 */

#define RCC_BDCR_RTCEN_Pos (15U)
#define RCC_BDCR_RTCEN_Msk (0x1UL << RCC_BDCR_RTCEN_Pos) /*!< 0x00008000 */
#define RCC_BDCR_RTCEN RCC_BDCR_RTCEN_Msk
#define RCC_BDCR_BDRST_Pos (16U)
#define RCC_BDCR_BDRST_Msk (0x1UL << RCC_BDCR_BDRST_Pos) /*!< 0x00010000 */
#define RCC_BDCR_BDRST RCC_BDCR_BDRST_Msk

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos (0U)
#define RCC_CSR_LSION_Msk (0x1UL << RCC_CSR_LSION_Pos) /*!< 0x00000001 */
#define RCC_CSR_LSION RCC_CSR_LSION_Msk
#define RCC_CSR_LSIRDY_Pos (1U)
#define RCC_CSR_LSIRDY_Msk (0x1UL << RCC_CSR_LSIRDY_Pos) /*!< 0x00000002 */
#define RCC_CSR_LSIRDY RCC_CSR_LSIRDY_Msk
#define RCC_CSR_RMVF_Pos (24U)
#define RCC_CSR_RMVF_Msk (0x1UL << RCC_CSR_RMVF_Pos) /*!< 0x01000000 */
#define RCC_CSR_RMVF RCC_CSR_RMVF_Msk
#define RCC_CSR_BORRSTF_Pos (25U)
#define RCC_CSR_BORRSTF_Msk (0x1UL << RCC_CSR_BORRSTF_Pos) /*!< 0x02000000 */
#define RCC_CSR_BORRSTF RCC_CSR_BORRSTF_Msk
#define RCC_CSR_PINRSTF_Pos (26U)
#define RCC_CSR_PINRSTF_Msk (0x1UL << RCC_CSR_PINRSTF_Pos) /*!< 0x04000000 */
#define RCC_CSR_PINRSTF RCC_CSR_PINRSTF_Msk
#define RCC_CSR_PORRSTF_Pos (27U)
#define RCC_CSR_PORRSTF_Msk (0x1UL << RCC_CSR_PORRSTF_Pos) /*!< 0x08000000 */
#define RCC_CSR_PORRSTF RCC_CSR_PORRSTF_Msk
#define RCC_CSR_SFTRSTF_Pos (28U)
#define RCC_CSR_SFTRSTF_Msk (0x1UL << RCC_CSR_SFTRSTF_Pos) /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF RCC_CSR_SFTRSTF_Msk
#define RCC_CSR_IWDGRSTF_Pos (29U)
#define RCC_CSR_IWDGRSTF_Msk (0x1UL << RCC_CSR_IWDGRSTF_Pos) /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF RCC_CSR_IWDGRSTF_Msk
#define RCC_CSR_WWDGRSTF_Pos (30U)
#define RCC_CSR_WWDGRSTF_Msk (0x1UL << RCC_CSR_WWDGRSTF_Pos) /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF RCC_CSR_WWDGRSTF_Msk
#define RCC_CSR_LPWRRSTF_Pos (31U)
#define RCC_CSR_LPWRRSTF_Msk (0x1UL << RCC_CSR_LPWRRSTF_Pos) /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF RCC_CSR_LPWRRSTF_Msk
/* Legacy defines */
#define RCC_CSR_PADRSTF RCC_CSR_PINRSTF
#define RCC_CSR_WDGRSTF RCC_CSR_IWDGRSTF

/********************  Bit definition for RCC_SSCGR register  *****************/
#define RCC_SSCGR_MODPER_Pos (0U)
#define RCC_SSCGR_MODPER_Msk (0x1FFFUL << RCC_SSCGR_MODPER_Pos) /*!< 0x00001FFF */
#define RCC_SSCGR_MODPER RCC_SSCGR_MODPER_Msk
#define RCC_SSCGR_INCSTEP_Pos (13U)
#define RCC_SSCGR_INCSTEP_Msk (0x7FFFUL << RCC_SSCGR_INCSTEP_Pos) /*!< 0x0FFFE000 */
#define RCC_SSCGR_INCSTEP RCC_SSCGR_INCSTEP_Msk
#define RCC_SSCGR_SPREADSEL_Pos (30U)
#define RCC_SSCGR_SPREADSEL_Msk (0x1UL << RCC_SSCGR_SPREADSEL_Pos) /*!< 0x40000000 */
#define RCC_SSCGR_SPREADSEL RCC_SSCGR_SPREADSEL_Msk
#define RCC_SSCGR_SSCGEN_Pos (31U)
#define RCC_SSCGR_SSCGEN_Msk (0x1UL << RCC_SSCGR_SSCGEN_Pos) /*!< 0x80000000 */
#define RCC_SSCGR_SSCGEN RCC_SSCGR_SSCGEN_Msk

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define RCC_PLLI2SCFGR_PLLI2SM_Pos (0U)
#define RCC_PLLI2SCFGR_PLLI2SM_Msk (0x3FUL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x0000003F */
#define RCC_PLLI2SCFGR_PLLI2SM RCC_PLLI2SCFGR_PLLI2SM_Msk
#define RCC_PLLI2SCFGR_PLLI2SM_0 (0x01UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000001 */
#define RCC_PLLI2SCFGR_PLLI2SM_1 (0x02UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000002 */
#define RCC_PLLI2SCFGR_PLLI2SM_2 (0x04UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000004 */
#define RCC_PLLI2SCFGR_PLLI2SM_3 (0x08UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000008 */
#define RCC_PLLI2SCFGR_PLLI2SM_4 (0x10UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000010 */
#define RCC_PLLI2SCFGR_PLLI2SM_5 (0x20UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000020 */

#define RCC_PLLI2SCFGR_PLLI2SN_Pos (6U)
#define RCC_PLLI2SCFGR_PLLI2SN_Msk (0x1FFUL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00007FC0 */
#define RCC_PLLI2SCFGR_PLLI2SN RCC_PLLI2SCFGR_PLLI2SN_Msk
#define RCC_PLLI2SCFGR_PLLI2SN_0 (0x001UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000040 */
#define RCC_PLLI2SCFGR_PLLI2SN_1 (0x002UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000080 */
#define RCC_PLLI2SCFGR_PLLI2SN_2 (0x004UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000100 */
#define RCC_PLLI2SCFGR_PLLI2SN_3 (0x008UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000200 */
#define RCC_PLLI2SCFGR_PLLI2SN_4 (0x010UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000400 */
#define RCC_PLLI2SCFGR_PLLI2SN_5 (0x020UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000800 */
#define RCC_PLLI2SCFGR_PLLI2SN_6 (0x040UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00001000 */
#define RCC_PLLI2SCFGR_PLLI2SN_7 (0x080UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00002000 */
#define RCC_PLLI2SCFGR_PLLI2SN_8 (0x100UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00004000 */

#define RCC_PLLI2SCFGR_PLLI2SP_Pos (16U)
#define RCC_PLLI2SCFGR_PLLI2SP_Msk (0x3UL << RCC_PLLI2SCFGR_PLLI2SP_Pos) /*!< 0x00030000 */
#define RCC_PLLI2SCFGR_PLLI2SP RCC_PLLI2SCFGR_PLLI2SP_Msk
#define RCC_PLLI2SCFGR_PLLI2SP_0 (0x1UL << RCC_PLLI2SCFGR_PLLI2SP_Pos) /*!< 0x00010000 */
#define RCC_PLLI2SCFGR_PLLI2SP_1 (0x2UL << RCC_PLLI2SCFGR_PLLI2SP_Pos) /*!< 0x00020000 */
#define RCC_PLLI2SCFGR_PLLI2SQ_Pos (24U)
#define RCC_PLLI2SCFGR_PLLI2SQ_Msk (0xFUL << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x0F000000 */
#define RCC_PLLI2SCFGR_PLLI2SQ RCC_PLLI2SCFGR_PLLI2SQ_Msk
#define RCC_PLLI2SCFGR_PLLI2SQ_0 (0x1UL << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x01000000 */
#define RCC_PLLI2SCFGR_PLLI2SQ_1 (0x2UL << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x02000000 */
#define RCC_PLLI2SCFGR_PLLI2SQ_2 (0x4UL << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x04000000 */
#define RCC_PLLI2SCFGR_PLLI2SQ_3 (0x8UL << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x08000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_Pos (28U)
#define RCC_PLLI2SCFGR_PLLI2SR_Msk (0x7UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x70000000 */
#define RCC_PLLI2SCFGR_PLLI2SR RCC_PLLI2SCFGR_PLLI2SR_Msk
#define RCC_PLLI2SCFGR_PLLI2SR_0 (0x1UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x10000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_1 (0x2UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x20000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_2 (0x4UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x40000000 */

/********************  Bit definition for RCC_PLLSAICFGR register  ************/
#define RCC_PLLSAICFGR_PLLSAIM_Pos (0U)
#define RCC_PLLSAICFGR_PLLSAIM_Msk (0x3FUL << RCC_PLLSAICFGR_PLLSAIM_Pos) /*!< 0x0000003F */
#define RCC_PLLSAICFGR_PLLSAIM RCC_PLLSAICFGR_PLLSAIM_Msk
#define RCC_PLLSAICFGR_PLLSAIM_0 (0x01UL << RCC_PLLSAICFGR_PLLSAIM_Pos) /*!< 0x00000001 */
#define RCC_PLLSAICFGR_PLLSAIM_1 (0x02UL << RCC_PLLSAICFGR_PLLSAIM_Pos) /*!< 0x00000002 */
#define RCC_PLLSAICFGR_PLLSAIM_2 (0x04UL << RCC_PLLSAICFGR_PLLSAIM_Pos) /*!< 0x00000004 */
#define RCC_PLLSAICFGR_PLLSAIM_3 (0x08UL << RCC_PLLSAICFGR_PLLSAIM_Pos) /*!< 0x00000008 */
#define RCC_PLLSAICFGR_PLLSAIM_4 (0x10UL << RCC_PLLSAICFGR_PLLSAIM_Pos) /*!< 0x00000010 */
#define RCC_PLLSAICFGR_PLLSAIM_5 (0x20UL << RCC_PLLSAICFGR_PLLSAIM_Pos) /*!< 0x00000020 */
#define RCC_PLLSAICFGR_PLLSAIN_Pos (6U)
#define RCC_PLLSAICFGR_PLLSAIN_Msk (0x1FFUL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00007FC0 */
#define RCC_PLLSAICFGR_PLLSAIN RCC_PLLSAICFGR_PLLSAIN_Msk
#define RCC_PLLSAICFGR_PLLSAIN_0 (0x001UL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000040 */
#define RCC_PLLSAICFGR_PLLSAIN_1 (0x002UL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000080 */
#define RCC_PLLSAICFGR_PLLSAIN_2 (0x004UL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000100 */
#define RCC_PLLSAICFGR_PLLSAIN_3 (0x008UL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000200 */
#define RCC_PLLSAICFGR_PLLSAIN_4 (0x010UL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000400 */
#define RCC_PLLSAICFGR_PLLSAIN_5 (0x020UL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000800 */
#define RCC_PLLSAICFGR_PLLSAIN_6 (0x040UL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00001000 */
#define RCC_PLLSAICFGR_PLLSAIN_7 (0x080UL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00002000 */
#define RCC_PLLSAICFGR_PLLSAIN_8 (0x100UL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00004000 */

#define RCC_PLLSAICFGR_PLLSAIP_Pos (16U)
#define RCC_PLLSAICFGR_PLLSAIP_Msk (0x3UL << RCC_PLLSAICFGR_PLLSAIP_Pos) /*!< 0x00030000 */
#define RCC_PLLSAICFGR_PLLSAIP RCC_PLLSAICFGR_PLLSAIP_Msk
#define RCC_PLLSAICFGR_PLLSAIP_0 (0x1UL << RCC_PLLSAICFGR_PLLSAIP_Pos) /*!< 0x00010000 */
#define RCC_PLLSAICFGR_PLLSAIP_1 (0x2UL << RCC_PLLSAICFGR_PLLSAIP_Pos) /*!< 0x00020000 */

#define RCC_PLLSAICFGR_PLLSAIQ_Pos (24U)
#define RCC_PLLSAICFGR_PLLSAIQ_Msk (0xFUL << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x0F000000 */
#define RCC_PLLSAICFGR_PLLSAIQ RCC_PLLSAICFGR_PLLSAIQ_Msk
#define RCC_PLLSAICFGR_PLLSAIQ_0 (0x1UL << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x01000000 */
#define RCC_PLLSAICFGR_PLLSAIQ_1 (0x2UL << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x02000000 */
#define RCC_PLLSAICFGR_PLLSAIQ_2 (0x4UL << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x04000000 */
#define RCC_PLLSAICFGR_PLLSAIQ_3 (0x8UL << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x08000000 */

/********************  Bit definition for RCC_DCKCFGR register  ***************/
#define RCC_DCKCFGR_PLLI2SDIVQ_Pos (0U)
#define RCC_DCKCFGR_PLLI2SDIVQ_Msk (0x1FUL << RCC_DCKCFGR_PLLI2SDIVQ_Pos) /*!< 0x0000001F */
#define RCC_DCKCFGR_PLLI2SDIVQ RCC_DCKCFGR_PLLI2SDIVQ_Msk
#define RCC_DCKCFGR_PLLI2SDIVQ_0 (0x01UL << RCC_DCKCFGR_PLLI2SDIVQ_Pos) /*!< 0x00000001 */
#define RCC_DCKCFGR_PLLI2SDIVQ_1 (0x02UL << RCC_DCKCFGR_PLLI2SDIVQ_Pos) /*!< 0x00000002 */
#define RCC_DCKCFGR_PLLI2SDIVQ_2 (0x04UL << RCC_DCKCFGR_PLLI2SDIVQ_Pos) /*!< 0x00000004 */
#define RCC_DCKCFGR_PLLI2SDIVQ_3 (0x08UL << RCC_DCKCFGR_PLLI2SDIVQ_Pos) /*!< 0x00000008 */
#define RCC_DCKCFGR_PLLI2SDIVQ_4 (0x10UL << RCC_DCKCFGR_PLLI2SDIVQ_Pos) /*!< 0x00000010 */

#define RCC_DCKCFGR_PLLSAIDIVQ_Pos (8U)
#define RCC_DCKCFGR_PLLSAIDIVQ_Msk (0x1FUL << RCC_DCKCFGR_PLLSAIDIVQ_Pos) /*!< 0x00001F00 */
#define RCC_DCKCFGR_PLLSAIDIVQ RCC_DCKCFGR_PLLSAIDIVQ_Msk
#define RCC_DCKCFGR_PLLSAIDIVQ_0 (0x01UL << RCC_DCKCFGR_PLLSAIDIVQ_Pos) /*!< 0x00000100 */
#define RCC_DCKCFGR_PLLSAIDIVQ_1 (0x02UL << RCC_DCKCFGR_PLLSAIDIVQ_Pos) /*!< 0x00000200 */
#define RCC_DCKCFGR_PLLSAIDIVQ_2 (0x04UL << RCC_DCKCFGR_PLLSAIDIVQ_Pos) /*!< 0x00000400 */
#define RCC_DCKCFGR_PLLSAIDIVQ_3 (0x08UL << RCC_DCKCFGR_PLLSAIDIVQ_Pos) /*!< 0x00000800 */
#define RCC_DCKCFGR_PLLSAIDIVQ_4 (0x10UL << RCC_DCKCFGR_PLLSAIDIVQ_Pos) /*!< 0x00001000 */
#define RCC_DCKCFGR_SAI1SRC_Pos (20U)
#define RCC_DCKCFGR_SAI1SRC_Msk (0x3UL << RCC_DCKCFGR_SAI1SRC_Pos) /*!< 0x00300000 */
#define RCC_DCKCFGR_SAI1SRC RCC_DCKCFGR_SAI1SRC_Msk
#define RCC_DCKCFGR_SAI1SRC_0 (0x1UL << RCC_DCKCFGR_SAI1SRC_Pos) /*!< 0x00100000 */
#define RCC_DCKCFGR_SAI1SRC_1 (0x2UL << RCC_DCKCFGR_SAI1SRC_Pos) /*!< 0x00200000 */
#define RCC_DCKCFGR_SAI2SRC_Pos (22U)
#define RCC_DCKCFGR_SAI2SRC_Msk (0x3UL << RCC_DCKCFGR_SAI2SRC_Pos) /*!< 0x00C00000 */
#define RCC_DCKCFGR_SAI2SRC RCC_DCKCFGR_SAI2SRC_Msk
#define RCC_DCKCFGR_SAI2SRC_0 (0x1UL << RCC_DCKCFGR_SAI2SRC_Pos) /*!< 0x00400000 */
#define RCC_DCKCFGR_SAI2SRC_1 (0x2UL << RCC_DCKCFGR_SAI2SRC_Pos) /*!< 0x00800000 */

#define RCC_DCKCFGR_TIMPRE_Pos (24U)
#define RCC_DCKCFGR_TIMPRE_Msk (0x1UL << RCC_DCKCFGR_TIMPRE_Pos) /*!< 0x01000000 */
#define RCC_DCKCFGR_TIMPRE RCC_DCKCFGR_TIMPRE_Msk
#define RCC_DCKCFGR_I2S1SRC_Pos (25U)
#define RCC_DCKCFGR_I2S1SRC_Msk (0x3UL << RCC_DCKCFGR_I2S1SRC_Pos) /*!< 0x06000000 */
#define RCC_DCKCFGR_I2S1SRC RCC_DCKCFGR_I2S1SRC_Msk
#define RCC_DCKCFGR_I2S1SRC_0 (0x1UL << RCC_DCKCFGR_I2S1SRC_Pos) /*!< 0x02000000 */
#define RCC_DCKCFGR_I2S1SRC_1 (0x2UL << RCC_DCKCFGR_I2S1SRC_Pos) /*!< 0x04000000 */

#define RCC_DCKCFGR_I2S2SRC_Pos (27U)
#define RCC_DCKCFGR_I2S2SRC_Msk (0x3UL << RCC_DCKCFGR_I2S2SRC_Pos) /*!< 0x18000000 */
#define RCC_DCKCFGR_I2S2SRC RCC_DCKCFGR_I2S2SRC_Msk
#define RCC_DCKCFGR_I2S2SRC_0 (0x1UL << RCC_DCKCFGR_I2S2SRC_Pos) /*!< 0x08000000 */
#define RCC_DCKCFGR_I2S2SRC_1 (0x2UL << RCC_DCKCFGR_I2S2SRC_Pos) /*!< 0x10000000 */

/********************  Bit definition for RCC_CKGATENR register  ***************/
#define RCC_CKGATENR_AHB2APB1_CKEN_Pos (0U)
#define RCC_CKGATENR_AHB2APB1_CKEN_Msk (0x1UL << RCC_CKGATENR_AHB2APB1_CKEN_Pos) /*!< 0x00000001 */
#define RCC_CKGATENR_AHB2APB1_CKEN RCC_CKGATENR_AHB2APB1_CKEN_Msk
#define RCC_CKGATENR_AHB2APB2_CKEN_Pos (1U)
#define RCC_CKGATENR_AHB2APB2_CKEN_Msk (0x1UL << RCC_CKGATENR_AHB2APB2_CKEN_Pos) /*!< 0x00000002 */
#define RCC_CKGATENR_AHB2APB2_CKEN RCC_CKGATENR_AHB2APB2_CKEN_Msk
#define RCC_CKGATENR_CM4DBG_CKEN_Pos (2U)
#define RCC_CKGATENR_CM4DBG_CKEN_Msk (0x1UL << RCC_CKGATENR_CM4DBG_CKEN_Pos) /*!< 0x00000004 */
#define RCC_CKGATENR_CM4DBG_CKEN RCC_CKGATENR_CM4DBG_CKEN_Msk
#define RCC_CKGATENR_SPARE_CKEN_Pos (3U)
#define RCC_CKGATENR_SPARE_CKEN_Msk (0x1UL << RCC_CKGATENR_SPARE_CKEN_Pos) /*!< 0x00000008 */
#define RCC_CKGATENR_SPARE_CKEN RCC_CKGATENR_SPARE_CKEN_Msk
#define RCC_CKGATENR_SRAM_CKEN_Pos (4U)
#define RCC_CKGATENR_SRAM_CKEN_Msk (0x1UL << RCC_CKGATENR_SRAM_CKEN_Pos) /*!< 0x00000010 */
#define RCC_CKGATENR_SRAM_CKEN RCC_CKGATENR_SRAM_CKEN_Msk
#define RCC_CKGATENR_FLITF_CKEN_Pos (5U)
#define RCC_CKGATENR_FLITF_CKEN_Msk (0x1UL << RCC_CKGATENR_FLITF_CKEN_Pos) /*!< 0x00000020 */
#define RCC_CKGATENR_FLITF_CKEN RCC_CKGATENR_FLITF_CKEN_Msk
#define RCC_CKGATENR_RCC_CKEN_Pos (6U)
#define RCC_CKGATENR_RCC_CKEN_Msk (0x1UL << RCC_CKGATENR_RCC_CKEN_Pos) /*!< 0x00000040 */
#define RCC_CKGATENR_RCC_CKEN RCC_CKGATENR_RCC_CKEN_Msk

/********************  Bit definition for RCC_DCKCFGR2 register  ***************/
#define RCC_DCKCFGR2_FMPI2C1SEL_Pos (22U)
#define RCC_DCKCFGR2_FMPI2C1SEL_Msk (0x3UL << RCC_DCKCFGR2_FMPI2C1SEL_Pos) /*!< 0x00C00000 */
#define RCC_DCKCFGR2_FMPI2C1SEL RCC_DCKCFGR2_FMPI2C1SEL_Msk
#define RCC_DCKCFGR2_FMPI2C1SEL_0 (0x1UL << RCC_DCKCFGR2_FMPI2C1SEL_Pos) /*!< 0x00400000 */
#define RCC_DCKCFGR2_FMPI2C1SEL_1 (0x2UL << RCC_DCKCFGR2_FMPI2C1SEL_Pos) /*!< 0x00800000 */
#define RCC_DCKCFGR2_CECSEL_Pos (26U)
#define RCC_DCKCFGR2_CECSEL_Msk (0x1UL << RCC_DCKCFGR2_CECSEL_Pos) /*!< 0x04000000 */
#define RCC_DCKCFGR2_CECSEL RCC_DCKCFGR2_CECSEL_Msk
#define RCC_DCKCFGR2_CK48MSEL_Pos (27U)
#define RCC_DCKCFGR2_CK48MSEL_Msk (0x1UL << RCC_DCKCFGR2_CK48MSEL_Pos) /*!< 0x08000000 */
#define RCC_DCKCFGR2_CK48MSEL RCC_DCKCFGR2_CK48MSEL_Msk
#define RCC_DCKCFGR2_SDIOSEL_Pos (28U)
#define RCC_DCKCFGR2_SDIOSEL_Msk (0x1UL << RCC_DCKCFGR2_SDIOSEL_Pos) /*!< 0x10000000 */
#define RCC_DCKCFGR2_SDIOSEL RCC_DCKCFGR2_SDIOSEL_Msk
#define RCC_DCKCFGR2_SPDIFRXSEL_Pos (29U)
#define RCC_DCKCFGR2_SPDIFRXSEL_Msk (0x1UL << RCC_DCKCFGR2_SPDIFRXSEL_Pos) /*!< 0x20000000 */
#define RCC_DCKCFGR2_SPDIFRXSEL RCC_DCKCFGR2_SPDIFRXSEL_Msk

/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define USART_SR_PE (0U)
#define USART_SR_FE (1U)
#define USART_SR_NE (2U)
#define USART_SR_ORE (3U)
#define USART_SR_IDLE (4U)
#define USART_SR_RXNE (5U)
#define USART_SR_TC (6U)
#define USART_SR_TXE (7U)
#define USART_SR_LBD (8U)
#define USART_SR_CTS (9U)

/*******************  Bit definition for USART_DR register  *******************/
#define USART_DR_DR (0U)

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_Fraction (0U)
#define USART_BRR_DIV_Mantissa (4U)

/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_SBK (0U)
#define USART_CR1_RWU (1U)
#define USART_CR1_RE (2U)
#define USART_CR1_TE (3U)
#define USART_CR1_IDLEIE (4U)
#define USART_CR1_RXNEIE (5U)
#define USART_CR1_TCIE (6U)
#define USART_CR1_TXEIE (7U)
#define USART_CR1_PEIE (8U)
#define USART_CR1_PS (9U)
#define USART_CR1_PCE (10U)
#define USART_CR1_WAKE (11U)
#define USART_CR1_M (12U)
#define USART_CR1_UE (13U)
#define USART_CR1_OVER8 (15U)

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADD (0U)
#define USART_CR2_LBDL (5U)
#define USART_CR2_LBDIE (6U)
#define USART_CR2_LBCL (8U)
#define USART_CR2_CPHA (9U)
#define USART_CR2_CPOL (10U)
#define USART_CR2_CLKEN (11U)

#define USART_CR2_STOP (12U)

#define USART_CR2_LINEN (14U)

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE (0U)
#define USART_CR3_IREN (1U)
#define USART_CR3_IRLP (2U)
#define USART_CR3_HDSEL (3U)
#define USART_CR3_NACK (4U)
#define USART_CR3_SCEN (5U)
#define USART_CR3_DMAR (6U)
#define USART_CR3_DMAT (7U)
#define USART_CR3_RTSE (8U)
#define USART_CR3_CTSE (9U)
#define USART_CR3_CTSIE (10U)
#define USART_CR3_ONEBIT (11U)

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC (0U)
#define USART_GTPR_PSC_0 (0x01UL << USART_GTPR_PSC) /*!< 0x0001 */
#define USART_GTPR_PSC_1 (0x02UL << USART_GTPR_PSC) /*!< 0x0002 */
#define USART_GTPR_PSC_2 (0x04UL << USART_GTPR_PSC) /*!< 0x0004 */
#define USART_GTPR_PSC_3 (0x08UL << USART_GTPR_PSC) /*!< 0x0008 */
#define USART_GTPR_PSC_4 (0x10UL << USART_GTPR_PSC) /*!< 0x0010 */
#define USART_GTPR_PSC_5 (0x20UL << USART_GTPR_PSC) /*!< 0x0020 */
#define USART_GTPR_PSC_6 (0x40UL << USART_GTPR_PSC) /*!< 0x0040 */
#define USART_GTPR_PSC_7 (0x80UL << USART_GTPR_PSC) /*!< 0x0080 */

#define USART_GTPR_GT (8U)

#endif /* INC_STM32F446XX_H_ */
