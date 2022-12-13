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

/**
 * @brief ARM Cortex Mx processor NVIC ICERx register addresses
 */
#define NVIC_ICER0 ((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1 ((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2 ((__vo uint32_t *)0xE000E18C)
#define NVIC_ICER3 ((__vo uint32_t *)0xE000E190)
#define NVIC_ICER4 ((__vo uint32_t *)0xE000E194)
#define NVIC_ICER5 ((__vo uint32_t *)0xE000E198)
#define NVIC_ICER6 ((__vo uint32_t *)0xE000E19C)

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
#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x3800)

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

#endif /* INC_STM32F446XX_H_ */
