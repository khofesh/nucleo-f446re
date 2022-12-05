/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Dec 5, 2022
 *      Author: fahmad
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/**
 * this is a configuration structure for a GPIO pin
 */
typedef struct
{
	/*!< Specifies the GPIO pins to be configured.
		This parameter can be any value of @ref GPIO_pins_define */
	uint32_t GPIO_PinNumber;
	/*!< Specifies the operating mode for the selected pins.
		This parameter can be a value of @ref GPIO_mode_define */
	uint32_t GPIO_PinMode;
	/*!< Specifies the speed for the selected pins.
		This parameter can be a value of @ref GPIO_speed_define */
	uint32_t GPIO_PinSpeed;
	/*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
		This parameter can be a value of @ref GPIO_pull_define */
	uint32_t GPIO_PinPuPdControl;

	uint32_t GPIO_PinOPType;
	/*!< Peripheral to be connected to the selected pins.
		This parameter can be a value of @ref GPIO_Alternate_function_selection */
	uint32_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/**
 * handle structure for a GPIO pin
 */
typedef struct
{
	// this holds the base address of the GPIO port to which the pin belongs
	GPIO_RegDef_t *pGPIOx;
	// this holds GPIO pin configuration settings
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/**
 * @brief GPIO pin possible modes
 * RM0390-*.pdf page 187
 * 00: Input (reset state)
 * 01: General purpose output mode
 * 10: Alternate function mode
 * 11: Analog mode
 */
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
// IT - input falling edge
#define GPIO_MODE_IT_FT 4
// input rising edge
#define GPIO_MODE_IT_RT 5
// rising edge-falling edge
#define GPIO_MODE_IT_RFT 6

/**
 * @brief GPIO pin possible output
 * These bits are written by software to configure the output type of the I/O port.
 * 0: Output push-pull (reset state)
 * 1: Output open-drain
 */

/*****************************************
 * APIs supported by this driver         *
 *****************************************
 */

/**
 * Peripheral clock setup
 * EnOrDi - enable or disable
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint32_t EnOrDi);

/**
 * init and de-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * data read and write
 */
uint32_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint32_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint32_t PinNumber, uint32_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint32_t PinNumber);

/**
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint32_t EnOrDi);
void GPIO_IRQHandler(uint32_t PinNumber);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
