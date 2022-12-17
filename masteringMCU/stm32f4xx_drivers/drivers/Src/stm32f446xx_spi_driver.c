/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Dec 17, 2022
 *      Author: fahmad
 */

#include "stm32f446xx_spi_driver.h"

/**
 * @brief
 *
 * @param pSPIx
 * @param EnOrDi
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {

        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }
}

/**
 * @brief
 *
 * @param pSPIHandle
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
}

/**
 * @brief
 *
 * @param pSPIx
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
}

/**
 * @brief
 *
 * @param pSPIx
 * @param pTxBuffer
 * @param Len
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
}

/**
 * @brief
 *
 * @param pSPIx
 * @param pTxBuffer
 * @param Len
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
}

/**
 * @brief
 *
 * @param IRQNumber
 * @param EnOrDi
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
}

/**
 * @brief
 *
 * @param IRQNumber
 * @param IRQPriority
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
}

/**
 * @brief
 *
 * @param pHandle
 */
void SPI_IRQHandler(SPI_Handle_t *pHandle)
{
}