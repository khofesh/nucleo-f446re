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
 * @brief initialize SPI
 * see RM0390-*.pdf page 886
 * "SPI control register 1 (SPI_CR1)"
 * @param pSPIHandle
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    // peripheral clock enable
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // configure SPI_CR1 register
    uint32_t tempReg = 0;

    // 1. configure the device mode
    tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // 2. configure the bus config
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        /* BIDIMODE should be cleared */
        tempReg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        /* BIDIMODE should be set */
        tempReg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        /*
            BIDIMODE should be cleared
            rxonly bit must be set
         */
        tempReg &= ~(1 << SPI_CR1_BIDIMODE);
        tempReg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. configure the SPI serial clock speed (baud rate)
    tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR0;

    // 4. configure the DFF
    tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    // 5. configure the CPOL
    tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    // 6. configure the CPHA
    tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    // 7. ssm
    tempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

    pSPIHandle->pSPIx->CR1 = tempReg;
}

/**
 * @brief
 *
 * @param pSPIx
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
}

bool SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
    if (pSPIx->SR & flagName)
    {
        return FLAG_SET;
    }

    return FLAG_RESET;
}

/**
 * @brief send data
 * blocking call
 * @param pSPIx
 * @param pTxBuffer
 * @param Len
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1. wait until TXE is set
        while ((SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)) == FLAG_RESET)
            ;

        // 2. check the DFF bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            /* 16 bit DFF */
            // 1. load the data into the DR
            pSPIx->DR = *((uint16_t *)pTxBuffer);

            // twice because you sent two bytes of data
            Len--;
            Len--;

            (uint16_t *)pTxBuffer++;
        }
        else
        {
            /* 8 bit DFF */
            pSPIx->DR = *(pTxBuffer);

            Len--;
            pTxBuffer++;
        }
    }
}

/**
 * @brief
 *
 * @param pSPIx
 * @param pRxBuffer
 * @param Len
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1. wait until RXNE is set
        while ((SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)) == FLAG_RESET)
            ;

        // 2. check the DFF bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            /* 16 bit DFF */
            // 1. load the data from DR to Rxbuffer address
            *((uint16_t *)pRxBuffer) = pSPIx->DR;

            // twice because you sent two bytes of data
            Len--;
            Len--;

            (uint16_t *)pRxBuffer++;
        }
        else
        {
            /* 8 bit DFF */
            *(pRxBuffer) = pSPIx->DR;

            Len--;
            pRxBuffer++;
        }
    }
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

/**
 * @brief SPI_PeripheralControl
 * enable/disable SPI_CR1_SPE
 *
 * @param pSPIx
 * @param EnOrDi
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/**
 * @brief SPI_SSIConfig
 *
 * @param pSPIx
 * @param EnOrDi
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

/**
 * @brief SPI_SSOEConfig
 *
 * @param pSPIx
 * @param EnOrDi
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}
