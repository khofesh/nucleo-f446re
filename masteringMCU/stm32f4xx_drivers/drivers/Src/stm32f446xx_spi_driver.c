/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Dec 17, 2022
 *      Author: fahmad
 */

#include "stm32f446xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
 * @brief SPI_SendDataIT
 * send data with interrupt mode
 * @param pSPIHandle
 * @param pTxBuffer
 * @param Len
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX)
    {
        // 1 . Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;
        // 2.  Mark the SPI state as busy in transmission so that
        //     no other code can take over same SPI peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }

    return state;
}

/**
 * @brief SPI_ReceiveDataIT
 * receive data with interrupt mode
 * @param pSPIHandle
 * @param pRxBuffer
 * @param Len
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX)
    {
        // 1. save the Tx buffer address and len information in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        // 2. mark the SPI state as busy in transmission so that
        // no other code can take over same SPI peripheral until transmission is over
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        // 3. enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        // see RM0390*.pdf page 888 - SPI control register 2 (SPI_CR2)
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

        // 4. data transmission will be handled by the ISR code
    }

    return state;
}

/**
 * @brief
 *
 * @param IRQNumber
 * @param EnOrDi
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            /* program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            /* program ISER1 register */
            *NVIC_ISER1 |= (1 << IRQNumber % 32);
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* program ISER2 register */
            *NVIC_ISER2 |= (1 << IRQNumber % 64);
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            /* program ICER0 register */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            /* program ICER1 register */
            *NVIC_ICER1 |= (1 << IRQNumber % 32);
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            /* program ICER2 register */
            *NVIC_ICER2 |= (1 << IRQNumber % 64);
        }
    }
}

/**
 * @brief
 *
 * @param IRQNumber
 * @param IRQPriority
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // 1. find out the IPR register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << (8 * shift_amount));
}

/**
 * @brief SPI_IRQHandler
 *
 * @param pHandle
 */
void SPI_IRQHandler(SPI_Handle_t *pHandle)
{
    uint8_t temp1, temp2;

    // 1. check TXE
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if (temp1 & temp2)
    {
        // handle TXE
        spi_txe_interrupt_handle(pHandle);
    }

    // 2. check RXE
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if (temp1 & temp2)
    {
        // handle RXNE
        spi_rxne_interrupt_handle(pHandle);
    }

    // check ovr flag
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if (temp1 & temp2)
    {
        // handle OVR error
        spi_ovr_err_interrupt_handle(pHandle);
    }
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

/**
 * @brief spi_txe_interrupt_handle
 *
 * @param pSPIHandle
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    // check the DFF bit in CR1
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        /* 16 bit DFF */
        // 1. load the data into the DR
        pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);

        // twice because you sent two bytes of data
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;

        (uint16_t *)pSPIHandle->pTxBuffer++;
    }
    else
    {
        /* 8 bit DFF */
        pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);

        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if (!pSPIHandle->TxLen)
    {
        /* txlen is zero, close the spi communication and inform
        the application that TX is over */

        // prevents interrupts from setting of txe flag
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

/**
 * @brief spi_rxne_interrupt_handle
 *
 * @param pSPIHandle
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    // check the DFF bit in CR1
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        /* 16 bit DFF */
        *((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;

        pSPIHandle->RxLen -= 2;
        pSPIHandle->pRxBuffer--;
        pSPIHandle->pRxBuffer--;
    }
    else
    {
        /* 8 bit DFF */
        *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;

        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer--;
    }

    if (!pSPIHandle->RxLen)
    {
        // reception is complete
        // turn off the rxneie interrupt
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

/**
 * @brief spi_ovr_err_interrupt_handle
 *
 * @param pSPIHandle
 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;
    // clear the ovr flag
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }

    (void)temp;

    // inform the application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;

    temp = pSPIHandle->pSPIx->DR;
    temp = pSPIHandle->pSPIx->SR;
    (void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent)
{
}
