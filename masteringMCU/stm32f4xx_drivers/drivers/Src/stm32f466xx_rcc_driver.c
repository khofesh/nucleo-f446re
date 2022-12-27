/*
 * stm32f466xx_rcc_driver.c
 *
 *  Created on: Dec 27, 2022
 *      Author: fahmad
 */

#include "stm32f466xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APBx_PreScaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value()
{
    uint32_t pclk1;
    uint32_t SystemClk;

    uint8_t clksrc;
    uint8_t temp;
    uint8_t ahbp;
    uint8_t apb1p;

    /*
        see RM0390-*.pdf page 133

        Bits 3:2 SWS[1:0]: System clock switch status
        Set and cleared by hardware to indicate which clock source is used as the system clock.
        00: HSI oscillator used as the system clock
        01: HSE oscillator used as the system clock
        10: PLL used as the system clock
        11: PLL_R used as the system clock
    */
    clksrc = ((RCC->CFGR >> RCC_CFGR_SWS_Pos) & 0x3UL);

    if (clksrc == 0)
    {
        SystemClk = 16000000;
    }
    else if (clksrc == 1)
    {
        SystemClk = 8000000;
    }
    else if (clksrc == 2)
    {
        SystemClk = RCC_GetPLLOutputClock();
    }

    temp = ((RCC->CFGR >> RCC_CFGR_HPRE_Pos) & 0xFUL);

    if (temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    temp = ((RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x7UL);

    if (temp < 4)
    {
        apb1p = 1;
    }
    else
    {
        apb1p = APBx_PreScaler[temp - 4];
    }

    pclk1 = (SystemClk / ahbp) / apb1p;

    return pclk1;
}

uint32_t RCC_GetPCLK2Value()
{
    uint32_t pclk1;
    uint32_t SystemClk;

    uint8_t clksrc;
    uint8_t temp;
    uint8_t ahbp;
    uint8_t apb2p;

    /*
        see RM0390-*.pdf page 133

        Bits 3:2 SWS[1:0]: System clock switch status
        Set and cleared by hardware to indicate which clock source is used as the system clock.
        00: HSI oscillator used as the system clock
        01: HSE oscillator used as the system clock
        10: PLL used as the system clock
        11: PLL_R used as the system clock
    */
    clksrc = ((RCC->CFGR >> RCC_CFGR_SWS_Pos) & 0x3UL);

    if (clksrc == 0)
    {
        SystemClk = 16000000;
    }
    else if (clksrc == 1)
    {
        SystemClk = 8000000;
    }
    else if (clksrc == 2)
    {
        SystemClk = RCC_GetPLLOutputClock();
    }

    temp = ((RCC->CFGR >> RCC_CFGR_HPRE_Pos) & 0xFUL);

    if (temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    temp = ((RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x7UL);

    if (temp < 4)
    {
        apb2p = 1;
    }
    else
    {
        apb2p = APBx_PreScaler[temp - 4];
    }

    pclk1 = (SystemClk / ahbp) / apb2p;

    return pclk1;
}

uint32_t RCC_GetPLLOutputClock()
{
    return 0;
}
