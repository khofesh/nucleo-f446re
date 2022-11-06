#include <stdint.h>

#define SRAM_START 0x20000000U
#define SRAM_SIZE (128 * 1024) // 128KB
#define SRAM_END ((SRAM_START) + (SRAM_SIZE))

#define STACK_START SRAM_END

void Reset_Handler();
void NMI_Handler() __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler() __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler() __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler() __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler() __attribute__((weak, alias("Default_Handler")));

void SVC_Handler() __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler() __attribute__((weak, alias("Default_Handler")));

void PendSV_Handler() __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler() __attribute__((weak, alias("Default_Handler")));
/*
 * Window Watchdog interrupt
 */
void WWDG_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * PVD through EXTI line detection interrupt
 */
void PVD_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * Tamper and TimeStamp interrupts through the EXTI line
 */
void TAMP_STAMP_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * RTC Wakeup interrupt through the EXTI line
 */
void RTC_WKUP_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * Flash global interrupt
 */
void FLASH_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * RCC global interrupt
 */
void RCC_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * EXTI Line0 interrupt
 */
void EXTI0_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * EXTI Line1 interrupt
 */
void EXTI1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * EXTI Line2 interrupt
 */
void EXTI2_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * EXTI Line3 interrupt
 */
void EXTI3_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * EXTI Line4 interrupt
 */
void EXTI4_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA1 Stream0 global interrupt
 */
void DMA1_Stream0_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA1 Stream1 global interrupt
 */
void DMA1_Stream1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA1 Stream2 global interrupt
 */
void DMA1_Stream2_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA1 Stream3 global interrupt
 */
void DMA1_Stream3_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA1 Stream4 global interrupt
 */
void DMA1_Stream4_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA1 Stream5 global interrupt
 */
void DMA1_Stream5_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA1 Stream6 global interrupt
 */
void DMA1_Stream6_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * ADC1 global interrupt
 */
void ADC_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * CAN1 TX interrupts
 */
void CAN1_TX_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * CAN1 RX0 interrupts
 */
void CAN1_RX0_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * CAN1 RX1 interrupts
 */
void CAN1_RX1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * CAN1 SCE interrupt
 */
void CAN1_SCE_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * EXTI Line[9:5] interrupts
 */
void EXTI9_5_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM1 Break interrupt and TIM9 global interrupt
 */
void TIM1_BRK_TIM9_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM1 Update interrupt and TIM10 global interrupt
 */
void TIM1_UP_TIM10_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
 */
void TIM1_TRG_COM_TIM11_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM1 Capture Compare interrupt
 */
void TIM1_CC_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM2 global interrupt
 */
void TIM2_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM3 global interrupt
 */
void TIM3_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM4 global interrupt
 */
void TIM4_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * I2C1 event interrupt
 */
void I2C1_EV_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * I2C1 error interrupt
 */
void I2C1_ER_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * I2C2 event interrupt
 */
void I2C2_EV_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * I2C2 error interrupt
 */
void I2C2_ER_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * SPI1 global interrupt
 */
void SPI1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * SPI2 global interrupt
 */
void SPI2_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USART1 global interrupt
 */
void USART1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USART2 global interrupt
 */
void USART2_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USART3 global interrupt
 */
void USART3_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * EXTI Line[15:10] interrupts
 */
void EXTI15_10_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * RTC Alarms (A and B) through EXTI line interrupt
 */
void RTC_Alarm_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USB On-The-Go FS Wakeup through EXTI line interrupt
 */
void OTG_FS_WKUP_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM8 Break interrupt and TIM12 global interrupt
 */
void TIM8_BRK_TIM12_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM8 Update interrupt and TIM13 global interrupt
 */
void TIM8_UP_TIM13_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM8 Trigger and Commutation interrupts and TIM14 global interrupt
 */
void TIM8_TRG_COM_TIM14_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM8 Capture Compare interrupt
 */
void TIM8_CC_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA1 Stream7 global interrupt
 */
void DMA1_Stream7_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * FMC global interrupt
 */
void FMC_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * SDIO global interrupt
 */
void SDIO_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM5 global interrupt
 */
void TIM5_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * SPI3 global interrupt
 */
void SPI3_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * UART4 global interrupt
 */
void UART4_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * UART5 global interrupt
 */
void UART5_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM6 global interrupt, DAC1 and DAC2 underrun error interrupt
 */
void TIM6_DAC_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * TIM7 global interrupt
 */
void TIM7_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA2 Stream0 global interrupt
 */
void DMA2_Stream0_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA2 Stream1 global interrupt
 */
void DMA2_Stream1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA2 Stream2 global interrupt
 */
void DMA2_Stream2_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA2 Stream3 global interrupt
 */
void DMA2_Stream3_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA2 Stream4 global interrupt
 */
void DMA2_Stream4_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * CAN2 TX interrupts
 */
void CAN2_TX_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * CAN2 RX0 interrupts
 */
void CAN2_RX0_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * CAN2 RX1 interrupts
 */
void CAN2_RX1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * CAN2 SCE interrupt
 */
void CAN2_SCE_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USB On The Go FS global interrupt
 */
void OTG_FS_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA2 Stream5 global interrupt
 */
void DMA2_Stream5_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA2 Stream6 global interrupt
 */
void DMA2_Stream6_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DMA2 Stream7 global interrupt
 */
void DMA2_Stream7_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USART6 global interrupt
 */
void USART6_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * I2C3 event interrupt
 */
void I2C3_EV_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * I2C3 error interrupt
 */
void I2C3_ER_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USB On The Go HS End Point 1 Out
 */
void OTG_HS_EP1_OUT_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USB On The Go HS End Point 1 In
 */
void OTG_HS_EP1_IN_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USB On The Go HS Wakeup
 */
void OTG_HS_WKUP_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * USB On The Go HS global interrupt
 */
void OTG_HS_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * DCMI global interrupt
 */
void DCMI_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * Floating point unit interrupt
 */
void FPU_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * SPI 4 global interrupt
 */
void SPI4_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * SAI1 global interrupt
 */
void SAI1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * SAI2 global interrupt
 */
void SAI2_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * QuadSPI global interrupt
 */
void QuadSPI_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * HDMI-CEC global interrupt
 */
void HDMI_CEC_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * SPDIF-Rx global interrupt
 */
void SPDIF_Rx_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * FMPI2C1 event interrupt
 */
void FMPI2C1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
/*
 * FMPI2C1 error interrupt
 */
void FMPI2C1_error_IRQHandler() __attribute__((weak, alias("Default_Handler")));

/**
 * @brief https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/Common-Variable-Attributes.html#Common-Variable-Attributes
 * reserved should be 0
 */
uint32_t vectors[] __attribute__((section(".isr_vector"))) = {
    STACK_START,
    (uint32_t)&Reset_Handler,
    (uint32_t)&NMI_Handler,
    (uint32_t)&HardFault_Handler,
    (uint32_t)&MemManage_Handler,
    (uint32_t)&BusFault_Handler,
    (uint32_t)&UsageFault_Handler,
    0,
    0,
    0,
    0,
    (uint32_t)&SVC_Handler,
    (uint32_t)&DebugMon_Handler,
    0,
    (uint32_t)&PendSV_Handler,
    (uint32_t)&SysTick_Handler,
    (uint32_t)&WWDG_IRQHandler,
    (uint32_t)&PVD_IRQHandler,
    (uint32_t)&TAMP_STAMP_IRQHandler,
    (uint32_t)&RTC_WKUP_IRQHandler,
    (uint32_t)&FLASH_IRQHandler,
    (uint32_t)&RCC_IRQHandler,
    (uint32_t)&EXTI0_IRQHandler,
    (uint32_t)&EXTI1_IRQHandler,
    (uint32_t)&EXTI2_IRQHandler,
    (uint32_t)&EXTI3_IRQHandler,
    (uint32_t)&EXTI4_IRQHandler,
    (uint32_t)&DMA1_Stream0_IRQHandler,
    (uint32_t)&DMA1_Stream1_IRQHandler,
    (uint32_t)&DMA1_Stream2_IRQHandler,
    (uint32_t)&DMA1_Stream3_IRQHandler,
    (uint32_t)&DMA1_Stream4_IRQHandler,
    (uint32_t)&DMA1_Stream5_IRQHandler,
    (uint32_t)&DMA1_Stream6_IRQHandler,
    (uint32_t)&ADC_IRQHandler,
    (uint32_t)&CAN1_TX_IRQHandler,
    (uint32_t)&CAN1_RX0_IRQHandler,
    (uint32_t)&CAN1_RX1_IRQHandler,
    (uint32_t)&CAN1_SCE_IRQHandler,
    (uint32_t)&EXTI9_5_IRQHandler,
    (uint32_t)&TIM1_BRK_TIM9_IRQHandler,
    (uint32_t)&TIM1_UP_TIM10_IRQHandler,
    (uint32_t)&TIM1_TRG_COM_TIM11_IRQHandler,
    (uint32_t)&TIM1_CC_IRQHandler,
    (uint32_t)&TIM2_IRQHandler,
    (uint32_t)&TIM3_IRQHandler,
    (uint32_t)&TIM4_IRQHandler,
    (uint32_t)&I2C1_EV_IRQHandler,
    (uint32_t)&I2C1_ER_IRQHandler,
    (uint32_t)&I2C2_EV_IRQHandler,
    (uint32_t)&I2C2_ER_IRQHandler,
    (uint32_t)&SPI1_IRQHandler,
    (uint32_t)&SPI2_IRQHandler,
    (uint32_t)&USART1_IRQHandler,
    (uint32_t)&USART2_IRQHandler,
    (uint32_t)&USART3_IRQHandler,
    (uint32_t)&EXTI15_10_IRQHandler,
    (uint32_t)&RTC_Alarm_IRQHandler,
    (uint32_t)&OTG_FS_WKUP_IRQHandler,
    (uint32_t)&TIM8_BRK_TIM12_IRQHandler,
    (uint32_t)&TIM8_UP_TIM13_IRQHandler,
    (uint32_t)&TIM8_TRG_COM_TIM14_IRQHandler,
    (uint32_t)&TIM8_CC_IRQHandler,
    (uint32_t)&DMA1_Stream7_IRQHandler,
    (uint32_t)&FMC_IRQHandler,
    (uint32_t)&SDIO_IRQHandler,
    (uint32_t)&TIM5_IRQHandler,
    (uint32_t)&SPI3_IRQHandler,
    (uint32_t)&UART4_IRQHandler,
    (uint32_t)&UART5_IRQHandler,
    (uint32_t)&TIM6_DAC_IRQHandler,
    (uint32_t)&TIM7_IRQHandler,
    (uint32_t)&DMA2_Stream0_IRQHandler,
    (uint32_t)&DMA2_Stream1_IRQHandler,
    (uint32_t)&DMA2_Stream2_IRQHandler,
    (uint32_t)&DMA2_Stream3_IRQHandler,
    (uint32_t)&DMA2_Stream4_IRQHandler,
    0,
    0,
    (uint32_t)&CAN2_TX_IRQHandler,
    (uint32_t)&CAN2_RX0_IRQHandler,
    (uint32_t)&CAN2_RX1_IRQHandler,
    (uint32_t)&CAN2_SCE_IRQHandler,
    (uint32_t)&OTG_FS_IRQHandler,
    (uint32_t)&DMA2_Stream5_IRQHandler,
    (uint32_t)&DMA2_Stream6_IRQHandler,
    (uint32_t)&DMA2_Stream7_IRQHandler,
    (uint32_t)&USART6_IRQHandler,
    (uint32_t)&I2C3_EV_IRQHandler,
    (uint32_t)&I2C3_ER_IRQHandler,
    (uint32_t)&OTG_HS_EP1_OUT_IRQHandler,
    (uint32_t)&OTG_HS_EP1_IN_IRQHandler,
    (uint32_t)&OTG_HS_WKUP_IRQHandler,
    (uint32_t)&OTG_HS_IRQHandler,
    (uint32_t)&DCMI_IRQHandler,
    0,
    0,
    (uint32_t)&FPU_IRQHandler,
    0,
    0,
    (uint32_t)&SPI4_IRQHandler,
    0,
    0,
    (uint32_t)&SAI1_IRQHandler,
    0,
    0,
    0,
    (uint32_t)&SAI2_IRQHandler,
    (uint32_t)&QuadSPI_IRQHandler,
    (uint32_t)&HDMI_CEC_IRQHandler,
    (uint32_t)&SPDIF_Rx_IRQHandler,
    (uint32_t)&FMPI2C1_IRQHandler,
    (uint32_t)&FMPI2C1_error_IRQHandler,
};

void Reset_Handler()
{
    // copy .data section to SRAM

    // initialize the .bss section to zero in SRAM

    // call init function of std. library

    // call main()
}

void Default_Handler()
{
    while (1)
    {
        /* code */
    }
}