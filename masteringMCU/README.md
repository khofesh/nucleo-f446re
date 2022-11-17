# Mastering MCU

# components

https://www.eevblog.com/forum/beginners/which-logic-analyzer-for-beginners/

# OpenOCD Debugger and Semi-hosting

Set the linker arguments

```shell
-specs=rdimon.specs -lc -lrdimon
```

Add semi-hosting run command

```
monitor arm semihosting enable
```

Add the below function call to main.c

```c
extern void initialise_monitor_handles(void);
initialise_monitor_handles();
```

# memory map

see `rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf`
![memory map](../images/Screenshot_2022-11-18_00-01-23.png)

see `stm32f446re.pdf`
![STM32F446xC/E block diagram](../images/Screenshot_2022-11-18_00-20-58.png)

important memory:

- base address of AHB1 BUS peripherals \
  start address: 0x4002 0000 \
  end address: 0x4007 FFFF
- base address of GPIOA registers:
- base address of RCC engine registers of the MCUE
- base address of APB1 peripherall registers:
- base address of FLASH memory
- base address of SRAM2
- base address of ADC registers
