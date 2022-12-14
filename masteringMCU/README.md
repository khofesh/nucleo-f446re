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
- base address of GPIOA registers: \
  0x4002 0000
- base address of RCC engine registers of the MCU \
  0x4002 3800
- base address of APB1 peripherall registers: \
  0x4000 0000
- base address of FLASH memory \
  0x8000000
- base address of SRAM2 \
  SRAM1 starts from 0x2000_0000 \
  size of SRAM1 = X bytes \
  base address of SRAM2 = 0x2000_0000 + X \
  base address of SRAM 2 = base address of sram1 + size of SRAM1
- base address of ADC registers \
  0x4001 2000

# MCU bus interfaces

## ICode memory interface

Instruction fetches from Code memory space, 0x00000000 to 0x1FFFFFFC, are performed over the 32-bit
AHB-Lite bus.

The Debugger cannot access this interface. All fetches are word-wide. The number of instructions
fetched per word depends on the code running and the alignment of the code in memory.

## DCode memory interface

Data and debug accesses to Code memory space, 0x00000000 to 0x1FFFFFFF, are performed over the
32-bit AHB-Lite bus.

## System interface

Instruction fetches and data and debug accesses to address ranges 0x20000000 to 0xDFFFFFFF and
0xE0100000 to 0xFFFFFFFF are performed over the 32-bit AHB-Lite bus.
For simultaneous accesses to the 32-bit AHB-Lite bus, the arbitration order in decreasing priority is:

• Data accesses. \
• Instruction and vector fetches. \
• Debug. \

The system bus interface contains control logic to handle unaligned accesses, FPB remapped accesses,
bit-band accesses, and pipelined instruction fetches.

# Clocking system

Three different clock sources can be used to drive the system clock (SYSCLK):

- HSI (high speec internal) oscillator clock or RC oscillator (internal to the MCU)
- HSE (high speec external) oscillator clock or crystal oscillator (external to the MCU) \
  8 MHz
- Two main PLL (Phase Locked Loop) clocks (internal to the MCU)

references: `rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf` page 117

# Clock tree

## sigrok, pulseview, sigrok-cli

I'm using sparkfun USB Logic Analyzer, because saleae is very expensive.

https://learn.sparkfun.com/tutorials/using-the-usb-logic-analyzer-with-sigrok-pulseview/all

install necessary softwares

```shell
sudo dnf install sigrok-cli sigrok-firmware-fx2lafw pulseview
```

detect hardware

```shell
[fahmad@ryzen ~]$  lsusb | grep Saleae
Bus 005 Device 011: ID 0925:3881 Lakeview Research Saleae Logic
```

open pulseview and select `fx2lafw (generic driver for FX2 based LAs)`

![pulseview](../images/Screenshot_2022-11-18_21-44-02.png)

### sigrok URLs:

- https://sigrok.org/wiki/Downloads
- https://sigrok.org/wiki/PulseView#Download
- https://sigrok.org/doc/pulseview/0.4.1/manual.html

## HSI measurement

usb logic analyzer \
connect ch0 to D7/PA8 \
and GND to GND

<img src="../images/photo1668786759.jpeg" alt="usb logic connected" width="500"/>

debug `006HSIMeasurement` and hit `run` on pulseview

<img src="../images/Screenshot_2022-11-18_22-40-15.png" alt="analysis one 4MHz" width="500"/>

in `main.c` of `006HSIMeasurement`, comment out the following lines

```c
	// configure MCO1 prescaler
//	*pRccCfgrReg |= (1 << 25);
//	*pRccCfgrReg |= (1 << 26);
```

you'll get 8MHz

<img src="../images/Screenshot_2022-11-18_23-11-17.png" alt="analysis two 8MHz" width="500"/>

> The sample rate you choose must at least be twice that of the highest frequency you want to capture - ideally 3 to 5 times as much so that you have some margin. That way, a jittering signal won’t ruin your measurements.

## HSE measurement

# vector table

check reset handler address

![reset handler](../images/Screenshot_2022-11-28_12-19-25.png)

# interrupts

nucleo f446re design for the blue button

![user blue button](../images/Screenshot_2022-11-28_13-09-00.png)

External interrupt/event GPIO mapping
![External interrupt/event GPIO mapping](../images/Screenshot_2022-11-28_13-11-00.png)

External interrupt/event controller block diagram
![External interrupt/event controller block diagram](../images/Screenshot_2022-11-28_14-25-55.png)

## how does a button issue interrupt to the processor in STM32 ?

1. The button is connected to a GPIO pin of the microcontroller
2. The GPIO pin should be configured to input mode
3. The link between a GPIO port and the relevant EXTI line must be established using the SYSCFG_EXTICRX register.
4. Configure the trigger detection (falling/rising/both) for relevant EX line (This is done via EXTI controller registers )
5. Implement the handler to service the interrupt.

# GPIO (General Purpose Input Output)

open drain with internal pull up and open drain with external pull up

![pull up](../images/Screenshot_2022-11-28_16-18-56.png)

# GPIO programming structure

## GPIO port mode register (GPIOx_MODER) (x = A..H)

Address offset: 0x00

### GPIO functional description

Basic structure of a 5 V tolerant I/O port bit

![Basic structure of a 5 V tolerant I/O port bit](../images/Screenshot_2022-11-28_18-25-44.png)

## GPIO port output speed register (GPIOx_OSPEEDR) (x = A..H)

Address offset: 0x08

![I/O AC characteristics](../images/Screenshot_2022-11-28_19-33-18.png)

check `stm32f446re.pdf` for the image above

## GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x = A..H)

Address offset: 0x0C

## GPIO port input data register (GPIOx_IDR) (x = A..H)

Address offset: 0x10

## GPIO port output data register (GPIOx_ODR) (x = A..H)

Address offset: 0x14

# alternate functionality

alternate function for GPIO port 'C'

![alternate function](../images/Screenshot_2022-11-28_19-47-47.png)

## GPIO alternate function low register (GPIOx_AFRL) (x = A..H)

Address offset: 0x20

![](../images/Screenshot_2022-11-28_19-56-38.png)

## GPIO alternate function high register (GPIOx_AFRH) (x = A..H)

Address offset: 0x24

![](../images/Screenshot_2022-11-28_19-57-16.png)

## example

### 1

find out the alternate functionally mode (AFx) and AFR (alternate function register) settings
to make

- PA0 as UART4_TX
- PA1 as UART4_RX
- PA10 as TIM1_CH3

### 2

Lets say you have to configure PB8 pin as I2C Functionality ? Which Alternate function Register will you modify ? What are the bit positions needs to modified with what value ?

GPIOB_AFRH (high register) \
Bit position: 0, 1, 2, 3 \
value: 4

### 3

For STM32VGx to configure PC6 as TIM3 functionality, which alternate function register will modify? What are the bit positions that need to modified with what value?

GPIOC_AFRL (log register) \
bit position: 24, 25, 26, 27 \
value: 2

# GPIO peripheral clock (RCC)

## RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)

Address offset: 0x30 \
Reset value: 0x0000 0000 \
Access: no wait state, word, half-word and byte access.

# GPIO pin interrupt configution

1. pin must be in input configuration
2. configure the edge trigger (RT, FT, RFT)
3. enable interrupt delivery from peripheral to the processor (on peripheral side)
4. identify the IRQ number on which the processor accepts the interrupt from that pin
5. configure the IRQ priority for the identified IRQ number (processor side)
6. enable interrupt reception on that IRQ number (processor side)
7. implement IRQ handler
