# printf

copy the following to `syscalls.c`

```c
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//           Implementation of printf like feature using ARM Cortex M3/M4/ ITM functionality
//           This function will not work for ARM Cortex M0/M0+
//           If you are using Cortex M0, then you can use semihosting feature of openOCD
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//Debug Exception and Monitor Control Register base address
#define DEMCR                   *((volatile uint32_t*) 0xE000EDFCU )

/* ITM register addresses */
#define ITM_STIMULUS_PORT0   	*((volatile uint32_t*) 0xE0000000 )
#define ITM_TRACE_EN          	*((volatile uint32_t*) 0xE0000E00 )

void ITM_SendChar(uint8_t ch)
{

	//Enable TRCENA
	DEMCR |= ( 1 << 24);

	//enable stimulus port 0
	ITM_TRACE_EN |= ( 1 << 0);

	// read FIFO status in bit [0]:
	while(!(ITM_STIMULUS_PORT0 & 1));

	//Write to ITM stimulus port0
	ITM_STIMULUS_PORT0 = ch;
}
```

and inside the `_write` function inside `syscalls.c`, replace `__io_putchar` with `ITM_SendChar`

```c
__attribute__((weak)) int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
//		__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}
```

in debug configuration, enable SWV

![debug configuration](../images/Screenshot_2022-11-01_23-11-55.png)

make sure the least significant bit is set to one (SWV setting during debugging)

![swv settings](../images/Screenshot_2022-11-02_21-40-44.png)

result

![printf](../images/Screenshot_2022-11-01_23-13-33.png)

# Fault handling

## system control block

![system control block](../images/Screenshot_2022-11-01_22-29-49.png)

you'll get the address of SHCSR `0xE000ED24` and you put it inside code like the following:

```shell
uint32_t *pSHCSR = (uint32_t*)0xE000ED24;
```

## fault status registers

when a fault happens, inside the fault handler, you can check a couple of fault status and address information registers to get more details about the fault and the instruction address at which the fault happened.

![fault status registers](../images/Screenshot_2022-11-01_22-27-16.png)

example usageFault --> register address (UFSR) is 0xE000ED2A

```c
uint32_t *pUFSR = (uint32_t*)0xE000ED2A;
printf("exception: UsageFault\n");
printf("USFR = %lx\n", (*pUFSR) & 0xFFFF);
```

the meaning of USFR

![meaning](../images/Screenshot_2022-11-01_23-38-27.png)

## System Handler Control and State Register

![System Handler Control and State Register](../images/Screenshot_2022-11-01_22-27-48.png)

# SVC exception

![SVC](../images/Screenshot_2022-11-02_21-37-00.png)

## SVCMath

remember to which register we store argument/result

![Core registers and AAPCS usage](../images/Screenshot_2022-10-28_22-05-24.png)

# PendSV exception

# implementation of task scheduler
