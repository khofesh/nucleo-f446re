ENTRY(Reset_Handler)

MEMORY
{
    FLASH   (rx)     : ORIGIN = 0x8000000,         LENGTH = 512K
    /* SRAM1   (rwx)    : ORIGIN = 0x20000000,        LENGTH = 116K
    SRAM2   (rwx)    : ORIGIN = 0x20000000+116K-4, LENGTH=16K */
    RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
}

SECTIONS
{
    .text :
    {
        *(.isr_vector)
        *(.text)
        *(.rodata)
        . = ALIGN(4);
        _etext = .;
    }> FLASH

    .data :
    {
        _sdata = .;
        *(.data)
        . = ALIGN(4);
        _edata = .;
    }> RAM AT> FLASH

    .bss :
    {
        _sbss = .;
        *(.bss)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    }> RAM
}


