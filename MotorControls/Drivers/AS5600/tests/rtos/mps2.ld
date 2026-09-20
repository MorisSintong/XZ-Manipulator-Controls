ENTRY(Reset_Handler)

MEMORY
{
    CODE (rx) : ORIGIN = 0x00000000, LENGTH = 256K
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

SECTIONS
{
    .vectors : ALIGN(256) { KEEP(*(.vectors)) } > CODE
    .text : ALIGN(4) { *(.text*) *(.rodata*) } > CODE
    .ARM.exidx : ALIGN(4) { *(.ARM.exidx*) } > CODE
    .data : ALIGN(4)
    {
        __data_start = .;
        *(.data*)
        . = ALIGN(4);
        __data_end = .;
    } > RAM AT > CODE
    __data_load = LOADADDR(.data);
    .bss (NOLOAD) : ALIGN(8)
    {
        __bss_start = .;
        *(.bss*) *(COMMON)
        . = ALIGN(8);
        __bss_end = .;
    } > RAM
    __stack_top = ORIGIN(RAM) + LENGTH(RAM);
    ASSERT(__bss_end + 4096 <= __stack_top, "RAM overlaps exception stack")
    /DISCARD/ : { *(.comment) *(.note*) }
}
