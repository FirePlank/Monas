/* nRF52833 (BBC micro:bit v2): 512 KB flash, 128 KB RAM.
 * RAM is also mapped at 0x00800000 on the I-code bus ("code RAM"), where the CPU
 * fetches instructions with no wait states and without competing with data accesses.
 * The search's hot functions are copied there at boot; from flash they would cost up
 * to three wait states per instruction-cache miss. */
MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x00000000, LENGTH = 512K
  CODERAM (rx)  : ORIGIN = 0x00800000, LENGTH = 128K
  RAM     (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

ENTRY(Reset);
EXTERN(VECTORS);

_stack_top = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .vectors ORIGIN(FLASH) :
  {
    KEEP(*(.vectors));
  } > FLASH

  .boot : ALIGN(4)
  {
    *(.boot .boot.*);
    . = ALIGN(4);
  } > FLASH

  .text : ALIGN(4)
  {
    *(.text .text.*);
    . = ALIGN(4);
  } > FLASH

  .rodata : ALIGN(4)
  {
    *(.rodata .rodata.*);
    . = ALIGN(4);
  } > FLASH

  /* Hot code, tagged with #[link_section = ".ramtext.*"], runs from code RAM. */
  .ramtext : ALIGN(8)
  {
    __ramtext_start = .;
    *(.ramtext .ramtext.*);
    . = ALIGN(8);
    __ramtext_end = .;
  } > CODERAM AT > FLASH
  __ramtext_load = LOADADDR(.ramtext);

  /* The bytes behind .ramtext occupy the start of the data-bus view of RAM. */
  .data (ORIGIN(RAM) + (__ramtext_end - ORIGIN(CODERAM))) : ALIGN(8)
  {
    __data_start = .;
    *(.data .data.*);
    . = ALIGN(8);
    __data_end = .;
  } > RAM AT > FLASH
  __data_load = LOADADDR(.data);

  .bss (NOLOAD) : ALIGN(8)
  {
    __bss_start = .;
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(8);
    __bss_end = .;
  } > RAM

  __heap_start = .;

  /DISCARD/ :
  {
    *(.ARM.exidx .ARM.exidx.* .ARM.extab.*);
  }
}

ASSERT(__bss_end <= _stack_top - 10K, "less than 12 KB left for the stack");
