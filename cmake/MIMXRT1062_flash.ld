ENTRY(ResetISR)

HEAP_SIZE = DEFINED(__heap_size__) ? __heap_size__ : 0x00002000;
STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x00004000;

MEMORY
{
  BOARD_FLASH (rx)  : ORIGIN = 0x60000000, LENGTH = 0x00800000
  SRAM_DTCM  (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x00020000
  SRAM_OC    (rwx)  : ORIGIN = 0x20200000, LENGTH = 0x00080000
}

SECTIONS
{
  .boot_hdr :
  {
    FILL(0xFF)
    . = ORIGIN(BOARD_FLASH) + 0x0000;
    KEEP(*(.boot_hdr.conf))
    . = ORIGIN(BOARD_FLASH) + 0x1000;
    KEEP(*(.boot_hdr.ivt))
    . = ORIGIN(BOARD_FLASH) + 0x1020;
    KEEP(*(.boot_hdr.boot_data))
    . = ORIGIN(BOARD_FLASH) + 0x1030;
    KEEP(*(.boot_hdr.dcd_data))
    . = ORIGIN(BOARD_FLASH) + 0x2000;
  } > BOARD_FLASH

  .interrupts :
  {
    __VECTOR_TABLE = .;
    KEEP(*(.isr_vector))
  } > BOARD_FLASH

  .text :
  {
    *(.after_vectors.reset)
    *(.after_vectors*)
    *(.text*)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    __data_section_table = .;
    LONG(LOADADDR(.data));
    LONG(ADDR(.data));
    LONG(SIZEOF(.data));
    __data_section_table_end = .;

    __bss_section_table = .;
    LONG(ADDR(.bss));
    LONG(SIZEOF(.bss));
    __bss_section_table_end = .;

    . = ALIGN(4);
    _etext = .;
  } > BOARD_FLASH

  .ARM.extab :
  {
    *(.ARM.extab*)
    *(.gnu.linkonce.armextab.*)
  } > BOARD_FLASH

  __exidx_start = .;
  .ARM.exidx :
  {
    *(.ARM.exidx*)
    *(.gnu.linkonce.armexidx.*)
  } > BOARD_FLASH
  __exidx_end = .;

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > BOARD_FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > BOARD_FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > BOARD_FLASH

  .data : ALIGN(4)
  {
    __data_start__ = .;
    *(.data*)
    *(.ramfunc*)
    . = ALIGN(4);
    __data_end__ = .;
  } > SRAM_OC AT > BOARD_FLASH

  .bss (NOLOAD) : ALIGN(4)
  {
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > SRAM_OC

  .noinit (NOLOAD) : ALIGN(4)
  {
    *(.noinit*)
    . = ALIGN(4);
  } > SRAM_OC

  __base_NCACHE_REGION = .;
  __top_NCACHE_REGION = .;
  __NCACHE_REGION_START = __base_NCACHE_REGION;
  __NCACHE_REGION_SIZE = 0;

  __HeapBase = .;
  __HeapLimit = ORIGIN(SRAM_OC) + LENGTH(SRAM_OC) - STACK_SIZE;
  __StackLimit = __HeapLimit;
  __StackTop = ORIGIN(SRAM_OC) + LENGTH(SRAM_OC);
  _vStackTop = __StackTop;

  PROVIDE(end = __HeapBase);
  PROVIDE(_end = __HeapBase);
  PROVIDE(__stack = __StackTop);

}
