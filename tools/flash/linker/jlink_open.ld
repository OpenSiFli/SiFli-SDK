OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)

ENTRY(Init)

_Min_Heap_Size = 0;
_Min_Stack_Size = 0;

MEMORY
{
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x00040000
}

SECTIONS
{
  PrgCode :
  {
    . = ALIGN(4);
    KEEP(*(PrgCode))
    KEEP(*(PrgCode.*))
    . = ALIGN(4);
  } > RAM

  .text :
  {
    . = ALIGN(4);
    KEEP(*(.text.Init))
    KEEP(*(.text.UnInit))
    KEEP(*(.text.EraseSector))
    KEEP(*(.text.ProgramPage))
    KEEP(*(.text.BlankCheck))
    KEEP(*(.text.EraseChip))
    KEEP(*(.text.Verify))
    KEEP(*(.text.SEGGER_OPEN_Read))
    KEEP(*(.text.SEGGER_OPEN_Program))
    KEEP(*(.text.SEGGER_OPEN_Erase))
    KEEP(*(.text.SEGGER_OPEN_CalcCRC))
    *(.text)
    *(.text*)
    *(.*l1_ret_text_*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > RAM

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    *(.*l1_ret_rodata_*)
    . = ALIGN(4);
  } > RAM

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > RAM

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > RAM

  PrgData :
  {
    . = ALIGN(4);
    KEEP(*(PrgData))
    KEEP(*(PrgData.*))
    . = ALIGN(4);
  } > RAM

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > RAM

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > RAM

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > RAM

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM

  DevDscr :
  {
    . = ALIGN(4);
    KEEP(*(DevDscr))
    KEEP(*(DevDscr.*))
    . = ALIGN(4);
  } > RAM

  /DISCARD/ :
  {
    *(.comment*)
    *(.note*)
    *(.gnu*)
  }

  .ARM.attributes 0 :
  {
    KEEP(*(.ARM.attributes))
  }
}
