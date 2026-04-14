OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)

EXTERN(Init UnInit EraseSector ProgramPage FlashDevice)
EXTERN(SEGGER_OPEN_Read SEGGER_OPEN_Program SEGGER_OPEN_Erase SEGGER_OPEN_CalcCRC)

PHDRS
{
  loader PT_LOAD FLAGS(7);
  descriptor PT_LOAD FLAGS(5);
}

SECTIONS
{
  PrgCode 0x00000000 :
  {
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
    *(.text*)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.vfp11_veneer)
    *(.ARM.extab*)
    *(.ARM.exidx*)
  } :loader

  PrgData :
  {
    . = ALIGN(8);
    *(.data*)
  } :loader

  PrgData (NOLOAD) :
  {
    . = ALIGN(8);
    *(.bss*)
    *(COMMON)
  } :loader

  DevDscr ALIGN(4) :
  {
    KEEP(*(DevDscr))
  } :descriptor

  /DISCARD/ :
  {
    *(.comment*)
    *(.note*)
    *(.eh_frame*)
    *(.gnu*)
  }
}
