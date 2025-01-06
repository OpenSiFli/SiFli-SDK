
SECTIONS
{
  .vectors :
  {
    _stext = ABSOLUTE(.);  
    KEEP(*(.vectors));
    /* workaround to avoid load address of .retm_data doesn't skip .vectors region */
    . = . + 4;
  } > ROM

  .stack :
  {
    . = ALIGN(8);
    __StackLimit = .;
    . = . + __STACK_SIZE;
    . = ALIGN(8);
    __StackTop = .;
  } > RAM
  PROVIDE(__stack = __StackTop);

  .heap :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    . = . + __HEAP_SIZE;
    . = ALIGN(8);
    __HeapLimit = .;
  } > RAM

  .retm_data :
  {
    . = ALIGN(4);
    __rw_retm_data_start__ = .;


    . = ALIGN(4);
    __rw_retm_data_end__ = .;

  } > RET_RAM AT > ROM
  

  .text :
  {

    *(.text*)
    *(.rodata*)

    KEEP(*(.init))
    KEEP(*(.fini))

    /* .ctors */
    /*
    *crtbegin.o(.ctors)
    *crtbegin?.o(.ctors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
    *(SORT(.ctors.*))
    *(.ctors)
    */

    /* .dtors */
    /*
    *crtbegin.o(.dtors)
    *crtbegin?.o(.dtors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
    *(SORT(.dtors.*))
    *(.dtors)
    */

    /* section information for finsh shell */
    . = ALIGN(4);
    __fsymtab_start = .;
    KEEP(*(FSymTab))
    __fsymtab_end = .;

    . = ALIGN(4);
    __vsymtab_start = .;
    KEEP(*(VSymTab))
    __vsymtab_end = .;

    . = ALIGN(4);
    LcdDriverDescTab_start = .;
    KEEP(*(LcdDriverDescTab))
    LcdDriverDescTab_end = .;
    
    . = ALIGN(4);
    __rt_utest_tc_tab_start = .;
    KEEP(*(UtestTcTab))
    __rt_utest_tc_tab_end = .;

    /* section information for initial. */
    . = ALIGN(4);
    __rt_init_start = .;
    KEEP(*(SORT(.rti_fn*)))
    __rt_init_end = .;

    . = ALIGN(4);
    BuiltinAppTab_start = .;
    KEEP(*(BuiltinAppTab))
    BuiltinAppTab_end = .;

    . = ALIGN(4);
    __app_font_start__ = .;
    KEEP(*(.app_font))
    __app_font_end__ = .;

    . = ALIGN(4);
    __SerialTranExport_start__ = .;
    KEEP(*(SerialTranExport))
    __SerialTranExport_end__ = .;

    . = ALIGN(4);
    __sifli_reg_start__ = .;
    KEEP(*(SORT(.sifli_reg*)))
    __sifli_reg_end__ = .;

    . = ALIGN(4);
    __bt_sifli_reg_start__ = .;
    KEEP(*(SORT(.bt_sifli_reg*)))
    __bt_sifli_reg_end__ = .;

    KEEP(*(.eh_frame*))
    _etext = ABSOLUTE(.);

  } > ROM 
  
  
  .rom_ex :
  {
  
    . = ALIGN(4);
    __rw_rom_ex_start__ = .;
    __ER_IROM1_EX$$RO_start__ = .;
    __ER_IROM1_EX$$RO_load_start__ = LOADADDR(.rom_ex);


    . = ALIGN(4);
    __rw_rom_ex_end__ = .;
    __ER_IROM1_EX$$RO_end__ = .;
    __ER_IROM1_EX$$RO_load_end__ = LOADADDR(.rom_ex) + SIZEOF(.rom_ex);
    
  } > ROM_EX AT > ROM

  /*
   * SG veneers:
   * All SG veneers are placed in the special output section .gnu.sgstubs. Its start address
   * must be set, either with the command line option `--section-start` or in a linker script,
   * to indicate where to place these veneers in memory.
   */
/*
  .gnu.sgstubs :
  {
    . = ALIGN(32);
  } > ROM
*/
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > ROM

  __exidx_start = .;
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > ROM
  __exidx_end = .;

  .copy.table :
  {
    . = ALIGN(4);
    __copy_table_start__ = .;
    LONG (LOADADDR(.data))
    LONG (ADDR(.data))
    LONG (SIZEOF(.data))

    LONG (LOADADDR(.retm_data))
    LONG (ADDR(.retm_data))
    LONG (SIZEOF(.retm_data))
    
    LONG (LOADADDR(.rom_ex))
    LONG (ADDR(.rom_ex))
    LONG (SIZEOF(.rom_ex))    
    
    __copy_table_end__ = .;
  } > ROM
  

  .zero.table :
  {
    . = ALIGN(4);
    __zero_table_start__ = .;
    /* Add each additional bss section here */
    
    LONG (__bss_start__)
    LONG (__bss_end__ - __bss_start__)    

    LONG (ADDR(.retm_bss))
    LONG (SIZEOF(.retm_bss))

    __zero_table_end__ = .;
  } > ROM

  .retm_bss :
  {
    . = ALIGN(4);
    __rw_retm_bss_start__ = .;
    __RW_IRAM_RET$$ZI_start__ = .;
    * (.bss.retm_bss_*)
 
    *idle.o (.bss.rt_thread_stack)
    *drv_psram.o(.bss.bf0_psram_handle)

    . = ALIGN(4);
    __RW_IRAM_RET$$ZI_end__ = .;
  } > RET_RAM

  .RW_IRAM0 :
  {
    *(non_ret)
    *(.*l1_non_ret_data_*)
    *(.*l1_non_ret_bss_*)
  } > RAM 

  /**
   * Location counter can end up 2byte aligned with narrow Thumb code but
   * __etext is assumed by startup code to be the LMA of a section in RAM
   * which must be 4byte aligned 
   */
  __etext = ALIGN (4);

  .data :
  {
    _sdata = ABSOLUTE(.);    
    __data_start__ = .;
    __RW_IRAM1_start__ = .;
    *(vtable)
    *(.data)
    *(.data.*)
    *(.l1_ret_data_*)

    . = ALIGN(4);
    /* preinit data */
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP(*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);

    . = ALIGN(4);
    /* init data */
    _sinit = ABSOLUTE(.);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    _einit = ABSOLUTE(.);
    PROVIDE_HIDDEN (__init_array_end = .);


    . = ALIGN(4);
    /* finit data */
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array))
    PROVIDE_HIDDEN (__fini_array_end = .);

    KEEP(*(.jcr*))
    . = ALIGN(4);
    /* All data end */
    __data_end__ = .;
    __RW_IRAM1_end__ = .;
    _edata = ABSOLUTE(.);    
  } > RAM AT > ROM

  /*
   * Secondary data section, optional
   *
   * Remember to add each additional data section
   * to the .copy.table above to asure proper
   * initialization during startup.
   */

  __etext2 = ALIGN (4);

  .RW_PSRAM1 :
  {
    . = ALIGN(4);
    __rw_psram1_start__ = .;
    *(.*l2_ret_data_*)
    *(.*l2_ret_bss_*)
    *(.*l2_cache_ret_data_*)
    *(.*l2_cache_ret_bss_*)
    . = ALIGN(4);
    __rw_psram1_end__ = .;

  } > PSRAM


  .RW_PSRAM_NON_RET :
  {
    /* aligned to cache line size */
    . = ALIGN(32);
    __RW_PSRAM_NON_RET_start__ = .;
    *(.nand_cache)
    *(.*l2_cache_non_ret_data_*)
    *(.*l2_cache_non_ret_bss_*)
    . = ALIGN(4);
    __RW_PSRAM_NON_RET_end__ = .;

  } > PSRAM

  .RW_PSRAM2 :
  {
    . = ALIGN(4);
    __rw_psram2_start__ = .;
    *(.*l2_non_ret_data_*)
    *(.*l2_non_ret_bss_*)
    *(.*l2_ret_data2_*)
    *(.*l2_ret_bss2_*)
    *(.*l2_cache_ret_data2_*)
    *(.*l2_cache_ret_bss2_*)
    . = ALIGN(4);
    __rw_psram2_end__ = .;

  } > PSRAM2


  .bss :
  {
    _sbss = ABSOLUTE(.);
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    *(.l1_ret_bss_*)   
    . = ALIGN(4);
    __bss_end__ = .;
    __bss_end = .;
    _ebss = ABSOLUTE(.);
    __end__ = .;
    PROVIDE(end = .);
  } > RAM AT > RAM
  
  /*
   * Secondary bss section, optional
   *
   * Remember to add each additional bss section
   * to the .zero.table above to asure proper
   * initialization during startup.
   */
/*
  .bss2 :
  {
    . = ALIGN(4);
    __bss2_start__ = .;
    *(.bss2)
    *(.bss2.*)
    . = ALIGN(4);
    __bss2_end__ = .;
  } > RAM2 AT > RAM2
*/

  /* Check if data + heap + stack exceeds RAM limit */
  /* ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack") */
  
  .rom2 :
  {
    *(.ROM1_IMG)
    *(.ROM3_IMG*.*)
  } > ROM2

  .rom3 :
  {
    *lvsf_font_*(.rodata*)
  } > ROM3
}
