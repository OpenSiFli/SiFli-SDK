/******************************************************************************
 * @file     gcc_arm.ld
 * @brief    GNU Linker Script for Cortex-M based device
 * @version  V2.0.0
 * @date     21. May 2019
 ******************************************************************************/
/*
 * Copyright (c) 2009-2019 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 *-------- <<< Use Configuration Wizard in Context Menu >>> -------------------
 */

/*---------------------- Flash Configuration ----------------------------------
  <h> Flash Configuration
    <o0> Flash Base Address <0x0-0xFFFFFFFF:8>
    <o1> Flash Size (in Bytes) <0x0-0xFFFFFFFF:8>
  </h>
  -----------------------------------------------------------------------------*/
__ROM_BASE = 0x10020000;
__ROM_SIZE = 0x000E0000;

/*--------------------- Embedded RAM Configuration ----------------------------
  <h> RAM Configuration
    <o0> RAM Base Address    <0x0-0xFFFFFFFF:8>
    <o1> RAM Size (in Bytes) <0x0-0xFFFFFFFF:8>
  </h>
 -----------------------------------------------------------------------------*/
__RAM_BASE = 0x20000000;
__RAM_SIZE = 0x000C8000;

/*--------------------- Stack / Heap Configuration ----------------------------
  <h> Stack / Heap Configuration
    <o0> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
    <o1> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
  </h>
  -----------------------------------------------------------------------------*/
__STACK_SIZE = 0x00001000;
__HEAP_SIZE  = 0x00000C00;

/*
 *-------------------- <<< end of configuration section >>> -------------------
 */
MEMORY
{
    ROM (rx) : ORIGIN = __ROM_BASE, LENGTH = __ROM_SIZE
    RAM (rw) : ORIGIN = __RAM_BASE, LENGTH = __RAM_SIZE
    ROM_EX(rw):ORIGIN = 0x200FBB80, LENGTH = 0x4080
    ITCM(rw) : ORIGIN = 0x00010000, LENGTH = 64k
    RETM(rw) : ORIGIN = 0x00020000, LENGTH = 64k
    PSRAM(rw): ORIGIN = 0x60000000, LENGTH = 4M
    ROM2 (rx): ORIGIN = 0x64000000, LENGTH = 0x300000
    ROM3 (rx): ORIGIN = 0x64300000, LENGTH = 0x880000
}

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 */
ENTRY(Reset_Handler)

SECTIONS
{
  .text :
  {
    KEEP(*(.vectors))
    EXCLUDE_FILE (*bf0_hal_qspi.o 
                  *flash_table.o
                  *drv_spi_flash
                  *bf0_hal_dma
                  *bf0_hal_hpaon.o
                  *bf0_hal_rcc.o
                  *drv_io.o
                  *bf0_hal_gpio.o) *(.text*)
          
    EXCLUDE_FILE (*bf0_hal_qspi.o 
                  *flash_table.o
                  *drv_spi_flash
                  *bf0_hal_hpaon.o
                  *lvsf_font_*) *(.rodata*)
          
    KEEP(*(.init))
    KEEP(*(.fini))

/*
    /* .ctors */
    *crtbegin.o(.ctors)
    *crtbegin?.o(.ctors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
    *(SORT(.ctors.*))
    *(.ctors)

    /* .dtors */
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


    KEEP(*(.eh_frame*))
  } > ROM
  
  .rom_ex :
  {
  
    . = ALIGN(4);
    __rw_rom_ex_start__ = .;
    __ER_IROM1_EX$$RO_start__ = .;
    __ER_IROM1_EX$$RO_load_start__ = LOADADDR(.rom_ex);

    *bf0_hal_qspi.o(.text* .rodata*)
    *flash_table.o (.text* .rodata*)
    *bf0_hal_hpaon.o (.text* .rodata*)
    *(.l1_non_ret_text_*)
    *(.l1_non_ret_rodata_*)

    . = ALIGN(4);
    __rw_rom_ex_end__ = .;
    __ER_IROM1_EX$$RO_end__ = .;
    __ER_IROM1_EX$$RO_load_end__ = LOADADDR(.rom_ex) + SIZEOF(.rom_ex);
    
  } > ROM_EX AT > ROM

  /*
   * SG veneers:
   * All SG veneers are placed in the special output section .gnu.sgstubs. Its start address
   * must be set, either with the command line option ‘--section-start’ or in a linker script,
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

    LONG (LOADADDR(.itcm_text))
    LONG (ADDR(.itcm_text))
    LONG (SIZEOF(.itcm_text))

    LONG (LOADADDR(.itcm_data))
    LONG (ADDR(.itcm_data))
    LONG (SIZEOF(.itcm_data))
    
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

    LONG (ADDR(.itcm_bss))
    LONG (SIZEOF(.itcm_bss))

    __zero_table_end__ = .;
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
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
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

  .RW_PSRAM2 :
  {
    . = ALIGN(4);
    __RW_PSRAM2_start__ = .;
    *(.nand_cache)
    *(.*l2_non_ret_data_*)
    *(.*l2_non_ret_bss_*)
    *(.*l2_cache_non_ret_data_*)
    *(.*l2_cache_non_ret_bss_*)
    . = ALIGN(4);
    __RW_PSRAM2_end__ = .;

  } > PSRAM

  .retm_data :
  {
    . = ALIGN(4);
    __rw_retm_data_start__ = .;
    * (.*l1_ret_text_*)
    * (.*l1_ret_rodata_*)
    * (.*retm_data_*)
 
    *bf0_hal_rcc.o   (.text.*)
    *bf0_pm_a0.o     (.text.sifli_light_handler)
    *bf0_pm_a0.o     (.text.sifli_deep_handler)
    *bf0_pm_a0.o     (.text.sifli_standby_handler)
    *drv_io.o        (.text*)
    *bf0_hal_gpio.o  (.text*)

    . = ALIGN(4);
    __rw_retm_data_end__ = .;

  } > RETM AT > ROM

  .retm_bss :
  {
    . = ALIGN(4);
    __rw_retm_bss_start__ = .;
    __RW_IRAM_RET$$ZI_start__ = .;
    * (.*retm_bss_*)
 
    *idle.o (.bss.rt_thread_stack)
    *drv_psram.o(.bss.bf0_psram_handle)

    . = ALIGN(4);
    __RW_IRAM_RET$$ZI_end__ = .;

  } > RETM
  
  .itcm_text :
  {
    . = ALIGN(4);
    __rw_itcm_ro_start__ = .;
    __ER_ITCM$$RO_start__ = .;
    __ER_ITCM$$RO_load_start__ = LOADADDR(.itcm_text);

    . = ALIGN(4);
    __rw_itcm_ro_end__ = .;
    __ER_ITCM$$RO_end__ = .;
    __ER_ITCM$$RO_load_end__ = LOADADDR(.itcm_text) + SIZEOF(.itcm_text);

  } > ITCM AT > ROM  

  .itcm_data :
  {
    . = ALIGN(4);
    __rw_itcm_data_start__ = .;
    __ER_ITCM$$RW_start__ = .;

    . = ALIGN(4);
    __rw_itcm_data_end__ = .;
    __ER_ITCM$$RW_end__ = .;

  } > ITCM AT > ROM

  .itcm_bss :
  {
    . = ALIGN(4);
    __rw_itcm_bss_start__ = .;
    __ER_ITCM$$ZI_start__ = .;    
    . = ALIGN(4);
    __rw_itcm_bss_end__ = .;
    __ER_ITCM$$ZI_end__ = .;    

  } > ITCM


  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    *(.l1_ret_bss_*)   
    . = ALIGN(4);
    __bss_end__ = .;
    __bss_end = .;
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

/*
  .heap (COPY) :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    . = . + __HEAP_SIZE;
    . = ALIGN(8);
    __HeapLimit = .;
  } > RAM
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
