/******************************************************************************
 * @file     gcc_arm.ld
 * @brief    GNU Linker Script for Cortex-M based device
 * @version  V2.0.0
 * @date     21. May 2019
 ******************************************************************************/
#include "rtconfig.h"
#include "mem_map.h"
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
__ROM_BASE = LCPU_RAM_CODE_START_ADDR;
__ROM_SIZE = LCPU_RAM_CODE_SIZE*2;


/*--------------------- Embedded RAM Configuration ----------------------------
  <h> RAM Configuration
    <o0> RAM Base Address    <0x0-0xFFFFFFFF:8>
    <o1> RAM Size (in Bytes) <0x0-0xFFFFFFFF:8>
  </h>
 -----------------------------------------------------------------------------*/
__RAM_BASE = LPSYS_RAM_BASE;
__RAM_SIZE = LPSYS_RAM_SIZE-LCPU_MBOX_SIZE;

/*--------------------- Stack / Heap Configuration ----------------------------
  <h> Stack / Heap Configuration
    <o0> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
    <o1> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
  </h>
  -----------------------------------------------------------------------------*/
__STACK_SIZE = 0x00000400;
__HEAP_SIZE  = 0x00000000;

/*
 *-------------------- <<< end of configuration section >>> -------------------
 */
MEMORY
{
    ROM (rx)  : ORIGIN = __ROM_BASE,  LENGTH = __ROM_SIZE
    RAM (rw)  : ORIGIN = __RAM_BASE,  LENGTH = __RAM_SIZE
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
    _stext = ABSOLUTE(.);
    KEEP(*(.vectors))

    *(.text*)
    *(.rodata*)

    * (.*l1_ret_text_*)
    * (.*l1_ret_rodata_*)

    KEEP(*(.init))
    KEEP(*(.fini))

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
    __rt_utest_tc_tab_start = .;
    KEEP(*(UtestTcTab))
    __rt_utest_tc_tab_end = .;

    /* section information for initial. */
    . = ALIGN(4);
    __rt_init_start = .;
    KEEP(*(SORT(.rti_fn*)))
    __rt_init_end = .;

    . = ALIGN(4);
    __SerialTranExport_start__ = .;
    KEEP(*(SerialTranExport))
    __SerialTranExport_end__ = .;

    . = ALIGN(4);
    __sifli_reg_start__ = .;
    KEEP(*(SORT(.sifli_reg*)))
    __sifli_reg_end__ = .;


    KEEP(*(.eh_frame*))
    _etext = ABSOLUTE(.);
  } > ROM

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
    /* no need to copy .data as it's at the same space as load region */
    /*
     * LONG (LOADADDR(.data))
     * LONG (ADDR(.data))
     * LONG (SIZEOF(.data))    
     */ 

    __copy_table_end__ = .;
  } > ROM

  .zero.table :
  {
    . = ALIGN(4);
    __zero_table_start__ = .;
    /* Add each additional bss section here */
    
    LONG (__bss_start__)
    LONG (__bss_end__ - __bss_start__)    
    __zero_table_end__ = .;
  } > ROM

  /**
   * Location counter can end up 2byte aligned with narrow Thumb code but
   * __etext is assumed by startup code to be the LMA of a section in RAM
   * which must be 4byte aligned 
   */
  __etext = ALIGN (4);
  
  .data (__etext - __ROM_BASE + __RAM_BASE) :
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
  ASSERT((LOADADDR(.data) | 0x20000000) == ADDR(.data), "not aligned")

}
