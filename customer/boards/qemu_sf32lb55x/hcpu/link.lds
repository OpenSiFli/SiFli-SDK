/******************************************************************************
 * @file     link.lds
 * @brief    GNU Linker Script for QEMU Cortex-M33 (MPS2-AN505) virtual board
 *
 * Memory layout matches the QEMU mps2-an505 machine model:
 *   ROM  : 0x10000000 (MPS2 ZBT PS RAM, code region)
 *   RAM  : 0x20000000 (32 KB, the only block mapped here)
 *   RAM2 : 0x28000000 (2 MB, where the data region lives)
 *
 * Copy/zero table layout matches the __copy_table_start__ / __zero_table_start__
 * convention consumed by startup_qemu.S (source, dest, size in bytes).
 ******************************************************************************/
ENTRY(Reset_Handler)

MEMORY
{
    ROM (rx)  : ORIGIN = 0x10000000, LENGTH = 0x00400000  /* 4 MB code region  */
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000  /* 32 KB SRAM        */
    /* The larger blocks, which the original script never used. Test images --
     * Unity, CMock and the module under test together -- do not fit in 32 KB,
     * so the data region lives here. */
    RAM2 (rwx): ORIGIN = 0x28000000, LENGTH = 0x00200000  /* 2 MB              */
}

/*
 * The RAM length above is what the machine actually maps, not what the AN505
 * memory map suggests. QEMU gives 0x20000000 only 32 KB (verified with
 * "-monitor stdio" and "info mtree -f"); the larger blocks live at 0x28000000.
 *
 * This matters because the over-sized figure made a whole class of failure
 * invisible: raising the heap pushed .stack past the end of real memory, so the
 * first push faulted and the board locked up before printing anything -- with no
 * link error to say why, since the linker believed the region was 4 MB. Getting
 * the size right means an over-large heap is now a build failure.
 */

/*
 * Heap and stack sizes.
 *
 * The heap has to hold the shell's 4 KB thread stack plus this BSP's 1 KB input
 * reader -- the original 8 KB could not fit both, and rt_thread_create simply
 * returned RT_NULL for whichever lost, leaving a board that boots, prints its
 * banner and has no console at all. Generous now that the data region sits in
 * the 2 MB block.
 */
__STACK_SIZE = 0x00002000;   /* 8 KB */
__HEAP_SIZE  = 0x00010000;   /* 64 KB */

SECTIONS
{
    .vectors :
    {
        KEEP(*(.vectors));
        . = ALIGN(4);
    } > ROM

    .text :
    {
        *(.text*)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        /*
         * Section information for initialization.
         *
         * RT-Thread's INIT_*_EXPORT puts function pointers into .rti_fn.<level>
         * sections and walks them by address at startup (see
         * rtos/rtthread/src/components.c). Without this KEEP, --gc-sections
         * discards every one of them: nothing is initialized, so there is no
         * shell and no components -- and nothing reports a problem, because the
         * image still links and still boots. SORT orders the entries by their
         * level string, which is what makes the ordering meaningful.
         */
        . = ALIGN(4);
        __rt_init_start = .;
        KEEP(*(SORT(.rti_fn*)))
        __rt_init_end = .;

        /*
         * Sections the rest of the runtime expects the linker script to define.
         *
         * These are not optional decoration: the moment the init table above is
         * kept, finsh and the C++ runtime are actually linked in, and they
         * reference these symbols by name. Leaving them out turns a silent
         * "no shell" into a link error -- which is how this list was found.
         */

        /* finsh's command and variable tables. */
        . = ALIGN(4);
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;

        . = ALIGN(4);
        __vsymtab_start = .;
        KEEP(*(VSymTab))
        __vsymtab_end = .;

        /* C++ static constructors: old GCC uses .ctors, new uses .init_array.
         * cxx_crt_init.c walks these between __ctors_start__ and __ctors_end__. */
        . = ALIGN(4);
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN(__preinit_array_end = .);

        . = ALIGN(4);
        PROVIDE(__ctors_start__ = .);
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.ctors.*)))
        KEEP(*(.ctors))
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN(__init_array_end = .);
        PROVIDE(__ctors_end__ = .);

        /* Destructors are never run on this target, but the symbols are part of
         * the same contract and cost nothing to define. */
        . = ALIGN(4);
        PROVIDE(__dtors_start__ = .);
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE(__dtors_end__ = .);

        . = ALIGN(4);
        __etext = .;
    } > ROM

    .data :
    {
        . = ALIGN(4);
        __data_start__ = .;
        *(.data*)
        *(.data.*)
        . = ALIGN(4);
        __data_end__ = .;
    } > RAM AT> ROM

    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;
        LONG (LOADADDR(.data))
        LONG (ADDR(.data))
        LONG (SIZEOF(.data))
        __copy_table_end__ = .;
    } > ROM

    .zero.table :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        LONG (__bss_start__)
        LONG (__bss_end__ - __bss_start__)
        __zero_table_end__ = .;
    } > ROM

    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM2

    .heap :
    {
        . = ALIGN(8);
        __end__ = .;
        PROVIDE(end = .);
        . = . + __HEAP_SIZE;
        . = ALIGN(8);
        __HeapLimit = .;
    } > RAM2

    .stack :
    {
        . = ALIGN(8);
        __StackLimit = .;
        . = . + __STACK_SIZE;
        . = ALIGN(8);
        __StackTop = .;
    } > RAM2
    PROVIDE(__stack = __StackTop);

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx*)
        *(.gnu.linkonce.armexidx.*)
    } > ROM
    __exidx_end = .;
}
