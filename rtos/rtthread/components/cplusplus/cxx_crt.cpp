/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-03-07     Bernard      Add copyright header.
 */

#include <rtthread.h>
#include "cxx_crt.h"

extern "C" void *cxx_mem_allocate(size_t size);
extern "C" void cxx_mem_free(void *ptr);

#ifdef BSP_USING_PC_SIMULATOR
extern "C" void rt_hw_board_init(void);
#else
RT_WEAK extern "C" void *cxx_mem_allocate(size_t size)
{
    return rt_malloc(size);
}
RT_WEAK extern "C" void cxx_mem_free(void *ptr)
{
    return rt_free(ptr);
}
#endif

#ifdef BSP_USING_PC_SIMULATOR
    extern "C" void rt_simu_heap_init(void);
#else
    #define rt_simu_heap_init()
#endif
void *operator new (size_t size)
{
    rt_simu_heap_init();
    void *p = cxx_mem_allocate(size);
    return p;
}

void *operator new[](size_t size)
{
    rt_simu_heap_init();
    void *p = cxx_mem_allocate(size);
    return p;
}

void operator delete (void *ptr)
{
    cxx_mem_free(ptr);
}

void operator delete[](void *ptr)
{
    cxx_mem_free(ptr);
}

void __cxa_pure_virtual(void)
{
    rt_kprintf("Illegal to call a pure virtual function.\n");
}
