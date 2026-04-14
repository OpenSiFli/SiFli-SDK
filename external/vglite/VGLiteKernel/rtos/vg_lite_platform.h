/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2022 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************/


#ifndef _VG_LITE_PLATFORM_H
#define _VG_LITE_PLATFORM_H

#include "stdint.h"
#include "stdlib.h"
#include <stdio.h>
#include "../vg_lite_debug.h"
#ifndef _vg_lite_h_
#include "../vg_lite_type.h"
#endif /* _vg_lite_h_ */
#include "../vg_lite_option.h"

#define _BAREMETAL 0
#define VG_SYSTEM_RESERVE_COUNT 1

/* Implementation of list. ****************************************/
typedef struct list_head {
    struct list_head *next;
    struct list_head *prev;
}list_head_t;

typedef struct heap_node {
    list_head_t list;
    uint32_t offset;
    unsigned long size;
    int32_t status;
    vg_lite_vidmem_pool_t pool;
}heap_node_t;

typedef struct vg_module_parameters
{

    uint32_t        register_mem_base;
    uint32_t        gpu_mem_base[VG_SYSTEM_RESERVE_COUNT];

    volatile void * contiguous_mem_base[VG_SYSTEM_RESERVE_COUNT];
    uint32_t        contiguous_mem_size[VG_SYSTEM_RESERVE_COUNT];
}
vg_module_parameters_t;

/*!
@brief Initialize the hardware mem setting.
*/
void vg_lite_init_mem(vg_module_parameters_t *param);

/*!
@brief The hardware IRQ handler.
*/
void vg_lite_IRQHandler(void);

#endif
