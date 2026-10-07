/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdint.h>
/*
 * A real board's board.h pulls in bf0_hal.h, and drivers rely on that for
 * basics such as RT_WEAK -- drv_spi_flash.c uses it at file scope, before its
 * own includes. This carrier has no HAL to include, so the RTOS headers stand in
 * for it: everything that expected those basics through board.h still compiles,
 * without pretending a chip exists.
 *
 * rthw.h is part of the same substitution. A real board.h also includes
 * drv_common.h, which is what declares rt_hw_interrupt_disable() and friends;
 * rtthread.h alone does not. Without it those calls become implicit
 * declarations, which only surfaces as a build error because the SDK builds
 * with -Werror -- the diagnostics point at the call site, not at board.h.
 */
#include <rtthread.h>
#include <rthw.h>

/* Heap spans the linker-defined RAM between .bss end and the heap limit */
extern int __bss_end__;
extern int __HeapLimit;

#define HEAP_BEGIN  ((void *)&__bss_end__)
#define HEAP_END    ((void *)&__HeapLimit)

#endif /* __BOARD_H__ */
