/*
 * Copyright (c) 2006-2025 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-30     SiFli        first version for SiFli input capture adapter
 */

#ifndef __DRV_INPUTCAPTURE_H__
#define __DRV_INPUTCAPTURE_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <drv_common.h>
#include "bf0_hal_tim.h"

/* NVIC refcount: all capture channels of one GPTIM share the same IRQn,
   so the IRQ may be disabled only after the last channel is closed. */
typedef struct
{
    uint8_t   ref;
} _incap_irq_ref_t;

struct bf0_inputcapture
{
    struct rt_inputcapture_device inputcapture_device;
    GPT_HandleTypeDef              tim_handle;
    IRQn_Type                      tim_irqn;
    uint8_t                        core;
    const char                    *name;
    uint32_t                       channel;
    rt_uint32_t                    last_ts;        /*!< 32-bit timestamp of the last capture edge */
    rt_uint32_t                    pulsewidth_us;
    rt_bool_t                      in_high;
    rt_bool_t                      first_capture;
    _incap_irq_ref_t               *ref;
};

int incap_register_all(void);

#endif /* __DRV_INPUTCAPTURE_H__ */
