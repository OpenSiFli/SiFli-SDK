/*
 * Copyright (c) 2006-2025 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-07-10     SiFli        first version for SiFli CAN adapter
 */

#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <drv_common.h>
#include "bf0_hal_can.h"

/* SiFli CAN: use secondary tx fifo for transmission */
#define CAN_SNDBOX_NUM       CAN_SECONDARY_TX_FIFO_DEPTH

struct bf0_can_config
{
    const char          *device_name;
    CAN_TypeDef         *Instance;
    IRQn_Type            irq_type;
};

struct bf0_can
{
    struct rt_can_device  device;          /* RT-Thread CAN device         */
    CAN_HandleTypeDef     CanHandle;       /* SiFli HAL CAN handle         */
    CAN_FilterTypeDef     FilterConfig;    /* Default filter config        */
    struct bf0_can_config *config;         /* CAN hardware config          */
    char                 *name;            /* Device name                  */
};

/* CAN instance indices */
enum
{
#ifdef BSP_USING_CAN1
    CAN1_INDEX,
#endif
#ifdef BSP_USING_CAN2
    CAN2_INDEX,
#endif
    CAN_MAX,
};

int rt_hw_can_init(void);

#endif /* __DRV_CAN_H__ */
