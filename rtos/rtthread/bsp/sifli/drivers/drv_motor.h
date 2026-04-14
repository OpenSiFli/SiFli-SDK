/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/
#ifndef _DRV_MOTOR_H
#define _DRV_MOTOR_H

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>


#define  MOTOR_DEVICE_NAME  "motor_device"

int rt_hw_motor_init(const struct rt_motor_ops *ops);
#endif /* _DRV_BT_H */


