/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __WHEEL_DEVICE_H__
#define __WHEEL_DEVICE_H__
#include <rtthread.h>

#define WHEEL_DEVICE_FLAG_OPEN (0x80)

typedef enum
{
    WHEEL_CONTROL_CLOSE_DEVICE = 0x20 + 1,     /**< close motor device */
    WHEEL_CONTROL_OPEN_DEVICE,                 /**< open motor device */
    WHEEL_CONTROL_GET_STATE,
    WHEEL_CONTROL_CMD_MAX
} wheel_control_cmd_t;

typedef enum
{
    WHEEL_STATE_POWER_OFF = 0,
    WHEEL_STATE_POWER_ON,
} wheel_state_t;

typedef enum
{
    WHEEL_EOK = 0,
    WHEEL_ERROR,
} wheel_err_t;

typedef struct
{
    wheel_state_t last_state;
    wheel_state_t cur_state;
} wheel_status_t;

typedef struct rt_wheel_device
{
    struct rt_device   parent;
    wheel_status_t status;
    uint8_t ctrl_status;
    const struct rt_wheel_ops *ops;
} rt_wheel_t;

typedef wheel_err_t (*wheel_control_cb)(struct rt_wheel_device *dev_handle, int cmd, void *arg);

struct rt_wheel_ops
{
    wheel_control_cb control;
};


#endif
