/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_RT_DEVICE_CONTROL_AG_H
#define _BT_RT_DEVICE_CONTROL_AG_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

bt_err_t bt_sifli_control_ag(struct rt_bt_device *bt_handle, int cmd, void *args);


#endif /* _BT_RT_DEVICE_CONTROL_AG_H */
