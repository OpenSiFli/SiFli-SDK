/*
 * SPDX-FileCopyrightText: 2022-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_RT_DEVICE_CONTROL_COMMON_H
#define _BT_RT_DEVICE_CONTROL_COMMON_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>


uint8_t bt_sifli_get_role_profile_connect_status(rt_bt_device_t *dev, bt_cm_link_type_t link_type, bt_profile_t profile);

bt_err_t bt_sifli_control_common(struct rt_bt_device *bt_handle, int cmd, void *args);


#endif /* _BT_RT_DEVICE_CONTROL_COMMON_H */

