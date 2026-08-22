/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>
#include <string.h>
#include "drv_bt.h"
//#include "utf8_unicode.h"
#include "bt_device.h"
#include "bt_rt_device.h"
#include "bts2_app_inc.h"
#include "bf0_ble_common.h"
#include "bf0_sibles.h"
#include "bt_connection_manager.h"


static const struct rt_bt_ops bt_sifli_ops =
{
    .control = bt_sifli_control
};

int bt_sifli_init(void)
{
    rt_hw_bt_init(&bt_sifli_ops, BT_DEVICE_FLAG_OPEN);
    return BT_EOK;
}

INIT_COMPONENT_EXPORT(bt_sifli_init);



