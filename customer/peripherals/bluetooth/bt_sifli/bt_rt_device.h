/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_RT_DEVICE_H
#define _BT_RT_DEVICE_H


#include "bt_rt_device_urc.h"
#ifdef BT_USING_HF
    #include "bt_rt_device_urc_hf.h"
#endif
#ifdef BT_USING_A2DP
    #include "bt_rt_device_urc_a2dp.h"
#endif
#ifdef BT_USING_AVRCP
    #include "bt_rt_device_urc_avrcp.h"
#endif
#ifdef BT_USING_AG
    #include "bt_rt_device_urc_ag.h"
#endif
#ifdef BT_USING_HID
    #include "bt_rt_device_urc_hid.h"
#endif
#ifdef BT_USING_SPP
    #include "bt_rt_device_urc_spp.h"
#endif
#ifdef BT_USING_PBAP
    #include "bt_rt_device_urc_pbap.h"
#endif
#ifdef BT_USING_GATT
    #include "bt_rt_device_urc_gatt.h"
#endif
#include "bt_rt_device_control.h"

#endif // _BT_RT_DEVICE_H
