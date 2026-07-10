/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_RT_DEVICE_CONTROL_H
#define _BT_RT_DEVICE_CONTROL_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

//bt set event info
#define BT_SET_CLOSE_EVENT             (0x0001 << 0)
#define BT_SET_RD_LOCAL_NAME_EVENT     (0x0001 << 1)
#define BT_SET_INQ_EVENT               (0x0001 << 2)
#define BT_SET_DIS_GAP_EVENT           (0x0001 << 3)
#define BT_SET_AVSNK_OPEN_EVENT        (0x0001 << 4)
#define BT_SET_AVSNK_CLOSE_EVENT       (0x0001 << 5)
#define BT_SET_SIRI_ON_EVENT           (0x0001 << 6)
#define BT_SET_SIRI_OFF_EVENT          (0x0001 << 7)
#define BT_SET_AVRCP_OPEN_EVENT        (0x0001 << 8)
#define BT_SET_AVRCP_CLOSE_EVENT       (0x0001 << 9)
#define BT_SET_HID_OPEN_EVENT          (0x0001 << 10)
#define BT_SET_HID_CLOSE_EVENT         (0x0001 << 11)
#define BT_SET_DIAL_EVENT              (0x0001 << 12)
#define BT_SET_CLCC_EVENT              (0x0001 << 13)
#define BT_SET_RD_LOCAL_RSSI_EVENT     (0x0001 << 14)
#define BT_SET_CANCEL_PAGE_EVENT       (0x0001 << 15)
#define BT_SET_OPEN_EVENT              (0x0001 << 16)
#define BT_SET_VGS_EVENT               (0x0001 << 17)
#define BT_SET_DTMF_EVENT              (0x0001 << 18)


bt_err_t bt_sifli_control(struct rt_bt_device *bt_handle, int cmd, void *args);

U8 bt_sifli_check_bt_event(uint32_t event);
U8 bt_sifli_set_bt_event(uint32_t event);
U8 bt_sifli_reset_bt_event(uint32_t event);
uint32_t bt_sifli_get_bt_event();
#endif /* _BT_RT_DEVICE_CONTROL_H */
