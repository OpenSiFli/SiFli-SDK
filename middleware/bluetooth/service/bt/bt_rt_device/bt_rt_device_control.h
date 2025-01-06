/**
  ******************************************************************************
  * @file   bt_rt_device_control.h
  * @author Sifli software development team
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2022 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
