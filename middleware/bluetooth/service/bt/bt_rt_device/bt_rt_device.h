/**
  ******************************************************************************
  * @file   bt_rt_device.h
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
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
