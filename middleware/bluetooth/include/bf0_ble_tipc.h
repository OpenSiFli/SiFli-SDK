/**
  ******************************************************************************
  * @file   bf0_ble_tipc.h
  * @author Sifli software development team
  * @brief Header file - Sibles time profile definition.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#ifndef __BF0_BLE_TIPC_H
#define __BF0_BLE_TIPC_H

#include "bf0_ble_common.h"


enum ble_tipc_event
{
    BLE_TIPC_CURRENT_TIME_NOTIFY = BLE_TIP_TYPE,
    BLE_TIPC_READ_CURRENT_TIME_RSP,
    BLE_TIPC_READ_LOCAL_INFO_RSP,
};

/// Time profile information
typedef struct
{
    /// year time element
    uint16_t year;
    /// month time element
    uint8_t month;
    /// day time element
    uint8_t day;
    /// hour time element
    uint8_t hour;
    /// minute time element
    uint8_t min;
    /// second time element
    uint8_t sec;
} ble_tipc_date_time_t;


///Current Time Characteristic Structure
typedef struct
{
    /// Date time
    ble_tipc_date_time_t date_time;
    /// Day of the week
    uint8_t day_of_week;
    /// 1/256th of a second
    uint8_t fraction_256;
    /// Adjust reason. Bit 0: Manual time update. Bit 1: External reference time update. Bit 2: Change of timezone.
    /// Bit 3: Change of DST.
    uint8_t adjust_reason;
} ble_tipc_curr_time_t;

///Local Time Info Characteristic Structure - UUID:0x2A0F
typedef struct
{
    /// 15 minutes increments between local time and UTC. Valid range is -48-56 and -128 means unknown.
    uint8_t time_zone;
    /// DaySaving Time. 0: Standard time. 2: +0.5h. 4: +1h. 8: +2h. 255: unknown. Others are reserved.
    uint8_t dst_offset;
} ble_tip_local_time_info_t;


void ble_tipc_init(uint8_t enable);

int8_t ble_tipc_read_current_time(uint8_t conn_idx);

int8_t ble_tipc_read_local_time_info(uint8_t conn_idx);

int8_t ble_tipc_enable(uint8_t conn_idx);

#endif //__BF0_BLE_TIPC_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
