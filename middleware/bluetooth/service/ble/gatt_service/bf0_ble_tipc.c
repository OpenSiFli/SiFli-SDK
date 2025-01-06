/**
  ******************************************************************************
  * @file   bf0_ble_tipc.c
  * @author Sifli software development team
  * @brief Header file - Sibles time profile implmentation.
 *
  ******************************************************************************
*/
/**
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

#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles.h"
#include "att.h"
#include "bf0_ble_tipc.h"
#include "bf0_ble_gap.h"
#include "os_adaptor.h"



#ifdef BSP_BLE_TIMEC
#ifndef BSP_USING_PC_SIMULATOR
    #include "rtc.h"
#endif
#define LOG_TAG "tip_srv"
#include "log.h"


typedef struct
{
    uint16_t hdl_start;
    uint16_t hdl_end;
} ble_tipc_svc_t;

typedef struct
{
    uint16_t attr_hdl;
    uint16_t value_hdl;
    uint16_t cccd_hdl;
    uint8_t prop;
    uint8_t enabled;
} ble_tipc_char_t;

enum ble_tipc_state_t
{
    BLE_TIPC_STATE_IDLE,
    BLE_TIPC_STATE_READY,
    BLE_TIPC_STATE_BUSY
};


typedef struct
{
    uint8_t state;
    uint8_t is_enable;
    uint8_t conn_idx;
    uint16_t remote_handle;
    ble_tipc_svc_t svc;
    ble_tipc_char_t ct_char;
    ble_tipc_char_t local_info;
} ble_tipc_env_t;

static ble_tipc_env_t g_ble_tipc_env;
static ble_tipc_env_t *ble_tipc_get_env(void)
{
    return &g_ble_tipc_env;
};

static uint8_t ble_tipc_unpack_date_time(uint8_t *packed_date, ble_tipc_date_time_t *date_time)
{
    date_time->year = (uint16_t)(packed_date[0] | (uint16_t)(packed_date[1] << 8));
    date_time->month = packed_date[2];
    date_time->day   = packed_date[3];
    date_time->hour  = packed_date[4];
    date_time->min   = packed_date[5];
    date_time->sec   = packed_date[6];

    return 7;
}

static void ble_tipc_current_time_hanlder(uint8_t event, uint8_t *data)
{
    ble_tipc_curr_time_t time;
    //Date-Time
    ble_tipc_unpack_date_time(data, &(time.date_time));

    //Day of Week
    time.day_of_week = data[7];

    //Fraction 256
    time.fraction_256 = data[8];

    //Adjust Reason
    time.adjust_reason = data[9];

    //rtc set will be done in app.
    //set_date((uint32_t)time.date_time.year, (uint32_t)time.date_time.month, (uint32_t)time.date_time.day);
    //set_time((uint32_t)time.date_time.hour, (uint32_t)time.date_time.min, (uint32_t)time.date_time.sec);

    ble_event_publish(event, &time, sizeof(ble_tipc_curr_time_t));

}


static void ble_tipc_local_info_handler(uint8_t *data)
{
    ble_tip_local_time_info_t local_info;
    local_info.time_zone = data[0];
    local_info.dst_offset = data[1];

    ble_event_publish(BLE_TIPC_READ_LOCAL_INFO_RSP, &local_info, sizeof(ble_tip_local_time_info_t));
}



int8_t ble_tipc_enable(uint8_t conn_idx)
{
    ble_tipc_env_t *env = ble_tipc_get_env();
    uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_CURRENT_TIME);
    int8_t ret = sibles_search_service(conn_idx, ATT_UUID_16_LEN, (uint8_t *)&svc_uuid);
    if (ret == 0)
        env->state = BLE_TIPC_STATE_BUSY;
    return ret;
}

int8_t ble_tipc_read_current_time(uint8_t conn_idx)
{
    int8_t ret;
    ble_tipc_env_t *env = ble_tipc_get_env();
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;
    value.handle = env->ct_char.value_hdl;
    value.length = 0;
    value.offset = 0;
    ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}

int8_t ble_tipc_read_local_time_info(uint8_t conn_idx)
{
    int8_t ret;
    ble_tipc_env_t *env = ble_tipc_get_env();
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;
    value.handle = env->local_info.value_hdl;
    value.length = 0;
    value.offset = 0;
    ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}

int ble_tipc_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    ble_tipc_env_t *env = ble_tipc_get_env();
    LOG_I("tipc gattc event handler %d\r\n", event_id);
    int8_t res;

    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        LOG_I("ct register ret %d\r\n", rsp->status);
        sibles_write_remote_value_t value;
        uint16_t enable = 1;
        value.handle = env->ct_char.cccd_hdl;
        value.write_type = SIBLES_WRITE;
        value.len = 2;
        value.value = (uint8_t *)&enable;
        res = sibles_write_remote_value(env->remote_handle, rsp->conn_idx, &value);

        OS_ASSERT(res == SIBLES_WRITE_NO_ERR);

        env->state = BLE_TIPC_STATE_READY;
        // Get tim directly to update to RTC.
        ble_tipc_read_current_time(env->conn_idx);
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        LOG_I("tipc handle:%d", ind->handle);
        if (ind->handle == env->ct_char.value_hdl)
        {
            ble_tipc_current_time_hanlder(BLE_TIPC_CURRENT_TIME_NOTIFY, ind->value);
        }

        // Notify upper layer
        break;
    }
    case SIBLES_READ_REMOTE_VALUE_RSP:
    {
        sibles_read_remote_value_rsp_t *rsp = (sibles_read_remote_value_rsp_t *)data;
        if (rsp->handle == env->ct_char.value_hdl)
        {
            ble_tipc_current_time_hanlder(BLE_TIPC_READ_CURRENT_TIME_RSP, rsp->value);
        }
        else if (rsp->handle == env->local_info.value_hdl)
        {
            ble_tipc_local_info_handler(rsp->value);
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

int ble_tipc_ble_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_tipc_env_t *env = ble_tipc_get_env();
    switch (event_id)
    {
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;
        if (rsp->result != HL_ERR_NO_ERROR)
            break; // Do nothing
        uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_CURRENT_TIME);
        uint16_t ct_char_uuid = ATT_UUID_16(ATT_CHAR_CT_TIME);
        uint16_t loc_char_uuid = ATT_UUID_16(ATT_CHAR_LOCAL_TIME_INFO);

        // rsp->svc may null
        if (memcmp(rsp->search_uuid, &svc_uuid, rsp->search_svc_len) != 0)
            break;

        env->svc.hdl_start = rsp->svc->hdl_start;
        env->svc.hdl_end = rsp->svc->hdl_end;
        uint32_t i;
        uint16_t offset = 0;
        sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
        for (i = 0; i < rsp->svc->char_count; i++)
        {
            if (!memcmp(chara->uuid, &ct_char_uuid, chara->uuid_len))
            {
                LOG_I("ct time received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                RT_ASSERT(chara->desc_count == 1);
                env->ct_char.attr_hdl = chara->attr_hdl;
                env->ct_char.value_hdl = chara->pointer_hdl;
                env->ct_char.prop = chara->prop;
                env->ct_char.cccd_hdl = chara->desc[0].attr_hdl;
                env->ct_char.enabled = 1;
            }
            else if (!memcmp(chara->uuid, &loc_char_uuid, chara->uuid_len))
            {
                LOG_I("local time received, att handle(%x)", chara->attr_hdl);
                env->local_info.attr_hdl = chara->attr_hdl;
                env->local_info.value_hdl = chara->pointer_hdl;
                env->local_info.prop = chara->prop;
                env->ct_char.enabled = 1;
            }

            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }
        //register first
        env->remote_handle = sibles_register_remote_svc(rsp->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_tipc_gattc_event_handler);
        // subscribe data src. then subscribe notfi src.
        break;
    }
    case SIBLES_REMOTE_CONNECTED_IND:
    {
        sibles_remote_connected_ind_t *ind = (sibles_remote_connected_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        break;
    }
    case BLE_GAP_BOND_IND:
    {
        ble_gap_bond_ind_t *evt = (ble_gap_bond_ind_t *)data;
        if (evt->info == GAPC_PAIRING_SUCCEED && env->is_enable)
            ble_tipc_enable(evt->conn_idx);
        break;
    }
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        if (env->is_enable)
            ble_tipc_enable(ind->conn_idx);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        if (env->conn_idx == ind->conn_idx)
            sibles_unregister_remote_svc(env->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_tipc_gattc_event_handler);
        break;
    }
    default:
        break;
    }
    return 0;
}

void ble_tipc_init(uint8_t enable)
{
    ble_tipc_env_t *env = ble_tipc_get_env();
    env->is_enable = enable;
    LOG_D("ble_tipc_init: tpic enable %d\n", enable);
}

BLE_EVENT_REGISTER(ble_tipc_ble_event_handler, (uint32_t)NULL);

#endif //BSP_BLE_TIMEC



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
