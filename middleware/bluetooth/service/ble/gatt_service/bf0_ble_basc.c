/**
  ******************************************************************************
  * @file   bf0_ble_basc.c
  * @author Sifli software development team
  * @brief  Sibles battery service collector.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2024 - 2024,  Sifli Technology
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
#include "bf0_ble_basc.h"
#include "bf0_ble_gap.h"
#include "os_adaptor.h"

#ifdef BSP_BLE_BASC
#define LOG_TAG "bas_srv"
#include "log.h"

enum ble_basc_state_t
{
    BLE_BASC_STATE_IDLE,
    BLE_BASC_STATE_READY,
    BLE_BASC_STATE_BUSY
};
typedef struct
{
    uint16_t hdl_start;
    uint16_t hdl_end;
} ble_basc_svc_t;
typedef struct
{
    uint16_t attr_hdl;
    uint16_t value_hdl;
    uint16_t cccd_hdl;
    uint8_t prop;
    uint8_t enabled;
} ble_basc_char_t;
typedef struct
{
    uint8_t state;
    uint8_t is_enable;
    uint8_t conn_idx;
    uint16_t remote_handle;
    ble_basc_svc_t svc;
    ble_basc_char_t bat_lev;
} ble_basc_env_t;
static ble_basc_env_t g_ble_basc_env;
static ble_basc_env_t *ble_basc_get_env(void)
{
    return &g_ble_basc_env;
};

int8_t ble_basc_enable(uint8_t conn_idx)
{
    ble_basc_env_t *env = ble_basc_get_env();
    uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_BATTERY_SERVICE);
    int8_t ret = sibles_search_service(conn_idx, ATT_UUID_16_LEN, (uint8_t *)&svc_uuid);
    if (ret == 0)
        env->state = BLE_BASC_STATE_BUSY;
    return ret;
}

int8_t ble_basc_enable_bat_lev_notify(uint8_t conn_idx, uint8_t enable)
{
    sibles_write_remote_value_t value;
    ble_basc_env_t *env = ble_basc_get_env();
    if ((env->bat_lev.prop & (1 << 4)) == 0)
    {
        LOG_I("remote not support battery level notify!");
        return -1;
    }
    value.handle = env->bat_lev.cccd_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = 2;
    value.value = (uint8_t *)&enable;
    int8_t ret = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);

    return ret;
}

int8_t ble_basc_read_battery_level(uint8_t conn_idx)
{
    ble_basc_env_t *env = ble_basc_get_env();
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;
    value.handle = env->bat_lev.value_hdl;
    value.length = 0;
    value.offset = 0;
    int8_t ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}

int basc(int argc, char *argv[])
{
    ble_basc_env_t *env = ble_basc_get_env();
    if (strcmp(argv[1], "read") == 0)
    {
        ble_basc_read_battery_level(env->conn_idx);
    }
    else if (strcmp(argv[1], "notify") == 0)
    {
        if (strcmp(argv[2], "enable") == 0)
        {
            uint8_t enable = atoi(argv[3]);
            ble_basc_enable_bat_lev_notify(env->conn_idx, enable);
        }
    }
    return 0;
}
#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(basc, "basc_cmd");
#endif

int ble_basc_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    ble_basc_env_t *env = ble_basc_get_env();
    LOG_I("basc gattc event handler %d\r\n", event_id);
    int8_t res;

    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        LOG_I("bas register ret %d\r\n", rsp->status);

        //enable battery level notify
        ble_basc_enable_bat_lev_notify(rsp->conn_idx, true);

        OS_ASSERT(res == SIBLES_WRITE_NO_ERR);

        env->state = BLE_BASC_STATE_READY;
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        LOG_I("SIBLES_REMOTE_EVENT_IND-->basc handle:%d", ind->handle);
        if (ind->handle == env->bat_lev.value_hdl)
        {
            ble_basc_bat_lev_t res;
            res.lev = ind->value[0];
            ble_event_publish(BLE_BASC_BAT_LEV_NOTIFY, &res, sizeof(ble_basc_bat_lev_t));
        }
        // Notify upper layer
        break;
    }
    case SIBLES_READ_REMOTE_VALUE_RSP:
    {
        sibles_read_remote_value_rsp_t *rsp = (sibles_read_remote_value_rsp_t *)data;
        LOG_I("SIBLES_READ_REMOTE_VALUE_RSP-->basc handle:%d", rsp->handle);
        if (rsp->handle == env->bat_lev.value_hdl)
        {
            ble_basc_bat_lev_t res;
            res.lev = rsp->value[0];
            ble_event_publish(BLE_BASC_READ_BAT_LEV_RSP, &res, sizeof(ble_basc_bat_lev_t));
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

int ble_basc_ble_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_basc_env_t *env = ble_basc_get_env();
    switch (event_id)
    {
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;
        if (rsp->result != HL_ERR_NO_ERROR)
            break; // Do nothing
        uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_BATTERY_SERVICE);
        uint16_t bat_lev_char_uuid = ATT_UUID_16(ATT_CHAR_BATTERY_LEVEL);


#ifdef SOC_SF32LB55X
        if (!(rsp->svc->uuid_len == ATT_UUID_16_LEN && !memcmp(rsp->svc->uuid, &svc_uuid, rsp->svc->uuid_len)))
        {
            // Not expected uuid
            break;
        }
#else
        // rsp->svc may null
        if (memcmp(rsp->search_uuid, &svc_uuid, rsp->search_svc_len) != 0)
            break;
#endif

        env->svc.hdl_start = rsp->svc->hdl_start;
        env->svc.hdl_end = rsp->svc->hdl_end;
        uint32_t i;
        uint16_t offset = 0;
        sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
        for (i = 0; i < rsp->svc->char_count; i++)
        {
            if (!memcmp(chara->uuid, &bat_lev_char_uuid, chara->uuid_len))
            {
                LOG_I("bat_lev received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                env->bat_lev.attr_hdl = chara->attr_hdl;
                env->bat_lev.value_hdl = chara->pointer_hdl;
                env->bat_lev.prop = chara->prop;
                if (chara->desc_count == 1)
                {
                    env->bat_lev.cccd_hdl = chara->desc[0].attr_hdl;
                }
                env->bat_lev.enabled = 1;
            }

            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }
        //register first
        env->remote_handle = sibles_register_remote_svc(rsp->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_basc_gattc_event_handler);
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
            ble_basc_enable(evt->conn_idx);
        break;
    }
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        if (env->is_enable)
            ble_basc_enable(ind->conn_idx);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        if (env->conn_idx == ind->conn_idx)
            sibles_unregister_remote_svc(env->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_basc_gattc_event_handler);
        break;
    }
    default:
        break;
    }
    return 0;
}

void ble_basc_init(uint8_t enable)
{
    ble_basc_env_t *env = ble_basc_get_env();
    env->is_enable = enable;
    LOG_I("ble_basc_init: basc enable %d\n", enable);
}

BLE_EVENT_REGISTER(ble_basc_ble_event_handler, (uint32_t)NULL);

#endif //BSP_BLE_BASC

