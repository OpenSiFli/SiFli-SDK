/**
  ******************************************************************************
  * @file   bf0_ble_hrpc.c
  * @author Sifli software development team
  * @brief  Sibles heart rate profile collector.
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
#include "bf0_ble_hrpc.h"
#include "bf0_ble_gap.h"
#include "os_adaptor.h"


#ifdef BSP_BLE_HRPC
#define LOG_TAG "hrp_srv"
#include "log.h"

enum ble_hrpc_state_t
{
    BLE_HRPC_STATE_IDLE,
    BLE_HRPC_STATE_READY,
    BLE_HRPC_STATE_BUSY
};
typedef struct
{
    uint16_t hdl_start;
    uint16_t hdl_end;
} ble_hrpc_svc_t;
typedef struct
{
    uint16_t attr_hdl;
    uint16_t value_hdl;
    uint16_t cccd_hdl;
    uint8_t prop;
    uint8_t enabled;
} ble_hrpc_char_t;
typedef struct
{
    uint8_t state;
    uint8_t is_enable;
    uint8_t conn_idx;
    uint16_t remote_handle;
    ble_hrpc_svc_t svc;
    ble_hrpc_char_t heart_rate_char;
    ble_hrpc_char_t body_char;
    ble_hrpc_char_t cntl_char;
} ble_hrpc_env_t;
static ble_hrpc_env_t g_ble_hrpc_env;
static ble_hrpc_env_t *ble_hrpc_get_env(void)
{
    return &g_ble_hrpc_env;
};

static const char *body_location[] =
{
    [HR_OTHER] = "OTHER",
    [HR_CHEST] = "CHEST",
    [HR_WRIST] = "WRIST",
    [HR_FINGER] = "FINGER",
    [HR_HAND] = "HAND",
    [HR_EARLOBE] = "EARLOBE",
    [HR_FOOT] = "FOOT",
};

static uint16_t co_read16p(void const *ptr16)
{
    uint16_t value = ((uint8_t const volatile *)ptr16)[0] | ((uint8_t const volatile *)ptr16)[1] << 8;
    return value;
}

static uint8_t ble_hrpc_unpack_heart_rate(uint16_t event, uint8_t *data, uint8_t len)
{
    int8_t cursor = 0;
    int8_t i = 0;
    ble_hrpc_heart_rate_t res;

    // Heart Rate measurement flags
    res.flags = data[0];
    cursor += 1;

    if ((res.flags & HRS_FLAG_HR_16BITS_VALUE) == HRS_FLAG_HR_16BITS_VALUE)
    {
        // Heart Rate Measurement Value 16 bits
        res.heart_rate = co_read16p(&data[cursor]);
        cursor += 2;
    }
    else
    {
        // Heart Rate Measurement Value 8 bits
        res.heart_rate = (uint16_t) data[cursor];
        cursor += 1;
    }

    if ((res.flags & HRS_FLAG_ENERGY_EXPENDED_PRESENT) == HRS_FLAG_ENERGY_EXPENDED_PRESENT)
    {
        // Energy Expended present
        res.energy_expended = co_read16p(&data[cursor]);
        cursor += 2;
        // Energy Expended not present
    }

    // retrieve number of rr intervals
    res.nb_rr_interval = ((len - cursor) > 0 ? ((len - cursor) / 2) : (0));

    for (i = 0 ; (i < (res.nb_rr_interval)) && (i < (HRS_MAX_RR_INTERVAL)) ; i++)
    {
        // RR-Intervals
        res.rr_intervals[i] = co_read16p(&data[cursor]);
        cursor += 2;
    }
    ble_event_publish(event, &res, sizeof(ble_hrpc_heart_rate_t));

    return 0;
}

static uint8_t ble_hrpc_unpack_body_sensor_location(uint16_t event, uint8_t *data)
{

    ble_hrpc_body_sensor_location_t res;
    res.location = data[0];
    ble_event_publish(event, &res, sizeof(ble_hrpc_body_sensor_location_t));
    return 0;
}


int8_t ble_hrpc_enable(uint8_t conn_idx)
{
    ble_hrpc_env_t *env = ble_hrpc_get_env();
    uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_HEART_RATE);
    int8_t ret = sibles_search_service(conn_idx, ATT_UUID_16_LEN, (uint8_t *)&svc_uuid);
    if (ret == 0)
        env->state = BLE_HRPC_STATE_BUSY;
    return ret;
}

int8_t ble_hrpc_enable_read_heart_rate(uint8_t conn_idx, uint8_t enable)
{

    sibles_write_remote_value_t value;
    ble_hrpc_env_t *env = ble_hrpc_get_env();
    value.handle = env->heart_rate_char.cccd_hdl;
    value.write_type = SIBLES_WRITE;
    uint16_t enable_hr = (uint16_t)enable;
    value.len = 2;
    value.value = (uint8_t *)&enable_hr;
    int8_t ret = sibles_write_remote_value(env->remote_handle, conn_idx, &value);

    return ret;
}

int8_t ble_hrpc_read_body_sensor_location(uint8_t conn_idx)
{
    int8_t ret;
    ble_hrpc_env_t *env = ble_hrpc_get_env();
    if ((env->body_char.prop & (1 << 1)) == 0)
    {
        LOG_I("remote not support body sensor location characteristic!");
        return -1;
    }
    sibles_read_remote_value_req_t value;
    value.read_type = SIBLES_READ;
    value.handle = env->body_char.value_hdl;
    value.length = 0;
    value.offset = 0;
    ret = sibles_read_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}

int8_t ble_hrpc_write_heart_rate_cntl_point(uint8_t conn_idx)
{
    ble_hrpc_env_t *env = ble_hrpc_get_env();
    int8_t ret;
    if ((env->cntl_char.prop & (1 << 3)) == 0)
    {
        LOG_I("remote not support heart rate control point characteristic!");
        return -1;
    }
    sibles_write_remote_value_t value;
    value.write_type = SIBLES_WRITE;
    value.handle = env->cntl_char.value_hdl;
    value.len = 1;
    uint8_t data = 1;
    value.value = (uint8_t *)&data;
    ret = sibles_write_remote_value(env->remote_handle, conn_idx, &value);
    return ret;
}

int hrpc(int argc, char *argv[])
{
    ble_hrpc_env_t *env = ble_hrpc_get_env();
    if (strcmp(argv[1], "notify") == 0)
    {
        if (strcmp(argv[2], "enable") == 0)
        {
            uint8_t enable = atoi(argv[3]);
            ble_hrpc_enable_read_heart_rate(env->conn_idx, enable);
        }
    }
    else if (strcmp(argv[1], "body") == 0)
    {
        ble_hrpc_read_body_sensor_location(env->conn_idx);
    }
    else if (strcmp(argv[1], "cntl") == 0)
    {
        ble_hrpc_write_heart_rate_cntl_point(env->conn_idx);
    }

    return 0;
}
#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(hrpc, "hrpc_cmd");
#endif

int ble_hrpc_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    ble_hrpc_env_t *env = ble_hrpc_get_env();
    LOG_I("hrpc gattc event handler %d\r\n", event_id);
    int8_t res;

    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        LOG_I("hrp register ret %d\r\n", rsp->status);

        //enable heart rate measurement notify
        ble_hrpc_enable_read_heart_rate(rsp->conn_idx, true);

        OS_ASSERT(res == SIBLES_WRITE_NO_ERR);

        env->state = BLE_HRPC_STATE_READY;
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        LOG_I("SIBLES_REMOTE_EVENT_IND-->hrpc handle:%d\n", ind->handle);
        if (ind->handle == env->heart_rate_char.value_hdl)
        {
            ble_hrpc_unpack_heart_rate(BLE_HRPC_HREAT_RATE_NOTIFY, ind->value, ind->length);
        }

        // Notify upper layer
        break;
    }
    case SIBLES_READ_REMOTE_VALUE_RSP:
    {
        sibles_read_remote_value_rsp_t *rsp = (sibles_read_remote_value_rsp_t *)data;
        LOG_I("SIBLES_READ_REMOTE_VALUE_RSP-->hrpc handle:%d\n", rsp->handle);
        if (rsp->handle == env->body_char.value_hdl)
        {
            ble_hrpc_unpack_body_sensor_location(BLE_HRPC_READ_BODY_SENSOR_LOCATION_RSP, rsp->value);
        }
        break;
    }
    default:
        break;
    }
    return 0;
}


int ble_hrpc_ble_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_hrpc_env_t *env = ble_hrpc_get_env();
    switch (event_id)
    {
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;
        if (rsp->result != HL_ERR_NO_ERROR)
            break; // Do nothing
        uint16_t svc_uuid = ATT_UUID_16(ATT_SVC_HEART_RATE);

        uint16_t heart_rate_char_uuid = ATT_UUID_16(ATT_CHAR_HEART_RATE_MEAS);
        uint16_t body_char_uuid = ATT_UUID_16(ATT_CHAR_BODY_SENSOR_LOCATION);
        uint16_t cntl_char_uuid = ATT_UUID_16(ATT_CHAR_HEART_RATE_CNTL_POINT);

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
            if (!memcmp(chara->uuid, &heart_rate_char_uuid, chara->uuid_len))
            {
                LOG_I("heart_rate received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                RT_ASSERT(chara->desc_count == 1);
                env->heart_rate_char.attr_hdl = chara->attr_hdl;
                env->heart_rate_char.value_hdl = chara->pointer_hdl;
                env->heart_rate_char.prop = chara->prop;
                env->heart_rate_char.cccd_hdl = chara->desc[0].attr_hdl;
                env->heart_rate_char.enabled = 1;
            }
            else if (!memcmp(chara->uuid, &body_char_uuid, chara->uuid_len))
            {
                LOG_I("body_location received, att handle(%x)", chara->attr_hdl);
                env->body_char.attr_hdl = chara->attr_hdl;
                env->body_char.value_hdl = chara->pointer_hdl;
                env->body_char.prop = chara->prop;
                env->body_char.enabled = 1;
            }
            else if (!memcmp(chara->uuid, &cntl_char_uuid, chara->uuid_len))
            {
                LOG_I("heart_rate cntl_point received, att handle(%x)", chara->attr_hdl);
                env->cntl_char.attr_hdl = chara->attr_hdl;
                env->cntl_char.value_hdl = chara->pointer_hdl;
                env->cntl_char.prop = chara->prop;
                env->cntl_char.enabled = 1;
            }

            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }
        //register first
        env->remote_handle = sibles_register_remote_svc(rsp->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_hrpc_gattc_event_handler);
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
            ble_hrpc_enable(evt->conn_idx);
        break;
    }
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        if (env->is_enable)
            ble_hrpc_enable(ind->conn_idx);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        if (env->conn_idx == ind->conn_idx)
            sibles_unregister_remote_svc(env->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_hrpc_gattc_event_handler);
        break;
    }
    default:
        break;
    }
    return 0;
}

void ble_hrpc_init(uint8_t enable)
{
    ble_hrpc_env_t *env = ble_hrpc_get_env();
    env->is_enable = enable;
    LOG_I("ble_hrpc_init: hrpc enable %d\n", enable);
}

BLE_EVENT_REGISTER(ble_hrpc_ble_event_handler, (uint32_t)NULL);

#endif //BSP_BLE_HRPC

