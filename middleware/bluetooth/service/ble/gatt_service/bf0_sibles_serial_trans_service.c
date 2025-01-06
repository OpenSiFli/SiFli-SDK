/**
  ******************************************************************************
  * @file   bf0_sibles_serial_trans_service.c
  * @author Sifli software development team
  * @brief Sibles serial transfer service source.
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
#include <stdlib.h>
#include "os_adaptor.h"
#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_serial_trans_service.h"
#include "att.h"
#include "section.h"
#include "ble_connection_manager.h"

#define LOG_TAG "sibles"
#include "log.h"

#define BLE_UART_RETRY      3
#define BLE_UART_TXTIMEOUT  500

#ifdef BSP_BLE_SERIAL_TRANSMISSION


enum ble_serial_tran_state
{
    BLE_SERIAL_TRAN_IDLE,
    BLE_SERIAL_TRAN_READY
};

enum ble_serial_tran_att_list
{
    BLE_SERIAL_TRAN_SVC,
    BLE_SERIAL_TRAN_CONFIGURE_CHAR,
    BLE_SERIAL_TRAN_CONFIGURE_VALUE,
    BLE_SERIAL_TRAN_DATA_CHAR,
    BLE_SERIAL_TRAN_DATA_VALUE,
    BLE_SERIAL_TRAN_DATA_CCCD,
    BLE_SERIAL_TRAN_ATT_NB
};


typedef struct
{
    uint8_t *ptr;
    uint16_t len;
    uint16_t offset;
    uint8_t id;
} ble_serial_tran_assemable_t;

typedef struct
{
    uint16_t cccd_enable;
    uint8_t cb_count;
    uint8_t is_assemable;
    uint16_t mtu;
    ble_serial_tran_export_t *cb_table;
    ble_serial_tran_assemable_t assemable;
} ble_serial_tran_env_t;


static ble_serial_tran_env_t g_serial_tran_env;
static uint8_t g_serial_tran_svc[ATT_UUID_128_LEN] = serial_tran_svc_uuid;
static sibles_hdl g_serial_tran_hdl;

#define SERIAL_UUID_16(x) {((uint8_t)(x&0xff)),((uint8_t)(x>>8))}


struct attm_desc_128 serial_trans_att_db[] =
{
    [BLE_SERIAL_TRAN_SVC] = {SERIAL_UUID_16(ATT_DECL_PRIMARY_SERVICE), PERM(RD, ENABLE), 0, 0},
    [BLE_SERIAL_TRAN_CONFIGURE_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC), PERM(RD, ENABLE), 0, 0},
    [BLE_SERIAL_TRAN_CONFIGURE_VALUE] = {
        serial_tran_configure_uuid, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WRITE_COMMAND, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE),
        1024
    },
    [BLE_SERIAL_TRAN_DATA_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC), PERM(RD, ENABLE), 0, 0},
    [BLE_SERIAL_TRAN_DATA_VALUE] = {
        serial_tran_data_uuid, PERM(RD, ENABLE) | PERM(NTF, ENABLE) | PERM(WRITE_COMMAND, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE),
        1024
    },
    [BLE_SERIAL_TRAN_DATA_CCCD] = {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), 2},

};

static ble_serial_tran_env_t *ble_serial_tran_get_env(void)
{
    return &g_serial_tran_env;
}


static void ble_serial_callback_event_notify(ble_serial_tran_event_t event, uint8_t *data)
{
    ble_serial_tran_env_t *env = ble_serial_tran_get_env();

    if (!data)
        return;
    uint32_t i;
    ble_serial_tran_export_t *tb = env->cb_table;
    for (i = 0; i < env->cb_count; i++)
    {
        LOG_D("tb cb:%x,id:%d", tb->callback, tb->cate_id);
        if (tb->callback)
            tb->callback(event, data);
        tb++;
    }
}

static void ble_serial_callback_error_notify(uint8_t conn_idx, uint8_t cate_id, uint8_t error)
{
    ble_serial_tran_env_t *env = ble_serial_tran_get_env();
    ble_serial_tran_export_t *tb = env->cb_table;

    uint32_t i;
    for (i = 0; i < env->cb_count; i++)
    {
        if (cate_id == tb->cate_id)
        {
            LOG_W("send error notify error:%x,id:%d", error, tb->cate_id);
            ble_serial_tran_error_t data;
            data.handle = conn_idx;
            data.cate_id = cate_id;
            data.error = error;
            tb->callback(BLE_SERIAL_TRAN_ERROR, (uint8_t *)&data);
            break;
        }
        tb++;
    }
}

static void ble_serial_callback_data_notify(uint8_t conn_idx, uint8_t cate_id, uint16_t len, uint8_t *value)
{
    ble_serial_tran_env_t *env = ble_serial_tran_get_env();

    if (!value)
        return;
    uint32_t i;
    ble_serial_tran_export_t *tb = env->cb_table;
    LOG_D("cb_id %d", cate_id);
    // value + 1 is fragment flag
    for (i = 0; i < env->cb_count; i++)
    {
        LOG_D("tb cb:%x,id:%d", tb->callback, tb->cate_id);
        if (cate_id == tb->cate_id)
        {
            ble_serial_tran_data_t data;
            data.handle = conn_idx;
            data.cate_id = cate_id;
            data.len = len;
            data.data = value;
            tb->callback(BLE_SERIAL_TRAN_DATA, (uint8_t *)&data);
            break;
        }
        tb++;
    }
}

static void ble_serial_clean_assemble()
{
    LOG_I("ble_serial_clean_assemble");
    ble_serial_tran_env_t *env = ble_serial_tran_get_env();
    env->is_assemable = 0;
    if (env->assemable.ptr)
    {
        bt_mem_free(env->assemable.ptr);
    }
    memset(&env->assemable, 0, sizeof(ble_serial_tran_assemable_t));
}

uint8_t *ble_serial_tran_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    ble_serial_tran_env_t *env = ble_serial_tran_get_env();
    switch (idx)
    {
    case BLE_SERIAL_TRAN_DATA_CCCD:
        *len = sizeof(env->cccd_enable);
        return (uint8_t *)&env->cccd_enable;
        break;
    default:
        *len = 1;
    }
    return 0;
}
uint8_t ble_serial_tran_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    ble_serial_tran_env_t *env = ble_serial_tran_get_env();
    //LOG_D("serial set cbk idx(%d), len(%d)", para->idx, para->len);
    switch (para->idx)
    {
    case BLE_SERIAL_TRAN_DATA_CCCD:
    {
        LOG_I("cccd enable %d", *(para->value));
        env->cccd_enable = *(para->value);

        break;
    }
    case BLE_SERIAL_TRAN_DATA_VALUE:
    {
        // should get the conn_idx from parameter
        // notify upper layer
        uint8_t frag_flag = *(para->value + 1);
        uint8_t cate_id = *(para->value);
        if (frag_flag != 0 && !env->is_assemable)
        {
            if (frag_flag != 1)
            {
                // only assemable when flag is 1
                break;
            }
            // start assemable
            env->is_assemable = 1;
            memcpy(&env->assemable.len, para->value + 2, sizeof(uint16_t));
            env->assemable.ptr = bt_mem_alloc(env->assemable.len);
            env->assemable.id = cate_id;
            // OS_ASSERT(env->assemable.ptr);
            if (!env->assemable.ptr)
            {
                ble_serial_callback_error_notify(conn_idx, env->assemable.id, BLE_SERIAL_TRAN_ERROR_LACK_OF_MEMORY);
                ble_serial_clean_assemble();
                break;
            }

            memcpy(env->assemable.ptr, para->value + 4, para->len - 4);
            env->assemable.offset = para->len - 4;
            break;
        }
        else if (frag_flag != 0 && env->is_assemable)
        {
            if (cate_id != env->assemable.id || !env->assemable.ptr)
            {
                //Just handle one uuid asseamblly.
                // OS_ASSERT(0);
                ble_serial_callback_error_notify(conn_idx, env->assemable.id, BLE_SERIAL_TRAN_ERROR_CATE_ID);
                ble_serial_clean_assemble();
                break;
            }
#ifdef DFU_OTA_MANAGER
            if (env->assemable.offset + (para->len - 2) > env->assemable.len)
            {
                ble_serial_clean_assemble();
                break;
            }
#else
            if ((env->assemable.offset + (para->len - 2)) > env->assemable.len)
            {
                LOG_E("assemable over len %d, %d", env->assemable.offset + (para->len - 2), env->assemable.len);
                ble_serial_callback_error_notify(conn_idx, env->assemable.id, BLE_SERIAL_TRAN_ERROR_ASSEMBLE_OVER_LEN);
                ble_serial_clean_assemble();
                break;
            }
#endif
            // assemabling
            memcpy(env->assemable.ptr + env->assemable.offset, para->value + 2, para->len - 2);
            env->assemable.offset += para->len - 2;
            if (frag_flag == 3)
            {
#ifdef DFU_OTA_MANAGER
                if (env->assemable.offset != env->assemable.len)
                {
                    ble_serial_clean_assemble();
                    break;
                }
#else
                if (env->assemable.offset != env->assemable.len)
                {
                    LOG_E("assemable error len %d, %d", env->assemable.offset, env->assemable.len);
                    ble_serial_callback_error_notify(conn_idx, env->assemable.id, BLE_SERIAL_TRAN_ERROR_ASSEMBLE_ERROR);
                    ble_serial_clean_assemble();
                    break;
                }
#endif
                ble_serial_callback_data_notify(conn_idx, env->assemable.id, env->assemable.len, env->assemable.ptr);
                bt_mem_free(env->assemable.ptr);
                memset(&env->assemable, 0, sizeof(ble_serial_tran_assemable_t));
                env->is_assemable = 0;
            }
            break;
        }
        else if (frag_flag == 0)
        {
            if (env->is_assemable)
            {
                if (env->assemable.id != cate_id)
                {
                    //just notify user
                }
                else
                {
                    // discard last packet
                    env->is_assemable = 0;
                    if (env->assemable.ptr)
                        bt_mem_free(env->assemable.ptr);
                    memset(&env->assemable, 0, sizeof(ble_serial_tran_assemable_t));
                }
            }
        }
        ble_serial_callback_data_notify(conn_idx, cate_id, para->len - 4, para->value + 4);
        break;
    }
    }
    return 0;
}

static uint32_t ble_serial_wait_time_get(uint8_t conn_idx)
{
    cm_conneciont_parameter_value_t con_para;
    uint32_t wait_time;
    uint8_t para_ret = connection_manager_get_connetion_parameter(conn_idx, (uint8_t *)&con_para);
    if (para_ret == CM_STATUS_OK && con_para.interval != 0)
    {
        wait_time = con_para.interval * 1.25f;
    }
    else
    {
        wait_time = 50;
    }
    return wait_time;
}

int ble_serial_tran_send_data(ble_serial_tran_data_t *data)
{
    sibles_value_t value;
    int ret;
    uint32_t wait_time;
    ble_serial_tran_env_t *env = ble_serial_tran_get_env();
    uint8_t *packet ;

    if (data == NULL || data->data == NULL)
        return -1;                                  // Parameter error;
    else if (!g_serial_tran_hdl)
        return -2;                                  // Not ready

    if (data->len <= env->mtu - 3 - 4)
    {
        if ((packet = bt_mem_alloc(data->len + 4)) == NULL)
            return -3;                                  // No enough memory
        *packet = data->cate_id;                        // Add cateID to packet
        *(packet + 1) = 0;                              // first not consider fragment
        memcpy(packet + 2, &data->len, 2);
        memcpy(packet + 4, data->data, data->len);
        value.hdl = g_serial_tran_hdl;
        value.idx = BLE_SERIAL_TRAN_DATA_VALUE;
        value.len = data->len + 4;
        value.value = packet;

        ret = sibles_write_value(data->handle, &value);
        if (ret == value.len)
        {
            bt_mem_free(packet);
            // write success
            return ret;
        }
        else if (ret == 0)
        {
            // tx queue is full, wait and retry
            int retry = 20;
            while (retry > 0)
            {
                retry--;
                wait_time = ble_serial_wait_time_get(data->handle);
                rt_thread_mdelay(wait_time);
                ret = sibles_write_value(data->handle, &value);
                if (ret == value.len)
                {
                    LOG_I("send retry success");
                    bt_mem_free(packet);
                    return ret;
                }
            }
            LOG_E("send fail");
            bt_mem_free(packet);
            return 0;
        }
        LOG_E("unexpect fail");
        bt_mem_free(packet);
        return ret;
    }
    else
    {
        // use fragment packet
        uint16_t single_packet_len = env->mtu - 3 - 4;
        uint16_t offset = 0;

        while (offset < data->len)
        {
            do
            {
                // first packet
                if (offset == 0)
                {
                    packet = bt_mem_alloc(single_packet_len);
                    if (!packet)
                    {
                        return -3;
                    }
                    *packet = data->cate_id;                        // Add cateID to packet
                    *(packet + 1) = 1;                              // first not consider fragment
                    memcpy(packet + 2, &data->len, 2);
                    memcpy(packet + 4, data->data, single_packet_len);
                    value.hdl = g_serial_tran_hdl;
                    value.idx = BLE_SERIAL_TRAN_DATA_VALUE;
                    value.len = single_packet_len + 4;
                    value.value = packet;
                    break;
                }
                else if (data->len - offset <= single_packet_len)
                {
                    // last packet
                    single_packet_len = data->len - offset;

                    packet = bt_mem_alloc(single_packet_len);
                    if (!packet)
                    {
                        return -3;
                    }

                    *packet = data->cate_id;
                    *(packet + 1) = 3;
                    memcpy(packet + 2, data->data + offset, single_packet_len);
                    value.hdl = g_serial_tran_hdl;
                    value.idx = BLE_SERIAL_TRAN_DATA_VALUE;
                    value.len = single_packet_len + 2;
                    value.value = packet;
                    break;
                }
                else
                {
                    // continue packet
                    packet = bt_mem_alloc(single_packet_len);
                    if (!packet)
                    {
                        return -3;
                    }

                    *packet = data->cate_id;
                    *(packet + 1) = 2;
                    memcpy(packet + 2, data->data + offset, single_packet_len);
                    value.hdl = g_serial_tran_hdl;
                    value.idx = BLE_SERIAL_TRAN_DATA_VALUE;
                    value.len = single_packet_len + 2;
                    value.value = packet;
                    break;
                }
            }
            while (0);

            ret = sibles_write_value(data->handle, &value);
            if (ret == value.len)
            {
                bt_mem_free(packet);
                // write success
            }
            else if (ret == 0)
            {
                // tx queue is full, wait and retry
                int retry = 20;
                uint8_t send_success = 0;
                while (retry > 0)
                {
                    retry--;
                    wait_time = ble_serial_wait_time_get(data->handle);
                    rt_thread_mdelay(wait_time);
                    ret = sibles_write_value(data->handle, &value);
                    if (ret == value.len)
                    {
                        bt_mem_free(packet);
                        send_success = 1;
                        break;
                    }
                }
                if (send_success == 0)
                {
                    LOG_E("send fail");
                    bt_mem_free(packet);
                    return 0;
                }
            }

            offset += single_packet_len;
        }
        return data->len;
    }
}

int ble_serial_tran_send_data_advance(uint8_t handle, uint8_t *data, uint16_t data_len)
{
    sibles_value_t value;
    value.hdl = g_serial_tran_hdl;
    value.idx = BLE_SERIAL_TRAN_DATA_VALUE;
    value.len = data_len;
    value.value = data;

    int ret;
    ret = sibles_write_value(handle, &value);

    if (ret == data_len)
    {
        // write success
        return data_len;
    }
    else if (ret == 0)
    {
        // tx queue is full, wait and retry
        int retry = 20;
        while (retry > 0)
        {
            retry--;
            uint32_t wait_time = ble_serial_wait_time_get(handle);
            rt_thread_mdelay(wait_time);
            ret = sibles_write_value(handle, &value);
            if (ret == data_len)
            {
                LOG_I("send retry success");
                return data_len;
            }
        }
        return 0;
    }
    return ret;
}

int ble_serial_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_serial_tran_env_t *env = ble_serial_tran_get_env();

    switch (event_id)
    {
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        ble_serial_open_t chan;
        chan.handle = ind->conn_idx;
        env->mtu = 23;
        ble_serial_callback_event_notify(BLE_SERIAL_TRAN_OPEN, (uint8_t *)&chan);
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        env->mtu = ind->mtu;
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        ble_serial_close_t chan;
        chan.handle = ind->conn_idx;

        // clear assemable env if disconnect
        env->is_assemable = 0;
        if (env->assemable.ptr)
        {
            bt_mem_free(env->assemable.ptr);
        }
        memset(&env->assemable, 0, sizeof(ble_serial_tran_assemable_t));

        ble_serial_callback_event_notify(BLE_SERIAL_TRAN_CLOSE, (uint8_t *)&chan);
        break;
    }
    case SIBLES_WRITE_VALUE_RSP:
    {
        ble_serial_callback_event_notify(BLE_SERIAL_TRAN_SEND_AVAILABLE, NULL);
        break;
    }
    default:
        break;
    }
    return 0;
}

BLE_EVENT_REGISTER(ble_serial_event_handler, NULL);

SECTION_DEF(SerialTranExport, ble_serial_tran_export_t);

void ble_serial_tran_init(void)
{

    ble_serial_tran_env_t *env = ble_serial_tran_get_env();

    sibles_register_svc_128_t svc;

    // Init callback table
    env->cb_table = (ble_serial_tran_export_t *)SECTION_START_ADDR(SerialTranExport);
    env->cb_count = (ble_serial_tran_export_t *)SECTION_END_ADDR(SerialTranExport) - env->cb_table;


    svc.att_db = (struct attm_desc_128 *)&serial_trans_att_db;
    svc.num_entry = BLE_SERIAL_TRAN_ATT_NB;
    svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH) | PERM(SVC_UUID_LEN, UUID_128);
    svc.uuid = g_serial_tran_svc;
    g_serial_tran_hdl = sibles_register_svc_128(&svc);
    if (g_serial_tran_hdl)
        sibles_register_cbk(g_serial_tran_hdl, ble_serial_tran_get_cbk, ble_serial_tran_set_cbk);
}


#endif // BSP_BLE_SERIAL_TRANSMISSION
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
