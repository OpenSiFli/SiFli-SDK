/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_advertising.h"

#define LOG_TAG "app_ancs"
#include "log.h"



#define SERIAL_UUID_16(x) {((uint8_t)(x&0xff)),((uint8_t)(x>>8))}
/* 24 * 1.25 = 30ms */
#define BLE_APP_HIGH_PERFORMANCE_INTERVAL (24)
#define BLE_APP_TIMEOUT_INTERVAL (5000)

typedef struct
{
    uint8_t is_power_on;
    uint8_t conn_idx;
    struct
    {
        bd_addr_t peer_addr;
        uint16_t mtu;
        uint16_t conn_interval;
        uint8_t peer_addr_type;
    } conn_para;
    struct
    {
        sibles_hdl srv_handle;
        uint32_t data;
        uint8_t is_config_on;
    } data;
    rt_timer_t time_handle;
    rt_mailbox_t mb_handle;
} app_env_t;

static app_env_t g_app_env;
static rt_mailbox_t g_app_mb;


static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}


SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);

static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data)
{
    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("ADV start resutl %d, mode %d\r\n", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("ADV stopped reason %d, mode %d\r\n", evt->reason, evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}


#define DEFAULT_LOCAL_NAME "SIFLI_APP"
/* Enable advertise via advertising service. */
static void ble_app_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;

    char local_name[31] = {0};
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;
    bd_addr_t addr;
    ret = ble_get_public_address(&addr);
    if (ret == HL_ERR_NO_ERROR)
        rt_snprintf(local_name, 31, "SIFLI_APP-%x-%x-%x-%x-%x-%x", addr.addr[0], addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4], addr.addr[5]);
    else
        memcpy(local_name, DEFAULT_LOCAL_NAME, sizeof(DEFAULT_LOCAL_NAME));

    ble_gap_dev_name_t *dev_name = malloc(sizeof(ble_gap_dev_name_t) + strlen(local_name));
    dev_name->len = strlen(local_name);
    memcpy(dev_name->name, local_name, dev_name->len);
    ble_gap_set_dev_name(dev_name);
    free(dev_name);

    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    /* Keep advertising till disable it or connected. */
    para.config.mode_config.conn_config.duration = 0x0;
    para.config.mode_config.conn_config.interval = 0x140;
    para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    para.config.is_auto_restart = 1;
    /* Scan rsp data is same as advertising data. */
    //para.config.is_rsp_data_duplicate = 1;

    /* Prepare name filed. Due to name is too long to put adv data, put it to rsp data.*/
    para.rsp_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.rsp_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.rsp_data.completed_name->name, local_name, para.rsp_data.completed_name->name_len);

    /* Prepare manufacturer filed .*/
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_advertising_event;

    ret = sibles_advertising_init(g_app_advertising_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_context);
    }

    rt_free(para.rsp_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}


int main(void)
{
    int count = 0;
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();

    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;
            env->conn_para.mtu = 23; /* Default value. */
            ble_app_advertising_start();
            LOG_I("receive BLE power on!\r\n");
        }
    }
    return RT_EOK;
}


static void ble_app_update_conn_param(uint8_t conn_idx, uint16_t inv_max, uint16_t inv_min, uint16_t timeout)
{
    ble_gap_update_conn_param_t conn_para;
    conn_para.conn_idx = conn_idx;
    conn_para.intv_max = inv_max;
    conn_para.intv_min = inv_min;
    /* value = argv * 1.25 */
    conn_para.ce_len_max = 0x100;
    conn_para.ce_len_min = 0x1;
    conn_para.latency = 0;
    conn_para.time_out = timeout;
    ble_gap_update_conn_param(&conn_para);
}

int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    app_env_t *env = ble_app_get_env();
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        /* Handle in own thread to avoid conflict */
        if (env->mb_handle)
            rt_mb_send(env->mb_handle, BLE_POWER_ON_IND);
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        env->conn_para.conn_interval = ind->con_interval;
        env->conn_para.peer_addr_type = ind->peer_addr_type;
        env->conn_para.peer_addr = ind->peer_addr;
        if (ind->role == 0)
            LOG_E("Peripheral should be slave!!!");

        LOG_I("Peer device(%x-%x-%x-%x-%x-%x) connected", env->conn_para.peer_addr.addr[5],
              env->conn_para.peer_addr.addr[4],
              env->conn_para.peer_addr.addr[3],
              env->conn_para.peer_addr.addr[2],
              env->conn_para.peer_addr.addr[1],
              env->conn_para.peer_addr.addr[0]);


        ble_gap_sec_req_t sec_req;
        sec_req.auth = GAP_AUTH_REQ_SEC_CON_BOND;
        sec_req.conn_idx = ind->conn_idx;
        ble_gap_security_request(&sec_req);

        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        env->conn_para.conn_interval = ind->con_interval;
        LOG_I("Updated connection interval :%d", ind->con_interval);
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        /* Negotiated MTU. */
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        env->conn_para.mtu = ind->mtu;
        LOG_I("Exchanged MTU size: %d", ind->mtu);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        LOG_I("BLE_GAP_DISCONNECTED_IND(%d)", ind->reason);
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

