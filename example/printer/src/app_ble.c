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

#include "app_common.h"
#include "bf0_sibles_internal.h"
#include "ble_connection_manager.h"

#define LOG_TAG "ble_app"
#include "log.h"

OS_TIMER_DECLAR(g_app_timer);


enum ble_app_att_list
{
    BLE_APP_SVC,
    BLE_APP_CHAR,
    BLE_APP_CHAR_VALUE,
    BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR,
    BLE_APP_ATT_NB
};


#define app_svc_uuid { \
    0x73, 0x69, 0x66, 0x6c, \
    0x69, 0x5f, 0x61, 0x70, \
    0x70, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00 \
};

#define app_chara_uuid { \
    0x73, 0x69, 0x66, 0x6c, \
    0x69, 0x5f, 0x61, 0x70, \
    0x70, 0x01, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00 \
}

#define SERIAL_UUID_16(x) {((uint8_t)(x&0xff)),((uint8_t)(x>>8))}
/* 24 * 1.25 = 30ms */
#define BLE_APP_HIGH_PERFORMANCE_INTERVAL (24)
#define BLE_APP_TIMEOUT_INTERVAL (5000)



static app_env_t g_app_env;
static rt_mailbox_t g_app_mb;

static uint8_t g_app_svc[ATT_UUID_128_LEN] = app_svc_uuid;

static uint8_t ble_connect_count;


struct attm_desc_128 app_att_db[] =
{
    // Service declaration
    [BLE_APP_SVC] = {SERIAL_UUID_16(ATT_DECL_PRIMARY_SERVICE), PERM(RD, ENABLE), 0, 0},
    // Characteristic  declaration
    [BLE_APP_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC), PERM(RD, ENABLE), 0, 0},
    // Characteristic value config
    [BLE_APP_CHAR_VALUE] = {
        /* The permissions are for: 1.Allowed read, write req, write command and notification.
                                    2.Write requires Unauthenticated link
           The ext_perm are for: 1. Support 128bit UUID. 2. Read will involve callback. */
        app_chara_uuid, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WRITE_COMMAND, ENABLE) | PERM(NTF, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024
    },
    // Descriptor config
    [BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR] = {
        SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ,
                ENABLE), PERM(RI, ENABLE), 2
    },

};

static void ble_write_to_remote(void *param);

#ifdef APP_ENABLE_BG_ADV
SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_bg_context);

static uint8_t ble_app_background_advertising_event(uint8_t event, void *context, void *data)
{
    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("Broadcast ADV start resutl %d, mode %d\r\n", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("Broadcast ADV stopped reason %d, mode %d\r\n", evt->reason, evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}


/* Enable advertise via advertising service. */
static void ble_app_bg_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;

    char local_name[] = "SIFLI_BG_INFO";
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;

    para.own_addr_type = GAPM_GEN_NON_RSLV_ADDR;
    para.config.adv_mode = SIBLES_ADV_BROADCAST_MODE;
    /* Keep advertising till disable it or connected. */
    para.config.mode_config.broadcast_config.duration = 0x0;
    para.config.mode_config.broadcast_config.interval = 0x140;
    para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    para.config.is_auto_restart = 1;
    /* Scan rsp data is same as advertising data. */
    para.config.is_rsp_data_duplicate = 1;

    /* Prepare name filed .*/
    para.adv_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.adv_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.adv_data.completed_name->name, local_name, para.adv_data.completed_name->name_len);

    /* Prepare manufacturer filed .*/
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_background_advertising_event;

    ret = sibles_advertising_init(g_app_advertising_bg_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_bg_context);
    }

    rt_free(para.adv_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}
#endif // APP_ENABLE_BG_ADV

SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);
static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data)
{
    app_env_t *env = ble_app_get_env();

    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
#ifdef APP_ENABLE_BG_ADV
        if (!env->is_bg_adv_on)
        {
            env->is_bg_adv_on = 1;
            ble_app_bg_advertising_start();
        }
#endif // APP_ENABLE_BG_ADV
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
void ble_app_advertising_start(void)
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
    para.config.mode_config.conn_config.interval = 0x30;
    para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    para.config.is_auto_restart = 0;
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

    para.adv_data.disc_mode = GAPM_ADV_MODE_CUSTOMIZE;
    para.adv_data.flags = 0x06;

    ret = sibles_advertising_init(g_app_advertising_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_context);
    }

    rt_free(para.rsp_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}



// Hanlde read operation
uint8_t *ble_app_gatts_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    uint8_t *ret_val = NULL;
    app_env_t *env = ble_app_get_env();
    *len = 0;
    switch (idx)
    {
    case BLE_APP_CHAR_VALUE:
    {
        // Prepare data to remote device
        ret_val = (uint8_t *)&env->data.data;
        *len = 4;
        break;
    }
    default:
        break;
    }
    return ret_val;
}

static void ble_app_start_send_thread(uint32_t param)
{
    rt_thread_t tid;
    tid = rt_thread_create("ble_send_example", ble_write_to_remote, (void *)(uint32_t)param, 1024, RT_THREAD_PRIORITY_LOW, 10);
    rt_thread_startup(tid);

}

// Hanlde read operation
uint8_t ble_app_gatts_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    app_env_t *env = ble_app_get_env();
    switch (para->idx)
    {
    case BLE_APP_CHAR_VALUE:
    {
        // Store value that remote device writes
        uint32_t old_val = env->data.data;
        LOG_HEX("BLE RX", 16, para->value, para->len);
        if (para->len <= 4)
            memcpy(&env->data.data, para->value, para->len);
        LOG_I("Updated app value from:%d to:%d", old_val, env->data.data);
        if (env->data.data == 0x1111)
        {
            LOG_I("example of ble notification");
            ble_app_start_send_thread(env->data.data);
        }
        break;
    }
    case BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR:
    {
        env->data.is_config_on = *(para->value);
        LOG_I("CCCD %d", env->data.is_config_on);
        // Enable notification via timer
        if (env->data.is_config_on)
            rt_timer_start(env->time_handle);
        else
            rt_timer_stop(env->time_handle);
        break;
    }
    default:
        break;
    }
    return 0;
}

/**
 * 1. Example of use notification to write a large amount of data to remote.
 * 2. This write function should not call in original ble task, should put into a low priority
 * 3. sibles_write_value_with_rsp can also use if indication permission is enabled
 * 4. except using rt_thread_mdelay to wait and retry, can also write next message
 * after receiving SIBLES_WRITE_VALUE_RSP message
 * 5. may adjust delay times and retry times if data is more larger.
 */
static void ble_write_to_remote(void *param)
{
    app_env_t *env = ble_app_get_env();
    uint32_t para = (uint32_t)param;
    uint8_t idx = para & 0xFF;
    // we will have 20 sets of data
    int data_count = (para & 0xFFFF0000) >> 16;
    int data_length = (para & 0xFF00) >> 8;

    // build up datas
    uint8_t *raw_data = malloc(data_length);

    uint8_t data_index = 0;

    while (data_count > 0)
    {
        // here is notify example
        sibles_value_t value;
        value.hdl = env->data.srv_handle;
        value.idx = BLE_APP_CHAR_VALUE;
        value.len = data_length;
        value.value = raw_data;
        memset(raw_data, data_index, data_length);

        int ret;
        ret = sibles_write_value(env->conn_idx, &value);

        if (ret == data_length)
        {
            // write success
            data_count--;
            data_index++;
        }
        else if (ret == 0)
        {
            // tx queue is full, wait and retry
            int retry = 20;
            while (retry > 0)
            {
                retry--;
                rt_thread_mdelay(50);
                ret = sibles_write_value(env->conn_idx, &value);
                if (ret == data_length)
                {
                    LOG_I("send retry success");
                    data_count--;
                    data_index++;
                    break;
                }
            }
        }
    }
}

void ble_app_service_init(void)
{
    app_env_t *env = ble_app_get_env();
    sibles_register_svc_128_t svc;

    svc.att_db = (struct attm_desc_128 *)&app_att_db;
    svc.num_entry = BLE_APP_ATT_NB;
    /* Service security level to control all characteristic. */
    svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH) | PERM(SVC_UUID_LEN, UUID_128);
    svc.uuid = g_app_svc;
    /* Reigster GATT service and related callback for next response. */
    env->data.srv_handle = sibles_register_svc_128(&svc);
    if (env->data.srv_handle)
        sibles_register_cbk(env->data.srv_handle, ble_app_gatts_get_cbk, ble_app_gatts_set_cbk);
}


void app_timeout_handler(void *parameter)
{
    app_env_t *env = ble_app_get_env();
    if (env->data.is_config_on)
    {
        sibles_value_t value;
        value.hdl = env->data.srv_handle;
        value.idx = BLE_APP_CHAR_VALUE;
        value.len = 4;
        value.value = (uint8_t *)&env->data.data;
        sibles_write_value(env->conn_idx, &value);
        rt_timer_start(env->time_handle);
    }
}




#ifndef NVDS_AUTO_UPDATE_MAC_ADDRESS_ENABLE
ble_common_update_type_t ble_request_public_address(bd_addr_t *addr)
{
    int ret = bt_mac_addr_generate_via_uid_v2(addr);

    if (ret != 0)
    {
        LOG_W("generate mac addres failed %d", ret);
        return BLE_UPDATE_NO_UPDATE;
    }

    return BLE_UPDATE_ONCE;
}
#endif // NVDS_AUTO_UPDATE_MAC_ADDRESS_ENABLE


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

static void app_timeout2_handler(void *para)
{
    app_env_t *env = ble_app_get_env();
    uint8_t conn_idx = (uint8_t)para;
    connection_manager_update_parameter(conn_idx, CONNECTION_MANAGER_INTERVAL_HIGH_PERFORMANCE, NULL);

    os_timer_stop(g_app_timer);
    os_timer_delete(g_app_timer);
    env->is_timer_on = 0;
}

static void app_timeout1_handler(void *para)
{
    uint8_t conn_idx = (uint8_t)para;
    ble_gap_update_phy_t phy;
    phy.conn_idx = conn_idx;
    phy.rx_phy = GAP_PHY_LE_2MBPS;
    phy.tx_phy = GAP_PHY_LE_2MBPS;
    ble_gap_update_phy(&phy);

    os_timer_stop(g_app_timer);
    os_timer_delete(g_app_timer);

    os_timer_create(g_app_timer, app_timeout2_handler, para, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    os_timer_start(g_app_timer, 5000);
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
        env->is_timer_on = 0;
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        env->conn_para.conn_interval = ind->con_interval;
        env->conn_para.peer_addr_type = ind->peer_addr_type;
        env->conn_para.peer_addr = ind->peer_addr;
#if defined(BSP_BLE_CONNECTION_MANAGER) && !defined(BLE_CM_BOND_DISABLE)
        connection_manager_set_bond_cnf_auth(GAP_AUTH_REQ_NO_MITM_BOND);
#endif // defined(BSP_BLE_CONNECTION_MANAGER) && !defined(BLE_CM_BOND_DISABLE)
        if (ind->role == 0)
            LOG_E("Peripheral should be slave!!!");

        LOG_I("Peer device(%x-%x-%x-%x-%x-%x) connected", env->conn_para.peer_addr.addr[5],
              env->conn_para.peer_addr.addr[4],
              env->conn_para.peer_addr.addr[3],
              env->conn_para.peer_addr.addr[2],
              env->conn_para.peer_addr.addr[1],
              env->conn_para.peer_addr.addr[0]);

        uint32_t conn_idx_para = ind->conn_idx;
        void *para = (void *)conn_idx_para;
        if (env->is_timer_on == 0)
        {
            env->is_timer_on = 1;
            os_timer_create(g_app_timer, app_timeout1_handler, para, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
            os_timer_start(g_app_timer, 5000);
        }
        ble_connect_count++;
        if (ble_connect_count < 3)
        {
            sibles_advertising_start(g_app_advertising_context);
        }
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
        if (env->is_timer_on == 1)
        {
            env->is_timer_on = 0;
            os_timer_stop(g_app_timer);
            os_timer_delete(g_app_timer);
        }
        if (ble_connect_count > 0)
        {
            ble_connect_count--;
            sibles_advertising_start(g_app_advertising_context);
        }
        break;
    }
    case SIBLES_WRITE_VALUE_RSP:
    {
        sibles_write_value_rsp_t *rsp = (sibles_write_value_rsp_t *)data;
        LOG_I("SIBLES_WRITE_VALUE_RSP %d", rsp->result);
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);

void ble_init()
{
    app_env_t *env = ble_app_get_env();
    env->time_handle  = rt_timer_create("app", app_timeout_handler,  NULL,
                                        rt_tick_from_millisecond(APP_TIMEOUT_INTERVAL), RT_TIMER_FLAG_SOFT_TIMER);

}

#ifdef FINSH_USING_MSH
int cmd_diss(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "trc") == 0)
        {
            if (strcmp(argv[2], "mode") == 0)
            {
                uint8_t mode = atoi(argv[3]);
                uint32_t mask = atoi(argv[4]);
                sibles_set_trc_cfg(mode, mask);
            }
        }
        else if (0 == strcmp(argv[1], "conn"))
        {
            uint32_t interval;
            app_env_t *env = ble_app_get_env();

            interval = atoi(argv[2]);
            // value = argv * 1.25
            interval = interval * 100 / 125;
            ble_app_update_conn_param(env->conn_idx, interval, interval, 500);
        }
        else if (strcmp(argv[1], "adv_start") == 0)
        {
            sibles_advertising_start(g_app_advertising_context);
        }
        else if (strcmp(argv[1], "adv_stop") == 0)
        {
            sibles_advertising_stop(g_app_advertising_context);
        }
        else if (strcmp(argv[1], "gen_addr") == 0)
        {
            bd_addr_t addr;
            int ret;
            ret = bt_mac_addr_generate_via_uid_v2(&addr);
            LOG_D("ret %d", ret);
        }

    }

    return 0;
}

MSH_CMD_EXPORT(cmd_diss, My device information service.);


int ble_config(int argc, char *argv[])
{
    if (argc < 3)
    {
        LOG_I("Wrong argument");
        return -1;
    }
    if (0 == strcmp(argv[1], "adv"))
    {
        uint32_t interval;
        sibles_adv_config_t new_adv_config = {0};
        uint8_t ret;
        interval = atoi(argv[2]);
        new_adv_config.adv_mode = SIBLES_ADV_CONNECT_MODE;
        new_adv_config.mode_config.conn_config.duration = 0x0;
        new_adv_config.mode_config.conn_config.interval = interval * 1000 / 625;
        new_adv_config.max_tx_pwr = 0x7F;
        new_adv_config.is_auto_restart = 1;
        ret = sibles_advertising_stop(g_app_advertising_context);
        LOG_I("stop: %d", ret);
        ret = sibles_advertising_reconfig(g_app_advertising_context, &new_adv_config);
        LOG_I("reconfig: %d", ret);
        ret = sibles_advertising_start(g_app_advertising_context);
        LOG_I("start: %d", ret);
    }
    else if (0 == strcmp(argv[1], "conn"))
    {
        uint32_t interval;
        app_env_t *env = ble_app_get_env();
        interval = atoi(argv[2]);
        interval = interval * 100 / 125;
        ble_app_update_conn_param(env->conn_idx, interval, interval, 500);
    }
    return 0;
}
MSH_CMD_EXPORT(ble_config, "BLE Configure")

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

