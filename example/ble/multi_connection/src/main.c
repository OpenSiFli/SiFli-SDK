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
#include "bf0_sibles_internal.h"
#include "bf0_sibles_advertising.h"
#include "ble_connection_manager.h"

#define LOG_TAG "ble_app"
#include "log.h"

#ifdef BSP_BLE_TIMEC
    #include "bf0_ble_tipc.h"
    #include "time.h"
#endif


#ifdef BSP_BLE_HRPC
    #include "bf0_ble_hrpc.h"
#endif

enum ble_app_att_list
{
    BLE_APP_SVC,
    BLE_APP_CHAR,
    BLE_APP_CHAR_VALUE,
    BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR,
    BLE_APP_ATT_NB
};

typedef enum
{
    BLE_APP_UUID_TYPE_SERVICE,
    BLE_APP_UUID_TYPE_CHARATER,
    BLE_APP_UUID_TYPE_DESCRIPTOR,
    BLE_APP_UUID_TYPE_TOTAL
} ble_app_uuid_display_type_t;


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
#define BLE_APP_MAX_ADV_COUNT 20
#define BLE_APP_MAX_CONN_COUNT 8
#define APP_MAX_DESC 2
#define BLE_APP_BIT_CONVERT_DIGIT_INC(n, m) (((n & (1 << m)) != 0) * (m+1))


typedef struct
{
    uint8_t is_power_on;
    uint8_t is_bg_adv_on;
    uint8_t adv_count;
    uint8_t conn_count;
    ble_gap_addr_t adv_addr[BLE_APP_MAX_ADV_COUNT];
    uint8_t adv_info[BLE_APP_MAX_ADV_COUNT];
    int8_t scan_rssi;
    ble_gap_connection_create_param_t last_init_conn;
    struct
    {
        uint8_t conn_idx;
        uint8_t role;
        uint8_t peer_addr_type;
        uint16_t mtu;
        uint16_t conn_interval;
        bd_addr_t peer_addr;
        struct
        {
            uint32_t data;
            uint8_t is_config_on;
        } data;
        sibles_svc_remote_svc_t svc;
        uint16_t rmt_svc_hdl;
    } conn[BLE_APP_MAX_CONN_COUNT];

    sibles_hdl srv_handle;

    rt_mailbox_t mb_handle;
} app_env_t;

static app_env_t g_app_env;
static rt_mailbox_t g_app_mb;

static uint8_t g_app_svc[ATT_UUID_128_LEN] = app_svc_uuid;



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
        app_chara_uuid, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WRITE_COMMAND, ENABLE) | PERM(NTF, ENABLE) |
        PERM(WP, NO_AUTH), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024
    },
    // Descriptor config
    [BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR] = {
        SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ,
                ENABLE) | PERM(WP, NO_AUTH), PERM(RI, ENABLE), 2
    },

};

static const char *const s_ble_app_addr_type[] =
{
    "public",
    "random",
    "public identity",
    "random identity",
    "not resovled",
    "anonymous"
};

static const char *const s_ble_app_report_type[] =
{
    "ext adv",
    "legacy adv",
    "ext scan response",
    "legacy scan response",
    "periodic adv",
};

static const char *const s_ble_app_info_type[] =
{
    "connectable",
    "scannable",
    "direct",
    "",
};


static const char *const s_ble_app_uuid_type[BLE_APP_UUID_TYPE_TOTAL] =
{
    "Service",
    "    Charateristic",
    "        Descriptor",
};

static const char *const s_ble_app_chara_prop[] =
{
    "",
    "Broadcast",
    "Read",
    "Write_wo_rsp",
    "Write",
    "Notify",
    "Indication",
    "Authticated_signed_writes",
    "Extended_properties",
};


static uint8_t ble_app_get_dev_by_idx(app_env_t *env, uint8_t conn_idx);


static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}


static void ble_write_to_remote(void *param);



SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);

static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data)
{
    app_env_t *env = ble_app_get_env();

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
#define EXAMPLE_LOCAL_NAME "SIFLI_EXAMPLE"
/* Enable advertise via advertising service. */
static void ble_app_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;

    char local_name[31] = {0};
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = SIG_SIFLI_COMPANY_ID;
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
    // in multi-connection
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

    ret = sibles_advertising_init(g_app_advertising_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_context);
    }

    rt_free(para.rsp_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}

static void update_adv_content()
{
    sibles_advertising_para_t para = {0};

    char temp_name[31] = {0};
    memcpy(temp_name, EXAMPLE_LOCAL_NAME, sizeof(EXAMPLE_LOCAL_NAME));

    para.rsp_data.completed_name = rt_malloc(rt_strlen(temp_name) + sizeof(sibles_adv_type_name_t));
    para.rsp_data.completed_name->name_len = rt_strlen(temp_name);
    rt_memcpy(para.rsp_data.completed_name->name, temp_name, para.rsp_data.completed_name->name_len);

    uint8_t manu_additnal_data[] = {0x23, 0x33, 0x33, 0x33};
    uint16_t manu_company_id = SIG_SIFLI_COMPANY_ID;
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    uint8_t ret = sibles_advertising_update_adv_and_scan_rsp_data(g_app_advertising_context, &para.adv_data, &para.rsp_data);
    LOG_I("update adv %d", ret);

    rt_free(para.rsp_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}


// Hanlde read operation
uint8_t *ble_app_gatts_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    uint8_t *ret_val = NULL;
    app_env_t *env = ble_app_get_env();
    uint8_t dev_idx = ble_app_get_dev_by_idx(env, conn_idx);
    *len = 0;
    switch (idx)
    {
    case BLE_APP_CHAR_VALUE:
    {
        // Prepare data to remote device
        ret_val = (uint8_t *)&env->conn[dev_idx].data.data;
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
    uint8_t idx = ble_app_get_dev_by_idx(env, conn_idx);
    switch (para->idx)
    {
    case BLE_APP_CHAR_VALUE:
    {
        // Store value that remote device writes
        uint32_t old_val = env->conn[idx].data.data;
        if (para->len <= 4)
            memcpy(&env->conn[idx].data.data, para->value, para->len);
        LOG_I("Device %d updated app value from:%d to:%d", idx, old_val, env->conn[idx].data.data);
        if ((env->conn[idx].data.data & 0xFF) == 0x11)
        {
            env->conn[idx].data.data = env->conn[idx].data.data & 0xFFFFFF00 + idx;
            LOG_I("example of ble notification %d", env->conn[idx].data.data);
            ble_app_start_send_thread(env->conn[idx].data.data);
        }
        break;
    }
    case BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR:
    {
        env->conn[idx].data.is_config_on = *(para->value);
        LOG_I("device %d CCCD %d", idx, env->conn[idx].data.is_config_on);
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

    LOG_I("NTF example count %d, len %d", data_count, data_length);

    // build up datas
    uint8_t *raw_data = malloc(data_length);

    uint8_t data_index = 0;

    while (data_count > 0)
    {
        // here is notify example
        sibles_value_t value;
        value.hdl = env->srv_handle;
        value.idx = BLE_APP_CHAR_VALUE;
        value.len = data_length;
        value.value = raw_data;
        memset(raw_data, data_index, data_length);

        int ret;
        ret = sibles_write_value(env->conn[idx].conn_idx, &value);

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
                ret = sibles_write_value(env->conn[idx].conn_idx, &value);
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


static void ble_write_to_remote_master(uint8_t conn_idx, uint16_t handle, uint32_t data_count, uint32_t data_length)
{
    app_env_t *env = ble_app_get_env();

    // build up datas
    uint8_t *raw_data = malloc(data_length);

    uint8_t data_index = 0;

    while (data_count > 0)
    {
        sibles_write_remote_value_t value;
        value.handle = (uint16_t)handle;
        value.write_type = SIBLES_WRITE_WITHOUT_RSP;
        value.len = data_length;

        memset(raw_data, data_index, data_length);
        value.value = (uint8_t *)raw_data;

        int8_t ret = sibles_write_remote_value(env->conn[conn_idx].rmt_svc_hdl, env->conn[conn_idx].conn_idx, &value);


        if (ret == SIBLES_WRITE_NO_ERR)
        {
            // write success
            data_count--;
            data_index++;
        }
        else
        {
            // tx queue is full, wait and retry
            int retry = 20;
            while (retry > 0)
            {
                retry--;
                rt_thread_mdelay(50);
                ret = sibles_write_remote_value(env->conn[conn_idx].rmt_svc_hdl, env->conn[conn_idx].conn_idx, &value);
                if (ret == SIBLES_WRITE_NO_ERR)
                {
                    LOG_I("send retry success");
                    data_count--;
                    data_index++;
                    break;
                }
            }

            if (retry == 0)
            {
                LOG_E("Write failed");
                break;
            }
        }
    }
}


static void ble_app_service_init(void)
{
    app_env_t *env = ble_app_get_env();
    sibles_register_svc_128_t svc;

    svc.att_db = (struct attm_desc_128 *)&app_att_db;
    svc.num_entry = BLE_APP_ATT_NB;
    /* Service security level to control all characteristic. */
    svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH) | PERM(SVC_UUID_LEN, UUID_128) | PERM(SVC_MI, ENABLE);
    svc.uuid = g_app_svc;
    /* Reigster GATT service and related callback for next response. */
    env->srv_handle = sibles_register_svc_128(&svc);
    if (env->srv_handle)
        sibles_register_cbk(env->srv_handle, ble_app_gatts_get_cbk, ble_app_gatts_set_cbk);
}


#ifdef ULOG_USING_FILTER
static void app_log_filter_set(void)
{
    ulog_tag_lvl_filter_set("BLE_GAP", LOG_LVL_WARNING);

    ulog_tag_lvl_filter_set("sibles", LOG_LVL_WARNING);
}
#endif // ULOG_USING_FILTER

int main(void)
{
    int count = 0;
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();
#ifdef ULOG_USING_FILTER
    app_log_filter_set();
#endif // ULOG_USING_FILTER

#ifdef BSP_BLE_TIMEC
    ble_tipc_init(1);
#endif

#ifdef BSP_BLE_HRPC
    ble_hrpc_init(true);
#endif

    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;
            // Initialnize conn idx;
            for (uint32_t i = 0; i < BLE_APP_MAX_CONN_COUNT; i++)
                env->conn[i].conn_idx = 0xFF;

            ble_app_service_init();
            /* First enable connectable adv then enable non-connectable. */
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





static bool ble_app_adv_filter(app_env_t *env, ble_gap_ext_adv_report_ind_t *ind)
{
    uint8_t ret = false;
    uint32_t i;
    for (i = 0; i < env->adv_count; i++)
    {
        if (memcmp(&env->adv_addr[i], &ind->addr, sizeof(ble_gap_addr_t)) == 0 &&
                env->adv_info[i] == ind->info)
        {
            ret = true;
            break;
        }
    }
    return ret;
}

static void ble_app_adv_add(app_env_t *env, ble_gap_ext_adv_report_ind_t *ind)
{
    memcpy(&env->adv_addr[env->adv_count], &ind->addr, sizeof(ble_gap_addr_t));
    env->adv_info[env->adv_count] = ind->info;
    env->adv_count++;
}

static void ble_app_display_adv_context(ble_gap_ext_adv_report_ind_t *ind, uint8_t adv_count)
{
    uint8_t addr_str_type = ind->addr.addr_type;
    char *adv_info_str = malloc(40);
    if (addr_str_type == 0xFE)
        addr_str_type = 0x04;
    else if (addr_str_type == 0xFF)
        addr_str_type = 0x05;

    LOG_I("advertising device %2d: addr type: %s, addr: %x-%x-%x-%x-%x-%x", adv_count, s_ble_app_addr_type[addr_str_type], ind->addr.addr.addr[0],
          ind->addr.addr.addr[1], ind->addr.addr.addr[2], ind->addr.addr.addr[3], ind->addr.addr.addr[4],
          ind->addr.addr.addr[5]);
    {
        uint8_t is_conn = ((ind->info & GAPM_REPORT_INFO_CONN_ADV_BIT) != 0) ? 0 : 3;
        uint8_t is_scan = ((ind->info & GAPM_REPORT_INFO_SCAN_ADV_BIT) != 0) ? 1 : 3;
        uint8_t is_dir = ((ind->info & GAPM_REPORT_INFO_DIR_ADV_BIT) != 0) ? 2 : 3;
        rt_snprintf(adv_info_str, 40, "%s %s %s adv", s_ble_app_info_type[is_conn],
                    s_ble_app_info_type[is_scan],
                    s_ble_app_info_type[is_dir]);
    }
    LOG_I("adv type: %s, adv info: %s", s_ble_app_report_type[ind->info & GAPM_REPORT_INFO_REPORT_TYPE_MASK], adv_info_str);

    LOG_I("adv tx pwr: %d, rssi: %d", ind->tx_pwr, ind->rssi);
    LOG_HEX("adv_data", 16, ind->data, ind->length);

    LOG_I("\r\n");
    rt_free(adv_info_str);
}

static void ble_app_display_connected_device(app_env_t *env)
{
    LOG_I("Total connected %d devices", env->conn_count);
    for (uint32_t i = 0; i < BLE_APP_MAX_CONN_COUNT; i++)
    {
        if (env->conn[i].conn_idx == 0xFF)
            continue;

        LOG_I("Device %d: role:%d, address(type %d): %x-%x-%x-%x-%x-%x", i, env->conn[i].role, env->conn[i].peer_addr_type,
              env->conn[i].peer_addr.addr[0],
              env->conn[i].peer_addr.addr[1],
              env->conn[i].peer_addr.addr[2],
              env->conn[i].peer_addr.addr[3],
              env->conn[i].peer_addr.addr[4],
              env->conn[i].peer_addr.addr[5]);
        LOG_I("MTU: %d, connection interval %d", env->conn[i].mtu, env->conn[i].conn_interval * 5 / 4);
        LOG_I("");
    }
}

static void ble_app_device_connected(app_env_t *env, connection_manager_connect_ind_t *ind)
{
    uint32_t i;
    for (i = 0; i < BLE_APP_MAX_CONN_COUNT; i++)
        if (env->conn[i].conn_idx == 0xFF)
            break;

    if (i == BLE_APP_MAX_CONN_COUNT)
        RT_ASSERT(0);

    env->conn_count++;
    env->conn[i].conn_idx = ind->conn_idx;
    env->conn[i].conn_interval = ind->con_interval;
    env->conn[i].peer_addr_type = ind->peer_addr_type;
    env->conn[i].peer_addr = ind->peer_addr;
    env->conn[i].role = ind->role;
    env->conn[i].mtu = 23;

    LOG_I("Peer device(role:%d) (%x-%x-%x-%x-%x-%x) connected as deivce %d", ind->role, env->conn[i].peer_addr.addr[5],
          env->conn[i].peer_addr.addr[4],
          env->conn[i].peer_addr.addr[3],
          env->conn[i].peer_addr.addr[2],
          env->conn[i].peer_addr.addr[1],
          env->conn[i].peer_addr.addr[0],
          i);
}

static void ble_app_deivce_disconnected(app_env_t *env, uint8_t idx, uint8_t reason)
{
    RT_ASSERT((idx < BLE_APP_MAX_CONN_COUNT) && (env->conn[idx].conn_idx != 0xFF));
    env->conn[idx].conn_idx = 0xFF;
    env->conn_count--;

    LOG_I("Device %d (%x-%x-%x-%x-%x-%x) disconnected(reason %d)", idx, env->conn[idx].peer_addr.addr[5],
          env->conn[idx].peer_addr.addr[4],
          env->conn[idx].peer_addr.addr[3],
          env->conn[idx].peer_addr.addr[2],
          env->conn[idx].peer_addr.addr[1],
          env->conn[idx].peer_addr.addr[0],
          reason);
}

static uint8_t ble_app_get_dev_by_idx(app_env_t *env, uint8_t conn_idx)
{
    uint32_t i;
    for (i = 0; i < BLE_APP_MAX_CONN_COUNT; i++)
        if (env->conn[i].conn_idx == conn_idx)
            break;

    return i == BLE_APP_MAX_CONN_COUNT ? 0xFF : (uint8_t)i;
}

static void ble_app_display_uuid(ble_app_uuid_display_type_t type, uint8_t uuid_len, uint8_t *uuid)
{
    if (type >= BLE_APP_UUID_TYPE_TOTAL)
        return;

    LOG_I("%s UUID(%d bit):", s_ble_app_uuid_type[type], uuid_len * 8);
    switch (uuid_len)
    {
    case 2:
    {
        LOG_I("%s UUID: 0x%02x%02x", s_ble_app_uuid_type[type], uuid[1], uuid[0]);
    }
    break;
    case 4:
    {
        LOG_I("%s UUID: 0x%02x%02x", s_ble_app_uuid_type[type], uuid[3], uuid[2], uuid[1], uuid[0]);
    }
    break;
    case 16:
    {
        LOG_I("%s UUID: 0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", s_ble_app_uuid_type[type], uuid[15], uuid[14], uuid[13], uuid[12], uuid[11],
              uuid[10], uuid[9], uuid[8], uuid[7], uuid[6],
              uuid[5], uuid[4], uuid[3], uuid[2], uuid[1],
              uuid[0]);
    }
    break;
    default:
    {
        ASSERT(0 && "Invalid service uuid length");
    }
    break;
    }
}


static void ble_app_display_service(sibles_svc_remote_svc_t *svc)
{
    if (svc == NULL || svc->att_db == NULL)
    {
        LOG_W("Displayed wrongly service.");
        return;
    }

    ble_app_display_uuid(BLE_APP_UUID_TYPE_SERVICE, svc->uuid_len, svc->uuid);

    uint32_t i, t;
    uint16_t offset = 0;
    uint8_t desc_num;
    sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)svc->att_db;
    for (i = 0; i < svc->char_count; i++)
    {
        LOG_I("    Charateristic(%d) handle %d, value handle %d", i, chara->attr_hdl, chara->pointer_hdl, chara->prop);
        ble_app_display_uuid(BLE_APP_UUID_TYPE_CHARATER, chara->uuid_len, chara->uuid);
        LOG_I("    Properties: %s %s %s %s %s %s %s %s", s_ble_app_chara_prop[BLE_APP_BIT_CONVERT_DIGIT_INC(chara->prop, 0)],
              s_ble_app_chara_prop[BLE_APP_BIT_CONVERT_DIGIT_INC(chara->prop, 1)],
              s_ble_app_chara_prop[BLE_APP_BIT_CONVERT_DIGIT_INC(chara->prop, 2)],
              s_ble_app_chara_prop[BLE_APP_BIT_CONVERT_DIGIT_INC(chara->prop, 3)],
              s_ble_app_chara_prop[BLE_APP_BIT_CONVERT_DIGIT_INC(chara->prop, 4)],
              s_ble_app_chara_prop[BLE_APP_BIT_CONVERT_DIGIT_INC(chara->prop, 5)],
              s_ble_app_chara_prop[BLE_APP_BIT_CONVERT_DIGIT_INC(chara->prop, 6)],
              s_ble_app_chara_prop[BLE_APP_BIT_CONVERT_DIGIT_INC(chara->prop, 7)]);

        for (t = 0; t < chara->desc_count; t++)
        {
            LOG_I("        Descriptor(%d) handle %d", t, chara->desc[t].attr_hdl);
            ble_app_display_uuid(BLE_APP_UUID_TYPE_DESCRIPTOR, chara->desc[t].uuid_len, chara->desc[t].uuid);
        }
        offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
        chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
    }

}


int ble_app_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    app_env_t *env = ble_app_get_env();
    LOG_I("app gattc event handler %d\r\n", event_id);
    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        LOG_I("client register ret %d\r\n", rsp->status);
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        LOG_I("Notify(handle %d) received", ind->handle);
        LOG_HEX("Notify content", 16, ind->value, ind->length);
        // Notify upper layer
        break;
    }
    case SIBLES_READ_REMOTE_VALUE_RSP:
    {
        sibles_read_remote_value_rsp_t *rsp = (sibles_read_remote_value_rsp_t *)data;
        LOG_I("Read(handle %d) received", rsp->handle);
        LOG_HEX("Read content", 16, rsp->value, rsp->length);
        break;
    }
    default:
        break;
    }
    return 0;
}

uint8_t ble_app_dis_enable()
{
    return 1;
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
    case BLE_GAP_CREATE_CONNECTION_CNF:
    {
        ble_gap_create_connection_cnf_t *cnf = (ble_gap_create_connection_cnf_t *)data;
        if (cnf->status != HL_ERR_NO_ERROR)
            LOG_E("Create connection failed %d!", cnf->status);
        break;
    }
    case BLE_GAP_CANCEL_CREATE_CONNECTION_CNF:
    {
        ble_gap_create_connection_cnf_t *cnf = (ble_gap_create_connection_cnf_t *)data;
        LOG_I("Create connection cancel status: %d", cnf->status);
        break;
    }
    case CONNECTION_MANAGER_CONNCTED_IND:
    {
        connection_manager_connect_ind_t *ind = (connection_manager_connect_ind_t *)data;
        if (env->conn_count == BLE_APP_MAX_CONN_COUNT)
        {
            LOG_E("Exceed maximum link number(%d)!", BLE_APP_MAX_CONN_COUNT);
            break;
        } else {
            LOG_I("restart adv");
            sibles_advertising_start(g_app_advertising_context);
        }

        ble_app_device_connected(env, ind);
        sibles_exchange_mtu(ind->conn_idx);
        // if (ind->role == 0)
        //   connection_manager_create_bond(ind->conn_idx);
        
        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        uint8_t idx = ble_app_get_dev_by_idx(env, ind->conn_idx);
        if (idx == 0xFF)
            break;
        env->conn[idx].conn_interval = ind->con_interval;
        LOG_I("Updated device %d connection interval :%d", idx, ind->con_interval);
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        /* Negotiated MTU. */
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        uint8_t idx = ble_app_get_dev_by_idx(env, ind->conn_idx);
        if (idx == 0xFF)
            break;

        env->conn[idx].mtu = ind->mtu;
        LOG_I("Exchanged device %d MTU size: %d", idx, ind->mtu);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        uint8_t idx = ble_app_get_dev_by_idx(env, ind->conn_idx);
        if (idx == 0xFF)
            break;

        ble_app_deivce_disconnected(env, idx, ind->reason);

        sibles_advertising_start(g_app_advertising_context);
        break;
    }
    case SIBLES_WRITE_VALUE_RSP:
    {
        sibles_write_value_rsp_t *rsp = (sibles_write_value_rsp_t *)data;
        LOG_I("SIBLES_WRITE_VALUE_RSP %d", rsp->result);
        break;
    }
    case BLE_GAP_SCAN_START_CNF:
    {
        ble_gap_start_scan_cnf_t *cnf = (ble_gap_start_scan_cnf_t *)data;
        LOG_I("Scan start status %d", cnf->status);
        env->adv_count = 0;
        break;
    }
    case BLE_GAP_SCAN_STOPPED_IND:
    {
        ble_gap_scan_stopped_ind_t *ind = (ble_gap_scan_stopped_ind_t *)data;
        LOG_I("Scan stopped %d", ind->reason);
        break;
    }
    case BLE_GAP_EXT_ADV_REPORT_IND:
    {
        ble_gap_ext_adv_report_ind_t *ind = (ble_gap_ext_adv_report_ind_t *)data;
        if (ble_app_adv_filter(env, ind) == true)
            break;

        if (ind->rssi < env->scan_rssi)
            break;

        if (env->adv_count == BLE_APP_MAX_ADV_COUNT)
        {
            break;
        }
        ble_app_adv_add(env, ind);
        ble_app_display_adv_context(ind, env->adv_count);
        break;
    }
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;
        if (rsp->result != HL_ERR_NO_ERROR)
            break; // Do nothing

        uint8_t dev_idx = ble_app_get_dev_by_idx(env, rsp->conn_idx);
        if (dev_idx == 0xFF)
            break;

        memcpy(&env->conn[dev_idx].svc, rsp->svc, sizeof(sibles_svc_remote_svc_t));
        env->conn[dev_idx].svc.att_db = malloc(env->conn[dev_idx].svc.char_count *
                                               (sizeof(sibles_svc_search_char_t) + APP_MAX_DESC * sizeof(struct sibles_disc_char_desc_ind)));

        if (env->conn[dev_idx].svc.att_db == NULL)
            break;

        LOG_I("Service searched");

        uint32_t i;
        uint16_t offset = 0;
        uint8_t desc_num;
        sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
        sibles_svc_search_char_t *curr_chara = (sibles_svc_search_char_t *)env->conn[dev_idx].svc.att_db;
        for (i = 0; i < rsp->svc->char_count; i++)
        {
            desc_num = chara->desc_count > APP_MAX_DESC ? APP_MAX_DESC : chara->desc_count;
            memcpy(curr_chara, chara, sizeof(sibles_svc_search_char_t) + desc_num * sizeof(struct sibles_disc_char_desc_ind));

            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
            offset = sizeof(sibles_svc_search_char_t) + desc_num * sizeof(struct sibles_disc_char_desc_ind);
            curr_chara = (sibles_svc_search_char_t *)((uint8_t *)curr_chara + offset);
        }

        ble_app_display_service(&env->conn[dev_idx].svc);
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);


static uint8_t ble_app_create_connection(ble_gap_addr_t *peer_addr, uint8_t own_addr_type, uint16_t super_timeout,
        uint16_t conn_itv, uint16_t scan_itv, uint16_t scan_wd)
{
    app_env_t *env = ble_app_get_env();
    ble_gap_connection_create_param_t *conn_param = &env->last_init_conn;
    conn_param->own_addr_type = own_addr_type;
    conn_param->conn_to = super_timeout;
    conn_param->type = GAPM_INIT_TYPE_DIRECT_CONN_EST;
    conn_param->conn_param_1m.scan_intv = scan_itv;
    conn_param->conn_param_1m.scan_wd = scan_wd;
    conn_param->conn_param_1m.conn_intv_max = conn_itv;
    conn_param->conn_param_1m.conn_intv_min = conn_itv == 12 ? conn_itv : (conn_itv - 16);
    conn_param->conn_param_1m.conn_latency = 0;
    conn_param->conn_param_1m.supervision_to = super_timeout;
    conn_param->conn_param_1m.ce_len_max = 48;
    conn_param->conn_param_1m.ce_len_min = 0;
    memcpy(&conn_param->peer_addr, peer_addr, sizeof(ble_gap_addr_t));
    return ble_gap_create_connection(conn_param);
}

static uint8_t ble_app_reconnect(void)
{
    app_env_t *env = ble_app_get_env();
    return ble_gap_create_connection(&env->last_init_conn);
}

static void write_cccd(uint8_t current_conn_idx, uint16_t write_handle)
{
    app_env_t *env = ble_app_get_env();

    sibles_write_remote_value_t value;
    value.handle = write_handle;
    value.write_type = SIBLES_WRITE_WITHOUT_RSP;
    value.len = 2;

    uint8_t *write_data = malloc(2);
    *write_data = 0x01;
    *(write_data + 1) = 0x00;

    value.value = (uint8_t *)write_data;

    sibles_write_remote_value(env->conn[current_conn_idx].rmt_svc_hdl, env->conn[current_conn_idx].conn_idx, &value);
    LOG_I("current_conn_idx %d, conn_idx %d", current_conn_idx, env->conn[current_conn_idx].conn_idx);


    free(write_data);
}


static void ble_app_display_command(void)
{
    LOG_I("Command list:");
    LOG_I("diss conn_idx. Connected with advertising index device. Details see: diss conn_idx help.");
    LOG_I("diss reconn. Reconnected last initial connected device.");
    LOG_I("diss scan. Scan advertising device. Details see: diss scan help.");
    LOG_I("diss search_svc [dev_idx] [len] [uuid]. Search GATT service with speicifed uuid for peer device.");
    LOG_I("diss up_conn [dev_idx] [interval max in ms] [interval min in ms] [latency] [supervision timeout in ms]. Update connection parameter.");
    LOG_I("diss up_phy [dev_idx] [phy, 0:1M, 1:2M, 2:coded]. Update physical.");
    LOG_I("diss up_len [dev_idx] [tx_octets] [tx_time]. Update data length.");
    LOG_I("diss exch_mtu [dev_idx]. Trigger exchange MTU.");
    LOG_I("diss show_dev. Show all connected device with device index.");
    LOG_I("diss bond [dev_idx]. Bonded peer device.");
    LOG_I("diss delete all_bond. Delete all bonded devices.");
    LOG_I("diss show_rmt_service [dev_idx]. Show last searched service.");
    LOG_I("diss read_val [dev_idx] [handle]. Read data via dedicated gatt handle.");
    LOG_I("diss write_val [dev_idx] [handle] [data_len] [data]. Write data via dedicated gatt handle.");
    LOG_I("diss adv_update. Updata advertising content.");
}

int cmd_diss(int argc, char *argv[])
{
    app_env_t *env = ble_app_get_env();
    if (argc > 1)
    {

        if (strcmp(argv[1], "conn_idx") == 0)
        {
            if (strcmp(argv[2], "start") == 0)
            {
                uint8_t idx = atoi(argv[3]);
                if (idx > env->adv_count || idx == 0)
                {
                    LOG_E("Wrongly idx(%d), Shall not equal 0 or exceed maximum idx(%d)", idx, env->adv_count);
                    return 0;
                }
                uint8_t own_addr_type = atoi(argv[4]);
                uint16_t timeout = atoi(argv[5]) / 10;
                uint16_t conn_itv = atoi(argv[6]) * 4 / 5;
                uint16_t scan_itv = atoi(argv[7]) * 8 / 5;
                uint8_t scan_wd = atoi(argv[8]) * 8 / 5;
                uint8_t ret = ble_app_create_connection(&env->adv_addr[idx - 1], own_addr_type, timeout, conn_itv, scan_itv, scan_wd);
                if (ret != HL_ERR_NO_ERROR)
                    LOG_E("Create connection failed %d", ret);
            }
            else if (strcmp(argv[2], "cancel") == 0)
            {
                ble_gap_cancel_create_connection();
            }
            else if (strcmp(argv[2], "addr") == 0)
            {
                int i;
                ble_gap_connection_create_param_t conn_param;
                conn_param.own_addr_type = GAPM_STATIC_ADDR;
                conn_param.conn_to = 500;
                conn_param.type = GAPM_INIT_TYPE_DIRECT_CONN_EST;
                conn_param.conn_param_1m.scan_intv = 0x30;
                conn_param.conn_param_1m.scan_wd = 0x30;
                conn_param.conn_param_1m.conn_intv_max = 0x80;
                conn_param.conn_param_1m.conn_intv_min = 0x60;
                conn_param.conn_param_1m.conn_latency = 0;
                conn_param.conn_param_1m.supervision_to = 500;
                conn_param.conn_param_1m.ce_len_max = 48;
                conn_param.conn_param_1m.ce_len_min = 0;
                conn_param.peer_addr.addr_type = atoi(argv[3]);

                hex2data(argv[4], conn_param.peer_addr.addr.addr, BD_ADDR_LEN);

                ble_gap_create_connection(&conn_param);
            }
            else
            {
                LOG_I("Create connection: diss conn_idx start [idx, select from adv idx] [own_addr_type, 0(public/randam)/1(resolve)]");
                LOG_I("[super_timeout(ms)] [conn_itv(ms)] [scan_itv(ms)] [scan_wd(ms)]");
                LOG_I("Create connection via addres: diss conn_idx addr [own_addr_type] [peer_addr, aabbccddeeff]");
                LOG_I("Cancel connection: diss conn_idx cancel");
            }

        }
        else if (strcmp(argv[1], "reconn") == 0)
        {
            uint8_t ret = ble_app_reconnect();
            if (ret != HL_ERR_NO_ERROR)
                LOG_E("Reconnection failed %d", ret);
        }
        else if (strcmp(argv[1], "scan") == 0)
        {
            if (strcmp(argv[2], "start") == 0)
            {
                ble_gap_scan_start_t scan_param;
                scan_param.own_addr_type = GAPM_STATIC_ADDR;
                scan_param.type = GAPM_SCAN_TYPE_OBSERVER;
                scan_param.dup_filt_pol = atoi(argv[3]);
                scan_param.scan_param_1m.scan_intv = atoi(argv[4]) * 8 / 5;
                scan_param.scan_param_1m.scan_wd = atoi(argv[5]) * 8 / 5;
                scan_param.duration = atoi(argv[6]) / 10;
                scan_param.period = 0;
                env->scan_rssi = atoi(argv[7]);
                ble_gap_scan_start(&scan_param);
            }
            else if (strcmp(argv[2], "stop") == 0)
            {
                ble_gap_scan_stop();
            }
            else
            {
                LOG_I("Scan start: diss scan start [dup, 0/1] [interval, ms] [window, ms] [duration, ms] [received_rssi]");
                LOG_I("Scan stop: diss scan stop");
            }
        }
        else if (strcmp(argv[1], "search_svc") == 0)
        {
            int i, len;
            uint8_t uuid[128] = {0};
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);

            len = atoi(argv[3]);

            hex2data(argv[4], uuid, len);
            //sibles_attm_convert_to128(uuid, uuid, len);
            sibles_search_service(env->conn[idx].conn_idx, len, uuid);
        }
        else if (strcmp(argv[1], "up_conn") == 0)
        {
            ble_gap_update_conn_param_t conn_para;
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);

            conn_para.conn_idx =  env->conn[idx].conn_idx;
            conn_para.intv_max = (uint16_t)atoi(argv[3]) * 4 / 5;
            conn_para.intv_min = (uint16_t)atoi(argv[4]) * 4 / 5;
            // value = argv * 1.25
            conn_para.ce_len_max = 0x100;
            conn_para.ce_len_min = 0x1;
            conn_para.latency = (uint16_t)atoi(argv[5]);
            conn_para.time_out = (uint16_t)atoi(argv[6]) / 10;
            ble_gap_update_conn_param(&conn_para);
        }
        else if (strcmp(argv[1], "up_phy") == 0)
        {
            ble_gap_update_phy_t phy;
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);

            phy.conn_idx = env->conn[idx].conn_idx;
            phy.rx_phy = 1 << atoi(argv[3]);
            phy.tx_phy = 1 << atoi(argv[3]);
            phy.phy_opt = 0;
            ble_gap_update_phy(&phy);
        }
        else if (strcmp(argv[1], "up_len") == 0)
        {
            ble_gap_update_data_len_t data_len;
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);
            data_len.conn_idx = env->conn[idx].conn_idx;
            data_len.tx_octets = (uint16_t)atoi(argv[3]);
            data_len.tx_time = (uint16_t)atoi(argv[4]);
            ble_gap_update_data_len(&data_len);
        }
        else if (strcmp(argv[1], "exch_mtu") == 0)
        {
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);
            sibles_exchange_mtu(env->conn[idx].conn_idx);
        }
        else if (strcmp(argv[1], "show_dev") == 0)
        {
            ble_app_display_connected_device(env);
        }
        else if (strcmp(argv[1], "show_rmt_service") == 0)
        {
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);
            ble_app_display_service(&env->conn[idx].svc);
        }
        else if (strcmp(argv[1], "delete") == 0)
        {
            if (strcmp(argv[2], "all_bond") == 0)
            {
                connection_manager_delete_all_bond();
            }
        }
        else if (strcmp(argv[1], "read_val") == 0)
        {
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);
            sibles_read_remote_value_req_t value;
            value.read_type = SIBLES_READ;
            value.handle = (uint16_t)atoi(argv[3]);
            value.length = 0;
            value.offset = 0;
            int8_t ret = sibles_read_remote_value(env->conn[idx].rmt_svc_hdl, env->conn[idx].conn_idx, &value);
            if (ret != 0)
                LOG_E("Read remote value ");
        }
        else if (strcmp(argv[1], "write_val") == 0)
        {
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);
            sibles_write_remote_value_t value;
            value.handle = (uint16_t)atoi(argv[3]);
            value.write_type = SIBLES_WRITE;
            value.len = (uint16_t)atoi(argv[4]);
            uint8_t *w_data = malloc(value.len);
            RT_ASSERT(w_data);
            hex2data(argv[5], w_data, value.len);
            LOG_HEX("write context", 16, w_data, value.len);
            value.value = (uint8_t *)w_data;
            int8_t ret = sibles_write_remote_value(env->conn[idx].rmt_svc_hdl, env->conn[idx].conn_idx, &value);
            if (ret != 0)
                LOG_E("Write remote value ");
            free(w_data);
        }
        else if (strcmp(argv[1], "write_test") == 0)
        {
            uint8_t idx = (uint16_t)atoi(argv[2]);
            uint16_t handle = (uint16_t)atoi(argv[3]);
            uint32_t data_count = (uint16_t)atoi(argv[4]);
            uint32_t data_length = (uint16_t)atoi(argv[5]);
            LOG_I("test start %d %d %d %d", idx, handle, data_count, data_length);
            ble_write_to_remote_master(idx, handle, data_count, data_length);
            LOG_I("test end");
        }
        else if (strcmp(argv[1], "bond") == 0)
        {
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);
            connection_manager_create_bond(env->conn[idx].conn_idx);
        }
        else if (strcmp(argv[1], "write_cccd") == 0)
        {
            uint8_t conn_idx = (uint8_t)atoi(argv[2]);
            uint16_t write_handle = (uint16_t)atoi(argv[3]);

            write_cccd(conn_idx, write_handle);
        }
        else if (strcmp(argv[1], "adv_update") == 0)
        {
            update_adv_content();
        }
 #ifdef BSP_BLE_TIMEC
        else if (strcmp(argv[1], "tipc") == 0)
        {
            if (strcmp(argv[2], "enabled") == 0)
            {
                uint8_t ti_index = atoi(argv[3]);
                LOG_I("conn(%d)\r\n", 0);
                ble_tipc_enable(ti_index);
            }
            else if (strcmp(argv[2], "read") == 0)
            {
                uint8_t read_type = atoi(argv[3]); // 0. curr time. 1. local time info
                uint8_t ti_index = atoi(argv[4]);
                if (read_type == 0)
                {
                    ble_tipc_read_current_time(ti_index);
                }
                else if (read_type == 1)
                {
                    ble_tipc_read_local_time_info(ti_index);
                }
            }
        }
#endif
        else if (strcmp(argv[1], "hrpc") == 0)
        {
            if (strcmp(argv[2], "enabled") == 0)
            {
                uint8_t hr_index = atoi(argv[3]);
                LOG_I("conn(%d)\r\n", hr_index);
                ble_hrpc_enable(hr_index);
            }
        }
        else if (strcmp(argv[1], "start_adv") == 0)
        {
            sibles_advertising_start(g_app_advertising_context);
        }
        else
        {
            ble_app_display_command();
        }
    }

    return 0;
}


FINSH_FUNCTION_EXPORT_ALIAS(cmd_diss, __cmd_diss, My device information service.);



#ifdef BSP_BLE_HRPC
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
#endif


int profile_msg_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    switch (event_id)
    {
#ifdef BSP_BLE_HRPC
    case BLE_HRPC_HREAT_RATE_NOTIFY:
    {
        ble_hrpc_heart_rate_t *res = (ble_hrpc_heart_rate_t *)data;
        if ((res->flags & HRS_FLAG_SENSOR_CCT_FET_SUPPORTED) == HRS_FLAG_SENSOR_CCT_FET_SUPPORTED)
        {
            if ((res->flags & HRS_FLAG_SENSOR_CCT_DETECTED) == HRS_FLAG_SENSOR_CCT_NOT_DETECTED)
                LOG_I("sensor not contact with skin!");
        }
        if ((res->flags & HRS_FLAG_ENERGY_EXPENDED_PRESENT) == HRS_FLAG_ENERGY_EXPENDED_PRESENT)
        {
            LOG_I("energy expendend is %d kj", res->energy_expended);
        }

        for (int i = 0; i < res->nb_rr_interval; i++)
        {
            LOG_I("RR-Intervals[%d] is %d", i, res->rr_intervals[i]);
        }
        LOG_I("heart rate is %d", res->heart_rate);
        break;
    }
    case BLE_HRPC_READ_BODY_SENSOR_LOCATION_RSP:
    {
        ble_hrpc_body_sensor_location_t *res = (ble_hrpc_body_sensor_location_t *)data;
        LOG_I("body location is %s\n", body_location[res->location]);
        break;
    }
#endif
#ifdef BSP_BLE_BASC
    case BLE_BASC_BAT_LEV_NOTIFY:
    {
        ble_basc_bat_lev_t *res = (ble_basc_bat_lev_t *)data;
        LOG_I("BLE_BASC_BAT_LEV_NOTIFY-battery level is %d\n", res->lev);
        break;
    }
    case BLE_BASC_READ_BAT_LEV_RSP:
    {
        ble_basc_bat_lev_t *res = (ble_basc_bat_lev_t *)data;
        LOG_I("BLE_BASC_READ_BAT_LEV_RSP-battery level is %d\n", res->lev);
        break;
    }
#endif
#ifdef BSP_BLE_CSCPC
    case BLE_CSCPC_CSC_MEASUREMENT_NOTIFY:
    {
        ble_csc_meas_value_ind const *rsp = (ble_csc_meas_value_ind const *) data;
        ble_csc_meas_value_ind ind;
        ind.att_code = rsp->att_code;
        ind.csc_meas.flags = rsp->csc_meas.flags;
        ind.csc_meas.cumul_wheel_rev = rsp->csc_meas.cumul_wheel_rev;
        ind.csc_meas.last_wheel_evt_time = rsp->csc_meas.last_wheel_evt_time;
        ind.csc_meas.cumul_crank_rev = rsp->csc_meas.cumul_crank_rev;
        ind.csc_meas.last_crank_evt_time = rsp->csc_meas.last_crank_evt_time;
        LOG_I("read csc meas flags is %d\r\n", ind.csc_meas.flags);
        LOG_I("read csc meas cumul_crank_rev is %d\r\n", ind.csc_meas.cumul_crank_rev);
        LOG_I("read csc meas cumul_wheel_rev is %d\r\n", ind.csc_meas.cumul_wheel_rev);
        LOG_I("read csc meas last_crank_evt_time is %d\r\n", ind.csc_meas.last_crank_evt_time);
        LOG_I("read csc meas last_wheel_evt_time is %d\r\n", ind.csc_meas.last_wheel_evt_time);
        break;
    }
    case BLE_CSCPC_CONTROL_POINT_INDICATE:
    {
        struct cscpc_ctnl_pt_rsp const *rsp = (struct cscpc_ctnl_pt_rsp const *)data;
        struct cscpc_ctnl_pt_rsp ind;
        ind.ctnl_pt_rsp.req_op_code = rsp->ctnl_pt_rsp.req_op_code;
        ind.ctnl_pt_rsp.resp_value = rsp->ctnl_pt_rsp.resp_value;
        ind.ctnl_pt_rsp.supp_loc = rsp->ctnl_pt_rsp.supp_loc;
        LOG_I("read ctnl pt opcode is %d\r\n", ind.ctnl_pt_rsp.req_op_code);
        LOG_I("read ctnl pt value is %d\r\n", ind.ctnl_pt_rsp.resp_value);
        LOG_I("read ctnl pt supp_loc is %d\r\n", ind.ctnl_pt_rsp.supp_loc);
        break;
    }
    case BLE_CSCPC_READ_CSC_FEATURE_RSP:
    {
        ble_csc_feat_value_ind const *rsp = (ble_csc_feat_value_ind const *)data;
        ble_csc_feat_value_ind ind;
        ind.att_code = rsp->att_code;
        ind.sensor_feat = rsp->sensor_feat;
        LOG_I("read csc feat code is %d\r\n", ind.att_code);
        LOG_I("read csc feat sensor feat is %d\r\n", ind.sensor_feat);
        break;
    }
    case BLE_CSCPC_READ_SENSOR_LOCATION_RSP:
    {
        ble_sensor_loc_value_ind const *rsp = (ble_sensor_loc_value_ind const *)data;
        ble_sensor_loc_value_ind ind;
        ind.att_code = rsp->att_code;
        ind.sensor_loc = rsp->sensor_loc;
        LOG_I("read sensor loc code is %d\r\n", ind.att_code);
        LOG_I("read sensor loc is %d\r\n", ind.sensor_loc);
        break;
    }
#endif
#ifdef BSP_BLE_CPPC
    case BLE_CPPC_CPM_NOTIFY:
    {
        ble_cppc_cpm *cpm = (ble_cppc_cpm *)data;
        LOG_I("cpm flags %x\r\n", cpm->flags);
        LOG_I("cpm inst_power %x\r\n", cpm->inst_power);
        if (cpm->flags & CPP_MEAS_PEDAL_POWER_BALANCE_PRESENT)
        {
            LOG_I("cpm pedal_power_balance %x\r\n", cpm->pedal_power_balance);
        }
        if (cpm->flags & CPP_MEAS_ACCUM_TORQUE_PRESENT)
        {
            LOG_I("cpm accum_torque %x\r\n", cpm->accum_torque);
        }
        if (cpm->flags & CPP_MEAS_WHEEL_REV_DATA_PRESENT)
        {
            LOG_I("cpm cumul_wheel_rev %x\r\n", cpm->cumul_wheel_rev);
            LOG_I("cpm last_wheel_evt_time %x\r\n", cpm->last_wheel_evt_time);
        }
        if (cpm->flags & CPP_MEAS_CRANK_REV_DATA_PRESENT)
        {
            LOG_I("cpm cumul_crank_rev %x\r\n", cpm->cumul_crank_rev);
            LOG_I("cpm last_crank_evt_time %x\r\n", cpm->last_crank_evt_time);
        }
        if (cpm->flags & CPP_MEAS_EXTREME_FORCE_MAGNITUDES_PRESENT)
        {
            LOG_I("cpm max_force_magnitude %x\r\n", cpm->max_force_magnitude);
            LOG_I("cpm min_force_magnitude %x\r\n", cpm->min_force_magnitude);
        }
        else if (cpm->flags & CPP_MEAS_EXTREME_TORQUE_MAGNITUDES_PRESENT)
        {
            LOG_I("cpm max_torque_magnitude %x\r\n", cpm->max_torque_magnitude);
            LOG_I("cpm min_torque_magnitude %x\r\n", cpm->min_torque_magnitude);
        }
        if (cpm->flags & CPP_MEAS_EXTREME_ANGLES_PRESENT)
        {
            LOG_I("cpm max_angle %x\r\n", cpm->max_angle);
            LOG_I("cpm min_angle %x\r\n", cpm->min_angle);
        }
        if (cpm->flags & CPP_MEAS_TOP_DEAD_SPOT_ANGLE_PRESENT)
        {
            LOG_I("cpm top_dead_spot_angle %x\r\n", cpm->top_dead_spot_angle);
        }
        if (cpm->flags & CPP_MEAS_BOTTOM_DEAD_SPOT_ANGLE_PRESENT)
        {
            LOG_I("cpm bot_dead_spot_angle %x\r\n", cpm->bot_dead_spot_angle);
        }
        if (cpm->flags & CPP_MEAS_ACCUM_ENERGY_PRESENT)
        {
            LOG_I("cpm accum_energy %x\r\n", cpm->accum_energy);
        }
        break;
    }
    case BLE_CPPC_CPV_NOTIFY:
    {

        ble_cppc_cpv *cpv = (ble_cppc_cpv *)data;

        LOG_I("cpv flags %x\r\n", cpv->flags);
        if (cpv->flags & CPP_VECTOR_CRANK_REV_DATA_PRESENT)
        {
            LOG_I("cpv cumul_crank_rev %x\r\n", cpv->cumul_crank_rev);

            LOG_I("cpv last_crank_evt_time %x\r\n", cpv->last_crank_evt_time);

        }
        if (cpv->flags & CPP_VECTOR_FIRST_CRANK_MEAS_ANGLE_PRESENT)
        {

            LOG_I("cpv first_crank_meas_angle %x\r\n", cpv->first_crank_meas_angle);

        }
        if (!(cpv->flags & CPP_VECTOR_INST_FORCE_MAGNITUDE_ARRAY_PRESENT) !=
                !(cpv->flags & CPP_VECTOR_INST_TORQUE_MAGNITUDE_ARRAY_PRESENT))
        {

            if (cpv->nb)
            {
                for (int i = 0; i < cpv->nb; i++)
                {
                    LOG_I("cpv force_torque_magnitude(%u) %x\r\n", i, cpv->force_torque_magnitude[i]);
                }
            }
        }
        break;
    }
    case BLE_CPPC_CPPC_NOTIFICATION_IND:
    {
        ble_cpcp_notyf_rsp *cppc = (ble_cpcp_notyf_rsp *)data;
        LOG_I("cppc req_op_code %x\r\n", cppc->req_op_code);
        LOG_I("cppc resp_value %x\r\n", cppc->resp_value);

        if ((cppc->resp_value == CPP_CTNL_PT_RESP_SUCCESS) && (len >= 3))
        {
            switch (cppc->req_op_code)
            {
            case (CPP_CTNL_PT_REQ_SUPP_SENSOR_LOC):
            {
                // Get the number of supported locations that have been received
                LOG_I("cppc supp_loc %x\r\n", cppc->value.supp_loc);
            }
            break;
            case (CPP_CTNL_PT_REQ_CRANK_LENGTH):
            {
                LOG_I("cppc crank_length %x\r\n", cppc->value.crank_length);
            }
            break;
            case (CPP_CTNL_PT_REQ_CHAIN_LENGTH):
            {
                LOG_I("cppc chain_length %x\r\n", cppc->value.chain_length);
            }
            break;
            case (CPP_CTNL_PT_REQ_CHAIN_WEIGHT):
            {
                LOG_I("cppc chain_weight %x\r\n", cppc->value.chain_weight);
            }
            break;
            case (CPP_CTNL_PT_REQ_SPAN_LENGTH):
            {
                LOG_I("cppc span_length %x\r\n", cppc->value.span_length);
            }
            break;
            case (CPP_CTNL_PT_START_OFFSET_COMP):
            {
                LOG_I("cppc Start Offset Compensation %x\r\n", cppc->value.offset_comp);
            }
            break;
            case (CPP_CTNL_REQ_SAMPLING_RATE):
            {

                LOG_I("cppc sampling_rate %x\r\n", cppc->value.sampling_rate);
            }
            break;
            case (CPP_CTNL_REQ_FACTORY_CALIBRATION_DATE):
            {
                LOG_I("cppc Calibration date year %u,month %u,day %u,hour %u,min %u,sec %u \r\n", cppc->value.factory_calibration.year
                      , cppc->value.factory_calibration.month, cppc->value.factory_calibration.day, cppc->value.factory_calibration.hour
                      , cppc->value.factory_calibration.min, cppc->value.factory_calibration.sec);
            }
            break;
            case (CPP_CTNL_PT_SET_CUMUL_VAL):
            case (CPP_CTNL_PT_UPD_SENSOR_LOC):
            case (CPP_CTNL_PT_SET_CRANK_LENGTH):
            case (CPP_CTNL_PT_SET_CHAIN_LENGTH):
            case (CPP_CTNL_PT_SET_CHAIN_WEIGHT):
            case (CPP_CTNL_PT_SET_SPAN_LENGTH):
            case (CPP_CTNL_MASK_CP_MEAS_CH_CONTENT):
            {
                // No parameters
            } break;

            default:
            {

            } break;
            }
        }
        break;
    }
    case BLE_CPPC_READ_CPF_RSP:
    {
        ble_cppc_cpf *cpf = (ble_cppc_cpf *)data;
        LOG_I("cpf CP Feature %x\r\n", cpf->sensor_feat);
        break;
    }
    case BLE_CPPC_READ_SL_RSP:
    {
        ble_cppc_sl *sl = (ble_cppc_sl *)data;
        LOG_I("sl Sensor Location %x\r\n", sl->sensor_loc);
        break;
    }
    case BLE_CPPC_READ_CPCP_CEP_RSP:
    {
        ble_cppc_cep *cep = (ble_cppc_cep *)data;
        LOG_I("cep cep_val %x\r\n", cep->cep_val);
        break;
    }
#endif
    }
    return 0;
}

BLE_EVENT_REGISTER(profile_msg_handler, (uint32_t)NULL);

#ifdef SF32LB52X
#include "rtconfig.h"
#include "bf0_hal_lcpu_config.h"
#include "string.h"

#define CO_BIT(pos) (1UL<<(pos))

uint16_t g_em_offset[HAL_LCPU_CONFIG_EM_BUF_MAX_NUM] =
{
0x178 , 0x178 , 0x990 , 0xA20 , 0xBE0 , 0xCC0 , 0x1120, 0x13F0, 0x1788, 0x3058, 
0x30C0, 0x3AE8, 0x46E8, 0x46E8, 0x46E8, 0x46E8, 0x46E8, 0x46E8, 0x46E8, 0x46E8, 
0x48C8, 0x48C8, 0x48F8, 0x4948, 0x4A68, 0x4A7C, 0x4A90, 0x4B80, 0x4B90, 0x4B90, 
0x4BAC, 0x53B4, 0x5BBC, 0x5BBC,
};

void lcpu_rom_config(void)
{
    uint8_t is_config_allowed = 0;
#ifdef SF32LB52X
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id >= HAL_CHIP_REV_ID_A4)
        is_config_allowed = 1;
#endif

    extern void lcpu_rom_config_default(void);
    lcpu_rom_config_default();

    if (is_config_allowed)
    {
        hal_lcpu_bluetooth_em_config_t em_offset;
        memcpy((void *)em_offset.em_buf, (void *)g_em_offset, HAL_LCPU_CONFIG_EM_BUF_MAX_NUM * 2);
        em_offset.is_valid = 1;
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_EM_BUF, &em_offset, sizeof(hal_lcpu_bluetooth_em_config_t));

        hal_lcpu_bluetooth_act_configt_t act_cfg;
        act_cfg.ble_max_act = 10;
        act_cfg.ble_max_iso = 0;
        act_cfg.ble_max_ral = 8;
        act_cfg.bt_max_acl = 0;
        act_cfg.bt_max_sco = 0;
        act_cfg.bit_valid = CO_BIT(0) | CO_BIT(1) | CO_BIT(2) | CO_BIT(3) | CO_BIT(4);
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_ACT_CFG, &act_cfg, sizeof(hal_lcpu_bluetooth_act_configt_t));
    }
}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

