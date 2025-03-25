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
    BLE_SPEED_TEST_STATE_IDLE,
    BLE_SPEED_TEST_STATE_SEARCHING,
    BLE_SPEED_TEST_STATE_SEARCH_COMPLETED,
    BLE_SPEED_TEST_STATE_CONFIGURAING,
    BLE_SPEED_TEST_STATE_READY
} ble_ancs_state_t;

typedef enum
{
    BLE_APP_UUID_TYPE_SERVICE,
    BLE_APP_UUID_TYPE_CHARATER,
    BLE_APP_UUID_TYPE_DESCRIPTOR,
    BLE_APP_UUID_TYPE_TOTAL
} ble_app_uuid_display_type_t;

#define BLE_SPEED_TEST_FLAG (0x1312)

typedef enum
{
    BLE_SPEED_COMMAND_START_REQ,
    BLE_SPEED_COMMAND_START_RSP,
    BLE_SPEED_COMMAND_PACKET,
    BLE_SPEED_COMMAND_PACKET_RSP,
    BLE_SPEED_COMMAND_END_REQ,
    BLE_SPEED_COMMAND_END_RSP,
} ble_app_speed_command_type_t;

typedef struct
{
    uint16_t attr_hdl;
    uint16_t value_hdl;
    uint8_t prop;
    uint16_t cccd_hdl;
} ble_app_char_t;



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
#define BLE_APP_MAX_CONN_COUNT 4
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
        uint8_t search_state;
        uint16_t mtu;
        uint16_t conn_interval;
        bd_addr_t peer_addr;

        uint16_t hdl_start;
        uint16_t hdl_end;
        ble_app_char_t app_char;
        uint16_t remote_handle;

        struct
        {
            uint32_t data;
            uint8_t is_config_on;
        } data;
        sibles_svc_remote_svc_t svc;
    } conn[BLE_APP_MAX_CONN_COUNT];

    sibles_hdl srv_handle;

    rt_mailbox_t mb_handle;

    uint8_t test_type;
    uint8_t response_frequency;
    uint8_t is_test_on;
    uint8_t conn_level;

    uint16_t packet_len;
    uint16_t local_mtu;
    uint32_t packet_count;

    uint32_t time_start;
    uint32_t time_end;
    uint32_t current_index;

} app_env_t;

static app_env_t g_app_env;
static rt_mailbox_t g_app_mb;

static uint8_t g_app_svc[ATT_UUID_128_LEN] = app_svc_uuid;

static void ble_speed_test_handler(uint8_t conn_idx, uint8_t *data, uint16_t size);
static void send_test_end_request(uint8_t conn_idx);

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


static uint8_t ble_app_get_dev_by_idx(app_env_t *env, uint8_t conn_idx);

static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}


static void write_cccd(uint8_t current_conn_idx, uint16_t write_handle)
{
    app_env_t *env = ble_app_get_env();

    sibles_write_remote_value_t value;
    value.handle = write_handle;
    value.write_type = SIBLES_WRITE;
    value.len = 2;

    uint8_t *write_data = malloc(2);
    *write_data = 0x01;
    *(write_data + 1) = 0x00;

    value.value = (uint8_t *)write_data;

    sibles_write_remote_value(env->conn[current_conn_idx].remote_handle, env->conn[current_conn_idx].conn_idx, &value);
    LOG_I("current_conn_idx %d, conn_idx %d", current_conn_idx, env->conn[current_conn_idx].conn_idx);


    free(write_data);
}

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
        // uint32_t old_val = env->conn[idx].data.data;
        // if (para->len <= 4)
        //     memcpy(&env->conn[idx].data.data, para->value, para->len);
        // LOG_I("Device %d updated app value from:%d to:%d", idx, old_val, env->conn[idx].data.data);
        // if ((env->conn[idx].data.data & 0xFF) == 0x11)
        // {
        //     env->conn[idx].data.data = env->conn[idx].data.data & 0xFFFFFF00 + idx;
        //     LOG_I("example of ble notification %d", env->conn[idx].data.data);
        //     ble_app_start_send_thread(env->conn[idx].data.data);
        // }

        // if ((env->conn[idx].data.data & 0xFF) == 0x11)
        // {
        //     env->conn[idx].data.data = env->conn[idx].data.data & 0xFFFFFF00 + idx;
        //     LOG_I("example of ble notification %d", env->conn[idx].data.data);
        //     ble_app_start_send_thread(env->conn[idx].data.data);
        // }

        if (para->len >= 2)
        {
            uint16_t flag;
            memcpy(&flag, para->value, 2);
            if (flag == BLE_SPEED_TEST_FLAG)
            {
                ble_speed_test_handler(conn_idx, para->value, para->len);
            }
            else
            {
                LOG_I("NO FLAG");
            }
        }
        else
        {
            LOG_I("NO LEN");
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

static int8_t send_speed_test_packet(uint8_t *data, uint16_t len, uint8_t conn_idx)
{
    app_env_t *env = ble_app_get_env();

    sibles_write_remote_value_t value;
    value.handle = env->conn[conn_idx].app_char.value_hdl;
    value.write_type = SIBLES_WRITE_WITHOUT_RSP;
    value.len = len + 4;
    uint16_t command_flag = BLE_SPEED_TEST_FLAG;


    uint8_t *write_data = malloc(value.len);
    memcpy(write_data, &command_flag, 2);
    memcpy(write_data + 2, &len, 2);
    // *write_data = BLE_SPEED_TEST_FLAG;
    // *(write_data + 1) = 0x00;
    memcpy(write_data + 4, data, len);

    value.value = (uint8_t *)write_data;
    int8_t ret = sibles_write_remote_value(env->conn[conn_idx].remote_handle, conn_idx, &value);

    free(write_data);
    return ret;
}

static int send_speed_test_packet_notify(uint8_t *data, uint16_t len, uint8_t conn_idx)
{
    app_env_t *env = ble_app_get_env();
    uint8_t *raw_data = malloc(len + 4);

    uint16_t command_flag = BLE_SPEED_TEST_FLAG;
    memcpy(raw_data, &command_flag, 2);
    memcpy(raw_data + 2, &len, 2);
    memcpy(raw_data + 4, data, len);

    sibles_value_t value;
    value.hdl = env->srv_handle;
    value.idx = BLE_APP_CHAR_VALUE;
    value.len = len + 4;
    value.value = raw_data;

    int ret = sibles_write_value(conn_idx, &value);
    free(raw_data);
    return ret;
}

uint16_t ble_max_mtu_get()
{
    app_env_t *env = ble_app_get_env();
    if (env->local_mtu > 23 && env->local_mtu < 512)
    {
        return env->local_mtu;
    }
    return 512;
}

void ble_test_ctrl(uint8_t *addr)
{
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
    conn_param.conn_param_1m.ce_len_max = 100;
    conn_param.conn_param_1m.ce_len_min = 60;

    conn_param.peer_addr.addr_type = 0;
    memcpy(conn_param.peer_addr.addr.addr, addr, BD_ADDR_LEN);

    ble_gap_create_connection(&conn_param);
}


static void ble_app_speed_write(void *param)
{
    app_env_t *env = ble_app_get_env();
    uint32_t para = (uint32_t)param;

    uint32_t data_count = (para & 0xFFFF0000) >> 16;
    uint8_t conn_idx = para & 0xFF;

    LOG_I("ble_app_speed_write %d", data_count);
    uint16_t raw_data_len = env->packet_len + 5;

    uint8_t *raw_data = malloc(raw_data_len);
    uint32_t data_index = env->current_index;


    while (data_index <= data_count)
    {
        // here is notify example
        *raw_data = BLE_SPEED_COMMAND_PACKET;
        memcpy(raw_data + 1, &data_index, 4);
        memset(raw_data + 5, 0, raw_data_len - 5);


        int ret;
        //LOG_I("SEND INDEX %d", data_index);
        ret = send_speed_test_packet(raw_data, raw_data_len, conn_idx);

        //ret = sibles_write_value(env->conn[idx].conn_idx, &value);

        if (ret == 0)
        {
            // write success
            data_index++;
        }
        else if (ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
        {
            // tx queue is full, wait and retry
            int retry = 20;
            while (retry > 0)
            {
                retry--;
                rt_thread_mdelay(50);
                ret = send_speed_test_packet(raw_data, raw_data_len, conn_idx);
                if (ret == 0)
                {
                    LOG_I("send retry success %d", data_index);
                    data_index++;
                    break;
                }


                if (retry == 0 && ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
                {
                    LOG_I("write limit reach");
                }
            }
        }


        if (env->response_frequency != 0)
        {
            if (env->current_index % env->response_frequency == 0)
            {
                env->current_index++;
                break;
            }
        }

        env->current_index++;
    }


    // LOG_D("send count %d", data_index - 1);

    if (data_index - 1 == env->packet_count)
    {
        send_test_end_request(conn_idx);
    }

    free(raw_data);
}

static void ble_app_speed_send_thread(uint32_t param)
{
    rt_thread_t tid;
    tid = rt_thread_create("ble_send_example", ble_app_speed_write, (void *)(uint32_t)param, 1024, RT_THREAD_PRIORITY_LOW, 10);
    rt_thread_startup(tid);

}

static void send_test_start_handler(uint8_t conn_idx, uint8_t *data, uint16_t size)
{
    LOG_I("send_test_start_handler");
    app_env_t *env = ble_app_get_env();

    uint8_t type;
    memcpy(&type, data + 5, 1);

    uint32_t packet_count;
    memcpy(&packet_count, data + 6, 4);

    uint16_t packet_len;
    memcpy(&packet_len, data + 10, 2);

    uint8_t frequency;
    frequency = *(data + 12);

    env->test_type = type;
    env->packet_count = packet_count;
    env->packet_len = packet_len;
    env->response_frequency = frequency;

    env->current_index = 0;

    uint8_t *rsp_data = malloc(1);
    *rsp_data = BLE_SPEED_COMMAND_START_RSP;
    send_speed_test_packet_notify(rsp_data, 1, conn_idx);


    if (type == 0)
    {
        // start record time
        env->time_start = rt_tick_get();
        LOG_I("send_test_start_handler start time %d", env->time_start);
    }
    else if (type == 1)
    {
        uint32_t param;
        param = (packet_count << 16) | conn_idx;

        ble_app_speed_send_thread(param);
    }

    free(rsp_data);
}

static void send_test_end_rsp_process(uint8_t conn_idx, uint8_t *data)
{
    app_env_t *env = ble_app_get_env();

    uint8_t result = *data;

    if (result == 0)
    {
        uint32_t speed;
        memcpy(&speed, data + 1, 4);
        double show_speed = (double)speed / 100;
        LOG_I("test finish, speed: %.2lf kB/s", show_speed);
    }
    else
    {
        LOG_I("test failed with %d", result);
    }


    ble_gap_disconnect_t dis_conn;
    dis_conn.conn_idx = conn_idx;
    dis_conn.reason = CO_ERROR_REMOTE_USER_TERM_CON;
    ble_gap_disconnect(&dis_conn);
}

static void send_test_end_response(uint8_t conn_idx, uint8_t result, uint32_t speed)
{
    app_env_t *env = ble_app_get_env();

    uint8_t command_len = 6;

    uint8_t *data = malloc(command_len);
    *data = BLE_SPEED_COMMAND_END_RSP;

    *(data + 1) = result;

    memcpy(data + 2, &speed, 4);
    send_speed_test_packet_notify(data, command_len, conn_idx);

    free(data);
}

static void send_test_end_handler(uint8_t conn_idx, uint8_t *data, uint16_t size)
{
    app_env_t *env = ble_app_get_env();
    LOG_I("send_test_end_handler");
    env->time_end = rt_tick_get();
    LOG_I("send_test_end_handler start time %d end time %d", env->time_start, env->time_end);
    uint8_t result = 0;

    if (env->current_index != env->packet_count)
    {
        LOG_I("expect count %d, %d,", env->packet_count, env->current_index);
        result = 1;
    }

    double speed;

    LOG_I("send_test_end_handler len %d, count %d", env->packet_len + 4 + 5, env->packet_count);
    speed = (env->packet_len + 4 + 5) * env->packet_count / ((double)(env->time_end - env->time_start) / 1000) / 1000;
    LOG_I("BLE speed %.2lf kB/s", speed);



    send_test_end_response(conn_idx, result, speed * 100);
}

static void send_test_packet_response(uint8_t conn_idx)
{
    app_env_t *env = ble_app_get_env();
    uint8_t command_len = 1;

    uint8_t *data = malloc(command_len);
    *data = BLE_SPEED_COMMAND_PACKET_RSP;

    send_speed_test_packet_notify(data, command_len, conn_idx);

    free(data);
}

static void send_test_packet_handler(uint8_t conn_idx, uint8_t *data, uint16_t size)
{
    app_env_t *env = ble_app_get_env();

    uint16_t packet_len = size - 4 - 5;
    if (packet_len != env->packet_len)
    {
        LOG_I("expect len %d, %d,", env->packet_len, packet_len);
        return;
    }

    uint32_t packet_index;
    memcpy(&packet_index, data + 5, 4);

    if (packet_index != env->current_index + 1)
    {
        LOG_I("expect index %d, %d,", env->current_index + 1, packet_index);
        return;
    }

    env->current_index++;

    //LOG_D("send_test_packet_handler %d %d", env->current_index, env->response_frequency);

    if (env->response_frequency != 0)
    {
        if (env->current_index % env->response_frequency == 0)
        {
            send_test_packet_response(conn_idx);
        }
    }
}

// process write
static void ble_speed_test_handler(uint8_t conn_idx, uint8_t *data, uint16_t size)
{
    uint16_t index = 0;
    index += 2;
    uint16_t len;
    memcpy(&len, data + index, 2);
    index += 2;

    uint8_t command;
    memcpy(&command, data + index, 1);
    index++;

    switch (command)
    {
    case BLE_SPEED_COMMAND_START_REQ:
    {
        send_test_start_handler(conn_idx, data, size);
        break;
    }
    case BLE_SPEED_COMMAND_PACKET:
    {
        send_test_packet_handler(conn_idx, data, size);
        break;
    }
    case BLE_SPEED_COMMAND_END_REQ:
    {
        send_test_end_handler(conn_idx, data, size);
        break;
    }
    }
}



// process notify
static void ble_speed_test_rsp_handler(uint8_t conn_idx, uint8_t *data, uint16_t size)
{
    app_env_t *env = ble_app_get_env();
    uint16_t index = 0;
    index += 2;
    uint16_t len;
    memcpy(&len, data + index, 2);
    index += 2;

    uint8_t command;
    memcpy(&command, data + index, 1);
    index++;

    LOG_I("ble_speed_test_rsp_handler %d", command);

    switch (command)
    {
    case BLE_SPEED_COMMAND_START_RSP:
    {
        uint32_t param;
        param = (env->packet_count << 16) | conn_idx;

        ble_app_speed_send_thread(param);
        break;
    }
    case BLE_SPEED_COMMAND_END_RSP:
    {
        send_test_end_rsp_process(conn_idx, data + index);
        break;
    }
    case BLE_SPEED_COMMAND_PACKET_RSP:
    {
        uint32_t param;
        param = (env->packet_count << 16) | conn_idx;
        ble_app_speed_send_thread(param);
        break;
    }
    }
}

static void send_test_start_request(uint8_t conn_idx, uint8_t type, uint32_t packet_count, uint16_t packet_len, uint8_t freq)
{
    //LOG_I("send_test_start_request");
    app_env_t *env = ble_app_get_env();

    if (packet_len == 0xFF || packet_len > env->conn[conn_idx].mtu - 3 - 4 - 5)
    {
        packet_len = env->conn[conn_idx].mtu - 3 - 4 - 5;
    }

    LOG_I("send_test_start_request %d, %d", packet_count, packet_len);
    env->packet_count = packet_count;
    env->packet_len = packet_len;
    env->response_frequency = freq;
    env->current_index = 1;

    uint8_t command_len = 9;

    uint8_t *data = malloc(command_len);
    *data = BLE_SPEED_COMMAND_START_REQ;
    *(data + 1) = type;

    memcpy(data + 2, &packet_count, 4);
    memcpy(data + 6, &packet_len, 2);
    memcpy(data + 8, &freq, 1);

    send_speed_test_packet(data, command_len, conn_idx);

    free(data);
}

static void send_test_end_request(uint8_t conn_idx)
{
    LOG_I("send_test_end_request");
    app_env_t *env = ble_app_get_env();

    uint8_t *data = malloc(1);
    *data = BLE_SPEED_COMMAND_END_REQ;
    int8_t ret = send_speed_test_packet(data, 1, conn_idx);

    while (ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
    {
        rt_thread_mdelay(20);
        ret = send_speed_test_packet(data, 1, conn_idx);
    }

    free(data);
}

static int ble_app_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    app_env_t *env = ble_app_get_env();
    int8_t  res;

    //LOG_I("ancs gattc event handler %d\r\n", event_id);
    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        uint8_t conn_idx = rsp->conn_idx;
        LOG_I("register ret %d\r\n", rsp->status);
        if (rsp->status != HL_ERR_NO_ERROR)
        {
            env->conn[conn_idx].search_state = BLE_SPEED_TEST_STATE_IDLE;
            break;
        }

        env->conn[conn_idx].search_state = BLE_SPEED_TEST_STATE_SEARCH_COMPLETED;
        write_cccd(conn_idx, env->conn[conn_idx].app_char.cccd_hdl);

        env->conn[conn_idx].search_state = BLE_SPEED_TEST_STATE_READY;

        send_test_start_request(conn_idx, 0, env->packet_count, env->packet_len, env->response_frequency);
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;

        uint8_t conn_idx = ind->conn_idx;
        if (env->conn[conn_idx].search_state != BLE_SPEED_TEST_STATE_READY)
        {
            LOG_I("ancs state error %d", env->conn[conn_idx].search_state);
            return 0;
        }
        //LOG_I("ancs handle:%d", ind->handle);
        if (ind->handle == env->conn[conn_idx].app_char.value_hdl)
        {
            ble_speed_test_rsp_handler(conn_idx, ind->value, ind->length);
        }


        // Notify upper layer
        break;
    }
    default:
        break;
    }
    return 0;
}

static void update_conn(uint8_t conn_idx, uint8_t level)
{
    uint8_t update_level;
    if (level == 0)
    {
        update_level = CONNECTION_MANAGER_INTERVAL_HIGH_PERFORMANCE;
    }
    else
    {
        update_level = CONNECTION_MANAGER_INTERVAL_BALANCED;
    }

    connection_manager_update_parameter(conn_idx, update_level, NULL);
}

static void update_phy(uint8_t conn_idx)
{
    ble_gap_update_phy_t phy;
    phy.conn_idx = conn_idx;
    phy.rx_phy = GAP_PHY_LE_2MBPS;
    phy.tx_phy = GAP_PHY_LE_2MBPS;
    ble_gap_update_phy(&phy);
}

void start_speed_test_search(uint8_t conn_idx)
{
    app_env_t *env = ble_app_get_env();

    uint8_t app_svc[] = app_svc_uuid;
    sibles_search_service(conn_idx, ATT_UUID_128_LEN, (uint8_t *)app_svc);
    env->conn[conn_idx].search_state = BLE_SPEED_TEST_STATE_SEARCHING;
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
            ble_gap_disconnect_t dis_conn;
            dis_conn.conn_idx = ind->conn_idx;
            dis_conn.reason = CO_ERROR_REMOTE_USER_TERM_CON;
            ble_gap_disconnect(&dis_conn);
            break;
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

        if (env->is_test_on)
        {
            update_phy(ind->conn_idx);

            start_speed_test_search(ind->conn_idx);
            //send_test_start_request(ind->conn_idx, 0, env->packet_count, env->packek_len, env->response_frequency);
        }


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

        if (env->is_test_on)
        {
            update_conn(ind->conn_idx, env->conn_level);
        }

        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        uint8_t idx = ble_app_get_dev_by_idx(env, ind->conn_idx);
        if (idx == 0xFF)
            break;

        ble_app_deivce_disconnected(env, idx, ind->reason);
        sibles_unregister_remote_svc(ind->conn_idx, env->conn[ind->conn_idx].hdl_start, env->conn[ind->conn_idx].hdl_end, ble_app_gattc_event_handler);

        break;
    }
    case SIBLES_WRITE_VALUE_RSP:
    {
        sibles_write_value_rsp_t *rsp = (sibles_write_value_rsp_t *)data;
        LOG_I("SIBLES_WRITE_VALUE_RSP %d", rsp->result);
        break;
    }
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;
        uint8_t conn_idx = rsp->conn_idx;

        uint8_t app_svc[] = app_svc_uuid;
        uint8_t app_char[] = app_chara_uuid;
        //memcpy(ser, ser, ATT_UUID_128_LEN);

        // rsp->svc may null
        if (memcmp(rsp->search_uuid, app_svc, rsp->search_svc_len) != 0)
            break;

        if (rsp->result != HL_ERR_NO_ERROR)
        {
            env->conn[conn_idx].search_state = BLE_SPEED_TEST_STATE_IDLE;
            break; // Do nothingf
        }

        env->conn[conn_idx].hdl_start = rsp->svc->hdl_start;
        env->conn[conn_idx].hdl_end = rsp->svc->hdl_end;



        uint32_t i;
        uint8_t ready = 0; // This char is madatory

        uint16_t offset = 0;
        sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
        for (i = 0; i < rsp->svc->char_count; i++)
        {
            if (!memcmp(chara->uuid, app_char, chara->uuid_len))
            {
                LOG_I("noti_uuid received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                if (chara->desc_count != 1)
                    break;
                env->conn[conn_idx].app_char.attr_hdl = chara->attr_hdl;
                env->conn[conn_idx].app_char.value_hdl = chara->pointer_hdl;
                env->conn[conn_idx].app_char.prop = chara->prop;
                env->conn[conn_idx].app_char.cccd_hdl = chara->desc[0].attr_hdl;
                ready = 1;
            }
            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }
        if (ready != 1)
        {
            env->conn[conn_idx].search_state = BLE_SPEED_TEST_STATE_IDLE;
            break;
        }
        //register first
        env->conn[conn_idx].remote_handle = sibles_register_remote_svc(rsp->conn_idx, env->conn[conn_idx].hdl_start, env->conn[conn_idx].hdl_end, ble_app_gattc_event_handler);
        // subscribe data src. then subscribe notfi src.
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
                conn_param.conn_param_1m.ce_len_max = 100;
                conn_param.conn_param_1m.ce_len_min = 60;
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
            int8_t ret = sibles_read_remote_value(env->conn[idx].remote_handle, env->conn[idx].conn_idx, &value);
            if (ret != 0)
                LOG_E("Read remote value ");
        }
        else if (strcmp(argv[1], "bond") == 0)
        {
            uint8_t idx = (uint16_t)atoi(argv[2]);
            if (idx > BLE_APP_MAX_CONN_COUNT)
                LOG_E("Wrongly device index(%d)", idx);
            connection_manager_create_bond(env->conn[idx].conn_idx);
        }
        else if (strcmp(argv[1], "speed_search") == 0)
        {
            start_speed_test_search(0);
        }
        else if (strcmp(argv[1], "speed_packet") == 0)
        {
            uint32_t param;
            uint32_t count = 100;
            uint8_t index = 0;

            param = (count << 16) | index;
            ble_app_speed_send_thread(param);
        }
        else if (strcmp(argv[1], "speed_start") == 0)
        {
            uint16_t t_c = (uint16_t)atoi(argv[2]);
            uint16_t t_f = (uint16_t)atoi(argv[3]);


            send_test_start_request(0, 0, t_c, 0xFF, t_f);
        }
        else if (strcmp(argv[1], "update_con") == 0)
        {
            update_conn(0, 0);
        }
        else if (strcmp(argv[1], "update_phy") == 0)
        {
            update_phy(0);
        }
        else if (strcmp(argv[1], "speed_test") == 0)
        {
            env->is_test_on = 1;

            uint8_t addr[BD_ADDR_LEN];
            hex2data(argv[2], addr, BD_ADDR_LEN);

            env->local_mtu = (uint16_t)atoi(argv[3]);
            env->conn_level = (uint16_t)atoi(argv[4]);

            env->packet_count = (uint32_t)atoi(argv[5]);
            env->packet_len = (uint16_t)atoi(argv[6]);
            env->response_frequency = (uint8_t)atoi(argv[7]);

            ble_test_ctrl(addr);
        }
    }

    return 0;
}


FINSH_FUNCTION_EXPORT_ALIAS(cmd_diss, __cmd_diss, My device information service.);


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

