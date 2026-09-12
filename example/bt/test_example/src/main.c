/*
 * SPDX-FileCopyrightText: 2021-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_advertising.h"

#include "bts2_app_inc.h"

#ifdef AUDIO_USING_MANAGER
    #include "audio_server.h"
#endif // AUDIO_USING_MANAGER

#define LOG_TAG "ble_app"

#include "bts2_app_inc.h"
#include "bt_connection_manager.h"
#include "log.h"

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

typedef struct
{
    uint8_t is_power_on;
    uint8_t conn_idx;
    uint8_t is_bg_adv_on;
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
#ifdef SF32LB52X
    rt_timer_t rc10k_time_handle;
#endif
    rt_mailbox_t mb_handle;
} app_env_t;

static app_env_t g_app_env;
static rt_mailbox_t g_app_mb;

static uint8_t g_app_svc[ATT_UUID_128_LEN] = app_svc_uuid;

struct attm_desc_128 app_att_db[] =
{
    [BLE_APP_SVC] = {SERIAL_UUID_16(ATT_DECL_PRIMARY_SERVICE), PERM(RD, ENABLE), 0, 0},
    [BLE_APP_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC), PERM(RD, ENABLE), 0, 0},
    [BLE_APP_CHAR_VALUE] = {
        /* The permissions are for: 1.Allowed read, write req, write command and notification.
                                    2.Write requires Unauthenticated link
           The ext_perm are for: 1. Support 128bit UUID. 2. Read will involve callback. */
        app_chara_uuid, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WRITE_COMMAND, ENABLE) | PERM(NTF, ENABLE) |
        PERM(WP, UNAUTH), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024
    },
    [BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR] = {
        SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ,
                ENABLE) | PERM(WP, UNAUTH), PERM(RI, ENABLE), 2
    },

};

static void ble_write_to_remote(void *param);

static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}

#if 0 //CFG_AV_AAC
#include "mem_section.h"

#define AAC_HEAP_SIZE  (1 * 1024 * 1024)

#if defined(__CC_ARM) || defined(__CLANG_ARM)
    L2_RET_BSS_SECT_BEGIN(g_xz_opus_stack)
    static  uint32_t g_aac_heap[AAC_HEAP_SIZE / sizeof(uint32_t)];
    L2_RET_BSS_SECT_END
#else
    static uint32_t g_aac_heap[AAC_HEAP_SIZE / sizeof(uint32_t)] L2_RET_BSS_SECT(g_aac_heap);
#endif

static struct rt_memheap aac_memheap;
static uint32_t g_aac_mem_size;

static int aac_heap_init(void)
{
    rt_memheap_init(&aac_memheap, "aac_memheap", (void *)g_aac_heap, AAC_HEAP_SIZE);
    return 0;
}

INIT_BOARD_EXPORT(aac_heap_init);

void *faad_malloc(size_t size)
{
    void *p = rt_memheap_alloc(&aac_memheap, size);
    RT_ASSERT(p);
    return p;
}

void faad_free(void *b)
{
    rt_memheap_free(b);
}

#endif

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

uint8_t *ble_app_gatts_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    uint8_t *ret_val = NULL;
    app_env_t *env = ble_app_get_env();
    *len = 0;
    switch (idx)
    {
    case BLE_APP_CHAR_VALUE:
    {
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
uint8_t ble_app_gatts_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    app_env_t *env = ble_app_get_env();
    switch (para->idx)
    {
    case BLE_APP_CHAR_VALUE:
    {
        uint32_t old_val = env->data.data;
        if (para->len <= 4)
            memcpy(&env->data.data, para->value, para->len);
        LOG_I("Updated app value from:%d to:%d", old_val, env->data.data);
        if ((env->data.data & 0xFF) == 0x11)
        {
            LOG_I("example of ble notification %d", env->data.data);
            ble_app_start_send_thread(env->data.data);
        }
        break;
    }
    case BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR:
    {
        env->data.is_config_on = *(para->value);
        LOG_I("CCCD %d", env->data.is_config_on);
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

static void ble_app_service_init(void)
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

#ifdef SF32LB52X
void rc10k_timeout_handler(void *parameter)
{
    app_env_t *env = ble_app_get_env();
    if (!HAL_RTC_LXT_ENABLED())
    {
        HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
    }
    else
    {
        rt_timer_stop(env->rc10k_time_handle);
    }
}
#endif
static void app_wakeup(void)
{
#if !defined(SOC_SF32LB55X)
    uint8_t pin = 0;
#if defined(SOC_SF32LB56X)
    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN6;
#elif defined(SOC_SF32LB58X)
    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN6;
#elif defined(SOC_SF32LB52X)
    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN0; //PA24
#else
    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN12;
#endif
    HAL_StatusTypeDef status = HAL_HPAON_EnableWakeupSrc(src, AON_PIN_MODE_LOW);
#endif
}

static int bt_app_interface_event_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    if (type == BT_NOTIFY_AVRCP)
    {
        switch (event_id)
        {
        case BT_NOTIFY_AVRCP_PROFILE_CONNECTED:
        {
            LOG_I("AVRCP connected success");
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
            bt_interface_set_avrcp_role_ext(&profile_info->mac, AVRCP_CT);
        }
        break;
        case BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED:
        {
            LOG_I("AVRCP disconnected");
        }
        break;
        case BT_NOTIFY_AVRCP_ABSOLUTE_VOLUME:
        {
            uint8_t *volume = (uint8_t *)data;
#ifdef AUDIO_USING_MANAGER
            uint8_t local_vol = bt_interface_avrcp_abs_vol_2_local_vol(*volume, audio_server_get_max_volume());
            audio_server_set_private_volume(AUDIO_TYPE_BT_MUSIC, local_vol);
#endif
        }
        break;
        default:
            break;
        }
    }

    return 0;
}

ble_common_update_type_t ble_request_public_address(bd_addr_t *addr)
{
    ble_common_update_type_t res = BLE_UPDATE_NO_UPDATE;
    int ret = bt_mac_addr_generate_via_uid_v2(addr);
    if (ret == 0)
        res = BLE_UPDATE_ONCE;

    return res;

}

int main(void)
{
    int count = 0;

    app_wakeup();
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();
    env->time_handle  = rt_timer_create("app", app_timeout_handler,  NULL,
                                        rt_tick_from_millisecond(BLE_APP_TIMEOUT_INTERVAL), RT_TIMER_FLAG_SOFT_TIMER);

#ifdef SF32LB52X
    env->rc10k_time_handle  = rt_timer_create("rc10", rc10k_timeout_handler,  NULL,
                              rt_tick_from_millisecond(15 * 1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 15s
    rt_timer_start(env->rc10k_time_handle);
#endif

    bt_interface_register_bt_event_notify_callback(bt_app_interface_event_handle);

    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;
            env->conn_para.mtu = 23; /* Default value. */
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

static void ble_app_update_conn_param_l2cap(uint8_t conn_idx, uint16_t inv_max, uint16_t inv_min, uint16_t timeout)
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
    ble_gap_update_conn_param_on_l2cap(&conn_para);
}

#if 0
void HAL_DBG_printf(const char *fmt, ...)
{
    va_list args;
    static char rt_log_buf[128];
    extern void rt_kputs(const char *str);

    va_start(args, fmt);
    rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    rt_kputs(rt_log_buf);
    rt_kputs("\r\n");
    va_end(args);
}
#endif

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

int cmd_diss(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "trc") == 0)
        {
            if (strcmp(argv[2], "mode") == 0)
            {
#ifndef SOC_SF32LB55X
                uint8_t mode = atoi(argv[3]);
                uint32_t mask = atoi(argv[4]);
                sibles_set_trc_cfg(mode, mask);
#endif
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
        else if (0 == strcmp(argv[1], "l2c_con"))
        {
            uint32_t interval;
            app_env_t *env = ble_app_get_env();

            interval = atoi(argv[2]);
            // value = argv * 1.25
            interval = interval * 100 / 125;
            ble_app_update_conn_param_l2cap(env->conn_idx, interval, interval, 500);
        }
        else if (strcmp(argv[1], "adv_start") == 0)
        {
            ble_app_advertising_start();
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

FINSH_FUNCTION_EXPORT_ALIAS(cmd_diss, __cmd_diss, My device information service.);

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

#if defined(LCPU_RUN_ROM_ONLY) || defined(FPGA)
void lcpu_rom_config(void)
{

    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();

#ifdef LXT_DISABLE
    uint8_t is_enable_lxt = 0;
    uint8_t is_lcpu_rccal = 1;
#else // !LXT_DISABLE
    uint8_t is_enable_lxt = 1;
    uint8_t is_lcpu_rccal = 0;
#endif // LXT_DISABLE
    uint32_t wdt_staus = 0xFF;
    uint32_t wdt_time = 10;
    uint16_t wdt_clk = 32768;

    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_XTAL_ENABLED, &is_enable_lxt, 1);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_STATUS, &wdt_staus, 4);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_TIME, &wdt_time, 4);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_CLK_FEQ, &wdt_clk, 2);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_RC_CAL_IN_L, &is_lcpu_rccal, 1);

    {
        uint32_t tx_queue = HCPU2LCPU_MB_CH1_BUF_START_ADDR;
        hal_lcpu_bluetooth_rom_config_t config = {0};
        config.bit_valid |= 1 << 10 | 1 << 6;
        config.is_fpga = 1;
        config.default_xtal_enabled = is_enable_lxt;
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_HCPU_TX_QUEUE, &tx_queue, 4);
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_CONFIG, &config, sizeof(config));
    }

    {
        hal_lcpu_bluetooth_actmove_config_t act_cfg;
        act_cfg.bit_valid = 1;
        act_cfg.act_mov = 0x10;
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_ACTMOVE_CONFIG, &act_cfg, LCPU_CONFIG_BT_ACTMOVE_ROM_LENGTH);
    }
}
#endif // defined(LCPU_RUN_ROM_ONLY) || defined(FPGA)

#ifdef LCPU_RUN_ROM_ONLY
void HAL_LPAON_ConfigStartAddr(uint32_t *start_addr)
{
    hwp_lpsys_aon->SPR = 0;
    hwp_lpsys_aon->PCR = 0;
}
#endif // LCPU_RUN_ROM_ONLY

