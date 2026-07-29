/*
 * SPDX-FileCopyrightText: 2021-2026 SiFli Technologies(Nanjing) Co., Ltd
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
#include "bf0_sibles_internal.h"
#include "bf0_sibles_advertising.h"
#include "ble_connection_manager.h"
#ifdef BSP_BLE_SERIAL_TRANSMISSION
    #include "bf0_sibles_serial_trans_service.h"
#endif

/* DFU V2 middleware */
#include "dfu_v2.h"
#include "dfu_app.h"
#include "mode/dfu_mode_host.h"
#include "engine/dfu_engine.h"
#define LOG_TAG "ble_app"
#include "log.h"

#ifdef BT_FINSH
    #include "bts2_app_interface.h"
#endif

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
    rt_timer_t write_time_handle;
    rt_mailbox_t mb_handle;
} app_env_t;

static app_env_t g_app_env;

static uint8_t g_app_svc[ATT_UUID_128_LEN] = app_svc_uuid;

BLE_GATT_SERVICE_DEFINE_128(app_att_db)
{
    BLE_GATT_SERVICE_DECLARE(BLE_APP_SVC, SERIAL_UUID_16_PRI_SERVICE, BLE_GATT_PERM_READ_ENABLE),
    BLE_GATT_CHAR_DECLARE(BLE_APP_CHAR, SERIAL_UUID_16_CHARACTERISTIC, BLE_GATT_PERM_READ_ENABLE),
    BLE_GATT_CHAR_VALUE_DECLARE(BLE_APP_CHAR_VALUE, app_chara_uuid,
            BLE_GATT_PERM_READ_ENABLE | BLE_GATT_PERM_WRITE_REQ_ENABLE |
            BLE_GATT_PERM_WRITE_COMMAND_ENABLE | BLE_GATT_PERM_NOTIFY_ENABLE,
            BLE_GATT_VALUE_PERM_UUID_128 | BLE_GATT_VALUE_PERM_RI_ENABLE,
            1024),
    BLE_GATT_DESCRIPTOR_DECLARE(BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR, SERIAL_UUID_16_CLIENT_CHAR_CFG,
            BLE_GATT_PERM_READ_ENABLE | BLE_GATT_PERM_WRITE_REQ_ENABLE, BLE_GATT_VALUE_PERM_RI_ENABLE, 2),
};

static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}

/*============================================================================
 * DFU V2 OTA event callback
 *============================================================================*/

static int on_dfu_event(const dfu_event_param_t *param, void *user_data)
{
    (void)user_data;
    switch (param->event)
    {
    case DFU_EVT_PROGRESS:
        LOG_D("OTA progress: %d%%", param->progress.percent);
        break;
    case DFU_EVT_IMG_COMPLETE:
        LOG_I("OTA image %d complete", param->img_complete.img_id);
        break;
    case DFU_EVT_ALL_COMPLETE:
        LOG_I("OTA all complete — firmware written to flash");
        /* Done. User can manually reboot to DFU mode if needed
         * via shell command: cmd_diss ota_reboot */
        break;
    case DFU_EVT_ERROR:
        LOG_E("OTA error: %d", param->err.error);
        break;
    case DFU_EVT_ABORTED:
        LOG_W("OTA aborted");
        break;
    default:
        break;
    }
    return 0;
}

/*============================================================================
 * Advertising (unchanged from original)
 *============================================================================*/

SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_bg_context);
static void ble_write_to_remote();

static uint8_t ble_app_background_advertising_event(uint8_t event, void *context, void *data)
{
    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("Broadcast ADV start result %d, mode %d", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("Broadcast ADV stopped reason %d, mode %d", evt->reason, evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}

static void ble_app_bg_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;
    char local_name[] = "SIFLI_BG_INFO";
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = SIG_SIFLI_COMPANY_ID;

    para.own_addr_type = GAPM_GEN_NON_RSLV_ADDR;
    para.config.adv_mode = SIBLES_ADV_BROADCAST_MODE;
    para.config.mode_config.broadcast_config.duration = 0x0;
    para.config.mode_config.broadcast_config.interval = 0x140;
    para.config.max_tx_pwr = 0x7F;
    para.config.is_auto_restart = 1;
    para.config.is_rsp_data_duplicate = 1;

    para.adv_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.adv_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.adv_data.completed_name->name, local_name, para.adv_data.completed_name->name_len);

    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_background_advertising_event;
    ret = sibles_advertising_init(g_app_advertising_bg_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
        sibles_advertising_start(g_app_advertising_bg_context);

    rt_free(para.adv_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
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
        if (!env->is_bg_adv_on)
        {
            env->is_bg_adv_on = 1;
            ble_app_bg_advertising_start();
        }
        LOG_I("ADV start result %d, mode %d", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("ADV stopped reason %d, mode %d", evt->reason, evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}

#define DEFAULT_LOCAL_NAME "SIFLI_APP"

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
        rt_snprintf(local_name, 31, "SIFLI_APP-%x-%x-%x-%x-%x-%x",
                    addr.addr[0], addr.addr[1], addr.addr[2],
                    addr.addr[3], addr.addr[4], addr.addr[5]);
    else
        memcpy(local_name, DEFAULT_LOCAL_NAME, sizeof(DEFAULT_LOCAL_NAME));

    ble_gap_dev_name_t *dev_name = malloc(sizeof(ble_gap_dev_name_t) + strlen(local_name));
    dev_name->len = strlen(local_name);
    memcpy(dev_name->name, local_name, dev_name->len);
    ble_gap_set_dev_name(dev_name);
    free(dev_name);

    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    para.config.mode_config.conn_config.duration = 0x0;
    para.config.mode_config.conn_config.interval = 0x30;
    para.config.max_tx_pwr = 0x7F;
    para.config.is_auto_restart = 1;

    para.rsp_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.rsp_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.rsp_data.completed_name->name, local_name, para.rsp_data.completed_name->name_len);

    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_advertising_event;
    ret = sibles_advertising_init(g_app_advertising_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
        sibles_advertising_start(g_app_advertising_context);

    rt_free(para.rsp_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}

/*============================================================================
 * GATT callbacks (unchanged from original)
 *============================================================================*/

uint8_t *ble_app_gatts_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    uint8_t *ret_val = NULL;
    app_env_t *env = ble_app_get_env();
    *len = 0;
    if (idx == BLE_APP_CHAR_VALUE)
    {
        ret_val = (uint8_t *)&env->data.data;
        *len = 4;
    }
    return ret_val;
}

static void ble_app_start_send_thread()
{
    rt_thread_t tid;
    tid = rt_thread_create("ble_send_example", ble_write_to_remote, NULL, 1024, RT_THREAD_PRIORITY_LOW, 10);
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
        LOG_HEX("BLE RX", 16, para->value, para->len);
        if (para->len <= 4)
            memcpy(&env->data.data, para->value, para->len);
        LOG_I("Updated app value from:%d to:%d", old_val, env->data.data);
        if (env->data.data == 0x1111)
        {
            LOG_I("example of ble notification");
            ble_app_start_send_thread();
        }
        break;
    }
    case BLE_APP_CLIENT_CHAR_CONFIG_DESCRIPTOR:
        env->data.is_config_on = *(para->value);
        LOG_I("CCCD %d", env->data.is_config_on);
        if (env->data.is_config_on)
            rt_timer_start(env->time_handle);
        else
            rt_timer_stop(env->time_handle);
        break;
    default:
        break;
    }
    return 0;
}

static void ble_write_to_remote()
{
    app_env_t *env = ble_app_get_env();
    int data_count = 20, data_length = 10, data_index = 0;
    uint8_t raw_data[20][10] = {0};
    for (int i = 0; i < data_count; i++)
        memset(raw_data[i], i, data_length);

    while (data_count > 0)
    {
        sibles_value_t value;
        value.hdl = env->data.srv_handle;
        value.idx = BLE_APP_CHAR_VALUE;
        value.len = data_length;
        value.value = raw_data[data_index];
        int ret = sibles_write_value(env->conn_idx, &value);
        if (ret == data_length) { data_count--; data_index++; }
        else if (ret == 0)
        {
            for (int retry = 20; retry > 0; retry--)
            {
                rt_thread_mdelay(50);
                ret = sibles_write_value(env->conn_idx, &value);
                if (ret == data_length) { data_count--; data_index++; break; }
            }
        }
    }
}

static void ble_app_service_init(void)
{
    app_env_t *env = ble_app_get_env();
    BLE_GATT_SERVICE_INIT_128(svc, app_att_db, BLE_APP_ATT_NB,
                              BLE_GATT_SERVICE_PERM_NOAUTH | BLE_GATT_SERVICE_PERM_UUID_128 | BLE_GATT_SERVICE_PERM_MULTI_LINK,
                              g_app_svc);
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
    if (ret != 0) { LOG_W("generate mac address failed %d", ret); return BLE_UPDATE_NO_UPDATE; }
    return BLE_UPDATE_ONCE;
}
#endif

/*============================================================================
 * Main — V2 middleware + BLE share same mailbox
 *============================================================================*/

int main(void)
{
    app_env_t *env = ble_app_get_env();
    LOG_I("=== peripheral_with_ota V2 Start ===");

    /* 1. Init DFU V2 middleware */
    dfu_config_t cfg = { .callback = on_dfu_event, .user_data = RT_NULL };
    if (dfu_init(&cfg) != 0)
        LOG_E("dfu_init failed - OTA disabled but BLE app continues");
    else
    {
        LOG_I("DFU V2 initialized (BLE Transport + Host Mode)");
        /*
         * User APP runs from HCPU partition (img_id=0).
         * Tell Host Mode to skip HCPU images — we can't overwrite ourselves.
         * Non-self images (DFU=6, RES=3, FONT=4, etc.) will be written.
         */
        dfu_mode_host_set_self_img_id(0);   /* 0 = HCPU */
    }

    /* Clear any stale install flags from interrupted sessions */
    dfu_check_install();
    /* 2. Share V2 mailbox for BLE events + DFU messages */
    env->mb_handle = dfu_get_mailbox();
    if (!env->mb_handle)
        env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);

    /* 3. BLE stack */
    sifli_ble_enable();
    env->time_handle = rt_timer_create("app", app_timeout_handler, NULL,
                                       rt_tick_from_millisecond(BLE_APP_TIMEOUT_INTERVAL),
                                       RT_TIMER_FLAG_SOFT_TIMER);
#if defined(BT_FINSH) && defined(SF32LB52X_58)
    bt_interface_acl_accept_role_set(0);
    bt_interface_set_linkpolicy(1, 1);
#endif

    /* 4. Main loop: V2 messages first, then BLE events */
    while (1)
    {
        uint32_t value;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);

        if (dfu_process(value) == 0)
        {
            /* Drain all queued BLE messages in one go.
             *
             * The BLE callback enqueues messages into a ring queue and posts
             * DFU_MSG_BLE_DATA to the mailbox for each one. But when APP
             * sends packets faster than the main loop processes them (batch
             * mode with num_of_rsp=10), the mailbox may overflow and some
             * notifications are lost — even though the data is safely in
             * the ring queue.
             *
             * By draining the ring queue here until empty, we ensure every
             * queued message is processed regardless of mailbox capacity.
             * This eliminates the packet index gap problem caused by
             * unprocessed messages sitting in the queue. */
            if (value == DFU_MSG_BLE_DATA)
            {
                while (dfu_process(DFU_MSG_BLE_DATA) == 0)
                    ;
            }
            continue;
        }

        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;
            env->conn_para.mtu = 23;
            ble_app_service_init();
#ifdef BSP_BLE_SERIAL_TRANSMISSION
            ble_serial_tran_init();
#endif
            ble_app_advertising_start();
            LOG_I("BLE power on, advertising started");
        }
    }
    return RT_EOK;
}

/*============================================================================
 * BLE event handler
 *============================================================================*/

static void ble_app_update_conn_param(uint8_t conn_idx, uint16_t inv_max,
                                      uint16_t inv_min, uint16_t timeout)
{
    ble_gap_update_conn_param(BLE_GAP_CREATE_UPDATE_CONN_PARA(conn_idx, inv_min, inv_max, 0, timeout));
}

int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    app_env_t *env = ble_app_get_env();
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
        if (env->mb_handle) rt_mb_send(env->mb_handle, BLE_POWER_ON_IND);
        break;
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        env->conn_para.conn_interval = ind->con_interval;
        env->conn_para.peer_addr_type = ind->peer_addr_type;
        env->conn_para.peer_addr = ind->peer_addr;
        LOG_I("Connected(%x-%x-%x-%x-%x-%x)", ind->peer_addr.addr[5],
              ind->peer_addr.addr[4], ind->peer_addr.addr[3],
              ind->peer_addr.addr[2], ind->peer_addr.addr[1], ind->peer_addr.addr[0]);
        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        env->conn_para.conn_interval = ind->con_interval;
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        env->conn_para.mtu = ind->mtu;
        LOG_I("MTU: %d", ind->mtu);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        LOG_I("Disconnected(%d)", ind->reason);
        break;
    }
    case SIBLES_WRITE_VALUE_RSP:
    {
        sibles_write_value_rsp_t *rsp = (sibles_write_value_rsp_t *)data;
        LOG_I("WRITE_RSP %d", rsp->result);
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);

uint8_t ble_app_dis_enable() { return 1; }

/*============================================================================
 * Finsh commands
 *============================================================================*/

int cmd_diss(int argc, char *argv[])
{
    app_env_t *env = ble_app_get_env();
    if (argc > 1)
    {
        if (strcmp(argv[1], "conn") == 0 && argc > 2)
        {
            uint32_t interval = atoi(argv[2]) * 100 / 125;
            ble_app_update_conn_param(env->conn_idx, interval, interval, 500);
        }
        else if (strcmp(argv[1], "adv_start") == 0)
            sibles_advertising_start(g_app_advertising_context);
        else if (strcmp(argv[1], "adv_stop") == 0)
            sibles_advertising_stop(g_app_advertising_context);
        else if (strcmp(argv[1], "bg_adv_del") == 0)
        {
            if (env->is_bg_adv_on)
            {
                env->is_bg_adv_on = 0;
                sibles_advertising_stop(g_app_advertising_bg_context);
                sibles_advertising_delete(g_app_advertising_bg_context);
            }
        }
        else if (strcmp(argv[1], "ota_reboot") == 0)
        {
            LOG_I("Entering DFU-BLE V2 mode...");
            dfu_enter_dfu_mode();  /* writes flag + reboots, does not return */
        }
        else if (strcmp(argv[1], "ota_info") == 0)
        {
            uint32_t received = 0, total = 0;
            if (dfu_get_progress(&received, &total) == 0 && total > 0)
                rt_kprintf("OTA: %d%% (%u/%u)\n", (int)(received * 100 / total), received, total);
            else
                rt_kprintf("OTA: idle\n");
        }
    }
    return 0;
}
#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(cmd_diss, BLE peripheral V2: conn|adv_start|adv_stop|ota_reboot|ota_info);
#endif

#ifdef SF32LB52X_58
uint16_t g_em_offset[HAL_LCPU_CONFIG_EM_BUF_MAX_NUM] =
{
    0x178, 0x178, 0x740, 0x7A0, 0x810, 0x880, 0xA00, 0xBB0, 0xD48,
    0x133C, 0x13A4, 0x19BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC,
    0x21BC, 0x21BC, 0x263C, 0x265C, 0x2734, 0x2784, 0x28D4, 0x28E8, 0x28FC,
    0x29EC, 0x29FC, 0x2BBC, 0x2BD8, 0x3BE8, 0x5804, 0x5804, 0x5804
};
void lcpu_rom_config(void)
{
    hal_lcpu_bluetooth_em_config_t em_offset;
    memcpy((void *)em_offset.em_buf, (void *)g_em_offset, HAL_LCPU_CONFIG_EM_BUF_MAX_NUM * 2);
    em_offset.is_valid = 1;
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_EM_BUF, &em_offset, sizeof(hal_lcpu_bluetooth_em_config_t));
    hal_lcpu_bluetooth_act_configt_t act_cfg;
    act_cfg.ble_max_act = 6; act_cfg.ble_max_iso = 0; act_cfg.ble_max_ral = 3;
    act_cfg.bt_max_acl = 7; act_cfg.bt_max_sco = 0;
    act_cfg.bit_valid = CO_BIT(0) | CO_BIT(1) | CO_BIT(2) | CO_BIT(3) | CO_BIT(4);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_ACT_CFG, &act_cfg, sizeof(hal_lcpu_bluetooth_act_configt_t));
}
#endif