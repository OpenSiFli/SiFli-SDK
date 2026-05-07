/*
 * SPDX-FileCopyrightText: 2024-2024 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_advertising.h"
#include "ble_connection_manager.h"


#define LOG_TAG "ble_app"
#include "log.h"

#define IBEACON_UUID {0x95, 0xd0, 0xb4, 0x22, 0xa4, 0xbd, 0x45, 0xd4, 0x99, 0x20, 0x57, 0x6b, 0xc6, 0x63, 0x23, 0x72}

typedef struct
{
    uint8_t length;
    uint8_t type;
    uint16_t company_id;
    uint16_t beacon_type;
} sifli_ibeacon_head_t;

typedef struct
{
    uint8_t proximity_uuid[ATT_UUID_128_LEN];
    uint16_t major;
    uint16_t minor;
    int8_t measured_power;
} sifli_ibeacon_supplier_t;

typedef struct
{
    sifli_ibeacon_head_t ibeacon_head;
    sifli_ibeacon_supplier_t ibeacon_supplier;
} sifli_ibeacon_t;


typedef struct
{
    uint8_t is_power_on;
    // Mbox thread
    rt_mailbox_t mb_handle;
    // work queue
    struct rt_delayed_work work;

    sifli_ibeacon_t ibeacon_data;
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
    app_env_t *env = ble_app_get_env();

    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("ADV start result %d, mode %d\r\n", evt->status, evt->adv_mode);
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

static void ideacon2data(sifli_ibeacon_t *ibeacon_data, uint8_t *data)
{
    data[0] = ibeacon_data->ibeacon_head.length;
    data[1] = ibeacon_data->ibeacon_head.type;
    /*little-endian*/
    data[2] = (uint8_t)(ibeacon_data->ibeacon_head.company_id & 0x00FF);
    data[3] = (uint8_t)((ibeacon_data->ibeacon_head.company_id & 0xFF00) >> 8);
    /*little-endian*/
    data[4] = (uint8_t)(ibeacon_data->ibeacon_head.beacon_type & 0x00FF);
    data[5] = (uint8_t)((ibeacon_data->ibeacon_head.beacon_type & 0xFF00) >> 8);

    rt_memcpy(&data[6], ibeacon_data->ibeacon_supplier.proximity_uuid, ATT_UUID_128_LEN);
    data[22] = (uint8_t)((ibeacon_data->ibeacon_supplier.major & 0xFF00) >> 8);
    data[23] = (uint8_t)(ibeacon_data->ibeacon_supplier.major & 0xFF);

    data[24] = (uint8_t)((ibeacon_data->ibeacon_supplier.minor & 0xFF00) >> 8);
    data[25] = (uint8_t)(ibeacon_data->ibeacon_supplier.minor & 0xFF);
    data[26] = ibeacon_data->ibeacon_supplier.measured_power;
}

static int uuid_string_to_bytes(const char *uuid_str, uint8_t *uuid_bytes)
{
    //Format: "12345678-1234-1234-1234-123456789abc"
    int result = sscanf(
                     uuid_str,
                     "%2hhx%2hhx%2hhx%2hhx-"
                     "%2hhx%2hhx-"
                     "%2hhx%2hhx-"
                     "%2hhx%2hhx-"
                     "%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx",
                     &uuid_bytes[0], &uuid_bytes[1], &uuid_bytes[2], &uuid_bytes[3],
                     &uuid_bytes[4], &uuid_bytes[5], &uuid_bytes[6], &uuid_bytes[7],
                     &uuid_bytes[8], &uuid_bytes[9], &uuid_bytes[10], &uuid_bytes[11],
                     &uuid_bytes[12], &uuid_bytes[13], &uuid_bytes[14], &uuid_bytes[15]);
    LOG_HEX("in uuid_string_to_bytes", 16, uuid_bytes, 16);
    return result == 16;
}


#define DEFAULT_LOCAL_NAME "SIFLI_APP1"
#define EXAMPLE_LOCAL_NAME "SIFLI_EXAMPLE"

/* Enable advertise via advertising service. */
static void ble_app_ibeacon_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;
    app_env_t *env = ble_app_get_env();

    bd_addr_t addr;
    ret = ble_get_public_address(&addr);
    if (ret == HL_ERR_NO_ERROR)
    {
        LOG_I("ble_get_public_address :%02x-%02x-%02x-%02x-%02x-%02x", addr.addr[5], addr.addr[4], addr.addr[3], addr.addr[2], addr.addr[1], addr.addr[0]);
    }
    else
    {
        LOG_I("ble_get_public_address ERROR");
    }


    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.adv_mode = SIBLES_ADV_BROADCAST_MODE;


    para.config.mode_config.broadcast_config.duration = 0;
    para.config.mode_config.broadcast_config.interval = 0x30;
    para.config.mode_config.broadcast_config.scannable_enable = 1;


    para.config.max_tx_pwr = 0x7F;

    uint8_t uuid[ATT_UUID_128_LEN] = IBEACON_UUID;

    rt_memcpy(env->ibeacon_data.ibeacon_supplier.proximity_uuid, uuid, ATT_UUID_128_LEN);
    env->ibeacon_data.ibeacon_supplier.major = 0x0100;
    env->ibeacon_data.ibeacon_supplier.minor = 0x0102;
    env->ibeacon_data.ibeacon_supplier.measured_power = 0xCE;


    para.adv_data.customized_data = rt_malloc(28);

    ideacon2data(&(env->ibeacon_data), para.adv_data.customized_data->data);

    para.adv_data.customized_data->len = 27;



    para.evt_handler = ble_app_advertising_event;


    ret = sibles_advertising_init(g_app_advertising_context, &para);
    LOG_I("sibles_advertising_init called");
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_context);
        LOG_I("sibles_advertising_start called");
    }


    rt_free(para.adv_data.customized_data);

}

static void update_adv_content()
{
    sibles_advertising_para_t para = {0};
    app_env_t *env = ble_app_get_env();


    para.adv_data.customized_data = rt_malloc(28);

    ideacon2data(&(env->ibeacon_data), para.adv_data.customized_data->data);

    para.adv_data.customized_data->len = 27;



    uint8_t ret = sibles_advertising_update_adv_and_scan_rsp_data(g_app_advertising_context, &para.adv_data, NULL);
    LOG_I("update adv %d", ret);

    rt_free(para.adv_data.customized_data);

}

int main(void)
{
    int count = 0;
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();


    /*iBeacon common header*/
    env->ibeacon_data.ibeacon_head.length = 0x1A;
    env->ibeacon_data.ibeacon_head.type = 0xFF;
    env->ibeacon_data.ibeacon_head.company_id = 0x004C;
    env->ibeacon_data.ibeacon_head.beacon_type = 0x1502;

    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;

            /* First enable connectable adv then enable non-connectable. */
            ble_app_ibeacon_advertising_start();
            LOG_I("receive BLE power on!\r\n");
        }
    }
    return RT_EOK;
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
    case BLE_GAP_SET_ADV_DATA_CNF:
    {
        ble_gap_set_adv_data_cnf_t *cnf = (ble_gap_set_adv_data_cnf_t *)data;
        LOG_I("Set ADV_DATA result %d", cnf->status);
        break;
    }
    case BLE_GAP_START_ADV_CNF:
    {
        ble_gap_start_adv_cnf_t *cnf = (ble_gap_start_adv_cnf_t *)data;
        LOG_I("Start ADV result %d", cnf->status);
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
    app_env_t *env = ble_app_get_env();

    if (argc > 1)
    {
        if (strcmp(argv[1], "adv_start") == 0)
        {
            sibles_advertising_start(g_app_advertising_context);
        }
        else if (strcmp(argv[1], "adv_stop") == 0)
        {
            sibles_advertising_stop(g_app_advertising_context);
        }
        else if (strcmp(argv[1], "adv_update") == 0)
        {
            // dynamic update adv content in auxiliary adv
            uuid_string_to_bytes(argv[2], env->ibeacon_data.ibeacon_supplier.proximity_uuid);

            LOG_HEX("env->ibeacon_data.ibeacon_supplier.proximity_uuid", 16, env->ibeacon_data.ibeacon_supplier.proximity_uuid, ATT_UUID_128_LEN);

            env->ibeacon_data.ibeacon_supplier.major = atoi(argv[3]);
            env->ibeacon_data.ibeacon_supplier.minor = atoi(argv[4]);
            env->ibeacon_data.ibeacon_supplier.measured_power = (int8_t)atoi(argv[5]);

            LOG_I("major: %d, minor: %d, measured_power: %d", env->ibeacon_data.ibeacon_supplier.major, env->ibeacon_data.ibeacon_supplier.minor, env->ibeacon_data.ibeacon_supplier.measured_power);

            update_adv_content();
        }
    }

    return 0;
}

#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(cmd_diss, My device information service.);
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
    act_cfg.ble_max_act = 6;
    act_cfg.ble_max_iso = 0;
    act_cfg.ble_max_ral = 3;
    act_cfg.bt_max_acl = 7;
    act_cfg.bt_max_sco = 0;
    act_cfg.bit_valid = CO_BIT(0) | CO_BIT(1) | CO_BIT(2) | CO_BIT(3) | CO_BIT(4);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_ACT_CFG, &act_cfg, sizeof(hal_lcpu_bluetooth_act_configt_t));
}
#endif

