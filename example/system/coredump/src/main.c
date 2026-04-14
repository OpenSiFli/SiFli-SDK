/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_advertising.h"

#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "drv_flash.h"
#include "drv_common.h"
#include "dfs_fs.h"
#include <stdlib.h>

#define LOG_TAG "app_coredump"
#include "log.h"

#define FS_ROOT "root"

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
    rt_mailbox_t mb_handle;
} app_env_t;

static app_env_t g_app_env;
SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);

static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}

static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data)
{
    (void)context;

    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
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

#define DEFAULT_LOCAL_NAME "SIFLI_COREDUMP"

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
        rt_snprintf(local_name, 31, "COREDUMP-%x-%x-%x-%x-%x-%x", addr.addr[0], addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4], addr.addr[5]);
    else
        memcpy(local_name, DEFAULT_LOCAL_NAME, sizeof(DEFAULT_LOCAL_NAME));

    ble_gap_dev_name_t *dev_name = rt_malloc(sizeof(ble_gap_dev_name_t) + strlen(local_name));
    dev_name->len = strlen(local_name);
    memcpy(dev_name->name, local_name, dev_name->len);
    ble_gap_set_dev_name(dev_name);
    rt_free(dev_name);

    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    para.config.mode_config.conn_config.duration = 0x0;
    para.config.mode_config.conn_config.interval = 0x140;
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
    {
        rt_kprintf("Advertising initialized successfully\n");
        sibles_advertising_start(g_app_advertising_context);
    }
    else
    {
        rt_kprintf("Advertising init failed: %d\n", ret);
    }

    rt_free(para.rsp_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}

int mnt_init(void)
{
    // rt_kprintf("FS_REGION_START_ADDR = %p\n", FS_REGION_START_ADDR);
    // rt_kprintf("FS_REGION_SIZE = %p\n", FS_REGION_SIZE);
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, FS_ROOT);

    if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);


/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    app_env_t *env = ble_app_get_env();

    rt_kprintf("Coredump BLE example start...\n");
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);

    rt_kprintf("Enabling BLE...\n");
    sifli_ble_enable();

    while (1)
    {
        uint32_t value;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            rt_kprintf("BLE power on event received\n");
            env->is_power_on = 1;
            env->conn_para.mtu = 23;
            rt_kprintf("Starting BLE advertising...\n");
            extern void ble_serial_tran_init(void);
            ble_serial_tran_init();
            ble_app_advertising_start();
            LOG_I("Coredump BLE ready for connection");
        }
    }

    return 0;
}

int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    app_env_t *env = ble_app_get_env();

    (void)len;
    (void)context;

    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
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

        LOG_I("Peer(%x-%x-%x-%x-%x-%x) connected", env->conn_para.peer_addr.addr[5],
              env->conn_para.peer_addr.addr[4],
              env->conn_para.peer_addr.addr[3],
              env->conn_para.peer_addr.addr[2],
              env->conn_para.peer_addr.addr[1],
              env->conn_para.peer_addr.addr[0]);
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
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

/* Finsh command: read memory bytes for quick inspection before triggering crash */
static int mem_read_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: memr <addr> [len]\n");
        return -RT_ERROR;
    }

    rt_uint32_t addr = (rt_uint32_t)strtoul(argv[1], RT_NULL, 0);
    rt_size_t len = 16;
    if (argc >= 3)
    {
        len = (rt_size_t)strtoul(argv[2], RT_NULL, 0);
    }

    rt_uint8_t *p = (rt_uint8_t *)addr;
    rt_size_t i;
    for (i = 0; i < len; i++)
    {
        if ((i % 16) == 0)
        {
            rt_kprintf("0x%08x: ", (unsigned int)(addr + i));
        }
        rt_kprintf("%02X ", p[i]);
        if ((i % 16) == 15 || i == len - 1)
        {
            rt_kprintf("\n");
        }
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(mem_read_cmd, memr <addr> [len]);
