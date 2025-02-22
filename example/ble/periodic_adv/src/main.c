/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
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

#define LOG_TAG "ble_app"
#include "log.h"


#define APP_MAX_PER_ADV_LEN (100)

typedef struct
{
    uint8_t is_power_on;
    // Periodic configure para
    uint8_t per_adv_change_inv;
    uint8_t per_adv_len;
    // Mbox thread
    rt_mailbox_t mb_handle;
    // work queue
    struct rt_delayed_work work;
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
#if 0
        if (!env->is_bg_adv_on)
        {
            env->is_bg_adv_on = 1;
            ble_app_bg_advertising_start();
        }
#endif
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
uint8_t g_peri_adv[APP_MAX_PER_ADV_LEN];

/* Enable advertise via advertising service. */
static void ble_app_peri_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;
    app_env_t *env = ble_app_get_env();

    char local_name[31] = {0};
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
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
    para.config.adv_mode = SIBLES_ADV_PERIODIC_MODE;

    para.config.mode_config.periodic_config.duration = 0;
    para.config.mode_config.periodic_config.interval = 0xA0;

    para.config.mode_config.periodic_config.max_skip = 0;
    para.config.mode_config.periodic_config.phy = GAP_PHY_TYPE_LE_1M;
    para.config.mode_config.periodic_config.adv_sid = 0;
    para.config.mode_config.periodic_config.connectable_enable = 0;

    para.config.mode_config.periodic_config.adv_intv_min = 16;
    para.config.mode_config.periodic_config.adv_intv_max = 16;


    para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    // in multi-connection
    para.config.is_auto_restart = 1;
    /* Scan rsp data is same as advertising data. */
    //para.config.is_rsp_data_duplicate = 1;

    /* Prepare name filed. Due to name is too long to put adv data, put it to rsp data.*/
    para.adv_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.adv_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.adv_data.completed_name->name, local_name, para.adv_data.completed_name->name_len);

    /* Prepare manufacturer filed .*/
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_advertising_event;

    uint8_t per_len = env->per_adv_len;
    para.periodic_data = rt_malloc(sizeof(sibles_periodic_adv_t) + per_len);
    memcpy(para.periodic_data->data, g_peri_adv, per_len);
    para.periodic_data->len = per_len;

    ret = sibles_advertising_init(g_app_advertising_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_context);
    }

    rt_free(para.adv_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
    rt_free(para.periodic_data);
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

#ifdef RT_USING_HEAP
#ifdef RT_USING_SYSTEM_WORKQUEUE
static void ble_app_per_adv_update(struct rt_work *work, void *work_data)
{
    app_env_t *env = (app_env_t *)work_data;
    if (env->per_adv_change_inv != 0)
    {
        memset(g_peri_adv, g_peri_adv[0] + 1, env->per_adv_len);
        sibles_periodic_adv_t *data = rt_malloc(sizeof(sibles_periodic_adv_t) + env->per_adv_len);
        memcpy(data->data, g_peri_adv, env->per_adv_len);
        data->len = env->per_adv_len;
        sibles_advertising_update_periodic_data(g_app_advertising_context, data);
        rt_free(data);

        rt_work_submit(work, env->per_adv_change_inv);
    }
}
#endif //RT_USING_SYSTEM_WORKQUEUE
#endif //RT_USING_HEAP


int main(void)
{
    int count = 0;
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();

    env->per_adv_len = 80;

#ifdef RT_USING_HEAP
#ifdef RT_USING_SYSTEM_WORKQUEUE
    rt_delayed_work_init(&env->work, ble_app_per_adv_update, env);
#endif //RT_USING_SYSTEM_WORKQUEUE
#endif //RT_USING_HEAP
    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;

            /* First enable connectable adv then enable non-connectable. */
            ble_app_peri_advertising_start();
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
    case BLE_GAP_CREATE_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_create_per_adv_sync_cnf_t *cnf = (ble_gap_create_per_adv_sync_cnf_t *)data;
        LOG_I("Create PER_ADV_SYNC result %d", cnf->status);
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
        if (strcmp(argv[1], "trc") == 0)
        {
            if (strcmp(argv[2], "mode") == 0)
            {
                uint8_t mode = atoi(argv[3]);
                uint32_t mask = atoi(argv[4]);
                sibles_set_trc_cfg(mode, mask);
            }
        }
        else if (strcmp(argv[1], "adv_start") == 0)
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
            update_adv_content();
        }
        else if (strcmp(argv[1], "gen_addr") == 0)
        {
            bd_addr_t addr;
            int ret;
            ret = bt_mac_addr_generate_via_uid_v2(&addr);
            LOG_D("ret %d", ret);
        }

#ifdef RT_USING_HEAP
#ifdef RT_USING_SYSTEM_WORKQUEUE
        else if (strcmp(argv[1], "keep_per") == 0)
        {
            if (strcmp(argv[2], "start") == 0)
            {
                env->per_adv_change_inv = atoi(argv[3]);
                uint8_t per_adv_len = atoi(argv[4]);
                if (per_adv_len > APP_MAX_PER_ADV_LEN)
                    per_adv_len = APP_MAX_PER_ADV_LEN;

                env->per_adv_len = per_adv_len;
                LOG_I("per_adv_refresh start");
                rt_work_submit(&env->work.work, env->per_adv_change_inv);
            }
            else if (strcmp(argv[2], "stop") == 0)
            {
                env->per_adv_change_inv = 0;
                rt_work_cancel(&env->work.work);
            }
        }
#endif //RT_USING_SYSTEM_WORKQUEUE
#endif //RT_USING_HEAP
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


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

