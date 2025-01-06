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

#include "button.h"
#include "main.h"



static app_env_t g_app_env;
static rt_mailbox_t g_app_mb;

app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}


static void ble_app_button_event_handler(int32_t pin, button_action_t action)
{
    app_env_t *env = ble_app_get_env();

    LOG_D("button(%d) %d, click(%d)", pin, action, env->click);
    if (action == BUTTON_CLICKED)
    {
        if ((env->click % 2) == 0)
        {
            ble_app_sender_trigger();
        }
        else
        {
            ble_app_sender_stop();
        }
        env->click++;
    }
}


static void ble_app_button_init(void)
{
    button_cfg_t cfg;

    cfg.pin = BSP_KEY1_PIN;

    cfg.active_state = BSP_KEY1_ACTIVE_HIGH;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = ble_app_button_event_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));

}


int main(void)
{
    int count = 0;
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    // enable button
    ble_app_button_init();
    // enable BLE
    sifli_ble_enable();
    // sender init
    ble_app_sender_init();
    // receiver init
    ble_app_receviver_init();

    // main loop
    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
    }
    return RT_EOK;
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


int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    app_env_t *env = ble_app_get_env();
    int ret = 0;
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        /* Handle in own thread to avoid conflict */
        env->is_power_on = 1;
        ble_app_scan_init();
        ble_app_peri_advertising_init();
        LOG_I("receive BLE power on!\r\n");

        break;
    }
    case BLE_GAP_SCAN_START_CNF:
    case BLE_GAP_SCAN_STOP_CNF:
    case BLE_GAP_SCAN_STOPPED_IND:
    case BLE_GAP_EXT_ADV_REPORT_IND:
    case BLE_GAP_CREATE_PERIODIC_ADV_SYNC_CNF:
    case BLE_GAP_START_PERIODIC_ADV_SYNC_CNF:
    case BLE_GAP_DELETE_PERIODIC_ADV_SYNC_CNF:
    case BLE_GAP_PERIODIC_ADV_SYNC_CREATED_IND:
    case BLE_GAP_PERIODIC_ADV_SYNC_STOPPED_IND:
    case BLE_GAP_PERIODIC_ADV_SYNC_ESTABLISHED_IND:
    {
        ret = ble_app_receiver_event_handler(event_id, data, len, context);
    }
    default:
        break;
    }
    return ret;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);


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

