/**
  ******************************************************************************
  * @file   sender.c
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

#include "main.h"


SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);

#define DEFAULT_LOCAL_NAME "SIFLI_APP"
#define EXAMPLE_LOCAL_NAME "SIFLI_EXAMPLE"

typedef enum
{
    APP_SNED_STATE_IDLE,
    APP_SEND_STATE_ADV_PREPARE,
    APP_SEND_STATE_ADVERTISING,
} app_send_state_t;


static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data);


static void ble_app_peri_advertising_start(void)
{
    sibles_advertising_start(g_app_advertising_context);
}


uint8_t ble_app_sender_is_working(void)
{
    app_env_t *env = ble_app_get_env();
    return (env->s_env.state == APP_SEND_STATE_ADVERTISING);
}

/* Enable advertise via advertising service. */
void ble_app_peri_advertising_init(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;
    app_env_t *env = ble_app_get_env();

    char local_name[31] = {0};
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
    para.config.mode_config.periodic_config.interval = 0x140;

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
    uint8_t cod_len = strlen(DEFAULT_NETWORK_CODE);
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + cod_len + 1);
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = cod_len + 1;
    para.adv_data.manufacturer_data->additional_data[0] = cod_len;
    rt_memcpy((void *)&para.adv_data.manufacturer_data->additional_data[1], DEFAULT_NETWORK_CODE, cod_len);

    para.evt_handler = ble_app_advertising_event;

    uint8_t per_len = APP_MAX_PER_ADV_LEN;
    para.periodic_data = rt_malloc(sizeof(sibles_periodic_adv_t) + per_len);
    memset(para.periodic_data->data, 0, per_len);
    para.periodic_data->len = per_len;

    ret = sibles_advertising_init(g_app_advertising_context, &para);
    RT_ASSERT(ret == SIBLES_ADV_NO_ERR);

    rt_free(para.adv_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
    rt_free(para.periodic_data);
}



static void ble_app_sender_work_handler(struct rt_work *work, void *work_data)
{
    app_env_t *env = (app_env_t *)work_data;
    if (env->s_env.state == APP_SEND_STATE_ADV_PREPARE)
    {
        ble_app_peri_advertising_start();
    }
    else if (env->s_env.state == APP_SEND_STATE_ADVERTISING)
    {
        sibles_advertising_stop(g_app_advertising_context);
#ifndef APP_MIC_ALWAYS_ON
        talk_deinit();
#endif
    }
}


uint8_t app_send_voice_data(uint16_t len, uint8_t *voice_data)
{
    // parameter error
    if (!voice_data)
        return 1;

    sibles_periodic_adv_t *data = rt_malloc(sizeof(sibles_periodic_adv_t) + len);
    memcpy(data->data, voice_data, len);
    data->len = len;
    sibles_advertising_update_periodic_data(g_app_advertising_context, data);
    rt_free(data);

    return 0;
}

uint8_t ble_app_sender_trigger(void)
{
    app_env_t *env = ble_app_get_env();
    uint8_t ret = 1;
    if (env->s_env.state == APP_SNED_STATE_IDLE)
    {
        env->s_env.state = APP_SEND_STATE_ADV_PREPARE;
        rt_work_submit(&env->s_env.work.work, 0);
        ret = 0;
    }
    return ret;
}

uint8_t ble_app_sender_stop(void)
{
    app_env_t *env = ble_app_get_env();
    if (env->s_env.state == APP_SEND_STATE_ADV_PREPARE
            || env->s_env.state == APP_SEND_STATE_ADVERTISING)
    {
        env->s_env.is_stopping = 1;
        if (env->s_env.state == APP_SEND_STATE_ADVERTISING)
        {
            rt_work_submit(&env->s_env.work.work, 0);
        }
    }

    return 0;
}

static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data)
{
    app_env_t *env = ble_app_get_env();

    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("ADV start resutl %d, mode %d\r\n", evt->status, evt->adv_mode);
        if (evt->status == HL_ERR_NO_ERROR &&
                evt->adv_mode == SIBLES_ADV_MODE_PERIODIC)
        {
            if (env->s_env.is_stopping)
            {
                sibles_advertising_stop(g_app_advertising_context);
            }
            else
            {
#ifndef APP_MIC_ALWAYS_ON
                talk_init(AUDIO_TX);
#endif
                ble_app_scan_restart();
                env->s_env.state = APP_SEND_STATE_ADVERTISING;
            }
        }
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("ADV stopped reason %d, mode %d\r\n", evt->reason, evt->adv_mode);
        env->s_env.is_stopping = 0;
        env->s_env.state = APP_SNED_STATE_IDLE;
        break;
    }
    default:
        break;
    }
    return 0;
}


int ble_app_sender_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    app_env_t *env = ble_app_get_env();

    switch (event_id)
    {
    default:
        break;
    }
    return 0;
}


void ble_app_sender_init(void)
{
    app_env_t *env = ble_app_get_env();

    rt_delayed_work_init(&env->s_env.work, ble_app_sender_work_handler, env);
}



int sender(int argc, char *argv[])
{
    app_env_t *env = ble_app_get_env();

    if (argc > 1)
    {
        if (strcmp(argv[1], "adv_init") == 0)
        {
            ble_app_peri_advertising_start();
        }
        else if (strcmp(argv[1], "adv_start") == 0)
        {
            sibles_advertising_start(g_app_advertising_context);
        }
        else if (strcmp(argv[1], "adv_stop") == 0)
        {
            sibles_advertising_stop(g_app_advertising_context);
        }
    }

    return 0;
}

#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(sender, sender cmd);
#endif


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
