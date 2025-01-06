/**
  ******************************************************************************
  * @file   bf0_sibles_advertising.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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
#include "stdlib.h"

#include "os_adaptor.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_util.h"

#include "bf0_ble_gap.h"
#include "bf0_ble_common.h"
#include "bf0_sibles_advertising.h"
#include "bf0_sibles_advertising_internal.h"
#include "log.h"

/*
 * Internal Functions
 ****************************************************************************************
 */


typedef enum
{
    SIBLES_ADV_START,
    SIBLES_ADV_RESTART,
    SIBLES_ADV_RECREATE
} sibles_adv_transist_t;

__WEAK uint8_t sibles_advertising_data_compose(sibles_advertising_context_t *context, sibles_adv_data_t *adv_data, uint8_t is_scan_rsp)
{
    // minus AD flag types
    uint8_t data[SIBLES_MAX_ADV_DATA_LENGTH - 3] = {0};
    uint8_t max_len = SIBLES_MAX_ADV_DATA_LENGTH - 3;
    uint8_t offset = 0;
    uint8_t ret = SIBLES_ADV_NO_ERR;
    do
    {
        if (adv_data->disc_mode != 0)
        {
            context->adv_para.disc_mode = adv_data->disc_mode;

            if (adv_data->disc_mode == GAPM_ADV_MODE_CUSTOMIZE)
            {
                ret = sibles_advertising_data_flags_compose(data, &offset, adv_data->flags, max_len);
                if (ret != SIBLES_ADV_NO_ERR)
                    break;
            }
        }

        if (adv_data->tx_pwr_level != NULL)
        {
            ret = sibles_advertising_data_tx_pwr_compose(data, &offset, *adv_data->tx_pwr_level, max_len);
            if (ret != SIBLES_ADV_NO_ERR)
                break;
        }

        if (adv_data->appearance != NULL)
        {
            ret = sibles_advertising_data_apperance_compose(data, &offset, *adv_data->appearance, max_len);
            if (ret != SIBLES_ADV_NO_ERR)
                break;
        }

        if (adv_data->advertising_interval != NULL)
        {
            ret = sibles_advertising_data_adv_interval_compose(data, &offset, *adv_data->advertising_interval, max_len);
            if (ret != SIBLES_ADV_NO_ERR)
                break;
        }

        if (adv_data->shortened_name != NULL)
        {
            ret = sibles_advertising_data_local_name_compose(data, &offset, adv_data->shortened_name, 1, max_len);
            if (ret != SIBLES_ADV_NO_ERR)
                break;
        }

        if (adv_data->completed_name != NULL)
        {
            ret = sibles_advertising_data_local_name_compose(data, &offset, adv_data->completed_name, 0, max_len);
            if (ret != SIBLES_ADV_NO_ERR)
                break;
        }

        if (adv_data->completed_uuid != NULL)
        {
            sibles_advertising_data_service_uuid_compose(data, &offset, adv_data->completed_uuid, 1, max_len);
        }

        if (adv_data->incompleted_uuid != NULL)
        {
            sibles_advertising_data_service_uuid_compose(data, &offset, adv_data->incompleted_uuid, 0, max_len);
        }

        if (adv_data->srv_data != NULL)
        {
            sibles_advertising_data_service_data_compose(data, &offset, adv_data->srv_data, max_len);
        }

        if (adv_data->preferred_conn_interval != NULL)
        {
            sibles_advertising_data_conn_interval_compose(data, &offset, adv_data->preferred_conn_interval, max_len);
        }

        if (adv_data->manufacturer_data)
        {
            sibles_advertising_data_manufacturer_compose(data, &offset, adv_data->manufacturer_data, max_len);
        }

        if (adv_data->customized_data)
        {
            if (offset + adv_data->customized_data->len > max_len)
            {
                ret = SIBLES_ADV_DATA_LENGTH_EXCEED;
                break;
            }

            memcpy(&data[offset], adv_data->customized_data->data, adv_data->customized_data->len);
            offset += adv_data->customized_data->len;
        }
    }
    while (0);

    if (ret == SIBLES_ADV_NO_ERR && offset != 0)
    {
        if (is_scan_rsp)
        {
            context->scan_rsp_data = bt_mem_alloc(sizeof(ble_gap_adv_data_t) + offset);
            BT_OOM_ASSERT(context->scan_rsp_data);
            context->scan_rsp_data->length = offset;
            memcpy(context->scan_rsp_data->data, data, offset);
        }
        else
        {
            context->adv_data = bt_mem_alloc(sizeof(ble_gap_adv_data_t) + offset);
            BT_OOM_ASSERT(context->adv_data);
            context->adv_data->length = offset;
            memcpy(context->adv_data->data, data, offset);
        }
    }

    return ret;
}


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


uint8_t sibles_advertising_init(sibles_advertising_context_t *context, sibles_advertising_para_t *para)
{
    uint32_t ret = SIBLES_ADV_PARAM_INVALID;

    do
    {

        if (!context || !para)
        {
            break;
        }

        if (context->adv_mode == SIBLES_ADV_PERIODIC_MODE &&
                para->periodic_data == NULL)
        {
            break;
        }

        ret = SIBLES_ADV_NO_ERR;

        // 1. Store configuration.
        context->config = para->config;
        context->evt_handler = para->evt_handler;

        // 2. Set ADV parameter.
        context->adv_para.own_addr_type = para->own_addr_type;
        context->adv_para.peer_addr = para->peer_addr;
        if (context->config.adv_mode == SIBLES_ADV_EXTENDED_MODE)
        {
            context->adv_para.type = GAPM_ADV_TYPE_EXTENDED;
        }
        else if (context->config.adv_mode == SIBLES_ADV_PERIODIC_MODE)
        {
            context->adv_para.type = GAPM_ADV_TYPE_PERIODIC;
        }
        else
        {
            context->adv_para.type = GAPM_ADV_TYPE_LEGACY;
        }
        context->adv_para.disc_mode = GAPM_ADV_MODE_NON_DISC;
        context->adv_para.max_tx_pwr = para->config.max_tx_pwr;
        context->adv_para.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;

        context->adv_para.prim_cfg.chnl_map = 0x07;
        context->adv_para.prim_cfg.phy = GAP_PHY_TYPE_LE_1M;
        context->adv_mode = para->config.adv_mode;


        // Set advertising properties.
        if (context->adv_mode == SIBLES_ADV_DIRECTED_CONNECT_MODE)
        {
            if (context->config.mode_config.directed_config.high_duty_cycle_enabled)
                context->adv_para.prop =  GAPM_ADV_PROP_DIRECTED_BIT | GAPM_ADV_PROP_HDC_BIT | GAPM_ADV_PROP_CONNECTABLE_BIT;
            else
                context->adv_para.prop = GAPM_ADV_PROP_DIRECTED_BIT | GAPM_ADV_PROP_CONNECTABLE_BIT;
            context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.directed_config.interval;
            context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.directed_config.interval;
            break;
        }
        else if (context->adv_mode == SIBLES_ADV_CONNECT_MODE)
        {
            context->adv_para.prop = GAPM_ADV_PROP_CONNECTABLE_BIT | GAPM_ADV_PROP_SCANNABLE_BIT;
            context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.conn_config.interval;
            context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.conn_config.interval;
        }
        else if (context->adv_mode == SIBLES_ADV_BROADCAST_MODE)
        {
            context->adv_para.prop = 0;
            if (context->config.mode_config.broadcast_config.scannable_enable)
            {
                context->adv_para.prop = GAPM_ADV_PROP_SCANNABLE_BIT;
            }
            context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.broadcast_config.interval;
            context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.broadcast_config.interval;
        }
        else if (context->adv_mode == SIBLES_ADV_EXTENDED_MODE)
        {
            context->adv_para.prop = 0;
            if (context->config.mode_config.extended_config.connectable_enable)
            {
                context->adv_para.prop = GAPM_ADV_PROP_CONNECTABLE_BIT;
            }
            context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.extended_config.interval;
            context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.extended_config.interval;
            context->adv_para.second_cfg.max_skip = context->config.mode_config.extended_config.max_skip;
            context->adv_para.second_cfg.phy = context->config.mode_config.extended_config.phy;
            context->adv_para.second_cfg.adv_sid = context->config.mode_config.extended_config.adv_sid;
        }
        else if (context->adv_mode == SIBLES_ADV_PERIODIC_MODE)
        {
            context->adv_para.prop = 0;
            context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.periodic_config.interval;
            context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.periodic_config.interval;
            context->adv_para.second_cfg.max_skip = context->config.mode_config.periodic_config.max_skip;
            context->adv_para.second_cfg.phy = context->config.mode_config.periodic_config.phy;
            context->adv_para.second_cfg.adv_sid = context->config.mode_config.periodic_config.adv_sid;
            context->adv_para.period_cfg.adv_intv_min = context->config.mode_config.periodic_config.adv_intv_min;
            context->adv_para.period_cfg.adv_intv_max = context->config.mode_config.periodic_config.adv_intv_max;
        }

        // Compose and save AD data and Scan RSP data
        sibles_advertising_data_compose(context, &para->adv_data, 0);
        // Broadcast mode is non-conn and non-scannable mode
        if (context->adv_mode == SIBLES_ADV_CONNECT_MODE)
        {
            if (context->config.is_rsp_data_duplicate)
                sibles_advertising_data_compose(context, &para->adv_data, 1);
            else
                sibles_advertising_data_compose(context, &para->rsp_data, 1);
        }

        if (context->adv_mode == SIBLES_ADV_PERIODIC_MODE)
        {
            context->periodic_data = bt_mem_alloc(sizeof(ble_gap_adv_data_t) + para->periodic_data->len);
            BT_OOM_ASSERT(context->periodic_data);
            context->periodic_data->length = para->periodic_data->len;
            memcpy(context->periodic_data->data, context->periodic_data->data, para->periodic_data->len);
        }

    }
    while (0);
    // Parser ADV data.

    if (ret == SIBLES_ADV_NO_ERR)
    {
        context->state = SIBLES_ADV_STATE_READY;
        context->adv_transist = SIBLES_ADV_START;
        context->conn_idx = INVALID_CONN_IDX;
    }

    return ret;
}

static uint8_t sibles_advertising_create(sibles_advertising_context_t *context)
{
    if (context->adv_mode == SIBLES_ADV_DIRECTED_CONNECT_MODE)
    {
        // req peer deivce address.
        if (context->evt_handler)
            context->evt_handler(SIBLES_ADV_EVT_REQUEST_PEER_DEVICE_ADDR, context, &context->adv_para.peer_addr);
        if (context->config.mode_config.directed_config.high_duty_cycle_enabled)
            context->adv_running_mode = SIBLES_ADV_MODE_HIGH_DUTY_DIRECTED;
        else
            context->adv_running_mode = SIBLES_ADV_MODE_LOW_DUTY_DIRECTED;
    }
    else if (context->adv_mode == SIBLES_ADV_CONNECT_MODE)
    {
        if (context->backgroud_adv)
        {
            context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.conn_config.backgroud_interval;
            context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.conn_config.backgroud_interval;
            context->adv_running_mode = SIBLES_ADV_MODE_BACKGROUD_CONNECT;
        }
        else
        {
            context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.conn_config.interval;
            context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.conn_config.interval;
            context->adv_running_mode = SIBLES_ADV_MODE_FAST_CONNECT;
        }
    }
    else if (context->adv_mode == SIBLES_ADV_BROADCAST_MODE)
    {
        context->adv_running_mode = SIBLES_ADV_MODE_BROADCAST;
    }
    else if (context->adv_mode == SIBLES_ADV_EXTENDED_MODE)
    {
        context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.extended_config.interval;
        context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.extended_config.interval;
        context->adv_para.second_cfg.max_skip = context->config.mode_config.extended_config.max_skip;
        context->adv_para.second_cfg.phy = context->config.mode_config.extended_config.phy;
        context->adv_para.second_cfg.adv_sid = context->config.mode_config.extended_config.adv_sid;
        context->adv_running_mode = SIBLES_ADV_MODE_EXTENDED;
    }
    else if (context->adv_mode == SIBLES_ADV_PERIODIC_MODE)
    {
        context->adv_running_mode = SIBLES_ADV_MODE_PERIODIC;
    }

    if (context->config.white_list_enable)
    {
        // req upper layer set whitelist
        if (context->evt_handler)
            context->evt_handler(SIBLES_ADV_EVT_REQUEST_SET_WHITE_LIST, context, NULL);
        context->adv_para.filter_pol = ADV_ALLOW_SCAN_WLST_CON_WLST;
    }

    context->state = SIBLES_ADV_STATE_STARTING;
    ble_gap_create_advertising(&context->adv_para);

    return SIBLES_ADV_NO_ERR;

}

static uint8_t sibles_advertising_restart(sibles_advertising_context_t *context)
{

    ble_gap_adv_start_t adv_start;
    adv_start.actv_idx = context->adv_idx;
    if (context->state == SIBLES_ADV_STATE_STARTING)
        return SIBLES_ADV_NOT_ALLOWED;

    if (context->adv_mode == SIBLES_ADV_DIRECTED_CONNECT_MODE)
    {
        adv_start.duration = context->config.mode_config.directed_config.duration;
    }
    else if (context->adv_mode == SIBLES_ADV_CONNECT_MODE)
    {
        if (context->backgroud_adv)
            adv_start.duration = context->config.mode_config.conn_config.backgroud_duration;
        else
            adv_start.duration = context->config.mode_config.conn_config.duration;
    }
    else if (context->adv_mode == SIBLES_ADV_BROADCAST_MODE)
    {
        adv_start.duration = context->config.mode_config.broadcast_config.duration;
    }
    else if (context->adv_mode == SIBLES_ADV_EXTENDED_MODE)
    {
        adv_start.duration = context->config.mode_config.extended_config.duration;
    }
    else if (context->adv_mode == SIBLES_ADV_PERIODIC_MODE)
    {
        adv_start.duration = context->config.mode_config.periodic_config.duration;
    }


    adv_start.max_adv_evt = 0;
    context->state = SIBLES_ADV_STATE_STARTING;
    // here should be optimized, set adv data may failed.
    ble_gap_start_advertising(&adv_start);
    LOG_I("Sibles ADV start %d!\r\n", context->adv_idx);
    return SIBLES_ADV_NO_ERR;

}

uint8_t sibles_advertising_start(sibles_advertising_context_t *context)
{
    uint8_t ret = SIBLES_ADV_NOT_ALLOWED;
    switch (context->adv_transist)
    {
    case SIBLES_ADV_START:
    {
        ret = sibles_advertising_create(context);
        break;
    }
    case SIBLES_ADV_RESTART:
    {
        ret = sibles_advertising_restart(context);
        break;
    }
    case SIBLES_ADV_RECREATE:
    {
        ret = SIBLES_ADV_NO_ERR;
        ble_gap_adv_delete_t del;
        del.actv_idx = context->adv_idx;
        ble_gap_delete_advertising(&del);
        break;
    }
    default:
        break;
    }

    return ret;
}


uint8_t sibles_advertising_stop(sibles_advertising_context_t *context)
{
    ble_gap_adv_stop_t stop_req;
    stop_req.actv_idx = context->adv_idx;
    uint8_t ret = SIBLES_ADV_NO_ERR;
    if (context->state != SIBLES_ADV_STATE_STARTED)
        return SIBLES_ADV_NOT_ALLOWED;

    context->state = SIBLES_ADV_STATE_STOPPING;
    if (ble_gap_stop_advertising(&stop_req) != HL_ERR_NO_ERROR)
        ret = SIBLES_ADV_PARAM_INVALID;

    return ret;
}


uint8_t sibles_advertising_delete(sibles_advertising_context_t *context)
{
    ble_gap_adv_stop_t stop_req;
    if (context->state == SIBLES_ADV_STATE_IDLE)
        return SIBLES_ADV_NO_ERR;
    stop_req.actv_idx = context->adv_idx;
    uint8_t ret = SIBLES_ADV_NO_ERR;

    ble_gap_adv_delete_t del;
    del.actv_idx = context->adv_idx;
    ble_gap_delete_advertising(&del);

    return ret;
}


uint8_t sibles_advertising_update_adv_and_scan_rsp_data(sibles_advertising_context_t *context,
        sibles_adv_data_t *adv_data,
        sibles_adv_data_t *rsp_data)
{
    uint8_t ret = SIBLES_ADV_NO_ERR;

    if (context->state == SIBLES_ADV_STATE_IDLE ||
            context->state == SIBLES_ADV_STATE_STARTING ||
            context->state == SIBLES_ADV_STATE_STOPPING)
        return SIBLES_ADV_NOT_ALLOWED;

    if (context->adv_mode == SIBLES_ADV_DIRECTED_CONNECT_MODE)
        return SIBLES_ADV_PARAM_INVALID;

    do
    {
        if (adv_data)
        {
            if (context->adv_data)
            {
                bt_mem_free(context->adv_data);
                context->adv_data = NULL;
            }
            ret = sibles_advertising_data_compose(context, adv_data, 0);
            if (ret != SIBLES_ADV_NO_ERR)
                break;

            if (context->state == SIBLES_ADV_STATE_STARTED)
            {
                if (context->adv_data)
                {
                    context->adv_data->actv_idx = context->adv_idx;
                    ble_gap_set_adv_data(context->adv_data);
                }
            }
        }

        if (rsp_data)
        {
            if (context->adv_mode == SIBLES_ADV_BROADCAST_MODE && !context->config.mode_config.broadcast_config.scannable_enable)
            {
                // Broadcast is non-scannable, but due to ADV is supported, just break
                break;
            }
            if (context->scan_rsp_data)
            {
                bt_mem_free(context->scan_rsp_data);
                context->scan_rsp_data = NULL;
            }

            if (context->config.is_rsp_data_duplicate)
                ret = sibles_advertising_data_compose(context, adv_data, 1);
            else
                ret = sibles_advertising_data_compose(context, rsp_data, 1);

            if (ret != SIBLES_ADV_NO_ERR)
                break;

            if (context->state == SIBLES_ADV_STATE_STARTED)
            {
                if (context->scan_rsp_data)
                {
                    context->scan_rsp_data->actv_idx = context->adv_idx;
                    ble_gap_set_scan_rsp_data(context->scan_rsp_data);
                }
            }

        }
        else if (context->adv_mode != SIBLES_ADV_BROADCAST_MODE && context->config.is_rsp_data_duplicate)
        {

        }
    }
    while (0);

    return ret;
}


uint8_t sibles_advertising_update_periodic_data(sibles_advertising_context_t *context,
        sibles_periodic_adv_t *periodic_data)
{
    uint8_t ret = SIBLES_ADV_NO_ERR;

    if (context->state == SIBLES_ADV_STATE_IDLE ||
            context->state == SIBLES_ADV_STATE_STARTING ||
            context->state == SIBLES_ADV_STATE_STOPPING)
        return SIBLES_ADV_NOT_ALLOWED;

    if (context->adv_mode != SIBLES_ADV_PERIODIC_MODE)
        return SIBLES_ADV_PARAM_INVALID;

    do
    {
        if (periodic_data)
        {
            if (context->periodic_data)
            {
                bt_mem_free(context->periodic_data);
                context->periodic_data = NULL;
            }

            context->periodic_data = bt_mem_alloc(sizeof(ble_gap_adv_data_t) + periodic_data->len);
            BT_OOM_ASSERT(context->periodic_data);
            context->periodic_data->length = periodic_data->len;
            memcpy(context->periodic_data->data, periodic_data->data, periodic_data->len);

            if (context->state >= SIBLES_ADV_STATE_READY)
            {
                if (context->periodic_data)
                {
                    context->periodic_data->actv_idx = context->adv_idx;
                    ble_gap_set_periodic_adv_data(context->periodic_data);
                }
            }
        }

    }
    while (0);

    return ret;
}


uint8_t sibles_advertising_reconfig(sibles_advertising_context_t *context, sibles_adv_config_t *config)
{
    //if (context->state != SIBLES_ADV_STATE_READY)
    //return SIBLES_ADV_NOT_ALLOWED;

    context->config = *config;
    context->adv_para.max_tx_pwr = config->max_tx_pwr;
    context->adv_mode = config->adv_mode;

    // Set advertising properties.
    if (context->adv_mode == SIBLES_ADV_DIRECTED_CONNECT_MODE)
    {
        if (context->config.mode_config.directed_config.high_duty_cycle_enabled)
            context->adv_para.prop = GAPM_ADV_PROP_DIRECTED_BIT | GAPM_ADV_PROP_HDC_BIT;
        else
            context->adv_para.prop = GAPM_ADV_PROP_DIRECTED_BIT;
        context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.directed_config.interval;
        context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.directed_config.interval;
    }
    else if (context->adv_mode == SIBLES_ADV_CONNECT_MODE)
    {
        context->adv_para.prop = GAPM_ADV_PROP_CONNECTABLE_BIT | GAPM_ADV_PROP_SCANNABLE_BIT;
        context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.conn_config.interval;
        context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.conn_config.interval;
    }
    else if (context->adv_mode == SIBLES_ADV_BROADCAST_MODE)
    {
        context->adv_para.prop = 0;
        if (context->config.mode_config.broadcast_config.scannable_enable)
        {
            context->adv_para.prop = GAPM_ADV_PROP_SCANNABLE_BIT;
        }
        context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.broadcast_config.interval;
        context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.broadcast_config.interval;
    }
    else if (context->adv_mode == SIBLES_ADV_EXTENDED_MODE)
    {
        context->adv_para.prop = 0;
        if (context->config.mode_config.extended_config.connectable_enable)
        {
            context->adv_para.prop = GAPM_ADV_PROP_CONNECTABLE_BIT;
        }
        context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.extended_config.interval;
        context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.extended_config.interval;
        context->adv_para.second_cfg.max_skip = context->config.mode_config.extended_config.max_skip;
        context->adv_para.second_cfg.phy = context->config.mode_config.extended_config.phy;
        context->adv_para.second_cfg.adv_sid = context->config.mode_config.extended_config.adv_sid;
    }
    else if (context->adv_mode == SIBLES_ADV_PERIODIC_MODE)
    {
        context->adv_para.prop = 0;
        context->adv_para.prim_cfg.adv_intv_max = context->config.mode_config.periodic_config.interval;
        context->adv_para.prim_cfg.adv_intv_min = context->config.mode_config.periodic_config.interval;
        context->adv_para.second_cfg.max_skip = context->config.mode_config.periodic_config.max_skip;
        context->adv_para.second_cfg.phy = context->config.mode_config.periodic_config.phy;
        context->adv_para.second_cfg.adv_sid = context->config.mode_config.periodic_config.adv_sid;
        context->adv_para.period_cfg.adv_intv_min = context->config.mode_config.periodic_config.adv_intv_min;
        context->adv_para.period_cfg.adv_intv_max = context->config.mode_config.periodic_config.adv_intv_max;
    }

    context->adv_transist = SIBLES_ADV_RECREATE;

    return SIBLES_ADV_NO_ERR;
}



int sibles_advertising_evt_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    sibles_advertising_context_t *context_p = (sibles_advertising_context_t *)context;
    switch (event_id)
    {
    case BLE_GAP_ADV_CREATED_IND:
    {
        ble_gap_adv_created_ind_t *ind = (ble_gap_adv_created_ind_t *)data;
        if (context_p->state != SIBLES_ADV_STATE_STARTING)
            break;

        context_p->adv_idx = ind->actv_idx;
        context_p->adv_transist = SIBLES_ADV_RESTART;
        /* Wait created cnf. */
        break;
    }
    case BLE_GAP_CREATE_ADV_CNF:
    {
        ble_gap_create_adv_cnf_t *ret = (ble_gap_create_adv_cnf_t *)data;
        if (context_p->state != SIBLES_ADV_STATE_STARTING)
            break;

        if (ret->status != HL_ERR_NO_ERROR)
        {
            sibles_adv_evt_startted_t start_evt;
            context_p->state = SIBLES_ADV_STATE_IDLE;
            if (context_p->adv_data)
            {
                bt_mem_free(context_p->adv_data);
                context_p->adv_data = NULL;
            }
            if (context_p->scan_rsp_data)
            {
                bt_mem_free(context_p->scan_rsp_data);
                context_p->scan_rsp_data = NULL;
            }
            if (context_p->periodic_data)
            {
                bt_mem_free(context_p->periodic_data);
                context_p->periodic_data = NULL;
            }
            start_evt.status = ret->status;
            start_evt.adv_mode = context_p->adv_running_mode;
            if (context_p->evt_handler)
                context_p->evt_handler(SIBLES_ADV_EVT_ADV_STARTED, context_p, &start_evt);
            LOG_E("Create advertising failed!");
            break;
        }

        ble_gap_adv_start_t adv_start;

        if (context_p->adv_mode == SIBLES_ADV_CONNECT_MODE ||
                context_p->adv_mode == SIBLES_ADV_BROADCAST_MODE ||
                context_p->adv_mode == SIBLES_ADV_EXTENDED_MODE ||
                context_p->adv_mode == SIBLES_ADV_PERIODIC_MODE)
        {
            if (context_p->adv_data)
            {
                context_p->adv_data->actv_idx = context_p->adv_idx;
                ble_gap_set_adv_data(context_p->adv_data);
            }
            if (context_p->scan_rsp_data)
            {
                context_p->scan_rsp_data->actv_idx = context_p->adv_idx;
                ble_gap_set_scan_rsp_data(context_p->scan_rsp_data);
            }
            if (context_p->periodic_data)
            {
                context_p->periodic_data->actv_idx = context_p->adv_idx;
                ble_gap_set_periodic_adv_data(context_p->periodic_data);
            }
        }
        adv_start.actv_idx = context_p->adv_idx;

        if (context_p->adv_mode == SIBLES_ADV_DIRECTED_CONNECT_MODE)
        {
            // TODO: Wrong assignment
            adv_start.duration = context_p->config.mode_config.directed_config.duration;
        }
        else if (context_p->adv_mode == SIBLES_ADV_CONNECT_MODE)
        {
            if (context_p->backgroud_adv)
                adv_start.duration = context_p->config.mode_config.conn_config.backgroud_duration;
            else
                adv_start.duration = context_p->config.mode_config.conn_config.duration;
        }
        else if (context_p->adv_mode == SIBLES_ADV_BROADCAST_MODE)
        {
            adv_start.duration = context_p->config.mode_config.broadcast_config.duration;
        }
        else if (context_p->adv_mode == SIBLES_ADV_EXTENDED_MODE)
        {
            adv_start.duration = context_p->config.mode_config.extended_config.duration;
        }
        else if (context_p->adv_mode == SIBLES_ADV_PERIODIC_MODE)
        {
            adv_start.duration = context_p->config.mode_config.periodic_config.duration;
        }

        adv_start.max_adv_evt = 0;
        // here should be optimized, set adv data may failed.
        ble_gap_start_advertising(&adv_start);
        LOG_I("Sibles ADV start %d!\r\n", context_p->adv_idx);
        break;
    }
    case BLE_GAP_START_ADV_CNF:
    {
        ble_gap_start_adv_cnf_t *cnf = (ble_gap_start_adv_cnf_t *)data;
        sibles_adv_evt_startted_t start_evt;
        if (context_p->state != SIBLES_ADV_STATE_STARTING)
            break;
        if (cnf->actv_index == context_p->adv_idx)
        {
            if (cnf->status == HL_ERR_NO_ERROR)
            {

                context_p->state = SIBLES_ADV_STATE_STARTED;
            }
            else
            {
                context_p->state = SIBLES_ADV_STATE_IDLE;
                if (context_p->adv_data)
                {
                    bt_mem_free(context_p->adv_data);
                    context_p->adv_data = NULL;
                }
                if (context_p->scan_rsp_data)
                {
                    bt_mem_free(context_p->scan_rsp_data);
                    context_p->scan_rsp_data = NULL;
                }
                if (context_p->periodic_data)
                {
                    bt_mem_free(context_p->periodic_data);
                    context_p->periodic_data = NULL;
                }
            }

            start_evt.status = cnf->status;
            start_evt.adv_mode = context_p->adv_running_mode;
            if (context_p->evt_handler)
                context_p->evt_handler(SIBLES_ADV_EVT_ADV_STARTED, context_p, &start_evt);
        }
        break;

    }
    case BLE_GAP_ADV_STOPPED_IND:
    {
        ble_gap_adv_stopped_ind_t *ind = (ble_gap_adv_stopped_ind_t *)data;
        sibles_adv_evt_stopped_t stop_evt;
        if (ind->actv_idx == context_p->adv_idx)
        {
            // ADV stopped due to duration expired
            if (ind->reason == GAP_ERR_TIMEOUT)
            {
                if (context_p->adv_mode == SIBLES_ADV_CONNECT_MODE &&
                        context_p->config.mode_config.conn_config.backgroud_mode_enabled)
                {
                    if (context_p->backgroud_adv == 0)
                        context_p->backgroud_adv = 1;
                    else
                        context_p->backgroud_adv = 0;
                }

                if (context_p->config.mode_config.conn_config.is_repeated == 1
                        || context_p->backgroud_adv == 1)
                {
                    // Swtich to idle and start adv again.
                    context_p->state = SIBLES_ADV_STATE_IDLE;
                    context_p->adv_transist = SIBLES_ADV_RECREATE;
                    sibles_advertising_start(context_p);
                    break;
                }

            }
            else
            {
                context_p->backgroud_adv = 0;
            }

            {
                // Notify user directly even in SIBLES_ADV_STATE_STOPPING state.
                stop_evt.reason = ind->reason;
                stop_evt.adv_mode = context_p->adv_running_mode;
                context_p->state = SIBLES_ADV_STATE_READY;
                if (context_p->evt_handler)
                    context_p->evt_handler(SIBLES_ADV_EVT_ADV_STOPPED, context_p, &stop_evt);
            }
        }
        break;
    }
    case BLE_GAP_STOP_ADV_CNF:
    {
        // Do nothing because BLE_GAP_STOP_ADV_CNF will always later than BLE_GAP_ADV_STOPPED_IND.
        break;
    }
    case BLE_GAP_DELETE_ADV_CNF:
    {
        ble_gap_delete_adv_cnf_t *ind = (ble_gap_delete_adv_cnf_t *)data;
        if (ind->actv_index == context_p->adv_idx)
        {
            if (context_p->adv_transist == SIBLES_ADV_RECREATE)
            {
                context_p->adv_transist = SIBLES_ADV_START;
                sibles_advertising_start(context_p);
            }
            else
            {
                context_p->state = SIBLES_ADV_STATE_IDLE;
            }
        }
        break;
    }
    case SIBLES_REMOTE_CONNECTED_IND:
    {
        sibles_remote_connected_ind_t *ind = (sibles_remote_connected_ind_t *)data;
        // Enter connected state, all adv except broadcast should not re-start again.

        if (context_p->conn_idx == INVALID_CONN_IDX)
        {
            context_p->conn_idx = ind->conn_idx;
        }
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        if
        (ind->conn_idx == context_p->conn_idx &&
                context_p->state == SIBLES_ADV_STATE_READY &&
                context_p->config.is_auto_restart == 1)
        {
            if (context_p->adv_mode == SIBLES_ADV_CONNECT_MODE &&
                    context_p->config.mode_config.conn_config.backgroud_mode_enabled)
            {
                if (context_p->adv_running_mode == SIBLES_ADV_MODE_BACKGROUD_CONNECT)
                {
                    context_p->backgroud_adv = 0;

                    context_p->state = SIBLES_ADV_STATE_IDLE;
                    context_p->adv_transist = SIBLES_ADV_RECREATE;

                    ble_gap_adv_delete_t del;
                    del.actv_idx = context_p->adv_idx;
                    ble_gap_delete_advertising(&del);
                }
                else
                {
                    sibles_advertising_start(context_p);
                }
            }
            else
            {
                sibles_advertising_start(context_p);
            }
        }
        if (ind->conn_idx == context_p->conn_idx)
        {
            context_p->conn_idx = INVALID_CONN_IDX;
        }
        break;
    }
    default:
        break;
    }
    return 0;



}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
