/**
  ******************************************************************************
  * @file   bf0_bt_metrics.c
  * @author Sifli software development team
  * @brief Metrics Collector source
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2023 - 2023,  Sifli Technology
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

#ifdef BT_METRICS


#include "bf0_sibles.h"
#include "bf0_sibles_util.h"

#include "bf0_ble_gap.h"
#include "bf0_ble_common.h"

#include "bts2_app_demo.h"
#include "bts2_task.h"
#include "gap_api.h"
#include "av_api.h"
#include "hfp_hf_api.h"
#include "avrcp_api.h"
#include "hci_spec.h"
#include "bts2_util.h"

#define DBG_LVL           DBG_INFO
#include "log.h"


#include "metrics_id_middleware.h"
#include "metrics_collector.h"

typedef struct
{
    uint16_t conn_hdl;
    uint8_t is_ble;
    uint8_t is_conn;
    uint8_t status;
    uint8_t incoming;
} bt_metrics_conn_t;

typedef struct
{
    uint8_t scan_type;
} bt_metrics_scan_t;

typedef struct
{
    uint8_t is_adv_on;
    uint8_t adv_idx;
} bt_metrics_adv_t;


static int bt_metrics_hci_event_handler(uint16_t event_id, uint8_t *msg)
{
    bt_metrics_conn_t *ptr;
    switch (event_id)
    {
    case DM_EN_ACL_OPENED_IND:
    {
        ptr = (bt_metrics_conn_t *)mc_alloc_metrics(METRICS_MW_BT_CONN, sizeof(bt_metrics_conn_t));
        LOG_D("conn %x", ptr);
        if (ptr != NULL)
        {
            BTS2S_DM_EN_ACL_OPENED_IND *ind = (BTS2S_DM_EN_ACL_OPENED_IND *)msg;
            ptr->is_ble = 0;
            ptr->is_conn = 1;
            ptr->conn_hdl = ind->phdl;
            ptr->status = ind->st;
            ptr->incoming = ind->incoming;
            mc_save_metrics(ptr, TRUE);
        }
        break;
    }
    case DM_ACL_DISC_IND:
    {
        ptr = (bt_metrics_conn_t *)mc_alloc_metrics(METRICS_MW_BT_CONN, sizeof(bt_metrics_conn_t));
        LOG_D("disconn %x", ptr);
        if (ptr != NULL)
        {
            BTS2S_DM_ACL_DISC_IND *ind = (BTS2S_DM_ACL_DISC_IND *)msg;
            ptr->is_ble = 0;
            ptr->is_conn = 0;
            ptr->conn_hdl = ind->hdl;
            ptr->status = ind->reason;
            mc_save_metrics(ptr, TRUE);
        }
        break;
    }
    default:
        break;
    }

    return 0;
}


static int bt_metrics_gap_event_handler(uint16_t event_id, uint8_t *msg)
{
    switch (event_id)
    {
    case BTS2MU_GAP_RD_SCAN_ENB_CFM:
    {
        bt_metrics_scan_t *ptr;
        BTS2S_GAP_RD_SCAN_ENB_CFM *cfm  = (BTS2S_GAP_RD_SCAN_ENB_CFM *)msg;
        if (cfm->res == BTS2_SUCC)
        {
            ptr = (bt_metrics_scan_t *)mc_alloc_metrics(METRICS_MW_BT_SCAN, sizeof(bt_metrics_scan_t));

            LOG_D("scan %x", ptr);
            if (ptr != NULL)
            {
                ptr->scan_type = cfm->scan_enb;
                mc_save_metrics(ptr, TRUE);
            }
        }
        break;
    }
    default:
        break;
    }
    return 0;
}


int bt_metrics_event_hdl(U16 type, U16 event_id, uint8_t *msg, uint32_t context)
{
    LOG_D("bt met evt %d, event %d ", type, event_id);
    if (type == BTS2M_HCI_CMD)
    {
        bt_metrics_hci_event_handler(event_id, msg);
    }
    else if (type == BTS2M_GAP)
    {
        bt_metrics_gap_event_handler(event_id, msg);
    }

    return 0;
}


BT_EVENT_REGISTER(bt_metrics_event_hdl, NULL);



static int ble_metrics_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    switch (event_id)
    {
    case BLE_GAP_CONNECTED_IND:
    {
        bt_metrics_conn_t *ptr;
        ptr = (bt_metrics_conn_t *)mc_alloc_metrics(METRICS_MW_BT_CONN, sizeof(bt_metrics_conn_t));
        if (ptr)
        {
            ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
            ptr->is_ble = 1;
            ptr->is_conn = 1;
            ptr->incoming = ind->role == 1 ? 1 : 0;
            ptr->conn_hdl = ind->conn_idx;
            mc_save_metrics(ptr, TRUE);
        }
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        bt_metrics_conn_t *ptr;
        ptr = (bt_metrics_conn_t *)mc_alloc_metrics(METRICS_MW_BT_CONN, sizeof(bt_metrics_conn_t));
        if (ptr)
        {
            ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
            ptr->is_ble = 1;
            ptr->is_conn = 0;
            ptr->status = ind->reason;
            ptr->conn_hdl = ind->conn_idx;
            mc_save_metrics(ptr, TRUE);
        }
        break;
    }
    case BLE_GAP_START_ADV_CNF:
    {
        bt_metrics_adv_t *ptr;
        ptr = (bt_metrics_adv_t *)mc_alloc_metrics(METRICS_MW_BT_ADV, sizeof(bt_metrics_adv_t));
        ble_gap_start_adv_cnf_t *cnf = (ble_gap_start_adv_cnf_t *)data;
        if (!ptr)
            break;

        if (cnf->status == HL_ERR_NO_ERROR)
        {
            ptr->is_adv_on = 1;
            ptr->adv_idx = cnf->actv_index;
            mc_save_metrics(ptr, TRUE);
        }
        else
            mc_free_metrics(ptr);
        break;
    }
    case BLE_GAP_ADV_STOPPED_IND:
    {
        bt_metrics_adv_t *ptr;
        ptr = (bt_metrics_adv_t *)mc_alloc_metrics(METRICS_MW_BT_ADV, sizeof(bt_metrics_adv_t));
        ble_gap_adv_stopped_ind_t *cnf = (ble_gap_adv_stopped_ind_t *)data;
        if (ptr)
        {
            ptr->is_adv_on = 0;
            ptr->adv_idx = cnf->actv_idx;
            mc_save_metrics(ptr, TRUE);
        }
        break;
    }
    case BT_DBG_RSSI_NOTIFY_IND:
    {
        bt_dbg_rssi_notify_ind_t *ind = (bt_dbg_rssi_notify_ind_t *)data;
        bt_dbg_rssi_notify_ind_t *ptr = (bt_dbg_rssi_notify_ind_t *)mc_alloc_metrics(METRICS_MW_BT_RSSI, sizeof(bt_dbg_rssi_notify_ind_t) + ind->rssi_num);
        if (ptr)
        {
            memcpy(ptr, ind, sizeof(bt_dbg_rssi_notify_ind_t) + ind->rssi_num);
            mc_save_metrics(ptr, TRUE);
        }
        break;
    }
    default:
        break;
    }

    return 0;
}


BLE_EVENT_REGISTER(ble_metrics_event_handler, NULL);


bool bt_metric_read_handler(uint16_t id, uint8_t core, uint16_t data_len, uint32_t time, void *data)
{
    LOG_I("id %d, core %d, len %d, time %d", id, core, data_len, time);
    LOG_HEX("mt_data", 16, data, data_len);
    return FALSE;
}


static void bt_mt(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "read") == 0)
        {
            mc_read_metrics(bt_metric_read_handler);
        }
    }
}



//#ifdef RT_USING_FINSH
MSH_CMD_EXPORT(bt_mt, BT metrics command);
//#endif // RT_USING_FINSH


#endif // BT_METRICS


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

