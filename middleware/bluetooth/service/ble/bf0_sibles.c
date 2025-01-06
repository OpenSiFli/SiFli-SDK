/**
  ******************************************************************************
  * @file   bf0_sibles.c
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
#include <stdlib.h>
#ifdef RWBT_ENABLE
    #include "rwip.h"
#endif
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_util.h"

#include "bf0_sibles_nvds.h"
#include "bf0_ble_gap.h"
#include "bf0_ble_common.h"

#include "bf0_ble_gap_internal.h"

#ifdef RT_USING_BT
    #include "bts2_app_inc.h"
#endif

#ifdef BSP_BLE_CONNECTION_MANAGER
    #include "ble_connection_manager.h"
#endif

#define SIBLES_MBOX_HLEN        4

#ifndef SIBLES_MAX_SVCS
    #define SIBLES_MAX_SVCS 8
#endif

#ifndef SIBLES_MAX_REMOTE_SVCS
    #define SIBLES_MAX_REMOTE_SVCS 5
#endif

// for android, search svc will return end handle as 0xffff
// this may alloc a big value
#define LAST_SVC_LEN 40

#define LOG_TAG "sibles"
#include "log.h"

#include "mem_section.h"



#ifdef BSP_USING_PSRAM
    #define SIBLES_NON_RET_SECT_BEGIN
    #define SIBLES_NON_RET_SECT_END
#else
    #define SIBLES_NON_RET_SECT_BEGIN L1_NON_RET_BSS_SECT_BEGIN(sible_not)
    #define SIBLES_NON_RET_SECT_END   L1_NON_RET_BSS_SECT_END
#endif



SIBLES_NON_RET_SECT_BEGIN
uint8_t data_buf[MAX_MAIL_SIZE];
SIBLES_NON_RET_SECT_END


uint8_t *sifli_get_mbox_buffer(void)
{
    return data_buf;
}

struct sibles_remote_svc
{
    sibles_svc_remote_svc_t svc;
    uint16_t db_len;
};


struct sibles_remote_info
{
    bd_addr_t addr;
    uint8_t conn_idx;
    struct sibles_remote_svc svc[SVC_SEARCH_MAX];
    uint16_t svc_search_id[SVC_SEARCH_MAX];
};


struct sibles_svc_env
{
    uint8_t hdl_start;
    uint8_t hdl_num;
    uint8_t svc_status;
    uint8_t svc_uuid_len;
    uint8_t *svc_uuid;
    uint8_t *att_db;
    sibles_get_cbk get_cbk;
    sibles_set_cbk set_cbk;
};


struct sibles_remote_svc_env
{
    uint16_t hdl_start;
    uint16_t hdl_stop;
    uint8_t svc_status;
    uint8_t conn_idx;
    sibles_remote_svc_cbk callback;
};

struct sibles_rte_wr_info
{
    rt_slist_t   next;
    struct sibles_value_write_req_content_t *value;
};

struct sibles_env
{
    rt_sem_t sem;
    sifli_task_id_t app_task_id;
    uint8_t status;
    uint8_t num_of_tx_pkt;
    struct sibles_svc_env svcs[SIBLES_MAX_SVCS];
#ifdef BLE_GATT_CLIENT
    struct sibles_remote_info remote_info;
    struct sibles_remote_svc_env remote_svc[SIBLES_MAX_REMOTE_SVCS];
    rt_slist_t wr_node;//add for buffer tx pkt
#endif //BLE_GATT_CLIENT
};

struct sibles_env g_sibles;

/******************** Mailbox help functions**********************************/

const char *const sibles_msg_str[] =
{
    "SET_VALUE_REQ",
    "SET_VALUE_RSP",
    "VALUE_REQ_IND",
    "VALUE_REQ_CFM",
    "VALUE_WRITE_IND",
    "VALUE_WRITE_CFM",
    "VALUE_IND_REQ",
    "VALUE_IND_RSP",
    "SVC_REG_REQ",
    "SVC_REG128_REQ",
    "SVC_RSP",
    "ADV_DATA_REQ",
    "SCAN_RSP_REQ",
    "ADV_CMD_REQ",
    "NAME_REQ",
    "CMD_RSP",
    "SVC_SEARCH",
    "DIS_SVC_IND",
    "DIS_CHAR_IND",
    "DIS_DESCR_IND",
    "SVC_SEARCH_RSP",
    "SVC_REG_NOTIFY_REQ",
    "SVC_REG_NOTIFY_RSP",
    "VALUE_WRITE_REQ",
    "EVENT_IND",
    "VALUE_READ_REQ",
    "VALUE_READ_RSP",
    "CONNECTED_IND",
    "VALUE_NTF_IND",
    "SET_STATIC_RANDOM_ADDR_REQ",
    "WLAN_COEX_ENABLE_REQ",
    "SIBLES_TRC_CFG_REQ",
    "WRTIE_VALUE_RSP",
    "ENABLE_DBG",
    "SIBLES_SVC_READY_IND",
};



const char *const ahi_task_str[] =
{
    "L2CC",
    "GATTM",
    "GATTC",
    "GAPM",
    "GAPC",
    "APP",
    "AHI",
    "HCI",
    "AHI_INT"
};

char *msg_str(uint16_t msg)
{
    char *r = NULL;

    if (msg == SIBLES_SVC_READY_IND)
        r = "SVC_READY_IND";
    else
    {
        switch (msg >> 8)
        {
        case TASK_ID_SIBLES:
            if (msg < sizeof(sibles_msg_str))
                r = (char *)sibles_msg_str[msg & 0xff];
            else
                r = "unknown sible msg";
            break;
        case TASK_ID_L2CC:
        case TASK_ID_GATTM:
        case TASK_ID_GATTC:
        case TASK_ID_GAPM:
        case TASK_ID_GAPC:
        case TASK_ID_APP:
        case TASK_ID_AHI:
        case TASK_ID_HCI:
        case TASK_ID_AHI_INT:
            r = (char *)ahi_task_str[(msg >> 8) - TASK_ID_L2CC];
            break;
        default:
            r = "??";
            break;
        }
    }
    return r;
}


/****************** SIBLE app help functions**********************************/

static struct sibles_svc_env *get_svc_handle(uint16_t msg_id)
{
    int i;
    struct sibles_svc_env *r = NULL;

    for (i = 0; i < SIBLES_MAX_SVCS; i++)
    {
        r = &g_sibles.svcs[i];
        if (r->svc_status == SIBLES_IDLE && msg_id == SIBLES_SVC_RSP)
            break;
        if (r->svc_status == SIBLES_BUSY)
            break;
    }
    if (i == SIBLES_MAX_SVCS)
        r = NULL;
    return r;
}

static void sibles_send_value_writecfm(uint8_t conn_idx, uint8_t hdl, uint8_t status)
{
    struct sibles_value_ack *cfm;
    sifli_task_id_t task_id = g_sibles.app_task_id;
    cfm = (struct sibles_value_ack *)sifli_msg_alloc(SIBLES_VALUE_WRITE_CFM, TASK_BUILD_ID(task_id, conn_idx), sifli_get_stack_id(), sizeof(struct sibles_value_ack));

    cfm->hdl = hdl;
    cfm->status = status;
    sifli_msg_send((void const *)cfm);
    //sifli_msg_free(cfm);


}

static void sibles_send_event_req_ind_cfm(sibles_event_ind_t *data)
{
    if (data->type == GATTC_INDICATE)
    {
        struct gattc_event_cfm *cfm = sifli_msg_alloc(GATTC_EVENT_CFM,
                                      TASK_BUILD_ID(TASK_ID_GATTC, 0),
                                      sifli_get_stack_id(),
                                      sizeof(struct gattc_event_cfm));

        cfm->handle = data->handle;
        sifli_msg_send((void const *)cfm);
    }
}

static void sibles_tx_pkts_init(void)
{
    g_sibles.num_of_tx_pkt = MAX_NUM_OF_TX_PKT;
}

RT_WEAK void sibles_acquire_tx_pkts_hook(uint8_t buffer_num)
{
    return;
}

uint8_t sibles_get_tx_pkts(void)
{
    return g_sibles.num_of_tx_pkt;
}

static unsigned int sibles_get_slist_len(const rt_slist_t *l)
{
    unsigned int buffer_num;
    rt_enter_critical();
    buffer_num = rt_slist_len(l);
    rt_exit_critical();
    return buffer_num;
}

static uint8_t sibles_acquire_tx_pkts(void)
{
#ifdef BLE_GATT_CLIENT
    uint8_t buffer_num = sibles_get_slist_len(&g_sibles.wr_node);
    sibles_acquire_tx_pkts_hook(buffer_num);
#endif //BLE_GATT_CLIENT
    rt_enter_critical();


    if (sibles_get_tx_pkts() == 0)
    {
        rt_exit_critical();
        return 0;
    }

    if (g_sibles.num_of_tx_pkt) // To avoid send data in ISR in future.
        g_sibles.num_of_tx_pkt--;
    rt_exit_critical();
    return 1;
}

#ifdef BLE_GATT_CLIENT
void sibles_clear_wr_list(uint8_t conn_idx)
{
    rt_enter_critical();
    rt_slist_t *fir_node = rt_slist_first(&(g_sibles.wr_node));
    rt_exit_critical();
    struct sibles_rte_wr_info *info;
    struct sibles_value_write_req_content_t *msginfo;
    uint8_t  buffer_num;
    rt_slist_t *nex_node;

    while (NULL != fir_node)
    {
        info = (struct sibles_rte_wr_info *)fir_node;
        rt_enter_critical();
        nex_node = rt_slist_next(fir_node);

        if (info->value->conn_idx == conn_idx)
        {
            rt_slist_remove(&(g_sibles.wr_node), fir_node);
            rt_exit_critical();
            msginfo = info->value;
            bt_mem_free(fir_node);
            bt_mem_free(msginfo);
            fir_node = nex_node;
        }
        else
        {
            rt_exit_critical();
            fir_node = nex_node;
        }
    }

    buffer_num = sibles_get_slist_len(&g_sibles.wr_node);

    LOG_I("sibles_clear_wr_list num %d\n", buffer_num);
}
#endif //BLE_GATT_CLIENT
void sibles_check_wr_list_msg(void)
{
    rt_enter_critical();
#ifdef BLE_GATT_CLIENT
    rt_slist_t *fir_node = rt_slist_first(&(g_sibles.wr_node));
#endif //BLE_GATT_CLIENT
    struct sibles_rte_wr_info *info;
    struct sibles_value_write_req_content_t *msginfo;
    sifli_task_id_t task_id = g_sibles.app_task_id;
    uint8_t conn_idx;

#ifdef BLE_GATT_CLIENT
    if (NULL == fir_node)
#endif //BLE_GATT_CLIENT
    {
        g_sibles.num_of_tx_pkt++;
        rt_exit_critical();
        RT_ASSERT(g_sibles.num_of_tx_pkt <= MAX_NUM_OF_TX_PKT);
        LOG_I("rev wr rsp:left_txnum %d\n", g_sibles.num_of_tx_pkt);
        return;
    }
#ifdef BLE_GATT_CLIENT
    rt_slist_remove(&(g_sibles.wr_node), fir_node);
    rt_exit_critical();
    info = (struct sibles_rte_wr_info *)fir_node;
    msginfo = info->value;
    bt_mem_free(fir_node);

    conn_idx = msginfo->conn_idx;
    struct sibles_value_write_req_t *req = (struct sibles_value_write_req_t *)sifli_msg_alloc(SIBLES_VALUE_WRITE_REQ,
                                           TASK_BUILD_ID(task_id, conn_idx), TASK_BUILD_ID(sifli_get_stack_id(), conn_idx), sizeof(struct sibles_value_write_req_t) +  msginfo->length);
    req->write_type = msginfo->write_type;
    req->seq_num = msginfo->seq_num;
    req->handle = msginfo->handle;
    req->length =  msginfo->length;
    memcpy(req->value, msginfo->value, msginfo->length);
    sifli_msg_send((void const *)req);


    bt_mem_free(msginfo);
#endif //BLE_GATT_CLIENT
}

static void sibles_release_tx_pkts(void)
{
    sibles_check_wr_list_msg();
}

#ifdef BLE_GATT_CLIENT
static struct sibles_remote_svc_env *sibles_get_remote_svc_by_handle(uint8_t conn_idx, uint16_t handle)
{
    uint32_t i;
    for (i = 0; i < SIBLES_MAX_REMOTE_SVCS; i++)
    {
        if (handle <= g_sibles.remote_svc[i].hdl_stop &&
                handle >= g_sibles.remote_svc[i].hdl_start &&
                conn_idx == g_sibles.remote_svc[i].conn_idx)
        {
            break;
        }
    }
    if (i == SIBLES_MAX_REMOTE_SVCS)
        return NULL;

    return &g_sibles.remote_svc[i];

}

#if 0
static sibles_svc_search_char_t *sifli_get_last_char(struct sibles_remote_svc *svc)
{
    sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)svc->svc.att_db;
    uint16_t offset = 0;
    for (uint32_t i = 0; i < svc->svc.char_count - 1; i++)
    {
        offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
        chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
    }
    return chara;

}
#endif

static void sifli_add_remote_char(struct sibles_remote_svc *svc, struct sibles_disc_char_ind *disc_char)
{
    sibles_svc_search_char_t *prev_char = (sibles_svc_search_char_t *)svc->svc.att_db;
    sibles_svc_search_char_t *chara;
    uint16_t offset = 0;
    if (svc->svc.char_count != 0)
    {
        for (uint32_t i = 0; i < svc->svc.char_count - 1; i++)
        {
            offset = sizeof(sibles_svc_search_char_t) + prev_char->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            prev_char = (sibles_svc_search_char_t *)((uint8_t *)prev_char + offset);
        }
    }
    if (svc->svc.char_count == 0)
    {
        chara = prev_char;
    }
    else
    {
        // one chara only has 1 attr handle and 1 vlaue handle
        prev_char->desc_count = disc_char->attr_hdl - prev_char->attr_hdl - 2;
        offset = sizeof(sibles_svc_search_char_t) + prev_char->desc_count * sizeof(struct sibles_disc_char_desc_ind);
        chara = (sibles_svc_search_char_t *)((uint8_t *)prev_char + offset);
    }
    if (((uint8_t *)chara - (uint8_t *)svc->svc.att_db) > svc->db_len)
        RT_ASSERT(0); /// Could not add to the database
    chara->attr_hdl = disc_char->attr_hdl;
    chara->pointer_hdl = disc_char->pointer_hdl;
    chara->prop = disc_char->prop;
    chara->uuid_len = disc_char->uuid_len;
    memcpy(chara->uuid, disc_char->uuid, disc_char->uuid_len);
    svc->svc.char_count++;
}


static void sifli_add_remote_char_desc(struct sibles_remote_svc *svc, struct sibles_disc_char_desc_ind *disc_char_desc)
{
    sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)svc->svc.att_db;
    uint16_t offset = 0;
    uint32_t i;
    if (svc->svc.char_count == 0)
        return;
    for (i = 0; i < svc->svc.char_count - 1; i++)
    {
        if (disc_char_desc->attr_hdl <= (chara->pointer_hdl + chara->desc_count) &&
                disc_char_desc->attr_hdl > chara->pointer_hdl)
        {
            memcpy((struct sibles_disc_char_desc_ind *)&chara->desc + (disc_char_desc->attr_hdl - chara->attr_hdl) - 2,
                   disc_char_desc, sizeof(struct sibles_disc_char_desc_ind));
            return;
        }
        offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
        chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
    }

    if (disc_char_desc->attr_hdl <= svc->svc.hdl_end && disc_char_desc->attr_hdl >= chara->attr_hdl + 2)
    {
        // Speicial handle for the last descriptor
        memcpy((struct sibles_disc_char_desc_ind *)&chara->desc + (disc_char_desc->attr_hdl - chara->attr_hdl) - 2,
               disc_char_desc, sizeof(struct sibles_disc_char_desc_ind));
        chara->desc_count++;
    }

}

static uint16_t sibles_get_last_handle(sibles_svc_search_rsp_t *rsp)
{
    uint16_t last_handle = 0;
    uint16_t offset = 0;
    sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;

    for (uint8_t i = 0; i < rsp->svc->char_count; i++)
    {
        if (i == rsp->svc->char_count - 1)
        {
            if (chara->desc_count == 0)
            {
                last_handle = chara->pointer_hdl;
                break;
            }
            else
            {
                last_handle = chara->desc[chara->desc_count - 1].attr_hdl;
                break;
            }
        }

        offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
        chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
    }
    return last_handle;
}

#ifdef BLE_GATT_CLIENT
static struct sibles_remote_svc *sibles_get_empty_search_id(uint8_t conn_idx)
{
    struct sibles_remote_svc *svc = NULL;
    for (int i = 0; i < SVC_SEARCH_MAX; i++)
    {
        if (SVC_SEARCH_STATE_GET(g_sibles.remote_info.svc_search_id[i]) == SVC_SEARCH_EMPTY)
        {
            g_sibles.remote_info.svc_search_id[i] = SVC_SEARCH_BUILD_ID(SVC_SEARCH_BUSY, conn_idx);
            svc = &g_sibles.remote_info.svc[i];
            break;
        }
    }
    return svc;
}

static struct sibles_remote_svc *sibles_get_current_search_id(uint8_t conn_idx, uint8_t reset)
{
    struct sibles_remote_svc *svc = NULL;
    for (int i = 0; i < SVC_SEARCH_MAX; i++)
    {
        if (SVC_SEARCH_STATE_GET(g_sibles.remote_info.svc_search_id[i]) == SVC_SEARCH_BUSY &&
                SVC_SEARCH_IDX_GET(g_sibles.remote_info.svc_search_id[i]) == conn_idx)
        {
            svc = &g_sibles.remote_info.svc[i];
            if (reset)
            {
                g_sibles.remote_info.svc_search_id[i] = SVC_SEARCH_BUILD_ID(SVC_SEARCH_EMPTY, conn_idx);
            }
            break;
        }
    }
    return svc;
}
#endif //BLE_GATT_CLIENT

#endif


void sifli_mbox_process(sibles_msg_para_t *header, uint8_t *data_ptr, uint16_t param_len)
{

    RT_ASSERT(header != NULL);
    RT_ASSERT(data_ptr != NULL);

    sifli_msg_id_t msg_id = header->id;
    uint8_t conn_idx = TASK_IDX_GET(header->src_id);

    if ((msg_id >= GAPM_CMP_EVT && msg_id <= GAPM_CONFIG_RESOLUTION) ||
            (msg_id >= GAPC_CMP_EVT && msg_id <= GAPC_SMP_REP_ATTEMPTS_TIMER_IND))
    {
        //LOG_D("GAP event %d", msg_id);
        ble_gap_event_process(header, data_ptr, param_len);
        return;
    }

    switch (msg_id)
    {
    case SIBLES_SVC_READY_IND:
        LOG_I("\nBLE ready!\n");
#ifndef SOC_SF32LB55X
        bt_system_mask_clear(BT_RESET_MASK_BLE);
#endif
        //sifli_sem_release();
        g_sibles.app_task_id = header->src_id;
        g_sibles.status = SIBLES_READY;
        // To avoid conflict between BT and BLE
#ifndef BT_FINSH
        ble_gap_get_local_version();
#endif
        ble_event_publish(BLE_POWER_ON_IND, NULL, 0);
        break;
    case SIBLES_SVC_RSP:
    {
        struct sibles_svc_rsp *rsp;
        struct sibles_svc_env *env;
        rsp = (struct sibles_svc_rsp *)data_ptr;
        env = get_svc_handle(msg_id);
        if (env == NULL)
            LOG_W("  Got SIBLES_SVC_RSP without request!!!");
        if (rsp->status == 0)
        {
            env->hdl_start = rsp->start_hdl;
            env->svc_status = SIBLES_READY;
        }
        else
            env->svc_status = SIBLES_EMPTY;
        LOG_I("status=%d, start_handle=0x%x", rsp->status, rsp->start_hdl);
        sifli_sem_release();
    }
    break;

    case SIBLES_SET_VALUE_RSP:
    {
        struct sibles_value_ack *rsp;
        struct sibles_svc_env *env;
        rsp = (struct sibles_value_ack *)data_ptr;
        env = get_svc_handle(msg_id);
        if (env == NULL)
        {
            LOG_W("  Got SIBLES_SET_VALUE_RSP without request!!!");
            break;
        }
        env->svc_status = SIBLES_READY;
        sifli_sem_release();
    }
    break;

    //for opration GATTC_NOTIFY&GATTC_INDICATE
    case SIBLES_VALUE_IND_RSP:
    {
        sibles_write_value_rsp_t rsp;
        struct sibles_value_ack *ack = (struct sibles_value_ack *)data_ptr;
        sibles_release_tx_pkts();
        rsp.result = ack->status;
        rsp.conn_idx = conn_idx;
        ble_event_publish(SIBLES_WRITE_VALUE_RSP, &rsp, sizeof(sibles_write_value_rsp_t));
    }
    break;

    //for opration GATTC_WRITE&GATTC_WRITE_NO_RESPONSE
    case SIBLES_WRTIE_VALUE_RSP:
    {
        sibles_write_remote_value_rsp_t rsp;
        struct sibles_value_ack *ack = (struct sibles_value_ack *)data_ptr;
        sibles_release_tx_pkts();
        rsp.result = ack->status;
        rsp.conn_idx = conn_idx;
        ble_event_publish(SIBLES_WRITE_REMOTE_VALUE_RSP, &rsp, sizeof(sibles_write_remote_value_rsp_t));
    }
    break;
    case SIBLES_VALUE_REQ_IND:
    {
        struct sibles_value_req_ind *ind;
        int i;

        ind = (struct sibles_value_req_ind *)data_ptr;
        for (i = 0; i < SIBLES_MAX_SVCS; i++)
            if (g_sibles.svcs[i].svc_status > SIBLES_IDLE &&
                    ind->hdl >= g_sibles.svcs[i].hdl_start &&
                    ind->hdl < (g_sibles.svcs[i].hdl_start + g_sibles.svcs[i].hdl_num))
                break;
        if (i < SIBLES_MAX_SVCS)
        {
            int idx = ind->hdl - g_sibles.svcs[i].hdl_start;
            sibles_value_t value;

            value.idx = idx;
            value.hdl = &(g_sibles.svcs[i]);
            value.value = NULL;

            if (g_sibles.svcs[i].get_cbk)
            {

                value.value = (*g_sibles.svcs[i].get_cbk)(conn_idx, idx, &value.len);
                value.len |= SIBLE_CFM_FLAG;

                sibles_set_value(conn_idx, &value);
                if (value.len & SIBLE_FREE_FLAG)
                    bt_mem_free(value.value);
            }
            else
            {
                value.len = SIBLE_CFM_FLAG | 0;
                sibles_set_value(conn_idx, &value);
            }
        }
        break;
    }
    case SIBLES_VALUE_WRITE_IND:
    {
        struct sibles_value_write_ind *ind;
        int i;
        uint8_t status;

        ind = (struct sibles_value_write_ind *)data_ptr;
        for (i = 0; i < SIBLES_MAX_SVCS; i++)
            if (g_sibles.svcs[i].svc_status > SIBLES_IDLE &&
                    ind->hdl >= g_sibles.svcs[i].hdl_start &&
                    ind->hdl < (g_sibles.svcs[i].hdl_start + g_sibles.svcs[i].hdl_num))
                break;
        if (i < SIBLES_MAX_SVCS)
        {
            int idx = ind->hdl - g_sibles.svcs[i].hdl_start;
            if (g_sibles.svcs[i].set_cbk)
            {
                sibles_set_cbk_t para;
                para.idx = idx;
                para.len = ind->length;
                para.offset = ind->offset;
                para.value  = ind->data;
                status = (*g_sibles.svcs[i].set_cbk)(conn_idx, &para);
            }
            else
                status = 1;
            if (!ind->is_cmd)
                sibles_send_value_writecfm(conn_idx, ind->hdl, status);
        };
        break;
    }
    case SIBLES_CMD_RSP:
    {
        sifli_sem_release();
        break;
    }
#ifdef SOC_SF32LB55X
    case APP_SIFLI_NVDS_GET_REQ:
    case APP_SIFLI_NVDS_SET_REQ:
    {
        sifli_nvds_handler((void *)header, data_ptr, param_len);
        break;
    }
#endif //SOC_SF32LB55X
#ifdef BLE_GATT_CLIENT
    case SIBLES_DISC_SVC_IND:
    {
        struct sibles_remote_svc *svc = sibles_get_empty_search_id(conn_idx);
        BT_OOM_ASSERT(svc);
        struct sibles_disc_svc_ind *ind = (struct sibles_disc_svc_ind *)data_ptr;

        if (ind->end_hdl == 0xFFFF && (ind->end_hdl - ind->start_hdl) > LAST_SVC_LEN)
        {
            LOG_I("svc disc malloc less");
            ind->end_hdl = ind->start_hdl + LAST_SVC_LEN;
        }

        if (ind->start_hdl == ind->end_hdl)
        {
            // sometimes remote rsp error handle, increase end handle just for malloc
            // serach will end with fail later
            LOG_I("svc disc malloc more");
            ind->end_hdl = ind->start_hdl + LAST_SVC_LEN;
        }

        svc->svc.hdl_start = ind->start_hdl;
        svc->svc.hdl_end = ind->end_hdl;
        svc->svc.uuid_len = ind->uuid_len;
        memcpy(svc->svc.uuid, ind->uuid, ind->uuid_len);
        /// estimiate most of chara has only 1 descriptor.
        svc->db_len = ((svc->svc.hdl_end - svc->svc.hdl_start) / 3) * (sizeof(struct sibles_disc_char_desc_ind) + sizeof(sibles_svc_search_char_t))
                      + ((svc->svc.hdl_end - svc->svc.hdl_start) % 3) * (sizeof(struct sibles_disc_char_desc_ind));
        svc->svc.att_db = bt_mem_alloc(svc->db_len);
        LOG_I("svc find start %d, end %d", ind->start_hdl, ind->end_hdl);
        BT_OOM_ASSERT(svc->svc.att_db);
        if (svc->svc.att_db)
            memset(svc->svc.att_db, 0, svc->db_len);
        break;
    }
    case SIBLES_DISC_CHAR_IND:
    {
        struct sibles_remote_svc *svc = sibles_get_current_search_id(conn_idx, 0);
        BT_OOM_ASSERT(svc);
        struct sibles_disc_char_ind *ind = (struct sibles_disc_char_ind *)data_ptr;
        LOG_I("char found hdl %d, %d", ind->attr_hdl, ind->pointer_hdl);

        if (svc->svc.hdl_end < ind->pointer_hdl)
        {
            LOG_W("char len over, svc end: %d, char end: %d", svc->svc.hdl_end, ind->pointer_hdl);
            svc->svc.hdl_end = ind->pointer_hdl + 1;
            uint8_t *p;

            uint16_t db_len = ((svc->svc.hdl_end - svc->svc.hdl_start) / 3) * (sizeof(struct sibles_disc_char_desc_ind) + sizeof(sibles_svc_search_char_t))
                              + ((svc->svc.hdl_end - svc->svc.hdl_start) % 3) * (sizeof(struct sibles_disc_char_desc_ind));

            p = bt_mem_alloc(db_len);
            BT_OOM_ASSERT(p);
            if (p)
            {
                memcpy(p, svc->svc.att_db, svc->db_len);
                bt_mem_free(svc->svc.att_db);

                svc->db_len = db_len;
                svc->svc.att_db = bt_mem_alloc(svc->db_len);
                BT_OOM_ASSERT(svc->svc.att_db);
                if (svc->svc.att_db)
                    memcpy(svc->svc.att_db, p, svc->db_len);
                bt_mem_free(p);
            }
        }

        sifli_add_remote_char(svc, ind);
        break;
    }
    case SIBLES_DISC_CHAR_DESC_IND:
    {
        struct sibles_remote_svc *svc = sibles_get_current_search_id(conn_idx, 0);
        BT_OOM_ASSERT(svc);
        struct sibles_disc_char_desc_ind *ind = (struct sibles_disc_char_desc_ind *)data_ptr;
        LOG_I("des found hdl %d", ind->attr_hdl);
        sifli_add_remote_char_desc(svc, ind);
        break;
    }
    case SIBLES_SVC_SEARCH_RSP:
    {
        struct sibles_remote_svc *svc = sibles_get_current_search_id(conn_idx, 1);
        struct sibles_svc_search_rsp *rsp = (struct sibles_svc_search_rsp *)data_ptr;
        LOG_I("svc search end %d", rsp->status);

        // Notify upper layer
        sibles_svc_search_rsp_t search_rsp;
        search_rsp.conn_idx = conn_idx;
        search_rsp.result = rsp->status;

        search_rsp.search_svc_len = rsp->len;
        memcpy(search_rsp.search_uuid, rsp->svc_uuid, rsp->len);

        if (rsp->status == HL_ERR_NO_ERROR)
        {
            search_rsp.svc = &svc->svc;
            uint16_t last = sibles_get_last_handle(&search_rsp);
            if (last < search_rsp.svc->hdl_end)
            {
                LOG_I("update end handle from %d to %d", search_rsp.svc->hdl_end, last);
                search_rsp.svc->hdl_end = last;
            }
        }
        else
            search_rsp.svc = NULL;
        ble_event_publish(SIBLES_SEARCH_SVC_RSP, &search_rsp, sizeof(sibles_svc_search_rsp_t));
        if (svc)
        {
            bt_mem_free(svc->svc.att_db);
            memset(svc, 0, sizeof(struct sibles_remote_svc));
        }
        break;
    }
    case SIBLES_REGISTER_NOTIFY_RSP:
    {
        struct sibles_register_notify_rsp_t *rsp = (struct sibles_register_notify_rsp_t *)data_ptr;
        sibles_register_remote_svc_rsp_t ret;
        ret.status = rsp->status;
        ret.conn_idx = conn_idx;
        if (g_sibles.remote_svc[rsp->seq_num].callback)
            g_sibles.remote_svc[rsp->seq_num].callback(SIBLES_REGISTER_REMOTE_SVC_RSP, (uint8_t *)&ret, sizeof(sibles_register_remote_svc_rsp_t));
        break;
    }
    case SIBLES_EVENT_IND:
    {
        struct gattc_event_ind *ind = (struct gattc_event_ind *)data_ptr;
        sibles_remote_event_ind_t noti;
        struct sibles_remote_svc_env *remote_env = sibles_get_remote_svc_by_handle(conn_idx, ind->handle);
        noti.conn_idx = conn_idx;
        noti.type = ind->type;
        noti.handle = ind->handle;
        noti.length = ind->length;
        noti.value = ind->value;
        //LOG_I("recevied notificaiton t(%d), handle(%x), len(%d)", noti.type, noti.handle, noti.length);
        if (remote_env && remote_env->callback)
            remote_env->callback(SIBLES_REMOTE_EVENT_IND, (uint8_t *)&noti, sizeof(sibles_remote_event_ind_t));
        else
            LOG_W("received unexpected gattc notification index %d", conn_idx);
        break;
    }
#endif //BLE_GATT_CLIENT
    case SIBLES_CONNECTED_IND:
    {
        sibles_remote_connected_ind_t ind;
        ind.conn_idx = conn_idx;
        LOG_I("connected %d", conn_idx);
        ble_event_publish(SIBLES_REMOTE_CONNECTED_IND, &ind, sizeof(sibles_remote_connected_ind_t));
        break;
    }
#ifdef BLE_GATT_CLIENT
    case SIBLES_VALUE_READ_RSP:
    {
        struct gattc_read_ind *ind = (struct gattc_read_ind *)data_ptr;
        sibles_read_remote_value_rsp_t rsp;
        struct sibles_remote_svc_env *remote_env = sibles_get_remote_svc_by_handle(conn_idx, ind->handle);
        rsp.conn_idx = conn_idx;
        rsp.handle = ind->handle;
        rsp.length = ind->length;
        rsp.offset = ind->offset;
        rsp.value = ind->value;
        LOG_I("received read rsp handle(%x), len(%d), offset(%d)", ind->handle, ind->length, ind->offset);
        if (remote_env && remote_env->callback)
            remote_env->callback(SIBLES_READ_REMOTE_VALUE_RSP, (uint8_t *)&rsp, sizeof(sibles_read_remote_value_rsp_t));
        else
            LOG_E("received unexpected read rsp index %d", conn_idx);
        break;
    }
#endif //BLE_GATT_CLIENT
    case GATTC_MTU_CHANGED_IND:
    {
        struct gattc_mtu_changed_ind *ind = (struct gattc_mtu_changed_ind *)data_ptr;
        sibles_mtu_exchange_ind_t ex_mtu;
        ex_mtu.mtu = ind->mtu;
        ex_mtu.conn_idx = conn_idx;
        ble_event_publish(SIBLES_MTU_EXCHANGE_IND, &ex_mtu, sizeof(sibles_mtu_exchange_ind_t));
        break;
    }
    case GATTC_CMP_EVT:
    {
        struct gattc_cmp_evt *evt = (struct gattc_cmp_evt *)data_ptr;
        LOG_I("GATTC_CMP_EVT %d, %d", evt->operation, evt->status);
        break;
    }
    case GATTC_EVENT_REQ_IND:
    {
        LOG_I("SIBLES, GATTC_EVENT_REQ_IND");
        struct gattc_event_ind *ind = (struct gattc_event_ind *)data_ptr;
        sibles_event_ind_t event_ind;
        event_ind.type = ind->type;
        event_ind.length = ind->length;
        event_ind.handle = ind->handle;
        event_ind.value = ind->value;
        sibles_send_event_req_ind_cfm(&event_ind);
        ble_event_publish(SIBLES_EVENT_REQ_IND, &event_ind, sizeof(sibles_event_ind_t));
        break;
    }
    case GATTC_TRANSACTION_TO_ERROR_IND:
    {
        LOG_I("Time out error");
        break;
    }
#ifdef BLE_SVC_CHG_ENABLE
    case GATTC_SVC_CHANGED_CFG_IND:
    {
        sibles_svc_changed_cfg_t rsp;
        struct gattc_svc_changed_cfg *cfg = (struct gattc_svc_changed_cfg *)data_ptr;

        rsp.ind_cfg = cfg->ind_cfg;
        rsp.conn_idx = conn_idx;
        LOG_I("SIBLES, GATTC_SVC_CHANGED_CFG_IND %d", cfg->ind_cfg);
        ble_event_publish(SIBLES_SVC_CHANGED_CFG, &rsp, sizeof(sibles_svc_changed_cfg_t));
        break;
    }
#endif //BLE_SVC_CHG_ENABLE
    case SIBLES_CH_BD_ADDR_RSP:
    {
        struct sibles_ch_bd_addr_rsp_t *evt = (struct sibles_ch_bd_addr_rsp_t *)data_ptr;
        sibles_change_bd_addr_rsp_t rsp;

        rsp.status = evt->status;
        LOG_I("BD ADDR change rsp %d", evt->status);
        ble_event_publish(SIBLES_CHANGE_BD_ADDR_RSP, &rsp, sizeof(sibles_change_bd_addr_rsp_t));
#ifdef RT_USING_BT
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_CHANGE_ADDR_RSP, &evt->status, 1);
#endif
        break;
    }
    case COMM_BT_TEST_MODE_CTRL_RSP:
    {
        bt_ns_test_mode_ctrl_rsp_t *rsp = (bt_ns_test_mode_ctrl_rsp_t *)data_ptr;
        int8_t op_str[5][10] = {"enter_dut", "exit_dut", "tx_test", "rx_test", "stop_test"};
        LOG_I("op %s, status %d, para %d", op_str[rsp->op], rsp->status, rsp->para.stop_para.cnt);
        ble_event_publish(BT_NS_DUT_RSP, data_ptr, sizeof(bt_ns_test_mode_ctrl_rsp_t));
        break;
    }
    case SIBLES_UPDATE_ATT_PERM_RSP:
    {
        struct sibles_update_att_perm_rsp *rsp = (struct sibles_update_att_perm_rsp *)data_ptr;
        LOG_I("att update %d, %d", rsp->handle, rsp->status);

        sibles_att_update_perm_ind_t ind;
        ind.handle = rsp->handle;
        ind.status = rsp->status;
        ble_event_publish(SIBLES_ATT_UPDATE_PERM_IND, &ind, sizeof(sibles_att_update_perm_ind_t));
        break;
    }
    case DISS_APP_SET_VALUE_RSP:
    {
        app_dis_set_rsp_t *app_rsp = (app_dis_set_rsp_t *)data_ptr;

        sibles_set_dis_rsp_t rsp;
        rsp.value = app_rsp->value;
        rsp.status = app_rsp->status;
        ble_event_publish(SIBLES_DIS_SET_VAL_RSP, &rsp, sizeof(sibles_set_dis_rsp_t));
        break;
    }
    case 0xFE:
    {
#ifdef SOC_BF_Z0
        // Just for test.
        hwp_ble_rcc->RSTR = BLE_RCC_RSTR_BCPU;
        while (!hwp_ble_rcc->RSTR);
        hwp_ble_rcc->RSTR &= ~BLE_RCC_RSTR_BCPU;
        while (hwp_ble_rcc->RSTR);
        //g_t_msg = rt_tick_get();
        //LOG_I("Reset BCPU, %d, %d\n", g_t_msg - g_r_msg, *((uint32_t *)data_buf));
#endif
        break;
    }
    }
}



/******************************Sifli BLE Enable API***********************************************************/
void sifli_ble_enable(void)
{
#ifdef BSP_USING_PC_SIMULATOR
    extern int uart_pc_available();
    if (uart_pc_available() == 0)
        return;
#endif
    sibles_tx_pkts_init();
#ifdef BLE_GATT_CLIENT
    rt_slist_init(&(g_sibles.wr_node));
#endif //BLE_GATT_CLIENT
    g_sibles.status = SIBLES_IDLE;
    sifli_env_init();

    LOG_I("enable BLE Core. Lib ver: %s", bt_lib_get_ver());
    //sifli_sem_take();
}

/********************************************SIBLE GATT service API************************************************************/


sibles_hdl sibles_register_svc(sibles_register_svc_t *svc)
{
    int i, len = sizeof(struct sibles_svc_reg_req) + sizeof(struct attm_desc) * (svc->num_entry) + 4;
    struct sibles_svc_reg_req *req;

    if (g_sibles.status != SIBLES_READY)
        return NULL;
    for (i = 0; i < SIBLES_MAX_SVCS; i++)
        if (g_sibles.svcs[i].svc_status == SIBLES_EMPTY)
            break;
    if (i == SIBLES_MAX_SVCS)   // Could not register more
        return NULL;
    g_sibles.svcs[i].svc_status = SIBLES_IDLE;
    g_sibles.svcs[i].att_db = (uint8_t *)(svc->att_db);
    g_sibles.svcs[i].hdl_num = svc->num_entry;
    g_sibles.svcs[i].svc_uuid = (uint8_t *)&svc->uuid;
    g_sibles.svcs[i].svc_uuid_len = ATT_UUID_16_LEN;

    sifli_task_id_t task_id = g_sibles.app_task_id;
    req = (struct sibles_svc_reg_req *)sifli_msg_alloc(SIBLES_SVC_REG_REQ,
            task_id, sifli_get_stack_id(), len);
    req->svc_uuid = svc->uuid;
    memcpy(req->att_db, svc->att_db, sizeof(struct attm_desc) * (svc->num_entry));
    req->attm_entries = svc->num_entry;
    req->sec_lvl = svc->sec_lvl;

    sifli_msg_send((void const *)req);
    //sifli_msg_free(req);
    sifli_sem_take();

    return (sibles_hdl) & (g_sibles.svcs[i]);
}




sibles_hdl sibles_register_svc_128(sibles_register_svc_128_t *svc)
{
    int i, len = sizeof(struct sibles_svc_reg128_req) + sizeof(struct attm_desc_128) * (svc->num_entry) + 4;
    struct sibles_svc_reg128_req *req;

    if (g_sibles.status != SIBLES_READY)
        return NULL;
    for (i = 0; i < SIBLES_MAX_SVCS; i++)
        if (g_sibles.svcs[i].svc_status == SIBLES_EMPTY)
            break;
    if (i == SIBLES_MAX_SVCS)   // Could not register more
        return NULL;
    g_sibles.svcs[i].svc_status = SIBLES_IDLE;
    g_sibles.svcs[i].att_db = (uint8_t *)(svc->att_db);
    g_sibles.svcs[i].hdl_num = svc->num_entry;
    g_sibles.svcs[i].svc_uuid = svc->uuid;
    g_sibles.svcs[i].svc_uuid_len = ATT_UUID_128_LEN;

    sifli_task_id_t task_id = g_sibles.app_task_id;
    req = (struct sibles_svc_reg128_req *)sifli_msg_alloc(SIBLES_SVC_REG128_REQ,
            task_id, sifli_get_stack_id(), len);

    memcpy(req->svc_uuid, svc->uuid,
           ((PERM_GET(svc->sec_lvl, SVC_UUID_LEN) == PERM_UUID_16) ? ATT_UUID_16_LEN
            : ((PERM_GET(svc->sec_lvl, SVC_UUID_LEN) == PERM_UUID_32) ? ATT_UUID_32_LEN
               : ATT_UUID_128_LEN)));
    memcpy(req->svc_uuid, svc->uuid, ATT_UUID_128_LEN);
    memcpy(req->att_db, svc->att_db, sizeof(struct attm_desc_128) * (svc->num_entry));
    req->attm_entries = svc->num_entry;
    req->sec_lvl = svc->sec_lvl;

    sifli_msg_send((void const *)req);
    //sifli_msg_free(req);
    sifli_sem_take();

    return (sibles_hdl) & (g_sibles.svcs[i]);
}

void sibles_update_att_permission(uint16_t handle, uint16_t access_mask, uint16_t perm)
{
    struct sibles_update_att_perm_req *req;
    uint16_t len = sizeof(struct sibles_update_att_perm_req);
    sifli_task_id_t task_id = g_sibles.app_task_id;
    req = (struct sibles_update_att_perm_req *)sifli_msg_alloc(SIBLES_UPDATE_ATT_PERM_REQ,
            task_id, sifli_get_stack_id(), len);
    req->handle = handle;
    req->access_mask = access_mask;
    req->perm = perm;

    sifli_msg_send((void const *)req);
}

void sibles_register_cbk(sibles_hdl hdl, sibles_get_cbk gcbk,  sibles_set_cbk scbk)
{
    struct sibles_svc_env *svc = (struct sibles_svc_env *) hdl;
    svc->get_cbk = gcbk;
    svc->set_cbk = scbk;
}




static void sibles_send_value(uint8_t conn_idx, sibles_send_value_t *value)
{
    struct sibles_value *val;

    sifli_task_id_t task_id = g_sibles.app_task_id;
    val = (struct sibles_value *)sifli_msg_alloc(value->msg,
            TASK_BUILD_ID(task_id, conn_idx), sifli_get_stack_id(), sizeof(struct sibles_value) + value->len);

    val->hdl = value->hdl;
    val->length = value->len;
    if (value->value)
        memcpy(val->data, value->value, value->len);

    sifli_msg_send((void const *)val);
    //sifli_msg_free(val);

}


uint8_t sibles_get_gatt_handle(sibles_hdl svc_handle, uint8_t idx)
{
    struct sibles_svc_env *svc = (struct sibles_svc_env *)(svc_handle);
    return idx + svc->hdl_start;
}

uint8_t sibles_get_gatt_handle_by_uuid(uint16_t attr_uuid)
{
    uint8_t index;
    uint8_t found_attr = 0;
    uint8_t svc_index;
    for (int i = 0; i < SIBLES_MAX_SVCS; i++)
    {
        if (g_sibles.svcs[i].svc_status == SIBLES_EMPTY)
        {
            continue;
        }

        struct attm_desc *att_db = (struct attm_desc *)g_sibles.svcs[i].att_db;
        if (att_db == NULL)
        {
            continue;
        }

        for (int j = 0; j < g_sibles.svcs[i].hdl_num; j++)
        {
            if (att_db[j].uuid == attr_uuid)
            {
                svc_index = i;
                index = j;
                found_attr = 1;
                break;
            }
        }
    }

    if (found_attr == 0)
    {
        return 0;
    }

    struct sibles_svc_env *svc = (struct sibles_svc_env *)((sibles_hdl) & (g_sibles.svcs[svc_index]));
    return svc->hdl_start + index;
}

uint16_t sibles_get_uuid_by_attr(uint8_t attr)
{
    uint16_t current_uuid = 0;
    struct sibles_svc_env *svc;

    for (int i = 0; i < SIBLES_MAX_SVCS; i++)
    {
        if (g_sibles.svcs[i].svc_status == SIBLES_EMPTY)
        {
            continue;
        }

        struct attm_desc *att_db = (struct attm_desc *)g_sibles.svcs[i].att_db;
        if (att_db == NULL)
        {
            continue;
        }

        svc = (struct sibles_svc_env *)((sibles_hdl) & (g_sibles.svcs[i]));
        LOG_I("sss %d", svc->hdl_start);
        for (uint8_t j = 0; j < g_sibles.svcs[i].hdl_num; j++)
        {
            if ((svc->hdl_start + j) == attr)
            {
                current_uuid = att_db[j].uuid;
                break;
            }
        }
    }
    return current_uuid;
}

sibles_hdl sibles_get_sible_handle_and_index_by_attr(uint8_t attr, uint8_t *write_index)
{
    struct sibles_svc_env *svc;
    for (int i = 0; i < SIBLES_MAX_SVCS; i++)
    {
        if (g_sibles.svcs[i].svc_status == SIBLES_EMPTY)
        {
            continue;
        }

        struct attm_desc *att_db = (struct attm_desc *)g_sibles.svcs[i].att_db;
        if (att_db == NULL)
        {
            continue;
        }

        svc = (struct sibles_svc_env *)((sibles_hdl) & (g_sibles.svcs[i]));

        for (uint8_t j = 0; j < g_sibles.svcs[i].hdl_num; j++)
        {
            if ((svc->hdl_start + j) == attr)
            {
                *write_index = j;
                return (sibles_hdl) & (g_sibles.svcs[i]);
            }
        }
    }

    return 0;
}

void sibles_get_all_gatt_handle(sibles_local_svc_t *svc)
{
    for (int i = 0; i < SIBLES_MAX_SVCS; i++)
    {
        if (g_sibles.svcs[i].svc_status == SIBLES_EMPTY)
        {
            svc[i].state = 0;
            continue;
        }

        svc[i].state = 1;
        if (g_sibles.svcs[i].svc_uuid_len == ATT_UUID_16_LEN)
        {
            LOG_I("dblen %d, %d, %d", i, g_sibles.svcs[i].hdl_start, g_sibles.svcs[i].hdl_start + g_sibles.svcs[i].hdl_num - 1);
            LOG_HEX("UUID", 16, (uint8_t *)g_sibles.svcs[i].svc_uuid, ATT_UUID_16_LEN);

            svc[i].uuid_len = ATT_UUID_16_LEN;
            memcpy(svc[i].uuid, (uint8_t *)g_sibles.svcs[i].svc_uuid, ATT_UUID_16_LEN);
            svc[i].start_handle = g_sibles.svcs[i].hdl_start;
            svc[i].end_handle = g_sibles.svcs[i].hdl_start + g_sibles.svcs[i].hdl_num - 1;
        }
        else if (g_sibles.svcs[i].svc_uuid_len == ATT_UUID_128_LEN)
        {
            LOG_I("dblen %d, %d, %d", i, g_sibles.svcs[i].hdl_start, g_sibles.svcs[i].hdl_start + g_sibles.svcs[i].hdl_num - 1);
            LOG_HEX("UUID", 16, (uint8_t *)g_sibles.svcs[i].svc_uuid, ATT_UUID_128_LEN);

            svc[i].uuid_len = ATT_UUID_128_LEN;
            memcpy(svc[i].uuid, (uint8_t *)g_sibles.svcs[i].svc_uuid, ATT_UUID_128_LEN);
            svc[i].start_handle = g_sibles.svcs[i].hdl_start;
            svc[i].end_handle = g_sibles.svcs[i].hdl_start + g_sibles.svcs[i].hdl_num - 1;
        }
    }
}


int sibles_set_value(uint8_t conn_idx, sibles_value_t *value)
{
    struct sibles_svc_env *svc = (struct sibles_svc_env *)(value->hdl);
    uint16_t msg = (value->len & SIBLE_CFM_FLAG) ? SIBLES_VALUE_REQ_CFM : SIBLES_SET_VALUE_REQ;
    sibles_send_value_t send_val;

    send_val.hdl = value->idx + svc->hdl_start;
    send_val.msg = msg;
    send_val.len = value->len & (~SIBLE_FLAG_ALL);
    send_val.value = value->value;

    //value->len &= ~SIBLE_FLAG_ALL;
    sibles_send_value(conn_idx, &send_val);

    if (msg == SIBLES_SET_VALUE_REQ)
    {
        svc->svc_status = SIBLES_BUSY;
        sifli_sem_take();
    }
    return send_val.len;
}

int sibles_write_value(uint8_t conn_idx, sibles_value_t *value)
{
#ifdef BSP_BLE_CONNECTION_MANAGER
    if (!connection_manager_check_normal_conn_idx(conn_idx))
    {
        LOG_I("unexpected conn idx %d", conn_idx);
        return -1;
    }
#endif

    struct sibles_svc_env *svc = (struct sibles_svc_env *) value->hdl;
    sibles_send_value_t send_val;

    send_val.hdl = value->idx + svc->hdl_start;
    send_val.msg = SIBLES_VALUE_NTF_IND;
    send_val.len = value->len;
    send_val.value = value->value;

    if (sibles_acquire_tx_pkts() == 0)
        return 0;
    sibles_send_value(conn_idx, &send_val);
    //svc->svc_status = SIBLES_BUSY;
    //sifli_sem_take();
    return send_val.len;
}

int sibles_write_value_with_rsp(uint8_t conn_idx, sibles_value_t *value)
{
#ifdef BSP_BLE_CONNECTION_MANAGER
    if (!connection_manager_check_normal_conn_idx(conn_idx))
    {
        LOG_I("unexpected conn idx %d", conn_idx);
        return -1;
    }
#endif

    struct sibles_svc_env *svc = (struct sibles_svc_env *) value->hdl;
    sibles_send_value_t send_val;

    send_val.hdl = value->idx + svc->hdl_start;
    send_val.msg = SIBLES_VALUE_IND_REQ;
    send_val.len = value->len;
    send_val.value = value->value;

    if (sibles_acquire_tx_pkts() == 0)
        return 0;
    sibles_send_value(conn_idx, &send_val);
    //svc->svc_status = SIBLES_BUSY;
    //sifli_sem_take();
    return send_val.len;

}

void sibles_stop_svc(sibles_hdl hdl)
{
}


void sibles_toggle_adv(void)
{

    sifli_task_id_t task_id = g_sibles.app_task_id;
    uint8_t *no_param = (uint8_t *)sifli_msg_alloc(SIBLES_ADV_CMD_REQ,
                        TASK_BUILD_ID(task_id, 0), sifli_get_stack_id(), 0);

    sifli_msg_send((void const *)no_param);
    //sifli_msg_free(no_param);
    sifli_sem_take();
}

void sibles_set_adv_data(int len, uint8_t *data)
{
    len &= ~SIBLE_FLAG_ALL;
    sibles_send_value_t send_val;

    send_val.hdl = 0;
    send_val.msg = SIBLES_ADV_DATA_REQ;
    send_val.len = len;
    send_val.value = data;

    sibles_send_value(0, &send_val);
    sifli_sem_take();
}

void sibles_set_scan_rsp_data(int len, uint8_t *data)
{
    len &= ~SIBLE_FLAG_ALL;

    sibles_send_value_t send_val;

    send_val.hdl = 0;
    send_val.msg = SIBLES_SCAN_RSP_REQ;
    send_val.len = len;
    send_val.value = data;

    sibles_send_value(0, &send_val);
    sifli_sem_take();
}

void sibles_set_dev_name(int len, uint8_t *name)
{
    len &= ~SIBLE_FLAG_ALL;
    sibles_send_value_t send_val;

    send_val.hdl = 0;
    send_val.msg = SIBLES_NAME_REQ;
    send_val.len = len;
    send_val.value = name;

    // 0 is for no-link.
    sibles_send_value(0, &send_val);
    sifli_sem_take();
}


/********************************************SIBLE GATT client API************************************************************/

#ifdef BLE_GATT_CLIENT

int8_t sibles_search_service(uint8_t conn_idx, uint8_t uuid_len, uint8_t *uuid)
{
    if (uuid == NULL || uuid_len > ATT_UUID_128_LEN)
        return (int8_t)PRF_ERR_INVALID_PARAM;

    // convert to 128
    sifli_task_id_t task_id = g_sibles.app_task_id;
    struct sibles_svc_search_req *req = (struct sibles_svc_search_req *)sifli_msg_alloc(SIBLES_SVC_SEARCH_REQ,
                                        TASK_BUILD_ID(task_id, conn_idx), TASK_BUILD_ID(sifli_get_stack_id(), conn_idx), sizeof(struct sibles_svc_search_req));
    req->conn_idx = conn_idx;
    req->len = uuid_len;
    memcpy(req->svc_uuid, uuid, uuid_len);
    sifli_msg_send((void const *)req);
    //sifli_msg_free(req);
    return 0;
}



uint16_t sibles_register_remote_svc(uint8_t conn_idx, uint16_t start_hdl, uint16_t end_hdl, sibles_remote_svc_cbk callback)
{
    sifli_task_id_t task_id = g_sibles.app_task_id;
    uint32_t i;
    for (i = 0; i < SIBLES_MAX_REMOTE_SVCS; i++)
        if (g_sibles.remote_svc[i].svc_status == SIBLES_EMPTY)
            break;
    if (i == SIBLES_MAX_REMOTE_SVCS)   // Could not register more
        return SIBLES_ERROR_REMOTE_HANDLE;

    g_sibles.remote_svc[i].svc_status = SIBLES_IDLE;
    g_sibles.remote_svc[i].hdl_start = start_hdl;
    g_sibles.remote_svc[i].hdl_stop = end_hdl;
    g_sibles.remote_svc[i].callback = callback;
    g_sibles.remote_svc[i].conn_idx = conn_idx;

    struct sibles_register_notify_req_t *req = (struct sibles_register_notify_req_t *)sifli_msg_alloc(SIBLES_REGISTER_NOTIFY_REQ,
            TASK_BUILD_ID(task_id, conn_idx), TASK_BUILD_ID(sifli_get_stack_id(), conn_idx), sizeof(struct sibles_register_notify_req_t));
    req->hdl_start = start_hdl;
    req->hdl_end = end_hdl;
    req->seq_num = i;
    sifli_msg_send((void const *)req);
    //sifli_msg_free(req);
    return i;
}

void sibles_unregister_remote_svc(uint8_t conn_idx, uint16_t start_hdl, uint16_t end_hdl, sibles_remote_svc_cbk callback)
{
    sifli_task_id_t task_id = g_sibles.app_task_id;

    uint32_t i;
    for (i = 0; i < SIBLES_MAX_REMOTE_SVCS; i++)
        if (g_sibles.remote_svc[i].svc_status > SIBLES_EMPTY &&
                g_sibles.remote_svc[i].hdl_start == start_hdl &&
                g_sibles.remote_svc[i].hdl_stop == end_hdl &&
                g_sibles.remote_svc[i].callback == callback &&
                g_sibles.remote_svc[i].conn_idx == conn_idx)
            break;
    if (i == SIBLES_MAX_REMOTE_SVCS)   // Could not find
        return;

    memset(&g_sibles.remote_svc[i], 0, sizeof(struct sibles_remote_svc_env));

    struct sibles_register_notify_req_t *req = (struct sibles_register_notify_req_t *)sifli_msg_alloc(SIBLES_UNREGISTER_NOTIFY_REQ,
            TASK_BUILD_ID(task_id, conn_idx), TASK_BUILD_ID(sifli_get_stack_id(), conn_idx), sizeof(struct sibles_register_notify_req_t));
    req->hdl_start = start_hdl;
    req->hdl_end = end_hdl;
    req->seq_num = i;
    sifli_msg_send((void const *)req);
}

void sibles_send_remote_svc_change_ind(uint8_t conn_idx, uint16_t start_hdl, uint16_t end_hdl)
{
    sibles_remote_svc_change_ind_t ind;
    ind.conn_idx = conn_idx;
    ind.start_handle = start_hdl;
    ind.end_handle = end_hdl;
    ble_event_publish(SIBLES_REMOTE_SVC_CHANGE_IND, &ind, sizeof(sibles_remote_svc_change_ind_t));
}

int8_t  sibles_tx_malloc_buff(uint16_t remote_handle, uint8_t conn_idx, sibles_write_remote_value_t *value)
{
    struct sibles_rte_wr_info  *node;
    struct sibles_value_write_req_content_t *req;
    int8_t ret = OTHER_ERR;
    //the size of struct + the length of value[__ARRAY_EMPTY];
    req = bt_mem_alloc(sizeof(struct sibles_value_write_req_content_t) + value->len);
    if (req)
    {
        req->conn_idx = conn_idx;
        req->write_type = value->write_type;
        req->seq_num = remote_handle;
        req->handle = value->handle;
        req->length =  value->len;
        memcpy(req->value, value->value, value->len);
        node = bt_mem_alloc(sizeof(struct sibles_rte_wr_info));
        BT_OOM_ASSERT(node);
        memset(node, 0, sizeof(struct sibles_rte_wr_info));

        node->value = req;

        rt_enter_critical();
        rt_slist_append(&g_sibles.wr_node, (rt_slist_t *)node);
        rt_exit_critical();


        LOG_D("add node addr %x\n", node);
        ret = SIBLES_WRITE_NO_ERR;
    }
    else
        ret = SIBLES_OUT_OF_MEMORY;

    return ret;
}

int8_t sibles_write_remote_value(uint16_t remote_handle, uint8_t conn_idx, sibles_write_remote_value_t *value)
{
    sifli_task_id_t task_id = g_sibles.app_task_id;
    uint8_t acq_tx;
    uint8_t res = SIBLES_WRITE_NO_ERR;
    uint8_t buffer_num = sibles_get_slist_len(&g_sibles.wr_node);

    if (remote_handle == SIBLES_ERROR_REMOTE_HANDLE)
        return SIBLES_WRITE_HANDLE_ERR;

    acq_tx = sibles_acquire_tx_pkts();
    LOG_I("send:acq_tx, buffer tx_num t_pkt %d %d %d\n", acq_tx, buffer_num, g_sibles.num_of_tx_pkt);
    if ((0 == acq_tx) && (20 == buffer_num))
    {
        return SIBLES_WIRTE_TX_FLOWCTRL_ERR;
    }
    if (0 == acq_tx)
    {
        res = sibles_tx_malloc_buff(remote_handle, conn_idx, value);
        return res;
    }

    struct sibles_value_write_req_t *req = (struct sibles_value_write_req_t *)sifli_msg_alloc(SIBLES_VALUE_WRITE_REQ,
                                           TASK_BUILD_ID(task_id, conn_idx), TASK_BUILD_ID(sifli_get_stack_id(), conn_idx), sizeof(struct sibles_value_write_req_t) + value->len);
    req->write_type = value->write_type;
    req->seq_num = remote_handle;
    req->handle = value->handle;
    req->length =  value->len;
    memcpy(req->value, value->value, value->len);

    sifli_msg_send((void const *)req);
    //sifli_msg_free(req);
    return 0;
}

int8_t sibles_read_remote_value(uint16_t remote_handle, uint8_t conn_idx, sibles_read_remote_value_req_t *value)
{
    sifli_task_id_t task_id = g_sibles.app_task_id;
    struct sibles_value_read_req_t *req = (struct sibles_value_read_req_t *)sifli_msg_alloc(SIBLES_VALUE_READ_REQ,
                                          TASK_BUILD_ID(task_id, conn_idx), TASK_BUILD_ID(sifli_get_stack_id(), conn_idx), sizeof(struct sibles_value_read_req_t));
    if (remote_handle == SIBLES_ERROR_REMOTE_HANDLE)
        return -1;
    req->read_type = value->read_type;
    req->seq_num = remote_handle;
    req->handle = value->handle;
    req->offset = value->offset;
    req->length =  value->length;
    sifli_msg_send((void const *)req);
    //sifli_msg_free(req);
    return 0;
}
#endif //BLE_GATT_CLIENT

uint8_t sibles_exchange_mtu(uint8_t conn_idx)
{
    struct gattc_exc_mtu_cmd *cmd = sifli_msg_alloc(GATTC_EXC_MTU_CMD,
                                    TASK_BUILD_ID(TASK_ID_GATTC, conn_idx),
                                    sifli_get_stack_id(),
                                    sizeof(struct gattc_exc_mtu_cmd));
    cmd->operation = GATTC_MTU_EXCH;
    cmd->seq_num = 0;
    sifli_msg_send((void const *)cmd);

    return HL_ERR_NO_ERROR;
}

#ifdef BLE_SVC_CHG_ENABLE
void sibles_send_svc_changed_ind(uint8_t conn_idx, uint16_t start_handle, uint16_t end_handle)
{
    struct gattc_send_svc_changed_cmd *cmd = sifli_msg_alloc(GATTC_SEND_SVC_CHANGED_CMD,
            TASK_BUILD_ID(TASK_ID_GATTC, conn_idx),
            sifli_get_stack_id(),
            sizeof(struct gattc_send_svc_changed_cmd));
    cmd->operation = GATTC_SVC_CHANGED;
    cmd->seq_num = 0;
    cmd->svc_shdl = start_handle;
    cmd->svc_ehdl = end_handle;
    sifli_msg_send((void const *)cmd);
}
#endif //BLE_SVC_CHG_ENABLE

void sibles_set_random_addr(uint8_t conn_idx, uint8_t *value)
{
    sifli_task_id_t task_id = g_sibles.app_task_id;

    struct sibles_random_addr *val = (struct sibles_random_addr *)value;
    struct sibles_random_addr *req = (struct sibles_random_addr *)sifli_msg_alloc(SIBLES_SET_STATIC_RANDOM_ADDR_REQ,
                                     TASK_BUILD_ID(task_id, conn_idx), sifli_get_stack_id(), sizeof(struct sibles_random_addr));

    req->addr_type = val->addr_type;
    memcpy(req->addr, val->addr, BD_ADDR_LEN);
    sifli_msg_send((void const *)req);
}

#if defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X) || defined(BSP_USING_PC_SIMULATOR)
void ble_gap_wlan_coex_enable(void)
{
    uint8_t *cmd = sifli_msg_alloc(SIBLES_WLAN_COEX_ENABLE_REQ,
                                   TASK_BUILD_ID(TASK_ID_SIBLES, 0),
                                   sifli_get_stack_id(),
                                   sizeof(uint8_t));
    sifli_msg_send((void const *)cmd);
}

void sibles_set_trc_cfg(sibles_trc_cfg_t cfg_mode, uint32_t mask_ext)
{
#if defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X)
    uint8_t index = 0;
#else
    uint8_t index = 1;
#endif

    if (cfg_mode >= SIBLES_TRC_LAST)
        return;
    uint32_t mask[SIBLES_TRC_LAST] = {0, TRC_HCI_MASK, TRC_ALL_MASK};
    struct sibles_trc_cfg_req_t *cmd  = sifli_msg_alloc(SIBLES_TRC_CFG_REQ,
                                        TASK_BUILD_ID(TASK_ID_SIBLES, index),
                                        sifli_get_stack_id(),
                                        sizeof(struct sibles_trc_cfg_req_t));

    if (cfg_mode <= SIBLES_TRC_ALL_ON)
        cmd->mask = mask[cfg_mode];
    else if (cfg_mode == SIBLES_TRC_CUSTOMIZE)
        cmd->mask = mask_ext;

    sifli_msg_send((void const *)cmd);
}

void sibles_dbg_config(uint32_t para)
{
#if defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X)
    uint8_t index = 0;
#else
    uint8_t index = 1;
#endif
    uint32_t *cmd  = sifli_msg_alloc(SIBLES_ENABLE_DBG,
                                     TASK_BUILD_ID(TASK_ID_SIBLES, index),
                                     sifli_get_stack_id(),
                                     sizeof(uint32_t));

    *cmd = para;
    sifli_msg_send((void const *)cmd);
}
enum dbg_patch_type
{
    RSSI_DBG_TYPE,
    BT_ACL_3M_TYPE,
    BT_DIS_DBG_TYPE,
    BLE_DIS_DBG_TYPE,
    MAX_DBG_PATCH_TYPE,
};

int btdm_dbg(int argc, char *argv[])
{
    uint8_t cmdpara[4] = {0, 0, 0, 0};

    if (argc > 1)
    {
        if (strcmp(argv[1], "rssi") == 0)
        {
            cmdpara[0] = RSSI_DBG_TYPE;
            cmdpara[1] = strtol(argv[2], NULL, 10);
        }
        else if (strcmp(argv[1], "bt3m") == 0)
        {
            cmdpara[0] = BT_ACL_3M_TYPE;
            cmdpara[1] = strtol(argv[2], NULL, 10);
        }
        else if (strcmp(argv[1], "btdis") == 0)
        {
            cmdpara[0] = BT_DIS_DBG_TYPE;
            cmdpara[1] = strtol(argv[2], NULL, 10);
            cmdpara[2] = strtol(argv[3], NULL, 10);
        }
        else if (strcmp(argv[1], "bledis") == 0)
        {
            cmdpara[0] = BLE_DIS_DBG_TYPE;
            cmdpara[1] = strtol(argv[2], NULL, 10);
            cmdpara[2] = strtol(argv[3], NULL, 10);
        }
    }

    LOG_I("btdm_dbg config 0x%x\n", *(uint32_t *)cmdpara);
    sibles_dbg_config(*(uint32_t *)cmdpara);


    return 0;
}

MSH_CMD_EXPORT(btdm_dbg, btdm debug config);

#if 0
void sibles_edr3M_acl_disable(uint8_t enable)
{
#if defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X)
    uint8_t index = 0;
#else
    uint8_t index = 1;
#endif
    uint8_t *cmd  = sifli_msg_alloc(SIBLES_EDR3M_DISABLE,
                                    TASK_BUILD_ID(TASK_ID_SIBLES, index),
                                    sifli_get_stack_id(),
                                    sizeof(uint8_t));

    *cmd = enable;
    sifli_msg_send((void const *)cmd);
}

int edr3m_enable(int argc, char *argv[])
{
    uint8_t enable;

    if (argc != 2)
    {
        LOG_E("arg para num error\n");
        return -1;
    }
    enable = strtol(argv[1], NULL, 10);

    sibles_edr3M_acl_disable(enable);


    return 0;
}

MSH_CMD_EXPORT(edr3m_enable, edr 3M acl enable);
#endif

uint8_t sibles_change_bd_addr(sibles_change_bd_addr_type_t type, sibles_change_bd_addr_method_t method, bd_addr_t *addr)
{
#if defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X)
    uint8_t index = 0;
#else
    uint8_t index = 1;
#endif

    if (type > SIBLES_CH_BD_TYPE_TOTAL || method > SIBLES_CH_BD_METHOD_TOTAL)
        return PRF_ERR_INVALID_PARAM;

    if (method == SIBLES_CH_BD_METHOD_CUSTOMIZE && addr == NULL)
        return PRF_ERR_INVALID_PARAM;

    struct sibles_ch_bd_addr_t *cmd  = sifli_msg_alloc(SIBLES_CH_BD_ADDR,
                                       TASK_BUILD_ID(TASK_ID_SIBLES, index),
                                       sifli_get_stack_id(),
                                       sizeof(struct sibles_ch_bd_addr_t));

    cmd->addr_type = type;
    cmd->addr_method = method;
    if (method == SIBLES_CH_BD_METHOD_CUSTOMIZE)
        memcpy(&cmd->addr, &addr->addr, BD_ADDR_LEN);

    sifli_msg_send((void const *)cmd);
    return HL_ERR_NO_ERROR;
}

#endif


int ble_bf0_sibles_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    switch (event_id)
    {
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
#ifdef BLE_GATT_CLIENT
        sibles_clear_wr_list(ind->conn_idx);
#endif //BLE_GATT_CLIENT
        break;
    }
    default:
        break;
    }
    return 0;
}

void sibles_set_dev_info(sibles_set_dis_t *param)
{
    sifli_task_id_t task_id = g_sibles.app_task_id;
    app_dis_set_req_t *cmd  = sifli_msg_alloc(DISS_APP_SET_VALUE_REQ,
                              TASK_BUILD_ID(TASK_ID_APP, 0),
                              TASK_BUILD_ID(sifli_get_stack_id(), 0),
                              (sizeof(app_dis_set_req_t) + param->len));
    cmd->value = param->value;
    cmd->len = param->len;
    memcpy(cmd->data, param->data, param->len);
    sifli_msg_send((void const *)cmd);
}

BLE_EVENT_REGISTER(ble_bf0_sibles_event_handler, NULL);

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
