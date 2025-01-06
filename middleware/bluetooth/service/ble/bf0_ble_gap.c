/**
  ******************************************************************************
  * @file   bf0_ble_gap.c
  * @author Sifli software development team
  * @brief Header file - Bluetooth GAP protocol interface implement.
 *
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


#include <string.h>
#include <stdlib.h>
#include "rtthread.h"
#include "rthw.h"
#include "os_adaptor.h"
#include "bf0_ble_gap.h"
#include "bf0_ble_gap_internal.h"
#include "bf0_ble_common.h"
#include "ble_connection_manager.h"

#define LOG_TAG "BLE_GAP"
#include "log.h"

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
#define BLE_GAP_EMPTY_CHECK(p) \
    if (p == NULL ) \
        return GAP_ERR_INVALID_PARAM;

enum ble_gap_actvity_state_t
{
    BLE_GAP_ACTV_IDLE = 0,
    /// Activity is being created - next state is CREATED
    BLE_GAP_ACTV_CREATING,
    /// Activity has been successfully created
    BLE_GAP_ACTV_CREATED,
    /// Activity is being started - next state is STARTED
    BLE_GAP_ACTV_STARTING,
    /// Activity has been successfully started
    BLE_GAP_ACTV_STARTED,
    /// Activity is being stopped - next state is CREATED
    BLE_GAP_ACTV_STOPPING,
    /// Activity is being deleted - no next state
    BLE_GAP_ACTV_DELETING,
};

typedef struct
{
    sifli_msg_id_t id;
    sifli_task_id_t task;
    uint16_t len;
    uint8_t param[__ARRAY_EMPTY];
} ble_gap_command_t;


typedef struct
{
    uint8_t *next_node;
    uint8_t active_index;
    uint8_t actv_type;
} ble_gap_activty_node_t;

typedef struct
{
    ble_gap_activty_node_t *first_node;
} ble_gap_activty_list_t;

typedef struct
{
    uint8_t own_addr_type;
    uint8_t act_index;
    uint8_t state;
    void   *param;
} ble_gap_actvity_info_t;

typedef struct
{
    ble_gap_actvity_info_t scan_info;
    ble_gap_actvity_info_t init_info;
    ble_gap_actvity_info_t per_sync_info;
} ble_gap_activty_int_t;

ble_gap_activty_list_t g_ble_gap_activty_list;

ble_gap_activty_int_t g_ble_gap_activity_info;

static char g_ble_gap_dev_name[GAP_MAX_LOCAL_NAME_LEN] = GAP_DEFAULT_LOCAL_NAME;
static uint8_t g_ble_gap_dev_name_len;

static uint16_t g_ble_gap_apperance;
#ifndef BSP_USING_PC_SIMULATOR
    __USED
#endif
static uint8_t g_ble_param_update_flag = 1;     //0:non-renewable 1:renewable
#ifndef BSP_USING_PC_SIMULATOR
    __USED
#endif
static ble_gap_local_version_ind_t g_ble_ver;
/*
 * Internal Functions
 ****************************************************************************************
 */



// enum gapm_actv_type
// stack will handle activity one by one, no need considerate opeartion.
static void ble_gap_push_activity(uint8_t actv_type, uint8_t actv_index)
{
    ble_gap_activty_node_t *new_node = (ble_gap_activty_node_t *)bt_mem_alloc(sizeof(ble_gap_activty_node_t));
    BT_OOM_ASSERT(new_node);
    if (new_node)
    {
        new_node->active_index = actv_index;
        new_node->actv_type = actv_type;
        new_node->next_node = NULL;
        uint32_t mask = rt_hw_interrupt_disable();
        ble_gap_activty_node_t *node = g_ble_gap_activty_list.first_node;
        if (node == NULL)
        {
            g_ble_gap_activty_list.first_node = new_node;
        }
        else
        {
            while (node != NULL)
            {
                if (node->next_node == NULL)
                {
                    node->next_node = (uint8_t *)new_node;
                    break;
                }
                node = (ble_gap_activty_node_t *)node->next_node;
            }
        }
        rt_hw_interrupt_enable(mask);
    }
}

// enum gapm_actv_type
static int8_t ble_gap_pop_activity(uint8_t *actv_index, uint8_t *actv_type)
{
    int8_t ret = -1;
    uint32_t mask = rt_hw_interrupt_disable();
    ble_gap_activty_node_t *node = g_ble_gap_activty_list.first_node;
    if (node != NULL)
    {
        g_ble_gap_activty_list.first_node = (ble_gap_activty_node_t *)node->next_node;
        rt_hw_interrupt_enable(mask);
        *actv_index = node->active_index;
        *actv_type = node->actv_type;
        bt_mem_free(node);
        ret = 0;
    }
    else
    {
        rt_hw_interrupt_enable(mask);
    }
    return ret;
}

static ble_gap_activty_int_t *ble_gap_get_activty_internal_info(void)
{
    return &g_ble_gap_activity_info;
}

#ifdef BLE_GAP_CENTRAL
static uint8_t ble_gap_delete_scan(ble_gap_activty_int_t *info)
{
    if (info->scan_info.state <= BLE_GAP_ACTV_CREATING)
        return GAP_ERR_COMMAND_DISALLOWED;

    struct gapm_activity_delete_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_DELETE_CMD,
                                           TASK_ID_GAPM, sifli_get_stack_id(),
                                           sizeof(struct gapm_activity_delete_cmd));
    cmd->operation = GAPM_DELETE_ACTIVITY;
    cmd->actv_idx = info->scan_info.act_index;
    // Ignore delete cnf
    info->scan_info.state = BLE_GAP_ACTV_IDLE;
    ble_gap_push_activity(GAPM_ACTV_TYPE_SCAN, cmd->actv_idx);

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;
}


static uint8_t ble_gap_create_connection_internal(ble_gap_connection_create_param_t *conn_param, ble_gap_activty_int_t
        *info)
{
    BLE_GAP_EMPTY_CHECK(conn_param);
    struct gapm_activity_start_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_START_CMD,
                                          TASK_ID_GAPM, sifli_get_stack_id(),
                                          sizeof(struct gapm_activity_start_cmd));
    cmd->operation = GAPM_START_ACTIVITY;
    cmd->actv_idx = info->init_info.act_index;
    cmd->u_param.init_param.type = conn_param->type;
    cmd->u_param.init_param.prop = GAPM_INIT_PROP_1M_BIT;
    cmd->u_param.init_param.conn_to = conn_param->conn_to;
    cmd->u_param.init_param.scan_param_1m.scan_intv = conn_param->conn_param_1m.scan_intv;
    cmd->u_param.init_param.scan_param_1m.scan_wd = conn_param->conn_param_1m.scan_wd;
    cmd->u_param.init_param.conn_param_1m.conn_intv_max = conn_param->conn_param_1m.conn_intv_max;
    cmd->u_param.init_param.conn_param_1m.conn_intv_min = conn_param->conn_param_1m.conn_intv_min;
    cmd->u_param.init_param.conn_param_1m.conn_latency = conn_param->conn_param_1m.conn_latency;
    cmd->u_param.init_param.conn_param_1m.supervision_to = conn_param->conn_param_1m.supervision_to;
    cmd->u_param.init_param.conn_param_1m.ce_len_max = conn_param->conn_param_1m.ce_len_max;
    cmd->u_param.init_param.conn_param_1m.ce_len_min = conn_param->conn_param_1m.ce_len_min;

    memcpy(&cmd->u_param.init_param.peer_addr, &conn_param->peer_addr, sizeof(ble_gap_addr_t));

    ble_gap_push_activity(GAPM_ACTV_TYPE_INIT, cmd->actv_idx);

    info->init_info.state = BLE_GAP_ACTV_STARTING;

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;

}
#endif //BLE_GAP_CENTRAL
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

// This API is to send undefined commands in protocol API but supported by stack.
uint8_t ble_gap_send_command(ble_gap_command_t *cmd)
{
    BLE_GAP_EMPTY_CHECK(cmd);

    uint8_t *para = sifli_msg_alloc(cmd->id, cmd->task, sifli_get_stack_id(), cmd->len);
    // The command may forward to BT task then to stack.
    memcpy(para, cmd, cmd->len);
    sifli_msg_send((void const *)para);
    //sifli_msg_free((void *)para);
    return HL_ERR_NO_ERROR;
}


//
uint8_t ble_gap_set_dev_name(ble_gap_dev_name_t *dev)
{
    BLE_GAP_EMPTY_CHECK(dev);
    uint16_t name_len = dev->len <= GAP_MAX_LOCAL_NAME_LEN ? dev->len : GAP_MAX_LOCAL_NAME_LEN;
    struct gapc_set_dev_info_req_ind *dev_name = sifli_msg_alloc(GAPC_SET_DEV_INFO_REQ_IND,
            TASK_ID_APP, sifli_get_stack_id(),
            sizeof(struct gapc_set_dev_info_req_ind) + name_len);

    dev_name->req = GAPC_DEV_NAME;

    dev_name->info.name.len = name_len;
    memcpy(dev_name->info.name.name, dev->name, dev_name->info.name.len);
    // Copy for central usage
    memcpy(g_ble_gap_dev_name, dev->name, dev_name->info.name.len);
    g_ble_gap_dev_name_len = name_len;

    sifli_msg_send((void const *)dev_name);
    //sifli_msg_free((void *)dev_name);
    return HL_ERR_NO_ERROR;
}



uint8_t ble_gap_set_appearance(uint16_t appearance)
{

    struct gapc_set_dev_info_req_ind *param = sifli_msg_alloc(GAPC_SET_DEV_INFO_REQ_IND,
            TASK_ID_APP, sifli_get_stack_id(),
            sizeof(struct gapc_set_dev_info_req_ind));

    param->req = GAPC_DEV_APPEARANCE;

    param->info.appearance = appearance;
    // Copy for central usage.
    g_ble_gap_apperance = appearance;

    sifli_msg_send((void const *)param);
    //sifli_msg_free((void *)param);
    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_set_slave_prefer_param(ble_gap_slave_prf_param_t *param)
{
    BLE_GAP_EMPTY_CHECK(param);
    struct gapc_set_dev_info_req_ind *slv_param = sifli_msg_alloc(GAPC_SET_DEV_INFO_REQ_IND,
            TASK_ID_APP, sifli_get_stack_id(),
            sizeof(struct gapc_set_dev_info_req_ind));

    slv_param->req = GAPC_DEV_SLV_PREF_PARAMS;

    slv_param->info.slv_pref_params.con_intv_min = param->con_intv_min;
    slv_param->info.slv_pref_params.con_intv_max = param->con_intv_max;
    slv_param->info.slv_pref_params.conn_timeout = param->conn_timeout;
    slv_param->info.slv_pref_params.slave_latency = param->slave_latency;

    sifli_msg_send((void const *)slv_param);
    //sifli_msg_free((void *)slv_param);
    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_configure_public_addr(bd_addr_t *addr)
{
    // configure to flash.
    return -1;
}

static uint8_t ble_gap_get_local_dev_info(uint8_t operation)
{
    struct gapm_get_dev_info_cmd *para = sifli_msg_alloc(GAPM_GET_DEV_INFO_CMD,
                                         TASK_ID_GAPM, sifli_get_stack_id(),
                                         sizeof(struct gapm_get_dev_info_cmd));
    para->operation = operation;
    sifli_msg_send((void const *)para);
    //sifli_msg_free((void *)para);
    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_get_local_version(void)
{
    return ble_gap_get_local_dev_info(GAPM_GET_DEV_VERSION);
}

uint8_t ble_gap_get_local_bdaddr(void)
{
    return ble_gap_get_local_dev_info(GAPM_GET_DEV_BDADDR);
}

uint8_t ble_gap_get_local_adv_tx_power(void)
{
    return ble_gap_get_local_dev_info(GAPM_GET_DEV_ADV_TX_POWER);
}

uint8_t ble_gap_get_local_max_data_len(void)
{
    return ble_gap_get_local_dev_info(GAPM_GET_MAX_LE_DATA_LEN);

}

uint8_t ble_gap_get_number_of_adv_sets(void)

{
    return ble_gap_get_local_dev_info(GAPM_GET_NB_ADV_SETS);
}

uint8_t ble_gap_get_local_max_adv_data_len(void)
{
    return ble_gap_get_local_dev_info(GAPM_GET_MAX_LE_ADV_DATA_LEN);
}

uint8_t ble_gap_set_white_list(ble_gap_white_list_t *white_list)
{
    BLE_GAP_EMPTY_CHECK(white_list);
    uint16_t len = sizeof(struct gapm_list_set_wl_cmd) + white_list->size * sizeof(ble_gap_addr_t);
    struct gapm_list_set_wl_cmd *cmd = sifli_msg_alloc(GAPM_LIST_SET_CMD,
                                       TASK_ID_GAPM, sifli_get_stack_id(),
                                       len);
    cmd->operation = GAPM_SET_WL;
    cmd->size = white_list->size;
    memcpy(&cmd->wl_info, white_list->addr, white_list->size * sizeof(ble_gap_addr_t));

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);
    return HL_ERR_NO_ERROR;
}

uint8_t ble_gap_set_resolving_list(ble_gap_resolving_list_t *rl_list)
{
    BLE_GAP_EMPTY_CHECK(rl_list);
    uint16_t len = sizeof(struct gapm_list_set_ral_cmd) + rl_list->size * sizeof(ble_gap_ral_dev_info_t);
    struct gapm_list_set_ral_cmd *cmd = sifli_msg_alloc(GAPM_LIST_SET_CMD,
                                        TASK_ID_GAPM, sifli_get_stack_id(),
                                        len);
    cmd->operation = GAPM_SET_RAL;
    cmd->size = rl_list->size;
    memcpy(&cmd->ral_info, rl_list->ral, rl_list->size * sizeof(ble_gap_ral_dev_info_t));

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);
    return HL_ERR_NO_ERROR;
}

uint8_t ble_gap_get_local_ral_addr(ble_gap_addr_t *addr)
{
    BLE_GAP_EMPTY_CHECK(addr);
    struct gapm_get_ral_addr_cmd *para = sifli_msg_alloc(GAPM_GET_RAL_ADDR_CMD,
                                         TASK_ID_GAPM, sifli_get_stack_id(),
                                         sizeof(struct gapm_get_ral_addr_cmd));
    para->operation = GAPM_GET_RAL_LOC_ADDR;

    memcpy(&para->peer_identity, addr, sizeof(ble_gap_addr_t));

    sifli_msg_send((void const *)para);
    //sifli_msg_free((void *)para);
    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_generate_rand_addr(uint8_t rand_type)
{
    if (rand_type != GAP_STATIC_ADDR &&
            rand_type != GAP_NON_RSLV_ADDR &&
            rand_type != GAP_RSLV_ADDR)
    {
        return GAP_ERR_INVALID_PARAM;
    }

    struct gapm_gen_rand_addr_cmd *addr = sifli_msg_alloc(GAPM_GEN_RAND_ADDR_CMD,
                                          TASK_ID_GAPM, sifli_get_stack_id(),
                                          sizeof(struct gapm_gen_rand_addr_cmd));
    addr->operation = GAPM_GEN_RAND_ADDR;
    addr->rnd_type = rand_type;

    sifli_msg_send((void const *)addr);
    //sifli_msg_free((void *)addr);
    return HL_ERR_NO_ERROR;
}

uint8_t ble_gap_set_irk(ble_gap_sec_key_t *new_irk)
{
    BLE_GAP_EMPTY_CHECK(new_irk);
    struct gapm_set_irk_cmd *cmd = sifli_msg_alloc(GAPM_SET_IRK_CMD,
                                   TASK_ID_GAPM, sifli_get_stack_id(),
                                   sizeof(struct gapm_set_irk_cmd));

    cmd->operation = GAPM_SET_IRK;
    memcpy(cmd->irk.key, new_irk->key, GAP_KEY_LEN);

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);
    return HL_ERR_NO_ERROR;
}


uint8_t ble_gap_create_advertising(ble_gap_adv_parameter_t *para)
{
    BLE_GAP_EMPTY_CHECK(para);
    struct gapm_activity_create_adv_cmd *adv = sifli_msg_alloc(GAPM_ACTIVITY_CREATE_CMD,
            TASK_ID_GAPM, sifli_get_stack_id(),
            sizeof(struct gapm_activity_create_adv_cmd));

    adv->operation = GAPM_CREATE_ADV_ACTIVITY;
    adv->own_addr_type = para->own_addr_type;
    adv->adv_param.type = para->type;
    adv->adv_param.disc_mode = para->disc_mode;
    adv->adv_param.prop = para->prop;
    adv->adv_param.max_tx_pwr = para->max_tx_pwr;
    adv->adv_param.filter_pol = para->filter_pol;
    memcpy((void *)&adv->adv_param.peer_addr, (void *)&para->peer_addr, sizeof(ble_gap_addr_t));
    memcpy((void *)&adv->adv_param.prim_cfg, (void *)&para->prim_cfg, sizeof(gapm_adv_prim_cfg_t));
    memcpy((void *)&adv->adv_param.second_cfg, (void *)&para->second_cfg, sizeof(gapm_adv_second_cfg_t));

    if (para->type == GAPM_ADV_TYPE_PERIODIC)
    {
        memcpy((void *)&adv->adv_param.period_cfg, (void *)&para->period_cfg, sizeof(gapm_adv_period_cfg_t));
    }

    sifli_msg_send((void const *)adv);
    //sifli_msg_free((void *)adv);

    return HL_ERR_NO_ERROR;
}

uint8_t ble_gap_set_scan_rsp_data(ble_gap_adv_data_t *data)
{
    BLE_GAP_EMPTY_CHECK(data);
    uint16_t len = sizeof(struct gapm_set_adv_data_cmd) + data->length;
    struct gapm_set_adv_data_cmd *adv_data = sifli_msg_alloc(GAPM_SET_ADV_DATA_CMD,
            TASK_ID_GAPM, sifli_get_stack_id(),
            len);
    adv_data->operation = GAPM_SET_SCAN_RSP_DATA;
    adv_data->actv_idx = data->actv_idx;
    adv_data->length = data->length;
    memcpy(adv_data->data, data->data, data->length);

    sifli_msg_send((void const *)adv_data);
    //sifli_msg_free((void *)adv_data);

    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_set_adv_data(ble_gap_adv_data_t *data)
{
    BLE_GAP_EMPTY_CHECK(data);
    uint16_t len = sizeof(struct gapm_set_adv_data_cmd) + data->length;
    struct gapm_set_adv_data_cmd *adv_data = sifli_msg_alloc(GAPM_SET_ADV_DATA_CMD,
            TASK_ID_GAPM, sifli_get_stack_id(),
            len);
    adv_data->operation = GAPM_SET_ADV_DATA;
    adv_data->actv_idx = data->actv_idx;
    adv_data->length = data->length;
    memcpy(adv_data->data, data->data, data->length);

    sifli_msg_send((void const *)adv_data);
    //sifli_msg_free((void *)adv_data);

    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_set_periodic_adv_data(ble_gap_adv_data_t *data)
{
    BLE_GAP_EMPTY_CHECK(data);
    uint16_t len = sizeof(struct gapm_set_adv_data_cmd) + data->length;
    struct gapm_set_adv_data_cmd *adv_data = sifli_msg_alloc(GAPM_SET_ADV_DATA_CMD,
            TASK_ID_GAPM, sifli_get_stack_id(),
            len);
    adv_data->operation = GAPM_SET_PERIOD_ADV_DATA;
    adv_data->actv_idx = data->actv_idx;
    adv_data->length = data->length;
    memcpy(adv_data->data, data->data, data->length);

    sifli_msg_send((void const *)adv_data);
    //sifli_msg_free((void *)adv_data);

    return HL_ERR_NO_ERROR;

}



uint8_t ble_gap_start_advertising(ble_gap_adv_start_t *adv_start)
{
    BLE_GAP_EMPTY_CHECK(adv_start);
    struct gapm_activity_start_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_START_CMD,
                                          TASK_ID_GAPM, sifli_get_stack_id(),
                                          sizeof(struct gapm_activity_start_cmd));
    cmd->operation = GAPM_START_ACTIVITY;
    cmd->actv_idx = adv_start->actv_idx;
    cmd->u_param.adv_add_param.duration = adv_start->duration;
    cmd->u_param.adv_add_param.max_adv_evt = adv_start->max_adv_evt;
    ble_gap_push_activity(GAPM_ACTV_TYPE_ADV, cmd->actv_idx);

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_stop_advertising(ble_gap_adv_stop_t *adv_stop)
{
    BLE_GAP_EMPTY_CHECK(adv_stop);
    struct gapm_activity_stop_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_STOP_CMD,
                                         TASK_ID_GAPM, sifli_get_stack_id(),
                                         sizeof(struct gapm_activity_stop_cmd));
    cmd->operation = GAPM_STOP_ACTIVITY;
    cmd->actv_idx = adv_stop->actv_idx;
    ble_gap_push_activity(GAPM_ACTV_TYPE_ADV, cmd->actv_idx);

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;


}

uint8_t ble_gap_delete_advertising(ble_gap_adv_delete_t *del)
{
    BLE_GAP_EMPTY_CHECK(del);
    struct gapm_activity_delete_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_DELETE_CMD,
                                           TASK_ID_GAPM, sifli_get_stack_id(),
                                           sizeof(struct gapm_activity_delete_cmd));
    cmd->operation = GAPM_DELETE_ACTIVITY;
    cmd->actv_idx = del->actv_idx;
    ble_gap_push_activity(GAPM_ACTV_TYPE_ADV, cmd->actv_idx);

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;
}

#ifdef BLE_GAP_CENTRAL
uint8_t ble_gap_scan_start(ble_gap_scan_start_t *scan_param)

{
    BLE_GAP_EMPTY_CHECK(scan_param);
    uint8_t ret = GAP_ERR_COMMAND_DISALLOWED;
    ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();
    // Stop scan will delete activity, so scan start only will create activity.
    // Scan will failed when received scan stopped ind and start scan immediately.
    if (act_info->scan_info.state == BLE_GAP_ACTV_IDLE)
    {
        ble_gap_scan_start_t *ptr = bt_mem_alloc(sizeof(ble_gap_scan_start_t));
        if (ptr)
        {
            memcpy((void *)ptr, (const void *)scan_param, sizeof(ble_gap_scan_start_t));
            act_info->scan_info.own_addr_type = scan_param->own_addr_type;
            act_info->scan_info.param = ptr;
            act_info->scan_info.state = BLE_GAP_ACTV_CREATING;

            struct gapm_activity_create_adv_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_CREATE_CMD,
                    TASK_ID_GAPM, sifli_get_stack_id(),
                    sizeof(struct gapm_activity_create_adv_cmd));
            cmd->operation = GAPM_CREATE_SCAN_ACTIVITY;
            cmd->own_addr_type = scan_param->own_addr_type;
            sifli_msg_send((void const *)cmd);
            ret = HL_ERR_NO_ERROR;
        }
        else
            ret = GAP_ERR_INSUFF_RESOURCES;
    }

    return ret;
}


uint8_t ble_gap_scan_stop(void)
{
    uint8_t ret = GAP_ERR_COMMAND_DISALLOWED;
    ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();
    if (act_info->scan_info.state != BLE_GAP_ACTV_IDLE &&
            act_info->scan_info.state != BLE_GAP_ACTV_CREATING)
    {
        struct gapm_activity_stop_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_STOP_CMD,
                                             TASK_ID_GAPM, sifli_get_stack_id(),
                                             sizeof(struct gapm_activity_stop_cmd));
        cmd->operation = GAPM_STOP_ACTIVITY;
        cmd->actv_idx = act_info->scan_info.act_index;

        act_info->scan_info.state = BLE_GAP_ACTV_STOPPING;

        ble_gap_push_activity(GAPM_ACTV_TYPE_SCAN, cmd->actv_idx);

        sifli_msg_send((void const *)cmd);
        ret = HL_ERR_NO_ERROR;
    }
    //sifli_msg_free((void *)cmd);
    return ret;
}


// For ADV set is not enough and need act as central role.
uint8_t ble_gap_delete_init(void)
{
    uint8_t ret = GAP_ERR_COMMAND_DISALLOWED;
    ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();

    if (act_info->init_info.state <= BLE_GAP_ACTV_CREATING)
    {
        struct gapm_activity_delete_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_DELETE_CMD,
                                               TASK_ID_GAPM, sifli_get_stack_id(),
                                               sizeof(struct gapm_activity_delete_cmd));
        cmd->operation = GAPM_DELETE_ACTIVITY;
        cmd->actv_idx = act_info->init_info.act_index;

        act_info->init_info.state = BLE_GAP_ACTV_DELETING;

        ble_gap_push_activity(GAPM_ACTV_TYPE_INIT, cmd->actv_idx);

        sifli_msg_send((void const *)cmd);
        ret = HL_ERR_NO_ERROR;
    }

    return ret;

}

uint8_t ble_gap_create_connection(ble_gap_connection_create_param_t *conn_param)
{
    BLE_GAP_EMPTY_CHECK(conn_param);
    uint8_t ret = GAP_ERR_COMMAND_DISALLOWED;
    ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();
    if (act_info->init_info.state == BLE_GAP_ACTV_IDLE)
    {
        ble_gap_connection_create_param_t *ptr = bt_mem_alloc(sizeof(ble_gap_connection_create_param_t));
        if (ptr)
        {
            memcpy((void *)ptr, (const void *)conn_param, sizeof(ble_gap_connection_create_param_t));
            act_info->init_info.own_addr_type = conn_param->own_addr_type;
            act_info->init_info.param = ptr;
            act_info->init_info.state = BLE_GAP_ACTV_CREATING;

            struct gapm_activity_create_adv_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_CREATE_CMD,
                    TASK_ID_GAPM, sifli_get_stack_id(),
                    sizeof(struct gapm_activity_create_adv_cmd));
            cmd->operation = GAPM_CREATE_INIT_ACTIVITY;
            cmd->own_addr_type = act_info->init_info.own_addr_type;
            sifli_msg_send((void const *)cmd);
            ret = HL_ERR_NO_ERROR;
        }
        else
            ret = GAP_ERR_INSUFF_RESOURCES;
    }
    else if (act_info->init_info.state != BLE_GAP_ACTV_CREATING && // Stack will handle other states.
             act_info->init_info.state != BLE_GAP_ACTV_DELETING)
    {
        if (conn_param->own_addr_type <= GAPM_GEN_NON_RSLV_ADDR &&
                conn_param->own_addr_type != act_info->init_info.own_addr_type)
        {
            // Should be re-create init activity due to valid but different own address type
            ble_gap_connection_create_param_t *ptr = bt_mem_alloc(sizeof(ble_gap_connection_create_param_t));
            if (ptr)
            {
                memcpy((void *)ptr, (const void *)conn_param, sizeof(ble_gap_connection_create_param_t));
                act_info->init_info.own_addr_type = conn_param->own_addr_type;
                act_info->init_info.param = ptr;
                ret = ble_gap_delete_init();
            }
            else
                ret = GAP_ERR_INSUFF_RESOURCES;
        }
        else
        {
            ret = ble_gap_create_connection_internal(conn_param, act_info);
        }
    }
    return ret;
}


uint8_t ble_gap_create_periodic_advertising_sync(void)
{
    uint8_t ret = HL_ERR_NO_ERROR;

    struct gapm_activity_create_adv_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_CREATE_CMD,
            TASK_ID_GAPM, sifli_get_stack_id(),
            sizeof(struct gapm_activity_create_adv_cmd));
    cmd->operation = GAPM_CREATE_PERIOD_SYNC_ACTIVITY;
    sifli_msg_send((void const *)cmd);
    return ret;
}


uint8_t ble_gap_start_periodic_advertising_sync(ble_gap_periodic_advertising_sync_start_t *sync_param)
{
    BLE_GAP_EMPTY_CHECK(sync_param);
    uint8_t ret = HL_ERR_NO_ERROR;

    struct gapm_activity_start_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_START_CMD,
                                          TASK_ID_GAPM, sifli_get_stack_id(),
                                          sizeof(struct gapm_activity_start_cmd));
    cmd->operation = GAPM_START_ACTIVITY;
    cmd->actv_idx = sync_param->actv_idx;
    cmd->u_param.per_sync_param.type = sync_param->type;
    cmd->u_param.per_sync_param.skip = sync_param->skip;
    cmd->u_param.per_sync_param.sync_to = sync_param->sync_to;
    cmd->u_param.per_sync_param.adv_addr.addr = sync_param->addr;
    cmd->u_param.per_sync_param.adv_addr.adv_sid = sync_param->adv_sid;
    ble_gap_push_activity(GAPM_ACTV_TYPE_PER_SYNC, cmd->actv_idx);

    sifli_msg_send((void const *)cmd);
    return ret;
}

uint8_t ble_gap_stop_periodic_advertising_sync(ble_gap_eriodic_advertising_sync_stop_t *sync_stop)
{
    BLE_GAP_EMPTY_CHECK(sync_stop);
    struct gapm_activity_stop_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_STOP_CMD,
                                         TASK_ID_GAPM, sifli_get_stack_id(),
                                         sizeof(struct gapm_activity_stop_cmd));
    cmd->operation = GAPM_STOP_ACTIVITY;
    cmd->actv_idx = sync_stop->actv_idx;
    ble_gap_push_activity(GAPM_ACTV_TYPE_PER_SYNC, cmd->actv_idx);

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;
}

uint8_t ble_gap_delete_periodic_advertising_sync(ble_gap_eriodic_advertising_sync_delete_t *del)
{
    BLE_GAP_EMPTY_CHECK(del);
    struct gapm_activity_delete_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_DELETE_CMD,
                                           TASK_ID_GAPM, sifli_get_stack_id(),
                                           sizeof(struct gapm_activity_delete_cmd));
    cmd->operation = GAPM_DELETE_ACTIVITY;
    cmd->actv_idx = del->actv_idx;
    ble_gap_push_activity(GAPM_ACTV_TYPE_PER_SYNC, cmd->actv_idx);

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;
}


#endif // BLE_GAP_CENTRAL

uint8_t ble_gap_connect_response(ble_gap_connection_response_t *rsp)
{
    struct gapc_connection_cfm *cfm = sifli_msg_alloc(GAPC_CONNECTION_CFM,
                                      TASK_BUILD_ID(TASK_ID_GAPC, rsp->conn_idx), sifli_get_stack_id(),
                                      sizeof(struct gapc_connection_cfm));
    cfm->auth = rsp->auth;
    cfm->ltk_present = rsp->ltk_present;
    memcpy((void *)cfm->lcsrk.key, (void *)rsp->lcsrk.key, GAP_KEY_LEN);
    memcpy(cfm->rcsrk.key, rsp->rcsrk.key, GAP_KEY_LEN);
    cfm->lsign_counter = rsp->lsign_counter;
    cfm->rsign_counter = rsp->rsign_counter;
    sifli_msg_send((void const *)cfm);
    return HL_ERR_NO_ERROR;
}

#ifdef BLE_GAP_CENTRAL
uint8_t ble_gap_cancel_create_connection(void)
{
    uint8_t ret = GAP_ERR_COMMAND_DISALLOWED;
    ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();

    if (act_info->init_info.state == BLE_GAP_ACTV_STARTED)
    {
        struct gapm_activity_stop_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_STOP_CMD,
                                             TASK_ID_GAPM, sifli_get_stack_id(),
                                             sizeof(struct gapm_activity_stop_cmd));
        cmd->operation = GAPM_STOP_ACTIVITY;
        cmd->actv_idx = act_info->init_info.act_index;

        act_info->init_info.state = BLE_GAP_ACTV_STOPPING;

        ble_gap_push_activity(GAPM_ACTV_TYPE_INIT, cmd->actv_idx);

        sifli_msg_send((void const *)cmd);
        ret = HL_ERR_NO_ERROR;
    }

    return ret;
}
#endif //BLE_GAP_CENTRAL



uint8_t ble_gap_disconnect(ble_gap_disconnect_t *conn)
{
    BLE_GAP_EMPTY_CHECK(conn);
    struct gapc_disconnect_cmd *cmd = sifli_msg_alloc(GAPC_DISCONNECT_CMD,
                                      TASK_BUILD_ID(TASK_ID_GAPC, conn->conn_idx),
                                      sifli_get_stack_id(),
                                      sizeof(struct gapc_disconnect_cmd));
    cmd->operation = GAPC_DISCONNECT;
    cmd->reason = conn->reason;

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;
}


static uint8_t ble_gap_get_remote_info(uint8_t conn_idx, uint8_t operation)
{
    struct gapc_get_info_cmd *cmd = sifli_msg_alloc(GAPC_GET_INFO_CMD,
                                    TASK_BUILD_ID(TASK_ID_GAPC, conn_idx),
                                    sifli_get_stack_id(),
                                    sizeof(struct gapc_get_info_cmd));
    cmd->operation = operation;

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;

}


// remote info
uint8_t ble_gap_get_remote_version(ble_gap_get_remote_version_t *ver)
{
    return ble_gap_get_remote_info(ver->conn_idx, GAPC_GET_PEER_VERSION);
}

uint8_t ble_gap_get_remote_feature(bt_gap_get_remote_feature_t *feature)
{
    return ble_gap_get_remote_info(feature->conn_idx, GAPC_GET_PEER_FEATURES);
}

uint8_t ble_gap_get_remote_rssi(ble_gap_get_rssi_t *rssi)
{
    return ble_gap_get_remote_info(rssi->conn_idx, GAPC_GET_CON_RSSI);
}

uint8_t ble_gap_get_remote_physical(ble_gap_get_phy_t *phy)
{
    return ble_gap_get_remote_info(phy->conn_idx, GAPC_GET_PHY);
}

uint8_t ble_gap_update_conn_param(ble_gap_update_conn_param_t *conn_para)
{
    BLE_GAP_EMPTY_CHECK(conn_para);

    if (!g_ble_param_update_flag)
    {
        return LL_ERR_CONTROLLER_BUSY;
    }
    g_ble_param_update_flag = 0;
    struct gapc_param_update_cmd *cmd = sifli_msg_alloc(GAPC_PARAM_UPDATE_CMD,
                                        TASK_BUILD_ID(TASK_ID_GAPC, conn_para->conn_idx),
                                        sifli_get_stack_id(),
                                        sizeof(struct gapc_param_update_cmd));
    cmd->operation = GAPC_UPDATE_PARAMS;
    cmd->intv_min = conn_para->intv_min;
    cmd->intv_max = conn_para->intv_max;
    cmd->latency = conn_para->latency;
    cmd->time_out = conn_para->time_out;
    cmd->ce_len_min = conn_para->ce_len_min;
    cmd->ce_len_max = conn_para->ce_len_max;

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_update_conn_param_on_l2cap(ble_gap_update_conn_param_t *conn_para)
{
    BLE_GAP_EMPTY_CHECK(conn_para);

    struct gapc_param_update_cmd *cmd = sifli_msg_alloc(GAPC_PARAM_UPDATE_L2CAP_CMD,
                                        TASK_BUILD_ID(TASK_ID_GAPC, conn_para->conn_idx),
                                        sifli_get_stack_id(),
                                        sizeof(struct gapc_param_update_cmd));
    cmd->operation = GAPC_UPDATE_PARAMS;
    cmd->intv_min = conn_para->intv_min;
    cmd->intv_max = conn_para->intv_max;
    cmd->latency = conn_para->latency;
    cmd->time_out = conn_para->time_out;
    cmd->ce_len_min = conn_para->ce_len_min;
    cmd->ce_len_max = conn_para->ce_len_max;

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;
}

uint8_t ble_gap_lepsm_register(void)
{
    struct gapm_lepsm_register_cmd *cmd = sifli_msg_alloc(GAPM_LEPSM_REGISTER_CMD,
                                          TASK_ID_GAPM,
                                          sifli_get_stack_id(),
                                          sizeof(struct gapm_lepsm_register_cmd));
    cmd->operation = GAPM_LEPSM_REG;
    cmd->le_psm = 0x27;
    cmd->app_task = sifli_get_stack_id();
    cmd->sec_lvl = 0x00;
    sifli_msg_send((void const *)cmd);
    LOG_D("ble_gap_lepsm_register");
    return HL_ERR_NO_ERROR;
}



uint8_t ble_gap_resolve_address(ble_gap_resolve_address_t *req)
{
    if (req->nb_key == 0)
        return GAP_ERR_INVALID_PARAM;

    struct gapm_resolv_addr_cmd *cmd = sifli_msg_alloc(GAPM_RESOLV_ADDR_CMD,
                                       TASK_ID_GAPM,
                                       sifli_get_stack_id(),
                                       sizeof(struct gapm_resolv_addr_cmd) + sizeof(struct gap_sec_key) * req->nb_key);
    cmd->operation = GAPM_RESOLV_ADDR;
    cmd->nb_key = req->nb_key;
    memcpy(&cmd->addr, &req->addr, sizeof(bd_addr_t));
    for (uint32_t i = 0; i < req->nb_key; i++)
    {
        memcpy((void *)&cmd->irk[i], (void *)&req->irk[i], sizeof(struct gap_sec_key));
    }
    sifli_msg_send((void const *)cmd);

    return HL_ERR_NO_ERROR;
}


uint8_t ble_gap_bond(ble_gap_bond_t *bond)
{
    BLE_GAP_EMPTY_CHECK(bond);
    struct gapc_bond_cmd *cmd = sifli_msg_alloc(GAPC_BOND_CMD,
                                TASK_BUILD_ID(TASK_ID_GAPC, bond->conn_idx),
                                sifli_get_stack_id(),
                                sizeof(struct gapc_bond_cmd));
    cmd->operation = GAPC_BOND;
    cmd->pairing.iocap = bond->pair_info.iocap;
    cmd->pairing.oob = bond->pair_info.oob;
    cmd->pairing.auth = bond->pair_info.auth;
    cmd->pairing.key_size = bond->pair_info.key_size;
    cmd->pairing.ikey_dist = bond->pair_info.ikey_dist;
    cmd->pairing.rkey_dist = bond->pair_info.rkey_dist;
    cmd->pairing.sec_req = bond->pair_info.sec_req;

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_bond_confirm(ble_gap_bond_confirm_t *cfm)
{
    BLE_GAP_EMPTY_CHECK(cfm);

    struct gapc_bond_cfm *cmd = sifli_msg_alloc(GAPC_BOND_CFM,
                                TASK_BUILD_ID(TASK_ID_GAPC, cfm->conn_idx),
                                sifli_get_stack_id(),
                                sizeof(struct gapc_bond_cfm));
    cmd->request = cfm->request;
    cmd->accept = cfm->accept;
    memcpy(&cmd->data, &cfm->cfm_data, sizeof(ble_gap_bond_cfm_data_t));

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;
}

uint8_t ble_gap_encrypt_confirm(ble_gap_encrypt_confirm_t *cfm)
{
    BLE_GAP_EMPTY_CHECK(cfm);

    struct gapc_encrypt_cfm *cmd = sifli_msg_alloc(GAPC_ENCRYPT_CFM,
                                   TASK_BUILD_ID(TASK_ID_GAPC, cfm->conn_idx),
                                   sifli_get_stack_id(),
                                   sizeof(struct gapc_encrypt_cfm));
    cmd->found = cfm->found;
    if (cmd->found)
    {
        memcpy(&cmd->ltk, &cfm->ltk, sizeof(ble_gap_sec_key_t));
        cmd->key_size = cfm->key_size;
    }

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_security_request(ble_gap_sec_req_t *req)
{
    BLE_GAP_EMPTY_CHECK(req);

    struct gapc_security_cmd *cmd = sifli_msg_alloc(GAPC_SECURITY_CMD,
                                    TASK_BUILD_ID(TASK_ID_GAPC, req->conn_idx),
                                    sifli_get_stack_id(),
                                    sizeof(struct gapc_security_cmd));

    cmd->operation = GAPC_SECURITY_REQ;
    cmd->auth = req->auth;

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_update_data_len(ble_gap_update_data_len_t *req)
{
    struct gapc_set_le_pkt_size_cmd *cmd = sifli_msg_alloc(GAPC_SET_LE_PKT_SIZE_CMD,
                                           TASK_BUILD_ID(TASK_ID_GAPC, req->conn_idx),
                                           sifli_get_stack_id(),
                                           sizeof(struct gapc_set_le_pkt_size_cmd));
    cmd->operation = GAPC_SET_LE_PKT_SIZE;
    cmd->tx_octets = req->tx_octets;
    cmd->tx_time = req->tx_time;
    sifli_msg_send((void const *)cmd);

    return HL_ERR_NO_ERROR;
}


static uint8_t ble_gap_param_update_response(ble_gap_update_conn_response_t *rsp)
{
    struct gapc_param_update_cfm *cfm = sifli_msg_alloc(GAPC_PARAM_UPDATE_CFM,
                                        TASK_BUILD_ID(TASK_ID_GAPC, rsp->conn_idx),
                                        sifli_get_stack_id(),
                                        sizeof(struct gapc_param_update_cfm));
    cfm->accept = rsp->accept;
    cfm->ce_len_min = rsp->ce_len_min;
    cfm->ce_len_max = rsp->ce_len_max;

    sifli_msg_send((void const *)cfm);
    return HL_ERR_NO_ERROR;
}



#if 0
uint8_t ble_gap_set_passkey(ble_gap_set_passkey_t *key)
{
// To be check
}
#endif

uint8_t ble_gap_update_phy(ble_gap_update_phy_t *phy)
{
    BLE_GAP_EMPTY_CHECK(phy);

    struct gapc_set_phy_cmd *cmd = sifli_msg_alloc(GAPC_SET_PHY_CMD,
                                   TASK_BUILD_ID(TASK_ID_GAPC, phy->conn_idx),
                                   sifli_get_stack_id(),
                                   sizeof(struct gapc_set_phy_cmd));
    cmd->operation = GAPC_SET_PHY;
    cmd->tx_phy = phy->tx_phy;
    cmd->rx_phy = phy->rx_phy;
    cmd->phy_opt = phy->phy_opt;

    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;

}

uint8_t ble_gap_update_channel_map(ble_gap_update_channel_map_t *map)
{
    BLE_GAP_EMPTY_CHECK(map);

    struct gapm_set_channel_map_cmd *cmd = sifli_msg_alloc(GAPM_SET_CHANNEL_MAP_CMD,
                                           TASK_ID_GAPM,
                                           sifli_get_stack_id(),
                                           sizeof(struct gapm_set_channel_map_cmd));

    cmd->operation = GAPM_SET_CHANNEL_MAP;
    memcpy(cmd->chmap.map, map->channel_map, GAP_LE_CHNL_MAP_LEN);
    sifli_msg_send((void const *)cmd);
    //sifli_msg_free((void *)cmd);

    return HL_ERR_NO_ERROR;
}

uint8_t ble_gap_aes_h6(uint8_t *w, uint8_t *key_id, uint32_t cb_request)
{
    BLE_GAP_EMPTY_CHECK(w);
    BLE_GAP_EMPTY_CHECK(key_id);
    struct gapm_aes_h6_cmd *cmd = sifli_msg_alloc(GAPM_AES_H6_CMD,
                                  TASK_ID_GAPM,
                                  sifli_get_stack_id(),
                                  sizeof(struct gapm_aes_h6_cmd));
    memcpy(cmd->w, w, GAP_KEY_LEN);
    memcpy(cmd->key_id, key_id, KEY_ID_LEN);
    cmd->cb_request = cb_request;
    sifli_msg_send((void const *)cmd);
    return HL_ERR_NO_ERROR;
}
uint8_t ble_gap_aes_h7(uint8_t *salt, uint8_t *w, uint32_t metainfo)
{
    return HL_ERR_NO_ERROR;
}

void ble_gap_event_process(sibles_msg_para_t *header, uint8_t *data_ptr, uint16_t param_len)
{
    sifli_msg_id_t msg_id = header->id;
    uint8_t conn_idx = TASK_IDX_GET(header->src_id);

    switch (msg_id)
    {
    case GAPM_DEV_VERSION_IND:
    {
        ble_gap_local_version_ind_t *evt = (ble_gap_local_version_ind_t *)data_ptr;
        memcpy(&g_ble_ver, evt, sizeof(ble_gap_local_version_ind_t));
        ble_event_publish(BLE_GAP_LOCAL_VER_IND, evt, sizeof(ble_gap_local_version_ind_t));
        break;
    }
    case GAPM_DEV_BDADDR_IND:
    {
        ble_gap_dev_bdaddr_ind_t *evt = (ble_gap_dev_bdaddr_ind_t *)data_ptr;
        ble_event_publish(BLE_GAP_LOCAL_BD_ADDR_IND, evt, sizeof(ble_gap_dev_bdaddr_ind_t));
        break;
    }
    case GAPM_DEV_ADV_TX_POWER_IND:
    {
        ble_gap_dev_adv_tx_power_ind_t *evt = (ble_gap_dev_adv_tx_power_ind_t *)data_ptr;
        ble_event_publish(BLE_GAP_LOCAL_ADV_TX_POWER_IND, evt, sizeof(ble_gap_dev_adv_tx_power_ind_t));
        break;
    }
    case GAPM_MAX_DATA_LEN_IND:
    {
        ble_gap_max_data_len_ind_t *evt = (ble_gap_max_data_len_ind_t *)data_ptr;
        ble_event_publish(BLE_GAP_LOCAL_MAX_DATA_LEN_IND, evt, sizeof(ble_gap_max_data_len_ind_t));
        break;
    }
    case GAPM_NB_ADV_SETS_IND:
    {
        ble_gap_nb_adv_sets_ind_t *evt = (ble_gap_nb_adv_sets_ind_t *)data_ptr;
        ble_event_publish(BLE_GAP_NUMBER_ADV_SETS_IND, evt, sizeof(ble_gap_nb_adv_sets_ind_t));
        break;
    }
    case GAPM_MAX_ADV_DATA_LEN_IND:
    {
        ble_gap_max_adv_data_len_ind_t *evt = (ble_gap_max_adv_data_len_ind_t *)data_ptr;
        ble_event_publish(BLE_GAP_LOCAL_MAX_ADV_DATA_LEN_IND, evt, sizeof(ble_gap_max_adv_data_len_ind_t));
        break;
    }
    case GAPM_ACTIVITY_CREATED_IND:
    {
        struct gapm_activity_created_ind *ind = (struct gapm_activity_created_ind *)data_ptr;
        ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();
        if (ind->actv_type == GAPM_ACTV_TYPE_ADV)
        {
            ble_gap_adv_created_ind_t evt;
            evt.actv_idx = ind->actv_idx;
            evt.tx_pwr = ind->tx_pwr;
            ble_event_publish(BLE_GAP_ADV_CREATED_IND, &evt, sizeof(ble_gap_adv_created_ind_t));
        }
#ifdef BLE_GAP_CENTRAL
        else if (ind->actv_type == GAPM_ACTV_TYPE_INIT)
        {
            act_info->init_info.state = BLE_GAP_ACTV_CREATED;
            act_info->init_info.act_index = ind->actv_idx;
            ble_gap_connection_create_param_t *init_param = (ble_gap_connection_create_param_t *)act_info->init_info.param;
            RT_ASSERT(init_param);
            ble_gap_create_connection_internal(init_param, act_info);
            bt_mem_free(init_param);
            act_info->init_info.param = NULL;
        }
        else if (ind->actv_type == GAPM_ACTV_TYPE_SCAN)
        {
            act_info->scan_info.state = BLE_GAP_ACTV_CREATED;
            act_info->scan_info.act_index = ind->actv_idx;
            ble_gap_scan_start_t *scan_param = (ble_gap_scan_start_t *)act_info->scan_info.param;
            RT_ASSERT(scan_param);

            struct gapm_activity_start_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_START_CMD,
                                                  TASK_ID_GAPM, sifli_get_stack_id(),
                                                  sizeof(struct gapm_activity_start_cmd));
            cmd->operation = GAPM_START_ACTIVITY;
            cmd->actv_idx = act_info->scan_info.act_index;
            cmd->u_param.scan_param.type = scan_param->type;
            //cmd->u_param.scan_param.prop = GAPM_SCAN_PROP_ACTIVE_1M_BIT | GAPM_SCAN_PROP_PHY_1M_BIT;
            cmd->u_param.scan_param.prop = GAPM_SCAN_PROP_PHY_1M_BIT | GAPM_SCAN_PROP_ACTIVE_1M_BIT;
            cmd->u_param.scan_param.dup_filt_pol = scan_param->dup_filt_pol;
            cmd->u_param.scan_param.scan_param_1m.scan_intv = scan_param->scan_param_1m.scan_intv;
            cmd->u_param.scan_param.scan_param_1m.scan_wd = scan_param->scan_param_1m.scan_wd;
            cmd->u_param.scan_param.duration = scan_param->duration;
            cmd->u_param.scan_param.period = scan_param->period;

            act_info->scan_info.state = BLE_GAP_ACTV_STARTING;

            ble_gap_push_activity(GAPM_ACTV_TYPE_SCAN, cmd->actv_idx);

            sifli_msg_send((void const *)cmd);

            bt_mem_free(scan_param);
            act_info->scan_info.param = NULL;
        }
        else if (ind->actv_type == GAPM_ACTV_TYPE_PER_SYNC)
        {
            ble_gap_per_adv_sync_created_ind_t evt;
            evt.actv_idx = ind->actv_idx;
            ble_event_publish(BLE_GAP_PERIODIC_ADV_SYNC_CREATED_IND, &evt, sizeof(ble_gap_per_adv_sync_created_ind_t));
        }
#endif //BLE_GAP_CENTRAL
        break;
    }
    case GAPM_ACTIVITY_STOPPED_IND:
    {
        struct gapm_activity_stopped_ind *ind = (struct gapm_activity_stopped_ind *)data_ptr;
        ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();
        if (ind->actv_type == GAPM_ACTV_TYPE_ADV)
        {
            ble_gap_adv_stopped_ind_t evt;
            evt.actv_idx = ind->actv_idx;
            evt.reason = ind->reason;
            evt.per_adv_stop = ind->per_adv_stop;
            ble_event_publish(BLE_GAP_ADV_STOPPED_IND, &evt, sizeof(ble_gap_adv_stopped_ind_t));
        }
#ifdef BLE_GAP_CENTRAL
        else if (ind->actv_type == GAPM_ACTV_TYPE_SCAN)
        {
            ble_gap_scan_stopped_ind_t evt;
            if (act_info->scan_info.state != BLE_GAP_ACTV_STOPPING) // STOPPING will delete in STOP CNF
                ble_gap_delete_scan(act_info);
            evt.reason = ind->reason;
            ble_event_publish(BLE_GAP_SCAN_STOPPED_IND, &evt, sizeof(ble_gap_scan_stopped_ind_t));
        }
        else if (ind->actv_type == GAPM_ACTV_TYPE_INIT)
        {
            // Except start stop cmd, others are all happened when init action completed. No need to notify upper layer
            act_info->init_info.state = BLE_GAP_ACTV_STARTED;
        }
        else if (ind->actv_type == GAPM_ACTV_TYPE_PER_SYNC)
        {
            ble_gap_per_adv_sync_stopped_ind_t evt;
            evt.actv_idx = ind->actv_idx;
            evt.reason = ind->reason;
            ble_event_publish(BLE_GAP_PERIODIC_ADV_SYNC_STOPPED_IND, &evt, sizeof(ble_gap_per_adv_sync_stopped_ind_t));
        }
#endif //BLE_GAP_CENTRAL
        break;
    }
    case GAPM_SYNC_ESTABLISHED_IND:
    {
        struct gapm_sync_established_ind *ind = (struct gapm_sync_established_ind *)data_ptr;
        ble_gap_per_adv_sync_established_t evt;
        evt.actv_idx = ind->actv_idx;
        evt.addr = ind->addr;
        evt.adv_sid = ind->adv_sid;
        evt.intv = ind->intv;
        evt.phy = ind->phy;
        evt.clk_acc = ind->clk_acc;
        ble_event_publish(BLE_GAP_PERIODIC_ADV_SYNC_ESTABLISHED_IND, &evt, sizeof(ble_gap_per_adv_sync_established_t));
        break;
    }
    case GAPM_DBG_RSSI_NOTIFY_IND:
    {
        bt_dbg_rssi_notify_ind_t *evt = (bt_dbg_rssi_notify_ind_t *)data_ptr;
        ble_event_publish(BT_DBG_RSSI_NOTIFY_IND, evt, sizeof(bt_dbg_rssi_notify_ind_t));
        break;
    }
    case GAPM_ASSERT_IND:
    {
#ifdef SOC_SF32LB56X
        ble_gap_assert_ind_t *evt = (ble_gap_assert_ind_t *)data_ptr;
        ble_event_publish(BT_DBG_ASSERT_NOTIFY_IND, evt, sizeof(ble_gap_assert_ind_t));
        if (evt->type == 1)
        {
            LOG_E("reset BT stack");
            bt_system_reset();
        }
        else
        {
            LOG_W("Stack happen warning error");
        }
#endif // SOC_SF32LB56X
        break;
    }
    case GAPM_ADDR_SOLVED_IND:
    {
        LOG_D("solved ind");
        ble_gap_solved_addr_ind_t *ind = (ble_gap_solved_addr_ind_t *)data_ptr;
        ble_event_publish(BLE_GAP_SOLVED_ADDRESS_IND, ind, sizeof(ble_gap_solved_addr_ind_t));
        break;
    }
#ifndef SOC_SF32LB55X
    case GAPM_PUB_KEY_IND:
    {
        LOG_D("pub genrate");
        ble_event_publish(BLE_GAP_PUBLIC_KEY_GEN_IND, NULL, 0);
        break;
    }
#endif // !SOC_SF32LB55X
    case GAPM_CMP_EVT:
    {
        struct gapm_cmp_evt *evt = (struct gapm_cmp_evt *)data_ptr;
        if (GAPM_SET_PERIOD_ADV_DATA != evt->operation)
            LOG_D("evt %d, ret %d\r\n", evt->operation, evt->status);

        switch (evt->operation)
        {
        case GAPM_SET_WL:
        {
            ble_gap_set_white_list_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_GAP_SET_WHITE_LIST_CNF, &ret, sizeof(ble_gap_set_white_list_cnf_t));
            break;
        }
        case GAPM_SET_RAL:
        {
            ble_gap_set_resolve_list_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_GAP_SET_RESOLVING_LIST_CNF, &ret, sizeof(ble_gap_set_resolve_list_cnf_t));
            break;
        }
        case GAPM_SET_IRK:
        {
            ble_gap_set_irk_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_GAP_SET_IRK_CNF, &ret, sizeof(ble_gap_set_irk_cnf_t));
            break;
        }
        case GAPM_CREATE_ADV_ACTIVITY:
        {
            ble_gap_create_adv_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_GAP_CREATE_ADV_CNF, &ret, sizeof(ble_gap_create_adv_cnf_t));
            break;
        }
        case GAPM_SET_ADV_DATA:
        {
            ble_gap_set_adv_data_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_GAP_SET_ADV_DATA_CNF, &ret, sizeof(ble_gap_set_adv_data_cnf_t));
            break;
        }
        case GAPM_SET_SCAN_RSP_DATA:
        {
            ble_gap_set_scan_rsp_data_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_GAP_SET_SCAN_RSP_DATA_CNF, &ret, sizeof(ble_gap_set_scan_rsp_data_cnf_t));
            break;
        }
        case GAPM_SET_PERIOD_ADV_DATA:
        {
            ble_gap_set_periodic_adv_data_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_GAP_SET_PERIODIC_ADV_DATA_CNF, &ret, sizeof(ble_gap_set_periodic_adv_data_cnf_t));
            break;
        }
        case GAPM_CREATE_PERIOD_SYNC_ACTIVITY:
        {
            ble_gap_create_per_adv_sync_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_GAP_CREATE_PERIODIC_ADV_SYNC_CNF, &ret, sizeof(ble_gap_create_per_adv_sync_cnf_t));
            break;
        }
        case GAPM_START_ACTIVITY:
        {
            uint8_t actv_index, actv_type;
            int8_t ret = ble_gap_pop_activity(&actv_index, &actv_type);
            ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();
            RT_ASSERT(ret == 0);
            switch (actv_type)
            {
            case GAPM_ACTV_TYPE_ADV:
            {
                ble_gap_start_adv_cnf_t ret;
                ret.actv_index = actv_index;
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_START_ADV_CNF, &ret, sizeof(ble_gap_start_adv_cnf_t));
                break;
            }
#ifdef BLE_GAP_CENTRAL
            case GAPM_ACTV_TYPE_SCAN:
            {
                ble_gap_start_scan_cnf_t ret;
                act_info->scan_info.state = BLE_GAP_ACTV_STARTED;
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_SCAN_START_CNF, &ret, sizeof(ble_gap_start_scan_cnf_t));
                break;
            }
            case GAPM_ACTV_TYPE_INIT:
            {
                ble_gap_create_connection_cnf_t ret;
                act_info->init_info.state = BLE_GAP_ACTV_STARTED;
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_CREATE_CONNECTION_CNF, &ret, sizeof(ble_gap_create_connection_cnf_t));
                break;
            }
            case GAPM_ACTV_TYPE_PER_SYNC:
            {
                ble_gap_start_per_adv_sync_cnf_t ret;
                ret.actv_index = actv_index;
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_START_PERIODIC_ADV_SYNC_CNF, &ret, sizeof(ble_gap_start_per_adv_sync_cnf_t));
                break;
            }
#endif //BLE_GAP_CENTRAL
            default:
                break;
            }
            break;
        }
        case GAPM_STOP_ACTIVITY:
        {
            uint8_t actv_index, actv_type;
            int8_t ret = ble_gap_pop_activity(&actv_index, &actv_type);
            ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();
            RT_ASSERT(ret == 0);
            switch (actv_type)
            {
            case GAPM_ACTV_TYPE_ADV:
            {
                ble_gap_stop_adv_cnf_t ret;
                ret.actv_index = actv_index;
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_STOP_ADV_CNF, &ret, sizeof(ble_gap_stop_adv_cnf_t));
                break;
            }
#ifdef BLE_GAP_CENTRAL
            case GAPM_ACTV_TYPE_SCAN:
            {
                ble_gap_stop_scan_cnf_t ret;
                // Delete scan activity.
                ble_gap_delete_scan(act_info);
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_SCAN_STOP_CNF, &ret, sizeof(ble_gap_stop_scan_cnf_t));
                break;
            }
            case GAPM_ACTV_TYPE_INIT:
            {
                ble_gap_cancel_create_connection_cnf_t ret;
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_CANCEL_CREATE_CONNECTION_CNF, &ret, sizeof(ble_gap_cancel_create_connection_cnf_t));
                break;
            }
            case GAPM_ACTV_TYPE_PER_SYNC:
            {
                ble_gap_stop_per_adv_sync_cnf_t ret;
                ret.actv_index = actv_index;
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_STOP_PERIODIC_ADV_SYNC_CNF, &ret, sizeof(ble_gap_stop_per_adv_sync_cnf_t));
                break;
            }
#endif //BLE_GAP_CENTRAL
            default:
                break;
            }
            break;
        }
        case GAPM_DELETE_ACTIVITY:
        {
            uint8_t actv_index, actv_type;
            int8_t ret = ble_gap_pop_activity(&actv_index, &actv_type);
            ble_gap_activty_int_t *act_info =  ble_gap_get_activty_internal_info();
            RT_ASSERT(ret == 0);
            switch (actv_type)
            {
            case GAPM_ACTV_TYPE_ADV:
            {
                ble_gap_delete_adv_cnf_t ret;
                ret.actv_index = actv_index;
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_DELETE_ADV_CNF, &ret, sizeof(ble_gap_delete_adv_cnf_t));
                break;
            }
#ifdef BLE_GAP_CENTRAL
            case GAPM_ACTV_TYPE_INIT:
            {
                if (act_info->init_info.state == BLE_GAP_ACTV_DELETING &&
                        act_info->init_info.param != NULL)
                {
                    // delete initied by GAP self, should continue create init act.
                    act_info->init_info.state = BLE_GAP_ACTV_CREATING;

                    struct gapm_activity_create_adv_cmd *cmd = sifli_msg_alloc(GAPM_ACTIVITY_CREATE_CMD,
                            TASK_ID_GAPM, sifli_get_stack_id(),
                            sizeof(struct gapm_activity_create_adv_cmd));
                    cmd->operation = GAPM_CREATE_INIT_ACTIVITY;
                    cmd->own_addr_type = act_info->init_info.own_addr_type;
                    sifli_msg_send((void const *)cmd);
                }
                else
                {
                    // Do nothing.
                    act_info->init_info.state = BLE_GAP_ACTV_IDLE;
                }
                break;
            }
            case GAPM_ACTV_TYPE_PER_SYNC:
            {
                ble_gap_delete_per_adv_sync_cnf_t ret;
                ret.actv_index = actv_index;
                ret.status = evt->status;
                ble_event_publish(BLE_GAP_DELETE_PERIODIC_ADV_SYNC_CNF, &ret, sizeof(ble_gap_delete_per_adv_sync_cnf_t));
                break;
            }
#endif //BLE_GAP_CENTRAL
            default:
            {
                //Others do nothing.
                break;
            }
            }
            break;
        }
        case GAPM_LE_TEST_RX_START:
        {
            ble_test_rx_start_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_DUT_RX_START_CNF, &ret, sizeof(ble_test_rx_start_cnf_t));
            break;
        }
        case GAPM_LE_TEST_TX_START:
        {
            ble_test_tx_start_cnf_t ret;
            ret.status = evt->status;
            ble_event_publish(BLE_DUT_TX_START_CNF, &ret, sizeof(ble_test_tx_start_cnf_t));
            break;
        }
        case GAPM_RESOLV_ADDR:
        {
            ble_gap_resolve_address_cnf_t ret;
            LOG_D("reslv ret %d", evt->status);
            ret.status = evt->status;
            ble_event_publish(BLE_GAP_RESOLVE_ADDRESS_CNF, &ret, sizeof(ble_gap_resolve_address_cnf_t));
            break;
        }
        default:
            break;
        }
        break;
    }
    case GAPM_RAL_ADDR_IND:
    {
        struct gapm_ral_addr_ind *evt = (struct gapm_ral_addr_ind *)data_ptr;
        if (evt->operation == GAPM_GET_RAL_LOC_ADDR)
        {
            ble_gap_ral_addr_ind_t ind;
            memcpy(&ind.addr, &evt->addr, sizeof(ble_gap_addr_t));
            ble_event_publish(BLE_GAP_RAL_ADDR_IND, &ind, sizeof(ble_gap_ral_addr_ind_t));
        }
        break;
    }
    case GAPC_CONNECTION_REQ_IND:
    {
        struct gapc_connection_req_ind *ind = (struct gapc_connection_req_ind *)data_ptr;
        ble_gap_connect_ind_t evt = {0};
        evt.conn_idx = conn_idx;
        evt.con_interval = ind->con_interval;
        evt.con_latency = ind->con_latency;
        evt.sup_to = ind->sup_to;
        evt.peer_addr_type = ind->peer_addr_type;
        evt.peer_addr = ind->peer_addr;
        evt.role = ind->role;
        ble_event_publish(BLE_GAP_CONNECTED_IND, &evt, sizeof(ble_gap_connect_ind_t));
        g_ble_param_update_flag = 1;
        /* Respond link configration for stack used. */
        if (!evt.config_info.not_respond)
        {
            struct gapc_connection_cfm *cfm = sifli_msg_alloc(GAPC_CONNECTION_CFM,
                                              TASK_BUILD_ID(TASK_ID_GAPC, conn_idx), sifli_get_stack_id(),
                                              sizeof(struct gapc_connection_cfm));
            cfm->auth = evt.config_info.auth;
            cfm->ltk_present = evt.config_info.ltk_present;
            memcpy((void *)cfm->lcsrk.key, (void *)evt.config_info.lcsrk.key, GAP_KEY_LEN);
            memcpy(cfm->rcsrk.key, evt.config_info.rcsrk.key, GAP_KEY_LEN);
            cfm->lsign_counter = evt.config_info.lsign_counter;
            cfm->rsign_counter = evt.config_info.rsign_counter;
            sifli_msg_send((void const *)cfm);
        }
        break;
    }
    case GAPC_DISCONNECT_IND:
    {
        struct gapc_disconnect_ind *ind = (struct gapc_disconnect_ind *)data_ptr;
        ble_gap_disconnected_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.reason = ind->reason;
        ble_event_publish(BLE_GAP_DISCONNECTED_IND, &evt, sizeof(ble_gap_disconnected_ind_t));
        break;
    }
    case GAPC_PEER_VERSION_IND:
    {
        struct gapc_peer_version_ind *ind = (struct gapc_peer_version_ind *)data_ptr;
        ble_gap_remote_version_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.compid = ind->compid;
        evt.lmp_subvers = ind->lmp_subvers;
        evt.lmp_vers = ind->lmp_vers;
        ble_event_publish(BLE_GAP_REMOTE_VER_IND, &evt, sizeof(ble_gap_remote_version_ind_t));
        break;
    }
    case GAPC_PEER_FEATURES_IND:
    {
        struct gapc_peer_features_ind *ind = (struct gapc_peer_features_ind *)data_ptr;
        ble_gap_remote_features_ind_t evt;
        evt.conn_idx = conn_idx;
        memcpy(evt.features, ind->features, GAP_LE_FEATS_LEN);
        ble_event_publish(BLE_GAP_REMOTE_FEATURE_IND, &evt, sizeof(ble_gap_remote_features_ind_t));
        break;
    }
    case GAPC_CON_RSSI_IND:
    {
        struct gapc_con_rssi_ind *ind = (struct gapc_con_rssi_ind *)data_ptr;
        ble_gap_remote_rssi_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.rssi = ind->rssi;
        ble_event_publish(BLE_GAP_REMOTE_RSSI_IND, &evt, sizeof(ble_gap_remote_rssi_ind_t));
        break;
    }
    case GAPC_LE_PHY_IND:
    {
        struct gapc_le_phy_ind *ind = (struct gapc_le_phy_ind *)data_ptr;
        ble_gap_remote_phy_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.rx_phy = ind->rx_phy;
        evt.tx_phy = ind->tx_phy;
        ble_event_publish(BLE_GAP_REMOTE_PHY_IND, &evt, sizeof(ble_gap_remote_phy_ind_t));
        break;
    }
    case GAPC_PARAM_UPDATED_IND:
    {
        struct gapc_param_updated_ind *ind = (struct gapc_param_updated_ind *)data_ptr;
        ble_gap_update_conn_param_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.con_interval = ind->con_interval;
        evt.con_latency = ind->con_latency;
        evt.sup_to = ind->sup_to;
        ble_event_publish(BLE_GAP_UPDATE_CONN_PARAM_IND, &evt, sizeof(ble_gap_update_conn_param_ind_t));
        break;
    }
#ifndef SOC_BF_Z0
    case GAPC_PARAM_UPDATE_REQ_IND:
    {
        struct gapc_param_update_req_ind *param = (struct gapc_param_update_req_ind *)data_ptr;
        ble_gap_update_conn_response_t rsp;
        rsp.conn_idx = conn_idx;
        rsp.accept = 1;
        rsp.ce_len_max = 12;
        rsp.ce_len_min = 0;
        ble_gap_param_update_response(&rsp);
        break;
    }
#endif
    case GAPC_BOND_REQ_IND:
    {
        struct gapc_bond_req_ind *ind = (struct gapc_bond_req_ind *)data_ptr;
        ble_gap_bond_req_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.request = ind->request;
        memcpy(&evt.data, &ind->data, sizeof(ble_gap_bond_req_data_t));
        ble_event_publish(BLE_GAP_BOND_REQ_IND, &evt, sizeof(ble_gap_bond_req_ind_t));
        break;
    }
    case GAPC_BOND_IND:
    {
        struct gapc_bond_ind *ind = (struct gapc_bond_ind *)data_ptr;
        ble_gap_bond_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.info = ind->info;
        memcpy(&evt.data, &ind->data, sizeof(ble_gap_bond_data_t));
        ble_event_publish(BLE_GAP_BOND_IND, &evt, sizeof(ble_gap_bond_ind_t));
        break;
    }
    case GAPC_ENCRYPT_REQ_IND:
    {
        struct gapc_encrypt_req_ind *ind = (struct gapc_encrypt_req_ind *)data_ptr;
        ble_gap_encrypt_req_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.ediv = ind->ediv;
        evt.rand_nb = ind-> rand_nb;
        ble_event_publish(BLE_GAP_ENCRYPT_REQ_IND, &evt, sizeof(ble_gap_encrypt_req_ind_t));
        break;
    }
    case GAPC_ENCRYPT_IND:
    {
        struct gapc_encrypt_ind *ind = (struct gapc_encrypt_ind *)data_ptr;
        ble_gap_encrypt_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.auth = ind->auth;
        ble_event_publish(BLE_GAP_ENCRYPT_IND, &evt, sizeof(ble_gap_encrypt_ind_t));
        break;
    }
    case GAPM_LE_TEST_END_IND:
    {
        ble_dut_end_ind_t *ind = (ble_dut_end_ind_t *)data_ptr;
        ble_event_publish(BLE_DUT_END_IND, ind, sizeof(ble_dut_end_ind_t));
        LOG_I("ble test end, %d", ind->nb_packet_received);
        break;
    }
    case GAPM_EXT_ADV_REPORT_IND:
    {
        ble_gap_ext_adv_report_ind_t *ind = (ble_gap_ext_adv_report_ind_t *)data_ptr;
        ble_event_publish(BLE_GAP_EXT_ADV_REPORT_IND, ind, sizeof(ble_gap_ext_adv_report_ind_t) + ind->length);
        break;
    }
    case GAPC_LE_PKT_SIZE_IND:
    {
        struct gapc_le_pkt_size_ind *ind = (struct gapc_le_pkt_size_ind *)data_ptr;
        ble_gap_update_data_length_ind_t evt;
        evt.conn_idx = conn_idx;
        evt.max_rx_octets = ind->max_rx_octets;
        evt.max_rx_time = ind->max_rx_time;
        evt.max_tx_octets = ind->max_tx_octets;
        evt.max_tx_time = ind->max_tx_time;
        ble_event_publish(BLE_GAP_UPDATE_DATA_LENGTH_IND, &evt, sizeof(ble_gap_update_data_length_ind_t));
        break;
    }
    case GAPC_SIGN_COUNTER_IND:
    {
        struct gapc_sign_counter_ind *ind = (struct gapc_sign_counter_ind *)data_ptr;
        ble_gap_sign_counter_update_ind_t sign_ind;
        sign_ind.local_sign_counter = ind->local_sign_counter;
        sign_ind.peer_sign_counter = ind->peer_sign_counter;
        ble_event_publish(BLE_GAP_SIGN_COUNTER_UPDATE_IND, &sign_ind, sizeof(ble_gap_sign_counter_update_ind_t));
        break;
    }
    case GAPC_CON_CHANNEL_MAP_IND:
    {
        struct gapc_con_channel_map_ind *ind = (struct gapc_con_channel_map_ind *)data_ptr;
        ble_gap_update_channel_map_ind_t map_ind;
        memcpy(map_ind.channel_map, ind->ch_map.map, GAP_LE_CHNL_MAP_LEN);
        ble_event_publish(BLE_GAP_UPDATE_CHANNEL_MAP_IND, &map_ind, sizeof(ble_gap_update_channel_map_ind_t));
        break;
    }
    case GAPC_SECURITY_IND:
    {
        struct gapc_security_ind *ind = (struct gapc_security_ind *)data_ptr;
        ble_gap_security_request_ind_t sec_ind;
        sec_ind.conn_idx = conn_idx;
        sec_ind.auth = ind->auth;
        ble_event_publish(BLE_GAP_SECURITY_REQUEST_IND, &sec_ind, sizeof(ble_gap_security_request_ind_t));
        break;
    }
#if defined(BSP_BLE_CONNECTION_MANAGER) && !defined(BLE_CM_BOND_DISABLE)
    case GAPM_AES_H6_IND:
    {
        struct gapm_aes_h6_ind *ind = (struct gapm_aes_h6_ind *)data_ptr;
        connection_manager_h6_result_cb(ind->aes_res, ind->metainfo);
        break;
    }
#endif // BSP_BLE_CONNECTION_MANAGER
    case GAPC_GET_DEV_INFO_REQ_IND:
    {
        struct gapc_get_dev_info_req_ind *ind = (struct gapc_get_dev_info_req_ind *)data_ptr;
        switch (ind->req)
        {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm *cfm = sifli_msg_alloc(GAPC_GET_DEV_INFO_CFM,
                                                header->src_id, sifli_get_stack_id(),
                                                sizeof(struct gapc_get_dev_info_cfm) + GAP_MAX_LOCAL_NAME_LEN);
            cfm->req = ind->req;
            cfm->info.name.length = g_ble_gap_dev_name_len;
            memcpy(cfm->info.name.value, g_ble_gap_dev_name, g_ble_gap_dev_name_len);

            // Send message
            sifli_msg_send((void const *)cfm);
        }
        break;

        case GAPC_DEV_APPEARANCE:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = sifli_msg_alloc(GAPC_GET_DEV_INFO_CFM,
                                                header->src_id, sifli_get_stack_id(),
                                                sizeof(struct gapc_get_dev_info_cfm));
            cfm->req = ind->req;
            cfm->info.appearance = g_ble_gap_apperance;
            // Send message
            sifli_msg_send((void const *)cfm);
        }
        break;
        default: /* Do Nothing */
            break;
        }
        break;
    }
    case GAPC_CMP_EVT:
    {
        struct gattc_cmp_evt *evt = (struct gattc_cmp_evt *)data_ptr;
        LOG_D("GAPC event status, op: %d, ret %d", evt->operation, evt->status);
        switch (evt->operation)
        {
        case GAPC_DISCONNECT:
        {
            ble_gap_disconnect_cnf_t ret;
            ret.status = evt->status;
            ret.conn_idx = conn_idx;
            ble_event_publish(BLE_GAP_DISCONNECT_CNF, &ret, sizeof(ble_gap_disconnect_cnf_t));
            break;
        }
        case GAPC_UPDATE_PARAMS:
        {
            ble_gap_update_conn_param_cnf_t ret;
            ret.status = evt->status;
            ret.conn_idx = conn_idx;
            ble_event_publish(BLE_GAP_UPDATE_CONN_PARAM_CNF, &ret, sizeof(ble_gap_update_conn_param_cnf_t));
            g_ble_param_update_flag = 1;
            break;
        }
        case GAPC_BOND:
        {
            ble_gap_bond_cnf_t ret;
            ret.status = evt->status;
            ret.conn_idx = conn_idx;
            ble_event_publish(BLE_GAP_BOND_CNF, &ret, sizeof(ble_gap_bond_cnf_t));
            break;
        }
        case GAPC_SECURITY_REQ:
        {
            ble_gap_security_request_cnf_t ret;
            ret.status = evt->status;
            ret.conn_idx = conn_idx;
            ble_event_publish(BLE_GAP_SECURITY_REQUEST_CNF, &ret, sizeof(ble_gap_security_request_cnf_t));
            break;
        }
        case GAPC_SET_LE_PKT_SIZE:
        {
            ble_gap_update_data_length_cnf_t ret;
            ret.status = evt->status;
            ret.conn_idx = conn_idx;
            ble_event_publish(BLE_GAP_UPDATE_DATA_LENGTH_CNF, &ret, sizeof(ble_gap_update_data_length_cnf_t));
            break;
        }
        case GAPM_SET_CHANNEL_MAP:
        {
            ble_gap_update_channel_map_cnf_t ret;
            ret.status = evt->status;
            ret.conn_idx = conn_idx;
            ble_event_publish(BLE_GAP_UPDATE_CHANNEL_MAP_CNF, &ret, sizeof(ble_gap_update_channel_map_cnf_t));
            break;
        }
        default:
            break;
        }
    }
    default:
        break;
    }
}



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
