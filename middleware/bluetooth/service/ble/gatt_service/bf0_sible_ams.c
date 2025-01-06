/**
  ******************************************************************************
  * @file   bf0_sible_ams.c
  * @author Sifli software development team
  * @brief Header file - Sibles AMS remote implmentation.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "os_adaptor.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_ble_ams.h"
#include "bf0_ble_common.h"
#include "bf0_ble_gap.h"


#ifdef BSP_BLE_AMS

#define LOG_TAG "ams_srv"
#include "log.h"


const uint8_t ams_service_uuid[ATT_UUID_128_LEN] =
{
    0xDC, 0XF8, 0X55, 0XAD, 0X02, 0XC5, 0XF4, 0X8E, 0X3A, 0X43, 0X36, 0X0F, 0X2B, 0X50, 0XD3, 0x89
};

const uint8_t ams_cmd_uuid[ATT_UUID_128_LEN] =
{
    0xC2, 0x51, 0xCA, 0xF7, 0x56, 0x0E, 0xDF, 0xB8, 0x8A, 0x4A, 0xB1, 0x57, 0xD8, 0x81, 0x3C, 0x9B
};

const uint8_t ams_entity_up_uuid[ATT_UUID_128_LEN] =
{
    0x02, 0xC1, 0x96, 0xBA, 0x92, 0xBB, 0x0C, 0x9A, 0x1F, 0x41, 0x8D, 0x80, 0xCE, 0xAB, 0x7C, 0x2F
};

const uint8_t ams_entity_att_uuid[ATT_UUID_128_LEN] =
{
    0xD7, 0xD5, 0xBB, 0x70, 0xA8, 0xA3, 0xAB, 0xA6, 0xD8, 0x46, 0xAB, 0x23, 0x8C, 0xF3, 0xB2, 0xC6
};


typedef enum
{
    BLE_AMS_STATE_IDLE,
    BLE_AMS_STATE_SEARCHING,
    BLE_AMS_STATE_SEARCH_COMPLETED,
    BLE_AMS_STATE_CONFIGURAING,
    BLE_AMS_STATE_READY
} ble_ams_state_t;


typedef struct
{
    uint8_t hdl_start;
    uint8_t hdl_end;
} ble_ams_svc_t;

typedef struct
{
    uint16_t attr_hdl;
    uint16_t value_hdl;
    uint8_t prop;
    uint16_t cccd_hdl;
} ble_ams_char_t;

typedef enum
{
    BLE_AMS_CCCD_NONE,
    BLE_AMS_CCCD_WRITE_CMD,
    BLE_AMS_CCCD_WRITE_ENTITY,
    BLE_AMS_CCCD_SUBSCRIBE_PLAYER,
    BLE_AMS_CCCD_SUBSCRIBE_QUEUE,
    BLE_AMS_CCCD_SUBSCRIBE_TRACK,
} ble_ams_cccd_enable_state;

typedef struct
{
    uint8_t cccd_enable_state;    // see @ble_ams_cccd_enable_state
    uint8_t cccd_enable_value;    // enable or disable
    uint8_t is_publish;   // need publish BLE_AMS_ENABLE_RSP event
    uint8_t retry_index;    // retry count if write busy
    uint8_t cccd_retry_processing;   // if is retry, ignore new
} ble_ams_cccd_state;

typedef struct
{
    ble_ams_svc_t svc;
    uint16_t remote_handle;
    uint16_t cmd_mask;
    ble_ams_char_t cmd_char;
    ble_ams_char_t entity_up_char;
    ble_ams_char_t entity_att_char;
    uint8_t player_attr_mask;
    uint8_t queue_attr_mask;
    uint8_t track_attr_mask;
    uint8_t conn_idx;
    uint8_t is_cccd_on;
    ble_ams_state_t state;

    ble_ams_cccd_state cccd_state;
} ble_ams_env_t;

#define BLE_AMS_CCCD_WRIRE_MAX 5

static ble_ams_env_t g_ble_ams_env;

static int ble_ams_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len);


static ble_ams_env_t *ble_ams_get_env(void)
{
    return &g_ble_ams_env;
}


static void ble_ams_disconnect_handler(ble_ams_env_t *env)
{
    sibles_unregister_remote_svc(env->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_ams_gattc_event_handler);

    env->conn_idx = INVALID_CONN_IDX;
    env->state = BLE_AMS_STATE_IDLE;
}




/* Entity update subscribe format .
 *  ---------------------------------------------------
 *  | 1Byte    | 1Byte | 1Byte | 1 Byte |     | 1 Byte |
 *  |--------  |-------|-------|--------|     |--------|
 *  | EntityID | ATTR1 | ATTR2 | ATTR 3 |     | ATTR N |
 *  |--------  |-------|-------|--------|     |--------|
 */


static uint8_t ble_ams_bit_count_cal(uint8_t mask)
{
    uint8_t count = 0;
    while (mask != 0)
    {
        count += mask & 1;
        mask >>= 1;
    }
    return count;
}

static int8_t ble_ams_compose_and_send_entity_cmd(ble_ams_env_t *env, uint8_t *data, uint8_t entity_id, uint8_t attr_mask)
{
    sibles_write_remote_value_t value;
    uint8_t pos = 0;
    uint8_t *tmp = data;
    if (!env || !data)
        return -1;

    /* No need send. */
    if (attr_mask == 0)
        return 0;

    *tmp++ = entity_id;
    while (attr_mask != 0)
    {
        if ((attr_mask & 1) != 0)
            *tmp++ = pos;
        attr_mask >>= 1;
        pos++;
    }

    value.handle = env->entity_up_char.value_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = (uint16_t)(tmp - data);
    value.value = (uint8_t *)data;
    return sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);

}

static uint8_t ble_ams_subscribe_entity(ble_ams_env_t *env)
{
    int8_t ret;

    /* Attribute in AMS not */
    uint8_t cmd[BLE_AMS_MAXIMUM_ATTR_COUNT + 1];

    /* Subscribe player.*/
    ret = ble_ams_compose_and_send_entity_cmd(env, cmd, BLE_AMS_ENTITY_ID_PLAYER, env->player_attr_mask);
    if (ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
    {
        env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_SUBSCRIBE_PLAYER;
        return ret;
    }

    /* Subscribe queue.*/
    ret = ble_ams_compose_and_send_entity_cmd(env, cmd, BLE_AMS_ENTITY_ID_QUEUE, env->queue_attr_mask);
    if (ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
    {
        env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_SUBSCRIBE_QUEUE;
        return ret;
    }

    /* Subscribe track.*/
    ret = ble_ams_compose_and_send_entity_cmd(env, cmd, BLE_AMS_ENTITY_ID_TRACK, env->track_attr_mask);
    if (ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
    {
        env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_SUBSCRIBE_TRACK;
        return ret;
    }
    return 0;
}


static void ble_ams_cccd_state_clear()
{
    LOG_I("ble_ams_cccd_state_clear");
    ble_ams_env_t *env = ble_ams_get_env();
    env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_NONE;
    env->cccd_state.is_publish = 0;
    env->cccd_state.retry_index = 0;
    env->cccd_state.cccd_retry_processing = 0;
}

static int8_t ble_ams_enable_cccd(ble_ams_env_t *env, uint8_t is_enable)
{
    int8_t ret1, ret2;
    if (env->cccd_state.cccd_enable_state != BLE_AMS_CCCD_NONE)
    {
        LOG_I("ble_ams_enable_cccd skip due to busy %d", env->cccd_state.cccd_enable_state);
        return BLE_AMS_ERR_CCCD_PROCESSING;
    }
    sibles_write_remote_value_t value;
    env->cccd_state.cccd_enable_value = is_enable;
    uint16_t enable = (uint16_t)is_enable;
    value.handle = env->cmd_char.cccd_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = 2;
    value.value = (uint8_t *)&enable;
    ret1 = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    if (ret1 == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
    {
        env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_WRITE_CMD;
        return SIBLES_WIRTE_TX_FLOWCTRL_ERR;
    }
    value.handle = env->entity_up_char.cccd_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = 2;
    value.value = (uint8_t *)&enable;
    ret2 = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    if (ret2 == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
    {
        env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_WRITE_ENTITY;
        return SIBLES_WIRTE_TX_FLOWCTRL_ERR;
    }

    if (is_enable)
    {
        uint8_t ret = ble_ams_subscribe_entity(env);
        if (ret != 0)
        {
            return ret;
        }
    }

    ble_ams_cccd_state_clear();
    return ret1 & ret2;
}

static void ble_ams_enable_cccd_pending()
{
    ble_ams_env_t *env = ble_ams_get_env();
    LOG_I("ble_ams_enable_cccd_pending %d, %d", env->cccd_state.retry_index, env->cccd_state.cccd_enable_state);
    env->cccd_state.cccd_retry_processing = 1;
    int8_t ret;

    env->cccd_state.retry_index++;

    do
    {
        if (env->cccd_state.cccd_enable_state == BLE_AMS_CCCD_WRITE_CMD)
        {
            sibles_write_remote_value_t value;
            uint16_t enable = (uint16_t)env->cccd_state.cccd_enable_value;
            value.handle = env->cmd_char.cccd_hdl;
            value.write_type = SIBLES_WRITE;
            value.len = 2;
            value.value = (uint8_t *)&enable;
            ret = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);

            if (ret == SIBLES_WRITE_NO_ERR)
            {
                env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_WRITE_ENTITY;
            }
            else
            {
                break;
            }
        }

        if (env->cccd_state.cccd_enable_state == BLE_AMS_CCCD_WRITE_ENTITY)
        {
            sibles_write_remote_value_t value;
            uint16_t enable = (uint16_t)env->cccd_state.cccd_enable_value;
            value.handle = env->entity_up_char.cccd_hdl;
            value.write_type = SIBLES_WRITE;
            value.len = 2;
            value.value = (uint8_t *)&enable;
            ret = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);

            if (ret == SIBLES_WRITE_NO_ERR)
            {
                if (env->cccd_state.cccd_enable_value)
                {
                    env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_SUBSCRIBE_PLAYER;
                }
            }
            else
            {
                break;
            }
        }

        if (env->cccd_state.cccd_enable_value)
        {
            /* Attribute in AMS not */
            uint8_t cmd[BLE_AMS_MAXIMUM_ATTR_COUNT + 1];

            if (env->cccd_state.cccd_enable_state == BLE_AMS_CCCD_SUBSCRIBE_PLAYER)
            {
                /* Subscribe player.*/
                ret = ble_ams_compose_and_send_entity_cmd(env, cmd, BLE_AMS_ENTITY_ID_PLAYER, env->player_attr_mask);
                if (ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
                {
                    break;
                }
                else
                {
                    env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_SUBSCRIBE_QUEUE;
                }
            }

            if (env->cccd_state.cccd_enable_state == BLE_AMS_CCCD_SUBSCRIBE_QUEUE)
            {
                /* Subscribe queue.*/
                ret = ble_ams_compose_and_send_entity_cmd(env, cmd, BLE_AMS_ENTITY_ID_QUEUE, env->queue_attr_mask);
                if (ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
                {
                    break;
                }
                else
                {
                    env->cccd_state.cccd_enable_state = BLE_AMS_CCCD_SUBSCRIBE_TRACK;
                }
            }

            if (env->cccd_state.cccd_enable_state == BLE_AMS_CCCD_SUBSCRIBE_TRACK)
            {
                /* Subscribe track.*/
                ret = ble_ams_compose_and_send_entity_cmd(env, cmd, BLE_AMS_ENTITY_ID_TRACK, env->queue_attr_mask);
                if (ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
                {
                    break;
                }
            }
        }
    }
    while (0);


    if (ret == SIBLES_WRITE_NO_ERR)
    {
        LOG_I("ble_ams_enable_cccd_pending success");
        // retry success
        if (env->cccd_state.is_publish)
        {
            env->state = BLE_AMS_STATE_READY;

            ble_ams_enable_rsp_t rsp;
            rsp.result = BLE_AMS_ERR_NO_ERR;
            ble_event_publish(BLE_AMS_ENABLE_RSP, (void *)&rsp, sizeof(ble_ams_enable_rsp_t));
        }
        else
        {
            ble_ams_enable_pending_ind_t ind;
            ind.result = BLE_AMS_ERR_NO_ERR;
            ble_event_publish(BLE_AMS_ENABLE_PENDING_IND, (void *)&ind, sizeof(ble_ams_enable_pending_ind_t));
        }
        ble_ams_cccd_state_clear();
    }
    else
    {
        LOG_I("ble_ams_enable_cccd_pending fail %d", env->cccd_state.retry_index);
        if (env->cccd_state.retry_index > BLE_AMS_CCCD_WRIRE_MAX)
        {
            // retry fail max time reach
            if (env->cccd_state.is_publish)
            {
                ble_ams_enable_rsp_t rsp;
                rsp.result = BLE_AMS_ERR_REGISTER_REMOTE_DEVICE_FAILED;
                ble_event_publish(BLE_AMS_ENABLE_RSP, (void *)&rsp, sizeof(ble_ams_enable_rsp_t));
            }
            else
            {
                ble_ams_enable_pending_ind_t ind;
                ind.result = BLE_AMS_ERR_REGISTER_REMOTE_DEVICE_FAILED;
                ble_event_publish(BLE_AMS_ENABLE_PENDING_IND, (void *)&ind, sizeof(ble_ams_enable_pending_ind_t));
            }
            ble_ams_cccd_state_clear();
        }
        else
        {
            LOG_I("ble_ams_enable_cccd_pending fail wait retry");
            // retry when next SIBLES_WRITE_REMOTE_VALUE_RSP
            env->cccd_state.cccd_retry_processing = 0;
        }
    }
}


/* Remote command format in Byte:  |cmd1|cmd2|cmd3|...|cmdn| */
static void ble_ams_remote_cmd_notify_handler(ble_ams_env_t *env, uint8_t *value, uint16_t len)
{
    OS_ASSERT(len < BLE_AMS_CMD_TOTAL);
    /* Clear origin mask.*/
    env->cmd_mask = 0;
    while (len--)
    {
        env->cmd_mask |= 1 << (*value++);
    }

    ble_ams_supported_cmd_notify_ind_t cmd_notify;
    cmd_notify.cmd_mask = env->cmd_mask;
    ble_event_publish(BLE_AMS_SUPPORTED_CMD_NOTIFY_IND, &cmd_notify, sizeof(ble_ams_supported_cmd_notify_ind_t));
}


/* Remote command format in Byte:  |entityID |AttID|Flag|Value...| */
static void ble_ams_entity_up_notify_handler(ble_ams_env_t *env, uint8_t *value, uint16_t len)
{
    ble_ams_entity_attr_value_t *value_pair = bt_mem_alloc(sizeof(ble_ams_entity_attr_value_t) + len - 3);
    BT_OOM_ASSERT(value_pair);
    if (value_pair)
    {
        value_pair->entity_id = *value++;
        value_pair->attr_id = *value++;
        value_pair->entity_up_flag = *value++;
        value_pair->len = len - 3;
        memcpy((void *)value_pair->value, value, len - 3);

        ble_event_publish(BLE_AMS_ENTITY_ATTRIBUTE_PAIR_IND, (void *)value_pair, sizeof(ble_ams_entity_attr_value_t) + len - 3);
        bt_mem_free(value_pair);
    }
}

void ble_ams_player_attr_enable(uint8_t attr_mask)
{
    ble_ams_env_t *env = ble_ams_get_env();
    env->player_attr_mask |= attr_mask;
}

void ble_ams_player_attr_disable(uint8_t attr_mask)
{
    ble_ams_env_t *env = ble_ams_get_env();
    env->player_attr_mask &= ~attr_mask;
}

void ble_ams_queue_attr_enable(uint8_t attr_mask)
{
    ble_ams_env_t *env = ble_ams_get_env();
    env->queue_attr_mask |= attr_mask;
}

void ble_ams_queue_attr_disable(uint8_t attr_mask)
{
    ble_ams_env_t *env = ble_ams_get_env();
    env->queue_attr_mask &= ~attr_mask;
}

void ble_ams_track_attr_enable(uint8_t attr_mask)
{
    ble_ams_env_t *env = ble_ams_get_env();
    env->track_attr_mask |= attr_mask;
}

void ble_ams_track_attr_disable(uint8_t attr_mask)
{
    ble_ams_env_t *env = ble_ams_get_env();
    env->track_attr_mask &= ~attr_mask;
}



uint8_t ble_ams_send_command(ble_ams_cmd_t cmd)
{
    uint8_t ret = BLE_AMS_ERR_NO_ERR;
    ble_ams_env_t *env = ble_ams_get_env();

    if (cmd >= BLE_AMS_CMD_TOTAL)
        return BLE_AMS_ERR_REJECTED;

    if (env->state == BLE_AMS_STATE_READY)
    {
        int8_t ret1;
        sibles_write_remote_value_t value;
        value.handle = env->cmd_char.value_hdl;
        value.write_type = SIBLES_WRITE;
        value.len = 2;
        value.value = (uint8_t *)&cmd;

        // avoid send full
        if (sibles_get_tx_pkts() == 0)
        {
            ret1 = SIBLES_WIRTE_TX_FLOWCTRL_ERR;
        }
        else
        {
            ret1 = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
        }

        if (ret1 != 0)
            ret = BLE_AMS_ERR_GENERATE;
    }
    else
        ret = BLE_AMS_ERR_NO_CONNECTION;
    return ret;
}




uint8_t ble_ams_cccd_enable(uint8_t is_enable)
{
    uint8_t ret = BLE_AMS_ERR_NO_ERR;
    ble_ams_env_t *env = ble_ams_get_env();
    int8_t  res;

    if (env->state == BLE_AMS_STATE_READY)
    {
        LOG_I("ams_cccd_enable current%d set %d", env->is_cccd_on, is_enable);
        if (is_enable != (env->is_cccd_on))
        {
            env->is_cccd_on = is_enable;
            res = ble_ams_enable_cccd(env, is_enable);
            if (res == SIBLES_WRITE_NO_ERR)
            {
                // success
            }
            else if (res == SIBLES_WRITE_NO_ERR)
            {
                return BLE_AMS_ERR_CCCD_PENDING;
            }
            else
            {
                return BLE_AMS_ERR_REJECTED;
            }
        }
        else
        {
            return SIBLES_WRITE_NO_ERR;
        }
    }
    else if (env->state <= BLE_AMS_STATE_SEARCH_COMPLETED)
    {
        env->is_cccd_on = is_enable;
    }
    else
        ret = BLE_AMS_ERR_REJECTED;
    return ret;
}

bool ble_ams_is_cccd_enable()
{
    ble_ams_env_t *env = ble_ams_get_env();
    return env->is_cccd_on;
}

uint8_t ble_ams_enable(uint8_t conn_idx)
{
    ble_ams_env_t *env = ble_ams_get_env();
    if (env->state >= BLE_AMS_STATE_SEARCHING)
        return 0xFF;
    sibles_search_service(conn_idx, ATT_UUID_128_LEN, (uint8_t *)ams_service_uuid);
    env->conn_idx = conn_idx;
    env->state = BLE_AMS_STATE_SEARCHING;
    env->cmd_mask = BLE_AMS_CMD_MASK_ALL - 1;
    return 0;
}




static int ble_ams_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    ble_ams_env_t *env = ble_ams_get_env();
    int8_t res;

    LOG_I("ams gattc event handler %d\r\n", event_id);
    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        ble_ams_enable_rsp_t ret;
        LOG_I("register ret %d\r\n", rsp->status);
        if (rsp->status != HL_ERR_NO_ERROR)
        {
            ret.result = BLE_AMS_ERR_REGISTER_REMOTE_DEVICE_FAILED;
            ble_event_publish(BLE_AMS_ENABLE_RSP, (void *)&ret, sizeof(ble_ams_enable_rsp_t));
            break;
        }
        env->state = BLE_AMS_STATE_SEARCH_COMPLETED;

        if (env->is_cccd_on)
        {
            env->cccd_state.is_publish = 1;
            res = ble_ams_enable_cccd(env, env->is_cccd_on);
            if (res != SIBLES_WRITE_NO_ERR)
            {
                LOG_I("return with res %d", res);
                break;
            }
        }

        env->state = BLE_AMS_STATE_READY;

        ret.result = BLE_AMS_ERR_NO_ERR;
        ble_event_publish(BLE_AMS_ENABLE_RSP, (void *)&ret, sizeof(ble_ams_enable_rsp_t));
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        if (env->state != BLE_AMS_STATE_READY)
        {
            LOG_I("ams state error %d", env->state);
            return 0;
        }

        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        LOG_I("ams handle:%d", ind->handle);
        if (ind->handle == env->cmd_char.value_hdl)
        {
            ble_ams_remote_cmd_notify_handler(env, ind->value, ind->length);
        }
        else if (ind->handle == env->entity_up_char.value_hdl)
        {
            ble_ams_entity_up_notify_handler(env, ind->value, ind->length);
        }

        break;
    }
    default:
        break;
    }
    return 0;
}

int ble_ams_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_ams_env_t *env = ble_ams_get_env();
    switch (event_id)
    {
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;

        // rsp->svc may null
        if (memcmp(rsp->search_uuid, ams_service_uuid, rsp->search_svc_len) != 0)
            break;

        if (rsp->result != HL_ERR_NO_ERROR)
        {
            ble_ams_enable_rsp_t ret;
            ret.result = BLE_AMS_ERR_SEARCH_REMOTE_SERVICE_FAILED;
            ble_event_publish(BLE_AMS_ENABLE_RSP, (void *)&ret, sizeof(ble_ams_enable_rsp_t));
            break; // Do nothing
        }

        env->svc.hdl_start = rsp->svc->hdl_start;
        env->svc.hdl_end = rsp->svc->hdl_end;
        uint32_t i;
        uint8_t attr_count = 0; // This char is madatory
        uint16_t offset = 0;
        sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
        uint8_t cccd_uuid[ATT_UUID_16_LEN] = {0x02, 0x29};
        LOG_I("char cont %d", rsp->svc->char_count);
        for (i = 0; i < rsp->svc->char_count; i++)
        {
            LOG_HEX("uuid", 16, (uint8_t *)chara->uuid, chara->uuid_len);
            if (!memcmp(chara->uuid, ams_cmd_uuid, chara->uuid_len))
            {
                LOG_I("Remote cmd received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                //OS_ASSERT(chara->desc_count == 1);
                env->cmd_char.attr_hdl = chara->attr_hdl;
                env->cmd_char.value_hdl = chara->pointer_hdl;
                env->cmd_char.prop = chara->prop;
                for (uint32_t j = 0; j < chara->desc_count; j++)
                {
                    LOG_I("desc %x", chara->desc[j].uuid[0]);
                    if (chara->desc[j].uuid_len == ATT_UUID_16_LEN &&
                            memcmp(chara->desc[j].uuid, cccd_uuid, ATT_UUID_16_LEN) == 0)
                    {
                        LOG_I("remote handle %d", chara->desc[j].attr_hdl);
                        env->cmd_char.cccd_hdl = chara->desc[j].attr_hdl;
                    }
                }
                attr_count++;
            }
            else if (!memcmp(chara->uuid, ams_entity_up_uuid, chara->uuid_len))
            {
                LOG_I("Entity up received, att handle(%x)", chara->attr_hdl);
                env->entity_up_char.attr_hdl = chara->attr_hdl;
                env->entity_up_char.value_hdl = chara->pointer_hdl;
                env->entity_up_char.prop = chara->prop;
                for (uint32_t j = 0; j < chara->desc_count; j++)
                {
                    LOG_I("desc1 %x", chara->desc[j].uuid[0]);
                    if (chara->desc[j].uuid_len == ATT_UUID_16_LEN &&
                            memcmp(chara->desc[j].uuid, cccd_uuid, ATT_UUID_16_LEN) == 0)
                    {
                        LOG_I("Entity handle %d", chara->desc[j].attr_hdl);
                        env->entity_up_char.cccd_hdl = chara->desc[j].attr_hdl;
                    }
                }
                attr_count++;
            }
            else if (!memcmp(chara->uuid, ams_entity_att_uuid, chara->uuid_len))
            {
                LOG_I("Entity att received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                //OS_ASSERT(chara->desc_count == 1);
                env->entity_att_char.attr_hdl = chara->attr_hdl;
                env->entity_att_char.value_hdl = chara->pointer_hdl;
                env->entity_att_char.prop = chara->prop;
                attr_count++;
            }
            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }
        if (attr_count != 3)
        {
            LOG_E("ams attr count error %d", attr_count);
            ble_ams_enable_rsp_t count_ret;
            count_ret.result = BLE_AMS_ERR_SEARCH_REMOTE_SERVICE_FAILED;
            ble_event_publish(BLE_AMS_ENABLE_RSP, (void *)&count_ret, sizeof(ble_ams_enable_rsp_t));
            break; // Do nothing
        }
        //register first
        env->remote_handle = sibles_register_remote_svc(rsp->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_ams_gattc_event_handler);
        // subscribe data src. then subscribe notfi src.
        break;
    }
    case BLE_POWER_ON_IND:
    {
        env->conn_idx = INVALID_CONN_IDX;
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        //sibles_remote_connected_ind_t *ind = (sibles_remote_connected_ind_t *)data;
        //env->conn_idx = ind->conn_idx;
        /* Temp set. */
        //env->cmd_mask = BLE_AMS_CMD_MASK_ALL - 1;
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        if (env->conn_idx == ind->conn_idx)
        {
            ble_ams_cccd_state_clear();
            ble_ams_disconnect_handler(env);
        }
        break;
    }
    case SIBLES_WRITE_REMOTE_VALUE_RSP:
    {
        // LOG_D("tx pkt num++");
        if (env->conn_idx != INVALID_CONN_IDX)
        {
            if (env->cccd_state.cccd_enable_state != BLE_AMS_CCCD_NONE)
            {
                if (env->cccd_state.cccd_retry_processing == 0)
                {
                    ble_ams_enable_cccd_pending();
                }
            }
        }
        break;
    }
    case SIBLES_REMOTE_SVC_CHANGE_IND:
    {
        sibles_remote_svc_change_ind_t *ind = (sibles_remote_svc_change_ind_t *)data;
        // LOG_I("ancs svc changed %d %d", ind->start_handle, ind->end_handle);
        // LOG_I("ancs %d, %d, state %d, sh %d, eh %d", ind->conn_idx, env->conn_idx, env->state, env->svc.hdl_start, env->svc.hdl_end);
        if (env->conn_idx == ind->conn_idx)
        {
            if (env->state == BLE_AMS_STATE_READY || env->state == BLE_AMS_STATE_IDLE)
            {
                if (ind->start_handle <= env->svc.hdl_start && ind->end_handle >= env->svc.hdl_end)
                {
                    LOG_I("update");
                    if (env->state == BLE_AMS_STATE_READY)
                    {
                        LOG_I("disconnect");
                        ble_ams_disconnect_handler(env);
                        rt_thread_mdelay(1000);
                    }

                    env->conn_idx = ind->conn_idx;
                    ble_ams_enable(env->conn_idx);
                }
            }
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

BLE_EVENT_REGISTER(ble_ams_event_handler, NULL);

#if defined(RT_USING_FINSH)&&!defined(LCPU_MEM_OPTIMIZE)
#include <finsh.h>
int cmd_ams(int argc, char *argv[])
{
    ble_ams_env_t *env = ble_ams_get_env();
    if (argc >= 1)
    {
        if (strcmp(argv[1], "enable") == 0)
        {
            ble_ams_cccd_enable(1);
            ble_ams_player_attr_enable(BLE_AMS_PLAYER_ATTR_ID_ALL_MASK);
            ble_ams_queue_attr_enable(BLE_AMS_QUEUE_ATTR_ID_ALL_MASK);
            ble_ams_track_attr_enable(BLE_AMS_TRACK_ATTR_ID_ALL_MASK);
            ble_ams_enable(env->conn_idx);
        }
        else if (strcmp(argv[1], "cmd") == 0)
        {
            ble_ams_cmd_t cmd = atoi(argv[2]);
            ble_ams_send_command(cmd);
        }
    }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_ams, __cmd_ams, My device information service.);
#endif

#endif // BSP_BLE_AMS


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
