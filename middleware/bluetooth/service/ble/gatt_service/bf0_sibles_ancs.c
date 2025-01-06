/**
  ******************************************************************************
  * @file   bf0_sibles_ancs.c
  * @author Sifli software development team
  * @brief Header file - Sibles ANCS implmentation.
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
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "os_adaptor.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_ble_ancs.h"
#include "bf0_ble_common.h"
#include "bf0_ble_gap.h"

#ifdef BSP_BLE_ANCS

#define LOG_TAG "sibles"
#include "log.h"


const uint8_t ancs_service_uuid[ATT_UUID_128_LEN] =
{
    0xD0, 0X00, 0X2D, 0X12, 0X1E, 0X4B, 0X0F, 0XA4, 0X99, 0X4E, 0XCE, 0XB5, 0X31, 0XF4, 0X05, 0x79
};

const uint8_t ancs_noti_src_uuid[ATT_UUID_128_LEN] =
{
    0xBD, 0x1D, 0xA2, 0x99, 0xE6, 0x25, 0x058, 0x8C, 0xD9, 0x42, 0x01, 0x63, 0x0D, 0x12, 0xBF, 0x9F
};

const uint8_t ancs_controlp_uuid[ATT_UUID_128_LEN] =
{
    0xD9, 0xD9, 0xAA, 0xFD, 0xBD, 0x9B, 0x21, 0x98, 0xA8, 0x49, 0xE1, 0x45, 0xF3, 0xD8, 0xD1, 0x69
};

const uint8_t ancs_data_src_uuid[ATT_UUID_128_LEN] =
{
    0xFB, 0x7B, 0x7C, 0xCE, 0x6A, 0xB3, 0x44, 0xBE, 0xB5, 0x4B, 0xD6, 0x24, 0xE9, 0xC6, 0xEA, 0x22
};

#define BLE_ANCS_NOTI_QUEUE_SIZE 16
#define BLE_ANCS_NOTI_ATTR_MAX_LEN 14 //8+6
#define BLE_ANCS_NOTI_ATTR_NUMBER 8
#define BLE_ANCS_APP_ATTR_NUMBER 1

typedef enum
{
    BLE_ANCS_STATE_IDLE,
    BLE_ANCS_STATE_SEARCHING,
    BLE_ANCS_STATE_SEARCH_COMPLETED,
    BLE_ANCS_STATE_CONFIGURAING,
    BLE_ANCS_STATE_READY
} ble_ancs_state_t;

typedef struct
{
    uint8_t enable;
    uint16_t len;
} ble_ancs_attr_config_list_t;


typedef struct
{
    uint8_t hdl_start;
    uint8_t hdl_end;
} ble_ancs_svc_t;

typedef struct
{
    uint16_t attr_hdl;
    uint16_t value_hdl;
    uint8_t prop;
    uint16_t cccd_hdl;
} ble_ancs_char_t;



typedef __PACKED_STRUCT
{
    uint8_t command_id;
    uint32_t noti_uid;
    uint8_t attr_id[BLE_ANCS_NOTI_ATTR_MAX_LEN];
} ble_ancs_get_noti_attr_t;


typedef __PACKED_STRUCT
{
    uint8_t command_id;
    uint32_t noti_uid;
    ble_ancs_attr_value_t value[__ARRAY_EMPTY];
} ble_ancs_get_noti_attr_rsp_t;


typedef __PACKED_STRUCT
{
    uint8_t command_id;
    uint32_t noti_uid;
    uint8_t action_id;
} ble_ancs_perform_noti_action_t;

typedef struct
{
    ble_ancs_svc_t svc;
    ble_ancs_char_t noti_src_char;
    ble_ancs_char_t controlp_char;
    ble_ancs_char_t data_src_char;
    ble_ancs_attr_config_list_t noti_attr_list[BLE_ANCS_NOTI_ATTR_NUMBER];
    ble_ancs_attr_config_list_t app_attr_list[BLE_ANCS_APP_ATTR_NUMBER];
    uint8_t *frag_ptr;
    uint16_t frag_offset;
    uint16_t remote_handle;
    uint16_t cate_mask;
    uint8_t is_fragment;
    uint8_t conn_idx;
    uint8_t is_cccd_on;
    ble_ancs_state_t state;
} ble_ancs_env_t;


typedef struct
{
    ble_ancs_noti_attr_t src;
    uint8_t is_used;
} ble_ancs_noti_attr_queue_t;

static ble_ancs_env_t g_ble_ancs_env;

static ble_ancs_noti_attr_queue_t g_ble_ancs_noti_queue[BLE_ANCS_NOTI_QUEUE_SIZE];

static int ble_ancs_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len);


static ble_ancs_env_t *ble_ancs_get_env(void)
{
    return &g_ble_ancs_env;
}

static void ble_ancs_clear_noti_buffer(void)
{
    memset((void *)&g_ble_ancs_noti_queue, 0, sizeof(g_ble_ancs_noti_queue));
}

static ble_ancs_noti_attr_queue_t *ble_ancs_get_noti_buffer(void)
{
    uint32_t i;
    for (i = 0; i < BLE_ANCS_NOTI_QUEUE_SIZE; i++)
    {
        if (g_ble_ancs_noti_queue[i].is_used == 0)
            return &g_ble_ancs_noti_queue[i];
    }

    return NULL;
}

static ble_ancs_noti_attr_queue_t *ble_ancs_find_noti_buffer_by_uuid(uint32_t noti_uid)
{
    uint32_t i;
    for (i = 0; i < BLE_ANCS_NOTI_QUEUE_SIZE; i++)
    {
        if (g_ble_ancs_noti_queue[i].is_used && g_ble_ancs_noti_queue[i].src.noti_uid == noti_uid)
            return &g_ble_ancs_noti_queue[i];
    }

    return NULL;
}

#if 0
static void ble_ancs_parser_notification_source(uint8_t *data, ble_ancs_noti_attr_t *header)
{
    RT_ASSERT(data && header);
    header->evt_id = *data++;
    header->evt_flag = *data++;
    header->cate_id = *data++;
    header->cate_count = *data++;
    memcpy(header->noti_uid, data, sizeof(uint32_t));
}
#endif

static void ble_ancs_parser_notification_source(uint8_t *data, ble_ancs_noti_attr_t *header)
{
    OS_ASSERT(data && header);
    header->evt_id = *data++;
    header->evt_flag = *data++;
    header->cate_id = *data++;
    header->cate_count = *data++;
    memcpy(&header->noti_uid, data, sizeof(uint32_t));
}

// estimate value
#define MAX_ANCS_ATTRIBUTE_OTHER_TOTAL_LENGTH 50

static uint16_t ble_ancs_get_maximum_attribute_len(void)
{
    ble_ancs_env_t *env = ble_ancs_get_env();
    // The 3 type length is set by user
    uint16_t len = env->noti_attr_list[BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE].len +
                   env->noti_attr_list[BLE_ANCS_NOTIFICATION_ATTR_ID_SUB_TITLE].len +
                   env->noti_attr_list[BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE].len;
    len += MAX_ANCS_ATTRIBUTE_OTHER_TOTAL_LENGTH;
    return len;
}

static void ble_ancs_parser_get_noti_attr_rsp(ble_ancs_get_noti_attr_rsp_t *data, uint16_t data_len)
{
    ble_ancs_noti_attr_queue_t *noti = ble_ancs_find_noti_buffer_by_uuid(data->noti_uid);
    ble_ancs_env_t *env = ble_ancs_get_env();
    OS_ASSERT(data);
    uint16_t len = data_len - sizeof(ble_ancs_get_noti_attr_rsp_t);
    ble_ancs_attr_value_t *attr_value = data->value;
    while (len)
    {
        // If len smaller than attribute len, just ignore. This case happen when ANCS assemble length set larger than GATT MTU set.
        if (len < (attr_value->len + sizeof(ble_ancs_attr_value_t)))
        {
            // Wait completed data and parse again.
            if (env->is_fragment == 0)
            {
                uint16_t att_len  = ble_ancs_get_maximum_attribute_len();
                env->is_fragment = 1;
                env->frag_ptr = bt_mem_alloc(att_len);
                env->frag_offset = 0;
                BT_OOM_ASSERT(env->frag_ptr);
                if (env->frag_ptr)
                    memcpy(env->frag_ptr, data, data_len);
            }
            // The data len already include
            env->frag_offset = data_len;
            noti->src.attr_count = 0;
            return;
        }
        len -= attr_value->len + sizeof(ble_ancs_attr_value_t);
        LOG_HEX("ancs_attr", 16, (uint8_t *)attr_value, attr_value->len + sizeof(ble_ancs_attr_value_t));
        attr_value = (ble_ancs_attr_value_t *)((uint8_t *)attr_value + sizeof(ble_ancs_attr_value_t) + attr_value->len);
        noti->src.attr_count++;
    }
    if (env->is_fragment)
        noti->src.value = ((ble_ancs_get_noti_attr_rsp_t *)env->frag_ptr)->value;
    else
        noti->src.value = data->value;
    // Notify user
    ble_event_publish(BLE_ANCS_NOTIFICATION_IND, &noti->src, sizeof(ble_ancs_noti_attr_t));
    // clear buffer
    if (env->is_fragment)
    {
        env->is_fragment = 0;
        bt_mem_free(env->frag_ptr);
        env->frag_ptr = NULL;
        env->frag_offset = 0;
    }
    memset(noti, 0, sizeof(ble_ancs_noti_attr_queue_t));
}

// APP ATTR need cache during a session so that need upper layer to trigger to acquire.
static void ble_ancs_parser_get_app_attr_rsp(uint8_t *data, uint16_t data_len)
{
    uint8_t command_id = *data++;
    // It must be has a NULL termintated
    uint8_t app_id_len = strlen((const char *)data) + 1;
    OS_ASSERT(app_id_len < data_len);

    ble_ancs_get_app_attr_rsp_t rsp;
    rsp.app_id = bt_mem_alloc(app_id_len);
    if (rsp.app_id == NULL)
    {
        // Stop to handle.
        // Notify upper layer failed.
        LOG_E("get attr rsp OOM!");
        return;
    }
    memcpy(rsp.app_id, data, app_id_len);
    rsp.app_id_len = app_id_len;
    // APP Only have one attribute currently.
    rsp.attr_count = 1;
    rsp.value = (ble_ancs_attr_value_t *)(data + app_id_len);
    // Notify upper layer
    ble_event_publish(BLE_ANCS_GET_APP_ATTR_RSP, &rsp, sizeof(ble_ancs_get_app_attr_rsp_t));
    // Then release
    bt_mem_free(rsp.app_id);
}

static void ble_ancs_parser_data_source(uint8_t *data, uint16_t data_len)
{
    OS_ASSERT(data);
    uint8_t command_id = *data;
    if (command_id == BLE_ANCS_COMMAND_ID_GET_NOTIFICATION_ATTR)
    {
        ble_ancs_parser_get_noti_attr_rsp((ble_ancs_get_noti_attr_rsp_t *)data, data_len);
    }
    else if (command_id == BLE_ANCS_COMMAND_ID_GET_APP_ATTR)
    {
        ble_ancs_parser_get_app_attr_rsp(data, data_len);
    }
}

static void ble_ancs_get_notification_attr(ble_ancs_noti_attr_t *header)
{
    // Get the attribute
    ble_ancs_get_noti_attr_t attr;
    ble_ancs_env_t *env = ble_ancs_get_env();
    uint32_t i;
    attr.command_id = BLE_ANCS_COMMAND_ID_GET_NOTIFICATION_ATTR;
    memcpy(&attr.noti_uid, &header->noti_uid, sizeof(uint32_t));
    uint8_t *attr_id = attr.attr_id;
    uint8_t attr_len = 0;
    int8_t res;

    for (i = BLE_ANCS_NOTIFICATION_ATTR_ID_APP_ID; i <= BLE_ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABLE; i++)
    {
        if (!env->noti_attr_list[i].enable)
            continue;

        // 0x20 is a speicial case that spec not defined this value. From currently case, once
        // 0x20 is existed in the evt_flags, get attribute with app_id will cause IOS won't send
        // notification any more. So this is a workaround that removed app_id for this case.
        //if (i == BLE_ANCS_NOTIFICATION_ATTR_ID_APP_ID && (header->evt_flag & 0x20))
        //continue;



        *attr_id = i;
        if (*attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE ||
                *attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_SUB_TITLE ||
                *attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE)
        {
            memcpy(attr_id + 1, &env->noti_attr_list[i].len, sizeof(uint16_t));
            attr_id += sizeof(uint16_t);
            attr_len += 2;
        }


        attr_id++;
        attr_len++;

    }
    // Write to control point
    sibles_write_remote_value_t value;
    value.handle = env->controlp_char.value_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = 1 + 4 + attr_len;
    value.value = (uint8_t *)&attr;
    res = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
}

static int8_t ble_ancs_enable_cccd(ble_ancs_env_t *env, uint8_t is_enable)
{
    int8_t ret1, ret2;
    sibles_write_remote_value_t value;
    uint16_t enable = (uint16_t)is_enable;
    value.handle = env->data_src_char.cccd_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = 2;
    value.value = (uint8_t *)&enable;
    ret1 = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    value.handle = env->noti_src_char.cccd_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = 2;
    value.value = (uint8_t *)&enable;
    ret2 = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    return ret1 & ret2;
}

static void ble_ancs_disconnect_handler(ble_ancs_env_t *env)
{
    // step1: de-register remote svc
    sibles_unregister_remote_svc(env->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_ancs_gattc_event_handler);
    // step2: clear noti buffer
    ble_ancs_clear_noti_buffer();
    // step3: reset env
    env->conn_idx = INVALID_CONN_IDX;
    env->state = BLE_ANCS_STATE_IDLE;
    if (env->is_fragment)
    {
        env->is_fragment = 0;
        bt_mem_free(env->frag_ptr);
        env->frag_ptr = NULL;
        env->frag_offset = 0;
    }
}

int32_t ble_ancs_get_app_attr(uint8_t *app_id, uint8_t app_id_len)
{
    // app_id_len + command_id + display_attr
    ble_ancs_env_t *env = ble_ancs_get_env();
    sibles_write_remote_value_t value;
    uint8_t *data = bt_mem_alloc(app_id_len + 2);
    int8_t  res;

    if (data == NULL)
    {
        LOG_E("get app attr OOM");
        return -1; // Define error code later
    }
    *data = BLE_ANCS_COMMAND_ID_GET_APP_ATTR;
    memcpy(data + 1, app_id, app_id_len);
    *(data + 1 + app_id_len) = BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME;
    // Write to control point
    value.handle = env->controlp_char.value_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = app_id_len + 2;
    value.value = data;
    res = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
    bt_mem_free(data);
    return 0;
}

void ble_ancs_perform_notification_action(uint32_t noti_uid, ble_ancs_action_id_val_t action_id)
{
    ble_ancs_env_t *env = ble_ancs_get_env();
    sibles_write_remote_value_t value;
    ble_ancs_perform_noti_action_t action;
    int8_t    res;


    action.command_id = BLE_ANCS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION;
    action.noti_uid = noti_uid;
    action.action_id = action_id;

    // Write to control point
    value.handle = env->controlp_char.value_hdl;
    value.write_type = SIBLES_WRITE;
    value.len = sizeof(ble_ancs_perform_noti_action_t);
    value.value = (uint8_t *)&action;
    res = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
    OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
}


static void ble_ancs_notification_source_notify_handler(uint8_t *data)
{
    ble_ancs_noti_attr_queue_t *header = ble_ancs_get_noti_buffer();
    ble_ancs_env_t *env = ble_ancs_get_env();
    if (header == NULL || data == NULL)
        return; // No buffer to handle
    ble_ancs_parser_notification_source(data, &header->src);

    if (!((1 << header->src.cate_id) & env->cate_mask))
    {
        return; // No need handle this type.
    }

    if (header->src.evt_flag & BLE_ANCS_EVENT_FLAG_PRE_EXISTING)
    {
        return; // No need to notify app removed.
    }

    // Removed evt should not get attribute
    if (header->src.evt_id == BLE_ANCS_EVENT_ID_NOTIFICATION_REMOVED)
    {
        header->src.attr_count = 0;
        header->src.value = NULL;
        ble_event_publish(BLE_ANCS_NOTIFICATION_IND, &header->src, sizeof(ble_ancs_noti_attr_t));
        memset(header, 0, sizeof(ble_ancs_noti_attr_queue_t));
        return;
    }
    ble_ancs_get_notification_attr(&header->src);
    header->is_used = 1;
}

void ble_ancs_attr_enable(uint8_t attr_index, uint8_t enable, uint16_t len)
{
    ble_ancs_env_t *env = ble_ancs_get_env();
    OS_ASSERT(attr_index < BLE_ANCS_NOTI_ATTR_NUMBER);
    env->noti_attr_list[attr_index].enable = enable;
    env->noti_attr_list[attr_index].len = len;
}

void ble_ancs_app_enable(uint8_t app_index, uint8_t enable, uint16_t len)
{
    ble_ancs_env_t *env = ble_ancs_get_env();
    OS_ASSERT(app_index < BLE_ANCS_APP_ATTR_NUMBER);
    env->app_attr_list[app_index].enable = enable;
    env->app_attr_list[app_index].len = len;
}

void ble_ancs_category_mask_set(uint16_t mask)
{
    ble_ancs_env_t *env = ble_ancs_get_env();
    env->cate_mask = mask;
}

uint8_t ble_ancs_cccd_enable(uint8_t is_enable)
{
    uint8_t ret = BLE_ANCS_ERR_NO_ERR;
    int8_t  res;

    ble_ancs_env_t *env = ble_ancs_get_env();

    if (env->state == BLE_ANCS_STATE_READY)
    {
        env->is_cccd_on = is_enable;
        res = ble_ancs_enable_cccd(env, is_enable);
        OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
    }
    else if (env->state <= BLE_ANCS_STATE_SEARCH_COMPLETED)
    {
        env->is_cccd_on = is_enable;
    }
    else
        ret = BLE_ANCS_ERR_REJECTED;
    return ret;
}


uint8_t ble_ancs_enable(uint8_t conn_idx)
{
    ble_ancs_env_t *env = ble_ancs_get_env();
    if (env->state >= BLE_ANCS_STATE_SEARCHING)
        return 0xFF;
    sibles_search_service(conn_idx, ATT_UUID_128_LEN, (uint8_t *)ancs_service_uuid);
    // Only treat enable conn as search target
    env->conn_idx = conn_idx;
    env->state = BLE_ANCS_STATE_SEARCHING;
    return 0;
}




static int ble_ancs_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    ble_ancs_env_t *env = ble_ancs_get_env();
    int8_t  res;

    //LOG_I("ancs gattc event handler %d\r\n", event_id);
    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        ble_ancs_enable_rsp_t enable_rsp;
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        enable_rsp.conn_idx = rsp->conn_idx;
        LOG_I("register ret %d\r\n", rsp->status);
        if (rsp->status != HL_ERR_NO_ERROR)
        {
            enable_rsp.result = BLE_ANCS_ERR_REGISTER_REMOTE_DEVICE_FAILED;
            env->state = BLE_ANCS_STATE_IDLE;
            ble_event_publish(BLE_ANCS_ENABLE_RSP, &enable_rsp, sizeof(ble_ancs_enable_rsp_t));
            break;
        }
        env->state = BLE_ANCS_STATE_SEARCH_COMPLETED;

        if (env->is_cccd_on)
        {
            res = ble_ancs_enable_cccd(env, env->is_cccd_on);
            OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
        }

        enable_rsp.result = BLE_ANCS_ERR_NO_ERR;
        env->state = BLE_ANCS_STATE_READY;
        ble_event_publish(BLE_ANCS_ENABLE_RSP, &enable_rsp, sizeof(ble_ancs_enable_rsp_t));
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        if (env->state != BLE_ANCS_STATE_READY)
        {
            LOG_I("ancs state error %d", env->state);
            return 0;
        }

        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        //LOG_I("ancs handle:%d", ind->handle);
        if (ind->handle == env->noti_src_char.value_hdl)
        {
            ble_ancs_notification_source_notify_handler(ind->value);
        }
        else if (ind->handle == env->data_src_char.value_hdl)
        {
            if (env->is_fragment)
            {
                OS_ASSERT(env->frag_ptr && "Fragment ptr should not be NULL!");
                // if this assert happened,try to increase length when calling ble_ancs_attr_enable
                OS_ASSERT((env->frag_offset + ind->length) <= ble_ancs_get_maximum_attribute_len());
                memcpy(env->frag_ptr + env->frag_offset, ind->value, ind->length);
                ble_ancs_parser_data_source(env->frag_ptr, env->frag_offset + ind->length);
            }
            else
                ble_ancs_parser_data_source(ind->value, ind->length);
        }

        // Notify upper layer
        break;
    }
    default:
        break;
    }
    return 0;
}

int ble_ancs_ble_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_ancs_env_t *env = ble_ancs_get_env();
    switch (event_id)
    {
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;

        // rsp->svc may null
        if (memcmp(rsp->search_uuid, ancs_service_uuid, rsp->search_svc_len) != 0)
            break;

        if (rsp->result != HL_ERR_NO_ERROR)
        {
            ble_ancs_enable_rsp_t enable_rsp;
            enable_rsp.conn_idx = rsp->conn_idx;
            enable_rsp.result = BLE_ANCS_ERR_SEARCH_REMOTE_SERVICE_FAILED;
            env->state = BLE_ANCS_STATE_IDLE;
            ble_event_publish(BLE_ANCS_ENABLE_RSP, &enable_rsp, sizeof(ble_ancs_enable_rsp_t));
            break; // Do nothing
        }

        env->svc.hdl_start = rsp->svc->hdl_start;
        env->svc.hdl_end = rsp->svc->hdl_end;
        uint32_t i;
        uint8_t noti_src_check = 0; // This char is madatory
        uint8_t data_src_check = 0;
        uint16_t offset = 0;
        sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
        for (i = 0; i < rsp->svc->char_count; i++)
        {
            if (!memcmp(chara->uuid, ancs_noti_src_uuid, chara->uuid_len))
            {
                LOG_I("noti_uuid received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                if (chara->desc_count != 1)
                    break;
                env->noti_src_char.attr_hdl = chara->attr_hdl;
                env->noti_src_char.value_hdl = chara->pointer_hdl;
                env->noti_src_char.prop = chara->prop;
                env->noti_src_char.cccd_hdl = chara->desc[0].attr_hdl;
                noti_src_check = 1;
            }
            else if (!memcmp(chara->uuid, ancs_controlp_uuid, chara->uuid_len))
            {
                LOG_I("control received, att handle(%x)", chara->attr_hdl);
                env->controlp_char.attr_hdl = chara->attr_hdl;
                env->controlp_char.value_hdl = chara->pointer_hdl;
                env->controlp_char.prop = chara->prop;
            }
            else if (!memcmp(chara->uuid, ancs_data_src_uuid, chara->uuid_len))
            {
                LOG_I("data src received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                if (chara->desc_count != 1)
                    break;
                env->data_src_char.attr_hdl = chara->attr_hdl;
                env->data_src_char.value_hdl = chara->pointer_hdl;
                env->data_src_char.prop = chara->prop;
                env->data_src_char.cccd_hdl = chara->desc[0].attr_hdl;
                data_src_check = 1;
            }
            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }
        if (noti_src_check != 1 || data_src_check != 1)
        {
            ble_ancs_enable_rsp_t enable_rsp;
            enable_rsp.conn_idx = rsp->conn_idx;
            enable_rsp.result = BLE_ANCS_ERR_SEARCH_REMOTE_SERVICE_FAILED;
            env->state = BLE_ANCS_STATE_IDLE;
            ble_event_publish(BLE_ANCS_ENABLE_RSP, &enable_rsp, sizeof(ble_ancs_enable_rsp_t));
            break;
        }
        //register first
        env->remote_handle = sibles_register_remote_svc(rsp->conn_idx, env->svc.hdl_start, env->svc.hdl_end, ble_ancs_gattc_event_handler);
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
        // To support multi-link but only one for ancs, change the behavior
        //sibles_remote_connected_ind_t *ind = (sibles_remote_connected_ind_t *)data;
        //env->conn_idx = ind->conn_idx;
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        if (env->conn_idx == ind->conn_idx)
            ble_ancs_disconnect_handler(env);
        break;
    }
    case SIBLES_REMOTE_SVC_CHANGE_IND:
    {
        sibles_remote_svc_change_ind_t *ind = (sibles_remote_svc_change_ind_t *)data;
        // LOG_I("ancs svc changed %d %d", ind->start_handle, ind->end_handle);
        // LOG_I("ancs %d, %d, state %d, sh %d, eh %d", ind->conn_idx, env->conn_idx, env->state, env->svc.hdl_start, env->svc.hdl_end);
        if (env->conn_idx == ind->conn_idx)
        {
            if (env->state == BLE_ANCS_STATE_READY || env->state == BLE_ANCS_STATE_IDLE)
            {
                if (ind->start_handle <= env->svc.hdl_start && ind->end_handle >= env->svc.hdl_end)
                {
                    LOG_I("update");
                    if (env->state == BLE_ANCS_STATE_READY)
                    {
                        LOG_I("disconnect");
                        ble_ancs_disconnect_handler(env);
                    }

                    env->conn_idx = ind->conn_idx;
                    ble_ancs_enable(env->conn_idx);
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

BLE_EVENT_REGISTER(ble_ancs_ble_event_handler, NULL);

#if defined(RT_USING_FINSH)&&!defined(LCPU_MEM_OPTIMIZE)
int cmd_ancs(int argc, char *argv[])
{
    ble_ancs_env_t *env = ble_ancs_get_env();
    if (argc >= 1)
    {
        if (strcmp(argv[1], "enable") == 0)
        {
            env->conn_idx = atoi(argv[2]);
            ble_ancs_enable(env->conn_idx);
        }
        else if (strcmp(argv[1], "disable") == 0)
        {
            LOG_I("ANCS DISABLE");
            ble_ancs_disconnect_handler(env);
        }
    }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_ancs, __cmd_ancs, My device information service.);
#endif

#endif // BSP_BLE_ANCS
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
