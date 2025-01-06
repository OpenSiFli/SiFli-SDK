/**
  ******************************************************************************
  * @file   ancs_service.c
  * @author Sifli software development team
  * @brief Source file - Data service for ANCS.
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
#include <rtdef.h>
#include <board.h>
#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_ble_ancs.h"
#include "ancs_service.h"
#include "data_service_provider.h"




#if defined(BSP_BLE_ANCS) && defined(BSP_USING_DATA_SVC)

#define LOG_TAG "ancs_dev"
#include "log.h"

#define ANCS_PARAM_CHECK(ptr) \
    if (!(ptr)) \
    { \
        LOG_I("%d", __LINE__); \
        break; \
    }


typedef struct
{
    uint8_t app_id[ANCS_APP_ID_LEN];
    uint16_t app_name_len;
    uint8_t app_name[ANCS_APP_NAME_LEN];
} ancs_srv_app_id_name_t;


typedef struct
{
    uint8_t evt_id;                 /**< Event ID(@see enum ble_ancs_event_id_t) */
    uint8_t evt_flag;               /**< Event flag(@see enum ble_ancs_event_flag_t). */
    uint8_t cate_id;                /**< Category ID(@see enum ble_ancs_category_id_t). */
    uint8_t cate_count;             /**< Category count. */
    uint32_t noti_uid;              /**< Noti UID. */
    /* Attr buffer */
    uint8_t *app_name;
    uint16_t name_len;
    uint8_t *title;
    uint16_t title_len;
    uint8_t *subtitle;
    uint16_t subtitle_len;
    uint8_t *message;
    uint16_t message_len;
    uint8_t *message_size;
    uint16_t message_size_len;
    uint8_t *date;
    uint16_t date_len;
    // notify buffer; Need optimize here
    ancs_service_noti_attr_t *buffer;
    uint16_t buffer_len;
} ancs_service_notify_t;

typedef struct
{
    rt_slist_t node;
    ancs_service_notify_t value;
} ancs_service_list_t;

typedef struct
{
    uint8_t conn_idx;
    rt_slist_t node;
    struct
    {
        uint8_t ref_count;
        datas_handle_t service;
        uint8_t *data;
        uint32_t data_size;
    } srv;
    struct
    {
        uint16_t attr_mask;
        uint16_t cate_mask;
        uint8_t is_cccd_enable;
    } config;
} ancs_service_env_t;



static ancs_srv_app_id_name_t g_ancs_service_app_pair[ANCS_SERVICE_MAX_APP];

static ancs_service_env_t g_ancs_srv;

static ancs_service_env_t *ancs_service_get_env(void)
{
    return &g_ancs_srv;
}



static ancs_srv_app_id_name_t *ancs_service_get_app_name(uint8_t *app_id, uint16_t app_id_len)
{
    uint32_t i;
    LOG_HEX("ancs_appid", 16, (uint8_t *)app_id, app_id_len);
    for (i = 0; i < ANCS_SERVICE_MAX_APP; i++)
    {
        //LOG_HEX("ancs_service_app_id", 16, (uint8_t *)g_ancs_service_app_pair[i].app_id, app_id_len);
        if (strncmp((char *)g_ancs_service_app_pair[i].app_id, (char *)app_id, app_id_len) == 0)
        {
            //LOG_HEX("ancs_service_app_id", 16, (uint8_t *)g_ancs_service_app_pair[i].app_id, app_id_len);
            return &g_ancs_service_app_pair[i];
        }
    }

    return NULL;
}

// name only deleted when connection lost, so it will add one by one. May change to a ring buffer
static void ancs_service_add_app_name(uint8_t *app_id, uint8_t *app_name, uint16_t *name_len)
{
    uint32_t i;
    if (*name_len > sizeof(g_ancs_service_app_pair[0].app_name))
    {
        *name_len = sizeof(g_ancs_service_app_pair[0].app_name);
    }
    for (i = 0; i < ANCS_SERVICE_MAX_APP; i++)
    {
        if (g_ancs_service_app_pair[i].app_id[0] == 0)
        {
            memcpy(g_ancs_service_app_pair[i].app_name, app_name, *name_len);
            g_ancs_service_app_pair[i].app_name_len = *name_len;
            strncpy((char *)g_ancs_service_app_pair[i].app_id, (char *)app_id, sizeof(g_ancs_service_app_pair[i].app_id));
            break;
        }
        if (strncmp((char *)app_id, (char *)g_ancs_service_app_pair[i].app_id, sizeof(g_ancs_service_app_pair[i].app_id)) == 0)
            break;
    }

}


#define NOTIFY_TAG_HEADER (3)
static uint16_t ancs_service_get_notify_buffer_length(ancs_service_notify_t *value)
{
    uint16_t len = 0;
    if (value->app_name)
        len += NOTIFY_TAG_HEADER + value->name_len;

    if (value->title)
        len += NOTIFY_TAG_HEADER + value->title_len;

    if (value->subtitle)
        len += NOTIFY_TAG_HEADER + value->subtitle_len;

    if (value->message)
        len += NOTIFY_TAG_HEADER + value->message_len;

    if (value->message_size)
        len += NOTIFY_TAG_HEADER + value->message_size_len;

    if (value->date)
        len += NOTIFY_TAG_HEADER + value->date_len;

    RT_ASSERT(len != 0);
    len += sizeof(ancs_service_noti_attr_t);
    return len;
}

// notify data service should use a integrade buffer
// Compose to id + len + buffer
static void ancs_service_notify_app(ancs_service_notify_t *value)
{
    ancs_service_env_t *env = ancs_service_get_env();
    // 9 is from 3 types plus associated len
    value->buffer_len = ancs_service_get_notify_buffer_length(value);
    value->buffer = malloc(value->buffer_len);
    RT_ASSERT(value->buffer);

    uint16_t len = 0;

    value->buffer->evt_id = value->evt_id;
    value->buffer->evt_flag = value->evt_flag;
    value->buffer->cate_id = value->cate_id;
    value->buffer->cate_count = value->cate_count;
    value->buffer->noti_uid = value->noti_uid;
    value->buffer->attr_count = 0;
    uint8_t *data = (uint8_t *)&value->buffer->value[0];
    if (value->app_name)
    {
        data[len++] = BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME;
        data[len++] = (uint8_t)(value->name_len & 0xFF);
        data[len++] = (uint8_t)((value->name_len >> 8) & 0xFF);
        memcpy(&data[len], value->app_name, value->name_len);
        len += value->name_len;
        value->buffer->attr_count++;
        free(value->app_name);
    }

    if (value->title)
    {
        data[len++] = BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE;
        data[len++] = (uint8_t)(value->title_len & 0xFF);
        data[len++] = (uint8_t)((value->title_len >> 8) & 0xFF);
        memcpy(&data[len], value->title, value->title_len);
        len += value->title_len;
        value->buffer->attr_count++;
        free(value->title);
    }

    if (value->message)
    {
        data[len++] = BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE;
        data[len++] = (uint8_t)(value->message_len & 0xFF);
        data[len++] = (uint8_t)((value->message_len >> 8) & 0xFF);
        memcpy(&data[len], value->message, value->message_len);
        len += value->message_len;
        value->buffer->attr_count++;
        free(value->message);
    }

    if (value->subtitle)
    {
        data[len++] = BLE_ANCS_NOTIFICATION_ATTR_ID_SUB_TITLE;
        data[len++] = (uint8_t)(value->subtitle_len & 0xFF);
        data[len++] = (uint8_t)((value->subtitle_len >> 8) & 0xFF);
        memcpy(&data[len], value->subtitle, value->subtitle_len);
        len += value->subtitle_len;
        value->buffer->attr_count++;
        free(value->subtitle);
    }

    if (value->message_size)
    {
        data[len++] = BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE;
        data[len++] = (uint8_t)(value->message_size_len & 0xFF);
        data[len++] = (uint8_t)((value->message_size_len >> 8) & 0xFF);
        memcpy(&data[len], value->message_size, value->message_size_len);
        len += value->message_size_len;
        value->buffer->attr_count++;
        free(value->message_size);
    }

    if (value->date)
    {
        data[len++] = BLE_ANCS_NOTIFICATION_ATTR_ID_DATE;
        data[len++] = (uint8_t)(value->date_len & 0xFF);
        data[len++] = (uint8_t)((value->date_len >> 8) & 0xFF);
        memcpy(&data[len], value->date, value->date_len);
        len += value->date_len;
        value->buffer->attr_count++;
        free(value->date);
    }


    {
        /* Notify data service */
        rt_err_t result = -RT_ERROR;
        RT_ASSERT(value->buffer_len == (len + sizeof(ancs_service_noti_attr_t)));
        if (env->srv.service)
        {
            result = datas_ind_size(env->srv.service, (uint32_t)value->buffer_len);
            RT_ASSERT(RT_EOK == result);
        }
    }
}


static void ancs_service_enable(ancs_service_env_t *env)
{
    uint32_t i;
    uint16_t attr_len[BLE_ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABLE + 1] = {0, ANCS_SERVICE_TITLE_LEN, ANCS_SERVICE_SUBTITLE_LEN, ANCS_SERVICE_MESSAGE_LEN};
    for (i = BLE_ANCS_NOTIFICATION_ATTR_ID_APP_ID; i <= BLE_ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABLE; i++)
        if (env->config.attr_mask & (1 << i))
            ble_ancs_attr_enable(i, 1, attr_len[i]);

    ble_ancs_category_mask_set(env->config.cate_mask);
    ble_ancs_cccd_enable(env->config.is_cccd_enable);
    ble_ancs_enable(env->conn_idx);
}

int ancs_service_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ancs_service_env_t *env = ancs_service_get_env();
    switch (event_id)
    {
    case BLE_GAP_BOND_IND:
    {
        ble_gap_bond_ind_t *evt = (ble_gap_bond_ind_t *)data;
        env->conn_idx = evt->conn_idx;
        LOG_I("bond ind %d, info %d", evt->conn_idx, evt->info);
        if (evt->info == GAPC_PAIRING_SUCCEED)
            ancs_service_enable(env);
        break;
    }
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        LOG_I("encryt ind %d", ind->conn_idx);
        ancs_service_enable(env);
        break;
    }
    case BLE_ANCS_NOTIFICATION_IND:
    {
        ble_ancs_noti_attr_t *notify = (ble_ancs_noti_attr_t *)data;

        LOG_I("received_notify %d, attr_count %d", notify->evt_id, notify->attr_count);
        if (notify->evt_id == BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED &&
                !(notify->evt_flag & BLE_ANCS_EVENT_FLAG_PRE_EXISTING))
        {
            ancs_service_list_t *node = malloc(sizeof(ancs_service_list_t));
            RT_ASSERT(node);
            memset(node, 0, sizeof(ancs_service_list_t));
            node->value.evt_id = notify->evt_id;
            node->value.evt_flag = notify->evt_flag;
            node->value.cate_id = notify->cate_id;
            node->value.cate_count = notify->cate_count;
            node->value.noti_uid = notify->noti_uid;
            ble_ancs_attr_value_t *value = notify->value;
            ancs_srv_app_id_name_t *app_id_name;
            uint32_t i;
            for (i = 0; i < notify->attr_count; i++)
            {
                if (value->len == 0)
                {
                    value = (ble_ancs_attr_value_t *)((uint8_t *)value + sizeof(ble_ancs_attr_value_t));
                    continue;
                }

                if (value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE)
                {
                    node->value.title = malloc(value->len);
                    RT_ASSERT(node->value.title);
                    node->value.title_len = value->len;
                    memcpy(node->value.title, value->data, value->len);
                }
                else if (value->attr_id  == BLE_ANCS_NOTIFICATION_ATTR_ID_APP_ID)
                {
                    app_id_name = ancs_service_get_app_name(value->data, value->len);
                    if (app_id_name == NULL)
                    {
                        uint8_t app_id[ANCS_APP_ID_LEN] = {0};

                        RT_ASSERT(value->len <= ANCS_APP_ID_LEN);
                        memcpy(app_id, value->data, value->len);
                        // app id in command should be NULL terminated
                        ble_ancs_get_app_attr(app_id, value->len + 1);
                    }
                    else
                    {
                        node->value.app_name = malloc(app_id_name->app_name_len);
                        RT_ASSERT(node->value.app_name);
                        node->value.name_len = app_id_name->app_name_len;
                        memcpy(node->value.app_name, app_id_name->app_name, app_id_name->app_name_len);
                    }
                }
                else if (value->attr_id  == BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE)
                {
                    node->value.message = malloc(value->len);
                    RT_ASSERT(node->value.message);
                    node->value.message_len = value->len;
                    memcpy(node->value.message, value->data, value->len);
                }
                else if (value->attr_id  == BLE_ANCS_NOTIFICATION_ATTR_ID_SUB_TITLE)
                {
                    node->value.subtitle = malloc(value->len);
                    RT_ASSERT(node->value.subtitle);
                    node->value.subtitle_len = value->len;
                    memcpy(node->value.subtitle, value->data, value->len);
                }
                else if (value->attr_id  == BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE)
                {
                    node->value.message_size = malloc(value->len);
                    RT_ASSERT(node->value.message_size);
                    node->value.message_size_len = value->len;
                    memcpy(node->value.message_size, value->data, value->len);
                }
                else if (value->attr_id  == BLE_ANCS_NOTIFICATION_ATTR_ID_DATE)
                {
                    node->value.date = malloc(value->len);
                    RT_ASSERT(node->value.date);
                    node->value.date_len = value->len;
                    memcpy(node->value.date, value->data, value->len);
                }
                value = (ble_ancs_attr_value_t *)((uint8_t *)value + sizeof(ble_ancs_attr_value_t) + value->len);
            }
            rt_slist_append(&env->node, (rt_slist_t *)node);
            // 0x20 is a speicial case that spec not defined this value. From currently case, once
            // 0x20 is existed in the evt_flags, get attribute with app_id will cause IOS won't send
            // notification any more. So this is a workaround.
            //if (node->value.app_name || (node->value.evt_flag & 0x20))
            if (node->value.app_name)
                ancs_service_notify_app(&node->value);

            break;
        }
        break;
    }
    case BLE_ANCS_GET_APP_ATTR_RSP:
    {
        ble_ancs_get_app_attr_rsp_t *rsp = (ble_ancs_get_app_attr_rsp_t *)data;
        ble_ancs_attr_value_t value_tmp;
        rt_slist_t *list;
        RT_ASSERT(rsp->value->attr_id == BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME);
        if (0 == rsp->value->len)
        {
            rsp->value = &value_tmp;
            if (rsp->app_id_len > ANCS_APP_NAME_LEN)
            {
                rsp->value->len = ANCS_APP_NAME_LEN;
            }
            else
            {
                rsp->value->len = rsp->app_id_len;
            }

            memcpy(rsp->value->data, rsp->app_id, rsp->value->len);
        }
        ancs_service_add_app_name(rsp->app_id, rsp->value->data, &rsp->value->len);
        for (list = env->node.next; list != NULL; list = list->next)
        {
            // find the first NULL app_name item
            if (((ancs_service_list_t *)list)->value.app_name == NULL)
            {
                ((ancs_service_list_t *)list)->value.app_name = malloc(rsp->value->len);
                RT_ASSERT(((ancs_service_list_t *)list)->value.app_name);
                memcpy(((ancs_service_list_t *)list)->value.app_name, rsp->value->data, rsp->value->len);
                ((ancs_service_list_t *)list)->value.name_len = rsp->value->len;
                ancs_service_notify_app(&((ancs_service_list_t *)list)->value);
                break;
            }
        }
        break;
    }
    default:
        break;

    }

    return 0;
}


BLE_EVENT_REGISTER(ancs_service_event_handler, NULL);



static int32_t ancs_service_subscribe(datas_handle_t service)
{
    ancs_service_env_t *env = ancs_service_get_env();

    env->srv.ref_count++;
    return 0;
}

static int32_t ancs_service_unsubscribe(datas_handle_t service)
{
    ancs_service_env_t *env = ancs_service_get_env();

    if (env->srv.ref_count > 0)
        env->srv.ref_count--;
    return 0;
}

static int32_t ancs_service_config(datas_handle_t service, void *config)
{
    ancs_service_env_t *env = ancs_service_get_env();
    ancs_service_config_t *setting = (ancs_service_config_t *)config;
    switch (setting->command)
    {
    case ANCS_SERVICE_SET_ATTRIBUTE_MASK:
    {
        env->config.attr_mask = setting->data.attr_mask;
        break;
    }
    case ANCS_SERVICE_SET_CATEGORY_MASK:
    {
        env->config.cate_mask = setting->data.cate_mask;
        break;
    }
    case ANCS_SERVICE_PERFORM_NOTIFY_ACTION:
    {
        // ANCS should already found
        ble_ancs_perform_notification_action(setting->data.action.uid, setting->data.action.act_id);
        break;
    }
    case ANCS_SERVICE_ENABLE_CCCD:
    {
        break;
    }
    default:
        break;

    }
    return 0;
}

static uint32_t ancs_service_data_notify(datas_handle_t service, uint32_t data_size)
{
    ancs_service_env_t *env = ancs_service_get_env();
    uint32_t notify_len = 0;

    do
    {

        ANCS_PARAM_CHECK(data_size > 0);

        //ANCS_PARAM_CHECK(NULL == env->srv.data);

        ancs_service_list_t *dev_list = (ancs_service_list_t *)rt_slist_first(&env->node);
        ANCS_PARAM_CHECK(dev_list != NULL);

        rt_slist_remove(&env->node, (rt_slist_t *)dev_list);

        if (!dev_list->value.buffer)
        {
            free(dev_list);
            break;
        }

#if 0
        if (dev_list->value.buffer_len != (uint16_t)data_size)
        {
            LOG_E("notify size is wrongly!");
            free(dev_list->value.buffer);
            free(dev_list);
            break;
        }
#endif
        datas_push_data_to_client(service, data_size, (uint8_t *)dev_list->value.buffer);
        notify_len = data_size;

        free(dev_list->value.buffer);
        free(dev_list);

    }
    while (0);

    return notify_len;
}




static int32_t ancs_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{

    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        ancs_service_subscribe(service);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        ancs_service_unsubscribe(service);
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        int32_t result = ancs_service_config(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_TX_REQ:
    {
        break;
    }
    case MSG_SERVICE_RX_REQ:
    {
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));

        RT_ASSERT(data_ind);

        uint32_t notify_len = ancs_service_data_notify(service, data_ind->len);
        if (!notify_len)
            LOG_E("ANCS notify failed !");

        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}


static data_service_config_t ancs_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .msg_handler = ancs_service_msg_handler,
};



int ancs_service_register(void)
{
    ancs_service_env_t *env = ancs_service_get_env();
    rt_slist_init(&env->node);
    env->config.attr_mask = ANCS_SERVICE_DEFAULT_ATTR_MASK;
    env->config.cate_mask = ANCS_SERVICE_DEFAULT_CATE_MASK;
    env->config.is_cccd_enable = 1;
    env->srv.service = datas_register("ANCS", &ancs_service_cb);
    rt_slist_init(&env->node);

    return 0;
}

INIT_COMPONENT_EXPORT(ancs_service_register);


#endif /* BSP_BLE_ANCS  */

