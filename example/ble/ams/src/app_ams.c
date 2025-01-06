/**
  ******************************************************************************
  * @file   app_ancs.c
  * @author Sifli software development team
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

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "bf0_sibles.h"
#include "bf0_ble_ams.h"
#include "data_service_subscriber.h"
#include "ams_service.h"

#define LOG_TAG "app_ams"
#include "log.h"

#define APP_ALLOC_CHECK(ptr) \
    if (!ptr) \
        break;

typedef struct
{
    datac_handle_t ams_handle;

} app_ancs_env_t;


typedef struct
{
    uint8_t *pb_state;
    uint8_t *pb_rate;
    uint8_t *elapsed_time;
} app_ams_pb_info_t;

static app_ancs_env_t g_app_ancs_env;


static app_ancs_env_t *app_ancs_get_env(void)
{
    return &g_app_ancs_env;
}



static void app_ancs_parser_pb_info(app_ams_pb_info_t *info, uint8_t *raw_data)
{
    uint32_t len = 0;
    uint8_t stage = 0;
    info->pb_state = raw_data;

    while (*(raw_data + len) != 0)
    {
        if (*(raw_data + len) == ',')
        {
            *(raw_data + len) = 0;
            if (stage == 0)
                info->pb_rate = raw_data + len + 1;
            if (stage == 1)
            {
                info->elapsed_time = raw_data + len + 1;
                break;
            }

            stage++;
        }
        len++;
    }
}

static void app_ams_player_attribute_display(ble_ams_entity_attr_value_t *value)
{
    if (!value)
    {
        LOG_E("received invalid value!");
        return;
    }
    uint8_t *str = malloc(value->len + 1);
    memcpy(str, value->value, value->len);
    str[value->len] = 0;
    switch (value->attr_id)
    {
    case BLE_AMS_PLAYER_ATTR_ID_NAME:
    {
        LOG_I("Media player name is %s", str);
        break;
    }
    case BLE_AMS_PLAYER_ATTR_ID_PB_INFO:
    {
        app_ams_pb_info_t pb_info;
        app_ancs_parser_pb_info(&pb_info, str);
        LOG_I("Playback state is %s", pb_info.pb_state);
        LOG_I("Playback rate is %s", pb_info.pb_rate);
        LOG_I("Playback elapsed time is %s", pb_info.elapsed_time);
        break;
    }
    case BLE_AMS_PLAYER_ATTR_ID_VOL:
    {
        LOG_I("Volume is %s", str);
        break;
    }
    default:
        break;
    }
    free(str);
}

static void app_ams_queue_attribute_display(ble_ams_entity_attr_value_t *value)
{
    if (!value)
    {
        LOG_E("received invalid value!");
        return;
    }
    uint8_t *str = malloc(value->len + 1);
    memcpy(str, value->value, value->len);
    str[value->len] = 0;
    switch (value->attr_id)
    {
    case BLE_AMS_QUEUE_ATTR_ID_INDEX:
    {
        LOG_I("Queue index is %s", str);
        break;
    }
    case BLE_AMS_QUEUE_ATTR_ID_COUNT:
    {
        LOG_I("Total tracks in queue are %s", str);
        break;
    }
    case BLE_AMS_QUEUE_ATTR_ID_SHUFFLE:
    {
        LOG_I("Shuffle mode is %s", str);
        break;
    }
    case BLE_AMS_QUEUE_ATTR_ID_REPEAT:
    {
        LOG_I("Repeat mode is %s", str);
    }
    default:
        break;
    }
    free(str);

}

static void app_ams_track_attribute_display(ble_ams_entity_attr_value_t *value)
{
    if (!value)
    {
        LOG_E("received invalid value!");
        return;
    }
    uint8_t *str = malloc(value->len + 1);
    memcpy(str, value->value, value->len);
    str[value->len] = 0;
    switch (value->attr_id)
    {
    case BLE_AMS_TRACK_ATTR_ID_ARTIST:
    {
        LOG_I("Artist is %s", str);
        break;
    }
    case BLE_AMS_TRACK_ATTR_ID_ALBUM:
    {
        LOG_I("Album is %s", str);
        break;
    }
    case BLE_AMS_TRACK_ATTR_ID_TILTE:
    {
        LOG_I("Title is %s", str);
        break;
    }
    case BLE_AMS_TRACK_ATTR_ID_DURATION:
    {
        LOG_I("Total duration is %s", str);
    }
    default:
        break;
    }
    free(str);

}


static int app_ams_callback(data_callback_arg_t *arg)
{
    app_ancs_env_t *env = app_ancs_get_env();

    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        RT_ASSERT(arg->data);
        int16_t len = arg->data_len;
        ble_ams_entity_attr_value_t *value = (ble_ams_entity_attr_value_t *)arg->data;

        switch (value->entity_id)
        {
        case BLE_AMS_ENTITY_ID_PLAYER:
        {
            app_ams_player_attribute_display(value);
        }
        break;
        case BLE_AMS_ENTITY_ID_QUEUE:
        {
            app_ams_queue_attribute_display(value);
        }
        break;
        case BLE_AMS_ENTITY_ID_TRACK:
        {
            app_ams_track_attribute_display(value);
        }
        break;
        default:
            break;
        }


    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("Subscrible AMS ret %d", rsp->result);
#if 0
        if (rsp->result == 0)
        {
            ancs_service_config_t config;
            rt_err_t ret;
            config.command = ANCS_SERVICE_SET_ATTRIBUTE_MASK;
            config.data.attr_mask = BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_ALL;
            ret = datac_config(env->ams_handle, sizeof(ancs_service_config_t), (uint8_t *)&config);
            LOG_I("ret %d", ret);
        }
#endif
    }
    return 0;
}


int app_ams_init(void)
{
    app_ancs_env_t *env = app_ancs_get_env();
    env->ams_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != env->ams_handle);
    datac_subscribe(env->ams_handle, "AMS", app_ams_callback, 0);
    return 0;
}

INIT_APP_EXPORT(app_ams_init);

int cmd_app_ams(int argc, char *argv[])
{
    app_ancs_env_t *env = app_ancs_get_env();
    if (argc >= 1)
    {
        if (strcmp(argv[1], "cmd") == 0)
        {
            ams_service_config_t config;
            rt_err_t ret;
            config.command = AMS_SERVICE_SEND_REMOTE_COMMAND;
            config.data.remote_cmd = atoi(argv[2]);
            ret = datac_config(env->ams_handle, sizeof(ams_service_config_t), (uint8_t *)&config);

        }
    }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_app_ams, __cmd_app_ams, My device information service.);



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

