/**
  ******************************************************************************
  * @file   main.c
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
#include "data_service_subscriber.h"
#include "ancs_service.h"


static void app_wakeup(void)
{
    uint8_t pin = 3;
#ifdef BSP_USING_BOARD_EC_LB557XXX
    pin = 0;
#endif
    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN0;
    HAL_StatusTypeDef status = HAL_HPAON_EnableWakeupSrc(src, AON_PIN_MODE_LOW);
}

int main(void)
{
    app_wakeup();
    //*(volatile uint32_t *)0x4004f000 = 1;
    while (1)
    {
        if (HAL_PMU_LXT_DISABLED())
        {
            rt_thread_mdelay(5*60*1000); // 5 minutes
            drv_rtc_calculate_delta(0);
        }
        else
        {
            rt_thread_mdelay(200000);
        }
    }
    return RT_EOK;
}


#define LOG_TAG "app_ancs"
#include "log.h"

#define APP_ALLOC_CHECK(ptr) \
    if (!ptr) \
        break;

typedef struct
{
    datac_handle_t ancs_handle;

} app_ancs_env_t;


const char app_cate_str[][20] =
{
    "Others",
    "Incoming call",
    "Missed call",
    "Voice mail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "Health and fitness",
    "Business and fiance",
    "Location",
    "Entertainment"
};

static app_ancs_env_t g_app_ancs_env;


static app_ancs_env_t *app_ancs_get_env(void)
{
    return &g_app_ancs_env;
}


static int app_ancs_callback(data_callback_arg_t *arg)
{
    app_ancs_env_t *env = app_ancs_get_env();

    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        RT_ASSERT(arg->data);
        int16_t len = arg->data_len;
        ancs_service_noti_attr_t *value = (ancs_service_noti_attr_t *)arg->data;
        ble_ancs_attr_value_t *att_value = &value->value[0];
        //rt_print_data((char *)buffer, 0, len);
        LOG_I("Category %s", app_cate_str[value->cate_id]);
        if (value->cate_id == BLE_ANCS_CATEGORY_ID_INCOMING_CALL)
        {
            ancs_service_config_t config;
            rt_err_t ret;
            config.command = ANCS_SERVICE_PERFORM_NOTIFY_ACTION;
            config.data.action.uid = value->noti_uid;
            config.data.action.act_id = BLE_ACTION_ID_NEGATIVE;
            ret = datac_config(env->ancs_handle, sizeof(ancs_service_config_t), (uint8_t *)&config);
            LOG_I("ret %d", ret);
        }
        for (uint32_t i = 0; i < value->attr_count; i++)
        {
            if (att_value->attr_id == BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME)
            {
                uint8_t *app_name = malloc(att_value->len + 1);
                APP_ALLOC_CHECK(app_name);
                memcpy(app_name, att_value->data, att_value->len);
                app_name[att_value->len] = 0;


                LOG_HEX("raw_data", 16, app_name, att_value->len + 1);
                LOG_I("App(%d): %s", att_value->len, app_name);
                free(app_name);
            }
            else if (att_value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE)
            {
                uint8_t *title_name = malloc(att_value->len + 1);
                APP_ALLOC_CHECK(title_name);
                memcpy(title_name, att_value->data, att_value->len);
                title_name[att_value->len] = 0;
                LOG_I("Title: %s", title_name);
                free(title_name);
            }
            else if (att_value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE)
            {
                uint8_t *message = malloc(att_value->len + 1);
                RT_ASSERT(message);
                memcpy(message, att_value->data, att_value->len);
                message[att_value->len] = 0;
                LOG_I("Context: %s", message);
                free(message);
            }
            att_value = (ble_ancs_attr_value_t *)((uint8_t *)att_value + sizeof(ble_ancs_attr_value_t) + att_value->len);
        }
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("Subscrible ANCS ret %d", rsp->result);
        if (rsp->result == 0)
        {
            ancs_service_config_t config;
            rt_err_t ret;
            config.command = ANCS_SERVICE_SET_ATTRIBUTE_MASK;
            config.data.attr_mask = BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_ALL;
            ret = datac_config(env->ancs_handle, sizeof(ancs_service_config_t), (uint8_t *)&config);
            LOG_I("ret %d", ret);
        }
    }
    return 0;
}


int app_ancs_init(void)
{
    app_ancs_env_t *env = app_ancs_get_env();
    env->ancs_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != env->ancs_handle);
    datac_subscribe(env->ancs_handle, "ANCS", app_ancs_callback, 0);
    return 0;
}

INIT_APP_EXPORT(app_ancs_init);




/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

