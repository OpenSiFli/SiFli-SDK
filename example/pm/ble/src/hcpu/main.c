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
#include "flashdb.h"
#include "drv_flash.h"

#ifdef FDB_USING_FILE_MODE
    #include "dfs_posix.h"
#endif /* FDB_USING_FILE_MODE */

#ifndef SF32LB55X
    #include "ble.h"
    #define LOG_TAG "ble_app"
    #include "log.h"
#endif /* SF32LB55X */

#ifdef FDB_USING_FILE_MODE
    int elm_init(void);
#endif /* FDB_USING_FILE_MODE */

static void app_wakeup(void)
{
    uint8_t pin;
#ifdef BSP_USING_BOARD_EC_LB557XXX
    /* PA77 */
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 77);
#elif defined(BSP_USING_BOARD_EH_LB555) || defined(BSP_USING_BOARD_EH_LB555XXX_V2)
    /* PA79 */
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 79);
#elif defined(SF32LB55X)
    /* PA80 */
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 80);
#elif defined(SF32LB56X)
    /* PA51 */
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 51);
#else
    /* PA64 */
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 64);
#endif
    RT_ASSERT(pin >= 0);
    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN0;
    HAL_StatusTypeDef status = HAL_HPAON_EnableWakeupSrc(src, AON_PIN_MODE_LOW);
}

void rc10k_timeout_handler(void *parameter)
{
    HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
}

#ifdef FDB_USING_FILE_MODE
static void init_test_fs(uint32_t flash_base, uint32_t offset, uint32_t size, char *name)
{
    const char *type = "elm";

#if defined(RT_DFS_ELM_DHARA_ENABLED)
    register_nand_device(flash_base, offset, size, name);
#else
    register_nor_device(flash_base, offset, size, name);
#endif

    /* if enable elm, initialize and mount it as soon as possible */
    elm_init();

    /*  try to mount fs first to avoid mkfs each time */
    if (dfs_mount(name, "/", type, 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on %s to root success\n", name);
    }
    else
    {
        rt_kprintf("mount fs on %s to root fail, some test cases could not run\n", name);
    }
}
#endif /* FDB_USING_FILE_MODE */


int main(void)
{
#ifndef SF32LB55X
    uint32_t value;
    int ret;
    app_env_t *env;
#endif /* SF32LB55X */

    app_wakeup();
    //*(volatile uint32_t *)0x4004f000 = 1;
    MODIFY_REG(hwp_pmuc->LXT_CR, PMUC_LXT_CR_BM_Msk, MAKE_REG_VAL(0xF, PMUC_LXT_CR_BM_Msk, PMUC_LXT_CR_BM_Pos));   // Increase current


#ifdef FDB_USING_FILE_MODE
    init_test_fs(FS_REGION_START_ADDR, 0, FS_REGION_SIZE, "mtdfs");
#endif

#ifndef SF32LB55X
    env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();
    env->time_handle  = rt_timer_create("app", app_timeout_handler,  NULL,
                                        rt_tick_from_millisecond(BLE_APP_TIMEOUT_INTERVAL), RT_TIMER_FLAG_SOFT_TIMER);

    rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
    if (value == BLE_POWER_ON_IND)
    {
        env->is_power_on = 1;
        env->conn_para.mtu = 23; /* Default value. */
        ble_app_service_init();
        ble_app_advertising_start();
        LOG_I("receive BLE power on!\r\n");
    }
#endif /* SF32LB55X */

    while (1)
    {
        if (HAL_PMU_LXT_DISABLED())
        {
            rt_thread_mdelay(5 * 60 * 1000); // 5 minutes
            drv_rtc_calculate_delta(0);
        }
        else
        {
            rt_thread_mdelay(400000);
        }
    }
    return RT_EOK;
}

#ifdef SF32LB55X

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
#endif /* SF32LB55X */



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

