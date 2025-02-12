/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2025 - 2025,  Sifli Technology
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
#include "bf0_hal.h"

/* Common functions for RT-Thread based platform -----------------------------------------------*/
#include "drv_io.h"
/**
  * @brief  Initialize board default configuration.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
    //__asm("B .");        /*For debugging purpose*/
    BSP_IO_Init();
}

/* User code start from here --------------------------------------------------------*/
#include "bf0_sibles.h"
#include "bts2_app_inc.h"
#include "bt_connection_manager.h"
#include "ulog.h"
#include "drv_flash.h"
#if defined(AUDIO_USING_MANAGER)
    #include "audio_server.h"
#endif
/* Common functions for RT-Thread based platform -----------------------------------------------*/

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#endif

#define FS_ROOT "root"

#define BT_APP_READY  1

typedef struct
{
    U16             connected_profile;
    BTS2S_BD_ADDR   bd_addr;
} bt_app_t;


typedef struct
{
    uint16_t            type;
    uint16_t            event_id;
    uint16_t            data_len;
    uint8_t             *data;
} bt_app_notify_data_t;

typedef enum
{
    CLCC_STATUS_START = 0x00,
    CLCC_STATUS_IN_PROGRESS,
    CLCC_STATUS_COMPLETE,
} clcc_status_t;

static rt_mq_t g_bt_hfp_service_queue;
static struct rt_thread g_bt_hfp_service_thread;
uint8_t bt_hfp_service_thread_stack[3072];
static rt_timer_t  clcc_timer_hdl = NULL;
uint8_t g_clcc_process_status = CLCC_STATUS_COMPLETE;

#ifdef BT_DEVICE_NAME
    static const char *local_name = BT_DEVICE_NAME;
#else
    static const char *local_name = "sifli_hfp_service";
#endif

static void bt_call_start_get_clcc();

/** Mount file system if using NAND, as BT NVDS is save in file*/
#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS) && !defined(ZBT)
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#define NAND_MTD_NAME    "root"
int mnt_init(void)
{
    //TODO: how to get base address
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000), FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (0xFC000000)), FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif
static void bt_sifli_clcc_timeout(void *parameter)
{
    if (CLCC_STATUS_START == g_clcc_process_status)
    {
        bt_interface_get_remote_ph_num();
    }
    else
    {
        LOG_I("%s:start clcc get again!", __func__);
        bt_call_start_get_clcc();
    }
    return;
}

static void bt_call_start_get_clcc()
{
    if (NULL == clcc_timer_hdl)
    {
        clcc_timer_hdl = rt_timer_create("bt_clcc", bt_sifli_clcc_timeout, &g_clcc_process_status,
                                         rt_tick_from_millisecond(100), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }

    RT_ASSERT(clcc_timer_hdl);
    if (CLCC_STATUS_COMPLETE == g_clcc_process_status)
    {
        g_clcc_process_status = CLCC_STATUS_START;
    }
    rt_timer_stop(clcc_timer_hdl);
    rt_timer_start(clcc_timer_hdl);
    return;
}

static void bt_app_inquiry_ind_handler(bt_notify_remote_device_info_t *info)
{
    LOG_I("device %s searched", info->bt_name);
    LOG_I("device COD is %x, addr is %02x:%02x:%02x:%02x:%02x:%02x", info->dev_cls, info->mac.addr[0],
          info->mac.addr[1], info->mac.addr[2],
          info->mac.addr[3], info->mac.addr[4],
          info->mac.addr[5]);
}

static int bt_app_interface_event_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    rt_err_t err;
    bt_app_notify_data_t *msg;
    msg = rt_malloc(sizeof(bt_app_notify_data_t) + data_len);
    RT_ASSERT(msg);
    msg->type = type;
    msg->event_id = event_id;
    msg->data_len = data_len;
    msg->data = (uint8_t *)((uint8_t *)msg + sizeof(bt_app_notify_data_t));
    if (NULL != data)
    {
        memcpy(msg->data, data, data_len);
    }
    err = rt_mq_send(g_bt_hfp_service_queue, &msg, sizeof(msg));

    return err;
}

static void bt_sifli_notify_hdl_at_cmd(uint8_t cmd_id, uint8_t res)
{
    switch (cmd_id)
    {
    case HFP_HF_AT_CIND_STATUS:
    {
        LOG_I("get remote all call status complete %d", res);
        break;
    }
    case HFP_HF_AT_CHLD_CMD:
    {
        LOG_I("control remote three_waiting call complete %d", res);
        break;
    }
    case HFP_HF_AT_CLIP:
    {
        LOG_I("open remote incoming call notify complete %d", res);
        break;
    }
    case HFP_HF_AT_CCWA:
    {
        LOG_I("open remote the second incoming call notify complete %d", res);
        break;
    }
    case HFP_HF_AT_BVRA:
    {
        LOG_I("start / stop phone voice recognition complete %d", res);
        break;
    }
    case HFP_HF_AT_CLCC:
    {
        LOG_I("get remote all call information complete %d", res);
        g_clcc_process_status = CLCC_STATUS_COMPLETE;
        break;
    }
    case HFP_HF_AT_ATA:
    {
        LOG_I("answer a call complete %d", res);
        break;
    }
    case HFP_HF_AT_CHUP:
    {
        LOG_I("hangup a call complete %d", res);
        break;
    }
    case HFP_HF_AT_ATD:
    {
        LOG_I("make a call complete %d", res);
        break;
    }
    case HFP_HF_AT_BLDN:
    {
        LOG_I("make a callback complete %d", res);
        break;
    }
    case HFP_HF_AT_VTS:
    {
        LOG_I("send a DTMF key complete %d", res);
        break;
    }
    case HFP_HF_AT_VGS:
    {
        LOG_I("change volume value complete %d", res);
        break;
    }
    case HFP_HF_AT_BCC:
    {
        break;
    }
    case HFP_HF_AT_CNUM:
    {
        LOG_I("get remote local phone number complete %d", res);
        break;
    }
    case HFP_HF_AT_BATT_UPDATE:
    {
        LOG_I("update local battery level complete %d", res);
        break;
    }
    default:
        break;
    }
}

static int bt_hfp_service_notify_event_handle(bt_app_notify_data_t *msg)
{
    if (msg->type == BT_NOTIFY_COMMON)
    {
        switch (msg->event_id)
        {
        case BT_NOTIFY_COMMON_BT_STACK_READY:
        {
            // Update Bluetooth name
            // BT ON
            bt_interface_set_local_name(strlen(local_name), (void *)local_name);
            break;
        }
        case BT_NOTIFY_COMMON_DISCOVER_IND:
        {
            bt_app_inquiry_ind_handler((bt_notify_remote_device_info_t *)msg->data);
            break;
        }
        case BT_NOTIFY_COMMON_INQUIRY_CMP:
        {
            LOG_I("Inquiry completed");
            break;
        }
        case BT_NOTIFY_COMMON_SCO_CONNECTED:
        {
            //handle hf sco conencted msg(bt_notify_device_sco_info_t *)
            LOG_I("HFP HF audio_connected");
            break;
        }
        case BT_NOTIFY_COMMON_SCO_DISCONNECTED:
        {
            //handle sco disconneted msg(bt_notify_device_sco_info_t *)
            LOG_I("HFP HF audio_disconnected");
            break;
        }
        default:
            break;
        }
    }
    else if (msg->type == BT_NOTIFY_HFP_HF)
    {
        switch (msg->event_id)
        {
        case BT_NOTIFY_HF_PROFILE_CONNECTED:
        {
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)msg->data;
            LOG_I("HFP HF connected");
            break;
        }
        case BT_NOTIFY_HF_PROFILE_DISCONNECTED:
        {
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)msg->data;
            LOG_I("HFP HF disconnected");
            break;
        }
        case BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE:
        {
            //msg->data[0]
            if (msg->data[0])
            {
                LOG_I("remote device support voice recognition");
            }
            else
            {
                LOG_I("remote device dont support voice recognition");
            }
            break;
        }
        case BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE:
        {
            if (msg->data[0])
            {
                LOG_I("remote device voice recognition status is on");
            }
            else
            {
                LOG_I("remote device voice recognition status is off");
            }
            break;
        }
        case BT_NOTIFY_HF_LOCAL_PHONE_NUMBER:
        {
            // msg->data  msg->data_len
            if (msg->data_len)
            {
                LOG_I("the remote phone local number:%s", msg->data);
            }
            break;
        }
        case BT_NOTIFY_HF_REMOTE_CALL_INFO_IND:
        {
            bt_notify_clcc_ind_t *clcc_info = (bt_notify_clcc_ind_t *)msg->data;
            if (msg->data_len)
            {
                g_clcc_process_status = CLCC_STATUS_IN_PROGRESS;
                LOG_I("the remote phone call info phone_number_type:%d", clcc_info->phone_number_type);
                LOG_I("the remote phone call info call_idx:%d", clcc_info->idx);
                LOG_I("the remote phone call info call_direction:%d", clcc_info->dir);
                LOG_I("the remote phone call info call_status:%d", clcc_info->st);
                LOG_I("the remote phone call info call_mode:%d", clcc_info->mode);
                LOG_I("the remote phone call info call_mpty:%d", clcc_info->mpty);
                LOG_I("the remote phone call info call_number_size:%d", clcc_info->number_size);
                LOG_I("the remote phone call info call_number:%s", clcc_info->number);
            }
            break;
        }
        case BT_NOTIFY_HF_VOLUME_CHANGE:
        {
            LOG_I("the remote phone want to change volume be: %d", msg->data[0]);
            break;
        }
        case BT_NOTIFY_HF_CALL_STATUS_UPDATE:
        {
            bt_notify_all_call_status *call_status = (bt_notify_all_call_status *)msg->data;
            LOG_I("the remote phone call_status: %d", call_status->call_status);
            LOG_I("the remote phone callsetup_status: %d", call_status->callsetup_status);
            LOG_I("the remote phone callheld_status: %d", call_status->callheld_status);
            break;
        }
        case BT_NOTIFY_HF_INDICATOR_UPDATE:
        {
            bt_notify_cind_ind_t   *cind_status = (bt_notify_cind_ind_t *)msg->data;
            bt_call_start_get_clcc();
            LOG_I("the remote phone call status type:%d, status%d: %d", cind_status->type, cind_status->val);
            break;
        }
        case BT_NOTIFY_HF_AT_CMD_CFM:
        {
            bt_notify_at_cmd_cfm_t *at_cmd_cfm = (bt_notify_at_cmd_cfm_t *) msg->data;
            bt_sifli_notify_hdl_at_cmd(at_cmd_cfm->at_cmd_id, at_cmd_cfm->res);
            break;
        }
        }
    }
    return 0;
}

static void bt_hfp_service_thread_entry(void *param)
{
    bt_app_notify_data_t *msg;
    rt_err_t ret;
    while (1)
    {
        ret = rt_mq_recv(g_bt_hfp_service_queue, &msg, sizeof(msg), RT_WAITING_FOREVER);
        RT_ASSERT(RT_EOK == ret);
        bt_hfp_service_notify_event_handle(msg);
        if (msg)
        {
            rt_free(msg);
        }
    }
    return;
}

/**
 * @brief Common initialization.
 */
static rt_err_t hfp_service_thread_init(void)
{
    g_bt_hfp_service_queue = rt_mq_create("bt_hfp_service_queue", sizeof(void *), 30, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_bt_hfp_service_queue);

    rt_err_t err = rt_thread_init(&g_bt_hfp_service_thread, "bt_hfp_service", bt_hfp_service_thread_entry, RT_NULL,
                                  bt_hfp_service_thread_stack, sizeof(bt_hfp_service_thread_stack), RT_THREAD_PRIORITY_MIDDLE, 4);
    RT_ASSERT(RT_EOK == err);

    rt_thread_startup(&g_bt_hfp_service_thread);

    return RT_EOK;
}

int main(void)
{
    LOG_I("[handfree profile]:HF role Example.");

    /// register events for bt interface
    bt_interface_register_bt_event_notify_callback(bt_app_interface_event_handle);

    /* ls files in root. */
    extern void ls(const char *name);
    ls("/");

    /* hfp service thread and message queue initialization. */
    hfp_service_thread_init();

    // Start BT/BLE stack/profile.
#ifdef ZBT
    bt_enable(NULL);
#else
    sifli_ble_enable();
#endif

    // // Update Bluetooth name
    // bt_interface_set_local_name(strlen(local_name), (void *)local_name);

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}



static void help(void)
{
}


__ROM_USED void hfp_cmd(int argc, char **argv)
{
    if (argc < 2)
        help();
    else
    {
        const char *cmd = argv[1];
        if (strcmp(cmd, "c") == 0)
        {
            bt_cm_delete_bonded_devs();
        }
        else if (strcmp(cmd, "start_inquiry") == 0)
        {
            bt_start_inquiry_ex_t para;
            para.max_rsp = MAX_DISCOV_RESS;
            para.max_timeout = 60;
            para.dev_cls_mask = BT_DEVCLS_PHONE;
            bt_interface_start_inquiry_ex(&para);
        }
        else if (strcmp(cmd, "stop_inquiry") == 0)
        {
            bt_interface_stop_inquiry();
        }
        else if (strcmp(cmd, "hfp_connect") == 0)
        {
            bd_addr_t mac;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            bt_interface_conn_ext((unsigned char *)&mac, BT_PROFILE_HFP);
        }
        else if (strcmp(cmd, "hfp_disconnect") == 0)
        {
            bd_addr_t mac;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            bt_interface_disc_ext((unsigned char *)&mac, BT_PROFILE_HFP);
        }
        else if (strcmp(cmd, "local_phone_number") == 0)
        {
            bt_interface_get_ph_num();
        }
        else if (strcmp(cmd, "remote_calls_info") == 0)
        {
            bt_interface_get_remote_ph_num();
        }
        else if (strcmp(cmd, "remote_calls_status") == 0)
        {
            bt_interface_get_remote_call_status();
        }
        else if (strcmp(cmd, "make_call") == 0)
        {
            bt_interface_hf_out_going_call(rt_strlen(argv[2]), argv[2]);
        }
        else if (strcmp(cmd, "call_back") == 0)
        {
            bt_interface_start_last_num_dial_req_send();
        }
        else if (strcmp(cmd, "answer_call") == 0)
        {
            bt_interface_start_hf_answer_req_send();
        }
        else if (strcmp(cmd, "handup_call") == 0)
        {
            bt_interface_handup_call();
        }
        else if (strcmp(cmd, "dtmf_key") == 0)
        {
            char key = argv[2][0];
            bt_interface_start_dtmf_req_send(key);
        }
        else if (strcmp(cmd, "volume_control") == 0)
        {
            bt_interface_set_speaker_volume(atoi(argv[2]));
#if defined(AUDIO_USING_MANAGER)
            audio_server_set_private_volume(AUDIO_TYPE_BT_VOICE, atoi(argv[2]));
#endif
        }
        else if (strcmp(cmd, "voice_recognition") == 0)
        {
            bt_interface_voice_recog(atoi(argv[2]));
        }
        else if (strcmp(cmd, "audio_connect") == 0)
        {
            bt_interface_audio_switch(0);
        }
        else if (strcmp(cmd, "audio_disconnect") == 0)
        {
            bt_interface_audio_switch(1);
        }
        else if (strcmp(cmd, "battery_update") == 0)
        {
            bt_interface_hf_update_battery(atoi(argv[2]));
        }
    }
}
MSH_CMD_EXPORT(hfp_cmd, hfp_cmd command)



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

