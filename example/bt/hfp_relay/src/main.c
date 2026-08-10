/*
 * SPDX-FileCopyrightText: 2025-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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

typedef struct
{
    uint16_t hf_channel;
    uint16_t ag_channel;
    hfp_cind_status_t cind_status;
    hfp_phone_number_t local_phone_num;
    hfp_phone_call_info_t remote_call;
    hfp_remote_calls_info_t remote_calls;
    uint8_t has_local_phone_num;
    uint8_t has_remote_call;
    uint8_t pending_ag_make_call;
    uint8_t phone_vgs;
    uint8_t phone_vgm;
    uint8_t phone_bsir;
    uint8_t phone_bvra;
} bt_hfp_relay_context_t;

static bt_hfp_relay_context_t g_hfp_relay_ctx =
{
    .hf_channel = 0xffff,
    .ag_channel = 0xffff,
    .cind_status =
    {
        .service_status = 1,
        .call = 0,
        .callsetup = 0,
        .batt_level = 5,
        .signal = 5,
        .roam_status = 0,
        .callheld = 0,
    },
};

#ifdef BT_DEVICE_NAME
    static const char *local_name = BT_DEVICE_NAME;
#else
    static const char *local_name = "sifli_hfp_relay_service";
#endif

uint8_t bt_hfp_ag_set_auto_answer_call_default()
{
    return 0;
}

static void bt_call_start_get_clcc();

static void bt_hfp_relay_reset_cached_call(void)
{
    memset(&g_hfp_relay_ctx.remote_call, 0, sizeof(g_hfp_relay_ctx.remote_call));
    g_hfp_relay_ctx.remote_calls.num_call = 0;
    g_hfp_relay_ctx.remote_calls.calls = &g_hfp_relay_ctx.remote_call;
    g_hfp_relay_ctx.has_remote_call = 0;
}

static void bt_hfp_relay_update_cind_from_all_status(bt_notify_all_call_status *call_status)
{
    g_hfp_relay_ctx.cind_status.call = call_status->call_status;
    g_hfp_relay_ctx.cind_status.callsetup = call_status->callsetup_status;
    g_hfp_relay_ctx.cind_status.callheld = call_status->callheld_status;
    g_hfp_relay_ctx.cind_status.service_status = call_status->service;
    g_hfp_relay_ctx.cind_status.signal = call_status->signal;
    g_hfp_relay_ctx.cind_status.batt_level = call_status->batt_level;
    g_hfp_relay_ctx.cind_status.roam_status = call_status->roam;
}

static void bt_hfp_relay_update_cind_by_indicator(uint8_t type, uint8_t val)
{
    switch (type)
    {
    case HFP_AG_CIND_SERVICE_TYPE:
        g_hfp_relay_ctx.cind_status.service_status = val;
        break;
    case HFP_AG_CIND_CALL_TYPE:
        g_hfp_relay_ctx.cind_status.call = val;
        break;
    case HFP_AG_CIND_CALLSETUP_TYPE:
        g_hfp_relay_ctx.cind_status.callsetup = val;
        break;
    case HFP_AG_CIND_BATT_TYPE:
        g_hfp_relay_ctx.cind_status.batt_level = val;
        break;
    case HFP_AG_CIND_SIGNAL_TYPE:
        g_hfp_relay_ctx.cind_status.signal = val;
        break;
    case HFP_AG_CIND_ROAM_TYPE:
        g_hfp_relay_ctx.cind_status.roam_status = val;
        break;
    case HFP_AG_CIND_CALLHELD_TYPE:
        g_hfp_relay_ctx.cind_status.callheld = val;
        break;
    default:
        break;
    }
}

static void bt_hfp_relay_notify_ag_call_state(void)
{
    uint8_t phone_len = 0;
    hfp_phone_call_info_t *call = &g_hfp_relay_ctx.remote_call;

    if (0xffff == g_hfp_relay_ctx.ag_channel)
    {
        return;
    }

    if (g_hfp_relay_ctx.has_remote_call)
    {
        phone_len = strlen(call->phone_info.phone_number);
        if (phone_len >= PHONE_NUM_LEN)
        {
            phone_len = PHONE_NUM_LEN - 1;
        }
    }

    HFP_CALL_INFO_T *call_info = rt_malloc(sizeof(HFP_CALL_INFO_T) + phone_len);
    RT_ASSERT(call_info);
    memset(call_info, 0, sizeof(HFP_CALL_INFO_T) + phone_len);

    call_info->mux_id = (uint8_t)g_hfp_relay_ctx.ag_channel;
    call_info->num_active = g_hfp_relay_ctx.cind_status.call;
    call_info->num_held = g_hfp_relay_ctx.cind_status.callheld;
    call_info->callsetup_state = g_hfp_relay_ctx.cind_status.callsetup;
    call_info->phone_type = call->phone_info.type ? call->phone_info.type : PHONE_NUMBER_TYPE_UNKNOWN;
    call_info->phone_len = phone_len;
    if (phone_len)
    {
        memcpy(call_info->phone_number, call->phone_info.phone_number, phone_len);
    }

    bt_interface_phone_state_changed(call_info);
    rt_free(call_info);
}

static void bt_hfp_relay_cache_remote_call(bt_notify_clcc_ind_t *clcc_info)
{
    hfp_phone_call_info_t *call = &g_hfp_relay_ctx.remote_call;

    memset(call, 0, sizeof(*call));
    call->call_idx = clcc_info->idx;
    call->call_dir = clcc_info->dir;
    call->call_status = clcc_info->st;
    call->call_mode = clcc_info->mode;
    call->call_mtpty = clcc_info->mpty;
    call->phone_info.type = clcc_info->phone_number_type ? clcc_info->phone_number_type : PHONE_NUMBER_TYPE_UNKNOWN;

    if (clcc_info->number_size)
    {
        uint8_t number_size = clcc_info->number_size;
        if (number_size >= PHONE_NUM_LEN)
        {
            number_size = PHONE_NUM_LEN - 1;
        }
        memcpy(call->phone_info.phone_number, clcc_info->number, number_size);
        call->phone_info.phone_number[number_size] = '\0';
    }

    g_hfp_relay_ctx.remote_calls.num_call = 1;
    g_hfp_relay_ctx.remote_calls.calls = call;
    g_hfp_relay_ctx.has_remote_call = 1;
}

static void bt_hfp_relay_cache_local_phone_num(uint8_t *number, uint16_t number_len)
{
    uint16_t number_size = number_len;

    memset(&g_hfp_relay_ctx.local_phone_num, 0, sizeof(g_hfp_relay_ctx.local_phone_num));
    if (number_size >= PHONE_NUM_LEN)
    {
        number_size = PHONE_NUM_LEN - 1;
    }
    memcpy(g_hfp_relay_ctx.local_phone_num.phone_number, number, number_size);
    g_hfp_relay_ctx.local_phone_num.phone_number[number_size] = '\0';
    g_hfp_relay_ctx.local_phone_num.type = PHONE_NUMBER_TYPE_UNKNOWN;
    g_hfp_relay_ctx.has_local_phone_num = 1;
}

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
        if (0xffff != g_hfp_relay_ctx.ag_channel)
        {
            bt_interface_remote_call_info_res(g_hfp_relay_ctx.ag_channel, &g_hfp_relay_ctx.remote_calls);
        }
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
        if (g_hfp_relay_ctx.pending_ag_make_call && (0xffff != g_hfp_relay_ctx.ag_channel))
        {
            g_hfp_relay_ctx.pending_ag_make_call = 0;
            bt_interface_make_call_res(g_hfp_relay_ctx.ag_channel, res);
        }
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
            LOG_I("HFP HF audio_connected");
            break;
        }
        case BT_NOTIFY_COMMON_SCO_DISCONNECTED:
        {
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
            g_hfp_relay_ctx.hf_channel = profile_info->profile_channel;
            LOG_I("HFP HF connected");
            break;
        }
        case BT_NOTIFY_HF_PROFILE_DISCONNECTED:
        {
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)msg->data;
            if (g_hfp_relay_ctx.hf_channel == profile_info->profile_channel)
            {
                g_hfp_relay_ctx.hf_channel = 0xffff;
                bt_hfp_relay_reset_cached_call();
                memset(&g_hfp_relay_ctx.local_phone_num, 0, sizeof(g_hfp_relay_ctx.local_phone_num));
                g_hfp_relay_ctx.has_local_phone_num = 0;
                g_hfp_relay_ctx.pending_ag_make_call = 0;
            }
            LOG_I("HFP HF disconnected");
            break;
        }
        case BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE:
        {
            //msg->data[0]
            bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
            g_hfp_relay_ctx.phone_bvra = data->payload[0];
            if (data->payload[0])
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
            bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
            if (data->payload[0])
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
            bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
            if (data->payload_len)
            {
                bt_hfp_relay_cache_local_phone_num(data->payload, data->payload_len);
                LOG_I("the remote phone local number:%s", g_hfp_relay_ctx.local_phone_num.phone_number);
            }
            if (0xffff != g_hfp_relay_ctx.ag_channel)
            {
                bt_interface_local_phone_info_res(g_hfp_relay_ctx.ag_channel, &g_hfp_relay_ctx.local_phone_num);
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
                bt_hfp_relay_cache_remote_call(clcc_info);
            }
            break;
        }
        case BT_NOTIFY_HF_VOLUME_CHANGE:
        {
            bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
            g_hfp_relay_ctx.phone_vgs = data->payload[0];

            LOG_I("the remote phone want to change volume be: %d", data->payload[0]);

            if (0xffff != g_hfp_relay_ctx.ag_channel)
            {
                bt_interface_spk_vol_change_req(g_hfp_relay_ctx.ag_channel, g_hfp_relay_ctx.phone_vgs);
            }
            break;
        }
        case BT_NOTIFY_HF_CALL_STATUS_UPDATE:
        {
            bt_notify_all_call_status *call_status = (bt_notify_all_call_status *)msg->data;
            bt_hfp_relay_update_cind_from_all_status(call_status);
            if ((0 == call_status->call_status) && (0 == call_status->callsetup_status) && (0 == call_status->callheld_status))
            {
                bt_hfp_relay_reset_cached_call();
            }
            bt_hfp_relay_notify_ag_call_state();
            LOG_I("the remote phone call_status: %d", call_status->call_status);
            LOG_I("the remote phone callsetup_status: %d", call_status->callsetup_status);
            LOG_I("the remote phone callheld_status: %d", call_status->callheld_status);
            break;
        }
        case BT_NOTIFY_HF_INDICATOR_UPDATE:
        {
            bt_notify_cind_ind_t   *cind_status = (bt_notify_cind_ind_t *)msg->data;
            HFP_IND_INFO_T ind_info;

            bt_hfp_relay_update_cind_by_indicator(cind_status->type, cind_status->val);
            ind_info.ind_type = cind_status->type;
            ind_info.ind_val = cind_status->val;
            if (0xffff != g_hfp_relay_ctx.ag_channel)
            {
                bt_interface_indicator_status_changed(g_hfp_relay_ctx.ag_channel, &ind_info);
            }
            LOG_I("the remote phone call status type:%d, status: %d", cind_status->type, cind_status->val);
            break;
        }
        case BT_NOTIFY_HF_INBAND_RING_STATUS_CHANGE:
        {
            bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
            g_hfp_relay_ctx.phone_bsir = data->payload[0];
            if (0xffff != g_hfp_relay_ctx.ag_channel)
            {
                // AG notify hf earphone
            }
            LOG_I("remote device in-band ring status is %s", data->payload[0] ? "on" : "off");
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
    else if (msg->type == BT_NOTIFY_HFP_AG)
    {
        switch (msg->event_id)
        {
        case BT_NOTIFY_AG_PROFILE_CONNECTED:
        {
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)msg->data;
            g_hfp_relay_ctx.ag_channel = profile_info->profile_channel;
            LOG_I("HFP AG connected");
            break;
        }
        case BT_NOTIFY_AG_PROFILE_DISCONNECTED:
        {
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)msg->data;
            if (g_hfp_relay_ctx.ag_channel == profile_info->profile_channel)
            {
                g_hfp_relay_ctx.ag_channel = 0xffff;
                g_hfp_relay_ctx.pending_ag_make_call = 0;
            }
            LOG_I("HFP AG disconnected");
            break;
        }
        case BT_NOTIFY_AG_MAKE_CALL_REQ:
        {
            bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
            if ((0xffff != g_hfp_relay_ctx.hf_channel) && at_arg->payload_len)
            {
                g_hfp_relay_ctx.pending_ag_make_call = 1;
                bt_interface_hf_out_going_call(at_arg->payload_len, at_arg->payload);
            }
            else
            {
                bt_interface_make_call_res(g_hfp_relay_ctx.ag_channel, BTS2_FAILED);
            }
            break;
        }
        case BT_NOTIFY_AG_ANSWER_CALL_REQ:
        {
            if (0xffff != g_hfp_relay_ctx.hf_channel)
            {
                bt_interface_start_hf_answer_req_send();
            }
            break;
        }
        case BT_NOTIFY_AG_HANGUP_CALL_REQ:
        {
            if (0xffff != g_hfp_relay_ctx.hf_channel)
            {
                bt_interface_handup_call();
            }
            break;
        }
        case BT_NOTIFY_AG_RECV_DTMF_KEY:
        {
            bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
            char key = at_arg->payload_len ? at_arg->payload[0] : 0;
            if ((0xffff != g_hfp_relay_ctx.hf_channel) && key)
            {
                bt_interface_start_dtmf_req_send(key);
            }
            break;
        }
        case BT_NOTIFY_AG_VOLUME_CHANGE:
        {
            bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
            if ((0xffff != g_hfp_relay_ctx.hf_channel) && at_arg->payload_len)
            {
                bt_interface_set_speaker_volume(at_arg->payload[0]);
#if defined(AUDIO_USING_MANAGER)
                audio_server_set_private_volume(AUDIO_TYPE_BT_VOICE, at_arg->payload[0]);
#endif
            }
            break;
        }
        case BT_NOTIFY_AG_GET_INDICATOR_STATUS_REQ:
        {
            bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
            if ((0xffff != g_hfp_relay_ctx.hf_channel) && (0xffff != g_hfp_relay_ctx.ag_channel))
            {
                bt_interface_get_remote_call_status_by_id(g_hfp_relay_ctx.hf_channel);
            }
            else
            {
                bt_interface_get_all_indicator_info_res(at_arg->profile_channel, &g_hfp_relay_ctx.cind_status);
            }
            break;
        }
        case BT_NOTIFY_AG_GET_ALL_REMT_CALLS_INFO_REQ:
        {
            bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
            g_hfp_relay_ctx.remote_calls.calls = &g_hfp_relay_ctx.remote_call;
            g_hfp_relay_ctx.remote_calls.num_call = g_hfp_relay_ctx.has_remote_call ? 1 : 0;
            if (0xffff != g_hfp_relay_ctx.hf_channel)
            {
                bt_call_start_get_clcc();
            }
            else
            {
                bt_interface_remote_call_info_res(at_arg->profile_channel, &g_hfp_relay_ctx.remote_calls);
            }
            break;
        }
        case BT_NOTIFY_AG_GET_LOCAL_PHONE_INFO_REQ:
        {
            bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;

            if (0xffff != g_hfp_relay_ctx.hf_channel)
            {
                bt_interface_get_ph_num();
            }
            else
            {
                bt_interface_local_phone_info_res(at_arg->profile_channel, &g_hfp_relay_ctx.local_phone_num);
            }
            break;
        }
        case BT_NOTIFY_AG_EXTERN_AT_CMD_KEY_REQ:
        {
            bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
            if ((0xffff != g_hfp_relay_ctx.hf_channel) && at_arg->payload_len)
            {
                bt_interface_hfp_set_extern_cmd(at_arg->payload, at_arg->payload_len);
                bt_interface_make_call_res(g_hfp_relay_ctx.ag_channel, BTS2_SUCC);
            }
            else
            {
                bt_interface_make_call_res(g_hfp_relay_ctx.ag_channel, BTS2_FAILED);
            }
            break;
        }
        case BT_NOTIFY_AG_BATTERY_UPDATE:
        {
            bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
            if ((0xffff != g_hfp_relay_ctx.hf_channel) && at_arg->payload_len)
            {
                bt_interface_hf_update_battery(at_arg->payload[0]);
            }

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
            para.dev_cls_mask = 0;
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
            gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
            bt_interface_conn_ext((unsigned char *)&mac, BT_PROFILE_HFP);
        }
        else if (strcmp(cmd, "hfp_ag_connect") == 0)
        {
            bd_addr_t mac;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
            bt_interface_conn_to_source_ext((unsigned char *)&mac, BT_PROFILE_HFP);
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

#if defined(SF32LB52X_58)|| (defined(SF32LB52X) && (defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)))

void lcpu_rom_config_default(void);
uint16_t g_em_offset[HAL_LCPU_CONFIG_EM_BUF_MAX_NUM] =
{
    0x178, 0x178, 0x740, 0x7A0, 0x848, 0x8B8, 0xB38, 0xCE8, 0xE80, 0x1474, 0x14DC,
    0x1AF4, 0x22F4, 0x22F4, 0x22F4, 0x22F4, 0x22F4, 0x22F4, 0x22F4, 0x22F4, 0x25F4,
    0x2614, 0x268C, 0x26DC, 0x27FC, 0x2810, 0x2824, 0x2914, 0x2924, 0x29E4, 0x2A00,
    0x3A10, 0x4E24, 0x5F04,

};

void lcpu_rom_config(void)
{
    lcpu_rom_config_default();
    hal_lcpu_bluetooth_em_config_t em_offset;
    memcpy((void *)em_offset.em_buf, (void *)g_em_offset, HAL_LCPU_CONFIG_EM_BUF_MAX_NUM * 2);
    em_offset.is_valid = 1;
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_EM_BUF, &em_offset, sizeof(hal_lcpu_bluetooth_em_config_t));
    hal_lcpu_bluetooth_act_configt_t act_cfg;
    act_cfg.bt_max_acl = 3;
    act_cfg.bt_max_sco = 2;
    act_cfg.ble_max_iso = 0;
    act_cfg.bit_valid = 1 << 4 | 1 << 1 | 1 << 0;
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_ACT_CFG, &act_cfg, sizeof(hal_lcpu_bluetooth_act_configt_t));

    hal_lcpu_bluetooth_rom_config_t config = {0};
    config.bit_valid |= 1 << 2 | 1 << 13;
    config.lld_prog_delay = 3;
    config.sco_cfg = 2;
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_CONFIG, &config, sizeof(config));
}
#endif


