/*
 * SPDX-FileCopyrightText: 2024-2025 SiFli Technologies(Nanjing) Co., Ltd
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
#include "bt_hfp_relay_mgr.h"
#ifdef     AUDIO_USING_MANAGER
    #include "audio_server.h"
#endif


#ifdef ZBT
    #include "zephyr/bluetooth/bluetooth.h"
#endif

#define BT_APP_READY  1
#define TRANSFOR_BUFFER_SIZE  (SINK_DATA_LIST_MAX_THRESHOLD * AV_MTU_SIZE)

#define AVRCP_PLAY_STATUS     (0)
#define AVRCP_PAUSE_STATUS    (1)

#define AVRCP_PREVIOUS_CMD    (0)
#define AVRCP_NEXT_CMD        (1)

#define CO_BIT(pos) (1UL<<(pos))

typedef struct
{
    uint8_t is_abs_enabled;
} bt_app_t;

static bt_app_t g_bt_app_env;
static rt_mailbox_t g_bt_app_mb;

static struct rt_ringbuffer32 transfor_rb;
static rt_uint8_t mempool[TRANSFOR_BUFFER_SIZE];

static bt_app_t *bt_app_get_env(void)
{
    return &g_bt_app_env;
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

static U8 need_trigger = 0;
static int bt_app_interface_event_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    bt_app_t *env = bt_app_get_env();

    if (type == BT_NOTIFY_COMMON)
    {
        switch (event_id)
        {
        case BT_NOTIFY_COMMON_BT_STACK_READY:
        {
            rt_mb_send(g_bt_app_mb, BT_APP_READY);
        }
        break;
        case BT_NOTIFY_COMMON_SCO_CONNECTED:
        {
            LOG_I("HFP HF audio_connected");
            bt_notify_device_sco_info_t *sco_info = (bt_notify_device_sco_info_t *)data;
            bt_hfp_relay_handle_sco_event(event_id, sco_info);
            break;
        }
        case BT_NOTIFY_COMMON_SCO_DISCONNECTED:
        {
            LOG_I("HFP HF audio_disconnected");
            bt_notify_device_sco_info_t *sco_info = (bt_notify_device_sco_info_t *)data;
            bt_hfp_relay_handle_sco_event(event_id, sco_info);
            break;
        }
        default:
            break;
        }
    }
    else if (type == BT_NOTIFY_A2DP)
    {
        switch (event_id)
        {
        case BT_NOTIFY_A2DP_PROFILE_CONNECTED:
        {
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
            if (profile_info->res == BTS2_SUCC)
            {
                if (profile_info->profile_role == AV_AUDIO_SRC)
                {
                    LOG_I("A2DP source connected");

                    bt_cm_device_manager_t *env = bt_cm_get_env();
                    BTS2S_BD_ADDR   bd_addr;

                    bt_addr_convert_to_bts((bd_addr_t *)&profile_info->mac, &bd_addr);

                    bt_cm_dev_acl_info_t *conn = bt_cm_get_conn_by_addr(env, &bd_addr);

                    if (conn && conn->info.role == BT_LINK_SLAVE)
                    {
                        bt_wr_link_policy(&bd_addr, 0x05);
                        bt_send_switch_role(&bd_addr, 0);
                    }

                    bt_interface_conn_to_source_ext((unsigned char *)&profile_info->mac, BT_PROFILE_HFP);

                    uint8_t con_idx;
                    bts2s_av_inst_data *inst = bt_av_get_inst_data();
                    uint8_t should_trigger = 0;
                    //!If the sink is already playing when the source is connected,
                    //!it needs to be played by trigger source
                    for (con_idx = 0; con_idx < MAX_CONNS; con_idx++)
                    {
                        if (inst->con[con_idx].st == avconned_streaming && inst->con[con_idx].cfg == AV_AUDIO_SNK)
                        {
                            should_trigger = 1;
                            break;
                        }
                    }

                    if (should_trigger)
                    {
                        for (con_idx = 0; con_idx < MAX_CONNS; con_idx++)
                        {
                            if (inst->con[con_idx].st == avconned_open && inst->con[con_idx].cfg == AV_AUDIO_SRC)
                            {
                                if (bt_av_get_src_streaming_number() == 0)
                                    need_trigger = 1;
                                bt_av_start_stream(con_idx);
                            }
                        }
                    }
                }
                else if (profile_info->profile_role == AV_AUDIO_SNK)
                {
                    LOG_I("A2DP sink connected");
                }
            }
            else
            {
                LOG_I("A2DP connect error!");
            }
        }
        break;
        case BT_NOTIFY_A2DP_PROFILE_DISCONNECTED:
        {
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
            if (profile_info->profile_role == AV_AUDIO_SRC)
            {
                LOG_I("A2DP source disconnected %d", profile_info->res);
                //!all source disconnect should reset ringbuffer
                if (bt_av_get_src_streaming_number() == 0)
                {
                    rt_ringbuffer32_reset(&transfor_rb);
                }
            }
            else if (profile_info->profile_role == AV_AUDIO_SNK)
            {
                LOG_I("A2DP sink disconnected %d", profile_info->res);

                uint8_t con_idx;
                bts2s_av_inst_data *inst = bt_av_get_inst_data();

                //!If the source is already playing when the sink is disconnected,
                //!it needs to be paused by trigger source
                for (con_idx = 0; con_idx < MAX_CONNS; con_idx++)
                {
                    if (inst->con[con_idx].st == avconned_streaming && inst->con[con_idx].cfg == AV_AUDIO_SRC)
                    {
                        bt_av_suspend_stream(con_idx);
                    }
                }
            }
        }
        break;
        case BT_NOTIFY_A2DP_START_IND:
        {
            uint8_t stream_hdl = *(uint8_t *)data;
            uint8_t con_idx;
            bts2s_av_inst_data *inst = bt_av_get_inst_data();

            for (con_idx = 0; con_idx < MAX_CONNS; con_idx++)
            {
                if (inst->con[con_idx].st == avconned_open && inst->con[con_idx].cfg == AV_AUDIO_SRC)
                {
                    if (bt_av_get_src_streaming_number() == 0)
                        need_trigger = 1;
                    bt_av_start_stream(con_idx);
                }
            }
        }
        break;
        case BT_NOTIFY_A2DP_START_CFM:
        {
            LOG_I("a2dp transfor start...");
        }
        break;
        case BT_NOTIFY_A2DP_SUSPEND_IND:
        {
            uint8_t stream_hdl = *(uint8_t *)data;
            uint8_t con_idx;
            bts2s_av_inst_data *inst = bt_av_get_inst_data();

            for (con_idx = 0; con_idx < MAX_CONNS; con_idx++)
            {
                if (inst->con[con_idx].st == avconned_streaming && inst->con[con_idx].cfg == AV_AUDIO_SRC)
                {
                    LOG_I("a2dp transfor suspend...");
                    bt_av_suspend_stream(con_idx);
                    break;
                }
            }
        }
        break;
        case BT_NOTIFY_A2DP_SUSPEND_CFM:
        {
            uint8_t con_idx;
            bts2s_av_inst_data *inst = bt_av_get_inst_data();

            for (con_idx = 0; con_idx < MAX_CONNS; con_idx++)
            {
                if (inst->con[con_idx].st == avconned_streaming && inst->con[con_idx].cfg == AV_AUDIO_SRC)
                {
                    LOG_I("a2dp transfor suspend...");
                    bt_av_suspend_stream(con_idx);
                    break;
                }
            }
        }
        break;
        case BT_NOTIFY_A2DP_DATA_IND:
        {
            bt_notify_a2dp_data_t *media_data = (bt_notify_a2dp_data_t *)data;
            uint8_t con_idx;
            bts2s_av_inst_data *inst = bt_av_get_inst_data();

            con_idx = bt_av_get_idx_from_shdl(inst, media_data->stream_handle);

            // LOG_I("a2dp data len = %d\n",media_data->data_length);

            if (bt_av_get_src_streaming_number() > 0)
            {
                if (need_trigger == 1 && \
                        rt_ringbuffer32_data_len(&transfor_rb) >= AV_MTU_SIZE * SINK_DATA_LIST_START_THRESHOLD && \
                        (bt_av_get_src_streaming_number() == bt_av_get_src_connected_number()))
                {
                    need_trigger = 0;
                    LOG_I("a2dp transfor data start...");
#ifdef CFG_AV_SHARING
                    bt_avsrc_sharing(&transfor_rb);
#endif
                }

                if (rt_ringbuffer32_space_len(&transfor_rb) < media_data->data_length + 2)
                {
                    LOG_I("a2dp transfor buffer overflow");
                }
                else
                {
                    uint16_t pkt_len = media_data->data_length;
                    rt_ringbuffer32_put(&transfor_rb, (uint8_t *)&pkt_len, sizeof(pkt_len));
                    rt_size_t len = rt_ringbuffer32_put(&transfor_rb, media_data->data, media_data->data_length);

                    if (len != media_data->data_length)
                    {
                        LOG_I("a2dp transfor ringbuffer error...");
                    }
                }
            }
            else
            {
                static U16 debug_count = 0;
                if (debug_count++ % 2000 == 0)
                    LOG_I("a2dp transfor wait start...");
            }
            bfree(media_data->data);
        }
        break;
        }
    }
    else if (type == BT_NOTIFY_AVRCP)
    {
        switch (event_id)
        {
        case BT_NOTIFY_AVRCP_PROFILE_CONNECTED:
        {
            LOG_I("AVRCP connected");
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
            bt_interface_set_avrcp_role_ext(&profile_info->mac, AVRCP_CT);
        }
        break;
        case BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED:
        {
            bt_notify_profile_state_info_t *info = (bt_notify_profile_state_info_t *)data;
            env->is_abs_enabled = 0;
            LOG_I("AVRCP disconnected %d", info->res);
        }
        break;
        case BT_NOTIFY_AVRCP_VOLUME_CHANGED_REGISTER:
        {
            env->is_abs_enabled = 1;
        }
        break;
        case BT_NOTIFY_AVRCP_ABSOLUTE_VOLUME:
        {
            uint8_t *volume = (uint8_t *)data;
#ifdef AUDIO_USING_MANAGER
            uint8_t local_vol = bt_interface_avrcp_abs_vol_2_local_vol(*volume, audio_server_get_max_volume());
            audio_server_set_private_volume(AUDIO_TYPE_BT_MUSIC, local_vol);
#endif
        }
        break;
        case BT_NOTIFY_AVRCP_PLAY_STATUS:
        {
            uint8_t *play_status_notify = (uint8_t *)data;
            bts2s_av_inst_data *inst = bt_av_get_inst_data();
            int con_idx;

            con_idx = bt_avsrc_get_plyback_conn(inst);

            if (con_idx == - 1)
                break;

            bt_notify_device_mac_t bd_addr_c;
            bt_addr_convert_to_general(&inst->con[1 - con_idx].av_rmt_addr, (bd_addr_t *)&bd_addr_c);

            switch (*play_status_notify)
            {
            case AVRCP_PLAY_STATUS:
            {
                //play
                bt_interface_avrcp_play_ext(&bd_addr_c);
            }
            break;
            case AVRCP_PAUSE_STATUS:
            {
                //pause or stop
                bt_interface_avrcp_pause_ext(&bd_addr_c);
            }
            break;
            default:
                break;
            }
        }
        break;
        case BT_NOTIFY_AVRCP_TRACK_CHANGE_STATUS:
        {
            uint8_t *track_change = (uint8_t *)data;
            bts2s_av_inst_data *inst = bt_av_get_inst_data();
            int con_idx;

            con_idx = bt_avsrc_get_plyback_conn(inst);

            if (con_idx == - 1)
                break;

            bt_notify_device_mac_t bd_addr_c;
            bt_addr_convert_to_general(&inst->con[1 - con_idx].av_rmt_addr, (bd_addr_t *)&bd_addr_c);

            switch (*track_change)
            {
            case AVRCP_PREVIOUS_CMD:
            {
                //previous
                bt_interface_avrcp_previous_ext(&bd_addr_c);
            }
            break;
            case AVRCP_NEXT_CMD:
            {
                //next
                bt_interface_avrcp_next_ext(&bd_addr_c);
            }
            break;
            default:
                break;
            }
        }
        break;
        default:
            break;
        }
    }
    else if (type == BT_NOTIFY_HFP_HF)
    {
        bt_hfp_relay_notify_data_t msg;
        msg.type = type;
        msg.event_id = event_id;
        msg.data_len = data_len;
        msg.data = data;
        return bt_hfp_relay_hf_event_handle(&msg);
    }
    else if (type == BT_NOTIFY_HFP_AG)
    {
        bt_hfp_relay_notify_data_t msg;
        msg.type = type;
        msg.event_id = event_id;
        msg.data_len = data_len;
        msg.data = data;
        return bt_hfp_relay_ag_event_handle(&msg);
    }
    return 0;
}

// uint32_t bt_get_class_of_device(void)
// {
//     return 0x040414;
// }


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
#ifdef BT_DEVICE_NAME
    static const char *local_name = BT_DEVICE_NAME;
#else
    static const char *local_name = "sifli_a2dp_transfor";
#endif

int main(void)
{
    uint32_t value;

    //__asm("B .");
    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_bt_app_mb);

    bt_interface_register_bt_event_notify_callback(bt_app_interface_event_handle);

    // Start BT/BLE stack/profile.
#ifdef ZBT
    bt_enable(NULL);
#else
    sifli_ble_enable();
#endif

    // Wait for stack/profile ready.
    if (RT_EOK == rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, 8000) && value == BT_APP_READY)
    {
        LOG_I("BT/BLE stack and profile ready");
        // Update Bluetooth name
        bt_interface_set_local_name(strlen(local_name), (void *)local_name);
    }
    else
        LOG_I("BT/BLE stack and profile init failed");

    rt_ringbuffer32_init(&transfor_rb, mempool, TRANSFOR_BUFFER_SIZE);

    while (1)
    {
        rt_thread_mdelay(15000);
    }
    return 0;
}


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

static void help(void)
{
}

__ROM_USED void a2dp_trans(int argc, char **argv)
{
    if (argc < 2)
        help();
    else
    {
        if (strcmp(argv[1], "c") == 0)
        {
            bt_cm_delete_bonded_devs();
        }
        else if (strcmp(argv[1], "inquiry") == 0)
        {
            if (strcmp(argv[2], "start") == 0)
                bt_interface_start_inquiry();
            else if (strcmp(argv[2], "stop") == 0)
                bt_interface_stop_inquiry();
        }
        else if (strcmp(argv[1], "conn") == 0)
        {
            bd_addr_t mac;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
            bt_interface_conn_to_source_ext((unsigned char *)&mac, BT_PROFILE_A2DP);
        }
        else if (strcmp(argv[1], "set_vol") == 0)
        {
            // 0-127
            uint8_t vol = atoi(argv[2]);
            bt_interface_avrcp_set_absolute_volume_as_tg_role(vol);
        }
    }
}
MSH_CMD_EXPORT(a2dp_trans, a2dp_trans command)



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/