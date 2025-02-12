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
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "audio_server.h"
#include "audio_mp3ctrl.h"
#include "drv_flash.h"

/* Common functions for RT-Thread based platform -----------------------------------------------*/

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#endif

#define FS_ROOT "root"


#ifdef ZBT
    #include "zephyr/bluetooth/bluetooth.h"
#endif

#define BT_APP_READY  1


typedef struct
{
    U16             connected_profile;
    BTS2S_BD_ADDR   bd_addr;
} bt_app_t;

static bt_app_t g_bt_app_env;
static rt_mailbox_t g_bt_app_mb;


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


/* User code start from here --------------------------------------------------------*/
#define MUSIC_FILE_PATH "/test.mp3"

/* Semaphore used to wait aes interrupt. */
/* mp3 handle */
static mp3ctrl_handle g_mp3_handle = NULL;
/* mp3 process thread */
static rt_thread_t g_mp3_proc_thread = NULL;
/* message queue used by mp3 process thread */
static rt_mq_t g_mp3_proc_mq = NULL;

typedef enum
{
    CMD_MP3_PALY = 0,  /* mp3 play */
    CMD_MP3_STOP,      /* mp3 stop */
    CMD_MP3_PAUSE,     /* mp3 pause */
    CMD_MP3_RESUME,    /* mp3 resume */
    CMD_MP3_MAX
} CMD_MP3_E;

typedef struct
{
    uint8_t cmd;  /* see enum CMD_MP3_E. */
    mp3_ioctl_cmd_param_t param;  /* mp3_ioctl_cmd_param_t */
    uint32_t loop;  /*loop times. 0 : play one time. 1 ~ n : play 2 ~ n+1 times. */
} mp3_ctrl_info_t;

/**
 * @brief send msg to mp3 proc thread.
 */
static void send_msg_to_mp3_proc(mp3_ctrl_info_t *info)
{
    rt_err_t err = rt_mq_send(g_mp3_proc_mq, info, sizeof(mp3_ctrl_info_t));
    RT_ASSERT(err == RT_EOK);
}

/**
 * @brief Example for play file.
 * @param file_name music file (mp3) path.
 * @param loop loop times. 0 : play one time. 1 ~ n : play 2 ~ n+1 times.
 *
 * @retval none
 */
void play_file(const char *file_name, uint32_t loop)
{
    rt_kprintf("[LOCAL MUSIC]%s %s\n", __func__, file_name);
    mp3_ctrl_info_t info = {0};

    info.cmd = CMD_MP3_PALY;
    info.loop = loop;
    info.param.filename = file_name;
    info.param.len = -1;

    send_msg_to_mp3_proc(&info);
}

/**
 * @brief Example for stop playing.
 *
 * @retval none
 */
void play_stop(void)
{
    rt_kprintf("[LOCAL MUSIC]%s\n", __func__);
    mp3_ctrl_info_t info = {0};
    info.cmd = CMD_MP3_STOP;
    send_msg_to_mp3_proc(&info);
}

/**
 * @brief Example for pause playing.
 *
 * @retval none
 */
void play_pause(void)
{
    rt_kprintf("[LOCAL MUSIC]%s\n", __func__);
    mp3_ctrl_info_t info = {0};
    info.cmd = CMD_MP3_PAUSE;
    send_msg_to_mp3_proc(&info);
}

/**
 * @brief Example for resume playing.
 *
 * @retval none
 */
void play_resume(void)
{
    rt_kprintf("[LOCAL MUSIC]%s\n", __func__);
    mp3_ctrl_info_t info = {0};
    info.cmd = CMD_MP3_RESUME;
    send_msg_to_mp3_proc(&info);
}

/**
 * @brief callback function for mp3ctrl_open.
 */
static int play_callback_func(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    rt_kprintf("[LOCAL MUSIC]%s cmd %d\n", __func__, cmd);
    switch (cmd)
    {
    case as_callback_cmd_play_to_end:
    {
        /* To close audio client when playing has been completed. */
        play_stop();
        break;
    }
    default:
        break;
    }

    return 0;
}

/**
 * @brief Mp3 process thread entry.
 */
void mp3_proc_thread_entry(void *params)
{
    rt_err_t err = RT_ERROR;
    mp3_ctrl_info_t msg;

    while (1)
    {
        err = rt_mq_recv(g_mp3_proc_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        RT_ASSERT(err == RT_EOK);
        rt_kprintf("[LOCAL MUSIC]RECV msg: cmd %d\n", msg.cmd);
        switch (msg.cmd)
        {
        case CMD_MP3_PALY:
            if (g_mp3_handle)
            {
                /* Close fistly if mp3 is playing. */
                mp3ctrl_close(g_mp3_handle);
            }
            g_mp3_handle = mp3ctrl_open(AUDIO_TYPE_LOCAL_MUSIC,  /* audio type, see enum audio_type_t. */
                                        msg.param.filename,  /* file path */
                                        play_callback_func,  /* play callback function. */
                                        NULL);
            RT_ASSERT(g_mp3_handle);
            /* Set loop times. */
            mp3ctrl_ioctl(g_mp3_handle,   /* handle returned by mp3ctrl_open. */
                          0,              /* cmd = 0, set loop times. */
                          msg.loop);      /* loop times. */
            /* To play. */
            mp3ctrl_play(g_mp3_handle);
            break;

        case CMD_MP3_STOP:
            mp3ctrl_close(g_mp3_handle);
            g_mp3_handle = NULL;
            break;

        case CMD_MP3_PAUSE:
            mp3ctrl_pause(g_mp3_handle);
            break;

        case CMD_MP3_RESUME:
            mp3ctrl_resume(g_mp3_handle);
            break;

        default:
            break;
        }
        rt_kprintf("[LOCAL MUSIC]RECV END.\n");
    }
}


/**
 * @brief Common initialization.
 */
static rt_err_t comm_init(void)
{
    g_mp3_proc_mq = rt_mq_create("mp3_proc_mq", sizeof(mp3_ctrl_info_t), 60, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_mp3_proc_mq);
    g_mp3_proc_thread = rt_thread_create("mp3_proc", mp3_proc_thread_entry, NULL, 2048, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(g_mp3_proc_thread);
    rt_err_t err = rt_thread_startup(g_mp3_proc_thread);
    RT_ASSERT(RT_EOK == err);

    rt_kprintf("[LOCAL MUSIC]%s\n", __func__);

    return RT_EOK;
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
    if (type == BT_NOTIFY_COMMON)
    {
        switch (event_id)
        {
        case BT_NOTIFY_COMMON_BT_STACK_READY:
        {
            rt_mb_send(g_bt_app_mb, BT_APP_READY);
        }
        break;
        case BT_NOTIFY_COMMON_DISCOVER_IND:
        {
            bt_app_inquiry_ind_handler((bt_notify_remote_device_info_t *)data);
        }
        break;
        case BT_NOTIFY_COMMON_INQUIRY_CMP:
        {
            LOG_I("Inquiry completed");
        }
        break;
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
            LOG_I("A2DP connected");
            audio_server_select_public_audio_device(AUDIO_DEVICE_A2DP_SINK);
        }
        break;
        case BT_NOTIFY_A2DP_PROFILE_DISCONNECTED:
        {
            LOG_I("A2DP disconnected");
            audio_server_select_public_audio_device(AUDIO_DEVICE_SPEAKER);
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
            BTS2S_BD_ADDR bd_addr;
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
            bt_addr_convert_to_bts((bd_addr_t *)&profile_info->mac, &bd_addr);
            bt_interface_set_avrcp_role(&bd_addr, AVRCP_TG);
        }
        break;
        case BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED:
        {
            LOG_I("AVRCP disconnected");
        }
        break;
        }
    }

    return 0;
}


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
#ifdef BT_DEVICE_NAME
    static const char *local_name = BT_DEVICE_NAME;
#else
    static const char *local_name = "sifli_music_source";
#endif

int main(void)
{
    uint32_t value;
    rt_kprintf("\n[Music source]Music source Example.\n");

    //__asm("B .");
    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_bt_app_mb);
    /// register events for bt interface
    bt_interface_register_bt_event_notify_callback(bt_app_interface_event_handle);

    /* ls files in root. */
    extern void ls(const char *name);
    ls("/");

    /* mp3 process thread and message queue initialization. */
    comm_init();

    // Start BT/BLE stack/profile.
#ifdef ZBT
    bt_enable(NULL);
#else
    sifli_ble_enable();
#endif

    // Wait for stack/profile ready.
    if (RT_EOK == rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, 8000) && value == BT_APP_READY)
        LOG_I("BT/BLE stack and profile ready");
    else
        LOG_I("BT/BLE stack and profile init failed");

    // Update Bluetooth name
    bt_interface_set_local_name(strlen(local_name), (void *)local_name);


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


__ROM_USED void music(int argc, char **argv)
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
            bt_interface_conn_to_source_ext((unsigned char *)&mac, BT_PROFILE_A2DP);
        }
        else if (strcmp(argv[1], "play_default") == 0)
        {
            /* Play MUSIC_FILE_PATH */
            play_file(MUSIC_FILE_PATH,
                      0    /* 0 : play one time. 1 ~ n : play 2 ~ n+1 times. */
                     );
        }
        else if (strcmp(argv[1], "play") == 0)
        {
            uint8_t loop_time = atoi(argv[2]);
            char *song_name = argv[3];
            play_file(song_name, loop_time);
        }
        else if (strcmp(argv[1], "stop") == 0)
        {
            play_stop();
        }
        else if (strcmp(argv[1], "set_vol") == 0)
        {
            // 0-127
            uint8_t vol = atoi(argv[2]);
            bt_interface_avrcp_set_absolute_volume(vol);
        }
    }
}
MSH_CMD_EXPORT(music, music command)



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

