#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "drv_flash.h"
#include "mp3_mem.h"
#include "mp3_ringbuffer.h"
#include "local_music.h"

/* notify download thread play progress */
extern void mp3_network_get_part_cancel(void);
extern void mp3_dl_read_more(int read_pos);

/* Common functions for RT-Thread based platform -----------------------------------------------*/

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#endif

#define FS_ROOT "root"

#ifndef AUDIO_MP3_RINGBUFF_SUPPORT
    #error "Need enable MP3 ringbuff for stream play."
#endif

/**
 * @brief Mount fs.
 */
int mnt_init(void)
{
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, FS_ROOT);
    if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)//Format file system
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0)
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

/* User code start from here --------------------------------------------------------*/

/* play status */
uint32_t g_mp3_play_seconds = 0;
bool g_mp3_play_is_end = false;

/* Semaphore used to wait aes interrupt. */
/* mp3 handle */
static mp3ctrl_handle g_mp3_handle = NULL;
/* mp3 process thread */
static rt_thread_t g_mp3_proc_thread = NULL;
/* message queue used by mp3 process thread */
static rt_mq_t g_mp3_proc_mq = NULL;

/**
 * @brief send msg to mp3 proc thread.
 */
static void send_msg_to_mp3_proc(mp3_ctrl_info_t *info)
{
    rt_err_t err = rt_mq_send(g_mp3_proc_mq, info, sizeof(mp3_ctrl_info_t));
    RT_ASSERT(err == RT_EOK);
}

void play_buff(const char *buff, int len)
{
    rt_kprintf("[LOCAL MUSIC]%s %x,%d\n", __func__, buff, len);
    mp3_ctrl_info_t info = {0};

    info.cmd = CMD_MP3_PALY;
    info.loop = 0;
    info.param.filename = buff;
    info.param.len = len;

    send_msg_to_mp3_proc(&info);
}

void play_ringbuff(int file_len)
{
    rt_kprintf("%s file_len:%d\n", __func__, file_len);
    mp3_ctrl_info_t info = {0};

    info.cmd = CMD_MP3_PALY;
    info.loop = 0;
    info.param.len = file_len;

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

uint32_t play_get_total_seconds(void)
{
    uint32_t seconds = 0;
    if (g_mp3_handle)
    {
        mp3ctrl_get_total_seconds(g_mp3_handle, &seconds);
    }
    return seconds;
}

/**
 * @brief callback function for mp3ctrl_open.
 */
static int play_callback_func(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    rt_kprintf("[LOCAL MUSIC]%s cmd %d\n", __func__, cmd);
    switch (cmd)
    {
        case as_callback_cmd_user:
            rt_kprintf("[LOCAL MUSIC]%s cmd user, data=%x secs=%d\n", __func__, callback_userdata, reserved);
            g_mp3_play_seconds = reserved;//底层更新header中保存的秒数
            break;
        case as_callback_cmd_user_read://底层读数据不够，通知应用层读取更多数据
            rt_kprintf("[LOCAL MUSIC]%s user read, read_pos=%d\n", __func__, reserved);
            /* notify mp3 download thread to get more */
            //send_read_msg_to_mp3_dl(reserved);
            mp3_dl_read_more(reserved);//通知下载线程读取更多数据
            break;
        case as_callback_cmd_play_to_end://底层播放结束通知
            /* To close audio client when playing has been completed. */
            g_mp3_play_is_end = true;
            play_stop();//停止播放
            mp3_network_get_part_cancel();//取消下载线程的下载操作
            break;
#if 0
        case as_callback_cmd_closed:
            /* audio_close() be called, I do not know this is just pause or real close */
            //g_mp3_play_is_end = true;
            //send_stop_msg_to_mp3_dl();
            break;
#endif
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

#if 0
    if (g_mp3_ring_buffer == NULL)
    {
        g_mp3_ring_buffer = (uint8_t *)mp3_mem_malloc(MP3_RING_BUFFER_SIZE);
        RT_ASSERT(g_mp3_ring_buffer != NULL);
    }
#endif

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
                /* Close firstly if mp3 is playing. */
                mp3ctrl_close(g_mp3_handle);
                g_mp3_handle = NULL;
            }
            g_mp3_handle = mp3ctrl_open_ringbuffer(AUDIO_TYPE_LOCAL_MUSIC,  /* audio type, see enum audio_type_t. */
                                        (struct rt_ringbuffer *)msg.param.filename,  /* buffer */
                                        msg.param.len,  /* file len */
                                        play_callback_func,  /* play callback function. */
                                        NULL);
            RT_ASSERT(g_mp3_handle);

            audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, 5); /* default volume */

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
            g_mp3_play_is_end = true;
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

#if 0
    if (g_mp3_ring_buffer)
    {
        mp3_mem_free(g_mp3_ring_buffer);
        g_mp3_ring_buffer = NULL;
    }
#endif
}


/**
 * @brief Common initialization.
 */
rt_err_t mp3_comm_init(void)
{
    mp3_ring_buffer_init();

    g_mp3_proc_mq = rt_mq_create("mp3_proc_mq", sizeof(mp3_ctrl_info_t), 60, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_mp3_proc_mq);
    g_mp3_proc_thread = rt_thread_create("mp3_proc", mp3_proc_thread_entry, NULL, 2048, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(g_mp3_proc_thread);
    rt_err_t err = rt_thread_startup(g_mp3_proc_thread);
    RT_ASSERT(RT_EOK == err);

    rt_kprintf("[LOCAL MUSIC]%s\n", __func__);

    return RT_EOK;
}

#if 0
/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("\n[LOCAL MUSIC]Local music Example.\n");

    /* ls files in root. */
    extern void ls(const char *name);
    ls("/");

    /* mp3 process thread and message queue initialization. */
    comm_init();

    /* Play /16k.wav */
    play_file(MUSIC_FILE_PATH,
              0    /* 0 : play one time. 1 ~ n : play 2 ~ n+1 times. */
             );

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}
#endif
