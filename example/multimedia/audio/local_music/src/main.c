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
#include "audio_server.h"
#include "audio_mp3ctrl.h"
#include "drv_flash.h"

/* Common functions for RT-Thread based platform -----------------------------------------------*/

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#endif

#define FS_ROOT "root"

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
#define MUSIC_FILE_PATH "/16k.wav"

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
 * @param file_name music file (.wav .mp3) path.
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
        /* To close audio client when playing has been completed. */
        play_stop();
        break;

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

