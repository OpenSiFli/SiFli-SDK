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
#define MIC_RECORD_FILE "/mic16k.pcm"
#define AUDIO_WRITE_CACHE_SIZE (4096)
#define AUDIO_READ_CACHE_SIZE  (2048)

/* Semaphore used to wait aes interrupt. */
static rt_sem_t g_audio_sem;
/* pcm buffer */
static uint16_t *s_pcm = NULL;
/* read offset */
static int g_pcm_file_offset = 0;
/* audio client */
static audio_client_t g_client;

/**
 * @brief Common initialization.
 */
static rt_err_t comm_init(void)
{
    g_audio_sem = rt_sem_create("audio_sem", 0, RT_IPC_FLAG_FIFO);
    return RT_EOK;
}

/**
 * @brief Audio callback function for recording.
 *        Save data to file.
 */
static int audio_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    int wr_len = 0;

    rt_kprintf("[RECORD]%s cmd %d\n", __func__, cmd);
    if (cmd == as_callback_cmd_data_coming)
    {
        /* get recording pcm data. */
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        /* pcm data left shift 4 bits to increase volume. */
        auido_gain_pcm((int16_t *)p->data, p->data_len, 4);
        /* save recording pcm data to file. */
        wr_len = write(fd, p->data, p->data_len);
        // rt_kprintf("[RECORD]%s recording save %d bytes.\n", __func__, wr_len);
    }

    return 0;
}

/**
 * @brief Example for recording.
 *        Recording for 5 seconds.
 */
static void start_recording(void)
{
    rt_kprintf("[RECORD]%s\n", __func__);

    int fd;
    audio_parameter_t param = {0};

    /* audio parameters. */
    param.write_bits_per_sample = 16;
    param.write_channnel_num = 1;
    param.write_samplerate = 16000; /* 16k sampling rate. */
    param.write_cache_size = AUDIO_WRITE_CACHE_SIZE;  /* write cache size */

    param.read_bits_per_sample = 16;
    param.read_channnel_num = 1;
    param.read_samplerate = 16000;  /* 16k sampling rate. */
    param.read_cache_size = AUDIO_READ_CACHE_SIZE;   /* read cache size */

    /* New file to store recording data. */
    fd = open(MIC_RECORD_FILE, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(fd >= 0);

    /* Start recording. */
    audio_client_t client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &param, audio_callback_record, (void *)fd);
    RT_ASSERT(client);

    /* Recording for 10 seconds.  */
    rt_thread_mdelay(10000);

    rt_kprintf("[RECORD]close audio client.\n");
    /* Stop recording. */
    audio_close(client);
    /* close file handle. */
    close(fd);
}

/**
 * @brief Audio callback function for recording play.
 *        Read from file.
 */
static int audio_callback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int fd = (int)callback_userdata;
    int rd_len = 0;
    int wr_len = 0;
    static uint8_t read_finish = 0;

    rt_kprintf("[RECORD]%s cmd: %d\n", __func__, cmd);
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        if (fd >= 0 && s_pcm && g_client)
        {
            lseek(fd, g_pcm_file_offset, SEEK_SET);
            rd_len = read(fd, (void *)s_pcm, (AUDIO_WRITE_CACHE_SIZE / 2));
            if (rd_len <= 0)
            {
                read_finish = 1;
                rt_kprintf("[RECORD]%s read finish. read len: %d offset:%d\n", __func__, rd_len, g_pcm_file_offset);
            }
            else
            {
                wr_len = audio_write(g_client, (uint8_t *)s_pcm, rd_len);
                g_pcm_file_offset += wr_len;
                // rt_kprintf("%s audio write %d bytes. \n", __func__, wr_len);
            }
        }
    }

    if (cmd == as_callback_cmd_cache_empty && read_finish)
    {
        /* Notify to exit playing. */
        rt_kprintf("[RECORD]%s ready to exit play.\n", __func__, rd_len);
        rt_sem_release(g_audio_sem);
    }

    return 0;
}

/**
 * @brief Example for recording play.
 *        Read from pcm file and play.
 */
static void recording_play(void)
{
    rt_kprintf("[RECORD]%s\n", __func__);

    /* Open recording pcm file. */
    int fd = -1;
    fd = open(MIC_RECORD_FILE, O_RDONLY | O_BINARY);
    RT_ASSERT(fd >= 0);

    /* Get pcm file size. */
    int size = lseek(fd, 0, SEEK_END);
    lseek(fd, 0, SEEK_SET);
    rt_kprintf("[RECORD]%s size %d.\n", MIC_RECORD_FILE, size);

    /* audio parameters. */
    audio_parameter_t param = {0};
    param.write_bits_per_sample = 16;
    param.write_channnel_num = 1;
    param.write_samplerate = 16000; /* 16k sampling rate. */
    param.write_cache_size = AUDIO_WRITE_CACHE_SIZE;  /* write cache size */
    param.read_bits_per_sample = 16;
    param.read_channnel_num = 1;
    param.read_samplerate = 16000;  /* 16k sampling rate. */
    param.read_cache_size = AUDIO_READ_CACHE_SIZE;   /* read cache size */
    /* Open audio client. */
    g_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &param, audio_callback_play, (void *)fd);
    RT_ASSERT(g_client >= 0);

    /* Read data from pcm file.
     * Read AUDIO_WRITE_CACHE_SIZE for the first time, then read in half afterward.
     */
    rt_kprintf("[RECORD]audio write %d bytes. \n", AUDIO_WRITE_CACHE_SIZE);
    s_pcm = (uint16_t *)rt_malloc(AUDIO_WRITE_CACHE_SIZE);
    RT_ASSERT(s_pcm);
    read(fd, (void *)s_pcm, AUDIO_WRITE_CACHE_SIZE);
    int wr_len = audio_write(g_client, (uint8_t *)s_pcm, AUDIO_WRITE_CACHE_SIZE);
    g_pcm_file_offset += wr_len;

    /* Wait for playback to complete. */
    rt_sem_take(g_audio_sem, RT_WAITING_FOREVER);

    /* Close audio client. */
    rt_kprintf("[RECORD]close audio client.\n");
    audio_close(g_client);
    close(fd);
    unlink(MIC_RECORD_FILE);
    free(s_pcm);
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("\n[RECORD]Record Example.\n");

    /* semaphore initialiation. */
    comm_init();

    /* Recording and save to file. */
    start_recording();
    /* Play recording file. */
    recording_play();

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}

