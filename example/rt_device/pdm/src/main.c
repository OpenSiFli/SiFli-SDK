#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
/* user start */
#include "drv_flash.h"
#include "audio_server.h"
/* user end */


/* Common functions for RT-Thread based platform -----------------------------------------------*/


/* User code start from here --------------------------------------------------------*/

#ifndef FS_REGION_START_ADDR
    #error "FS_REGION_START_ADDR is not defined."
#endif

#define PDM_SAVE_PART_BASE           (FS_REGION_START_ADDR)
#define PDM_SAVE_PART_LIMIT          (FS_REGION_SIZE)

#define PDM_WR_CACHE_LIMIT          (128*1024)

#define RX_RING_BUFFER_SIZE         (((30*1024)/CFG_AUDIO_RECORD_PIPE_SIZE) * CFG_AUDIO_RECORD_PIPE_SIZE)

#define RX_THREAD_STACK_SIZE    (2*1024)

#define PDM1_DEVICE_HNAME        "pdm1"


#undef MIN
#define MIN(a, b)               ( (a) < (b) ? (a) : (b) )

rt_device_t g_pdm_device;
/* write to flash once per 128k. */
uint8_t g_wr_cache[PDM_WR_CACHE_LIMIT + CFG_AUDIO_RECORD_PIPE_SIZE] = {0};
uint32_t g_wr_cache_offset = 0;
uint32_t g_saved_size = 0;

static struct rt_ringbuffer g_rx_ring;
static uint8_t *g_rx_pool;
static rt_sem_t g_rx_sema;
static rt_sem_t g_save_sema;
static rt_event_t g_rx_ev;

static rt_thread_t g_rx_tid;
static rt_thread_t g_save_tid;

// play
static audio_client_t g_play_client;
static uint32_t g_read_pos = 0;
static rt_sem_t g_play_sema;


extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);

/**
 * @brief Rx ind.
 */
static rt_err_t audio_rx_ind(rt_device_t dev, rt_size_t size)
{
    // rt_kprintf("audio_rx_ind %d\n", size);
    rt_event_send(g_rx_ev, 1);
    return RT_EOK;
}

/**
 * @brief Rx thread.
 *        Read from Audio Pipe, cache to g_rx_ring (ringbuffer).
 */
void rx_thread_entry(void *parameter)
{
    rt_err_t ret = RT_EOK;
    rt_size_t len = 0;
    rt_size_t size = 0;
    uint8_t tmp_buf[CFG_AUDIO_RECORD_PIPE_SIZE] = {0};
    rt_uint32_t evt;

    while (1)
    {
        rt_event_recv(g_rx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        while (1)
        {
            len = rt_device_read(g_pdm_device, 0, tmp_buf, CFG_AUDIO_RECORD_PIPE_SIZE);
            RT_ASSERT(len == CFG_AUDIO_RECORD_PIPE_SIZE);

            if (rt_ringbuffer_space_len(&g_rx_ring) < CFG_AUDIO_RECORD_PIPE_SIZE)
            {
                /* drop it */
                rt_kprintf("g_rx_ring is full. drop it.\n");
                continue;
            }
            size = rt_ringbuffer_put(&g_rx_ring, tmp_buf, CFG_AUDIO_RECORD_PIPE_SIZE);
            RT_ASSERT(size == CFG_AUDIO_RECORD_PIPE_SIZE);
            rt_sem_release(g_save_sema);
        }
    }

}

/**
 * @brief Save thread, used to read data from g_rx_ring(ringbuffer), Write once to flash per 128k.
 */
void save_thread_entry(void *parameter)
{
    rt_err_t ret = RT_EOK;
    rt_size_t len = 0;
    rt_size_t size = 0;

    while (1)
    {
        ret = rt_sem_take(g_save_sema, RT_WAITING_FOREVER);
        RT_ASSERT(ret == RT_EOK);
        len = rt_ringbuffer_get(&g_rx_ring, &g_wr_cache[g_wr_cache_offset], CFG_AUDIO_RECORD_PIPE_SIZE);
        RT_ASSERT(len == CFG_AUDIO_RECORD_PIPE_SIZE);
        g_wr_cache_offset += CFG_AUDIO_RECORD_PIPE_SIZE;
        if (g_wr_cache_offset >= PDM_WR_CACHE_LIMIT)
        {
            /* up to 128k */
            if (g_saved_size + PDM_WR_CACHE_LIMIT <= PDM_SAVE_PART_LIMIT)
            {
                FLASH_HandleTypeDef *fhandle = Addr2Handle((uint32_t)(PDM_SAVE_PART_BASE + g_saved_size));
                if (fhandle == NULL)    // get nor flash handler fail, it should be nand
                    size = rt_nand_write((uint32_t)(PDM_SAVE_PART_BASE + g_saved_size), g_wr_cache, PDM_WR_CACHE_LIMIT);
                else
                    size = rt_flash_write((uint32_t)(PDM_SAVE_PART_BASE + g_saved_size), g_wr_cache, PDM_WR_CACHE_LIMIT);
                RT_ASSERT(size == PDM_WR_CACHE_LIMIT);
                g_saved_size += size;
            }
            else
            {
                rt_kprintf("SAVE TO PART LIMIT(%d) : %d\n", PDM_SAVE_PART_LIMIT, g_saved_size);
                g_save_tid = NULL;
                break;
            }

            g_wr_cache_offset = g_wr_cache_offset - PDM_WR_CACHE_LIMIT;
            if (g_wr_cache_offset > 0)
            {
                /* save left part. */
                memcpy(g_wr_cache, &g_wr_cache[PDM_WR_CACHE_LIMIT], g_wr_cache_offset);
            }
        }
    }

}

static void save_left_data(void)
{
    uint32_t cache_left_size, save_part_left_size = 0;
    int size = 0;
    uint32_t align_size = 0;

    RT_ASSERT(PDM_SAVE_PART_LIMIT >= g_saved_size);

    cache_left_size = g_wr_cache_offset;
    save_part_left_size = PDM_SAVE_PART_LIMIT - g_saved_size;
    align_size = RT_ALIGN_DOWN(MIN(cache_left_size, save_part_left_size), 2048);
    rt_kprintf("g_wr_cache_offset %d g_saved_size %d align_size %d\n", g_wr_cache_offset, g_saved_size, align_size);
    FLASH_HandleTypeDef *fhandle = Addr2Handle((uint32_t)(PDM_SAVE_PART_BASE + g_saved_size));
    if (fhandle == NULL)    // get nor flash handler fail, it should be nand
        size = rt_nand_write((uint32_t)(PDM_SAVE_PART_BASE + g_saved_size), g_wr_cache, align_size);
    else
        size = rt_flash_write((uint32_t)(PDM_SAVE_PART_BASE + g_saved_size), g_wr_cache, align_size);
    g_saved_size += size;
    rt_kprintf("FINSH. Total size : 0x%x\n", g_saved_size);
}

/**
 * @brief Create work threads and ipc vars.
 */
static void pdm_record_init(void)
{
    rt_kprintf("%s\n", __func__);

    g_rx_ev = rt_event_create("audio_evt", RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_rx_ev);

    g_rx_sema = rt_sem_create("rx_sem", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_rx_sema);

    g_save_sema = rt_sem_create("save_sem", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_save_sema);

    /* rx ringbuffer */
    g_rx_pool  = rt_malloc(RX_RING_BUFFER_SIZE);
    RT_ASSERT(g_rx_pool);
    rt_ringbuffer_init(&g_rx_ring, g_rx_pool, RX_RING_BUFFER_SIZE);

    /* rx thread */
    g_rx_tid = rt_thread_create("rx_t", rx_thread_entry, NULL, RX_THREAD_STACK_SIZE, 3, 10);
    RT_ASSERT(g_rx_tid);
    rt_thread_startup(g_rx_tid);

    /* save thread */
    g_save_tid = rt_thread_create("save_t", save_thread_entry, NULL, RX_THREAD_STACK_SIZE, 4, 10);
    RT_ASSERT(g_save_tid);
    rt_thread_startup(g_save_tid);

    /* erease flash */
    rt_kprintf("erasing flash.\n");
    FLASH_HandleTypeDef *fhandle = Addr2Handle((uint32_t)PDM_SAVE_PART_BASE);
    if (fhandle == NULL)    // get nor flash handler fail, it should be nand
        rt_nand_erase((uint32_t)PDM_SAVE_PART_BASE, PDM_SAVE_PART_LIMIT);
    else
        rt_flash_erase((uint32_t)PDM_SAVE_PART_BASE, PDM_SAVE_PART_LIMIT);
    rt_kprintf("erase finished.\n");
    g_saved_size = 0;
}

/**
 * @brief Release threads and ipc vars.
 */
static void pdm_record_deinit(void)
{
    rt_kprintf("%s\n", __func__);

    /* save thread */
    if (g_save_tid)
    {
        rt_thread_delete(g_save_tid);
    }

    /* rx thread */
    if (g_rx_tid)
    {
        rt_thread_delete(g_rx_tid);
    }

    rt_sem_delete(g_rx_sema);
    rt_sem_delete(g_save_sema);

    save_left_data();

    g_wr_cache_offset = 0;
    //g_saved_size = 0;

    if (g_rx_pool)
    {
        rt_free(g_rx_pool);
    }

}

/**
 * @brief Audio callback function for play.
 *        Read from flash.
 */
static int audio_callback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int len = 0;
    int wr_len = 0;
    uint8_t *data = (uint8_t *)callback_userdata;
    RT_ASSERT(data);
    //RT_ASSERT(g_play_client);

    rt_kprintf("%s cmd= %d\n", __func__, cmd);
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        rt_kprintf("g_saved_size %d POS %d\n", g_saved_size, g_read_pos);
        if (g_play_client)
        {
            if (g_read_pos < g_saved_size)
            {
                FLASH_HandleTypeDef *fhandle = Addr2Handle((uint32_t)(PDM_SAVE_PART_BASE + g_read_pos));
                if (fhandle == NULL)    // get nor flash handler fail, it should be nand
                    len = rt_nand_read((uint32_t)(PDM_SAVE_PART_BASE + g_read_pos), data, 2048);
                else
                    len = rt_flash_read((uint32_t)(PDM_SAVE_PART_BASE + g_read_pos), data, 2048);
                RT_ASSERT(len > 0);
                g_read_pos += len;
            }
            else
            {
                /* PLAY END */
                rt_sem_release(g_play_sema);
                return 0;
            }

            wr_len = audio_write(g_play_client, (uint8_t *)data, len);
            if (wr_len == 0)
            {
                rt_kprintf("record paly. cache full.\n");
            }
        }
    }
    return 0;
}

/**
 * @brief Example: play after pdm recording.
 * @param channel_num number of channels.
 * @param bits_per_sample  bits depth.
 * @param samplerate       sample rate.
 */
static void pdm_record_play(uint8_t channel_num, uint8_t bits_per_sample, uint32_t samplerate)
{
    rt_kprintf("%s\n", __func__);

    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = bits_per_sample; //16;
    pa.write_channnel_num = channel_num;
    pa.write_samplerate = samplerate; //16000;
    pa.write_cache_size = 4096;

    pa.read_bits_per_sample = bits_per_sample;
    pa.read_channnel_num = channel_num;
    pa.read_samplerate = samplerate;
    pa.read_cache_size = 2048;

    uint8_t *pcm_data = rt_malloc(2048);
    RT_ASSERT(pcm_data);

    g_read_pos = 0;
    g_play_sema = rt_sem_create("play_sem", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_play_sema);

    g_play_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &pa, audio_callback_play, (void *)pcm_data);
    RT_ASSERT(g_play_client >= 0);

    audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, 15);

    rt_kprintf("PDM start play. g_play_client %d samplefmt %d samplerate %d\n", g_play_client, bits_per_sample, samplerate);
    /* WAIT FOR PLAY END */
    rt_err_t err = rt_sem_take(g_play_sema, RT_WAITING_FOREVER);
    RT_ASSERT(err == RT_EOK);

    rt_kprintf("PDM play end.\n");

    audio_close(g_play_client);
    rt_free(pcm_data);
    rt_sem_delete(g_play_sema);
    g_play_sema = NULL;
}

/**
 * @brief Example: pdm recording.
 * @param channels   1 : PDM_CHANNEL_LEFT_ONLY others : PDM_CHANNEL_STEREO
 * @param samplefmt  see PDM_ChannelDepthTypeDef.
 * @param samplerate see PDM_SampleRateTypeDef.
 *
 * @retval return RT_EOK if success.
 */
static rt_err_t example_pdm_record_start(rt_uint32_t channels, rt_uint32_t samplefmt, rt_uint32_t samplerate)
{
    rt_kprintf("%s\n", __func__);
    if (g_pdm_device == NULL)
    {
        g_pdm_device = rt_device_find(PDM1_DEVICE_HNAME);
        if (g_pdm_device)
        {
            rt_device_init(g_pdm_device);
            rt_device_open(g_pdm_device, RT_DEVICE_FLAG_RDONLY);
            rt_kprintf("PDM opened\n");
        }
        else
        {
            rt_kprintf("Could not find PDM device\n");
            return RT_ERROR;
        }

        /* Create sem and threads. */
        pdm_record_init();

        /* Config channel, samplefmt and samplerate. */
        struct rt_audio_caps caps;
        caps.main_type = AUDIO_TYPE_INPUT;
        caps.sub_type = AUDIO_DSP_PARAM;
        caps.udata.config.channels = channels;
        caps.udata.config.samplefmt = samplefmt;    // depth
        caps.udata.config.samplerate = samplerate;
        rt_kprintf("CONFIG ch %d samplefmt %d samplerate %d\n", caps.udata.config.channels, caps.udata.config.samplefmt, caps.udata.config.samplerate);
        rt_device_control(g_pdm_device, AUDIO_CTL_CONFIGURE, &caps);

        int vol = 90; /* set volume */
        rt_device_control(g_pdm_device, AUDIO_CTL_SETVOLUME, (void *)vol);

        /* Set rx callback. */
        rt_device_set_rx_indicate(g_pdm_device, audio_rx_ind);
        /* Start recording. */
        int stream = AUDIO_STREAM_RECORD;
        rt_device_control(g_pdm_device, AUDIO_CTL_START, &stream);
    }

    return RT_EOK;
}

/**
 * @brief Example: stop pdm recording.
 */
static void example_pdm_record_stop(void)
{
    rt_kprintf("%s\n", __func__);
    if (g_pdm_device)
    {
        int stream = 0;

        stream = AUDIO_STREAM_RECORD;
        rt_device_control(g_pdm_device, AUDIO_CTL_STOP, &stream);
        rt_device_close(g_pdm_device);
        g_pdm_device = NULL;
        pdm_record_deinit();
    }
}

/**
 * @brief test cmd for pdm recording.
 * EX. pdm open 1 16 16000 : start recording. channel(left-only), 16bit, 16k-samplerate
 *     pdm stop            : stop recording.
 *     pdm play            : play after recording.
 */
int pdm_record_test(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "open") == 0)
        {
            example_pdm_record_start(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        }

        if (strcmp(argv[1], "stop") == 0)
        {
            example_pdm_record_stop();
        }

        if (strcmp(argv[1], "play") == 0)
        {
            pdm_record_play(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        }

    }
    return RT_EOK;
}

MSH_CMD_EXPORT_ALIAS(pdm_record_test, pdm, pdm record and play)



/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("PDM Record Example.\n");

    /* PIN CONFIG */
#ifdef SOC_SF32LB52X
#if !defined(BSP_USING_LCD) && defined(BSP_USING_PDM1)
    HAL_PIN_Set(PAD_PA07, PDM1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA08, PDM1_DATA, PIN_PULLDOWN, 1);
#else
#error "Need to confirm PDM pin config."
#endif
#else
#error "Need to confirm PDM pin config."
#endif

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }
    return 0;
}

