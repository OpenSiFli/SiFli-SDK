/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* user start */
#include "mem_section.h"
#include "drv_flash.h"
#include <drivers/audio.h>
#ifdef PA_USING_AW87390
    #include "sifli_aw87390.h"
#endif
#ifdef PA_USING_AW8155
    #include "sifli_aw8155.h"
#endif

// #ifndef FS_REGION_START_ADDR
//     #error "FS_REGION_START_ADDR is not defined."
// #endif

#define AUDCODEC_DEVICE_NAME "audcodec"
#define AUDPRC_DEVICE_NAME   "audprc"
#define I2S_DEVICE_NAME "i2s2"

/* i2s DMA Buffer size is defined in drv_i2s_audio.c {see AUDIO_DATA_SIZE} */
#define AUDIO_BUF_SIZE  (640)
/* Buffer size of rx data.  */
#define AUDRX_BUF_MAX (1024*1024)

static rt_event_t g_tx_ev, g_rx_ev, g_i2s_tx_ev, g_i2s_rx_ev;
static rt_sem_t complete_sem = NULL;
rt_device_t g_audprc_dev = NULL, g_audcodec_dev = NULL, g_i2s_dev = NULL;
rt_thread_t g_rx_tx_tid;

/* temporary buffer */
static uint8_t g_pipe_data[AUDIO_BUF_SIZE];
/* Buffer of rx data, used to save rx pcm data. */
L2_RET_BSS_SECT_BEGIN(g_audrx_buf)
ALIGN(4) uint8_t g_audrx_buf[AUDRX_BUF_MAX] L2_RET_BSS_SECT(g_audrx_buf);
L2_RET_BSS_SECT_END
/* Buffer offset */
uint32_t g_audrx_offset = 0;

// extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);

#if defined(BSP_ENABLE_AUD_PRC) && defined(BSP_ENABLE_AUD_CODEC)

/**
* @brief  Audio receiving callback.
* This callback will send event to receiving thread for further audio precessing.
* @param[in]  dev: audio device.
* @param[in]  size: received audio data size.
* @retval RT_EOK
*/
static rt_err_t audio_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_event_send(g_rx_ev, 1);
    // rt_kprintf("[EX_I2S]audio_rx_ind %d\n", size);
    return RT_EOK;
}

/**
* @brief  Tx callback.
* When Isr arrived, need to write audio data (half dma buffer) to tx device.
* @param[in]  dev: audio device.
* @retval RT_EOK
*/
static rt_err_t audio_tx_done(rt_device_t dev, void *buffer)
{
    // rt_kprintf("[EX_I2S]audio_tx_done \n");
    rt_event_send(g_tx_ev, 1);
    return RT_EOK;
}

/**
 * @brief Open PA.
 */
static void audio_pa_open(void)
{
#ifdef PA_USING_AW87390
    sifli_aw87390_start();
#elif defined(PA_USING_AW8155)
    sifli_aw8155_start();
#else
#error "PA NOT DEFINED."
#endif
}

/**
 * @brief Close PA.
 */
static void audio_pa_close(void)
{
#ifdef PA_USING_AW87390
    sifli_aw87390_stop();
#elif defined(PA_USING_AW8155)
    sifli_aw8155_stop();
#else
#error "PA NOT DEFINED."
#endif
}

/**
 * @brief Configuration RX (ADC) for AUDCODEC and AUDPRC.
 */
static void config_rx(rt_uint32_t channels, rt_uint32_t sr, rt_uint32_t fmt)
{
    struct rt_audio_caps caps;
    int stream;

    /* AUDCODEC : set input as "AUDPRC_RX_FROM_CODEC" */
    rt_device_control(g_audcodec_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);
    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = 1 << HAL_AUDCODEC_ADC_CH0; // ADC CH0
    caps.udata.config.channels   = channels;    // channels
    caps.udata.config.samplerate = sr; // sample rate
    caps.udata.config.samplefmt = fmt; // depth. 8 16 24 or 32
    rt_device_control(g_audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);

    rt_kprintf("[EX_I2S]codec input parameter:sub_type=%d channels %d, rate %d, bits %d\n", caps.sub_type, caps.udata.config.channels,
               caps.udata.config.samplerate, caps.udata.config.samplefmt);

    /* AUDPRC : set input as "AUDPRC_RX_FROM_CODEC" */
    rt_device_control(g_audprc_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);
    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = HAL_AUDPRC_RX_CH0 - HAL_AUDPRC_RX_CH0;
    caps.udata.config.channels   = channels;
    caps.udata.config.samplerate = sr;
    caps.udata.config.samplefmt = fmt;
    rt_kprintf("[EX_I2S]mic input:rx channel %d, channels %d, rate %d, bitwidth %d\n", 0, caps.udata.config.channels,
               caps.udata.config.samplerate, caps.udata.config.samplefmt);
    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);
}

/**
 * @brief Start RX.
 */
static void start_rx(void)
{
    int stream;
    rt_kprintf("[EX_I2S]%s\n", __func__);
    rt_device_set_rx_indicate(g_audprc_dev, audio_rx_ind);
    stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_START, &stream);
    stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
    rt_device_control(g_audprc_dev, AUDIO_CTL_START, &stream);
}

/**
 * @brief Stop RX.
 */
static void stop_rx(void)
{
    int stream;
    rt_kprintf("[EX_I2S]%s\n", __func__);
    stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_STOP, &stream);
    stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
    rt_device_control(g_audprc_dev, AUDIO_CTL_STOP, &stream);
}

/**
 * @brief The thread used to read rx audio data and save to the ram buffer,
 *        the ram buffer'size is up to AUDIORX_BUF_MAX.
 */
static void audprc_rx_entry(void *param)
{
    rt_uint32_t evt;
    rt_size_t len;
    rt_uint32_t circle = 0;

    rt_kprintf("[EX_I2S]%s\n", __func__);
    g_audrx_offset = 0;
    memset(g_audrx_buf, 0, sizeof(g_audrx_buf));

    while (1)
    {
        rt_event_recv(g_rx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        while (1)
        {
            /* RX read (from mic)*/
            len = rt_device_read(g_audprc_dev, 0, g_pipe_data, AUDIO_BUF_SIZE / 2); //AUDIO_BUF_SIZE/2
            if (len != (AUDIO_BUF_SIZE / 2))
            {
                rt_kprintf("[EX_I2S]Got abnormal audio size = %d\n", len);
            }

            /* Save to RAM Buffer. */
            if ((g_audrx_offset + (AUDIO_BUF_SIZE / 2)) < AUDRX_BUF_MAX)
            {
                memcpy(&g_audrx_buf[g_audrx_offset], g_pipe_data, AUDIO_BUF_SIZE / 2);
                g_audrx_offset += (AUDIO_BUF_SIZE / 2);
            }
            else
            {
                /* Record buffer is up to  AUDRX_BUF_MAX. Stop recording. */
                goto __EXIT;
            }

            circle ++;
            if (circle == 20)
            {
                rt_kprintf("[EX_I2S]recording. size:%d\n", g_audrx_offset);
                circle = 0;
            }
        }
    }

__EXIT:
    rt_kprintf("[EX_I2S]recording finished. size:%d\n", g_audrx_offset);
    rt_sem_release(complete_sem);
}

/**
 * @brief Example : record from mic and save pcm data to ram buffer.
 *
 * @param rx_channels 1(single) / 2 (stereo)
 * @param rx_sr sample rate , 8000/12000/16000/24000/32000/48000/11025/22050/44010, see <user manual>.
 * @param rx_fmt depth, see <user manual>
 */
static void example_audprc_rx(rt_uint32_t channels, rt_uint32_t sr, rt_uint32_t fmt)
{
    rt_kprintf("[EX_I2S]Start recording.\n");
    // create rx thread
    rt_thread_t rx_tid = rt_thread_create("audprc_rx", audprc_rx_entry, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (rx_tid == NULL)
    {
        rt_kprintf("[EX_I2S]Create rx thread fail\n");
        return;
    }
    rt_thread_startup(rx_tid);

    config_rx(channels, sr, fmt);
    start_rx();

    rt_sem_take(complete_sem, RT_WAITING_FOREVER);
    rt_kprintf("[EX_I2S]Recording finished.\n");
    stop_rx();
}

/**
 * @brief Configuration TX (DAC) for AUDCODEC and AUDPRC.
 */
static void config_tx(rt_uint32_t channels, rt_uint32_t sr, rt_uint32_t fmt)
{
#define     mixer_sel  0x5150
    struct rt_audio_caps caps;
    struct rt_audio_sr_convert cfg;
    // uint8_t g_hardware_mix_enable = 1;
    /* mix left & right channel to mono channel output, too big volume sometime */
    int out_sel = 0x5050;
    // if (caps.udata.config.channels == 2 && g_hardware_mix_enable)
    // {
    //     out_sel = 0x5010; //mix left & right to speaker.  speaker pcm = left pcm + right pcm
    // }

    /* AUCODEC : set output as "AUDPRC_TX_TO_CODEC" (codec/mem/i2s). */
    rt_device_control(g_audcodec_dev, AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);
    caps.main_type = AUDIO_TYPE_OUTPUT;
    caps.sub_type = 1 << HAL_AUDCODEC_DAC_CH0;
    caps.udata.config.channels   = channels; // L,R,L,R,L,R, ......
    caps.udata.config.samplerate = sr;
    caps.udata.config.samplefmt = fmt; //8 16 24 or 32
    rt_kprintf("[EX_I2S]prc_codec : sub_type=%d channel %d, samplerate %d, bits %d\n", caps.sub_type, caps.udata.config.channels,
               caps.udata.config.samplerate, caps.udata.config.samplefmt);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);

    /* AUDPRC : set output as "AUDPRC_TX_TO_CODEC" (codec/mem/i2s). */
    rt_device_control(g_audprc_dev, AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);
    cfg.channel = channels;
    cfg.source_sr = sr;
    cfg.dest_sr = sr;
    rt_kprintf("[EX_I2S]speaker OUTPUTSRC channel=%d in_rate=%d out_rate=%d\n", cfg.channel, cfg.source_sr, cfg.dest_sr);
    rt_device_control(g_audprc_dev, AUDIO_CTL_OUTPUTSRC, (void *)(&cfg));

    /* AUDPRC : SELECTOR */
    rt_kprintf("[EX_I2S]speaker select=0x%x mixer=0x%x\n", out_sel, mixer_sel);
    caps.main_type = AUDIO_TYPE_SELECTOR;
    caps.sub_type = 0xFF;
    caps.udata.value   = out_sel;
    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    /* AUDPRC : MIXER */
    caps.main_type = AUDIO_TYPE_MIXER;
    caps.sub_type = 0xFF;
    caps.udata.value   = out_sel;
    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    /* data source format */
    caps.main_type = AUDIO_TYPE_OUTPUT;
    caps.sub_type = HAL_AUDPRC_TX_CH0;
    caps.udata.config.channels   = channels;
    caps.udata.config.samplerate = sr;
    caps.udata.config.samplefmt = fmt;
    rt_kprintf("[EX_I2S]tx[0]: sub_type %d, ch %d, samrate %d, bits %d\n",
               caps.sub_type,
               caps.udata.config.channels,
               caps.udata.config.samplerate,
               caps.udata.config.samplefmt);

    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    /* Set volume */
    int vol = -18;
    rt_kprintf("[EX_I2S]init volume=%d\n", vol);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_SETVOLUME, (void *)vol);
}

/**
 * @brief Start TX and open PA.
 */
static void start_tx(void)
{
    int stream_audprc, stream_audcodec;

    rt_kprintf("[EX_I2S]%s\n", __func__);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_MUTE, (void *)1);
    // rt_base_t level = rt_hw_interrupt_disable();
    stream_audcodec = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    stream_audprc   = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDPRC_TX_CH0) << 8);

    rt_device_control(g_audcodec_dev, AUDIO_CTL_START, &stream_audcodec);
    rt_device_set_tx_complete(g_audprc_dev, audio_tx_done);
    rt_device_control(g_audprc_dev, AUDIO_CTL_START, &stream_audprc);
    // rt_hw_interrupt_enable(level);

    /* Open PA. */
    audio_pa_open();

    rt_thread_mdelay(30);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_MUTE, (void *)0);
}

/**
 * @brief Stop TX and close PA.
 */
static void stop_tx(void)
{
    int stream_audprc, stream_audcodec;

    rt_kprintf("[EX_I2S]%s\n", __func__);
    // rt_base_t level = rt_hw_interrupt_disable();
    stream_audcodec = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    stream_audprc   = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDPRC_TX_CH0) << 8);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_STOP, &stream_audcodec);
    rt_device_control(g_audprc_dev, AUDIO_CTL_STOP, &stream_audprc);
    // rt_hw_interrupt_enable(level);
    /* Close PA */
    audio_pa_close();
}

/**
 * @brief The thread used to read audio data from ram buffer, and then write to tx device (speaker).
 */
static void audprc_tx_entry(void *param)
{
    rt_uint32_t evt;
    uint32_t wr_offset = 0;

    rt_kprintf("[EX_I2S]%s\n", __func__);

    while (1)
    {
        rt_event_recv(g_tx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

        int len = rt_device_write(g_audprc_dev, 0, &g_audrx_buf[wr_offset], AUDIO_BUF_SIZE / 2);
        if (len != (AUDIO_BUF_SIZE / 2))
        {
            rt_kprintf("[EX_I2S]Abnormal write len :%d\n", len);
        }
        wr_offset += (AUDIO_BUF_SIZE / 2);
        if (wr_offset + (AUDIO_BUF_SIZE / 2)  >= AUDRX_BUF_MAX)
        {
            goto __EXIT;
        }
    }

__EXIT:
    rt_kprintf("[EX_I2S]Play finished.\n");
    rt_sem_release(complete_sem);
}

/**
 * @brief Example : read pcm data form ram buffer and play with speaker.
 *
 * @param tx_channels 1(single) / 2 (stereo)
 * @param tx_sr sample rate , 8000/12000/16000/24000/32000/48000/11025/22050/44010, see <user manual>.
 * @param tx_fmt depth, see <user manual>
 */
static void example_audprc_tx(rt_uint32_t tx_channels, rt_uint32_t tx_sr, rt_uint32_t tx_fmt)
{
    rt_thread_t tx_tid = rt_thread_create("audprc_tx", audprc_tx_entry, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (tx_tid == NULL)
    {
        rt_kprintf("[EX_I2S]Create audprc_tx thread fail\n");
        return;
    }
    rt_thread_startup(tx_tid);

    config_tx(tx_channels, tx_sr, tx_fmt);
    start_tx();

    rt_sem_take(complete_sem, RT_WAITING_FOREVER);
    rt_kprintf("[EX_I2S]Play finished.\n");
    stop_tx();
}

/**
 * @brief test cmd for audprc & audcodec.
 * Command details see Readme.md
 */
int audprc_test(int argc, char *argv[])
{
    rt_uint32_t tx_channels = 0, tx_sr = 0, tx_fmt = 0;
    rt_uint32_t rx_channels = 0, rx_sr = 0, rx_fmt = 0;

    if (argc > 1)
    {
        if (strcmp(argv[1], "rx") == 0)
        {
            rx_channels = atoi(argv[2]);
            rx_sr = atoi(argv[3]);
            rx_fmt = atoi(argv[4]);
            example_audprc_rx(rx_channels, rx_sr, rx_fmt);
        }

        if (strcmp(argv[1], "tx") == 0)
        {
            tx_channels = atoi(argv[2]);
            tx_sr = atoi(argv[3]);
            tx_fmt = atoi(argv[4]);
            example_audprc_tx(tx_channels, tx_sr, tx_fmt);
        }
    }

    return RT_EOK;
}

MSH_CMD_EXPORT_ALIAS(audprc_test, audprc, audcodec and audproc test)

#endif //defined(BSP_ENABLE_AUD_PRC) && defined(BSP_ENABLE_AUD_CODEC)


#ifdef BSP_ENABLE_I2S_CODEC

/**
* @brief  Audio receiving callback.
* This callback will send event to receiving thread for further audio precessing.
* @param[in]  dev: audio device.
* @param[in]  size: received audio data size.
* @retval RT_EOK
*/
static rt_err_t i2s_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_event_send(g_i2s_rx_ev, 1);
    // rt_kprintf("[EX_I2S]i2s_rx_ind %d\n", size);
    return RT_EOK;
}

/**
* @brief  Tx callback.
* When Isr arrived, need to write audio data (half dma buffer) to tx device.
* @param[in]  dev: audio device.
* @retval RT_EOK
*/
static rt_err_t i2s_tx_done(rt_device_t dev, void *buffer)
{
    // rt_kprintf("[EX_I2S]i2s_tx_done \n");
    rt_event_send(g_i2s_tx_ev, 1);
    return RT_EOK;
}

/**
 * @brief Configuration RX (ADC) for i2s.
 */
static void i2s_config_rx(rt_uint32_t channels, rt_uint32_t sr, rt_uint32_t fmt)
{
    /* i2s configuration */
    struct rt_audio_caps caps;
    caps.main_type = AUDIO_TYPE_INPUT;      // for I2S2, configure RX will configure RX+TX
    caps.sub_type = AUDIO_DSP_PARAM;
    caps.udata.config.channels   = 2;    /* i2s is always 2 */
    caps.udata.config.samplerate = sr;   /* sample rate */
    caps.udata.config.samplefmt = fmt;   /* depth */
    rt_device_control(g_i2s_dev, AUDIO_CTL_CONFIGURE, &caps);
    rt_kprintf("[EX_I2S]Config i2s parameter: channel %d, samplerate %d, bitwidth %d\n", caps.udata.config.channels,
               caps.udata.config.samplerate, caps.udata.config.samplefmt);

    /* set i2s rx external interface */
    /* 0:DMA (by default) 1:AUDPRC */
    rt_device_control(g_i2s_dev, AUDIO_CTL_SETINPUT, (void *)0);

    /* set i2s mode (slave or master) */
    caps.main_type = AUDIO_TYPE_INPUT;      // for I2S2, configure RX will configure RX+TX
    caps.sub_type = AUDIO_DSP_MODE;
    caps.udata.value = 1;    /* 1:slave mode 0:master mode */
    rt_device_control(g_i2s_dev, AUDIO_CTL_CONFIGURE, &caps);
}

/**
 * @brief Start RX (i2s).
 */
static void i2s_start_rx(void)
{
    int stream_i2s;

    rt_kprintf("[EX_I2S]%s\n", __func__);
    /* set i2s rx callback */
    rt_device_set_rx_indicate(g_i2s_dev, i2s_rx_ind);
    /* i2s start */
    stream_i2s = AUDIO_STREAM_RECORD;
    rt_device_control(g_i2s_dev, AUDIO_CTL_START, &stream_i2s);
}

/**
 * @brief Stop RX (i2s).
 */
static void i2s_stop_rx(void)
{
    int stream_i2s;
    rt_kprintf("[EX_I2S]%s\n", __func__);

    /* i2s stop */
    stream_i2s = AUDIO_STREAM_RECORD;
    rt_device_control(g_i2s_dev, AUDIO_CTL_STOP, &stream_i2s);
}

/**
 * @brief The thread used to read rx audio data and save to the ram buffer,
 *        the ram buffer'size is up to AUDIORX_BUF_MAX.
 */
static void i2s_rx_entry(void *param)
{
    rt_uint32_t evt;
    rt_uint32_t circle = 0;
    rt_size_t len;

    rt_kprintf("[EX_I2S]%s", __func__);
    g_audrx_offset = 0;
    memset(g_audrx_buf, 0, sizeof(g_audrx_buf));

    while (1)
    {
        rt_event_recv(g_i2s_rx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        while (1)
        {
            /* RX read (from mic)*/
            len = rt_device_read(g_i2s_dev, 0, g_pipe_data, AUDIO_BUF_SIZE / 2);
            // rt_kprintf("[EX_I2S]from is2 len = %d", len);
            if (len != (AUDIO_BUF_SIZE / 2))
            {
                rt_kprintf("[EX_I2S]Got abnormal audio size = %d\n", len);
            }

            /* Save to RAM Buffer. */
            if ((g_audrx_offset + (AUDIO_BUF_SIZE / 2)) < AUDRX_BUF_MAX)
            {
                memcpy(&g_audrx_buf[g_audrx_offset], g_pipe_data, AUDIO_BUF_SIZE / 2);
                g_audrx_offset += (AUDIO_BUF_SIZE / 2);
            }
            else
            {
                /* Record buffer is up to  AUDRX_BUF_MAX. Stop recording. */
                goto __EXIT;
            }
            circle ++;
            if (circle == 20)
            {
                rt_kprintf("[EX_I2S]I2S RX. size:%d\n", g_audrx_offset);
                circle = 0;
            }
        }
    }

__EXIT:
    rt_kprintf("[EX_I2S]I2S RX finished.\n");
    rt_sem_release(complete_sem);
}

/**
 * @brief Example : RX from i2s and save pcm data to ram buffer.
 *
 * @param rx_channels 1(single) / 2 (stereo)
 * @param rx_sr sample rate , 8000/12000/16000/24000/32000/48000/11025/22050/44010, see <user manual>.
 * @param rx_fmt depth, see <user manual>
 */
static void example_i2s_rx(rt_uint32_t channels, rt_uint32_t sr, rt_uint32_t fmt)
{
    rt_kprintf("[EX_I2S]I2S RX.\n");
    /* create i2s-rx thread */
    rt_thread_t rx_tid = rt_thread_create("i2s_rx", i2s_rx_entry, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (rx_tid == NULL)
    {
        rt_kprintf("[EX_I2S]Create i2s rx thread fail\n");
        return;
    }
    rt_thread_startup(rx_tid);

    /* i2s rx configuration. */
    i2s_config_rx(channels, sr, fmt);
    /* start i2s rx */
    i2s_start_rx();

    /* wait i2s-rx finished (rx data is up to AUDRX_BUF_MAX). */
    rt_sem_take(complete_sem, RT_WAITING_FOREVER);
    /* stop i2s rx */
    i2s_stop_rx();
    rt_kprintf("[EX_I2S]I2S RX finished.\n");
}


/**
 * @brief Configuration TX (DAC) for i2s.
 */
static void i2s_config_tx(rt_uint32_t channels, rt_uint32_t sr, rt_uint32_t fmt)
{
    /* i2s configuration */
    struct rt_audio_caps caps;
    caps.main_type = AUDIO_TYPE_INPUT;      // for I2S2, configure RX will configure RX+TX
    caps.sub_type = AUDIO_DSP_PARAM;
    caps.udata.config.channels = 2;     /* for i2s, channels is always 2. */
    caps.udata.config.samplefmt = fmt;  /* depth */
    caps.udata.config.samplerate = sr;  /* sample rate */
    rt_device_control(g_i2s_dev, AUDIO_CTL_CONFIGURE, &caps);

    /* set i2s rx external interface */
    /* 0:DMA (by default) 1:AUDPRC */
    rt_device_control(g_i2s_dev, AUDIO_CTL_SETINPUT, (void *)0);

    /* set i2s mode (slave or master) */
    caps.main_type = AUDIO_TYPE_INPUT;      // for I2S2, configure RX will configure RX+TX
    caps.sub_type = AUDIO_DSP_MODE;
    caps.udata.value = 0;    /* 1:slave mode 0:master mode */
    rt_device_control(g_i2s_dev, AUDIO_CTL_CONFIGURE, &caps);
}

/**
 * @brief Start TX (i2s).
 */
static void i2s_start_tx(void)
{
    int stream_i2s;
    rt_kprintf("[EX_I2S]%s\n", __func__);
    /* set i2s tx callback */
    rt_device_set_tx_complete(g_i2s_dev, i2s_tx_done);
    /* i2 start */
    stream_i2s = AUDIO_STREAM_REPLAY;
    rt_device_control(g_i2s_dev, AUDIO_CTL_START, &stream_i2s);
}

/**
 * @brief Stop TX (i2s).
 */
static void i2s_stop_tx(void)
{
    int stream_i2s;
    rt_kprintf("[EX_I2S]%s\n", __func__);
    /* i2s stops */
    stream_i2s = AUDIO_STREAM_REPLAY;
    rt_device_control(g_i2s_dev, AUDIO_CTL_STOP, &stream_i2s);
}

/**
 * @brief The thread used to read audio data from ram buffer, and then write to tx device (i2s).
 */
static void i2s_tx_entry(void *param)
{
    rt_uint32_t evt;
    rt_uint32_t circle = 0;
    uint32_t wr_offset = 0;

    rt_kprintf("[EX_I2S]%s\n", __func__);

    while (1)
    {
        rt_event_recv(g_i2s_tx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        /* half i2s dma buffer size each time. */
        int len = rt_device_write(g_i2s_dev, 0, &g_audrx_buf[wr_offset], AUDIO_BUF_SIZE / 2);
        // rt_kprintf("[EX_I2S]to is2s len = %d\n", len);
        if (len != (AUDIO_BUF_SIZE / 2))
        {
            rt_kprintf("[EX_I2S]Abnormal write len :%d\n", len);
        }
        wr_offset += (AUDIO_BUF_SIZE / 2);
        if (wr_offset + (AUDIO_BUF_SIZE / 2)  >= AUDRX_BUF_MAX)
        {
            /* up to AUDRX_BUF_MAX, finish tx. */
            goto __EXIT;
        }
        circle ++;
        if (circle == 20)
        {
            rt_kprintf("[EX_I2S]I2S TX. size:%d\n", wr_offset);
            circle = 0;
        }
    }

__EXIT:
    rt_kprintf("[EX_I2S]I2S TX finished.\n");
    rt_sem_release(complete_sem);
}

/**
 * @brief Example : read pcm data form ram buffer and write to i2s device.
 *
 * @param tx_channels 1(single) / 2 (stereo)
 * @param tx_sr sample rate , 8000/12000/16000/24000/32000/48000/11025/22050/44010, see <user manual>.
 * @param tx_fmt depth, see <user manual>
 */
static void example_i2s_tx(rt_uint32_t tx_channels, rt_uint32_t tx_sr, rt_uint32_t tx_fmt)
{
    rt_kprintf("[EX_I2S]I2S TX:\n");
    /* create i2s-tx thread */
    rt_thread_t tx_tid = rt_thread_create("i2s_tx", i2s_tx_entry, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (tx_tid == NULL)
    {
        rt_kprintf("[EX_I2S]Create i2s_tx thread fail\n");
        return;
    }
    rt_thread_startup(tx_tid);

    i2s_config_tx(tx_channels, tx_sr, tx_fmt);
    i2s_start_tx();

    /* wait tx finished. */
    rt_sem_take(complete_sem, RT_WAITING_FOREVER);
    /* stop i2x tx */
    i2s_stop_tx();
    rt_kprintf("[EX_I2S]I2S TX finished.\n");
}


/**
 * @brief test cmd for audprc & audcodec.
 * Command details see Readme.md
 */
int i2s_test(int argc, char *argv[])
{
    rt_uint32_t rx_channels = 0, rx_sr = 0, rx_fmt = 0;
    rt_uint32_t tx_channels = 0, tx_sr = 0, tx_fmt = 0;

    if (argc > 1)
    {
        if (strcmp(argv[1], "rx") == 0)
        {
            rx_channels = atoi(argv[2]);
            rx_sr = atoi(argv[3]);
            rx_fmt = atoi(argv[4]);
            example_i2s_rx(rx_channels, rx_sr, rx_fmt);
        }

        if (strcmp(argv[1], "tx") == 0)
        {
            tx_channels = atoi(argv[2]);
            tx_sr = atoi(argv[3]);
            tx_fmt = atoi(argv[4]);
            example_i2s_tx(tx_channels, tx_sr, tx_fmt);
        }
    }

    return RT_EOK;
}

MSH_CMD_EXPORT_ALIAS(i2s_test, i2s, i2s rx test)

#endif


/**
 * @brief Common initialization.
 * 1. open audprc and audcodec device.
 * 2. create sems.
 */
static void device_init(void)
{
    int err = RT_EOK;

    /* create sem. */
    g_tx_ev = rt_event_create("tx_evt", RT_IPC_FLAG_FIFO);
    g_rx_ev = rt_event_create("rx_evt", RT_IPC_FLAG_FIFO);
    g_i2s_tx_ev = rt_event_create("i2s_tx", RT_IPC_FLAG_FIFO);
    g_i2s_rx_ev = rt_event_create("i2s_rx", RT_IPC_FLAG_FIFO);
    complete_sem = rt_sem_create("complete_sem", 0, RT_IPC_FLAG_FIFO);

#if defined(BSP_ENABLE_AUD_PRC) && defined(BSP_ENABLE_AUD_CODEC)
    /* find and open "audprc" */
    g_audprc_dev = rt_device_find(AUDPRC_DEVICE_NAME);
    if (NULL == g_audprc_dev)
    {
        rt_kprintf("[EX_I2S]Find audprc device failed.\n");
        return;
    }
    err = rt_device_open(g_audprc_dev, RT_DEVICE_FLAG_RDWR);
    if (RT_EOK != err)
    {
        rt_kprintf("[EX_I2S]Open audprc device failed. err=%d\n", err);
    }

    /* find and open "audcodec" */
    g_audcodec_dev = rt_device_find(AUDCODEC_DEVICE_NAME);
    if (NULL == g_audcodec_dev)
    {
        rt_kprintf("[EX_I2S]Find audcodec device failed.\n");
        return;
    }
    err = rt_device_open(g_audcodec_dev, RT_DEVICE_FLAG_WRONLY);
    if (RT_EOK != err)
    {
        rt_kprintf("[EX_I2S]Open audcodec device failed. err=%d\n", err);
    }
#endif

#ifdef BSP_ENABLE_I2S_CODEC
    /* find and open "i2s2" */
    g_i2s_dev = rt_device_find(I2S_DEVICE_NAME);
    if (NULL == g_i2s_dev)
    {
        rt_kprintf("[EX_I2S]Find i2s device failed.\n");
        return;
    }
    err = rt_device_open(g_i2s_dev, RT_DEVICE_FLAG_RDWR);
    if (RT_EOK != err)
    {
        rt_kprintf("[EX_I2S]Open i2s device failed. err=%d\n", err);
    }
#endif
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("[EX_I2S]I2S Example.\n");

    /* PIN CONFIG */
#ifdef BSP_ENABLE_I2S_CODEC
#ifdef SOC_SF32LB52X
    HAL_PIN_Set(PAD_PA06, I2S1_LRCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA05, I2S1_BCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA04, I2S1_SDI, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA03, I2S1_SDO, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA02, I2S1_MCLK, PIN_NOPULL, 1);
#elif defined(SOC_SF32LB56X)
    HAL_PIN_Set(PAD_PA71, I2S1_LRCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA40, I2S1_BCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, I2S1_SDI, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA39, I2S1_SDO, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA37, I2S1_MCLK, PIN_NOPULL, 1);
#elif defined(SOC_SF32LB58X)
    HAL_PIN_Set(PAD_PA84, I2S2_LRCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA91, I2S2_BCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA86, I2S2_SDI, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA82, I2S2_SDO, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA90, I2S2_MCLK, PIN_NOPULL, 1);
#else
#error "Need to confirm I2S pin config."
#endif
#else
#error "Need to confirm I2S pin config."
#endif

    device_init();

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }
    return 0;
}

