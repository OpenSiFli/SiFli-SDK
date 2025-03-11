#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* user start */
#include "drv_flash.h"
#include <drivers/audio.h>
#ifdef PA_USING_AW87390
    #include "sifli_aw87390.h"
#endif
#ifdef PA_USING_AW8155
    #include "sifli_aw8155.h"
#endif
#ifdef PA_USING_AW882XX
    #include "sifli_aw882xx.h"
#endif
#include "mem_section.h"
/* user end */


/* Common functions for RT-Thread based platform -----------------------------------------------*/


/* User code start from here --------------------------------------------------------*/

#ifndef FS_REGION_START_ADDR
    #error "FS_REGION_START_ADDR is not defined."
#endif

#define AUDCODEC_DEVICE_NAME "audcodec"
#define AUDPRC_DEVICE_NAME   "audprc"

#define AUDIO_BUF_SIZE  (640) //1024
#define AUDRX_BUF_MAX (1024*1024)

static uint8_t g_pipe_data[AUDIO_BUF_SIZE];
static rt_event_t g_tx_ev, g_rx_ev;
static rt_sem_t complete_sem = NULL;
rt_device_t g_audprc_dev = NULL, g_audcodec_dev = NULL;
rt_thread_t g_rx_tx_tid;

L2_RET_BSS_SECT_BEGIN(g_audrx_buf)
ALIGN(4) uint8_t g_audrx_buf[AUDRX_BUF_MAX] L2_RET_BSS_SECT(g_audrx_buf);
L2_RET_BSS_SECT_END
uint32_t g_tmp_pos = 0;

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
    // rt_kprintf("audio_rx_ind %d\n", size);
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
    // rt_kprintf("audio_tx_done \n");
    rt_event_send(g_tx_ev, 1);
    return RT_EOK;
}

/**
 * @brief PA initialization.
 */
static void audio_pa_init(void)
{
#ifdef PA_USING_AW87390
    sifli_aw87390_init();
#elif defined(PA_USING_AW8155)
    /* do nothing */
#elif defined(PA_USING_AW882XX)
    rt_aw882xx_init();
#else
#error "PA NOT DEFINED."
#endif
}

/**
 * @brief Open PA.
 */
static void audio_pa_open(rt_uint32_t samplerate, rt_uint32_t reserved)
{
#ifdef PA_USING_AW87390
    (void)samplerate;
    (void)reserved;
    sifli_aw87390_start();
#elif defined(PA_USING_AW8155)
    (void)samplerate;
    (void)reserved;
    sifli_aw8155_start();
#elif defined(PA_USING_AW882XX)
    (void)reserved;
    sifli_aw882xx_start(samplerate, 3);
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
#elif defined(PA_USING_AW882XX)
    sifli_aw882xx_stop();
#else
#error "PA NOT DEFINED."
#endif
}

/**
 * @brief The thread used to read rx audio data and write to tx device (Loopback test).
 */
static void audprc_rx_tx_entry(void *param)
{
    rt_uint32_t evt;
    rt_size_t len;

    rt_kprintf("%s\n", __func__);
    memset(g_audrx_buf, 0, sizeof(g_audrx_buf));
    while (1)
    {
        rt_event_recv(g_rx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        while (1)
        {
            /* RX read (from mic)*/
            len = rt_device_read(g_audprc_dev, 0, g_pipe_data, (AUDIO_BUF_SIZE / 2));
            // rt_kprintf("len = %d\n", len);
            if (len != (AUDIO_BUF_SIZE / 2))
            {
                rt_kprintf("Got abnormal audio size = %d\n", len);
            }

            /* TX write (to speaker)*/
            len = rt_device_write(g_audprc_dev, 0, g_pipe_data, (AUDIO_BUF_SIZE / 2));
            if (len != (AUDIO_BUF_SIZE / 2))
            {
                rt_kprintf("Write abnormal size = %d\n", len);
            }
        }
    }
}

/**
 * @brief The thread used to read rx audio data and save to the ram buffer,
 *        the ram buffer'size is up to AUDIORX_BUF_MAX.
 */
static void audprc_rx_entry(void *param)
{
    rt_uint32_t evt;
    rt_size_t len;

    rt_kprintf("%s\n", __func__);
    g_tmp_pos = 0;
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
                rt_kprintf("Got abnormal audio size = %d\n", len);
            }

            /* Save to RAM Buffer. */
            if ((g_tmp_pos + (AUDIO_BUF_SIZE / 2)) < AUDRX_BUF_MAX)
            {
                memcpy(&g_audrx_buf[g_tmp_pos], g_pipe_data, AUDIO_BUF_SIZE / 2);
                g_tmp_pos += (AUDIO_BUF_SIZE / 2);
            }
            else
            {
                /* Record buffer is up to  AUDRX_BUF_MAX. Stop recording. */
                goto __EXIT;
            }
        }
    }

__EXIT:
    rt_kprintf("Record finished.\n");
    rt_sem_release(complete_sem);
}

/**
 * @brief The thread used to read audio data from ram buffer, and then write to tx device (speaker).
 */
static void audprc_tx_entry(void *param)
{
    rt_uint32_t evt;
    uint32_t wr_offset = 0;

    rt_kprintf("%s\n", __func__);

    while (1)
    {
        rt_event_recv(g_tx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

        int len = rt_device_write(g_audprc_dev, 0, &g_audrx_buf[wr_offset], AUDIO_BUF_SIZE / 2);
        if (len != (AUDIO_BUF_SIZE / 2))
        {
            rt_kprintf("Abnormal write len :%d\n", len);
        }
        wr_offset += (AUDIO_BUF_SIZE / 2);
        if (wr_offset + (AUDIO_BUF_SIZE / 2)  >= AUDRX_BUF_MAX)
        {
            goto __EXIT;
        }
    }

__EXIT:
    rt_kprintf("Play finished.\n");
    rt_sem_release(complete_sem);
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

    rt_kprintf("codec input parameter:sub_type=%d channels %d, rate %d, bits %d", caps.sub_type, caps.udata.config.channels,
               caps.udata.config.samplerate, caps.udata.config.samplefmt);

    /* AUDPRC : set input as "AUDPRC_RX_FROM_CODEC" */
    rt_device_control(g_audprc_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);
    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = HAL_AUDPRC_RX_CH0 - HAL_AUDPRC_RX_CH0;
    caps.udata.config.channels   = channels;
    caps.udata.config.samplerate = sr;
    caps.udata.config.samplefmt = fmt;
    rt_kprintf("mic input:rx channel %d, channels %d, rate %d, bitwidth %d", 0, caps.udata.config.channels,
               caps.udata.config.samplerate, caps.udata.config.samplefmt);
    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);
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
#if defined(BSP_USING_BOARD_EM_LB587XXX)
    caps.sub_type |= (1 << HAL_AUDCODEC_DAC_CH1);
#endif
    caps.udata.config.channels   = channels; // L,R,L,R,L,R, ......
    caps.udata.config.samplerate = sr;
    caps.udata.config.samplefmt = fmt; //8 16 24 or 32
    rt_kprintf("prc_codec : sub_type=%d channel %d, samplerate %d, bits %d", caps.sub_type, caps.udata.config.channels,
               caps.udata.config.samplerate, caps.udata.config.samplefmt);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);

    /* AUDPRC : set output as "AUDPRC_TX_TO_CODEC" (codec/mem/i2s). */
    rt_device_control(g_audprc_dev, AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);
    cfg.channel = channels;
    cfg.source_sr = sr;
    cfg.dest_sr = sr;
    rt_kprintf("speaker OUTPUTSRC channel=%d in_rate=%d out_rate=%d", cfg.channel, cfg.source_sr, cfg.dest_sr);
    rt_device_control(g_audprc_dev, AUDIO_CTL_OUTPUTSRC, (void *)(&cfg));

    /* AUDPRC : SELECTOR */
    rt_kprintf("speaker select=0x%x mixer=0x%x", out_sel, mixer_sel);
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
    rt_kprintf("tx[0]: sub_type %d, ch %d, samrate %d, bits %d\n",
               caps.sub_type,
               caps.udata.config.channels,
               caps.udata.config.samplerate,
               caps.udata.config.samplefmt);

    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    /* Set volume */
    int vol = -18;
    rt_kprintf("init volume=%d", vol);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_SETVOLUME, (void *)vol);
}

/**
 * @brief Start RX.
 */
static void start_rx(void)
{
    int stream;

    rt_kprintf("%s\n", __func__);
    stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_START, &stream);
    stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
    rt_device_set_rx_indicate(g_audprc_dev, audio_rx_ind);
    rt_device_control(g_audprc_dev, AUDIO_CTL_START, &stream);
}

/**
 * @brief Stop RX.
 */
static void stop_rx(void)
{
    int stream;

    rt_kprintf("%s\n", __func__);
    stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_STOP, &stream);
    stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
    rt_device_set_rx_indicate(g_audprc_dev, audio_rx_ind);
    rt_device_control(g_audprc_dev, AUDIO_CTL_STOP, &stream);
    // audio_pa_close();
}

/**
 * @brief Start TX and open PA.
 */
static void start_tx(rt_uint32_t samplerate)
{
    int stream_audprc, stream_audcodec;

    rt_kprintf("%s\n", __func__);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_MUTE, (void *)1);
    // rt_base_t level = rt_hw_interrupt_disable();
    stream_audcodec = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    stream_audprc   = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDPRC_TX_CH0) << 8);

    rt_device_control(g_audcodec_dev, AUDIO_CTL_START, &stream_audcodec);
    rt_device_set_tx_complete(g_audprc_dev, audio_tx_done);
    rt_device_control(g_audprc_dev, AUDIO_CTL_START, &stream_audprc);
    // rt_hw_interrupt_enable(level);

    /* Open PA. */
    audio_pa_open(samplerate, 0);

    rt_thread_mdelay(30);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_MUTE, (void *)0);
}

/**
 * @brief Stop TX and close PA.
 */
static void stop_tx(void)
{
    int stream_audprc, stream_audcodec;

    rt_kprintf("%s\n", __func__);
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
 * @brief Start TX and RX for AUDCODEC and AUDPRC.
 */
static void start_txrx(rt_uint32_t samplerate)
{
    int stream_audprc, stream_audcodec;

    rt_kprintf("%s\n", __func__);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_MUTE, (void *)1);
    // rt_base_t level = rt_hw_interrupt_disable();
    stream_audcodec = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDCODEC_ADC_CH0) << 8) | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    stream_audprc   = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDPRC_RX_CH0) << 8) | ((1 << HAL_AUDPRC_TX_CH0) << 8);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_START, &stream_audcodec);
    rt_device_set_tx_complete(g_audprc_dev, audio_tx_done);
    rt_device_set_rx_indicate(g_audprc_dev, audio_rx_ind);
    rt_device_control(g_audprc_dev, AUDIO_CTL_START, &stream_audprc);
    // rt_hw_interrupt_enable(level);

    /* Open PA */
    audio_pa_open(samplerate, 0);
    rt_thread_mdelay(30);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_MUTE, (void *)0);
}

/**
 * @brief Stop TX and RX , and close PA.
 */
static void stop_txrx(void)
{
    int stream_audprc, stream_audcodec;

    stream_audcodec = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDCODEC_ADC_CH0) << 8) | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    stream_audprc   = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDPRC_RX_CH0) << 8) | ((1 << HAL_AUDPRC_TX_CH0) << 8);
    rt_device_control(g_audcodec_dev, AUDIO_CTL_STOP, &stream_audcodec);
    rt_device_control(g_audprc_dev, AUDIO_CTL_STOP, &stream_audprc);
    /* Close PA */
    audio_pa_close();
}

/**
 * @brief Example : record from mic and play to speaker (Loopback).
 * @param rx_channels 1(single) / 2 (stereo)
 * @param rx_sr sample rate , 8000/12000/16000/24000/32000/48000/11025/22050/44010, see <user manual>.
 * @param rx_fmt depth, see <user manual>
 * @param tx_channels 1(single) / 2 (stereo)
 * @param tx_sr sample rate , 8000/12000/16000/24000/32000/48000/11025/22050/44010, see <user manual>.
 * @param tx_fmt depth, see <user manual>
 */
static void example_audprc_tx_rx(rt_uint32_t rx_channels, rt_uint32_t rx_sr, rt_uint32_t rx_fmt,
                                 rt_uint32_t tx_channels, rt_uint32_t tx_sr, rt_uint32_t tx_fmt)
{
    rt_kprintf("%s\n", __func__);

    if (g_rx_tx_tid)
    {
        rt_kprintf("%s failed. Thread has been existed.\n");
        return;
    }
    g_rx_tx_tid = rt_thread_create("audtx_rx", audprc_rx_tx_entry, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (g_rx_tx_tid == NULL)
    {
        rt_kprintf("Create audtx_rx thread fail\n");
        return;
    }
    rt_thread_startup(g_rx_tx_tid);

    config_rx(rx_channels, rx_sr, rx_fmt);
    config_tx(tx_channels, tx_sr, tx_fmt);
    start_txrx(tx_sr);
}

/**
 * @brief Example : Stop loopback.
 */
static void example_audprc_tx_rx_stop(void)
{
    rt_kprintf("%s\n", __func__);
    stop_txrx();
    if (g_rx_tx_tid)
        rt_thread_delete(g_rx_tx_tid);
    g_rx_tx_tid = NULL;
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
    // create rx thread
    rt_thread_t rx_tid = rt_thread_create("audprc_rx", audprc_rx_entry, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (rx_tid == NULL)
    {
        rt_kprintf("Create rx thread fail\n");
        return;
    }
    rt_thread_startup(rx_tid);

    config_rx(channels, sr, fmt);
    start_rx();

    rt_sem_take(complete_sem, RT_WAITING_FOREVER);
    rt_kprintf("RX finished.\n");
    stop_rx();
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
        rt_kprintf("Create audprc_tx thread fail\n");
        return;
    }
    rt_thread_startup(tx_tid);

    config_tx(tx_channels, tx_sr, tx_fmt);
    start_tx(tx_sr);

    rt_sem_take(complete_sem, RT_WAITING_FOREVER);
    rt_kprintf("Play finished.\n");
    stop_tx();
}

/**
 * @brief test cmd for audprc & audcodec.
 * Command details see Readme.md
 */
int audprc_test(int argc, char *argv[])
{
    rt_uint32_t rx_channels = 0, rx_sr = 0, rx_fmt = 0;
    rt_uint32_t tx_channels = 0, tx_sr = 0, tx_fmt = 0;

    if (argc > 1)
    {
        if (strcmp(argv[1], "txrx") == 0)
        {
            rx_channels = atoi(argv[2]);
            rx_sr = atoi(argv[3]);
            rx_fmt = atoi(argv[4]);
            tx_channels = atoi(argv[5]);
            tx_sr = atoi(argv[6]);
            tx_fmt = atoi(argv[7]);
            example_audprc_tx_rx(rx_channels, rx_sr, rx_fmt, tx_channels, tx_sr, tx_fmt);
        }

        if (strcmp(argv[1], "txrxstop") == 0)
        {
            example_audprc_tx_rx_stop();
        }

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

/**
 * @brief Common initialization.
 * 1. open audprc and audcodec device.
 * 2. create sems.
 */
static void audprc_init(void)
{
    int err = RT_EOK;

    // open device
    g_audprc_dev = rt_device_find(AUDPRC_DEVICE_NAME);
    if (NULL == g_audprc_dev)
    {
        rt_kprintf("Find audprc device failed.\n");
        return;
    }
    err = rt_device_open(g_audprc_dev, RT_DEVICE_FLAG_RDWR);
    if (RT_EOK != err)
    {
        rt_kprintf("Open audprc device failed. err=%d\n", err);
    }

    g_audcodec_dev = rt_device_find(AUDCODEC_DEVICE_NAME);
    if (NULL == g_audcodec_dev)
    {
        rt_kprintf("Find audcodec device failed.\n");
        return;
    }
    err = rt_device_open(g_audcodec_dev, RT_DEVICE_FLAG_WRONLY);
    if (RT_EOK != err)
    {
        rt_kprintf("Open audcodec device failed. err=%d\n", err);
    }

    // create sem and threads.
    g_tx_ev = rt_event_create("audioprc_tx_evt", RT_IPC_FLAG_FIFO);
    g_rx_ev = rt_event_create("audio_rx_evt", RT_IPC_FLAG_FIFO);
    complete_sem = rt_sem_create("complete_sem", 0, RT_IPC_FLAG_FIFO);

    // PA initialization
    audio_pa_init();
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Audprc Example.\n");

    audprc_init();

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }
    return 0;
}

