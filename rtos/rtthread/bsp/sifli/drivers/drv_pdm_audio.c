/**
  ******************************************************************************
  * @file   drv_pdm_audio.c
  * @author Sifli software development team
  * @brief   PDM Audio driver adaption layer
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include "board.h"
#include "drv_config.h"

#if defined(BSP_USING_PDM) ||defined(_SIFLI_DOXYGEN_)

#define LOG_TAG              "drv.pdm_audio"
//#define DBG_LVL            DBG_LOG //   DBG_INFO   //DBG_ERROR
#include "drv_log.h"


#if defined (SYS_HEAP_IN_PSRAM)
    #undef calloc
    #undef free
    #undef malloc
    extern void *app_sram_calloc(rt_size_t count, rt_size_t size);
    extern void app_sram_free(void *ptr);
    extern void *app_sram_alloc(rt_size_t size);
    #define  malloc(s)      app_sram_alloc(s)
    #define  calloc(c, s)   app_sram_calloc(c, s)
    #define  free(p)        app_sram_free(p)
#endif

struct bf0_pdm_audio
{
    struct rt_audio_device audio_device;    /*!< parent  audio device registerd to OS*/

    PDM_HandleTypeDef hpdm;
};


#ifdef BSP_USING_PDM1
    static struct bf0_pdm_audio h_pdm_audio1;
#endif /* BSP_USING_PDM1 */

#ifdef BSP_USING_PDM2
    static struct bf0_pdm_audio h_pdm_audio2;
#endif /* BSP_USING_PDM2 */

void HAL_PDM_RxCpltCallback(PDM_HandleTypeDef *hpdm)
{
    struct bf0_pdm_audio *hpdm_audio = rt_container_of(hpdm, struct bf0_pdm_audio, hpdm);
    //LOG_I("HAL_PDM_RxCpltCallback\n");
#ifdef SF32LB55X
    if (32 == hpdm->Init.ChannelDepth)
    {
        uint8_t shift8_prefix = 0;

        /*
            Shift received 32bit data:
                0x00FFABCD   ->  0xFFABCD00
        */
        rt_audio_rx_done(&(hpdm_audio->audio_device), &shift8_prefix, 1);
        rt_audio_rx_done(&(hpdm_audio->audio_device), hpdm->pRxBuffPtr + (hpdm->RxXferSize / 2), (hpdm->RxXferSize / 2) - 1);

    }
    else
#endif
    {
        rt_audio_rx_done(&(hpdm_audio->audio_device), hpdm->pRxBuffPtr + (hpdm->RxXferSize / 2), hpdm->RxXferSize / 2);
    }
}


void HAL_PDM_RxHalfCpltCallback(PDM_HandleTypeDef *hpdm)
{
    struct bf0_pdm_audio *hpdm_audio = rt_container_of(hpdm, struct bf0_pdm_audio, hpdm);

    //LOG_I("HAL_PDM_RxHalfCpltCallback\n");
#ifdef SF32LB55X
    if (32 == hpdm->Init.ChannelDepth)
    {
        uint8_t shift8_prefix = 0;

        /*
            Shift received 32bit data:
                0x00FFABCD   ->  0xFFABCD00
        */
        rt_audio_rx_done(&(hpdm_audio->audio_device), &shift8_prefix, 1);
        rt_audio_rx_done(&(hpdm_audio->audio_device), hpdm->pRxBuffPtr, (hpdm->RxXferSize / 2) - 1);
    }
    else
#endif
    {
        rt_audio_rx_done(&(hpdm_audio->audio_device), hpdm->pRxBuffPtr, hpdm->RxXferSize / 2);
    }
}



#ifdef BSP_USING_PDM1
void PDM1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_PDM_IRQHandler(&h_pdm_audio1.hpdm);

    //LOG_I("PDM1_IRQHandler");

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void PDM1_L_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("PDM1_L_DMA_IRQHandler\n");

    HAL_DMA_IRQHandler(h_pdm_audio1.hpdm.hdmarx);

    /* leave interrupt */
    rt_interrupt_leave();
}
void PDM1_R_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("PDM1_R_DMA_IRQHandler\n");

    HAL_DMA_IRQHandler(h_pdm_audio1.hpdm.hdmarx_r);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#endif /* BSP_USING_PDM1 */

#ifdef BSP_USING_PDM2
void PDM2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_PDM_IRQHandler(&h_pdm_audio2.hpdm);

    //LOG_I("PDM2_IRQHandler");

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void PDM2_L_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("PDM2_L_DMA_IRQHandler\n");

    HAL_DMA_IRQHandler(h_pdm_audio2.hpdm.hdmarx);

    /* leave interrupt */
    rt_interrupt_leave();
}
void PDM2_R_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("PDM2_R_DMA_IRQHandler\n");

    HAL_DMA_IRQHandler(h_pdm_audio2.hpdm.hdmarx_r);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#endif /* BSP_USING_PDM2 */

static rt_err_t bf0_audio_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;
    struct bf0_pdm_audio *hpdm_audio = (struct bf0_pdm_audio *) audio;
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *) & (hpdm_audio->hpdm);

    switch (caps->main_type)
    {
    case AUDIO_TYPE_QUERY: /* qurey the types of hw_codec device */
    {
        switch (caps->sub_type)
        {
        case AUDIO_TYPE_QUERY:
            caps->udata.mask = AUDIO_TYPE_INPUT;
            break;
        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }
    case AUDIO_TYPE_INPUT: /* Provide capabilities of OUTPUT unit */
        //case AUDIO_TYPE_OUTPUT:
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
            caps->udata.config.samplerate = hpdm->Init.SampleRate;
            caps->udata.config.samplefmt = hpdm->Init.ChannelDepth;
            if (hpdm->Init.Channels == PDM_CHANNEL_STEREO)
                caps->udata.config.channels = 2;
            else
                caps->udata.config.channels = 1;
            LOG_I("Get PDM samplerate %d, bits %d, channes %d\n", hpdm->Init.SampleRate, hpdm->Init.ChannelDepth, caps->udata.config.channels);
            break;
        case AUDIO_DSP_SAMPLERATE:
            caps->udata.config.samplerate = hpdm->Init.SampleRate;
            LOG_I("Get PDM samplerate %d\n", hpdm->Init.SampleRate);
            break;

        default:
            result = -RT_ERROR;
            break;
        }
        break;
    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}


/**
  * @brief  Config audio device.
  * @param[in]  audio: audio device handle.
  * @param[in]  caps: capability to config
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_configure(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    struct bf0_pdm_audio *hpdm_audio = (struct bf0_pdm_audio *) audio;
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *) & (hpdm_audio->hpdm);

    rt_err_t result = RT_EOK;

    switch (caps->main_type)
    {
    case AUDIO_TYPE_INPUT:
    {
        /*
             Can NOT put it in bf0_audio_init as it will be invoked once,

             but we except it been invoked before HAL_PDM_Config
        */
        HAL_RCC_EnableModule(RCC_MOD_PDM1);
        HAL_PDM_Init(hpdm);
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
        {
            hpdm->Init.SampleRate = (uint32_t) caps->udata.config.samplerate;

            if (1 == caps->udata.config.channels)
                hpdm->Init.Channels = PDM_CHANNEL_LEFT_ONLY;
            else
                hpdm->Init.Channels = PDM_CHANNEL_STEREO;
            hpdm->Init.ChannelDepth = (uint32_t) caps->udata.config.samplefmt;
            HAL_PDM_Config(hpdm, PDM_CFG_CHANNEL | PDM_CFG_SAMPLERATE | PDM_CFG_DEPTH);
            LOG_I("Config PDM channel %d, depth %d, samplerate %d\n", hpdm->Init.Channels, hpdm->Init.ChannelDepth, hpdm->Init.SampleRate);
        }
        break;

        case AUDIO_DSP_SAMPLERATE:              // Config audio sample rate
        {
            int rate = caps->udata.value;

            hpdm->Init.SampleRate = (uint32_t) rate;
            HAL_PDM_Config(hpdm, PDM_CFG_SAMPLERATE);
            LOG_I("Config PDM samplerate %d\n", hpdm->Init.SampleRate);
        }
        break;

        case AUDIO_DSP_CHANNELS:              // Config channel
        {
            int ch = caps->udata.value;
            if (1 == ch)
                hpdm->Init.Channels = PDM_CHANNEL_LEFT_ONLY;
            else
                hpdm->Init.Channels = PDM_CHANNEL_STEREO;
            HAL_PDM_Config(hpdm, PDM_CFG_CHANNEL);
            LOG_I("Config PDM channel %d\n", hpdm->Init.Channels);
        }
        break;

        case AUDIO_DSP_FMT:              // Config channel depth
        {
            int depth = caps->udata.value;

            hpdm->Init.ChannelDepth = (uint32_t) depth;
            HAL_PDM_Config(hpdm, PDM_CFG_DEPTH);
            LOG_I("Config PDM depth %d\n", hpdm->Init.ChannelDepth);
        }
        break;

        default:
        {
            result = -RT_ERROR;
        }
        break;
        }
    }
    break;


    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}


/**
  * @brief  Initialize audio device.
  * @param[in]  audio: audio device handle.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_init(struct rt_audio_device *audio)
{
    struct bf0_pdm_audio *hpdm_audio = (struct bf0_pdm_audio *) audio;
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *) & (hpdm_audio->hpdm);


    hpdm->Init.Mode = PDM_MODE_LOOP;
    hpdm->Init.Channels = PDM_CHANNEL_LEFT_ONLY;
    hpdm->Init.SampleRate = PDM_SAMPLE_16KHZ;
    hpdm->Init.ChannelDepth = PDM_CHANNEL_DEPTH_16BIT;
#ifndef ASIC  // on FPGA
    hpdm->Init.clkSrc = 3072000;
#else // for AISC
    hpdm->Init.clkSrc = 9600000;
#endif

    hpdm->RxXferSize = 0;
    hpdm->pRxBuffPtr = NULL;

    LOG_I("PDM audio init");

    return RT_EOK;
}

/**
  * @brief  Shtudown audio device.
  * @param[in]  audio: audio device handle.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_shutdown(struct rt_audio_device *audio)
{

    LOG_I("PDM audio shutdown");


    return RT_EOK;
}


/**
  * @brief  Start audio device for recording/playback.
  * @param[in]  audio: audio device handle.
  * @param[in]  stream: stream ID.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_start(struct rt_audio_device *audio, int stream)
{
    struct bf0_pdm_audio *hpdm_audio = (struct bf0_pdm_audio *) audio;
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *) & (hpdm_audio->hpdm);

    HAL_StatusTypeDef res = HAL_OK;

    // 3.072M = 49.152M(audpll)/16, 96k sampling use 3.072M as clock.
    if (hpdm->Init.clkSrc == 3072000 || hpdm->Init.SampleRate == PDM_SAMPLE_96KHZ)
    {
#ifndef SF32LB55X
        bf0_enable_pll(hpdm->Init.SampleRate, 0);
#endif
    }

    if (stream == AUDIO_STREAM_RECORD || stream == AUDIO_STREAM_PDM_PRESTART)
    {
        // init dma handle and request, other parameters configure in HAL driver
        hpdm->hdmarx = malloc(sizeof(DMA_HandleTypeDef));

        if (NULL == hpdm->hdmarx)
        {
            LOG_E("Alloc pdm dma fail\n");
            return RT_ENOMEM;
        }
        memset(hpdm->hdmarx, 0, sizeof(DMA_HandleTypeDef));

        hpdm->RxXferSize = 2 * CFG_AUDIO_RECORD_PIPE_SIZE;
        if (hpdm->Init.Channels >= PDM_CHANNEL_STEREO)
        {
            hpdm->RxXferSize *= 2;
        }
        hpdm->pRxBuffPtr = (uint8_t *) calloc(1, hpdm->RxXferSize);

        if (NULL == hpdm->pRxBuffPtr)
        {
            LOG_E("Alloc pdm buffer fail\n");
            free(hpdm->hdmarx);
            return RT_ENOMEM;
        }

        LOG_I("bf0_audio_init len=%d, hdmarx=%x", hpdm->RxXferSize, hpdm->hdmarx);
    }
    if (stream == AUDIO_STREAM_RECORD || stream == AUDIO_STREAM_PDM_START)
    {
        if (hwp_pdm1 == hpdm->Instance)
        {
#ifdef BSP_USING_PDM1
            hpdm->hdmarx->Instance                 = PDM1_L_DMA_INSTANCE;
            hpdm->hdmarx->Init.Request             = PDM1_L_DMA_REQUEST;
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(PDM1_L_DMA_IRQ);
            HAL_NVIC_EnableIRQ(PDM1_IRQn);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif /* BSP_USING_PDM1 */
        }
#ifdef BSP_USING_PDM2
        else if (hwp_pdm2 == hpdm->Instance)
        {

            hpdm->hdmarx->Instance                 = PDM2_L_DMA_INSTANCE;
            hpdm->hdmarx->Init.Request             = PDM2_L_DMA_REQUEST;
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(PDM2_L_DMA_IRQ);
            HAL_NVIC_EnableIRQ(PDM2_IRQn);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
        }
#endif /* BSP_USING_PDM2 */
        else
        {
            RT_ASSERT(0);
        }
        res = HAL_PDM_Receive_DMA(hpdm, hpdm->pRxBuffPtr, hpdm->RxXferSize);

        //LOG_I("bf0_audio_start buf=%x, len=%d", hpdm->pRxBuffPtr, hpdm->RxXferSize);
        //LOG_I("bf0_audio_start %d done\n", stream);

        return RT_EOK;
    }
    else
    {
#ifndef PKG_USING_3MICS
        LOG_E("Not support stream=%d", stream);
#endif
        return RT_EINVAL;
    }

}

/**
  * @brief  Stop audio device for recording/playback.
  * @param[in]  audio: audio device handle.
  * @param[in]  stream: stream ID.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_stop(struct rt_audio_device *audio, int stream)
{
    struct bf0_pdm_audio *hpdm_audio = (struct bf0_pdm_audio *) audio;
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *) & (hpdm_audio->hpdm);

    rt_err_t ret = RT_EOK;

    if (hpdm->Init.clkSrc == 3072000 || hpdm->Init.SampleRate == PDM_SAMPLE_96KHZ)
    {
#ifndef SF32LB55X
        bf0_disable_pll();
#endif
    }

    if (stream == AUDIO_STREAM_RECORD) // rx
    {
        LOG_I("PDM audio stop");

        if (hwp_pdm1 == hpdm->Instance)
        {
#ifdef BSP_USING_PDM1
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(PDM1_L_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            HAL_NVIC_DisableIRQ(PDM1_IRQn);
#endif /* BSP_USING_PDM1 */
        }
#ifdef BSP_USING_PDM2
        else if (hwp_pdm2 == hpdm->Instance)
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(PDM2_L_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            HAL_NVIC_DisableIRQ(PDM2_IRQn);
        }
#endif /* BSP_USING_PDM2 */

        ret = HAL_PDM_DMAStop(&(hpdm_audio->hpdm));
        HAL_PDM_DeInit(&(hpdm_audio->hpdm));

        if (PDM_ERROR_NONE != hpdm->ErrorCode)
        {
            LOG_E("PDM error happened %d", hpdm->ErrorCode);
        }

        if (hpdm->pRxBuffPtr)
        {
            free(hpdm->pRxBuffPtr);
            hpdm->pRxBuffPtr = NULL;
        }

        if (hpdm->hdmarx)
        {
            free(hpdm->hdmarx);
            hpdm->hdmarx = NULL;
        }
    }

    return ret;
}

/**
* @brief  Suspend audio device for recording/playback. (Currently unused)
* @param[in]  audio: audio device handle.
* @param[in]  stream: stream ID.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/

static rt_err_t bf0_audio_suspend(struct rt_audio_device *audio, int stream)
{
    rt_err_t ret = RT_ERROR;

    return ret;
}

/**
* @brief  Resume audio device for recording/playback. (Currently unused)
* @param[in]  audio: audio device handle.
* @param[in]  stream: stream ID.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t    bf0_audio_resume(struct rt_audio_device *audio, int stream)
{
    rt_err_t ret = RT_ERROR;


    return ret;
}

/**
* @brief  AUDIO controls. (Currently unused)
* @param[in]  audio: audio device handle.
* @param[in]  cmd: control commands.
* @param[in]  args: control command arguments.
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
static rt_err_t bf0_audio_control(struct rt_audio_device *audio, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    struct bf0_pdm_audio *hpdm_audio = (struct bf0_pdm_audio *) audio;
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *) & (hpdm_audio->hpdm);

    switch (cmd)
    {
    case AUDIO_CTL_SETVOLUME:
    {
        int val = (int)args;
        if (val < -30)
        {
            val = -30;
        }
        else if (val > 90)
        {
            val = 90;
        }
        val += 30;
        HAL_PDM_Set_Gain(hpdm, PDM_CHANNEL_STEREO, (uint8_t)val);
        break;
    }
    case AUDIO_CTL_HWRESET:
        break;
    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}


void set_pdm_gain_to_register(int val)
{
    struct bf0_pdm_audio *hpdm_audio = &h_pdm_audio1;
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *) & (hpdm_audio->hpdm);

    val += 30;
    HAL_PDM_Set_Gain(hpdm, PDM_CHANNEL_STEREO, (uint8_t)val);

}

/**
* @brief  AUDIO controls. (Currently unused)
* @param[in]  audio: audio device handle.
* @param[in]  writeBuf: write data buffer.
* @param[in]  readBuf: read data buffer.
* @param[in]  size:  read/write data size.
* @retval read/write data size
*/
static rt_size_t bf0_audio_trans(struct rt_audio_device *audio, const void *writeBuf, void *readBuf, rt_size_t size)
{
    struct bf0_pdm_audio *hpdm_audio = (struct bf0_pdm_audio *) audio;
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *) & (hpdm_audio->hpdm);

    HAL_StatusTypeDef res = HAL_OK;



    if (res != HAL_OK)
        return 0;

    return size;
}


static const struct rt_audio_ops       _g_audio_ops =
{
    .getcaps    = bf0_audio_getcaps,
    .configure  = bf0_audio_configure,

    .init       = bf0_audio_init,
    .shutdown   = bf0_audio_shutdown,
    .start      = bf0_audio_start,
    .stop       = bf0_audio_stop,
    .suspend    = bf0_audio_suspend,
    .resume     = bf0_audio_resume,
    .control    = bf0_audio_control,
    .transmit   = bf0_audio_trans,
};


/**
* @} PDM Audio_device
*/

/**
* @brief  PDM Audio devices initialization
*/
int rt_bf0_pdm_audio_init(void)
{
    int result;

#ifdef BSP_USING_PDM1
    memset(&h_pdm_audio1, 0, sizeof(h_pdm_audio1));

    h_pdm_audio1.hpdm.Instance = hwp_pdm1;
    h_pdm_audio1.audio_device.ops = (struct rt_audio_ops *)&_g_audio_ops;

    result = rt_audio_register((struct rt_audio_device *)&h_pdm_audio1,
                               "pdm1", RT_DEVICE_FLAG_RDONLY, NULL);
    RT_ASSERT(result == RT_EOK);
#endif /* BSP_USING_PDM1 */

#ifdef BSP_USING_PDM2
    memset(&h_pdm_audio2, 0, sizeof(h_pdm_audio2));

    h_pdm_audio2.hpdm.Instance = hwp_pdm2;
    h_pdm_audio2.audio_device.ops = (struct rt_audio_ops *)&_g_audio_ops;

    result = rt_audio_register((struct rt_audio_device *)&h_pdm_audio2,
                               "pdm2", RT_DEVICE_FLAG_RDONLY, NULL);
    RT_ASSERT(result == RT_EOK);
#endif /* BSP_USING_PDM2 */

    return result;
}

INIT_DEVICE_EXPORT(rt_bf0_pdm_audio_init);


//#define DRV_TEST
#if defined(DRV_TEST)

#include "string.h"
#include "drv_flash.h"


#define PDM_LOAD_FROM_FLASH
#define PDM_SAVE_MEM_BASE           (0x1c180000)

#define PDM_BUF_SIZE 1024
#define PDM_TEST_HNAME        "pdm1"
#define PDM_TEST_FLEN       (0x7f000)


static rt_device_t g_pdm;
static uint8_t g_pipe_data[PDM_BUF_SIZE];

static rt_thread_t rx_tid;
static rt_event_t g_rx_ev;

static char *pdm_buf = (char *)PDM_SAVE_MEM_BASE;
static int aud_flen = 0;

static int pdm_test_save(char *dst, uint8_t *src, uint32_t size)
{
    uint32_t addr = (uint32_t)dst;
    int res = 0;
#ifdef PDM_LOAD_FROM_FLASH
    // for flash , use flash write
    res = rt_flash_write(addr, src, size);
#else
    // for psram , use memcpy
    memcpy(dst, src, size);
    res = size;
#endif
    return res;
}

static void pdm_test_init_buf(char *buf, int data, uint32_t size)
{
    uint32_t addr = (uint32_t)buf;
#ifdef PDM_LOAD_FROM_FLASH
    // for flash, use erase
    rt_flash_erase(addr, size);
#else
    // for psram
    memset(buf, data, size);
#endif
    return;
}

typedef struct
{
    uint8_t riff[4];
    uint32_t lenth;
    uint8_t wave[4];
    uint8_t fmt[4];
    uint32_t size1;
    uint16_t fmt_tag;
    uint16_t channel;
    uint32_t sampleRate;
    uint32_t bytePerSec;
    uint16_t blockAlign;
    uint16_t bitPerSample;
    uint8_t data[4];
    uint32_t size2;
} AUD_WAV_HDR_T;


static void pdm_fill_wav_header(uint32_t sr, uint16_t channel, uint16_t bps)
{
    AUD_WAV_HDR_T hdr;

    hdr.riff[0] = 'R';
    hdr.riff[1] = 'I';
    hdr.riff[2] = 'F';
    hdr.riff[3] = 'F';
    hdr.lenth = PDM_TEST_FLEN + 36;
    hdr.wave[0] = 'W';
    hdr.wave[1] = 'A';
    hdr.wave[2] = 'V';
    hdr.wave[3] = 'E';
    hdr.fmt[0] = 'f';
    hdr.fmt[1] = 'm';
    hdr.fmt[2] = 't';
    hdr.fmt[3] = ' ';
    hdr.size1 = 16;
    hdr.fmt_tag = 1;
    hdr.channel = channel; // 1
    hdr.sampleRate = sr;
    //hdr.bytePerSec = 32000;
    hdr.blockAlign = 2;
    hdr.bitPerSample = bps; // 16
    hdr.bytePerSec = hdr.sampleRate * hdr.channel * hdr.bitPerSample / 8;
    hdr.data[0] = 'd';
    hdr.data[1] = 'a';
    hdr.data[2] = 't';
    hdr.data[3] = 'a';
    hdr.size2 = PDM_TEST_FLEN; //;    // record data lenght
    LOG_I("Save wav sampleRate %d, bitPerSample %d, channel %d\n", hdr.sampleRate, hdr.bitPerSample, hdr.channel);

    pdm_test_init_buf(pdm_buf, 0, 0x80000);
    pdm_test_save(pdm_buf, (uint8_t *)&hdr, 44);
    aud_flen = 0;
    pdm_buf += 44;
}


void bf0_pdm_rx_entry(void *param)
{
    rt_uint32_t evt;
    int size;

    g_rx_ev = rt_event_create("audio_evt", RT_IPC_FLAG_FIFO);
    rt_kprintf("bf0_pdm_rx_entry started, wait event\n");
    while (1)
    {
        rt_event_recv(g_rx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        //LOG_I("Got Audio event 0x%x\n", evt);
        while (1)
        {
            rt_size_t len;
            len = rt_device_read(g_pdm, 0, g_pipe_data, PDM_BUF_SIZE);

            if (aud_flen + len <= PDM_TEST_FLEN)
            {
                pdm_test_save(pdm_buf, g_pipe_data, len);
                pdm_buf += len;
                aud_flen += len;
            }

            if (len < PDM_BUF_SIZE)
                break;
        }
    }
}

static rt_err_t audio_rx_ind(rt_device_t dev, rt_size_t size)
{
    //LOG_I("audio_rx_ind %d\n", size);
    rt_event_send(g_rx_ev, 1);
    return RT_EOK;
}

int cmd_pdm(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "open") == 0)
        {
            if (g_pdm == NULL)
            {
                g_pdm = rt_device_find(PDM_TEST_HNAME);
                if (g_pdm)
                {
                    rt_device_init(g_pdm);
                    rt_device_open(g_pdm, RT_DEVICE_FLAG_RDONLY);
                    LOG_I("PDM opened\n");
                }
                else
                {
                    LOG_E("Could not find PDM device\n");
                    return -RT_ERROR;
                }
            }
        }
        if (strcmp(argv[1], "config") == 0)
        {
            if (g_pdm)
            {
                struct rt_audio_caps caps;

                caps.main_type = AUDIO_TYPE_INPUT;
                if (strcmp(argv[2], "sr") == 0)
                {
                    caps.sub_type = AUDIO_DSP_SAMPLERATE;
                    caps.udata.value = atoi(argv[3]);
                    rt_device_control(g_pdm, AUDIO_CTL_CONFIGURE, &caps);
                }
                if (strcmp(argv[2], "ch") == 0)
                {
                    caps.sub_type = AUDIO_DSP_CHANNELS;
                    int value = atoi(argv[3]);
                    caps.udata.value = value > 1 ? 2 : 1;
                    rt_device_control(g_pdm, AUDIO_CTL_CONFIGURE, &caps);
                }
                if (strcmp(argv[2], "bit") == 0)
                {
                    caps.sub_type = AUDIO_DSP_FMT;
                    caps.udata.value = atoi(argv[3]);
                    rt_device_control(g_pdm, AUDIO_CTL_CONFIGURE, &caps);
                }
                if (strcmp(argv[2], "param") == 0)
                {
                    caps.sub_type = AUDIO_DSP_PARAM;
                    caps.udata.config.channels = atoi(argv[3]);
                    caps.udata.config.samplefmt = atoi(argv[4]);    // depth
                    caps.udata.config.samplerate = atoi(argv[5]);
                    rt_device_control(g_pdm, AUDIO_CTL_CONFIGURE, &caps);
                }
            }
        }
        if (strcmp(argv[1], "start") == 0)
        {
            if (g_pdm)
            {
                int stream = 0;

                if (strcmp(argv[2], "rx") == 0)
                {
                    stream = AUDIO_STREAM_RECORD;
                    struct rt_audio_caps caps;
                    caps.main_type = AUDIO_TYPE_INPUT;
                    caps.sub_type = AUDIO_DSP_PARAM;
                    rt_device_control(g_pdm, AUDIO_CTL_GETCAPS, &caps);
                    pdm_fill_wav_header(caps.udata.config.samplerate, (uint16_t)caps.udata.config.channels, (uint16_t)caps.udata.config.samplefmt);
                    // start record thread
                    rx_tid = rt_thread_create("aud_th", bf0_pdm_rx_entry, g_pdm, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
                    if (rx_tid == NULL)
                    {
                        LOG_E("Create pdm rx thread fail\n");
                        return RT_ERROR;
                    }
                    rt_thread_startup(rx_tid);
                    rt_device_set_rx_indicate(g_pdm, audio_rx_ind);
                    rt_device_control(g_pdm, AUDIO_CTL_START, &stream);
                }
            }
        }
        if (strcmp(argv[1], "stop") == 0)
        {
            if (g_pdm)
            {
                int stream = 0;

                if (strcmp(argv[2], "rx") == 0)
                {
                    stream = AUDIO_STREAM_RECORD;
                    rt_device_control(g_pdm, AUDIO_CTL_STOP, &stream);
                }

            }
        }

    }
    return RT_EOK;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pdm, __cmd_pdm, Test pdm driver);

#endif  // DRV_TEST

#endif
