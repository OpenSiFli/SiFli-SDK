/**
  ******************************************************************************
  * @file   drv_i2s_mic.c
  * @author Sifli software development team
  * @brief   Audio driver adaption layer
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
#include "board.h"
#include "drv_config.h"

#if defined(BSP_ENABLE_I2S_MIC)||defined(_SIFLI_DOXYGEN_)

//#define DBG_LEVEL                      DBG_LOG
#define LOG_TAG              "drv.i2s_mic"
#include "drv_log.h"

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_audio Audio
  * @brief Audio BSP driver
  * This driver use DMA to driver I2S interface, support audio capture functions.
  * It register "mic0" devices to OS. User could open this device to config and capture audio
  * @{
  */

struct i2s_audio_cfg_t
{
    DMA_Channel_TypeDef   *dma_handle;      /*!< DMA device Handle used by this driver */
    I2S_TypeDef        *i2s_handle;         /*!< I2S device Handle used by this driver */
    char               *name;               /*!< Audio device name, for example, 'mic' for recording device */
    rt_uint8_t          dma_request;        /*!< DMA request type for I2S, defined in dma_config.h */
    rt_uint8_t          is_record;          /*!< Audio device type, 1: for recording, 0: for playback*/
    rt_uint8_t          reqdma_tx;        /*!< DMA request type for I2S TX */
    DMA_Channel_TypeDef   *hdma_tx;      /*!< DMA device Handle used I2S TX */
};

struct bf0_i2s_audio
{
    struct rt_audio_device audio_device;    /*!< audio device registerd to OS*/
    I2S_HandleTypeDef hi2s;
    uint8_t *rx_buf;
    uint8_t *tx_buf;
};


#define AUDIO_DATA_SIZE 1920 //480
static uint8_t audio_data[AUDIO_DATA_SIZE];
static uint8_t audio_tx_data[AUDIO_DATA_SIZE];
#ifdef ASIC
#if 0 //xtal
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 125, 125,  5}, {44100, 136, 136,  4}, {32000, 185, 190,  5}, {24000, 250, 250, 10}, {22050, 272, 272,  8},
    {16000, 384, 384, 12}, {12000, 500, 500, 20}, {11025, 544, 544, 16}, { 8000, 750, 750, 30}
};//{16000, 375, 375, 15}  { 8000, 750, 750, 30}} { 8000, 768, 768, 24}
#else  //PLL
//PLL 16k 49.152M  44.1k  45.1584M
//lrclk_duty_high:PLL/spclk_div/samplerate/2: 64=49.152M/48k/8/2
//bclk:lrclk_duty_high/32
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 64, 64,  2}, {44100, 64, 64,  2}, {32000, 96, 96,  3}, {24000, 128, 128, 4}, {22050, 128, 128,  4},
    {16000, 192, 192, 6}, {12000, 256, 256, 8}, {11025, 256, 256, 8}, { 8000, 384, 384, 12}
};
#endif
#else
//clk:3.072M  spclk:1  only 16k 8k used
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 64, 64,  2}, {44100, 64, 64,  2}, {32000, 96, 96,  3}, {24000, 128, 128, 4}, {22050, 128, 128,  4},
    {16000, 96, 96,  3}, {12000, 256, 256, 8}, {11025, 256, 256, 8}, { 8000, 192, 192, 6}
};
#endif


/**
 *  Register and use Mic device
*/


static struct i2s_audio_cfg_t bf0_i2s_audio_obj[1] =
{
#ifdef BSP_ENABLE_I2S_MIC
    BF0_MIC_CONFIG,                         /*!< Mic driver configuration*/
#endif // BSP_ENABLE_I2S_MIC
};

static struct bf0_i2s_audio h_i2s_mic;


static void audio_debug_out_i2sr()
{
    I2S_TypeDef *hi2s = h_i2s_mic.hi2s.Instance;
    LOG_I("RX_RE_SAMPLE_CLK_DIV = 0X%x\n", hi2s->RX_RE_SAMPLE_CLK_DIV);
    LOG_I("AUDIO_RX_LRCK_DIV = 0X%x\n", hi2s->AUDIO_RX_LRCK_DIV);
    LOG_I("AUDIO_RX_BCLK_DIV = 0X%x\n", hi2s->AUDIO_RX_BCLK_DIV);
    LOG_I("AUDIO_RX_SERIAL_TIMING = 0X%x\n", hi2s->AUDIO_RX_SERIAL_TIMING);
    LOG_I("AUDIO_RX_PCM_DW = 0X%x\n", hi2s->AUDIO_RX_PCM_DW);
    LOG_I("RECORD_FORMAT = 0X%x\n", hi2s->RECORD_FORMAT);
    LOG_I("RX_CH_SEL = 0X%x\n", hi2s->RX_CH_SEL);
    LOG_I("DMA_MASK = 0X%x\n", hi2s->DMA_MASK);
    LOG_I("AUDIO_RX_FUNC_EN = 0X%x\n", hi2s->AUDIO_RX_FUNC_EN);
    LOG_I("AUDIO_RX_PAUSE = 0X%x\n", hi2s->AUDIO_RX_PAUSE);

}

static void audio_debug_out_rxdma()
{
    DMA_Channel_TypeDef *hdma = h_i2s_mic.hi2s.hdmarx->Instance;
    LOG_I("RX CCR = 0X%x\n", hdma->CCR);
    LOG_I("RX CNDTR = 0X%x\n", hdma->CNDTR);
    LOG_I("RX CPAR = 0X%x\n", hdma->CPAR);
    LOG_I("RX CM0AR = 0X%x\n", hdma->CM0AR);
}

/** @defgroup Audio_device Audio device functions registered to OS
 * @ingroup drv_audio
 * @{
 */

/**
  * @brief  Get audio device capabilities.
  * @param[in]      audio: audio device handle.
  * @param[in,out]  caps: capability to get
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;

    switch (caps->main_type)
    {
    case AUDIO_TYPE_QUERY: /* qurey the types of hw_codec device */
    {
        switch (caps->sub_type)
        {
        case AUDIO_TYPE_QUERY:
            caps->udata.mask = AUDIO_TYPE_INPUT;
            caps->udata.mask |= AUDIO_TYPE_OUTPUT;
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
            if (audio->replay == NULL)
            {
                result = -RT_ERROR;
                break;
            }
            // use samplefmt for input width, samplefmts for output width, samplerate for real number but not flag
            caps->udata.config.channels     = (aud->hi2s.Init.rx_cfg.track == 1) ? 1 : 2;
            caps->udata.config.samplefmt    = aud->hi2s.Init.rx_cfg.data_dw; //AUDIO_FMT_PCM_U16_LE;
            caps->udata.config.samplerate   = aud->hi2s.Init.rx_cfg.sample_rate; //AUDIO_SAMP_RATE_16K;

            break;
        case AUDIO_DSP_SAMPLERATE:
            caps->udata.value = aud->hi2s.Init.rx_cfg.sample_rate;
            //LOG_I("bf0_audio_getcaps %d\n", caps->udata.value);
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
    rt_err_t result = RT_EOK;
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;

    switch (caps->main_type)
    {
    case AUDIO_TYPE_INPUT:
    {
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
        {
            I2S_HandleTypeDef *hi2s = &(aud->hi2s);
            uint8_t index;
            for (index = 0; index < 9; index++)
            {
                if (txrx_clk_div[index].samplerate == caps->udata.config.samplerate)
                {
                    break;
                }
            }
            RT_ASSERT(index < 9);
            hi2s->Init.rx_cfg.sample_rate = caps->udata.config.samplerate;
            if (caps->udata.config.channels == 1)
                hi2s->Init.rx_cfg.track = 1; //(uint8_t)caps->udata.config.channels;
            else
                hi2s->Init.rx_cfg.track = 0;
            hi2s->Init.rx_cfg.data_dw = (uint8_t)caps->udata.config.samplefmt;
            hi2s->Init.rx_cfg.clk_div_index = index;
            hi2s->Init.rx_cfg.clk_div = &txrx_clk_div[hi2s->Init.rx_cfg.clk_div_index];
            HAL_I2S_Config_Receive(hi2s, &(hi2s->Init.rx_cfg));
        }
        break;
        case AUDIO_DSP_SAMPLERATE:              // Config audio sample rate
        {
            int rate = caps->udata.value;
            I2S_HandleTypeDef *hi2s = &(aud->hi2s);
            uint8_t index;
            for (index = 0; index < 9; index++)
            {
                if (txrx_clk_div[index].samplerate == caps->udata.config.samplerate)
                {
                    break;
                }
            }
            RT_ASSERT(index < 9);
            hi2s->Init.rx_cfg.sample_rate = rate;
            hi2s->Init.rx_cfg.clk_div_index = index;
            hi2s->Init.rx_cfg.clk_div = &txrx_clk_div[hi2s->Init.rx_cfg.clk_div_index];
            HAL_I2S_Config_Receive(hi2s, &(hi2s->Init.rx_cfg));
        }
        break;
        case AUDIO_DSP_CHANNELS:              // Config channel
        {
            int chnl = caps->udata.value;
            I2S_HandleTypeDef *hi2s = &(aud->hi2s);
            hi2s->Init.rx_cfg.track = (chnl == 1) ? 1 : 0;
            HAL_I2S_Config_Receive(hi2s, &(hi2s->Init.rx_cfg));
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
    return RT_EOK;
}

/**
  * @brief  Shtudown audio device.
  * @param[in]  audio: audio device handle.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_shutdown(struct rt_audio_device *audio)
{
    return RT_EOK;
}


/**
  * @brief  Start audio device for recording/playback.
  * @param[in]  audio: audio device handle.
  * @param[in]  stream: stream ID.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */

static rt_err_t bf0_audio_i2s_start(struct bf0_i2s_audio *aud)
{
    I2S_HandleTypeDef *hi2s = &aud->hi2s;

    bf0_enable_pll(hi2s->Init.rx_cfg.sample_rate, 0);


#ifndef ASIC  //i2s mic on FPGA  
    /*FPGA have NONE I2S TX device*/

#else    //i2s codec on evb_z0
#ifndef SF32LB55X
    HAL_RCC_EnableModule(RCC_MOD_I2S1);
    // TODO: set to xtal now, change it if PLL can used
    //__HAL_I2S_CLK_XTAL(hi2s);   // xtal use 48M for asic
    //__HAL_I2S_SET_SPCLK_DIV(hi2s, 4);   // set to 12M to i2s
    //hi2s->Init.src_clk_freq = 12000000;
    // if use pll, set divider and freq as pll setting.
    __HAL_I2S_CLK_PLL(hi2s); //PLL
    __HAL_I2S_SET_SPCLK_DIV(hi2s, 8);   // set to 6.144M to i2s   PLL
#else
    HAL_RCC_SetModuleFreq(RCC_MOD_I2S_ALL, 12000000);
#endif

#endif

    HAL_I2S_Init(hi2s);
    return RT_EOK;
}

static rt_err_t bf0_audio_start(struct rt_audio_device *audio, int stream)
{
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;
    HAL_StatusTypeDef res = HAL_OK;

    if ((aud->hi2s.State == HAL_I2S_STATE_RESET) || (aud->hi2s.State == HAL_I2S_STATE_READY))
    {
        bf0_audio_i2s_start(aud);
    }


    {
        res = HAL_I2S_Receive_DMA(&(aud->hi2s), aud->rx_buf, AUDIO_DATA_SIZE);
        if (res != HAL_OK)
            return RT_ERROR;

        //audio_debug_out_i2s(aud->hi2s.Instance);
        //audio_debug_dma(aud->hi2s.hdmarx->Instance);

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
        HAL_NVIC_EnableIRQ(MIC_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

        //__HAL_I2S_RX_ENABLE(&(aud->hi2s));
        /* Clear I2S pause bit */
        //CLEAR_BIT(aud->hi2s.Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
    }

    LOG_I("bf0_audio_start %d done\n", stream);
    return RT_EOK;
}

/**
  * @brief  Stop audio device for recording/playback.
  * @param[in]  audio: audio device handle.
  * @param[in]  stream: stream ID.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_stop(struct rt_audio_device *audio, int stream)
{
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;
    rt_err_t ret = RT_EOK;
    //HAL_I2S_DMAPause(&(aud->hi2s));
    //HAL_I2S_DMAStop(&(aud->hi2s));

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    HAL_NVIC_DisableIRQ(MIC_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
    ret = HAL_I2S_RX_DMAStop(&(aud->hi2s));

    HAL_I2S_DeInit(&(aud->hi2s));
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
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;
    rt_err_t ret = RT_EOK;
    //HAL_I2S_DMAPause(&(aud->hi2s));

    ret = HAL_I2S_RX_DMAPause(&(aud->hi2s));

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
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;
    //HAL_I2S_DMAResume(&(aud->hi2s));
    rt_err_t ret = RT_EOK;

    ret = HAL_I2S_RX_DMAResume(&(aud->hi2s));

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

    switch (cmd)
    {
    case AUDIO_CTL_HWRESET:
        break;
    case RT_DEVICE_CTRL_SUSPEND:
    {

        for (int i = 0; i < sizeof(bf0_i2s_audio_obj) / sizeof(bf0_i2s_audio_obj[0]); i++)
        {
            if (bf0_i2s_audio_obj[i].i2s_handle != NULL)
            {
                I2S_HandleTypeDef *hi2s = &(h_i2s_mic.hi2s);
                HAL_I2S_DeInit(hi2s);
                //LOG_I("i2s RT_DEVICE_CTRL_SUSPEND\n");
            }
        }
        break;
    }
    case RT_DEVICE_CTRL_RESUME:
    {
        for (int i = 0; i < sizeof(bf0_i2s_audio_obj) / sizeof(bf0_i2s_audio_obj[0]); i++)
        {
            if (bf0_i2s_audio_obj[i].i2s_handle != NULL)
            {
                I2S_HandleTypeDef *hi2s = &(h_i2s_mic.hi2s);
                HAL_I2S_Init(hi2s);
            }
        }
        //LOG_I("i2s RT_DEVICE_CTRL_RESUME\n");
        break;
    }
    default:
        result = -RT_ERROR;
        break;
    }

    return result;
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
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;
    HAL_StatusTypeDef res = HAL_OK;
    if (writeBuf != NULL)
    {
        res = HAL_I2S_Transmit_DMA(&(aud->hi2s), (uint8_t *)writeBuf, size);
        //res = HAL_I2S_Transmit(&(aud->hi2s), (uint8_t *)writeBuf, size, 1000);
        if (res != HAL_OK)
        {
            LOG_I("HAL_I2S_Transmit %d\n", res);
            return 0;
        }
    }

    if (readBuf != NULL)
        res = HAL_I2S_Receive_DMA(&(aud->hi2s), readBuf, size);
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
* @} I2S Audio_device
*/

/**
* @brief  I2S Audio devices initialization
*/
int rt_bf0_i2s_mic_init(void)
{
    int result;

    h_i2s_mic.audio_device.ops = (struct rt_audio_ops *)&_g_audio_ops;
    h_i2s_mic.rx_buf = audio_data; // &(audio_data[0]);
    h_i2s_mic.tx_buf = audio_tx_data; //&(audio_data[AUDIO_DATA_SIZE / 2]);

    if (bf0_i2s_audio_obj[0].i2s_handle != NULL)
    {
        I2S_HandleTypeDef *hi2s = &(h_i2s_mic.hi2s);

        hi2s->Instance = bf0_i2s_audio_obj[0].i2s_handle;

        // init dma handle and request, other parameters configure in HAL driver
        hi2s->hdmarx = malloc(sizeof(DMA_HandleTypeDef));
        hi2s->hdmarx->Instance = bf0_i2s_audio_obj[0].dma_handle;
        hi2s->hdmarx->Init.Request = bf0_i2s_audio_obj[0].dma_request;

#ifndef ASIC  //i2s mic on FPGA  
        hi2s->Init.src_clk_freq =     3 * 1024 * 1000;   //FPGA A0 I2S clk source is 3.072MHz

        hi2s->Init.rx_cfg.data_dw = 16;
        hi2s->Init.rx_cfg.bus_dw = 32;
        hi2s->Init.rx_cfg.slave_mode = 0;   // master mode
        hi2s->Init.rx_cfg.chnl_sel = 0;     // left/right all set to left
        hi2s->Init.rx_cfg.sample_rate = 16000;
        hi2s->Init.rx_cfg.track = 1;            // default mono
        hi2s->Init.rx_cfg.lrck_invert = 0;
        hi2s->Init.rx_cfg.bclk = 1536000;

        /*FPGA have NONE I2S TX device*/

#else    //i2s codec on evb_z0
#ifndef SF32LB55X
        hi2s->Init.src_clk_freq = 1024000;
#else
        hi2s->Init.src_clk_freq = HAL_RCC_GetModuleFreq(RCC_MOD_I2S2);  //As GetFreq may NOT be 48000000
#endif
        RT_ASSERT(hi2s->Init.src_clk_freq);


        hi2s->Init.rx_cfg.data_dw = 16;
        hi2s->Init.rx_cfg.bus_dw = 32;
        hi2s->Init.rx_cfg.pcm_dw = 16;
        hi2s->Init.rx_cfg.slave_mode = 0;   // master mode
        hi2s->Init.rx_cfg.chnl_sel = 0;     // left/right all set to left
        hi2s->Init.rx_cfg.sample_rate = 16000;
        hi2s->Init.rx_cfg.track = 1;            // default mono
        hi2s->Init.rx_cfg.lrck_invert = 0;
        hi2s->Init.rx_cfg.bclk = 800000;
        hi2s->Init.rx_cfg.extern_intf = 0;


#endif
        hi2s->Init.rx_cfg.clk_div_index = 5;
        hi2s->Init.rx_cfg.clk_div = &txrx_clk_div[hi2s->Init.rx_cfg.clk_div_index];

    }
    result = rt_audio_register(&(h_i2s_mic.audio_device),
                               bf0_i2s_audio_obj[0].name, RT_DEVICE_FLAG_RDWR, &h_i2s_mic);

    return result;
}

INIT_DEVICE_EXPORT(rt_bf0_i2s_mic_init);

/// @} drv_audio
/// @} bsp_driver



/** @addtogroup bsp_sample BSP driver sample commands.
  * @{
  */

/** @defgroup bsp_sample_audio Audio sample commands
  * @brief Audio sample commands
  *
  * This sample commands demonstrate the usage of audio driver.
  * @{
  */


/**
  * @brief RX DMA interrupt handler.
  */
void MIC_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(h_i2s_mic.hi2s.hdmarx);

    /* leave interrupt */
    rt_interrupt_leave();

}

/**
  * @brief Rx Transfer half completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    struct bf0_i2s_audio *haudio = rt_container_of(hi2s, struct bf0_i2s_audio, hi2s);
    struct rt_audio_device *audio = &(haudio->audio_device);
    if (audio != NULL)
        rt_audio_rx_done(audio, &(audio_data[0]), AUDIO_DATA_SIZE / 2);

    //LOG_I("HAL_I2S_RxHalfCpltCallback\n");
}

/**
  * @brief Rx Transfer completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    struct bf0_i2s_audio *haudio = rt_container_of(hi2s, struct bf0_i2s_audio, hi2s);
    struct rt_audio_device *audio = &(haudio->audio_device);

    if (audio != NULL)
        rt_audio_rx_done(audio, &(audio_data[AUDIO_DATA_SIZE / 2]), AUDIO_DATA_SIZE / 2);

    // start next transmit
    //HAL_I2S_Receive_DMA(hi2s   , &(audio_data[0]), AUDIO_DATA_SIZE / 4);
    //LOG_I("HAL_I2S_RxCpltCallback\n");
}

/**
  * @brief I2S error callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    LOG_I("HAL_I2S_ErrorCallback\n");
}


//#define DRV_TEST
#if defined(DRV_TEST) || defined (APP_BSP_TEST)

#include "string.h"

/****
To use file system on SDCARD, you need create and mount FS MANUAL!
Make sure SDCARD insert on your board and detected success, there are LOG for it.
Then use cmd "mkfs -t elm sd0" to create FS,this step only need once if do not format SDCAR.
After FS created success, use cmd "mountfs -t elm sd0 /" to mount FS to root.
File system used, uart do not need any more, 2 choose 1
**/
#define MIC_TEST_FILE_SAVE

#ifdef MIC_TEST_FILE_SAVE
    #define MIC_SAVE2RAM
#endif

#define AUDIO_BUF_SIZE 512
#define AUDIO_TEST_HNAME        "i2s1"


static rt_device_t g_mic;
static uint8_t g_pipe_data[AUDIO_BUF_SIZE];
static rt_event_t g_audio_ev;
static uint8_t *buf_flag = NULL;

#ifdef MIC_TEST_FILE_SAVE
#include <dfs_posix.h>

#ifdef MIC_SAVE2RAM
    #define MIC_SAVE_MEM_BASE           (PSRAM_BASE + 0x300000)
    static char *mic_buf = (char *)MIC_SAVE_MEM_BASE;
#endif //MIC_SAVE2RAM
#define ARX_TEST_FNAME      "rec_test.wav"

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
#define AUD_TEST_FLEN       (0x80000)
static int aud_fptr = -1;
static int aud_flen = 0;
static char aud_fname[16];

static void atest_fill_header(uint32_t sr)
{
    AUD_WAV_HDR_T hdr;
    hdr.riff[0] = 'R';
    hdr.riff[1] = 'I';
    hdr.riff[2] = 'F';
    hdr.riff[3] = 'F';
    hdr.lenth = AUD_TEST_FLEN + 36;
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
    hdr.channel = 1;
    hdr.sampleRate = sr;
    //hdr.bytePerSec = 32000;
    hdr.blockAlign = 2;
    hdr.bitPerSample = 16;
    hdr.bytePerSec = hdr.sampleRate * hdr.channel * hdr.bitPerSample / 8;
    hdr.data[0] = 'd';
    hdr.data[1] = 'a';
    hdr.data[2] = 't';
    hdr.data[3] = 'a';
    hdr.size2 = AUD_TEST_FLEN; //;    // record data lenght

#ifndef MIC_SAVE2RAM
    if (aud_fptr >= 0)
    {
        write(aud_fptr, &hdr, 44);
        aud_flen = 0;
        LOG_I("Write file header with SR %d, bytePs %d\n", hdr.sampleRate, hdr.bytePerSec);
    }
    else
    {
        LOG_I("File pointer not exist\n");
    }
#else
    memcpy(mic_buf, &hdr, 44);
    aud_flen = 0;
    mic_buf += 44;
#endif  //MIC_SAVE2RAM

}


#else
static rt_device_t g_uart;
static char g_transport_name[8];

#endif  //MIC_TEST_FILE_SAVE

/**
* @brief  Audio receiving thread.
* This us audio process thread. It will capture audio data from I2S, and send to transport.
* @param[in]  param: unused.
*/
static void bf0_audio_entry(void *param)
{
    rt_uint32_t evt;
#ifdef MIC_TEST_FILE_SAVE
#ifndef MIC_SAVE2RAM

    aud_fptr = open(aud_fname, O_WRONLY | O_CREAT);
    if (aud_fptr < 0)
    {
        rt_kprintf("Open file %s FAIL\n", aud_fname);
    }
#endif
#else
    g_uart = rt_device_find(g_transport_name);
    if (g_uart)
    {
        rt_err_t res = rt_device_open(g_uart, RT_DEVICE_FLAG_RDWR);
        LOG_I("Open uart %s res %d\n", g_transport_name, res);
    }
    else
    {
        LOG_E("Could not open transport for logging.\n");
        return;
    }
#endif  //    MIC_TEST_FILE_SAVE

    g_audio_ev = rt_event_create("audio_evt", RT_IPC_FLAG_FIFO);
    while (1)
    {
        rt_event_recv(g_audio_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        while (1)
        {
            rt_size_t len;
            len = rt_device_read(g_mic, 0, g_pipe_data, AUDIO_BUF_SIZE);
            if (len != AUDIO_DATA_SIZE / 2)
            {
                LOG_I("Got Audio size=%d\n", len);
                LOG_HEX("g_pipe_data", 10, g_pipe_data, len > 16 ? 16 : len);
            }
#if 0
            {
                // fill buffer for test
                int i;
                static uint8_t sin1k[] =
                {
                    00, 0x40, 0x9F, 0x53, 0x83, 0x5A, 0x9F, 0x53, 0x00, 0x40, 0xA3, 0x22, 0x00, 0x00, 0x5D, 0xDD,
                    00, 0xC0, 0x61, 0xAC, 0x7D, 0xA5, 0x61, 0xAC, 0x00, 0xC0, 0x5D, 0xDD, 0x00, 0x00, 0xA3, 0x22,
                };
                for (i = 0; i < len; i++)
                {
                    g_pipe_data[i] = sin1k[i % 32];
                }
            }
#endif
#ifdef MIC_TEST_FILE_SAVE
#ifndef MIC_SAVE2RAM
            if (aud_fptr >= 0)
            {
                if (aud_flen + len <= AUD_TEST_FLEN)
                {
                    write(aud_fptr, g_pipe_data, len);
                    aud_flen += len;
                }
            }
#else
            if (aud_flen + len <= AUD_TEST_FLEN)
            {
                memcpy(mic_buf, g_pipe_data, len);
                mic_buf += len;
                aud_flen += len;
            }
#endif //MIC_SAVE2RAM
#else
            if (g_uart)
            {
                //LOG_I("Out Audio size=%d\n", len);
                rt_device_write(g_uart, 0, g_pipe_data, len);
            }
#endif  // MIC_TEST_FILE_SAVE
            if (len < AUDIO_BUF_SIZE)
                break;
        }
    }
}

/**
* @brief  Audio receiving callback.
* This callback will send event to receiving thread for further audio precessing.
* @param[in]  dev: audio device.
* @param[in]  size: received audio data size.
* @retval RT_EOK
*/
static rt_err_t audio_rx_ind(rt_device_t dev, rt_size_t size)
{
    //LOG_I("audio_rx_ind %d\n", size);
    rt_event_send(g_audio_ev, 1);
    return RT_EOK;
}


/**
* @brief  Mic commands.
* This function provide 'audio' command to shell(FINSH) .
* The commands supported:
*   - audio open

      Open microphone and speaker device

    - audio config [sample rate]

      Configure microphone catpure sample rate

    - audio start rx

      Audio start capture, it will start \ref bf0_audio_entry thread.

    - audio stop rx

      Audio stop capture

* @retval RT_EOK
*/
int cmd_mic(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "open") == 0)
        {
            if (g_mic == NULL)
            {
                rt_thread_t tid;
                g_mic = rt_device_find(AUDIO_TEST_HNAME);
                if (g_mic)
                {
                    rt_device_open(g_mic, RT_DEVICE_FLAG_RDWR);
                    LOG_I("Microphone opened\n");
                }
                else
                {
                    LOG_E("Could not find audio device\n");
                    return -RT_ERROR;
                }

#ifdef MIC_TEST_FILE_SAVE
                aud_fname[15] = '\0';
                if (argc > 2)
                    strncpy(aud_fname, argv[2], 15);
                else
                    strcpy(aud_fname, ARX_TEST_FNAME);
#else
                if (argc > 2)
                    strcpy(g_transport_name, argv[2]);
                else
                    strcpy(g_transport_name, "uart4");
#endif //MIC_TEST_FILE_SAVE  
                // start record thread
                tid = rt_thread_create("aud_th", bf0_audio_entry, g_mic, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
                rt_thread_startup(tid);

            }
        }
        else if (strcmp(argv[1], "config") == 0)
        {
            if (g_mic)
            {
                struct rt_audio_caps caps;
                //caps.main_type = AUDIO_TYPE_OUTPUT;
                caps.main_type = AUDIO_TYPE_INPUT;      // for I2S2, configure RX will configure RX+TX
                caps.sub_type = AUDIO_DSP_SAMPLERATE;
                caps.udata.value = atoi(argv[2]);
                rt_device_control(g_mic, AUDIO_CTL_CONFIGURE, &caps);
            }
        }
        else if (strcmp(argv[1], "start") == 0)
        {
            if (g_mic)
            {
                int stream = 0;

                stream = AUDIO_STREAM_RECORD;
#ifdef MIC_TEST_FILE_SAVE
                atest_fill_header(16000);   // TODO : can get SampleRate by interface 'getcaps'
#endif
                rt_device_set_rx_indicate(g_mic, audio_rx_ind);
                rt_device_control(g_mic, AUDIO_CTL_START, &stream);

            }
        }
        else if (strcmp(argv[1], "stop") == 0)
        {
            if (g_mic)
            {
                int stream = 0;

                stream = AUDIO_STREAM_RECORD;
                rt_device_control(g_mic, AUDIO_CTL_STOP, &stream);
#ifdef MIC_TEST_FILE_SAVE
#ifndef MIC_SAVE2RAM
                if (aud_fptr >= 0)
                {
                    if (aud_flen < AUD_TEST_FLEN) // fill to fixed length
                    {
                        uint32_t dbuf = 0;
                        int i;
                        for (i = 0; i < (AUD_TEST_FLEN - aud_flen) / 4; i++)
                            write(aud_fptr, &dbuf, 4);
                    }
                    close(aud_fptr);
                }
#endif //MIC_SAVE2RAM                
#endif

            }
        }
        else if (strcmp(argv[1], "rdebug") == 0)
        {
            audio_debug_out_i2sr();
            audio_debug_out_rxdma();
        }
    }
    return RT_EOK;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mic, __cmd_mic, Test mic driver);

#endif

/// @} bsp_sample_audio
/// @} bsp_sample

#endif  /* BSP_ENABLE_I2S_MIC */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
