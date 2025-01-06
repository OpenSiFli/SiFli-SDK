/**
  ******************************************************************************
  * @file   drv_audio.c
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
#include <stdlib.h>
#include "board.h"
#include "drv_config.h"

#include "string.h"

#ifdef FPGA
static int bf0_enable_pll(uint32_t freq, uint8_t type)
{
    return 0;
}
static void bf0_disable_pll()
{

}
static void set_pll_state(uint8_t state)
{

}

#endif

#if defined(BSP_ENABLE_I2S_CODEC)||defined(BSP_ENABLE_I2S3)||defined(_SIFLI_DOXYGEN_)

//#define DBG_LEVEL                      DBG_LOG
#define LOG_TAG              "drv.i2s_audio"
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
    uint8_t *tx_pos;
};


#define AUDIO_DATA_SIZE 640 //480
ALIGN(4) static uint8_t audio_data[AUDIO_DATA_SIZE];
ALIGN(4) static uint8_t audio_tx_data[AUDIO_DATA_SIZE];
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

#ifdef SF32LB58X
    #ifndef SOC_BF0_HCPU
        // for LCPU, DMAC2 IRQ need tranfer by hpsys_cfg
        #define ADMA_HPIRQ_TX      (48)    //DMAC2_CH3_IRQn
        #define ADMA_HPIRQ_RX      (49)    //DMAC2_CH4_IRQn
        #define ADMA_LPIRQ_TX      (30)    //Interrupt30_IRQn
        #define ADMA_LPIRQ_RX      (31)    //Interrupt31_IRQn
    #endif
#endif /* SF32LB58X */

static struct i2s_audio_cfg_t bf0_i2s_audio_obj[] =
{
#ifdef BSP_ENABLE_I2S_CODEC
    BF0_I2S2_CONFIG,
#endif // BSP_ENABLE_I2S_CODEC
#ifdef BSP_ENABLE_I2S3
    BF0_I2S3_CONFIG,
#endif // BSP_ENABLE_I2S_CODEC

};

static struct bf0_i2s_audio h_i2s_audio[sizeof(bf0_i2s_audio_obj) / sizeof(bf0_i2s_audio_obj[0])];


static void audio_debug_out_i2sr()
{
    I2S_TypeDef *hi2s = h_i2s_audio[0].hi2s.Instance;
    LOG_D("RX_RE_SAMPLE_CLK_DIV = 0X%08x\n", hi2s->RX_RE_SAMPLE_CLK_DIV);
    LOG_D("AUDIO_RX_LRCK_DIV = 0X%08x\n", hi2s->AUDIO_RX_LRCK_DIV);
    LOG_D("AUDIO_RX_BCLK_DIV = 0X%08x\n", hi2s->AUDIO_RX_BCLK_DIV);
    LOG_D("AUDIO_RX_SERIAL_TIMING = 0X%08x\n", hi2s->AUDIO_RX_SERIAL_TIMING);
    LOG_D("AUDIO_RX_PCM_DW = 0X%08x\n", hi2s->AUDIO_RX_PCM_DW);
    LOG_D("RECORD_FORMAT = 0X%08x\n", hi2s->RECORD_FORMAT);
    LOG_D("RX_CH_SEL = 0X%08x\n", hi2s->RX_CH_SEL);
    LOG_D("DMA_MASK = 0X%08x\n", hi2s->DMA_MASK);
    LOG_D("AUDIO_RX_FUNC_EN = 0X%08x\n", hi2s->AUDIO_RX_FUNC_EN);
    LOG_D("AUDIO_RX_PAUSE = 0X%08x\n", hi2s->AUDIO_RX_PAUSE);

}
static void audio_debug_out_i2st()
{
    I2S_TypeDef *hi2s = h_i2s_audio[0].hi2s.Instance;
    LOG_D("TX_PCM_FORMAT = 0X%08x\n", hi2s->TX_PCM_FORMAT);
    LOG_D("TX_PCM_SAMPLE_CLK = 0X%08x\n", hi2s->TX_PCM_SAMPLE_CLK);
    LOG_D("TX_PCM_CH_SEL = 0X%08x\n", hi2s->TX_PCM_CH_SEL);
    LOG_D("AUDIO_TX_LRCK_DIV = 0X%08x\n", hi2s->AUDIO_TX_LRCK_DIV);
    LOG_D("AUDIO_TX_BCLK_DIV = 0X%08x\n", hi2s->AUDIO_TX_BCLK_DIV);
    LOG_D("AUDIO_TX_FORMAT = 0X%08x\n", hi2s->AUDIO_TX_FORMAT);
    LOG_D("AUDIO_SERIAL_TIMING = 0X%08x\n", hi2s->AUDIO_SERIAL_TIMING);
    LOG_D("AUDIO_TX_FUNC_EN = 0X%08x\n", hi2s->AUDIO_TX_FUNC_EN);
    LOG_D("AUDIO_TX_PAUSE = 0X%08x\n", hi2s->AUDIO_TX_PAUSE);
    LOG_D("DMA_MASK = 0X%08x\n", hi2s->DMA_MASK);
}

static void audio_debug_out_txdma()
{
    DMA_Channel_TypeDef *hdma = h_i2s_audio[0].hi2s.hdmatx->Instance;
    LOG_D("TX CCR = 0X%08x\n", hdma->CCR);
    LOG_D("TX CNDTR = 0X%08x\n", hdma->CNDTR);
    LOG_D("TX CPAR = 0X%08x\n", hdma->CPAR);
    LOG_D("TX CM0AR = 0X%08x\n", hdma->CM0AR);
    LOG_D("TX CM0AR = 0X%08x\n", hdma->CM0AR);
    LOG_D("TX CBSR = 0X%08x\n", hdma->CBSR);
}
static void audio_debug_out_rxdma()
{
    DMA_Channel_TypeDef *hdma = h_i2s_audio[0].hi2s.hdmarx->Instance;
    LOG_D("RX CCR = 0X%08x\n", hdma->CCR);
    LOG_D("RX CNDTR = 0X%08x\n", hdma->CNDTR);
    LOG_D("RX CPAR = 0X%08x\n", hdma->CPAR);
    LOG_D("RX CM0AR = 0X%08x\n", hdma->CM0AR);
    LOG_D("RX CBSR = 0X%08x\n", hdma->CBSR);
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

    LOG_D("CONFIG: main %d, sub %d\n", caps->main_type, caps->sub_type);
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
            // TODO: for i2s2, rx clock from tx loopback, their clock should always be same
            hi2s->Init.tx_cfg.sample_rate = caps->udata.config.samplerate;
            if (caps->udata.config.channels == 1)
                hi2s->Init.tx_cfg.track = 1; //(uint8_t)caps->udata.config.channels;
            else
                hi2s->Init.tx_cfg.track = 0;
            hi2s->Init.tx_cfg.data_dw = (uint8_t)caps->udata.config.samplefmt;
            hi2s->Init.tx_cfg.clk_div_index = index;
            hi2s->Init.tx_cfg.clk_div = &txrx_clk_div[hi2s->Init.tx_cfg.clk_div_index];
            LOG_I("Configure audio chn %d, samplerate %d, bitwidth %d\n", caps->udata.config.channels, caps->udata.config.samplerate, caps->udata.config.samplefmt);
            HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));
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
            LOG_D("Configure audio RX sample rate to %d\n", rate);
            HAL_I2S_Config_Receive(hi2s, &(hi2s->Init.rx_cfg));
            // TODO: for i2s2, rx clock from tx loopback, their clock should always be same
            hi2s->Init.tx_cfg.sample_rate = rate;
            hi2s->Init.tx_cfg.clk_div_index = index;
            hi2s->Init.tx_cfg.clk_div = &txrx_clk_div[hi2s->Init.tx_cfg.clk_div_index];
            HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));
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
        case AUDIO_DSP_MODE:              // Config device work mode
        {
            int mode = caps->udata.value;
            I2S_HandleTypeDef *hi2s = &(aud->hi2s);
            hi2s->Init.rx_cfg.slave_mode = 1; //rx in slave mode all the time
            HAL_I2S_Config_Receive(hi2s, &(hi2s->Init.rx_cfg));
            hi2s->Init.tx_cfg.slave_mode = (uint8_t)mode;
            HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));
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
    case AUDIO_TYPE_OUTPUT:
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
            hi2s->Init.tx_cfg.sample_rate = caps->udata.config.samplerate;
            if (caps->udata.config.channels == 1)
                hi2s->Init.tx_cfg.track = 1; //(uint8_t)caps->udata.config.channels;
            else
                hi2s->Init.tx_cfg.track = 0;
            hi2s->Init.tx_cfg.data_dw = (uint8_t)caps->udata.config.samplefmt;
            hi2s->Init.tx_cfg.clk_div_index = index;
            hi2s->Init.tx_cfg.clk_div = &txrx_clk_div[hi2s->Init.tx_cfg.clk_div_index];
            HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));
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
            hi2s->Init.tx_cfg.sample_rate = rate;
            hi2s->Init.tx_cfg.clk_div_index = index;
            hi2s->Init.tx_cfg.clk_div = &txrx_clk_div[hi2s->Init.tx_cfg.clk_div_index];
            LOG_D("Configure audio TX sample rate to %d\n", rate);
            HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));
            //audio_debug_out_i2s(hi2s->Instance);

        }
        break;
        case AUDIO_DSP_CHANNELS:              // Config channel
        {
            int chnl = caps->udata.value;
            I2S_HandleTypeDef *hi2s = &(aud->hi2s);
            hi2s->Init.tx_cfg.track = (chnl == 1) ? 1 : 0;
            HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));
        }
        break;
        case AUDIO_DSP_MODE:              // Config device work mode
        {
            int mode = caps->udata.value;
            I2S_HandleTypeDef *hi2s = &(aud->hi2s);
            hi2s->Init.tx_cfg.slave_mode = (uint8_t)mode;
            HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));
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
    case AUDIO_TYPE_SELECTOR:
    {
        I2S_HandleTypeDef *hi2s = &(aud->hi2s);
        if (caps->sub_type == 1)  // left/right all set to left
        {
            hi2s->Init.tx_cfg.chnl_sel = 1;
            hi2s->Init.rx_cfg.chnl_sel = 1;
        }
        else if (caps->sub_type == 2)  //left/right all set to right
        {
            hi2s->Init.tx_cfg.chnl_sel = 4;
            hi2s->Init.rx_cfg.chnl_sel = 4;

        }
        else
        {
            hi2s->Init.tx_cfg.chnl_sel = 0;
            hi2s->Init.rx_cfg.chnl_sel = 0;

        }
        HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));
        HAL_I2S_Config_Receive(hi2s, &(hi2s->Init.rx_cfg));
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

static rt_err_t bf0_audio_i2s_start(struct bf0_i2s_audio *aud, int stream)
{
    I2S_HandleTypeDef *hi2s = &aud->hi2s;

    if (stream == AUDIO_STREAM_REPLAY)
    {
        bf0_enable_pll(hi2s->Init.tx_cfg.sample_rate, 0);
    }
    else
    {
        bf0_enable_pll(hi2s->Init.rx_cfg.sample_rate, 0);
    }

#ifdef SF32LB58X
#ifndef SOC_BF0_HCPU
    // for LCPU, need send DMA irq to LCPU
    MODIFY_REG(hwp_hpsys_cfg->LPIRQ, HPSYS_CFG_LPIRQ_SEL0_Msk,
               MAKE_REG_VAL(ADMA_HPIRQ_TX, HPSYS_CFG_LPIRQ_SEL0_Msk, HPSYS_CFG_LPIRQ_SEL0_Pos));
    MODIFY_REG(hwp_hpsys_cfg->LPIRQ, HPSYS_CFG_LPIRQ_SEL1_Msk,
               MAKE_REG_VAL(ADMA_HPIRQ_RX, HPSYS_CFG_LPIRQ_SEL1_Msk, HPSYS_CFG_LPIRQ_SEL1_Pos));
#endif /* !SOC_BF0_HCPU */
#endif /* SF32LB58X */

#ifndef ASIC  //i2s mic on FPGA  

    /*FPGA have NONE I2S TX device*/
#ifndef SF32LB55X
    __HAL_I2S_CLK_XTAL(hi2s);
    __HAL_I2S_SET_SPCLK_DIV(hi2s, 1);
#endif
#else    //i2s codec on evb
#ifndef SF32LB55X
#ifdef SF32LB58X
    if (hi2s->Instance == hwp_i2s3)
    {
        HAL_RCC_EnableModule(RCC_MOD_I2S3);
    }
    else if (hi2s->Instance == hwp_i2s2)
    {
        HAL_RCC_EnableModule(RCC_MOD_I2S2);
    }
    else
    {
        HAL_RCC_EnableModule(RCC_MOD_I2S1);
    }
#else
    HAL_RCC_EnableModule(RCC_MOD_I2S1);
#endif
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
    RT_ASSERT(hi2s->Init.src_clk_freq);
    //rt_kprintf("Audio I2S2 CLK %d\n", hi2s->Init.src_clk_freq);
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
        bf0_audio_i2s_start(aud, stream);
    }
    if (stream == AUDIO_STREAM_REPLAY) //
    {
        aud->hi2s.Init.tx_cfg.vol = 4;  // set to 0 db (4) if no other request
        HAL_I2S_Config_Transmit(&(aud->hi2s), &(aud->hi2s.Init.tx_cfg));
        if (aud->hi2s.Init.tx_cfg.extern_intf)
        {
            __HAL_I2S_TX_INTF_ENABLE(&(aud->hi2s));
            __HAL_I2S_TX_ENABLE(&(aud->hi2s));
            CLEAR_BIT(aud->hi2s.Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);
            LOG_I("bf0_audio_start tx with external interface\n");
        }
        else
        {
            __HAL_I2S_TX_INTF_DISABLE(&(aud->hi2s));

            res = HAL_I2S_Transmit_DMA(&(aud->hi2s), aud->tx_buf, AUDIO_DATA_SIZE);
            if (res != HAL_OK)
            {
                LOG_E("HAL_I2S_Transmit_DMA fail %d\n", res);
                return RT_ERROR;
            }
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
#ifdef I2S3_TX_DMA_IRQ
            HAL_NVIC_EnableIRQ(I2S3_TX_DMA_IRQ);
#else
            HAL_NVIC_EnableIRQ(I2S_TX_DMA_IRQ);
#endif /* I2S3_TX_DMA_IRQ */
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            LOG_I("bf0_audio_start enable irq\n");


            audio_debug_out_i2st();
            audio_debug_out_txdma();
        }
    }
    else    //AUDIO_STREAM_RECORD
    {
        // for i2s2, need enable tx first to make clk work
        if (HAL_IS_BIT_CLR(aud->hi2s.Instance->AUDIO_TX_FUNC_EN, I2S_AUDIO_TX_FUNC_EN_TX_EN))
        {
            /* Enable I2S peripheral */
            __HAL_I2S_TX_ENABLE(&(aud->hi2s));
        }
        if (aud->hi2s.Init.rx_cfg.extern_intf)
        {
            __HAL_I2S_RX_INTF_ENABLE(&(aud->hi2s));
            __HAL_I2S_RX_ENABLE(&(aud->hi2s));
            /* Clear I2S pause bit */
            CLEAR_BIT(aud->hi2s.Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
            LOG_I("bf0_audio_start rx with external interface\n");
        }
        else
        {
            __HAL_I2S_RX_INTF_DISABLE(&(aud->hi2s));
            //rt_thread_delay(600); // wait clock stable to meet outside pll
            res = HAL_I2S_Receive_DMA(&(aud->hi2s), aud->rx_buf, AUDIO_DATA_SIZE);
            if (res != HAL_OK)
                return RT_ERROR;

            audio_debug_out_i2sr();
            audio_debug_out_rxdma();
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
#ifdef MIC_DMA_IRQ
            HAL_NVIC_EnableIRQ(MIC_DMA_IRQ);
#elif defined(I2S3_RX_DMA_IRQ)
            HAL_NVIC_EnableIRQ(I2S3_RX_DMA_IRQ);
#else
            HAL_NVIC_EnableIRQ(I2S_RX_DMA_IRQ);
            LOG_I("bf0_audio_start enable irq %d\n", I2S_RX_DMA_IRQ);
#endif /* MIC_DMA_IRQ */
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            //__HAL_I2S_RX_ENABLE(&(aud->hi2s));
            /* Clear I2S pause bit */
            //CLEAR_BIT(aud->hi2s.Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
        }
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
    LOG_I("bf0_audio_stop %d \n", stream);
    if (stream == AUDIO_STREAM_REPLAY) // tx
    {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
#ifdef I2S3_TX_DMA_IRQ
        HAL_NVIC_DisableIRQ(I2S3_TX_DMA_IRQ);
#else
        HAL_NVIC_DisableIRQ(I2S_TX_DMA_IRQ);
#endif
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
        ret = HAL_I2S_TX_DMAStop(&(aud->hi2s));
    }
    else // rx
    {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
#ifdef MIC_DMA_IRQ
        HAL_NVIC_DisableIRQ(MIC_DMA_IRQ);
#elif defined(I2S3_RX_DMA_IRQ)
        HAL_NVIC_DisableIRQ(I2S3_RX_DMA_IRQ);
#else
        HAL_NVIC_DisableIRQ(I2S_RX_DMA_IRQ);
#endif
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
        ret = HAL_I2S_RX_DMAStop(&(aud->hi2s));
        audio_debug_out_i2sr();
        audio_debug_out_rxdma();
    }

    // Deinit I2S
    HAL_I2S_DeInit(&(aud->hi2s));
    LOG_I("bf0_audio_stop %d done, ret %d\n", stream, ret);
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
    if (stream == AUDIO_STREAM_REPLAY) // tx
    {
        ret = HAL_I2S_TX_DMAPause(&(aud->hi2s));
    }
    else // rx
    {
        ret = HAL_I2S_RX_DMAPause(&(aud->hi2s));
    }

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
    if (stream == AUDIO_STREAM_REPLAY) // tx
    {
        ret = HAL_I2S_TX_DMAResume(&(aud->hi2s));
    }
    else // rx
    {
        ret = HAL_I2S_RX_DMAResume(&(aud->hi2s));
    }

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
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case AUDIO_CTL_SETOUTPUT:
    {
        uint32_t intf = (uint32_t)args;
        aud->hi2s.Init.tx_cfg.extern_intf = (uint8_t)intf;
        LOG_I("I2S use exteranl interface %d\n", intf);
        break;
    }
    case AUDIO_CTL_SETINPUT:
    {
        uint32_t intf = (uint32_t)args;
        aud->hi2s.Init.rx_cfg.extern_intf = (uint8_t)intf;
        break;
    }
    case RT_DEVICE_CTRL_SUSPEND:
    {

        for (int i = 0; i < sizeof(bf0_i2s_audio_obj) / sizeof(bf0_i2s_audio_obj[0]); i++)
        {
            if (bf0_i2s_audio_obj[i].i2s_handle != NULL)
            {
                I2S_HandleTypeDef *hi2s = &(h_i2s_audio[i].hi2s);
                HAL_I2S_DeInit(hi2s);
                //LOG_I("i2s RT_DEVICE_CTRL_SUSPEND\n");
            }
        }
        set_pll_state(0);
        break;
    }
    case RT_DEVICE_CTRL_RESUME:
    {
        for (int i = 0; i < sizeof(bf0_i2s_audio_obj) / sizeof(bf0_i2s_audio_obj[0]); i++)
        {
            if (bf0_i2s_audio_obj[i].i2s_handle != NULL)
            {
                I2S_HandleTypeDef *hi2s = &(h_i2s_audio[i].hi2s);
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
    //LOG_I("bf0_audio_trans");
    RT_ASSERT(size <= AUDIO_DATA_SIZE / 2);

    if (writeBuf != NULL)
    {
        if (aud->tx_pos != NULL)
            memcpy(aud->tx_pos, writeBuf, size);
    }

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
int rt_bf0_i2s_audio_init(void)
{
    int result;
    int i;

    for (i = 0; i < sizeof(bf0_i2s_audio_obj) / sizeof(bf0_i2s_audio_obj[0]); i++)
    {
        h_i2s_audio[i].audio_device.ops = (struct rt_audio_ops *)&_g_audio_ops;
        h_i2s_audio[i].rx_buf = audio_data; // &(audio_data[0]);
        h_i2s_audio[i].tx_buf = audio_tx_data; //&(audio_data[AUDIO_DATA_SIZE / 2]);
        h_i2s_audio[i].tx_pos = audio_tx_data; //NULL;

        if (bf0_i2s_audio_obj[i].i2s_handle != NULL)
        {
            I2S_HandleTypeDef *hi2s = &(h_i2s_audio[i].hi2s);

            hi2s->Instance = bf0_i2s_audio_obj[i].i2s_handle;

            // init dma handle and request, other parameters configure in HAL driver
            hi2s->hdmarx = malloc(sizeof(DMA_HandleTypeDef));
            hi2s->hdmarx->Instance = bf0_i2s_audio_obj[i].dma_handle;
            hi2s->hdmarx->Init.Request = bf0_i2s_audio_obj[i].dma_request;

            hi2s->hdmatx = malloc(sizeof(DMA_HandleTypeDef));
            hi2s->hdmatx->Instance = bf0_i2s_audio_obj[i].hdma_tx;
            hi2s->hdmatx->Init.Request = bf0_i2s_audio_obj[i].reqdma_tx;

#ifndef ASIC  //i2s mic on FPGA  
            hi2s->Init.src_clk_freq =     3 * 1024 * 1000;   //FPGA A0 I2S clk source is 3.072MHz

            hi2s->Init.rx_cfg.data_dw = 16;
            hi2s->Init.rx_cfg.bus_dw = 32;
            hi2s->Init.rx_cfg.pcm_dw = 16;
            hi2s->Init.rx_cfg.slave_mode = 1;   // slave mode
            hi2s->Init.rx_cfg.chnl_sel = 0;
            hi2s->Init.rx_cfg.sample_rate = 16000;
            hi2s->Init.rx_cfg.track = 1;            // default mono
            hi2s->Init.rx_cfg.lrck_invert = 0;
            hi2s->Init.rx_cfg.bclk = 1024000;
            hi2s->Init.rx_cfg.extern_intf = 0;

            /*FPGA have NONE I2S TX device*/
#ifndef SF32LB55X
            hi2s->Init.tx_cfg.data_dw = 16;
            hi2s->Init.tx_cfg.bus_dw = 32;
            hi2s->Init.tx_cfg.pcm_dw = 16;
            hi2s->Init.tx_cfg.slave_mode = 0;
            hi2s->Init.tx_cfg.track = 1;        // default stereo
            hi2s->Init.tx_cfg.vol = 4;     // default set to mute(15) or 0 db (4)
            hi2s->Init.tx_cfg.balance_en = 0;
            hi2s->Init.tx_cfg.balance_vol = 0;
            hi2s->Init.tx_cfg.chnl_sel = 0;
            hi2s->Init.tx_cfg.lrck_invert = 0;
            hi2s->Init.tx_cfg.sample_rate = 16000;
            hi2s->Init.tx_cfg.bclk = 1024000;
            hi2s->Init.tx_cfg.extern_intf = 0;
#endif
#else    //i2s codec on evb
#ifndef SF32LB55X
            hi2s->Init.src_clk_freq = 1024000;
#else
            hi2s->Init.src_clk_freq = HAL_RCC_GetModuleFreq(RCC_MOD_I2S2);  //As GetFreq may NOT be 48000000
#endif
            RT_ASSERT(hi2s->Init.src_clk_freq);
            //rt_kprintf("Audio I2S2 CLK %d\n", hi2s->Init.src_clk_freq);

            hi2s->Init.rx_cfg.data_dw = 16;
            hi2s->Init.rx_cfg.bus_dw = 32;
            hi2s->Init.rx_cfg.pcm_dw = 16;
            hi2s->Init.rx_cfg.slave_mode = 1;   // for rx, default to slave
            hi2s->Init.rx_cfg.chnl_sel = 0;     // left/right all set to left
            hi2s->Init.rx_cfg.sample_rate = 16000;
            hi2s->Init.rx_cfg.track = 1;            // default mono
            hi2s->Init.rx_cfg.lrck_invert = 0;
            hi2s->Init.rx_cfg.bclk = 800000;
            hi2s->Init.rx_cfg.extern_intf = 0;


            hi2s->Init.tx_cfg.data_dw = 16;
            hi2s->Init.tx_cfg.bus_dw = 32;
            hi2s->Init.tx_cfg.pcm_dw = 16;
            hi2s->Init.tx_cfg.slave_mode = 0;
            hi2s->Init.tx_cfg.track = 1;        // default stereo
            hi2s->Init.tx_cfg.vol = 4;     // default set to mute(15) or 0 db (4)
            hi2s->Init.tx_cfg.balance_en = 0;
            hi2s->Init.tx_cfg.balance_vol = 0;
            hi2s->Init.tx_cfg.chnl_sel = 0;
            hi2s->Init.tx_cfg.lrck_invert = 0;
            hi2s->Init.tx_cfg.sample_rate = 16000;
            hi2s->Init.tx_cfg.bclk = 800000;
            hi2s->Init.tx_cfg.extern_intf = 0;
#endif
            hi2s->Init.rx_cfg.clk_div_index = 5;
            hi2s->Init.tx_cfg.clk_div_index = 5;
            hi2s->Init.rx_cfg.clk_div = &txrx_clk_div[hi2s->Init.rx_cfg.clk_div_index];
            hi2s->Init.tx_cfg.clk_div = &txrx_clk_div[hi2s->Init.tx_cfg.clk_div_index];

            HAL_I2S_Init(hi2s);
        }
        result = rt_audio_register(&(h_i2s_audio[i].audio_device),
                                   bf0_i2s_audio_obj[i].name, RT_DEVICE_FLAG_RDWR, &(h_i2s_audio[i]));

    }

    return result;
}

INIT_DEVICE_EXPORT(rt_bf0_i2s_audio_init);

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
  * @brief TX DMA interrupt handler.
  */
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void I2S_TX_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
#if defined(SF32LB58X) && defined(SOC_BF0_LCPU)
    /* clear interrupt */
    MODIFY_REG(hwp_hpsys_cfg->LPIRQ, HPSYS_CFG_LPIRQ_IF0_Msk,
               MAKE_REG_VAL(1, HPSYS_CFG_LPIRQ_IF0_Msk, HPSYS_CFG_LPIRQ_IF0_Pos));
#endif /* SF32LB58X && SOC_BF0_LCPU */

    //LOG_I("En I2S_TX_DMA_IRQHandler ISR 0x%08x, SRC 0x%08x\n", h_i2s_audio[0].hi2s.hdmatx->DmaBaseAddress->ISR, h_i2s_audio[0].hi2s.hdmatx->Instance->CCR);
    HAL_DMA_IRQHandler(h_i2s_audio[0].hi2s.hdmatx);

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
  * @brief RX DMA interrupt handler.
  */
void I2S_RX_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
#if defined(SF32LB58X) && defined(SOC_BF0_LCPU)
    /* clear interrupt */
    MODIFY_REG(hwp_hpsys_cfg->LPIRQ, HPSYS_CFG_LPIRQ_IF1_Msk,
               MAKE_REG_VAL(1, HPSYS_CFG_LPIRQ_IF1_Msk, HPSYS_CFG_LPIRQ_IF1_Pos));
#endif /* SF32LB58X && SOC_BF0_LCPU */

    //LOG_I("En I2S_RX_DMA_IRQHandler\n");
    HAL_DMA_IRQHandler(h_i2s_audio[0].hi2s.hdmarx);

    /* leave interrupt */
    rt_interrupt_leave();

}

/**
  * @brief I2S3 TX DMA interrupt handler.
  */
void I2S3_TX_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("En I2S_TX_DMA_IRQHandler ISR 0x%08x, SRC 0x%08x\n", h_i2s_audio[0].hi2s.hdmatx->DmaBaseAddress->ISR, h_i2s_audio[0].hi2s.hdmatx->Instance->CCR);
    HAL_DMA_IRQHandler(h_i2s_audio[0].hi2s.hdmatx);

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
  * @brief I2S3 RX DMA interrupt handler.
  */
void I2S3_RX_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("En I2S_RX_DMA_IRQHandler\n");
    HAL_DMA_IRQHandler(h_i2s_audio[0].hi2s.hdmarx);

    /* leave interrupt */
    rt_interrupt_leave();

}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#ifndef BSP_ENABLE_I2S_MIC
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
#endif
/**
  * @brief Tx Transfer Half completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    struct bf0_i2s_audio *haudio = rt_container_of(hi2s, struct bf0_i2s_audio, hi2s);
    /* Prevent unused argument(s) compilation warning */
    //LOG_I("HAL_I2S_TxHalfCpltCallback\n");
    //LOG_I("HALF: %d\n", haudio->hi2s.hdmatx->Instance->CNDTR);
    haudio->tx_pos = audio_tx_data;
    rt_audio_tx_complete(&(haudio->audio_device), audio_tx_data);
}

/**
  * @brief Tx Transfer completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    struct bf0_i2s_audio *haudio = rt_container_of(hi2s, struct bf0_i2s_audio, hi2s);
    /* Prevent unused argument(s) compilation warning */
    //LOG_I("HAL_I2S_TxCpltCallback\n");
    //LOG_I("CMPL: %d\n", haudio->hi2s.hdmatx->Instance->CNDTR);
    haudio->tx_pos = &(audio_tx_data[AUDIO_DATA_SIZE / 2]);
    rt_audio_tx_complete(&(haudio->audio_device), &(audio_tx_data[AUDIO_DATA_SIZE / 2]));
    //rt_audio_tx_complete(&(haudio->audio_device), audio_tx_data);
    //bf0_audio_stop(&(haudio->audio_device), 0);
}

#ifndef BSP_ENABLE_I2S_MIC
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
#endif


//#define DRV_TEST
#if defined(DRV_TEST) || defined (APP_BSP_TEST)

#include "drv_flash.h"
#include "ipc/ringbuffer.h"


//#define ATEST_LOOPBACK

/****
To use file system on SDCARD, you need create and mount FS MANUAL!
Make sure SDCARD insert on your board and detected success, there are LOG for it.
Then use cmd "mkfs -t elm sd0" to create FS,this step only need once if do not format SDCAR.
After FS created success, use cmd "mountfs -t elm sd0 /" to mount FS to root.
File system used, uart do not need any more, 2 choose 1
**/


#define AUDIO_SAVE2RAM      // save to mem, if not define, save to file
#define AUDIO_LOADRAM       // load from ram, if not define, load from file

/****
 * if define use flash, i2s rx data save to flash and tx load from flash, use jlink to check memory
 * if node define use flash, default use psram memory, save/load with psram, use jlink to savebin/loadbin
**/
#define SAVE_LOAD_FROM_FLASH
#define AUD_SAVE_MEM_BASE           (0x1c180000)
#define AUD_LOAD_MEM_BASE           (0x1c180000)

#define AUDIO_BUF_SIZE  1920
#define AUDIO_TEST_HNAME        "i2s2"
#define AUD_TEST_FLEN       (0x7f000)


static rt_device_t g_mic;
static uint8_t g_pipe_data[AUDIO_BUF_SIZE];
static uint8_t tx_pipe_data[AUDIO_BUF_SIZE];

static rt_thread_t rx_tid;
static rt_thread_t tx_tid;
static rt_event_t g_rx_ev;
static rt_event_t g_tx_ev;
static uint8_t *buf_flag = NULL;

static char *mic_buf = (char *)AUD_SAVE_MEM_BASE;
static char *audio_mem_buf = (char *)AUD_LOAD_MEM_BASE;
static int aud_flen = 0;

#ifdef ATEST_LOOPBACK
    static struct rt_ringbuffer atest_rbf;
    static uint8_t *rbf_pool;
    static int32_t rbf_length = 0x20000;
    static int tx_start_flag = 0;
#endif

static int atest_save(char *dst, uint8_t *src, uint32_t size)
{
    uint32_t addr = (uint32_t)dst;
    int res = 0;
#ifdef SAVE_LOAD_FROM_FLASH
    // for flash , use flash write
    res = rt_flash_write(addr, src, size);
#else
    // for psram , use memcpy
    memcpy(dst, src, size);
    res = size;
#endif
    return res;
}

static void atest_init_buf(char *buf, int data, uint32_t size)
{
    uint32_t addr = (uint32_t)buf;
#ifdef SAVE_LOAD_FROM_FLASH
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


static void atest_fill_header(uint32_t sr, uint16_t channel, uint16_t bps)
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
    hdr.channel = channel;
    hdr.sampleRate = sr;
    //hdr.bytePerSec = 32000;
    hdr.blockAlign = 2;
    hdr.bitPerSample = bps;
    hdr.bytePerSec = hdr.sampleRate * hdr.channel * hdr.bitPerSample / 8;
    hdr.data[0] = 'd';
    hdr.data[1] = 'a';
    hdr.data[2] = 't';
    hdr.data[3] = 'a';
    hdr.size2 = AUD_TEST_FLEN; //;    // record data lenght
    LOG_I("Save wav sampleRate %d, bitPerSample %d, channel %d\n", hdr.sampleRate, hdr.bitPerSample, hdr.channel);

    atest_init_buf(mic_buf, 0, 0x80000);
    atest_save(mic_buf, (uint8_t *)&hdr, 44);
    aud_flen = 0;
    mic_buf += 44;
}
/**
* @brief  Audio transmit thread.
* This is audio process thread. It will send audio data to I2S.
* @param[in]  param: audio device handleer.
*/
void bf0_audio_tx_entry(void *param)
{
    rt_uint32_t evt, j, cnt;

    cnt = 0;

#ifndef ATEST_LOOPBACK
#if 1
    // read header if wave file include wav header
    memcpy(tx_pipe_data, audio_mem_buf + cnt, 44);
    cnt += 44;

    // read data
    memcpy(tx_pipe_data, audio_mem_buf + cnt, AUDIO_BUF_SIZE);
    cnt += AUDIO_BUF_SIZE;
#else
    if (0)
    {
        for (j = 0; j < AUDIO_BUF_SIZE; j++)
        {
            tx_pipe_data[j] = (uint8_t)(j & 0xff);
        }
    }
    else
    {
        // fill buffer
        static uint8_t sin1k[] =
        {
            00, 0x40, 0x9F, 0x53, 0x83, 0x5A, 0x9F, 0x53, 0x00, 0x40, 0xA3, 0x22, 0x00, 0x00, 0x5D, 0xDD,
            00, 0xC0, 0x61, 0xAC, 0x7D, 0xA5, 0x61, 0xAC, 0x00, 0xC0, 0x5D, 0xDD, 0x00, 0x00, 0xA3, 0x22,
        };
        for (j = 0; j < AUDIO_BUF_SIZE; j++)
        {
            tx_pipe_data[j] = sin1k[j % 32];
        }
    }

#endif //

    //cnt += AUDIO_BUF_SIZE;
    //memcpy(audio_tx_data, tx_pipe_data, AUDIO_BUF_SIZE);
    rt_device_write(g_mic, 0, tx_pipe_data, AUDIO_BUF_SIZE);
    j = AUDIO_BUF_SIZE / 2;
#endif

    g_tx_ev = rt_event_create("audio_tx_evt", RT_IPC_FLAG_FIFO);
    rt_kprintf("bf0_audio_tx_entry started, wait event\n");
    while (1)
    {
#ifndef ATEST_LOOPBACK
        j = AUD_TEST_FLEN - cnt > (AUDIO_BUF_SIZE / 2) ? (AUDIO_BUF_SIZE / 2) : AUD_TEST_FLEN - cnt;
        // read data
        memcpy(tx_pipe_data, audio_mem_buf + cnt, j);
        cnt += j;
        if (cnt >= AUD_TEST_FLEN)
            cnt = 44; // 0; // to data header
#else
        cnt = rt_ringbuffer_data_len(&atest_rbf);
        if (tx_start_flag == 0)
        {
            if (cnt < rbf_length / 4)
            {
                rt_thread_delay(20);
                rt_kprintf("buffered data %d\n", cnt);
                continue;
            }

            tx_start_flag = 1;
            rt_kprintf("Start TX !\n");
        }
        if (cnt > rbf_length / 8)
            rt_ringbuffer_get(&atest_rbf, tx_pipe_data, AUDIO_BUF_SIZE / 2);
        else // under flow
        {
            tx_start_flag = 0;
            rt_kprintf("Stop TX to cache data\n");
        }
#endif
        // wait dma done to fill to buffer
        rt_event_recv(g_tx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        if (buf_flag != NULL)
        {
            //rt_kprintf("Get g_tx_ev\n");
            //memcpy(buf_flag, tx_pipe_data, j);
            rt_device_write(g_mic, 0, tx_pipe_data, j);
            //rt_device_write(dev, 0, buf_flag, j);
            //buf_flag = NULL;
        }
        //LOG_I("filled %d\n", cnt);
    }
}


/**
* @brief  Audio receiving thread.
* This us audio process thread. It will capture audio data from I2S, and send to transport.
* @param[in]  param: unused.
*/
void bf0_audio_rx_entry(void *param)
{
    rt_uint32_t evt;
    int size;

    g_rx_ev = rt_event_create("audio_evt", RT_IPC_FLAG_FIFO);
    rt_kprintf("bf0_audio_rx_entry started, wait event\n");
    while (1)
    {
        rt_event_recv(g_rx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        //LOG_I("Got Audio event 0x%x\n", evt);
        while (1)
        {
            rt_size_t len;
            len = rt_device_read(g_mic, 0, g_pipe_data, AUDIO_BUF_SIZE);
            if (len != AUDIO_DATA_SIZE / 2)
            {
                //LOG_I("Got Audio size=%d\n", len);
                //LOG_HEX("g_pipe_data", 10, g_pipe_data, len > 16 ? 16 : len);
            }
            //LOG_I("Got Audio size=%d\n", len);
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
#ifndef ATEST_LOOPBACK
            if (aud_flen + len <= AUD_TEST_FLEN)
            {
                atest_save(mic_buf, g_pipe_data, len);
                mic_buf += len;
                aud_flen += len;
            }
#else
            size = rt_ringbuffer_data_len(&atest_rbf);
            if (size + len <= rbf_length)
                rt_ringbuffer_put(&atest_rbf, g_pipe_data, len);
            else
                rt_kprintf("Rx buffer almost full %d\n", size);
#endif
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
    rt_event_send(g_rx_ev, 1);
    return RT_EOK;
}

rt_err_t audio_tx_done(rt_device_t dev, void *buffer)
{
    //LOG_I("audio_tx_done \n");
    buf_flag = (uint8_t *)buffer;
    rt_event_send(g_tx_ev, 1);
    return RT_EOK;
}

#ifdef ATEST_LOOPBACK
static int atest_loop_start(void)
{
    // 1. malloc buffer,
    rbf_length = 0x100000;
    //rbf_pool = malloc(rbf_length);
    rbf_pool = (uint8_t *)0x60000000;
    if (rbf_pool == 0)
    {
        rt_kprintf("Alloc buffer 0x%x fail\n", rbf_length);
        return -1;
    }
    // 2. create ring buffer
    rt_ringbuffer_init(&atest_rbf, rbf_pool, rbf_length);
    // 3. start thread

    return 0;
}

static void atest_loop_stop(void)
{
    // 1. stop thread,

    // 2. destory ring buffer

    // 3. free buffer
    return;
}
#endif

/**
* @brief  Audio commands.
* This function provide 'audio' command to shell(FINSH) .
* The commands supported:
*   - audio open

      Open microphone and speaker device

    - audio config [sample rate]

      Configure microphone catpure sample rate

    - audio start rx

      Audio start capture, it will start \ref bf0_audio_rx_entry thread.

    - audio start tx

      Audio start replay, it will start \ref bf0_audio_tx_entry thread.

    - audio stop rx

      Audio stop capture

   - audio stop tx

      Audio stop replay
* @retval RT_EOK
*/
int cmd_audio(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "open") == 0)
        {
            if (g_mic == NULL)
            {
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
            }
        }
        if (strcmp(argv[1], "config") == 0)
        {
            if (g_mic)
            {
                struct rt_audio_caps caps;
                //caps.main_type = AUDIO_TYPE_OUTPUT;
                caps.main_type = AUDIO_TYPE_INPUT;      // for I2S2, configure RX will configure RX+TX
                caps.sub_type = AUDIO_DSP_PARAM;
                caps.udata.config.channels = atoi(argv[2]);
                caps.udata.config.samplefmt = atoi(argv[3]);    // depth
                caps.udata.config.samplerate = atoi(argv[4]);
                //caps.udata.value = atoi(argv[2]);
                rt_device_control(g_mic, AUDIO_CTL_CONFIGURE, &caps);

            }
        }
        if (strcmp(argv[1], "start") == 0)
        {
            if (g_mic)
            {
                int stream = 0;
                if (strcmp(argv[2], "tx") == 0)
                {
                    stream = AUDIO_STREAM_REPLAY;
                    // start replay thread
                    tx_tid = rt_thread_create("tx_th", bf0_audio_tx_entry, g_mic, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
                    if (tx_tid == NULL)
                    {
                        LOG_E("Create tx thread fail\n");
                        return RT_ERROR;
                    }
                    rt_thread_startup(tx_tid);
                    rt_device_set_tx_complete(g_mic, audio_tx_done);
                    rt_device_control(g_mic, AUDIO_CTL_START, &stream);
                    //rt_device_write(g_mic, 0, audio_tx_data, AUDIO_BUF_SIZE);
                }
                if (strcmp(argv[2], "rx") == 0)
                {
                    stream = AUDIO_STREAM_RECORD;
                    struct rt_audio_caps caps;
                    caps.main_type = AUDIO_TYPE_INPUT;      // for I2S2, configure RX will configure RX+TX
                    //caps.sub_type = AUDIO_DSP_SAMPLERATE;
                    caps.sub_type = AUDIO_DSP_PARAM;
                    rt_device_control(g_mic, AUDIO_CTL_GETCAPS, &caps);
                    atest_fill_header(caps.udata.config.samplerate, (uint16_t)caps.udata.config.channels, (uint16_t)caps.udata.config.samplefmt);
                    // start record thread
                    rx_tid = rt_thread_create("aud_th", bf0_audio_rx_entry, g_mic, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
                    if (rx_tid == NULL)
                    {
                        LOG_E("Create rx thread fail\n");
                        return RT_ERROR;
                    }
                    rt_thread_startup(rx_tid);
                    rt_device_set_rx_indicate(g_mic, audio_rx_ind);
                    rt_device_control(g_mic, AUDIO_CTL_START, &stream);
                }
#ifdef ATEST_LOOPBACK
                if (strcmp(argv[2], "loopback") == 0)
                {
                    tx_start_flag = 0;
                    int res = atest_loop_start();
                    if (res != 0)
                    {
                        LOG_E("Start loop test fail\n");
                        return 0;
                    }
                    stream = AUDIO_STREAM_RECORD;
                    // start record thread
                    rx_tid = rt_thread_create("aud_th", bf0_audio_rx_entry, g_mic, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
                    rt_thread_startup(rx_tid);
                    rt_device_set_rx_indicate(g_mic, audio_rx_ind);
                    rt_device_control(g_mic, AUDIO_CTL_START, &stream);

                    stream = AUDIO_STREAM_REPLAY;
                    // start replay thread
                    tx_tid = rt_thread_create("tx_th", bf0_audio_tx_entry, g_mic, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
                    rt_thread_startup(tx_tid);
                    rt_device_set_tx_complete(g_mic, audio_tx_done);
                    rt_device_control(g_mic, AUDIO_CTL_START, &stream);
                }
#endif
            }
        }
        if (strcmp(argv[1], "stop") == 0)
        {
            if (g_mic)
            {
                int stream = 0;
                if (strcmp(argv[2], "tx") == 0)
                {
                    stream = AUDIO_STREAM_REPLAY;
                    rt_device_control(g_mic, AUDIO_CTL_STOP, &stream);
                }
                if (strcmp(argv[2], "rx") == 0)
                {
                    stream = AUDIO_STREAM_RECORD;
                    rt_device_control(g_mic, AUDIO_CTL_STOP, &stream);
                }

            }
        }
        if (strcmp(argv[1], "tdebug") == 0)
        {
            audio_debug_out_i2st();
            audio_debug_out_txdma();
        }
        if (strcmp(argv[1], "rdebug") == 0)
        {
            audio_debug_out_i2sr();
            audio_debug_out_rxdma();
        }
    }
    return RT_EOK;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_audio, __cmd_audio, Test audio driver);

#endif

/// @} bsp_sample_audio
/// @} bsp_sample

#endif  /* BSP_USING_I2S */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
