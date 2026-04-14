/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "board.h"
#include "drv_config.h"

#include "string.h"
#include "drv_i2s_audio.h"

#if defined (SYS_HEAP_IN_PSRAM)
    #undef calloc
    #undef free
    #undef malloc
    extern void *app_sram_alloc(rt_size_t size);
    extern void *app_sram_calloc(rt_size_t count, rt_size_t size);
    extern void *app_sram_free(void *ptr);
    #define  malloc(s)      app_sram_alloc(s)
    #define  calloc(c, s)   app_sram_calloc(c, s)
    #define  free(p)        app_sram_free(p)
#endif


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
    uint16_t tx_buf_size;         /*!< I2S TX buffer size */
};

static i2s_rx_callback_t rx_callback;

#ifdef ASIC
#ifdef SF32LB55X //xtal
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 125, 125,  5}, {44100, 136, 136,  4}, {32000, 185, 190,  5}, {24000, 250, 250, 10}, {22050, 272, 272,  8},
    {16000, 384, 384, 12}, {12000, 500, 500, 20}, {11025, 544, 544, 16}, { 8000, 750, 750, 30}
};//{16000, 375, 375, 15}  { 8000, 750, 750, 30}} { 8000, 768, 768, 24}
#else  //PLL
//PLL 16k 49.152M  44.1k  45.1584M
//lrclk_duty_high:PLL/spclk_div/samplerate/2: 64=49.152M/48k/8/2
//bclk:lrclk_duty_high/32
#define I2S_USE_DOUBLE_MCLK 0
#if I2S_USE_DOUBLE_MCLK
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 128, 128,  4}, {44100, 128, 128,  4}, {32000, 192, 192,  6}, {24000, 256, 256, 8}, {22050, 256, 256,  8},
    {16000, 384, 384, 12}, {12000, 512, 512, 16}, {11025, 512, 512, 16}, { 8000, 768, 768, 24}
};
#else
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 64, 64,  2}, {44100, 64, 64,  2}, {32000, 96, 96,  3}, {24000, 128, 128, 4}, {22050, 128, 128,  4},
    {16000, 192, 192, 6}, {12000, 256, 256, 8}, {11025, 256, 256, 8}, { 8000, 384, 384, 12}
};
#endif /* I2S_USE_DOUBLE_MCLK */
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
                if (txrx_clk_div[index].samplerate == rate)
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
#ifdef SF32LB58X
            // for i2s1,  rx must be same as tx
            if (hwp_i2s1 == hi2s->Instance && mode == 0)
            {
                hi2s->Init.rx_cfg.slave_mode = 0;
            }
#endif
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
                if (txrx_clk_div[index].samplerate == rate)
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
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;
    aud->tx_buf_size = AUDIO_DATA_SIZE;
    rx_callback = NULL;
    return RT_EOK;
}

/**
  * @brief  Shtudown audio device.
  * @param[in]  audio: audio device handle.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_shutdown(struct rt_audio_device *audio)
{
    struct bf0_i2s_audio *aud = (struct bf0_i2s_audio *) audio->parent.user_data;
    rx_callback = NULL;
    aud->tx_buf_size = AUDIO_DATA_SIZE;
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
#ifdef hwp_i2s3
    if (hi2s->Instance == hwp_i2s3)
    {
        HAL_RCC_EnableModule(RCC_MOD_I2S3);
    }
#endif
#ifdef hwp_i2s2
    if (hi2s->Instance == hwp_i2s2)
    {
        HAL_RCC_EnableModule(RCC_MOD_I2S2);
    }
#endif
#ifdef hwp_i2s1
    if (hi2s->Instance == hwp_i2s1)
    {
        HAL_RCC_EnableModule(RCC_MOD_I2S1);
    }
#endif
    // TODO: set to xtal now, change it if PLL can used
    //__HAL_I2S_CLK_XTAL(hi2s);   // xtal use 48M for asic
    //__HAL_I2S_SET_SPCLK_DIV(hi2s, 4);   // set to 12M to i2s
    //hi2s->Init.src_clk_freq = 12000000;
    // if use pll, set divider and freq as pll setting.
    __HAL_I2S_CLK_PLL(hi2s); //PLL
#if I2S_USE_DOUBLE_MCLK
    __HAL_I2S_SET_SPCLK_DIV(hi2s, 4);   // set to 12.288M to i2s (49.152M/4=12.288M)  PLL
#else
    __HAL_I2S_SET_SPCLK_DIV(hi2s, 8);   // set to 6.144M to i2s   PLL
#endif

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

    LOG_I("i2s buf size=%d", aud->tx_buf_size);
    RT_ASSERT((uint32_t)aud->tx_buf >= 0x20000000 && (uint32_t)aud->tx_buf < 0x60000000); //must in sram
    memset(aud->tx_buf, 0, AUDIO_DATA_SIZE);
    memset(aud->rx_buf, 0, AUDIO_DATA_SIZE);

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

            res = HAL_I2S_Transmit_DMA(&(aud->hi2s), aud->tx_buf, aud->tx_buf_size);
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
            res = HAL_I2S_Receive_DMA(&(aud->hi2s), aud->rx_buf, aud->tx_buf_size);
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
    case AUDIO_CTL_SET_TX_DMA_SIZE:
    {
        uint32_t dma_size = (uint32_t)args;
        aud->tx_buf_size = dma_size * 2;
        RT_ASSERT(dma_size  <= AUDIO_DATA_SIZE / 2);
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
    RT_ASSERT(size <= aud->tx_buf_size / 2);

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

void bf0_i2s_device_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{

    // i2s_save_to_file((uint8_t *)buffer, size);  // Save the I2S output music
    struct rt_audio_device *audio = (struct rt_audio_device *)dev;

    bf0_audio_trans(audio, buffer, NULL, size);
}

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
        h_i2s_audio[i].rx_buf = malloc(AUDIO_DATA_SIZE);
        h_i2s_audio[i].tx_buf = malloc(AUDIO_DATA_SIZE);
        h_i2s_audio[i].tx_pos = h_i2s_audio[i].tx_buf;

        if (bf0_i2s_audio_obj[i].i2s_handle != NULL)
        {
            I2S_HandleTypeDef *hi2s = &(h_i2s_audio[i].hi2s);

            hi2s->Instance = bf0_i2s_audio_obj[i].i2s_handle;

            // init dma handle and request, other parameters configure in HAL driver
            hi2s->hdmarx = calloc(1, sizeof(DMA_HandleTypeDef));
            hi2s->hdmarx->Instance = bf0_i2s_audio_obj[i].dma_handle;
            hi2s->hdmarx->Init.Request = bf0_i2s_audio_obj[i].dma_request;

            hi2s->hdmatx = calloc(1, sizeof(DMA_HandleTypeDef));
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
void rt_device_set_i2s_dma_rx_callback(i2s_rx_callback_t callback)
{
    rx_callback = callback;
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
    {
        if (rx_callback)
        {
            rx_callback(audio->parent.parent.name, haudio->rx_buf, haudio->tx_buf_size / 2);
        }
        else
        {
            rt_audio_rx_done(audio, haudio->rx_buf, haudio->tx_buf_size / 2);
        }
    }
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
    {
        if (rx_callback)
        {
            rx_callback(audio->parent.parent.name, haudio->rx_buf + haudio->tx_buf_size / 2, haudio->tx_buf_size / 2);
        }
        else
        {
            rt_audio_rx_done(audio, haudio->rx_buf + haudio->tx_buf_size / 2, haudio->tx_buf_size / 2);
        }
    }
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
    haudio->tx_pos = haudio->tx_buf;
    rt_audio_tx_complete(&(haudio->audio_device), haudio->tx_buf);
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
    haudio->tx_pos = haudio->tx_buf + haudio->tx_buf_size / 2;
    rt_audio_tx_complete(&(haudio->audio_device), haudio->tx_buf + haudio->tx_buf_size / 2);
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

#endif  /* BSP_USING_I2S */
