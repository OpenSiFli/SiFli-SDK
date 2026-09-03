/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_config.h"
#include "drv_spi.h"
#if defined(BSP_ENABLE_SPI_I2S_SLAVE_RX)

#define LOG_TAG "drv.i2s_slave_rx"
#include "drv_log.h"
#include "drv_spi_i2s_slave_rx.h"

/** @addtogroup bsp_driver Driver IO
 * @{
 */

/** @defgroup drv_i2s_slave_rx Audio
 * @brief I2s Slave  Mic BSP driver
 * This driver use spi + DMA + ptc to simulate I2S slave interface, support audio capture functions.
 * It register "mic_s" devices to OS. User could open this device to config and capture audio
 * @{
 */

#define BSP_SPI_I2S_SLAVE_RX_BUF_WORD_SIZE      (BSP_SPI_I2S_SLAVE_RX_BUF_SIZE / sizeof(uint32_t))
#define BSP_SPI_I2S_SLAVE_RX_HALF_BUF_WORD_SIZE (BSP_SPI_I2S_SLAVE_RX_BUF_WORD_SIZE >> 1)

static uint32_t slave_rx_data[BSP_SPI_I2S_SLAVE_RX_BUF_WORD_SIZE]; /* mic receive buffer */

static struct spi_i2s_audio_cfg_t bf0_i2s_slave_rx_cfg =
{
    .name                    = BSP_I2S_SLAVE_RX_NAME,
    .ptc_instance            = hwp_ptc1,
    .is_record               = 1,
#ifdef BSP_SPI_I2S_SLAVE_RX_USE_SPI1
    .spi_instance            = hwp_spi1,
    .dma_instance            = SPI1_RX_DMA_INSTANCE,
    .dma_request             = SPI1_RX_DMA_REQUEST,
    .dma_irq                 = SPI1_RX_DMA_IRQ,
    .spi_mod                 = RCC_MOD_SPI1,
#elif defined(BSP_SPI_I2S_SLAVE_RX_USE_SPI2)
    .spi_instance            = hwp_spi2,
    .dma_instance            = SPI2_RX_DMA_INSTANCE,
    .dma_request             = SPI2_RX_DMA_REQUEST,
    .dma_irq                 = SPI2_RX_DMA_IRQ,
    .spi_mod                 = RCC_MOD_SPI2,
#endif /* BSP_SPI_I2S_SLAVE_RX_USE_SPI2 */
};

static struct bf0_spi_i2s_slave_rx spi_i2s_slave_rx_obj;

/**
 * @brief  Get audio device capabilities.
 * @param[in]      audio: audio device handle.
 * @param[in,out]  caps: capability to get
 * @retval RT_EOK if success, otherwise -RT_ERROR
 */
static rt_err_t bf0_i2s_slave_rx_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;
    struct bf0_spi_i2s_slave_rx *mic = (struct bf0_spi_i2s_slave_rx *)audio->parent.user_data;

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
    {
        // case AUDIO_TYPE_OUTPUT:
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
            if (audio->replay == NULL)
            {
                result = -RT_ERROR;
                break;
            }
            // use samplefmt for input width, samplefmts for output width, samplerate for real number but not flag
            caps->udata.config.channels = (mic->i2s_rx_cfg.track == 1) ? 1 : 2;
            caps->udata.config.samplefmt = mic->i2s_rx_cfg.data_dw;      // AUDIO_FMT_PCM_U16_LE;
            caps->udata.config.samplerate = mic->i2s_rx_cfg.sample_rate; // AUDIO_SAMP_RATE_16K;

            break;
        case AUDIO_DSP_SAMPLERATE:
            caps->udata.value = mic->i2s_rx_cfg.sample_rate;
            // LOG_I("bf0_audio_getcaps %d\n", caps->udata.value);
            break;

        default:
            result = -RT_ERROR;
            break;
        }
        break;
    }
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
static rt_err_t bf0_i2s_slave_rx_configure(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;
    struct bf0_spi_i2s_slave_rx *mic = (struct bf0_spi_i2s_slave_rx *)audio->parent.user_data;
    I2S_CFG_T *p_i2s_rx_cfg = &mic->i2s_rx_cfg;

    switch (caps->main_type)
    {
    case AUDIO_TYPE_INPUT:
    {
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
        {
            if (16 != caps->udata.config.samplefmt)
            {
                /* only 16bit data width is supported */
                result = -RT_EINVAL;
            }
            p_i2s_rx_cfg->sample_rate = caps->udata.config.samplerate;
            p_i2s_rx_cfg->track = (caps->udata.config.channels == 1) ? 1 : 0;
        }
        break;
        case AUDIO_DSP_SAMPLERATE: // Config audio sample rate
        {
            p_i2s_rx_cfg->sample_rate = (uint32_t)caps->udata.value;
        }
        break;
        case AUDIO_DSP_CHANNELS: // Config channel
        {
            int chnl = caps->udata.value;
            p_i2s_rx_cfg->track = (chnl == 1) ? 1 : 0;
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
    {
        result = -RT_ERROR;
        break;
    }
    }

    return result;
}
/**
 * @brief  Initialize audio device.
 * @param[in]  audio: audio device handle.
 * @retval RT_EOK if success, otherwise -RT_ERROR
 */
static rt_err_t bf0_i2s_slave_rx_init(struct rt_audio_device *audio)
{
    return RT_EOK;
}

/**
 * @brief  Shtudown audio device.
 * @param[in]  audio: audio device handle.
 * @retval RT_EOK if success, otherwise -RT_ERROR
 */
static rt_err_t bf0_i2s_slave_rx_shutdown(struct rt_audio_device *audio)
{
    return RT_EOK;
}

/**
 * @brief  Start audio device for recording/playback.
 * @param[in]  audio: audio device handle.
 * @param[in]  stream: stream ID.
 * @retval RT_EOK if success, otherwise -RT_ERROR
 */
static rt_err_t bf0_i2s_slave_rx_spi_init(struct bf0_spi_i2s_slave_rx *mic)
{
    SPI_HandleTypeDef *p_hspi = &(mic->hspi);
    DMA_HandleTypeDef *p_hdma_rx = &(mic->hdma_rx);

    LOG_I("%s 0\n", __FUNCTION__);
    if (HAL_SPI_Init(p_hspi) != HAL_OK)
    {

        LOG_I("spi init fail!\n");
        goto ERROR_SPI;
    }
    LOG_I("%s 1\n", __FUNCTION__);

    HAL_DMA_Init(p_hdma_rx);
    __HAL_LINKDMA(p_hspi, hdmarx, mic->hdma_rx);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    HAL_NVIC_SetPriority(mic->cfg->dma_irq, 0, 0);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
    LOG_I("%s 2\n", __FUNCTION__);
    return RT_EOK;
ERROR_SPI:
    return RT_ERROR;
}

// __weak void BSP_I2s_Slave_Mic_Init()
// {
// }

static rt_err_t bf0_i2s_slave_rx_ptc_start(struct bf0_spi_i2s_slave_rx *mic)
{
    SPI_HandleTypeDef *p_hspi = &mic->hspi;
    PTC_HandleTypeDef *p_hptc0 = &mic->hptc[0];
    PTC_HandleTypeDef *p_hptc1 = &mic->hptc[1];
    PTC_HandleTypeDef *p_hptc2 = &mic->hptc[2];
    uint8_t ptc_trigger_pin;
    uint8_t sel_source;
    uint32_t ptc_trigger_pin_pad;
    uint32_t ptc_trigger_pin_pad_new_val;

    HAL_RCC_EnableModule(RCC_MOD_PTC1);
    HAL_RCC_ResetModule(RCC_MOD_PTC1);

    ptc_trigger_pin = BSP_SPI_I2S_SLAVE_RX_CS_PIN;
    ptc_trigger_pin_pad = (uint32_t)(&hwp_pinmux1->PAD_PA00 + ptc_trigger_pin);
    ptc_trigger_pin_pad_new_val = *(uint32_t *)ptc_trigger_pin_pad;
    mic->ptc_trigger_pin_pad_val_bak = ptc_trigger_pin_pad_new_val;
    /* disable input to cut off the CS signal */
    ptc_trigger_pin_pad_new_val &= ~HPSYS_PINMUX_PAD_PA00_IE_Msk;
    if (ptc_trigger_pin < 32)
    {
        sel_source = PTC_HCPU_PA31_0_A;
    }
    else if (ptc_trigger_pin < 64)
    {
        sel_source = PTC_HCPU_PA63_32_A;
    }
#if defined(HPSYS_PAD_PA_NUM) && (HPSYS_PAD_PA_NUM > 64)
    else if (ptc_trigger_pin < 96)
    {
        sel_source = PTC_HCPU_PA95_64_A;
    }
#endif /* HPSYS_PAD_PA_NUM > 64*/

    /* task1: enable SPI by the positive edge of CS pin */
    p_hptc0->Instance = bf0_i2s_slave_rx_cfg.ptc_instance;
    p_hptc0->Init.Channel = 0;                                         // Use PTC Channel 1
    p_hptc0->Init.Address = (uint32_t) & (p_hspi->Instance->TOP_CTRL); // SPI top control register
    p_hptc0->Init.data = SPI_TOP_CTRL_SSE;                             // data to handle with value in Address.
    p_hptc0->Init.Sel = sel_source;
    p_hptc0->Init.Operation = PTC_OP_OR;
    p_hptc0->Init.Tripol = 0;
    p_hptc0->Init.Trigger_Pin = ptc_trigger_pin;
    p_hptc0->Init.Delay = 0;
    p_hptc0->Init.RepEn = 1;
    p_hptc0->Init.RepTrig = 0;
    p_hptc0->Init.RepIRQ = 0;
    p_hptc0->Init.Pen = 0; /*init ptc0 don't run*/

    /* task2: enable task3 when task1 finishes */
    p_hptc1->Instance = bf0_i2s_slave_rx_cfg.ptc_instance;
    p_hptc1->Init.Channel = 1;                                      // Use PTC Channel 2
    p_hptc1->Init.Address = (uint32_t) & (p_hptc1->Instance->RCR3); // PTC channel 3 control register
    p_hptc1->Init.data = (1 << PTC_RCR3_REP_Pos);                   // enable task 3.
    p_hptc1->Init.Sel = PTC_HCPU_PTC1_DONE1;                        // PT1 CH1 DONE will trigger PTC1 channel2
    p_hptc1->Init.Operation = PTC_OP_OR;                            // PT
    p_hptc1->Init.Tripol = 0;
    p_hptc1->Init.Trigger_Pin = ptc_trigger_pin;
    p_hptc1->Init.Delay = 0;
    p_hptc1->Init.RepEn = 1; // Enable channel 2
    p_hptc1->Init.RepTrig = 0;
    p_hptc1->Init.RepIRQ = 0;
    p_hptc1->Init.Pen = 1;

    /* task3: disable CS pad input by the negative edge of CS pin */
    p_hptc2->Instance = bf0_i2s_slave_rx_cfg.ptc_instance;
    p_hptc2->Init.Channel = 2;                   // Use PTC Channel 3
    p_hptc2->Init.Address = ptc_trigger_pin_pad; // CS pad register
    p_hptc2->Init.data = ptc_trigger_pin_pad_new_val;                   // data to handle with value in Address.
    p_hptc2->Init.Operation = PTC_OP_WRITE;      //
    p_hptc2->Init.Sel = sel_source;
    p_hptc2->Init.Tripol = 1;
    p_hptc2->Init.Trigger_Pin = ptc_trigger_pin;
    p_hptc2->Init.Delay = 0;
    p_hptc2->Init.RepEn = 1;
    p_hptc2->Init.RepTrig = 0;
    p_hptc2->Init.RepIRQ = 0;
    p_hptc2->Init.Pen = 0;

    if (HAL_PTC_Init(p_hptc0) != HAL_OK) /*Initialize PTC0*/
    {
        /* Initialization Error */
        LOG_I("ptc0 init fail!\n");
        goto __ERROR;
    }
    HAL_PTC_Enable(p_hptc0, 1);

    if (HAL_PTC_Init(p_hptc1) != HAL_OK) /* Initialize PTC1*/
    {
        /* Initialization Error */
        LOG_I("ptc1 init fail!\n");
        goto __ERROR;
    }
    HAL_PTC_Enable(p_hptc1, 1);

    if (HAL_PTC_Init(p_hptc2) != HAL_OK) /*Initialize PTC2*/
    {
        /* Initialization Error */
        LOG_I("ptc2 init fail!\n");
        goto __ERROR;
    }
    HAL_PTC_Enable(p_hptc2, 1);

    /* kickoff the first task */
    p_hptc0->Instance->RCR1 |= (1 << PTC_RCR3_REP_Pos);

    return RT_EOK;

__ERROR:
    HAL_PTC_Enable(p_hptc1, 0);
    HAL_PTC_Enable(p_hptc0, 0);

    return RT_ERROR;
}

static void bf0_i2s_slave_rx_ptc_stop(struct bf0_spi_i2s_slave_rx *mic)
{
    PTC_HandleTypeDef *p_hptc0 = &mic->hptc[0];
    PTC_HandleTypeDef *p_hptc1 = &mic->hptc[1];
    PTC_HandleTypeDef *p_hptc2 = &mic->hptc[2];

    HAL_PTC_Enable(p_hptc2, 0);
    HAL_PTC_Enable(p_hptc1, 0);
    HAL_PTC_Enable(p_hptc0, 0);

    /* restore CS signal connection */
    *(&hwp_pinmux1->PAD_PA00 + BSP_SPI_I2S_SLAVE_RX_CS_PIN) = mic->ptc_trigger_pin_pad_val_bak;
}

static rt_err_t bf0_i2s_slave_rx_start(struct rt_audio_device *audio, int stream)
{
    struct bf0_spi_i2s_slave_rx *mic = (struct bf0_spi_i2s_slave_rx *)audio->parent.user_data;
    HAL_StatusTypeDef res = HAL_OK;
    SPI_HandleTypeDef *p_hspi = &mic->hspi;
    DMA_HandleTypeDef *p_hdma_rx = &mic->hdma_rx;

    // BSP_I2s_Slave_Mic_Init();

    if ((mic->hspi.State == HAL_I2S_STATE_RESET) || (mic->hspi.State == HAL_I2S_STATE_READY))
    {
        res = bf0_i2s_slave_rx_spi_init(mic);

        if (res != RT_EOK)
        {
            goto __ERROR_OUT;
        }
    }

    LOG_I("%s 1\n", __FUNCTION__);
    res = HAL_SPI_Receive_DMA(p_hspi, (uint8_t *)mic->buf_pool, mic->buf_size);
    if (res != RT_EOK)
    {
        goto __ERROR_OUT;
    }

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    HAL_NVIC_EnableIRQ(mic->cfg->dma_irq);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

    __HAL_SPI_DISABLE_IT(p_hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
    __HAL_SPI_DISABLE_IT(p_hspi, (SPI_IT_ERR));
    __HAL_SPI_ENABLE_IT(p_hspi, (SPI_MSK_IT_ERR));
    __HAL_SPI_DISABLE(p_hspi);

    if (RT_EOK != bf0_i2s_slave_rx_ptc_start(mic))
    {
        goto __ERROR_PTC;
    }

    LOG_I("bf0_i2s_slave_rx_start %d done\n", stream);
    return RT_EOK;

__ERROR_PTC:
    HAL_DMA_DeInit(p_hdma_rx);
    HAL_SPI_DeInit(p_hspi);

__ERROR_OUT:
    return RT_ERROR;
}

/**
 * @brief  Stop audio device for recording/playback.
 * @param[in]  audio: audio device handle.
 * @param[in]  stream: stream ID.
 * @retval RT_EOK if success, otherwise -RT_ERROR
 */
static rt_err_t bf0_i2s_slave_rx_stop(struct rt_audio_device *audio, int stream)
{
    struct bf0_spi_i2s_slave_rx *mic = (struct bf0_spi_i2s_slave_rx *)audio->parent.user_data;
    rt_err_t ret = RT_EOK;
    SPI_HandleTypeDef *p_hspi = &(mic->hspi);

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    HAL_NVIC_DisableIRQ(mic->cfg->dma_irq);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
    HAL_SPI_DMAStop(p_hspi);
    __HAL_SPI_DISABLE(p_hspi);

    bf0_i2s_slave_rx_ptc_stop(mic);
    // BSP_I2s_Slave_Mic_Init();

    return ret;
}

/**
 * @brief  Suspend audio device for recording/playback. (Currently unused)
 * @param[in]  audio: audio device handle.
 * @param[in]  stream: stream ID.
 * @retval RT_EOK if success, otherwise -RT_ERROR
 */

static rt_err_t bf0_i2s_slave_rx_suspend(struct rt_audio_device *audio, int stream)
{
    struct bf0_spi_i2s_slave_rx *mic = (struct bf0_spi_i2s_slave_rx *)audio->parent.user_data;
    rt_err_t ret = RT_EOK;
    SPI_HandleTypeDef *p_hspi = &(mic->hspi);

    HAL_SPI_DMAPause(p_hspi);
    return ret;
}

/**
 * @brief  Resume audio device for recording/playback. (Currently unused)
 * @param[in]  audio: audio device handle.
 * @param[in]  stream: stream ID.
 * @retval RT_EOK if success, otherwise -RT_ERROR
 */
static rt_err_t bf0_i2s_slave_rx_resume(struct rt_audio_device *audio, int stream)
{
    struct bf0_spi_i2s_slave_rx *mic = (struct bf0_spi_i2s_slave_rx *)audio->parent.user_data;
    rt_err_t ret = RT_EOK;
    SPI_HandleTypeDef *p_hspi = &(mic->hspi);
    HAL_SPI_DMAResume(p_hspi);
    return ret;
}

/**
 * @brief  AUDIO controls. (Currently unused)
 * @param[in]  audio: audio device handle.
 * @param[in]  cmd: control commands.
 * @param[in]  args: control command arguments.
 * @retval RT_EOK if success, otherwise -RT_ERROR
 */
static rt_err_t bf0_i2s_slave_rx_control(struct rt_audio_device *audio, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    struct bf0_spi_i2s_slave_rx *mic = (struct bf0_spi_i2s_slave_rx *)audio->parent.user_data;
    SPI_HandleTypeDef *p_hspi = &(mic->hspi);

    switch (cmd)
    {
    case AUDIO_CTL_HWRESET:
        break;
    case RT_DEVICE_CTRL_SUSPEND:
    {

        break;
    }
    case RT_DEVICE_CTRL_RESUME:
    {
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
static rt_size_t bf0_i2s_slave_rx_trans(struct rt_audio_device *audio, const void *writeBuf, void *readBuf, rt_size_t size)
{
    struct bf0_spi_i2s_slave_rx *mic = (struct bf0_spi_i2s_slave_rx *)audio->parent.user_data;
    HAL_StatusTypeDef res = HAL_OK;
    if (writeBuf != NULL)
    {
        return 0;
    }

    if (readBuf != NULL)
    {
        // rt_kprintf("bf0_i2s_slave_rx_trans: 0x%p, %d\n", readBuf, size);

        res = HAL_SPI_Receive_DMA(&(mic->hspi), readBuf, size);
    }
    if (res != HAL_OK)
        return 0;

    return size;
}

static const struct rt_audio_ops s_slave_rx_ops =
{
    .getcaps = bf0_i2s_slave_rx_getcaps,
    .configure = bf0_i2s_slave_rx_configure,

    .init = bf0_i2s_slave_rx_init,
    .shutdown = bf0_i2s_slave_rx_shutdown,
    .start = bf0_i2s_slave_rx_start,
    .stop = bf0_i2s_slave_rx_stop,
    .suspend = bf0_i2s_slave_rx_suspend,
    .resume = bf0_i2s_slave_rx_resume,
    .control = bf0_i2s_slave_rx_control,
    .transmit = bf0_i2s_slave_rx_trans,
};

/**
 * @} I2S slave mic device
 */

/**
 * @brief  I2S slave mic device create
 */
int rt_bf0_i2s_rx_init(void)
{
    int result;
    SPI_HandleTypeDef *p_hspi;
    DMA_HandleTypeDef *p_hdma_rx;
    I2S_CFG_T *p_i2s_rx_cfg;

    spi_i2s_slave_rx_obj.audio_device.ops = (struct rt_audio_ops *)&s_slave_rx_ops;
    spi_i2s_slave_rx_obj.buf_pool   = slave_rx_data;
    spi_i2s_slave_rx_obj.buf_size   = BSP_SPI_I2S_SLAVE_RX_BUF_WORD_SIZE;
    spi_i2s_slave_rx_obj.cfg        = &bf0_i2s_slave_rx_cfg;

    RT_ASSERT(bf0_i2s_slave_rx_cfg.spi_instance && bf0_i2s_slave_rx_cfg.ptc_instance);

    p_hspi = &spi_i2s_slave_rx_obj.hspi;
    p_hdma_rx = &spi_i2s_slave_rx_obj.hdma_rx;

    HAL_RCC_EnableModule(bf0_i2s_slave_rx_cfg.spi_mod);

    p_hspi->Instance = bf0_i2s_slave_rx_cfg.spi_instance;

    // init spi dma handle and request, other parameters configure in HAL driver
    p_hspi->Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
    p_hspi->Init.Mode = SPI_MODE_SLAVE;
    p_hspi->Init.DataSize = SPI_DATASIZE_32BIT;
    p_hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
    p_hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    p_hspi->Init.BaudRatePrescaler = 1; // slave mode clock(HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1) + baundRate / 2) / baundRate;
    p_hspi->Init.FrameFormat = SPI_FRAME_FORMAT_SPI;
    p_hspi->Init.SFRMPol = SPI_SFRMPOL_LOW;
    __HAL_LINKDMA(p_hspi, hdmarx, spi_i2s_slave_rx_obj.hdma_rx);

    p_hspi->State = HAL_SPI_STATE_RESET;

    p_hdma_rx->Instance = bf0_i2s_slave_rx_cfg.dma_instance;      // SPI1_RX_DMA_INSTANCE;
    p_hdma_rx->Init.Request = bf0_i2s_slave_rx_cfg.dma_request; // SPI1_RX_DMA_REQUEST;
    p_hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    p_hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    p_hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
    p_hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    p_hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    p_hdma_rx->Init.Mode = DMA_CIRCULAR; // DMA_CIRCULAR;
    p_hdma_rx->Init.Priority = DMA_PRIORITY_LOW;

    /*config i2s */
    p_i2s_rx_cfg = &spi_i2s_slave_rx_obj.i2s_rx_cfg;
    p_i2s_rx_cfg->data_dw = 16;
    p_i2s_rx_cfg->bus_dw = 32;
    p_i2s_rx_cfg->pcm_dw = 16;
    p_i2s_rx_cfg->slave_mode = 0; // master mode
    p_i2s_rx_cfg->chnl_sel = 0;   // left/right all set to left
    p_i2s_rx_cfg->sample_rate = 16000;
    p_i2s_rx_cfg->track = 0; // stereo
    p_i2s_rx_cfg->lrck_invert = 0;
    p_i2s_rx_cfg->bclk = 800000;
    p_i2s_rx_cfg->extern_intf = 0;

    p_i2s_rx_cfg->clk_div_index = 5;
    p_i2s_rx_cfg->clk_div = NULL;

    result = rt_audio_register(&(spi_i2s_slave_rx_obj.audio_device),
                               bf0_i2s_slave_rx_cfg.name, RT_DEVICE_FLAG_RDWR, &spi_i2s_slave_rx_obj);
    return result;
}

INIT_DEVICE_EXPORT(rt_bf0_i2s_rx_init);

/** @addtogroup bsp_sample BSP driver sample commands.
 * @{
 */

/** @defgroup bsp_sample_slave_rx  sample commands
 * @brief Audio sample commands
 *
 * This sample commands demonstrate the usage of i2s slave mic driver.
 * @{
 */

/**
 * @brief  DMA IRQHandler
 */

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
#ifdef BSP_SPI_I2S_SLAVE_RX_USE_SPI2
void SPI2_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(spi_i2s_slave_rx_obj.hspi.hdmarx);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_SPI_I2S_SLAVE_RX_USE_SPI2 */

#ifdef BSP_SPI_I2S_SLAVE_RX_USE_SPI1
void SPI1_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(spi_i2s_slave_rx_obj.hspi.hdmarx);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_SPI_I2S_SLAVE_RX_USE_SPI1 */
#endif /* DMA_SUPPORT_DYN_CHANNEL_ALLOC */

/**
 * @brief Process SPI Rx Transfer completed data for the I2S slave mic.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *         the configuration information for SPI module
 * @retval None
 */
static void i2s_slave_rx_cplt_impl(SPI_HandleTypeDef *hspi)
{
    struct bf0_spi_i2s_slave_rx *mic = rt_container_of(hspi, struct bf0_spi_i2s_slave_rx, hspi);
    struct rt_audio_device *audio = &(mic->audio_device);
    uint8_t *read_buffer;

    /*L R  data process    */
    for (uint32_t index = BSP_SPI_I2S_SLAVE_RX_HALF_BUF_WORD_SIZE; index < BSP_SPI_I2S_SLAVE_RX_BUF_WORD_SIZE; index++)
    {
        if (index < (BSP_SPI_I2S_SLAVE_RX_BUF_WORD_SIZE - 1))
        {
            slave_rx_data[index] = ((slave_rx_data[index] << 1) | (slave_rx_data[index + 1] >> 31));
        }
        else
        {
            slave_rx_data[index] = (slave_rx_data[index] << 1);
        }
    }

    read_buffer = (uint8_t *)(mic->buf_pool + BSP_SPI_I2S_SLAVE_RX_HALF_BUF_WORD_SIZE);

    if (audio != NULL)
        rt_audio_rx_done(audio, read_buffer, BSP_SPI_I2S_SLAVE_RX_HALF_BUF_WORD_SIZE * sizeof(uint32_t));
}

/**
 * @brief Process SPI Rx Transfer half completed data for the I2S slave mic.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *         the configuration information for SPI module
 * @retval None
 */
static void i2s_slave_rx_half_cplt_impl(SPI_HandleTypeDef *hspi)
{
    struct bf0_spi_i2s_slave_rx *mic = rt_container_of(hspi, struct bf0_spi_i2s_slave_rx, hspi);
    struct rt_audio_device *audio = &(mic->audio_device);
    uint8_t *read_buffer;

    /*L R  data process    */
    for (uint32_t index = 0; index < BSP_SPI_I2S_SLAVE_RX_HALF_BUF_WORD_SIZE; index++)
    {
        if (index < (BSP_SPI_I2S_SLAVE_RX_HALF_BUF_WORD_SIZE - 1))
        {
            slave_rx_data[index] = ((slave_rx_data[index] << 1) | (slave_rx_data[index + 1] >> 31));
        }
        else
        {
            slave_rx_data[index] = (slave_rx_data[index] << 1);
        }
    }

    read_buffer = (uint8_t *)mic->buf_pool;

    if (audio != NULL)
        rt_audio_rx_done(audio, read_buffer, BSP_SPI_I2S_SLAVE_RX_HALF_BUF_WORD_SIZE * sizeof(uint32_t));
}

/*
 * When drv_spi.c provides the HAL SPI callbacks (see DRV_SPI_PROVIDES_HAL_CALLBACKS
 * in drv_spi.h) it dispatches them to the hooks below for this driver's SPI handle.
 * Otherwise (drv_spi.c not built) this driver provides the HAL callbacks itself so
 * it works standalone.
 */
#ifdef DRV_SPI_PROVIDES_HAL_CALLBACKS
rt_bool_t bsp_i2s_slave_rx_owns(SPI_HandleTypeDef *hspi)
{
    return (hspi == &spi_i2s_slave_rx_obj.hspi) ? RT_TRUE : RT_FALSE;
}

void bsp_i2s_slave_rx_cplt(SPI_HandleTypeDef *hspi)
{
    i2s_slave_rx_cplt_impl(hspi);
}

void bsp_i2s_slave_rx_half_cplt(SPI_HandleTypeDef *hspi)
{
    i2s_slave_rx_half_cplt_impl(hspi);
}
#else
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    i2s_slave_rx_cplt_impl(hspi);
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    i2s_slave_rx_half_cplt_impl(hspi);
}
#endif /* DRV_SPI_PROVIDES_HAL_CALLBACKS */

/// @} bsp_sample_audio
/// @} bsp_sample

#endif
