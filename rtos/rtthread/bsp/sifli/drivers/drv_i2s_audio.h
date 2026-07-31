/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_I2S_AUDIO_H__
#define __DRV_I2S_AUDIO_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <drv_common.h>

#define MUTE_UNDER_MIN_VOLUME  (-256)

#define AUDIO_DATA_SIZE 640 //480

typedef void (*i2s_rx_callback_t)(char *name, uint8_t *data, rt_size_t len);


/**
 * I2S audio config structure definition
 */
struct i2s_audio_cfg_t
{
    DMA_Channel_TypeDef *dma_handle;      /*!< DMA device Handle used by this driver */
    I2S_TypeDef         *i2s_handle;         /*!< I2S device Handle used by this driver */
    char                *name;               /*!< Audio device name, for example, 'mic' for recording device */
    rt_uint8_t          dma_request;        /*!< DMA request type for I2S, defined in dma_config.h */
    rt_uint8_t          is_record;          /*!< Audio device type, 1: for recording, 0: for playback*/
    rt_uint8_t          reqdma_tx;        /*!< DMA request type for I2S TX */
    DMA_Channel_TypeDef *hdma_tx;      /*!< DMA device Handle used I2S TX */
    IRQn_Type            dma_irq;       /*!< DMA interrupt for I2S RX */
    IRQn_Type            tx_dma_irq;    /*!< DMA interrupt for I2S TX */
};

/**
 * I2S audio structure definition
 */
struct bf0_i2s_audio
{
    struct rt_audio_device audio_device;    /*!< audio device registerd to OS*/
    I2S_HandleTypeDef hi2s;
    const struct i2s_audio_cfg_t *config;
    uint8_t *rx_buf;
    uint8_t *tx_buf;
    uint8_t *tx_pos;
    uint16_t tx_buf_size;         /*!< I2S TX buffer size */
    i2s_rx_callback_t rx_callback;
};
/**
 * I2S track source structure definition
 */
typedef enum
{
    I2S_TRK_LSL_RSR    = 0, //left src left, right src right
    I2S_TRK_LSR_RSL   = 1, //left src right, right src left
    I2S_TRK_SRC_LPRD2 = 2, //left = (left+right)/2, right = (left+right)/2
    I2S_TRK_ALL_SRCL  = 3, //left src left, right src left
    I2S_TRK_ALL_SRCR  = 4, //left src right, right src right
} I2S_TRACK_SRC;

/**
 * I2S transfer interface definition
 */
typedef enum
{
    I2S_TRANS_INTF_DMA      = 0, //i2s data transfer from dma
    I2S_TRANS_INTF_AUDPRC   = 1, //i2s data transfer from audprc
} I2S_TRANS_INTF;
/**
 * I2S work mode definition
 */
typedef enum
{
    I2S_WORK_MASTER  = 0, //i2s work master mode
    I2S_WORK_SLAVE   = 1, //i2s work slave mode
} I2S_WORK_MODE;

#ifdef I2S_TDM_MODE_SUPPORT
/**
 * I2S tdm mode definition
 */
typedef enum
{
    TDM_I2S  = 0, //tdm i2s mode
    TDM_PCM  = 1, //tdm pcm mode
} I2S_TDM_MODE;
/**
  * @}
  */

/**
  * @brief I2S tdm cfg definition
  * @param[in] __USED__: 1:use tmd mode, 0:close tdm mode, default 0.
  * @param[in] __MODE__: 1:tmd pcm mode, 0:tdm i2s mode, default 0.
  * @param[in] __CLKDEG__: 1:use gclk for bclk de-glitching, 0:unused, default 0.
  * @param[in] __FYWG__: only tdm pcm mode, pcm fsync signal width, default 1.
  */
#define I2S_TDM_CFG(__USED__, __MODE__, __CLKDEG__, __FYWG__) \
        (__USED__ | (__MODE__ << 1) | (__CLKDEG__ << 2) | ((__FYWG__ &0xFFUL) << 4))
#endif

/**
 * I2S tx volume value definition
 */
typedef enum
{
    I2S_VOL_P_6DB,
    I2S_VOL_P_4P5DB,
    I2S_VOL_P_3DB,
    I2S_VOL_P_1P5DB,
    I2S_VOL_P_0DB,
    I2S_VOL_N_1P5DB,
    I2S_VOL_N_3DB,
    I2S_VOL_N_4P5DB,
    I2S_VOL_N_6DB,
    I2S_VOL_N_7P5DB,
    I2S_VOL_N_9DB,
    I2S_VOL_N_10P5DB,
    I2S_VOL_N_12DB,
    I2S_VOL_N_13P5DB,
    I2S_VOL_N_15DB,
    I2S_VOL_MUTE,
} I2S_VOL_VALUE;


void rt_device_set_i2s_dma_rx_callback(rt_device_t dev, i2s_rx_callback_t callback);

void bf0_i2s_device_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size); /*para is same to rt_device_write*/

#endif
