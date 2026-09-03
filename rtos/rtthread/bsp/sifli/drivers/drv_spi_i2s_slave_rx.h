/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _DRV_SPI_I2S_SLAVE_RX_H_
#define _DRV_SPI_I2S_SLAVE_RX_H_

#include "rtdevice.h"

struct spi_i2s_audio_cfg_t
{
    SPI_TypeDef *spi_instance;         /*!< SPI device Handle used by this driver */
    DMA_Channel_TypeDef *dma_instance; /*!< DMA device Handle used by this driver */
    rt_uint8_t dma_request;          /*!< DMA request type for SPI, defined in dma_config.h */
    IRQn_Type dma_irq;               /*!< DMA irq  for SPI, defined in dma_config.h */
    PTC_TypeDef *ptc_instance;         /*!< PTC device Handle used by this driver */
    char *name;                      /*!< MIC device name, for example, 'mic_s' for recording device */
    rt_uint8_t is_record;            /*!< MIC device type, 1: for recording, 0: for playback*/
    RCC_MODULE_TYPE spi_mod;
};

struct bf0_spi_i2s_slave_rx
{
    struct rt_audio_device audio_device;
    SPI_HandleTypeDef hspi;
    DMA_HandleTypeDef hdma_rx;
    PTC_HandleTypeDef hptc[3];
    I2S_CFG_T i2s_rx_cfg;
    uint32_t *buf_pool;
    uint32_t buf_size;

    struct spi_i2s_audio_cfg_t *cfg;
    uint32_t ptc_trigger_pin_pad_val_bak;
};

#endif /* _DRV_SPI_I2S_SLAVE_RX_H_ */