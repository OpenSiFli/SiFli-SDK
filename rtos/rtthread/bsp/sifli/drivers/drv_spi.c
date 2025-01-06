/**
  ******************************************************************************
  * @file   drv_spi.c
  * @author Sifli software development team
  * @brief SPI BSP driver
  * @{
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

#include "board.h"

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_spi SPI
  * @brief SPI BSP driver
  * @{
  */

#if defined(RT_USING_SPI) || defined(_SIFLI_DOXYGEN_)

//#if defined(BSP_USING_SPI1) || defined(BSP_USING_SPI2) || defined(BSP_USING_SPI3) || defined(BSP_USING_SPI4) || defined(BSP_USING_SPI5) || defined(BSP_USING_SPI6) || defined(_SIFLI_DOXYGEN_)
/* this driver can be disabled at menuconfig → RT-Thread Components → Device Drivers */

#include "drv_spi.h"
#include "drv_config.h"
#include <string.h>

//#define DRV_DEBUG
#define LOG_TAG              "drv.spi"
#include <drv_log.h>

enum
{
#ifdef BSP_USING_SPI1
    SPI1_INDEX,
#endif
#ifdef BSP_USING_SPI2
    SPI2_INDEX,
#endif
#ifdef BSP_USING_SPI3
    SPI3_INDEX,
#endif
#ifdef BSP_USING_SPI4
    SPI4_INDEX,
#endif
#ifdef BSP_USING_SPI5
    SPI5_INDEX,
#endif
#ifdef BSP_USING_SPI6
    SPI6_INDEX,
#endif
    SPI_MAX
};

static struct sifli_spi_config spi_config[] =
{
#ifdef BSP_USING_SPI1
    SPI1_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI2
    SPI2_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI3
    SPI3_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI4
    SPI4_BUS_CONFIG,
#endif

};

static struct rt_spi_configuration rt_spi_cfg_default[SPI_MAX] =
{
#ifdef BSP_USING_SPI1
    {
        RT_SPI_MODE_1 | RT_SPI_MSB | RT_SPI_MASTER,//rt_uint8_t mode;
        8,        //rt_uint8_t data_width;
        RT_SPI_MOTO,        //rt_uint16_t reserved;
        24000000  //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_SPI2
    {
        RT_SPI_MODE_1 | RT_SPI_MSB | RT_SPI_SLAVE,//rt_uint8_t mode;
        8,        //rt_uint8_t data_width;
        RT_SPI_MOTO,        //rt_uint16_t reserved;
        24000000  //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_SPI3
    {
        RT_SPI_MODE_1 | RT_SPI_MSB | RT_SPI_MASTER,//rt_uint8_t mode;
        8,        //rt_uint8_t data_width;
        RT_SPI_MOTO,        //rt_uint16_t reserved;
        24000000  //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_SPI4
    {
        RT_SPI_MODE_1 | RT_SPI_MSB | RT_SPI_MASTER,//rt_uint8_t mode;
        8,        //rt_uint8_t data_width;
        RT_SPI_MOTO,        //rt_uint16_t reserved;
        24000000  //rt_uint32_t max_hz;
    }
#endif
};


struct sifli_spi spi_bus_obj[SPI_MAX];


static uint32_t get_index_by_bus_handle(struct rt_spi_bus *bus)
{
    for (uint32_t i = 0; i < SPI_MAX; i++)
    {
        if (bus == &spi_bus_obj[i].spi_bus) return i;
    }

    return UINT32_MAX;
}

/**
* @brief  Spi initial.
* @param[in]  spi_drv: spi driver handler structure.
* @param[in]  cfg: spi configure structure.
* @retval RT_EOK if success.
*/
static rt_err_t sifli_spi_init(struct rt_spi_device *device, struct sifli_spi *spi_drv, struct rt_spi_configuration *cfg)
{
    RT_ASSERT(spi_drv != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    SPI_HandleTypeDef *spi_handle = &spi_drv->handle;

    if (cfg->mode & RT_SPI_SLAVE)
    {
        spi_handle->Init.Mode = SPI_MODE_SLAVE;
    }
    else
    {
        spi_handle->Init.Mode = SPI_MODE_MASTER;
    }


    if (cfg->mode & RT_SPI_3WIRE)
    {
        spi_handle->Init.Direction = SPI_DIRECTION_1LINE;
    }
    else
    {
        //RT_ASSERT(0);
        spi_handle->Init.Direction = SPI_DIRECTION_2LINES;
    }

    if (cfg->data_width == 8)
    {
        spi_handle->Init.DataSize = SPI_DATASIZE_8BIT;
        //spi_handle->TxXferSize = 8;
        //spi_handle->RxXferSize = 8;
    }
    else if (cfg->data_width == 16)
    {
        spi_handle->Init.DataSize = SPI_DATASIZE_16BIT;
        //spi_handle->TxXferSize = 16;
        //spi_handle->RxXferSize = 16;
    }
    else
    {
        return RT_EIO;
    }

    if (cfg->mode & RT_SPI_CPHA)
    {
        spi_handle->Init.CLKPhase = SPI_PHASE_2EDGE;
    }
    else
    {
        spi_handle->Init.CLKPhase = SPI_PHASE_1EDGE;
    }

    if (cfg->mode & RT_SPI_CPOL)
    {
        spi_handle->Init.CLKPolarity = SPI_POLARITY_HIGH;
    }
    else
    {
        spi_handle->Init.CLKPolarity = SPI_POLARITY_LOW;
    }
    spi_handle->core = spi_drv->config->core;

#ifdef SF32LB55X
    rt_uint32_t SPI_APB_CLOCK = HAL_RCC_GetPCLKFreq(spi_handle->core, 1);
#else
    rt_uint32_t SPI_APB_CLOCK = 48000000; /* always 48MHz to SPI1&2 */

#ifdef BSP_USING_SPI3
    if (SPI3 == spi_handle->Instance)
    {
        SPI_APB_CLOCK = 24000000;  /* always 24MHz to SPI3*/
    }
#endif /* BSP_USING_SPI3 */
#ifdef BSP_USING_SPI4
    if (SPI4 == spi_handle->Instance)
    {
        SPI_APB_CLOCK = 24000000;  /* always 24MHz to SPI4 */
    }
#endif /* BSP_USING_SPI4 */
#endif /* SF32LB55X */

    //SPI_APB_CLOCK = HAL_RCC_GetPCLK2Freq();
    //SPI_APB_CLOCK = 0;
    //spi_handle->Init.BaudRatePrescaler = 0x30;
    //spi_handle->Init.BaudRatePrescaler = 0xC;
    spi_handle->Init.BaudRatePrescaler = (SPI_APB_CLOCK + cfg->max_hz / 2) / cfg->max_hz;

    if (RT_SPI_MOTO == (cfg->frameMode & RT_SPI_FRAME_MASK))
    {
        spi_handle->Init.FrameFormat = SPI_FRAME_FORMAT_SPI;
    }
    else if (RT_SPI_TI == (cfg->frameMode & RT_SPI_FRAME_MASK))
    {
        spi_handle->Init.FrameFormat = SPI_FRAME_FORMAT_SSP;
    }
    else if (RT_SPI_NM == (cfg->frameMode & RT_SPI_FRAME_MASK))
    {
        spi_handle->Init.FrameFormat = SPI_FRAME_FORMAT_NM;
    }
    spi_handle->Init.SFRMPol = SPI_SFRMPOL_HIGH;

    //spi_handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi_handle->State = HAL_SPI_STATE_RESET;

    if (hwp_spi1 == spi_handle->Instance)
        HAL_RCC_EnableModule(RCC_MOD_SPI1);
    else if (hwp_spi2 == spi_handle->Instance)
        HAL_RCC_EnableModule(RCC_MOD_SPI2);
#ifdef BSP_USING_SPI3
    else if (hwp_spi3 == spi_handle->Instance)
        HAL_RCC_EnableModule(RCC_MOD_SPI3);
#endif /* BSP_USING_SPI3 */
#ifdef hwp_spi4
    else if (hwp_spi4 == spi_handle->Instance)
        HAL_RCC_EnableModule(RCC_MOD_SPI4);
#endif /* BSP_USING_SPI4 */

    if (HAL_SPI_Init(spi_handle) != HAL_OK)
    {
        return RT_EIO;
    }

    /* DMA configuration
       Enable DMA if SPI bus support DMA & device open flag enable DMA
    */
    if ((spi_drv->spi_dma_flag & SPI_USING_RX_DMA_FLAG) && (device->parent.open_flag & RT_DEVICE_FLAG_DMA_RX))
    {
        if (cfg->data_width <= 8)
        {
            spi_drv->dma.handle_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            spi_drv->dma.handle_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        }
        else
        {
            spi_drv->dma.handle_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
            spi_drv->dma.handle_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        }
#ifdef BSP_USING_SPI_CAMERA
        // <add for spi camera start: circular  RX DMA
        if (!strcmp(device->parent.parent.name, "camera"))
        {
            LOG_D("spi dma rx circular\n");
            spi_drv->dma.handle_rx.Init.Mode = DMA_CIRCULAR;
        }
        // add for spi camera end>
#endif
        HAL_DMA_Init(&spi_drv->dma.handle_rx);

        __HAL_LINKDMA(&spi_drv->handle, hdmarx, spi_drv->dma.handle_rx);

        /* NVIC configuration for DMA transfer complete interrupt */
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
        HAL_NVIC_SetPriority(spi_drv->config->dma_rx->dma_irq, 0, 0);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
        //HAL_NVIC_EnableIRQ(spi_drv->config->dma_rx->dma_irq);
    }

    if ((spi_drv->spi_dma_flag & SPI_USING_TX_DMA_FLAG) && (device->parent.open_flag & RT_DEVICE_FLAG_DMA_TX))
    {
        if (cfg->data_width <= 8)
        {
            spi_drv->dma.handle_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            spi_drv->dma.handle_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        }
        else
        {
            spi_drv->dma.handle_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
            spi_drv->dma.handle_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        }

        HAL_DMA_Init(&spi_drv->dma.handle_tx);

        __HAL_LINKDMA(&spi_drv->handle, hdmatx, spi_drv->dma.handle_tx);

        /* NVIC configuration for DMA transfer complete interrupt */
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
        HAL_NVIC_SetPriority(spi_drv->config->dma_tx->dma_irq, 0, 1);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
        //HAL_NVIC_EnableIRQ(spi_drv->config->dma_tx->dma_irq);
    }

    //__HAL_SPI_ENABLE(spi_handle);

    LOG_D("%s init done", spi_drv->config->bus_name);
    return RT_EOK;
}

/**
* @brief  Spi tranfer data.
* @param[in]  device: spi device handler.
* @param[in]  message: message structure.
* @retval transfer data length.
*/
#define USR_CONTROL_CS      // cs high/low control by cs_take/cs_release flag
static rt_uint32_t spixfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    HAL_StatusTypeDef state;
    rt_size_t message_length, already_send_length;
    rt_uint16_t send_length;
    rt_uint8_t *recv_buf;
    const rt_uint8_t *send_buf;
    rt_err_t rt_err_v;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->bus->parent.user_data != RT_NULL);
    RT_ASSERT(message != RT_NULL);

    struct sifli_spi *spi_drv =  rt_container_of(device->bus, struct sifli_spi, spi_bus);
    SPI_HandleTypeDef *spi_handle = &spi_drv->handle;
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif  /* RT_USING_PM */

#ifdef USR_CONTROL_CS
    // cs_take==1 set cs low manual , hold do not release
    if (message->cs_take == 1)
    {
        LOG_D("Transfer entry : take %d, release %d\n", message->cs_take, message->cs_release);
        __HAL_SPI_TAKE_CS(spi_handle);
    }
#endif
    rt_uint32_t spi_index;

    spi_index = get_index_by_bus_handle(device->bus);


    LOG_D("%s transfer prepare and start", spi_drv->config->bus_name);
    LOG_D("%s sendbuf: %X, recvbuf: %X, length: %d",
          spi_drv->config->bus_name,
          (uint32_t)message->send_buf,
          (uint32_t)message->recv_buf, message->length);

    message_length = message->length;
    recv_buf = message->recv_buf;
    send_buf = message->send_buf;
    while (message_length)
    {
        rt_err_v = rt_sem_control(spi_drv->spi_sema, RT_IPC_CMD_RESET, 0);
        RT_ASSERT(RT_EOK == rt_err_v);

        /* the HAL library use uint16 to save the data length */
        if (message_length > 65535)
        {
            send_length = 65535;
            message_length = message_length - 65535;
        }
        else
        {
            send_length = message_length;
            message_length = 0;
        }

        /* calculate the start address */
        already_send_length = message->length - send_length - message_length;
        send_buf = (rt_uint8_t *)message->send_buf + already_send_length;
        recv_buf = (rt_uint8_t *)message->recv_buf + already_send_length;

        /* start once data exchange in DMA mode */
        if (message->send_buf && message->recv_buf)
        {

            //if((pDevice->open_flag & RT_DEVICE_FLAG_DMA_TX) && (pDevice->open_flag & RT_DEVICE_FLAG_DMA_RX))
            //if ((spi_drv->spi_dma_flag & SPI_USING_TX_DMA_FLAG) && (spi_drv->spi_dma_flag & SPI_USING_RX_DMA_FLAG))
            if ((device->parent.open_flag & RT_DEVICE_FLAG_DMA_TX) && (device->parent.open_flag & RT_DEVICE_FLAG_DMA_RX))
            {
                mpu_dcache_invalidate(recv_buf, send_length);
                state = HAL_SPI_TransmitReceive_DMA(spi_handle, (uint8_t *)send_buf, (uint8_t *)recv_buf, send_length);
            }
            else if ((device->parent.open_flag & RT_DEVICE_FLAG_INT_TX) && (device->parent.open_flag & RT_DEVICE_FLAG_INT_RX))
            {
                state = HAL_SPI_TransmitReceive_IT(spi_handle, (uint8_t *)send_buf, (uint8_t *)recv_buf, send_length);
            }
            else
            {
                state = HAL_SPI_TransmitReceive(spi_handle, (uint8_t *)send_buf, (uint8_t *)recv_buf, send_length, 5000);
            }
        }
        else if (message->send_buf)
        {
            //if (spi_drv->spi_dma_flag & SPI_USING_TX_DMA_FLAG)
            if (device->parent.open_flag & RT_DEVICE_FLAG_DMA_TX)
            {
                state = HAL_SPI_Transmit_DMA(spi_handle, (uint8_t *)send_buf, send_length);
            }
            else if (device->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
            {
                state = HAL_SPI_Transmit_IT(spi_handle, (uint8_t *)send_buf, send_length);
            }
            else
            {
                state = HAL_SPI_Transmit(spi_handle, (uint8_t *)send_buf, send_length, 5000);
            }
        }
        else if (message->recv_buf)
        {
            //memset((uint8_t *)recv_buf, 0xff, send_length);
            //if (spi_drv->spi_dma_flag & SPI_USING_RX_DMA_FLAG)
            if (device->parent.open_flag & RT_DEVICE_FLAG_DMA_RX)
            {
                mpu_dcache_invalidate(recv_buf, send_length);
                state = HAL_SPI_Receive_DMA(spi_handle, (uint8_t *)recv_buf, send_length);
            }
            else if (device->parent.open_flag & RT_DEVICE_FLAG_INT_RX)
            {
                state = HAL_SPI_Receive_IT(spi_handle, (uint8_t *)recv_buf, send_length);
            }
            else
            {
                state = HAL_SPI_Receive(spi_handle, (uint8_t *)recv_buf, send_length, 5000);
            }
        }

        /*Check return result*/
        if (state != HAL_OK)
        {
            LOG_E("spi transfer errorA : %d, errcode=%x", state, HAL_SPI_GetError(spi_handle));
            spi_handle->State = HAL_SPI_STATE_READY;
            break;
        }
        else
        {
            switch (HAL_SPI_GetState(spi_handle))
            {
            case HAL_SPI_STATE_READY:
                state = HAL_OK;
                break;


            case HAL_SPI_STATE_BUSY: //Interrupt or DMA mode, wait semaphore
            case HAL_SPI_STATE_BUSY_RX:
            case HAL_SPI_STATE_BUSY_TX:
            case HAL_SPI_STATE_BUSY_TX_RX:
            {
                rt_err_v = rt_sem_take(spi_drv->spi_sema, 5000);
                if (-RT_ETIMEOUT == rt_err_v)
                {
                    LOG_E("spi sem timeout!");
                    state = HAL_TIMEOUT;
                }
            }
            break;

            case HAL_SPI_STATE_ERROR:
            default:
                state = HAL_ERROR;
                break;
            }
        }

        if ((HAL_OK == state) && (HAL_SPI_ERROR_NONE != HAL_SPI_GetError(spi_handle)))
        {
            LOG_E("Error occurred.");
            state = HAL_ERROR;
        }

        if (state != HAL_OK)
        {
            LOG_E("spi transfer errorB : %d, errcode=%x", state, HAL_SPI_GetError(spi_handle));
            spi_handle->State = HAL_SPI_STATE_READY;
            break;
        }

    }


#ifdef USR_CONTROL_CS
    // cs_release == 1 need manual release
    if (message->cs_release == 1)
    {
        LOG_D("Transfer exit : take %d, release %d\n", message->cs_take, message->cs_release);
        __HAL_SPI_RELEASE_CS(spi_handle);
    }
#endif

#ifdef RT_USING_PM
    rt_pm_hw_device_stop();
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */


    return (HAL_OK == state) ? message->length : 0;
}

/**
* @brief  Spi configuration.
* @param[in]  device: spi device handler.
* @param[in]  configuration: configuration structure.
* @retval RT_EOK if success.
*/
static rt_err_t spi_configure(struct rt_spi_device *device,
                              struct rt_spi_configuration *configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    struct sifli_spi *spi_drv =  rt_container_of(device->bus, struct sifli_spi, spi_bus);
    spi_drv->cfg = configuration;

    return sifli_spi_init(device, spi_drv, configuration);
}

static rt_err_t spi_control(struct rt_spi_device *device, int cmd, void *arg)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(arg != RT_NULL);

    struct sifli_spi *spi_drv =  rt_container_of(device->bus, struct sifli_spi, spi_bus);

    switch (cmd)
    {
    /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        if ((((rt_uint32_t)arg) & RT_DEVICE_FLAG_INT_RX) || (((rt_uint32_t)arg) & RT_DEVICE_FLAG_INT_TX))
        {
            NVIC_DisableIRQ(spi_drv->config->irq_type);
            __HAL_SPI_DISABLE_IT(&(spi_drv->handle), (SPI_IT_RXNE | SPI_IT_ERR | SPI_IT_TXE));
        }

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
        if (((rt_uint32_t)arg) & RT_DEVICE_FLAG_DMA_RX)
        {
            NVIC_DisableIRQ(spi_drv->config->dma_rx->dma_irq);
        }

        if (((rt_uint32_t)arg) & RT_DEVICE_FLAG_DMA_TX)
        {
            NVIC_DisableIRQ(spi_drv->config->dma_tx->dma_irq);
        }
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
        break;

    /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        if ((((rt_uint32_t)arg) & RT_DEVICE_FLAG_INT_RX) || (((rt_uint32_t)arg) & RT_DEVICE_FLAG_INT_TX))
        {
            NVIC_EnableIRQ(spi_drv->config->irq_type);
            //__HAL_SPI_ENABLE_IT(&(spi_drv->handle), (SPI_IT_RXNE | SPI_IT_ERR | SPI_IT_TXE));
        }

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
        if (((rt_uint32_t)arg) & RT_DEVICE_FLAG_DMA_RX)
        {
            NVIC_EnableIRQ(spi_drv->config->dma_rx->dma_irq);
        }

        if (((rt_uint32_t)arg) & RT_DEVICE_FLAG_DMA_TX)
        {
            NVIC_EnableIRQ(spi_drv->config->dma_tx->dma_irq);
        }
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
        break;

    case RT_DEVICE_CTRL_CONFIG:
        if (arg == (void *) RT_DEVICE_FLAG_DMA_RX)
        {
            //sifli_dma_config(serial);
        }
        break;

    }

    return RT_EOK;
}


static const struct rt_spi_ops spi_ops =
{
    .configure = spi_configure,
    .control = spi_control,
    .xfer = spixfer,
};

/**
* @brief  Spi bus initial.
* @retval RT_EOK if success.
*/
__ROM_USED int rt_hw_spi_bus_init(struct sifli_spi *objs, struct sifli_spi_config *cfg, struct rt_spi_configuration *default_cfg, rt_size_t obj_num)
{
    rt_err_t result;
    rt_uint32_t flag = RT_DEVICE_FLAG_RDWR;

    for (int i = 0; i < obj_num; i++)
    {
        objs[i].config = &cfg[i];
        objs[i].spi_bus.parent.user_data = &cfg[i];
        objs[i].handle.Instance = cfg[i].Instance;
        objs[i].cfg = &default_cfg[i];

        if (objs[i].spi_dma_flag & SPI_USING_RX_DMA_FLAG)
        {
            /* Configure the DMA handler for Transmission process */
            objs[i].dma.handle_rx.Instance = cfg[i].dma_rx->Instance;
            objs[i].dma.handle_rx.Init.Request = cfg[i].dma_rx->request;
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            objs[i].dma.handle_rx.Init.IrqPrio = cfg[i].dma_rx->dma_irq_prio;
#endif /* DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            objs[i].dma.handle_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
            objs[i].dma.handle_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
            objs[i].dma.handle_rx.Init.MemInc              = DMA_MINC_ENABLE;
            objs[i].dma.handle_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            objs[i].dma.handle_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
            objs[i].dma.handle_rx.Init.Mode                = DMA_NORMAL;
            objs[i].dma.handle_rx.Init.Priority            = DMA_PRIORITY_HIGH;
            {
                rt_uint32_t tmpreg = 0x00U;
                //SET_BIT(RCC->AHB1ENR, cfg[i].dma_rx->dma_rcc);
                /* Delay after an RCC peripheral clock enabling */
                //tmpreg = READ_BIT(RCC->AHB1ENR, cfg[i].dma_rx->dma_rcc);
                UNUSED(tmpreg); /* To avoid compiler warnings */
            }

            flag |= RT_DEVICE_FLAG_DMA_RX;
        }

        if (objs[i].spi_dma_flag & SPI_USING_TX_DMA_FLAG)
        {
            /* Configure the DMA handler for Transmission process */
            objs[i].dma.handle_tx.Instance = cfg[i].dma_tx->Instance;
            objs[i].dma.handle_tx.Init.Request = cfg[i].dma_tx->request;
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            objs[i].dma.handle_tx.Init.IrqPrio = cfg[i].dma_tx->dma_irq_prio;
#endif /* DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            objs[i].dma.handle_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
            objs[i].dma.handle_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
            objs[i].dma.handle_tx.Init.MemInc              = DMA_MINC_ENABLE;
            objs[i].dma.handle_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            objs[i].dma.handle_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
            objs[i].dma.handle_tx.Init.Mode                = DMA_NORMAL;
            objs[i].dma.handle_tx.Init.Priority            = DMA_PRIORITY_LOW;
            {
                rt_uint32_t tmpreg = 0x00U;
                //SET_BIT(RCC->AHB1ENR, cfg[i].dma_tx->dma_rcc);
                /* Delay after an RCC peripheral clock enabling */
                //tmpreg = READ_BIT(RCC->AHB1ENR, cfg[i].dma_tx->dma_rcc);
                UNUSED(tmpreg); /* To avoid compiler warnings */
            }

            flag |= RT_DEVICE_FLAG_DMA_TX;
        }

        HAL_NVIC_SetPriority(objs[i].config->irq_type, 0, 0);
        //HAL_NVIC_EnableIRQ(objs[i].config->irq_type);
        flag |= RT_DEVICE_FLAG_INT_RX;
        flag |= RT_DEVICE_FLAG_INT_TX;

        result = rt_spi_bus_register(&objs[i].spi_bus, cfg[i].bus_name, flag, &spi_ops);
        RT_ASSERT(result == RT_EOK);

        //sifli_spi_init(&objs[i], &rt_spi_cfg_default[i]);
        objs[i].spi_sema = rt_sem_create(cfg[i].bus_name, 0, RT_IPC_FLAG_FIFO);
        RT_ASSERT(objs[i].spi_sema);

        LOG_D("%s bus init done", cfg[i].bus_name);
    }
    return result;
}

/**
  * Attach the spi device to SPI bus, this function must be used after initialization.
  */



/**
* @brief  attach spi device to spi buss.
* @param[in]  bus_name: spi bus name.
* @param[in]  device_name: spi device name.
* @retval RT_EOK if success.
*/
__ROM_USED rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name)
{
    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);

    rt_err_t result;
    struct rt_spi_device *spi_device;

    /* attach the device to spi bus*/
    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_device != RT_NULL);
    result = rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)RT_NULL);

    if (result != RT_EOK)
    {
        LOG_E("%s attach to %s faild, %d\n", device_name, bus_name, result);
    }

    RT_ASSERT(result == RT_EOK);

    LOG_D("%s attach to %s done", device_name, bus_name);

    return result;
}

#ifdef BSP_USING_SPI

static void SPIx_IRQHandler(uint32_t index)
{
    SPI_HandleTypeDef *handle;

    /* enter interrupt */
    rt_interrupt_enter();

    handle = &spi_bus_obj[index].handle;

    HAL_SPI_IRQHandler(handle);

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    if ((HAL_SPI_STATE_BUSY != handle->State) && (HAL_SPI_STATE_BUSY_TX != handle->State)
            && (HAL_SPI_STATE_BUSY_RX != handle->State) && (HAL_SPI_STATE_BUSY_TX_RX != handle->State))
    {
        rt_sem_release(spi_bus_obj[index].spi_sema);
    }
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

    /* leave interrupt */
    rt_interrupt_leave();
}

enum
{
    SPI_DMA_TX,
    SPI_DMA_RX
};

#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    struct sifli_spi *spi_drv =  rt_container_of(hspi, struct sifli_spi, handle);

    rt_sem_release(spi_drv->spi_sema);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    struct sifli_spi *spi_drv =  rt_container_of(hspi, struct sifli_spi, handle);

    rt_sem_release(spi_drv->spi_sema);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    struct sifli_spi *spi_drv =  rt_container_of(hspi, struct sifli_spi, handle);

    rt_sem_release(spi_drv->spi_sema);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    struct sifli_spi *spi_drv =  rt_container_of(hspi, struct sifli_spi, handle);

    rt_sem_release(spi_drv->spi_sema);
}
#else

static void SPIx_DMA_IRQHandler(uint32_t index, uint32_t trx)
{
    SPI_HandleTypeDef *handle;

    /* enter interrupt */
    rt_interrupt_enter();

    handle = &spi_bus_obj[index].handle;

    if (SPI_DMA_TX == trx)
    {
        HAL_DMA_IRQHandler(handle->hdmatx);
    }
    else if (SPI_DMA_RX == trx)
    {
        HAL_DMA_IRQHandler(handle->hdmarx);
    }

    if ((HAL_SPI_STATE_BUSY != handle->State) && (HAL_SPI_STATE_BUSY_TX != handle->State)
            && (HAL_SPI_STATE_BUSY_RX != handle->State) && (HAL_SPI_STATE_BUSY_TX_RX != handle->State))
    {
        if (((SPI_DMA_TX == trx) && handle->hdmatx->XferCpltCallback)
                || ((SPI_DMA_RX == trx) && handle->hdmarx->XferCpltCallback))
        {
            rt_sem_release(spi_bus_obj[index].spi_sema);
        }
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#endif


#if defined(BSP_USING_SPI1)
/**
* @brief SPI1 interrupt request function.
*/
void SPI1_IRQHandler(void)
{
    SPIx_IRQHandler(SPI1_INDEX);
}

#if defined(BSP_SPI1_RX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
/**
  * @brief  Handles DMA Rx interrupt request.
  */
void SPI1_DMA_RX_IRQHandler(void)
{
    SPIx_DMA_IRQHandler(SPI1_INDEX, SPI_DMA_RX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#if defined(BSP_SPI1_TX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
/**
  * @brief  Handles DMA Tx interrupt request.
  */
void SPI1_DMA_TX_IRQHandler(void)
{
    SPIx_DMA_IRQHandler(SPI1_INDEX, SPI_DMA_TX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif /* defined(BSP_USING_SPI1)*/

#if defined(BSP_USING_SPI2)
/**
* @brief SPI2 interrupt request function.
*/
void SPI2_IRQHandler(void)
{
    SPIx_IRQHandler(SPI2_INDEX);
}

#if defined(BSP_SPI2_RX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
/**
  * @brief  Handles DMA Rx interrupt request.
  */
void SPI2_DMA_RX_IRQHandler(void)
{
    SPIx_DMA_IRQHandler(SPI2_INDEX, SPI_DMA_RX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#if defined(BSP_SPI2_TX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
/**
  * @brief  Handles DMA Tx interrupt request.
  */
void SPI2_DMA_TX_IRQHandler(void)
{
    SPIx_DMA_IRQHandler(SPI2_INDEX, SPI_DMA_TX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif /* defined(BSP_USING_SPI2) */

#if defined(BSP_USING_SPI3)
void SPI3_IRQHandler(void)
{
    SPIx_IRQHandler(SPI3_INDEX);
}

#if defined(BSP_SPI3_RX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
/**
  * @brief  This function handles DMA Rx interrupt request.
  * @param  None
  * @retval None
  */
void SPI3_DMA_RX_IRQHandler(void)
{
    SPIx_DMA_IRQHandler(SPI3_INDEX, SPI_DMA_RX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#if defined(BSP_SPI3_TX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None
  */
void SPI3_DMA_TX_IRQHandler(void)
{
    SPIx_DMA_IRQHandler(SPI3_INDEX, SPI_DMA_TX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif /* defined(BSP_USING_SPI3) */

#if defined(BSP_USING_SPI4)
void SPI4_IRQHandler(void)
{
    SPIx_IRQHandler(SPI4_INDEX);
}

#if defined(BSP_SPI4_RX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
/**
  * @brief  This function handles DMA Rx interrupt request.
  * @param  None
  * @retval None
  */
void SPI4_DMA_RX_IRQHandler(void)
{
    SPIx_DMA_IRQHandler(SPI4_INDEX, SPI_DMA_RX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#if defined(BSP_SPI4_TX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None
  */
void SPI4_DMA_TX_IRQHandler(void)
{
    SPIx_DMA_IRQHandler(SPI4_INDEX, SPI_DMA_TX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif /* defined(BSP_USING_SPI4) */

/**
* @brief  Get SPI DMA setting.
*/
static void sifli_get_dma_info(void)
{
#ifdef BSP_SPI1_RX_USING_DMA
    spi_bus_obj[SPI1_INDEX].spi_dma_flag |= SPI_USING_RX_DMA_FLAG;
    static struct dma_config spi1_dma_rx = SPI1_RX_DMA_CONFIG;
    spi_config[SPI1_INDEX].dma_rx = &spi1_dma_rx;
#endif
#ifdef BSP_SPI1_TX_USING_DMA
    spi_bus_obj[SPI1_INDEX].spi_dma_flag |= SPI_USING_TX_DMA_FLAG;
    static struct dma_config spi1_dma_tx = SPI1_TX_DMA_CONFIG;
    spi_config[SPI1_INDEX].dma_tx = &spi1_dma_tx;
#endif

#ifdef BSP_SPI2_RX_USING_DMA
    spi_bus_obj[SPI2_INDEX].spi_dma_flag |= SPI_USING_RX_DMA_FLAG;
    static struct dma_config spi2_dma_rx = SPI2_RX_DMA_CONFIG;
    spi_config[SPI2_INDEX].dma_rx = &spi2_dma_rx;
#endif
#ifdef BSP_SPI2_TX_USING_DMA
    spi_bus_obj[SPI2_INDEX].spi_dma_flag |= SPI_USING_TX_DMA_FLAG;
    static struct dma_config spi2_dma_tx = SPI2_TX_DMA_CONFIG;
    spi_config[SPI2_INDEX].dma_tx = &spi2_dma_tx;
#endif

#ifdef BSP_SPI3_RX_USING_DMA
    spi_bus_obj[SPI3_INDEX].spi_dma_flag |= SPI_USING_RX_DMA_FLAG;
    static struct dma_config spi3_dma_rx = SPI3_RX_DMA_CONFIG;
    spi_config[SPI3_INDEX].dma_rx = &spi3_dma_rx;
#endif
#ifdef BSP_SPI3_TX_USING_DMA
    spi_bus_obj[SPI3_INDEX].spi_dma_flag |= SPI_USING_TX_DMA_FLAG;
    static struct dma_config spi3_dma_tx = SPI3_TX_DMA_CONFIG;
    spi_config[SPI3_INDEX].dma_tx = &spi3_dma_tx;
#endif

#ifdef BSP_SPI4_RX_USING_DMA
    spi_bus_obj[SPI4_INDEX].spi_dma_flag |= SPI_USING_RX_DMA_FLAG;
    static struct dma_config spi4_dma_rx = SPI4_RX_DMA_CONFIG;
    spi_config[SPI4_INDEX].dma_rx = &spi4_dma_rx;
#endif
#ifdef BSP_SPI4_TX_USING_DMA
    spi_bus_obj[SPI4_INDEX].spi_dma_flag |= SPI_USING_TX_DMA_FLAG;
    static struct dma_config spi4_dma_tx = SPI4_TX_DMA_CONFIG;
    spi_config[SPI4_INDEX].dma_tx = &spi4_dma_tx;
#endif

}

/**
* @brief  SPI hardware initial, include dma,buf configure.
* @retval RT_EOK if success.
*/


int rt_hw_spi_init(void)
{
    rt_size_t obj_num = SPI_MAX;
    int r = RT_EOK;

    if (obj_num > 0)
    {
        sifli_get_dma_info();
        r = rt_hw_spi_bus_init(spi_bus_obj, spi_config, rt_spi_cfg_default, obj_num);
    }
    return r;
}
INIT_BOARD_EXPORT(rt_hw_spi_init);

#ifdef BSP_USING_SPI_CAMERA
static uint8_t *g_camera_dma_buffer_start;
static uint8_t *g_camera_dma_buffer_half;

__weak void camera_rx_ind(uint8_t *p)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p);
}
static void camera_receive_half_dma(struct __DMA_HandleTypeDef *hdma)
{
    camera_rx_ind(g_camera_dma_buffer_start);
}

static void camera_dma_receive_callback(struct __DMA_HandleTypeDef *hdma)
{
    camera_rx_ind(g_camera_dma_buffer_half);
}
static void camera_dma_error_callback(DMA_HandleTypeDef *hdma)
{
    LOG_E("\ncamera dma err:\n");
    LOG_E("CM0AR4=0x%x\n",     hdma->DmaBaseAddress->CM0AR4);
#if 0
    rt_kprintf("hdma->Instance=0x%p\n", hdma->Instance);
    rt_kprintf("hdma->Instance.CCR=0x%x\n", hdma->Instance->CCR);
    rt_kprintf("hdma->Instance.CNDTR=0x%x\n", hdma->Instance->CNDTR);
    rt_kprintf("hdma->Instance.CPAR=0x%x\n", hdma->Instance->CPAR);
    rt_kprintf("hdma->Instance.CM0AR=0x%x\n", hdma->Instance->CM0AR);
    rt_kprintf("hdma->DmaBaseAddress: 0x%p\n", hdma->DmaBaseAddress);
    rt_kprintf("ISR=0x%x\n",        hdma->DmaBaseAddress->ISR);
    rt_kprintf("IFCR=0x%x\n",       hdma->DmaBaseAddress->IFCR);
    rt_kprintf("CCR1=0x%x\n",       hdma->DmaBaseAddress->CCR1);
    rt_kprintf("CNDTR1=0x%x\n",     hdma->DmaBaseAddress->CNDTR1);
    rt_kprintf("CPAR1=0x%x\n",      hdma->DmaBaseAddress->CPAR1);
    rt_kprintf("CM0AR1=0x%x\n",     hdma->DmaBaseAddress->CM0AR1);
    rt_kprintf("CBSR1=0x%x\n",      hdma->DmaBaseAddress->CBSR1);
    rt_kprintf("CCR2=0x%x\n",       hdma->DmaBaseAddress->CCR2);
    rt_kprintf("CNDTR2=0x%x\n",     hdma->DmaBaseAddress->CNDTR2);
    rt_kprintf("CPAR2=0x%x\n",      hdma->DmaBaseAddress->CPAR2);
    rt_kprintf("CM0AR2=0x%x\n",     hdma->DmaBaseAddress->CM0AR2);
    rt_kprintf("CBSR2=0x%x\n",      hdma->DmaBaseAddress->CBSR2);
    rt_kprintf("CCR3=0x%x\n",       hdma->DmaBaseAddress->CCR3);
    rt_kprintf("CNDTR3=0x%x\n",     hdma->DmaBaseAddress->CNDTR3);
    rt_kprintf("CPAR3=0x%x\n",      hdma->DmaBaseAddress->CPAR3);
    rt_kprintf("CM0AR3=0x%x\n",     hdma->DmaBaseAddress->CM0AR3);
    rt_kprintf("CBSR3=0x%x\n",      hdma->DmaBaseAddress->CBSR3);
    rt_kprintf("CCR4=0x%x\n",       hdma->DmaBaseAddress->CCR4);
    rt_kprintf("CNDTR4=0x%x\n",     hdma->DmaBaseAddress->CNDTR4);
    rt_kprintf("CPAR4=0x%x\n",      hdma->DmaBaseAddress->CPAR4);
    rt_kprintf("CM0AR4=0x%x\n",     hdma->DmaBaseAddress->CM0AR4);
    rt_kprintf("CBSR4=0x%x\n",      hdma->DmaBaseAddress->CBSR4);
    rt_kprintf("CCR5=0x%x\n",       hdma->DmaBaseAddress->CCR5);
    rt_kprintf("CNDTR5=0x%x\n",     hdma->DmaBaseAddress->CNDTR5);
    rt_kprintf("CPAR5=0x%x\n",      hdma->DmaBaseAddress->CPAR5);
    rt_kprintf("CM0AR5=0x%x\n",     hdma->DmaBaseAddress->CM0AR5);
    rt_kprintf("CBSR5=0x%x\n",      hdma->DmaBaseAddress->CBSR5);
    rt_kprintf("CCR6=0x%x\n",       hdma->DmaBaseAddress->CCR6);
    rt_kprintf("CNDTR6=0x%x\n",     hdma->DmaBaseAddress->CNDTR6);
    rt_kprintf("CPAR6=0x%x\n",      hdma->DmaBaseAddress->CPAR6);
    rt_kprintf("CM0AR6=0x%x\n",     hdma->DmaBaseAddress->CM0AR6);
    rt_kprintf("CBSR6=0x%x\n",      hdma->DmaBaseAddress->CBSR6);
    rt_kprintf("CCR7=0x%x\n",       hdma->DmaBaseAddress->CCR7);
    rt_kprintf("CNDTR7=0x%x\n",     hdma->DmaBaseAddress->CNDTR7);
    rt_kprintf("CPAR7=0x%x\n",      hdma->DmaBaseAddress->CPAR7);
    rt_kprintf("CM0AR7=0x%x\n",     hdma->DmaBaseAddress->CM0AR7);
    rt_kprintf("CBSR7=0x%x\n",      hdma->DmaBaseAddress->CBSR7);
    rt_kprintf("CCR8=0x%x\n",       hdma->DmaBaseAddress->CCR8);
    rt_kprintf("CNDTR8=0x%x\n",     hdma->DmaBaseAddress->CNDTR8);
    rt_kprintf("CPAR8=0x%x\n",      hdma->DmaBaseAddress->CPAR8);
    rt_kprintf("CM0AR8=0x%x\n",     hdma->DmaBaseAddress->CM0AR8);
    rt_kprintf("CBSR8=0x%x\n",      hdma->DmaBaseAddress->CBSR8);
    rt_kprintf("CSELR1=0x%x\n",     hdma->DmaBaseAddress->CSELR1);
    rt_kprintf("CSELR2=0x%x\n",     hdma->DmaBaseAddress->CSELR2);
    rt_kprintf("DBGSEL=0x%x\n",     hdma->DmaBaseAddress->DBGSEL);
#endif
}

void camera_stop_dma(SPI_HandleTypeDef *hspi)
{
    if (hspi)
    {
        HAL_SPI_DMAStop(hspi);
        /* Process Unlocked */
        __HAL_UNLOCK(hspi);
    }
}

//size must be power of 2
int camera_start_dma(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint32_t size)
{
    rt_kprintf("dma buffer=0x%x\n", buffer);

    /*
        set CS pilldown
        HAL_PIN_Set(PAD_PB13, SPI3_CLK, PIN_PULLDOWN, 0);             // SPI3 (GSensor)
        HAL_PIN_Set(PAD_PB13, SPI3_CLK, PIN_PULLDOWN, 0);             // SPI3 (GSensor)
    */
    g_camera_dma_buffer_start = buffer;
    g_camera_dma_buffer_half = buffer + size / 2;

    HAL_ASSERT(g_camera_dma_buffer_start);
    HAL_ASSERT((size > 128) && ((size & 1) == 0));

    /* check rx dma handle */
    HAL_ASSERT(IS_SPI_DMA_HANDLE(hspi->hdmarx));

    /* Start DMA transfer */

    /* Process Locked */
    __HAL_LOCK(hspi);

    rt_kprintf("camera dma lock ok\n");

    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

    /*Init field not used in handle to zero */
    hspi->RxISR       = NULL;
    hspi->TxISR       = NULL;
    hspi->TxXferSize  = 0U;
    hspi->TxXferCount = 0U;
    hspi->pRxBuffPtr  = (uint8_t *)buffer;
    hspi->RxXferSize  = size;
    hspi->RxXferCount = size;
    /* Set the SPI RxDMA Half transfer complete callback */
    hspi->hdmarx->XferHalfCpltCallback = camera_receive_half_dma;

    /* Set the SPI Rx DMA transfer complete callback */
    hspi->hdmarx->XferCpltCallback = camera_dma_receive_callback;

    /* Set the DMA error callback */
    hspi->hdmarx->XferErrorCallback = camera_dma_error_callback;

    /* Set the DMA AbortCpltCallback */
    hspi->hdmarx->XferAbortCallback = NULL;

    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE));
    hspi->Instance->FIFO_CTRL |= SPI_FIFO_CTRL_RSRE;
    hspi->Instance->FIFO_CTRL &= ~SPI_FIFO_CTRL_TSRE;

    /* Enable the Rx DMA Stream/Channel  */
    HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DATA, (uint32_t)buffer, size);

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

    /* Enable the SPI Error Interrupt Bit */
    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_ERR));

    return 0;
}

#endif

//#define DRV_SPI_TEST
#ifdef DRV_SPI_TEST
// test only when 2 spi enabled
#ifdef BSP_USING_SPI1
#ifdef BSP_USING_SPI2

#define SPI_SEND_LEN        (8)
#define SPI_BIT_WIDTH       (8)

#include "string.h"
rt_device_t g_spi_bus = NULL;
rt_device_t g_spi_bus2 = NULL;
static rt_thread_t tid1 = RT_NULL;
static rt_thread_t tid2 = RT_NULL;
static int th_flag1 = 0;
static int th_flag2 = 0;
static char outbuff[SPI_SEND_LEN];

struct rt_spi_device *g_mspi = NULL;
struct rt_spi_device *g_sspi = NULL;

static int rt_spi_mdev_init(void)
{
    struct rt_spi_device *pdev = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));

    return rt_spi_bus_attach_device(pdev, "spi_m", "spi1", NULL);
}
INIT_COMPONENT_EXPORT(rt_spi_mdev_init);

static int rt_spi_sdev_init(void)
{
    struct rt_spi_device *pdev = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device)) ;

    return rt_spi_bus_attach_device(pdev, "spi_s", "spi2", NULL);;
}
INIT_COMPONENT_EXPORT(rt_spi_sdev_init);

static void thread1_entry(void *parameter)
{
#if (SPI_BIT_WIDTH == 8)
    char *buf = parameter;
#else
    short *buf = parameter;
#endif
    int cnt;
    char rev;
    int i;
    th_flag1 = 0;
    rt_thread_delay(10);
    LOG_I("thread spi1(master) entry");

    i = 0;
    //while(1)
    {
        cnt = rt_spi_send(g_mspi, &buf[i], SPI_SEND_LEN);
        //cnt = rt_spi_recv(g_mspi, &buf[i],1);
        if (cnt != 0)
        {
            LOG_I("spi1 send %d", buf[i]);
#if 0
            rt_thread_delay(5);
            if (cnt != 0)
            {
                cnt = rt_spi_recv(g_mspi, &rev, 1);
                if (cnt != 0)
                {
                    outbuff[i] = rev;
                    LOG_I("spi1 recv %d", rev);
                }
            }
#endif
        }
        else
        {
            LOG_I("spi1 send fail");
        }
        rt_thread_delay(1);
        //i++;
        //if(i>=SPI_SEND_LEN)
        //    break;
    }
    th_flag1 = 1;
    LOG_I("thread spi1 exit");
}

static void thread2_entry(void *parameter)
{
#if (SPI_BIT_WIDTH == 8)
    char *buf = parameter;
#else
    short *buf = parameter;
#endif
    char rev;
    int cnt;
    int i;

    th_flag2 = 0;
    rt_thread_delay(8);
    LOG_I("thread spi2(slave) entry");

    i = 0;
    //while (1)
    {
        cnt = rt_spi_recv(g_sspi, &buf[i], SPI_SEND_LEN);
        //cnt = rt_spi_send(g_sspi, &buf[i],1);
        if (cnt != 0)
        {
            LOG_I("spi2 recv %d", buf[i]);
#if 0
            rev = buf[i] + 1;
            rt_thread_delay(5);
            cnt = rt_spi_send(g_sspi, &rev, 1);
            if (cnt != 0)
                LOG_I("spi2 send %d", rev);
            else
                LOG_E("spi2 send fail");
#endif
        }
        else
        {
            LOG_I("spi2 recv fail");
        }
        //i++;
        //if(i>=SPI_SEND_LEN)
        //    break;
        rt_thread_delay(1);
    }
    th_flag2 = 1;
    LOG_I("thread spi2 exit");
}


int cmd_spitest(int argc, char *argv[])
{
    struct rt_spi_configuration cfg1;
    struct rt_spi_configuration cfg2;
    struct rt_spi_message msg1;
    struct rt_spi_message msg2;
#if (SPI_BIT_WIDTH == 8)
    char sendbuf[SPI_SEND_LEN];
    char recvbuf[SPI_SEND_LEN];
#else
    short sendbuf[SPI_SEND_LEN];
    short recvbuf[SPI_SEND_LEN];
#endif
    int i;

    if (1)
    {
        if (g_spi_bus == NULL)
        {
            g_spi_bus = rt_device_find("spi1");
            if (g_spi_bus)
            {
                rt_device_open(g_spi_bus, RT_DEVICE_FLAG_RDWR);
                LOG_I("spi bus find");
            }
            g_spi_bus2 = rt_device_find("spi2");
            if (g_spi_bus2)
            {
                rt_device_open(g_spi_bus2, RT_DEVICE_FLAG_RDWR);
                LOG_I("spi 2 bus find");
            }
        }
        if (g_spi_bus && g_spi_bus2)     // spi1 as master, spi2 as slave
        {
            LOG_I("create spi device");
            g_mspi = (struct rt_spi_device *)rt_device_find("spi_m");
            g_sspi = (struct rt_spi_device *)rt_device_find("spi_s");
            if (g_sspi && g_mspi)
            {
                th_flag1 = 0;
                th_flag2 = 0;
                LOG_I("dev: %s find, dev: %s find", g_mspi->parent.parent.name, g_sspi->parent.parent.name);
                rt_spi_take_bus(g_mspi);
                //rt_spi_take(g_mspi);
                cfg1.data_width = SPI_BIT_WIDTH;
                cfg1.max_hz = 50 * 1000 * 1000; // 50m
                cfg1.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_MASTER;
                rt_spi_configure(g_mspi, &cfg1);

                rt_spi_take_bus(g_sspi);
                //rt_spi_take(g_sspi);
                cfg2.data_width = SPI_BIT_WIDTH;
                cfg2.max_hz = 50 * 1000 * 1000; // 50m
                cfg2.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_SLAVE;
                rt_spi_configure(g_sspi, &cfg2);
                LOG_I("configure spi1 as master and spi2 as slave");

                // begin data trasfer
                //init buffer
                for (i = 0; i < SPI_SEND_LEN; i++)
                    sendbuf[i] = i * 2 + 1;
                for (i = 0; i < SPI_SEND_LEN; i++)
                    recvbuf[i] = 0xf0;
                for (i = 0; i < SPI_SEND_LEN; i++)
                    outbuff[i] = 0;
#if 1
                tid1 = rt_thread_create("t1",
                                        thread1_entry, sendbuf,
                                        1024, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
                if (tid1 != RT_NULL)
                    rt_thread_startup(tid1);
                else
                {
                    LOG_E("create thread 1 fail");
                    return 0;
                }

                tid2 = rt_thread_create("t2",
                                        thread2_entry, recvbuf,
                                        1024, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
                if (tid2 != RT_NULL)
                    rt_thread_startup(tid2);
                else
                {
                    LOG_E("create thread 2 fail");
                    return 0;
                }

                while (!(th_flag1 && th_flag2))
                {
                    rt_thread_delay(100);
                }

#else

                rt_size_t res = rt_spi_send(g_mspi, sendbuf, SPI_SEND_LEN);
                LOG_I("send res = %d", res);

                res = rt_spi_recv(g_sspi, recvbuf, SPI_SEND_LEN);
                LOG_I("receive res = %d", res);

                msg1.send_buf = &sendbuf[0];
                msg1.recv_buf = NULL;
                msg1.length = SPI_SEND_LEN;
                msg1.next = NULL;
                msg1.cs_take = 1;
                msg1.cs_release = 1;
                rt_spi_transfer_message(g_mspi, &msg1);
                LOG_I("send data");

                msg2.send_buf = NULL;
                msg2.recv_buf = &recvbuf[0];
                msg2.length = SPI_SEND_LEN;
                msg2.next = NULL;
                msg2.cs_take = 1;
                msg2.cs_release = 1;
                rt_spi_transfer_message(g_sspi, &msg2);
                LOG_I("receive data");
#endif
                for (i = 0; i < SPI_SEND_LEN; i++)
                {
                    LOG_I("s %d, r %d, out %d", sendbuf[i], recvbuf[i], outbuff[i]);
                    if (sendbuf[i] != recvbuf[i])
                        break;
                }
                if (i == SPI_SEND_LEN)
                    LOG_I("test pass!");
                else
                    LOG_E("test fail");
            }
            else
            {
                LOG_E("some device not found");
            }
        }
        else
        {

            LOG_E("bus not found");
        }

    }
    else
    {
        LOG_E("invalid parameter");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_spitest, __cmd_spitest, Test hw spi);

#endif
#endif
#endif  //DRV_SPI_TEST


//#endif /* BSP_USING_SPI1 || BSP_USING_SPI2 || BSP_USING_SPI3 || BSP_USING_SPI4 || BSP_USING_SPI5 */
#endif /* RT_USING_SPI */

/// @} drv_spi
/// @} bsp_driver
/// @} file

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
