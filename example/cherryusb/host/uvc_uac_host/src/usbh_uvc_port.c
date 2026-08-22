/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbh_uvc_stream.h"
#include <stdint.h>
#include <string.h>
#include "bf0_hal.h"
#include "bf0_hal_dma.h"

#ifndef DMA_REQUEST_MEM2MEM
    #define DMA_REQUEST_MEM2MEM DMA_REQUEST_0
#endif

typedef struct
{
    uint32_t src;
    uint32_t dst;
    uint32_t nbytes;
    uint32_t counts;
    uint32_t p_align;
    uint32_t m_align;
} usbh_video_dma_desc_t;

static DMA_HandleTypeDef s_uvc_dma;
static usbh_video_dma_desc_t s_uvc_dma_desc[VIDEO_ISO_PACKETS];
static volatile uint8_t s_uvc_dma_busy;
static uint8_t s_uvc_dma_desc_num;
static uint8_t s_uvc_dma_inited;

static void usbh_video_dma_desc_calc(uint32_t src_addr, uint32_t dst_addr, uint32_t nbytes,
                                     uint32_t *counts, uint32_t *p_align, uint32_t *m_align)
{
    if (((src_addr | dst_addr | nbytes) & 0x3U) == 0U)
    {
        *counts = nbytes >> 2;
        *p_align = DMA_PDATAALIGN_WORD;
        *m_align = DMA_MDATAALIGN_WORD;
    }
    else if (((src_addr | dst_addr | nbytes) & 0x1U) == 0U)
    {
        *counts = nbytes >> 1;
        *p_align = DMA_PDATAALIGN_HALFWORD;
        *m_align = DMA_MDATAALIGN_HALFWORD;
    }
    else
    {
        *counts = nbytes;
        *p_align = DMA_PDATAALIGN_BYTE;
        *m_align = DMA_MDATAALIGN_BYTE;
    }
}

volatile uint32_t g_uvc_fps = 0;

ATTR_FAST_RAM_SECTION void usbh_video_transfer_abort_callback(void)
{
    g_uvc_fps = 0;
}

ATTR_FAST_RAM_SECTION void usbh_video_fps_record(void)
{
    static uint32_t time_last;
    static uint16_t fps_cnt;

    fps_cnt++;
    if (fps_cnt >= 10)
    {
        uint32_t time = rt_tick_get();
        g_uvc_fps = (1000 * 10) / (uint32_t)(time - time_last);
        time_last = time;
        fps_cnt = 0;
    }
}

static void usbh_vide_fps_thread(void *argument)
{
    while (1)
    {
        USB_LOG_INFO("fps:%d\r\n", g_uvc_fps);
        USB_LOG_INFO("vc:%d\r\n", video_complete_count);
        usb_osal_msleep(5000);
    }
}

void usbh_video_fps_init(void)
{
    usb_osal_thread_create("usbh_video", 1024, 5, usbh_vide_fps_thread, NULL);
}

void usbh_video_dma_init(void)
{
    memset(&s_uvc_dma, 0, sizeof(s_uvc_dma));

    s_uvc_dma.Instance = DMA1_Channel1;
    s_uvc_dma.Init.Request = DMA_REQUEST_MEM2MEM;
    s_uvc_dma.Init.Direction = DMA_MEMORY_TO_MEMORY;
    s_uvc_dma.Init.PeriphInc = DMA_PINC_ENABLE;
    s_uvc_dma.Init.MemInc = DMA_MINC_ENABLE;
    s_uvc_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    s_uvc_dma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    s_uvc_dma.Init.Mode = DMA_NORMAL;
    s_uvc_dma.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&s_uvc_dma) != HAL_OK)
    {
        USB_LOG_ERR("uvc dma init failed\r\n");
        s_uvc_dma_inited = 0;
        return;
    }

    s_uvc_dma_desc_num = 0;
    s_uvc_dma_busy = 0;
    s_uvc_dma_inited = 1;
}

ATTR_FAST_RAM_SECTION void usbh_video_dma_lli_fill(uint32_t desc_index, uint32_t src_addr, uint32_t dst_addr, uint32_t nbytes)
{
    usbh_video_dma_desc_t *desc;

    if ((desc_index >= VIDEO_ISO_PACKETS) || (nbytes == 0U))
    {
        return;
    }

    desc = &s_uvc_dma_desc[desc_index];
    desc->src = src_addr;
    desc->dst = dst_addr;
    desc->nbytes = nbytes;
    usbh_video_dma_desc_calc(src_addr, dst_addr, nbytes, &desc->counts, &desc->p_align, &desc->m_align);

    if (s_uvc_dma_desc_num < (desc_index + 1U))
    {
        s_uvc_dma_desc_num = desc_index + 1U;
    }
}

ATTR_FAST_RAM_SECTION void usbh_video_dma_start(void)
{
    HAL_StatusTypeDef status;

    if (!s_uvc_dma_inited)
    {
        usbh_video_dma_init();
        if (!s_uvc_dma_inited)
        {
            return;
        }
    }

    if (s_uvc_dma_desc_num == 0U)
    {
        return;
    }

    s_uvc_dma_busy = 1;

    for (uint8_t i = 0; i < s_uvc_dma_desc_num; i++)
    {
        usbh_video_dma_desc_t *desc = &s_uvc_dma_desc[i];

        s_uvc_dma.Init.PeriphDataAlignment = desc->p_align;
        s_uvc_dma.Init.MemDataAlignment = desc->m_align;
        if (HAL_DMA_Init(&s_uvc_dma) != HAL_OK)
        {
            USB_LOG_ERR("uvc dma reinit failed\r\n");
            break;
        }

        status = HAL_DMA_Start(&s_uvc_dma, desc->src, desc->dst, desc->counts);
        if (status != HAL_OK)
        {
            USB_LOG_ERR("uvc dma start failed\r\n");
            break;
        }

        status = HAL_DMA_PollForTransfer(&s_uvc_dma, HAL_DMA_FULL_TRANSFER, 1000);
        if (status != HAL_OK)
        {
            USB_LOG_ERR("uvc dma wait timeout\r\n");
            break;
        }

        mpu_dcache_invalidate((void *)desc->dst, desc->nbytes);
    }

    s_uvc_dma_desc_num = 0;
    s_uvc_dma_busy = 0;

}

ATTR_FAST_RAM_SECTION void usbh_video_dma_stop(void)
{
    if (s_uvc_dma_inited)
    {
        (void)HAL_DMA_Abort(&s_uvc_dma);
    }

    s_uvc_dma_desc_num = 0;
    s_uvc_dma_busy = 0;
}

ATTR_FAST_RAM_SECTION bool usbh_video_dma_isbusy(void)
{
    return (s_uvc_dma_busy != 0U);
}
