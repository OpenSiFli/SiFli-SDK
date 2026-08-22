/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include <drv_config.h>

#if defined(BSP_USING_DCMI)

#include <drv_dcmi.h>

#define DRV_DEBUG
#define LOG_TAG             "drv.dcmi"
#include <drv_log.h>

static struct rt_dcmi_device rt_dcmi;

const struct dma_config dcmi_config = BF0_DCMI_DMA_CONFIG;

struct rt_dcmi_device *rt_get_dcmi_device(void)
{
    return &rt_dcmi;
}
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
static void DCMI_DMAXferCpltHalf(DMA_HandleTypeDef *hdma)
{
    if (rt_dcmi.parent.rx_indicate)
        rt_dcmi.parent.rx_indicate((rt_device_t)&rt_dcmi, rt_dcmi.buffer_size / 2);
}

static void DCMI_DMAXferCplt(DMA_HandleTypeDef *hdma)
{
    if (rt_dcmi.parent.rx_indicate)
        rt_dcmi.parent.rx_indicate((rt_device_t)&rt_dcmi, rt_dcmi.buffer_size);
}

static void DCMI_DMAXferAbort(DMA_HandleTypeDef *hdma)
{
    LOG_E("dcmi dma abort");
    if (rt_dcmi.parent.rx_indicate)
        rt_dcmi.parent.rx_indicate((rt_device_t)&rt_dcmi, 0);
}

static void DCMI_DMAXferError(DMA_HandleTypeDef *hdma)
{
    LOG_E("dcmi dma error");
    if (rt_dcmi.parent.rx_indicate)
        rt_dcmi.parent.rx_indicate((rt_device_t)&rt_dcmi, 0);
}
#else /*DMA_SUPPORT_DYN_CHANNEL_ALLOC*/

void DCMI_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if (__HAL_DMA_GET_FLAG(&rt_dcmi.hdma, DCMI_DMA_TC) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(&rt_dcmi.hdma, DCMI_DMA_TC);
        if (rt_dcmi.parent.rx_indicate)
            rt_dcmi.parent.rx_indicate((rt_device_t)&rt_dcmi, rt_dcmi.buffer_size);
    }
    else if (__HAL_DMA_GET_FLAG(&rt_dcmi.hdma, DCMI_DMA_HT) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(&rt_dcmi.hdma, DCMI_DMA_HT);
        if (rt_dcmi.parent.rx_indicate)
            rt_dcmi.parent.rx_indicate((rt_device_t)&rt_dcmi, rt_dcmi.buffer_size / 2);
    }
    else
    {
        // Check DMA interrupt flag
        RT_ASSERT(0);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#ifdef USE_PTM_DVP_8WIRE


void start_dvp_ptm(void)
{
    uint32_t core_code[] =
    {
        PTM_SET(PTM_Y, PTM_ZERO, OP_ADD, 0), /* clear Y */
        PTM_WAIT(WAIT_IN10, 1, 0), /* wait vsync neg edge */
        PTM_WAIT(WAIT_IN10, 0, 0),
        PTM_WAIT(WAIT_IN8, 0, 0), /* wait clk pos edge */
        PTM_WAIT(WAIT_IN8, 1, 0),
        PTM_JMP(4, PTM_IN10, JMP_EQCH, PTM_ALL1, 0), /* if vsync high, finish current frame */
        PTM_JMP(-3, PTM_IN9, JMP_EQCH, PTM_ZERO, 0), /* if hsync low, wait for next line */
        PTM_IO(IO_PULL8, IO_NOP, IO_NOP, 0), /* sample 8 bits */
        PTM_JMP(-5, PTM_ZERO, JMP_EQ, PTM_ZERO, 0), /* to next clk */
        PTM_SET(PTM_X, PTM_X, OP_ADD, 1),
    };
    //Disable USB output
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN);

    HAL_RCC_EnableModule(RCC_MOD_PTM1);
    memcpy(PTM1_CORE1_TCM, core_code, sizeof(core_code));

    hwp_ptm1->CER |= PTM_CER_RST1;
    hwp_ptm1->CCR1 = ((sizeof(core_code) / 4 - 1) << PTM_CCR1_END0_Pos) |
                     (0  << PTM_CCR1_PSC_Pos)  |
                     (255 << PTM_CCR1_REP_Pos); //repeat 1
    hwp_ptm1->ICR1 = (0 << PTM_ICR1_IBLOCK_Pos) | //no block when IFIFO full
                     (1 << PTM_ICR1_IRSH_Pos)   | //right shift
                     (1 << PTM_ICR1_IAUTO_Pos)  | //enable IFIFO auto push from Y
                     (1 << PTM_ICR1_IFIFO_Pos)  | //enable IFIFO
                     (0 << PTM_ICR1_IBASE_Pos);   //input CHx offset

    hwp_ptm1->CER |= PTM_CER_EN1;
}

void stop_dvp_ptm(void)
{
    hwp_ptm1->CER |= PTM_CER_RST1;
}
#endif
static void rt_hw_dcmi_dma_init(rt_device_t dev)
{
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;

    //TODO:
    // HAL_RCC_EnableModule(dcmi_config.dma_rcc);

    dcmi->hdma.Instance                 = dcmi_config.Instance;
    dcmi->hdma.Init.Request             = dcmi_config.request;
    dcmi->hdma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    dcmi->hdma.Init.PeriphInc           = DMA_PINC_ENABLE;
    dcmi->hdma.Init.MemInc              = DMA_MINC_ENABLE;
    dcmi->hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dcmi->hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    dcmi->hdma.Init.Priority            = DMA_PRIORITY_HIGH;
    dcmi->hdma.Init.Mode                = DMA_CIRCULAR;//DMA_CIRCULAR;DMA_NORMAL
    dcmi->hdma.Init.BurstSize           = 0x3; //burst 16 words
#ifdef USE_PTM_DVP_8WIRE
    dcmi->hdma.Init.Request             = DMA_REQUEST_73;
    dcmi->hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    dcmi->hdma.Init.Priority            = DMA_PRIORITY_HIGH;
    dcmi->hdma.Init.BurstSize   = 0;
#endif

    HAL_DMA_Init(&dcmi->hdma);
    __HAL_LINKDMA(&dcmi->handle, DMA_Handle, dcmi->hdma);
    HAL_NVIC_SetPriority(dcmi_config.dma_irq, 0x00, 0x00);
    HAL_NVIC_EnableIRQ(dcmi_config.dma_irq);
}

void rt_hw_dcmi_dma_start(rt_device_t dev)
{
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;
    RT_ASSERT(dcmi->frame_buffer != RT_NULL);

    __HAL_UNLOCK(&dcmi->handle);

#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    dcmi->hdma.XferHalfCpltCallback    = DCMI_DMAXferCpltHalf;
    dcmi->hdma.XferCpltCallback        = DCMI_DMAXferCplt;
    dcmi->hdma.XferAbortCallback       = DCMI_DMAXferAbort;
    dcmi->hdma.XferErrorCallback       = DCMI_DMAXferError;
#endif /* DMA_SUPPORT_DYN_CHANNEL_ALLOC */



    //__HAL_DCMI_DMA_NUM(&dcmi->handle, 0x10);//carry words num of one dma request
#ifndef USE_PTM_DVP_8WIRE
    HAL_StatusTypeDef status = HAL_DMA_Start_IT(&dcmi->hdma, (rt_uint32_t)&dcmi->handle.Instance->DR, (rt_uint32_t)dcmi->frame_buffer, (rt_uint32_t)(dcmi->buffer_size >> 2));
#else
    HAL_StatusTypeDef status = HAL_DMA_Start_IT(&dcmi->hdma, (rt_uint32_t)&hwp_ptm1->IFIFO1, (rt_uint32_t)dcmi->frame_buffer, (rt_uint32_t)(dcmi->buffer_size >> 2));
#endif /* USE_PTM_DVP_8WIRE */
    RT_ASSERT(status == HAL_OK);
}

void rt_hw_dcmi_spi_config(rt_device_t dev, struct dcmi_spi_cfg *spi_cfg)
{
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;
    RT_ASSERT(spi_cfg != RT_NULL);
    RT_ASSERT(spi_cfg->pkt_size <= 0xFFFFUL);

    __HAL_DCMI_SPI_PACKET_SIZE(&dcmi->handle, spi_cfg->pkt_size);
    __HAL_DCMI_SPI_DATA_LINE(&dcmi->handle, spi_cfg->spi_dataline); //1line 2line 4line
    __HAL_DCMI_SPI_IS_MTKMODE(&dcmi->handle, 1); //is mtk mode
    __HAL_DCMI_SPI_PACKTID(&dcmi->handle, spi_cfg->pkt_id);
    __HAL_DCMI_SPI_CLKEG(&dcmi->handle, spi_cfg->spi_clkeg); //0: spi clk posedge; 1: spi clk negedge
    __HAL_DCMI_SPI_SYNC(&dcmi->handle, spi_cfg->spi_sync);
}

static rt_err_t rt_hw_dcmi_init(rt_device_t dev)
{
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;
    RT_ASSERT(dev != RT_NULL);

    dcmi->handle.Instance               = hwp_dcmi;
    dcmi->handle.Init.SynchroMode       = DCMI_SYNCHRO_HARDWARE;
    dcmi->handle.Init.PCKPolarity       = DCMI_PCKPOLARITY_RISING;
    dcmi->handle.Init.VSPolarity        = DCMI_VSPOLARITY_LOW;
    dcmi->handle.Init.HSPolarity        = DCMI_HSPOLARITY_HIGH;
    dcmi->handle.Init.ExtendedDataMode  = DCMI_EXTEND_DATA_8B;

    if (HAL_DCMI_Init(&(dcmi->handle)) != HAL_OK)
    {
        LOG_E("dcmi init error!");
        return RT_ERROR;
    }
    dcmi->dcmi_irq = DCMI_IRQn;
    __HAL_DCMI_TRS_IF(&dcmi->handle, DCMI_IF_DVP); //0: DVP interface; 1:spi interface
    __HAL_DCMI_CAP_MODE(&dcmi->handle, DCMI_CPT_CON); //0: continous mode 1: single shot snap
    __HAL_DCMI_ENABLE_IT(&dcmi->handle, DCMI_IE_FRAME_IE | DCMI_IE_OVR_IE | DCMI_IE_FSM_ERR_IE | DCMI_IE_PACK_SIZE_ERR_IE | DCMI_IE_VSYNC_IE);
    __HAL_DCMI_DISABLE_IT(&dcmi->handle, DCMI_IE_LINE_IE);
    HAL_NVIC_EnableIRQ(dcmi->dcmi_irq);
    __HAL_DCMI_ENABLE(&dcmi->handle);

    return RT_EOK;
}

int rt_hw_dcmi_get_error(rt_device_t dev)
{
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;
    RT_ASSERT(dev != RT_NULL);
    return dcmi->error;
}

void DCMI_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DCMI_IRQHandler(&rt_dcmi.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    if (__HAL_DCMI_GET_FLAG(hdcmi, DCMI_MIS_INDEX | DCMI_IE_VSYNC_IE) != RESET)
    {
        __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_ISC_VSYNC_ISC);
        rt_dcmi.error = DCMI_IE_VSYNC_IE;
    }
}

/* Capture a frame of the image */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    if (rt_dcmi.parent.rx_indicate)
    {
        if (__HAL_DCMI_GET_FLAG(hdcmi, DCMI_MIS_INDEX | DCMI_IE_PACK_SIZE_ERR_IE) != RESET)
        {
            __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_ISC_PACK_SIZE_ERR_ISC);
            rt_dcmi.error = DCMI_ERROR_PACK_SIZE;
        }
        if (__HAL_DCMI_GET_FLAG(hdcmi, DCMI_MIS_INDEX | DCMI_IE_FSM_ERR_IE) != RESET)
        {
            __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_ISC_FSM_ERR_ISC);
            rt_dcmi.error = DCMI_IE_FSM_ERR_IE;
        }
        if (__HAL_DCMI_GET_FLAG(hdcmi, DCMI_MIS_INDEX | DCMI_IE_OVR_IE) != RESET)
        {
            __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_ISC_OVR_ISC);
            rt_dcmi.error = DCMI_IE_OVR_IE;
        }
        if (__HAL_DCMI_GET_FLAG(hdcmi, DCMI_MIS_INDEX | DCMI_IE_FRAME_IE) != RESET)
        {
            __HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_ISC_FRAME_ISC);
            rt_dcmi.error = 0;
        }
        rt_dcmi.parent.rx_indicate((rt_device_t)&rt_dcmi, 0);
    }
    __HAL_DCMI_ENABLE_IT(&rt_dcmi.handle, DCMI_IE_FRAME_IE | DCMI_IE_OVR_IE | DCMI_IE_FSM_ERR_IE | DCMI_IE_PACK_SIZE_ERR_IE);
}



static rt_err_t rt_dcmi_init(rt_device_t dev)
{
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;

    RT_ASSERT(dev != RT_NULL);
    rt_err_t result = RT_EOK;

    result = rt_hw_dcmi_init(dev);
    if (result != RT_EOK)
    {
        LOG_E("dcmi init failed");
        return result;
    }

    rt_hw_dcmi_dma_init(dev);

    return result;
}

static rt_err_t rt_dcmi_open(rt_device_t dev, rt_uint16_t oflag)
{
    RT_ASSERT(dev != RT_NULL);

    return RT_EOK;
}

static rt_err_t rt_dcmi_close(rt_device_t dev)
{
    RT_ASSERT(dev != RT_NULL);

    return RT_EOK;
}

static rt_err_t rt_dcmi_yuv2rgb_config(struct rt_dcmi_device *dcmi, struct rt_dcmi_config *cfg)
{
    uint32_t cr1 = 0, cr2 = 0;
    struct dcmi_yuv2rgb_cfg *yuv2rgb = &cfg->yuv2rgb;

    if (cfg->yuv2rgb_en == DCMI_YUV2RGB_DFT_EN)
    {
        __HAL_DCMI_YUV2RGB_ENABLE(&dcmi->handle);
        __HAL_DCMI_RANGE_SEL(&dcmi->handle, 1);
    }
    else if (cfg->yuv2rgb_en == DCMI_YUV2RGB_UPD_EN)
    {

        cr1 = (yuv2rgb->fub | (yuv2rgb->fug << 10) | (yuv2rgb->fy << 10));
        cr2 = (yuv2rgb->fvg | (yuv2rgb->fvr << 10));
        __HAL_DCMI_YUV2RGB_ENABLE(&dcmi->handle);
        __HAL_DCMI_RANGE_SEL(&dcmi->handle, 1);
        __HAL_DCMI_YUV2RGB_PARAMS(&dcmi->handle, cr1, cr2);
    }
    else
    {
        __HAL_DCMI_YUV2RGB_DISABLE(&dcmi->handle);
    }

    return RT_EOK;
}
static rt_err_t rt_dcmi_config(rt_device_t dev, struct rt_dcmi_config *cfg)
{
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;
    RT_ASSERT(cfg != RT_NULL);

    switch (cfg->dcmi_cfg)
    {
    case DCMI_IF_CFG:
        __HAL_DCMI_TRS_IF(&dcmi->handle, cfg->dcmi_if);
        break;
    case DCMI_SPI_CFG:
        rt_hw_dcmi_spi_config(dev, &cfg->dcmi_spi);
        break;
    case DCMI_CM_CFG:
        __HAL_DCMI_CAP_MODE(&dcmi->handle, cfg->dcmi_cm);
        break;
    case DCMI_YUV2RGB_CFG:
        rt_dcmi_yuv2rgb_config(dcmi, cfg);
        break;
    case DCMI_FB_CFG:
        dcmi->frame_buffer = cfg->frame_buffer;
        dcmi->buffer_size = cfg->buffer_size;
        break;
    case DCMI_ALL_CFG:
    {
        __HAL_DCMI_TRS_IF(&dcmi->handle, cfg->dcmi_if);
        if (cfg->dcmi_if == DCMI_IF_SPI)
        {
            rt_hw_dcmi_spi_config(dev, &cfg->dcmi_spi);
        }
        __HAL_DCMI_CAP_MODE(&dcmi->handle, cfg->dcmi_cm);
        rt_dcmi_yuv2rgb_config(dcmi, cfg);
        dcmi->frame_buffer = cfg->frame_buffer;
        dcmi->buffer_size = cfg->buffer_size;
    }
    break;
    default:
        break;
    }

    return RT_EOK;
}

static rt_err_t rt_dcmi_start(rt_device_t dev)
{
    LOG_I("rt_dcmi_start");
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;
    RT_ASSERT(dev != RT_NULL);

#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif  /* RT_USING_PM */

#ifdef USE_PTM_DVP_8WIRE
    rt_hw_dcmi_dma_start(dev);
    start_dvp_ptm();
#else
    dcmi->handle.Instance->CR |= (DCMI_CR_RSTB_Msk);
    while ((dcmi->handle.Instance->CR & DCMI_CR_RSTB_S_Msk) == 0);
    rt_hw_dcmi_dma_start(dev);
    __HAL_DCMI_CAPTURE_START(&dcmi->handle);
#endif
    LOG_I("rt_dcmi_start done");
    return RT_EOK;
}

static rt_err_t rt_dcmi_stop(rt_device_t dev)
{
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;
    RT_ASSERT(dev != RT_NULL);
#ifdef USE_PTM_DVP_8WIRE
    HAL_DMA_Abort_IT(&dcmi->hdma);
    stop_dvp_ptm();
#else
    __HAL_DCMI_CAPTURE_STOP(&dcmi->handle);
    HAL_DMA_Abort_IT(&dcmi->hdma);
#endif
#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_stop();
#endif  /* RT_USING_PM */

    return RT_EOK;
}

static rt_err_t rt_dcmi_control(rt_device_t dev, int cmd, void *args)
{
    struct rt_dcmi_device *dcmi = (struct rt_dcmi_device *)dev;
    RT_ASSERT(dev != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_START:
        rt_dcmi_start(dev);
        break;
    case RT_DEVICE_CTRL_STOP:
        rt_dcmi_stop(dev);
        break;
    case RT_DEVICE_CTRL_SUSPEND:
        break;
    case RT_DEVICE_CTRL_RESUME:
        break;
    case RT_DEVICE_CTRL_CONFIG:
        rt_dcmi_config(dev, (struct rt_dcmi_config *)args);
        break;
    default:
        LOG_I("SR:%x, RIS:%x, dcmi_err:%x, DMA_CNT:%x",
              dcmi->handle.Instance->SR,
              dcmi->handle.Instance->RIS,
              dcmi->error,
              dcmi->hdma.Instance->CNDTR);
        break;
    }

    return RT_EOK;
}

static rt_size_t rt_dcmi_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    RT_ASSERT(dev != RT_NULL);

    return RT_EOK;
}

static rt_size_t rt_dcmi_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    RT_ASSERT(dev != RT_NULL);

    return RT_EOK;
}

int dcmi_init(void)
{
    rt_dcmi.parent.type      = RT_Device_Class_Miscellaneous;
    rt_dcmi.parent.init      = rt_dcmi_init;
    rt_dcmi.parent.open      = rt_dcmi_open;
    rt_dcmi.parent.close     = rt_dcmi_close;
    rt_dcmi.parent.read      = rt_dcmi_read;
    rt_dcmi.parent.write     = rt_dcmi_write;
    rt_dcmi.parent.control   = rt_dcmi_control;
    rt_dcmi.parent.user_data = RT_NULL;

    rt_device_register(&rt_dcmi.parent, "dcmi", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);

    LOG_I("dcmi init success!");

    return RT_EOK;
}
INIT_BOARD_EXPORT(dcmi_init);
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
