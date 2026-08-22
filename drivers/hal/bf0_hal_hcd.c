/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

#ifdef HAL_HCD_MODULE_ENABLED

#include "bf0_hal_usb_common.h"


/** @defgroup HCD HCD
  * @brief HCD HAL module driver
  * @{
  */
#ifndef SF32LB55X
    #define USB_TX_DMA_ENABLED 1
    //#define USB_RX_DMA_ENABLED 1
    #define USB_RX_CONT_DMA_ENABLED  1
#endif /*SF32LB55X*/
#define USB_DATA_DEBUG  0
#if USB_DATA_DEBUG
    #include "rtthread.h"
#endif /*USB_DATA_DEBUG*/

static uint8_t tx_packet_max = 0, rx_packet_max = 0;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup HCD_Private_Functions HCD Private Functions
  * @{
  */
static void HCD_HC_IN_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t chnum);
static void HCD_HC_OUT_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t chnum);
static void HCD_HC_EP0_IRQHandler(HCD_HandleTypeDef *hhcd);
#if 0
    static void HCD_RXQLVL_IRQHandler(HCD_HandleTypeDef *hhcd);
#endif
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup HCD_Exported_Functions HCD Exported Functions
  * @{
  */

/** @defgroup HCD_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
          ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:

@endverbatim
  * @{
  */

void __HAL_HCD_ENABLE(HCD_HandleTypeDef *hhcd)
{
    HCD_TypeDef *mbase = hhcd->Instance;

#if defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)
    uint16_t irq = 0x00E1;
    mbase->intrtxe = irq;
    mbase->intrrxe = 0x001E;
#else
    mbase->intrtxe = hhcd->epmask;
    mbase->intrrxe = hhcd->epmask & 0xfffe;
#endif /*defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X) */

    mbase->intrusbe = 0xf7;
}

void __HAL_HCD_DISABLE(HCD_HandleTypeDef *hhcd)
{
    HCD_TypeDef *mbase = hhcd->Instance;

    /* disable interrupts */
    mbase->intrusbe = 0x10;     // Keep connect enable
    mbase->intrtxe = 0;
    mbase->intrrxe = 0;

    /*  flush pending interrupts, w1c */
    mbase->intrusb = mbase->intrusb;
    mbase->intrtx = mbase->intrtx;
    mbase->intrrx = mbase->intrrx;
}

static int ep_2_ch(HCD_HandleTypeDef *hhcd, int ep_num)
{
    int chnum;

    for (chnum = 0; chnum < 16; chnum++)
    {
        if (((ep_num & USB_DIR_IN) ? 1 : 0) != hhcd->hc[chnum].ep_is_in)
            continue;
        if (hhcd->hc[chnum].ep_num == (ep_num & 0x7f) && hhcd->hc[chnum].max_packet > 0)
        {
            ep_num = hhcd->hc[chnum].ep_num;
            break;
        }
    }

    if (chnum == 16)
    {
        chnum = -1;
    }
    return chnum;
}

/**
  * @brief  Initialize the host driver.
  * @param  hhcd HCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_Init(HCD_HandleTypeDef *hhcd)
{
    USB_OTG_GlobalTypeDef *USBx;

    /* Check the HCD handle allocation */
    if (hhcd == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */

    USBx = hhcd->Instance;

    if (hhcd->State == HAL_HCD_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hhcd->Lock = HAL_UNLOCKED;

#if (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
        hhcd->SOFCallback = HAL_HCD_SOF_Callback;
        hhcd->ConnectCallback = HAL_HCD_Connect_Callback;
        hhcd->DisconnectCallback = HAL_HCD_Disconnect_Callback;
        hhcd->PortEnabledCallback = HAL_HCD_PortEnabled_Callback;
        hhcd->PortDisabledCallback = HAL_HCD_PortDisabled_Callback;
        hhcd->HC_NotifyURBChangeCallback = HAL_HCD_HC_NotifyURBChange_Callback;

        if (hhcd->MspInitCallback == NULL)
        {
            hhcd->MspInitCallback = HAL_HCD_MspInit;
        }

        /* Init the low level hardware */
        hhcd->MspInitCallback(hhcd);
#else
        /* Init the low level hardware : GPIO, CLOCK, NVIC... */
        HAL_HCD_MspInit(hhcd);
#endif /* (USE_HAL_HCD_REGISTER_CALLBACKS) */
    }

    hhcd->State = HAL_HCD_STATE_BUSY;
    __HAL_HCD_DISABLE(hhcd);

    hhcd->State = HAL_HCD_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  Initialize a host channel.
  * @param  hhcd HCD handle
  * @param  ch_num Channel number.
  *         This parameter can be a value from 1 to 15
  * @param  epnum Endpoint number.
  *          This parameter can be a value from 1 to 15
  * @param  dev_address Current device address
  *          This parameter can be a value from 0 to 255
  * @param  speed Current device speed.
  *          This parameter can be one of these values:
  *            HCD_DEVICE_SPEED_HIGH: High speed mode,
  *            HCD_DEVICE_SPEED_FULL: Full speed mode,
  *            HCD_DEVICE_SPEED_LOW: Low speed mode
  * @param  ep_type Endpoint Type.
  *          This parameter can be one of these values:
  *            EP_TYPE_CTRL: Control type,
  *            EP_TYPE_ISOC: Isochronous type,
  *            EP_TYPE_BULK: Bulk type,
  *            EP_TYPE_INTR: Interrupt type
  * @param  mps Max Packet Size.
  *          This parameter can be a value from 0 to32K
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd,
                                  uint8_t ch_num,
                                  uint8_t epnum,
                                  uint8_t dev_address,
                                  uint8_t speed,
                                  uint8_t ep_type,
                                  uint16_t mps)
{
    HAL_StatusTypeDef status;

    __HAL_LOCK(hhcd);
    hhcd->hc[ch_num].do_ping = 0U;
    hhcd->hc[ch_num].dev_addr = dev_address;
    hhcd->hc[ch_num].max_packet = mps;
    hhcd->hc[ch_num].ch_num = ch_num;
    hhcd->hc[ch_num].ep_type = ep_type;
    hhcd->hc[ch_num].ep_num = epnum & 0x7FU;

    if ((epnum & 0x80U) == 0x80U)
    {
        hhcd->hc[ch_num].ep_is_in = 1U;//rx
    }
    else
    {
        hhcd->hc[ch_num].ep_is_in = 0U;//tx
    }

    hhcd->hc[ch_num].speed = speed;
    hhcd->epmask |= (1 << hhcd->hc[ch_num].ep_num);
    __HAL_HCD_ENABLE(hhcd);

    __HAL_UNLOCK(hhcd);

    return status;
}

/**
  * @brief  Halt a host channel.
  * @param  hhcd HCD handle
  * @param  ch_num Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_HC_Halt(HCD_HandleTypeDef *hhcd, uint8_t ch_num)
{
    HAL_StatusTypeDef status = HAL_OK;
    HCD_TypeDef *mbase = hhcd->Instance;

    __HAL_LOCK(hhcd);
#if defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)
    int ep_num = (hhcd->hc[ch_num].ep_num & 0x00E3);
#else
    int ep_num = (hhcd->hc[ch_num].ep_num & 0x7f);
#endif/*defined(SF32LB52X) || defined(SF32LB56X)  || defined(SF32LB57X)  */


    hhcd->epmask &= ~(1 << ep_num);


    /* disable interrupts */
    mbase->intrtxe &= ~(1 << ep_num);
    mbase->intrrxe &= ~(1 << ep_num);

    /*  flush pending interrupts, w1c */
    mbase->intrtx = (1 << ep_num);
    mbase->intrrx = (1 << ep_num);

    __HAL_UNLOCK(hhcd);

    return status;
}

/**
  * @brief  DeInitialize the host driver.
  * @param  hhcd HCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_DeInit(HCD_HandleTypeDef *hhcd)
{
    /* Check the HCD handle allocation */
    if (hhcd == NULL)
    {
        return HAL_ERROR;
    }

    hhcd->State = HAL_HCD_STATE_BUSY;

#if (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
    if (hhcd->MspDeInitCallback == NULL)
    {
        hhcd->MspDeInitCallback = HAL_HCD_MspDeInit; /* Legacy weak MspDeInit  */
    }

    /* DeInit the low level hardware */
    hhcd->MspDeInitCallback(hhcd);
#else
    /* DeInit the low level hardware: CLOCK, NVIC.*/
    HAL_HCD_MspDeInit(hhcd);
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */

    __HAL_HCD_DISABLE(hhcd);

    hhcd->State = HAL_HCD_STATE_RESET;

    return HAL_OK;
}

/**
  * @brief  Initialize the HCD MSP.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void  HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd)
{
#ifdef SOC_SF32LB57X
    if (0 == (hwp_hpsys_cfg->ANAU_CR & HPSYS_CFG_ANAU_CR_EN_BG))
    {
        hwp_hpsys_cfg->ANAU_CR |= HPSYS_CFG_ANAU_CR_EN_BG;
    }
#endif /* SOC_SF32LB57X */

    HAL_RCC_EnableModule(RCC_MOD_USBC);

#ifdef SF32LB57X
    /* Delay 10us to make sure BG and rcc usb clock is ready */
    HAL_Delay_us(10);
    /* switch to 48MHz (clk_en=1 and mode_48m=1). mode_48m default value is 1, write 1 again to avoid read back */
    hwp_usbc->mode_48m = 3;
#endif /* SF32LB57X */

#ifdef SF32LB58X
    //hwp_usbc->utmicfg12 = hwp_usbc->utmicfg12 | 0x3; //set xo_clk_sel
    hwp_usbc->utmicfg23 = 0xd8;
    hwp_usbc->ldo25 = hwp_usbc->ldo25 | 0xa; //set psw_en and ldo25_en
    HAL_Delay(1);
    hwp_usbc->swcntl3 = 0x1; //set utmi_en for USB2.0
    hwp_usbc->usbcfg = hwp_usbc->usbcfg | 0x40; //enable usb PLL.
    hwp_usbc->dpbrxdisl = 0xff;
    hwp_usbc->dpbtxdisl = 0xff;
    hwp_usbc->utmicfg25 = hwp_usbc->utmicfg25 | 0xc0;
#elif defined(SF32LB56X)||defined(SF32LB52X) || defined(SF32LB57X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN;
#elif defined(SF32LB55X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN;
#endif /*SF32LB58X*/
}

/**
  * @brief  DeInitialize the HCD MSP.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void  HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd)
{
#ifdef SF32LB58X
    hwp_usbc->usbcfg &= ~0x40;  // Disable usb PLL.
    hwp_usbc->swcntl3 = 0x0;
    hwp_usbc->ldo25 &= ~0xa;    // Disable psw_en and ldo25_en
#elif defined(SF32LB56X)||defined(SF32LB52X)|| defined(SF32LB57X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN);
#elif defined(SF32LB55X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN);
#endif/*SF32LB58X*/
    HAL_RCC_DisableModule(RCC_MOD_USBC);
}

/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group2 Input and Output operation functions
  *  @brief   HCD IO operation functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..] This subsection provides a set of functions allowing to manage the USB Host Data
    Transfer

@endverbatim
  * @{
  */

/**
  * @brief  Submit a new URB for processing.
  * @param  hhcd HCD handle
  * @param  ch_num Channel number.
  *         This parameter can be a value from 1 to 15
  * @param  direction Channel number.
  *          This parameter can be one of these values:
  *           0 : Output / 1 : Input
  * @param  ep_type Endpoint Type.
  *          This parameter can be one of these values:
  *            EP_TYPE_CTRL: Control type/
  *            EP_TYPE_ISOC: Isochronous type/
  *            EP_TYPE_BULK: Bulk type/
  *            EP_TYPE_INTR: Interrupt type/
  * @param  token Endpoint Type.
  *          This parameter can be one of these values:
  *            0: HC_PID_SETUP / 1: HC_PID_DATA1
  * @param  pbuff pointer to URB data
  * @param  length Length of URB data
  * @param  do_ping activate do ping protocol (for high speed only).
  *          This parameter can be one of these values:
  *           0 : do ping inactive / 1 : do ping active
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_HCD_HC_SubmitRequest(HCD_HandleTypeDef *hhcd,
        uint8_t ch_num,
        uint8_t direction,
        uint8_t ep_type,
        uint8_t token,
        uint8_t *pbuff,
        uint16_t length,
        uint8_t do_ping)
{
    HCD_TypeDef *mbase = hhcd->Instance;
    uint8_t ep_num = hhcd->hc[ch_num].ep_num;

    hhcd->hc[ch_num].ep_is_in = direction;
    hhcd->hc[ch_num].ep_type  = ep_type;

    mbase->faddr = hhcd->hc[ch_num].dev_addr;
#ifdef SF32LB58X
    hhcd->Instance->swcntl1 = 0x40;
#endif
    if (direction == 0)     // TX
    {
        if (ep_num == 0)
        {
            int i;
            uint16_t csr;
            __IO uint8_t *fifox = (__IO uint8_t *) & (hhcd->Instance->fifox[ep_num]);
            __IO struct musb_ep0_regs *ep0 = &(hhcd->Instance->ep[ep_num].ep0);

            //ep0->csr0 = USB_CSR0_FLUSHFIFO;
            //HAL_DBG_printf("tx ep0: fifox=%p, length=%d\n", fifox, length);
            for (i = 0; i < length; i++) // REVISIT: Use 16bits/32bits FIFO to speed up
                *fifox = *(pbuff + i);
#if USB_DATA_DEBUG
            rt_kprintf("%s %d TX ep=%d usb chnum=%d tx\n", __func__, __LINE__, ep_num, ch_num);
            for (i = 0; i < length; i++)
                rt_kprintf("%x ", pbuff[i]);
            rt_kprintf("\n");
#endif/*USB_DATA_DEBUG */
            csr = USB_CSR0_TXPKTRDY;
            //HAL_DBG_print_data((char *)pBuf, 0, len);

            if (token == USB_PID_SETUP)
            {
                hhcd->ep0_state = HAL_HCD_EP0_SETUP;
                csr |= USB_CSR0_H_SETUPPKT;
            }
            else
            {
                if (length == 0)
                    csr |= USB_CSR0_H_STATUSPKT;
                hhcd->ep0_state = HAL_HCD_EP0_TX;
            }
            ep0->host_naklimit0 = (HAL_HCD_GetCurrentSpeed(hhcd) == HCD_SPEED_HIGH ? 8 : 4);
            //HAL_DBG_printf("tx ep0: csr=0x%x, token=%x, %d, %d\n", csr, token, length,ep0->host_naklimit0);
            hhcd->hc[ch_num].xfer_count = length;
            ep0->csr0 = csr;
        }
        else
        {
            int i;
            uint16_t csr;
#if USB_DATA_DEBUG
            //pbuff = 0x20004ac4;
            rt_kprintf("%s %d TX ep=%d usb chnum=%d tx pbuff=0x%p\n", __func__, __LINE__, ep_num, ch_num, pbuff);
            for (i = 0; i < length; i++)
            {
                rt_kprintf("%x ", pbuff[i]);
            }
            rt_kprintf("\n");
#endif /*USB_DATA_DEBUG */
#if defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)
            __IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[ch_num].epN);
#else
            __IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[ep_num].epN);
#endif /*defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)*/
            uint8_t num = ch_num;
            epn->txcsr = USB_TXCSR_FLUSHFIFO;
            csr = epn->txcsr;
            csr &= ~(USB_TXCSR_H_NAKTIMEOUT
                     | USB_TXCSR_AUTOSET
                     | USB_TXCSR_DMAENAB
                     | USB_TXCSR_FRCDATATOG
                     | USB_TXCSR_H_RXSTALL
                     | USB_TXCSR_H_ERROR
                     | USB_TXCSR_TXPKTRDY);
            csr |= USB_TXCSR_MODE;

#ifdef USB_TX_DMA_ENABLED
            if (IS_DCACHED_RAM((uint32_t)pbuff))
                mpu_dcache_clean((void *)pbuff, length);
            csr |= USB_TXCSR_DMAENAB | USB_TXCSR_AUTOSET;
            epn->txcsr = csr;
            epn->txmaxp = hhcd->hc[num].max_packet;
            //epn->txinterval = (HAL_HCD_GetCurrentSpeed(hhcd) == HCD_SPEED_HIGH ? 16 : 2);
#if defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)
            epn->txtype = (hhcd->hc[ep_num].ep_type << 4) + ep_num;
#else
            epn->txtype = (hhcd->hc[ch_num].ep_type << 4) + ep_num;
#endif /*defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X) */
            struct musb_dma_regs *dma = (struct musb_dma_regs *) & (hhcd->Instance->dma[num]);
            dma->addr = (REG32)pbuff;
            dma->count = length;
            dma->cntl = (1 << USB_DMACTRL_TRANSMIT_SHIFT) |
#if defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)
                        ((num) << USB_DMACTRL_ENDPOINT_SHIFT) |
#else
                        ((ep_num) << USB_DMACTRL_ENDPOINT_SHIFT) |
#endif /*defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X) */
                        (1 << USB_DMACTRL_IRQENABLE_SHIFT) | USB_DMACTRL_BURSTMODE;
            if (length / hhcd->hc[num].max_packet)
            {
                dma->cntl |= (1 << USB_DMACTRL_ENABLE_SHIFT) | (1 << USB_DMACTRL_MODE1_SHIFT);
                tx_packet_max = 1;
            }
            else
            {
                dma->cntl |= (1 << USB_DMACTRL_ENABLE_SHIFT) | (0 << USB_DMACTRL_MODE1_SHIFT);
                tx_packet_max = 0;
            }
#if USB_DATA_DEBUG
            rt_kprintf("%s %d dma usb num=%d tx\n", __func__, __LINE__, num);
#endif /*USB_DATA_DEBUG */

#else /*no dma*/

#if defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)
            __IO uint8_t *fifox = (__IO uint8_t *) & (hhcd->Instance->fifox[ch_num]);
#else
            __IO uint8_t *fifox = (__IO uint8_t *) & (hhcd->Instance->fifox[ep_num]);
#endif /*defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X) */
            for (i = 0; i < length; i++) // REVISIT: Use 16bits/32bits FIFO to speed up
                *fifox = *(pbuff + i);
#if USB_DATA_DEBUG
            rt_kprintf("%s %d TX ep=%d usb chnum=%d tx num=%d ep_type=%d\n", __func__, __LINE__, ep_num, ch_num, num, hhcd->hc[num].ep_type);
            for (i = 0; i < length; i++)
                rt_kprintf("%x ", pbuff[i]);
            rt_kprintf("\n");
#endif /*USB_DATA_DEBUG */
            csr |= USB_TXCSR_AUTOSET;
            csr |= USB_TXCSR_TXPKTRDY;

            epn->txmaxp = hhcd->hc[ch_num].max_packet;
            epn->txtype = (hhcd->hc[ch_num].ep_type << 4) + ep_num;

            epn->txinterval = (HAL_HCD_GetCurrentSpeed(hhcd) == HCD_SPEED_HIGH ? 16 : 2);
            epn->txcsr = csr;

#endif
#ifdef SF32LB58X
            hhcd->Instance->swcntl1 = 0x0;
#endif
        }
    }
    else//RX
    {
        uint16_t csr;
        uint16_t csr_tx;

        ep_num &= 0x7f;
        hhcd->hc[ch_num].xfer_buff = pbuff;
        if (ep_num == 0)
        {
            __IO struct musb_ep0_regs *ep0 = &(hhcd->Instance->ep[ep_num].ep0);
            csr = ep0->csr0;
            csr |= USB_CSR0_H_REQPKT;
            ep0->csr0 = csr;
            hhcd->ep0_state = HAL_HCD_EP0_RX;
        }
        else
        {
#if defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)
            __IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[ch_num].epN);
#ifndef SF32LB55X
            struct musb_rqpktcount *count_t = (struct musb_rqpktcount *) & (hhcd->Instance->rqpktcount[ch_num - 1]);
#endif
#else
            __IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[ep_num].epN);
#ifndef SF32LB55X
            struct musb_rqpktcount *count_t = (struct musb_rqpktcount *) & (hhcd->Instance->rqpktcount[ep_num - 1]);
#endif /*SF32LB55X*/
#endif /*defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X) */

            epn->rxcsr &= ~(USB_RXCSR_H_REQPKT
                            | USB_RXCSR_DMAENAB
                            | USB_RXCSR_AUTOCLEAR
                            | USB_RXCSR_H_AUTOREQ);
#if USB_DATA_DEBUG
            rt_kprintf("%s %d RX ep=%d usb chnum=%d rx pbuff=0x%p\n", __func__, __LINE__, ep_num, ch_num, pbuff);
#endif /*USB_DATA_DEBUG */
#ifdef USB_RX_CONT_DMA_ENABLED
            uint32_t packet_num = length / hhcd->hc[ch_num].max_packet;
            if (packet_num > 1)
            {
#ifndef SF32LB55X
                count_t->count = packet_num;
#endif/*SF32LB55X*/
                //RT_ASSERT(0);
            }
            struct musb_dma_regs *dma = (struct musb_dma_regs *) & (hhcd->Instance->dma[ch_num]);
            dma->addr = (REG32)pbuff;
            dma->count = length;
            hhcd->hc[ch_num].xfer_count = length;
            dma->cntl = (0 << USB_DMACTRL_TRANSMIT_SHIFT) | //RX
#if defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)
                        ((ch_num) << USB_DMACTRL_ENDPOINT_SHIFT) |
#else
                        ((ep_num) << USB_DMACTRL_ENDPOINT_SHIFT) |
#endif/*defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X) */
                        (1 << USB_DMACTRL_IRQENABLE_SHIFT) | USB_DMACTRL_BURSTMODE;
            if (packet_num > 1)
            {
                dma->cntl |= (1 << USB_DMACTRL_ENABLE_SHIFT) | (1 << USB_DMACTRL_MODE1_SHIFT);
                rx_packet_max = 1;
            }
            else
            {
                rx_packet_max = 0;
            }

#endif/*USB_RX_CONT_DMA_ENABLED */

            csr = epn->rxcsr;
            csr_tx = epn->txcsr;
            csr_tx &= ~USB_TXCSR_MODE;

            /* scrub any stale state, leaving toggle alone */
            // csr &= USB_RXCSR_DISNYET;
            csr |= USB_RXCSR_H_REQPKT;
#ifdef USB_RX_CONT_DMA_ENABLED
            if (rx_packet_max)
                csr |= USB_RXCSR_DMAENAB | USB_RXCSR_AUTOCLEAR | USB_RXCSR_H_AUTOREQ;
#endif/*USB_RX_CONT_DMA_ENABLED */
            epn->rxmaxp = hhcd->hc[ch_num].max_packet;
            epn->rxtype = (hhcd->hc[ch_num].ep_type << 4) + ep_num;
            epn->rxinterval = (HAL_HCD_GetCurrentSpeed(hhcd) == HCD_SPEED_HIGH ? 16 : 2);
            epn->txcsr = csr_tx;
            epn->rxcsr = csr;
            //if(test_pcd) RT_ASSERT(0);
        }
#ifdef SF32LB58X
        hhcd->Instance->swcntl1 = 0x0;
#endif

    }
    return HAL_OK;
}



static int musbh_stage0_irq(HCD_HandleTypeDef *hhcd, uint8_t int_usb)
{
    HCD_TypeDef *mbase = hhcd->Instance;
    int r = 0;

    if (int_usb & USB_INTR_RESUME)
    {
    }

    if (int_usb & USB_INTR_SUSPEND)
    {
        if ((mbase->devctl & USB_DEVCTL_HM) == 0) // A0 only, disconnect HCD will change to device mode.
        {
            __HAL_HCD_DISABLE(hhcd);
            HAL_HCD_Disconnect_Callback(hhcd);
        }

    }

    /* see manual for the order of the tests */
    if (int_usb & USB_INTR_SESSREQ)
    {
        mbase->devctl = USB_INTR_SESSREQ;
        HAL_DBG_printf("%s %d,mbase->devctl=%d\n", __func__, __LINE__, mbase->devctl);
    }

    if (int_usb & USB_INTR_VBUSERROR)
    {
        mbase->devctl |= USB_INTR_SESSREQ;
        HAL_DBG_printf("%s %d,mbase->devctl=%d\n", __func__, __LINE__, mbase->devctl);
    }


    if (int_usb & USB_INTR_CONNECT)
    {
        //HAL_Delay_us(5);
        //mbase->power &= (~USB_POWER_RESET);
        __HAL_HCD_ENABLE(hhcd);
        HAL_HCD_Connect_Callback(hhcd);
    }

    if (int_usb & USB_INTR_DISCONNECT)
    {
        __HAL_HCD_DISABLE(hhcd);
        HAL_HCD_Disconnect_Callback(hhcd);
    }

    if (int_usb & USB_INTR_RESET)
    {
        __HAL_HCD_DISABLE(hhcd);
        HAL_HCD_Disconnect_Callback(hhcd);
    }


    return r;
}
void HAL_HCD_Timerout_Callback(HCD_HandleTypeDef *hhcd)
{
#if !defined(SF32LB55X) && !defined(SF32LB52X) && !defined(SF32LB57X)
    HCD_TypeDef *mbase = hhcd->Instance;
    mbase->rsvd0 = 0x5f;
    mbase->utmicfg13 = 0x4;
    mbase->swcntl1 = 0x40;
    mbase->swcntl2 = 0x03;
    if (mbase->swcntl2 & 0x80)
    {
        mbase->rsvd0 = 0;
        mbase->utmicfg13 = 0;
        mbase->swcntl1 = 0;
        mbase->swcntl2 = 0;
        mbase->devctl = 0x00;
        mbase->usbcfg &= 0xf7;
        HAL_HCD_Disconnect_Callback(hhcd);
    }
    else
    {
        mbase->rsvd0 = 0;
        mbase->utmicfg13 = 0;
        mbase->swcntl1 = 0;
        mbase->swcntl2 = 0;
    }
#endif/*SF32LB55X && SF32LB52X && SF32LB57X */
}
/**
  * @brief  Handle HCD interrupt request.
  * @param  hhcd HCD handle
  * @retval None
  */
void HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd)
{
    uint32_t reg;
    int ep_num;
    uint8_t int_usb = hhcd->Instance->intrusb;
    uint16_t int_tx = hhcd->Instance->intrtx;
    uint16_t int_rx = hhcd->Instance->intrrx;
    uint32_t dmaintr = hhcd->Instance->dmaintr;
#if USB_DATA_DEBUG
    rt_kprintf("USB interrupt usb=%x, tx=%d, rx=%d, usbe=%x, dma_intr=%x\r\n", int_usb, int_tx, int_rx, hhcd->Instance->intrusbe, dmaintr);
#endif /*USB_DATA_DEBUG */
    if (int_usb != USB_INTR_SOF)
    {
        musbh_stage0_irq(hhcd, int_usb);
    }

    if (int_tx & 1)
        HCD_HC_EP0_IRQHandler(hhcd);

#if defined(USB_TX_DMA_ENABLED)||defined(USB_RX_DMA_ENABLED) || defined(USB_RX_CONT_DMA_ENABLED)
    reg = (dmaintr >> 1);
    ep_num = 1;
    while (reg)
    {
        if (reg & 1)
        {
            uint8_t ch_num = hhcd->hc[ep_num].ep_num;
#if defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X)
            __IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[ep_num].epN);

#else
            __IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[ch_num].epN);

#endif/*defined(SF32LB52X) || defined(SF32LB56X) || defined(SF32LB57X) */
            uint16_t csr = epn->txcsr;
            if (hhcd->hc[ep_num].ep_is_in)
            {
//rx
#if USB_DATA_DEBUG
                rt_kprintf("%d usb chnum=%d RX,epn->rxcount=%d xfer_buff=%p\n", __LINE__, ep_num, hhcd->hc[ep_num].xfer_count, hhcd->hc[ep_num].xfer_buff);
                for (int i = 0; i < 0x20; i++)
                    rt_kprintf("%x ", hhcd->hc[ep_num].xfer_buff[i]);
                rt_kprintf("\n");
#endif/*USB_DATA_DEBUG */
                hhcd->hc[ep_num].state = HC_XFRC;
                hhcd->hc[ep_num].urb_state = URB_DONE;
                if (0 == rx_packet_max)
                    epn->rxcsr &= (~USB_RXCSR_RXPKTRDY);//
                epn->rxcsr &= ~(USB_RXCSR_H_REQPKT
                                | USB_RXCSR_DMAENAB
                                | USB_RXCSR_AUTOCLEAR
                                | USB_RXCSR_H_AUTOREQ);
                HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)ep_num, hhcd->hc[ep_num].urb_state);
            }
            else
            {
//tx

                epn->txmaxp = hhcd->hc[ep_num].max_packet;
                if (0 == tx_packet_max)
                {
                    csr |= USB_TXCSR_AUTOSET;
                    csr |= USB_TXCSR_TXPKTRDY;

#if USB_DATA_DEBUG
                    rt_kprintf("11111111111111 ch_num=%d,ep_num=%d dmaintr=%x,ep_is_in=%d csr=0x%x\n", ep_num, ch_num, dmaintr, hhcd->hc[ep_num].ep_is_in, csr);
#endif/*USB_DATA_DEBUG */
                    epn->txcsr = csr;
                }
                else if (!(epn->txcsr & USB_TXCSR_TXPKTRDY))
                {
                    HCD_HC_OUT_IRQHandler(hhcd, ep_num);
                }
                else
                    hhcd->hc[ep_2_ch(hhcd, ep_num)].max_packet_unfinish = 1;

            }


        }
        ep_num++;
        reg >>= 1;
    }
#endif/*defined(USB_TX_DMA_ENABLED)||defined(USB_RX_DMA_ENABLED) || defined(USB_RX_CONT_DMA_ENABLED) */

    if (int_tx & 0xFFFE)
    {
        int_tx >>= 1;
        ep_num = 1;
        while (int_tx)
        {
            if (int_tx & 1)
            {
                if ((0 == tx_packet_max) ||
                        (hhcd->hc[ep_2_ch(hhcd, ep_num)].max_packet_unfinish == 1))
                {
                    if (hhcd->hc[ep_2_ch(hhcd, ep_num)].max_packet_unfinish == 1)
                        hhcd->hc[ep_2_ch(hhcd, ep_num)].max_packet_unfinish = 0;
                    HCD_HC_OUT_IRQHandler(hhcd, ep_num);
                }
            }
            int_tx >>= 1;
            ep_num++;
        }
    }

    if (int_rx)
    {
        int_rx >>= 1;
        ep_num = 1;
        while (int_rx)
        {
            if (int_rx & 1)
            {
                HCD_HC_IN_IRQHandler(hhcd, ep_num);
            }
            int_rx >>= 1;
            ep_num++;
        }
    }

    __DSB();
}


/**
  * @brief  Handles HCD Wakeup interrupt request.
  * @param  hhcd HCD handle
  * @retval HAL status
  */
void HAL_HCD_WKUP_IRQHandler(HCD_HandleTypeDef *hhcd)
{
    UNUSED(hhcd);
}


/**
  * @brief  SOF callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hhcd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_HCD_SOF_Callback could be implemented in the user file
     */
}

/**
  * @brief Connection Event callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hhcd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_HCD_Connect_Callback could be implemented in the user file
     */
}

/**
  * @brief  Disconnection Event callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hhcd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_HCD_Disconnect_Callback could be implemented in the user file
     */
}

/**
  * @brief  Port Enabled  Event callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hhcd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_HCD_Disconnect_Callback could be implemented in the user file
     */
}

/**
  * @brief  Port Disabled  Event callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hhcd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_HCD_Disconnect_Callback could be implemented in the user file
     */
}

/**
  * @brief  Notify URB state change callback.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @param  urb_state:
  *          This parameter can be one of these values:
  *            URB_IDLE/
  *            URB_DONE/
  *            URB_NOTREADY/
  *            URB_NYET/
  *            URB_ERROR/
  *            URB_STALL/
  * @retval None
  */
__weak void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum, HCD_URBStateTypeDef urb_state)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hhcd);
    UNUSED(chnum);
    UNUSED(urb_state);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_HCD_HC_NotifyURBChange_Callback could be implemented in the user file
     */
}

#if (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User USB HCD Callback
  *         To be used instead of the weak predefined callback
  * @param  hhcd USB HCD handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_HCD_SOF_CB_ID USB HCD SOF callback ID
  *          @arg @ref HAL_HCD_CONNECT_CB_ID USB HCD Connect callback ID
  *          @arg @ref HAL_HCD_DISCONNECT_CB_ID OTG HCD Disconnect callback ID
  *          @arg @ref HAL_HCD_PORT_ENABLED_CB_ID USB HCD Port Enable callback ID
  *          @arg @ref HAL_HCD_PORT_DISABLED_CB_ID USB HCD Port Disable callback ID
  *          @arg @ref HAL_HCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref HAL_HCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_RegisterCallback(HCD_HandleTypeDef *hhcd,
        HAL_HCD_CallbackIDTypeDef CallbackID,
        pHCD_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;
        return HAL_ERROR;
    }
    /* Process locked */
    __HAL_LOCK(hhcd);

    if (hhcd->State == HAL_HCD_STATE_READY)
    {
        switch (CallbackID)
        {
        case HAL_HCD_SOF_CB_ID :
            hhcd->SOFCallback = pCallback;
            break;

        case HAL_HCD_CONNECT_CB_ID :
            hhcd->ConnectCallback = pCallback;
            break;

        case HAL_HCD_DISCONNECT_CB_ID :
            hhcd->DisconnectCallback = pCallback;
            break;

        case HAL_HCD_PORT_ENABLED_CB_ID :
            hhcd->PortEnabledCallback = pCallback;
            break;

        case HAL_HCD_PORT_DISABLED_CB_ID :
            hhcd->PortDisabledCallback = pCallback;
            break;

        case HAL_HCD_MSPINIT_CB_ID :
            hhcd->MspInitCallback = pCallback;
            break;

        case HAL_HCD_MSPDEINIT_CB_ID :
            hhcd->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (hhcd->State == HAL_HCD_STATE_RESET)
    {
        switch (CallbackID)
        {
        case HAL_HCD_MSPINIT_CB_ID :
            hhcd->MspInitCallback = pCallback;
            break;

        case HAL_HCD_MSPDEINIT_CB_ID :
            hhcd->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hhcd);
    return status;
}

/**
  * @brief  Unregister an USB HCD Callback
  *         USB HCD callback is redirected to the weak predefined callback
  * @param  hhcd USB HCD handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_HCD_SOF_CB_ID USB HCD SOF callback ID
  *          @arg @ref HAL_HCD_CONNECT_CB_ID USB HCD Connect callback ID
  *          @arg @ref HAL_HCD_DISCONNECT_CB_ID OTG HCD Disconnect callback ID
  *          @arg @ref HAL_HCD_PORT_ENABLED_CB_ID USB HCD Port Enabled callback ID
  *          @arg @ref HAL_HCD_PORT_DISABLED_CB_ID USB HCD Port Disabled callback ID
  *          @arg @ref HAL_HCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref HAL_HCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_UnRegisterCallback(HCD_HandleTypeDef *hhcd, HAL_HCD_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hhcd);

    /* Setup Legacy weak Callbacks  */
    if (hhcd->State == HAL_HCD_STATE_READY)
    {
        switch (CallbackID)
        {
        case HAL_HCD_SOF_CB_ID :
            hhcd->SOFCallback = HAL_HCD_SOF_Callback;
            break;

        case HAL_HCD_CONNECT_CB_ID :
            hhcd->ConnectCallback = HAL_HCD_Connect_Callback;
            break;

        case HAL_HCD_DISCONNECT_CB_ID :
            hhcd->DisconnectCallback = HAL_HCD_Disconnect_Callback;
            break;

        case HAL_HCD_PORT_ENABLED_CB_ID :
            hhcd->PortEnabledCallback = HAL_HCD_PortEnabled_Callback;
            break;

        case HAL_HCD_PORT_DISABLED_CB_ID :
            hhcd->PortDisabledCallback = HAL_HCD_PortDisabled_Callback;
            break;

        case HAL_HCD_MSPINIT_CB_ID :
            hhcd->MspInitCallback = HAL_HCD_MspInit;
            break;

        case HAL_HCD_MSPDEINIT_CB_ID :
            hhcd->MspDeInitCallback = HAL_HCD_MspDeInit;
            break;

        default :
            /* Update the error code */
            hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (hhcd->State == HAL_HCD_STATE_RESET)
    {
        switch (CallbackID)
        {
        case HAL_HCD_MSPINIT_CB_ID :
            hhcd->MspInitCallback = HAL_HCD_MspInit;
            break;

        case HAL_HCD_MSPDEINIT_CB_ID :
            hhcd->MspDeInitCallback = HAL_HCD_MspDeInit;
            break;

        default :
            /* Update the error code */
            hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hhcd);
    return status;
}

/**
  * @brief  Register USB HCD Host Channel Notify URB Change Callback
  *         To be used instead of the weak HAL_HCD_HC_NotifyURBChange_Callback() predefined callback
  * @param  hhcd HCD handle
  * @param  pCallback pointer to the USB HCD Host Channel Notify URB Change Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_RegisterHC_NotifyURBChangeCallback(HCD_HandleTypeDef *hhcd,
        pHCD_HC_NotifyURBChangeCallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;

        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hhcd);

    if (hhcd->State == HAL_HCD_STATE_READY)
    {
        hhcd->HC_NotifyURBChangeCallback = pCallback;
    }
    else
    {
        /* Update the error code */
        hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hhcd);

    return status;
}

/**
  * @brief  Unregister the USB HCD Host Channel Notify URB Change Callback
  *         USB HCD Host Channel Notify URB Change Callback is redirected to the weak HAL_HCD_HC_NotifyURBChange_Callback() predefined callback
  * @param  hhcd HCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_UnRegisterHC_NotifyURBChangeCallback(HCD_HandleTypeDef *hhcd)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hhcd);

    if (hhcd->State == HAL_HCD_STATE_READY)
    {
        hhcd->HC_NotifyURBChangeCallback = HAL_HCD_HC_NotifyURBChange_Callback; /* Legacy weak DataOutStageCallback  */
    }
    else
    {
        /* Update the error code */
        hhcd->ErrorCode |= HAL_HCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hhcd);

    return status;
}
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group3 Peripheral Control functions
  *  @brief   Management functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the HCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Start the host driver.
  * @param  hhcd HCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_Start(HCD_HandleTypeDef *hhcd)
{
    HCD_TypeDef *mbase = hhcd->Instance;
    uint8_t power = 0;
    __HAL_LOCK(hhcd);

    NVIC_EnableIRQ(USBC_IRQn);
    __HAL_SYSCFG_Enable_USB();
    __HAL_SYSCFG_USB_DM_PD();

    __HAL_HCD_ENABLE(hhcd);
    mbase->testmode = 0;
#ifdef SF32LB58X
    power |= USB_POWER_HSENAB;//hs
    //power &= (~USB_POWER_HSENAB);//fs
#endif/*SF32LB58X */
    power |= USB_POWER_SOFTCONN;

    mbase->power = power;

    // Start Host
#ifdef SF32LB55X
    mbase->devctl |= USB_DEVCTL_HR;
#else
    mbase->usbcfg &= 0xEF;
    mbase->devctl |= 0x01;
#endif/*SF32LB55X */

    HAL_DBG_printf("%s %d,mbase->devctl=%d\n", __func__, __LINE__, mbase->devctl);
    __HAL_UNLOCK(hhcd);

    return HAL_OK;
}

/**
  * @brief  Stop the host driver.
  * @param  hhcd HCD handle
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_HCD_Stop(HCD_HandleTypeDef *hhcd)
{
    HCD_TypeDef *mbase = hhcd->Instance;

    __HAL_LOCK(hhcd);

    __HAL_HCD_DISABLE(hhcd);
    mbase->devctl = 0;

    __HAL_UNLOCK(hhcd);

    return HAL_OK;
}

/**
  * @brief  Reset the host port.
  * @param  hhcd HCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_ResetPort(HCD_HandleTypeDef *hhcd)
{
    HCD_TypeDef *mbase = hhcd->Instance;
    uint8_t power;

    power = mbase->power;

    if (power & USB_POWER_RESUME)
    {
        HAL_Delay(20);
        mbase->power = power & (~USB_POWER_RESUME);
    }
    else
    {
#if defined(SF32LB58X)
        mbase->utmicfg25 |= 0xc0;
        mbase->utmicfg21 = 0x23;
        mbase->swcntl2 = 0x7c;
        mbase->utmicfg0 = 0x30;

#endif/*SF32LB58X */
        power &= 0xf0;
        mbase->power = power | USB_POWER_RESET;
#if defined(SF32LB58X)
        hwp_usbc->rsvd0 = 0xc;//58
#endif/*SF32LB58X */
        HAL_Delay(500);
        //__asm("B .");
        mbase->power &= (~USB_POWER_RESET);
        HAL_Delay(5);
        //mbase->power &= (~USB_POWER_HSENAB);
#if defined(SF32LB58X)
        hwp_usbc->rsvd0 = 0x0;//58
#endif /* SF32LB58X*/
#if defined(SF32LB58X)
        mbase->utmicfg25 &= ~0xc0;
        mbase->utmicfg21 = 0x2f;
        mbase->swcntl2 = 0x40;
        mbase->utmicfg0 = 0x00;
#endif/*SF32LB58X */

    }

    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group4 Peripheral State functions
  *  @brief   Peripheral State functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the HCD handle state.
  * @param  hhcd HCD handle
  * @retval HAL state
  */
HCD_StateTypeDef HAL_HCD_GetState(HCD_HandleTypeDef *hhcd)
{
    return hhcd->State;
}

/**
  * @brief  Return  URB state for a channel.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval URB state.
  *          This parameter can be one of these values:
  *            URB_IDLE/
  *            URB_DONE/
  *            URB_NOTREADY/
  *            URB_NYET/
  *            URB_ERROR/
  *            URB_STALL
  */
HCD_URBStateTypeDef HAL_HCD_HC_GetURBState(HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
    return hhcd->hc[chnum].urb_state;
}


/**
  * @brief  Return the last host transfer size.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval last transfer size in byte
  */
uint32_t HAL_HCD_HC_GetXferCount(HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
    return hhcd->hc[chnum].xfer_count;
}

/**
  * @brief  Return the Host Channel state.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval Host channel state
  *          This parameter can be one of these values:
  *            HC_IDLE/
  *            HC_XFRC/
  *            HC_HALTED/
  *            HC_NYET/
  *            HC_NAK/
  *            HC_STALL/
  *            HC_XACTERR/
  *            HC_BBLERR/
  *            HC_DATATGLERR
  */
HCD_HCStateTypeDef  HAL_HCD_HC_GetState(HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
    return hhcd->hc[chnum].state;
}

/**
  * @brief  Return the current Host frame number.
  * @param  hhcd HCD handle
  * @retval Current Host frame number
  */
uint32_t HAL_HCD_GetCurrentFrame(HCD_HandleTypeDef *hhcd)
{

    HCD_TypeDef *mbase = hhcd->Instance;

    return mbase->frame;
}

/**
  * @brief  Return the Host enumeration speed.
  * @param  hhcd HCD handle
  * @retval Enumeration speed
  */
uint32_t HAL_HCD_GetCurrentSpeed(HCD_HandleTypeDef *hhcd)
{
    HCD_TypeDef *mbase = hhcd->Instance;

    HAL_ASSERT((mbase->devctl & USB_DEVCTL_HM));
    if (mbase->power & USB_POWER_HSMODE)
        return HCD_SPEED_HIGH;
    if (mbase->devctl & USB_DEVCTL_FSDEV)
        return HCD_SPEED_FULL;
    if (mbase->devctl & USB_DEVCTL_LSDEV)
        return HCD_SPEED_LOW;
    return  HCD_SPEED_FULL;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup HCD_Private_Functions
  * @{
  */
/**
  * @brief  Handle Host Channel IN interrupt requests.
  * @param  hhcd HCD handle
  * @param  ep_num endpoint number.
  *         This parameter can be a value from 1 to 15
  * @retval none
  */
static void HCD_HC_IN_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t ep_num)
{
    USBC_X_Typedef *musb = hhcd->Instance;
    uint8_t *pBuf;
    __IO uint8_t *fifox = (__IO uint8_t *) & (hhcd->Instance->fifox[ep_num]);
#ifdef SF32LB58X
    int chnum = ep_2_ch(hhcd, ep_num | USB_DIR_IN);
#else
    int chnum = ep_num;//ep_2_ch(hhcd, ep_num | USB_DIR_IN);
#endif/*SF32LB58X */
    if (chnum < 0)
        return;

    volatile struct musb_epN_regs *epn = &musb->ep[ep_num].epN;
    uint16_t rxcsr = epn->rxcsr;
    uint16_t rx_count = epn->rxcount;
    //HAL_DBG_printf("rx complete: ep_num=%d,chnum=%d,csr=0x%x, count=%d\n", ep_num, chnum, rxcsr,rx_count);
#if USB_DATA_DEBUG
    rt_kprintf("%s %d ep_num=%d,chnum=%d,rxcsr=0x%x,rx_count=%d\n", __func__, __LINE__, ep_num, chnum, rxcsr, rx_count);
#endif/*USB_DATA_DEBUG */
    if (rxcsr & USB_RXCSR_H_RXSTALL)
    {
        hhcd->hc[chnum].state = HC_STALL;
        hhcd->hc[chnum].urb_state = URB_STALL;
        epn->rxcsr &= (~USB_RXCSR_H_RXSTALL);
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else if (rxcsr & USB_RXCSR_H_ERROR)
    {
        epn->rxinterval = 0;
        epn->rxcsr &= ~USB_RXCSR_H_ERROR;

        hhcd->hc[chnum].state = HC_XACTERR;
        hhcd->hc[chnum].urb_state = URB_ERROR;
        epn->rxcsr &= (~USB_RXCSR_H_ERROR);
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else if (rxcsr & USB_RXCSR_DATAERROR)
    {
        rxcsr |= USB_RXCSR_H_WZC_BITS;
        rxcsr &= ~USB_RXCSR_DATAERROR;
        hhcd->hc[chnum].state = HC_DATATGLERR;
        hhcd->hc[chnum].urb_state = URB_ERROR;
        epn->rxcsr &= (~USB_RXCSR_DATAERROR);
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else
    {
        int i;
        if ((rxcsr | USB_RXCSR_H_DATATOGGLE))
        {
            hhcd->hc[chnum].xfer_count = rx_count;
            pBuf = hhcd->hc[chnum].xfer_buff;
            HAL_DBG_printf("%s %d chnum=%d,ep_num=%d,rxcsr=0x%x\n", __func__, __LINE__, chnum, ep_num, rxcsr);
            HCD_HCTypeDef *ep = &hhcd->hc[chnum];
            struct musb_dma_regs *dma = (struct musb_dma_regs *) & (hhcd->Instance->dma[chnum]);
#if USB_RX_DMA_ENABLED
            //__IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[chnum].epN);

            HAL_DBG_printf("DMA RX pipe=%d, len=%d,xfer_buff=%p\n", chnum,  epn->rxcount, ep->xfer_buff);
            if (epn->rxcount)
            {
                if (IS_DCACHED_RAM(ep->xfer_buff))
                    SCB_InvalidateDCache_by_Addr(ep->xfer_buff, epn->rxcount);

                dma->addr = (REG32)ep->xfer_buff;
                dma->count = epn->rxcount;
                ep->xfer_count = dma->count;
                //epn->rxcount = 0;
                dma->cntl = (1 << USB_DMACTRL_ENABLE_SHIFT)   |
                            (0 << USB_DMACTRL_TRANSMIT_SHIFT) |
                            (0 << USB_DMACTRL_MODE1_SHIFT)    |
                            (ep_num << USB_DMACTRL_ENDPOINT_SHIFT) |
                            (1 << USB_DMACTRL_IRQENABLE_SHIFT);
            }
#elif USB_RX_CONT_DMA_ENABLED
            if (0 == rx_packet_max)
            {
                if (IS_DCACHED_RAM(ep->xfer_buff))
                    SCB_InvalidateDCache_by_Addr(ep->xfer_buff, epn->rxcount);
                ep->xfer_count = rx_count;
                epn->rxcsr &= ~(USB_RXCSR_H_REQPKT
                                | USB_RXCSR_DMAENAB
                                | USB_RXCSR_AUTOCLEAR
                                | USB_RXCSR_H_AUTOREQ);
                dma->cntl |= (1 << USB_DMACTRL_ENABLE_SHIFT) | (0 << USB_DMACTRL_MODE1_SHIFT);
            }
#else
            for (i = 0; i < rx_count; i++)
                *(pBuf + i) = *fifox;
#if USB_DATA_DEBUG
            rt_kprintf("%d usb chnum=%d rx\n", __LINE__, chnum);
            for (i = 0; i < rx_count; i++)
                rt_kprintf("%x ", pBuf[i]);
            rt_kprintf("\n");
#endif/*USB_DATA_DEBUG */
            hhcd->hc[chnum].state = HC_XFRC;
            hhcd->hc[chnum].urb_state = URB_DONE;
            epn->rxcsr &= (~USB_RXCSR_RXPKTRDY);
            if ((rxcsr | USB_RXCSR_H_DATATOGGLE))
                HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
#endif/*USB_RX_DMA_ENABLED */
        }

    }

#if 0
    if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_AHBERR) == USB_OTG_HCINT_AHBERR)
    {
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_AHBERR);
        __HAL_HCD_UNMASK_HALT_HC_INT(ch_num);
    }
    else if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_BBERR) == USB_OTG_HCINT_BBERR)
    {
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_BBERR);
        hhcd->hc[ch_num].state = HC_BBLERR;
        __HAL_HCD_UNMASK_HALT_HC_INT(ch_num);
        (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    }
    else if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_ACK) == USB_OTG_HCINT_ACK)
    {
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_ACK);
    }
    else if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_STALL) == USB_OTG_HCINT_STALL)
    {
        __HAL_HCD_UNMASK_HALT_HC_INT(ch_num);
        hhcd->hc[ch_num].state = HC_STALL;
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_NAK);
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_STALL);
        (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    }
    else if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_DTERR) == USB_OTG_HCINT_DTERR)
    {
        __HAL_HCD_UNMASK_HALT_HC_INT(ch_num);
        hhcd->hc[ch_num].state = HC_DATATGLERR;
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_NAK);
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_DTERR);
        (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    }
    else if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_TXERR) == USB_OTG_HCINT_TXERR)
    {
        __HAL_HCD_UNMASK_HALT_HC_INT(ch_num);
        hhcd->hc[ch_num].state = HC_XACTERR;
        (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_TXERR);
    }
    else
    {
        /* ... */
    }

    if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_FRMOR) == USB_OTG_HCINT_FRMOR)
    {
        __HAL_HCD_UNMASK_HALT_HC_INT(ch_num);
        (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_FRMOR);
    }
    else if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_XFRC) == USB_OTG_HCINT_XFRC)
    {
        if (hhcd->Init.dma_enable != 0U)
        {
            hhcd->hc[ch_num].xfer_count = hhcd->hc[ch_num].XferSize - \
                                          (USBx_HC(ch_num)->HCTSIZ & USB_OTG_HCTSIZ_XFRSIZ);
        }

        hhcd->hc[ch_num].state = HC_XFRC;
        hhcd->hc[ch_num].ErrCnt = 0U;
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_XFRC);

        if ((hhcd->hc[ch_num].ep_type == EP_TYPE_CTRL) ||
                (hhcd->hc[ch_num].ep_type == EP_TYPE_BULK))
        {
            __HAL_HCD_UNMASK_HALT_HC_INT(ch_num);
            (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
            __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_NAK);
        }
        else if (hhcd->hc[ch_num].ep_type == EP_TYPE_INTR)
        {
            USBx_HC(ch_num)->HCCHAR |= USB_OTG_HCCHAR_ODDFRM;
            hhcd->hc[ch_num].urb_state = URB_DONE;

#if (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
            hhcd->HC_NotifyURBChangeCallback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#else
            HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */
        }
        else if (hhcd->hc[ch_num].ep_type == EP_TYPE_ISOC)
        {
            hhcd->hc[ch_num].urb_state = URB_DONE;
            hhcd->hc[ch_num].toggle_in ^= 1U;

#if (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
            hhcd->HC_NotifyURBChangeCallback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#else
            HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */
        }
        else
        {
            /* ... */
        }

        if (hhcd->Init.dma_enable == 1U)
        {
            if (((hhcd->hc[ch_num].XferSize / hhcd->hc[ch_num].max_packet) & 1U) != 0U)
            {
                hhcd->hc[ch_num].toggle_in ^= 1U;
            }
        }
        else
        {
            hhcd->hc[ch_num].toggle_in ^= 1U;
        }
    }
    else if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_CHH) == USB_OTG_HCINT_CHH)
    {
        __HAL_HCD_MASK_HALT_HC_INT(ch_num);

        if (hhcd->hc[ch_num].state == HC_XFRC)
        {
            hhcd->hc[ch_num].urb_state = URB_DONE;
        }
        else if (hhcd->hc[ch_num].state == HC_STALL)
        {
            hhcd->hc[ch_num].urb_state = URB_STALL;
        }
        else if ((hhcd->hc[ch_num].state == HC_XACTERR) ||
                 (hhcd->hc[ch_num].state == HC_DATATGLERR))
        {
            hhcd->hc[ch_num].ErrCnt++;
            if (hhcd->hc[ch_num].ErrCnt > 2U)
            {
                hhcd->hc[ch_num].ErrCnt = 0U;
                hhcd->hc[ch_num].urb_state = URB_ERROR;
            }
            else
            {
                hhcd->hc[ch_num].urb_state = URB_NOTREADY;

                /* re-activate the channel */
                tmpreg = USBx_HC(ch_num)->HCCHAR;
                tmpreg &= ~USB_OTG_HCCHAR_CHDIS;
                tmpreg |= USB_OTG_HCCHAR_CHENA;
                USBx_HC(ch_num)->HCCHAR = tmpreg;
            }
        }
        else if (hhcd->hc[ch_num].state == HC_NAK)
        {
            hhcd->hc[ch_num].urb_state  = URB_NOTREADY;

            /* re-activate the channel */
            tmpreg = USBx_HC(ch_num)->HCCHAR;
            tmpreg &= ~USB_OTG_HCCHAR_CHDIS;
            tmpreg |= USB_OTG_HCCHAR_CHENA;
            USBx_HC(ch_num)->HCCHAR = tmpreg;
        }
        else if (hhcd->hc[ch_num].state == HC_BBLERR)
        {
            hhcd->hc[ch_num].ErrCnt++;
            hhcd->hc[ch_num].urb_state = URB_ERROR;
        }
        else
        {
            /* ... */
        }
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_CHH);
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
    }
    else if ((USBx_HC(ch_num)->HCINT & USB_OTG_HCINT_NAK) == USB_OTG_HCINT_NAK)
    {
        if (hhcd->hc[ch_num].ep_type == EP_TYPE_INTR)
        {
            hhcd->hc[ch_num].ErrCnt = 0U;
            __HAL_HCD_UNMASK_HALT_HC_INT(ch_num);
            (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
        }
        else if ((hhcd->hc[ch_num].ep_type == EP_TYPE_CTRL) ||
                 (hhcd->hc[ch_num].ep_type == EP_TYPE_BULK))
        {
            hhcd->hc[ch_num].ErrCnt = 0U;

            if (hhcd->Init.dma_enable == 0U)
            {
                hhcd->hc[ch_num].state = HC_NAK;
                __HAL_HCD_UNMASK_HALT_HC_INT(ch_num);
                (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
            }
        }
        else
        {
            /* ... */
        }
        __HAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCINT_NAK);
    }
    else
    {
        /* ... */
    }
#endif/* USB_RX_DMA_ENABLED */

}

/**
  * @brief  Handle Host Channel OUT interrupt requests.
  * @param  hhcd HCD handle
  * @param  endpoint number.
  *         This parameter can be a value from 1 to 15
  * @retval none
  */
static void HCD_HC_OUT_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t ep_num)
{
    USBC_X_Typedef *musb = hhcd->Instance;
#ifdef SF32LB58X
    uint8_t chnum = ep_2_ch(hhcd, ep_num | USB_DIR_OUT);
#else
    uint8_t chnum = ep_num;
#endif/*SF32LB58X */
    uint16_t txcsr = musb->ep[ep_num].epN.txcsr;

    if (chnum < 0)
        return;
    if (txcsr & USB_TXCSR_H_RXSTALL)
    {
        hhcd->hc[chnum].state = HC_STALL;
        hhcd->hc[chnum].urb_state = URB_STALL;
        txcsr &= (~USB_TXCSR_H_RXSTALL);
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else if (txcsr & USB_TXCSR_H_ERROR)
    {
        hhcd->hc[chnum].state = HC_XACTERR;
        hhcd->hc[chnum].urb_state = URB_ERROR;
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else if (txcsr & USB_TXCSR_H_NAKTIMEOUT)
    {
        hhcd->hc[chnum].state = HC_NAK;
        hhcd->hc[chnum].urb_state  = URB_NOTREADY;
        musb->ep[ep_num].epN.txcsr = USB_TXCSR_H_WZC_BITS | USB_TXCSR_TXPKTRDY;
        musb->ep[ep_num].epN.txcsr &= ~USB_TXCSR_H_NAKTIMEOUT;
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else if (txcsr & 0x0002)
    {
        //HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else
    {
        hhcd->hc[chnum].state = HC_XFRC;
        hhcd->hc[chnum].urb_state  = URB_DONE;
        txcsr &= (~0xa000);
        if (txcsr == 0x000) HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
        if ((txcsr | USB_TXCSR_H_DATATOGGLE)) HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
        else musb->ep[chnum].epN.txcsr &= ~USB_TXCSR_H_DATATOGGLE;
    }

}


static void HCD_HC_EP0_IRQHandler(HCD_HandleTypeDef *hhcd)
{
    uint16_t csr;
    uint16_t len;
    int chnum;
    __IO struct musb_ep0_regs *ep0 = &(hhcd->Instance->ep[0].ep0);
    __IO uint8_t *fifox = (__IO uint8_t *) & (hhcd->Instance->fifox[0]);

    csr = ep0->csr0;
    len = ep0->count0;

    if (hhcd->ep0_state == HAL_HCD_EP0_SETUP || hhcd->ep0_state == HAL_HCD_EP0_TX)
    {
        chnum = ep_2_ch(hhcd, 0);
    }
    else if (hhcd->ep0_state == HAL_HCD_EP0_RX)
    {
        chnum = ep_2_ch(hhcd, 0 | USB_DIR_IN);
    }
    else
    {
        HAL_ASSERT(0);
    }
    HAL_DBG_printf("ep0 ISR: csr=0x%x, len=%d, chnum=%d\r\n", csr, len, chnum);
    if ((csr & 0xa0) == 0xa0)
    {
        //csr = USB_CSR0_RXPKTRDY;
        ep0->csr0 &= (~0xa0);
    }

    //HAL_DBG_printf("ep0 ISR: csr=0x%x, len=%d, chnum=%d\r\n", csr, len, chnum);

    if (csr & USB_CSR0_H_ERROR)
    {
        hhcd->hc[chnum].state = HC_XACTERR;
        hhcd->hc[chnum].urb_state = URB_ERROR;
        ep0->csr0 &= (~USB_CSR0_H_ERROR);
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else if (csr & USB_CSR0_H_RXSTALL)
    {
        hhcd->hc[chnum].state = HC_STALL;
        hhcd->hc[chnum].urb_state = URB_STALL;
        ep0->csr0 &= (~USB_CSR0_H_RXSTALL);
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else if (csr & USB_CSR0_H_NAKTIMEOUT)
    {
        hhcd->hc[chnum].state = HC_NAK;
        hhcd->hc[chnum].urb_state = URB_NYET;
        ep0->csr0 &= (~USB_CSR0_H_NAKTIMEOUT);
        HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    else
    {
        if (csr & USB_CSR0_RXPKTRDY)
        {
            int i;
            hhcd->hc[chnum].xfer_count = len;
            uint8_t *pBuf = hhcd->hc[chnum].xfer_buff;
            for (i = 0; i < len; i++)
                *(pBuf + i) = *fifox;
#if USB_DATA_DEBUG
            rt_kprintf("%d usb chnum=%d rx\n", __LINE__, chnum);
            for (i = 0; i < len; i++)
                rt_kprintf("%x ", pBuf[i]);
            rt_kprintf("\n");
#endif/*USB_DATA_DEBUG */
            ep0->csr0 &= (~USB_CSR0_RXPKTRDY);
        }
        hhcd->hc[chnum].state = HC_XFRC;
        hhcd->hc[chnum].urb_state = URB_DONE;
        if ((csr | USB_CSR0_H_DATATOGGLE))
            HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
    }
    hhcd->ep0_state = HAL_HCD_EP0_IDLE;

}

#if 0
/**
  * @brief  Handle Rx Queue Level interrupt requests.
  * @param  hhcd HCD handle
  * @retval none
  */
static void HCD_RXQLVL_IRQHandler(HCD_HandleTypeDef *hhcd)
{

    uint32_t GrxstspReg = hhcd->Instance->GRXSTSP;
    uint32_t ch_num = GrxstspReg & USB_OTG_GRXSTSP_EPNUM;
    uint32_t pktsts = (GrxstspReg & USB_OTG_GRXSTSP_PKTSTS) >> 17;
    uint32_t pktcnt = (GrxstspReg & USB_OTG_GRXSTSP_BCNT) >> 4;

    switch (pktsts)
    {
    case GRXSTS_PKTSTS_IN:
        /* Read the data into the host buffer. */
        if ((pktcnt > 0U) && (hhcd->hc[ch_num].xfer_buff != (void *)0))
        {
            if ((hhcd->hc[ch_num].xfer_count + pktcnt) <= hhcd->hc[ch_num].xfer_len)
            {
                (void)USB_ReadPacket(hhcd->Instance,
                                     hhcd->hc[ch_num].xfer_buff, (uint16_t)pktcnt);

                /* manage multiple Xfer */
                hhcd->hc[ch_num].xfer_buff += pktcnt;
                hhcd->hc[ch_num].xfer_count += pktcnt;

                /* get transfer size packet count */
                uint32_t xferSizePktCnt = (USBx_HC(ch_num)->HCTSIZ & USB_OTG_HCTSIZ_PKTCNT) >> 19;

                if ((hhcd->hc[ch_num].max_packet == pktcnt) && (xferSizePktCnt > 0U))
                {
                    /* re-activate the channel when more packets are expected */
                    uint32_t tmpreg;
                    tmpreg = USBx_HC(ch_num)->HCCHAR;
                    tmpreg &= ~USB_OTG_HCCHAR_CHDIS;
                    tmpreg |= USB_OTG_HCCHAR_CHENA;
                    USBx_HC(ch_num)->HCCHAR = tmpreg;
                    hhcd->hc[ch_num].toggle_in ^= 1U;
                }
            }
            else
            {
                hhcd->hc[ch_num].urb_state = URB_ERROR;
            }
        }
        break;

    case GRXSTS_PKTSTS_DATA_TOGGLE_ERR:
        break;

    case GRXSTS_PKTSTS_IN_XFER_COMP:
    case GRXSTS_PKTSTS_CH_HALTED:
    default:
        break;
    }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_HCD_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */
