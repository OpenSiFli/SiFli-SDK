
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

#if defined(SF32LB52X) || defined(SF32LB56X)
    uint16_t irq = 0x00E1;
    mbase->intrtxe = irq;
    mbase->intrrxe = 0x001E;
#else
    mbase->intrtxe = hhcd->epmask;
    mbase->intrrxe = hhcd->epmask & 0xfffe;
#endif

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
        if (((ep_num & USB_DIR_IN) && (hhcd->hc[chnum].ep_is_in == 0)) ||
                ((ep_num & USB_DIR_IN) == 0 && (hhcd->hc[chnum].ep_is_in)))
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

#if 1
    /* Disable the Interrupts */
    __HAL_HCD_DISABLE(hhcd);

    /* Init the Core (common init.) */
    //(void)USB_CoreInit(hhcd->Instance, hhcd->Init);

    /* Force Host Mode*/
    //(void)USB_SetCurrentMode(hhcd->Instance, USB_HOST_MODE);

    /* Init Host */
    //(void)USB_HostInit(hhcd->Instance, hhcd->Init);
#endif

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
        hhcd->hc[ch_num].ep_is_in = 1U;
    }
    else
    {
        hhcd->hc[ch_num].ep_is_in = 0U;
    }

    hhcd->hc[ch_num].speed = speed;
    hhcd->epmask |= (1 << hhcd->hc[ch_num].ep_num);
    __HAL_HCD_ENABLE(hhcd);

#if 0
    HCD_TypeDef *mbase = hhcd->Instance;
    status =  USB_HC_Init(hhcd->Instance,
                          ch_num,
                          epnum,
                          dev_address,
                          speed,
                          ep_type,
                          mps);
#endif
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
#if defined(SF32LB52X) || defined(SF32LB56X)
    int ep_num = (hhcd->hc[ch_num].ep_num & 0x00E3);
#else
    int ep_num = (hhcd->hc[ch_num].ep_num & 0x7f);
#endif


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
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hhcd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_HCD_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitialize the HCD MSP.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void  HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hhcd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_HCD_MspDeInit could be implemented in the user file
     */
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
#if 0
            HAL_DBG_printf("%s %d TX ep=%d usb chnum=%d tx\n", __func__, __LINE__, ep_num, ch_num);
            for (i = 0; i < length; i++)
                HAL_DBG_printf("%x ", pbuff[i]);
            HAL_DBG_printf("\n");
#endif
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
#if defined(SF32LB52X) || defined(SF32LB56X)
            __IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[ch_num].epN);
#else
            __IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[ep_num].epN);
#endif
            epn->txcsr = USB_TXCSR_FLUSHFIFO;
            csr = epn->txcsr;
#if defined(SF32LB52X) || defined(SF32LB56X)
            __IO uint8_t *fifox = (__IO uint8_t *) & (hhcd->Instance->fifox[ch_num]);
#else
            __IO uint8_t *fifox = (__IO uint8_t *) & (hhcd->Instance->fifox[ep_num]);
#endif

            csr &= ~(USB_TXCSR_H_NAKTIMEOUT
                     | USB_TXCSR_AUTOSET
                     | USB_TXCSR_DMAENAB
                     | USB_TXCSR_FRCDATATOG
                     | USB_TXCSR_H_RXSTALL
                     | USB_TXCSR_H_ERROR
                     | USB_TXCSR_TXPKTRDY);
            csr |= USB_TXCSR_MODE;
            for (i = 0; i < length; i++) // REVISIT: Use 16bits/32bits FIFO to speed up
                *fifox = *(pbuff + i);
#if 0
            HAL_DBG_printf("%s %d TX ep=%d usb chnum=%d tx\n", __func__, __LINE__, ep_num, ch_num);
            for (i = 0; i < length; i++)
                HAL_DBG_printf("%x ", pbuff[i]);
            HAL_DBG_printf("\n");
#endif
            csr |= USB_TXCSR_AUTOSET;
            csr |= USB_TXCSR_TXPKTRDY;

            epn->txmaxp = hhcd->hc[ch_num].max_packet;
            epn->txtype = (hhcd->hc[ch_num].ep_type << 4) + ep_num;

            epn->txinterval = (HAL_HCD_GetCurrentSpeed(hhcd) == HCD_SPEED_HIGH ? 16 : 2);
            epn->txcsr = csr;
        }
    }
    else//RX
    {
        uint16_t csr;
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
            __IO struct musb_epN_regs *epn = &(hhcd->Instance->ep[ep_num].epN);

            csr = epn->rxcsr;
            //HAL_ASSERT((csr & (USB_RXCSR_RXPKTRDY | USB_RXCSR_DMAENAB | USB_RXCSR_H_REQPKT)) == 0);
            /* scrub any stale state, leaving toggle alone */
            csr &= USB_RXCSR_DISNYET;
            csr |= USB_RXCSR_H_REQPKT;
            epn->rxmaxp = hhcd->hc[ch_num].max_packet;
            epn->rxtype = (hhcd->hc[ch_num].ep_type << 4) + ep_num;
            epn->rxinterval = (HAL_HCD_GetCurrentSpeed(hhcd) == HCD_SPEED_HIGH ? 16 : 2);
            epn->rxcsr = csr;

        }
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

    }


    return r;
}

/**
  * @brief  Handle HCD interrupt request.
  * @param  hhcd HCD handle
  * @retval None
  */
void HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd)
{
    int ep_num;
    uint8_t int_usb = hhcd->Instance->intrusb;
    uint16_t int_tx = hhcd->Instance->intrtx;
    uint16_t int_rx = hhcd->Instance->intrrx;
    uint32_t dmaintr = hhcd->Instance->dmaintr;

    HAL_DBG_printf("USB interrupt usb=%x, tx=%d, rx=%d, usbe=%x, dma_intr=%x\r\n", int_usb, int_tx, int_rx, hhcd->Instance->intrusbe, dmaintr);

    if (int_usb != USB_INTR_SOF)
    {
        musbh_stage0_irq(hhcd, int_usb);
    }

    if (int_tx & 1)
        HCD_HC_EP0_IRQHandler(hhcd);

    if (int_tx & 0xFFFE)
    {
        int_tx >>= 1;
        ep_num = 1;
        while (int_tx)
        {
            if (int_tx & 1)
                HCD_HC_OUT_IRQHandler(hhcd, ep_num);
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
                HCD_HC_IN_IRQHandler(hhcd, ep_num);
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

    power |= USB_POWER_SOFTCONN;
#ifdef SF32LB58X
    power |= USB_POWER_HSENAB;
#endif
    mbase->power = power;

    // Start Host
#ifdef SF32LB55X
    mbase->devctl |= USB_DEVCTL_HR;
#else
    mbase->usbcfg &= 0xEF;
    mbase->devctl |= 0x01;

#endif

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
        power &= 0xf0;
        mbase->power = power | USB_POWER_RESET;
#if defined(SF32LB58X)
        hwp_usbc->rsvd0 = 0xc;//58
#endif
        HAL_Delay(50);
        mbase->power &= (~USB_POWER_RESET);
#if defined(SF32LB58X)
        hwp_usbc->rsvd0 = 0x0;//58
#endif
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
    if (mbase->devctl & USB_DEVCTL_LSDEV)
        return HCD_SPEED_LOW;
    if (mbase->devctl & USB_DEVCTL_FSDEV)
        return HCD_SPEED_FULL;
    if (mbase->power & USB_POWER_HSMODE)
        return HCD_SPEED_HIGH;
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
    int chnum = 0;

    chnum = ep_2_ch(hhcd, ep_num | USB_DIR_IN);
    if (chnum < 0)
        return;

    volatile struct musb_epN_regs *epn = &musb->ep[ep_num].epN;
    uint16_t rxcsr = epn->rxcsr;
    uint16_t rx_count = epn->rxcount;
    //HAL_DBG_printf("rx complete: ep_num=%d,chnum=%d,csr=0x%x, count=%d\n", ep_num, chnum, rxcsr,rx_count);

    if (rxcsr & USB_RXCSR_H_RXSTALL)
    {
        hhcd->hc[chnum].state = HC_STALL;
        hhcd->hc[chnum].urb_state = URB_STALL;
    }
    else if (rxcsr & USB_RXCSR_H_ERROR)
    {
        epn->rxinterval = 0;
        epn->rxcsr &= ~USB_RXCSR_H_ERROR;

        hhcd->hc[chnum].state = HC_XACTERR;
        hhcd->hc[chnum].urb_state = URB_ERROR;
    }
    else if (rxcsr & USB_RXCSR_DATAERROR)
    {
        rxcsr |= USB_RXCSR_H_WZC_BITS;
        rxcsr &= ~USB_RXCSR_DATAERROR;
        hhcd->hc[chnum].state = HC_DATATGLERR;
        hhcd->hc[chnum].urb_state = URB_ERROR;

    }
    else
    {
        int i;
        hhcd->hc[chnum].xfer_count = rx_count;
        pBuf = hhcd->hc[chnum].xfer_buff;
        for (i = 0; i < rx_count; i++)
            *(pBuf + i) = *fifox;
#if 0
        HAL_DBG_printf("%d usb chnum=%d rx\n", __LINE__, chnum);
        for (i = 0; i < rx_count; i++)
            HAL_DBG_printf("%x ", pBuf[i]);
        HAL_DBG_printf("\n");
#endif
        hhcd->hc[chnum].state = HC_XFRC;
        hhcd->hc[chnum].urb_state = URB_DONE;
        epn->rxcsr &= (~USB_RXCSR_RXPKTRDY);
    }
    HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
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
#endif
}

/**
  * @brief  Handle Host Channel OUT interrupt requests.
  * @param  hhcd HCD handle
  * @param  endpoint number.
  *         This parameter can be a value from 1 to 15
  * @retval none
  */
static void HCD_HC_OUT_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
    USBC_X_Typedef *musb = hhcd->Instance;
    //int chnum = ep_2_ch(hhcd, ep_num);
    uint16_t txcsr = musb->ep[chnum].epN.txcsr;

    HAL_DBG_printf("tx complete: chnum=%d,csr=0x%x\n", chnum, txcsr);
    if (txcsr & USB_TXCSR_H_RXSTALL)
    {
        hhcd->hc[chnum].state = HC_STALL;
        hhcd->hc[chnum].urb_state = URB_STALL;
    }
    else if (txcsr & USB_TXCSR_H_ERROR)
    {
        hhcd->hc[chnum].state = HC_XACTERR;
        hhcd->hc[chnum].urb_state = URB_ERROR;
    }
    else if (txcsr & USB_TXCSR_H_NAKTIMEOUT)
    {
        hhcd->hc[chnum].state = HC_NAK;
        hhcd->hc[chnum].urb_state  = URB_NOTREADY;
        musb->ep[chnum].epN.txcsr = USB_TXCSR_H_WZC_BITS | USB_TXCSR_TXPKTRDY;
    }
    else
    {
        hhcd->hc[chnum].state = HC_XFRC;
        hhcd->hc[chnum].urb_state  = URB_DONE;
    }
    HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
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
    if (csr == 0xa0)
    {
        csr = USB_CSR0_RXPKTRDY;
    }

    //HAL_DBG_printf("ep0 ISR: csr=0x%x, len=%d, chnum=%d\r\n", csr, len, chnum);

    if (csr & USB_CSR0_H_ERROR)
    {
        hhcd->hc[chnum].state = HC_XACTERR;
        hhcd->hc[chnum].urb_state = URB_ERROR;
        ep0->csr0 &= (~USB_CSR0_H_ERROR);
    }
    else if (csr & USB_CSR0_H_RXSTALL)
    {
        hhcd->hc[chnum].state = HC_STALL;
        hhcd->hc[chnum].urb_state = URB_STALL;
        ep0->csr0 &= (~USB_CSR0_H_RXSTALL);
    }
    else if (csr & USB_CSR0_H_NAKTIMEOUT)
    {
        hhcd->hc[chnum].state = HC_NAK;
        hhcd->hc[chnum].urb_state = URB_NYET;
        ep0->csr0 &= (~USB_CSR0_H_NAKTIMEOUT);
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
#if 0
            HAL_DBG_printf("%d usb chnum=%d rx\n", __LINE__, chnum);
            for (i = 0; i < len; i++)
                HAL_DBG_printf("%x ", pBuf[i]);
            HAL_DBG_printf("\n");
#endif
            ep0->csr0 &= (~USB_CSR0_RXPKTRDY);
        }
        hhcd->hc[chnum].state = HC_XFRC;
        hhcd->hc[chnum].urb_state = URB_DONE;
    }
    hhcd->ep0_state = HAL_HCD_EP0_IDLE;
    HAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)chnum, hhcd->hc[chnum].urb_state);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
