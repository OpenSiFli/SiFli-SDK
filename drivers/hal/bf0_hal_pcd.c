/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */

#include "bf0_hal.h"
#include "bf0_hal_usb_common.h"
#include "rtconfig.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

#if defined(HAL_PCD_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)


/** @defgroup PCD USB Device
  * @brief PCD HAL module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup PCD_Private_Functions PCD Private Functions
  * @{
  */
static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd);
static const char *ep0_state_str(uint8_t state);
static volatile uint8_t s_tx_packet_max[16] = {0};

#define ep0_state_change(hpcd,new_state) \
{ \
    HAL_DBG_printf("EP0 State %s -> %s, line %d\r\n", ep0_state_str(hpcd->ep0_state), ep0_state_str(new_state), __LINE__); \
    hpcd->ep0_state = new_state; \
}

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PCD_Exported_Functions PCD Exported Functions
  * @{
  */

/** @defgroup PCD_Exported_Functions_Group1 Initialization and de-initialization functions
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

/**
  * @brief  Initializes the PCD according to the specified
  *         parameters in the PCD_InitTypeDef and create the associated handle.
  * @param  hpcd PCD handle
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd)
{
    uint32_t i = 0U;


    /* Check the PCD handle allocation */
    if (hpcd == NULL)
    {
        return HAL_ERROR;
    }

    if (hpcd->State == HAL_PCD_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hpcd->Lock = HAL_UNLOCKED;

        /* Init the low level hardware : GPIO, CLOCK, NVIC... */
        HAL_PCD_MspInit(hpcd);
    }

    hpcd->State = HAL_PCD_STATE_BUSY;

    /* Init endpoints structures */
    for (i = 0U; i < hpcd->Init.dev_endpoints ; i++)
    {
        /* Init ep structure */
        hpcd->IN_ep[i].is_in = 1U;
        hpcd->IN_ep[i].num = i;
        /* Control until ep is actvated */
        hpcd->IN_ep[i].type = PCD_EP_TYPE_CTRL;
        hpcd->IN_ep[i].maxpacket =  0U;
        hpcd->IN_ep[i].xfer_buff = 0U;
        hpcd->IN_ep[i].xfer_len = 0U;
    }

    for (i = 0U; i < hpcd->Init.dev_endpoints ; i++)
    {
        hpcd->OUT_ep[i].is_in = 0U;
        hpcd->OUT_ep[i].num = i;
        /* Control until ep is activated */
        hpcd->OUT_ep[i].type = PCD_EP_TYPE_CTRL;
        hpcd->OUT_ep[i].maxpacket = 0U;
        hpcd->OUT_ep[i].xfer_buff = 0U;
        hpcd->OUT_ep[i].xfer_len = 0U;
    }

    /* Init Device, Clear session*/
    // hpcd->Instance->devctl |= USB_DEVCTL_SESSION;

    // TODO: Turn on PHY???

    /* , start software connection */
    {
        uint8_t power = hpcd->Instance->power;
        if (hpcd->Init.speed == PCD_SPEED_HIGH)
        {
            power |= USB_POWER_HSENAB;
        }
        else
        {
            power &= ~USB_POWER_HSENAB;
        }
        power |= USB_POWER_SOFTCONN;
        hpcd->Instance->power = power;
    }


    hpcd->USB_Address = 0U;
    hpcd->State = HAL_PCD_STATE_READY;
    ep0_state_change(hpcd, HAL_PCD_EP0_SETUP);
    hpcd->phy_state = OTG_STATE_B_PERIPHERALS;

    return HAL_OK;
}

/**
  * @brief  DeInitializes the PCD peripheral
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd)
{
    /* Check the PCD handle allocation */
    if (hpcd == NULL)
    {
        return HAL_ERROR;
    }

    hpcd->State = HAL_PCD_STATE_BUSY;

    /* Stop Device */
    HAL_PCD_Stop(hpcd);

    /* DeInit the low level hardware */
    HAL_PCD_MspDeInit(hpcd);

    hpcd->State = HAL_PCD_STATE_RESET;

    return HAL_OK;
}

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    HAL_RCC_EnableModule(RCC_MOD_USBC);

#ifdef SF32LB58X
    //hwp_usbc->utmicfg12 = hwp_usbc->utmicfg12 | 0x3; //set xo_clk_sel
    hwp_usbc->utmicfg23 = 0xd8;
    hwp_usbc->ldo25 = hwp_usbc->ldo25 | 0xa; //set psw_en and ldo25_en
    HAL_Delay(1);
    hwp_usbc->swcntl3 = 0x1; //set utmi_en for USB2.0
    hwp_usbc->usbcfg = hwp_usbc->usbcfg | 0x40; //enable usb PLL.
#elif defined(SF32LB56X)||defined(SF32LB52X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN;
#elif defined(SF32LB55X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN;
#endif
}

/**
  * @brief  DeInitializes PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
#ifdef SF32LB58X
    hwp_usbc->usbcfg &= ~0x40;  // Disable usb PLL.
    hwp_usbc->swcntl3 = 0x0;
    hwp_usbc->ldo25 &= ~0xa;    // Disable psw_en and ldo25_en
#elif defined(SF32LB56X)||defined(SF32LB52X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN);
#elif defined(SF32LB55X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN);
#endif
    /* reset USB to make DP change to PULLDOWN state */
    hwp_hpsys_rcc->RSTR2 |= HPSYS_RCC_RSTR2_USBC;
    HAL_Delay_us(100);
    hwp_hpsys_rcc->RSTR2 &= ~HPSYS_RCC_RSTR2_USBC;
    HAL_RCC_DisableModule(RCC_MOD_USBC);
}

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group2 IO operation functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the PCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Start the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd)
{
    /* Enabling DP Pull-Down bit to Connect internal pull-up on USB DP line */
    /* TODO: ?????*/
    HAL_PCD_EP_Open(hpcd, 0x00, 0x40, EP_TYPE_CTRL);
    HAL_PCD_EP_Open(hpcd, 0x80, 0x40, EP_TYPE_CTRL);
    HAL_DBG_printf("Enable IRQ %d\n", USBC_IRQn);
    NVIC_EnableIRQ(USBC_IRQn);
    __HAL_SYSCFG_Enable_USB();

    return HAL_OK;
}

/**
  * @brief  Stop the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd)
{
    __HAL_LOCK(hpcd);

    NVIC_DisableIRQ(USBC_IRQn);
    HAL_PCD_EP_Close(hpcd, 0x00);
    HAL_PCD_EP_Close(hpcd, 0x80);

    __HAL_UNLOCK(hpcd);
    return HAL_OK;
}
__weak uint8_t HAL_PCD_Get_RxbuffControl(uint8_t ep_num)
{
    return 0;
}
__weak void HAL_PCD_Set_RxbuffControl(uint8_t ep_num, uint8_t flag)
{
}
__weak void HAL_PCD_Set_RxscrACK(uint8_t epnum)
{
}
/**
  * @brief  This function handles PCD interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd)
{
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    PCD_EP_ISR_Handler(hpcd);
    // TODO: Other interrupt such as wake/suspend/stall etc not implemented yet
}

/**
  * @brief  Data out stage callbacks
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);
    UNUSED(epnum);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_DataOutStageCallback could be implemented in the user file
     */
}

/**
  * @brief  Data IN stage callbacks
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);
    UNUSED(epnum);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_DataInStageCallback could be implemented in the user file
     */
}

/**
  * @brief  Setup stage callback
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_SetupStageCallback could be implemented in the user file
     */
}

/**
  * @brief  USB Start Of Frame callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_SOFCallback could be implemented in the user file
     */
}

/**
  * @brief  USB Reset callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_ResetCallback could be implemented in the user file
     */
}

/**
  * @brief  Suspend event callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_SuspendCallback could be implemented in the user file
     */
}

/**
  * @brief  Resume event callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_ResumeCallback could be implemented in the user file
     */
}

/**
  * @brief  Incomplete ISO OUT callbacks
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);
    UNUSED(epnum);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_ISOOUTIncompleteCallback could be implemented in the user file
     */
}

/**
  * @brief  Incomplete ISO IN  callbacks
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);
    UNUSED(epnum);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_ISOINIncompleteCallback could be implemented in the user file
     */
}

/**
  * @brief  Connection event callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_ConnectCallback could be implemented in the user file
     */
}

/**
  * @brief  Disconnection event callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpcd);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PCD_DisconnectCallback could be implemented in the user file
     */
}
/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group3 Peripheral Control functions
 *  @brief   management functions
 *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the PCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Set the USB Device address
  * @param  hpcd PCD handle
  * @param  address new device address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
{
    __HAL_LOCK(hpcd);

    /* set device address and enable function */
    hpcd->Instance->faddr = address;
    hpcd->USB_Address = address;
    __HAL_UNLOCK(hpcd);
    return HAL_OK;
}
/**
  * @brief  Open and configure an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  ep_mps endpoint max packert size
  * @param  ep_type endpoint type
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type)
{
    HAL_StatusTypeDef  ret = HAL_OK;
    PCD_EPTypeDef *ep;
    uint16_t csr;

    if ((ep_addr & 0x80U) == 0x80U)
    {
        ep = &hpcd->IN_ep[ep_addr & 0x7FU];
    }
    else
    {
        ep = &hpcd->OUT_ep[ep_addr & 0x7FU];
    }
    ep->num   = ep_addr & 0x7FU;
    ep->is_in = (0x80U & ep_addr) != 0U;
    ep->maxpacket = ep_mps;
    ep->type = ep_type;

    HAL_DBG_printf("ep_addr=%x, num=%d, is_in=%d\n", ep_addr, ep->num, ep->is_in);
    __HAL_LOCK(hpcd);
    NVIC_DisableIRQ(USBC_IRQn);
    if (ep->num)
    {
        hpcd->Instance->index = ep->num;
        if (ep->is_in)
        {
            hpcd->Instance->intrtx |= (1 << ep->num);
            hpcd->Instance->txmaxp = ep_mps;
            csr = USB_TXCSR_MODE | USB_TXCSR_CLRDATATOG;
            if (hpcd->Instance->csr0_txcsr & USB_TXCSR_FIFONOTEMPTY)
                csr |= USB_TXCSR_FLUSHFIFO;
            hpcd->Instance->csr0_txcsr = csr;
        }
        else
        {
            hpcd->Instance->intrrx |= (1 << ep->num);
            hpcd->Instance->rxmaxp = ep_mps;
            csr = USB_RXCSR_FLUSHFIFO | USB_RXCSR_CLRDATATOG ;
            // USB_RXCSR_AUTOCLEAR

            //if (ep->type == PCD_EP_TYPE_INTR)
            csr |= USB_RXCSR_DISNYET;
            // Set twice in case of double buffering.
            hpcd->Instance->rxcsr = csr;
            hpcd->Instance->rxcsr = csr;
        }
    }
    NVIC_EnableIRQ(USBC_IRQn);
    __HAL_UNLOCK(hpcd);
    return ret;
}


/**
  * @brief  Deactivate an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    PCD_EPTypeDef *ep;

    if ((ep_addr & 0x80U) == 0x80U)
    {
        ep = &hpcd->IN_ep[ep_addr & 0x7FU];
    }
    else
    {
        ep = &hpcd->OUT_ep[ep_addr & 0x7FU];
    }
    ep->num   = ep_addr & 0x7FU;

    ep->is_in = (0x80U & ep_addr) != 0U;

    __HAL_LOCK(hpcd);
    NVIC_DisableIRQ(USBC_IRQn);

    if (ep->num)
    {
        hpcd->Instance->index = ep->num;
        if (ep->is_in)
        {
            hpcd->Instance->intrtx &= ~(1 << ep->num);
            hpcd->Instance->txmaxp = 0;
            // The programming guide says that we must not clear the DMAMODE bit before DMAENAB,
            // so we only clear it in the sconed write...
            hpcd->Instance->csr0_txcsr = (USB_TXCSR_DMAMODE | USB_TXCSR_FLUSHFIFO);
            hpcd->Instance->csr0_txcsr = USB_TXCSR_FLUSHFIFO;
        }
        else
        {
            hpcd->Instance->intrrx &= ~(1 << ep->num);
            hpcd->Instance->rxmaxp = 0;
            hpcd->Instance->rxcsr = USB_RXCSR_FLUSHFIFO;
            hpcd->Instance->rxcsr = USB_RXCSR_FLUSHFIFO; // Why twice ???
        }
    }
    NVIC_EnableIRQ(USBC_IRQn);
    __HAL_UNLOCK(hpcd);
    return HAL_OK;
}


/**
  * @brief  Receive an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @retval received packet size.
  */
uint32_t HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf)
{
    PCD_EPTypeDef *ep;
    uint32_t r = 0;

    ep = &hpcd->OUT_ep[ep_addr & 0x7FU];

    if (ep->num)
    {
        __IO struct musb_epN_regs *epn;
        uint16_t csr;

        epn = &(hpcd->Instance->ep[ep->num].epN);
        csr = epn->rxcsr;
        if (csr & USB_RXCSR_P_SENDSTALL)
        {
            HAL_DBG_printf("ep stalling, rxcsr %03x\n", csr);
        }
        else
        {
            r = ep->xfer_count;
            //HAL_DBG_printf("EP %d Rx: %p, %d, csr=%x\r\n", ep->num, pBuf, r, csr);
            //HAL_DBG_print_data((char *)pBuf, 0, r);
            if (csr & USB_RXCSR_RXPKTRDY)
            {
#if !defined(USB_RX_DMA_ENABLED)
                __IO uint8_t *fifox = (__IO uint8_t *) & (hpcd->Instance->fifox[ep->num]);
                int i;
                r = epn->rxcount;
                for (i = 0; i < r; i++)
                    *(pBuf + i) = *fifox;
#endif
            }
            ep->xfer_count = 0;
        }
    }
    else
    {
        __IO struct musb_ep0_regs *ep0;
        int i;
        uint16_t csr;
        __IO uint8_t *fifox = (__IO uint8_t *) & (hpcd->Instance->fifox[ep->num]);

        ep0 = &(hpcd->Instance->ep[ep->num].ep0);
        r = ep0->count0;
        if (r > ep->xfer_len)
            r = ep->xfer_len;
        for (i = 0; i < (int)r; i++)
            *(pBuf + i) = *fifox;
        csr = USB_CSR0_P_SVDRXPKTRDY;
        if (r < ep->maxpacket)
        {
            ep0_state_change(hpcd, HAL_PCD_EP0_STATUSIN);
            csr |= USB_CSR0_P_DATAEND;
        }
        ep0->csr0 = csr;
        HAL_DBG_printf("EP0 Rx:%d\r\n", r);
        //HAL_DBG_print_data((char *)pBuf, 0, r);
    }

    return r;
}

/**
  * @brief  Receive an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @param  len amount of data to be received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Prepare_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{

    PCD_EPTypeDef *ep;
    uint16_t csr;

    ep = &hpcd->OUT_ep[ep_addr & 0x7FU];

    /*setup and start the Xfer */
    ep->xfer_buff = pBuf;
    ep->xfer_len = len;
    ep->is_in = 0U;
    ep->num = ep_addr & 0x7FU;

    if (ep->num)
    {
        __IO struct musb_epN_regs *epn = &(hpcd->Instance->ep[ep->num].epN);

        HAL_DBG_printf("Prepare_read on pipe %d:%p,len %d, is_in=%d\n", ep->num, pBuf, len, ep->is_in);
        csr = epn->rxcsr;
        /*Delay ACK here to solve the BUG of 58 transferring large files*/
        if (HAL_PCD_Get_RxbuffControl(ep->num)) //Control the receiving data flow of the channel 1:Control the receiving data flow of the channel;0 Stop receiving
            epn->rxcsr = 0;
    }
    else
        ep0_state_change(hpcd, HAL_PCD_EP0_RX);

    return HAL_OK;
}

/**
  * @brief  Get Received Data Size
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval Data Size
  */
uint16_t HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    uint16_t r = 0;

    NVIC_DisableIRQ(USBC_IRQn);
    hpcd->Instance->index = ep_addr & 0x7FU;
    r = hpcd->Instance->rxcount;
    NVIC_EnableIRQ(USBC_IRQn);
    return r;
}

/**
  * @brief  Send an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the transmission buffer
  * @param  len amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
    PCD_EPTypeDef *ep;

    ep = &hpcd->IN_ep[ep_addr & 0x7FU];

    /*setup and start the Xfer */
    ep->xfer_buff = pBuf;
    ep->xfer_len = len;
    ep->is_in = 1U;
    ep->num = ep_addr & 0x7FU;
    ep->xfer_count = len;

    if (ep->num)
    {
        uint16_t csr;
        __IO struct musb_epN_regs *epn = &(hpcd->Instance->ep[ep->num].epN);

        csr = epn->txcsr;
        HAL_DBG_printf("tx ep %d: csr=0x%x, %d\n", ep->num, csr, len);
#if USB_TX_DMA_ENABLED
        if (IS_DCACHED_RAM((uint32_t)pBuf))
            mpu_dcache_clean((void *)pBuf, len);
        s_tx_packet_max[ep->num] = (len > ep->maxpacket) ? 1 : 0;

        struct musb_dma_regs *dma = (struct musb_dma_regs *) & (hpcd->Instance->dma[ep->num]);
        dma->addr = (REG32)pBuf;
        dma->count = len;
        dma->cntl = //(1 << USB_DMACTRL_ENABLE_SHIFT)   |
            (1 << USB_DMACTRL_TRANSMIT_SHIFT) |
            //(0 << USB_DMACTRL_MODE1_SHIFT)    |
            (ep->num << USB_DMACTRL_ENDPOINT_SHIFT) |
            (1 << USB_DMACTRL_IRQENABLE_SHIFT);
        if (s_tx_packet_max[ep->num])
        {
            dma->cntl |= (1 << USB_DMACTRL_ENABLE_SHIFT) | (1 << USB_DMACTRL_MODE1_SHIFT);
            csr |= USB_TXCSR_DMAENAB | USB_TXCSR_AUTOSET;
        }
        else
        {
            dma->cntl |= (1 << USB_DMACTRL_ENABLE_SHIFT) | (0 << USB_DMACTRL_MODE1_SHIFT);
        }
        epn->txcsr = csr;

#else
        __IO uint8_t *fifox = (__IO uint8_t *) & (hpcd->Instance->fifox[ep->num]);

        if (csr & USB_TXCSR_TXPKTRDY)
        {
            //LOG_W("old packet still ready, txcsr %03x\n", csr);
            while (epn->txcsr & USB_TXCSR_TXPKTRDY)
            {
                // Wait for the previous packet to be sent
            }
        }
        if (csr & USB_TXCSR_P_SENDSTALL)
        {
            //LOG_W("ep stalling, txcsr %03x\n", csr);
        }
        uint32_t max_len = (len > ep->maxpacket) ? ep->maxpacket : len;
        ep->xfer_count = max_len;
        for (int i = 0; i < max_len; i++) // REVISIT: Use 16bits/32bits FIFO to speed up
            *fifox = *(pBuf + i);
        csr &= ~USB_TXCSR_P_UNDERRUN;
        csr |= USB_TXCSR_TXPKTRDY;

        epn->txcsr = csr;
#endif
        //HAL_DBG_print_data((char *)pBuf, 0, len);
    }
    else
    {
        int i;
        uint16_t csr;
        __IO uint8_t *fifox = (__IO uint8_t *) & (hpcd->Instance->fifox[ep->num]);
        __IO struct musb_ep0_regs *ep0 = &(hpcd->Instance->ep[ep->num].ep0);

        for (i = 0; i < (int)len; i++) // REVISIT: Use 16bits/32bits FIFO to speed up
            *fifox = *(pBuf + i);
        csr = USB_CSR0_TXPKTRDY;

        HAL_DBG_printf("tx ep0: csr=0x%x, %d\n", csr, len);
        //HAL_DBG_print_data((char *)pBuf, 0, len);

        if (len && len < ep->maxpacket)
        {
            csr |= USB_CSR0_P_DATAEND;
            ep0_state_change(hpcd, HAL_PCD_EP0_STATUSOUT);
        }
        ep0->csr0 = csr;

    }
    return HAL_OK;
}

static void musb_stall(__IO struct musb_epN_regs *epn, int value, int is_in)
{
    uint16_t csr;

    if (is_in)
    {
        csr = epn->txcsr;
        csr |= USB_TXCSR_P_WZC_BITS | USB_TXCSR_CLRDATATOG;
        if (value)
            csr |= USB_TXCSR_P_SENDSTALL;
        else
            csr &= ~(USB_TXCSR_P_SENDSTALL | USB_TXCSR_P_SENTSTALL);
        csr &= ~USB_TXCSR_TXPKTRDY;
        epn->txcsr = csr;
    }
    else
    {
        csr = epn->rxcsr;
        csr |= USB_RXCSR_P_WZC_BITS | USB_RXCSR_FLUSHFIFO | USB_RXCSR_CLRDATATOG;
        if (value)
            csr |= USB_RXCSR_P_SENDSTALL;
        else
            csr &= ~(USB_RXCSR_P_SENDSTALL | USB_RXCSR_P_SENTSTALL);
        epn->rxcsr = csr;
    }

}
/**
  * @brief  Set a STALL condition over an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    PCD_EPTypeDef *ep;

    if ((0x80U & ep_addr) == 0x80U)
        ep = &hpcd->IN_ep[ep_addr & 0x7FU];
    else
        ep = &hpcd->OUT_ep[ep_addr];

    ep->is_stall = 1;
    ep->num   = ep_addr & 0x7FU;
    ep->is_in = ((ep_addr & 0x80U) == 0x80U);

    __HAL_LOCK(hpcd);
    if (ep->num)
    {
        __IO struct musb_epN_regs *epn = &(hpcd->Instance->ep[ep->num].epN);
        musb_stall(epn, 1, ep->is_in);
    }
    else
    {
        __IO struct musb_ep0_regs *ep0 = &(hpcd->Instance->ep[ep->num].ep0);
        ep0->csr0 |= USB_CSR0_P_SENDSTALL;
    }
    __HAL_UNLOCK(hpcd);
    //ep->is_stall = 0;

    return HAL_OK;
}

/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    PCD_EPTypeDef *ep;

    if ((0x80U & ep_addr) == 0x80U)
        ep = &hpcd->IN_ep[ep_addr & 0x7FU];
    else
        ep = &hpcd->OUT_ep[ep_addr];

    ep->is_stall = 0U;
    ep->num   = ep_addr & 0x7FU;
    ep->is_in = ((ep_addr & 0x80U) == 0x80U);

    __HAL_LOCK(hpcd);
    if (ep->num)
    {
        __IO struct musb_epN_regs *epn = &(hpcd->Instance->ep[ep->num].epN);
        musb_stall(epn, 0, ep->is_in);
    }
    else
    {
        __IO struct musb_ep0_regs *ep0 = &(hpcd->Instance->ep[ep->num].ep0);
        ep0->csr0 &= ~USB_CSR0_P_SENDSTALL;
    }
    __HAL_UNLOCK(hpcd);

    return HAL_OK;
}

/**
  * @brief  Flush an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    PCD_EPTypeDef *ep;
    uint16_t csr;

    if ((0x80U & ep_addr) == 0x80U)
        ep = &hpcd->IN_ep[ep_addr & 0x7FU];
    else
        ep = &hpcd->OUT_ep[ep_addr];
    __HAL_LOCK(hpcd);
    if (ep->num)
    {
        __IO struct musb_epN_regs *epn = &(hpcd->Instance->ep[ep->num].epN);
        hpcd->Instance->intrtx &= ~(1 << ep->num);
        if (ep->is_in)
        {
            csr = epn->txcsr;
            if (csr & USB_TXCSR_FIFONOTEMPTY)
            {
                csr |= USB_TXCSR_FLUSHFIFO | USB_TXCSR_P_WZC_BITS;
                csr &= ~USB_TXCSR_TXPKTRDY;
                /*
                    Setting both TXPKTRDY and FLUSHFIFO makes controller
                    to interrupt current FIFP loading, but not flushing the already loaded ones.
                */
                epn->txcsr = csr;
                /*  REVISIT: may be inapproperiate w/o FIFONOTEMPTY... */
                epn->txcsr = csr;
            }
        }
        else
        {
            csr = epn->rxcsr;
            csr |= USB_RXCSR_P_WZC_BITS | USB_RXCSR_FLUSHFIFO;
            epn->rxcsr = csr;
            epn->rxcsr = csr;
        }
        hpcd->Instance->intrtx |= ~(1 << ep->num);
    }
    __HAL_UNLOCK(hpcd);

    return HAL_OK;
}


/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group4 Peripheral State functions
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
  * @brief  Return the PCD state
  * @param  hpcd PCD handle
  * @retval HAL state
  */
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef *hpcd)
{
    return hpcd->State;
}

HAL_StatusTypeDef HAL_PCD_TestMode(PCD_HandleTypeDef *hpcd, uint16_t tm, uint8_t *data, uint8_t len)
{
    tm >>= 8;   // The most significant byte is the test mode

    switch (tm)
    {
    case 1:     // Test_J
        hpcd->Instance->testmode = USB_TEST_J;
        break;
    case 2:     // Test_K
        hpcd->Instance->testmode = USB_TEST_K;
        break;
    case 3:     // Test_SE0_NAK
        hpcd->Instance->testmode = USB_TEST_SE0_NAK;
        break;
    case 4:      // Test_Packet
    {
        __IO uint8_t *fifo = (__IO uint8_t *) & (hpcd->Instance->fifox[0]);
        __IO struct musb_ep0_regs *ep0 = &(hpcd->Instance->ep[0].ep0);
        int i;

        for (i = 0; i < len; i++) // REVISIT: Use 16bits/32bits FIFO to speed up
            *fifo = *(data + i);
        hpcd->Instance->testmode = USB_TEST_PACKET;
        ep0->csr0 |= USB_CSR0_TXPKTRDY;
        break;
    }
    case 5:     // Test_Force_Enable.
        hpcd->Instance->testmode = USB_TEST_FORCE_FS;
        break;
    default:    // Reserved
        ;
    }
    return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup PCD_Private_Functions
  * @{
  */

static const char *ep0_state_str(uint8_t state)
{
    static const char *states_str[] =
    {
        "IDLE",
        "SETUP",
        "TX",
        "RX",
        "STATUSIN",
        "STATUSOUT",
        "ACKWAIT",
    };
    return states_str[state];
}


/*
 * Handle all control requests with no DATA stage, including standard
 * requests such as:
 * USB_REQ_SET_CONFIGURATION, USB_REQ_SET_INTERFACE, unrecognized
 *  always delegated to the gadget driver
 * USB_REQ_SET_ADDRESS, USB_REQ_CLEAR_FEATURE, USB_REQ_SET_FEATURE
 *  always handled here, except for class/vendor/... features
 *
 * Context:  caller holds controller lock
 */
static int
service_zero_data_request(PCD_HandleTypeDef *hpcd,  struct urequest *req)
{
    int handled = -1;
    const uint8_t recip = req->request_type & USB_REQ_TYPE_RECIPIENT_MASK;

    /* the gadget driver handles everything except what we MUST handle */
    if ((req->request_type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
        switch (req->bRequest)
        {
        case USB_REQ_SET_ADDRESS:
        case USB_REQ_SET_CONFIGURATION:
            /* change it after the status stage */
            handled = 1;
            break;

        case USB_REQ_CLEAR_FEATURE:
            switch (recip)
            {
            case USB_REQ_TYPE_DEVICE:
                if (req->wValue != USB_REMOTE_WAKEUP)
                    break;
                handled = 1;
                break;
            case USB_REQ_TYPE_INTERFACE:
                break;
            case USB_REQ_TYPE_ENDPOINT:
            {
                const uint8_t       epnum = req->wIndex & 0x0f;
                uint16_t            csr;
                __IO struct musb_epN_regs *epn = &(hpcd->Instance->ep[epnum].epN);

                if (epnum == 0 || req->wValue != USB_FEATURE_ENDPOINT_HALT)
                    break;
                handled = 1;

                if (req->wIndex & USB_DIR_IN)
                {
                    csr  = epn->txcsr;
                    csr |= USB_TXCSR_CLRDATATOG | USB_TXCSR_P_WZC_BITS;
                    csr &= ~(USB_TXCSR_P_SENDSTALL | USB_TXCSR_P_SENTSTALL | USB_TXCSR_TXPKTRDY);
                    epn->txcsr = csr;
                }
                else
                {
                    csr  = epn->rxcsr;
                    csr |= USB_RXCSR_CLRDATATOG | USB_RXCSR_P_WZC_BITS;
                    csr &= ~(USB_RXCSR_P_SENDSTALL | USB_RXCSR_P_SENTSTALL);
                    epn->rxcsr = csr;
                }
                break;
            }
            default:
                /* class, vendor, etc ... delegate */
                handled = 0;
                break;
            }
            break;

        case USB_REQ_SET_FEATURE:
            switch (recip)
            {
            case USB_REQ_TYPE_DEVICE:
                handled = 1;
                switch (req->wValue)
                {
                case USB_REMOTE_WAKEUP:
                    //musb->may_wakeup = 1;
                    break;
                default:
                    handled = -1;
                    break;
                }
                break;
            case USB_REQ_TYPE_INTERFACE:
                break;

            case USB_REQ_TYPE_ENDPOINT:
            {
                const uint8_t       epnum = req->wIndex & 0x0f;
                uint16_t            csr;
                __IO struct musb_epN_regs *epn = &(hpcd->Instance->ep[epnum].epN);

                if (epnum == 0 || req->wValue   != USB_FEATURE_ENDPOINT_HALT)
                    break;

                if (req->wIndex & USB_DIR_IN)
                {
                    csr = epn->txcsr;
                    if (csr & USB_TXCSR_FIFONOTEMPTY)
                        csr |= USB_TXCSR_FLUSHFIFO;
                    csr |= (USB_TXCSR_P_SENDSTALL | USB_TXCSR_CLRDATATOG | USB_TXCSR_P_WZC_BITS);
                    epn->txcsr = csr;
                }
                else
                {
                    csr = epn->rxcsr;
                    csr |= (USB_RXCSR_P_SENDSTALL | USB_RXCSR_FLUSHFIFO | USB_RXCSR_CLRDATATOG | USB_RXCSR_P_WZC_BITS);
                    epn->rxcsr = csr;
                }
                handled = 1;
            }
            break;

            default:
                /* class, vendor, etc ... delegate */
                handled = 0;
                break;
            }
            break;
            handled = 1;
            break;
        default:
            handled = 0;
        }
    }
    else
        handled = 0;
    return handled;
}

/**
  * @brief  This function handles PCD Endpoint 0 interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP0_ISR_Handler(PCD_HandleTypeDef *hpcd)
{
    uint16_t csr;
    uint16_t len;

    __IO struct musb_ep0_regs *ep0 = &(hpcd->Instance->ep[0].ep0);
    __IO uint8_t *fifox = (__IO uint8_t *) & (hpcd->Instance->fifox[0]);

    csr = ep0->csr0;
    len = ep0->count0;

    HAL_DBG_printf("ep0 ISR: %s, csr=0x%x\r\n", ep0_state_str(hpcd->ep0_state), csr);
    /*
        if DATAEND is set, we should not call the callback,
        hence the status stage is not complete.
    */
    if (csr & USB_CSR0_P_DATAEND)
    {
        csr |= (~USB_CSR0_P_DATAEND);
        //return HAL_OK;//Will cause the failure of sending character descriptors
    }

    /*
        I sent a stall, need to acknowledge it now..
    */
    if (csr & USB_CSR0_P_SENTSTALL)
    {
        ep0->csr0 = (csr & (~USB_CSR0_P_SENTSTALL));
        ep0_state_change(hpcd, HAL_PCD_EP0_IDLE);
        csr = ep0->csr0;
    }

    /* Request ended "early". */
    if (csr & USB_CSR0_P_SETUPEND)
    {
        ep0->csr0 = USB_CSR0_P_SETUPEND;
        switch (hpcd->ep0_state)
        {
        case HAL_PCD_EP0_TX:
            ep0_state_change(hpcd, HAL_PCD_EP0_STATUSOUT);
            break;
        case HAL_PCD_EP0_RX:
            ep0_state_change(hpcd, HAL_PCD_EP0_STATUSIN);
        case HAL_PCD_EP0_IDLE:
            break;
        default:
            HAL_DBG_printf("SetupEnd case in wrong state %s\r\n", ep0_state_str(hpcd->ep0_state));
            break;
        }
        csr = ep0->csr0;
    }

    /*
        Docs from Mentor only describe tx,rx, and idle/setup states.
        We need to handle nuances around status stage, and also the
        case where status and setup stages come back-to-back..
    */
    switch (hpcd->ep0_state)
    {
    case HAL_PCD_EP0_TX:
        /*ireq on clearing txpktrdy*/
        if ((csr & USB_CSR0_TXPKTRDY) == 0)
            HAL_PCD_DataInStageCallback(hpcd, 0);
        break;
    case HAL_PCD_EP0_RX:
        /* irq on set rxpktrdy*/
        if (csr & USB_CSR0_RXPKTRDY)
            HAL_PCD_DataOutStageCallback(hpcd, 0);
        break;
    case HAL_PCD_EP0_STATUSIN:
    // Set USB address ???
    case HAL_PCD_EP0_STATUSOUT:
    case HAL_PCD_EP0_ACKWAIT:
        if (csr & USB_CSR0_RXPKTRDY)
            goto setup;
        ep0_state_change(hpcd, HAL_PCD_EP0_IDLE);
        break;

    case HAL_PCD_EP0_IDLE:
        ep0_state_change(hpcd, HAL_PCD_EP0_SETUP);
    /*FALL THROUGH*/
    case HAL_PCD_EP0_SETUP:
setup:
        if (csr & USB_CSR0_RXPKTRDY)
        {
            uint8_t *p = (uint8_t *)hpcd->Setup;
            struct urequest *req = (struct urequest *)p;
            int i;


            if (len > 16 * sizeof(uint32_t))
            {
                len = 8;
            }
            for (i = 0; i < len; i++)
                *(p + i) = *fifox;
            HAL_DBG_printf("Setup rx:%d", len);
            //HAL_DBG_print_data((char *)p, 0, len);
            if (len)
            {
                ep0->csr0 = USB_CSR0_P_SVDSETUPEND;
                while ((ep0->csr0 & USB_CSR0_P_SETUPEND) != 0);

                hpcd->ackpend = USB_CSR0_P_SVDRXPKTRDY | USB_CSR0_FLUSHFIFO;
                if (req->wLength == 0)
                {
                    if (req->request_type & USB_REQ_TYPE_DIR_IN)
                        hpcd->ackpend |= USB_CSR0_TXPKTRDY;
                    ep0_state_change(hpcd, HAL_PCD_EP0_ACKWAIT);
                }
                else if (req->request_type & USB_REQ_TYPE_DIR_IN)
                {
                    ep0_state_change(hpcd, HAL_PCD_EP0_TX);
                    ep0->csr0 = USB_CSR0_P_SVDRXPKTRDY | USB_CSR0_FLUSHFIFO;
                    while ((ep0->csr0 & USB_CSR0_RXPKTRDY) != 0);
                    hpcd->ackpend = 0;
                }
                else
                    ep0_state_change(hpcd, HAL_PCD_EP0_RX);
                switch (hpcd->ep0_state)
                {
                case HAL_PCD_EP0_ACKWAIT:
                {
                    int handled = service_zero_data_request(hpcd, req);
                    hpcd->ackpend |= USB_CSR0_P_DATAEND;

                    HAL_DBG_printf("Zero data handled %d\r\n", handled);
                    if (handled > 0)
                    {
                        ep0_state_change(hpcd, HAL_PCD_EP0_STATUSIN);
                    }
                    break;
                }
                case HAL_PCD_EP0_TX:
                {
                    /*TODO: validate this*/
                    break;
                }
                default:
                    break;
                }
                ep0->csr0 = hpcd->ackpend;
                hpcd->ackpend = 0;
                HAL_PCD_SetupStageCallback(hpcd);
            }
        }
        break;
    default:
        break;
    }
    return HAL_OK;
}

static int musbd_stage0_irq(PCD_HandleTypeDef *hpcd, uint8_t int_usb)
{
    uint8_t devctl = hpcd->Instance->devctl;
    int r = 0;

    if (devctl & USB_DEVCTL_HM)     // No host mode supported yet.
        HAL_ASSERT(0);

    if (int_usb & USB_INTR_RESUME)
    {
        switch (hpcd->phy_state)
        {
        case OTG_STATE_B_WAIT_ACON:
        case OTG_STATE_B_PERIPHERALS:
            if ((devctl & USB_DEVCTL_VBUS) != (3 << USB_DEVCTL_VBUS_SHIFT))
            {
                hpcd->Instance->intrusb |= USB_INTR_DISCONNECT;
                hpcd->Instance->intrusb &= ~USB_INTR_SUSPEND;
            }
            // REVISIT: Resume???
            break;
        case OTG_STATE_B_IDLE:
            hpcd->Instance->intrusb &= ~USB_INTR_SUSPEND;
            break;
        default:
            HAL_ASSERT(0);        // Incorrect resume.
        }

        if (int_usb & USB_INTR_SUSPEND)
            int_usb &= ~USB_INTR_SUSPEND;
    }

    if (int_usb & USB_INTR_SESSREQ)
    {
        if (((devctl & USB_DEVCTL_VBUS) == USB_DEVCTL_VBUS) && (devctl & USB_DEVCTL_BDEVICE))
            return 1;

        // REVISIT: A device ONLY????
        hpcd->Instance->devctl = USB_DEVCTL_SESSION;
        ep0_state_change(hpcd, HAL_PCD_EP0_IDLE);
        r = 1;
    }

    if (int_usb & USB_INTR_VBUSERROR)   // This is for A device only.
    {
        HAL_ASSERT(0);
        r = 1;
    }

    if (int_usb & USB_INTR_RESET)
    {
        r = 1;
        switch (hpcd->phy_state)
        {
        case OTG_STATE_B_WAIT_ACON:
        case OTG_STATE_B_PERIPHERALS:
        case OTG_STATE_B_IDLE:
            hpcd->phy_state = OTG_STATE_B_PERIPHERALS;
            ep0_state_change(hpcd, HAL_PCD_EP0_IDLE);
            HAL_PCD_ResetCallback(hpcd);
            break;
        default:
            HAL_ASSERT(0);
        }
        if (int_usb & USB_INTR_SUSPEND)
            int_usb &= ~USB_INTR_SUSPEND;
    }

    if (int_usb & USB_INTR_SUSPEND)
    {
        r = 1;
        switch (hpcd->phy_state)
        {
        case OTG_STATE_B_PERIPHERALS:
            // Go suspend;
            hpcd->phy_state = OTG_STATE_B_WAIT_ACON;
            break;
        case OTG_STATE_B_IDLE:
            break;
        default:
            HAL_DBG_printf("phy_state=%d\r\n", hpcd->phy_state);
            HAL_ASSERT(0);        // Should not happen
            break;
        }
    }
    if (int_usb & USB_INTR_CONNECT)
    {
        r = 1;
        ep0_state_change(hpcd, HAL_PCD_EP0_IDLE);
        HAL_PCD_ConnectCallback(hpcd);
        if (int_usb & USB_INTR_SUSPEND)
            int_usb &= ~USB_INTR_SUSPEND;
    }
    if (int_usb & USB_INTR_DISCONNECT)
    {
        r = 1;
        HAL_PCD_DisconnectCallback(hpcd);
        hpcd->Instance->devctl = USB_DEVCTL_SESSION;
        hpcd->phy_state = OTG_STATE_B_IDLE;
    }

    return r;
}

/**
  * @brief  This function handles PCD Endpoint interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd)
{
    uint32_t reg;
    int ep_num;
    uint8_t int_usb = hpcd->Instance->intrusb;
    uint16_t int_tx = hpcd->Instance->intrtx;
    uint16_t int_rx = hpcd->Instance->intrrx;
    uint32_t dmaintr = hpcd->Instance->dmaintr;

    HAL_DBG_printf("USB interrupt usb=%x, tx=%d, rx=%d, usbe=%x, dma_intr=%x\r\n", int_usb, int_tx, int_rx, hpcd->Instance->intrusbe, dmaintr);

    if (int_usb)
        musbd_stage0_irq(hpcd, int_usb);

    if (int_tx && (int_tx & 1))
        PCD_EP0_ISR_Handler(hpcd);

#if defined(USB_TX_DMA_ENABLED)||defined(USB_RX_DMA_ENABLED)
    reg = (dmaintr >> 1);
    ep_num = 1;
    while (reg)
    {
        if (reg & 1)
        {
            PCD_EPTypeDef *ep;
            __IO struct musb_epN_regs *epn = &(hpcd->Instance->ep[ep_num].epN);

            if (hpcd->IN_ep[ep_num].maxpacket)
                ep = &hpcd->IN_ep[ep_num];
            else
                ep = &hpcd->OUT_ep[ep_num];

            HAL_DBG_printf("DMA Done on EP: %d, is_in=%d, dmaintr=%x\n", ep->num, ep->is_in, dmaintr);

            if (ep->is_in)
            {
                //TX
                if (0 == s_tx_packet_max[ep_num])
                {
                    uint16_t csr = epn->txcsr;
                    csr &= ~USB_TXCSR_P_UNDERRUN;
                    //csr |= USB_TXCSR_AUTOSET;
                    if (ep->xfer_len <= ep->maxpacket)
                        csr |= USB_TXCSR_TXPKTRDY;
                    epn->txcsr = csr;
                    /*When occasional TX occurs, there will be no TX interruption, so TX callback can be performed directly here*/
                    //HAL_PCD_DataInStageCallback(hpcd, ep_num);
                }
                else
                {
                    HAL_PCD_DataInStageCallback(hpcd, ep_num);
                }
            }
            else
            {
                HAL_PCD_DataOutStageCallback(hpcd, ep_num); //RX
            }

        }
        ep_num++;
        reg >>= 1;
    }
#endif

    reg = (int_rx >> 1);
    ep_num = 1;
    while (reg)
    {
        if (reg & 1)
        {
            PCD_EPTypeDef *ep = &hpcd->OUT_ep[ep_num];
            __IO struct musb_epN_regs *epn = &(hpcd->Instance->ep[ep->num].epN);
#if USB_RX_DMA_ENABLED

            struct musb_dma_regs *dma = (struct musb_dma_regs *) & (hpcd->Instance->dma[ep->num]);
            HAL_DBG_printf("DMA RX pipe=%d, len=%d\n", ep->num,  epn->rxcount);
            if (epn->rxcount)
            {
                if (IS_DCACHED_RAM(ep->xfer_buff))
                    SCB_InvalidateDCache_by_Addr(ep->xfer_buff, epn->rxcount);
                dma->addr = (REG32)ep->xfer_buff;
                dma->count = epn->rxcount;
                ep->xfer_count = dma->count;
                epn->rxcount = 0;
                dma->cntl = (1 << USB_DMACTRL_ENABLE_SHIFT)   |
                            (0 << USB_DMACTRL_TRANSMIT_SHIFT) |
                            (0 << USB_DMACTRL_MODE1_SHIFT)    |
                            (ep->num << USB_DMACTRL_ENDPOINT_SHIFT) |
                            (1 << USB_DMACTRL_IRQENABLE_SHIFT);
            }
            else
            {
                ep->xfer_count = epn->rxcount;
                HAL_PCD_DataOutStageCallback(hpcd, ep_num);
            }
#else
            ep->xfer_count = epn->rxcount;
            HAL_PCD_DataOutStageCallback(hpcd, ep_num);
#endif

        }
        ep_num++;
        reg >>= 1;
    }

    reg = (int_tx >> 1);
    ep_num = 1;
    while (reg)
    {
        if ((reg & 1) && (0 == s_tx_packet_max[ep_num]))
            HAL_PCD_DataInStageCallback(hpcd, ep_num);
        ep_num++;
        reg >>= 1;
    }
    __DSB();
    return HAL_OK;
}


/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_PCD_MODULE_ENABLED */

/**
  * @}
  */