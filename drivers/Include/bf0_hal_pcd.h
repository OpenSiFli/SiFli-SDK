/**
  ******************************************************************************
  * @file   bf0_hal_pcd.h
  * @author Sifli software development team
  * @brief   Header file of PCD HAL module.
  * @attention
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_PCD_H
#define __BF0_HAL_PCD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"


typedef USBC_X_Typedef PCD_TypeDef;


/** @defgroup USB_LL_EP_Type USB Low Layer EP Type
  * @ingroup PCD
  * @{
  */
#define EP_TYPE_CTRL    0x14U
#define EP_TYPE_BULKOUT 0x12U
#define EP_TYPE_BULKIN  0x11U
/**
  * @}
  */


/** @addtogroup PCD USB Device
  * @ingroup BF0_HAL_Driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PCD_Exported_Types PCD Exported Types
  * @{
  */

/**
  * @brief  PCD State structure definition
  */
typedef enum
{
    HAL_PCD_STATE_RESET   = 0x00U,
    HAL_PCD_STATE_READY   = 0x01U,
    HAL_PCD_STATE_ERROR   = 0x02U,
    HAL_PCD_STATE_BUSY    = 0x03U,
    HAL_PCD_STATE_TIMEOUT = 0x04U
} PCD_StateTypeDef;


/**
  * @brief  PCD PHY State
  */
typedef enum
{
    OTG_STATE_A_SUSPEND,
    OTG_STATE_A_HOST,
    OTG_STATE_A_WAIT_BCON,
    OTG_STATE_A_WAIT_VFALL,
    OTG_STATE_A_WAIT_VRISE,
    OTG_STATE_B_HOST,
    OTG_STATE_B_PERIPHERALS,
    OTG_STATE_B_WAIT_ACON,
    OTG_STATE_B_IDLE,
} PCD_PhyStateTypeDef;

/**
  * @brief  PCD Endpont 0 State structure definition
  */
typedef enum
{
    HAL_PCD_EP0_IDLE,       /*!< Idle, waiting for setup*/
    HAL_PCD_EP0_SETUP,      /*!< Received setup*/
    HAL_PCD_EP0_TX,         /*!< IN data*/
    HAL_PCD_EP0_RX,         /*!< OUT data*/
    HAL_PCD_EP0_STATUSIN,   /*!< (After OUT data) */
    HAL_PCD_EP0_STATUSOUT,  /*!< (After IN data) */
    HAL_PCD_EP0_ACKWAIT,    /*!< After zlp, before statusin*/
} PCD_EP0_StateTypeDef;

/**
  * @brief  PCD Initialization Structure definition
  */
typedef struct
{
    uint32_t dev_endpoints;               /*!< Device Endpoints number.
                                             This parameter depends on the used USB core.
                                             This parameter must be a number between Min_Data = 1 and Max_Data = 15 */

    uint32_t speed;                       /*!< USB Core speed.
                                             This parameter can be any value of @ref PCD_Core_Speed                 */

    uint32_t ep0_mps;                     /*!< Set the Endpoint 0 Max Packet size.
                                             This parameter can be any value of @ref PCD_EP0_MPS                    */

    uint32_t phy_itface;                  /*!< Select the used PHY interface.
                                             This parameter can be any value of @ref PCD_Core_PHY                   */

    uint32_t Sof_enable;                  /*!< Enable or disable the output of the SOF signal.
                                             This parameter can be set to ENABLE or DISABLE                         */

    uint32_t low_power_enable;            /*!< Enable or disable Low Power mode
                                             This parameter can be set to ENABLE or DISABLE                         */

    uint32_t lpm_enable;                  /*!< Enable or disable the Link Power Management .
                                             This parameter can be set to ENABLE or DISABLE                         */

    uint32_t battery_charging_enable;     /*!< Enable or disable Battery charging.
                                             This parameter can be set to ENABLE or DISABLE                         */

} PCD_InitTypeDef;

typedef struct
{
    uint8_t   num;            /*!< Endpoint number
                                This parameter must be a number between Min_Data = 1 and Max_Data = 15    */

    uint8_t   is_in;          /*!< Endpoint direction
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1     */

    uint8_t   is_stall;       /*!< Endpoint stall condition
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1     */

    uint8_t   type;           /*!< Endpoint type
                                 This parameter can be any value of @ref PCD_EP_Type                      */

    void *dma_cfg;            /*< DMA Config */
    uint32_t  pmaadress;
    uint32_t  maxpacket;      /*!< Endpoint Max packet size
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 64KB */

    uint8_t   *xfer_buff;     /*!< Pointer to transfer buffer                                               */

    uint32_t  xfer_len;       /*!< Current transfer length                                                  */

    uint32_t  xfer_count;     /*!< Partial transfer length in case of multi packet transfer                 */

} PCD_EPTypeDef;


/**
  * @brief  PCD Handle Structure definition
  */
typedef struct
{
    PCD_TypeDef             *Instance;      /*!< Register base address              */
    PCD_InitTypeDef         Init;           /*!< PCD required parameters            */
    __IO uint8_t            USB_Address;    /*!< USB Address                        */
    PCD_EPTypeDef           IN_ep[8];       /*!< IN endpoint parameters             */
    PCD_EPTypeDef           OUT_ep[8];      /*!< OUT endpoint parameters            */
    HAL_LockTypeDef         Lock;           /*!< PCD peripheral status              */
    __IO PCD_StateTypeDef   State;          /*!< PCD communication state            */
    uint32_t                Setup[12];      /*!< Setup packet buffer                */
    void                    *pData;         /*!< Pointer to upper stack Handler     */
    PCD_EP0_StateTypeDef    ep0_state;      /*!< EP0 state                          */
    PCD_PhyStateTypeDef     phy_state;      /*!< PHY state                          */
    uint16_t                ackpend;        /*!< Pending ACK, EP0 only              */
} PCD_HandleTypeDef;

/**
  * @}
  */



/* Exported constants --------------------------------------------------------*/
/** @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
  */

/** @defgroup PCD_Core_Speed PCD Core Speed
  * @{
  */
#define PCD_SPEED_HIGH               0 /* Not Supported */
#define PCD_SPEED_FULL               2
/**
  * @}
  */

/** @defgroup PCD_EP0_MPS PCD EP0 Max Packet Size
  * @{
  */
#define PCD_EP0_MPSIZE               16 /* Not Supported */
/**
  * @}
  */

/** @defgroup PCD_Core_PHY PCD Core PHY
* @{
*/
#define PCD_PHY_EMBEDDED             2
/**
  * @}
  */

#define USB_USBCFG_AVALID                   (1<<3)
#define USB_USBCFG_AVALID_DR                (1<<2)

// TODO: Check when have analog PHY
#define USB_ENABLE_PHY(hpcd) hpcd->Instance->usbcfg|=(USB_USBCFG_AVALID|USB_USBCFG_AVALID_DR)

// Each endpoint is single buffer (Not PINGPONG)
#define USB_DISABLE_DOUBLE_BUFFER(hpcd) { \
    hpcd->Instance->dpbrxdisl=0xFE; \
    hpcd->Instance->dpbtxdisl=0xFE; \
}



/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PCD_Exported_Functions PCD Exported Functions
  * @{
  */

/* Initialization/de-initialization functions  ********************************/
/** @addtogroup PCD_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  Initializes the PCD according to the specified
  *         parameters in the PCD_InitTypeDef and create the associated handle.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd);

/**
  * @brief  DeInitializes the PCD peripheral
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd);

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);

/**
  * @brief  DeInitializes PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);

/**
  * @brief  Enable/disable PCD.
  * @param  enable 1: enable, 0: disable
  * @retval None
  */
void HAL_PCD_Enable(uint8_t enable);

/**
  * @}
  */

/* I/O operation functions  ***************************************************/
/* Non-Blocking mode: Interrupt */
/** @addtogroup PCD_Exported_Functions_Group2 IO operation functions
  * @{
  */

/**
  * @brief  Start the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd);

/**
  * @brief  Stop the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd);

/**
  * @brief  This function handles PCD interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);

/**
  * @brief  Data out stage callbacks
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);

/**
  * @brief  Data IN stage callbacks
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);

/**
  * @brief  Setup stage callback
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);

/**
  * @brief  USB Start Of Frame callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);

/**
  * @brief  USB Reset callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);

/**
  * @brief  Suspend event callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);

/**
  * @brief  Resume event callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);

/**
  * @brief  Incomplete ISO OUT callbacks
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);

/**
  * @brief  Incomplete ISO IN  callbacks
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);

/**
  * @brief  Connection event callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);

/**
  * @brief  Disconnection event callbacks
  * @param  hpcd PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);
/**
  * @}
  */

/* Peripheral Control functions  **********************************************/
/** @addtogroup PCD_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */
/**
  * @brief  Set the USB Device address
  * @param  hpcd PCD handle
  * @param  address new device address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);

/**
  * @brief  Open and configure an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  ep_mps endpoint max packert size
  * @param  ep_type endpoint type
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);

/**
  * @brief  Deactivate an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);


/**
  * @brief  Prepare to receive an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @param  len amount of data to be received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Prepare_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);


/**
  * @brief  Receive an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @retval received packet size.
  */
uint32_t HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf);


/**
  * @brief  Send an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the transmission buffer
  * @param  len amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);

/**
  * @brief  Get Received Data Size
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval Data Size
  */
uint16_t          HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);

/**
  * @brief  Set a STALL condition over an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);


/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);

/**
  * @brief  Flush an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
/**
  * @}
  */

/* Peripheral State functions  ************************************************/
/** @addtogroup PCD_Exported_Functions_Group4 Peripheral State functions
  * @{
  */

/**
  * @brief  Return the PCD state
  * @param  hpcd PCD handle
  * @retval HAL state
  */
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef *hpcd);

/** @addtogroup PCDEx_Exported_Functions_Group1 Peripheral Control functions
  * @{
  */
HAL_StatusTypeDef HAL_PCDEx_PMAConfig(PCD_HandleTypeDef *hpcd,
                                      uint16_t ep_addr,
                                      void *dma_config);


/**
* @brief  Set test mode
* @param  hpcd PCD handle
* @param  tm test mode see USB 2.0 spec, section 9.4.9 wIndex
* @param  data used in TEST_PACKET
* @param  len length of data
* @retval HAL status
*/
HAL_StatusTypeDef HAL_PCD_TestMode(PCD_HandleTypeDef *hpcd, uint16_t tm, uint8_t *data, uint8_t len);

void HAL_PCD_Set_RxbuffControl(uint8_t ep_num, uint8_t flag);
void HAL_PCD_Set_RxscrACK(uint8_t epnum);

/**
  * @}
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup PCD_Private_Constants PCD Private Constants
  * @{
  */

/** @defgroup PCD_EP_Type PCD EP Type
  * @{
  */
#define PCD_EP_TYPE_CTRL                       0
#define PCD_EP_TYPE_ISOC                       1
#define PCD_EP_TYPE_BULK                       2
#define PCD_EP_TYPE_INTR                       3
/**
  * @}
  */

/** @defgroup PCD_ENDP PCD ENDP
  * @{
  */
#define PCD_ENDP0                              ((uint8_t)0U)
#define PCD_ENDP1                              ((uint8_t)1U)
#define PCD_ENDP2                              ((uint8_t)2U)
#define PCD_ENDP3                              ((uint8_t)3U)
#define PCD_ENDP4                              ((uint8_t)4U)
#define PCD_ENDP5                              ((uint8_t)5U)
#define PCD_ENDP6                              ((uint8_t)6U)
#define PCD_ENDP7                              ((uint8_t)7U)
/**
  * @}
  */

/**
  * @}
  */



/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif


#endif /* __BF0_HAL_PCD_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

