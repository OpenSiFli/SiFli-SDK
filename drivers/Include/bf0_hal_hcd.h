/**
  ******************************************************************************
  * @file   bf0_hal_hcd.h
  * @author Sifli software development team
  * @brief   Header file of USB HCD HAL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BF0_HAL_HCD_H
#define BF0_HAL_HCD_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup HCD HCD
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup HCD_Exported_Types HCD Exported Types
  * @{
  */

/** @defgroup HCD_Exported_Types_Group1 HCD State Structure definition
  * @{
  */
typedef enum
{
    HAL_HCD_STATE_RESET    = 0x00,
    HAL_HCD_STATE_READY    = 0x01,
    HAL_HCD_STATE_ERROR    = 0x02,
    HAL_HCD_STATE_BUSY     = 0x03,
    HAL_HCD_STATE_TIMEOUT  = 0x04
} HCD_StateTypeDef;


typedef enum
{
    HAL_HCD_EP0_IDLE,       /*!< Idle, waiting for setup*/
    HAL_HCD_EP0_SETUP,      /*!< Sent setup*/
    HAL_HCD_EP0_TX,         /*!< OUT data*/
    HAL_HCD_EP0_RX,         /*!< IN data*/
} HCD_EP0_StateTypeDef;


typedef enum
{
    USB_DEVICE_MODE  = 0,
    USB_HOST_MODE    = 1,
    USB_DRD_MODE     = 2
} USB_OTG_ModeTypeDef;

/**
  * @brief  URB States definition
  */
typedef enum
{
    URB_IDLE = 0,
    URB_DONE,
    URB_NOTREADY,
    URB_NYET,
    URB_ERROR,
    URB_STALL
} USB_OTG_URBStateTypeDef;

/**
  * @brief  Host channel States  definition
  */
typedef enum
{
    HC_IDLE = 0,
    HC_XFRC,
    HC_HALTED,
    HC_NAK,
    HC_NYET,
    HC_STALL,
    HC_XACTERR,
    HC_BBLERR,
    HC_DATATGLERR
} USB_OTG_HCStateTypeDef;

/**
  * @brief  USB Instance Initialization Structure definition
  */
typedef struct
{
    uint32_t dev_endpoints;           /*!< Device Endpoints number.
                                         This parameter depends on the used USB core.
                                         This parameter must be a number between Min_Data = 1 and Max_Data = 15 */

    uint32_t Host_channels;           /*!< Host Channels number.
                                         This parameter Depends on the used USB core.
                                         This parameter must be a number between Min_Data = 1 and Max_Data = 15 */

    uint32_t speed;                   /*!< USB Core speed.                                                        */

    uint32_t dma_enable;              /*!< Enable or disable of the USB embedded DMA used only for OTG HS.        */

    uint32_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */

    uint32_t phy_itface;              /*!< Select the used PHY interface.                                         */

    uint32_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */

    uint32_t low_power_enable;        /*!< Enable or disable the low power mode.                                  */

    uint32_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */

    uint32_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */

    uint32_t vbus_sensing_enable;     /*!< Enable or disable the VBUS Sensing feature.                            */

    uint32_t use_dedicated_ep1;       /*!< Enable or disable the use of the dedicated EP1 interrupt.              */

    uint32_t use_external_vbus;       /*!< Enable or disable the use of the external VBUS.                        */

} USB_OTG_CfgTypeDef;

typedef struct
{
    uint8_t   num;                  /*!< Endpoint number
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 15   */

    uint8_t   is_in;                /*!< Endpoint direction
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

    uint8_t   is_stall;             /*!< Endpoint stall condition
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

    uint8_t   type;                 /*!< Endpoint type                                                          */

    uint8_t   data_pid_start;       /*!< Initial data PID
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

    uint8_t   even_odd_frame;       /*!< IFrame parity
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

    uint16_t  tx_fifo_num;          /*!< Transmission FIFO number
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 15   */

    uint32_t  maxpacket;            /*!< Endpoint Max packet size
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 64KB */

    uint8_t   *xfer_buff;           /*!< Pointer to transfer buffer                                               */

    uint32_t  dma_addr;             /*!< 32 bits aligned transfer buffer address                                  */

    uint32_t  xfer_len;             /*!< Current transfer length                                                  */

    uint32_t  xfer_count;           /*!< Partial transfer length in case of multi packet transfer                 */
} USB_OTG_EPTypeDef;

typedef struct
{
    uint8_t   dev_addr;           /*!< USB device address.
                                     This parameter must be a number between Min_Data = 1 and Max_Data = 255    */

    uint8_t   ch_num;             /*!< Host channel number.
                                     This parameter must be a number between Min_Data = 1 and Max_Data = 15     */

    uint8_t   ep_num;             /*!< Endpoint number.
                                     This parameter must be a number between Min_Data = 1 and Max_Data = 15     */

    uint8_t   ep_is_in;           /*!< Endpoint direction
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 1      */

    uint8_t   speed;              /*!< USB Host speed.                                                          */

    uint8_t   do_ping;            /*!< Enable or disable the use of the PING protocol for HS mode.                */

    uint8_t   process_ping;       /*!< Execute the PING protocol for HS mode.                                     */

    uint8_t   ep_type;            /*!< Endpoint Type.                                                          */

    uint16_t  max_packet;         /*!< Endpoint Max packet size.
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 64KB   */

    uint8_t   data_pid;           /*!< Initial data PID.
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 1      */

    uint8_t   *xfer_buff;         /*!< Pointer to transfer buffer.                                                */

    uint32_t  XferSize;             /*!< OTG Channel transfer size.                                                   */

    uint32_t  xfer_len;           /*!< Current transfer length.                                                   */

    uint32_t  xfer_count;         /*!< Partial transfer length in case of multi packet transfer.                  */

    uint8_t   toggle_in;          /*!< IN transfer current toggle flag.
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 1      */

    uint8_t   toggle_out;         /*!< OUT transfer current toggle flag
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 1      */

    uint32_t  dma_addr;           /*!< 32 bits aligned transfer buffer address.                                   */

    uint32_t  ErrCnt;             /*!< Host channel error count.                                                  */

    USB_OTG_URBStateTypeDef urb_state;  /*!< URB state.
                                            This parameter can be any value of @ref USB_OTG_URBStateTypeDef */

    USB_OTG_HCStateTypeDef state;       /*!< Host Channel state.
                                            This parameter can be any value of @ref USB_OTG_HCStateTypeDef  */
} USB_OTG_HCTypeDef;

typedef USBC_X_Typedef HCD_TypeDef;
typedef USBC_X_Typedef USB_OTG_GlobalTypeDef;
typedef USB_OTG_CfgTypeDef      HCD_InitTypeDef;
typedef USB_OTG_HCTypeDef       HCD_HCTypeDef;
typedef USB_OTG_URBStateTypeDef HCD_URBStateTypeDef;
typedef USB_OTG_HCStateTypeDef  HCD_HCStateTypeDef;
/**
  * @}
  */

/** @defgroup HCD_Exported_Types_Group2 HCD Handle Structure definition
  * @{
  */
#if (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
typedef struct __HCD_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */
{
    HCD_TypeDef               *Instance;  /*!< Register base address    */
    HCD_InitTypeDef           Init;       /*!< HCD required parameters  */
    HCD_HCTypeDef             hc[16];     /*!< Host channels parameters */
    HAL_LockTypeDef           Lock;       /*!< HCD peripheral status    */
    __IO HCD_StateTypeDef     State;      /*!< HCD communication state  */
    __IO HCD_EP0_StateTypeDef ep0_state;  /*!< HCD EP0 state            */
    __IO  uint32_t            ErrorCode;  /*!< HCD Error code           */
    void                      *pData;     /*!< Pointer Stack Handler    */
    uint16_t                  epmask;     /*!< End point enabled        */
#if (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
    void (* SOFCallback)(struct __HCD_HandleTypeDef *hhcd);                               /*!< USB OTG HCD SOF callback                */
    void (* ConnectCallback)(struct __HCD_HandleTypeDef *hhcd);                           /*!< USB OTG HCD Connect callback            */
    void (* DisconnectCallback)(struct __HCD_HandleTypeDef *hhcd);                        /*!< USB OTG HCD Disconnect callback         */
    void (* PortEnabledCallback)(struct __HCD_HandleTypeDef *hhcd);                       /*!< USB OTG HCD Port Enable callback        */
    void (* PortDisabledCallback)(struct __HCD_HandleTypeDef *hhcd);                      /*!< USB OTG HCD Port Disable callback       */
    void (* HC_NotifyURBChangeCallback)(struct __HCD_HandleTypeDef *hhcd, uint8_t chnum,
                                        HCD_URBStateTypeDef urb_state);                   /*!< USB OTG HCD Host Channel Notify URB Change callback  */

    void (* MspInitCallback)(struct __HCD_HandleTypeDef *hhcd);                           /*!< USB OTG HCD Msp Init callback           */
    void (* MspDeInitCallback)(struct __HCD_HandleTypeDef *hhcd);                         /*!< USB OTG HCD Msp DeInit callback         */
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */
} HCD_HandleTypeDef;
/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup HCD_Exported_Constants HCD Exported Constants
  * @{
  */

/** @defgroup HCD_Speed HCD Speed
  * @{
  */
#define HCD_SPEED_HIGH               0
#define HCD_SPEED_FULL               1
#define HCD_SPEED_LOW                2

#define HCD_DEVICE_SPEED_HIGH               0U
#define HCD_DEVICE_SPEED_FULL               1U
#define HCD_DEVICE_SPEED_LOW                2U

/**
  * @}
  */

/** @defgroup HCD_PHY_Module HCD PHY Module
  * @{
  */
#define HCD_PHY_ULPI                 1U
#define HCD_PHY_EMBEDDED             2U
/**
  * @}
  */

/** @defgroup HCD_Error_Code_definition HCD Error Code definition
  * @brief  HCD Error Code definition
  * @{
  */
#if (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
#define  HAL_HCD_ERROR_INVALID_CALLBACK                        (0x00000010U)    /*!< Invalid Callback error  */
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup HCD_Exported_Macros HCD Exported Macros
  *  @brief macros to handle interrupts and specific clock configurations
  * @{
  */
#define __HAL_HCD_GET_FLAG(__HANDLE__, __INTERRUPT__)      ((USB_ReadInterrupts((__HANDLE__)->Instance) & (__INTERRUPT__)) == (__INTERRUPT__))
#define __HAL_HCD_CLEAR_FLAG(__HANDLE__, __INTERRUPT__)    (((__HANDLE__)->Instance->GINTSTS) = (__INTERRUPT__))
#define __HAL_HCD_IS_INVALID_INTERRUPT(__HANDLE__)         (USB_ReadInterrupts((__HANDLE__)->Instance) == 0U)

#define __HAL_HCD_CLEAR_HC_INT(chnum, __INTERRUPT__)  (USBx_HC(chnum)->HCINT = (__INTERRUPT__))
#define __HAL_HCD_MASK_HALT_HC_INT(chnum)             (USBx_HC(chnum)->HCINTMSK &= ~USB_OTG_HCINTMSK_CHHM)
#define __HAL_HCD_UNMASK_HALT_HC_INT(chnum)           (USBx_HC(chnum)->HCINTMSK |= USB_OTG_HCINTMSK_CHHM)
#define __HAL_HCD_MASK_ACK_HC_INT(chnum)              (USBx_HC(chnum)->HCINTMSK &= ~USB_OTG_HCINTMSK_ACKM)
#define __HAL_HCD_UNMASK_ACK_HC_INT(chnum)            (USBx_HC(chnum)->HCINTMSK |= USB_OTG_HCINTMSK_ACKM)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HCD_Exported_Functions HCD Exported Functions
  * @{
  */

/** @defgroup HCD_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
HAL_StatusTypeDef HAL_HCD_Init(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_DeInit(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd, uint8_t ch_num,
                                  uint8_t epnum, uint8_t dev_address,
                                  uint8_t speed, uint8_t ep_type, uint16_t mps);

HAL_StatusTypeDef HAL_HCD_HC_Halt(HCD_HandleTypeDef *hhcd, uint8_t ch_num);
void              HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd);
void              HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd);

#if (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
/** @defgroup HAL_HCD_Callback_ID_enumeration_definition HAL USB OTG HCD Callback ID enumeration definition
  * @brief  HAL USB OTG HCD Callback ID enumeration definition
  * @{
  */
typedef enum
{
    HAL_HCD_SOF_CB_ID            = 0x01,       /*!< USB HCD SOF callback ID           */
    HAL_HCD_CONNECT_CB_ID        = 0x02,       /*!< USB HCD Connect callback ID       */
    HAL_HCD_DISCONNECT_CB_ID     = 0x03,       /*!< USB HCD Disconnect callback ID    */
    HAL_HCD_PORT_ENABLED_CB_ID   = 0x04,       /*!< USB HCD Port Enable callback ID   */
    HAL_HCD_PORT_DISABLED_CB_ID  = 0x05,       /*!< USB HCD Port Disable callback ID  */

    HAL_HCD_MSPINIT_CB_ID        = 0x06,       /*!< USB HCD MspInit callback ID       */
    HAL_HCD_MSPDEINIT_CB_ID      = 0x07        /*!< USB HCD MspDeInit callback ID     */

} HAL_HCD_CallbackIDTypeDef;
/**
  * @}
  */

/** @defgroup HAL_HCD_Callback_pointer_definition HAL USB OTG HCD Callback pointer definition
  * @brief  HAL USB OTG HCD Callback pointer definition
  * @{
  */

typedef void (*pHCD_CallbackTypeDef)(HCD_HandleTypeDef *hhcd);                   /*!< pointer to a common USB OTG HCD callback function  */
typedef void (*pHCD_HC_NotifyURBChangeCallbackTypeDef)(HCD_HandleTypeDef *hhcd,
        uint8_t epnum,
        HCD_URBStateTypeDef urb_state);   /*!< pointer to USB OTG HCD host channel  callback */
/**
  * @}
  */

HAL_StatusTypeDef HAL_HCD_RegisterCallback(HCD_HandleTypeDef *hhcd,
        HAL_HCD_CallbackIDTypeDef CallbackID,
        pHCD_CallbackTypeDef pCallback);

HAL_StatusTypeDef HAL_HCD_UnRegisterCallback(HCD_HandleTypeDef *hhcd,
        HAL_HCD_CallbackIDTypeDef CallbackID);

HAL_StatusTypeDef HAL_HCD_RegisterHC_NotifyURBChangeCallback(HCD_HandleTypeDef *hhcd,
        pHCD_HC_NotifyURBChangeCallbackTypeDef pCallback);

HAL_StatusTypeDef HAL_HCD_UnRegisterHC_NotifyURBChangeCallback(HCD_HandleTypeDef *hhcd);
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */
/**
  * @}
  */

/* I/O operation functions  ***************************************************/
/** @addtogroup HCD_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
HAL_StatusTypeDef HAL_HCD_HC_SubmitRequest(HCD_HandleTypeDef *hhcd, uint8_t ch_num,
        uint8_t direction, uint8_t ep_type,
        uint8_t token, uint8_t *pbuff,
        uint16_t length, uint8_t do_ping);

/* Non-Blocking mode: Interrupt */
void HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd);
void HAL_HCD_WKUP_IRQHandler(HCD_HandleTypeDef *hhcd);
void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
        HCD_URBStateTypeDef urb_state);
/**
  * @}
  */

/* Peripheral Control functions  **********************************************/
/** @addtogroup HCD_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */
HAL_StatusTypeDef HAL_HCD_ResetPort(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_Start(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_Stop(HCD_HandleTypeDef *hhcd);
/**
  * @}
  */

/* Peripheral State functions  ************************************************/
/** @addtogroup HCD_Exported_Functions_Group4 Peripheral State functions
  * @{
  */
HCD_StateTypeDef        HAL_HCD_GetState(HCD_HandleTypeDef *hhcd);
HCD_URBStateTypeDef     HAL_HCD_HC_GetURBState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
HCD_HCStateTypeDef      HAL_HCD_HC_GetState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                HAL_HCD_HC_GetXferCount(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                HAL_HCD_GetCurrentFrame(HCD_HandleTypeDef *hhcd);
uint32_t                HAL_HCD_GetCurrentSpeed(HCD_HandleTypeDef *hhcd);

/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup HCD_Private_Macros HCD Private Macros
  * @{
  */
/**
  * @}
  */
/* Private functions prototypes ----------------------------------------------*/

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
