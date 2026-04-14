/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */
#include "bf0_hal_def.h"
#include "string.h"

//extern void rt_kprintf(const char *fmt, ...);

#if defined(HAL_CAN_MODULE_ENABLED) ||defined(_SIFLI_DOXYGEN_)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup CAN_Private_Constants CAN Private Constants
  * @{
  */
#define CAN_TIMEOUT_VALUE 10U
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup CAN_Exported_Functions CAN Exported Functions
  * @{
  */

/** @defgroup CAN_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) HAL_CAN_Init                       : Initialize and configure the CAN.
      (+) HAL_CAN_DeInit                     : De-initialize the CAN.
      (+) HAL_CAN_MspInit                    : Initialize the CAN MSP.
      (+) HAL_CAN_MspDeInit                  : DeInitialize the CAN MSP.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the CAN peripheral according to the specified
  *         parameters in the CAN_InitStruct.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan)
{
    uint32_t tickstart;

    /* Check CAN handle */
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }


#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
    if (hcan->State == HAL_CAN_STATE_RESET)
    {
        /* Reset callbacks to legacy functions */
        hcan->RxFifo0MsgPendingCallback  =  HAL_CAN_RxFifo0MsgPendingCallback;  /* Legacy weak RxFifo0MsgPendingCallback  */
        hcan->RxFifo0FullCallback        =  HAL_CAN_RxFifo0FullCallback;        /* Legacy weak RxFifo0FullCallback        */
        hcan->RxFifo1MsgPendingCallback  =  HAL_CAN_RxFifo1MsgPendingCallback;  /* Legacy weak RxFifo1MsgPendingCallback  */
        hcan->RxFifo1FullCallback        =  HAL_CAN_RxFifo1FullCallback;        /* Legacy weak RxFifo1FullCallback        */
        hcan->TxMailbox0CompleteCallback =  HAL_CAN_TxMailbox0CompleteCallback; /* Legacy weak TxMailbox0CompleteCallback */
        hcan->TxMailbox1CompleteCallback =  HAL_CAN_TxMailbox1CompleteCallback; /* Legacy weak TxMailbox1CompleteCallback */
        hcan->TxMailbox2CompleteCallback =  HAL_CAN_TxMailbox2CompleteCallback; /* Legacy weak TxMailbox2CompleteCallback */
        hcan->TxMailbox0AbortCallback    =  HAL_CAN_TxMailbox0AbortCallback;    /* Legacy weak TxMailbox0AbortCallback    */
        hcan->TxMailbox1AbortCallback    =  HAL_CAN_TxMailbox1AbortCallback;    /* Legacy weak TxMailbox1AbortCallback    */
        hcan->TxMailbox2AbortCallback    =  HAL_CAN_TxMailbox2AbortCallback;    /* Legacy weak TxMailbox2AbortCallback    */
        hcan->SleepCallback              =  HAL_CAN_SleepCallback;              /* Legacy weak SleepCallback              */
        hcan->WakeUpFromRxMsgCallback    =  HAL_CAN_WakeUpFromRxMsgCallback;    /* Legacy weak WakeUpFromRxMsgCallback    */
        hcan->ErrorCallback              =  HAL_CAN_ErrorCallback;              /* Legacy weak ErrorCallback              */

        if (hcan->MspInitCallback == NULL)
        {
            hcan->MspInitCallback = HAL_CAN_MspInit; /* Legacy weak MspInit */
        }

        /* Init the low level hardware: CLOCK, NVIC */

        hcan->MspInitCallback(hcan);
    }

#else
    // If the callback registration function is not enabled and the current state is the reset state
    if (hcan->State == HAL_CAN_STATE_RESET)
    {
        /* Init the low level hardware: CLOCK, NVIC */
        HAL_CAN_MspInit(hcan);
    }
#endif /* (USE_HAL_CAN_REGISTER_CALLBACKS) */



    hcan->Instance->CR2 &= ~CAN_CR2_MEMMASK;
    hcan->Instance->CR |= CAN_CR_RESET;

    hcan->Instance->PRESC = hcan->Init.Prescaler;

    // Release and reset
    hcan->Instance->CR = 0;

    // Clear the interrupt flag
    hcan->Instance->IR = 0;

    hcan->ErrorCode = HAL_CAN_ERROR_NONE;

    /*Initialize CAN status*/
    hcan->State = HAL_CAN_STATE_READY;
    return HAL_OK;
}

/**
  * @brief  Deinitializes the CAN peripheral registers to their default
  *         reset values.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    // Reset the CAN controller
    hcan->Instance->CR = CAN_CR_RESET;

    return HAL_OK;
}

/**
  * @brief  Initializes the CAN MSP.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitializes the CAN MSP.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_MspDeInit could be implemented in the user file
     */
}

#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
/**
  * @brief  Register a CAN CallBack.
  *         To be used instead of the weak predefined callback
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for CAN module
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref HAL_CAN_TX_MAILBOX0_COMPLETE_CALLBACK_CB_ID Tx Mailbox 0 Complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX1_COMPLETE_CALLBACK_CB_ID Tx Mailbox 1 Complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX2_COMPLETE_CALLBACK_CB_ID Tx Mailbox 2 Complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX0_ABORT_CALLBACK_CB_ID Tx Mailbox 0 Abort callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX1_ABORT_CALLBACK_CB_ID Tx Mailbox 1 Abort callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX2_ABORT_CALLBACK_CB_ID Tx Mailbox 2 Abort callback ID
  *           @arg @ref HAL_CAN_RX_FIFO0_MSG_PENDING_CALLBACK_CB_ID Rx Fifo 0 message pending callback ID
  *           @arg @ref HAL_CAN_RX_FIFO0_FULL_CALLBACK_CB_ID Rx Fifo 0 full callback ID
  *           @arg @ref HAL_CAN_RX_FIFO1_MSGPENDING_CALLBACK_CB_ID Rx Fifo 1 message pending callback ID
  *           @arg @ref HAL_CAN_RX_FIFO1_FULL_CALLBACK_CB_ID Rx Fifo 1 full callback ID
  *           @arg @ref HAL_CAN_SLEEP_CALLBACK_CB_ID Sleep callback ID
  *           @arg @ref HAL_CAN_WAKEUP_FROM_RX_MSG_CALLBACK_CB_ID Wake Up from Rx message callback ID
  *           @arg @ref HAL_CAN_ERROR_CALLBACK_CB_ID Error callback ID
  *           @arg @ref HAL_CAN_MSPINIT_CB_ID MspInit callback ID
  *           @arg @ref HAL_CAN_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_RegisterCallback(CAN_HandleTypeDef *hcan, HAL_CAN_CallbackIDTypeDef CallbackID, void (* pCallback)(CAN_HandleTypeDef *_hcan))
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_INVALID_CALLBACK;

        return HAL_ERROR;
    }

    if (hcan->State == HAL_CAN_STATE_READY)
    {
        switch (CallbackID)
        {
        case HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID :
            hcan->TxMailbox0CompleteCallback = pCallback;
            break;

        case HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID :
            hcan->TxMailbox1CompleteCallback = pCallback;
            break;

        case HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID :
            hcan->TxMailbox2CompleteCallback = pCallback;
            break;

        case HAL_CAN_TX_MAILBOX0_ABORT_CB_ID :
            hcan->TxMailbox0AbortCallback = pCallback;
            break;

        case HAL_CAN_TX_MAILBOX1_ABORT_CB_ID :
            hcan->TxMailbox1AbortCallback = pCallback;
            break;

        case HAL_CAN_TX_MAILBOX2_ABORT_CB_ID :
            hcan->TxMailbox2AbortCallback = pCallback;
            break;

        case HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID :
            hcan->RxFifo0MsgPendingCallback = pCallback;
            break;

        case HAL_CAN_RX_FIFO0_FULL_CB_ID :
            hcan->RxFifo0FullCallback = pCallback;
            break;

        case HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID :
            hcan->RxFifo1MsgPendingCallback = pCallback;
            break;

        case HAL_CAN_RX_FIFO1_FULL_CB_ID :
            hcan->RxFifo1FullCallback = pCallback;
            break;

        case HAL_CAN_SLEEP_CB_ID :
            hcan->SleepCallback = pCallback;
            break;

        case HAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID :
            hcan->WakeUpFromRxMsgCallback = pCallback;
            break;

        case HAL_CAN_ERROR_CB_ID :
            hcan->ErrorCallback = pCallback;
            break;

        case HAL_CAN_MSPINIT_CB_ID :
            hcan->MspInitCallback = pCallback;
            break;

        case HAL_CAN_MSPDEINIT_CB_ID :
            hcan->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hcan->ErrorCode |= HAL_CAN_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (hcan->State == HAL_CAN_STATE_RESET)
    {
        switch (CallbackID)
        {
        case HAL_CAN_MSPINIT_CB_ID :
            hcan->MspInitCallback = pCallback;
            break;

        case HAL_CAN_MSPDEINIT_CB_ID :
            hcan->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hcan->ErrorCode |= HAL_CAN_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    return status;
}

/**
  * @brief  Unregister a CAN CallBack.
  *         CAN callabck is redirected to the weak predefined callback
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for CAN module
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *           @arg @ref HAL_CAN_TX_MAILBOX0_COMPLETE_CALLBACK_CB_ID Tx Mailbox 0 Complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX1_COMPLETE_CALLBACK_CB_ID Tx Mailbox 1 Complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX2_COMPLETE_CALLBACK_CB_ID Tx Mailbox 2 Complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX0_ABORT_CALLBACK_CB_ID Tx Mailbox 0 Abort callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX1_ABORT_CALLBACK_CB_ID Tx Mailbox 1 Abort callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX2_ABORT_CALLBACK_CB_ID Tx Mailbox 2 Abort callback ID
  *           @arg @ref HAL_CAN_RX_FIFO0_MSG_PENDING_CALLBACK_CB_ID Rx Fifo 0 message pending callback ID
  *           @arg @ref HAL_CAN_RX_FIFO0_FULL_CALLBACK_CB_ID Rx Fifo 0 full callback ID
  *           @arg @ref HAL_CAN_RX_FIFO1_MSGPENDING_CALLBACK_CB_ID Rx Fifo 1 message pending callback ID
  *           @arg @ref HAL_CAN_RX_FIFO1_FULL_CALLBACK_CB_ID Rx Fifo 1 full callback ID
  *           @arg @ref HAL_CAN_SLEEP_CALLBACK_CB_ID Sleep callback ID
  *           @arg @ref HAL_CAN_WAKEUP_FROM_RX_MSG_CALLBACK_CB_ID Wake Up from Rx message callback ID
  *           @arg @ref HAL_CAN_ERROR_CALLBACK_CB_ID Error callback ID
  *           @arg @ref HAL_CAN_MSPINIT_CB_ID MspInit callback ID
  *           @arg @ref HAL_CAN_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_UnRegisterCallback(CAN_HandleTypeDef *hcan, HAL_CAN_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (hcan->State == HAL_CAN_STATE_READY)
    {
        switch (CallbackID)
        {
        case HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID :
            hcan->TxMailbox0CompleteCallback = HAL_CAN_TxMailbox0CompleteCallback;
            break;

        case HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID :
            hcan->TxMailbox1CompleteCallback = HAL_CAN_TxMailbox1CompleteCallback;
            break;

        case HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID :
            hcan->TxMailbox2CompleteCallback = HAL_CAN_TxMailbox2CompleteCallback;
            break;

        case HAL_CAN_TX_MAILBOX0_ABORT_CB_ID :
            hcan->TxMailbox0AbortCallback = HAL_CAN_TxMailbox0AbortCallback;
            break;

        case HAL_CAN_TX_MAILBOX1_ABORT_CB_ID :
            hcan->TxMailbox1AbortCallback = HAL_CAN_TxMailbox1AbortCallback;
            break;

        case HAL_CAN_TX_MAILBOX2_ABORT_CB_ID :
            hcan->TxMailbox2AbortCallback = HAL_CAN_TxMailbox2AbortCallback;
            break;

        case HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID :
            hcan->RxFifo0MsgPendingCallback = HAL_CAN_RxFifo0MsgPendingCallback;
            break;

        case HAL_CAN_RX_FIFO0_FULL_CB_ID :
            hcan->RxFifo0FullCallback = HAL_CAN_RxFifo0FullCallback;
            break;

        case HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID :
            hcan->RxFifo1MsgPendingCallback = HAL_CAN_RxFifo1MsgPendingCallback;
            break;

        case HAL_CAN_RX_FIFO1_FULL_CB_ID :
            hcan->RxFifo1FullCallback = HAL_CAN_RxFifo1FullCallback;
            break;

        case HAL_CAN_SLEEP_CB_ID :
            hcan->SleepCallback = HAL_CAN_SleepCallback;
            break;

        case HAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID :
            hcan->WakeUpFromRxMsgCallback = HAL_CAN_WakeUpFromRxMsgCallback;
            break;

        case HAL_CAN_ERROR_CB_ID :
            hcan->ErrorCallback = HAL_CAN_ErrorCallback;
            break;

        case HAL_CAN_MSPINIT_CB_ID :
            hcan->MspInitCallback = HAL_CAN_MspInit;
            break;

        case HAL_CAN_MSPDEINIT_CB_ID :
            hcan->MspDeInitCallback = HAL_CAN_MspDeInit;
            break;

        default :
            /* Update the error code */
            hcan->ErrorCode |= HAL_CAN_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (hcan->State == HAL_CAN_STATE_RESET)
    {
        switch (CallbackID)
        {
        case HAL_CAN_MSPINIT_CB_ID :
            hcan->MspInitCallback = HAL_CAN_MspInit;
            break;

        case HAL_CAN_MSPDEINIT_CB_ID :
            hcan->MspDeInitCallback = HAL_CAN_MspDeInit;
            break;

        default :
            /* Update the error code */
            hcan->ErrorCode |= HAL_CAN_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    return status;
}
#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group2 Configuration functions
 *  @brief    Configuration functions.
 *
@verbatim
  ==============================================================================
              ##### Configuration functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) HAL_CAN_ConfigFilter            : Configure the CAN reception filters

@endverbatim
  * @{
  */

/**
  * @brief  Configures the CAN reception filter according to the specified
  *         parameters in the CAN_FilterInitStruct.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  sFilterConfig pointer to a CAN_FilterTypeDef structure that
  *         contains the filter configuration information.
  * @retval None
  */
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig)
{
    uint32_t filterIndex = sFilterConfig->FilterBank;
    uint32_t maskValue, filterValue;

    if (hcan == NULL || filterIndex >= 16)
    {
        return HAL_ERROR;
    }

    // Construct 29-bit filter values and masks

    // Convert the FilterId to a 29-digit value and automatically handle the higher bits.
    filterValue = sFilterConfig->FilterId & 0x1FFFFFFF;

    // Construct a 29-bit mask using bitwise operators
    maskValue = sFilterConfig->FilterMask & 0x1FFFFFFF;

    // Configure the filter mask
    hcan->Instance->ACFCTRL = CAN_ACFCTRL_SELMASK | filterIndex;
    hcan->Instance->ACF = maskValue;

    // Waiting for the register update to be completed
    HAL_Delay_us(10);

    // Configure filter values
    hcan->Instance->ACFCTRL = filterIndex;
    hcan->Instance->ACF = filterValue;

    // Waiting for the register update to be completed
    HAL_Delay_us(10);

    // Configure IDE checks and frame types
    // AIDE: Enable IDE check
    // AIDEE:IDE value (0 = only accepts standard frames, 1 = only accepts extended frames)

    uint32_t acfReg = hcan->Instance->ACF;

    if (sFilterConfig->IDECheckEnable == ENABLE)
    {
        acfReg |= CAN_ACF_AIDE;
    }
    else
    {
        acfReg &= ~CAN_ACF_AIDE;
    }

    if (sFilterConfig->IDEValue == CAN_ID_EXT)
    {
        acfReg |= CAN_ACF_AIDEE;
    }
    else
    {
        acfReg &= ~CAN_ACF_AIDEE;
    }

    // Write back to the ACF register
    hcan->Instance->ACF = acfReg;

    // Update Enable Bit
    if (sFilterConfig->FilterActivation == ENABLE)
    {
        hcan->Init.FilterEnable |= (1 << (16 + filterIndex));
    }
    else
    {
        hcan->Init.FilterEnable &= ~(1 << (16 + filterIndex));
    }

    // Write the enable bits to the register
    hcan->Instance->ACFCTRL = hcan->Init.FilterEnable;

    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group3 Control functions
 *  @brief    Control functions
 *
@verbatim
  ==============================================================================
                      ##### Control functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) HAL_CAN_Start                    : Start the CAN module
      (+) HAL_CAN_Stop                     : Stop the CAN module
      (+) HAL_CAN_RequestSleep             : Request sleep mode entry.
      (+) HAL_CAN_WakeUp                   : Wake up from sleep mode.
      (+) HAL_CAN_IsSleepActive            : Check is sleep mode is active.
      (+) HAL_CAN_AddTxMessage             : Add a message to the Tx mailboxes
                                             and activate the corresponding
                                             transmission request
      (+) HAL_CAN_AbortTxRequest           : Abort transmission request
      (+) HAL_CAN_GetTxMailboxesFreeLevel  : Return Tx mailboxes free level
      (+) HAL_CAN_IsTxMessagePending       : Check if a transmission request is
                                             pending on the selected Tx mailbox
      (+) HAL_CAN_GetRxMessage             : Get a CAN frame from the Rx FIFO
      (+) HAL_CAN_GetRxFifoFillLevel       : Return Rx FIFO fill level

@endverbatim
  * @{
  */

/**
  * @brief  Start the CAN module.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    // Release reset and start CAN
    hcan->Instance->CR = 0;

    return HAL_OK;
}

/**
  * @brief  Stop the CAN module and enable access to configuration registers.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    // Set bit, reset bit, stop CAN
    hcan->Instance->CR |= CAN_CR_RESET;

    return HAL_OK;
}

/**
  * @brief  Request the sleep mode (low power) entry.
  *         When returning from this function, Sleep mode will be entered
  *         as soon as the current CAN activity (transmission or reception
  *         of a CAN frame) has been completed.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status.
  */
#if 0
HAL_StatusTypeDef HAL_CAN_RequestSleep(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_StateTypeDef state = hcan->State;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Request Sleep mode */
        SET_BIT(hcan->Instance->MCR, CAN_MCR_SLEEP);

        /* Return function status */
        return HAL_OK;
    }
    else
    {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

        /* Return function status */
        return HAL_ERROR;
    }
}
#endif
/**
  * @brief  Wake up from sleep mode.
  *         When returning with HAL_OK status from this function, Sleep mode
  *         is exited.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status.
  */
#if 0
HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan)
{
    __IO uint32_t count = 0;
    uint32_t timeout = 1000000U;
    HAL_CAN_StateTypeDef state = hcan->State;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Wake up request */
        CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_SLEEP);

        /* HAL_Delay_us sleep mode is exited */
        do
        {
            /* Increment counter */
            count++;

            /* Check if timeout is reached */
            if (count > timeout)
            {
                /* Update error code */
                hcan->ErrorCode |= HAL_CAN_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
        while ((hcan->Instance->MSR & CAN_MSR_SLAK) != 0U);

        /* Return function status */
        return HAL_OK;
    }
    else
    {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

        return HAL_ERROR;
    }
}

/**
  * @brief  Check is sleep mode is active.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval Status
  *          - 0 : Sleep mode is not active.
  *          - 1 : Sleep mode is active.
  */
uint32_t HAL_CAN_IsSleepActive(CAN_HandleTypeDef *hcan)
{
    uint32_t status = 0U;
    HAL_CAN_StateTypeDef state = hcan->State;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Check Sleep mode */
        if ((hcan->Instance->MSR & CAN_MSR_SLAK) != 0U)
        {
            status = 1U;
        }
    }

    /* Return function status */
    return status;
}
#endif
/**
  * @brief  Add a message to the first free Tx mailbox and activate the
  *         corresponding transmission request.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  pHeader pointer to a CAN_TxHeaderTypeDef structure.
  * @param  aData array containing the payload of the Tx frame.
  * @param  pTxMailbox pointer to a variable where the function will return
  *         the TxMailbox used to store the Tx message.
  *         This parameter can be a value of @arg CAN_Tx_Mailboxes.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint32_t aData[])
{
    uint32_t tbuf_data[18] ;
    // uint32_t frame_word = 0;
    uint8_t i;

    if ((hcan == NULL) || (pHeader == NULL) || (aData == NULL))
    {
        return HAL_ERROR;
    }

    // Check whether the primary sending buffer (PTB) is idle
    if ((hcan->Instance->CR & CAN_CR_TACTIVE) != 0)
    {
        // rt_kprintf("The sending buffer is busy.\n");
        return HAL_BUSY;
    }

    // Construct the CAN frame header
    // Construct the frame header (split the ID and CTRL registers, in accordance with the CAN-CTRL hardware rules)
    // Discard the merged frame_word and split the ID/CTRL according to the hardware rules.
    if (pHeader->IDE == CAN_ID_STD)
    {
        // Standard Frame
        tbuf_data[0] = pHeader->StdId & 0x7FF; // TBUF_ID = 11-digit standard ID
    }
    else
    {
        // Extended Frame
        tbuf_data[0] = pHeader->ExtId & 0x1FFFFFFF; // TBUF_ID = 29-digit extended ID
    }

    tbuf_data[1] = ((pHeader->IDE & 0x01) << 7) |
                   ((pHeader->RTR & 0x01) << 6) |
                   (pHeader->DLC & 0x0F); // The DLC is only 4 bits (0 to 8) and does not require splitting.

    // Store to tbuf_data array
    tbuf_data[2] = aData[0]; // Data field starting position
    tbuf_data[3] = aData[1];

    memcpy((void *) & (hcan->Instance->TBUF), &tbuf_data[0], 72);

    // Start sending (using PTB)
    hcan->Instance->CR |= CAN_CR_TPE;

    if ((hcan->Instance->CR & CAN_CR_TPE) == 0)
    {
        // rt_kprintf("Failed to initiate the transmission.\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Abort transmission requests
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  TxMailboxes List of the Tx Mailboxes to abort.
  *         This parameter can be any combination of @arg CAN_Tx_Mailboxes.
  * @retval HAL status
  */
#if 0
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes)
{
    HAL_CAN_StateTypeDef state = hcan->State;

    /* Check function parameters */
    HAL_ASSERT(IS_CAN_TX_MAILBOX_LIST(TxMailboxes));

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Check Tx Mailbox 0 */
        if ((TxMailboxes & CAN_TX_MAILBOX0) != 0U)
        {
            /* Add cancellation request for Tx Mailbox 0 */
            SET_BIT(hcan->Instance->TSR, CAN_TSR_ABRQ0);
        }

        /* Check Tx Mailbox 1 */
        if ((TxMailboxes & CAN_TX_MAILBOX1) != 0U)
        {
            /* Add cancellation request for Tx Mailbox 1 */
            SET_BIT(hcan->Instance->TSR, CAN_TSR_ABRQ1);
        }

        /* Check Tx Mailbox 2 */
        if ((TxMailboxes & CAN_TX_MAILBOX2) != 0U)
        {
            /* Add cancellation request for Tx Mailbox 2 */
            SET_BIT(hcan->Instance->TSR, CAN_TSR_ABRQ2);
        }

        /* Return function status */
        return HAL_OK;
    }
    else
    {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

        return HAL_ERROR;
    }
}

/**
  * @brief  Return Tx Mailboxes free level: number of free Tx Mailboxes.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval Number of free Tx Mailboxes.
  */
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan)
{
    uint32_t freelevel = 0U;
    HAL_CAN_StateTypeDef state = hcan->State;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Check Tx Mailbox 0 status */
        if ((hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
        {
            freelevel++;
        }

        /* Check Tx Mailbox 1 status */
        if ((hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
        {
            freelevel++;
        }

        /* Check Tx Mailbox 2 status */
        if ((hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
        {
            freelevel++;
        }
    }

    /* Return Tx Mailboxes free level */
    return freelevel;
}

/**
  * @brief  Check if a transmission request is pending on the selected Tx
  *         Mailboxes.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  TxMailboxes List of Tx Mailboxes to check.
  *         This parameter can be any combination of @arg CAN_Tx_Mailboxes.
  * @retval Status
  *          - 0 : No pending transmission request on any selected Tx Mailboxes.
  *          - 1 : Pending transmission request on at least one of the selected
  *                Tx Mailbox.
  */
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes)
{
    uint32_t status = 0U;
    HAL_CAN_StateTypeDef state = hcan->State;

    /* Check function parameters */
    HAL_ASSERT(IS_CAN_TX_MAILBOX_LIST(TxMailboxes));

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Check pending transmission request on the selected Tx Mailboxes */
        if ((hcan->Instance->TSR & (TxMailboxes << CAN_TSR_TME0_Pos)) != (TxMailboxes << CAN_TSR_TME0_Pos))
        {
            status = 1U;
        }
    }

    /* Return status */
    return status;
}

/**
  * @brief  Return timestamp of Tx message sent, if time triggered communication
            mode is enabled.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  TxMailbox Tx Mailbox where the timestamp of message sent will be
  *         read.
  *         This parameter can be one value of @arg CAN_Tx_Mailboxes.
  * @retval Timestamp of message sent from Tx Mailbox.
  */
uint32_t HAL_CAN_GetTxTimestamp(CAN_HandleTypeDef *hcan, uint32_t TxMailbox)
{
    uint32_t timestamp = 0U;
    uint32_t transmitmailbox;
    HAL_CAN_StateTypeDef state = hcan->State;

    /* Check function parameters */
    HAL_ASSERT(IS_CAN_TX_MAILBOX(TxMailbox));

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Select the Tx mailbox */
        transmitmailbox = POSITION_VAL(TxMailbox);

        /* Get timestamp */
        timestamp = (hcan->Instance->sTxMailBox[transmitmailbox].TDTR & CAN_TDT0R_TIME) >> CAN_TDT0R_TIME_Pos;
    }

    /* Return the timestamp */
    return timestamp;
}
#endif
/**
  * @brief  Get an CAN frame from the Rx FIFO zone into the message RAM.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  RxFifo Fifo number of the received message to be read.
  *         This parameter can be a value of @arg CAN_receive_FIFO_number.
  * @param  pHeader pointer to a CAN_RxHeaderTypeDef structure where the header
  *         of the Rx frame will be stored.
  * @param  aData array where the payload of the Rx frame will be stored.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan,  CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{

    // Receiving buffer (of the same size as RBUF, stored in 32-bit words: ID → CTRL → Data)
    uint32_t rbuf_data[18] = {0};
    uint32_t *rbuf_ptr = rbuf_data;
    uint8_t i;

    if ((hcan == NULL) || (pHeader == NULL) || (aData == NULL))
    {
        return HAL_ERROR;
    }

    // Check the receiving FIFO status (refer to your register configuration)
    // FIFO is empty: Return an empty frame
    if ((hcan->Instance->CR & CAN_CR_RSTAT_Msk) == 0)
    {
        // rt_kprintf("The FIFO has no data to receive and there is no data available for reading.\n");
        pHeader->DLC = 0;
        return HAL_OK;
    }

    // Check the overflow status (ROV bit 29)
    if (hcan->Instance->CR & CAN_CR_ROV)
    {
        // rt_kprintf("Receive FIFO overflow\n");

        pHeader->Overflow = 1;
    }

    // Copy all data from RBUF to the local buffer.
    memcpy(&rbuf_data, (void *) & (hcan->Instance->RBUF), sizeof(rbuf_data));

    // Analyze the frame structure
    // Analysis ID
    uint32_t act_id = *rbuf_ptr++;
    // Analysis of CTRL (the second 32-bit word, including IDE/RTR/DLC)
    uint32_t act_ctrl = *rbuf_ptr++;

    // Extract IDE/RTR/DLC from CTRL (aligned with the format of the transmitting end CTRL)
    pHeader->IDE = (act_ctrl >> 7) & 0x1;
    pHeader->RTR = (act_ctrl >> 6) & 0x1;
    pHeader->DLC = act_ctrl & 0xF;

    // Parsing ID (based on the IDE distinction standard/extended frame)
    if (pHeader->IDE == CAN_ID_STD)
    {
        pHeader->StdId = act_id & 0x7FF;  // The standard ID consists of only 11 digits.
        pHeader->ExtId = 0;
    }
    else
    {
        pHeader->ExtId = act_id & 0x1FFFFFFF;  // The extended ID is 29 bits.
        pHeader->StdId = 0;
    }

    // Parse the data (starting from the third 32-bit word, extracting it byte by byte)
    uint8_t *data_ptr = (uint8_t *)rbuf_ptr;
    for (i = 0; i < pHeader->DLC && i < 8; i++)
    {
        aData[i] = data_ptr[i];  // Copy in byte order
    }

    // Release the receive buffer (allowing hardware to receive the next frame)
    hcan->Instance->CR |= CAN_CR_RREL;

    return HAL_OK;

}
#if 0
/**
  * @brief  Return Rx FIFO fill level.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  RxFifo Rx FIFO.
  *         This parameter can be a value of @arg CAN_receive_FIFO_number.
  * @retval Number of messages available in Rx FIFO.
  */
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan, uint32_t RxFifo)
{
    uint32_t filllevel = 0U;
    HAL_CAN_StateTypeDef state = hcan->State;

    /* Check function parameters */
    HAL_ASSERT(IS_CAN_RX_FIFO(RxFifo));

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        if (RxFifo == CAN_RX_FIFO0)
        {
            filllevel = hcan->Instance->RF0R & CAN_RF0R_FMP0;
        }
        else /* RxFifo == CAN_RX_FIFO1 */
        {
            filllevel = hcan->Instance->RF1R & CAN_RF1R_FMP1;
        }
    }

    /* Return Rx FIFO fill level */
    return filllevel;
}

/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group4 Interrupts management
 *  @brief    Interrupts management
 *
@verbatim
  ==============================================================================
                       ##### Interrupts management #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) HAL_CAN_ActivateNotification      : Enable interrupts
      (+) HAL_CAN_DeactivateNotification    : Disable interrupts
      (+) HAL_CAN_IRQHandler                : Handles CAN interrupt request

@endverbatim
  * @{
  */

/**
  * @brief  Enable interrupts.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  ActiveITs indicates which interrupts will be enabled.
  *         This parameter can be any combination of @arg CAN_Interrupts.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)
{
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    hcan->Instance->IR |= ActiveITs;

    return HAL_OK;
}

/**
  * @brief  Disable interrupts.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  InactiveITs indicates which interrupts will be disabled.
  *         This parameter can be any combination of @arg CAN_Interrupts.
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs)
{
    HAL_CAN_StateTypeDef state = hcan->State;

    /* Check function parameters */
    HAL_ASSERT(IS_CAN_IT(InactiveITs));

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Disable the selected interrupts */
        __HAL_CAN_DISABLE_IT(hcan, InactiveITs);

        /* Return function status */
        return HAL_OK;
    }
    else
    {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

        return HAL_ERROR;
    }
}

/**
  * @brief  Handles CAN interrupt request
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan)
{
    uint32_t ir = hcan->Instance->IR;

    if (ir & CAN_IR_TSIF)
    {
        if (hcan->TxCpltCallback != NULL)
        {
            hcan->TxCpltCallback(hcan);
        }
        hcan->Instance->IR = CAN_IR_TSIF;
    }

    if (ir & CAN_IR_RIF)
    {
        if (hcan->RxCpltCallback != NULL)
        {
            hcan->RxCpltCallback(hcan);
        }
        hcan->Instance->IR = CAN_IR_RIF;
    }

    if (ir & CAN_IR_EIF)
    {
        if (hcan->ErrorCallback != NULL)
        {
            hcan->ErrorCallback(hcan);
        }
        hcan->Instance->IR = CAN_IR_EIF;
    }
}
#endif
/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group5 Callback functions
 *  @brief   CAN Callback functions
 *
@verbatim
  ==============================================================================
                          ##### Callback functions #####
  ==============================================================================
    [..]
    This subsection provides the following callback functions:
      (+) HAL_CAN_TxMailbox0CompleteCallback
      (+) HAL_CAN_TxMailbox1CompleteCallback
      (+) HAL_CAN_TxMailbox2CompleteCallback
      (+) HAL_CAN_TxMailbox0AbortCallback
      (+) HAL_CAN_TxMailbox1AbortCallback
      (+) HAL_CAN_TxMailbox2AbortCallback
      (+) HAL_CAN_RxFifo0MsgPendingCallback
      (+) HAL_CAN_RxFifo0FullCallback
      (+) HAL_CAN_RxFifo1MsgPendingCallback
      (+) HAL_CAN_RxFifo1FullCallback
      (+) HAL_CAN_SleepCallback
      (+) HAL_CAN_WakeUpFromRxMsgCallback
      (+) HAL_CAN_ErrorCallback

@endverbatim
  * @{
  */

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_TxMailbox0CompleteCallback could be implemented in the
              user file
     */
}

/**
  * @brief  Transmission Mailbox 1 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_TxMailbox1CompleteCallback could be implemented in the
              user file
     */
}

/**
  * @brief  Transmission Mailbox 2 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_TxMailbox2CompleteCallback could be implemented in the
              user file
     */
}

/**
  * @brief  Transmission Mailbox 0 Cancellation callback.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_TxMailbox0AbortCallback could be implemented in the
              user file
     */
}

/**
  * @brief  Transmission Mailbox 1 Cancellation callback.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_TxMailbox1AbortCallback could be implemented in the
              user file
     */
}

/**
  * @brief  Transmission Mailbox 2 Cancellation callback.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_TxMailbox2AbortCallback could be implemented in the
              user file
     */
}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
              user file
     */
}

/**
  * @brief  Rx FIFO 0 full callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_RxFifo0FullCallback could be implemented in the user
              file
     */
}

/**
  * @brief  Rx FIFO 1 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_RxFifo1MsgPendingCallback could be implemented in the
              user file
     */
}

/**
  * @brief  Rx FIFO 1 full callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_RxFifo1FullCallback could be implemented in the user
              file
     */
}

/**
  * @brief  Sleep callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_SleepCallback could be implemented in the user file
     */
}

/**
  * @brief  WakeUp from Rx message callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_WakeUpFromRxMsgCallback could be implemented in the
              user file
     */
}

/**
  * @brief  Error CAN callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_ErrorCallback could be implemented in the user file
     */
}

/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group6 Peripheral State and Error functions
 *  @brief   CAN Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral State and Error functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) HAL_CAN_GetState()  : Return the CAN state.
      (+) HAL_CAN_GetError()  : Return the CAN error codes if any.
      (+) HAL_CAN_ResetError(): Reset the CAN error codes if any.

@endverbatim
  * @{
  */

/**
  * @brief  Return the CAN state.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL state
  */
#if 0
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_StateTypeDef state = hcan->State;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Check sleep mode acknowledge flag */
        if ((hcan->Instance->CR & CAN_MSR_SLAK) != 0U)
        {
            /* Sleep mode is active */
            state = HAL_CAN_STATE_SLEEP_ACTIVE;
        }
        /* Check sleep mode request flag */
        else if ((hcan->Instance->MCR & CAN_MCR_SLEEP) != 0U)
        {
            /* Sleep mode request is pending */
            state = HAL_CAN_STATE_SLEEP_PENDING;
        }
        else
        {
            /* Neither sleep mode request nor sleep mode acknowledge */
        }
    }

    /* Return CAN state */
    return state;
}

/**
  * @brief  Return the CAN error code.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval CAN Error Code
  */
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *hcan)
{
    /* Return CAN error code */
    return hcan->ErrorCode;
}

/**
  * @brief  Reset the CAN error code.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef status = HAL_OK;
    HAL_CAN_StateTypeDef state = hcan->State;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /* Reset CAN error code */
        hcan->ErrorCode = 0U;
    }
    else
    {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

        status = HAL_ERROR;
    }

    /* Return the status */
    return status;
}
#endif
/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_CAN_MODULE_ENABLED */

/**
  * @}
  */



/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
