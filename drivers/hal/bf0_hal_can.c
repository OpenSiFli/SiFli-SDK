/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */
#include "bf0_hal_def.h"
#include "string.h"
#include <stdio.h>

#if defined(HAL_CAN_MODULE_ENABLED) ||defined(_SIFLI_DOXYGEN_)

#define CAN_FRAME_BYTE_SIZE  (sizeof(HAL_CAN_FrameTypeDef))
#define CAN_FRAME_WORD_SIZE  (CAN_FRAME_BYTE_SIZE / sizeof(uint32_t))

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
        hcan->RxMsgPendingCallback  =  HAL_CAN_RxMsgPendingCallback;    /* Legacy weak RxMsgPendingCallback    */
        hcan->RxFifoFullCallback    =  HAL_CAN_RxFifoFullCallback;      /* Legacy weak RxFifoFullCallback      */
        hcan->RxAlmostFullCallback  =  HAL_CAN_RxAlmostFullCallback;    /* Legacy weak RxAlmostFullCallback    */
        hcan->RxOverflowCallback    =  HAL_CAN_RxOverflowCallback;      /* Legacy weak RxOverflowCallback      */
        hcan->TxPtbCompleteCallback =  HAL_CAN_TxPtbCompleteCallback;   /* Legacy weak TxPtbCompleteCallback   */
        hcan->TxStbCompleteCallback =  HAL_CAN_TxStbCompleteCallback;   /* Legacy weak TxStbCompleteCallback   */
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
  *           @arg @ref HAL_CAN_TX_PTB_COMPLETE_CB_ID PTB complete callback ID
  *           @arg @ref HAL_CAN_TX_STB_COMPLETE_CB_ID STB complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID Tx Mailbox 2 Complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX0_ABORT_CB_ID Tx Mailbox 0 Abort callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX1_ABORT_CB_ID Tx Mailbox 1 Abort callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX2_ABORT_CB_ID Tx Mailbox 2 Abort callback ID
  *           @arg @ref HAL_CAN_RX_MSG_PENDING_CB_ID Rx msg pending callback ID
  *           @arg @ref HAL_CAN_RX_FIFO_FULL_CB_ID Rx FIFO full callback ID
  *           @arg @ref HAL_CAN_RX_ALMOST_FULL_CB_ID Rx almost full callback ID
  *           @arg @ref HAL_CAN_RX_OVERFLOW_CB_ID Rx overflow callback ID
  *           @arg @ref HAL_CAN_SLEEP_CB_ID Sleep callback ID
  *           @arg @ref HAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID Wake Up from Rx message callback ID
  *           @arg @ref HAL_CAN_ERROR_CB_ID Error callback ID
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
        case HAL_CAN_TX_PTB_COMPLETE_CB_ID :
            hcan->TxPtbCompleteCallback = pCallback;
            break;

        case HAL_CAN_TX_STB_COMPLETE_CB_ID :
            hcan->TxStbCompleteCallback = pCallback;
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

        case HAL_CAN_RX_MSG_PENDING_CB_ID :
            hcan->RxMsgPendingCallback = pCallback;
            break;

        case HAL_CAN_RX_FIFO_FULL_CB_ID :
            hcan->RxFifoFullCallback = pCallback;
            break;

        case HAL_CAN_RX_ALMOST_FULL_CB_ID :
            hcan->RxAlmostFullCallback = pCallback;
            break;

        case HAL_CAN_RX_OVERFLOW_CB_ID :
            hcan->RxOverflowCallback = pCallback;
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
  *           @arg @ref HAL_CAN_TX_PTB_COMPLETE_CB_ID PTB complete callback ID
  *           @arg @ref HAL_CAN_TX_STB_COMPLETE_CB_ID STB complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID Tx Mailbox 2 Complete callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX0_ABORT_CB_ID Tx Mailbox 0 Abort callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX1_ABORT_CB_ID Tx Mailbox 1 Abort callback ID
  *           @arg @ref HAL_CAN_TX_MAILBOX2_ABORT_CB_ID Tx Mailbox 2 Abort callback ID
  *           @arg @ref HAL_CAN_RX_MSG_PENDING_CB_ID Rx msg pending callback ID
  *           @arg @ref HAL_CAN_RX_FIFO_FULL_CB_ID Rx FIFO full callback ID
  *           @arg @ref HAL_CAN_RX_ALMOST_FULL_CB_ID Rx almost full callback ID
  *           @arg @ref HAL_CAN_RX_OVERFLOW_CB_ID Rx overflow callback ID
  *           @arg @ref HAL_CAN_SLEEP_CB_ID Sleep callback ID
  *           @arg @ref HAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID Wake Up from Rx message callback ID
  *           @arg @ref HAL_CAN_ERROR_CB_ID Error callback ID
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
        case HAL_CAN_TX_PTB_COMPLETE_CB_ID :
            hcan->TxPtbCompleteCallback = HAL_CAN_TxPtbCompleteCallback;
            break;

        case HAL_CAN_TX_STB_COMPLETE_CB_ID :
            hcan->TxStbCompleteCallback = HAL_CAN_TxStbCompleteCallback;
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

        case HAL_CAN_RX_MSG_PENDING_CB_ID :
            hcan->RxMsgPendingCallback = HAL_CAN_RxMsgPendingCallback;
            break;

        case HAL_CAN_RX_FIFO_FULL_CB_ID :
            hcan->RxFifoFullCallback = HAL_CAN_RxFifoFullCallback;
            break;

        case HAL_CAN_RX_ALMOST_FULL_CB_ID :
            hcan->RxAlmostFullCallback = HAL_CAN_RxAlmostFullCallback;
            break;

        case HAL_CAN_RX_OVERFLOW_CB_ID :
            hcan->RxOverflowCallback = HAL_CAN_RxOverflowCallback;
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

    // Convert the FilterId to a 29-bit value and automatically handle the higher bits.
    filterValue = sFilterConfig->FilterId & 0x1FFFFFFF;

    // Build a 29-bit mask.
    // Note: ST HAL uses 1=match / 0=don't-care, but SiFli hardware uses the
    // opposite convention (0=match / 1=don't-care), so we invert the mask.
    maskValue = (~sFilterConfig->FilterMask) & 0x1FFFFFFF;

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

    // Release reset and start CAN,
    // TODO:
    hcan->Instance->CR = 0;

    // Clear the interrupt flag after reset release
    // TODO:
    hcan->Instance->IR = 0;

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

static void HAL_CAN_FillFrame(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[])
{
    HAL_CAN_FrameTypeDef frame;
    uint32_t i;
    __IO uint32_t *tbuf;
    uint32_t *src;

    if (CAN_ID_STD == pHeader->IDE)
    {
        // Standard Frame
        frame.Identifier = pHeader->StdId & CAN_MAX_STD_ID;
    }
    else
    {
        // Extended Frame
        frame.Identifier = pHeader->ExtId & CAN_MAX_EXT_ID;
    }
    frame.Control = (pHeader->IDE | pHeader->RTR | MAKE_REG_VAL2(pHeader->DLC, CAN_FRAME_CONTROL_DLC));
    memcpy((void *)&frame.Data, aData, sizeof(frame.Data));

    /* make sure word aligned write */
    tbuf = &hcan->Instance->TBUF;
    src = (uint32_t *)&frame;
    for (i = 0; i < CAN_FRAME_WORD_SIZE; i++)
    {
        tbuf[i] = src[i];
    }
}

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
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[])
{
    if ((hcan == NULL) || (pHeader == NULL) || (aData == NULL))
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    HAL_ASSERT(IS_CAN_IDTYPE(pHeader->IDE));
    HAL_ASSERT(IS_CAN_RTR(pHeader->RTR));
    HAL_ASSERT(IS_CAN_DLC(pHeader->DLC));

    // Check whether the primary sending buffer (PTB) is idle
    if (hcan->Instance->CR & CAN_CR_TPE)
    {
        return HAL_BUSY;
    }

    /* select PTB */
    hcan->Instance->CR &= ~CAN_CR_TBSEL;

    HAL_CAN_FillFrame(hcan, pHeader, aData);

    /* Start tranmission */
    hcan->Instance->CR |= CAN_CR_TPE;

    return HAL_OK;
}

/**
  * @brief  Add a message to the STB (Secondary Transmit Buffer) queue.
  *         Call this function multiple times to queue frames, then call
  *         HAL_CAN_STB_Transmit to send all queued frames at once.
  *
  *         Note: STB frames have lower priority than PTB frames. If PTB is
  *         triggered while STB frames are being sent, the hardware will
  *         automatically send the PTB frame first.
  *
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  pHeader pointer to a CAN_TxHeaderTypeDef structure.
  * @param  aData array containing the payload of the Tx frame.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_STB_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[])
{
    HAL_CAN_FrameTypeDef frame;
    uint32_t i;
    __IO uint32_t *tbuf;

    if ((hcan == NULL) || (pHeader == NULL) || (aData == NULL))
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    HAL_ASSERT(IS_CAN_IDTYPE(pHeader->IDE));
    HAL_ASSERT(IS_CAN_RTR(pHeader->RTR));
    HAL_ASSERT(IS_CAN_DLC(pHeader->DLC));

    /* select STB */
    hcan->Instance->CR |= CAN_CR_TBSEL;

    if (hcan->Instance->CR & CAN_CR_TSNEXT)
    {
        return HAL_BUSY;
    }

    HAL_CAN_FillFrame(hcan, pHeader, aData);

    /* transmit all */
    hcan->Instance->CR |= CAN_CR_TSALL;

    /* Commit current frame and advance to next STB slot */
    hcan->Instance->CR |= CAN_CR_TSNEXT;

#if 0

    // Wait for TSNEXT to be cleared by hardware.
    // TSNEXT stays 1 when STB FIFO is full (7 frames for 56X, 16 for 58X),
    // indicating that the oldest frame has not been sent yet and a new frame
    // should not be written.
    {
        uint32_t timeout = 100000U;
        while (hcan->Instance->CR & CAN_CR_TSNEXT)
        {
            if (--timeout == 0U)
            {
                /* STB stuck (e.g. all frames failing, TEC climbing),
                   abort STB to free the buffer and avoid infinite loop. */
                hcan->Instance->CR |= CAN_CR_TSA;
                return HAL_ERROR;
            }
        }
    }
#endif

    return HAL_OK;
}

/**
  * @brief  Abort transmission requests
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  ptb boolean indicating whether to abort PTB requests.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, bool primary)
{
    HAL_CAN_StateTypeDef state = hcan->State;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        if (primary)
        {
            /* Abort PTB transmission request */
            hcan->Instance->CR |= CAN_CR_TPA;
        }
        else
        {
            /* Abort STB transmission request */
            hcan->Instance->CR |= CAN_CR_TSA;
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
  * @param  primary boolean indicating whether to check primary mailboxes.
  * @retval Number of free Tx Mailboxes.
  */
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan, bool primary)
{
    uint32_t freelevel = 0U;
    uint8_t tsstat;
    HAL_CAN_StateTypeDef state = hcan->State;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        if (primary)
        {
            /* if TPE is active, transmission is ongoing and tbuf is not available for ptb */
            freelevel = (hcan->Instance->CR & CAN_CR_TPE) ? 0U : 1U;
        }
        else
        {
            tsstat = GET_REG_VAL2(hcan->Instance->CR, CAN_CR_TSSTAT);
            HAL_ASSERT(tsstat <= CAN_SECONDARY_TX_FIFO_DEPTH);

            freelevel = CAN_SECONDARY_TX_FIFO_DEPTH - tsstat;
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
  * @retval Status
  *          - 0 : No pending transmission request on any selected Tx Mailboxes.
  *          - 1 : Pending transmission request on primary or secondary Tx Mailbox.
  */
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *hcan)
{
    uint32_t status = 0U;
    HAL_CAN_StateTypeDef state = hcan->State;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        if ((hcan->Instance->CR & CAN_CR_TACTIVE) != 0U)
        {
            status = 1U;
        }
    }

    /* Return status */
    return status;
}

/**
  * @brief  Return Rx FIFO fill level.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  RxFifo Rx FIFO.
  *         This parameter can be a value of @arg CAN_receive_FIFO_number.
  * @return Rx FIFO fill level, value: 0 or 1
  * @retval 0: no messages available in Rx FIFO
  * @retval 1: message available in Rx FIFO
  */
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan)
{
    uint32_t filllevel = 0U;
    HAL_CAN_StateTypeDef state = hcan->State;
    uint8_t rstat;

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        rstat = GET_REG_VAL2(hcan->Instance->CR, CAN_CR_RSTAT);

        filllevel = rstat ? 1U : 0U;
    }

    /* Return Rx FIFO fill level */
    return filllevel;
}

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
    HAL_CAN_FrameTypeDef frame;
    uint8_t i;
    __IO uint32_t *rbuf;
    uint32_t *dest;

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

    /* make sure word aligned access  */
    rbuf = &hcan->Instance->RBUF;
    dest = (uint32_t *)&frame;
    for (i = 0; i < CAN_FRAME_WORD_SIZE; i++)
    {
        dest[i] = rbuf[i];
    }

    // Extract IDE/RTR/DLC from CTRL (aligned with the format of the transmitting end CTRL)
    pHeader->IDE = frame.Control & CAN_FRAME_CONTROL_IDE_Msk;
    pHeader->RTR = frame.Control & CAN_FRAME_CONTROL_RTR_Msk;
    pHeader->DLC = GET_REG_VAL2(frame.Control, CAN_FRAME_CONTROL_DLC);

    // Parsing ID (based on the IDE distinction standard/extended frame)
    if (pHeader->IDE == CAN_ID_STD)
    {
        pHeader->StdId = frame.Identifier & CAN_MAX_STD_ID;  // The standard ID consists of only 11 digits.
        pHeader->ExtId = 0;
    }
    else
    {
        pHeader->ExtId = frame.Identifier & CAN_MAX_EXT_ID;  // The extended ID is 29 bits.
        pHeader->StdId = 0;
    }

    memcpy(aData, &frame.Data, sizeof(frame.Data));

    // Release the receive buffer (allowing hardware to receive the next frame)
    hcan->Instance->CR |= CAN_CR_RREL;

    return HAL_OK;

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
  *         This parameter can be any combination of CAN_IR_xxxE bits:
  *         CAN_IR_TSFF, CAN_IR_EIE, CAN_IR_TSIE, CAN_IR_TPIE,
  *         CAN_IR_RAFIE, CAN_IR_RFIE, CAN_IR_ROIE, CAN_IR_RIE.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)
{
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    HAL_ASSERT(IS_CAN_IT(ActiveITs));

    __HAL_CAN_ENABLE_IT(hcan, ActiveITs);


    return HAL_OK;
}

/**
  * @brief  Disable interrupts.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  InactiveITs indicates which interrupts will be disabled.
  *         This parameter can be any combination of CAN_IR_xxxE bits.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs)
{
    HAL_CAN_StateTypeDef state = hcan->State;

    HAL_ASSERT(IS_CAN_IT(InactiveITs));

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {

        __HAL_CAN_DISABLE_IT(hcan, InactiveITs);

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
  * @brief  Handles CAN interrupt request.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan)
{
    uint32_t ir = hcan->Instance->IR;

    /* ─── Error / status flags ─── */
    if (ir & CAN_IR_EIF)
    {
        HAL_CAN_ErrorCallback(hcan);
    }

    /* ─── Receive processing (4-branch, ordered by severity high→low) ─── */

    /* 1. ROIF: Overflow — frames were lost. Record, drain whatever remains, notify. */
    if (ir & CAN_IR_ROIF)
    {
        // hcan->RxOverflowCount++;
        // CAN_DrainRxFifo(hcan);
        HAL_CAN_RxOverflowCallback(hcan);
        HAL_ASSERT(0);
        return;
    }

    /* 2. RFIF: RBUF full — drain all immediately to prevent imminent overflow. */
    if (ir & CAN_IR_RFIF)
    {
        // CAN_DrainRxFifo(hcan);
        HAL_CAN_RxFifoFullCallback(hcan);
    }

    /* 3. RAFIF: RBUF almost full — batch drain, notify so upper layer can speed up. */
    if (ir & CAN_IR_RAFIF)
    {
        // CAN_DrainRxFifo(hcan);
        HAL_CAN_RxAlmostFullCallback(hcan);
    }

    /* 4. RIF: Normal receive — at least one frame available, drain it. */
    if (ir & CAN_IR_RIF)
    {
        /* Note: RIF might be set together with RAFIF/RFIF above.
           If already drained by those branches, this is a no-op.
           Otherwise drain the normal one-frame case. */
        // if (CAN_DrainRxFifo(hcan) > 0U)

        HAL_CAN_RxMsgPendingCallback(hcan);
    }

    /* ─── Transmit processing ─── */

    /* STB transmission fail — abort STB to prevent FIFO deadlock.
       When TEC enters Error-Passive (≥128), all STB frames fail
       and accumulate until STB is full, causing TSNEXT to hang.
       Requires TSFF (IR bit 0) to be enabled via ActivateNotification. */
    if (ir & CAN_IR_TSFF)
    {
        hcan->Instance->CR |= CAN_CR_TSA;  /* Abort all STB frames */
    }

    /* STB transmission complete — TSIF auto-clears on next STB trigger */
    if (ir & CAN_IR_TSIF)
    {
        HAL_CAN_TxStbCompleteCallback(hcan);
    }

    /* PTB transmission complete — TPIF auto-clears on next TPE */
    if (ir & CAN_IR_TPIF)
    {
        HAL_CAN_TxPtbCompleteCallback(hcan);
    }

}

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
      (+) HAL_CAN_TxPtbCompleteCallback
      (+) HAL_CAN_TxStbCompleteCallback
      (+) HAL_CAN_TxMailbox2CompleteCallback
      (+) HAL_CAN_TxMailbox0AbortCallback
      (+) HAL_CAN_TxMailbox1AbortCallback
      (+) HAL_CAN_TxMailbox2AbortCallback
      (+) HAL_CAN_RxMsgPendingCallback
      (+) HAL_CAN_RxFifoFullCallback
      (+) HAL_CAN_RxAlmostFullCallback
      (+) HAL_CAN_RxOverflowCallback
      (+) HAL_CAN_SleepCallback
      (+) HAL_CAN_WakeUpFromRxMsgCallback
      (+) HAL_CAN_ErrorCallback

@endverbatim
  * @{
  */

/**
 * @brief  PTB transmission complete callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void HAL_CAN_TxPtbCompleteCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
}

/**
  * @brief  STB transmission complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_TxStbCompleteCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
}

/**
  * @brief  Rx msg pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_RxMsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
    /* Silent by default — fires every frame. Data already in software FIFO.
       Override to add per-frame logging. */
}

/**
  * @brief  Rx FIFO full callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_RxFifoFullCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
}

/**
  * @brief  Rx almost full callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_RxAlmostFullCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
}

/**
  * @brief  Rx overflow callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_RxOverflowCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
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
    UNUSED(hcan);
}

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
