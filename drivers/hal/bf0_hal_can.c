/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-FileCopyrightText: 2019 Alexander Wachter
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */
#include "bf0_hal_def.h"
#include "string.h"
#include <stdio.h>

#if defined(HAL_CAN_MODULE_ENABLED) ||defined(_SIFLI_DOXYGEN_)

#define CAN_FRAME_BYTE_SIZE  (sizeof(HAL_CAN_FrameTypeDef))
#define CAN_FRAME_WORD_SIZE  (CAN_FRAME_BYTE_SIZE / sizeof(uint32_t))

/* CAN sync segment is always one time quanta */
#define CAN_SYNC_SEG   (1)

/* Bitrate greater than threshold is regarded as fast speed */
#define CAN_FAST_BITRATE_THRESHOLD   (100000)

#define CAN_IS_FASTSPEED(bitrate)    ((bitrate) > CAN_FAST_BITRATE_THRESHOLD)

#define CAN_CLOCK_FREQ_HZ   (48000000)

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
    CAN_TypeDef *inst;
    uint32_t timing_cfg;

    /* Check CAN handle */
    if ((hcan == NULL) || (!hcan->Instance))
    {
        return HAL_ERROR;
    }

    // If the callback registration function is not enabled and the current state is the reset state
    if (hcan->State == HAL_CAN_STATE_RESET)
    {
        /* Init the low level hardware: CLOCK, NVIC */
        HAL_CAN_MspInit(hcan);
    }

    inst = hcan->Instance;
    hcan->NumOfTxMsgs = 0;
    inst->CR2 &= ~CAN_CR2_MEMMASK;
    inst->CR |= CAN_CR_RESET;

    MODIFY_REG(inst->PRESC, CAN_PRESC_S_PRESC_Msk, MAKE_REG_VAL2(hcan->Init.Prescaler - 1, CAN_PRESC_S_PRESC));

    /* sync_seg + prop_seg + phase_seg_1 = 1 + CAN_BITTIME_S_SEG_1 + 1
     * phase_seg_2 = CAN_BITTIME_S_SEG_2 + 1
     * sjw = CAN_BITTIME_S_SJW + 1
     */
    timing_cfg = MAKE_REG_VAL2(hcan->Init.SyncJumpWidth - 1, CAN_BITTIME_S_SJW)
                 | MAKE_REG_VAL2(hcan->Init.TimeSeg1 - 1, CAN_BITTIME_S_SEG_1)
                 | MAKE_REG_VAL2(hcan->Init.TimeSeg2 - 1, CAN_BITTIME_S_SEG_2);
    MODIFY_REG(inst->BITTIME, CAN_BITTIME_S_SJW_Msk | CAN_BITTIME_S_SEG_1_Msk | CAN_BITTIME_S_SEG_2,
               timing_cfg);

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
 * @brief Get the sample point location for a given bitrate
 *
 * @param  bitrate The bitrate in bits/second.
 * @return The sample point in permille.
 */
static uint16_t CAN_GetSamplePoint(uint32_t bitrate)
{
    uint16_t sample_pnt;

    if (bitrate > 800000)
    {
        /* 75.0% */
        sample_pnt = 750;
    }
    else if (bitrate > 500000)
    {
        /* 80.0% */
        sample_pnt = 800;
    }
    else
    {
        /* 87.5% */
        sample_pnt = 875;
    }

    return sample_pnt;
}

/**
 * @brief Update the timing given a total number of time quanta and a sample point.
 *
 * @code{.text}
 *
 * +---------------------------------------------------+
 * |     Nominal bit time in time quanta (total_tq)    |
 * +--------------+----------+------------+------------+
 * |   sync_seg   | prop_seg | phase_seg1 | phase_seg2 |
 * +--------------+----------+------------+------------+
 * | CAN_SYNG_SEG |        tseg1          |   tseg2    |
 * +--------------+-----------------------+------------+
 *                                        ^
 *                                   sample_pnt
 * @endcode
 *
 * @param total_tq              Total number of time quanta.
 * @param sample_pnt            Sample point in permille of the entire bit time.
 * @param min                   minimum timing configuration
 * @param max                   maximum timing configuration
 * @param[out] res              Result timing configuration
 * @retval           0 or positive sample point error on success.
 * @retval           -HAL_ERROR if the requested sample point cannot be met.
 */
static int32_t CAN_UpdateSamplePnt(uint32_t total_tq, uint32_t sample_pnt,
                                   CAN_InitTypeDef *min, CAN_InitTypeDef *max,
                                   CAN_InitTypeDef *res)
{
    uint16_t tseg1_max = max->TimeSeg1;
    uint16_t tseg1_min = min->TimeSeg1;
    uint32_t sample_pnt_res;
    uint16_t tseg1;
    uint16_t tseg2;

    /* Calculate number of time quanta in tseg2 for given sample point */
    tseg2 = total_tq - (total_tq * sample_pnt) / 1000;
    tseg2 = HAL_CLAMP(tseg2, min->TimeSeg2, max->TimeSeg2);

    /* Calculate number of time quanta in tseg1 */
    tseg1 = total_tq - CAN_SYNC_SEG - tseg2;
    if (tseg1 > tseg1_max)
    {
        /* Sample point location must be decreased */
        tseg1 = tseg1_max;
        tseg2 = total_tq - CAN_SYNC_SEG - tseg1;

        if (tseg2 > max->TimeSeg2)
        {
            return -HAL_ERROR;
        }
    }
    else if (tseg1 < tseg1_min)
    {
        /* Sample point location must be increased */
        tseg1 = tseg1_min;
        tseg2 = total_tq - CAN_SYNC_SEG - tseg1;

        if (tseg2 < min->TimeSeg2)
        {
            return -HAL_ERROR;
        }
    }
    else
    {
        /* Sample point location within range */
    }

    /* required by CAN 2.0 and CAN FD nominal bit rate,
     * for CAN FD data bit rate, it's required: tseg1 >= (tseg2+1)
     */
    if (tseg1 < (tseg2 + 2))
    {
        return -HAL_ERROR;
    }

    res->TimeSeg2 = tseg2;
    res->TimeSeg1 = tseg1;

    /* Calculate the resulting sample point */
    sample_pnt_res = (CAN_SYNC_SEG + tseg1) * 1000 / total_tq;

    /* Return the absolute sample point error */
    return sample_pnt_res > sample_pnt ?
           sample_pnt_res - sample_pnt :
           sample_pnt - sample_pnt_res;
}

int32_t HAL_CAN_CalcTiming(CAN_HandleTypeDef *hcan, uint32_t bitrate)
{
    uint32_t total_tq;
    int32_t err_min = INT32_MAX;
    uint32_t clock_freq = CAN_CLOCK_FREQ_HZ;
    int32_t err;
    uint16_t sample_pnt;
    uint32_t prescaler;
    CAN_InitTypeDef timing_cfg;
    CAN_InitTypeDef timing_cfg_min;
    CAN_InitTypeDef timing_cfg_max;

    if (bitrate == 0)
    {
        return -HAL_ERROR;
    }

    sample_pnt = CAN_GetSamplePoint(bitrate);

    timing_cfg_min.TimeSeg1 = CAN_TIME_SEG1_MIN;
    timing_cfg_min.TimeSeg2 = CAN_TIME_SEG2_MIN;
    timing_cfg_min.Prescaler = CAN_PRESCALER_MIN;
    timing_cfg_min.SyncJumpWidth = CAN_SYNC_JUMP_WIDTH_MIN;

    timing_cfg_max.TimeSeg1 = CAN_TIME_SEG1_MAX;
    timing_cfg_max.TimeSeg2 = CAN_TIME_SEG2_MAX;
    timing_cfg_max.Prescaler = CAN_PRESCALER_MAX;
    timing_cfg_max.SyncJumpWidth = CAN_SYNC_JUMP_WIDTH_MAX;

    total_tq = CAN_SYNC_SEG + timing_cfg_max.TimeSeg1 + timing_cfg_max.TimeSeg2;
    prescaler = HAL_MAX(clock_freq / (total_tq * bitrate), (uint32_t)timing_cfg_min.Prescaler);
    for (; prescaler <= timing_cfg_max.Prescaler; prescaler++)
    {
        total_tq = clock_freq / (prescaler * bitrate);

        if ((total_tq * prescaler * bitrate) != clock_freq)
        {
            /* No integer total_tq for this prescaler setting */
            continue;
        }

        err = CAN_UpdateSamplePnt(total_tq, sample_pnt, &timing_cfg_min, &timing_cfg_max, &timing_cfg);
        if (err < 0)
        {
            /* Sample point cannot be met for this prescaler setting */
            continue;
        }

        if (err < err_min)
        {
            /* Improved sample point match */
            err_min = err;
            hcan->Init.TimeSeg1 = timing_cfg.TimeSeg1;
            hcan->Init.TimeSeg2 = timing_cfg.TimeSeg2;
            hcan->Init.Prescaler = (uint16_t)prescaler;

            if (err == 0)
            {
                /* Perfect sample point match */
                break;
            }
        }
    }

    /* Calculate default sjw as phase_seg2 / 2 and clamp the result
     * CAN controller requirement: sjw <= seg2
     */
    hcan->Init.SyncJumpWidth = hcan->Init.TimeSeg2 / 2;
    hcan->Init.SyncJumpWidth = HAL_CLAMP(hcan->Init.SyncJumpWidth, timing_cfg_min.SyncJumpWidth, timing_cfg_max.SyncJumpWidth);

    return err_min == INT32_MAX ? -HAL_ERROR : err_min;
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

static void HAL_CAN_FillFrame(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[])
{
    HAL_CAN_FrameTypeDef frame;
    uint32_t i;
    __IO uint32_t *tbuf;
    uint32_t *src;
    uint32_t mask;

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

    /* make sure NumOfTxMsgs update and STB write is atomic */
    mask = HAL_DisableInterrupt();
    if (hcan->Instance->CR & CAN_CR_TBSEL)
    {
        /* Increment the number of transmit messages in the secondary buffer */
        hcan->NumOfTxMsgs++;
    }
    /* make sure word aligned write */
    tbuf = &hcan->Instance->TBUF;
    src = (uint32_t *)&frame;
    for (i = 0; i < CAN_FRAME_WORD_SIZE; i++)
    {
        tbuf[i] = src[i];
    }
    HAL_EnableInterrupt(mask);
}

HAL_StatusTypeDef HAL_CAN_AddPrimaryTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[])
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

HAL_StatusTypeDef HAL_CAN_AddSecondaryTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[])
{
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

    /*
     * TSALL starts transmission of all frames currently in STB.
     * If TSALL is set before TSNEXT commits the frame, the hardware
     * sees an empty STB, completes 0-frame transmission, and fires
     * TSIF — leaving the just-committed frame stuck until the next
    */
    hcan->Instance->CR |= CAN_CR_TSNEXT;

    /* transmit all */
    hcan->Instance->CR |= CAN_CR_TSALL;

    return HAL_OK;
}

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

uint32_t HAL_CAN_GetTxFifoFreeLevel(CAN_HandleTypeDef *hcan, bool primary)
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

void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan)
{
    uint32_t ir = hcan->Instance->IR;
    int32_t msg_num;

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
        HAL_CAN_RxFifoOverflowCallback(hcan);
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
        HAL_CAN_RxFifoAlmostFullCallback(hcan);
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
    if ((ir & CAN_IR_TSIF)
            && (0 == (hcan->Instance->CR & CAN_CR_TSSTAT))) /* all STB frames transmitted */
    {
        msg_num = hcan->NumOfTxMsgs;
        hcan->NumOfTxMsgs = 0;
        for (; msg_num > 0; msg_num--)
        {
            /* Call the STB complete callback for each transmitted message */
            HAL_CAN_TxSecondaryCompleteCallback(hcan);
        }
    }

    /* PTB transmission complete — TPIF auto-clears on next TPE */
    if (ir & CAN_IR_TPIF)
    {
        HAL_CAN_TxPrimaryCompleteCallback(hcan);
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
      (+) HAL_CAN_TxPrimaryCompleteCallback
      (+) HAL_CAN_TxSecondaryCompleteCallback
      (+) HAL_CAN_RxMsgPendingCallback
      (+) HAL_CAN_RxFifoFullCallback
      (+) HAL_CAN_RxFifoAlmostFullCallback
      (+) HAL_CAN_RxFifoOverflowCallback
      (+) HAL_CAN_ErrorCallback

@endverbatim
  * @{
  */

__weak void HAL_CAN_TxPrimaryCompleteCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
}

__weak void HAL_CAN_TxSecondaryCompleteCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
}

__weak void HAL_CAN_RxMsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
    /* Silent by default — fires every frame. Data already in software FIFO.
       Override to add per-frame logging. */
}

__weak void HAL_CAN_RxFifoFullCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
}

__weak void HAL_CAN_RxFifoAlmostFullCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
}

__weak void HAL_CAN_RxFifoOverflowCallback(CAN_HandleTypeDef *hcan)
{
    UNUSED(hcan);
}

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
