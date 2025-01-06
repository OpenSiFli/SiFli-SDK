/**
  ******************************************************************************
  * @file   bf0_hal_ext_dma.c
  * @author Sifli software development team
  * @brief EXT_DMA HAL module driver
  * @{
  ******************************************************************************
*/
/**
 *
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

#include "bf0_hal.h"
#include "bf0_hal_ext_dma.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup ext_dma Ext DMA
  * @brief EXT_DMA HAL module driver
  * @{
  */

#if defined(HAL_EXTDMA_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define HAL_EXT_DMA_SRC_BIT_CNT8        (0)
#define HAL_EXT_DMA_SRC_BIT_CNT16       (1<<EXTDMA_CCR_SRCSIZE_Pos)
#define HAL_EXT_DMA_SRC_BIT_CNT32       (2<<EXTDMA_CCR_SRCSIZE_Pos)

#define HAL_EXT_DMA_DST_BIT_CNT8        (0)
#define HAL_EXT_DMA_DST_BIT_CNT16       (1<<EXTDMA_CCR_DSTSIZE_Pos)
#define HAL_EXT_DMA_DST_BIT_CNT32       (2<<EXTDMA_CCR_DSTSIZE_Pos)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup EXT_DMA_Private_Functions EXT_DMA Private Functions
  * @{
  */
static void EXT_DMA_SetConfig(EXT_DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts);
/**
  * @} EXT_DMA_Private_Functions
  */

/** @defgroup EXT_DMA_Exported_Functions Ext DMA Exported Functions
  * @{
  */

/** @defgroup EXT_DMA_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief   Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
             ##### Initialization and de-initialization functions  #####
 ===============================================================================
    [..]
    This section provides functions allowing to initialize the DMA Channel source
    and destination addresses, incrementation and data sizes, transfer direction,
    circular/normal mode selection, memory-to-memory mode selection and Channel priority value.
    [..]
    The HAL_DMA_Init() function follows the DMA configuration procedures as described in
    reference manual.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the DMA according to the specified
  *         parameters in the DMA_InitTypeDef and initialize the associated handle.
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_Init(EXT_DMA_HandleTypeDef *hdma)
{
    uint32_t tmp;

    /* Check the DMA handle allocation */
    if (hdma == NULL)
    {
        return HAL_ERROR;
    }

    /* single handle for ext dma*/
    hdma->Instance = hwp_extdma;

    /* Change DMA peripheral state */
    hdma->State = HAL_EXT_DMA_STATE_BUSY;

    /* Get the CR register value */
    tmp = hdma->Instance->CCR;

    /* Clear SRCSIZE,DSTSIZE,SRCBURST, DSTBURST,SRCINC, DSTINC, INTR,EN bits */
    tmp &= ((uint32_t)~(EXTDMA_CCR_DSTSIZE    | EXTDMA_CCR_SRCSIZE |
                        EXTDMA_CCR_SRCBURST | EXTDMA_CCR_DSTBURST |
                        EXTDMA_CCR_SRCINC  | EXTDMA_CCR_DSTINC  |
                        EXTDMA_CCR_TEIE | EXTDMA_CCR_TCIE | EXTDMA_CCR_HTIE | EXTDMA_CCR_OFIE |
                        EXTDMA_CCR_EN));

    /* Prepare the EXT DMA transfer configuration */
    /* burst should set 16 for high performance, cnt should fixed to 32*/
    tmp |=  hdma->Init.SrcInc        | hdma->Init.DstInc  |
            HAL_EXT_DMA_SRC_BIT_CNT32 | HAL_EXT_DMA_DST_BIT_CNT32;
    //hdma->Init.SrcAddr  | hdma->Init.DstAddr;

    /* Write to DMA Channel CR register */
    hdma->Instance->CCR = tmp;

    if (hdma->Init.cmpr_en)
    {
#ifdef SF32LB52X
        HAL_ASSERT(0);//Compression is not supported.
#else
        //RGB565 compression
        HAL_ASSERT((EXTDMA_CMPRCR_SRCFMT_RGB565 == hdma->Init.src_format)
                   || (EXTDMA_CMPRCR_SRCFMT_RGB888 == hdma->Init.src_format)
                   || (EXTDMA_CMPRCR_SRCFMT_ARGB8888 == hdma->Init.src_format));

        hdma->Instance->CMPRCR = hdma->Init.src_format | (0x0 << EXTDMA_CMPRCR_SRCPOS_Pos) | EXTDMA_CMPRCR_CMPREN;
#endif /* SF32LB52X */
    }
    else
    {
        hdma->Instance->CMPRCR = 0;
    }


    /* Initialise the error code */
    hdma->ErrorCode = HAL_EXT_DMA_ERROR_NONE;

    /* Initialize the DMA state*/
    hdma->State  = HAL_EXT_DMA_STATE_READY;

    /* Allocate lock resource and initialize it */
    hdma->Lock = HAL_UNLOCKED;

    return HAL_OK;
}

/**
  * @brief  DeInitialize the EXT_DMA peripheral.
  * @param  hdma pointer to a EXT_DMA_HandleTypeDef structure that contains
  *               the configuration information.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_DeInit(EXT_DMA_HandleTypeDef *hdma)
{

    /* Check the DMA handle allocation */
    if (NULL == hdma)
    {
        return HAL_ERROR;
    }

    /* Reset DMA Channel control register */
    hdma->Instance->CCR  = 0;

    /* Clear all flags */
    hdma->Instance->IFCR = EXTDMA_IFCR_CGIF;

    /* Clean callbacks */
    hdma->XferCpltCallback = NULL;
    hdma->XferHalfCpltCallback = NULL;
    hdma->XferErrorCallback = NULL;
    hdma->XferAbortCallback = NULL;

    /* Initialise the error code */
    hdma->ErrorCode = HAL_EXT_DMA_ERROR_NONE;

    /* Initialize the DMA state */
    hdma->State = HAL_EXT_DMA_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hdma);

    return HAL_OK;
}

/**
  * @} EXT_DMA_Exported_Functions_Group1
  */

/** @defgroup EXT_DMA_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Input and Output operation functions
 *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and Start DMA transfer
      (+) Configure the source, destination address and data length and
          Start DMA transfer with interrupt
      (+) Abort DMA transfer
      (+) Poll for transfer complete
      (+) Handle DMA interrupt request

@endverbatim
  * @{
  */

/**
  * @brief  Start the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  Counts The counts of data transfer action
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_Start(EXT_DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (Counts > HAL_EXT_DMA_SINGLE_MAX)
    {
        hdma->ErrorCode = HAL_EXT_DMA_ERROR_SIZE_ERR;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_EXT_DMA_STATE_READY == hdma->State)
    {
        /* Change DMA peripheral state */
        hdma->State = HAL_EXT_DMA_STATE_BUSY;
        hdma->ErrorCode = HAL_EXT_DMA_ERROR_NONE;

        /* Disable the peripheral */
        __HAL_EXT_DMA_DISABLE(hdma);

        /* Configure the source, destination address and the data length & clear flags*/
        EXT_DMA_SetConfig(hdma, SrcAddress, DstAddress, Counts);

        if (hdma->Init.SrcInc & HAL_EXT_DMA_SRC_INC)
            mpu_dcache_clean((void *)SrcAddress, Counts << 2);
        else
            mpu_dcache_clean((void *)SrcAddress, 4);

        /* Enable the Peripheral */
        __HAL_EXT_DMA_ENABLE(hdma);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
        status = HAL_BUSY;
    }
    return status;
}

/**
  * @brief  Start the DMA Transfer with interrupt enabled.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  Counts The counts of data transfer action
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_Start_IT(EXT_DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (Counts > HAL_EXT_DMA_SINGLE_MAX)
    {
        hdma->ErrorCode = HAL_EXT_DMA_ERROR_SIZE_ERR;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_EXT_DMA_STATE_READY == hdma->State)
    {
        /* Change DMA peripheral state */
        hdma->State = HAL_EXT_DMA_STATE_BUSY;
        hdma->ErrorCode = HAL_EXT_DMA_ERROR_NONE;

        /* Disable the peripheral */
        __HAL_EXT_DMA_DISABLE(hdma);

        /* Configure the source, destination address and the data length & clear flags*/
        EXT_DMA_SetConfig(hdma, SrcAddress, DstAddress, Counts);

        /* Enable the transfer complete interrupt */
        /* Enable the transfer Error interrupt */
        if (NULL != hdma->XferHalfCpltCallback)
        {
            /* Enable the Half transfer complete interrupt as well */
            __HAL_EXT_DMA_ENABLE_IT(hdma, (EXTDMA_ISR_TCIF | EXTDMA_ISR_HTIF | EXTDMA_ISR_TEIF | EXTDMA_ISR_OFIF));
            //__HAL_EXT_DMA_ENABLE_IT(hdma, (EXTDMA_ISR_TCIF | EXTDMA_ISR_HTIF | EXTDMA_ISR_TEIF));
        }
        else
        {
            __HAL_EXT_DMA_DISABLE_IT(hdma, EXTDMA_ISR_HTIF);
            __HAL_EXT_DMA_ENABLE_IT(hdma, (EXTDMA_ISR_TCIF | EXTDMA_ISR_TEIF | EXTDMA_ISR_OFIF));
            //__HAL_EXT_DMA_ENABLE_IT(hdma, (EXTDMA_ISR_TCIF | EXTDMA_ISR_TEIF));
        }

        if (hdma->Init.SrcInc & HAL_EXT_DMA_SRC_INC)
            mpu_dcache_clean((void *)SrcAddress, Counts << 2);
        else
            mpu_dcache_clean((void *)SrcAddress, 4);

        //HAL_sw_breakpoint();

        /* Enable the Peripheral */
        __HAL_EXT_DMA_ENABLE(hdma);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        /* Remain BUSY */
        status = HAL_BUSY;
    }
    return status;
}

/**
  * @brief  Abort the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
    * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_Abort(EXT_DMA_HandleTypeDef *hdma)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the DMA peripheral handle */
    if (NULL == hdma)
    {
        return HAL_ERROR;
    }

    /* Disable DMA IT */
    __HAL_EXT_DMA_DISABLE_IT(hdma, (EXTDMA_ISR_TCIF | EXTDMA_ISR_HTIF | EXTDMA_ISR_TEIF | EXTDMA_ISR_OFIF));


    /* Disable the channel */
    __HAL_EXT_DMA_DISABLE(hdma);

    /* Clear all flags */
    hdma->Instance->IFCR = EXTDMA_IFCR_CGIF;

    /* Change the DMA state */
    hdma->State = HAL_EXT_DMA_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hdma);

    return status;
}

/**
  * @brief  Aborts the DMA Transfer in Interrupt mode.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_Abort_IT(EXT_DMA_HandleTypeDef *hdma)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (HAL_EXT_DMA_STATE_BUSY != hdma->State)
    {
        /* no transfer ongoing */
        hdma->ErrorCode = HAL_EXT_DMA_ERROR_NO_XFER;

        status = HAL_ERROR;
    }
    else
    {
        /* Disable DMA IT */
        __HAL_EXT_DMA_DISABLE_IT(hdma, (EXTDMA_ISR_TCIF | EXTDMA_ISR_HTIF | EXTDMA_ISR_TEIF | EXTDMA_ISR_OFIF));

        /* Disable the channel */
        __HAL_EXT_DMA_DISABLE(hdma);

        /* Clear all flags */
        hdma->Instance->IFCR = EXTDMA_IFCR_CGIF;

        /* Change the DMA state */
        hdma->State = HAL_EXT_DMA_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        /* Call User Abort callback */
        if (hdma->XferAbortCallback != NULL)
        {
            hdma->XferAbortCallback(hdma);
        }
    }
    return status;
}

/**
  * @brief  Polling for transfer complete.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                  the configuration information for the specified DMA Channel.
  * @param  CompleteLevel Specifies the DMA level complete.
  * @param  Timeout       Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_PollForTransfer(EXT_DMA_HandleTypeDef *hdma, HAL_EXT_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout)
{
    uint32_t temp;
    uint32_t tickstart;

    if (HAL_EXT_DMA_STATE_BUSY != hdma->State)
    {
        /* no transfer ongoing */
        hdma->ErrorCode = HAL_EXT_DMA_ERROR_NO_XFER;
        __HAL_UNLOCK(hdma);
        return HAL_ERROR;
    }

    /* Get the level transfer complete flag */
    if (HAL_EXT_DMA_FULL_TRANSFER == CompleteLevel)
    {
        /* Transfer Complete flag */
        temp = EXTDMA_ISR_TCIF;
    }
    else
    {
        /* Half Transfer Complete flag */
        temp = EXTDMA_ISR_HTIF;
    }

    /* Get tick */
    tickstart = HAL_GetTick();

    while (0U == (hdma->Instance->ISR & temp))
    {
        if ((0U != (hdma->Instance->ISR & (EXTDMA_ISR_TEIF | EXTDMA_ISR_OFIF))))
        {
            /* When a EXT_DMA transfer error occurs */
            /* A hardware clear of its EN bits is performed */
            /* Clear all flags */
            hdma->Instance->IFCR = EXTDMA_IFCR_CGIF;

            /* Update error code */
            hdma->ErrorCode = HAL_EXT_DMA_ERROR_TE;

            /* Change the DMA state */
            hdma->State = HAL_EXT_DMA_STATE_READY;

            /* Process Unlocked */
            __HAL_UNLOCK(hdma);

            return HAL_ERROR;
        }
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
            {
                /* Update error code */
                hdma->ErrorCode = HAL_EXT_DMA_ERROR_TIMEOUT;

                /* Change the DMA state */
                hdma->State = HAL_EXT_DMA_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(hdma);

                return HAL_ERROR;
            }
        }
    }

    if (HAL_EXT_DMA_FULL_TRANSFER == CompleteLevel)
    {
        /* Clear the transfer complete flag */
        hdma->Instance->IFCR = EXTDMA_IFCR_CTCIF;

        /* The selected Channelx EN bit is cleared (DMA is disabled and
        all transfers are complete) */
        hdma->State = HAL_EXT_DMA_STATE_READY;
    }
    else
    {
        /* Clear the half transfer complete flag */
        hdma->Instance->IFCR = EXTDMA_IFCR_CHTIF;
    }

    /* Process unlocked */
    __HAL_UNLOCK(hdma);

    return HAL_OK;
}

/**
  * @brief  Handle DMA interrupt request.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval None
  */
__HAL_ROM_USED void HAL_EXT_DMA_IRQHandler(EXT_DMA_HandleTypeDef *hdma)
{
    uint32_t flag_it = hdma->Instance->ISR;
    uint32_t source_it = hdma->Instance->CCR;

    /* Half Transfer Complete Interrupt management ******************************/
    if ((0U != (flag_it & EXTDMA_ISR_HTIF)) && (0U != (source_it & EXTDMA_CCR_HTIE)))
    {
        /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
        /* Disable the half transfer interrupt */
        __HAL_EXT_DMA_DISABLE_IT(hdma, EXTDMA_CCR_HTIE);
        /* Clear the half transfer complete flag */
        hdma->Instance->IFCR = EXTDMA_IFCR_CHTIF;

        /* DMA peripheral state is not updated in Half Transfer */
        /* but in Transfer Complete case */

        if (hdma->XferHalfCpltCallback != NULL)
        {
            /* Half transfer callback */
            hdma->XferHalfCpltCallback(hdma);
        }
    }

    /* Transfer Complete Interrupt management ***********************************/
    else if ((0U != (flag_it & EXTDMA_ISR_TCIF)) && (0U != (source_it & EXTDMA_CCR_TCIE)))
    {
        /* Disable the transfer complete and error interrupt */
        __HAL_EXT_DMA_DISABLE_IT(hdma, EXTDMA_CCR_TCIE | EXTDMA_CCR_TEIE | EXTDMA_CCR_OFIE);

        /* Change the DMA state */
        hdma->State = HAL_EXT_DMA_STATE_READY;
        /* Clear the transfer complete flag */
        hdma->Instance->IFCR = EXTDMA_IFCR_CTCIF;

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        if (hdma->XferCpltCallback != NULL)
        {
            /* Transfer complete callback */
            hdma->XferCpltCallback(hdma);
        }
    }

    /* Transfer Error Interrupt management **************************************/
    else if ((0U != (flag_it & EXTDMA_ISR_TEIF)) && (0U != (source_it & EXTDMA_CCR_TEIE)))
    {
        /* When a DMA transfer error occurs */
        /* A hardware clear of its EN bits is performed */
        /* Disable ALL DMA IT */
        __HAL_EXT_DMA_DISABLE_IT(hdma, (EXTDMA_CCR_TCIE | EXTDMA_CCR_HTIE | EXTDMA_CCR_TEIE | EXTDMA_CCR_OFIE));

        /* Clear all flags */
        hdma->Instance->IFCR = EXTDMA_IFCR_CTEIF;

        /* Update error code */
        hdma->ErrorCode = HAL_EXT_DMA_ERROR_TE;

        /* Change the DMA state */
        hdma->State = HAL_EXT_DMA_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        if (hdma->XferErrorCallback != NULL)
        {
            /* Transfer error callback */
            hdma->XferErrorCallback(hdma);
        }
    }
    else if ((0U != (flag_it & EXTDMA_ISR_OFIF)) && (0U != (source_it & EXTDMA_CCR_OFIE)))
    {
        /* When a DMA compress overflow occurs, do NOT disable other DMA IT*/
        //__HAL_EXT_DMA_DISABLE_IT(hdma, (EXTDMA_CCR_TCIE | EXTDMA_CCR_HTIE | EXTDMA_CCR_TEIE | EXTDMA_CCR_OFIE));

        /* Clear overflag flag */
        hdma->Instance->IFCR = EXTDMA_IFCR_COFIF;

        /* Update error code */
        hdma->ErrorCode = HAL_EXT_DMA_ERROR_OVERFLOW;

        if (hdma->XferErrorCallback != NULL)
        {
            /* Transfer error callback */
            hdma->XferErrorCallback(hdma);
        }

    }
    else
    {
        /* Nothing To Do */
    }

    return;
}

/**
  * @brief  Register callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Channel.
  * @param  CallbackID           User Callback identifer
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @param  pCallback            pointer to private callbacsk function which has pointer to
  *                               a DMA_HandleTypeDef structure as parameter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_RegisterCallback(EXT_DMA_HandleTypeDef *hdma, HAL_EXT_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(EXT_DMA_HandleTypeDef *_hdma))
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_EXT_DMA_STATE_READY == hdma->State)
    {
        switch (CallbackID)
        {
        case  HAL_EXT_DMA_XFER_CPLT_CB_ID:
            hdma->XferCpltCallback = pCallback;
            break;

        case  HAL_EXT_DMA_XFER_HALFCPLT_CB_ID:
            hdma->XferHalfCpltCallback = pCallback;
            break;

        case  HAL_EXT_DMA_XFER_ERROR_CB_ID:
            hdma->XferErrorCallback = pCallback;
            break;

        case  HAL_EXT_DMA_XFER_ABORT_CB_ID:
            hdma->XferAbortCallback = pCallback;
            break;

        default:
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hdma);

    return status;
}

/**
  * @brief  UnRegister callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Channel.
  * @param  CallbackID           User Callback identifer
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_UnRegisterCallback(EXT_DMA_HandleTypeDef *hdma, HAL_EXT_DMA_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_EXT_DMA_STATE_READY == hdma->State)
    {
        switch (CallbackID)
        {
        case  HAL_EXT_DMA_XFER_CPLT_CB_ID:
            hdma->XferCpltCallback = NULL;
            break;

        case  HAL_EXT_DMA_XFER_HALFCPLT_CB_ID:
            hdma->XferHalfCpltCallback = NULL;
            break;

        case  HAL_EXT_DMA_XFER_ERROR_CB_ID:
            hdma->XferErrorCallback = NULL;
            break;

        case  HAL_EXT_DMA_XFER_ABORT_CB_ID:
            hdma->XferAbortCallback = NULL;
            break;

        case   HAL_EXT_DMA_XFER_ALL_CB_ID:
            hdma->XferCpltCallback = NULL;
            hdma->XferHalfCpltCallback = NULL;
            hdma->XferErrorCallback = NULL;
            hdma->XferAbortCallback = NULL;
            break;

        default:
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hdma);

    return status;
}

/**
  * @} EXT_DMA_Exported_Functions_Group2
  */



/** @defgroup EXT_DMA_Exported_Functions_Group3 Peripheral State and Errors functions
 *  @brief    Peripheral State and Errors functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DMA state
      (+) Get error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the DMA hande state.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL state
  */
HAL_EXT_DMA_StateTypeDef HAL_EXT_DMA_GetState(EXT_DMA_HandleTypeDef *hdma)
{
    /* Return DMA handle state */
    return hdma->State;
}

/**
  * @brief  Return the DMA error code.
  * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Channel.
  * @retval DMA Error Code
  */
uint32_t HAL_EXT_DMA_GetError(EXT_DMA_HandleTypeDef *hdma)
{
    return hdma->ErrorCode;
}

/**
  * @brief  Return the DMA CMPRQR value.
  * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Channel.
  * @retval DMA CMPRQR register value
  */
uint32_t HAL_EXT_DMA_GetCMPRQR(EXT_DMA_HandleTypeDef *hdma)
{
    return hdma->Instance->CMPRQR;
}

/**
  * @brief  Return the DMA CMPRDR value.
  * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Channel.
  * @retval DMA CMPRDR register value
  */
uint32_t HAL_EXT_DMA_GetCMPRDR(EXT_DMA_HandleTypeDef *hdma)
{
    return hdma->Instance->CMPRDR;
}


/**
  * @} EXT_DMA_Exported_Functions_Group3
  */

/**
  * @} EXT_DMA_Exported_Functions
  */

/** @addtogroup DMA_Private_Functions
  * @{
  */

/**
  * @brief  Sets the DMA Transfer parameter.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  Counts The counts of data transfer action
  * @retval HAL status
  */
static void EXT_DMA_SetConfig(EXT_DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts)
{
    EDMA_InitTypeDef *init_param;

    /* Clear all flags */
    hdma->Instance->IFCR = EXTDMA_IFCR_CGIF;

    /* Configure DMA Channel data length */
    hdma->Instance->CNDTR = Counts;

    /* Configure DMA Channel source address */

#ifndef SF32LB55X
    hdma->Instance->SRCAR = HCPU_MPI_SBUS_ADDR(SrcAddress);
#else
    hdma->Instance->SRCAR = SrcAddress;
#endif

    /* Configure DMA Channel destination address */
#ifndef SF32LB55X
    hdma->Instance->DSTAR = HCPU_MPI_SBUS_ADDR(DstAddress);
#else
    hdma->Instance->DSTAR = DstAddress;
#endif

    init_param = &hdma->Init;
    if (init_param->cmpr_en)
    {
#ifdef SF32LB52X
        HAL_ASSERT(0);//Compression is not supported.
#else
        uint32_t compressed_size;
        uint32_t target_size;
        uint32_t cfg0;
        uint32_t cfg1;
        uint8_t pixel_size = 2;
        uint32_t line_words;

        if (EXTDMA_CMPRCR_SRCFMT_RGB565 == init_param->src_format)
        {
            pixel_size = 2;
        }
        else if (EXTDMA_CMPRCR_SRCFMT_RGB888 == init_param->src_format)
        {
            pixel_size = 3;
        }
        else if (EXTDMA_CMPRCR_SRCFMT_ARGB8888 == init_param->src_format)
        {
            pixel_size = 4;
        }

        //HAL_ASSERT(0 == ((Counts * 4) % init_param->row_num));
        line_words = Counts / init_param->row_num;

        HAL_EXT_DMA_CalcCompressedSize(line_words, init_param->cmpr_rate,
                                       1, pixel_size,
                                       &compressed_size, &target_size);
        HAL_ASSERT(target_size <= EXTDMA_CMPRSR_TGTSIZE_Msk);

        hdma->Instance->CMPRSR = MAKE_REG_VAL(init_param->col_num, EXTDMA_CMPRSR_LINESIZE_Msk, EXTDMA_CMPRSR_LINESIZE_Pos)
                                 | MAKE_REG_VAL(target_size, EXTDMA_CMPRSR_TGTSIZE_Msk, EXTDMA_CMPRSR_TGTSIZE_Pos);

        hdma->Instance->CMPRNDTR = compressed_size * init_param->row_num;



        HAL_EXT_DMA_GetConfig(target_size, &cfg0, &cfg1);
        hdma->Instance->CMPRCFG0 = cfg0;
        hdma->Instance->CMPRCFG1 = cfg1;
#endif /* SF32LB52X */
    }
}

HAL_StatusTypeDef HAL_EXT_DMA_GetConfig(uint32_t target_size, uint32_t *cfg0, uint32_t *cfg1)
{
    if (!cfg0 || !cfg1)
    {
        return HAL_ERROR;
    }
#if 0
    switch (target_size)
    {
    case 42:
    {
        *cfg0 = 0x46623304;
        *cfg1 = 0x01066AA2;
        break;
    }
    case 44:
    {
        *cfg0 = 0x46624304;
        *cfg1 = 0x01066AA2;
        break;
    }
    case 46:
    {
        *cfg0 = 0x46633304;
        *cfg1 = 0x01066AA2;
        break;
    }
    case 48:
    {
        *cfg0 = 0x46634304;
        *cfg1 = 0x01066AA2;

        break;
    }
    case 50:
    {
        *cfg0 = 0x46634304;
        *cfg1 = 0x01066AA2;

        break;
    }
    case 52:
    {
        *cfg0 = 0x46634304;
        *cfg1 = 0x01066AA2;
        break;
    }
    case 54:
    {
        *cfg0 = 0x46634304;
        *cfg1 = 0x01066AA2;
        break;
    }
    case 56:
    {
        *cfg0 = 0x46634304;
        *cfg1 = 0x01066AA2;
        break;
    }
    default:
    {
        *cfg0 = 0x40023345;
        *cfg1 = 0x01044aa2;
    }
    }
#elif 0
    switch (target_size)
    {
    case 42:
    {
        *cfg0 = 0x86623304;
        *cfg1 = 0x010666A4;
        break;
    }
    case 44:
    {
        *cfg0 = 0x86624304;
        *cfg1 = 0x010666A4;
        break;
    }
    case 46:
    {
        *cfg0 = 0x86633304;
        *cfg1 = 0x010666A4;
        break;
    }
    case 48:
    {
        *cfg0 = 0x86634304;
        *cfg1 = 0x010666A4;

        break;
    }
    case 50:
    {
        *cfg0 = 0x86634304;
        *cfg1 = 0x010666A4;

        break;
    }
    case 52:
    {
        *cfg0 = 0x86634304;
        *cfg1 = 0x010666A4;
        break;
    }
    case 54:
    {
        *cfg0 = 0x86635304;
        *cfg1 = 0x010666A4;
        break;
    }
    case 56:
    {
        *cfg0 = 0x86635304;
        *cfg1 = 0x010666A4;
        break;
    }
    default:
    {
        *cfg0 = 0x40023345;
        *cfg1 = 0x01044aa2;
    }
    }

#elif 0

    switch (target_size)
    {
    case 42:
    {
        *cfg0 = 0xC7723304;
        *cfg1 = 0x010777B8;
        break;
    }
    case 44:
    {
        *cfg0 = 0xC7724304;
        *cfg1 = 0x010777B8;
        break;
    }
    case 46:
    {
        *cfg0 = 0xC7733304;
        *cfg1 = 0x010777B8;
        break;
    }
    case 48:
    {
        *cfg0 = 0xC7734304;
        *cfg1 = 0x010777B8;

        break;
    }
    case 50:
    {
        *cfg0 = 0xC7734304;
        *cfg1 = 0x010777B8;

        break;
    }
    case 52:
    {
        *cfg0 = 0xC7734304;
        *cfg1 = 0x010777B8;
        break;
    }
    case 54:
    {
        *cfg0 = 0xC7734304;
        *cfg1 = 0x010777B8;
        break;
    }
    case 56:
    {
        *cfg0 = 0xC7735304;
        *cfg1 = 0x010777B8;
        break;
    }
    default:
    {
        *cfg0 = 0x40023344;
        *cfg1 = 0x01044aa2;
    }
    }

#elif 0
    switch (target_size)
    {
    case 42:
    {
        *cfg0 = 0xC7723305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 44:
    {
        *cfg0 = 0xC7724305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 46:
    {
        *cfg0 = 0xC7733305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 48:
    {
        *cfg0 = 0xC7734305;
        *cfg1 = 0x010777B8;

        break;
    }
    case 50:
    {
        *cfg0 = 0xC7734305;
        *cfg1 = 0x010777B8;

        break;
    }
    case 52:
    {
        *cfg0 = 0xC7734305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 54:
    {
        *cfg0 = 0xC7734305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 56:
    {
        *cfg0 = 0xC7735305;
        *cfg1 = 0x010777B8;
        break;
    }
    default:
    {
        *cfg0 = 0x40023345;
        *cfg1 = 0x01044aa2;
    }
    }
#elif 0

    switch (target_size)
    {
    case 42:
    {
        *cfg0 = 0xC7723305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 44:
    {
        *cfg0 = 0xC7723305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 46:
    {
        *cfg0 = 0xC7733305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 48:
    {
        *cfg0 = 0xC7733305;
        *cfg1 = 0x010777B8;

        break;
    }
    case 50:
    {
        *cfg0 = 0xC7733305;
        *cfg1 = 0x010777B8;

        break;
    }
    case 52:
    {
        *cfg0 = 0xC7734305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 54:
    {
        *cfg0 = 0xC7734305;
        *cfg1 = 0x010777B8;
        break;
    }
    case 56:
    {
        *cfg0 = 0xC7734305;
        *cfg1 = 0x010777B8;
        break;
    }
    default:
    {
        *cfg0 = 0x40023345;
        *cfg1 = 0x01044aa2;
    }
    }
#else

    *cfg0 = 0x40023344; //0x40023345;
    *cfg1 = 0x01044aa2;

#endif
    return HAL_OK;
}

/**
  * @} DMA_Private_Functions
  */

#endif /* BSP_USING_EXT_DMA */
/**
  * @} ext_dma
  */

/**
  * @} BF0_HAL_Driver
  */


/**
  * @} file
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
