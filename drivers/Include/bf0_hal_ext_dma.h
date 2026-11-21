/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */

#ifndef __BF0_HAL_EXT_DMA_H
#define __BF0_HAL_EXT_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

extern float ceilf(float arg);


/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup ext_dma EXT DMA
 * @brief External DMA to support fast data movement among flash, PSRAM, and SRAM.
 * @{
 */

/** @defgroup EXT_DMA_Exported_Types EXT DMA Exported Types
 * @{
 */



/**
 * @brief  EXT_DMA Configuration Structure definition
 */
typedef struct
{
    uint32_t SrcInc;      /*!< Source address increase after every DMA transfer
                            This parameter can be a combination of burst length and increasement in @ref EXT_DMA_Src_Inc*/

    uint32_t DstInc;      /*!< Destination address increase after every DMA transfer
                                This parameter can be a combination of burst length and increasement in @ref EXT_DMA_Dst_Inc*/

    bool     cmpr_en;     /*!< Enable compress while DMA transfer    */
    uint8_t  cmpr_rate;   /*!< Compress rate :  1 ~ 9 */
    uint16_t col_num;     /*!< Buffer column number in pixels                       */
    uint16_t row_num;     /*!< Buffer row number in pixels                       */
    uint32_t src_format;  /*!< Buffer color format
                            This parameter can be a value of @ref EXT_DMA_Src_Format */
} EDMA_InitTypeDef;

/**
 * @brief  HAL EXT_DMA State structures definition
 */
typedef enum
{
    HAL_EXT_DMA_STATE_RESET             = 0x00U,  /*!< DMA not yet initialized or disabled    */
    HAL_EXT_DMA_STATE_READY             = 0x01U,  /*!< DMA initialized and ready for use      */
    HAL_EXT_DMA_STATE_BUSY              = 0x02U,  /*!< DMA process is ongoing                 */
    HAL_EXT_DMA_STATE_TIMEOUT           = 0x03U,  /*!< DMA timeout state                     */
} HAL_EXT_DMA_StateTypeDef;

/**
 * @brief  HAL EXT_DMA Error Code structure definition
 */
typedef enum
{
    HAL_EXT_DMA_FULL_TRANSFER      = 0x00U,    /*!< Full transfer     */
    HAL_EXT_DMA_HALF_TRANSFER      = 0x01U     /*!< Half Transfer     */
} HAL_EXT_DMA_LevelCompleteTypeDef;


/**
 * @brief  HAL EXT_DMA Callback ID structure definition
 */
typedef enum
{
    HAL_EXT_DMA_XFER_CPLT_CB_ID          = 0x00U,    /*!< Full transfer     */
    HAL_EXT_DMA_XFER_HALFCPLT_CB_ID      = 0x01U,    /*!< Half transfer     */
    HAL_EXT_DMA_XFER_ERROR_CB_ID         = 0x02U,    /*!< Error             */
    HAL_EXT_DMA_XFER_ABORT_CB_ID         = 0x03U,    /*!< Abort             */
    HAL_EXT_DMA_XFER_ALL_CB_ID           = 0x04U     /*!< All               */
} HAL_EXT_DMA_CallbackIDTypeDef;

/**
 * @brief  EXT_DMA handle Structure definition
 */
typedef struct __EXT_DMA_HandleTypeDef
{
    EXTDMA_TypeDef    *Instance;                                                  /*!< Register base address                */

    EDMA_InitTypeDef       Init;                                                        /*!< DMA communication parameters         */

    HAL_LockTypeDef       Lock;                                                        /*!< DMA locking object                   */

    __IO HAL_EXT_DMA_StateTypeDef  State;                                                  /*!< DMA transfer state                   */

    void (* XferCpltCallback)(struct __EXT_DMA_HandleTypeDef *hdma);                       /*!< DMA transfer complete callback       */

    void (* XferHalfCpltCallback)(struct __EXT_DMA_HandleTypeDef *hdma);                   /*!< DMA Half transfer complete callback  */

    void (* XferErrorCallback)(struct __EXT_DMA_HandleTypeDef *hdma);                      /*!< DMA transfer error callback          */

    void (* XferAbortCallback)(struct __EXT_DMA_HandleTypeDef *hdma);                      /*!< DMA transfer abort callback          */

    __IO uint32_t          ErrorCode;                                                  /*!< DMA Error code                       */


} EXT_DMA_HandleTypeDef;

/**
 * @}EXT_DMA_Exported_Types
 */

/* Exported constants --------------------------------------------------------*/

/** @defgroup EXT_DMA_Exported_Constants EXT DMA Exported Constants
 * @ingroup ext_dma
 * @{
 */

/** @defgroup EXT_DMA_Src_Format EXT_DMA source buffer format
 * @brief
 * @{
 */
#define EDMA_SRCFMT_RGB565    EXTDMA_CMPRCR_SRCFMT_RGB565
#define EDMA_SRCFMT_RGB888    EXTDMA_CMPRCR_SRCFMT_RGB888
#define EDMA_SRCFMT_ARGB8888  EXTDMA_CMPRCR_SRCFMT_ARGB8888
/**
 * @}
 */

/** @defgroup EXT_DMA_Error_Code DMA Error Code
 * @{
 */
#define HAL_EXT_DMA_ERROR_NONE                 0x00000000U    /*!< No error                                */
#define HAL_EXT_DMA_ERROR_TE                   0x00000001U    /*!< Transfer error                          */
#define HAL_EXT_DMA_ERROR_NO_XFER              0x00000004U    /*!< Abort requested with no Xfer ongoing    */
#define HAL_EXT_DMA_ERROR_TIMEOUT              0x00000020U    /*!< Timeout error                           */
#define HAL_EXT_DMA_ERROR_NOT_SUPPORTED        0x00000100U    /*!< Not supported mode                      */
#define HAL_EXT_DMA_ERROR_SIZE_ERR             0x00000200U    /*!< Transfer size  error                    */
#define HAL_EXT_DMA_ERROR_OVERFLOW             0x00000400U    /*!< Overflow error occurred                 */

/**
 * @}
 */

/** @defgroup EXT_DMA_Src_Inc EXT_DMA source incresement configuration
 * @brief For example you can use ``HAL_EXT_DMA_SRC_INC | HAL_EXT_DMA_SRC_BURST16`` while copy a buffer to another.
 * @{
 */
#define HAL_EXT_DMA_SRC_BURST1          (0)
#define HAL_EXT_DMA_SRC_BURST4          (1<<EXTDMA_CCR_SRCBURST_Pos)
#define HAL_EXT_DMA_SRC_BURST8          (2<<EXTDMA_CCR_SRCBURST_Pos)
#define HAL_EXT_DMA_SRC_BURST16         (EXTDMA_CCR_SRCBURST)           /*!< Burst 16 while   */

#define HAL_EXT_DMA_SRC_INC             (EXTDMA_CCR_SRCINC)             /*!< Source address increase itself   */
/**
 * @}
 */


/** @defgroup EXT_DMA_Dst_Inc EXT_DMA destination incresement configuration
 * @brief For example you can use ``HAL_EXT_DMA_DST_BURST16 | HAL_EXT_DMA_DST_INC`` while copy a buffer to another.
 * @{
 */

#define HAL_EXT_DMA_DST_BURST1          (0)
#define HAL_EXT_DMA_DST_BURST4          (1<<EXTDMA_CCR_DSTBURST_Pos)
#define HAL_EXT_DMA_DST_BURST8          (2<<EXTDMA_CCR_DSTBURST_Pos)
#define HAL_EXT_DMA_DST_BURST16         (EXTDMA_CCR_DSTBURST)

#define HAL_EXT_DMA_DST_INC             (EXTDMA_CCR_DSTINC)
/**
 * @}
 */







#define HAL_EXT_DMA_SINGLE_MAX          (0X100000 - 1)


/**
 * @}EXT_DMA_Exported_Constants
 */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup EXT_DMA_Exported_Macros EXT DMA Exported Macros
 * @ingroup ext_dma
 * @{
 */

/** @brief  Reset EXT_DMA handle state.
 * @param  \__HANDLE__: DMA handle
 * @retval None
 */
#define __HAL_EXT_DMA_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_EXT_DMA_STATE_RESET)

/**
 * @brief  Enable the EXT_DMA .
 * @param  \__HANDLE__: DMA handle
 * @retval None
 */
#define __HAL_EXT_DMA_ENABLE(__HANDLE__)        ((__HANDLE__)->Instance->CCR |=  EXTDMA_CCR_EN)

/**
 * @brief  Disable the specified DMA Channel.
 * @param  \__HANDLE__: DMA handle
 * @retval None
 */
#define __HAL_EXT_DMA_DISABLE(__HANDLE__)       ((__HANDLE__)->Instance->CCR &=  ~EXTDMA_CCR_EN)


/**
 * @brief  Get the DMA Channel pending flags.
 * @param  \__HANDLE__: DMA handle
 * @param  \__FLAG__: Get the specified flag.
 *          This parameter can be any combination of the following values:
 *            @arg DMA_FLAG_TCx:  Transfer complete flag
 *            @arg DMA_FLAG_HTx:  Half transfer complete flag
 *            @arg DMA_FLAG_TEx:  Transfer error flag
 *            @arg DMA_FLAG_GLx:  Global interrupt flag
 *         Where x can be from 1 to 7 to select the DMA Channel x flag.
 * @retval The state of FLAG (SET or RESET).
 */
#define __HAL_EXT_DMA_GET_FLAG(__HANDLE__, __FLAG__) ((uint32_t)((__HANDLE__)->Instance->ISR & (__FLAG__)))

/**
 * @brief  Clear the DMA Channel pending flags.
 * @param  \__HANDLE__: DMA handle
 * @param  \__FLAG__: specifies the flag to clear.
 *          This parameter can be any combination of the following values:
 *            @arg DMA_FLAG_TCx:  Transfer complete flag
 *            @arg DMA_FLAG_HTx:  Half transfer complete flag
 *            @arg DMA_FLAG_TEx:  Transfer error flag
 *            @arg DMA_FLAG_GLx:  Global interrupt flag
 *         Where x can be from 1 to 7 to select the DMA Channel x flag.
 * @retval None
 */
#define __HAL_EXT_DMA_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->IFCR = (__FLAG__))

/**
 * @brief  Enable the specified DMA Channel interrupts.
 * @param  \__HANDLE__: DMA handle
 * @param \__INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled.
 *          This parameter can be any combination of the following values:
 *            @arg DMA_IT_TC:  Transfer complete interrupt mask
 *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
 *            @arg DMA_IT_TE:  Transfer error interrupt mask
 * @retval None
 */
#define __HAL_EXT_DMA_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->CCR |= (__INTERRUPT__))

/**
 * @brief  Disable the specified DMA Channel interrupts.
 * @param  \__HANDLE__: DMA handle
 * @param \__INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled.
 *          This parameter can be any combination of the following values:
 *            @arg DMA_IT_TC:  Transfer complete interrupt mask
 *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
 *            @arg DMA_IT_TE:  Transfer error interrupt mask
 * @retval None
 */
#define __HAL_EXT_DMA_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->Instance->CCR &= ~(__INTERRUPT__))

/**
 * @brief  Check whether the specified DMA Channel interrupt is enabled or not.
 * @param  \__HANDLE__: DMA handle
 * @param  \__INTERRUPT__: specifies the DMA interrupt source to check.
 *          This parameter can be one of the following values:
 *            @arg DMA_IT_TC:  Transfer complete interrupt mask
 *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
 *            @arg DMA_IT_TE:  Transfer error interrupt mask
 * @retval The state of DMA_IT (SET or RESET).
 */
#define __HAL_EXT_DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  (((__HANDLE__)->Instance->CCR & (__INTERRUPT__)))

/**
 * @brief  Return the number of remaining data units in the current DMA Channel transfer.
 * @param  \__HANDLE__: DMA handle
 * @retval The number of remaining data units in the current DMA Channel transfer.
 */
#define __HAL_EXT_DMA_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNDTR)

/**
 * @}EXT_DMA_Exported_Macros
 */


/* Exported functions --------------------------------------------------------*/
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
/* Initialization and de-initialization functions *****************************/
/**
 * @brief  Initialize the DMA according to the specified
 *         parameters in the DMA_InitTypeDef and initialize the associated handle.
 * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
 *               the configuration information for the specified DMA Channel.
 * @retval HAL status
 */

HAL_StatusTypeDef HAL_EXT_DMA_Init(EXT_DMA_HandleTypeDef *hdma);

/**
 * @brief  DeInitialize the EXT_DMA peripheral.
 * @param  hdma pointer to a EXT_DMA_HandleTypeDef structure that contains
 *               the configuration information.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EXT_DMA_DeInit(EXT_DMA_HandleTypeDef *hdma);
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
/* IO operation functions *****************************************************/
/**
 * @brief  Start the DMA Transfer.
 * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
 *               the configuration information for the specified DMA Channel.
 * @param  SrcAddress The source memory Buffer address
 * @param  DstAddress The destination memory Buffer address
 * @param  Counts The counts of data transfer action from SrcAddress. Counts are uncompressed size from source if compressed is enable.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EXT_DMA_Start(EXT_DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts);
/**
 * @brief  Start the DMA Transfer with interrupt enabled.
 * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
 *               the configuration information for the specified DMA Channel.
 * @param  SrcAddress The source memory Buffer address
 * @param  DstAddress The destination memory Buffer address
 * @param  Counts The counts of data transfer action from SrcAddress. Counts are uncompressed size from source if compressed is enable.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EXT_DMA_Start_IT(EXT_DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts);
/**
  * @brief  Abort the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
    * @retval HAL status
  */
HAL_StatusTypeDef HAL_EXT_DMA_Abort(EXT_DMA_HandleTypeDef *hdma);
/**
 * @brief  Aborts the DMA Transfer in Interrupt mode.
 * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
 *                 the configuration information for the specified DMA Channel.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EXT_DMA_Abort_IT(EXT_DMA_HandleTypeDef *hdma);

/**
 * @brief  Polling for transfer complete.
 * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
 *                  the configuration information for the specified DMA Channel.
 * @param  CompleteLevel Specifies the DMA level complete.
 * @param  Timeout       Timeout duration.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EXT_DMA_PollForTransfer(EXT_DMA_HandleTypeDef *hdma, HAL_EXT_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
/**
 * @brief  Handle DMA interrupt request.
 * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
 *               the configuration information for the specified DMA Channel.
 * @retval None
 */
void HAL_EXT_DMA_IRQHandler(EXT_DMA_HandleTypeDef *hdma);

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
HAL_StatusTypeDef HAL_EXT_DMA_RegisterCallback(EXT_DMA_HandleTypeDef *hdma, HAL_EXT_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(EXT_DMA_HandleTypeDef *_hdma));

/**
 * @brief  UnRegister callbacks
 * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
 *                               the configuration information for the specified DMA Channel.
 * @param  CallbackID           User Callback identifer
 *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EXT_DMA_UnRegisterCallback(EXT_DMA_HandleTypeDef *hdma, HAL_EXT_DMA_CallbackIDTypeDef CallbackID);

/**
 * @brief  Get ext DAM compression/decompression configuration
 * @param  target_size     obsolete.
 * @param  cfg0            Compression configuration 0
 * @param  cfg1            Compression configuration 1
 * @retval HAL status
 */
#define HAL_EXT_DMA_GetConfig(target_size, cfg0, cfg1) CMPR_GetConfig(target_size, cfg0, cfg1)
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
HAL_EXT_DMA_StateTypeDef HAL_EXT_DMA_GetState(EXT_DMA_HandleTypeDef *hdma);

/**
 * @brief  Return the DMA error code.
 * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
 *              the configuration information for the specified DMA Channel.
 * @retval DMA Error Code
 */
uint32_t             HAL_EXT_DMA_GetError(EXT_DMA_HandleTypeDef *hdma);

/**
 * @brief  Return the DMA CMPRQR value.
 * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
 *              the configuration information for the specified DMA Channel.
 * @retval DMA CMPRQR register value
 */
uint32_t             HAL_EXT_DMA_GetCMPRQR(EXT_DMA_HandleTypeDef *hdma);

/**
 * @brief  Return the DMA CMPRDR value.
 * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
 *              the configuration information for the specified DMA Channel.
 * @retval DMA CMPRDR register value
 */
uint32_t             HAL_EXT_DMA_GetCMPRDR(EXT_DMA_HandleTypeDef *hdma);

#include "sifli_cmpr.h"

__STATIC_INLINE HAL_StatusTypeDef HAL_EXT_DMA_CalcCompressedSize(uint32_t counts, uint8_t cmpr_rate, uint32_t row_num,
        uint8_t pixel_size, uint32_t *compressed_size, uint32_t *target_size)
{
    if ((NULL == target_size)
            || (NULL == compressed_size))
    {
        return HAL_ERROR;
    }


    if (cmpr_rate && ((cmpr_rate - 1) < MAX_CMPR_TBL_SIZE))
    {
        if (2 == pixel_size)
        {
            switch (cmpr_rate)
            {
            case 1:
                *target_size = CMPR_1_RGB565_TGT_SIZE(counts);
                break;
            case 2:
                *target_size = CMPR_2_RGB565_TGT_SIZE(counts);
                break;
            case 3:
                *target_size = CMPR_3_RGB565_TGT_SIZE(counts);
                break;
            }
        }
        else if (3 == pixel_size)
        {
            switch (cmpr_rate)
            {
            case 1:
                *target_size = CMPR_1_RGB888_TGT_SIZE(counts);
                break;
            case 2:
                *target_size = CMPR_2_RGB888_TGT_SIZE(counts);
                break;
            case 3:
                *target_size = CMPR_3_RGB888_TGT_SIZE(counts);
                break;
            }
        }
        else if (4 == pixel_size)
        {
            switch (cmpr_rate)
            {
            case 1:
                *target_size = CMPR_1_ARGB8888_TGT_SIZE(counts);
                break;
            case 2:
                *target_size = CMPR_2_ARGB8888_TGT_SIZE(counts);
                break;
            case 3:
                *target_size = CMPR_3_ARGB8888_TGT_SIZE(counts);
                break;
            }
        }
        else
        {
            return HAL_ERROR;
        }
    }
    else
    {
        *target_size = CMPR_0_TGT_SIZE(counts);
    }

    *compressed_size = TARGET_SIZE_TO_CMPR_WORDS(*target_size) * row_num;
    *target_size     *= row_num;

    return HAL_OK;
}

/**
 * @} EXT_DMA_Exported_Functions_Group3
 */

/* Private functions ---------------------------------------------------------*/

/**
 * @} ext_dma
 */

/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_EXT_DMA_H */