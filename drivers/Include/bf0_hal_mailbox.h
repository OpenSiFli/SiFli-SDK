/**
  ******************************************************************************
  * @file   bf0_hal_mailbox.h
  * @author Sifli software development team
  * @brief   Header file of Mailbox HAL module.
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

#ifndef BF0_HAL_MAILBOX_H
#define BF0_HAL_MAILBOX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup MAILBOX MAILBOX
  * @brief MAILBOX HAL module driver
  * @{
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup MAILBOX_Exported_Constants MAILBOX Exported Constants
  * @{
  */

/** @defgroup MAILBOX_Channel MAILBOX Channel
  * @{
  */
#define MAILBOX_CHANNEL_1 0x00000000U
#define MAILBOX_CHANNEL_2 0x00000001U
#define MAILBOX_CHANNEL_3 0x00000002U
#define MAILBOX_CHANNEL_4 0x00000003U
#define MAILBOX_CHANNEL_5 0x00000004U
#define MAILBOX_CHANNEL_6 0x00000005U
#define MAILBOX_CHANNEL_7 0x00000006U
#define MAILBOX_CHANNEL_8 0x00000007U
#define MAILBOX_CHANNEL_9 0x00000008U
#define MAILBOX_CHANNEL_10 0x00000009U
#define MAILBOX_CHANNEL_11 0x0000000AU
#define MAILBOX_CHANNEL_12 0x0000000BU
#define MAILBOX_CHANNEL_13 0x0000000CU
#define MAILBOX_CHANNEL_14 0x0000000DU
#define MAILBOX_CHANNEL_15 0x0000000EU
#define MAILBOX_CHANNEL_16 0x0000000FU

/**
  * @}
  */

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup MAILBOX_Exported_Types MAILBOX Exported Types
  * @{
  */

typedef struct
{
    __IO uint32_t CxIER;
    __IO uint32_t CxITR;
    __IO uint32_t CxICR;
    __IO uint32_t CxISR;
    __IO uint32_t CxMISR;
#ifndef SF32LB55X
    __IO uint32_t CxEXR;
#endif
} MAILBOX_CH_TypeDef;

#ifndef SF32LB55X
typedef MAILBOX_CH_TypeDef MUTEX_CH_TypeDef;

#define HMUTEX_CHANNLE_NUM               (4)
#define LMUTEX_CHANNLE_NUM               (2)

#define MAILBOX_C1EXR_EX MAILBOX1_C1EXR_EX

#define MAILBOX_C2EXR_ID_Pos            MAILBOX1_C2EXR_ID_Pos
#define MAILBOX_C2EXR_ID_Msk            MAILBOX1_C2EXR_ID_Msk
#define MAILBOX_C2EXR_ID                MAILBOX1_C2EXR_ID

#define MAILBOX_C1IER_INT0_Msk          MAILBOX1_C1IER_INT0_Msk
#define MAILBOX_C1ITR_INT0_Msk          MAILBOX1_C1ITR_INT0_Msk

#else

typedef struct
{
    __IO uint32_t CxEXR;
} MUTEX_CH_TypeDef;

#define HMUTEX_CHANNLE_NUM               (2)
#define LMUTEX_CHANNLE_NUM               (2)

#endif /* SF32LB55X */

/**
  * @brief HAL MAILBOX State structures definition
  */
typedef enum
{
    HAL_MAILBOX_STATE_RESET             = 0x00U,  /*!< MAILBOX not yet initialized or disabled  */
    HAL_MAILBOX_STATE_READY             = 0x01U,  /*!< MAILBOX initialized and ready for use    */
    HAL_MAILBOX_STATE_BUSY              = 0x02U   /*!< MAILBOX internal processing is ongoing   */
} HAL_MAILBOX_StateTypeDef;


/**
 * @brief  MUTEX channel status structure definition
  */
typedef enum
{
    MUTEX_UNLOCKED          = 0,    /**< MUTEX is not locked */
    MUTEX_HCPU_LOCKED       = 1,    /**< MUTEX is locked by HCPU */
    MUTEX_LCPU_LOCKED       = 2,    /**< MUTEX is locked by LCPU */
    MUTEX_BCPU_LOCKED       = 3,    /**< MUTEX is locked by BCPU */
    MUTEX_LOCK_CORE_INVALID = 4
} MUTEX_LockCoreIdTypeDef;


/**
 * @brief  MAILBOX handle structure definition
 */
typedef struct __MAILBOX_HandleTypeDef
{
    MAILBOX_CH_TypeDef              *Instance;     /*!< MAILBOX registers base address */
    __IO HAL_MAILBOX_StateTypeDef   State;         /*!< MAILBOX State: initialized or not */
    /** notification callback
     *
     * @param[in] hmailbox mailbox handle
     * @param[in ch_idx channel index, starting from 0, refer to @ref MAILBOX_Channel
     */
    void (*NotificationCallback)(struct __MAILBOX_HandleTypeDef *hmailbox, uint8_t ch_idx);
} MAILBOX_HandleTypeDef;

/**
 * @brief  MUTEX handle structure definition
 */
typedef struct __MUTEX_HandleTypeDef
{
    MUTEX_CH_TypeDef                *Instance;     /*!< MAILBOX registers base address */
} MUTEX_HandleTypeDef;


/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup MAILBOX_Exported_Macros MAILBOX Exported Macros
  * @{
  */


/**
  * @brief  Mask the specified interrupt.
  * @param  \__HANDLE__ specifies the MAILBOX Handle
  * @param  \__CHINDEX__ specifies the channels number:
  *         This parameter can be one of the following values (@ref MAILBOX_Channel):
  *            @arg MAILBOX_CHANNEL_1: MAILBOX Channel 1
  *            @arg MAILBOX_CHANNEL_2: MAILBOX Channel 2
  *            @arg MAILBOX_CHANNEL_3: MAILBOX Channel 3
  *            @arg MAILBOX_CHANNEL_4: MAILBOX Channel 4
  *            @arg MAILBOX_CHANNEL_5: MAILBOX Channel 5
  *            @arg MAILBOX_CHANNEL_6: MAILBOX Channel 6
  */
#define __HAL_MAILBOX_MASK_CHANNEL_IT(__HANDLE__, __CHINDEX__) \
                ((__HANDLE__)->Instance->CxIER &= ~(MAILBOX_C1IER_INT0_Msk << (__CHINDEX__)))

/**
  * @brief  Unmask the specified interrupt.
  * @param  \__HANDLE__ specifies the MAILBOX Handle
  * @param  \__CHINDEX__ specifies the channels number:
  *         This parameter can be one of the following values (@ref MAILBOX_Channel):
  *            @arg MAILBOX_CHANNEL_1: MAILBOX Channel 1
  *            @arg MAILBOX_CHANNEL_2: MAILBOX Channel 2
  *            @arg MAILBOX_CHANNEL_3: MAILBOX Channel 3
  *            @arg MAILBOX_CHANNEL_4: MAILBOX Channel 4
  *            @arg MAILBOX_CHANNEL_5: MAILBOX Channel 5
  *            @arg MAILBOX_CHANNEL_6: MAILBOX Channel 6
  */
#define __HAL_MAILBOX_UNMASK_CHANNEL_IT(__HANDLE__, __CHINDEX__) \
            ((__HANDLE__)->Instance->CxIER |= (MAILBOX_C1IER_INT0_Msk << (__CHINDEX__)))


/**
  * @brief  Trigger the specified interrupt.
  * @param  \__HANDLE__ specifies the MAILBOX Handle
  * @param  \__CHINDEX__ specifies the channels number :
  *         This parameter can be one of the following values (@ref MAILBOX_Channel):
  *            @arg MAILBOX_CHANNEL_1: MAILBOX Channel 1
  *            @arg MAILBOX_CHANNEL_2: MAILBOX Channel 2
  *            @arg MAILBOX_CHANNEL_3: MAILBOX Channel 3
  *            @arg MAILBOX_CHANNEL_4: MAILBOX Channel 4
  *            @arg MAILBOX_CHANNEL_5: MAILBOX Channel 5
  *            @arg MAILBOX_CHANNEL_6: MAILBOX Channel 6
  */
#define __HAL_MAILBOX_TRIGGER_CHANNEL_IT(__HANDLE__, __CHINDEX__) \
            ((__HANDLE__)->Instance->CxITR |= (MAILBOX_C1ITR_INT0_Msk << (__CHINDEX__)))


/**
  * @brief  Check whether interrupt has been handled
  * @param  \__HANDLE__ specifies the MAILBOX Handle
  * @param  \__CHINDEX__ specifies the channels number :
  *         This parameter can be one of the following values (@ref MAILBOX_Channel):
  *            @arg MAILBOX_CHANNEL_1: MAILBOX Channel 1
  *            @arg MAILBOX_CHANNEL_2: MAILBOX Channel 2
  *            @arg MAILBOX_CHANNEL_3: MAILBOX Channel 3
  *            @arg MAILBOX_CHANNEL_4: MAILBOX Channel 4
  *            @arg MAILBOX_CHANNEL_5: MAILBOX Channel 5
  *            @arg MAILBOX_CHANNEL_6: MAILBOX Channel 6
  * @retval 0 interrupt has been handled
  * @retval others interrupt has not been handled
  */
#define __HAL_MAILBOX_CHECK_CHANNEL_IT(__HANDLE__, __CHINDEX__) \
            ((__HANDLE__)->Instance->CxISR & (MAILBOX_C1ITR_INT0_Msk << (__CHINDEX__)))

/**
  * @brief  Clear the specified interrupt.
  * @param  \__HANDLE__ specifies the MAILBOX Handle
  * @param  \__CHINDEX__ specifies the channels number :
  *         This parameter can be one of the following values (@ref MAILBOX_Channel):
  *            @arg MAILBOX_CHANNEL_1: MAILBOX Channel 1
  *            @arg MAILBOX_CHANNEL_2: MAILBOX Channel 2
  *            @arg MAILBOX_CHANNEL_3: MAILBOX Channel 3
  *            @arg MAILBOX_CHANNEL_4: MAILBOX Channel 4
  *            @arg MAILBOX_CHANNEL_5: MAILBOX Channel 5
  *            @arg MAILBOX_CHANNEL_6: MAILBOX Channel 6
  */
#define __HAL_MAILBOX_CLEAR_CHANNEL_IT(__HANDLE__, __CHINDEX__) \
            ((__HANDLE__)->Instance->CxICR |= (MAILBOX_C1ICR_INT0_Msk << (__CHINDEX__)))


/**
  * @brief  get the masked ISR status
  * @param  \__HANDLE__ specifies the MAILBOX Handle
  *
  * @retval masked ISR status, uint32_t type
  */
#define __HAL_MAILBOX_GET_STATUS(__HANDLE__) ((__HANDLE__)->Instance->CxMISR)

/**
  * @brief  clear the ISR status
  * @param  \__HANDLE__ specifies the MAILBOX Handle
  * @param  \__STATUS__ status value to be cleared
  *
  */
#define __HAL_MAILBOX_CLEAR_STATUS(__HANDLE__, __STATUS__) ((__HANDLE__)->Instance->CxICR = (__STATUS__))


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup MAILBOX_Exported_Functions MAILBOX Exported Functions
  * @{
  */

/* Initialization and de-initialization functions *******************************/
/** @defgroup MAILBOX_Exported_Functions_Group1 Initialization and deinitialization functions
 *  @{
 */
/**
  * @brief  Initialize the MAILBOX peripheral.
  * @param  hmailbox MAILBOX handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_MAILBOX_Init(MAILBOX_HandleTypeDef *hmailbox);

/**
  * @brief  DeInitialize the MAILBOX peripheral.
  * @param  hmailbox MAILBOX handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_MAILBOX_DeInit(MAILBOX_HandleTypeDef *hmailbox);

/**
  * @brief Initialize the MAILBOX MSP.
  * @param  hmailbox MAILBOX handle
  * @retval None
  */
void HAL_MAILBOX_MspInit(MAILBOX_HandleTypeDef *hmailbox);

/**
  * @brief MAILBOX MSP DeInit
  * @param  hmailbox MAILBOX handle
  * @retval None
  */

void HAL_MAILBOX_MspDeInit(MAILBOX_HandleTypeDef *hmailbox);
/**
  * @}
  */

/** @defgroup MAILBOX_Exported_Functions_Group3 Peripheral State and Error functions
 *  @{
 */
/* Peripheral State and Error functions ****************************************/
/**
  * @brief Return the MAILBOX handle state.
  * @param  hmailbox MAILBOX handle
  * @retval MAILBOX handle state
  */
HAL_MAILBOX_StateTypeDef HAL_MAILBOX_GetState(MAILBOX_HandleTypeDef const *const hmailbox);


/**
  * @}
  */

/** @defgroup MAILBOX_Exported_Functions_Group4 Mutex lock and unlock functions
 *  @{
 */


/**
 * @brief  Lock mutex
 *
 * @param  hmutex mutex handle
 * @param  ch_id channel id, range, 0~1, it's used if mutex instance is not specified
 *
 * @retval 0: lock succeed, >0: lock fail, core
 */
MUTEX_LockCoreIdTypeDef HAL_MAILBOX_Lock(MUTEX_HandleTypeDef const *const hmutex, uint8_t ch_id);

/**
 * @brief  Lock mutex advanced version
 *
 * @param  hmutex mutex handle
 * @param  ch_id channel id, range, 0~1, it's used if mutex instance is not specified
 * @param  timeout_ms time out value in ms.
 * @retval 0: lock succeed, >0: lock fail, core
 */
MUTEX_LockCoreIdTypeDef HAL_MAILBOX_LockEx(MUTEX_HandleTypeDef const *const hmutex, uint8_t ch_id, uint32_t timeout_ms);

/**
 * @brief  Get mutex based on CPU core and channel
 *
 * @param  core_id core ID, CORE_ID_HCPU/CORE_ID_LCPU/CORE_ID_BCPU
 * @param  ch_id channel id, range, 0~1, it's used if mutex instance is not specified
 * @retval Mutex handle if successful, NULL if failed.
 */
MUTEX_CH_TypeDef *HAL_MAILBOX_GetMutex(uint8_t core_id, uint8_t ch_id);

/**
 * @brief  Unlock mutex
 *
 * @param  hmutex mutex handle
 * @param  ch_id channel id, range, 0~1, it's used if mutex instance is not specified
 *
 * @retval void
 */
void HAL_MAILBOX_UnLock(MUTEX_HandleTypeDef const *const hmutex, uint8_t ch_id);

/**
  * @}
  */

/** @brief mailbox irq handler
 *
 * @param[in] hmailbox mailbox handle
 *
 */
void HAL_MAILBOX_IRQHandler(MAILBOX_HandleTypeDef *hmailbox);

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

#endif /* BF0_HAL_MAILBOX_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
