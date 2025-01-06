/**
  ******************************************************************************
  * @file   bf0_hal_mailbox.c
  * @author Sifli software development team
  * @brief   MAILBOX HAL module driver.
  *          This file provides firmware functions to manage the following
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

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup MAILBOX
  * @{
  */

#if defined(HAL_MAILBOX_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup MAILBOX_Private_Constants MAILBOX Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup MAILBOX_Private_Functions MAILBOX Private Functions
  * @{
  */

/**
  * @}
  */

/** @addtogroup MAILBOX_Exported_Functions
  * @{
  */

/** @addtogroup MAILBOX_Exported_Functions_Group1
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
             ##### Initialization and de-initialization functions  #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the MAILBOX peripheral:

      (+) User must Implement HAL_MAILBOX_MspInit() function in which he configures
          all related peripherals resources (CLOCK and NVIC ).

      (+) Call the function HAL_MAILBOX_Init() to configure the MAILBOX register.

      (+) Call the function HAL_PKA_DeInit() to restore the default configuration
          of the selected MAILBOX peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the MAILBOX peripheral.
  * @param  hmailbox MAILBOX handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_MAILBOX_Init(MAILBOX_HandleTypeDef *hmailbox)
{
    HAL_StatusTypeDef err = HAL_OK;


    return err;
}

/**
  * @brief  DeInitialize the MAILBOX peripheral.
  * @param  hmailbox MAILBOX handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_MAILBOX_DeInit(MAILBOX_HandleTypeDef *hmailbox)
{
    HAL_StatusTypeDef err = HAL_OK;


    return err;
}

/**
  * @brief Initialize the MAILBOX MSP.
  * @param  hmailbox MAILBOX handle
  * @retval None
  */
__weak void HAL_MAILBOX_MspInit(MAILBOX_HandleTypeDef *hmailbox)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hmailbox);

    /* NOTE : This function should not be modified. When the callback is needed
              the HAL_MAILBOX_MspInit should be implemented in the user file
     */
}

/**
  * @brief MAILBOX MSP DeInit
  * @param  hmailbox MAILBOX handle
  * @retval None
  */
__weak void HAL_MAILBOX_MspDeInit(MAILBOX_HandleTypeDef *hmailbox)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hmailbox);

    /* NOTE : This function should not be modified. When the callback is needed
              the HAL_MAILBOX_MspDeInit should be implemented in the user file
     */
}

/**
  * @}
  */


/** @addtogroup MAILBOX_Exported_Functions_Group3
 *  @brief   MAILBOX Peripheral State and Error functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral State and Error functions #####
  ==============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief Return the MAILBOX handle state.
  * @param  hmailbox MAILBOX handle
  * @retval MAILBOX handle state
  */
__HAL_ROM_USED HAL_MAILBOX_StateTypeDef HAL_MAILBOX_GetState(MAILBOX_HandleTypeDef const *const hmailbox)
{
    return hmailbox->State;
}


__HAL_ROM_USED MUTEX_CH_TypeDef *HAL_MAILBOX_GetMutex(uint8_t core_id, uint8_t ch_id)
{
    MUTEX_CH_TypeDef *mutex = NULL;
    uint8_t max_chan_num;

    max_chan_num = (core_id == CORE_ID_HCPU) ? HMUTEX_CHANNLE_NUM : LMUTEX_CHANNLE_NUM;

    if (ch_id >= max_chan_num)
    {
        return mutex;
    }

    if (core_id == CORE_ID_HCPU)
    {
        mutex = HMUTEX_CH1;
    }
    else if (core_id == CORE_ID_LCPU)
    {
        mutex = LMUTEX_CH1;
    }
    else
    {
        HAL_ASSERT(0);
    }

    if (mutex)
    {
        mutex = mutex + ch_id;
    }

    return mutex;
}

__HAL_ROM_USED MUTEX_LockCoreIdTypeDef HAL_MAILBOX_Lock(MUTEX_HandleTypeDef const *const hmutex, uint8_t ch_id)
{
    MUTEX_LockCoreIdTypeDef core = MUTEX_LOCK_CORE_INVALID;
    MUTEX_CH_TypeDef *mutex = NULL;

    if (hmutex)
        mutex = hmutex->Instance;
    else
    {
        mutex = HAL_MAILBOX_GetMutex(CORE_ID_CURRENT, ch_id);
    }
    if (mutex)
    {
        uint32_t exr = mutex->CxEXR;
        if (exr & MAILBOX_C1EXR_EX)
        {
            core = MUTEX_UNLOCKED;
        }
        else
        {
            core = GET_REG_VAL(exr, MAILBOX_C2EXR_ID_Msk, MAILBOX_C2EXR_ID_Pos);
        }
    }
    return core;
}

__HAL_ROM_USED MUTEX_LockCoreIdTypeDef HAL_MAILBOX_LockEx(MUTEX_HandleTypeDef const *const hmutex, uint8_t ch_id, uint32_t timeout_ms)
{
    MUTEX_LockCoreIdTypeDef core = MUTEX_LOCK_CORE_INVALID;
    while (1)
    {
        core = HAL_MAILBOX_Lock(hmutex, ch_id);
        if (core == MUTEX_UNLOCKED)
        {
            break;
        }
        if (timeout_ms == 0)
        {
            break;
        }
        timeout_ms--;
        HAL_Delay(1);
    }
    return core;
}

__HAL_ROM_USED void HAL_MAILBOX_UnLock(MUTEX_HandleTypeDef const *const hmutex, uint8_t ch_id)
{
    MUTEX_CH_TypeDef *mutex = NULL;

    if (hmutex)
        mutex = hmutex->Instance;
    else
        mutex = HAL_MAILBOX_GetMutex(CORE_ID_CURRENT, ch_id);
    if (mutex)
        mutex->CxEXR |= MAILBOX_C1EXR_EX;
}

/**
  * @}
  */



/** @brief mailbox irq handler
 *
 * @param[in] hmailbox mailbox handle
 *
 */
__HAL_ROM_USED void HAL_MAILBOX_IRQHandler(MAILBOX_HandleTypeDef *hmailbox)
{
    uint32_t status;
    uint32_t clear_status;
    uint8_t q_idx;
    MAILBOX_CH_TypeDef *mailbox;

    mailbox = hmailbox->Instance;

    status = mailbox->CxMISR;
    clear_status = status;
    mailbox->CxICR = clear_status;
    q_idx = 0;
    while (status)
    {
        if (1 & status)
        {
            if (hmailbox->NotificationCallback)
            {
                hmailbox->NotificationCallback(hmailbox, q_idx);
            }
        }
        status >>= 1;
        q_idx++;
    }
}

/**
  * @}
  */

/** @addtogroup MAILBOX_Private_Functions
  * @{
  */



/**
  * @}
  */

#endif /* HAL_MAILBOX_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
