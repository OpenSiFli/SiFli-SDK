/**
  ******************************************************************************
  * @file   bf0_hal_rng.c
  * @author Sifli software development team
  * @brief   RNG HAL module driver.
  *          This file provides firmware functions to manage the following
  ******************************************************************************
*/
/**
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

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup RNG RNG
  * @brief RNG HAL module driver.
  * @{
  */

#ifdef HAL_RNG_MODULE_ENABLED



/* Private types -------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup RNG_Private_Constants RNG_Private_Constants
  * @{
  */
#define RNG_TIMEOUT_VALUE     2
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup RNG_Exported_Functions
  * @{
  */

/** @addtogroup RNG_Exported_Functions_Group1
 *  @brief   Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
          ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the RNG according to the specified parameters
          in the RNG_InitTypeDef and create the associated handle
      (+) DeInitialize the RNG peripheral
      (+) Initialize the RNG MSP (MCU Specific Package)
      (+) DeInitialize the RNG MSP

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the RNG peripheral and initialize the associated handle.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_Init(RNG_HandleTypeDef *hrng)
{
    /* Check the RNG handle allocation */
    if (hrng == NULL)
    {
        return HAL_ERROR;
    }

    HAL_ASSERT(IS_RNG_ALL_INSTANCE(hrng->Instance));

#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
    if (hrng->State == HAL_RNG_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hrng->Lock = HAL_UNLOCKED;

        hrng->RandReadyCallback  = HAL_RNG_ReadyDataCallback;  /* Legacy weak ReadyDataCallback  */
        hrng->SeedReadyCallback  = HAL_RNG_ErrorCallback;      /* Legacy weak ErrorCallback      */

        if (hrng->MspInitCallback == NULL)
        {
            hrng->MspInitCallback = HAL_RNG_MspInit; /* Legacy weak MspInit  */
        }

        /* Init the low level hardware */
        hrng->MspInitCallback(hrng);
    }
#else
    if (hrng->State == HAL_RNG_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hrng->Lock = HAL_UNLOCKED;

        /* Init the low level hardware */
        HAL_RNG_MspInit(hrng);
    }
#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */

    /* Change RNG peripheral state */
    hrng->State = HAL_RNG_STATE_BUSY;

    /* Enable the RNG Peripheral */
    __HAL_RNG_ENABLE(hrng);

    /* Initialize the RNG state */
    hrng->State = HAL_RNG_STATE_READY;

    /* Initialise the error code */
    hrng->ErrorCode = HAL_RNG_ERROR_NONE;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  DeInitialize the RNG peripheral.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_DeInit(RNG_HandleTypeDef *hrng)
{
    /* Check the RNG handle allocation */
    if (hrng == NULL)
    {
        return HAL_ERROR;
    }

    __HAL_RNG_DISABLE(hrng);

#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
    if (hrng->MspDeInitCallback == NULL)
    {
        hrng->MspDeInitCallback = HAL_RNG_MspDeInit; /* Legacy weak MspDeInit  */
    }

    /* DeInit the low level hardware */
    hrng->MspDeInitCallback(hrng);
#else
    /* DeInit the low level hardware */
    HAL_RNG_MspDeInit(hrng);
#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */

    /* Update the RNG state */
    hrng->State = HAL_RNG_STATE_RESET;

    /* Initialise the error code */
    hrng->ErrorCode = HAL_RNG_ERROR_NONE;

    /* Release Lock */
    __HAL_UNLOCK(hrng);

    /* Return the function status */
    return HAL_OK;
}

/**
  * @brief  Initialize the RNG MSP.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval None
  */
__weak void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrng);

    /* NOTE : This function should not be modified. When the callback is needed,
              function HAL_RNG_MspInit must be implemented in the user file.
     */
}

/**
  * @brief  DeInitialize the RNG MSP.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval None
  */
__weak void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrng);

    /* NOTE : This function should not be modified. When the callback is needed,
              function HAL_RNG_MspDeInit must be implemented in the user file.
     */
}

#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User RNG Callback
  *         To be used instead of the weak predefined callback
  * @param  hrng RNG handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_RNG_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_RNG_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_RNG_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_RegisterCallback(RNG_HandleTypeDef *hrng, HAL_RNG_CallbackIDTypeDef CallbackID, pRNG_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
        return HAL_ERROR;
    }
    /* Process locked */
    __HAL_LOCK(hrng);

    if (HAL_RNG_STATE_READY == hrng->State)
    {
        switch (CallbackID)
        {
        case HAL_RNG_ERROR_CB_ID :
            hrng->ErrorCallback = pCallback;
            break;

        case HAL_RNG_MSPINIT_CB_ID :
            hrng->MspInitCallback = pCallback;
            break;

        case HAL_RNG_MSPDEINIT_CB_ID :
            hrng->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_RNG_STATE_RESET == hrng->State)
    {
        switch (CallbackID)
        {
        case HAL_RNG_MSPINIT_CB_ID :
            hrng->MspInitCallback = pCallback;
            break;

        case HAL_RNG_MSPDEINIT_CB_ID :
            hrng->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hrng);
    return status;
}

/**
  * @brief  Unregister an RNG Callback
  *         RNG callabck is redirected to the weak predefined callback
  * @param  hrng RNG handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_RNG_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_RNG_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_RNG_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_UnRegisterCallback(RNG_HandleTypeDef *hrng, HAL_RNG_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hrng);

    if (HAL_RNG_STATE_READY == hrng->State)
    {
        switch (CallbackID)
        {
        case HAL_RNG_ERROR_CB_ID :
            hrng->ErrorCallback = HAL_RNG_ErrorCallback;          /* Legacy weak ErrorCallback  */
            break;

        case HAL_RNG_MSPINIT_CB_ID :
            hrng->MspInitCallback = HAL_RNG_MspInit;              /* Legacy weak MspInit  */
            break;

        case HAL_RNG_MSPDEINIT_CB_ID :
            hrng->MspDeInitCallback = HAL_RNG_MspDeInit;          /* Legacy weak MspDeInit  */
            break;

        default :
            /* Update the error code */
            hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_RNG_STATE_RESET == hrng->State)
    {
        switch (CallbackID)
        {
        case HAL_RNG_MSPINIT_CB_ID :
            hrng->MspInitCallback = HAL_RNG_MspInit;              /* Legacy weak MspInit  */
            break;

        case HAL_RNG_MSPDEINIT_CB_ID :
            hrng->MspDeInitCallback = HAL_RNG_MspDeInit;          /* Legacy weak MspInit  */
            break;

        default :
            /* Update the error code */
            hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hrng);
    return status;
}

/**
  * @brief  Register Data Ready RNG Callback
  *         To be used instead of the weak HAL_RNG_ReadyDataCallback() predefined callback
  * @param  hrng RNG handle
  * @param  pCallback pointer to the Data Ready Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_RegisterReadyDataCallback(RNG_HandleTypeDef *hrng, pRNG_ReadyDataCallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
        return HAL_ERROR;
    }
    /* Process locked */
    __HAL_LOCK(hrng);

    if (HAL_RNG_STATE_READY == hrng->State)
    {
        hrng->ReadyDataCallback = pCallback;
    }
    else
    {
        /* Update the error code */
        hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hrng);
    return status;
}

/**
  * @brief  UnRegister the Data Ready RNG Callback
  *         Data Ready RNG Callback is redirected to the weak HAL_RNG_ReadyDataCallback() predefined callback
  * @param  hrng RNG handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_UnRegisterReadyDataCallback(RNG_HandleTypeDef *hrng)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hrng);

    if (HAL_RNG_STATE_READY == hrng->State)
    {
        hrng->ReadyDataCallback = HAL_RNG_ReadyDataCallback; /* Legacy weak ReadyDataCallback  */
    }
    else
    {
        /* Update the error code */
        hrng->ErrorCode |= HAL_RNG_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hrng);
    return status;
}

#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup RNG_Exported_Functions_Group2
 *  @brief    Management functions.
 *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Get the 32 bit Random number
      (+) Get the 32 bit Random number with interrupt enabled
      (+) Handle RNG interrupt request

@endverbatim
  * @{
  */

/**
  * @brief  Generate a 32-bit random number or seed.
  * @note   Each time the random number data is read the RNG_FLAG_DRDY flag
  *         is automatically cleared.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @param  random32bit: pointer to generated random number variable if successful.
  * @param  is_seed: 1:generate seed. 0:Generate random number
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_RNG_Generate(RNG_HandleTypeDef *hrng, uint32_t *value32bit, uint8_t is_seed)
{
    uint32_t tickstart = 0;
    HAL_StatusTypeDef status = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hrng);

    /* Check RNS peripheral state */
    if (hrng->State == HAL_RNG_STATE_READY)
    {
        __HAL_RNG_COMMAND(hrng, is_seed ? RNG_CMD_SEED : RNG_CMD_RAND);

        /* Change RNG peripheral state */
        hrng->State = HAL_RNG_STATE_BUSY;

        /* Get tick */
        tickstart = HAL_GetTick();

        /* Check if data register contains valid random data */
        while (__HAL_RNG_GET_FLAG(hrng, is_seed ? RNG_FLAG_SEED_VALID : RNG_FLAG_RAND_VALID) == RESET)
        {
            if ((HAL_GetTick() - tickstart) > RNG_TIMEOUT_VALUE)
            {
                hrng->State = HAL_RNG_STATE_ERROR;

                /* Process Unlocked */
                __HAL_UNLOCK(hrng);

                return HAL_TIMEOUT;
            }
        }

        if (is_seed == 0)
        {
            *value32bit = hrng->Instance->RAND_NUM0;
            hrng->RandomNumber = *value32bit;
        }
        else
            *value32bit = hrng->Instance->RAND_SEED0;
        hrng->State = HAL_RNG_STATE_READY;
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hrng);

    return status;
}

/**
  * @brief  Generate random number or seed in interrupt mode.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @param  is_seed: 1:generate seed. 0:Generate random number
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_Generate_IT(RNG_HandleTypeDef *hrng, uint8_t is_seed)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hrng);

    /* Check RNG peripheral state */
    if (hrng->State == HAL_RNG_STATE_READY)
    {
        /* Change RNG peripheral state */
        hrng->State = HAL_RNG_STATE_BUSY;

        /* Process Unlocked */
        __HAL_UNLOCK(hrng);

        /* Enable the RNG Interrupts: Data Ready, Clock error, Seed error */
        __HAL_RNG_ENABLE_IT(hrng);

        /* Start to generate random number or seed*/
        __HAL_RNG_COMMAND(hrng, is_seed ? RNG_CMD_SEED : RNG_CMD_RAND);

    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hrng);

        status = HAL_ERROR;
    }

    return status;
}

/**
  * @brief  Handle RNG interrupt request.
  * @note   In the case of a clock error, the RNG is no more able to generate
  *         random numbers because the PLL48CLK clock is not correct. User has
  *         to check that the clock controller is correctly configured to provide
  *         the RNG clock and clear the CEIS bit using __HAL_RNG_CLEAR_IT().
  *         The clock error has no impact on the previously generated
  *         random numbers, and the RNG_DR register contents can be used.
  * @note   In the case of a seed error, the generation of random numbers is
  *         interrupted as long as the SECS bit is '1'. If a number is
  *         available in the RNG_DR register, it must not be used because it may
  *         not have enough entropy. In this case, it is recommended to clear the
  *         SEIS bit using __HAL_RNG_CLEAR_IT(), then disable and enable
  *         the RNG peripheral to reinitialize and restart the RNG.
  * @note   RNG ErrorCallback() API is called once whether SEIS or CEIS are set.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval None

  */
void HAL_RNG_IRQHandler(RNG_HandleTypeDef *hrng)
{

    /* Generate random number once, so disable the IT */
    __HAL_RNG_DISABLE_IT(hrng);
    /* Check RNG data ready interrupt occurred */
    if (__HAL_RNG_GET_IT(hrng, RNG_IT_RAND) != RESET)
    {
        /* Get the 32bit Random number (DRDY flag automatically cleared) */
        hrng->RandomNumber = hrng->Instance->RAND_NUM0;

        if (hrng->State != HAL_RNG_STATE_ERROR)
        {
            /* Change RNG peripheral state */
            hrng->State = HAL_RNG_STATE_READY;

#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
            /* Call registered Data Ready callback */
            hrng->RandReadyCallback(hrng, hrng->RandomNumber);
#else
            /* Call legacy weak Data Ready callback */
            HAL_RNG_RandReadyCallback(hrng, hrng->RandomNumber);
#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */
        }
    }
    /* Check RNG Seed ready interrupt occurred */
    if (__HAL_RNG_GET_IT(hrng, RNG_IT_SEED) != RESET)
    {
        /* Get the 32bit seed */
        uint32_t seed = hrng->Instance->RAND_SEED0;

        if (hrng->State != HAL_RNG_STATE_ERROR)
        {
            /* Change RNG peripheral state */
            hrng->State = HAL_RNG_STATE_READY;

#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
            /* Call registered Seed Ready callback */
            hrng->SeedReadyCallback(hrng, seed);
#else
            /* Call legacy weak Seed Ready callback */
            HAL_RNG_SeedReadyCallback(hrng, seed);
#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */
        }
    }
}

/**
  * @brief  Read latest generated random number.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval random value
  */
uint32_t HAL_RNG_ReadLastRandomNumber(RNG_HandleTypeDef *hrng)
{
    return (hrng->RandomNumber);
}

/**
  * @brief  Random Data Ready callback in non-blocking mode.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @param  random32bit: generated random value
  * @retval None
  */
__weak void HAL_RNG_RandReadyCallback(RNG_HandleTypeDef *hrng, uint32_t random32bit)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrng);
    UNUSED(random32bit);

    /* NOTE : This function should not be modified. When the callback is needed,
              function HAL_RNG_ReadyDataCallback must be implemented in the user file.
     */
}

/**
  * @brief  Random Seed Ready callback in non-blocking mode.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @param  seed32bit: generated random seed
  * @retval None
  */
__weak void HAL_RNG_SeedReadyCallback(RNG_HandleTypeDef *hrng, uint32_t seed32bit)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrng);
    UNUSED(seed32bit);

    /* NOTE : This function should not be modified. When the callback is needed,
              function HAL_RNG_ErrorCallback must be implemented in the user file.
     */
}

/**
  * @}
  */

/** @addtogroup RNG_Exported_Functions_Group3
 *  @brief    Peripheral State functions.
 *
@verbatim
 ===============================================================================
                ##### Peripheral State and Error functions #####
 ===============================================================================
    [..]
    This subsection permits to :
      (+) Return in run-time the status of the peripheral.
      (+) Return the RNG handle error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the RNG handle state.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval HAL state
  */
HAL_RNG_StateTypeDef HAL_RNG_GetState(RNG_HandleTypeDef *hrng)
{
    /* Return RNG handle state */
    return hrng->State;
}

/**
  * @brief  Return the RNG handle error code.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval RNG Error Code
*/
uint32_t HAL_RNG_GetError(RNG_HandleTypeDef *hrng)
{
    /* Return RNG Error Code */
    return hrng->ErrorCode;
}
/**
  * @}
  */

/**
  * @}
  */


#endif /* HAL_RNG_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

