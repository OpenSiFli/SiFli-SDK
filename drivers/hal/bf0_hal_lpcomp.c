/**
  ******************************************************************************
  * @file   bf0_hal_lpcomp.c
  * @author Sifli software development team
  * @brief   lcdc HAL module driver.
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

#include <string.h>
#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

#ifdef HAL_COMP_MODULE_ENABLED


/** @defgroup COMP COMP
  * @brief COMP HAL module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup COMP_Private_Constants
  * @{
  */

/* Delay for COMP startup time.                                               */
/* Note: Delay required to reach propagation delay specification.             */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tSTART").                                                       */
/* Unit: us                                                                   */
#define COMP_DELAY_STARTUP_US          (200UL) /*!< Delay for COMP startup time */

/* Delay for COMP voltage scaler stabilization time.                          */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tSTART_SCALER").                                                */
/* Unit: us                                                                   */
#define COMP_DELAY_VOLTAGE_SCALER_STAB_US (200UL)  /*!< Delay for COMP voltage scaler stabilization time */

#define COMP_OUTPUT_LEVEL_BITOFFSET_POS    (LPCOMP_CR1_VAL_Pos)

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup COMP_Exported_Functions COMP Exported Functions
  * @{
  */

/** @defgroup COMP_Exported_Functions_Group1 Initialization/de-initialization functions
 *  @brief    Initialization and de-initialization functions.
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions to initialize and de-initialize comparators

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the COMP according to the specified
  *         parameters in the COMP_InitTypeDef and initialize the associated handle.
  * @note   If the selected comparator is locked, initialization can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  hcomp  COMP handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_Init(COMP_HandleTypeDef *hcomp)
{
    uint32_t tmp_csr;
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the COMP handle allocation and lock status */
    if (hcomp == NULL)
    {
        status = HAL_ERROR;
    }
    else
    {
        if (hcomp->State == HAL_COMP_STATE_RESET)
        {
            /* Allocate lock resource and initialize it */
            hcomp->Lock = HAL_UNLOCKED;

            /* Set COMP error code to none */
            COMP_CLEAR_ERRORCODE(hcomp);

            /* Init SYSCFG and the low level hardware to access comparators */
            /* Note: HAL_COMP_Init() calls __HAL_RCC_SYSCFG_CLK_ENABLE()            */
            /*       to enable internal control clock of the comparators.           */
            /*       However, this is a legacy strategy. In future STM32 families,  */
            /*       COMP clock enable must be implemented by user                  */
            /*       in "HAL_COMP_MspInit()".                                       */
            /*       Therefore, for compatibility anticipation, it is recommended   */
            /*       to implement __HAL_RCC_SYSCFG_CLK_ENABLE()                     */
            /*       in "HAL_COMP_MspInit()".                                       */
            //__HAL_RCC_SYSCFG_CLK_ENABLE();
            hwp_tsen->BGR |= TSEN_BGR_EN; //enable anau bandgap

#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
            /* Init the COMP Callback settings */
            hcomp->TriggerCallback = HAL_COMP_TriggerCallback; /* Legacy weak callback */

            if (hcomp->MspInitCallback == NULL)
            {
                hcomp->MspInitCallback = HAL_COMP_MspInit; /* Legacy weak MspInit  */
            }

            /* Init the low level hardware */
            hcomp->MspInitCallback(hcomp);
#else
            /* Init the low level hardware */
            HAL_COMP_MspInit(hcomp);
#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */
        }

        /* Set COMP parameters */
        tmp_csr = (hcomp->Init.NonInvertingInput
                   | hcomp->Init.InvertingInput
                   | hcomp->Init.InternalVRef
                   | hcomp->Init.Hysteresis
                   | hcomp->Init.Mode
                   | hcomp->Init.TriggerMode
                  );

        /* Set parameters in COMP register */
        /* Note: Update all bits except read-only, lock and enable bits */

        MODIFY_REG(hcomp->Instance->CR1,
                   LPCOMP_CR1_POL  | LPCOMP_CR1_MODE   | LPCOMP_CR1_VREFINT  |
                   LPCOMP_CR1_INMSEL  | LPCOMP_CR1_INPSEL | LPCOMP_CR1_HYST    |
                   LPCOMP_CR1_SENS | LPCOMP_CR1_IE,
                   tmp_csr
                  );

        /* Set HAL COMP handle state */
        /* Note: Transition from state reset to state ready,                      */
        /*       otherwise (coming from state ready or busy) no state update.     */
        if (hcomp->State == HAL_COMP_STATE_RESET)
        {
            hcomp->State = HAL_COMP_STATE_READY;
        }
    }

    return status;
}

/**
  * @brief  DeInitialize the COMP peripheral.
  * @note   Deinitialization cannot be performed if the COMP configuration is locked.
  *         To unlock the configuration, perform a system reset.
  * @param  hcomp  COMP handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_DeInit(COMP_HandleTypeDef *hcomp)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the COMP handle allocation and lock status */
    if (hcomp == NULL)
    {
        status = HAL_ERROR;
    }
    else
    {
        /* Check the parameter */

        /* Set COMP_CSR register to reset value */
        //WRITE_REG(hcomp->Instance->CR1, 0x00000000UL);

#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
        if (hcomp->MspDeInitCallback == NULL)
        {
            hcomp->MspDeInitCallback = HAL_COMP_MspDeInit; /* Legacy weak MspDeInit  */
        }

        /* DeInit the low level hardware: GPIO, RCC clock, NVIC */
        hcomp->MspDeInitCallback(hcomp);
#else
        /* DeInit the low level hardware: GPIO, RCC clock, NVIC */
        HAL_COMP_MspDeInit(hcomp);
#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */

        hwp_tsen->BGR &= ~TSEN_BGR_EN;

        /* Set HAL COMP handle state */
        hcomp->State = HAL_COMP_STATE_RESET;

        /* Release Lock */
        __HAL_UNLOCK(hcomp);
    }

    return status;
}

/**
  * @brief  Initialize the COMP MSP.
  * @param  hcomp  COMP handle
  * @retval None
  */
__weak void HAL_COMP_MspInit(COMP_HandleTypeDef *hcomp)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcomp);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_COMP_MspInit could be implemented in the user file
     */
    // set lpcomp out pin
    //HAL_PIN_Set(PAD_PB24, LPCOMP1_OUT, PIN_NOPULL, 0);
    //HAL_PIN_Set(PAD_PB25, LPCOMP2_OUT, PIN_NOPULL, 0);

    // set lpcomp input pin, they are analog pin, no function in list, set to GPIO and set mode to input
    HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_NOPULL, 0);  //#LPCOMP1_P
    HAL_PIN_Set(PAD_PB03, GPIO_B3, PIN_NOPULL, 0);  //#LPCOMP1_N
    HAL_PIN_Select(PAD_PB01, 10, 0);
    HAL_PIN_Select(PAD_PB03, 10, 0);
    //HAL_PIN_Set(PAD_PB04, GPIO_B4, PIN_NOPULL, 0);  //#LPCOMP2_P
    //HAL_PIN_Set(PAD_PB05, GPIO_B5, PIN_NOPULL, 0);  //#LPCOMP2_N
    hwp_tsen->BGR |= TSEN_BGR_EN;
}

/**
  * @brief  DeInitialize the COMP MSP.
  * @param  hcomp  COMP handle
  * @retval None
  */
__weak void HAL_COMP_MspDeInit(COMP_HandleTypeDef *hcomp)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcomp);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_COMP_MspDeInit could be implemented in the user file
     */

    //HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_NOPULL, 0);  //#LPCOMP1_P
    //HAL_PIN_Set(PAD_PB03, GPIO_B3, PIN_NOPULL, 0);  //#LPCOMP1_N
    //HAL_PIN_Select(PAD_PB01, 10, 0);
    //HAL_PIN_Select(PAD_PB03, 10, 0);
    // recover to default
    HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_PULLUP, 0);              // I2C4 INT
    HAL_PIN_Set(PAD_PB03, GPIO_B3, PIN_NOPULL, 0);
    //HAL_PIN_Set(PAD_PB04, GPIO_B4, PIN_NOPULL, 0);  //#LPCOMP2_P
    //HAL_PIN_Set(PAD_PB05, GPIO_B5, PIN_NOPULL, 0);  //#LPCOMP2_N
    hwp_tsen->BGR &= ~TSEN_BGR_EN;
}

#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User COMP Callback
  *         To be used instead of the weak predefined callback
  * @param  hcomp Pointer to a COMP_HandleTypeDef structure that contains
  *                the configuration information for the specified COMP.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_COMP_TRIGGER_CB_ID Trigger callback ID
  *          @arg @ref HAL_COMP_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_COMP_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_RegisterCallback(COMP_HandleTypeDef *hcomp, HAL_COMP_CallbackIDTypeDef CallbackID, pCOMP_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hcomp->ErrorCode |= HAL_COMP_ERROR_INVALID_CALLBACK;

        return HAL_ERROR;
    }

    if (HAL_COMP_STATE_READY == hcomp->State)
    {
        switch (CallbackID)
        {
        case HAL_COMP_TRIGGER_CB_ID :
            hcomp->TriggerCallback = pCallback;
            break;

        case HAL_COMP_MSPINIT_CB_ID :
            hcomp->MspInitCallback = pCallback;
            break;

        case HAL_COMP_MSPDEINIT_CB_ID :
            hcomp->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hcomp->ErrorCode |= HAL_COMP_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status = HAL_ERROR;
            break;
        }
    }
    else if (HAL_COMP_STATE_RESET == hcomp->State)
    {
        switch (CallbackID)
        {
        case HAL_COMP_MSPINIT_CB_ID :
            hcomp->MspInitCallback = pCallback;
            break;

        case HAL_COMP_MSPDEINIT_CB_ID :
            hcomp->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hcomp->ErrorCode |= HAL_COMP_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hcomp->ErrorCode |= HAL_COMP_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    return status;
}

/**
  * @brief  Unregister a COMP Callback
  *         COMP callback is redirected to the weak predefined callback
  * @param  hcomp Pointer to a COMP_HandleTypeDef structure that contains
  *                the configuration information for the specified COMP.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_COMP_TRIGGER_CB_ID Trigger callback ID
  *          @arg @ref HAL_COMP_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_COMP_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_UnRegisterCallback(COMP_HandleTypeDef *hcomp, HAL_COMP_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (HAL_COMP_STATE_READY == hcomp->State)
    {
        switch (CallbackID)
        {
        case HAL_COMP_TRIGGER_CB_ID :
            hcomp->TriggerCallback = HAL_COMP_TriggerCallback;         /* Legacy weak callback */
            break;

        case HAL_COMP_MSPINIT_CB_ID :
            hcomp->MspInitCallback = HAL_COMP_MspInit;                 /* Legacy weak MspInit */
            break;

        case HAL_COMP_MSPDEINIT_CB_ID :
            hcomp->MspDeInitCallback = HAL_COMP_MspDeInit;             /* Legacy weak MspDeInit */
            break;

        default :
            /* Update the error code */
            hcomp->ErrorCode |= HAL_COMP_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_COMP_STATE_RESET == hcomp->State)
    {
        switch (CallbackID)
        {
        case HAL_COMP_MSPINIT_CB_ID :
            hcomp->MspInitCallback = HAL_COMP_MspInit;                 /* Legacy weak MspInit */
            break;

        case HAL_COMP_MSPDEINIT_CB_ID :
            hcomp->MspDeInitCallback = HAL_COMP_MspDeInit;             /* Legacy weak MspDeInit */
            break;

        default :
            /* Update the error code */
            hcomp->ErrorCode |= HAL_COMP_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hcomp->ErrorCode |= HAL_COMP_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    return status;
}

#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup COMP_Exported_Functions_Group2 Start-Stop operation functions
 *  @brief   Start-Stop operation functions.
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Start a comparator instance.
      (+) Stop a comparator instance.

@endverbatim
  * @{
  */

/**
  * @brief  Start the comparator.
  * @param  hcomp  COMP handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_Start(COMP_HandleTypeDef *hcomp)
{
    __IO uint32_t wait_loop_index = 0UL;
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the COMP handle allocation and lock status */
    if (hcomp == NULL)
    {
        status = HAL_ERROR;
    }
    else
    {
        if (hcomp->State == HAL_COMP_STATE_READY)
        {
            /* Enable the selected comparator */
            if (hcomp->Init.WorkingPin == 0)
                SET_BIT(hcomp->Instance->CR1, LPCOMP_CR1_EN);
            else if (hcomp->Init.WorkingPin == 1)
                SET_BIT(hcomp->Instance->CR2, LPCOMP_CR2_EN);
            else
                SET_BIT(hcomp->Instance->CR1, LPCOMP_CR1_EN);

            /* Set HAL COMP handle state */
            hcomp->State = HAL_COMP_STATE_BUSY;

            /* Delay for COMP startup time */
            /* Wait loop initialization and execution */
            /* Note: Variable divided by 2 to compensate partially              */
            /*       CPU processing cycles, scaling in us split to not          */
            /*       exceed 32 bits register capacity and handle low frequency. */
            wait_loop_index = COMP_DELAY_STARTUP_US / 10UL;
            while (wait_loop_index != 0UL)
            {
                wait_loop_index--;
                HAL_Delay_us(10);
            }
        }
        else
        {
            status = HAL_ERROR;
        }
    }

    return status;
}

/**
  * @brief  Stop the comparator.
  * @param  hcomp  COMP handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_Stop(COMP_HandleTypeDef *hcomp)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the COMP handle allocation and lock status */
    if (hcomp == NULL)
    {
        status = HAL_ERROR;
    }
    else
    {

        /* Check compliant states: HAL_COMP_STATE_READY or HAL_COMP_STATE_BUSY    */
        /* (all states except HAL_COMP_STATE_RESET and except locked status.      */
        if (hcomp->State != HAL_COMP_STATE_RESET)
        {
            /* Disable the selected comparator */
            if (hcomp->Init.WorkingPin == 0)
                CLEAR_BIT(hcomp->Instance->CR1, LPCOMP_CR1_EN);
            else if (hcomp->Init.WorkingPin == 1)
                CLEAR_BIT(hcomp->Instance->CR2, LPCOMP_CR2_EN);
            else
                CLEAR_BIT(hcomp->Instance->CR1, LPCOMP_CR1_EN);

            /* Set HAL COMP handle state */
            hcomp->State = HAL_COMP_STATE_READY;
        }
        else
        {
            status = HAL_ERROR;
        }
    }

    return status;
}

/**
  * @brief  select which comparator to work.
  * @param  hcomp  COMP handle
  * @param  cfg configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_Config(COMP_HandleTypeDef *hcomp, COMP_ConfigTypeDef *cfg)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t tmp_csr;

    /* Check the COMP handle allocation and lock status */
    if (hcomp == NULL || cfg == NULL)
    {
        return HAL_ERROR;
    }
    // check use which comp
    if (cfg->WorkingPin == 0)   // comp1
    {
        /* Set COMP1 parameters */
        tmp_csr = (cfg->InvertingInput
                   | cfg->InternalVRef
                   | cfg->Mode
                   | cfg->TriggerMode
                  );

        /* Set parameters in COMP register */
        /* Note: Update all bits except read-only, lock and enable bits */

        MODIFY_REG(hcomp->Instance->CR1,
                   LPCOMP_CR1_MODE | LPCOMP_CR1_VREFINT | LPCOMP_CR1_INMSEL | LPCOMP_CR1_SENS,
                   tmp_csr
                  );
    }
    else if (cfg->WorkingPin == 1)  // comp2
    {
        /* Set COMP2 parameters */
        tmp_csr = (cfg->InvertingInput
                   | cfg->InternalVRef
                   | cfg->Mode
                   | cfg->TriggerMode
                  );

        /* Set parameters in COMP register */
        /* Note: Update all bits except read-only, lock and enable bits */

        MODIFY_REG(hcomp->Instance->CR2,
                   LPCOMP_CR2_MODE | LPCOMP_CR2_VREFINT | LPCOMP_CR2_INMSEL | LPCOMP_CR2_SENS,
                   tmp_csr
                  );
    }
    else if (cfg->WorkingPin == 2)  // comb
    {
        /* Set COMP1 parameters */
        tmp_csr = (cfg->InvertingInput
                   | cfg->InternalVRef
                   | cfg->Mode
                   | cfg->TriggerMode
                  );

        /* Set parameters in COMP register */
        /* Note: Update all bits except read-only, lock and enable bits */

        MODIFY_REG(hcomp->Instance->CR1,
                   LPCOMP_CR1_MODE | LPCOMP_CR1_VREFINT | LPCOMP_CR1_INMSEL | LPCOMP_CR1_SENS,
                   tmp_csr
                  );
        MODIFY_REG(hcomp->Instance->CR2,
                   LPCOMP_CR2_MODE | LPCOMP_CR2_VREFINT | LPCOMP_CR2_INMSEL | LPCOMP_CR2_SENS,
                   tmp_csr
                  );
        CLEAR_BIT(hcomp->Instance->CR1, LPCOMP_CR1_INPSEL); // SET input to COMP1_P
        SET_BIT(hcomp->Instance->CR2, LPCOMP_CR2_INPSEL);   // SET input to COMP2_P
    }
    else
        return HAL_ERROR;
    return status;
}

/**
  * @brief  polling compare result.
  * @param  hcomp  COMP handle
  * @param  channel channel select, 0, 1 valid
  * @param  timeout timeout in ms
  * @retval compare result
  */
int HAL_COMP_PollForComp(COMP_HandleTypeDef *hcomp, uint8_t channel, uint32_t timeout)
{
    uint32_t value;
    volatile uint32_t *creg;
    __IO uint32_t wait_loop_index = 0UL;

    /* Check the COMP handle allocation and lock status */
    if (hcomp == NULL || channel > 2)
    {
        return -1;
    }

    //status = HAL_COMP_Start(hcomp);
    //if (status != HAL_OK)
    //    return -1;

    /* Get tick count */
    if (channel == 1)
        creg = &(hcomp->Instance->CR2);
    else
        creg = &(hcomp->Instance->CR1);

    SET_BIT(*creg, LPCOMP_CR1_EN);
#if 0
    uint32_t tickstart = HAL_GetTick();
    while (HAL_IS_BIT_CLR(*creg, LPCOMP_CR1_STAT))
    {
        /* Check if timeout is disabled (set to infinite wait) */
        if (timeout != HAL_MAX_DELAY)
        {
            if ((timeout == 0) || ((HAL_GetTick() - tickstart) > timeout))
            {
                /* Update COMP state machine to timeout */
                //SET_BIT(hcomp->State, HAL_ADC_STATE_TIMEOUT);

                /* Process unlocked */
                __HAL_UNLOCK(hcomp);

                return -2;
            }
        }
        HAL_Delay_us(10);
    }
    // Clear state
    *creg |=  LPCOMP_CR1_STAT;
#else
    wait_loop_index = COMP_DELAY_STARTUP_US / 10UL;
    while (wait_loop_index != 0UL)
    {
        wait_loop_index--;
        HAL_Delay_us(10);
    }
#endif
    value = (*creg & LPCOMP_CR1_VAL) >> LPCOMP_CR1_VAL_Pos;

    CLEAR_BIT(*creg, LPCOMP_CR1_EN);

    //hcomp->State = HAL_COMP_STATE_READY;

    return value;
}


/**
  * @brief  Comparator IRQ handler.
  * @param  hcomp  COMP handle
  * @retval None
  */
void HAL_COMP_IRQHandler(COMP_HandleTypeDef *hcomp)
{
    /* COMP trigger user callback */
#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
    hcomp->TriggerCallback(hcomp);
#else
    HAL_COMP_TriggerCallback(hcomp);
#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */

    // clear enable to avoid int always come for level int
    if (hcomp->Init.WorkingPin == 0)
    {
        //CLEAR_BIT(hcomp->Instance->CR1, LPCOMP_CR1_EN);
        hcomp->Instance->CR1 |= LPCOMP_CR1_STAT;
    }
    else
    {
        //CLEAR_BIT(hcomp->Instance->CR2, LPCOMP_CR2_EN);
        hcomp->Instance->CR2 |= LPCOMP_CR2_STAT;
    }

}

/**
  * @}
  */

/** @defgroup COMP_Exported_Functions_Group3 Peripheral Control functions
 *  @brief   Management functions.
 *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the comparators.

@endverbatim
  * @{
  */


/**
  * @brief  Return the output level (high or low) of the selected comparator.
  *         The output level depends on the selected polarity.
  *         If the polarity is not inverted:
  *           - Comparator output is low when the input plus is at a lower
  *             voltage than the input minus
  *           - Comparator output is high when the input plus is at a higher
  *             voltage than the input minus
  *         If the polarity is inverted:
  *           - Comparator output is high when the input plus is at a lower
  *             voltage than the input minus
  *           - Comparator output is low when the input plus is at a higher
  *             voltage than the input minus
  * @param  hcomp  COMP handle
  * @retval Returns the selected comparator output level:
  *         @arg COMP_OUTPUT_LEVEL_LOW
  *         @arg COMP_OUTPUT_LEVEL_HIGH
  *
  */
uint32_t HAL_COMP_GetOutputLevel(COMP_HandleTypeDef *hcomp)
{
    /* Check the parameter */
    if (hcomp->Init.WorkingPin == 1)
        return (uint32_t)(READ_BIT(hcomp->Instance->CR2, LPCOMP_CR2_VAL)
                          >> LPCOMP_CR2_VAL_Pos);
    else
        return (uint32_t)(READ_BIT(hcomp->Instance->CR1, LPCOMP_CR1_VAL)
                          >> LPCOMP_CR1_VAL_Pos);
}

/**
  * @brief  Comparator trigger callback.
  * @param  hcomp  COMP handle
  * @retval None
  */
__weak void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcomp);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_COMP_TriggerCallback should be implemented in the user file
     */
}


/**
  * @}
  */

/** @defgroup COMP_Exported_Functions_Group4 Peripheral State functions
 *  @brief   Peripheral State functions.
 *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Return the COMP handle state.
  * @param  hcomp  COMP handle
  * @retval HAL state
  */
HAL_COMP_StateTypeDef HAL_COMP_GetState(COMP_HandleTypeDef *hcomp)
{
    /* Check the COMP handle allocation */
    if (hcomp == NULL)
    {
        return HAL_COMP_STATE_RESET;
    }

    /* Return HAL COMP handle state */
    return hcomp->State;
}

/**
  * @brief  Return the COMP error code.
  * @param hcomp COMP handle
  * @retval COMP error code
  */
uint32_t HAL_COMP_GetError(COMP_HandleTypeDef *hcomp)
{
    /* Check the parameters */

    return hcomp->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


#endif /* HAL_COMP_MODULE_ENABLED */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
