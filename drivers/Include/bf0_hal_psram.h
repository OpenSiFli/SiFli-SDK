/**
  ******************************************************************************
  * @file   bf0_hal_psram.h
  * @author Sifli software development team
  * @brief   Header file of FLASH HAL module.
  *
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

#ifndef BF0_HAL_PSRAM_H
#define BF0_HAL_PSRAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
#include "psramc.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup PSRAM
  * @{
  */


/**
  * @brief  HAL State structures definition
  */
typedef enum
{
    HAL_PSRAM_STATE_RESET             = 0x00U,  /*!< PSRAM not yet initialized or disabled */
    HAL_PSRAM_STATE_READY             = 0x01U,  /*!< PSRAM initialized and ready for use   */
    HAL_PSRAM_STATE_BUSY              = 0x02U,  /*!< PSRAM process is ongoing              */
    HAL_PSRAM_STATE_TIMEOUT           = 0x03U,  /*!< PSRAM timeout state                   */
    HAL_PSRAM_STATE_ERROR             = 0x04U   /*!< PSRAM error state                     */
} HAL_PSRAMStateTypeDef;

/**
  * @brief  PSRAM Configuration Structure definition
  */
typedef struct
{
    uint32_t delay;
} PSRAM_InitTypeDef;


/**
  * @brief  PSRAM Handle Structure definition
  */
typedef struct
{
    PSRAMC_TypeDef                 *Instance;  /*!< Register base address    */

    PSRAM_InitTypeDef              Init;       /*!< PSRAM required parameters  */

    HAL_LockTypeDef                Lock;       /*!< PSRAM locking object       */

    __IO HAL_PSRAMStateTypeDef     State;      /*!< PSRAM communication state */

    bool                           dual_psram;  /*!< if use dual psram, true: yes, false: no */
    bool                           is_xccela;   /*!< if use xccela psram, true: yes, false, no */
    bool                           wakeup;      /*!< if reinit for wakeup from standby */

} PSRAM_HandleTypeDef;

/**
  * @brief  PSRAM Configure Structure definition, use register table
  */
typedef struct
{
    uint32_t ctrl_time;
    uint32_t read_ctrl;
    uint32_t delay_final_add;
    uint32_t dqs_ctrl;
    uint32_t clk_ctrl;
    uint32_t power_up;
    uint32_t power_time;
    uint32_t reg_time;
    uint32_t imr;
} PSRAM_CONFIG_HAL_T;



#define __HAL_PSRAM_SET_LOW_POWER(__HANDLE__)  \
    do                                                 \
    {                                                  \
            ((__HANDLE__)->Instance->CRE) |= PSRAMC_CRE_CRE;    \
            ((__HANDLE__)->Instance->MR4) |= 0x8;     \
            ((__HANDLE__)->Instance->MR6) |= 0xF0;    \
            ((__HANDLE__)->Instance->CRE) = 0;        \
            HAL_Delay_us(20);                         \
    }                                                 \
    while (0)



#define __HAL_PSRAM_EXIT_LOW_POWER(__HANDLE__)  \
        do                                                 \
        {                                                  \
                ((__HANDLE__)->Instance->CRE) |= PSRAMC_CRE_CRE;    \
                ((__HANDLE__)->Instance->POWER_UP) |= (0X1F << PSRAMC_POWER_UP_WAKE_UP_TIME_Pos);      \
                ((__HANDLE__)->Instance->POWER_UP) |= PSRAMC_POWER_UP_WAKE_UP_TRIG;      \
                ((__HANDLE__)->Instance->CRE) = 0;         \
                HAL_Delay_us(200);                         \
        }                                                  \
        while (0)


#define __HAL_PSRAM_SET_LOW_POWER_XCCELA(__HANDLE__)                \
        do                                                          \
        {                                                           \
                ((__HANDLE__)->Instance->CRE) |= PSRAMC_CRE_CRE;    \
                ((__HANDLE__)->Instance->CTRL_TIME) &= ~PSRAMC_CTRL_TIME_WL;    \
                ((__HANDLE__)->Instance->MR4) = 0x00480048UL;               \
                ((__HANDLE__)->Instance->MR6) = 0xF0;               \
                ((__HANDLE__)->Instance->CTRL_TIME) |= (7<<PSRAMC_CTRL_TIME_WL_Pos);    \
                ((__HANDLE__)->Instance->CRE) = 0;                  \
                HAL_Delay_us(200);                                  \
        }                                                           \
        while (0)



#define __HAL_PSRAM_EXIT_LOW_POWER_XCCELA(__HANDLE__)                                                  \
        do                                                                                             \
        {                                                                                              \
                ((__HANDLE__)->Instance->CRE) |= PSRAMC_CRE_CRE;                                       \
                ((__HANDLE__)->Instance->POWER_UP) |= (0X1F << PSRAMC_POWER_UP_WAKE_UP_TIME_Pos);      \
                ((__HANDLE__)->Instance->POWER_UP) |= PSRAMC_POWER_UP_WAKE_UP_TRIG;                    \
                ((__HANDLE__)->Instance->CRE) = 0;                                                     \
                HAL_Delay_us(200);                                                                     \
        }                                                                                              \
        while (0)

/* Exported functions --------------------------------------------------------*/

/** @addtogroup PSRAM_Exported_Functions
  * @{
  */

/** @addtogroup PSRAM_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/

/**
* @brief  Initialize PSRAM
* @param  hpsram Handle of PSRAM
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_PSRAM_Init(PSRAM_HandleTypeDef *hpsram);
/**
* @brief  UnInitialize PSRAM
* @param  hpsram Handle of PSRAM
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_PSRAM_DeInit(PSRAM_HandleTypeDef *hpsram);

/**
  * @}
  */

/** @addtogroup PSRAM_Exported_Functions_Group2
  * @{
  */
/* configure operation functions *****************************************************/

/**
* @brief  Configure PSRAM
* @param  hpsram Handle of PSRAM
* @param  conf Configuration of PSRAM
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_PSRAM_Config(PSRAM_HandleTypeDef *hpsram, PSRAM_CONFIG_HAL_T *conf);

/**
  * @brief Initialize the PSRAM MSP.
  * @param hpsram PSRAM handle.
  * @retval None
  */
void HAL_PSRAM_MspInit(PSRAM_HandleTypeDef *hpsram);



/**
  * @}
  */

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

#endif /* BF0_HAL_PSRAM_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
