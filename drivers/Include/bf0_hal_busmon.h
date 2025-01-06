/**
  ******************************************************************************
  * @file   bf0_hal_busmon.h
  * @author Sifli software development team
  * @brief Bus monitor
  * @{
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_BUSMON_H
#define __BF0_HAL_BUSMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup BUSMON Bus monitor
  * @brief Bus monitor for debug purpose.
  * @{
  */

/**
  * @addtogroup  BUSMON_exported_constants
  * @{
*/
#define BUSMON_OPFLAG_READ      0
#define BUSMON_OPFLAG_WRITE     1
#define BUSMON_OPFLAG_RW        3

typedef enum
{
    HAL_BUSMON_CHN1,
    HAL_BUSMON_CHN2,
    HAL_BUSMON_CHN3,
    HAL_BUSMON_CHN4,
    HAL_BUSMON_CHN5,
    HAL_BUSMON_CHN6,
    HAL_BUSMON_CHN7,
    HAL_BUSMON_CHN8
} HAL_BUSMON_Channel;

typedef enum
{
    HAL_BUSMON_STATE_RESET             = 0x00U,   /*!< Peripheral is not yet Initialized         */
    HAL_BUSMON_STATE_READY             = 0x01U,   /*!< Peripheral Initialized and ready for use  */
    HAL_BUSMON_STATE_RUNNING           = 0x02U    /*!< Peripheral in use                         */
} HAL_BUSMON_StateTypeDef;

#define HAL_BUSMON_CHN_UNASSIGN 0xFF
#define HAL_BUSMON_MAXSEL   8

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t CNT;
    __IO uint32_t MIN;
    __IO uint32_t MAX;
} BUSMON_ChnTypeDef;


typedef enum
{
    HAL_BUSMON_NA = 0,

    HAL_BUSMON_HCPU_C = 1,      // HCPU
    HAL_BUSMON_HCPU_S,
    HAL_BUSMON_DMAC1,
    HAL_BUSMON_EXTDMA,
    HAL_BUSMON_HP_APB,
    HAL_BUSMON_HP_AHB,
    HAL_BUSMON_LCDC1,
    HAL_BUSMON_EPIC_A,
    HAL_BUSMON_EPIC_B,
    HAL_BUSMON_SDMMC1,
    HAL_BUSMON_SDMMC2,
    HAL_BUSMON_EZIP,
    HAL_BUSMON_USBC,
    HAL_BUSMON_PTC1,
    HAL_BUSMON_QSPI1,
    HAL_BUSMON_QSPI2,
    HAL_BUSMON_QSPI3,
    HAL_BUSMON_OPSRAM,

    HAL_BUSMON_LCPU_C,          // LCPU
    HAL_BUSMON_LCPU_S,
    HAL_BUSMON_DMAC2,
    HAL_BUSMON_LP_APB,
    HAL_BUSMON_LP_AHB,
    HAL_BUSMON_LCDC2,
    HAL_BUSMON_PTC2,
    HAL_BUSMON_QSPI4,
    HAL_BUSMON_PHY_DUMP,
    HAL_BUSMON_AHBT,

    HAL_BUSMON_LP2HP,           // Common
    HAL_BUSMON_HP2LP,
    HAL_BUSMON_RAM0,
    HAL_BUSMON_RAM1,
    HAL_BUSMON_RAM2,
    HAL_BUSMON_RAM3,
    HAL_BUSMON_RAM4,
    HAL_BUSMON_RAM5,

    // For 58x/56x
    HAL_BUSMON_JENC,            // HCPU
    HAL_BUSMON_JDEC,
    HAL_BUSMON_EZIP1,
    HAL_BUSMON_GPU_A,
    HAL_BUSMON_GPU_B,
    HAL_BUSMON_NNACC1_A,
    HAL_BUSMON_NNACC1_B,
    HAL_BUSMON_NNACC1_C,
    HAL_BUSMON_AES,
    HAL_BUSMON_A2HP,
    HAL_BUSMON_HP2A,
    HAL_BUSMON_MPI1,
    HAL_BUSMON_MPI2,
    HAL_BUSMON_MPI3,
    HAL_BUSMON_MPI4,

    HAL_BUSMON_ACPU_C,          // ACPU
    HAL_BUSMON_ACPU_S,
    HAL_BUSMON_FFT1,
    HAL_BUSMON_FACC1,
    HAL_BUSMON_RAM6,
    HAL_BUSMON_RAM7,
    HAL_BUSMON_RAM8,
    HAL_BUSMON_RAM9,

    HAL_BUSMON_DMAC3,           // LCPU
    HAL_BUSMON_FFT2,
    HAL_BUSMON_NNACC2_A,
    HAL_BUSMON_NNACC2_B,
    HAL_BUSMON_NNACC2_C,
    HAL_BUSMON_FACC2,
    HAL_BUSMON_MPI5,
    HAL_BUSMON_TCM,
} HAL_BUSMON_FuncTypeDef;


/**
  * @} BUSMON_exported_constants
*/

/** @defgroup BUSMON_Exported_Types BUSMON Handle Structure definition
  * @{
  */
typedef struct
{
    uint32_t             Min;       /*!< Busmon rangme minimal address   */
    uint32_t             Max;       /*!< Busmon rangme maxium  address   */
    uint32_t             count;     /*!< Count to generate PTC interrupt */
    uint8_t              SelFunc;   /*!< Selected module to be monitored*/
    uint8_t              Flags;     /*!< Selected operations to be monitored.  */
    uint8_t              Channel;   /*!< Channel for Busmon   */
    uint8_t              State;     /*!< State for Busmon   */
} BUSMON_InitTypeDef;

typedef struct
{
    BUSMON_TypeDef              *Instance;  /*!< Register base address   */
    BUSMON_ChnTypeDef           *Chn;       /*!< Channel Register base address   */
    BUSMON_InitTypeDef          Init;       /*!< Busmon init parameters     */
    HAL_BUSMON_StateTypeDef     State;      /*!< State for Busmon   */
} BUSMON_HandleTypeDef;
/**
  * @}
  */



/**
  * @defgroup BUSMON_exported_functions Bus monitor Exported functions
  * @ingroup BUSMON
  * @{
  *
 */
/**
* @brief  Initialize Busmonitor
* @param  hbusmon Handle of busmonitor
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_BUSMON_Init(BUSMON_HandleTypeDef *hbusmon);

/**
* @brief  Enable/disable Busmonitor
* @param  hbusmon Handle of busmonitor
* @param  enable 1: enable, 0: disable
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_BUSMON_Enable(BUSMON_HandleTypeDef *hbusmon, int enable);


/**
* @brief  Get Busmonitor count
* @param  hbusmon Handle of busmonitor
* @param  count count for bus monitor
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_BUSMON_GetCount(BUSMON_HandleTypeDef *hbusmon, int32_t *count);


/**
  *@} BUSMON_exported_functions
*/

/**
  *@} BUSMON
  */

/**
  *@} BF0_HAL_Driver
  */

#ifdef __cplusplus
}
#endif

#endif

/// @}  file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
