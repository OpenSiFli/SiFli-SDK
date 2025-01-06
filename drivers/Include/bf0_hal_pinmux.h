/**
  ******************************************************************************
  * @file   bf0_hal_pinmux.h
  * @author Sifli software development team
  * @brief   Header file for pinmux
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

#ifndef __BF0_HAL_PINMUX_H
#define __BF0_HAL_PINMUX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
#include "bf0_pin_const.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup PINMUX PINMUX
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup PIN_flags PIN pull flag
  * @brief PIN Pull-Up or Pull-Down Activation
  * @{
  */
// Pullup: PE=1, PS=1,
// Pulldown:PE=1, PS=0
// All pins has same flags position, use HPSYS_PINMUX_PAD_PA_00 flags
#define  PIN_NOPULL        (0x00000000u)                                /*!< No Pull-up or Pull-down activation  */
#define  PIN_PULLUP        (HPSYS_PINMUX_PAD_PA00_PE|HPSYS_PINMUX_PAD_PA00_PS)   /*!< Pull-up activation                  */
#define  PIN_PULLDOWN      (HPSYS_PINMUX_PAD_PA00_PE)                  /*!< Pull-down activation                */

/**
 * @} PIN_flags
 */

#ifdef SF32LB58X

#define SFPIN_MPI_PIN_HPSRAM         (0)     // USED FOR HPI PSRAM
#define SFPIN_MPI_PIN_OPSRAM         (1)     // USED FOR OPI PSRAM
#define SFPIN_MPI1_PIN_P32           (2)     // MPI1 USE PUYA 32Mb
#define SFPIN_MPI1_PIN_P64           (3)     // MPI1 USE PUYA 64Mb
#define SFPIN_MPI1_PIN_G32           (4)     // MPI1 USE GIGA 32Mb
#define SFPIN_MPI1_PIN_G64           (5)     // MPI1 USE GIGA 64Mb
#define SFPIN_MPI1_PIN_AUTO          (0XFF)

#elif defined(SF32LB56X)
typedef enum
{
    SFPIN_SIP1_NONE = 0,
    SFPIN_SIP1_APM_XCA64 = 1,
    SFPIN_SIP1_APM_LEG32 = 2,
    SFPIN_SIP1_WINB_HYP64 = 3,
    SFPIN_SIP1_WINB_HYP32 = 4,
    SFPIN_SIP1_PY_NOR64 = 5,
    SFPIN_SIP1_PY_NOR32 = 6,
    SFPIN_SIP1_GD_NOR64 = 7,
    SFPIN_SIP1_GD_NOR32 = 8
} SFPIN_SIP1_MODE;

typedef enum
{
    SFPIN_SIP2_NONE = 0,
    SFPIN_SIP2_APM_XCA128 = 1,
    SFPIN_SIP2_APM_LEG32 = 2,
    SFPIN_SIP2_WINB_HYP128 = 3,
    SFPIN_SIP2_WINB_HYP32 = 4,
    SFPIN_SIP2_PY_NOR16 = 5
} SFPIN_SIP2_MODE;

#endif

#ifdef SF32LB56X
/* they're used by xtal32, PE cannot be enabled and no valid function should be selected */
#define HAL_PIN_SetXT32()    \
    do                       \
    {                        \
        hwp_pinmux1->PAD_PA55 = 10;  \
        hwp_pinmux1->PAD_PA56 = 10;  \
    }                                \
    while (0)
#elif defined(SF32LB52X)
#define HAL_PIN_SetXT32()    \
    do                       \
    {                        \
        hwp_pinmux1->PAD_PA22 = 8;  \
        hwp_pinmux1->PAD_PA23 = 8;  \
    }                                \
    while (0)
#else
#define HAL_PIN_SetXT32()
#endif



/* Pin mode type defines                */
typedef enum
{
    PIN_ANALOG_INPUT,
    PIN_DIGITAL_IO_NORMAL,
    PIN_DIGITAL_IO_PULLUP,
    PIN_DIGITAL_IO_PULLDOWN,
    PIN_DIGITAL_O_NORMAL,
    PIN_DIGITAL_O_PULLUP,
} PIN_ModeTypeDef;


#ifdef SOC_BF0_HCPU
#define PIN_SAVE_PINMUX_INSTANCE_SIZE       (HPSYS_PAD_NUM)
#define PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE   (HPSYS_CFG_PINR_SIZE)
#else
#define PIN_SAVE_PINMUX_INSTANCE_SIZE       (LPSYS_PAD_NUM)
#define PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE   (LPSYS_CFG_PINR_SIZE)
#endif /* SOC_BF0_HCPU */

typedef struct
{
    uint32_t pad_regs[PIN_SAVE_PINMUX_INSTANCE_SIZE];
    uint32_t ext_regs[PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE];
} PIN_BackupBufTypeDef;


/* Exported functions --------------------------------------------------------*/
/** @addtogroup PIN_Exported_Functions PIN Exported Functions
 * @{
 */

/**
  * @brief  Select pin function only.
  * @param  pad: physical pin
  * @param  func: Pin function.
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  */
void HAL_PIN_Select(int pad, int func, int hcpu);


/**
 * @brief  Set pin function.
 * @param  pad: physical pin, #pin_pad_hcpu or #pin_pad_lcpu
 * @param  func: Pin function.
 * @param  flags: flag of the pin (pullup/pulldown), @ref PIN_flags
 * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
 * @retval -1 if invalid, otherwise 0
 */
int HAL_PIN_Set(int pad, pin_function func, int flags, int hcpu);


/**
  * @brief  Set pin for analog function, fix for ROM patch, avoid pin_const update.
  * @param  pad: physical pin
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @retval -1 if invalid, otherwise 0
  */
int HAL_PIN_Set_Analog(int pad, int hcpu);


/**
  * @brief  Update pin flags.
  * @param  pad: physical pin
  * @param  flags: Flags for the pin to update
  * @param  mask: Mask of the flags
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @retval -1 if invalid, otherwise 0
  */
int HAL_PIN_Update(int pad, uint32_t flags, uint32_t mask, int hcpu);

/**
  * @brief  Get pin function.
  * @param  pad: physical pin, refer #pin_pad_hcpu and #pin_pad_lcpu
  * @param  p_func: Pointer of variable to save pin function.
  * @param  p_mode: Pointer of varibale to save flag of the pin mode, see PIN_ModeTypeDef
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @retval -1 if invalid, else function idx(>= 0)
  */
int HAL_PIN_Get(int pad, pin_function *p_func, PIN_ModeTypeDef *p_mode, int hcpu);

/**
 * @brief  Set pin DS0.
 * @param  pad physical pin, #pin_pad_hcpu or #pin_pad_lcpu
 * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
 * @param  set 1: select, 0: deselect
 * @retval -1 if invalid, otherwise 0
 */
int HAL_PIN_Set_DS0(int pad, int hcpu, uint8_t set);

/**
 * @brief  Set pin DS1.
 * @param  pad physical pin, refer #pin_pad_hcpu and #pin_pad_lcpu
 * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
 * @param  set 1: select, 0: deselect
 * @retval -1 if invalid, otherwise 0
 */
int HAL_PIN_Set_DS1(int pad, int hcpu, uint8_t set);


int HAL_PIN_SetMode(int pad, int hcpu, PIN_ModeTypeDef mode);


/**
 * @brief  Set pinmux of dual mode of flash1.
 */
void HAL_PIN_Set_Dual_flash1(void);

/**
 * @brief  Set pinmux of single mode of flash2.
 */
void HAL_PIN_Set_Single_flash2(void);

/**
 * @brief  Set pinmux of dual mode of flash2.
 */
void HAL_PIN_Set_Dual_flash2(void);

/**
 * @brief  Reset dual mode of flash1 to default configuration.
 */
void HAL_PIN_Set_Dual_flash1_default(void);

/**
 * @brief  Reset single mode of flash2 to default configuration.
 */
void HAL_PIN_Set_Single_flash2_default(void);

/**
 * @brief  Reset dual mode of flash2 to default configuration.
 */
void HAL_PIN_Set_Dual_flash2_default(void);

/**
 * @brief  Set pinmux for flash3.
 */
void HAL_PIN_SetFlash3(void);

/**
 * @brief  Set pinmux for flash4.
 */
void HAL_PIN_SetFlash4(void);

#ifdef SF32LB58X
#define MPI1_PIN_HPSRAM  (0)
#define MPI1_PIN_OPSRAM  (1)
#define MPI1_PIN_P32     (2)
#define MPI1_PIN_P64     (3)
#define MPI1_PIN_G32     (4)
#define MPI1_PIN_G64     (5)

#define MPI2_PIN_HPSRAM  (0)
#define MPI2_PIN_OPSRAM  (1)
#endif /* SF32LB58X */

/**
 * @brief  Set pinmux for mpi1 with input mode.
 *
 * @param mode MPI1 pin mode, e.g. MPI1_PIN_HPSRAM for sf32lb58x
 */
void HAL_PIN_SetFlash1_WithMode(int mode);

/**
 * @brief  Set pinmux for mpi2 with input mode, for 58x, only HPSRAM and OPSRAM support.
 *
 * @param mode MPI2 pin mode, e.g. MPI2_PIN_HPSRAM for sf32lb58x
 */
void HAL_PIN_SetFlash2_WithMode(int mode);

#ifdef SF32LB56X
int HAL_SIP_AUTO_CFG(void);
#endif

/**
 * @brief  Backup pinmux register
 * @param[in]  buf buffer to save pinmux register value,
                   it will be used by HAL_PIN_Restore to restore pinmux registe,
                   the buffer should be in Retention RAM
 * @return status
 */
HAL_StatusTypeDef HAL_PIN_Backup(PIN_BackupBufTypeDef *buf);

/**
 * @brief  Restore pinmux register
 * @param[in]  buf buffer which is used by HAL_PIN_Backup to save pinmux register value,
                   the buffer should be in Retention RAM
 * @return status
 */
HAL_StatusTypeDef HAL_PIN_Restore(PIN_BackupBufTypeDef *buf);

/**
  * @} PIN_Exported_Functions
  */

/**
  * @} PINMUX
  */

/**
  * @} BF0_HAL_Driver
  */

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_PINMUX_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

