/**
  ******************************************************************************
  * @file   bf0_hal_hlp.h
  * @author Sifli software development team
  * @brief HAL help functions.
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

#ifndef __BF0_HAL_HLP_H
#define __BF0_HAL_HLP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "bf0_hal_def.h"
/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup HAL_DBG HAL Debug module
 * @brief HAL Debug module
 * @{
 */

/**
 * @addtogroup  HAL_DBG_exported_constants
 * @{
*/
/**
 * @} HAL_DBG_exported_constants
 */


/**
 * @defgroup HAL_DBG_exported_functions HAL Debug Exported functions
 * @ingroup HAL_DBG
 * @{
 *
 */

/**
 * @brief  HAL Debug printf
 * @param[in] fmt format for printf
 * @retval None.
 */
#ifndef WIN32
void HAL_DBG_printf(const char *fmt, ...);
#else
#include "stdio.h"
#include "memory.h"
#define HAL_DBG_printf printf
#define __HAL_ROM_USED
#endif
/**
 * @brief  Dump a data in hex, also display ASCII on the right.
 * @param[in] data data to dump.
 * @param[in] offset start offset from data.
 * @param[in] len length of data to dump.
 * @retval None.
 */
void HAL_DBG_print_data(char *data, unsigned int offset, int len);


#ifndef WIN32
/**
 * @brief  Initialize I2C pins for debug printing.
 * @param[in] scl Pin for SCL.    (GPIOA0-87, GPIOB0-63)
 * @param[in] sda Pin for SDA.    (GPIOA0-87, GPIOB0-63)
 * @retval None.
 */
void HAL_DBG_i2c_pins(uint16_t scl, uint16_t sda);

/**
 * @brief  Print byte on I2C pins.
 * @param[in] byte Byte to print.
 * @retval None.
 */
void HAL_DBG_i2c_print_byte(uint8_t byte);

/**
 * @brief  Print strings on I2C pins.
 * @param[in] data Data to print.
 * @param[in] len length of data in bytes.
 * @retval None.
 */
void HAL_DBG_i2c_print(uint8_t *data, int len);


#ifdef DWT

/**
 * @brief  Initialize Data Watchpoint and Trace Register (DWT) module.
 * @retval None.
 */
__STATIC_INLINE void HAL_DBG_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk | DWT_CTRL_CPIEVTENA_Msk;
}

/**
 * @brief  Deinitialize Data Watchpoint and Trace Register (DWT) module.
 * @retval None.
 */
__STATIC_INLINE void HAL_DBG_DWT_DeInit(void)
{
    DWT->CTRL &= ~(DWT_CTRL_CYCCNTENA_Msk | DWT_CTRL_CPIEVTENA_Msk);
}

/**
 * @brief  Check if DWT initialized.
 * @retval 1 if DWT has been initialized before, or return 0.
 */
__STATIC_INLINE uint32_t HAL_DBG_DWT_IsInit(void)
{
    if (0 == (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
        return 0;
    else
        return 1;
}

/**
 * @brief  Reset DWT counter
 * @retval None.
 */
__STATIC_INLINE void HAL_DBG_DWT_Reset(void)
{
    DWT->CYCCNT = 0; /* Clear DWT cycle counter */
}


/**
 * @brief  Get DWT counter
 * @retval  Cycle Count since last reset.
 */
__STATIC_INLINE uint32_t HAL_DBG_DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief  Trigger breakpoint on RAM content change.
 * @retval  None
 */
__STATIC_INLINE void HAL_DBG_DWT_Trigger(uint32_t addr)
{
    // TODO: Revisit to verify.
    DWT->COMP3 = addr;
    DWT->FUNCTION3 &= ~DWT_FUNCTION_MATCH_Msk;
    DWT->FUNCTION3 |= 0x15;
}

#endif /* DWT */

/**
 * @brief 32bit lock read, use in pair with HAL_LOCK_Write32
 * @param[in] addr address of data to be read
 * @return  read value
 */
__STATIC_INLINE uint32_t HAL_LOCK_Read32(volatile uint32_t *addr)
{
    return __LDREXW(addr);
}

/**
 * @brief 32bit lock write, follow HAL_LOCK_Read32
 * @param[in] addr address of data to be written
 * @param[in] value value to be written
 * @return write succeeed or not, true: success, false: fail. If fail, try lock_read and lock_write again
 */
__STATIC_INLINE bool HAL_LOCK_Write32(volatile uint32_t *addr, uint32_t value)
{
    return (__STREXW(value, addr) == 0) ? true : false;
}


/**
 * @brief 16bit lock read, use in pair with HAL_LOCK_Write16
 * @param[in] addr address of data to be read
 * @return  read value
 */
__STATIC_INLINE uint16_t HAL_LOCK_Read16(volatile uint16_t *addr)
{
    return __LDREXH(addr);
}

/**
 * @brief 16bit lock write, follow HAL_LOCK_Read16
 * @param[in] addr address of data to be written
 * @param[in] value value to be written
 * @return write succeeed or not, true: success, false: fail. If fail, try lock_read and lock_write again
 */
__STATIC_INLINE bool HAL_LOCK_Write16(volatile uint16_t *addr, uint16_t value)
{
    return (__STREXH(value, addr) == 0) ? true : false;
}

/**
 * @brief 8bit lock read, use in pair with HAL_LOCK_Write8
 * @param[in] addr address of data to be read
 * @return  read value
 */
__STATIC_INLINE uint8_t HAL_LOCK_Read8(volatile uint8_t *addr)
{
    return __LDREXB(addr);
}

/**
 * @brief 8bit lock write, follow HAL_LOCK_Read8
 * @param[in] addr address of data to be written
 * @param[in] value value to be written
 * @return write succeeed or not, true: success, false: fail. If fail, try lock_read and lock_write again
 */
__STATIC_INLINE bool HAL_LOCK_Write8(volatile uint8_t *addr, uint8_t value)
{
    return (__STREXB(value, addr) == 0) ? true : false;
}

#endif




/**
  * @brief  General Purpose backup registers in RTC, used to share value among H/L CPU
  */
#define RTC_BACKUP_FLAGS_RESET  1               /*!< Inform other CPU core that RTC is reset*/
#define RTC_BACKUP_FLAGS_ACK    0x80000         /*!< Ack for RTC reset from other CPU core*/
typedef enum
{
    RTC_BACKUP_BOOTOPT = 0,                     /*!< Boot option  */
    RTC_BACKUP_LPCYCLE = 1,                     /*!< RTC start LP Cycles in  48M counts plus software PPM  */
    RTC_BACKUP_INITIALIZED = 2,                 /*!< RTC Initialized  */
    RTC_BACKUP_LPCYCLE_CUR = 3,                 /*!< RTC current LP Cycles in  48M counts  */
    RTC_BACKUP_LPCYCLE_AVE = 4,                 /*!< RTC average LP Cycles in  48M counts  */
    RTC_BAKCUP_OTA_FORCE_MODE = 5,              /*!< OTA Failed Mode                      */
    RTC_BAKCUP_WDT_STATUS = 6,                  /*!< WDT status                           */
    RTC_BACKUP_BT_TXPWR = 7,                   /*!< Bluetooth TX Power                  */
    RTC_BACKUP_NAND_OTA_DES = 8,                /*!< NAND OTA hcput switch                */
    RTC_BACKUP_MODULE_RECORD = 9,               /*!< module execution record */
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
    RTC_BACKUP_BT_LPCYCLE = 10,
    RTC_BACKUP_RC_ALGO_PARA = 11,                /*!< RC ALGO para: bit:0~7 lp_cycle; bit: 8~11: interval; bit:12~21: thd0; bit:22~31: thd1. */
#endif
} RTC_BACKUP;

#ifdef WIN32
#define HAL_Set_backup(idx,value)
#define HAL_Get_backup(idx) 0
#else
/**
  * @brief  Set the RTC backup register, used to share value between H/L CPU
  * @param  idx index of backup register
  * @param  value value to set into backup register
  * @retval None
  */
void HAL_Set_backup(uint8_t idx, uint32_t value);

/**
  * @brief  Get the RTC backup register, used to share value between H/L CPU
  * @param  idx index of backup register
  * @retval value saved in the backup register
  */
uint32_t HAL_Get_backup(uint8_t idx);
#endif

/**
  * @brief  Convert a hex string to integer.
  * @param[in] astr hex string
  * @retval integer represented by astr.
  */
uint32_t atoh(char *astr);

/**
  * @brief  Convert a hex string to byte array
  * @param[in] hexstr hex string
  * @param[in, out] data byte array
  * @param[in] max_len max length of byte array
  * @retval length of hex data array.
  */
int hex2data(char *hexstr, uint8_t *data, uint8_t max_len);

/**
  * @brief  Convert a hex string to byte array in revert sequence
  * @param[in] hexstr hex string
  * @param[in, out] data byte array
  * @param[in] max_len max length of byte array
  * @retval integer represented by astr.
  */
int hex2data_revert(char *hexstr, uint8_t *data, uint8_t max_len);


/**
  *@} HAL_DBG_exported_functions
*/

/**
  *@} HAL_DBG
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
