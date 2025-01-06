/**
  ******************************************************************************
  * @file   system_bf0_ap.h
  * @author Sifli software development team
  * @brief    CMSIS Device System Header File for
 *           ARMCM33 Device
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

#ifndef SYSTEM_BF0_AP_H
#define SYSTEM_BF0_AP_H

#ifdef __cplusplus
extern "C" {
#endif

/** power on mode */
typedef enum
{
    PM_COLD_BOOT = 0,  /**< cold boot */
    PM_STANDBY_BOOT,   /**< boot from standby power mode */
    PM_HIBERNATE_BOOT, /**< boot from hibernate mode, system can be woken up by RTC and PIN precisely */
    PM_SHUTDOWN_BOOT,   /**< boot from shutdown mode, system can be woken by RTC and PIN, but wakeup time is not accurate */
    PM_REBOOT_BOOT     /**< boot from reboot */
} pm_power_on_mode_t;


extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock) */


/**
  \brief Setup the microcontroller system.

   Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit(void);


/**
  \brief  Update SystemCoreClock variable.

   Updates the SystemCoreClock with current core Clock retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate(void);

pm_power_on_mode_t SystemPowerOnModeGet(void);

void SystemVectorTableRemapping(void);

void mpu_config(void);

void cache_enable(void);


#define IS_DCACHED_RAM(addr)  (((uint32_t) addr) >= (PSRAM_BASE))
#ifdef PSRAM_CACHE_WB
//TODO: replaced by HAL function, such that no need to implement it for every SoC
int mpu_dcache_clean(void *data, uint32_t size);
#else
#define mpu_dcache_clean(data,size) 0
#endif
//Include clean & invalidte
int mpu_dcache_invalidate(void *data, uint32_t size);
int mpu_icache_invalidate(void *data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_BF0_AP_H */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
