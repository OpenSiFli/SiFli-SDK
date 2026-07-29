/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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

//void mpu_config(void);

void cache_enable(void);

void cache_disable(void);


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