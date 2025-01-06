/**
  ******************************************************************************
  * @file   bf0_pm.h
  * @author Sifli software development team
  * @brief Sifli Power management API
 * @{
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

#ifndef BF0_PM_H
#define BF0_PM_H
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "sf_type.h"
#ifdef BSP_USING_PM
    #include "rtdevice.h"
#endif /* BSP_USING_PM */

/**
 ****************************************************************************************
* @addtogroup pm Power Management
* @ingroup middleware
* @brief Power Management API
* @{
****************************************************************************************
*/

typedef struct
{
    uint32_t init_enter;
    uint32_t lcpu_wakeup;
    uint32_t preinit_begin;
    uint32_t acr_before;
    uint32_t acr_after;
    uint32_t xtal48_rdy;
    uint32_t dll_rdy;
    uint32_t pa_init_done;
    uint32_t pin_init_done;
    uint32_t power_rdy_exp;
    uint32_t power_rdy;
    uint32_t msp_init_done;
    uint32_t psram_rdy;
    uint32_t hal_init_done;
    uint32_t mpu_config_done;
    uint32_t ram_code_load_done;
    uint32_t restore_static_data_begin;
    uint32_t restore_static_data_done;
    uint32_t restore_ram_done;
    uint32_t restore_done;
    uint32_t device_resume_begin;
    uint32_t device_resume_done;

}  test_pm_data_restore_time_t;

typedef struct
{
    uint32_t device_suspend_begin;
    uint32_t device_suspend_done;
    uint32_t save_start;
    uint32_t save_static_begin;
    uint32_t save_static_done;
    uint32_t save_done;

} test_pm_data_save_time_t;

typedef struct
{
    test_pm_data_save_time_t save_time;
    test_pm_data_restore_time_t restore_time;
} test_pm_data_t;

//extern test_pm_data_t test_pm_data;

typedef enum
{
    PM_SCENARIO_UI,
    PM_SCENARIO_AUDIO,
    PM_SCENARIO_RFTEST,
} pm_scenario_name_t;

#ifdef PM_DEBUG_PIN_ENABLED

#ifdef SOC_BF0_HCPU
#define PM_DEBUG_PIN_HIGH()      ((GPIO1_TypeDef *)hwp_gpio1)->DOSR0 |= (1UL << 24)
#define PM_DEBUG_PIN_LOW()       ((GPIO1_TypeDef *)hwp_gpio1)->DOCR0 |= (1UL << 24)
#define PM_DEBUG_PIN_TOGGLE()    ((GPIO1_TypeDef *)hwp_gpio1)->DOR0  ^= (1UL << 24)
#define PM_DEBUG_PIN_ENABLE()    ((GPIO1_TypeDef *)hwp_gpio1)->DOESR0 |= (1UL << 24)
#define PM_DEBUG_PIN_INIT()                         \
    do                                              \
    {                                               \
        GPIO_InitTypeDef GPIO_InitStruct;           \
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;    \
        GPIO_InitStruct.Pin = 24;                   \
        GPIO_InitStruct.Pull = GPIO_NOPULL;         \
        HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct); \
    }                                               \
    while (0)
#else
#if 1
#define PM_DEBUG_PIN_HIGH()
#define PM_DEBUG_PIN_LOW()
#define PM_DEBUG_PIN_TOGGLE()
#define PM_DEBUG_PIN_ENABLE()
#define PM_DEBUG_PIN_INIT()
#else
#define PM_DEBUG_PIN_HIGH()      ((GPIO1_TypeDef *)hwp_gpio2)->DOSR1 |= (1UL << 1)
#define PM_DEBUG_PIN_LOW()       ((GPIO1_TypeDef *)hwp_gpio2)->DOCR1 |= (1UL << 1)
#define PM_DEBUG_PIN_TOGGLE()    ((GPIO1_TypeDef *)hwp_gpio2)->DOR1  ^= (1UL << 1)
#define PM_DEBUG_PIN_ENABLE()    ((GPIO1_TypeDef *)hwp_gpio2)->DOESR1 |= (1UL << 1)
#define PM_DEBUG_PIN_INIT()                         \
    do                                              \
    {                                               \
        GPIO_InitTypeDef GPIO_InitStruct;           \
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;    \
        GPIO_InitStruct.Pin = 33;                   \
        GPIO_InitStruct.Pull = GPIO_NOPULL;         \
        HAL_GPIO_Init(hwp_gpio2, &GPIO_InitStruct); \
    }                                               \
    while (0)
#endif
#endif


#define PM_DEBUG_PIN2_HIGH()      ((GPIO1_TypeDef *)hwp_gpio2)->DOSR1 |= (1UL << 2)
#define PM_DEBUG_PIN2_LOW()       ((GPIO1_TypeDef *)hwp_gpio2)->DOCR1 |= (1UL << 2)
#define PM_DEBUG_PIN2_TOGGLE()    ((GPIO1_TypeDef *)hwp_gpio2)->DOR1  ^= (1UL << 2)

#define PM_DEBUG_PIN2_INIT()                        \
    do                                              \
    {                                               \
        GPIO_InitTypeDef GPIO_InitStruct;           \
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;    \
        GPIO_InitStruct.Pin = 34;                   \
        GPIO_InitStruct.Pull = GPIO_NOPULL;         \
        HAL_GPIO_Init(hwp_gpio2, &GPIO_InitStruct); \
    }                                               \
    while (0)


#else
#define PM_DEBUG_PIN_HIGH()
#define PM_DEBUG_PIN_LOW()
#define PM_DEBUG_PIN_TOGGLE()
#define PM_DEBUG_PIN_ENABLE()
#define PM_DEBUG_PIN_INIT()

#endif /* PM_DEBUG_PIN_ENABLED */


/** non-retention section name */
#define PM_NON_RETENTION_SECTION_NAME non_ret

/* non-retention bss section definition macro */
#if defined(__CC_ARM)
    #define PM_NON_RETENTION_SECTION_BEGIN    _Pragma(STRINGIFY(arm section zidata=STRINGIFY(PM_NON_RETENTION_SECTION_NAME)))
    #define PM_NON_RETENTION_SECTION_END      _Pragma(STRINGIFY(arm section zidata))

#elif defined(__CLANG_ARM)
    #define PM_NON_RETENTION_SECTION_BEGIN    _Pragma(STRINGIFY(clang section bss=STRINGIFY(PM_NON_RETENTION_SECTION_NAME)))
    #define PM_NON_RETENTION_SECTION_END      _Pragma(STRINGIFY(clang section bss=""))
#elif defined(__GNUC__)
    #define PM_NON_RETENTION_SECTION_BEGIN    _Pragma(STRINGIFY(clang section bss=STRINGIFY(PM_NON_RETENTION_SECTION_NAME)))
    #define PM_NON_RETENTION_SECTION_END      _Pragma(STRINGIFY(clang section bss=""))
#endif

/// Sleep ticks
extern rt_uint32_t g_sleep_tick;

#ifdef SOC_BF0_HCPU
    extern void pm_shutdown(void);
    #ifdef BSP_USING_CHARGER
        /* for charger int wakeup*/
        extern int pm_get_charger_pin_wakeup(void);
    #endif
#endif

/**
 *  @brief AON IRQHandler hook
 *
 *  It's defined as weak function in pm module and called by AON_IRQHandler.
 *  User could reimplement it and perform customized action.
 *
 *  @param[in] wsr wakeup source
 *  @retval void
 */
void aon_irq_handler_hook(uint32_t wsr);

/**
 * @brief get last wakeup source
 *
 * @retval wakeup source,
 *         for HPSYS, return HPSYS_AON_WSR_RTC and other value if SystemPowerOnModeGet() is PM_COLD_BOOT or PM_COLD_STANDBY
                      return PMUC_WSR_RTC and other value if SystemPowerOnModeGet() is PM_HIBERNATE_BOOT or PM_SHUTDOWN_BOOT
           for LPSYS, return LPSYS_AON_WSR_RTC and other value if SystemPowerOnModeGet() is PM_COLD_BOOT or PM_COLD_STANDBY
 */
uint32_t pm_get_wakeup_src(void);

/**
 * @brief get last low power mode
 *
 * @retval low power mode
 */
uint32_t pm_get_power_mode(void);


#ifdef BSP_USING_PM
    /**
    * @brief Enable pin wakeup
    *
    * @param pin pin number, range: 0~5, 0 means H/LPAON_WAKEUP_SRC_PIN0, etc.
    * @param mode pin wakeup mode
    * @retval status
    */
    rt_err_t pm_enable_pin_wakeup(uint8_t pin, AON_PinModeTypeDef mode);
#endif

/**
 * @brief Disable pin wakeup
 *
 * @param pin pin number, range: 0~5
 * @retval status
 */
rt_err_t pm_disable_pin_wakeup(uint8_t pin);

/**
 * @brief Enable RTC wakeup
 *
 * @retval status
 */
rt_err_t pm_enable_rtc_wakeup(void);

/**
 * @brief Disable RTC wakeup
 *
 * @retval status
 */
rt_err_t pm_disable_rtc_wakeup(void);

/**
 * @brief Calculate the compensated rt_tick
 *
 *  @param[in] curr_tick current rt_tick
 *  @param[in] curr_time current GTIME
 *  @param[in] gtime_freq GTIMER frequency
 *  @param[in] user_data user data
 *
 * @retval compensated rt_tick
 */
rt_tick_t pm_latch_tick(rt_tick_t curr_tick, uint32_t curr_time, float gtime_freq, void *user_data);


/**
 * @brief Update last latch tick if rt_tick cannot be updated as expected
 *
 *  @param[in] last_latch_tick last_latch_tick
 *
 * @return void
 */
void pm_set_last_latch_tick(rt_tick_t last_latch_tick);

/** Indicate specified scanario is started
 *
 * @param[in] scenario scenario
 */
rt_err_t pm_scenario_start(pm_scenario_name_t scenario);


/** Indicate specified scanario is stopped
 *
 * @param[in] scenario scenario
 */
rt_err_t pm_scenario_stop(pm_scenario_name_t scenario);

#ifdef BSP_USING_PM
    /**
    * @brief weak pm_run function for frequency change
    *
    *  @param[in] pm   pm
    *  @param[in] mode run mode, e.g. PM_RUN_MODE_HIGH_SPEED
    *
    * @return void
    */
    void sifli_pm_run(struct rt_pm *pm, uint8_t mode);
#endif /* BSP_USING_PM */

/**
 * @brief Restore pin hardware state
 *
 * @return void
 */
void pm_pin_restore(void);

/**
 * @brief Backup pin hardware state
 *
 * @return void
 */
void pm_pin_backup(void);

///@} pm
#endif

/// @} file


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
