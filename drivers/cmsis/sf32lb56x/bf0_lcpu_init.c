/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtconfig.h>
#include <bf0_hal.h>
#include "bf0_lcpu_init.h"
#include <string.h>

#if defined (APP_BSP_TEST)
    #define bt_rf_cal()
#else
    #if defined(FPGA)
        #define bt_rf_cal() bt_rf_cal_9364()
    #endif
    extern void bt_rf_cal(void);
#endif


#if defined(LCPU_RUN_SEPERATE_IMG)
    #define lcpu_patch_install()
#else
    extern void lcpu_patch_install();
#endif

#if defined(SOC_BF0_HCPU)
extern void lcpu_patch_install();
extern void lcpu_patch_install_rev_b();

static uint8_t g_lcpu_rf_cal_disable;

__WEAK void adc_resume(void)
{
}

__WEAK void lcpu_rom_config(void)
{

}

#if defined(LCPU_RUN_ROM_ONLY)
#define lcpu_img_install()
#else
__WEAK void lcpu_img_install(void)
{
}
#endif

static void lcpu_ble_patch_install()
{
#if !defined(LCPU_RUN_SEPERATE_IMG)
    lcpu_patch_install();
#endif

    if (g_lcpu_rf_cal_disable == 0)
        bt_rf_cal();

    adc_resume();

// rf cal used em memory, to avoid wrongly init value bring wrongly result, just clear the section.
    memset((void *)0x20418000, 0, 0x5000);
    memset((void *)0x2041fc00, 0, 0x100);
}

void lcpu_disable_rf_cal(uint8_t is_disable)
{
    g_lcpu_rf_cal_disable = is_disable;
}



uint8_t lcpu_power_on(void)
{
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    HAL_RCC_Reset_and_Halt_LCPU(0);

    lcpu_rom_config();
    lcpu_img_install();

    HAL_LPAON_ConfigStartAddr((uint32_t *)HCPU_LCPU_CODE_START_ADDR);
    lcpu_ble_patch_install();
    HAL_RCC_ReleaseLCPU();
    HAL_Delay_us(5000);
    return 0;
}

uint8_t lcpu_power_off(void)
{
    HAL_RCC_Reset_and_Halt_LCPU(0);
    return 0;
}

#else
uint8_t lcpu_power_on(void)
{
    return 0;
}
#endif /* SOC_BF0_HCPU */


/** @} */
