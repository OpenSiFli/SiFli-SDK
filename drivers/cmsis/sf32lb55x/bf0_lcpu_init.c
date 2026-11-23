/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtconfig.h>
#include <bf0_hal.h>
#include "bf0_lcpu_init.h"
#include <string.h>

#if defined(FPGA)
    #define bt_rf_cal()
#else
    extern void bt_rf_cal(void);
#endif



#if defined(SOC_BF0_HCPU)
extern void lcpu_patch_install();
extern void lcpu_patch_install_rev_b();

static uint8_t g_lcpu_rf_cal_disable;

__WEAK void adc_resume(void)
{
}

#if defined(LCPU_RUN_ROM_ONLY)
#define lcpu_img_install()
#else
__WEAK void lcpu_img_install(void)
{
}
#endif

__WEAK void lcpu_rom_config(void)
{
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_CHIP_REV, &rev_id, 1);
}

static void lcpu_ble_patch_install()
{
    memset((void *)0x20134000, 0, 0x2000);
#if defined(BSP_CHIP_ID_COMPATIBLE)
    extern void lcpu_patch_install_a3();

    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id == HAL_CHIP_REV_ID_A3)
        lcpu_patch_install_a3();
    else
        lcpu_patch_install();
#else
    lcpu_patch_install();
#endif


    if (g_lcpu_rf_cal_disable == 0)
        bt_rf_cal();

    adc_resume();
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
