/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtconfig.h>
#include <bf0_hal.h>
#include <string.h>


#if defined (APP_BSP_TEST)
    #define bt_rf_cal()
#else
    #if defined(FPGA)
        extern void bt_rf_cal_9364();
        #define bt_rf_cal() bt_rf_cal_9364()
    #else
        extern void bt_rf_cal(void);
    #endif
#endif


#ifdef LXT_DISABLE
    #define USE_LXT 0
#else
    #define USE_LXT 1
#endif


#if defined(SOC_BF0_HCPU)
extern void lcpu_patch_install();
extern void lcpu_patch_install_rev_b();

static uint8_t g_lcpu_rf_cal_disable;

__WEAK void adc_resume(void)
{
}

__WEAK void lcpu_img_install(void)
{
}

void lcpu_rom_config_default(void)
{
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    uint8_t is_enable_lxt = USE_LXT;
    uint8_t is_lcpu_rccal = 1 - USE_LXT;
    uint32_t wdt_staus = 0xFF;
    uint32_t wdt_time = 10;
    uint16_t wdt_clk = 32768;

    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_STATUS, &wdt_staus, 4);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_TIME, &wdt_time, 4);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_CLK_FEQ, &wdt_clk, 2);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_RC_CAL_IN_L, &is_lcpu_rccal, 1);


    hal_lcpu_bluetooth_rom_config_t config = {0};
    config.bit_valid |= 1 << 6;
    config.default_xtal_enabled = is_enable_lxt;
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_CONFIG, &config, sizeof(config));
}


__WEAK void lcpu_rom_config(void)
{
    lcpu_rom_config_default();
}



static void lcpu_ble_patch_install()
{

    lcpu_patch_install();


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

    /* LPSYS HCLK cannot exceed 24MHz when loading code */
    if (HAL_RCC_GetHCLKFreq(CORE_ID_LCPU) > 24000000)
    {
        /* restore to default value */
        HAL_RCC_LCPU_SetDiv(2, 1, 5);
        HAL_ASSERT(HAL_RCC_GetHCLKFreq(CORE_ID_LCPU) <= 24000000);
    }



    HAL_LPAON_ConfigStartAddr((uint32_t *)HCPU_LCPU_CODE_START_ADDR);
    lcpu_ble_patch_install();
    HAL_RCC_ReleaseLCPU();
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#ifdef USING_SEC_ENV
    // hcpu_exit_safe_mode()
    HAL_SECU_SetAttr(SECU_MOD_HCPU, SECU_ROLE_MASTER, SECU_FLAG_NONE);
    HAL_SECU_Apply(SECU_GROUP_HPMST);
#endif
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


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
