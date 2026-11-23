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

#if defined(SOC_BF0_HCPU)
extern void lcpu_patch_install();
extern void lcpu_patch_install_rev_b();

static uint8_t g_lcpu_rf_cal_disable;

__WEAK void adc_resume(void)
{
}

#if (!defined(SF32LB52X_REV_B)) && !defined(LCPU_RUN_ROM_ONLY)
__WEAK void lcpu_img_install(void)
{
}
#else
#define lcpu_img_install()
#endif

void lcpu_rom_config_default(void)
{
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    uint8_t is_enable_lxt;
    uint8_t is_lcpu_rccal = 0;
    uint32_t wdt_staus = 0xFF;
    uint32_t wdt_time = 10;
    uint16_t wdt_clk = 32768;

    if (HAL_LXT_DISABLED())
    {
        is_lcpu_rccal = 1;
    }
    is_enable_lxt = 1 - is_lcpu_rccal;
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_XTAL_ENABLED, &is_enable_lxt, 1);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_STATUS, &wdt_staus, 4);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_TIME, &wdt_time, 4);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_WDT_CLK_FEQ, &wdt_clk, 2);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_RC_CAL_IN_L, &is_lcpu_rccal, 1);

#if defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)
    if (rev_id >= HAL_CHIP_REV_ID_A4)
    {
        uint32_t tx_queue = HCPU2LCPU_MB_CH1_BUF_START_ADDR;
        hal_lcpu_bluetooth_rom_config_t config = {0};

        config.bit_valid |= 1 << 10 | 1 << 6 | 1 << 2;
        config.lld_prog_delay = 3;
        config.is_fpga = 0;
        config.default_xtal_enabled = is_enable_lxt;
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_HCPU_TX_QUEUE, &tx_queue, 4);
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_CONFIG, &config, sizeof(config));

        hal_lcpu_ble_mem_config_t ble_config = {0};
        ble_config.max_nb_of_hci_completed = 6;
        ble_config.bit_valid = 1 << 6;
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_KE_BUF, &ble_config, sizeof(ble_config));
    }
#endif // defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)
}

__WEAK void lcpu_rom_config(void)
{
    lcpu_rom_config_default();
}

static void lcpu_ble_patch_install()
{
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id < HAL_CHIP_REV_ID_A4)
    {
#if !defined(SF32LB52X_REV_B)
        lcpu_patch_install();
#else // SF32LB52X_REV_B
        HAL_ASSERT(0 && "Wrongly config");
#endif // !SF32LB52X_REV_B

    }
    else
    {
#if !defined(SF32LB52X_REV_A)
        memset((void *)0x20400000, 0, 0x500);
        lcpu_patch_install_rev_b();
#else // SF32LB52X_REV_A
        HAL_ASSERT(0 && "Wrongly config");
#endif // !SF32LB52X_REV_A

    }

    if (g_lcpu_rf_cal_disable == 0)
        bt_rf_cal();

    adc_resume();

// rf cal used em memory, to avoid wrongly init value bring wrongly result, just clear the section.
    memset((void *)0x20408000, 0, 0x5000);
}

void lcpu_disable_rf_cal(uint8_t is_disable)
{
    g_lcpu_rf_cal_disable = is_disable;
}

__WEAK __NOINLINE void lcpu_nvds_config()
{
}

uint8_t lcpu_power_on(void)
{
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    HAL_RCC_Reset_and_Halt_LCPU(0);

    lcpu_nvds_config();
    lcpu_rom_config();

    /* LPSYS HCLK cannot exceed 24MHz when loading code */
    if (HAL_RCC_GetHCLKFreq(CORE_ID_LCPU) > 24000000)
    {
        /* restore to default value */
        HAL_RCC_LCPU_SetDiv(2, 1, 5);
        HAL_ASSERT(HAL_RCC_GetHCLKFreq(CORE_ID_LCPU) <= 24000000);
    }

    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id < HAL_CHIP_REV_ID_A4)
    {
        lcpu_img_install();
    }

    HAL_LPAON_ConfigStartAddr((uint32_t *)HCPU_LCPU_CODE_START_ADDR);
    lcpu_ble_patch_install();
    HAL_RCC_ReleaseLCPU();
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#ifdef USING_SEC_ENV
    // hcpu_exit_safe_mode()
    HAL_SECU_SetAttr(SECU_MOD_HCPU, SECU_ROLE_MASTER, SECU_FLAG_NONE);
#endif
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

