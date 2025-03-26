/**
  ******************************************************************************
  * @file   bf0_lcpu_init.c
  * @author Sifli software development team
  * @brief
 * @{
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2025,  Sifli Technology
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

#include <rtconfig.h>
#include <bf0_hal.h>
#include <string.h>


#if defined (APP_BSP_TEST)
    #define bt_rf_cal()
#else
    #if defined(FPGA)
        #define bt_rf_cal() bt_rf_cal_9364()
    #endif
    extern void bt_rf_cal(void);
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

#if (!defined(SF32LB52X_REV_B)) && !defined(LCPU_RUN_ROM_ONLY)
__WEAK void lcpu_img_install(void)
{
}
#else
#define lcpu_img_install()
#endif

__WEAK void lcpu_rom_config(void)
{

    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    uint8_t is_enable_lxt = USE_LXT;
    uint8_t is_lcpu_rccal = 1 - USE_LXT;
    uint32_t wdt_staus = 0xFF;
    uint32_t wdt_time = 10;
    uint16_t wdt_clk = 32768;

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
        config.bit_valid |= 1 << 10 | 1 << 6;
        config.is_fpga = 0;
        config.default_xtal_enabled = is_enable_lxt;
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_HCPU_TX_QUEUE, &tx_queue, 4);
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_CONFIG, &config, sizeof(config));
    }
#endif // defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)
}



static void lcpu_ble_patch_install()
{
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id < HAL_CHIP_REV_ID_A4)
        lcpu_patch_install();
    else
    {
        memset((void *)0x20400000, 0, 0x500);
        lcpu_patch_install_rev_b();
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
