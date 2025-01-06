/**
  ******************************************************************************
  * @file   bf0_hal_lcpu_config.c
  * @author Sifli software development team
  * @brief Set configuration parameters to LCPU
  * @{
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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


#include "bf0_hal.h"
#include "string.h"

#if (!defined(SF32LB52X) && (!defined(SF32LB52X_58)))
typedef struct
{
    uint32_t magic_num;
#ifndef SF32LB55X
    uint32_t adc_cal[2];
#else
    uint32_t adc_cal;
#endif
    uint32_t sdadc_cal[2];
    uint8_t sn[HAL_LCPU_CONFIG_SN_MAX_NUM];
    uint16_t sn_len;
    uint8_t chip_rev;
    uint8_t reserved;
    uint32_t battery_a;
    uint32_t battery_b;
} HAL_LCPU_CONFIG_T;
#else
// Should not more than 64byte
typedef struct
{
    uint32_t magic_num;
    uint32_t lpcycle_curr;
    uint32_t lpcycle_ave;
    uint32_t wdt_time;
    uint32_t wdt_status;
    uint32_t bt_txpwr;
    uint16_t wdt_clk;
    uint8_t is_xtal_enable;
    uint8_t is_rccal_in_L;
    uint32_t is_soft_cvsd;//u32 for magic
    hal_lcpu_bluetooth_em_config_t em_buf;
    hal_lcpu_bluetooth_act_configt_t bt_act_config;
    hal_lcpu_ble_mem_config_t ke_mem_config;
    hal_lcpu_bluetooth_rom_config_t bt_rom_config;
    uint32_t sec_addr;
    uint32_t hcpu_ipc_addr;
} HAL_LCPU_CONFIG_T;
#endif


#if defined(SF32LB55X)
    #define HAL_LCPU_CONFIG_START_ADDR 0x20137000
    #define HAL_LPCU_CONFIG_MAGIC_NUM 0x45457878

    //HAL_LCPU_ASSERT_INFO_ADDR  4byte indicate lcpu assert flow over
    #define HAL_LCPU_ASSERT_INFO_ADDR  0x20137200
    #define HAL_LPCU_ASSERT_OVER_FLAG  0xa5a5a5a5
#elif defined(SF32LB58X)
    #ifdef SF32LB58X_3SCO
        #define HAL_LCPU_CONFIG_START_ADDR 0x2047BE00
    #elif defined(SF32LB52X_58)
        #define HAL_LCPU_CONFIG_START_ADDR 0x204f8000
    #else
        #define HAL_LCPU_CONFIG_START_ADDR 0x204FE000
    #endif
    #define HAL_LPCU_CONFIG_MAGIC_NUM 0x45457878

    //HAL_LCPU_ASSERT_INFO_ADDR  4byte indicate lcpu assert flow over
    #define HAL_LCPU_ASSERT_INFO_ADDR  0x204FE800
    #define HAL_LPCU_ASSERT_OVER_FLAG  0xa5a5a5a5
#elif defined(SF32LB56X)
    #define HAL_LCPU_CONFIG_START_ADDR 0x2041FF00
    #define HAL_LPCU_CONFIG_MAGIC_NUM 0x45457878

    #define HAL_LCPU_ASSERT_INFO_ADDR  0x2041FFF0
    #define HAL_LPCU_ASSERT_OVER_FLAG  0xa5a5a5a5
#elif defined(SF32LB52X)
    #define HAL_LCPU_CONFIG_START_ADDR 0x2040FDC0
    #define HAL_LPCU_CONFIG_MAGIC_NUM 0x45457878

    #define HAL_LCPU_ASSERT_INFO_ADDR  0x2040FDBC
    #define HAL_LPCU_ASSERT_OVER_FLAG  0xa5a5a5a5
#else
    #error "Not defined"
#endif

__HAL_ROM_USED void HAL_LCPU_ASSERT_INFO_clear(void)
{
    volatile uint32_t *p = (volatile uint32_t *)HAL_LCPU_ASSERT_INFO_ADDR;

    *p = 0;
}

__HAL_ROM_USED void HAL_LCPU_ASSERT_INFO_set(void)
{
    volatile uint32_t *p = (volatile uint32_t *)HAL_LCPU_ASSERT_INFO_ADDR;

    *p = HAL_LPCU_ASSERT_OVER_FLAG;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCPU_ASSERT_INFO_get(void)
{
    volatile uint32_t *p = (volatile uint32_t *)HAL_LCPU_ASSERT_INFO_ADDR;

    if (*p == HAL_LPCU_ASSERT_OVER_FLAG)
    {
        return HAL_OK;
    }

    return HAL_BUSY;
}

static volatile HAL_LCPU_CONFIG_T *g_lcpu_config_context;

static void HAL_LCPU_CONIFG_init(void)
{
    if (g_lcpu_config_context)
        return;


#ifdef SF32LB52X
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id >= HAL_CHIP_REV_ID_A4)
    {
        g_lcpu_config_context = (volatile HAL_LCPU_CONFIG_T *)LCPU2HCPU_MB_CH2_BUF_REV_B_START_ADDR;
    }
    else
#endif
    {
        g_lcpu_config_context = (volatile HAL_LCPU_CONFIG_T *)HAL_LCPU_CONFIG_START_ADDR;
    }


#ifdef SOC_BF0_HCPU
#ifdef SF32LB52X
    if (rev_id >= HAL_CHIP_REV_ID_A4)
    {
        memset((void *)g_lcpu_config_context, 0, sizeof(HAL_LCPU_CONFIG_T));
    }
    else
    {
        memset((void *)g_lcpu_config_context, 0, 0x40);
    }
#else
    memset((void *)g_lcpu_config_context, 0, sizeof(HAL_LCPU_CONFIG_T));
#endif
    g_lcpu_config_context->magic_num = HAL_LPCU_CONFIG_MAGIC_NUM;
#endif
}

void HAL_LCPU_CONFIG_InitContext(void)
{
    g_lcpu_config_context = (volatile HAL_LCPU_CONFIG_T *)HAL_LCPU_CONFIG_START_ADDR;

#ifdef SOC_BF0_HCPU
    memset((void *)g_lcpu_config_context, 0, sizeof(HAL_LCPU_CONFIG_T));
    g_lcpu_config_context->magic_num = HAL_LPCU_CONFIG_MAGIC_NUM;
#endif /* SOC_BF0_HCPU */
}


#if (!defined(SF32LB52X)) && (!defined(SF32LB52X_58))
__HAL_ROM_USED HAL_StatusTypeDef HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_TYPE_T type, void *value, uint16_t length)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
#ifdef SOC_BF0_HCPU
    if (type >= HAL_LCPU_CONFIG_MAX || !value)
        return ret;

    HAL_LCPU_CONIFG_init();

    switch (type)
    {
    case HAL_LCPU_CONFIG_ADC_CALIBRATION:
    {
#ifndef SF32LB55X
        if (length == 8) // 64 bit
        {
            memcpy((void *)&g_lcpu_config_context->adc_cal[0], value, length);
            ret = HAL_OK;
        }
#else
        if (length == 4) // 32 bit
        {
            memcpy((void *)&g_lcpu_config_context->adc_cal, value, length);
            ret = HAL_OK;
        }
#endif
        break;
    }
    case HAL_LCPU_CONFIG_SDADC_CALIBRATION:
    {
        if (length == 8) // 64 bit
        {
            memcpy((void *)&g_lcpu_config_context->sdadc_cal, value, 8);
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SN:
    {
        if (length <= 256)
        {
            memcpy((void *)g_lcpu_config_context->sn, value, length);
            g_lcpu_config_context->sn_len = length;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_CHIP_REV:
    {
        if (length == 1)
        {
            g_lcpu_config_context->chip_rev = *(uint8_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BATTERY_CALIBRATION:
    {
        if (length == 8)
        {
            uint32_t *p = (uint32_t *)value;
            g_lcpu_config_context->battery_a = p[0];
            g_lcpu_config_context->battery_b = p[1];
            ret = HAL_OK;
        }
        else if (length == 12) //maybe 12 bytes
        {
            uint32_t *p = (uint32_t *)value;
            g_lcpu_config_context->battery_a = p[1];
            g_lcpu_config_context->battery_b = p[2];
            ret = HAL_OK;
        }
        break;
    }


    default:
        break;
    }
#endif // No need set in LCPU currently

    return ret;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_TYPE_T type, uint8_t *value, uint16_t *length)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (type >= HAL_LCPU_CONFIG_MAX || !value || !length)
        return ret;

    HAL_LCPU_CONIFG_init();

    // No need to handle if HCPU didn't set any value.
    if (g_lcpu_config_context->magic_num != HAL_LPCU_CONFIG_MAGIC_NUM)
        return ret;

    switch (type)
    {
    case HAL_LCPU_CONFIG_ADC_CALIBRATION:
    {
#ifndef SF32LB55X
        if (*length == 8) // 64 bit
        {
            memcpy((void *)value, (void *)&g_lcpu_config_context->adc_cal[0], 8);
            ret = HAL_OK;
        }
#else
        if (*length == 4) // 32 bit
        {
            memcpy((void *)value, (void *)&g_lcpu_config_context->adc_cal, 4);
            ret = HAL_OK;
        }
#endif
        break;
    }
    case HAL_LCPU_CONFIG_SDADC_CALIBRATION:
    {
        if (*length == 8) // 64 bit
        {
            memcpy((void *)value, (void *)&g_lcpu_config_context->sdadc_cal, 8);
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SN:
    {
        if (*length >= g_lcpu_config_context->sn_len)
        {
            memcpy(value, (uint8_t *)g_lcpu_config_context->sn, g_lcpu_config_context->sn_len);
            *length = g_lcpu_config_context->sn_len;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_CHIP_REV:
    {
        if (*length == 1)
        {
            *(uint8_t *)value = g_lcpu_config_context->chip_rev;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BATTERY_CALIBRATION:
    {
        if (*length == 8)
        {
            uint32_t *p = (uint32_t *)value;
            p[0] = g_lcpu_config_context->battery_a;
            p[1] = g_lcpu_config_context->battery_b;
            ret = HAL_OK;
        }
        else if (*length == 12)
        {
            uint32_t *p = (uint32_t *)value;
            p[0] = 0xe8091ad7;
            p[1] = g_lcpu_config_context->battery_a;
            p[2] = g_lcpu_config_context->battery_b;
            ret = HAL_OK;
        }
        break;
    }
    default:
        break;
    }
    return ret;

}
#else

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_TYPE_T type, void *value, uint16_t length)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    if (type >= HAL_LCPU_CONFIG_MAX || !value)
        return ret;

#ifdef SF32LB52X
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id < HAL_CHIP_REV_ID_A4)
    {
        if (type > HAL_LCPU_CONFIG_SOFT_CVSD)
            return ret;
    }
#endif

#if (defined(SOC_BF0_HCPU)) && (!defined(SF32LB52X_58))
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif
    HAL_LCPU_CONIFG_init();

    switch (type)
    {
    case HAL_LCPU_CONFIG_XTAL_ENABLED:
    {
        if (length == 1)
        {
            g_lcpu_config_context->is_xtal_enable = *(uint8_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_WDT_TIME:
    {
        if (length == 4)
        {
            g_lcpu_config_context->wdt_time = *(uint32_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_LPCYCLE_AVE:
    {
        if (length == 4)
        {
            g_lcpu_config_context->lpcycle_ave = *(uint32_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_LPCYCLE_CURR:
    {
        if (length == 4)
        {
            g_lcpu_config_context->lpcycle_curr = *(uint32_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_WDT_STATUS:
    {
        if (length == 4)
        {
            g_lcpu_config_context->wdt_status = *(uint32_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_WDT_CLK_FEQ:
    {
        if (length == 2)
        {
            g_lcpu_config_context->wdt_clk = *(uint16_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_TX_PWR:
    {
        if (length == 4)
        {
            g_lcpu_config_context->bt_txpwr = *(uint32_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_RC_CAL_IN_L:
    {
        if (length == 1)
        {
            g_lcpu_config_context->is_rccal_in_L = *(uint8_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SOFT_CVSD:
    {
        if (length == 4)
        {
            g_lcpu_config_context->is_soft_cvsd = *(uint32_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_EM_BUF:
    {
        if (length == sizeof(hal_lcpu_bluetooth_em_config_t))
        {
            memcpy((void *)&g_lcpu_config_context->em_buf, value, sizeof(hal_lcpu_bluetooth_em_config_t));
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_ACT_CFG:
    {
        if (length == sizeof(hal_lcpu_bluetooth_act_configt_t))
        {
            memcpy((void *)&g_lcpu_config_context->bt_act_config, value, sizeof(hal_lcpu_bluetooth_act_configt_t));
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_KE_BUF:
    {
        if (length == sizeof(hal_lcpu_ble_mem_config_t))
        {
            memcpy((void *)&g_lcpu_config_context->ke_mem_config, value, sizeof(hal_lcpu_ble_mem_config_t));
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_CONFIG:
    {
        if (length == sizeof(hal_lcpu_bluetooth_rom_config_t))
        {
            memcpy((void *)&g_lcpu_config_context->bt_rom_config, value, sizeof(hal_lcpu_bluetooth_rom_config_t));
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SEC_ADDR:
    {
        if (length == 4)
        {
            g_lcpu_config_context->sec_addr = *(uint32_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_HCPU_TX_QUEUE:
    {
        if (length == 4)
        {
            g_lcpu_config_context->hcpu_ipc_addr = *(uint32_t *)value;
            ret = HAL_OK;
        }
        break;
    }
    default:
        break;
    }
#if (defined(SOC_BF0_HCPU)) && (!defined(SF32LB52X_58))
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
    return ret;


}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_TYPE_T type, uint8_t *value, uint16_t *length)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (type >= HAL_LCPU_CONFIG_MAX || !value || !length)
        return ret;
#if (defined(SOC_BF0_HCPU)) && (!defined(SF32LB52X_58))
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif
    HAL_LCPU_CONIFG_init();

    // No need to handle if HCPU didn't set any value.
    if (g_lcpu_config_context->magic_num != HAL_LPCU_CONFIG_MAGIC_NUM)
    {
#if (defined(SOC_BF0_HCPU)) && (!defined(SF32LB52X_58))
        HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
        return ret;
    }

    switch (type)
    {
    case HAL_LCPU_CONFIG_XTAL_ENABLED:
    {
        if (*length == 1)
        {
            *(uint8_t *)value = g_lcpu_config_context->is_xtal_enable;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_WDT_TIME:
    {
        if (*length == 4)
        {
            *(uint32_t *)value = g_lcpu_config_context->wdt_time;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_LPCYCLE_AVE:
    {
        if (*length == 4)
        {
            *(uint32_t *)value = g_lcpu_config_context->lpcycle_ave;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_LPCYCLE_CURR:
    {
        if (*length == 4)
        {
            *(uint32_t *)value = g_lcpu_config_context->lpcycle_curr;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_WDT_STATUS:
    {
        if (*length == 4)
        {
            *(uint32_t *)value = g_lcpu_config_context->wdt_status;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_WDT_CLK_FEQ:
    {
        if (*length == 2)
        {
            *(uint16_t *)value = g_lcpu_config_context->wdt_clk;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_TX_PWR:
    {
        if (*length == 4)
        {
            *(uint32_t *)value = g_lcpu_config_context->bt_txpwr;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_RC_CAL_IN_L:
    {
        if (*length == 1)
        {
            *(uint8_t *)value = g_lcpu_config_context->is_rccal_in_L;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SOFT_CVSD:
    {
        if (*length == 4)
        {
            *(uint32_t *)value = g_lcpu_config_context->is_soft_cvsd;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_EM_BUF:
    {
        if (*length == sizeof(hal_lcpu_bluetooth_em_config_t))
        {
            memcpy(value, (void *)&g_lcpu_config_context->em_buf, sizeof(hal_lcpu_bluetooth_em_config_t));
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_ACT_CFG:
    {
        if (*length == sizeof(hal_lcpu_bluetooth_act_configt_t))
        {
            memcpy(value, (void *)&g_lcpu_config_context->bt_act_config, sizeof(hal_lcpu_bluetooth_act_configt_t));
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_KE_BUF:
    {
        if (*length == sizeof(hal_lcpu_ble_mem_config_t))
        {
            memcpy(value, (void *)&g_lcpu_config_context->ke_mem_config, sizeof(hal_lcpu_ble_mem_config_t));
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BT_CONFIG:
    {
        if (*length == sizeof(hal_lcpu_bluetooth_rom_config_t))
        {
            memcpy(value, (void *)&g_lcpu_config_context->bt_rom_config, sizeof(hal_lcpu_bluetooth_rom_config_t));
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SEC_ADDR:
    {
        if (*length == 4)
        {
            *(uint32_t *)value = g_lcpu_config_context->sec_addr;
            ret = HAL_OK;
        }
        break;
    }
    case HAL_LCPU_CONFIG_HCPU_TX_QUEUE:
    {
        if (*length == 4)
        {
            *(uint32_t *)value = g_lcpu_config_context->hcpu_ipc_addr;
            ret = HAL_OK;
        }
        break;
    }
    default:
        break;
    }
#if (defined(SOC_BF0_HCPU)) && (!defined(SF32LB52X_58))
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
    return ret;

}

#endif


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

