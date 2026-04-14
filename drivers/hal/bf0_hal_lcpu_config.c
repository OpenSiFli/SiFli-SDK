/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include "string.h"
#include "lcpu_config_type.h"

typedef struct
{
    uint32_t magic_num;
} HAL_LCPU_CONFIG_T;


#define HAL_LCPU_CONFIG_START_ADDR LCPU_CONFIG_START_ADDR
#define HAL_LPCU_CONFIG_MAGIC_NUM LPCU_CONFIG_MAGIC_NUM

//HAL_LCPU_ASSERT_INFO_ADDR  4byte indicate lcpu assert flow over
#define HAL_LCPU_ASSERT_INFO_ADDR  LCPU_ASSERT_INFO_ADDR
#define HAL_LPCU_ASSERT_OVER_FLAG  LPCU_ASSERT_OVER_FLAG


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
    uint16_t size = LCPU_CONFIG_get_total_size();
    memset((void *)g_lcpu_config_context, 0, size);
    g_lcpu_config_context->magic_num = HAL_LPCU_CONFIG_MAGIC_NUM;
#endif //SOC_BF0_HCPU
}

void HAL_LCPU_CONFIG_InitContext(void)
{
    g_lcpu_config_context = (volatile HAL_LCPU_CONFIG_T *)HAL_LCPU_CONFIG_START_ADDR;

#ifdef SOC_BF0_HCPU
    uint16_t size = LCPU_CONFIG_get_total_size();
    memset((void *)g_lcpu_config_context, 0, size);
    g_lcpu_config_context->magic_num = HAL_LPCU_CONFIG_MAGIC_NUM;
#endif /* SOC_BF0_HCPU */
}

HAL_StatusTypeDef HAL_LCPU_CONFIG_get_type(HAL_LCPU_CONFIG_TYPE_T type, uint8_t *config_type)
{
    HAL_StatusTypeDef ret = HAL_OK;

// LCPU_CONFIG_AUTO will convert to LCPU_CONFIG_V1 or LCPU_CONFIG_V2
#if defined(LCPU_CONFIG_V1)
    switch (type)
    {
    case HAL_LCPU_CONFIG_ADC_CALIBRATION:
        *config_type = HAL_LCPU_CONFIG_ADC_CALIBRATION_ROM;
        break;
    case HAL_LCPU_CONFIG_SDADC_CALIBRATION:
        *config_type = HAL_LCPU_CONFIG_SDADC_CALIBRATION_ROM;
        break;
    case HAL_LCPU_CONFIG_SN:
        *config_type = HAL_LCPU_CONFIG_SN_ROM;
        break;
    case HAL_LCPU_CONFIG_CHIP_REV:
        *config_type = HAL_LCPU_CONFIG_CHIP_REV_ROM;
        break;
    case HAL_LCPU_CONFIG_BATTERY_CALIBRATION:
        *config_type = HAL_LCPU_CONFIG_BATTERY_CALIBRATION_ROM;
        break;
#ifdef SF32LB56X
    case HAL_LCPU_CONFIG_SOFT_CVSD:
        *config_type = HAL_LCPU_CONFIG_IS_SOFT_CVSD_ROM;
        break;
#endif
    default:
        *config_type  = HAL_LCPU_CONFIG_MAX;
    }
#elif defined(LCPU_CONFIG_V2)
    *config_type = type;
#else // no version defined
// If no version defeind, just treat as LCPU_CONFIG_AUTO
#if defined(SF32LB55X) || defined(SF32LB56X) || defined(SF32LB58X)
    switch (type)
    {
    case HAL_LCPU_CONFIG_ADC_CALIBRATION:
        *config_type = HAL_LCPU_CONFIG_ADC_CALIBRATION_ROM;
        break;
    case HAL_LCPU_CONFIG_SDADC_CALIBRATION:
        *config_type = HAL_LCPU_CONFIG_SDADC_CALIBRATION_ROM;
        break;
    case HAL_LCPU_CONFIG_SN:
        *config_type = HAL_LCPU_CONFIG_SN_ROM;
        break;
    case HAL_LCPU_CONFIG_CHIP_REV:
        *config_type = HAL_LCPU_CONFIG_CHIP_REV_ROM;
        break;
    case HAL_LCPU_CONFIG_BATTERY_CALIBRATION:
        *config_type = HAL_LCPU_CONFIG_BATTERY_CALIBRATION_ROM;
        break;
#ifdef SF32LB56X
    case HAL_LCPU_CONFIG_SOFT_CVSD:
        *config_type = HAL_LCPU_CONFIG_IS_SOFT_CVSD_ROM;
        break;
#endif
    default:
        *config_type  = HAL_LCPU_CONFIG_MAX;
    }

#else // !(defiend(SF32LB55X) || defiend(SF32LB56X) || defiend(SF32LB58X))
    *config_type = type;
#endif // defiend(SF32LB55X) || defiend(SF32LB56X) || defiend(SF32LB58X)
#endif // defined(LCPU_CONFIG_V1)

    return ret;
}


__WEAK HAL_StatusTypeDef HAL_LCPU_CONFIG_set_core(uint8_t config_type, void *value, uint16_t length)
{
    uint8_t ret = LCPU_CONFIG_set((uint8_t *)(g_lcpu_config_context), config_type, (uint8_t *)value, length);
    return ret == 0 ? HAL_OK : HAL_ERROR;
}


__WEAK HAL_StatusTypeDef HAL_LCPU_CONFIG_get_core(uint8_t config_type, uint8_t *value, uint16_t *length)
{
    uint8_t ret = LCPU_CONFIG_get((uint8_t *)(g_lcpu_config_context), config_type, value, length);
    return ret == 0 ? HAL_OK : HAL_ERROR;
}



__HAL_ROM_USED HAL_StatusTypeDef HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_TYPE_T type, void *value, uint16_t length)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t config_type;

    if (HAL_LCPU_CONFIG_get_type(type, &config_type) != HAL_OK)
        return ret;

    if (!value || config_type >= HAL_LCPU_CONFIG_MAX)
        return ret;

#if (defined(SOC_BF0_HCPU)) && (defined(SF32LB52X))
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif

    HAL_LCPU_CONIFG_init();

    ret = HAL_LCPU_CONFIG_set_core(config_type, value, length);


#if (defined(SOC_BF0_HCPU)) && (defined(SF32LB52X))
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif

    return ret;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_TYPE_T type, uint8_t *value, uint16_t *length)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t config_type;

    if (HAL_LCPU_CONFIG_get_type(type, &config_type) != HAL_OK)
        return ret;

    if (!value || !length || (config_type >= HAL_LCPU_CONFIG_MAX))
        return ret;

#if (defined(SOC_BF0_HCPU)) && (defined(SF32LB52X))
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif

    HAL_LCPU_CONIFG_init();

    // No need to handle if HCPU didn't set any value.
    if (g_lcpu_config_context->magic_num != HAL_LPCU_CONFIG_MAGIC_NUM)
    {
#if (defined(SOC_BF0_HCPU)) && (defined(SF32LB52X))
        HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
        return ret;
    }

    ret = HAL_LCPU_CONFIG_get_core(config_type, value, length);

#if (defined(SOC_BF0_HCPU)) && (defined(SF32LB52X))
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif

    return ret;
}