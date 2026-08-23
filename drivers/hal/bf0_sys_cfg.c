/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include <string.h>
#include "mem_map.h"
#include "rtconfig.h"
#include "register.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */

#if defined(SF32LB55X) || defined(SF32LB56X) || defined(SF32LB58X)

#ifdef HAL_SYSTEM_CONFIG_ENABLED

#if defined(LCPU_RUN_SEPERATE_IMG) || defined(LCPU_RUN_ROM_ONLY)
    #define BSP_CFG_IN_HCPU         (1)
#else
    #define BSP_CFG_IN_HCPU         (0)
#endif /* LCPU_RUN_SEPERATE_IMG || LCPU_RUN_ROM_ONLY */

//static uint32_t conf_buf[CFG_SYS_SIZE / 4];
HAL_RETM_BSS_SECT(sip1_mode, static uint8_t sip1_mode);
HAL_RETM_BSS_SECT(sip2_mode, static uint8_t sip2_mode);


typedef struct
{
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
} HAL_HCPU_CONFIG_T;

#if BSP_CFG_IN_HCPU

HAL_HCPU_CONFIG_T g_bsp_hcpu_config;

HAL_StatusTypeDef BSP_CONFIG_set(int type, uint8_t *value, int length)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
#ifdef SOC_BF0_HCPU

    switch (type)
    {
    case FACTORY_CFG_ID_ADC:
    {
#ifndef SF32LB55X
        if (length == 8) // 64 bit
        {
            memcpy((void *)&g_bsp_hcpu_config.adc_cal[0], value, length);
            ret = HAL_OK;
        }
#else
        if (length == 4) // 32 bit
        {
            memcpy((void *)&g_bsp_hcpu_config.adc_cal, value, length);
            ret = HAL_OK;
        }
#endif
        break;
    }
    case FACTORY_CFG_ID_SDMADC:
    {
        if (length == 8) // 64 bit
        {
            memcpy((void *)&g_bsp_hcpu_config.sdadc_cal, value, 8);
            ret = HAL_OK;
        }
        break;
    }
    case FACTORY_CFG_ID_BATTERY:
    {
        if (length == 8)
        {
            uint32_t *p = (uint32_t *)value;
            g_bsp_hcpu_config.battery_a = p[0];
            g_bsp_hcpu_config.battery_b = p[1];
            ret = HAL_OK;
        }
        else if (length == 12) //maybe 12 bytes
        {
            uint32_t *p = (uint32_t *)value;
            g_bsp_hcpu_config.battery_a = p[1];
            g_bsp_hcpu_config.battery_b = p[2];
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


int BSP_CONFIG_get(int type, uint8_t *value, int length)
{
    int ret = 0;

    switch (type)
    {
    case FACTORY_CFG_ID_ADC:
    {
#ifndef SF32LB55X
        if (length == 8) // 64 bit
        {
            memcpy((void *)value, (void *)&g_bsp_hcpu_config.adc_cal[0], 8);
            ret = length;
        }
#else
        if (length == 4) // 32 bit
        {
            memcpy((void *)value, (void *)&g_bsp_hcpu_config.adc_cal, 4);
            ret = length;
        }
#endif
        break;
    }
    case FACTORY_CFG_ID_SDMADC:
    {
        if (length == 8) // 64 bit
        {
            memcpy((void *)value, (void *)&g_bsp_hcpu_config.sdadc_cal, 8);
            ret = length;
        }
        break;
    }
    case FACTORY_CFG_ID_BATTERY:
    {
        if (length == 8)
        {
            uint32_t *p = (uint32_t *)value;
            p[0] = g_bsp_hcpu_config.battery_a;
            p[1] = g_bsp_hcpu_config.battery_b;
            ret = length;
        }
        else if (length == 12)
        {
            uint32_t *p = (uint32_t *)value;
            p[0] = 0xe8091ad7;
            p[1] = g_bsp_hcpu_config.battery_a;
            p[2] = g_bsp_hcpu_config.battery_b;
            ret = length;
        }
        break;
    }
    default:
        break;
    }
    return ret;

}

#else
HAL_LCPU_CONFIG_TYPE_T BSP_get_lcpu_type(int type)
{
    HAL_LCPU_CONFIG_TYPE_T lcpu_type = HAL_LCPU_CONFIG_MAX;

    switch (type)
    {
    case FACTORY_CFG_ID_ADC:
    {
        lcpu_type = HAL_LCPU_CONFIG_ADC_CALIBRATION;
        break;
    }
    case FACTORY_CFG_ID_SDMADC:
    {
        lcpu_type = HAL_LCPU_CONFIG_SDADC_CALIBRATION;
        break;
    }
    case FACTORY_CFG_ID_BATTERY:
    {
        lcpu_type = HAL_LCPU_CONFIG_BATTERY_CALIBRATION;
        break;
    }
    default:
        break;
    }
    return lcpu_type;
}
HAL_StatusTypeDef BSP_CONFIG_set(int type, uint8_t *buf, int length)
{
    HAL_StatusTypeDef ret;
    HAL_LCPU_CONFIG_TYPE_T lcpu_type;

    lcpu_type = BSP_get_lcpu_type(type);

    ret = HAL_LCPU_CONFIG_set(lcpu_type, buf, (uint16_t)length);

    return ret;
}
int BSP_CONFIG_get(int type, uint8_t *buf, int length)
{
    HAL_StatusTypeDef ret;
    HAL_LCPU_CONFIG_TYPE_T lcpu_type;
    uint16_t len = (uint16_t)length;

    lcpu_type = BSP_get_lcpu_type(type);

    ret = HAL_LCPU_CONFIG_get(lcpu_type, buf, &len);

    if (ret == HAL_OK)
    {
        return length;
    }
    else
    {
        return 0;
    }
}
#endif /* BSP_CFG_IN_HCPU */

#ifdef SF32LB55X
static void BSP_CFG_CALIB_PMU(FACTORY_CFG_VBK_LDO_T *cfg)
{
    if (cfg == NULL)
        return;

    MODIFY_REG(hwp_pmuc->BG1_CR, PMUC_BG1_CR_BG1_VREF12_Msk, cfg->vbuck1 << PMUC_BG1_CR_BG1_VREF12_Pos);
    MODIFY_REG(hwp_pmuc->LDO_CR, PMUC_LDO_CR_HPSYS_LDO_VREF_Msk, cfg->hp_ldo << PMUC_LDO_CR_HPSYS_LDO_VREF_Pos);
    MODIFY_REG(hwp_pmuc->LDO_CR, PMUC_LDO_CR_LPSYS_LDO_VREF_Msk, cfg->lp_ldo << PMUC_LDO_CR_LPSYS_LDO_VREF_Pos);
    MODIFY_REG(hwp_pmuc->VRET_CR, PMUC_VRET_CR_TRIM_Msk, cfg->vret << PMUC_VRET_CR_TRIM_Pos);
    MODIFY_REG(hwp_pmuc->BG2_CR, PMUC_BG2_CR_BG2_VREF12_Msk, cfg->vbuck2 << PMUC_BG2_CR_BG2_VREF12_Pos);
}
#elif defined(SF32LB58X)
static void BSP_CFG_CALIB_PMU(FACTORY_CFG_VBK_LDO_T *cfg)
{
    if (cfg == NULL)
        return;

    MODIFY_REG(hwp_pmuc->BG1_CR, PMUC_BG1_CR_BG1_VREF12_Msk, cfg->vbuck1 << PMUC_BG1_CR_BG1_VREF12_Pos);
    MODIFY_REG(hwp_pmuc->LDO_CR, PMUC_LDO_CR_HPSYS_LDO_VREF_Msk, cfg->hp_ldo << PMUC_LDO_CR_HPSYS_LDO_VREF_Pos);
    MODIFY_REG(hwp_pmuc->VRET_CR, PMUC_VRET_CR_TRIM_Msk, cfg->vret << PMUC_VRET_CR_TRIM_Pos);
    MODIFY_REG(hwp_pmuc->BG2_CR, PMUC_BG2_CR_BG2_VREF12_Msk, cfg->vbuck2 << PMUC_BG2_CR_BG2_VREF12_Pos);
}
#elif defined(SF32LB56X)
static void BSP_CFG_CALIB_PMU(FACTORY_CFG_VBK_LDO_T *cfg)
{
    if (cfg == NULL)
        return;

    MODIFY_REG(hwp_pmuc->BG1_CR, PMUC_BG1_CR_BG1_VREF12_Msk, cfg->vbuck1 << PMUC_BG1_CR_BG1_VREF12_Pos);
    MODIFY_REG(hwp_pmuc->HPSYS_LDO, PMUC_HPSYS_LDO_VREF_Msk, cfg->hp_ldo << PMUC_HPSYS_LDO_VREF_Pos);
    MODIFY_REG(hwp_pmuc->LPSYS_LDO, PMUC_LPSYS_LDO_VREF_Msk, cfg->lp_ldo << PMUC_LPSYS_LDO_VREF_Pos);
    MODIFY_REG(hwp_pmuc->VRET_CR, PMUC_VRET_CR_TRIM_Msk, cfg->vret << PMUC_VRET_CR_TRIM_Pos);
#ifdef SOC_BF0_HCPU
    HAL_PMU_SaveCalData(cfg);
#endif /* SOC_BF0_HCPU */
}

#endif /* SF32LB55X */

int BSP_System_Config(void)
{
    FLASH_HandleTypeDef fhandle;
    int res, len;
    uint8_t *buf;
    FACTORY_CFG_SDMADC_T sdm_cfg;
    FACTORY_CFG_BATTERY_CALI_T battery_cfg;
    FACTORY_CFG_VBK_LDO_T vbk_cfg;
    FACTORY_CFG_SIP_MOD_T sip_cfg;
    uint32_t conf_buf[CFG_SYS_SIZE / 4];
    FACTORY_CFG_CRYSTAL_T xtal_cfg;

    buf = (uint8_t *)conf_buf;

#if defined(CFG_SUPPORT_NON_OTP)
    return 0;
#endif

    uint32_t addr = BSP_GetOtpBase();
    memset(&fhandle, 0, sizeof(fhandle));
    fhandle.Instance = BSP_GetFlashByAddr(addr);
    HAL_ASSERT(fhandle.Instance);
    res = HAL_FLASH_PreInit(&fhandle);
    HAL_ASSERT(0 == res);

    len = HAL_QSPI_READ_OTP(&fhandle, CFG_IN_OTP_PAGE << 12, buf, CFG_SYS_SIZE);
    HAL_ASSERT(len > 0);

    res = BSP_OTP_CFG_READ(FACTORY_CFG_ID_VBUCK, (uint8_t *)&vbk_cfg, sizeof(FACTORY_CFG_VBK_LDO_T), buf, len);
    if (res > 0)
    {
        // set vbuck / ldo as configure.
        BSP_CFG_CALIB_PMU(&vbk_cfg);
    }
    else // change default
    {
        HAL_PMU_SET_HPSYS_LDO_VREF(PMU_HPSYS_LDO_VREF_DEFAULT);
    }

#ifdef SF32LB55X
    res = BSP_OTP_CFG_READ(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&xtal_cfg, sizeof(FACTORY_CFG_CRYSTAL_T), buf, len);
    if ((res > 0) && (xtal_cfg.cbank_sel != 0) && (xtal_cfg.cbank_sel != 0x3ff)) // add xtal invalid data check
    {
        // set crystal configure.
        HAL_PMU_SET_HXT_CBANK(xtal_cfg.cbank_sel);
    }
    else // do not set by factory test, use defualt 0x1ea;
    {
        HAL_PMU_SET_HXT_CBANK(0x1EA);
    }
#endif /* SF32LB55X */


    FACTORY_CFG_ADC_T adc_cfg;
    res = BSP_OTP_CFG_READ(FACTORY_CFG_ID_ADC, (uint8_t *)&adc_cfg, sizeof(FACTORY_CFG_ADC_T), buf, len);
    if (res > 0)
    {
        BSP_CONFIG_set(FACTORY_CFG_ID_ADC, (uint8_t *)&adc_cfg, sizeof(FACTORY_CFG_ADC_T));
    }
    res = BSP_OTP_CFG_READ(FACTORY_CFG_ID_SDMADC, (uint8_t *)&sdm_cfg, sizeof(FACTORY_CFG_SDMADC_T), buf, len);
    if (res > 0)
    {
        BSP_CONFIG_set(FACTORY_CFG_ID_SDMADC, (uint8_t *)&sdm_cfg, sizeof(FACTORY_CFG_SDMADC_T));
    }
    res = BSP_OTP_CFG_READ(FACTORY_CFG_ID_BATTERY, (uint8_t *)&battery_cfg, sizeof(FACTORY_CFG_BATTERY_CALI_T), buf, len);
    if (res > 0)
    {
        BSP_CONFIG_set(FACTORY_CFG_ID_BATTERY, (uint8_t *)&battery_cfg, sizeof(FACTORY_CFG_BATTERY_CALI_T));
    }

    res = BSP_OTP_CFG_READ(FACTORY_CFG_ID_SIPMODE, (uint8_t *)&sip_cfg, sizeof(FACTORY_CFG_SIP_MOD_T), buf, len);
    if (res <= 0)
    {
        sip1_mode = 0;
        sip2_mode = 0;
    }
    else
    {
        sip1_mode = sip_cfg.mpi1_mode;
        sip2_mode = sip_cfg.mpi2_mode;
    }

    // load user otp page to cache buffer
    buf = (uint8_t *)BSP_Get_UserOTP_Cache();
    len = HAL_QSPI_READ_OTP(&fhandle, CFG_USER_OTP_PAGE << 12, buf, CFG_USER_SIZE);
    HAL_ASSERT(len > 0);

    buf = (uint8_t *)BSP_Get_CustOTP_Cache();
    len = HAL_QSPI_READ_OTP(&fhandle, CFG_CUST_OTP_PAGE << 12, buf, CFG_USER_SIZE);
    HAL_ASSERT(len > 0);

#ifndef SF32LB55X
    res = BSP_OTP_ReadCfg(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&xtal_cfg, sizeof(FACTORY_CFG_CRYSTAL_T), (uint8_t *)conf_buf, CFG_SYS_SIZE);
    if ((res > 0) && (xtal_cfg.cbank_sel != 0) && (xtal_cfg.cbank_sel != 0x3ff)) // add xtal invalid data check
    {
#ifdef SOC_BF0_HCPU
        BSP_HxtCbank_Config(xtal_cfg.cbank_sel);
#endif
    }
#endif /* !SF32LB55X */

    return 0;
}

HAL_RAM_RET_CODE_SECT(BSP_Get_Sip1_Mode, uint32_t BSP_Get_Sip1_Mode())
{
    return (uint32_t)sip1_mode;
}

HAL_RAM_RET_CODE_SECT(BSP_Get_Sip2_Mode, uint32_t BSP_Get_Sip2_Mode())
{
    return (uint32_t)sip2_mode;
}

#endif //HAL_SYSTEM_CONFIG_ENABLED

#endif /* SF32LB55X || SF32LB56X || SF32LB58X */

/**
  * @}
  */