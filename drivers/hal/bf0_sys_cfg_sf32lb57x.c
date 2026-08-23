/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
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
#ifdef SF32LB57X

#ifdef HAL_SYSTEM_CONFIG_ENABLED

static uint32_t conf_sys[CFG_SYS_SIZE / 4];

int BSP_System_Config(void)
{
    int res, i;
    uint8_t *data;
    uint16_t ate_efuse_offset = EFUSE_ATE_DATA_OFFSET; // bank1 , offset 0
    FLASH_HandleTypeDef fhandle;
    int len;
    uint8_t *buf;
    uint32_t conf_buf[CFG_SYS_SIZE / 4];
    FACTORY_CFG_CRYSTAL_T xtal_cfg;

    data  = (uint8_t *)&conf_sys[0];
    res = HAL_EFUSE_Init();
    if (res != 0)
    {
        //rt_kprintf("efuse init fail %d\n", res);
        return 1;
    }
    HAL_Delay_us(0);
    HAL_Delay_us(10);

    // initial data buffer to 0
    for (i = 0; i < CFG_SYS_SIZE / 4; i++)
        conf_sys[i] = 0;

    res = HAL_EFUSE_Read(ate_efuse_offset, data, CFG_SYS_SIZE);
    if (res != CFG_SYS_SIZE)
    {
        //rt_kprintf("Read EFUSE fail\n");
        return 2;
    }

    HAL_PMU_LoadCalData();

#if defined(CFG_SUPPORT_NON_OTP)
    return 0;
#endif /* CFG_SUPPORT_NON_OTP */

    uint32_t addr = BSP_GetOtpBase();
    memset(&fhandle, 0, sizeof(fhandle));
    fhandle.Instance = BSP_GetFlashByAddr(addr);
    HAL_ASSERT(fhandle.Instance);
    res = HAL_FLASH_PreInit(&fhandle);
    HAL_ASSERT(0 == res);

    // load sys otp page to cache buffer
    buf = (uint8_t *)conf_buf;
    len = HAL_QSPI_READ_OTP(&fhandle, CFG_IN_OTP_PAGE << 12, buf, CFG_SYS_SIZE);
    HAL_ASSERT(len > 0);

    // load user otp page to cache buffer
    buf = (uint8_t *)BSP_Get_UserOTP_Cache();
    len = HAL_QSPI_READ_OTP(&fhandle, CFG_USER_OTP_PAGE << 12, buf, CFG_USER_SIZE);
    HAL_ASSERT(len > 0);

    buf = (uint8_t *)BSP_Get_CustOTP_Cache();
    len = HAL_QSPI_READ_OTP(&fhandle, CFG_CUST_OTP_PAGE << 12, buf, CFG_USER_SIZE);
    HAL_ASSERT(len > 0);

    /* set hxt_cbank */
    res = BSP_OTP_ReadCfg(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&xtal_cfg, sizeof(FACTORY_CFG_CRYSTAL_T), (uint8_t *)conf_buf, len);
    if ((res > 0) && (xtal_cfg.cbank_sel != 0) && (xtal_cfg.cbank_sel != 0x3ff)) // add xtal invalid data check
    {
#ifdef SOC_BF0_HCPU
        BSP_HxtCbank_Config(xtal_cfg.cbank_sel);
#endif
    }

    return 0;
}

char *BSP_Get_SysCfg_Cache(void)
{
    return (char *)conf_sys;
}

// split to adc/pmu/charge and move to driver code later?
int BSP_CONFIG_get(int type, uint8_t *buf, int length)
{
    int ret = 0;
    uint8_t *data = (uint8_t *)conf_sys;

    if (buf == NULL || length <= 0)
        return 0;

    // data[0] include PMU TRIM/POLAR/VOUT, it should not be 0 if do ate calibrate
    if (data[0] == 0)
        return 0;

    if (type == FACTORY_CFG_ID_ADC)
    {
        if (length >= (int)sizeof(FACTORY_CFG_ADC_T))
        {
            FACTORY_CFG_ADC_T *cfg = (FACTORY_CFG_ADC_T *)buf;
            ret = length;

            cfg->vol10 = (uint16_t)data[4] | ((uint16_t)(data[5] & 0xf) << 8);
            cfg->low_mv = ((data[5] & 0xf0) >> 4) | ((data[6] & 1) << 4);
            cfg->vol25 = (uint16_t)((data[6] & 0xfe) >> 1) | ((uint16_t)(data[7] & 0x1f) << 7);
            cfg->high_mv = ((data[7] & 0xe0) >> 5) | ((data[8] & 0x3) << 3);
            cfg->vbat_reg = ((uint16_t)(data[8] & 0xfc) >> 2) | ((uint16_t)(data[9] & 0x3f) << 6);
            cfg->vbat_mv = ((data[9] & 0xc0) >> 6) | ((data[10] & 0xf) << 2);
            cfg->low_mv *= 100;     // data in efuse with 100 mv based
            cfg->high_mv *= 100;
            cfg->vbat_mv *= 100;

            if (cfg->vol10 == 0 || cfg->low_mv == 0 || cfg->vol25 == 0
                    || cfg->high_mv == 0 || cfg->vbat_reg == 0 || cfg->vbat_mv == 0)  // all data should be valid
                ret = 0;
        }
    }
    else if (type == FACTORY_CFG_ID_VBUCK)
    {
        if (length >= (int)sizeof(FACTORY_CFG_VBK_LDO_T))
        {
            FACTORY_CFG_VBK_LDO_T *cfg = (FACTORY_CFG_VBK_LDO_T *)buf;
            ret = length;

            cfg->buck_vos_trim = data[0] & 7;
            cfg->buck_vos_polar = (data[0] & 8) >> 3;
            cfg->hpsys_ldo_vout = (data[0] & 0xf0) >> 4;
            cfg->lpsys_ldo_vout = data[1] & 0xf;
            cfg->vret_trim = (data[1] & 0xf0) >> 4;

            cfg->hpsys_ldo_vout2 = (data[13] & 0xf0) >> 4;
            cfg->lpsys_ldo_vout2 = data[14] & 0xf;

            cfg->vdd33_ldo2_vout = (data[2] & 0xf0) >> 4;
            cfg->vdd33_ldo3_vout = data[3] & 0xf;
            cfg->aon_vos_trim = (data[3] & 0x70) >> 4;
            cfg->aon_vos_polar = (data[3] & 0x80) >> 7;

            cfg->ldo18_vref_sel = data[2] & 0xf;

            if (cfg->hpsys_ldo_vout == 0 || cfg->hpsys_ldo_vout2 == 0)
                ret = 0;
        }
    }
    else if (type == FACTORY_CFG_ID_CHARGER)
    {
        if (length >= (int)sizeof(FACTORY_CFG_CHARGER_T))
        {
            FACTORY_CFG_CHARGER_T *cfg = (FACTORY_CFG_CHARGER_T *)buf;
            ret = length;
            cfg->prog_v1p2 = (data[10] & 0xf0) >> 4;
            cfg->cv_vctrl = data[11] & 0x3f;
            cfg->cc_mn = (data[11] >> 6) | ((data[12] & 7) << 2);
            cfg->cc_mp = data[12] >> 3;

            cfg->chg_step = ((data[14] & 0xf0) >> 4) | ((data[15] & 0xf) << 4);
        }
    }
    else
    {
        ret = 0;
    }
    return ret;
}

#endif /* HAL_SYSTEM_CONFIG_ENABLED */

#endif /* SF32LB57X */
/**
  * @}
  */