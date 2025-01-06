/**
  ******************************************************************************
  * @file   bf0_sys_cfg.c
  * @author Sifli software development team
  * @brief   This file provides system configure functions
  ******************************************************************************
*/
/**
 *
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

#include "bf0_hal.h"
#include <string.h>
#include "mem_map.h"
#include "rtconfig.h"
#include "register.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */

#ifdef HAL_SYSTEM_CONFIG_ENABLED


//static uint32_t conf_buf[CFG_SYS_SIZE / 4];
HAL_RETM_BSS_SECT(sip1_mode, static uint8_t sip1_mode);
HAL_RETM_BSS_SECT(sip2_mode, static uint8_t sip2_mode);

static uint32_t conf_user[CFG_USER_SIZE / 4];
static uint32_t conf_cust[CFG_USER_SIZE / 4];

static uint8_t BSP_OTP_CFG_READ(uint8_t id, uint8_t *data, uint8_t size, uint8_t *buf, uint32_t buf_size)
{
    int i = 0;
    int len = 0;
    uint32_t fac_cfg_size = 0;

    if (buf == NULL)
    {
        return 0;
    }
    fac_cfg_size = buf_size ;

    uint8_t *p = buf ;

    while (p[i] != FACTORY_CFG_ID_UNINIT)
    {
        len = p[i + 1];
        if (p[i] == id)                               // Found config
        {
            break;
        }

        if ((i + len + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size)   // More than max configuration area?
        {
            len = 0;
            break;
        }

        i += (len + SYSCFG_FACTORY_HDR_SIZE);       // Next config
        len = 0;
    }
    if (len)                                        // Found config
    {
        if (len > size)
            len = size;
        memcpy(data, &p[i + SYSCFG_FACTORY_HDR_SIZE], len);
    }

    return len;
}

static uint8_t BSP_OTP_CUST_CFG_READ(uint8_t id, uint8_t *data, uint8_t size, uint8_t *buf, uint32_t buf_size)
{
    uint8_t res;
    uint8_t *buf_user;
    res = BSP_OTP_CFG_READ(id, data, size, buf, buf_size);
    if (res > 0)
        goto end;
    buf_user = (uint8_t *)conf_user;
    res = BSP_OTP_CFG_READ(id, data, size, buf_user, CFG_USER_SIZE);
    if (res > 0)
        goto end;
    buf_user = (uint8_t *)conf_cust;
    res = BSP_OTP_CFG_READ(id, data, size, buf_user, CFG_USER_SIZE);
end:
    return res;
}

#ifndef SF32LB55X
static MPI_TypeDef *BSP_GetFlashByAddr(uint32_t addr)
{
    MPI_TypeDef *fhandle = NULL;

    if ((addr >= QSPI1_MEM_BASE) && (addr < (QSPI1_MEM_BASE + QSPI1_MAX_SIZE)))
        fhandle = FLASH1;
    else if ((addr >= QSPI2_MEM_BASE) && (addr < (QSPI2_MEM_BASE + QSPI2_MAX_SIZE)))
        fhandle = FLASH2;
#ifdef FLASH3
    else if ((addr >= QSPI3_MEM_BASE) && (addr < (QSPI3_MEM_BASE + QSPI3_MAX_SIZE)))
        fhandle = FLASH3;
#endif
#ifdef FLASH4
    else if ((addr >= QSPI4_MEM_BASE) && (addr < (QSPI4_MEM_BASE + QSPI4_MAX_SIZE)))
        fhandle = FLASH4;
#endif
#ifdef FLASH5
    else if ((addr >= QSPI5_MEM_BASE) && (addr < (QSPI5_MEM_BASE + QSPI5_MAX_SIZE)))
        fhandle = FLASH5;
#endif
    return fhandle;
}
#else //!55x
static QSPI_TypeDef *BSP_GetFlashByAddr(uint32_t addr)
{
    QSPI_TypeDef *fhandle = NULL;

    if ((addr >= QSPI1_MEM_BASE) && (addr < (QSPI1_MEM_BASE + QSPI1_MAX_SIZE)))
        fhandle = FLASH1;
    else if ((addr >= QSPI2_MEM_BASE) && (addr < (QSPI2_MEM_BASE + QSPI2_MAX_SIZE)))
        fhandle = FLASH2;
    else if ((addr >= QSPI3_MEM_BASE) && (addr < (QSPI3_MEM_BASE + QSPI3_MAX_SIZE)))
        fhandle = FLASH3;
    else if ((addr >= QSPI4_MEM_BASE) && (addr < (QSPI4_MEM_BASE + QSPI4_MAX_SIZE)))
        fhandle = FLASH4;

    return fhandle;
}
#endif /* !SF32LB55X */

#ifdef SOC_BF0_HCPU

HAL_RAM_RET_CODE_SECT(BSP_HxtCbank_Config, void BSP_HxtCbank_Config(uint32_t cbank_sel))
{
    int clk_src = HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS);

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
    uint32_t dll1_freq = HAL_RCC_HCPU_GetDLL1Freq();
    uint32_t dll2_freq = HAL_RCC_HCPU_GetDLL2Freq();

    HAL_PMU_SET_HXT_CBANK(cbank_sel);

    HAL_Delay_us(0);
    HAL_Delay_us(40);

    HAL_RCC_HCPU_DisableDLL1();
    HAL_RCC_HCPU_DisableDLL2();

    HAL_RCC_HCPU_EnableDLL1(dll1_freq);
    HAL_RCC_HCPU_EnableDLL2(dll2_freq);


    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, clk_src);
    HAL_Delay_us(0);

    return;
}
#endif

#ifdef SF32LB52X

static uint32_t conf_sys[CFG_SYS_SIZE / 4];

int BSP_System_Config(void)
{
    int res, i;
    uint8_t *data;
    uint16_t ate_efuse_offset = 256; // bank1 , offset 0
    FLASH_HandleTypeDef fhandle;
    int len;
    uint8_t *buf;
    FACTORY_CFG_BATTERY_CALI_T battery_cfg;
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
    buf = (uint8_t *)conf_user;
    len = HAL_QSPI_READ_OTP(&fhandle, CFG_USER_OTP_PAGE << 12, buf, CFG_USER_SIZE);
    HAL_ASSERT(len > 0);

    buf = (uint8_t *)conf_cust;
    len = HAL_QSPI_READ_OTP(&fhandle, CFG_CUST_OTP_PAGE << 12, buf, CFG_USER_SIZE);
    HAL_ASSERT(len > 0);

    /* set hxt_cbank */
    res = BSP_OTP_CUST_CFG_READ(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&xtal_cfg, sizeof(FACTORY_CFG_CRYSTAL_T), (uint8_t *)conf_buf, len);
    if ((res > 0) && (xtal_cfg.cbank_sel != 0) && (xtal_cfg.cbank_sel != 0x3ff)) // add xtal invalid data check
    {
#ifdef SOC_BF0_HCPU
        BSP_HxtCbank_Config(xtal_cfg.cbank_sel);
#endif
    }

    return 0;
}

char *BSP_Get_SysCfg_Cache()
{
    return (char *)conf_sys;
}

// split to adc/pmu/charge and move to driver code later?
int BSP_CONFIG_get(int type, uint8_t *buf, int length)
{
    int ret = 0;
    uint8_t *data = (uint8_t *)conf_sys;
    uint8_t pid = __HAL_SYSCFG_GET_PID();
    uint8_t avdd_v18_en = 0, pvdd_v18_en = 0;

#if defined(AVDD_V18_ENABLE)
    avdd_v18_en = 1;
#endif/* AVDD_V18_ENABLE */
#if defined(PVDD_V18_ENABLE)
    pvdd_v18_en = 1;
#endif/* PVDD_1V8_ENABLE */

    if (buf == NULL || length <= 0)
        return 0;

#if 1
    // data[0] include PMU TRIM/POLAR/VOUT, it should not be 0 if do ate calibrate
    if (data[0] == 0)
        return 0;
#else
    // add debug data
    data[0] = (0xa << 4) | 0xd;
#endif
    if (type == FACTORY_CFG_ID_ADC)
    {
        if (length >= sizeof(FACTORY_CFG_ADC_T))
        {
            FACTORY_CFG_ADC_T *cfg = (FACTORY_CFG_ADC_T *)buf;
            ret = length;
            if (!SF32LB52X_LETTER_SERIES() && avdd_v18_en)
            {
                HAL_ASSERT(0);
            }

            if (SF32LB52X_LETTER_SERIES()  && avdd_v18_en)
            {
                cfg->vol10 = (uint16_t)data[22] | ((uint16_t)(data[23] & 0xf) << 8);
                cfg->low_mv = ((data[23] & 0xf0) >> 4) | ((data[24] & 1) << 4);
                cfg->vol25 = (uint16_t)((data[24] & 0xfe) >> 1) | ((uint16_t)(data[25] & 0x1f) << 7);
                cfg->high_mv = ((data[25] & 0xe0) >> 5) | ((data[26] & 0x3) << 3);
                cfg->vbat_reg = ((uint16_t)(data[26] & 0xfc) >> 2) | ((uint16_t)(data[27] & 0x3f) << 6);
                cfg->vbat_mv = ((data[27] & 0xc0) >> 6) | ((data[28] & 0xf) << 2);
                cfg->low_mv *= 100;     // data in efuse with 100 mv based
                cfg->high_mv *= 100;
                cfg->vbat_mv *= 100;
            }
            else
            {
                cfg->vol10 = (uint16_t)data[4] | ((uint16_t)(data[5] & 0xf) << 8);
                cfg->low_mv = ((data[5] & 0xf0) >> 4) | ((data[6] & 1) << 4);
                cfg->vol25 = (uint16_t)((data[6] & 0xfe) >> 1) | ((uint16_t)(data[7] & 0x1f) << 7);
                cfg->high_mv = ((data[7] & 0xe0) >> 5) | ((data[8] & 0x3) << 3);
                cfg->vbat_reg = ((uint16_t)(data[8] & 0xfc) >> 2) | ((uint16_t)(data[9] & 0x3f) << 6);
                cfg->vbat_mv = ((data[9] & 0xc0) >> 6) | ((data[10] & 0xf) << 2);
                cfg->low_mv *= 100;     // data in efuse with 100 mv based
                cfg->high_mv *= 100;
                cfg->vbat_mv *= 100;
            }

            if (SF32LB52X_LETTER_SERIES())
            {
                cfg->ldovref_flag = (data[17] & 0x8) >> 3;
                cfg->ldovref_sel = (data[17] & 0xf0) >> 4;
            }


#if 0
            cfg->vol10 = 1758;
            cfg->low_mv = 1000;
            cfg->vol25 = 3162; //3170;  //3162
            cfg->high_mv = 2500;
            cfg->vbat_reg = 2788;
            cfg->vbat_mv = 4200;
#endif
            if (cfg->vol10 == 0 || cfg->low_mv == 0 || cfg->vol25 == 0
                    || cfg->high_mv == 0 || cfg->vbat_reg == 0 || cfg->vbat_mv == 0)  // all data should be valid
                ret = 0;
        }
    }
    else if (type == FACTORY_CFG_ID_VBUCK)
    {
        if (length >= sizeof(FACTORY_CFG_VBK_LDO_T))
        {
            FACTORY_CFG_VBK_LDO_T *cfg = (FACTORY_CFG_VBK_LDO_T *)buf;
            ret = length;
            if (!SF32LB52X_LETTER_SERIES() && pvdd_v18_en)
            {
                HAL_ASSERT(0);
            }
            /*only 52D 3.3v use new*/
            if (((SF32LB52X_LETTER_SERIES()) && (pid == 4) && !pvdd_v18_en) || ((SF32LB52X_LETTER_SERIES()) && (pid != 4) && pvdd_v18_en))  /*52D 3v3, 52A 1v8*/
            {
                cfg->buck_vos_trim = data[20] & 7;
                cfg->buck_vos_polar = (data[20] & 8) >> 3;
                cfg->hpsys_ldo_vout = (data[20] & 0xf0) >> 4;
                cfg->lpsys_ldo_vout = data[21] & 0xf;
                cfg->vret_trim = (data[21] & 0xf0) >> 4;
                cfg->hpsys_ldo_vout2 = (data[28] & 0x78) >> 3;
            }
            else
            {
                cfg->buck_vos_trim = data[0] & 7;
                cfg->buck_vos_polar = (data[0] & 8) >> 3;
                cfg->hpsys_ldo_vout = (data[0] & 0xf0) >> 4;
                cfg->lpsys_ldo_vout = data[1] & 0xf;
                cfg->vret_trim = (data[1] & 0xf0) >> 4;

                //cfg->buck_vos_trim2 = data[13] & 7;
                //cfg->buck_vos_polar2 = (data[13] & 8) >> 3;
                cfg->hpsys_ldo_vout2 = (data[13] & 0xf0) >> 4;
                cfg->lpsys_ldo_vout2 = data[14] & 0xf;
            }


            if (!SF32LB52X_LETTER_SERIES())
            {
                cfg->vdd33_ldo2_vout = (data[2] & 0xf0) >> 4;
                cfg->vdd33_ldo3_vout = data[3] & 0xf;
                cfg->aon_vos_trim = (data[3] & 0x70) >> 4;
                cfg->aon_vos_polar = (data[3] & 0x80) >> 7;
            }

            if (!(SF32LB52X_LETTER_SERIES()) || ((SF32LB52X_LETTER_SERIES()) && !pvdd_v18_en))
                cfg->ldo18_vref_sel = data[2] & 0xf;

            if (cfg->hpsys_ldo_vout == 0 || cfg->hpsys_ldo_vout2 == 0)
                ret = 0;
        }
    }
    else if (type == FACTORY_CFG_ID_CHARGER)
    {
        if (length >= sizeof(FACTORY_CFG_CHARGER_T))
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

#else //52x

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
#endif

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

    HAL_PMU_SaveCalData(cfg);
}

#else // 52x?
static void BSP_CFG_CALIB_PMU(FACTORY_CFG_VBK_LDO_T *cfg)
{
    if (cfg == NULL)
        return;
}

#endif //55x

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

#ifndef SF32LB52X
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
#endif
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
    buf = (uint8_t *)conf_user;
    len = HAL_QSPI_READ_OTP(&fhandle, CFG_USER_OTP_PAGE << 12, buf, CFG_USER_SIZE);
    HAL_ASSERT(len > 0);

    buf = (uint8_t *)conf_cust;
    len = HAL_QSPI_READ_OTP(&fhandle, CFG_CUST_OTP_PAGE << 12, buf, CFG_USER_SIZE);
    HAL_ASSERT(len > 0);

#ifndef SF32LB55X
    res = BSP_OTP_CUST_CFG_READ(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&xtal_cfg, sizeof(FACTORY_CFG_CRYSTAL_T), (uint8_t *)conf_buf, CFG_SYS_SIZE);
    if ((res > 0) && (xtal_cfg.cbank_sel != 0) && (xtal_cfg.cbank_sel != 0x3ff)) // add xtal invalid data check
    {
#ifdef SOC_BF0_HCPU
        BSP_HxtCbank_Config(xtal_cfg.cbank_sel);
#endif
    }
#endif /* !SF32LB55X */

    return 0;
}
#endif // SF32LB52X

char *BSP_Get_UserOTP_Cache()
{
    return (char *)conf_user;
}

char *BSP_Get_CustOTP_Cache()
{
    return (char *)conf_cust;
}

HAL_RAM_RET_CODE_SECT(BSP_Get_Sip1_Mode, uint32_t BSP_Get_Sip1_Mode())
{
    return (uint32_t)sip1_mode;
}

HAL_RAM_RET_CODE_SECT(BSP_Get_Sip2_Mode, uint32_t BSP_Get_Sip2_Mode())
{
    return (uint32_t)sip2_mode;
}

__weak uint32_t BSP_GetOtpBase(void)
{
#if defined(SF32LB56X)||defined(SF32LB58X)
    return 0x1C000000;
#else
    return 0x10000000;
#endif
}

#endif //HAL_SYSTEM_CONFIG_ENABLED
/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
