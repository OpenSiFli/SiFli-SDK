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

#ifdef HAL_SYSTEM_CONFIG_ENABLED

//static uint32_t conf_buf[CFG_SYS_SIZE / 4];

static uint32_t conf_user[CFG_USER_SIZE / 4];
static uint32_t conf_cust[CFG_USER_SIZE / 4];

uint8_t BSP_OTP_CFG_READ(uint8_t id, uint8_t *data, uint8_t size, uint8_t *buf, uint32_t buf_size)
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

        if ((i + len + SYSCFG_FACTORY_HDR_SIZE) >= (int)fac_cfg_size)   // More than max configuration area?
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

uint8_t BSP_OTP_ReadCfg(uint8_t id, uint8_t *data, uint8_t size, uint8_t *buf, uint32_t buf_size)
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
MPI_TypeDef *BSP_GetFlashByAddr(uint32_t addr)
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
QSPI_TypeDef *BSP_GetFlashByAddr(uint32_t addr)
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
#endif /* SOC_BF0_HCPU */

char *BSP_Get_UserOTP_Cache(void)
{
    return (char *)conf_user;
}

char *BSP_Get_CustOTP_Cache(void)
{
    return (char *)conf_cust;
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