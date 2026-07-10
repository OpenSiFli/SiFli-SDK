/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include "register.h"

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define  XTAL            (50000000UL)     /* Oscillator frequency */

#define  SYSTEM_CLOCK    (XTAL / 2U)


/*----------------------------------------------------------------------------
  Externals
 *----------------------------------------------------------------------------*/
#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
    extern uint32_t __Vectors;
#endif

/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = 48000000;//SYSTEM_CLOCK;  /* System Core Clock Frequency */


/*----------------------------------------------------------------------------
  System Core Clock update function
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = SYSTEM_CLOCK;
}

__WEAK void boot_images(void)
{
}

__WEAK void hw_preinit0(void)
{
}

enum
{
    ATTR_CODE_IDX,
    ATTR_RAM_IDX,
    ATTR_DEVICE_IDX,
    ATTR_PSRAM_WB_IDX,
    ATTR_PSRAM_WT_IDX,
};

#define ATTR_CODE       ARM_MPU_ATTR(ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0), ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0))
#define ATTR_RAM        ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE)
#define ATTR_DEVICE     ARM_MPU_ATTR(ARM_MPU_ATTR_DEVICE, ARM_MPU_ATTR_DEVICE_nGnRnE)
/* write back */
#define ATTR_PSRAM_WB   ARM_MPU_ATTR(ARM_MPU_ATTR_MEMORY_(0, 1, 1, 1), ARM_MPU_ATTR_MEMORY_(0, 1, 1, 1))
/* write through */
#define ATTR_PSRAM_WT   ARM_MPU_ATTR(ARM_MPU_ATTR_MEMORY_(0, 0, 1, 1), ARM_MPU_ATTR_MEMORY_(0, 0, 1, 1))

// To avoid different config lead region overlap
static void mpu_clear_region(void)
{
    for (uint32_t i = 0; i < MPU_REGION_NUM; i++)
    {
        ARM_MPU_ClrRegion(i);
    }
}

#ifdef SOC_BF0_ACPU
#define DCACHE_SIZE 8192
#define ICACHE_SIZE 16384
__WEAK void mpu_config(void)
{
    uint32_t rnr, rbar, rlar;

    SCB_DisableDCache();
    SCB_DisableICache();

    ARM_MPU_Disable();

    mpu_clear_region();

    ARM_MPU_SetMemAttr(ATTR_CODE_IDX, ATTR_CODE);
    ARM_MPU_SetMemAttr(ATTR_RAM_IDX, ATTR_RAM);
    ARM_MPU_SetMemAttr(ATTR_DEVICE_IDX, ATTR_DEVICE);
    ARM_MPU_SetMemAttr(ATTR_PSRAM_WB_IDX, ATTR_PSRAM_WB);
    ARM_MPU_SetMemAttr(ATTR_PSRAM_WT_IDX, ATTR_PSRAM_WT);

    rnr = 0;

    // XIP Code region
    rbar = ARM_MPU_RBAR(0x62000000, ARM_MPU_SH_NON, 1, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x6FFFFFFF, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // hpsys ram, disable sram cache
    rbar = ARM_MPU_RBAR(0x20000000, ARM_MPU_SH_OUTER, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x2007ffff, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // peripheral
    rbar = ARM_MPU_RBAR(0x40000000, ARM_MPU_SH_NON, 0, 1, 1); //Non-shareable,RW,any privilege,non-executable
    rlar = ARM_MPU_RLAR(0x5fffffff, ATTR_DEVICE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // psram
    rbar = ARM_MPU_RBAR(0x60000000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
#ifdef PSRAM_CACHE_WB
    rlar = ARM_MPU_RLAR(0x61ffffff, ATTR_PSRAM_WB_IDX);
#else
    rlar = ARM_MPU_RLAR(0x61ffffff, ATTR_PSRAM_WT_IDX);
#endif

    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // lpsys ram
    rbar = ARM_MPU_RBAR(0x20400000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x20413fff, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    HAL_ASSERT(rnr <= MPU_REGION_NUM);
    ARM_MPU_Enable(MPU_CTRL_HFNMIENA_Msk);

    SCB_EnableDCache();
    SCB_EnableICache();
}
#elif defined(SOC_BF0_HCPU)
#define DCACHE_SIZE 16384
#define ICACHE_SIZE (DCACHE_SIZE<<1)
__WEAK void mpu_config(void)
{
    uint32_t rnr, rbar, rlar;

    SCB_DisableDCache();
    SCB_DisableICache();

    ARM_MPU_Disable();

    mpu_clear_region();

    ARM_MPU_SetMemAttr(ATTR_CODE_IDX, ATTR_CODE);
    ARM_MPU_SetMemAttr(ATTR_RAM_IDX, ATTR_RAM);
    ARM_MPU_SetMemAttr(ATTR_DEVICE_IDX, ATTR_DEVICE);
    ARM_MPU_SetMemAttr(ATTR_PSRAM_WB_IDX, ATTR_PSRAM_WB);
    ARM_MPU_SetMemAttr(ATTR_PSRAM_WT_IDX, ATTR_PSRAM_WT);

    rnr = 0;

    //  hpsys rom
    rbar = ARM_MPU_RBAR(0x0, ARM_MPU_SH_NON, 1, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x0000ffff, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    //  flash1, region 1
    rbar = ARM_MPU_RBAR(0x10000000, ARM_MPU_SH_NON, 1, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x1fffffff, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // nand
    rbar = ARM_MPU_RBAR(0x64000000, ARM_MPU_SH_NON, 1, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x6FFFFFFF, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // hpsys ram, disable sram cache
    rbar = ARM_MPU_RBAR(0x20000000, ARM_MPU_SH_OUTER, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x2027ffff, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // peripheral
    rbar = ARM_MPU_RBAR(0x40000000, ARM_MPU_SH_NON, 0, 1, 1); //Non-shareable,RW,any privilege,non-executable
    rlar = ARM_MPU_RLAR(0x5fffffff, ATTR_DEVICE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // psram
    rbar = ARM_MPU_RBAR(0x60000000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
#ifdef PSRAM_CACHE_WB
    rlar = ARM_MPU_RLAR(0x63ffffff, ATTR_PSRAM_WB_IDX);
#else
    rlar = ARM_MPU_RLAR(0x63ffffff, ATTR_PSRAM_WT_IDX);
#endif

    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // lpsys ram
    rbar = ARM_MPU_RBAR(0x20400000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x20413fff, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // ble rom
    rbar = ARM_MPU_RBAR(0x20800000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x208fffff, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    HAL_ASSERT(rnr <= MPU_REGION_NUM);
    ARM_MPU_Enable(MPU_CTRL_HFNMIENA_Msk);

    SCB_EnableDCache();
    SCB_EnableICache();
}
#else
__WEAK void mpu_config(void)
{
//#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uint32_t rnr, rbar, rlar;

    ARM_MPU_Disable();

    mpu_clear_region();

    ARM_MPU_SetMemAttr(ATTR_CODE_IDX, ATTR_CODE);
    ARM_MPU_SetMemAttr(ATTR_RAM_IDX, ATTR_RAM);
    ARM_MPU_SetMemAttr(ATTR_DEVICE_IDX, ATTR_DEVICE);

    rnr  = 0;

    // LPSYS ROM, ITCM and C-BUS RAM space
    rbar = ARM_MPU_RBAR(0x0, ARM_MPU_SH_NON, 1, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x004bffff, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    //mpi1 and mpi2
    rbar = ARM_MPU_RBAR(0x60000000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x9FFFFFFF, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // HPSYS RAM
    /* disable cache */
    rbar = ARM_MPU_RBAR(0x2A000000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x2A27FFFF, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // peripheral
    rbar = ARM_MPU_RBAR(0x40000000, ARM_MPU_SH_NON, 0, 1, 1); //Non-shareable,RW,any privilege,non-executable
    rlar = ARM_MPU_RLAR(0x5FFFFFFF, ATTR_DEVICE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // LPSYS DTCM and RAM
    // disable cache
    rbar = ARM_MPU_RBAR(0x203fc000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x204FFFFF, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    HAL_ASSERT(rnr <= MPU_REGION_NUM);

    ARM_MPU_Enable(MPU_CTRL_HFNMIENA_Msk);
//#endif

}
#endif



#ifdef PSRAM_CACHE_WB
__WEAK int mpu_dcache_clean(void *data, uint32_t size)
{
    int r = 0;
    if (IS_DCACHED_RAM(data))
    {
        if (size > DCACHE_SIZE)
        {
            SCB_CleanDCache();
            r = 1;
        }
        else
            SCB_CleanDCache_by_Addr(data, size);
    }
    return r;
}

#endif

__WEAK int mpu_dcache_invalidate(void *data, uint32_t size)
{
    int r = 0;
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (IS_DCACHED_RAM(data))
    {
        if (size > DCACHE_SIZE)
        {
            SCB_InvalidateDCache();
            r = 1;
        }
        else
            SCB_InvalidateDCache_by_Addr(data, size);
    }
#endif /* __DCACHE_PRESENT == 1U */
    return r;
}

__WEAK int mpu_icache_invalidate(void *data, uint32_t size)
{
    int r = 0;
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
    if (IS_DCACHED_RAM(data))
    {
        if (size > ICACHE_SIZE)
        {
            SCB_InvalidateICache();
            r = 1;
        }
        else
            SCB_InvalidateICache_by_Addr(data, size);
    }
#endif /* __ICACHE_PRESENT == 1U */
    return r;
}

__WEAK void cache_enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();

}

__WEAK void cache_disable(void)
{
    SCB_DisableICache();
    SCB_DisableDCache();
}

typedef void (*scatter_load_fun)(uint32_t, uint32_t, uint32_t);

typedef struct
{
    uint32_t exe_reg;
    uint32_t load_reg;
    uint32_t size;
    scatter_load_fun func;
} region_load_t;

extern int Region$$Table$$Base;
extern int Region$$Table$$Limit;

__WEAK void rom_scatterload(void)
{
    uint32_t reg_start = (uint32_t)&Region$$Table$$Base;
    uint32_t reg_stop = (uint32_t)&Region$$Table$$Limit;
    region_load_t *reg_ctrl;
    scatter_load_fun func;
    while (reg_start < reg_stop)
    {
        reg_ctrl = (region_load_t *)reg_start;
        func = (scatter_load_fun)((uint32_t)reg_ctrl->func | 1);
        func(reg_ctrl->exe_reg, reg_ctrl->load_reg, reg_ctrl->size);
        // sizeof region load
        reg_start += 16;
    }
}

__WEAK void SystemPowerOnModeInit(void)
{

}


__WEAK pm_power_on_mode_t SystemPowerOnModeGet(void)
{
    return PM_COLD_BOOT;
}

__WEAK void SystemVectorTableRemapping(void)
{
    return;
}


#ifdef BL_INIT_TEST
#include <string.h>
#ifdef SOC_BF0_HCPU
    #define TESTPRINTADDR 0x4002FFFC
#else
    #define TESTPRINTADDR 0x5002FFFC
#endif
static void system_init_test(void)
{
#ifdef SOC_BF0_HCPU
    hwp_lpsys_aon->PMR &= ~LPSYS_AON_PMR_CPUWAIT;
#endif
    uint8_t pr_char[] = "hello world\n";
    for (uint32_t i = 0; i < strlen((char *)pr_char); i++)
        *((volatile uint32_t *)TESTPRINTADDR) = pr_char[i];
    *((volatile uint32_t *)TESTPRINTADDR) = 1;
    __ASM
    (
        "B . \n"
    );
}
#endif

/*----------------------------------------------------------------------------
  System initialization function
 *----------------------------------------------------------------------------*/
void SystemInit(void)
{
#ifdef BL_INIT_TEST
    system_init_test();
#endif

#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
    SCB->VTOR = (uint32_t) &__Vectors;
#endif

    /* enable CP0/CP1/CP2 Full Access */
    SCB->CPACR |= (3U << (0U * 2U)) | (3U << (1U * 2U)) | (3U << (2U * 2U));

#if defined (__FPU_USED) && (__FPU_USED == 1U)
    SCB->CPACR |= ((3U << 10U * 2U) |         /* enable CP10 Full Access */
                   (3U << 11U * 2U));         /* enable CP11 Full Access */
#endif

#ifdef UNALIGNED_SUPPORT_DISABLE
    SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    TZ_SAU_Setup();
#endif

    {
        hw_preinit0();
        cache_disable();
        mpu_config();
        cache_enable();
    }

    SystemPowerOnModeInit();

#if defined(SOC_BF0_LCPU)&&defined(ROM_ENABLED)
    rom_scatterload();
#endif // SOC_BF0_LCPU

    SystemCoreClock = SYSTEM_CLOCK;
}

void lcpu_rom_jump(void)
{
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    __asm("MOV SP, %0" :: "r"(hwp_lpsys_aon->SPR));
    __asm("MOV PC, %0" :: "r"(hwp_lpsys_aon->PCR));
#endif
}
