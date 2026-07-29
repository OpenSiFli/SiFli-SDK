/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"
#include "bf0_hal_rtc.h"

#ifndef LXT_LP_CYCLE
    #define LXT_LP_CYCLE 200
#endif

#ifndef PWRKEY_CNT_CLOCK_FREQ
    #define PWRKEY_CNT_CLOCK_FREQ  (32000)
#endif

#ifndef PWRKEY_HARD_RESET_TIME
    #define PWRKEY_HARD_RESET_TIME     (10)   /* unit:s */
#endif

static uint16_t mpi1_div = 1;
static uint16_t mpi2_div = 1;
static uint16_t mpi3_div = 1;

static uint32_t otp_flash_addr = AUTO_FLASH_MAC_ADDRESS;

#define FUNC_BSP_FLASH_DIV_GET(i) \
uint16_t BSP_GetFlash##i##DIV(void) \
{ \
    return mpi##i##_div; \
}\

#define FUNC_BSP_FLASH_DIV_SET(i) \
void BSP_SetFlash##i##DIV(uint16_t div) \
{ \
    mpi##i##_div = div; \
}\

FUNC_BSP_FLASH_DIV_GET(1);
FUNC_BSP_FLASH_DIV_GET(2);
FUNC_BSP_FLASH_DIV_GET(3);

FUNC_BSP_FLASH_DIV_SET(1)
FUNC_BSP_FLASH_DIV_SET(2)
FUNC_BSP_FLASH_DIV_SET(3)

int rt_psram_init(void);
int rt_hw_flash1_init(uint8_t auto_detect);
int rt_hw_flash2_init(uint8_t auto_detect);
int rt_hw_flash_init(void);

#if !defined(CFG_FACTORY_DEBUG)
uint32_t BSP_GetOtpBase(void)
{
    return otp_flash_addr;
}
#endif

#ifdef SOC_BF0_HCPU
static void LRC_init(void)
{
    HAL_PMU_RC10Kconfig();

    HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
}
#endif

#ifdef SOC_BF0_HCPU
    __USED
#endif
void HAL_PreInit(void)
{
#ifdef SOC_BF0_HCPU

    //__asm("B .");
    /* not switch back to XT48 if other clock source has been selected already */
    if (RCC_SYSCLK_HRC48 == HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS))
    {
        // To avoid somebody cancel request.
        HAL_HPAON_EnableXT48();
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    }

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);

    if (PM_STANDBY_BOOT != SystemPowerOnModeGet())
    {
        // Halt LCPU first to avoid LCPU in running state
        HAL_HPAON_WakeCore(CORE_ID_LCPU);
        HAL_RCC_Reset_and_Halt_LCPU(1);
        HAL_HPAON_StartGTimer();
        HAL_PMU_EnableRC32K(1);
        HAL_PMU_LpCLockSelect(PMU_LPCLK_RC32);
        hwp_pmuc->PWRKEY_CNT = PWRKEY_CNT_CLOCK_FREQ * PWRKEY_HARD_RESET_TIME ;     //set pwrkey hard reset time time*32000

#ifndef LXT_DISABLE
        HAL_PMU_EnableXTAL32();
        if (HAL_PMU_LXTReady() != HAL_OK)
            HAL_ASSERT(0);
        // RTC/GTIME/LPTIME Using same low power clock source
        HAL_RTC_ENABLE_LXT();
#endif

#ifndef CFG_BOOTLOADER
        {
            HAL_PMU_SetWdt((uint32_t)hwp_wdt2);   // Add reboot cause for watchdog2
        }
#endif /* CFG_BOOTLOADER */
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_LP_PERI, RCC_CLK_PERI_HXT48);

        HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
        if (HAL_LXT_DISABLED())
            LRC_init();
    }

    HAL_RCC_HCPU_ConfigHCLK(240);
    HAL_RCC_HCPU_EnableDLL2(288000000);

    // Reset sysclk used by HAL_Delay_us
    HAL_Delay_us(0);

    mpi1_div = 2;
    mpi2_div = 2;
    mpi3_div = 4;

    /* Init the low level hardware */
    HAL_MspInit();

#if defined (BSP_USING_PSRAM)
#ifdef BSP_USING_PSRAM1
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM1, RCC_CLK_FLASH_DLL2);
#endif /* BSP_USING_PSRAM1 */
#ifdef BSP_USING_PSRAM2
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM2, RCC_CLK_FLASH_DLL2);
#endif /* BSP_USING_PSRAM2 */
    bsp_psramc_init();
#endif

#if defined(BSP_USING_NOR_FLASH1) || defined(BSP_USING_NOR_FLASH2) || defined(BSP_USING_NOR_FLASH3)

#ifdef BSP_USING_NOR_FLASH1
    mpi1_div = 3;
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
#endif /* BSP_USING_NOR_FLASH1 */


#ifdef BSP_USING_NOR_FLASH2
    mpi2_div = 3;
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
#endif /* BSP_USING_NOR_FLASH2 */

#ifdef BSP_USING_NOR_FLASH3
    mpi3_div = 4;
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH3, RCC_CLK_FLASH_DLL2);
#endif /* BSP_USING_NOR_FLASH3 */

    if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
    {
        /* rt_hw_flash_init cannot be called as data has not been restored at this moment,
            so rt_sem_init cannot be called */
        BSP_Flash_Init();
    }
    else
    {
#ifdef BSP_USING_RTTHREAD
        rt_hw_flash_init();
#else
        BSP_Flash_Init();
#endif /* BSP_USING_RTTHREAD */
    }
#endif /* BSP_USING_NOR_FLASH1 || BSP_USING_NOR_FLASH2 || BSP_USING_NOR_FLASH3 */


#elif defined(SOC_BF0_LCPU)
    HAL_LPAON_EnableXT48();
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_LP_PERI, RCC_CLK_PERI_HXT48);
    HAL_RCC_LCPU_SetDiv(2, 1, 3);
    HAL_MspInit();
#endif
}

extern void BSP_PIN_Init(void);
// Called in HAL_MspInit
#ifdef SOC_BF0_HCPU
    __USED
#endif
void BSP_IO_Init(void)
{
    BSP_PIN_Init();
    BSP_Power_Up(true);
}

__WEAK void SystemClock_Config(void)
{

}

