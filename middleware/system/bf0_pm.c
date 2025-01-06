/**
  ******************************************************************************
  * @file   bf0_pm.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
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

#include <rtthread.h>
#include <stdlib.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "section.h"
#include "drv_psram.h"
#include "drv_flash.h"
#include "drv_ext_dma.h"
#ifdef BSP_USING_WDT
    #include "drv_wdt.h"
#endif /* BSP_USING_WDT */
#include "bf0_pm.h"
#include "drv_io.h"
#include "drv_gpio.h"
#ifdef USING_IPC_QUEUE
    #include "ipc_queue.h"
#endif
#include "mem_section.h"
#ifdef USING_CONTEXT_BACKUP
    #include "context_backup.h"
#endif /* USING_CONTEXT_BACKUP */

#ifdef PM_METRICS_USE_COLLECTOR
    #include "metrics_collector.h"
    #include "metrics_id_middleware.h"
#endif /* PM_METRICS_USE_COLLECTOR */

#define LOG_TAG       "sys.pm"
#include "log.h"


#define RET_MEM_PC_OFFSET       (0)
#define RET_MEM_LR_OFFSET       (4)
#define RET_MEM_SP_OFFSET       (8)
#define RET_MEM_RESERVED_LEN    (RET_MEM_LR_OFFSET + 8)

#ifdef SF32LB52X
    #define LPSYS_WAKEUP_SRC_LPTIM  (LPAON_WAKEUP_SRC_LPTIM3)
    #define LPSYS_WSR_LPTIM         (LPSYS_AON_WSR_LPTIM3)
#else
    #define LPSYS_WAKEUP_SRC_LPTIM  (LPAON_WAKEUP_SRC_LPTIM2)
    #define LPSYS_WSR_LPTIM         (LPSYS_AON_WSR_LPTIM2)
#endif /* SF32LB52X */

//#define PM_PROFILING_ENABLED


#if defined(SOC_BF0_HCPU) && !defined(SF32LB55X)
    /* hardware supports deep WFI mode for freq scaling */
    #define PM_HW_DEEP_WFI_SUPPORT
#endif /* SOC_BF0_HCPU && !SF32LB55X */

#ifdef HPSYS_ITCM_BASE
    EXEC_REGION_DEF(ER_ITCM$$RW);
    EXEC_REGION_DEF(ER_ITCM$$ZI);
    EXEC_REGION_DEF(ER_ITCM$$RO);
    EXEC_REGION_LOAD_SYM_DEF(ER_ITCM$$RO);
#endif /* HPSYS_ITCM_BASE */

EXEC_REGION_DEF(ER_IROM1_EX$$RO);
EXEC_REGION_DEF(RW_IRAM1);
EXEC_REGION_LOAD_SYM_DEF(ER_IROM1_EX$$RO);


typedef struct
{
    /*general purpose register, r0~r12,r13(sp),r14(lr)*/
    uint32_t greg[15];
    uint32_t pc;
    uint32_t psr;
    uint32_t primask;
    uint32_t faultmask;
    uint32_t basepri;
    uint32_t control;
    uint32_t msp;
    uint32_t psp;
    uint32_t psplim;
    uint32_t msplim;

#ifdef SOC_BF0_HCPU
    /** reserved space for other data
     * [0]: PC
     * [1]: LR
     * [2]: SP
     */
    uint32_t reserved[RET_MEM_RESERVED_LEN];
#endif

} pm_reg_ctx_t;


#ifdef PM_METRICS_ENABLED

#define PM_METRICS_MAX_WAKEUP_SRC_NUM  (8)
typedef struct
{
    /** wakeup source, e.g. HPSYS_AON_WSR_RTC_Pos and HPSYS_AON_WSR_LPTIM1_Pos */
    uint8_t wakeup_src;
    uint8_t reserved[3];
    /** wakeup times of the specified wakeup_src */
    uint32_t wakeup_times;
    /** run time in second */
    float run_time;
} pm_wakeup_src_stat_t;

typedef struct pm_stat_tag
{
    /** sleep time in second  */
    float sleep_time;
    uint32_t total_wakeup_times;

    /** last WFI start GTime */
    uint32_t last_wfi_start_time;
    /** WFI duration in GTime */
    uint32_t wfi_time;

    /** last DeepWFI start GTime */
    uint32_t last_deepwfi_start_time;
    /** DeepWFI duration in GTime */
    uint32_t deepwfi_time;

    /** start run time(GTime) of one run mode */
    uint32_t run_start_time[PM_RUN_MODE_MAX];
    /** run time(GTime) of one run mode */
    uint32_t run_time[PM_RUN_MODE_MAX];
    /** number of wakeup source types that happne in last record period  */
    uint8_t wakeup_src_num;
    /** whether last_wfi_start_time is valid  */
    bool last_wfi_start_time_valid;
    /** current run mode, e.g. PM_RUN_MODE_HIGH_SPEED */
    uint8_t  curr_run_mode;
    pm_wakeup_src_stat_t wakeup_src_stat[PM_METRICS_MAX_WAKEUP_SRC_NUM];
} pm_stat_t;

typedef struct
{
    /** wakeup source, e.g. HPSYS_AON_WSR_RTC_Pos and HPSYS_AON_WSR_LPTIM1_Pos */
    uint8_t wakeup_src;
    uint8_t reserved[3];
    /** wakeup times of the specified wakeup_src */
    uint32_t wakeup_times;
    /** run time in second */
    float run_time;
} pm_wakeup_src_stat_metrics_t;

typedef struct pm_stat_metrics_tag
{
    /** sleep time in second */
    float sleep_time;
    /** total wakeup times */
    uint32_t total_wakeup_times;
    /** WFI duration in second */
    float wfi_time;
    /** DeepWFI duration in second */
    float deepwfi_time;
    /** run time in second */
    float run_time[PM_RUN_MODE_MAX];
    /** number of wakeup src types that happen in last record period */
    uint8_t wakeup_src_num;
    uint8_t reserved[3];
    /** array length is specified by wakeup_src_num */
    pm_wakeup_src_stat_metrics_t wakeup_src_stat[0];
} pm_stat_metrics_t;


#define PM_TIMER_NAME_LEN  (8)
#define PM_DEBUG_METRICS_REPORT_THRESH_TBL_SIZE  (4)
#define PM_DEBUG_METRICS_REPORT_THRESH_TICK      (5*60*RT_TICK_PER_SECOND)

typedef struct
{
    uint32_t wsr;
    uint32_t idle_time;
    char timer_name[PM_TIMER_NAME_LEN];
    uint8_t idle_mode_cnt;
    uint8_t ipc_queue_state;
} pm_debug_metrics_t;


typedef struct
{
    uint32_t last_sleep_tick;
    uint8_t  report_thresh;
    rt_timer_t timer;
} pm_debug_metrics_ctx_t;

#endif /* PM_METRICS_ENABLED */

#if defined(BSP_PM_FREQ_SCALING) && !defined(PM_HW_DEEP_WFI_SUPPORT)
typedef struct
{
    int sys_clk_src;
    int hdiv;
    int pdiv1;
    int pdiv2;
} pm_freq_scaling_param_t;
#endif /* BSP_PM_FREQ_SCALING && !PM_HW_DEEP_WFI_SUPPORT */

typedef struct
{
    bool init;
    /** GTIME when systick is stopped */
    uint32_t sleep_start_time;
    /** GTIME when GTIME and rt_tick are latched */
    uint32_t last_latch_time;
    /** rt_tick when GTIME nd rt_tick are latched */
    uint32_t last_latch_tick;
    /** fractional part of tick error which cannot be compensated on rt_tick yet, must be less than 1 */
    float tick_error_frac;
    uint32_t tick_acc;
    /** GTIME when system wakeups */
    uint32_t wakeup_time;
    uint32_t tick_reset_cnt;
} pm_tick_cal_t;

typedef struct
{
    bool ui_active;
    bool audio_active;
    bool rftest_active;
} pm_scenario_ctx_t;

#if defined(CONTEXT_BACKUP_COMPRESSION_ENABLED) && defined(BSP_USING_PSRAM)
    #error "It's not recommended to enable compression when PSRAM is used"
#endif


#ifdef SOC_BF0_HCPU
    #ifdef BSP_USING_PSRAM
        EXEC_REGION_DEF(RW_PSRAM_NON_RET);
        #define PM_RETENTION_RAM_START_ADDR (EXEC_REGION_START_ADDR(RW_PSRAM_NON_RET))
        #define PM_RETENTION_RAM_SIZE       (PSRAM_BASE + PSRAM_SIZE - (uint32_t)PM_RETENTION_RAM_START_ADDR)
    #else
        EXEC_REGION_DEF(RW_IRAM_RET$$ZI);
        #define PM_RETENTION_RAM_START_ADDR (EXEC_REGION_END_ADDR(RW_IRAM_RET$$ZI))
        #define PM_RETENTION_RAM_SIZE       (HPSYS_RETM_BASE + HPSYS_RETM_SIZE - (uint32_t)PM_RETENTION_RAM_START_ADDR)

    #endif

    EXEC_REGION_DEF(RW_IRAM1);
    #define PM_BACKUP_REGION_START_ADDR (EXEC_REGION_START_ADDR(RW_IRAM1))
    #define PM_BACKUP_REGION_SIZE       ((uint32_t)HEAP_BEGIN - (uint32_t)PM_BACKUP_REGION_START_ADDR)

#endif

RETM_BSS_SECT_BEGIN(pm_reg)
__ROM_USED pm_reg_ctx_t pm_reg_ctx RETM_BSS_SECT(pm_reg);
__ROM_USED uint32_t pm_init_sp RETM_BSS_SECT(pm_reg);
RETM_BSS_SECT_END

#ifdef PM_PROFILING_ENABLED
    RETM_BSS_SECT_BEGIN(test_pm_data)
    static test_pm_data_t test_pm_data RETM_BSS_SECT(test_pm_data);
    RETM_BSS_SECT_END
#endif /* PM_PROFILING_ENABLED */

#ifdef __CC_ARM
    #pragma arm section rwdata="UNINITZI"
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang section data="UNINITRW"
#else
#endif
rt_device_t g_t_hwtimer;
#ifdef __CC_ARM
    #pragma arm section rwdata
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang section data=""
#else
#endif

#define SIFLI_LP_RAM   0x0
#define DEFAULT_LPTIM_FREQ  (8192)

__ROM_USED uint32_t g_sbcr;
__ROM_USED uint32_t g_dscr;
__ROM_USED uint32_t g_lscr;
#if defined(BSP_PM_FREQ_SCALING)
static bool freq_scaling_enabled = true;
#ifndef PM_HW_DEEP_WFI_SUPPORT
static pm_freq_scaling_param_t pm_freq_scaling_param = {.sys_clk_src = -1};
#endif /* !PM_HW_DEEP_WFI_SUPPORT */
#endif /* BSP_PM_FREQ_SCALING */


__ROM_USED uint32_t iser_bak[16];


RT_WEAK const pm_policy_t pm_policy[] =
{
#ifdef PM_STANDBY_ENABLE
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_STANDBY},
#else
    {10, PM_SLEEP_MODE_STANDBY},
#endif /* SOC_BF0_HCPU */
#elif defined(PM_DEEP_ENABLE)
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_DEEP},
#else
    {10, PM_SLEEP_MODE_DEEP},
#endif /* SOC_BF0_HCPU */
#else
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_LIGHT},
#else
    {15, PM_SLEEP_MODE_LIGHT},
#endif /* SOC_BF0_HCPU */
#endif /* PM_STANDBY_ENABLE */
};



#ifdef RT_USING_PM

/** last low power mode  */
__ROM_USED uint32_t g_power_mode;
uint32_t g_from_sdby;

PM_NON_RETENTION_SECTION_BEGIN

/** wakeup source */
__ROM_USED uint32_t g_wakeup_src;

/** power on mode */
__ROM_USED pm_power_on_mode_t g_pwron_mode;
PM_NON_RETENTION_SECTION_END



#ifdef PM_METRICS_ENABLED
    static pm_stat_t    pm_stat;
#endif /* PM_METRICS_ENABLED */


#ifdef PM_METRICS_USE_COLLECTOR
static mc_collector_t   pm_metrics_collector;

#define PM_DEBUG_METRICS_THRESH_FACTOR (1)

static const int32_t    pm_debug_metrics_report_thresh_tbl[PM_DEBUG_METRICS_REPORT_THRESH_TBL_SIZE] =
{
    5 * 60 * RT_TICK_PER_SECOND / PM_DEBUG_METRICS_THRESH_FACTOR,
    5 * 60 * RT_TICK_PER_SECOND / PM_DEBUG_METRICS_THRESH_FACTOR,
    10 * 60 * RT_TICK_PER_SECOND / PM_DEBUG_METRICS_THRESH_FACTOR,
    30 * 60 * RT_TICK_PER_SECOND / PM_DEBUG_METRICS_THRESH_FACTOR,
};

static pm_debug_metrics_ctx_t pm_debug_metrics_ctx;

#endif /* PM_METRICS_USE_COLLECTOR */


static pm_tick_cal_t pm_tick_cal;
static pm_scenario_ctx_t pm_scenario_ctx;

#ifndef BSP_PM_PIN_BACKUP_DISABLED
#ifdef SOC_BF0_HCPU
    #define SAVE_GPIO_INSTANCE hwp_gpio1
    #define SAVE_GPIO_INSTANCE_SIZE (GPIO1_PIN_NUM/32)
#else
    #define SAVE_GPIO_INSTANCE hwp_gpio2
    #define SAVE_GPIO_INSTANCE_SIZE (GPIO2_PIN_NUM/32)
#endif /* SOC_BF0_HCPU */

RETM_BSS_SECT_BEGIN(gpio_save_buf)
static GPIOxRestore_TypeDef gpio_save_buf[SAVE_GPIO_INSTANCE_SIZE] RETM_BSS_SECT(gpio_save_buf);
static PIN_BackupBufTypeDef pinmux_save_buf RETM_BSS_SECT(pinmux_save_buf);
RETM_BSS_SECT_END

L1_RET_CODE_SECT(pm_pin_retore, void pm_pin_restore(void))
{
    HAL_StatusTypeDef err;

    err = HAL_PIN_Restore(&pinmux_save_buf);
    RT_ASSERT(HAL_OK == err);

    err = HAL_GPIO_Restore(SAVE_GPIO_INSTANCE, &gpio_save_buf[0], SAVE_GPIO_INSTANCE_SIZE);

    RT_ASSERT(HAL_OK == err);
}

L1_RET_CODE_SECT(pm_pin_backeup, void pm_pin_backup(void))
{
    HAL_StatusTypeDef err;

    err = HAL_PIN_Backup(&pinmux_save_buf);
    RT_ASSERT(HAL_OK == err);

    err = HAL_GPIO_Save(SAVE_GPIO_INSTANCE, &gpio_save_buf[0], SAVE_GPIO_INSTANCE_SIZE);

    RT_ASSERT(HAL_OK == err);
}
#endif /* BSP_PM_PIN_BACKUP_DISABLED */

void rt_hw_systick_init(void);

#if defined(USING_CONTEXT_BACKUP) && defined(SOC_BF0_HCPU)
    int32_t pm_init_mem_map();
#endif /* USING_CONTEXT_BACKUP && SOC_BF0_HCPU */

void rt_flash_wait_idle(uint32_t addr);
void rt_psram_wait_idle(char *name);


/**
   @brief Resume device in specific mode
   @param[in] device device to be resumed
   @param[in] mode Current power mode
*/
void sifli_resume(const struct rt_device *device, uint8_t mode);


#ifdef SOC_BF0_HCPU
    static
#endif
void restore_context(void);

#ifndef SF32LB55X
    extern uint32_t __Vectors;
#endif /* SF32LB55X */


__WEAK void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
}


__WEAK void BSP_Power_Up(bool is_deep_sleep)
{
}


L1_RET_CODE_SECT(soc_power_down, __WEAK void soc_power_down(void))
{
#if defined(SF32LB58X)
#ifdef SOC_BF0_HCPU
#else

#ifdef BSP_USING_NOR_FLASH5
    FLASH_HandleTypeDef *flash_handle;
    flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI5_MEM_BASE);
    HAL_FLASH_DEEP_PWRDOWN(flash_handle);
    // wait tDP
    HAL_Delay_us(3);
#endif /* BSP_USING_NOR_FLASH5 */

#endif /* SOC_BF0_HCPU */
#endif /* SF32LB58X */


#ifdef SF32LB56X
#ifdef SOC_BF0_HCPU
#else

#ifdef BSP_USING_NOR_FLASH5
    FLASH_HandleTypeDef *flash_handle;
    flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI5_MEM_BASE);
    HAL_FLASH_DEEP_PWRDOWN(flash_handle);
    // wait tDP
    HAL_Delay_us(3);
#endif /* BSP_USING_NOR_FLASH5 */

#endif /* SOC_BF0_HCPU */
#endif /* SF32LB56X */

}

L1_RET_CODE_SECT(soc_power_up, __WEAK void soc_power_up(void))
{
#if defined(SF32LB58X)
#ifdef SOC_BF0_HCPU
#else

#ifdef BSP_USING_NOR_FLASH5
    FLASH_HandleTypeDef *flash_handle;
    flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI5_MEM_BASE);
    HAL_FLASH_RELEASE_DPD(flash_handle);
    /* wait for complete */
    HAL_Delay_us(8);
#endif /* BSP_USING_NOR_FLASH5 */

#endif /* SOC_BF0_HCPU */
#endif /* SF32LB58X */


#ifdef SF32LB56X
#ifdef SOC_BF0_HCPU
#else

#ifdef BSP_USING_NOR_FLASH5
    FLASH_HandleTypeDef *flash_handle;
    flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI5_MEM_BASE);
    HAL_FLASH_RELEASE_DPD(flash_handle);
    /* wait for complete */
    HAL_Delay_us(8);
#endif /* BSP_USING_NOR_FLASH5 */

#endif /* SOC_BF0_HCPU */
#endif /* SF32LB56X */
}


#if defined(__CLANG_ARM) || defined(__GNUC__)
static void save_core_greg(void)
{
    pm_reg_ctx.psr = __get_xPSR();
    pm_reg_ctx.primask = __get_PRIMASK();
    pm_reg_ctx.faultmask = __get_FAULTMASK();
    pm_reg_ctx.basepri = __get_BASEPRI();
    pm_reg_ctx.control = __get_CONTROL();
    pm_reg_ctx.psp = __get_PSP();
    pm_reg_ctx.msp = __get_MSP();
    pm_reg_ctx.psplim = __get_PSPLIM();
    pm_reg_ctx.msplim = __get_MSPLIM();
    pm_reg_ctx.greg[13] = pm_reg_ctx.psp; //SP

    __asm
    (
        "PUSH    {r1}                    \n" /* push r1 */
        "MOV     r1, %[ctx]              \n"
        "STM     r1!, {r0}               \n" /* save r0 */
        "MOV     r0, r1                  \n"
        "POP     {r1}                    \n" /* pop r1  */
        "STM     r0!, {r1}               \n" /* save r1 */
        "MOV     r1, r0                  \n"
        "STM     r1!, {r2 - r12}         \n" /* save r2 - r12 register */
        : /* Outputs */
        : [ctx] "r"(&pm_reg_ctx.greg[0])/* Inputs */
        : "r0", "r1" /* Clobbers */
    );
}

static void restore_core_greg(void)
{
    __asm
    (
        "MSR     apsr,   %[psr_val]      \n"
#if !defined(SF32LB52X) || !defined(SOC_BF0_LCPU)  //workaround build error if DSP_FPU is not present
        "MSR     apsr_g, %[psr_val]      \n"
#endif
        "MSR     psp,    %[psp_val]      \n"
        "MOV     r1, %[pc_val]           \n"
        "PUSH    {r1}                    \n" /* push pc */
        "MOV     r1, %[ctx]              \n"
        "LDM     r1!, {r0}               \n" /* restore r0 */
        "PUSH    {r0}                    \n" /* push r0 */
        "MOV     r0, r1                  \n"
        "LDM     r0!, {r1 - r12}         \n" /* restore r1 - r12 register */
        "LDR     lr,  [r0, #4]           \n" /* restore lr */
        "POP     {r0}                    \n"
        "POP     {pc}                    \n" /* pop pc */
        : /* Outputs */
        : [ctx] "r"(&pm_reg_ctx.greg[0]), [pc_val] "r"(pm_reg_ctx.pc),
        [psr_val] "r"(pm_reg_ctx.psr),  [psp_val] "r"(pm_reg_ctx.psp) /* Inputs */
        : "r0", "r1" /* Clobbers */
    );

}

static void restore_core_reg(void)
{

    //TODO: not restored
    //pm_reg_ctx.psr = __get_xPSR();
    __set_PRIMASK(pm_reg_ctx.primask);
    __set_BASEPRI(pm_reg_ctx.basepri);
    __set_FAULTMASK(pm_reg_ctx.faultmask);
    __set_CONTROL(pm_reg_ctx.control);
    __set_PSP(pm_reg_ctx.psp);
    __set_PSPLIM(pm_reg_ctx.psplim);
    __set_MSP(pm_reg_ctx.msp);
    __set_MSPLIM(pm_reg_ctx.msplim);

    NVIC_EnableIRQ(AON_IRQn);

    restore_core_greg();
}

#endif



#if defined(SOC_BF0_HCPU)
void rt_application_init_power_on_mode(void)
{
    PMU_BootModeTypeDef pmu_boot_mode;
    uint32_t pmu_wakeup_src;

    if (PM_COLD_BOOT == g_pwron_mode)
    {
        if (HAL_OK == HAL_PMU_CheckBootMode(&pmu_boot_mode, &pmu_wakeup_src))
        {
            if (PMU_SHUTDOWN_BOOT == pmu_boot_mode)
            {
                g_pwron_mode = PM_SHUTDOWN_BOOT;
            }
            else if (PMU_HIBERNATE_BOOT == pmu_boot_mode)
            {
                g_pwron_mode = PM_HIBERNATE_BOOT;
            }
            else if (PMU_REBOOT_BOOT == pmu_boot_mode)
            {
                g_pwron_mode = PM_REBOOT_BOOT;
            }
            else
            {
                /* cold boot, do nothing */;
            }
            g_wakeup_src = pmu_wakeup_src;
        }
    }
}

__WEAK void ram_ro_reload(void)
{
    uint32_t exec_addr;
    uint32_t load_addr;
    uint32_t len;

#ifdef HPSYS_ITCM_BASE
    exec_addr = (uint32_t)EXEC_REGION_START_ADDR(ER_ITCM$$RO);
    load_addr = (uint32_t)EXEC_REGION_LOAD_START_ADDR(ER_ITCM$$RO);
    len = (uint32_t)EXEC_REGION_END_ADDR(ER_ITCM$$RO) - (uint32_t)EXEC_REGION_START_ADDR(ER_ITCM$$RO);
    if (len > 0)
    {
        memcpy((void *)exec_addr, (void *)load_addr, len);
    }
#endif
    exec_addr = (uint32_t)EXEC_REGION_START_ADDR(ER_IROM1_EX$$RO);
    load_addr = (uint32_t)EXEC_REGION_LOAD_START_ADDR(ER_IROM1_EX$$RO);
    len = (uint32_t)EXEC_REGION_END_ADDR(ER_IROM1_EX$$RO) - (uint32_t)EXEC_REGION_START_ADDR(ER_IROM1_EX$$RO);
    if (len > 0)
    {
        memcpy((void *)exec_addr, (void *)load_addr, len);
    }
}

#ifndef SF32LB55X
void SystemInitFromStandby(void)
{
    if (AON_PMR_STANDBY != HAL_HPAON_GET_POWER_MODE())
    {
        return;
    }

    __set_MSP(pm_init_sp);
    __set_MSPLIM(pm_reg_ctx.msplim);

#ifdef PM_PROFILING_ENABLED
    test_pm_data.restore_time.init_enter = HAL_GTIMER_READ();
    test_pm_data.restore_time.power_rdy = 0;
#endif /* PM_PROFILING_ENABLED */

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

    g_pwron_mode = PM_STANDBY_BOOT;

    // Recovery VHP at the very beginning
    HAL_HPAON_ENABLE_VHP();

    /* Reset sysclk_m used by HAL_Delay_us as sram is not restored yet */
    HAL_Delay_us(0);

#ifndef BSP_PM_PIN_BACKUP_DISABLED
    /* restore pinmux and gpio register */
    pm_pin_restore();

    /* disable ISO as pinmux and gpio have been restored */
    HAL_HPAON_ENABLE_PAD();
#endif /* BSP_PM_PIN_BACKUP_DISABLED */

#ifndef SF32LB52X
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    /* delay 1ms to workaround crash issue which might be caused by unstable state when LCPU wakeup*/
    HAL_Delay_us(1000);
#endif /* SF32LB52X */

#if defined(RT_DEBUG) && !defined(SF32LB55X)
    __HAL_SYSCFG_Enable_Assert_Trigger(1);
#endif // defined(RT_DEBUG) && !defined(SF32LB55X)


#ifdef PM_PROFILING_ENABLED
    test_pm_data.restore_time.lcpu_wakeup = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

#ifndef LXT_DISABLE
    HAL_PIN_SetXT32();
#endif /* LXT_DISABLE */

    HAL_Init();

#ifdef PM_PROFILING_ENABLED
    test_pm_data.restore_time.hal_init_done = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

    mpu_config();

    SCB_EnableICache();
    SCB_EnableDCache();

#ifdef PM_PROFILING_ENABLED
    test_pm_data.restore_time.mpu_config_done = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

    ram_ro_reload();

#ifdef PM_PROFILING_ENABLED
    test_pm_data.restore_time.ram_code_load_done = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

    //HAL_HPAON_ENABLE_PAD();

    rt_hw_interrupt_disable();

    SystemClock_Config();
    rt_hw_systick_init();

    rt_hw_cfg_pendsv_pri();

    restore_context();
}

void SystemPowerOnModeInit(void)
{
    g_pwron_mode = PM_COLD_BOOT;
}

#else
void SystemPowerOnModeInit(void)
{
    uint32_t mode;
    PMU_BootModeTypeDef pmu_boot_mode;
    uint32_t pmu_wakeup_src;

    mode = HAL_HPAON_GET_POWER_MODE();

    g_pwron_mode = PM_COLD_BOOT;
    if (AON_PMR_STANDBY == mode)
    {
        g_pwron_mode = PM_STANDBY_BOOT;
    }

    HAL_HPAON_ENABLE_PAD();
    //HAL_HPAON_CLEAR_POWER_MODE();

    if (AON_PMR_STANDBY == mode)
    {
#ifndef BSP_PM_PIN_BACKUP_DISABLED
        /* restore pinmux and gpio register */
        pm_pin_restore();
#endif /* BSP_PM_PIN_BACKUP_DISABLED */

        ram_ro_reload();
        /* Reset sysclk_m used by HAL_Delay_us as sram is not restored yet */
        HAL_Delay_us(0);
        HAL_HPAON_WakeCore(CORE_ID_LCPU);
        HAL_Init();

        rt_hw_interrupt_disable();

        SystemClock_Config();
        rt_hw_systick_init();

        rt_hw_cfg_pendsv_pri();

        restore_context();
    }
}

#endif /* SF32LB55X */

#elif defined(SOC_BF0_LCPU)

//__USED uint32_t g_ret_count;
void rt_application_init_power_on_mode(void)
{
}

__ROM_USED void restore_context(void)
{
    restore_core_reg();
}


L1_RET_CODE_SECT(SystemPowerOnInitLCPU, __ROM_USED void SystemPowerOnInitLCPU(void))
{
    uint32_t mode = HAL_LPAON_GET_POWER_MODE();

    g_pwron_mode = PM_COLD_BOOT;
    if (AON_PMR_STANDBY == mode)
    {
        g_pwron_mode = PM_STANDBY_BOOT;
    }

    if (AON_PMR_STANDBY == mode)
    {
#ifdef BSP_USING_WDT
        rt_wdt_restore();
#endif /* BSP_USING_WDT */

#if defined(RT_DEBUG) && !defined(SF32LB55X)
        __HAL_SYSCFG_Enable_Assert_Trigger(1);
#endif // defined(RT_DEBUG) && !defined(SF32LB55X)

#ifndef SF32LB55X
        // Recovery VLP at the beginning
        HAL_LPAON_ENABLE_VLP();
#endif // SF32LB55X

#ifndef BSP_PM_PIN_BACKUP_DISABLED
        /* restore pinmux and gpio register */
        pm_pin_restore();
        /* disable ISO as pinmux and gpio have been restored */
        HAL_LPAON_ENABLE_PAD();
#endif /* BSP_PM_PIN_BACKUP_DISABLED */

#ifndef SF32LB52X
        if (!HAL_LXT_ENABLED())
        {
#ifdef SF32LB55X
            hwp_ble_mac->RCCAL_CTRL &= ~BLE_MAC_RCCAL_CTRL_RCCAL_AUTO;
#else
            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_AUTO;
#endif
        }
#endif
#if defined(BSP_USING_NOR_FLASH) || defined(BSP_USING_SPI_FLASH)
        /* deinit flash driver as hw is reset but sw variable value is unchanged */
        rt_hw_flash_deinit();
#endif /* BSP_USING_NOR_FLASH || BSP_USING_SPI_FLASH */

        HAL_Init();

        //HAL_LPAON_ENABLE_PAD();

        rt_hw_interrupt_disable();
        g_from_sdby = 1;
        SystemClock_Config();
        rt_hw_systick_init();

        rt_hw_cfg_pendsv_pri();

        restore_context();
    }

}



void SystemPowerOnModeInit(void)
{
    SystemPowerOnInitLCPU();
}

#endif // SOC_BF0_HCPU

pm_power_on_mode_t SystemPowerOnModeGet(void)
{
    return g_pwron_mode;
}

#endif // RT_USING_PM


uint32_t pm_get_wakeup_src(void)
{
    return g_wakeup_src;
}

uint32_t pm_get_power_mode(void)
{
    return g_power_mode;
}
#ifdef BSP_USING_CHARGER
/*for charger int wakeup*/
int pm_get_charger_pin_wakeup(void)
{
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;
    gpio = GET_GPIO_INSTANCE(BSP_CHARGER_INT_PIN);
    gpio_pin = GET_GPIOx_PIN(BSP_CHARGER_INT_PIN);

    wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    return wakeup_pin;
}
#endif
#ifdef SOC_BF0_HCPU

#ifndef SF32LB52X
    static void save_ram(void);
    static void restore_ram(void);
#endif /* SF32LB52X */

static void clear_interrupt_setting(void)
{
    uint32_t i;
    for (i = 0; i < 16; i++)
    {
        iser_bak[i] = NVIC->ISER[i];
        NVIC->ICER[i] = 0xFFFFFFFF;
        __DSB();
        __ISB();
    }
}

static void restore_interrupt_setting(void)
{
    uint32_t i;
    for (i = 0; i < 16; i++)
    {
        __COMPILER_BARRIER();
        NVIC->ISER[i] = iser_bak[i];
        __COMPILER_BARRIER();
    }
}


__WEAK void sifli_light_handler(void)
{
    uint32_t dll1_freq;
    uint32_t dll2_freq;
    int clk_src;

    clear_interrupt_setting();

    BSP_IO_Power_Down(CORE_ID_HCPU, false);

#ifndef SF32LB52X
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif /* SF32LB52X */

    NVIC_EnableIRQ(AON_IRQn);

    clk_src = HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);

    dll1_freq = HAL_RCC_HCPU_GetDLL1Freq();
    dll2_freq = HAL_RCC_HCPU_GetDLL2Freq();

    HAL_RCC_HCPU_DisableDLL1();
    HAL_RCC_HCPU_DisableDLL2();

    HAL_HPAON_CLEAR_HP_ACTIVE();
    HAL_HPAON_EnterLightSleep(g_lscr);

    __WFI();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    HAL_HPAON_SET_HP_ACTIVE();
    HAL_HPAON_CLEAR_POWER_MODE();

    HAL_RCC_HCPU_EnableDLL1(dll1_freq);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, clk_src);
    HAL_RCC_HCPU_EnableDLL2(dll2_freq);

#ifndef SF32LB52X
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif /* SF32LB52X */

    BSP_Power_Up(false);

    restore_interrupt_setting();
}

//void rt_flash_wait_idle(uint32_t addr);

__WEAK int32_t sifli_deep_handler(void)
{
    uint32_t dll1_freq;
    uint32_t dll2_freq;
    int clk_src;

    clear_interrupt_setting();

    PM_DEBUG_PIN_TOGGLE();

#ifndef SF32LB52X
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();

    save_ram();

    SCB_InvalidateDCache();
    SCB_InvalidateICache();
#endif /* SF32LB52X */

#ifdef BSP_USING_NOR_FLASH2
    rt_flash_wait_idle(MPI2_MEM_BASE);
#endif /* BSP_USING_NOR_FLASH2 */

#ifdef BSP_USING_NOR_FLASH3
    rt_flash_wait_idle(MPI3_MEM_BASE);
#endif /* BSP_USING_NOR_FLASH3 */

#ifdef SF32LB52X
    BSP_IO_Power_Down(CORE_ID_HCPU, false);
#endif
    NVIC_EnableIRQ(AON_IRQn);

    clk_src = HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);

    dll1_freq = HAL_RCC_HCPU_GetDLL1Freq();
    dll2_freq = HAL_RCC_HCPU_GetDLL2Freq();

    HAL_RCC_HCPU_DisableDLL1();
    HAL_RCC_HCPU_DisableDLL2();

    PM_DEBUG_PIN_TOGGLE();

#ifdef SF32LB52X
    HAL_HPAON_DISABLE_PAD();
    HAL_HPAON_DISABLE_VHP();
#endif // SF32LB52X    

    HAL_HPAON_CLEAR_HP_ACTIVE();
    /* PAD no need to be disabled and enabled when entering deep sleep2 mode,
     * Otherwise glitch may be introduced and result in error in some board.
     * Don't remember why this code is added when implementing deep sleep2 mode.
     *
     */
    //HAL_HPAON_DISABLE_PAD();
    HAL_HPAON_EnterDeepSleep(0);

    __WFI();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

#ifdef SF32LB52X
    HAL_HPAON_ENABLE_PAD();
    HAL_HPAON_ENABLE_VHP();

    PM_DEBUG_PIN_TOGGLE();

#endif // SF32LB52X    

    //HAL_HPAON_ENABLE_PAD();
    HAL_HPAON_SET_HP_ACTIVE();
    HAL_HPAON_CLEAR_POWER_MODE();

    if (0 != dll1_freq)
    {
        while (0 == (hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HXT48_RDY))
        {
            /* wait until HXT48 ready */
        }
    }

    HAL_RCC_HCPU_EnableDLL1(dll1_freq);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, clk_src);
    HAL_RCC_HCPU_EnableDLL2(dll2_freq);
    HAL_Delay_us(0);

#ifdef SF32LB52X
    BSP_Power_Up(false);
#endif // SF32LB52X    

#ifndef SF32LB52X
    HAL_HPAON_WakeCore(CORE_ID_LCPU);

    ram_ro_reload();
    restore_ram();
#endif /* SF32LB52X */

    restore_interrupt_setting();

    return -1;
}

#ifndef SF32LB52X
static void save_ram(void)
{
#if 0
    uint32_t addr;
    uint32_t i;
    addr = pm_mem_map.ret_ram_start_addr + RET_MEM_RESERVED_LEN;
    /* save ram data */
    for (i = 0; i < pm_mem_map.retained_region_num; i++)
    {
        if (pm_mem_map.retained_region[i].len > 0)
        {
            memcpy((void *)addr, (void *)pm_mem_map.retained_region[i].start_addr,
                   pm_mem_map.retained_region[i].len);
            addr += pm_mem_map.retained_region[i].len;
        }
    }
#else
    rt_err_t err;

    pm_init_mem_map();
    err = cb_save_context();
    if (RT_EOK != err)
    {
        /* wakeup LCPU so that SWD is connectable after assertion */
        HAL_HPAON_WakeCore(CORE_ID_LCPU);
        RT_ASSERT(0);
    }

    SCB_CleanDCache();
#endif
}

static void restore_ram(void)
{
#if 0
    uint32_t i;
    uint32_t addr;
    EXT_DMA_HandleTypeDef hdma;
    HAL_StatusTypeDef res;

    addr = pm_mem_map.ret_ram_start_addr + RET_MEM_RESERVED_LEN;
    for (i = 0; i < pm_mem_map.retained_region_num; i++)
    {
        if (pm_mem_map.retained_region[i].len > 0)
        {
#if 1
            hdma.Init.SrcInc = HAL_EXT_DMA_SRC_INC | HAL_EXT_DMA_SRC_BURST16;
            hdma.Init.DstInc = HAL_EXT_DMA_DST_INC | HAL_EXT_DMA_DST_BURST16;
            hdma.Init.cmpr_en = false;
            RT_ASSERT(HAL_OK == HAL_EXT_DMA_Init(&hdma));

            res = HAL_EXT_DMA_Start(&hdma, addr,
                                    pm_mem_map.retained_region[i].start_addr,
                                    (pm_mem_map.retained_region[i].len + 3) >> 2);
            RT_ASSERT(HAL_OK == res);

            res = HAL_EXT_DMA_PollForTransfer(&hdma, HAL_EXT_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
            RT_ASSERT(HAL_OK == res);
#else
            memcpy((void *)pm_mem_map.retained_region[i].start_addr, (void *)addr,
                   pm_mem_map.retained_region[i].len);
            addr += pm_mem_map.retained_region[i].len;
#endif
        }
    }
    pm_reg_ctx.pc = *(uint32_t *)(pm_mem_map.ret_ram_start_addr + RET_MEM_PC_OFFSET);
    pm_reg_ctx.greg[14] = *(uint32_t *)(pm_mem_map.ret_ram_start_addr + RET_MEM_LR_OFFSET);
    pm_reg_ctx.psp = *(uint32_t *)(pm_mem_map.ret_ram_start_addr + RET_MEM_SP_OFFSET);
#else

    rt_err_t err;

    err = cb_restore_context();
    RT_ASSERT(RT_EOK == err);

    cb_deinit();
#endif
}
#endif /* !SF32LB52X */


static void restore_context(void)
{
#ifndef SF32LB52X
    restore_ram();
#endif /* SF32LB52X */

    pm_reg_ctx.pc = pm_reg_ctx.reserved[RET_MEM_PC_OFFSET / sizeof(uint32_t)];
    pm_reg_ctx.greg[14] = pm_reg_ctx.reserved[RET_MEM_LR_OFFSET / sizeof(uint32_t)];
    pm_reg_ctx.psp = pm_reg_ctx.reserved[RET_MEM_SP_OFFSET / sizeof(uint32_t)];

#ifdef PM_PROFILING_ENABLED
    test_pm_data.restore_time.restore_ram_done = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

    restore_core_reg();
}

__WEAK int sifli_standby_handler(void)
{
    uint32_t pendsv_set;
    uint32_t dll1_freq;
    uint32_t dll2_freq;
    int clk_src;
    // begin: test code
    //static bool not_first;
    // end: test code


    /* save pendsv state */
    pendsv_set = SCB->ICSR & SCB_ICSR_PENDSVSET_Msk;
    /* clear pending pendsv to avoid from failing to enter low power mode*/
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;

#ifndef SF32LB55X
    hwp_hpsys_aon->RESERVE0 = (uint32_t)SystemInitFromStandby;
#endif /* SF32LB55X */

    clear_interrupt_setting();

    NVIC_EnableIRQ(AON_IRQn);

    clk_src = HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS);
    dll1_freq = HAL_RCC_HCPU_GetDLL1Freq();
    dll2_freq = HAL_RCC_HCPU_GetDLL2Freq();

#ifdef PM_PROFILING_ENABLED
    test_pm_data.save_time.save_start = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

#ifndef SF32LB52X
    save_ram();
#endif /* SF32LB52X */
#ifdef PM_PROFILING_ENABLED
    test_pm_data.save_time.save_done = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

    BSP_IO_Power_Down(CORE_ID_HCPU, true);
#ifndef BSP_PM_PIN_BACKUP_DISABLED
    /* save the latest state so it can be restored after wakeup */
    pm_pin_backup();
#endif /* BSP_PM_PIN_BACKUP_DISABLED */

#ifndef SF32LB52X
    /* cancel active request after bsp_power_down because bsp_power_down may access LPSYS */
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif /* SF32LB52X */

    // begin: test code
#if 0
    if (!not_first)
    {
        not_first = true;
        hwp_lpsys_aon->PMR = 0x80000007;
        HAL_LPAON_DISABLE_PAD();
#ifndef SF32LB55X
        HAL_LPAON_DISABLE_VLP();
#endif //SF32LB55X        
    }
#endif


    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
    HAL_HPAON_EnterStandby(g_sbcr);

    save_core_greg();

    __asm
    (
        "MOV     r0, pc                  \n"
        "ADDS    r0, #55                 \n" /* point to NOP after WFI */
        "STR     r0, [%[pc_val]]         \n" /* save pc */
        "STR     lr, [%[pc_val], #4]     \n" /* save lr */
        "STR     sp, [%[pc_val], #8]     \n" /* save sp */
        "BL      HAL_RCC_HCPU_DisableDLL1          \n"
        "BL      HAL_RCC_HCPU_DisableDLL2          \n"
        "BL      HAL_HPAON_Deactivate              \n"
        "DSB                             \n"
        "ISB                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "WFI                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        : /* Outputs */
        : [pc_val] "r"(&pm_reg_ctx.reserved), [core_id] "r"(CORE_ID_HCPU), [deep_sleep] "r"(true)  /* Inputs */
        : "r0", "r1" /* Clobbers */
    );

    //HAL_HPAON_ENABLE_PAD();
    HAL_HPAON_SET_HP_ACTIVE();
    HAL_HPAON_CLEAR_POWER_MODE();

    if (clk_src != HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS))
    {
        /* restore clock in case entering low power mode fail, boot sequence doesn't execute */
        HAL_RCC_HCPU_EnableDLL1(dll1_freq);
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, clk_src);
        HAL_RCC_HCPU_EnableDLL2(dll2_freq);
    }

    if (pendsv_set)
    {
        /* restore pendsv if set before entering sleep to trigger context switch */
        SCB->ICSR = pendsv_set;
    }

#ifndef SF32LB55X
    hwp_hpsys_aon->RESERVE0 = 0;
#endif /* SF32LB55X */

#ifndef SF32LB52X
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif /* SF32LB52X */

    BSP_Power_Up(true);
    restore_interrupt_setting();

#ifdef PM_PROFILING_ENABLED
    test_pm_data.restore_time.restore_done = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

    return 0;
}

static void init_default_wakeup_src(void)
{
    HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_LP2HP_REQ, AON_PIN_MODE_HIGH);
    HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_LP2HP_IRQ, AON_PIN_MODE_HIGH);
    HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_LPTIM1, AON_PIN_MODE_HIGH);
#ifdef SF32LB52X
    HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_GPIO1, AON_PIN_MODE_HIGH);
#endif /* SF32LB52X */
}

static void pm_save_wakeup_src(void)
{
    g_wakeup_src = HAL_HPAON_GET_WSR();
}


#elif defined(SOC_BF0_LCPU)

#ifdef BLUETOOTH
    __WEAK int32_t ble_deep_sleep_pre_handler();
    __WEAK void ble_deep_sleep_after_handler();
    __WEAK int32_t ble_standby_sleep_pre_handler();
    __WEAK void ble_standby_sleep_after_handler();
#endif

static void clear_interrupt_setting(void)
{
    uint32_t i;
    for (i = 0; i < 16; i++)
    {
        iser_bak[i] = NVIC->ISER[i];
        NVIC->ICER[i] = 0xFFFFFFFF;
    }
}

static void restore_interrupt_setting(void)
{
    uint32_t i;
    for (i = 0; i < 16; i++)
    {
        NVIC->ISER[i] = iser_bak[i];
    }
}

__WEAK void sifli_light_handler(void)
{
    clear_interrupt_setting();

    /* avoid race condtion for manual wakeup by HAL_HPAON_WakeCore()
     * If WSR.HP2LP_REQ is set, LCPU cannot enter sleep mode,
     * otherwise excpetion would be triggered when accessing LPSYS module
     */
    if (HAL_LPAON_GET_WSR() & HAL_LPAON_GET_WER())
    {
        restore_interrupt_setting();
        return;
    }
    HAL_LPAON_CLEAR_LP_ACTIVE();
    if (HAL_LPAON_GET_WSR() & HAL_LPAON_GET_WER())
    {
        HAL_LPAON_SET_LP_ACTIVE();
        restore_interrupt_setting();
        return;
    }

    HAL_LPAON_EnterLightSleep(g_lscr);

    NVIC_EnableIRQ(AON_IRQn);
    //HAL_LPAON_CLEAR_LP_ACTIVE();

    __WFI();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    HAL_LPAON_SET_LP_ACTIVE();
    HAL_LPAON_CLEAR_POWER_MODE();
    restore_interrupt_setting();
    return;
}
__WEAK int32_t sifli_deep_handler(void)
{
    clear_interrupt_setting();

#ifdef BLUETOOTH
    if (ble_deep_sleep_pre_handler && ble_deep_sleep_pre_handler() != 0)
    {
        restore_interrupt_setting();
        return -1;
    }
#endif

    /* avoid race condtion for manual wakeup by HAL_HPAON_WakeCore()
     * If WSR.HP2LP_REQ is set, LCPU cannot enter sleep mode,
     * otherwise excpetion would be triggered when accessing LPSYS module
     */
    if (HAL_LPAON_GET_WSR() & HAL_LPAON_GET_WER())
    {
        restore_interrupt_setting();
        return -1;
    }
    HAL_LPAON_CLEAR_LP_ACTIVE();
    if (HAL_LPAON_GET_WSR() & HAL_LPAON_GET_WER())
    {
        HAL_LPAON_SET_LP_ACTIVE();
        restore_interrupt_setting();
        return -1;
    }

    BSP_IO_Power_Down(CORE_ID_LCPU, false);
    soc_power_down();

    NVIC_EnableIRQ(AON_IRQn);

#ifndef SF32LB55X
    HAL_LPAON_DISABLE_VLP();
#endif //SF32LB55X

    /* enable it if test ble/bt low power, it may not work on some board */
    //HAL_PMU_SET_BUCK2_LOW_VOLTAGE();

    HAL_LPAON_EnterDeepSleep(g_dscr);

    //HAL_LPAON_CLEAR_LP_ACTIVE();
#ifdef SF32LB52X
    HAL_LPAON_DISABLE_PAD();
#endif /* SF32LB52X */

    __WFI();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();


    //HAL_PMU_SET_BUCK2_HIGH_VOLTAGE();

#ifdef SF32LB52X
    HAL_LPAON_ENABLE_PAD();
#endif /* SF32LB52X */

#ifndef SF32LB55X
    HAL_LPAON_ENABLE_VLP();
#endif //SF32LB55X

    HAL_LPAON_SET_LP_ACTIVE();
    HAL_LPAON_CLEAR_POWER_MODE();


    soc_power_up();
    BSP_Power_Up(false);

#ifdef BLUETOOTH
    ble_deep_sleep_after_handler();
#endif
    restore_interrupt_setting();

    return -1;
}

L1_RET_CODE_SECT(sifli_standby_handler_core, __ROM_USED void sifli_standby_handler_core(void))
{
    clear_interrupt_setting();

    /* avoid race condtion for manual wakeup by HAL_HPAON_WakeCore()
     * If WSR.HP2LP_REQ is set, LCPU cannot enter sleep mode,
     * otherwise excpetion would be triggered when accessing LPSYS module
     */
    if (HAL_LPAON_GET_WSR() & HAL_LPAON_GET_WER())
    {
        restore_interrupt_setting();
        return;
    }
    HAL_LPAON_CLEAR_LP_ACTIVE();
    if (HAL_LPAON_GET_WSR() & HAL_LPAON_GET_WER())
    {
        HAL_LPAON_SET_LP_ACTIVE();
        restore_interrupt_setting();
        return;
    }

    BSP_IO_Power_Down(CORE_ID_LCPU, true);
#ifndef BSP_PM_PIN_BACKUP_DISABLED
    /* save the latest state so it can be restored after wakeup */
    pm_pin_backup();
#endif /* BSP_PM_PIN_BACKUP_DISABLED */
    soc_power_down();

    NVIC_EnableIRQ(AON_IRQn);
    //HAL_LPAON_CLEAR_LP_ACTIVE();
    HAL_LPAON_DISABLE_PAD();
#ifndef SF32LB55X
    HAL_LPAON_DISABLE_VLP();
#endif //SF32LB55X
#ifndef PM_WAKEUP_PIN_AS_OUTPUT_IN_SLEEP
    /* Enable PB_AON_ISO if wakeup pin is not used as output pin  */
    HAL_LPAON_DISABLE_AON_PAD();
#endif /* PM_WAKEUP_PIN_AS_OUTPUT_IN_SLEEP */

    HAL_LPAON_EnterStandby(g_sbcr);

    save_core_greg();

    __asm
    (
        "MOV     r0, pc                  \n"
        "ADDS    r0, #45                 \n" /* point to NOP after WFI */
        "STR     r0, [%[pc_val]]         \n" /* save pc */
        "STR     lr, [%[lr_val]]         \n" /* save lr */
        "STR     sp, [%[psp_val]]        \n" /* save sp */
        "DSB                             \n"
        "ISB                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "WFI                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        "NOP                             \n"
        : /* Outputs */
        : [pc_val] "r"(&pm_reg_ctx.pc), [lr_val] "r"(&pm_reg_ctx.greg[14]),
        [psp_val] "r"(&pm_reg_ctx.psp) /* Inputs */
        : "r0", "r1" /* Clobbers */
    );

    //HAL_LPAON_ENABLE_PAD();
    HAL_LPAON_SET_LP_ACTIVE();
    HAL_LPAON_CLEAR_POWER_MODE();

    soc_power_up();
    BSP_Power_Up(true);

    restore_interrupt_setting();


}

static uint32_t sifli_standby_check_bt_sleep_enable(void)
{
    uint32_t ret = 0;
#ifndef SF32LB55X
    ret = hwp_lpsys_aon->WER & LPSYS_AON_WER_BT;
#else
    ret = hwp_lpsys_aon->WER & LPSYS_AON_WER_BLE;
#endif //!SF32LB55X
    return ret;
}

__WEAK int sifli_standby_handler(void)
{

#ifdef BLUETOOTH
    if ((sifli_standby_check_bt_sleep_enable() != 0) && ble_standby_sleep_pre_handler && ble_standby_sleep_pre_handler() != 0)
    {
        return -1;
    }
#endif

    sifli_standby_handler_core();
#ifdef BLUETOOTH
    if ((sifli_standby_check_bt_sleep_enable() != 0) && g_from_sdby)
    {
        ble_standby_sleep_after_handler();
        g_from_sdby = 0;
    }
#endif


    return 0;
}

__ROM_USED void init_default_wakeup_src(void)
{
    HAL_LPAON_EnableWakeupSrc(LPSYS_WAKEUP_SRC_LPTIM, AON_PIN_MODE_HIGH);
#ifdef BLUETOOTH
#ifdef SF32LB55X
    HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_BLE, AON_PIN_MODE_HIGH);
#else
    HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_BT, AON_PIN_MODE_HIGH);
#endif
#endif
    HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_HP2LP_REQ, AON_PIN_MODE_HIGH);
    HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_HP2LP_IRQ, AON_PIN_MODE_HIGH);
}

static void pm_save_wakeup_src(void)
{
    g_wakeup_src = HAL_LPAON_GET_WSR();
}

#else
#error "Core not defined"
#endif

__WEAK void sifli_shutdown_handler(void)
{
    return;
}

#define SIFLI_TIMER_MASK (0xfc)     // Sleep timer is for all sleep mode

rt_uint32_t g_sleep_tick;

#ifndef PM_LP_TIMER_DISABLE
static rt_err_t hwtm_rx_ind(rt_device_t dev, rt_size_t size)
{
    //LOG_D("HW timeout at tick %d, slept for %d", rt_tick_get(), g_sleep_tick);
    return RT_EOK;
}
#endif

#if defined(PM_METRICS_ENABLED)

static void pm_update_wakeup_src_stat(void)
{
    uint32_t i;
    uint32_t wakeup_src = g_wakeup_src;
    pm_wakeup_src_stat_t *stat;

    stat = &pm_stat.wakeup_src_stat[0];
    for (i = 0; (i < pm_stat.wakeup_src_num) && wakeup_src; i++, stat++)
    {
        if ((1UL << stat->wakeup_src) & wakeup_src)
        {
            stat->wakeup_times++;
            stat->run_time += (float)(pm_tick_cal.sleep_start_time - pm_tick_cal.wakeup_time) / HAL_LPTIM_GetFreq();
            wakeup_src &= ~(1UL << stat->wakeup_src);
        }
    }

    for (i = 0; wakeup_src && (i < 32); i++)
    {
        if ((1UL << i) & wakeup_src)
        {
            /* add new wakeup source */
            RT_ASSERT(pm_stat.wakeup_src_num < PM_METRICS_MAX_WAKEUP_SRC_NUM);
            stat = &pm_stat.wakeup_src_stat[pm_stat.wakeup_src_num];
            stat->wakeup_src = i;
            stat->wakeup_times = 1;
            stat->run_time = (float)(pm_tick_cal.sleep_start_time - pm_tick_cal.wakeup_time) / HAL_LPTIM_GetFreq();

            wakeup_src &= ~(1UL << i);
            pm_stat.wakeup_src_num++;
        }
    }

    pm_stat.run_time[pm_stat.curr_run_mode] += HAL_GTIMER_READ() - pm_stat.run_start_time[pm_stat.curr_run_mode];
}

#endif /* PM_METRICS_ENABLED */


static void sifli_sleep(struct rt_pm *pm, uint8_t mode)
{
#if defined(PM_METRICS_ENABLED) || defined(BSP_PM_DEBUG)
    uint32_t start_time;
    uint32_t end_time;
#endif /* PM_METRICS_ENABLED || BSP_PM_DEBUG */

    g_power_mode = mode;

#if defined(PM_METRICS_ENABLED) || defined(BSP_PM_DEBUG)
    start_time = HAL_GTIMER_READ();
#endif /* PM_METRICS_ENABLED || BSP_PM_DEBUG */

#ifdef PM_METRICS_USE_COLLECTOR
    pm_debug_metrics_ctx.last_sleep_tick = rt_tick_get();
    pm_debug_metrics_ctx.report_thresh = 0;
#endif /* PM_METRICS_USE_COLLECTOR */

#ifdef PM_METRICS_ENABLED
    pm_update_wakeup_src_stat();
#endif /* PM_METRICS_ENABLED */


#ifdef BSP_PM_DEBUG
    rt_kprintf("[pm]S:%d,%d\n", mode, start_time);
#endif /* BSP_PM_DEBUG */

    switch (mode)
    {
    case PM_SLEEP_MODE_SHUTDOWN:
    {
        sifli_shutdown_handler();
        break;
    }
    case PM_SLEEP_MODE_STANDBY:
    {
        struct rt_object_information *info;
        struct rt_list_node *list;
        struct rt_device *device;
        struct rt_device *pin_device;
        struct rt_list_node *node;
        int r;

        info = rt_object_get_information(RT_Object_Class_Device);

        list = &info->object_list;

#ifdef PM_PROFILING_ENABLED
        test_pm_data.save_time.device_suspend_begin = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

        for (node = list->next; node != list; node = node->next)
        {
            device = (struct rt_device *)(rt_list_entry(node, struct rt_object, list));
            rt_device_control(device, RT_DEVICE_CTRL_SUSPEND, (void *)(uint32_t)mode);
        }

#ifdef PM_PROFILING_ENABLED
        test_pm_data.save_time.device_suspend_done = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

        r = sifli_standby_handler();

        pm_tick_cal.wakeup_time = HAL_GTIMER_READ();

#ifdef PM_PROFILING_ENABLED
        test_pm_data.restore_time.device_resume_begin = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

        /*Resume pin device before release PAD ISO*/
        pin_device = rt_device_find("pin");
        if (pin_device)
            rt_device_control(pin_device, RT_DEVICE_CTRL_RESUME, (void *)(uint32_t)mode);

#ifdef SOC_BF0_HCPU
        HAL_HPAON_ENABLE_PAD();
#endif

#ifdef SOC_BF0_LCPU
        HAL_LPAON_ENABLE_PAD();
        HAL_GPIO_ClearInterrupt(hwp_gpio2);
#endif /* SOC_BF0_LCPU */

#if !defined(SF32LB56X) || defined(SOC_BF0_HCPU)
#if defined(BSP_USING_NOR_FLASH) || defined(BSP_USING_SPI_FLASH)
        if (0 == r)
        {
            rt_hw_flash_deinit();
            rt_hw_nand_deinit();
            rt_hw_flash_init();
        }
#endif
#endif /* SF32LB56X */

        /*And then resume other devices*/
        for (node = list->next; node != list; node = node->next)
        {
            device = (struct rt_device *)(rt_list_entry(node, struct rt_object, list));

            if (device != pin_device)
                rt_device_control(device, RT_DEVICE_CTRL_RESUME, (void *)(uint32_t)mode);
        }

#if !defined(SF32LB56X) || defined(SOC_BF0_HCPU)
#if defined(BSP_USING_NOR_FLASH) || defined(BSP_USING_SPI_FLASH)
        if (0 == r)
        {
            rt_hw_nand_init();
        }
#endif
#endif /* SF32LB56X */

#ifdef USING_IPC_QUEUE
        ipc_queue_restore_all();
#ifdef SOC_BF0_LCPU
        ipc_queue_restore_all_rom();
#endif /* SOC_BF0_LCPU */
#endif /* USING_IPC_QUEUE */

#ifdef PM_PROFILING_ENABLED
        test_pm_data.restore_time.device_resume_done = HAL_GTIMER_READ();
#endif /* PM_PROFILING_ENABLED */

        break;
    }
    case PM_SLEEP_MODE_DEEP:
    {
        int32_t i = sifli_deep_handler();
        pm_tick_cal.wakeup_time = HAL_GTIMER_READ();
        if (i >= 0)
            return;
        break;
    }
    case PM_SLEEP_MODE_LIGHT:
        sifli_light_handler();
        pm_tick_cal.wakeup_time = HAL_GTIMER_READ();
        break;
    default:
        break;
    }
    //LOG_D("wk, N %x", NVIC->ISPR[0U]);

#if defined(PM_METRICS_ENABLED) || defined(BSP_PM_DEBUG)
    end_time = HAL_GTIMER_READ();
#endif /* PM_METRICS_ENABLED || BSP_PM_DEBUG */

    pm_save_wakeup_src();

#if defined(PM_METRICS_ENABLED)
    pm_stat.sleep_time += (float)(end_time - start_time) / HAL_LPTIM_GetFreq();
    pm_stat.total_wakeup_times += 1;
    pm_stat.run_start_time[pm_stat.curr_run_mode] = end_time;
#endif /* PM_METRICS_ENABLED */

#ifdef BSP_PM_DEBUG
    rt_kprintf("[pm]W:%d\n", end_time);
#endif /* BSP_PM_DEBUG */
}



L1_RET_CODE_SECT(sifli_pm_run, __WEAK void sifli_pm_run(struct rt_pm *pm, uint8_t mode))
{
#if defined(SF32LB52X) && defined(SOC_BF0_HCPU)

    HAL_StatusTypeDef status;
    rt_base_t level;

    level = rt_hw_interrupt_disable();

#ifdef PM_METRICS_ENABLED
    if (mode != pm_stat.curr_run_mode)
    {
        pm_stat.run_time[pm_stat.curr_run_mode] += HAL_GTIMER_READ() - pm_stat.run_start_time[pm_stat.curr_run_mode];
    }
#endif /* PM_METRICS_ENABLED */

#ifdef BSP_USING_PSRAM1
    rt_psram_wait_idle("psram1");
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_PSRAM2
    rt_psram_wait_idle("psram2");
#endif /* BSP_USING_PSRAM2 */

#ifdef BSP_USING_NOR_FLASH2
    rt_flash_wait_idle(MPI2_MEM_BASE);
#endif /* BSP_USING_NOR_FLASH2 */

#ifdef BSP_USING_NOR_FLASH3
    rt_flash_wait_idle(MPI3_MEM_BASE);
#endif /* BSP_USING_NOR_FLASH3 */


    switch (mode)
    {
    case PM_RUN_MODE_HIGH_SPEED:
    {
        status = HAL_RCC_HCPU_ConfigHCLK(240);
        RT_ASSERT(HAL_OK == status);
        status = HAL_RCC_HCPU_EnableDLL2(288000000);
        RT_ASSERT(HAL_OK == status);

#ifdef BSP_USING_PSRAM1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM1, RCC_CLK_PSRAM_DLL2);
        BSP_SetFlash1DIV(2);
        rt_psram_init();
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_NOR_FLASH1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
        BSP_SetFlash1DIV(5);
        BSP_Flash_hw1_init();
#endif /* BSP_USING_NOR_FLASH1 */

#ifdef BSP_USING_NOR_FLASH2
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
        BSP_SetFlash2DIV(5);
        BSP_Flash_hw2_init();
        //hwp_mpi2->PSCLR = 5;
#endif /* BSP_USING_NOR_FLASH2 */

#ifdef BSP_USING_NAND_FLASH2
//#error "Not supported"
#endif /* BSP_USING_NAND_FLASH2 */

        break;
    }
    case PM_RUN_MODE_NORMAL_SPEED:
    {
        /* switch to sysclk before disabling DLL2 as clock must be present when switching MPI clock */
#ifdef BSP_USING_PSRAM1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM1, RCC_CLK_PSRAM_SYSCLK);
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_NOR_FLASH1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_SYSCLK);
#endif /* BSP_USING_NOR_FLASH1 */

#ifdef BSP_USING_NOR_FLASH2
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_SYSCLK);
#endif /* BSP_USING_NOR_FLASH2 */

        HAL_RCC_HCPU_DisableDLL2();
        status = HAL_RCC_HCPU_ConfigHCLK(144);
        RT_ASSERT(HAL_OK == status);
        status = HAL_RCC_HCPU_EnableDLL2(240000000);
        RT_ASSERT(HAL_OK == status);

#ifdef BSP_USING_PSRAM1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM1, RCC_CLK_PSRAM_DLL2);
        BSP_SetFlash1DIV(2);
        rt_psram_init();
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_NOR_FLASH1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
        BSP_SetFlash1DIV(4);
        BSP_Flash_hw1_init();
#endif /* BSP_USING_NOR_FLASH1 */

#ifdef BSP_USING_NOR_FLASH2
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
        BSP_SetFlash2DIV(4);
        BSP_Flash_hw2_init();
        //hwp_mpi2->PSCLR = 4;
#endif /* BSP_USING_NOR_FLASH2 */

        break;
    }
    case PM_RUN_MODE_MEDIUM_SPEED:
    {
        /* switch to sysclk before disabling DLL2 as clock must be present when switching MPI clock */
#ifdef BSP_USING_PSRAM1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM1, RCC_CLK_PSRAM_SYSCLK);
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_NOR_FLASH1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_SYSCLK);
#endif /* BSP_USING_NOR_FLASH1 */

#ifdef BSP_USING_NOR_FLASH2
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_SYSCLK);
#endif /* BSP_USING_NOR_FLASH2 */

        HAL_RCC_HCPU_DisableDLL2();
        status = HAL_RCC_HCPU_ConfigHCLK(48);
        RT_ASSERT(HAL_OK == status);

#ifdef BSP_USING_PSRAM1
        BSP_SetFlash1DIV(1);
        rt_psram_init();
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_NOR_FLASH1
        BSP_SetFlash1DIV(1);
        BSP_Flash_hw1_init();
#endif /* BSP_USING_NOR_FLASH1 */

#ifdef BSP_USING_NOR_FLASH2
        BSP_SetFlash2DIV(1);
        BSP_Flash_hw2_init();
        //hwp_mpi2->PSCLR = 1;
#endif /* BSP_USING_NOR_FLASH2 */

        break;
    }
    case PM_RUN_MODE_LOW_SPEED:
    {
        /* switch to sysclk before disabling DLL2 as clock must be present when switching MPI clock */
#ifdef BSP_USING_PSRAM1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM1, RCC_CLK_PSRAM_SYSCLK);
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_NOR_FLASH1
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_SYSCLK);
#endif /* BSP_USING_NOR_FLASH1 */

#ifdef BSP_USING_NOR_FLASH2
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_SYSCLK);
#endif /* BSP_USING_NOR_FLASH2 */

        HAL_RCC_HCPU_DisableDLL2();
        status = HAL_RCC_HCPU_ConfigHCLK(24);
        RT_ASSERT(HAL_OK == status);

#ifdef BSP_USING_PSRAM1
        BSP_SetFlash1DIV(2);
        rt_psram_init();
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_NOR_FLASH1
        BSP_SetFlash1DIV(2);
        BSP_Flash_hw1_init();
#endif /* BSP_USING_NOR_FLASH1 */

#ifdef BSP_USING_NOR_FLASH2
        BSP_SetFlash2DIV(2);
        BSP_Flash_hw2_init();
        //hwp_mpi2->PSCLR = 2;
#endif /* BSP_USING_NOR_FLASH2 */

        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

#ifdef PM_METRICS_ENABLED
    pm_stat.run_start_time[mode] = HAL_GTIMER_READ();
    pm_stat.curr_run_mode = mode;
#endif /* PM_METRICS_ENABLED */

    rt_hw_interrupt_enable(level);

#endif /* SF32LB52X && SOC_BF0_HCPU*/
}

#if defined(BSP_PM_FREQ_SCALING)

L1_RET_CODE_SECT(sifli_deep_wfi, static void sifli_deep_wfi(void))
{
#ifdef PM_HW_DEEP_WFI_SUPPORT

#ifdef BSP_USING_PSRAM1
    rt_psram_wait_idle("psram1");
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_PSRAM2
    rt_psram_wait_idle("psram2");
#endif /* BSP_USING_PSRAM2 */

#ifdef BSP_USING_NOR_FLASH2
    rt_flash_wait_idle(MPI2_MEM_BASE);
#endif /* BSP_USING_NOR_FLASH2 */

#ifdef BSP_USING_NOR_FLASH3
    rt_flash_wait_idle(MPI3_MEM_BASE);
#endif /* BSP_USING_NOR_FLASH3 */

    __DSB();
    __ISB();

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
#else

#ifdef SOC_BF0_HCPU
    HAL_RCC_HCPU_GetDiv(&pm_freq_scaling_param.hdiv,
                        &pm_freq_scaling_param.pdiv1,
                        &pm_freq_scaling_param.pdiv2);
    pm_freq_scaling_param.sys_clk_src = HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_SYS);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_RCC_HCPU_SetDiv(48, 0, 1);

    __WFI();

    HAL_RCC_HCPU_SetDiv(pm_freq_scaling_param.hdiv, pm_freq_scaling_param.pdiv1, pm_freq_scaling_param.pdiv2);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, pm_freq_scaling_param.sys_clk_src);
    pm_freq_scaling_param.sys_clk_src = -1;
#else

    HAL_RCC_LCPU_GetDiv(&pm_freq_scaling_param.hdiv,
                        &pm_freq_scaling_param.pdiv1,
                        &pm_freq_scaling_param.pdiv2);
    pm_freq_scaling_param.sys_clk_src = 1;
    HAL_RCC_LCPU_SetDiv(6, 0, 1);

    __WFI();

    HAL_RCC_LCPU_SetDiv(pm_freq_scaling_param.hdiv, pm_freq_scaling_param.pdiv1, pm_freq_scaling_param.pdiv2);
    pm_freq_scaling_param.sys_clk_src = -1;

#endif /* SOC_BF0_HCPU */

#endif /* PM_HW_DEEP_WFI_SUPPORT */

}

#endif /* BSP_PM_FREQ_SCALING */


L1_RET_CODE_SECT(sifli_enter_idle, static void sifli_enter_idle(struct rt_pm *pm))
{
#if defined(BSP_PM_FREQ_SCALING)

    if (!rt_pm_is_hw_device_running() /* no hclk related device is running */
#ifdef USING_IPC_QUEUE
            && ipc_queue_check_idle()  /* ipc queue is empty */
#endif /* USING_IPC_QUEUE */

#ifdef SOC_BF0_LCPU
            /* bt is in sleep */
#ifdef LPSYS_AON_SLP_CTRL_BLE_WKUP_Msk
            && (0 == GET_REG_VAL(hwp_lpsys_aon->SLP_CTRL, LPSYS_AON_SLP_CTRL_BLE_WKUP_Msk, LPSYS_AON_SLP_CTRL_BLE_WKUP_Pos))
#else
            && (0 == GET_REG_VAL(hwp_lpsys_aon->SLP_CTRL, LPSYS_AON_SLP_CTRL_BT_WKUP_Msk, LPSYS_AON_SLP_CTRL_BT_WKUP_Pos))
#endif /* LPSYS_AON_SLP_CTRL_BLE_WKUP_Msk */
#endif /* SOC_BF0_LCPU */

#ifndef PM_HW_DEEP_WFI_SUPPORT
            && (-1 == pm_freq_scaling_param.sys_clk_src)
#endif /* !PM_HW_DEEP_WFI_SUPPORT */
            && freq_scaling_enabled)
    {
#ifdef PM_METRICS_ENABLED
        pm_stat.last_deepwfi_start_time = HAL_GTIMER_READ();
        pm_stat.run_time[pm_stat.curr_run_mode] += HAL_GTIMER_READ() - pm_stat.run_start_time[pm_stat.curr_run_mode];
#endif /* PM_METRICS_ENABLED */

        sifli_deep_wfi();

#ifdef PM_METRICS_ENABLED
        pm_stat.deepwfi_time += HAL_GTIMER_READ() - pm_stat.last_deepwfi_start_time;
        pm_stat.run_start_time[pm_stat.curr_run_mode] = HAL_GTIMER_READ();
#endif /* PM_METRICS_ENABLED */

    }
    else

#endif /* BSP_PM_FREQ_SCALING  */
    {
#ifdef PM_METRICS_ENABLED
        pm_stat.last_wfi_start_time = HAL_GTIMER_READ();
        pm_stat.last_wfi_start_time_valid = true;
        pm_stat.run_time[pm_stat.curr_run_mode] += HAL_GTIMER_READ() - pm_stat.run_start_time[pm_stat.curr_run_mode];
#endif /* PM_METRICS_ENABLED */
    }
}

#if defined(BSP_PM_FREQ_SCALING)

static int freq_scale(int argc, char **argv)
{
    uint32_t en_flag;
    if (argc < 2)
    {
        rt_kprintf("wrong arg\n");
    }

    en_flag = atoi(argv[1]);
    if (en_flag)
    {
        freq_scaling_enabled = true;
    }
    else
    {
        freq_scaling_enabled = false;
    }

    return 0;
}
MSH_CMD_EXPORT(freq_scale, freq scaling);

#endif /* BSP_PM_FREQ_SCALING */


static void sifli_exit_idle(struct rt_pm *pm)
{
#if defined(BSP_PM_FREQ_SCALING) && !defined(PM_HW_DEEP_WFI_SUPPORT)
#ifdef SOC_BF0_HCPU
    if (pm_freq_scaling_param.sys_clk_src > RCC_SYSCLK_HXT48)
    {
        HAL_RCC_HCPU_SetDiv(pm_freq_scaling_param.hdiv, pm_freq_scaling_param.pdiv1, pm_freq_scaling_param.pdiv2);
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, pm_freq_scaling_param.sys_clk_src);
        pm_freq_scaling_param.sys_clk_src = -1;
    }
#else
    if (pm_freq_scaling_param.sys_clk_src != -1)
    {
        HAL_RCC_LCPU_SetDiv(pm_freq_scaling_param.hdiv, pm_freq_scaling_param.pdiv1, pm_freq_scaling_param.pdiv2);
        pm_freq_scaling_param.sys_clk_src = -1;
    }

#endif /* SOC_BF0_HCPU */
#endif /* BSP_PM_FREQ_SCALING && !PM_HW_DEEP_WFI_SUPPORT */

#ifdef PM_METRICS_ENABLED
    if (pm_stat.last_wfi_start_time_valid)
    {
        pm_stat.wfi_time += HAL_GTIMER_READ() - pm_stat.last_wfi_start_time;
        pm_stat.last_wfi_start_time_valid = false;
        pm_stat.run_start_time[pm_stat.curr_run_mode] = HAL_GTIMER_READ();
    }
#endif /* PM_METRICS_ENABLED */
}


__ROM_USED void sifli_timer_start(struct rt_pm *pm, rt_uint32_t timeout)
{
#ifndef PM_LP_TIMER_DISABLE
    rt_hwtimerval_t t;
    rt_hwtimer_t *timer_dev;
    int32_t freq;
    uint32_t max_timeout;
#endif // !PM_LP_TIMER_DISABLE

    PM_DEBUG_PIN_TOGGLE();

    // Stop systick
    SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
#ifndef PM_LP_TIMER_DISABLE
    t.sec = timeout / RT_TICK_PER_SECOND;
    t.usec = timeout % RT_TICK_PER_SECOND;
    t.usec *= (1000000 / RT_TICK_PER_SECOND);
    g_sleep_tick = 0;

    pm_tick_cal.sleep_start_time = HAL_GTIMER_READ();

    timer_dev = (rt_hwtimer_t *)g_t_hwtimer;


    /* find the appropriate freq for sleep time */
    freq = DEFAULT_LPTIM_FREQ;

    if (!HAL_LXT_ENABLED())
    {
        rt_device_control(g_t_hwtimer, HWTIMER_CTRL_FREQ_SET, (void *)&freq);
        timer_dev->freq = freq;
    }
    else
    {
        while (freq > timer_dev->info->minfreq)
        {
            max_timeout = (uint64_t)timer_dev->info->maxcnt * RT_TICK_PER_SECOND / freq;
            if (timeout > max_timeout)
            {
                /* reduce frequency to make counter doesn't overflow before timeout */
                freq >>= 1;
            }
            else
            {
                break;
            }
        }
        if (freq < timer_dev->info->minfreq)
        {
            freq = timer_dev->info->minfreq;
        }

        if (freq != timer_dev->freq)
        {
            rt_device_control(g_t_hwtimer, HWTIMER_CTRL_FREQ_SET, (void *)&freq);
        }
    }

    // timeout_calc split timeout into smaller intervals to increase accuracy,
    // However, sleep timeout value is re-calculate each time before sleep,
    // Use bf0_lptimer_start to sleep as longer as possible, do not split timeout.
#if 0
    rt_device_write(g_t_hwtimer, 0, &t, sizeof(t));
#else
    {
        extern void bf0_lptimer_start(rt_hwtimer_t *timer, uint32_t timeout);
        bf0_lptimer_start((rt_hwtimer_t *)g_t_hwtimer, timeout);
    }
#endif
#endif // !PM_LP_TIMER_DISABLE
}

__ROM_USED void sifli_timer_stop(struct rt_pm *pm)
{
#ifndef PM_LP_TIMER_DISABLE
    rt_device_control(g_t_hwtimer, HWTIMER_CTRL_STOP, NULL);
#endif // !PM_LP_TIMER_DISABLE
    // Enable systick
    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}

__ROM_USED rt_tick_t sifli_timer_get_tick(struct rt_pm *pm)
{
    uint32_t curr_time;
    uint32_t delta_time;
    rt_tick_t new_tick;
    float gtime_freq;
    rt_tick_t base_tick;

#if 0
    rt_hwtimerval_t t;
    rt_device_read(g_t_hwtimer, 0, &t, sizeof(t));
    g_sleep_tick = t.sec * RT_TICK_PER_SECOND + t.usec * RT_TICK_PER_SECOND / 1000000;
#endif

    gtime_freq = HAL_LPTIM_GetFreq();
    /* wait until GTIMER has been latched, will wait for one lp clock at most */
    while (HAL_GTIMER_READ() == pm_tick_cal.wakeup_time)
    {
    }
    curr_time = HAL_GTIMER_READ();
    delta_time = (curr_time - pm_tick_cal.sleep_start_time);

    g_sleep_tick = (uint64_t)delta_time * RT_TICK_PER_SECOND / gtime_freq;

    base_tick = rt_tick_get();
    new_tick = base_tick + g_sleep_tick;

    new_tick = pm_latch_tick(new_tick, curr_time, gtime_freq, (void *)0);
    g_sleep_tick = new_tick - base_tick;

    return g_sleep_tick;
}

struct rt_pm_ops sifli_pm =
{
    .sleep = sifli_sleep,
    .run = sifli_pm_run,
    .enter_idle = sifli_enter_idle,
    .exit_idle = sifli_exit_idle,
    .timer_start = sifli_timer_start,
    .timer_stop = sifli_timer_stop,
    .timer_get_tick = sifli_timer_get_tick,
};

__ROM_USED rt_tick_t pm_latch_tick(rt_tick_t curr_tick, uint32_t curr_time, float gtime_freq, void *user_data)
{
    uint32_t delta_time;
    rt_tick_t new_tick_ref;
    float delta_tick_ref;

    if (!pm_tick_cal.init)
    {
        return curr_tick;
    }

    delta_time = (curr_time - pm_tick_cal.last_latch_time);
    delta_tick_ref = ((float)delta_time * RT_TICK_PER_SECOND / gtime_freq + pm_tick_cal.tick_error_frac);
    new_tick_ref = pm_tick_cal.last_latch_tick + (uint32_t)delta_tick_ref;

    if ((new_tick_ref != curr_tick)      /* tick needs to be adjusted */
            || (delta_tick_ref > (60 * 1000))) /* tick has not been latched for 1 minutes */
    {
        /* only support 1ms tick */
        RT_ASSERT(RT_TICK_PER_SECOND == 1000);
        // rt_kprintf("adj:%d,%d,%d,%d,%.2f\n", curr_tick, curr_time, new_tick_ref, (uint32_t)delta_tick_ref, gtime_freq);
        // rt_kprintf("adj2:%d,%d,%.2f,%.2f\n", pm_tick_cal.last_latch_time, pm_tick_cal.last_latch_tick, pm_tick_cal.tick_error_frac, delta_tick_ref);

        if (((int32_t)(new_tick_ref - curr_tick) < 0) && (1 != (uint32_t)user_data))
        {
            /* new_tick_ref is less than curr_tick, use curr_tick instead */
            new_tick_ref = curr_tick;
            delta_tick_ref = (float)0;
            pm_tick_cal.tick_reset_cnt++;
        }
        pm_tick_cal.last_latch_tick = new_tick_ref;
        pm_tick_cal.last_latch_time = curr_time;
        pm_tick_cal.tick_error_frac = (delta_tick_ref - (uint32_t)delta_tick_ref);
        // rt_kprintf("adj2:%d,%d,%.2f,%.2f\n", pm_tick_cal.last_latch_time, pm_tick_cal.last_latch_tick, pm_tick_cal.tick_error_frac, delta_tick_ref);

        RT_ASSERT(pm_tick_cal.tick_error_frac < 1);
    }

    return new_tick_ref;
}

void pm_set_last_latch_tick(rt_tick_t last_latch_tick)
{
    pm_tick_cal.last_latch_tick = last_latch_tick;
}


#ifdef SOC_BF0_HCPU
#define LPTIMER "lptim1"

static int sifli_suspend(const struct rt_device *device, uint8_t mode)
{
    int r = RT_EOK;

#ifdef USING_IPC_QUEUE
    if (!ipc_queue_check_idle())
    {
        goto __EBUSY;
    }
#endif

    if (HAL_HPAON_GET_WSR() & HAL_HPAON_GET_WER())
    {
        goto __EBUSY;
    }

    return r;

__EBUSY:
    r = RT_EBUSY;

    return r;
}

void sifli_resume(const struct rt_device *device, uint8_t mode)
{
    return ;
}



RT_WEAK void aon_irq_handler_hook(uint32_t wsr)
{
}



void AON_IRQHandler(void)
{
    uint32_t status;
    GPIO_TypeDef *gpio;
    uint16_t pin;
    uint32_t pin_wsr;

    rt_interrupt_enter();

    NVIC_DisableIRQ(AON_IRQn);
    HAL_HPAON_CLEAR_POWER_MODE();

    status = HAL_HPAON_GET_WSR();
    status &= ~HPSYS_AON_WSR_PIN_ALL;//Clear PIN WSR bits in GPIO handler
    HAL_HPAON_CLEAR_WSR(status);


    PM_DEBUG_PIN_TOGGLE();

#ifdef BSP_PM_DEBUG
    rt_kprintf("[pm]WSR:0x%x\n", g_wakeup_src);
#endif /* BSP_PM_DEBUG */

    pin_wsr = status & HPSYS_AON_WSR_PIN_ALL;
    pin_wsr >>= HPSYS_AON_WSR_PIN0_Pos;
#ifdef RT_USING_PIN
    drv_pin_irq_from_wsr(pin_wsr);
#endif /* RT_USING_PIN */

    aon_irq_handler_hook(status);

    rt_interrupt_leave();
}

/**
 * @brief Enable pin wakeup
 *
 * @param pin pin number
 * @param mode pin wakeup mode
 * @retval status
 */
rt_err_t pm_enable_pin_wakeup(uint8_t pin, AON_PinModeTypeDef mode)
{
    HPAON_WakeupSrcTypeDef src;
    HAL_StatusTypeDef status;
    rt_err_t result;

    if (pin > (HPAON_WAKEUP_SRC_PIN_LAST - HPAON_WAKEUP_SRC_PIN0))
    {
        return -RT_EINVAL;
    }

    src = pin + HPAON_WAKEUP_SRC_PIN0;
    status = HAL_HPAON_EnableWakeupSrc(src, mode);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }

    result = RT_EOK;

__EXIT:
    return result;
}

/**
 * @brief Disable pin wakeup
 *
 * @param pin pin number
 * @retval status
 */
rt_err_t pm_disable_pin_wakeup(uint8_t pin)
{
    HPAON_WakeupSrcTypeDef src;
    HAL_StatusTypeDef status;
    rt_err_t result;

    if (pin > (HPAON_WAKEUP_SRC_PIN_LAST - HPAON_WAKEUP_SRC_PIN0))
    {
        return -RT_EINVAL;
    }

    src = pin + HPAON_WAKEUP_SRC_PIN0;
    status = HAL_HPAON_DisableWakeupSrc(src);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }

    result = RT_EOK;

__EXIT:
    return result;
}


rt_err_t pm_enable_rtc_wakeup(void)
{
    HAL_StatusTypeDef status;
    rt_err_t result;


    status = HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_RTC, AON_PIN_MODE_HIGH);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }

    status = HAL_PMU_EnableRtcWakeup();
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }
    result = RT_EOK;


__EXIT:
    return result;
}


rt_err_t pm_disable_rtc_wakeup(void)
{
    HAL_StatusTypeDef status;
    rt_err_t result;


    status = HAL_HPAON_DisableWakeupSrc(HPAON_WAKEUP_SRC_RTC);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }

    status = HAL_PMU_DisableRtcWakeup();
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }
    result = RT_EOK;


__EXIT:
    return result;
}



#elif defined(SOC_BF0_LCPU)

#ifdef BLUETOOTH
    __WEAK void ble_aon_irq_handler(void);
#endif

#ifdef SF32LB52X
    #define LPTIMER "lptim3"
#else
    #define LPTIMER "lptim2"
#endif /* SF32LB52X */

static int sifli_suspend(const struct rt_device *device, uint8_t mode)
{
    int r = RT_EOK;

    if (mode <= PM_SLEEP_MODE_IDLE)
        return r;

#ifdef USING_IPC_QUEUE
    if (!ipc_queue_check_idle() || !ipc_queue_check_idle_rom())
    {
        goto __EBUSY;
    }
#endif

    if (HAL_LPAON_GET_WSR() & HAL_LPAON_GET_WER())
    {
        goto __EBUSY;
    }


    return r;

__EBUSY:
    r = RT_EBUSY;


    return r;
}

void sifli_resume(const struct rt_device *device, uint8_t mode)
{
    return;
}

RT_WEAK void aon_irq_handler_hook(uint32_t wsr)
{
}

__ROM_USED void AON_LCPU_IRQHandler(void)
{
    uint32_t status;
    uint32_t pin_wsr;

    NVIC_DisableIRQ(AON_IRQn);

    /* workaround: if power mode is cleared before processing AON interrupt,
       the AON INT will get lost on FPGA */
    HAL_LPAON_CLEAR_POWER_MODE();

    status = HAL_LPAON_GET_WSR();
    status &= ~LPSYS_AON_WSR_PIN_ALL;//Clear PIN WSR bits in GPIO handler
    HAL_LPAON_CLEAR_WSR(status);

#ifdef BLUETOOTH
    //    ble_aon_irq_handler();
    // Switch to XT48
    //HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    //HAL_RCC_SetMacFreq();
#endif


#ifdef BSP_PM_DEBUG
    rt_kprintf("[pm]WSR:0x%x\n", g_wakeup_src);
#endif /* BSP_PM_DEBUG */


    pin_wsr = status & LPSYS_AON_WSR_PIN_ALL;
    pin_wsr >>= LPSYS_AON_WSR_PIN0_Pos;
#ifdef RT_USING_PIN
    drv_pin_irq_from_wsr(pin_wsr);
#endif /* RT_USING_PIN */


    aon_irq_handler_hook(status);

}



void AON_IRQHandler(void)
{
    rt_interrupt_enter();
    AON_LCPU_IRQHandler();
    rt_interrupt_leave();
}


/**
 * @brief Enable pin wakeup
 *
 * @param pin pin number
 * @param mode pin wakeup mode
 * @retval status
 */
rt_err_t pm_enable_pin_wakeup(uint8_t pin, AON_PinModeTypeDef mode)
{
    LPAON_WakeupSrcTypeDef src;
    HAL_StatusTypeDef status;
    rt_err_t result;

    if (pin > (LPAON_WAKEUP_SRC_PIN_LAST - LPAON_WAKEUP_SRC_PIN0))
    {
        return -RT_EINVAL;
    }

    src = pin + LPAON_WAKEUP_SRC_PIN0;
    status = HAL_LPAON_EnableWakeupSrc(src, mode);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }

#if defined(HAL_PMU_MODULE_ENABLED) && defined(SF32LB55X)
    if (AON_PIN_MODE_LOW == mode)
    {
        mode = AON_PIN_MODE_NEG_EDGE;
    }
    else if (AON_PIN_MODE_HIGH == mode)
    {
        mode = AON_PIN_MODE_POS_EDGE;
    }
    else
    {
        //do nothing
    }
    status = HAL_PMU_EnablePinWakeup(pin, mode);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }
#endif /* HAL_PMU_MODULE_ENABLED && SF32LB55X */

    result = RT_EOK;

__EXIT:
    return result;

}

/**
 * @brief Disable pin wakeup
 *
 * @param pin pin number
 * @retval status
 */
rt_err_t pm_disable_pin_wakeup(uint8_t pin)
{
    LPAON_WakeupSrcTypeDef src;
    HAL_StatusTypeDef status;
    rt_err_t result;

    if (pin > (LPAON_WAKEUP_SRC_PIN_LAST - LPAON_WAKEUP_SRC_PIN0))
    {
        return -RT_EINVAL;
    }

    src = pin + LPAON_WAKEUP_SRC_PIN0;
    status = HAL_LPAON_DisableWakeupSrc(src);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }

#if defined(HAL_PMU_MODULE_ENABLED) && defined(SF32LB55X)
    status = HAL_PMU_DisablePinWakeup(pin);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }
#endif /* HAL_PMU_MODULE_ENABLED && SF32LB55X */

    result = RT_EOK;

__EXIT:
    return result;
}

rt_err_t pm_enable_rtc_wakeup(void)
{
    HAL_StatusTypeDef status;
    rt_err_t result;


    status = HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_RTC, AON_PIN_MODE_HIGH);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }

#ifdef HAL_PMU_MODULE_ENABLED
    status = HAL_PMU_EnableRtcWakeup();
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }
#endif /* HAL_PMU_MODULE_ENABLED */
    result = RT_EOK;

__EXIT:
    return result;
}


rt_err_t pm_disable_rtc_wakeup(void)
{
    HAL_StatusTypeDef status;
    rt_err_t result;


    status = HAL_LPAON_DisableWakeupSrc(LPAON_WAKEUP_SRC_RTC);
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }

#ifdef HAL_PMU_MODULE_ENABLED
    status = HAL_PMU_DisableRtcWakeup();
    if (HAL_OK != status)
    {
        result = -RT_ERROR;
        goto __EXIT;
    }
#endif  /* HAL_PMU_MODULE_ENABLED */
    result = RT_EOK;

__EXIT:
    return result;
}

#else
#error "Core not defined"
#endif

const struct rt_device_pm_ops sifli_pm_op =
{
    .suspend = sifli_suspend,
    .resume = sifli_resume,
};

#if defined(USING_CONTEXT_BACKUP) && defined(SOC_BF0_HCPU)
int32_t pm_init_mem_map()
{
#if 0

    int32_t backup_len;
    int32_t ret_ram_len;

    pm_mem_map.ram_start_addr = HPSYS_RAM0_BASE;
    pm_mem_map.ram_end_addr = HPSYS_RAM_END + 1;
    backup_len = 0;
    pm_mem_map.retained_region_num = 2;
    pm_mem_map.retained_region[0].start_addr = (uint32_t)EXEC_REGION_START_ADDR(ER_ITCM$$RW);
    pm_mem_map.retained_region[0].len = (uint32_t)EXEC_REGION_END_ADDR(ER_ITCM$$ZI) - (uint32_t)EXEC_REGION_START_ADDR(ER_ITCM$$RW);
    backup_len += pm_mem_map.retained_region[0].len;
    pm_mem_map.retained_region[1].start_addr = (uint32_t)EXEC_REGION_START_ADDR(RW_IRAM1);
    pm_mem_map.retained_region[1].len = (uint32_t)(HPSYS_RAM_END + 1) - (uint32_t)EXEC_REGION_START_ADDR(RW_IRAM1);
    backup_len += pm_mem_map.retained_region[1].len;
    pm_mem_map.ret_ram_start_addr = (uint32_t)PM_RETENTION_RAM_START_ADDR;
#ifdef BSP_USING_PSRAM
    pm_mem_map.ret_ram_end_addr = PSRAM_BASE + PSRAM_SIZE;
#else
    pm_mem_map.ret_ram_end_addr = HPSYS_RETM_BASE + HPSYS_RETM_SIZE;
#endif


    ret_ram_len = pm_mem_map.ret_ram_end_addr - pm_mem_map.ret_ram_start_addr;
    /*  retention memory structure:
     *
     *  ram_backup_data
     *
     */

#ifdef BSP_USING_PSRAM
    RT_ASSERT((backup_len + RET_MEM_RESERVED_LEN) <= ret_ram_len);
#else
    // TODO: Update to minimize content in RET RAM.
#endif
#else
    cb_backup_param_t param;
    cb_retained_region_t *backup_region;
    rt_err_t err;

    param.ret_mem_start_addr = (uint32_t)PM_RETENTION_RAM_START_ADDR;
    param.ret_mem_size = PM_RETENTION_RAM_SIZE;
    param.backup_mask = CB_BACKUP_HEAP_MASK | CB_BACKUP_STATIC_DATA_MASK | CB_BACKUP_STACK_MASK;
    param.backup_region_num = 1;
    backup_region = &param.backup_region_list[0];
    backup_region->start_addr = (uint32_t)PM_BACKUP_REGION_START_ADDR;
    backup_region->len = PM_BACKUP_REGION_SIZE;
#if defined(HPSYS_ITCM_BASE) && !defined(PM_ITCM_NOT_BACKUP)
    param.backup_region_num++;
    backup_region++;
    backup_region->start_addr = (uint32_t)EXEC_REGION_START_ADDR(ER_ITCM$$RW);
    backup_region->len = (uint32_t)EXEC_REGION_END_ADDR(ER_ITCM$$ZI) - (uint32_t)EXEC_REGION_START_ADDR(ER_ITCM$$RW);
#endif /* PM_ITCM_NOT_BACKUP && !PM_ITCM_NOT_BACKUP */
    err = cb_init(&param);
    RT_ASSERT(RT_EOK == err);
#endif

    return 0;
}
#endif /* USING_CONTEXT_BACKUP && SOC_BF0_HCPU*/

__ROM_USED int low_power_init(void)
{
#ifndef PM_LP_TIMER_DISABLE
    if (g_t_hwtimer == NULL)
    {
        g_t_hwtimer = rt_device_find(LPTIMER);
        if (g_t_hwtimer)
        {
            int freq = DEFAULT_LPTIM_FREQ;
            rt_device_open(g_t_hwtimer, RT_DEVICE_FLAG_RDWR);
            rt_device_control(g_t_hwtimer, HWTIMER_CTRL_FREQ_SET, (void *)&freq);
            rt_device_set_rx_indicate(g_t_hwtimer, hwtm_rx_ind);
        }
    }
#endif // !PM_LP_TIMER_DISABLE

#if defined(BSP_PM_FREQ_SCALING) && !defined(PM_HW_DEEP_WFI_SUPPORT)
    pm_freq_scaling_param.sys_clk_src = -1;
#endif /* BSP_PM_FREQ_SCALING && !PM_HW_DEEP_WFI_SUPPORT */

    rt_system_pm_init(&sifli_pm, SIFLI_TIMER_MASK, NULL);
    rt_pm_policy_register(sizeof(pm_policy) / sizeof(pm_policy[0]), pm_policy);
    rt_pm_device_register(NULL, &sifli_pm_op);
    init_default_wakeup_src();

#if !defined(SF32LB55X) && defined(SOC_BF0_HCPU)

    pm_init_sp = *(uint32_t *)&__Vectors;

#endif /* SF32LB55X && SOC_BF0_HCPU */

    pm_tick_cal.last_latch_time = HAL_GTIMER_READ();
    pm_tick_cal.last_latch_tick = rt_tick_get();
    pm_tick_cal.init = true;

#if defined(SOC_BF0_LCPU) && defined(PM_DEEP_ENABLE) && defined(SF32LB58X)
    HAL_LPAON_ENABLE_DS_PWR_REQ();

    HAL_PMU_LPSYS_DS_USE_BUCK2();

    HAL_PMU_ENABLE_BUCK2_LOW_PWR();
#endif /* SOC_BF0_LCPU && PM_DEEP_ENABLE && SF32LB58X*/


#if defined(SOC_BF0_LCPU) && defined(SF32LB56X)
#ifdef PM_DEEP_ENABLE
    MODIFY_REG(hwp_pmuc->LPSYS_SWR, PMUC_LPSYS_SWR_PSW_RET_Msk,
               MAKE_REG_VAL(2, PMUC_LPSYS_SWR_PSW_RET_Msk, PMUC_LPSYS_SWR_PSW_RET_Pos));
#else
    MODIFY_REG(hwp_pmuc->LPSYS_SWR, PMUC_LPSYS_SWR_PSW_RET_Msk,
               MAKE_REG_VAL(1, PMUC_LPSYS_SWR_PSW_RET_Msk, PMUC_LPSYS_SWR_PSW_RET_Pos));
#endif /* PM_DEEP_ENABLE */
#endif /* SOC_BF0_LCPU */


    PM_DEBUG_PIN_INIT();


    return 0;
}

INIT_COMPONENT_EXPORT(low_power_init);

rt_err_t pm_scenario_start(pm_scenario_name_t scenario)
{
    rt_enter_critical();

    if (PM_SCENARIO_UI == scenario)
    {
        pm_scenario_ctx.ui_active = true;
    }
    else if (PM_SCENARIO_AUDIO == scenario)
    {
        pm_scenario_ctx.audio_active = true;
    }
    else if (PM_SCENARIO_RFTEST == scenario)
    {
        pm_scenario_ctx.rftest_active = true;
    }
    rt_exit_critical();

    if (pm_scenario_ctx.ui_active || pm_scenario_ctx.audio_active || pm_scenario_ctx.rftest_active)
    {
        rt_pm_run_enter(PM_RUN_MODE_HIGH_SPEED);
    }

#if !defined(SF32LB55X)
    if (pm_scenario_ctx.audio_active)
    {
        /* if audio is active, downscale to 48MHz  */
        HAL_RCC_HCPU_SetDeepWFIDiv(1, 0, 0);
#ifdef SF32LB52X
        /* force clock on to avoid audprc clock is closed during DeepWFI */
        hwp_hpsys_rcc->DBGR |= HPSYS_RCC_DBGR_FORCE_HP;
#endif /* SF32LB52X */
    }
#endif /* !SF32LB55X */

    return RT_EOK;
}

rt_err_t pm_scenario_stop(pm_scenario_name_t scenario)
{
    rt_enter_critical();

    if (PM_SCENARIO_UI == scenario)
    {
        pm_scenario_ctx.ui_active = false;
    }
    else if (PM_SCENARIO_AUDIO == scenario)
    {
        pm_scenario_ctx.audio_active = false;
    }
    else if (PM_SCENARIO_RFTEST == scenario)
    {
        pm_scenario_ctx.rftest_active = false;
    }

    rt_exit_critical();

    if (!(pm_scenario_ctx.ui_active || pm_scenario_ctx.audio_active || pm_scenario_ctx.rftest_active))
    {
        rt_pm_run_enter(PM_RUN_MODE_MEDIUM_SPEED);
    }

#if !defined(SF32LB55X)
    if (!pm_scenario_ctx.audio_active)
    {
#ifndef SF32LB52X
        HAL_RCC_HCPU_SetDeepWFIDiv(48, 0, 1);
#else
        HAL_RCC_HCPU_SetDeepWFIDiv(12, 0, 0);
        /* disable clock force */
        hwp_hpsys_rcc->DBGR &= ~HPSYS_RCC_DBGR_FORCE_HP;
#endif /* !SF32LB52X */
    }
#endif /* !SF32LB55X */

    return RT_EOK;
}

#ifdef PM_METRICS_ENABLED
static void pm_metrics_core_init(void)
{
    uint32_t i;

    pm_debug_metrics_ctx.last_sleep_tick = rt_tick_get();

    for (i = 0; i < PM_METRICS_MAX_WAKEUP_SRC_NUM; i++)
    {
        pm_stat.wakeup_src_stat[i].wakeup_src = 0xFF;
    }

    pm_stat.run_start_time[PM_RUN_MODE_HIGH_SPEED] = HAL_GTIMER_READ();
    pm_stat.curr_run_mode = PM_RUN_MODE_HIGH_SPEED;
}
#endif /* PM_METRICS_ENABLED */


#if defined(PM_METRICS_USE_COLLECTOR)
static void pm_metrics_collect(void *user_data)
{
    pm_stat_metrics_t *metrics;
    mc_err_t err;
    uint32_t i;
    uint32_t data_size;

    /* output data according actual wakeup_src_num */
    data_size = sizeof(pm_stat_metrics_t) + pm_stat.wakeup_src_num * sizeof(pm_wakeup_src_stat_metrics_t);

    metrics = mc_alloc_metrics(METRICS_MW_PM_STAT, data_size);
    RT_ASSERT(metrics);

    metrics->sleep_time = pm_stat.sleep_time;
    metrics->total_wakeup_times = pm_stat.total_wakeup_times;
    metrics->wfi_time = (float)pm_stat.wfi_time / HAL_LPTIM_GetFreq();
    metrics->deepwfi_time = (float)pm_stat.deepwfi_time / HAL_LPTIM_GetFreq();
    for (i = 0; i < PM_RUN_MODE_MAX; i++)
    {
        if (pm_stat.run_time[i] > 0)
        {
            metrics->run_time[i] = pm_stat.run_time[i] / HAL_LPTIM_GetFreq();
        }
        else
        {
            metrics->run_time[i] = 0;
        }
        pm_stat.run_time[i] = 0;
    }
    metrics->wakeup_src_num = pm_stat.wakeup_src_num;
    memcpy(&metrics->wakeup_src_stat[0], &pm_stat.wakeup_src_stat[0],
           pm_stat.wakeup_src_num * sizeof(pm_wakeup_src_stat_metrics_t));

    pm_stat.sleep_time = 0;
    pm_stat.total_wakeup_times = 0;
    pm_stat.wfi_time = 0;
    pm_stat.deepwfi_time = 0;
    pm_stat.wakeup_src_num = 0;

    err = mc_save_metrics(metrics, true);
    RT_ASSERT(MC_OK == err);
}


static void pm_debug_metrics_report_timer_timeout(void *parameter)
{
    pm_debug_metrics_t *metrics;
    rt_tick_t next_timeout;
    rt_timer_t timer;
    uint8_t name_len;

    rt_timer_delete(pm_debug_metrics_ctx.timer);
    pm_debug_metrics_ctx.timer = NULL;

    metrics = (pm_debug_metrics_t *)mc_alloc_metrics(METRICS_MW_PM_DEBUG, sizeof(*metrics));
    RT_ASSERT(metrics);

#ifdef SOC_BF0_HCPU
    metrics->wsr = HAL_HPAON_GET_WSR();
#else
    metrics->wsr = HAL_LPAON_GET_WSR();
#endif
    next_timeout = rt_timer_next_timeout_tick();
    if (RT_TICK_MAX != next_timeout)
    {
        metrics->idle_time = next_timeout - rt_tick_get();
    }
    else
    {
        metrics->idle_time = UINT32_MAX;
    }
    timer = rt_timer_next_timer();
    if (timer)
    {
        if ((PM_TIMER_NAME_LEN - 1) > RT_NAME_MAX)
        {
            name_len = RT_NAME_MAX;
        }
        else
        {
            name_len = (PM_TIMER_NAME_LEN - 1);
        }
        strncpy(metrics->timer_name, timer->parent.name, name_len);
        metrics->timer_name[name_len] = 0;
    }
    else
    {
        metrics->timer_name[0] = 0;
    }
    metrics->idle_mode_cnt = rt_pm_sleep_mode_state_get(PM_SLEEP_MODE_IDLE);
    metrics->ipc_queue_state = ipc_queue_check_idle();
#ifdef SOC_BF0_LCPU
    metrics->ipc_queue_state |= ipc_queue_check_idle_rom() << 4;
#endif /* SOC_BF0_LCPU */

    mc_save_metrics(metrics, true);
}

static void pm_debug_idle_hook(void)
{
    uint32_t curr_tick;
    int32_t delta;
    pm_debug_metrics_ctx_t *ctx = &pm_debug_metrics_ctx;
    int32_t thresh;

    curr_tick = rt_tick_get();
    delta = (int32_t)(curr_tick - ctx->last_sleep_tick);
    // RT_ASSERT(delta >= 0);

    RT_ASSERT(ctx->report_thresh < PM_DEBUG_METRICS_REPORT_THRESH_TBL_SIZE);
    thresh = pm_debug_metrics_report_thresh_tbl[ctx->report_thresh];
    if (delta > thresh)
    {
        if ((ctx->report_thresh + 1) < PM_DEBUG_METRICS_REPORT_THRESH_TBL_SIZE)
        {
            ctx->report_thresh++;
        }
        ctx->last_sleep_tick = rt_tick_get();

        ctx->timer = rt_timer_create("pm", pm_debug_metrics_report_timer_timeout,
                                     0, 50, RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        RT_ASSERT(ctx->timer);
        rt_timer_start(ctx->timer);
    }
}

int pm_metrics_init(void)
{
    mc_err_t err;

    pm_metrics_core_init();

    rt_thread_idle_sethook(pm_debug_idle_hook);

    pm_metrics_collector.callback = pm_metrics_collect;
    pm_metrics_collector.period = MC_PERIOD_EVERY_HOUR;
    pm_metrics_collector.user_data = 0;

    err = mc_register_collector(&pm_metrics_collector);
    RT_ASSERT(MC_OK == err);
    return 0;
}
INIT_APP_EXPORT(pm_metrics_init);

#ifndef MC_CLIENT_ENABLED
#if 0
static bool metrics_read_callback(uint16_t id, uint8_t core, uint16_t data_len, uint32_t time, void *data)
{
    pm_metrics_t *pm_metrics;
    pm_debug_metrics_t *pm_debug_metrics;

    if (METRICS_MW_PM == id)
    {
        pm_metrics = (pm_metrics_t *)data;
        LOG_I("[%d][%d][pm]: %7.2f", time, core, pm_metrics->sleep_time);
    }
    else if (METRICS_MW_PM_DEBUG == id)
    {
        pm_debug_metrics = (pm_debug_metrics_t *)data;
        LOG_I("[%d][%d][pm]: %d,%d,%d,0x%08x,(%s)", time, core, pm_debug_metrics->idle_mode_cnt,
              pm_debug_metrics->idle_time, pm_debug_metrics->ipc_queue_state,
              pm_debug_metrics->wsr, pm_debug_metrics->timer_name);
    }

    return false;
}

static void pm_metrics_list(int argc, char **argv)
{
    mc_err_t err;

    err = mc_read_metrics(metrics_read_callback);
    RT_ASSERT(MC_OK == err);

}
MSH_CMD_EXPORT(pm_metrics_list, list pm metrics);
#endif
#endif /* MC_CLIENT_ENABLED */


#endif /* PM_METRICS_USE_COLLECTOR */


#ifdef PM_METRICS_PRINT_DIRECTLY
static void print_timer_callback(void *parameter)
{
    LOG_I("============================");
    LOG_I("sleep time: %7.2f", pm_metrics.sleep_time);
    LOG_I("============================");
    pm_metrics.sleep_time = 0;
}

int pm_metrics_init(void)
{
    rt_err_t err;
    rt_timer_t timer;

    pm_metrics_core_init();

    timer = rt_timer_create("pm_metrics", print_timer_callback, 0, rt_tick_from_millisecond(PM_METRICS_PRINT_PERIOD * 1000),
                            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    RT_ASSERT(timer);
    err = rt_timer_start(timer);

    RT_ASSERT(RT_EOK == err);

    return 0;
}
INIT_APP_EXPORT(pm_metrics_init);

#endif /* PM_METRICS_PRINT_DIRECTLY */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
