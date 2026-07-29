/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_COMMON_H__
#define __DRV_COMMON_H__

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include "board.h"

#ifdef BSP_USING_HWMAILBOX
    #include "bf0_mbox_common.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void _Error_Handler(char *s, int num);

#ifndef Error_Handler
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#endif

#define DMA_NOT_AVAILABLE ((DMA_INSTANCE_TYPE *)0xFFFFFFFFU)

#ifdef BSP_USING_LCPU_IMG
void lcpu_img_install(void);
#endif

typedef struct
{
    uint32_t sysclk;
    uint32_t hclk;
    uint32_t pclk1;
    uint32_t pclk2;
} hpsys_clk_setting_t;

typedef struct
{
    uint32_t sysclk;
    uint32_t hclk;
    uint32_t pclk1;
    uint32_t pclk2;
} lpsys_clk_setting_t;

typedef struct
{
    uint32_t sysclk;
    uint32_t hclk;
    uint32_t macclk;
    uint32_t macfreq;
} blesys_clk_setting_t;

void drv_get_hpsys_clk(hpsys_clk_setting_t *clk_setting);
void drv_get_lpsys_clk(lpsys_clk_setting_t *clk_setting);
void drv_get_blesys_clk(blesys_clk_setting_t *clk_setting);

/**
* @brief  Update RTC time with calculated delta
*/
void drv_rtc_apply_delta(void);

/**
* @brief  Calculate difference between RTC reading and estimation based on calibrated RC10K reading
* @param  reset called from reset
* @retval none
*/
void drv_rtc_calculate_delta(int reset);

/**
* @brief  Get current time stamp stored on RTC hardware
* @retval timestamp returned
*/
time_t drv_get_timestamp(void) ;

/**
* @brief  Update RC10K calibration data.
* @param  backup RC10K calibartion value.
* @retval none
*/
void drv_set_soft_rc10_backup(uint32_t backup);

uint8_t drv_tp_get_wakeup_check_enable(void);
void drv_tp_set_wakeup_check_enable(uint8_t is_enalbe);
void drv_reboot(void);

/**
 * @brief  Weak function to get the max TX power.
 *         User code can override this function to provide a custom maximum TX power.
 * @return max TX power in dBm
 */
int8_t bt_tx_pwr_max_override(void);

/**
 * @brief  Weak function to get the min TX power.
 *         User code can override this function to provide a custom minimum TX power.
 * @return min TX power in dBm
 */
int8_t bt_tx_pwr_min_override(void);

/**
 * @brief  Weak function to get the initial TX power.
 *         User code can override this function to provide a custom initial TX power.
 * @return initial TX power in dBm
 */
int8_t bt_tx_pwr_init_override(void);

void rt_hw_systick_init(void);

#ifdef BSP_USING_BUSMON

/**
* @brief  Use bus monitor for debugging purpose
* @param  func Bus master/slave type
* @param  address Start address
* @param  address_end End address
* @param  ishcpu 1: HCPU, 0: LCPU
* @param  count Count to generate PTC interrupt
* @param  access Access type, one of BUSMON_OPFLAG_READ, BUSMON_OPFLAG_WRITE, BUSMON_OPFLAG_RW
*/
void dbg_busmon(HAL_BUSMON_FuncTypeDef func, uint32_t address, uint32_t address_end, uint8_t ishcpu, uint32_t count, uint8_t access);

/**
* @brief  Register callback for bus monitor
* @param  callback Callback function when bus activitiy triggered.
*/
void dbg_busmon_reg_callback(void(*callback)(void));

#ifdef SOC_BF0_HCPU
#define dbg_busmon_write(address,count)    dbg_busmon(HAL_BUSMON_HCPU_S,address,address+4,1,count,BUSMON_OPFLAG_WRITE)
#define dbg_busmon_read(address,count)     dbg_busmon(HAL_BUSMON_HCPU_S,address,address+4,1,count,BUSMON_OPFLAG_READ)
#define dbg_busmon_access(address,count)   dbg_busmon(HAL_BUSMON_HCPU_S,address,address+4,1,count,BUSMON_OPFLAG_RW)
/**
* @brief  Use bus monitor to monitor PSRAM access (HCPU Only).
* @param  address Start address
* @param  count Count to generate PTC interrupt
* @param  access Access type, one of BUSMON_OPFLAG_READ, BUSMON_OPFLAG_WRITE, BUSMON_OPFLAG_RW
*/
void dbg_busmon_psram(uint32_t address, uint32_t count, uint8_t access);
#else
#define dbg_busmon_write(address,count)    dbg_busmon(HAL_BUSMON_LCPU_S,address,address+4,0,count,BUSMON_OPFLAG_WRITE)
#define dbg_busmon_read(address,count)     dbg_busmon(HAL_BUSMON_LCPU_S,address,address+4,0,count,BUSMON_OPFLAG_READ)
#define dbg_busmon_access(address,count)   dbg_busmon(HAL_BUSMON_LCPU_S,address,address+4,0,count,BUSMON_OPFLAG_RW)
#define dbg_busmon_psram(address,count,access)
#endif

#endif

/**
 * @brief  Malloc a SRAM memory which is accessible by DMA controller
 *
 * @param  n Size of memory to allocate
 *
 * @return Pointer to allocated memory
 */
void *malloc_dma_friendly_sram(rt_size_t n);
void *calloc_dma_friendly_sram(rt_size_t count, rt_size_t size);
void free_dma_friendly_sram(void *ptr);

/**
 * @brief  Called at the beginning of rt_hw_board_init
 *
 * By default it's implemented as a weak funtion which is an empty funciton, user could implement a new one.
 */
void rt_hw_preboard_init(void);

#ifndef _MSC_VER

/* Support sub-level for init routines. Priority of sub-level from high to low
   is 0 to 9. Init macro without sub-level use 5 sub-level.
*/

/* board init routines will be called in board_init() function */
//#define INIT_BOARD_EXPORT(fn)           INIT_EXPORT(fn, "1", "5")
#define INIT_BOARD_EXPORT_DYN(fn, sub_lvl)           INIT_EXPORT(fn, "1", STR1_(sub_lvl))

/* pre/device/component/env/app init routines will be called in init_thread */
/* components pre-initialization (pure software initilization) */
//#define INIT_PREV_EXPORT(fn)            INIT_EXPORT(fn, "2", "5")
#define INIT_PREV_EXPORT_DYN(fn, sub_lvl)            INIT_EXPORT(fn, "2", STR1_(sub_lvl))

/* device initialization */
//#define INIT_DEVICE_EXPORT(fn)          INIT_EXPORT(fn, "3", "5")
#define INIT_DEVICE_EXPORT_DYN(fn, sub_lvl)          INIT_EXPORT(fn, "3", STR1_(sub_lvl))

/* components initialization (dfs, lwip, ...) */
//#define INIT_COMPONENT_EXPORT(fn)       INIT_EXPORT(fn, "4", "5")
#define INIT_COMPONENT_EXPORT_DYN(fn, sub_lvl)       INIT_EXPORT(fn, "4", STR1_(sub_lvl))

/* environment initialization (mount disk, ...) */
//#define INIT_ENV_EXPORT(fn)             INIT_EXPORT(fn, "5", "5")
#define INIT_ENV_EXPORT_DYN(fn, sub_lvl)             INIT_EXPORT(fn, "5", STR1_(sub_lvl))

/* pre appliation initialization (rtgui application etc ...) */
//#define INIT_PRE_APP_EXPORT(fn)         INIT_EXPORT(fn, "6", "5")
#define INIT_PRE_APP_EXPORT_DYN(fn, sub_lvl)         INIT_EXPORT(fn, "6", STR1_(sub_lvl))

/* appliation initialization (rtgui application etc ...) */
//#define INIT_APP_EXPORT(fn)             INIT_EXPORT(fn, "7", "5")
#define INIT_APP_EXPORT_DYN(fn, sub_lvl)             INIT_EXPORT(fn, "7", STR1_(sub_lvl))

#else

/** Converts a macro argument into a character constant.
 */

/* board init routines will be called in board_init() function */
//#define INIT_BOARD_EXPORT(fn)           INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 1, 5))
#define INIT_BOARD_EXPORT_DYN(fn, sub_lvl)           INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 1, sub_lvl))

/* pre/device/component/env/app init routines will be called in init_thread */
/* components pre-initialization (pure software initilization) */
//#define INIT_PREV_EXPORT(fn)            INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 2, 5))
#define INIT_PREV_EXPORT_DYN(fn, sub_lvl)            INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 2, sub_lvl))

/* device initialization */
//#define INIT_DEVICE_EXPORT(fn)          INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 3, 5))
#define INIT_DEVICE_EXPORT_DYN(fn, sub_lvl)          INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 3, sub_lvl))

/* components initialization (dfs, lwip, ...) */
//#define INIT_COMPONENT_EXPORT(fn)       INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 4, 5))
#define INIT_COMPONENT_EXPORT_DYN(fn, sub_lvl)       INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 4, sub_lvl))

/* environment initialization (mount disk, ...) */
//#define INIT_ENV_EXPORT(fn)             INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 5, 5))
#define INIT_ENV_EXPORT_DYN(fn, sub_lvl)             INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 5, sub_lvl))

/* pre appliation initialization (rtgui application etc ...) */
//#define INIT_PRE_APP_EXPORT(fn)         INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 6, 5))
#define INIT_PRE_APP_EXPORT_DYN(fn, sub_lvl)         INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 6, sub_lvl))

/* appliation initialization (rtgui application etc ...) */
//#define INIT_APP_EXPORT(fn)             INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 7, 5))
#define INIT_APP_EXPORT_DYN(fn, sub_lvl)             INIT_EXPORT(fn, STR_CONCAT(rti_fn$, 7, sub_lvl))

#endif

#ifdef __cplusplus
}
#endif

#endif

