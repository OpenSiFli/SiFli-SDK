/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BSP_BOARD_H__
#define __BSP_BOARD_H__

#include "rtconfig.h"
#include "drv_io.h"
#include "bf0_hal.h"
#ifdef PMIC_CTRL_ENABLE
    #include "pmic_controller.h"
#endif /* PMIC_CTRL_ENABLE */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif defined ( __GNUC__ )
extern int __bss_end;
#define HEAP_BEGIN      ((void *)&__bss_end)
#endif

#ifdef SOC_BF0_HCPU
#ifndef HEAP_END
#define HEAP_END       (HCPU_RAM_DATA_START_ADDR + HCPU_RAM_DATA_SIZE) //TODO:
#endif
#else
#define HEAP_END       (LCPU_RAM_DATA_START_ADDR + LCPU_RAM_DATA_SIZE) //TODO:
#endif

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
