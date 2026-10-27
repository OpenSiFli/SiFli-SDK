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

/* AW9523B I/O expander pin assignments (0xPp: P = port 0/1, p = bit 0..7). */
#define AW9523_PIN_USB_DET      0x00    /* P0_0 input, USB VBUS detect */
#define AW9523_PIN_TF_DET       0x01    /* P0_1 input, TF card detect */
#define AW9523_PIN_PDM_PWREN    0x04    /* P0_4 */
#define AW9523_PIN_AU_PA_EN     0x05    /* P0_5 */
#define AW9523_PIN_DVP_PDN      0x06    /* P0_6 */
#define AW9523_PIN_HST_WAK_WF   0x07    /* P0_7 */
#define AW9523_PIN_2G4_RFSW     0x10    /* P1_0 */
#define AW9523_PIN_LED_R        0x11    /* P1_1 */
#define AW9523_PIN_LED_G        0x12    /* P1_2 */
#define AW9523_PIN_LED_B        0x13    /* P1_3 */
#define AW9523_PIN_WF_PWRKEY    0x14    /* P1_4 */
#define AW9523_PIN_WF_PWREN     0x15    /* P1_5 */
#define AW9523_PIN_CTP_RST      0x16    /* P1_6 */
#define AW9523_PIN_LCD_RST      0x17    /* P1_7 */
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

#ifdef SOC_BF0_ACPU
#ifndef HEAP_END
#define HEAP_END       (ACPU_RAM_DATA_START_ADDR + ACPU_RAM_DATA_SIZE)
#endif
#elif defined(SOC_BF0_HCPU)
#ifndef HEAP_END
#define HEAP_END       (HCPU_RAM_DATA_START_ADDR + HCPU_RAM_DATA_SIZE) //TODO:
#endif
#else
#define HEAP_END       (LCPU_RAM_DATA_START_ADDR + LCPU_RAM_DATA_SIZE) //TODO:
#endif

void SystemClock_Config(void);
void aw9523_ensure_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
