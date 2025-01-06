/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <register.h>
#include "drv_io.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALIGN(n)                    __attribute__((aligned(n)))

typedef enum
{
    BOOT_FROM_MPI3_NAND,
    BOOT_FROM_MPI3_NOR,
    BOOT_FROM_SD_EMMC,
    BOOT_FROM_SD_NAND,
} BOOT_SRC;

#define BOOT_FROM_QFN()   (__HAL_SYSCFG_GET_PID()==0)
#define BOOT_FROM_NAND()  (sec_config_cache.ftab[DFU_FLASH_IMG_HCPU].base!=sec_config_cache.ftab[DFU_FLASH_IMG_HCPU].xip_base)

#define BOOT_MPI1_PSRAM 1
#define BOOT_MPI2_PSRAM 2

#define BOOT_PSRAM_APS_128P         0
#define BOOT_PSRAM_APS_64P          1
#define BOOT_PSRAM_APS_32P          2
#define BOOT_PSRAM_APS_16P          3
#define BOOT_PSRAM_WINBOND          4

extern int board_boot_src;
int board_boot_from(void);
void board_flash_power_on();

extern void board_pinmux_uart(int qfn);
extern void board_pinmux_psram(uint8_t       mpi, uint8_t type);
extern void board_pinmux_mpi3(int qfn);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
