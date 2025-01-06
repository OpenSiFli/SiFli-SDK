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

#include "rtconfig.h"
#include <register.h>
#include "drv_io.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALIGN(n)                    __attribute__((aligned(n)))

// Boot source
#define BOOT_FROM_SIP_PUYA          1
#define BOOT_FROM_SIP_GD            2
#define BOOT_FROM_NOR               3
#define BOOT_FROM_NAND              4
#define BOOT_FROM_SD                5
#define BOOT_FROM_EMMC              6

// MPI1 SIP for 54x
#define BOOT_SIP_PUYA               0
#define BOOT_SIP_GD                 1
#define BOOT_PSRAM_APS_128P         2
#define BOOT_PSRAM_APS_64P          3
#define BOOT_PSRAM_APS_32P          4
#define BOOT_PSRAM_APS_16P          5
#define BOOT_PSRAM_WINBOND          6
#define BOOT_SIP_NONE               7

extern int board_boot_src;
int board_boot_from(void);
void board_flash_power_on();
void board_init_psram(void);

extern void board_pinmux_uart();
extern void board_pinmux_mpi1_puya_base();
extern void board_pinmux_mpi1_puya_ext(int is64Mb);
extern void board_pinmux_mpi1_gd();
extern void board_pinmux_mpi2();
extern void board_pinmux_sd();
extern void boot_error(unsigned char code);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
