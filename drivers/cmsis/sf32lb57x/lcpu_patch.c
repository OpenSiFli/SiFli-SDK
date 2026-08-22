/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include "bf0_hal.h"
#include "mem_map.h"
#include "register.h"
#include "bf0_hal_patch.h"
#ifdef HAL_LCPU_PATCH_MODULE
const unsigned int g_lcpu_patch_list[] = {   0x50544348, 0x00000010, 0x00031178, 0xBF5CF3DE,
                                             0x0002FEE4, 0xB8B6F3E0,
                                         };
const unsigned int g_lcpu_patch_bin[] = {    0x49064805, 0x48066001, 0x70012102, 0xF44F4805,
                                             0xF8C02100, 0x477010C4, 0x204001F4, 0x00410065,
                                             0x20400A4C, 0x20400954, 0x0005F895, 0xF895B938,
                                             0xB920002D, 0x107FF241, 0x0003F2C0, 0xF2414700,
                                             0xF2C0108B, 0x47000003, 0x162CF240, 0xF64F1B92,
                                             0xF2C066ED, 0x47300602, 0xBF082831, 0x2F00F5B1,
                                             0x480AD112, 0x4A0C490A, 0x1400F8C0, 0x600A4909,
                                             0x604A4A0A, 0x6801220D, 0x210EF362, 0x60012204,
                                             0xF3626801, 0x60010106, 0x47702000, 0x40090490,
                                             0x00090908, 0x40090894, 0x00070504, 0x1CA80907,
                                        };
void lcpu_patch_install()
{
    uint32_t entry[3] = {0x48434150, 0x2, LCPU_PATCH_BUF_START_ADDR + 13};
    memcpy((void *)LCPU_PATCH_BUF_START_ADDR, (void *)&entry, 12);
#ifdef SOC_BF0_HCPU
    memset((void *)(LCPU_PATCH_BUF_START_ADDR + 12), 0, LCPU_PATCH_TOTAL_SIZE);
    memcpy((void *)(LCPU_PATCH_BUF_START_ADDR + 12), g_lcpu_patch_bin, sizeof(g_lcpu_patch_bin));
#else
    memset((void *)(LCPU_PATCH_BUF_START_ADDR - 0x20000000 + 12), 0, LCPU_PATCH_TOTAL_SIZE);
    memcpy((void *)(LCPU_PATCH_BUF_START_ADDR - 0x20000000 + 12), g_lcpu_patch_bin, sizeof(g_lcpu_patch_bin));
#endif
    HAL_PATCH_install();
}
uint32_t *HAL_PATCH_GetEntryAddr(void)
{
    uint32_t *entry_addr = (uint32_t *)g_lcpu_patch_list;
    return entry_addr;
}
#endif
