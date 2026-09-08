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
const unsigned int g_lcpu_patch_list[] = {   0x50544348, 0x00000028, 0x00045A60, 0x46374BBF,
                                             0x00031178, 0xBF70F3DE, 0x0002FEE4, 0xB8CAF3E0,
                                             0x000045A8, 0xB570F00B, 0x0002D0E0, 0xBFE6F3E2,
                                         };
const unsigned int g_lcpu_patch_bin[] = {    0x2105480B, 0x48577001, 0x70012101, 0x490A4809,
                                             0x480A6001, 0x6001490A, 0x490B480A, 0x480B6001,
                                             0x2120F44F, 0x10C4F8C0, 0xF8C02120, 0x47701084,
                                             0x20400A4C, 0x204001F4, 0x0041018D, 0x204001D4,
                                             0x00410179, 0x204001D0, 0x004100F9, 0x20400954,
                                             0x0005F895, 0xF895B938, 0xB920002D, 0x107FF241,
                                             0x0003F2C0, 0xF2414700, 0xF2C0108B, 0x47000003,
                                             0x162CF240, 0xF64F1B92, 0xF2C066ED, 0x47300602,
                                             0xB401B402, 0x10F4F240, 0xFF2AF447, 0xBC014601,
                                             0x0011F100, 0x0001EBA0, 0x54ADF244, 0x3101BC02,
                                             0x47204288, 0x46394630, 0xF808F000, 0xF8B5B282,
                                             0xF24D1094, 0xF2C006FD, 0x47300602, 0x460DB5B0,
                                             0xF4434604, 0x2802FDA7, 0x2801D005, 0x42A5D106,
                                             0x3401BF88, 0x1928E006, 0xE0030844, 0xBF8842A5,
                                             0x462C3D01, 0x436020C8, 0xBDB0B280, 0x461CB5F8,
                                             0x20004603, 0x60102B21, 0x2920BF08, 0xBDF8D000,
                                             0x68014817, 0x29002001, 0x42A1D0F9, 0x7EA5D1F7,
                                             0xF447200C, 0x4285FD85, 0x4F13D11F, 0x25002600,
                                             0xFD5AF443, 0xD20A4286, 0xF8506838, 0xB1200026,
                                             0x0099F890, 0xBF082800, 0x36013501, 0x0628E7F0,
                                             0x4A08D00B, 0xB2E96860, 0x2A007812, 0x220CBF18,
                                             0x0001FB02, 0x4070F020, 0x20016060, 0xBF00BDF8,
                                             0x2040037C, 0x204101F8, 0x20400358, 0xBF022831,
                                             0x3F00F5B1, 0x62904801, 0x47702000, 0x00410201,
                                             0xBF082831, 0x2F00F5B1, 0x4811D11F, 0x22584911,
                                             0x48116001, 0xF8C04911, 0x491113F4, 0x13F8F8C0,
                                             0xF3626801, 0x4A104117, 0x490E6001, 0x4A0F600A,
                                             0x4A0F604A, 0x220D608A, 0xF3626841, 0x2204210E,
                                             0x68416041, 0x0106F362, 0x20006041, 0xBF004770,
                                             0x40090070, 0x20341000, 0x4009048C, 0x00500537,
                                             0x00500350, 0x40090890, 0x00090908, 0x00070504,
                                             0x1CA80907, 0x4770B240,
                                        };
void lcpu_patch_install()
{
    uint32_t entry[3] = {0x48434150, 0x5, LCPU_PATCH_BUF_START_ADDR + 13};
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
