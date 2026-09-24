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
                                             0x00031178, 0xBF54F3DE, 0x0002FEE4, 0xB8AEF3E0,
                                             0x000045A8, 0xB554F00B, 0x0002D0E0, 0xBFCAF3E2,
                                         };
const unsigned int g_lcpu_patch_bin[] = {    0x21184885, 0x48348001, 0x48028001, 0x70012107,
                                             0xB904F000, 0x20400A4C, 0x0005F895, 0xF895B938,
                                             0xB920002D, 0x107FF241, 0x0003F2C0, 0xF2414700,
                                             0xF2C0108B, 0x47000003, 0x162CF240, 0xF64F1B92,
                                             0xF2C066ED, 0x47300602, 0xB401B402, 0x10F4F240,
                                             0xFF46F447, 0xBC014601, 0x0011F100, 0x0001EBA0,
                                             0x54ADF244, 0x3101BC02, 0x47204288, 0x46394630,
                                             0xF834F000, 0xF8B5B282, 0xF24D1094, 0xF2C006FD,
                                             0x47300602, 0x4D0AB570, 0x26002400, 0xFDA4F443,
                                             0xBF244284, 0xBD70B2F0, 0xF8506828, 0xB1200024,
                                             0x0099F890, 0xBF082800, 0x34013601, 0xBF00E7EE,
                                             0x20400358, 0x4807B510, 0xF7FF6804, 0xB144FFE3,
                                             0x4A05B138, 0x88126861, 0x1000FB02, 0x4070F020,
                                             0xBD106060, 0x20400374, 0x204102AC, 0x460DB5B0,
                                             0xF4434604, 0x2802FD97, 0x2801D005, 0x42A5D106,
                                             0x3401BF88, 0x1928E006, 0xE0030844, 0xBF8842A5,
                                             0x462C3D01, 0x436020C8, 0xBDB0B280, 0x4604B510,
                                             0x2C212000, 0xBF086010, 0xD0002920, 0x4618BD10,
                                             0xF85EF000, 0xBD102001, 0xBF082831, 0x3F00F5B1,
                                             0xB580D105, 0xF0004610, 0xE8BDF817, 0x20004080,
                                             0xB5804770, 0xD0072817, 0xBF042831, 0x2F00F5B1,
                                             0xF810F000, 0xBD802000, 0x7F00F5B1, 0xF7FFD1FA,
                                             0x2001FFA9, 0x0000BD80, 0x62814901, 0xBF004770,
                                             0x004101E9, 0x49114810, 0x60012258, 0x49114810,
                                             0x13F4F8C0, 0xF8C04910, 0x680113F8, 0x4117F362,
                                             0x60014A0F, 0x600A490D, 0x604A4A0E, 0x608A4A0E,
                                             0x6841220D, 0x210EF362, 0x60412204, 0xF3626841,
                                             0x60410106, 0xBF004770, 0x40090070, 0x20341000,
                                             0x4009048C, 0x00500537, 0x00500350, 0x40090890,
                                             0x00090908, 0x00070504, 0x1CA80907, 0x4770B240,
                                             0x4604B5B0, 0x6800480B, 0x42A0B190, 0x7EA5D110,
                                             0xF447200C, 0x4285FD15, 0xF7FFD10A, 0xB138FF43,
                                             0x68614A05, 0xFB028812, 0xF0201000, 0x60604070,
                                             0xBDB02000, 0x2040037C, 0x204102AE, 0x490B480A,
                                             0x480B6001, 0x6001490B, 0x490C480B, 0x480C6001,
                                             0x2120F44F, 0x10C4F8C0, 0xF8C02120, 0xF44F1084,
                                             0x65C17100, 0xBF004770, 0x204001D4, 0x00410135,
                                             0x204001F4, 0x0041014F, 0x204001D0, 0x00410119,
                                             0x20400954, 0x20413900, 0x20418000, 0x20418000,
                                             0x20418000, 0x20418000, 0x20418000, 0x20418000,
                                             0x20418000, 0x20418000, 0x20418000, 0x20418000,
                                             0x20418000, 0x20418000, 0x20418000, 0x20418000,
                                        };
void lcpu_patch_install()
{
    uint32_t entry[3] = {0x48434150, 0x7, LCPU_PATCH_BUF_START_ADDR + 13};
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
