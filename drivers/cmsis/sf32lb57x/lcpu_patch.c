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
const unsigned int g_lcpu_patch_list[] = {   0x50544348, 0x00000048, 0x00045A60, 0x46374BBF,
                                             0x00031178, 0xBF5CF3DE, 0x0002FEE4, 0xB8B6F3E0,
                                             0x000045A8, 0xB55CF00B, 0x0002D0E0, 0xBFD2F3E2,
                                             0x000221CC, 0xBF9AF3ED, 0x00030EA4, 0xB98AF3DF,
                                             0x0005E07C, 0x004100A1, 0x00021368, 0xBEB4F3EE,
                                         };
const unsigned int g_lcpu_patch_bin[] = {    0x21184806, 0x485D8001, 0x48718001, 0x70012109,
                                             0x49044803, 0xF0006001, 0xBF00BBA5, 0x2041080A,
                                             0x204001C4, 0x004105D9, 0x0005F895, 0xF895B938,
                                             0xB920002D, 0x107FF241, 0x0003F2C0, 0xF2414700,
                                             0xF2C0108B, 0x47000003, 0x162CF240, 0xF64F1B92,
                                             0xF2C066ED, 0x47300602, 0xB401B402, 0x10F4F240,
                                             0xFF3EF447, 0xBC014601, 0x0011F100, 0x0001EBA0,
                                             0x54ADF244, 0x3101BC02, 0x47204288, 0x46394630,
                                             0xF87EF000, 0xF8B5B282, 0xF24D1094, 0xF2C006FD,
                                             0x47300602, 0x460AB510, 0x5088F44F, 0x230A2100,
                                             0xF4092400, 0x2111FB93, 0x71017041, 0x214CF640,
                                             0x494780C1, 0x70047809, 0x7140F501, 0x80418101,
                                             0xFEECF404, 0xBD102000, 0x460AB570, 0x68094909,
                                             0x01C0EB01, 0xF106684E, 0x46210440, 0xF87AF000,
                                             0xF1064605, 0x46210050, 0xF7F02210, 0x4628F3F6,
                                             0xBF00BD70, 0x20400348, 0xF640B5E0, 0xF8AD224C,
                                             0x22112002, 0x2001F88D, 0xF3622225, 0xF88D015F,
                                             0x492F1000, 0xF5017809, 0xF8AD7140, 0x46691004,
                                             0xFD9EF411, 0xBF00BD8C, 0x4D0AB570, 0x26002400,
                                             0xFD52F443, 0xBF244284, 0xBD70B2F0, 0xF8506828,
                                             0xB1200024, 0x0099F890, 0xBF082800, 0x34013601,
                                             0xBF00E7EE, 0x20400358, 0x4807B510, 0xF7FF6804,
                                             0xB144FFE3, 0x4A05B138, 0x88126861, 0x1000FB02,
                                             0x4070F020, 0xBD106060, 0x20400374, 0x20410808,
                                             0x460DB5B0, 0xF4434604, 0x2802FD45, 0x2801D005,
                                             0x42A5D106, 0x3401BF88, 0x1928E006, 0xE0030844,
                                             0xBF8842A5, 0x462C3D01, 0x436020C8, 0xBDB0B280,
                                             0x4906B5E0, 0x91002200, 0x78094905, 0x7140F501,
                                             0x1004F8AD, 0xF4204669, 0xBD8CFD0F, 0x0A4C110C,
                                             0x20400A4C, 0x4FF0E92D, 0x460FB095, 0xD0632A06,
                                             0xD0722A05, 0xD1782A03, 0xF8DF463B, 0xF81343C8,
                                             0x6825CF06, 0xC050F8CD, 0x6020F855, 0x29007831,
                                             0xF8DFD06D, 0x261F23B4, 0x31017811, 0xF8DF7011,
                                             0xB2C923B0, 0x42917812, 0x8126F200, 0x139CF8DF,
                                             0x29027809, 0x8120F200, 0x1020F855, 0x463D900B,
                                             0x7808930D, 0x8888900C, 0x1F07F815, 0x46A9462C,
                                             0x46AB46A8, 0xF81446AA, 0xF8392F01, 0xF818EF05,
                                             0xF83A6D03, 0x940A3F03, 0x4D02F81B, 0xE400E9CD,
                                             0x6C02E9CD, 0x980C9004, 0xF43D4CD2, 0x4606FB89,
                                             0xF0402800, 0xE9CD80EA, 0x68209808, 0x802CF8DD,
                                             0x9028F8DD, 0xB030F8CD, 0xF8507829, 0xF8990028,
                                             0x88C32000, 0xF43D7800, 0x2800FC1B, 0x49C7D043,
                                             0x0B04F04F, 0xE0622000, 0x1DB91CFC, 0x020CF107,
                                             0x030AF107, 0x3200E9CD, 0x1402E9CD, 0x1D3A1D79,
                                             0x030EF107, 0xFD06F43C, 0x4639E0D3, 0xFB84F43C,
                                             0x28124606, 0x80BBF000, 0x261FE0C6, 0x79F9E0C5,
                                             0x202BF896, 0xF0404291, 0x46808087, 0xE00CF8B7,
                                             0x930D7978, 0x793D897B, 0x88B67A3A, 0xE000E9CD,
                                             0xE9CD2000, 0x96045C02, 0xFB3AF43D, 0x28004606,
                                             0x809BF040, 0xF04079F8, 0x28020002, 0xF04FD15D,
                                             0x26000B00, 0x6820E06C, 0x717AF44F, 0xF85046A3,
                                             0x8AC00028, 0xF2404348, 0xFBB02171, 0xF899F4F1,
                                             0x78281000, 0xFAC4F43C, 0x980C4606, 0x7802990D,
                                             0x19907809, 0x42A04408, 0x4997D941, 0xF04F465C,
                                             0x78080B03, 0x70083801, 0x78084994, 0x70083001,
                                             0x9E096820, 0x1028F850, 0xF8817830, 0xF8910028,
                                             0xF43B002B, 0x498FFCF4, 0x880A462B, 0x46324310,
                                             0x68218008, 0x1028F851, 0xEA2488CC, 0x80C80000,
                                             0xE9CD9908, 0x1CF89A00, 0x990D9102, 0x1003E9CD,
                                             0x4640990C, 0xFFBAF43B, 0x487E4606, 0x68002E00,
                                             0x1028F850, 0x4C7B80CC, 0xF8DDD13F, 0x2600802C,
                                             0xB1987A08, 0x70C878B8, 0x261CE012, 0xF8DBE044,
                                             0xAB140000, 0x0028F850, 0xF8807E01, 0x46306034,
                                             0xFBCFF43D, 0xD0412800, 0xE035261E, 0x70B878C8,
                                             0xF8506820, 0x7A010028, 0x7879B111, 0xE0017081,
                                             0x70797881, 0x1F0AF857, 0x102EF8C0, 0x1C02F817,
                                             0xB005F887, 0x463A463B, 0xF880463C, 0xF813102C,
                                             0xF8801F04, 0xF8521033, 0x62811D06, 0x1D07F814,
                                             0x1032F880, 0x000BEA56, 0x2E12D003, 0x261ED10C,
                                             0x1CB8E00B, 0xE9CD1F79, 0x980D7000, 0x0402E9CD,
                                             0xF43C4640, 0x2600FC37, 0x4857B11E, 0x70012100,
                                             0x2600E000, 0xB0154630, 0x8FF0E8BD, 0xAB14980C,
                                             0x465C4632, 0x46407801, 0xFB8EF43B, 0x95064606,
                                             0x4949B148, 0x38017808, 0x49487008, 0x30017808,
                                             0x20047008, 0x2000E000, 0x20049007, 0x804AF8AD,
                                             0xF8AD793D, 0x68200046, 0x0028F850, 0x79787844,
                                             0x0840EA4F, 0x404CF8AD, 0x8044F8AD, 0xFF4CF41A,
                                             0xA80E4683, 0xF4134621, 0x2000FF8D, 0x4C3578F9,
                                             0x004EF8AD, 0x4458980E, 0x4070F020, 0x60C1EA80,
                                             0xF1F8FBB0, 0xF108FB01, 0xEB0B1A08, 0x44080145,
                                             0x4170F020, 0xF1F8FBB1, 0x0018FB01, 0x802CF8DD,
                                             0x0048F8AD, 0xF448A811, 0x6822F9B3, 0xF852793B,
                                             0xF8811028, 0xB1F03028, 0x0050F89D, 0x78FA2100,
                                             0xF88D2302, 0x79791044, 0xE9CD71B8, 0xAA110200,
                                             0xF43C4640, 0x6820F9DB, 0x1028F850, 0x0044F89D,
                                             0x2028F891, 0xD0034290, 0xF8819A09, 0x70100028,
                                             0x0B02F04F, 0x9807E734, 0xD1042804, 0x9D069E09,
                                             0x0B04F04F, 0x9D0DE703, 0x782B9814, 0xD1024298,
                                             0x0B00F04F, 0x2B06E724, 0xB17BD802, 0xB96B7E0B,
                                             0x70284B09, 0x0B04F04F, 0x39017819, 0x4B077019,
                                             0x31017819, 0xF8527019, 0xE7111028, 0xF04F7028,
                                             0xE70D0B01, 0x20400354, 0x20400295, 0x20400297,
                                             0x20400296, 0x204002CA, 0x20400298, 0x28254902,
                                             0x2100BF18, 0x47704608, 0x004107C0, 0x4605B57C,
                                             0x0A0C4819, 0xF8506800, 0x46086024, 0xF976F409,
                                             0xD8072819, 0xFA012101, 0x4914F000, 0xD0014208,
                                             0xBD7C2000, 0x0002F8D5, 0x67B02126, 0xF8867868,
                                             0x480F007C, 0xF5007800, 0xF8AD7040, 0xF6400004,
                                             0xF8AD204C, 0x20110002, 0x0001F88D, 0xF3617828,
                                             0x4669005F, 0x0000F88D, 0xF4114620, 0x2001FB11,
                                             0x008AF886, 0xBF00E7DC, 0x20400344, 0x03C00001,
                                             0x20400A4C, 0x4604B510, 0x2C212000, 0xBF086010,
                                             0xD0002920, 0x4618BD10, 0xF85EF000, 0xBD102001,
                                             0xBF082831, 0x3F00F5B1, 0xB580D105, 0xF0004610,
                                             0xE8BDF817, 0x20004080, 0xB5804770, 0xD0072817,
                                             0xBF042831, 0x2F00F5B1, 0xF810F000, 0xBD802000,
                                             0x7F00F5B1, 0xF7FFD1FA, 0x2001FD57, 0x0000BD80,
                                             0x62814901, 0xBF004770, 0x00410731, 0x49114810,
                                             0x60012258, 0x49114810, 0x13F4F8C0, 0xF8C04910,
                                             0x680113F8, 0x4117F362, 0x60014A0F, 0x600A490D,
                                             0x604A4A0E, 0x608A4A0E, 0x6841220D, 0x210EF362,
                                             0x60412204, 0xF3626841, 0x60410106, 0xBF004770,
                                             0x40090070, 0x20341000, 0x4009048C, 0x00500537,
                                             0x00500350, 0x40090890, 0x00090908, 0x00070504,
                                             0x1CA80907, 0x4770B240, 0x4604B5B0, 0x6800480B,
                                             0x42A0B190, 0x7EA5D110, 0xF447200C, 0x4285FA71,
                                             0xF7FFD10A, 0xB138FCF1, 0x68614A05, 0xFB028812,
                                             0xF0201000, 0x60604070, 0xBDB02000, 0x2040037C,
                                             0x2041080A, 0x490B480A, 0x480B6001, 0x6001490B,
                                             0x490C480B, 0x480C6001, 0x2120F44F, 0x10C4F8C0,
                                             0xF8C02120, 0xF44F1084, 0x65C17100, 0xBF004770,
                                             0x204001D4, 0x0041067D, 0x204001F4, 0x00410697,
                                             0x204001D0, 0x00410661, 0x20400954, 0x48484242,
                                             0x00000000, 0x004105E9, 0x004107B8, 0x000000C6,
                                             0x20413900, 0x20418000, 0x20418000, 0x20418000,
                                             0x20418000, 0x20418000, 0x20418000, 0x20418000,
                                             0x20418000, 0x20418000, 0x20418000, 0x20418000,
                                             0x20418000, 0x20418000, 0x20418000,
                                        };
void lcpu_patch_install()
{
    uint32_t entry[3] = {0x48434150, 0x9, LCPU_PATCH_BUF_START_ADDR + 13};
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
