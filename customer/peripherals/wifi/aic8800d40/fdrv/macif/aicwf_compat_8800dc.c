/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "aicwf_compat_8800dc.h"
#include "aic_plat_log.h"
#include "rwnx_main.h"
#include "sdio_port.h"
#include "wifi_def.h"

#define FW_PATCH_DPD_LOG 0

typedef u32(*array2_tbl_t)[2];
typedef u32(*array3_tbl_t)[3];

u32 syscfg_tbl_8800dc[][2] =
{
    {0x40500010, 0x00000004},
    {0x40500010, 0x00000006},//160m clk
};

u32 syscfg_tbl_8800dc_sdio_u01[][2] =
{
    {0x40030000, 0x00036724}, // loop forever after assert_err
    {0x0011E800, 0xE7FE4070},
    {0x40030084, 0x0011E800},
    {0x40030080, 0x00000001},
    {0x4010001C, 0x00000000},
};

u32 syscfg_tbl_8800dc_sdio_u02[][2] =
{
    {0x40030000, 0x00036DA4}, // loop forever after assert_err
    {0x0011E800, 0xE7FE4070},
    {0x40030084, 0x0011E800},
    {0x40030080, 0x00000001},
    {0x4010001C, 0x00000000},
};

#ifdef CONFIG_OOB
u32 oobcfg_tbl_8800dc_sdio_u02[][2] =
{
    {0x40504044, 0x2},//oob_enable
    {0x40500060, 0x03020700},
    {0x40500040, 0},
    {0x40100030, 1},
    {0x40241020, 1},
    {0x402400f0, 0x340022},
};
#endif //CONFIG_OOB

u32 syscfg_tbl_masked_8800dc[][3] =
{
#if defined(CONFIG_VRF_DCDC_MODE)
    {0x7000216C, (0x3 << 2), (0x1 << 2)}, // pmic_pmu_init
    {0x700021BC, (0x3 << 2), (0x1 << 2)},
    {0x70002118, ((0x7 << 4) | (0x1 << 7)), ((0x2 << 4) | (0x1 << 7))},
    {0x70002104, ((0x3F << 0) | (0x1 << 6)), ((0x2 << 0) | (0x1 << 6))},
    {0x7000210C, ((0x3F << 0) | (0x1 << 6)), ((0x2 << 0) | (0x1 << 6))},
    {0x70002170, (0xF << 0), (0x1 << 0)},
    {0x70002190, (0x3F << 0), (24 << 0)},
    {0x700021CC, ((0x7 << 4) | (0x1 << 7)), ((0x0 << 4) | (0x0 << 7))},
    {0x700010A0, (0x1 << 11), (0x1 << 11)},
    {0x70001034, ((0x1 << 20) | (0x7 << 26)), ((0x0 << 20) | (0x2 << 26))},
    {0x70001038, (0x1 << 8), (0x1 << 8)},
    {0x70001094, (0x3 << 2), (0x0 << 2)},
    {0x700021D0, ((0x1 << 5) | (0x1 << 6)), ((0x1 << 5) | (0x1 << 6))},
    {
        0x70001000, ((0x1 << 0) | (0x1 << 20) | (0x1 << 22)),
        ((0x1 << 0) | (0x1 << 20) | (0x0 << 22))
    },
    {0x70001028, (0xf << 2), (0x1 << 2)},
#else
    {0x7000216C, (0x3 << 2), (0x1 << 2)}, // pmic_pmu_init
    {0x700021BC, (0x3 << 2), (0x1 << 2)},
    {0x70002118, ((0x7 << 4) | (0x1 << 7)), ((0x2 << 4) | (0x1 << 7))},
    {0x70002104, ((0x3F << 0) | (0x1 << 6)), ((0x2 << 0) | (0x1 << 6))},
    {0x7000210C, ((0x3F << 0) | (0x1 << 6)), ((0x2 << 0) | (0x1 << 6))},
    {0x70002170, (0xF << 0), (0x1 << 0)},
    {0x70002190, (0x3F << 0), (24 << 0)},
    {0x700021CC, ((0x7 << 4) | (0x1 << 7)), ((0x0 << 4) | (0x0 << 7))},
    {0x700010A0, (0x1 << 11), (0x1 << 11)},
    {0x70001034, ((0x1 << 20) | (0x7 << 26)), ((0x0 << 20) | (0x2 << 26))},
    {0x70001038, (0x1 << 8), (0x1 << 8)},
    {0x70001094, (0x3 << 2), (0x0 << 2)},
    {0x700021D0, ((0x1 << 5) | (0x1 << 6)), ((0x1 << 5) | (0x1 << 6))},
    {
        0x70001000, ((0x1 << 0) | (0x1 << 20) | (0x1 << 22)),
        ((0x0 << 0) | (0x1 << 20) | (0x0 << 22))
    },
    {0x70001028, (0xf << 2), (0x1 << 2)},
#endif
    {0x00000000, 0x00000000, 0x00000000}, // last one
};

u32 syscfg_tbl_masked_8800dc_h[][3] =
{
    {0x7000216C, ((0x3 << 2) | (0x3 << 4)), ((0x2 << 2) | (0x2 << 4))}, // pmic_pmu_init
    {0x70002138, (0xFF << 0), (0xFF << 0)},
    {0x7000213C, (0xFF << 0), (0xFF << 0)},
    {0x70002144, (0xFF << 0), (0xFF << 0)},
    {0x700021BC, (0x3 << 2), (0x1 << 2)},
    {0x70002118, ((0x7 << 4) | (0x1 << 7)), ((0x2 << 4) | (0x1 << 7))},
    {0x70002104, ((0x3F << 0) | (0x1 << 6)), ((0x2 << 0) | (0x1 << 6))},
    {0x7000210C, ((0x3F << 0) | (0x1 << 6)), ((0x2 << 0) | (0x1 << 6))},
    {0x70002170, (0xF << 0), (0x1 << 0)},
    {0x70002190, (0x3F << 0), (24 << 0)},
    {0x700021CC, ((0x7 << 4) | (0x1 << 7)), ((0x0 << 4) | (0x0 << 7))},
    {0x700010A0, (0x1 << 11), (0x1 << 11)},
    //{0x70001034, ((0x1 << 20) | (0x7 << 26)), ((0x0 << 20) | (0x2 << 26))},
    {0x70001038, (0x1 << 8), (0x1 << 8)},
    {0x70001094, (0x3 << 2), (0x0 << 2)},
    {0x700021D0, ((0x1 << 5) | (0x1 << 6)), ((0x1 << 5) | (0x1 << 6))},
#if defined(CONFIG_VRF_DCDC_MODE)
    {
        0x70001000, ((0x1 << 0) | (0x1 << 20) | (0x1 << 22)),
        ((0x1 << 0) | (0x1 << 20) | (0x0 << 22))
    },
#else
    {
        0x70001000, ((0x1 << 0) | (0x1 << 20) | (0x1 << 22)),
        ((0x0 << 0) | (0x1 << 20) | (0x0 << 22))
    },
#endif
    {0x70001028, (0xf << 2), (0x1 << 2)},

    {0x00000000, 0x00000000, 0x00000000}, // last one
};

u32 syscfg_tbl_masked_8800dc_u01[][3] =
{
    //#ifdef CONFIG_PMIC_SETTING
    {0x70001000, (0x1 << 16), (0x1 << 16)}, // for low temperature
    {0x70001028, (0x1 << 6), (0x1 << 6)},
    {0x70001000, (0x1 << 16), (0x0 << 16)},
    //#endif /* CONFIG_PMIC_SETTING */
};

uint32_t ldpc_cfg_ram[] =
{
    0x00767679,
    0x1DF8F870,
    0x1DF8F870,
    0x1DF8F870,
    0x1DF8F870,
    0x006E6E72,
    0x1DF8F869,
    0x1DF8F869,
    0x1DF8F869,
    0x1DF8F869,
    0x0076767B,
    0x1DF8F870,
    0x1DF8F870,
    0x1DF8F870,
    0x1DF8F870,
    0x007E7E85,
    0x1DF4F876,
    0x1DF4F876,
    0x1DF4F876,
    0x1DF8F876,
    0x0081818A,
    0x1DF8F87B,
    0x1DF8F87B,
    0x1DF8F87B,
    0x1DF8F87B,
    0x0081818D,
    0x1DF8F87B,
    0x1DF8F87B,
    0x1DF8F87B,
    0x1DF8F87B,
    0x0081818A,
    0x1DF8F87B,
    0x1DF8F87C,
    0x1DF8F87B,
    0x1DF8F87B,
    0x007E7E40,
    0x1DF8F87B,
    0x1DF8F87B,
    0x1DF8F87B,
    0x1DF8F87B,
    0x008B8B92,
    0x1DF8F887,
    0x1DF8F889,
    0x1DF8F887,
    0x1DF8F887,
    0x00515155,
    0x1DF8F84C,
    0x1DF8F84C,
    0x1DF8F889,
    0x1DF8F889,
    0x00515154,
    0x1DF8F84C,
    0x1DF8F84C,
    0x1DF8F888,
    0x1DF8F888,
    0x004F4F53,
    0x1DF8F84A,
    0x1DF8F84A,
    0x1DF8F84A,
    0x1DF8F84A,
    0x004F4F53,
    0x1DF8F84A,
    0x1DF8F84A,
    0x1DF8F84A,
    0x1DF8F84A,
    0x004F4F53,
    0x1DF8F84A,
    0x1DF8F84A,
    0x1DF8F84A,
    0x1DF8F84A,
    0x004E4E53,
    0x1DF8F849,
    0x1DF8F848,
    0x1DF8F848,
    0x1DF8F848,
    0x004D4D52,
    0x1DF8F847,
    0x1DF8F847,
    0x1DF8F847,
    0x1DF8F847,
    0x004F4F55,
    0x1DF8F84B,
    0x1DF8F84A,
    0x1DF8F84A,
    0x1DF8F84A,
    0x004E4E53,
    0x1DF8F849,
    0x1DF8F848,
    0x1DF8F848,
    0x1DF8F848,
    0x0049494D,
    0x1DF8F844,
    0x1DF8F844,
    0x1DF8F844,
    0x1DF8F844,
    0x0051518F,
    0x1DF8F849,
    0x1DF8F848,
    0x1DF8F848,
    0x1DF8F848,
    0x00424277,
    0x1DF8F83F,
    0x1DF8F83C,
    0x1DF8F83C,
    0x1DF8F83C,
    0x00424275,
    0x1DF8F89E,
    0x1DF8F83C,
    0x1DF8F83C,
    0x1DF8F83C,
    0x0055555C,
    0x1DF8F84C,
    0x1DF8F84C,
    0x1DF8F84C,
    0x1DF8F84C,
    0x0053535C,
    0x1DF8F84C,
    0x1DF8F84B,
    0x1DF8F84B,
    0x1DF8F84B,
    0x00F8F89E,
    0x1DF8F88C,
    0x1DF8F84A,
    0x1DF8F84A,
    0x1DF8F84A,
    0x00898940,
    0x18F8F846,
    0x18CFF845,
    0x18CFF844,
    0x18CFF844,
    0x0056565F,
    0x1DF8F84F,
    0x1DF8F84F,
    0x1DF8F84F,
    0x1DF8F84F,
    0x0055555E,
    0x1DF8F84E,
    0x1DF8F84E,
    0x1DF8F84E,
    0x1DF8F84E,
    0x0056565F,
    0x1DF8F84F,
    0x1DF8F84F,
    0x1DF8F84F,
    0x1DF8F84F,
    0x00555561,
    0x1DF8F850,
    0x1DF8F84E,
    0x1DF8F84E,
    0x1DF8F84E,
    0x0053535F,
    0x1DF8F84D,
    0x1DF8F84C,
    0x1DF8F84C,
    0x1DF8F84C,
    0x0055555F,
    0x1DF8F84F,
    0x1DF8F84E,
    0x1DF8F84E,
    0x1DF8F84E,
    0x005555AA,
    0x1DF8F854,
    0x1DF8F84E,
    0x1DF8F84E,
    0x1DF8F84E,
    0x005959A6,
    0x1DF8F84D,
    0x1DF8F84C,
    0x1DF8F84C,
    0x1DF8F84C,
    0x004F4F9B,
    0x1DF8F84E,
    0x1DF8F846,
    0x1DF8F846,
    0x1DF8F846,
    0x00F8F8A5,
    0x1DF8F894,
    0x1DF8F84C,
    0x1DF8F84C,
    0x1DF8F84C,
    0x009898A4,
    0x1DF8F84D,
    0x1DF8F84C,
    0x1DF8F84C,
    0x1DF8F84C,
    0x00464686,
    0x1DF8F8B3,
    0x1DF8F83D,
    0x1DF8F83D,
    0x1DF8F83D,
    0x008E8E40,
    0x1AF8F848,
    0x1ADFF848,
    0x1ADFF846,
    0x1ADFF846,
    0x007F7F40,
    0x18D2D275,
    0x18D2D23A,
    0x18D2D23A,
    0x18D2D239,
    0x00454540,
    0x0F868664,
    0x0F86863E,
    0x0F86863D,
    0x0F86863D,
    0x005C5C64,
    0x1DF8F856,
    0x1DF8F855,
    0x1DF8F855,
    0x1DF8F855,
    0x005B5B68,
    0x1DF8F858,
    0x1DF8F855,
    0x1DF8F855,
    0x1DF8F855,
    0x005A5A64,
    0x1DF8F855,
    0x1DF8F854,
    0x1DF8F854,
    0x1DF8F854,
    0x005A5AB5,
    0x1DF8F85B,
    0x1DF8F855,
    0x1DF8F854,
    0x1DF8F854,
    0x00F8F8B0,
    0x1DF8F8A3,
    0x1DF8F852,
    0x1DF8F852,
    0x1DF8F852,
    0x00A4A4AE,
    0x1DF8F854,
    0x1DF8F852,
    0x1DF8F852,
    0x1DF8F852,
    0x009A9A40,
    0x1DF8F84E,
    0x1DF8F84D,
    0x1DF8F84C,
    0x1DF8F84C,
    0x009C9C40,
    0x1DF8F895,
    0x1DF8F849,
    0x1DF8F84A,
    0x1DF8F84A,
    0x00494940,
    0x1197976F,
    0x11979742,
    0x11979741,
    0x11979741,
    0x006E6E74,
    0x1DF8F869,
    0x1DF8F869,
    0x1DF8F869,
    0x1DF8F869,
    0x006E6E40,
    0x1ADEF869,
    0x1ADEF869,
    0x1ADEF869,
    0x1ADEF869,
    0x00757540,
    0x0D78F86E,
    0x0D78F86E,
    0x0D78F86E,
    0x0D79F86E,
    0x00787885,
    0x1DF8F873,
    0x1DF8F873,
    0x1DF8F873,
    0x1DF8F873,
    0x00787840,
    0x1DF8F873,
    0x1DF8F873,
    0x1DF8F873,
    0x1DF8F873,
    0x00787840,
    0x0E81F873,
    0x0E81F873,
    0x0E81F873,
    0x0E82F873,
    0x00404040,
    0x0E82F873,
    0x0E82F873,
    0x0E82F873,
    0x0E82F873,
    0x00818140,
    0x1092F87E,
    0x1092F87E,
    0x1092F87E,
    0x1092F87E,
    0x00404040,
    0x1092F87E,
    0x1092F87E,
    0x1092F87E,
    0x1092F87E,
    0x00737340,
    0x14B2B26B,
    0x14B2B235,
    0x14B2B235,
    0x14B2B235,
    0x00404040,
    0x0E828260,
    0x0E82823D,
    0x0E82823C,
    0x0E82823C,
    0x00404040,
    0x0F8B8B66,
    0x0F8B8B3F,
    0x0F8B8B3D,
    0x0F8B8B3D,
    0x00404040,
    0x0B68683D,
    0x0B68681E,
    0x0B68681E,
    0x0B68681E,
    0x00222240,
    0x06434318,
    0x06434329,
    0x06434318,
    0x06434318,
    0x00404040,
    0x129D9D72,
    0x129D9D43,
    0x129D9D41,
    0x129D9D41,
    0x00404040,
    0x0D757542,
    0x0D757520,
    0x0D757520,
    0x0D757520,
    0x00232340,
    0x084C4C19,
    0x084C4C2C,
    0x084C4C19,
    0x084C4C19
};

uint32_t agc_cfg_ram[] =
{
    0x20000000,
    0x0400000E,
    0x3000200E,
    0x5B000000,
    0x0400004B,
    0x3000008E,
    0x32000000,
    0x0400007B,
    0x40000000,
    0xF8000026,
    0x04000011,
    0x4819008E,
    0x9C000020,
    0x08000191,
    0x38008000,
    0x0A000000,
    0x08104411,
    0x38018000,
    0x0C004641,
    0x08D00014,
    0x30000000,
    0x01000000,
    0x04000017,
    0x30000000,
    0x3C000000,
    0x0400001A,
    0x38020000,
    0x40000001,
    0x0800001D,
    0x3808008E,
    0x14000050,
    0x08000020,
    0x4000008E,
    0xA400007B,
    0x00000101,
    0x3000339F,
    0x41000700,
    0x04104420,
    0x90000000,
    0x49000000,
    0xF00E842F,
    0xEC0E842C,
    0xEC0E842C,
    0x04000032,
    0x30000000,
    0x48000101,
    0x04000032,
    0x30000000,
    0x48000202,
    0x04000032,
    0x30000000,
    0x46000000,
    0x04000011,
    0x58010006,
    0x3D040472,
    0xDC204439,
    0x081DD4D2,
    0x480A0006,
    0xDC2044DC,
    0x081DD43C,
    0x38050004,
    0x0EF1F1C3,
    0x342044DC,
    0x30000000,
    0x01000000,
    0x04000042,
    0x30000000,
    0x33000000,
    0x04104445,
    0x38008000,
    0x2200109C,
    0x08104448,
    0x38008000,
    0x23D4509C,
    0x08104417,
    0x9000A000,
    0x32000000,
    0x18000063,
    0x14000060,
    0x1C000051,
    0x10000057,
    0x38028000,
    0x0C000001,
    0x08D04466,
    0x3000200F,
    0x00000000,
    0x00000000,
    0x38030000,
    0x0C002601,
    0x08D0445A,
    0x30000000,
    0x3D020230,
    0x0400005D,
    0x30000000,
    0x3E000100,
    0x04000066,
    0x38028000,
    0x0C001601,
    0x34204466,
    0x38028000,
    0x0C000A01,
    0x34204466,
    0x38008004,
    0xFF000000,
    0x0800007B,
    0x3800802F,
    0x26000000,
    0x0800006C,
    0x380404AF,
    0x1F191010,
    0x0800006F,
    0x20000CAF,
    0x04000071,
    0x60000CAF,
    0x18700079,
    0x14000077,
    0x10000075,
    0x28140CAF,
    0x09B00084,
    0x280A0CAF,
    0x09B00084,
    0x28060CAF,
    0x09B00084,
    0x28048086,
    0x0800007D,
    0x38000086,
    0x22800000,
    0x04000080,
    0x30000000,
    0x0EF1F101,
    0x36004883,
    0x28020000,
    0x08000085,
    0x3802008E,
    0x3D040431,
    0x08000088,
    0x3805008E,
    0x1F241821,
    0x0800008B,
    0x3000008E,
    0xA0163021,
    0x0400008E,
    0x3000008E,
    0x0EF10012,
    0x34000091,
    0x300000CC,
    0x50000000,
    0x04000094,
    0x380095FE,
    0x32010000,
    0x04000097,
    0x50001FFE,
    0x5A010000,
    0x6DC9989B,
    0xFC19D4B9,
    0x30000186,
    0x3D840373,
    0x0400009E,
    0x3000008E,
    0x0A000000,
    0x040000A1,
    0x3000008E,
    0x22C00000,
    0x040000A4,
    0x9000028E,
    0x32010001,
    0x8E4000AA,
    0xC80000B0,
    0x00000000,
    0x00000000,
    0x3000008E,
    0x32010001,
    0x040000CB,
    0x3000008E,
    0x29000000,
    0x94045011,
    0x300019B6,
    0x32010000,
    0x040000B3,
    0x300019B6,
    0x3D040431,
    0x040000B6,
    0x300019B6,
    0x22800000,
    0x04000097,
    0x30000186,
    0x3D840473,
    0x040000BC,
    0x3000008E,
    0x29030000,
    0x040000BF,
    0x9AEE028E,
    0x32010100,
    0x7C0000C5,
    0xCC0000B0,
    0x080000B0,
    0x00000000,
    0x3000008E,
    0x32010100,
    0x040000C8,
    0x3000028E,
    0x29000000,
    0x94045011,
    0x5000038E,
    0x29000000,
    0x94045011,
    0xC0000035,
    0x38010006,
    0x3D040472,
    0x080000D2,
    0x30000004,
    0x0EF1F141,
    0x340000D5,
    0x28040004,
    0x080000D7,
    0x2808000E,
    0x080000D9,
    0x3000018E,
    0x0EF10052,
    0x340000DC,
    0x3000038E,
    0x29000000,
    0x94045011,
    0x38020000,
    0x32000000,
    0x080000E2,
    0x60000000,
    0xD80000E6,
    0xD40000E9,
    0x040000EC,
    0x30000000,
    0x0EF1F121,
    0x360048EF,
    0x30000000,
    0x0C002421,
    0x360048EF,
    0x30000000,
    0x0C000021,
    0x360048EF,
    0x28020000,
    0x0800007B,
    0x50001EFE,
    0x5A010000,
    0x6DC998F5,
    0xFC19D4F8,
    0x3000028E,
    0x32000040,
    0x040000FB,
    0x3AEE028E,
    0x32000080,
    0x040000FB,
    0x30000000,
    0x0EF1F101,
    0x360048FE,
    0x28020000,
    0x08000100,
    0x3802008E,
    0x3D040431,
    0x08000103,
    0x3805008E,
    0x1F241821,
    0x08000106,
    0x3000008E,
    0xA0163021,
    0x04000109,
    0x3000008E,
    0x0EF10012,
    0x3400010C,
    0x300014F6,
    0x32010000,
    0x04000114,
    0x20000000,
    0x04000111,
    0x300000EC,
    0x50000000,
    0x040000F1,
    0x300014F6,
    0x32030000,
    0x04000117,
    0x30001086,
    0x3D840473,
    0x0400011A,
    0x5000108E,
    0x22C00000,
    0x8E47C0CB,
    0xCB30011E,
    0x300019B6,
    0x32040000,
    0x04000121,
    0x300019B6,
    0x3D040431,
    0x04000124,
    0x300019B6,
    0x22800000,
    0x04000111,
    0x00000000,
    0x00000000,
    0x00000000,
    0x30000186,
    0x3D840473,
    0x0400012D,
    0x5000038E,
    0x29000000,
    0x94045011,
    0xC0000131,
    0x380C800E,
    0xFF000000,
    0x08000134,
    0x30000004,
    0x0FF1F103,
    0x34000137,
    0x28020000,
    0x08000139,
    0x3000038E,
    0x29000000,
    0x94045011,
    0x00000000,
    0x00000000,
    0x00000000,
    0x58010006,
    0x3D040472,
    0xDC204543,
    0x081DD4D2,
    0x480A0006,
    0xDC2044DC,
    0x081DD546,
    0x38050004,
    0x0EF1F141,
    0x342044DC,
    0x2802800E,
    0x080000DC,
    0x48000035,
    0x0400014A,
    0x7896638F,
    0x4100000F,
    0x8C00014F,
    0x080450C4,
    0x90104574,
    0x88C8620F,
    0xC000015A,
    0x90104574,
    0x08104554,
    0x94104557,
    0x3000628F,
    0x29000000,
    0x9404517A,
    0x3000638F,
    0x29000000,
    0x0410457A,
    0x3800E005,
    0x3D010131,
    0x0810455D,
    0xA832600F,
    0x90104574,
    0x08000154,
    0x94104557,
    0xC6104567,
    0xC4185563,
    0x5802E00F,
    0x0FEEEA07,
    0x80000174,
    0x3420456B,
    0x5802E00F,
    0x0EEEEA07,
    0x80000174,
    0x3420456B,
    0x30004000,
    0x33000001,
    0x0400016E,
    0x38034005,
    0x3D030373,
    0x08000171,
    0x30006007,
    0x33000000,
    0x04000174,
    0x3000608F,
    0x29000000,
    0x94045177,
    0x4000608F,
    0xA010457D,
    0x0410457A,
    0x3000608F,
    0x64000101,
    0x04104411,
    0x3000608F,
    0x64000101,
    0x04104580,
    0x3000618F,
    0x42000001,
    0x04000183,
    0x38028000,
    0x32000000,
    0x08104586,
    0x280A618F,
    0x08000188,
    0x480A618F,
    0xBC00018B,
    0x0800018E,
    0x3000618F,
    0x34000001,
    0x04000005,
    0x3000618F,
    0x34000000,
    0x04000008,
    0x3000008F,
    0x0EEAED0F,
    0x36000194,
    0x38038000,
    0x34000000,
    0x08000197,
    0x38028005,
    0x29010002,
    0x0800019A,
    0x3000028F,
    0x2200209C,
    0x0400019D,
    0x3000028F,
    0x23D4509C,
    0x040001A0,
    0x2814028F,
    0x080001A2,
    0x3000028F,
    0x43010201,
    0x040001A5,
    0x3000128F,
    0x32000100,
    0x040001A8,
    0x5AEE138F,
    0x4100000F,
    0x7C0001AC,
    0x080000F9,
    0x592C138F,
    0x29000000,
    0x8C0001B0,
    0x080000F9,
    0x2000138F,
    0x94045011,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000
};

uint32_t txgain_map[96] =
{
    //11b
    0x00ffd780,
    0x00ffd872,
    0x00ffd880,
    0x00ffd972,
    0x00ffd980,
    0x00ffda75,
    0x00ffda86,
    0x00ffdb77,
    0x00ffdb86,
    0x00ffdc78,
    0x00ffdc89,
    0x00ffdd79,
    0x00ffdd89,
    0x00ffde83,
    0x00ffdf79,
    0x00ffdf8b,
    0x00ffd072,
    0x00ffd072,
    0x00ffd080,
    0x00ffd172,
    0x00ffd180,
    0x00ffd272,
    0x00ffd280,
    0x00ffd36d,
    0x00ffd379,
    0x00ffd46d,
    0x00ffd479,
    0x00ffd572,
    0x00ffd580,
    0x00ffd672,
    0x00ffd680,
    0x00ffd772,
    //high
    0x00ffc87d,
    0x00ffc88b,
    0x00ffc979,
    0x00ffc989,
    0x00ffca7d,
    0x00ffca88,
    0x00ffcc5e,
    0x00ffcc69,
    0x00ffcc78,
    0x00ffcc85,
    0x00ffcd70,
    0x00ffcd80,
    0x00ffce70,
    0x00ffce80,
    0x00ffcf7d,
    0x00ffcf90,
    0x00ffc080,
    0x00ffc090,
    0x00ffc180,
    0x00ffc190,
    0x00ffc27b,
    0x00ffc28b,
    0x00ffc37b,
    0x00ffc390,
    0x00ffc485,
    0x00ffc495,
    0x00ffc579,
    0x00ffc589,
    0x00ffc679,
    0x00ffc689,
    0x00ffc780,
    0x00ffc790,
    //low
    0x00ffc87d,
    0x00ffc88b,
    0x00ffc979,
    0x00ffc989,
    0x00ffca7d,
    0x00ffca88,
    0x00ffcc5e,
    0x00ffcc69,
    0x00ffcc78,
    0x00ffcc85,
    0x00ffcd70,
    0x00ffcd80,
    0x00ffcd90,
    0x00ffcf68,
    0x00ffcf75,
    0x00ffcf83,
    0x00ffc080,
    0x00ffc090,
    0x00ffc180,
    0x00ffc190,
    0x00ffc27b,
    0x00ffc28b,
    0x00ffc37b,
    0x00ffc390,
    0x00ffc485,
    0x00ffc495,
    0x00ffc579,
    0x00ffc589,
    0x00ffc679,
    0x00ffc689,
    0x00ffc780,
    0x00ffc790,
};

const uint32_t txgain_map_h[96] =
{
    //11b
    0xffd888, //11
    0xffd979, //12
    0xffd988, //13
    0xffda79, //14
    0xffda88, //15
    0xffdb79, //16
    0xffdb88, //17
    0xffdc72, //18
    0xffdc80, //19
    0xffdd80, //20
    0xffde66, //21
    0xffde72, //22
    0xffde80, //23
    0xffdf79, //24
    0xffdf88, //25
    0xffdf98, //26
    0xffd079, //-5
    0xffd088, //-4
    0xffd179, //-3
    0xffd188, //-2
    0xffd288, //-1
    0xffd36c, //0
    0xffd379, //1
    0xffd388, //2
    0xffd479, //3
    0xffd488, //4
    0xffd579, //5
    0xffd588, //6
    0xffd679, //7
    0xffd688, //8
    0xffd779, //9
    0xffd879, //10
    //high
    0xffc879, //8
    0xffc96b, //9
    0xffc979, //10
    0xffcc45, //11
    0xffcc4d, //12
    0xffcc56, //13
    0xffcc60, //14
    0xffcc6b, //15
    0xffcc79, //16
    0xffcd72, //17
    0xffce60, //18
    0xffce72, //19
    0xffcf72, //20
    0xffcf80, //21
    0xffcf90, //22
    0xffcf90, //23
    0xffc079, //-8
    0xffc16b, //-7
    0xffc179, //-6
    0xffc26b, //-5
    0xffc279, //-4
    0xffc36b, //-3
    0xffc379, //-2
    0xffc46b, //-1
    0xffc479, //0
    0xffc56b, //1
    0xffc579, //2
    0xffc66b, //3
    0xffc679, //4
    0xffc76b, //5
    0xffc779, //6
    0xffc86b, //7
    //low
    0xffc879, //8
    0xffc96b, //9
    0xffc979, //10
    0xffcc45, //11
    0xffcc4d, //12
    0xffcc56, //13
    0xffcc60, //14
    0xffcc6b, //15
    0xffcc79, //16
    0xffcd72, //17
    0xffcf5b, //18
    0xffcf66, //19
    0xffcf72, //20
    0xffcf80, //21
    0xffcf90, //22
    0xffcf90, //23
    0xffc079, //-8
    0xffc16b, //-7
    0xffc179, //-6
    0xffc26b, //-5
    0xffc279, //-4
    0xffc36b, //-3
    0xffc379, //-2
    0xffc46b, //-1
    0xffc479, //0
    0xffc56b, //1
    0xffc579, //2
    0xffc66b, //3
    0xffc679, //4
    0xffc76b, //5
    0xffc779, //6
    0xffc86b, //7
};


u32 patch_tbl_wifisetting_8800dc_u01[][2] =
{
    {0x010c, 0x01001E01}
};

u32 patch_tbl_wifisetting_8800dc_u02[][2] =
{
#if defined(DCONFIG_SDIO_USR_AP_CLK) || defined(CONFIG_SDIO_BUS_PWRCTRL)
    {0x0124, 0x01011E01},
#else
    {0x0124, 0x01001E01},
#endif

#if FW_PATCH_DPD_LOG
    // wifi_setting_base: 0x11019c
    {0x00B0, 0xAF190100},    // 11024c: uint8_t lp_level; 11024e: uint16_t debug_mask;
#endif /* FW_PATCH_DPD_LOG */
};

u32 adaptivity_patch_tbl_8800dc[][2] =
{
    {0x000C, 0x0000320A}, //linkloss_thd
    {0x009C, 0x00000000}, //ac_param_conf
    {0x0128, 0xF6140001}, //tx_adaptivity_en
};

u32 jump_tbl[][2] =
{
#ifndef CONFIG_FOR_IPCOM
    {296, 0x180001},
    {137, 0x180011},
    {303, 0x1810f9},
    {168, 0x18186d},
    {308, 0x181bbd},
    {288, 0x1820c1},
#else
    {308, 0x181001},
    {288, 0x181031},
    {296, 0x18120d},
    {137, 0x18121d},
    {303, 0x182305},
    {168, 0x182a79},
    {258, 0x182ae1},
#endif
};

u32 jump_tbl_u02[][2] =
{
    {303, 0x00180d25},
    {168, 0x001814a5},
    {265, 0x001816b1},
    {266, 0x00181849},
    {256, 0x001818ad},
    {288, 0x00181bf9},
    {333, 0x00182d0d},
    { 26, 0x00182d45}
};

u32 patch_tbl_func[][2] =
{
#ifndef CONFIG_FOR_IPCOM
    {0x00110054, 0x0018186D}, // same as jump_tbl idx 168
    {0x0011005C, 0x0018186D}, // same as jump_tbl idx 168
#else
    {0x00110054, 0x00182A79}, // same as jump_tbl idx 168
    {0x0011005C, 0x00182A79}, // same as jump_tbl idx 168
    {0x001118D4, 0x00000011},
#endif
};

u32 patch_tbl_func_u02[][2] =
{
    {0x00110054, 0x001814a5}, // same as jump_tbl idx 168
    {0x0011005C, 0x001814a5}, // same as jump_tbl idx 168
    {0x001109c0, 0x00181e3d},
    {0x00110bb4, 0x001824e1},
    {0x00110f08, 0x00182d25},
};

u32 patch_tbl_rf_func[][2] =
{
    {0x00110bf0, 0x00180001},
};

void aicwf_patch_wifi_setting_8800dc(struct rwnx_hw *rwnx_hw)
{
#if FW_PATCH_DPD_LOG
    int ret = 0, cnt = 0;
    const u32 cfg_base = 0x10164;
    struct dbg_mem_read_cfm cfm;
    u32 wifisetting_cfg_addr;
    u32 patch_tbl_wifisetting_num;
    array2_tbl_t patch_tbl_wifisetting_8800dc_base = NULL;

    AIC_LOG_PRINTF("aicwf_patch_wifi_setting_8800dc\n");

    if (chip_sub_id == 0)
    {
        patch_tbl_wifisetting_num = sizeof(patch_tbl_wifisetting_8800dc_u01) / sizeof(u32) / 2;
        patch_tbl_wifisetting_8800dc_base = patch_tbl_wifisetting_8800dc_u01;
    }
    else if (chip_sub_id == 1 || chip_sub_id == 2)
    {
        patch_tbl_wifisetting_num = sizeof(patch_tbl_wifisetting_8800dc_u02) / sizeof(u32) / 2;
        patch_tbl_wifisetting_8800dc_base = patch_tbl_wifisetting_8800dc_u02;
    }
    else
    {
        AIC_LOG_PRINTF("unsupported id: %d", chip_sub_id);
        return;
    }

    if ((ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base, &cfm)))
    {
        AIC_LOG_PRINTF("setting base[0x%x] rd fail: %d\n", cfg_base, ret);
    }
    wifisetting_cfg_addr = cfm.memdata;

    for (cnt = 0; cnt < patch_tbl_wifisetting_num; cnt++)
    {
        if (chip_sub_id == 0)
        {
            if (patch_tbl_wifisetting_8800dc_base[cnt][0] == 0x010C)          //sdio_func_config, gpio_wakeup_en
                func_flag_rx = false;
        }
        else
        {
            if (patch_tbl_wifisetting_8800dc_base[cnt][0] == 0x0124)          //sdio_func_config, gpio_wakeup_en
                func_flag_rx = false;
        }
        if ((ret = rwnx_send_dbg_mem_write_req(rwnx_hw, wifisetting_cfg_addr + patch_tbl_wifisetting_8800dc_base[cnt][0], patch_tbl_wifisetting_8800dc_base[cnt][1])))
        {
            AIC_LOG_PRINTF("wifisetting %x write fail\n", patch_tbl_wifisetting_8800dc_base[cnt][0]);
        }
    }
#endif /* FW_PATCH_DPD_LOG */
}

void aicwf_patch_config_8800dc(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;
    int cnt = 0;
    if (rwnx_hw->mode != WIFI_MODE_RFTEST)
    {
        const u32 cfg_base = 0x10164;
        struct dbg_mem_read_cfm cfm;
        int i = 0;
        u32 wifisetting_cfg_addr;
        u32 ldpc_cfg_addr;
        u32 agc_cfg_addr;
        u32 txgain_cfg_addr;
        u32 jump_tbl_addr = 0;

        u32 patch_tbl_wifisetting_num;
        u32 ldpc_cfg_size = sizeof(ldpc_cfg_ram);
        u32 agc_cfg_size = sizeof(agc_cfg_ram);
        u32 txgain_cfg_size, *txgain_cfg_array;
        u32 jump_tbl_size = 0;
        u32 patch_tbl_func_num = 0 ;

        array2_tbl_t jump_tbl_base = NULL;
        array2_tbl_t patch_tbl_func_base = NULL;
        array2_tbl_t patch_tbl_wifisetting_8800dc_base = NULL;

        if (chip_sub_id == 0)
        {
            jump_tbl_base = jump_tbl;
            jump_tbl_size = sizeof(jump_tbl) / 2;
            patch_tbl_func_base = patch_tbl_func;
            patch_tbl_func_num = sizeof(patch_tbl_func) / sizeof(u32) / 2;
            patch_tbl_wifisetting_num = sizeof(patch_tbl_wifisetting_8800dc_u01) / sizeof(u32) / 2;
            patch_tbl_wifisetting_8800dc_base = patch_tbl_wifisetting_8800dc_u01;
        }
        else if (chip_sub_id == 1 || chip_sub_id == 2)
        {
            patch_tbl_wifisetting_num = sizeof(patch_tbl_wifisetting_8800dc_u02) / sizeof(u32) / 2;
            patch_tbl_wifisetting_8800dc_base = patch_tbl_wifisetting_8800dc_u02;
        }
        else
        {
            AIC_LOG_PRINTF("unsupported id: %d", chip_sub_id);
            return;
        }

        if ((ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base, &cfm)))
        {
            AIC_LOG_PRINTF("setting base[0x%x] rd fail: %d\n", cfg_base, ret);
        }
        wifisetting_cfg_addr = cfm.memdata;

        if (chip_sub_id == 0)
        {
            if ((ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 4, &cfm)))
            {
                AIC_LOG_PRINTF("setting base[0x%x] rd fail: %d\n", cfg_base + 4, ret);
            }
            jump_tbl_addr = cfm.memdata;
        }

        if ((ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 8, &cfm)))
        {
            AIC_LOG_PRINTF("setting base[0x%x] rd fail: %d\n", cfg_base + 8, ret);
        }
        ldpc_cfg_addr = cfm.memdata;

        if ((ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 0xc, &cfm)))
        {
            AIC_LOG_PRINTF("setting base[0x%x] rd fail: %d\n", cfg_base + 0xc, ret);
        }
        agc_cfg_addr = cfm.memdata;

        if ((ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 0x10, &cfm)))
        {
            AIC_LOG_PRINTF("setting base[0x%x] rd fail: %d\n", cfg_base + 0x10, ret);
        }
        txgain_cfg_addr = cfm.memdata;

        AIC_LOG_PRINTF("wifisetting_cfg_addr=%x, ldpc_cfg_addr=%x, agc_cfg_addr=%x, txgain_cfg_addr=%x,jump_tbl_addr=%x\n", wifisetting_cfg_addr, ldpc_cfg_addr, agc_cfg_addr, txgain_cfg_addr, jump_tbl_addr);

        for (cnt = 0; cnt < patch_tbl_wifisetting_num; cnt++)
        {
            if (chip_sub_id == 0)
            {
                if (patch_tbl_wifisetting_8800dc_base[cnt][0] == 0x010C)          //sdio_func_config, gpio_wakeup_en
                    func_flag_rx = false;
            }
            else
            {
                if (patch_tbl_wifisetting_8800dc_base[cnt][0] == 0x0124)          //sdio_func_config, gpio_wakeup_en
                    func_flag_rx = false;
            }
            if ((ret = rwnx_send_dbg_mem_write_req(rwnx_hw, wifisetting_cfg_addr + patch_tbl_wifisetting_8800dc_base[cnt][0], patch_tbl_wifisetting_8800dc_base[cnt][1])))
            {
                AIC_LOG_PRINTF("wifisetting %x write fail\n", patch_tbl_wifisetting_8800dc_base[cnt][0]);
            }
        }
#if 1
        if (adap_test)
        {
            AIC_LOG_PRINTF("%s for adaptivity test \r\n", __func__);
            int adap_patch_num = sizeof(adaptivity_patch_tbl_8800dc) / sizeof(u32) / 2;
            for (cnt = 0; cnt < adap_patch_num; cnt++)
            {
                if ((ret = rwnx_send_dbg_mem_write_req(rwnx_hw, wifisetting_cfg_addr + adaptivity_patch_tbl_8800dc[cnt][0], adaptivity_patch_tbl_8800dc[cnt][1])))
                {
                    AIC_LOG_PRINTF("%x write fail\n", wifisetting_cfg_addr + adaptivity_patch_tbl_8800dc[cnt][0]);
                }
            }
        }
#endif
        if (ldpc_cfg_size > 512)  // > 0.5KB data
        {
            for (i = 0; i < (ldpc_cfg_size - 512); i += 512)  //each time write 0.5KB
            {
                ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, ldpc_cfg_addr + i, 512, ldpc_cfg_ram + i / 4);
                if (ret)
                {
                    AIC_LOG_PRINTF("ldpc upload fail: %x, err:%d\r\n", ldpc_cfg_addr + i, ret);
                    break;
                }
            }
        }

        if (!ret && (i < ldpc_cfg_size))  // < 0.5KB data
        {
            ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, ldpc_cfg_addr + i, ldpc_cfg_size - i, ldpc_cfg_ram + i / 4);
            if (ret)
            {
                AIC_LOG_PRINTF("ldpc upload fail: %x, err:%d\r\n", ldpc_cfg_addr + i, ret);
            }
        }
        if (agc_cfg_size > 512)  // > 0.5KB data
        {
            for (i = 0; i < (agc_cfg_size - 512); i += 512)  //each time write 0.5KB
            {
                ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, agc_cfg_addr + i, 512, agc_cfg_ram + i / 4);
                if (ret)
                {
                    AIC_LOG_PRINTF("agc upload fail: %x, err:%d\r\n", agc_cfg_addr + i, ret);
                    break;
                }
            }
        }
        if (!ret && (i < agc_cfg_size))  // < 0.5KB data
        {
            ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, agc_cfg_addr + i, agc_cfg_size - i, agc_cfg_ram + i / 4);
            if (ret)
            {
                AIC_LOG_PRINTF("agc upload fail: %x, err:%d\r\n", agc_cfg_addr + i, ret);
            }
        }

#if !defined(CONFIG_FPGA_VERIFICATION)
        if ((IS_CHIP_ID_H()))
        {
            txgain_cfg_size = sizeof(txgain_map_h);
            txgain_cfg_array = (u32 *)txgain_map_h;
        }
        else
        {
            txgain_cfg_size = sizeof(txgain_map);
            txgain_cfg_array = (u32 *)txgain_map;
        }
        ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, txgain_cfg_addr, txgain_cfg_size, txgain_cfg_array);
        if (ret)
        {
            AIC_LOG_PRINTF("txgain upload fail: %x, err:%d\r\n", txgain_cfg_addr, ret);
        }
        if (chip_sub_id == 0)
        {
            for (cnt = 0; cnt < jump_tbl_size / 4; cnt += 1)
            {
                AIC_LOG_PRINTF("%x = %x\n", jump_tbl_base[cnt][0] * 4 + jump_tbl_addr, jump_tbl_base[cnt][1]);
                if ((ret = rwnx_send_dbg_mem_write_req(rwnx_hw, jump_tbl_base[cnt][0] * 4 + jump_tbl_addr, jump_tbl_base[cnt][1])))
                {
                    AIC_LOG_PRINTF("%x write fail\n", jump_tbl_addr + 8 * cnt);
                }
            }

            for (cnt = 0; cnt < patch_tbl_func_num; cnt++)
            {
                AIC_LOG_PRINTF("%x = %x\n", patch_tbl_func_base[cnt][0], patch_tbl_func_base[cnt][1]);
                if ((ret = rwnx_send_dbg_mem_write_req(rwnx_hw, patch_tbl_func_base[cnt][0], patch_tbl_func_base[cnt][1])))
                {
                    AIC_LOG_PRINTF("patch_tbl_func %x write fail\n", patch_tbl_func_base[cnt][0]);
                }
            }
        }
        else if (chip_sub_id == 1 || chip_sub_id == 2)
        {
            ret = aicwf_patch_table_load(rwnx_hw);
        }
        else
        {
            AIC_LOG_PRINTF("unsupported id: %d\n", chip_sub_id);
        }
#endif
    }
    else
    {
        if (chip_sub_id == 0)
        {
            u32 patch_tbl_rf_func_num = sizeof(patch_tbl_rf_func) / sizeof(u32) / 2;
            for (cnt = 0; cnt < patch_tbl_rf_func_num; cnt++)
            {
                if ((ret = rwnx_send_dbg_mem_write_req(rwnx_hw, patch_tbl_rf_func[cnt][0], patch_tbl_rf_func[cnt][1])))
                {
                    AIC_LOG_PRINTF("patch_tbl_rf_func %x write fail\n", patch_tbl_rf_func[cnt][0]);
                }
            }
        }
    }
}

int system_config_8800dc(struct rwnx_hw *rwnx_hw)
{
    int syscfg_num;
    array3_tbl_t p_syscfg_msk_tbl;
    int ret, cnt;
    const u32 mem_addr = 0x40500000;
    struct dbg_mem_read_cfm rd_mem_addr_cfm;
#ifdef CONFIG_OOB
    int oobcfg_num;
#endif

    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, mem_addr, &rd_mem_addr_cfm);
    if (ret)
    {
        AIC_LOG_PRINTF("%x rd fail: %d\n", mem_addr, ret);
        return ret;
    }
    chip_id = (u8)(rd_mem_addr_cfm.memdata >> 16);
    //AIC_LOG_PRINTF("%x=%x\n", rd_mem_addr_cfm.memaddr, rd_mem_addr_cfm.memdata);
    if (((rd_mem_addr_cfm.memdata >> 25) & 0x01UL) == 0x00UL)
    {
        chip_mcu_id = 1;
    }

    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, 0x00000020, &rd_mem_addr_cfm);
    if (ret)
    {
        AIC_LOG_PRINTF("[0x00000020] rd fail: %d\n", ret);
        return ret;
    }
    chip_sub_id = (u8)(rd_mem_addr_cfm.memdata);
    //AIC_LOG_PRINTF("%x=%x\n", rd_mem_addr_cfm.memaddr, rd_mem_addr_cfm.memdata);
    AIC_LOG_PRINTF("chip_id=%x, chip_sub_id=%x\n", chip_id, chip_sub_id);
    if (IS_CHIP_ID_H())
    {
        AIC_LOG_PRINTF("is 8800dcdw_h chip");
    }

    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, 0x40500010, &rd_mem_addr_cfm);
    AIC_LOG_PRINTF("[0x40500010]=%x\n", rd_mem_addr_cfm.memdata);
    if (ret)
    {
        AIC_LOG_PRINTF("[0x40500010] rd fail: %d\n", ret);
        return ret;
    }

    syscfg_num = sizeof(syscfg_tbl_8800dc) / sizeof(u32) / 2;

    for (cnt = 0; cnt < syscfg_num; cnt++)
    {
        ret = rwnx_send_dbg_mem_write_req(rwnx_hw, syscfg_tbl_8800dc[cnt][0], syscfg_tbl_8800dc[cnt][1]);
        if (ret)
        {
            AIC_LOG_PRINTF("%x write fail: %d\n", syscfg_tbl_8800dc[cnt][0], ret);
            return ret;
        }
    }

    if (chip_mcu_id == 0)
    {
        if (chip_sub_id == 0)
        {
            syscfg_num = sizeof(syscfg_tbl_8800dc_sdio_u01) / sizeof(u32) / 2;
            for (cnt = 0; cnt < syscfg_num; cnt++)
            {
                ret = rwnx_send_dbg_mem_write_req(rwnx_hw, syscfg_tbl_8800dc_sdio_u01[cnt][0], syscfg_tbl_8800dc_sdio_u01[cnt][1]);
                if (ret)
                {
                    AIC_LOG_PRINTF("%x write fail: %d\n", syscfg_tbl_8800dc_sdio_u01[cnt][0], ret);
                    return ret;
                }
            }
        }
        else if (chip_sub_id == 1)
        {
            syscfg_num = sizeof(syscfg_tbl_8800dc_sdio_u02) / sizeof(u32) / 2;
            for (cnt = 0; cnt < syscfg_num; cnt++)
            {
                ret = rwnx_send_dbg_mem_write_req(rwnx_hw, syscfg_tbl_8800dc_sdio_u02[cnt][0], syscfg_tbl_8800dc_sdio_u02[cnt][1]);
                if (ret)
                {
                    AIC_LOG_PRINTF("%x write fail: %d\n", syscfg_tbl_8800dc_sdio_u02[cnt][0], ret);
                    return ret;
                }
            }
        }
    }

#ifdef CONFIG_OOB
    if ((chip_sub_id == 1) || (chip_sub_id == 2))
    {
        oobcfg_num = sizeof(oobcfg_tbl_8800dc_sdio_u02) / sizeof(u32) / 2;
        for (cnt = 0; cnt < oobcfg_num; cnt++)
        {
            ret = rwnx_send_dbg_mem_write_req(rwnx_hw, oobcfg_tbl_8800dc_sdio_u02[cnt][0], oobcfg_tbl_8800dc_sdio_u02[cnt][1]);
            if (ret)
            {
                AIC_LOG_PRINTF("%x write fail: %d\n", oobcfg_tbl_8800dc_sdio_u02[cnt][0], ret);
                return ret;
            }
        }
    }
#endif

#if defined(CONFIG_VRF_DCDC_MODE)
    AIC_LOG_PRINTF("VRF dcdc mode\n");
#else
    AIC_LOG_PRINTF("VRF ldo mode\n");
#endif
    if (IS_CHIP_ID_H())
    {
        syscfg_num = sizeof(syscfg_tbl_masked_8800dc_h) / sizeof(u32) / 3;
        p_syscfg_msk_tbl = syscfg_tbl_masked_8800dc_h;
    }
    else
    {
        syscfg_num = sizeof(syscfg_tbl_masked_8800dc) / sizeof(u32) / 3;
        p_syscfg_msk_tbl = syscfg_tbl_masked_8800dc;
    }

    for (cnt = 0; cnt < syscfg_num; cnt++)
    {
        if (p_syscfg_msk_tbl[cnt][0] == 0x00000000)
        {
            break;
        }
        else if (p_syscfg_msk_tbl[cnt][0] == 0x70001000)
        {
            if (chip_mcu_id == 0)
            {
                p_syscfg_msk_tbl[cnt][1] |= ((0x1 << 8) | (0x1 << 15)); // mask
                p_syscfg_msk_tbl[cnt][2] |= ((0x1 << 8) | (0x1 << 15));
            }
        }

        ret = rwnx_send_dbg_mem_mask_write_req(rwnx_hw,
                                               p_syscfg_msk_tbl[cnt][0], p_syscfg_msk_tbl[cnt][1], p_syscfg_msk_tbl[cnt][2]);
        if (ret)
        {
            AIC_LOG_PRINTF("%x mask write fail: %d\n", p_syscfg_msk_tbl[cnt][0], ret);
            return ret;
        }
    }

    if (chip_sub_id == 0)
    {
        syscfg_num = sizeof(syscfg_tbl_masked_8800dc_u01) / sizeof(u32) / 3;
        for (cnt = 0; cnt < syscfg_num; cnt++)
        {
            ret = rwnx_send_dbg_mem_mask_write_req(rwnx_hw,
                                                   syscfg_tbl_masked_8800dc_u01[cnt][0], syscfg_tbl_masked_8800dc_u01[cnt][1], syscfg_tbl_masked_8800dc_u01[cnt][2]);
            if (ret)
            {
                AIC_LOG_PRINTF("%x mask write fail: %d\n", syscfg_tbl_masked_8800dc_u01[cnt][0], ret);
                return ret;
            }
        }
    }

    return 0;
}

