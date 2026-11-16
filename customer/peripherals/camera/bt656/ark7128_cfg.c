/**
 * @file ark7128_cfg.c
 * @brief ARK7128 register tables, geometry and DCMI sync codes per video format.
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @brief ARK7128 configuration table per video format.
 *
 * To add an input format, add one ark7128_fmt_xxx here (register table +
 * geometry + sync codes) and one option to the Kconfig choice; no other file
 * needs to change.
 *
 * Note: the AHD 720P and CVBS NTSC tables are verified on hardware; check the
 * datasheet before changing their contents.
 */

#include <rtthread.h>
#include "ark7128_cfg.h"

/** @brief ITU output pad drive strength (top page 0x10 registers 0x24~0x27,
 *         2 bits per pad).
 *
 * Field layout (bit7:6 / bit5:4 / bit3:2 / bit1:0):
 *   0x24: d1     d0     vsync  hsync
 *   0x25: d5     d4     d3     d2
 *   0x26: --     irq    d7     d6      (-- are reserved bits, must be written as 0)
 *   0x27: --     clko   sda    scl     (-- are reserved bits, must be written as 0)
 *
 * Measured: raising the ITU side (clko + d0~d7) two steps from 00 (weakest) to
 * 10 clearly improved the edges on long cables and adapter boards;
 * vsync/hsync/sda/scl/irq stay at 00.
 * Encoding:
 *   0x24 = d1,d0 10 10 + vsync,hsync 00 00   -> 0xA0
 *   0x25 = d5..d2 all 10                     -> 0xAA
 *   0x26 = [7:6] 00 + irq 00 + d7,d6 10 10   -> 0x0A
 *   0x27 = [7:6] 00 + clko 10 + sda,scl 00   -> 0x20
 *
 * If overshoot or ringing gets worse on another board (00 = weakest,
 * 11 = strongest), set these four macros back to 0x00.
 */
#define ARK7128_PAD_DRV_REG24       0xA0U
#define ARK7128_PAD_DRV_REG25       0xAAU
#define ARK7128_PAD_DRV_REG26       0x0AU
#define ARK7128_PAD_DRV_REG27       0x20U

/** 25fps and 30fps AHD 720P share the same decoder table; only 0xD1 differs */
static const rt_uint8_t ark7128_tbl_ahd720p[][2] =
{
    /** stride * lines = 2560 * 720 */
    0xE8, 0x10,
    0x06, 0x24,
    0x0A, 0x0C,
    0x0C, 0x37,
    0x0F, 0x60,
    0x12, 0x80,
    0x20, 0x4D,
    0x38, 0x80,
    0x39, 0x10,
    0x54, 0x98,

    0x20, 0x55,
    0x21, 0x55,
    0x22, 0x55,

    0xEE, 0xFF,
    0xEF, 0xFF,

    0x24, ARK7128_PAD_DRV_REG24,
    0x25, ARK7128_PAD_DRV_REG25,
    0x26, ARK7128_PAD_DRV_REG26,
    0x27, ARK7128_PAD_DRV_REG27,
};

/**
 * CVBS NTSC main table.
 *
 * The write order of registers such as 0x21/0x22/0x19/0x60/0xA1 is verified on
 * hardware (writing 0x19 re-locks the PLL); do not reorder them casually.
 */
static const rt_uint8_t ark7128_tbl_cvbs_ntsc[][2] =
{
    /** stride * lines = 1440 * 240 */
    0xE8, 0x10,
    0x06, 0x24,
    0x0A, 0x0C,
    0x0C, 0x37,
    0x0F, 0x60,
    0x12, 0x80,
    0x20, 0x4D,
    0x38, 0x00,
    0x39, 0x10,
    0x54, 0x98,

    0x20, 0x55,
    0x21, 0x55,
    0x22, 0x55,

    0xEE, 0xFF,
    0xEF, 0xFF,

    0x24, ARK7128_PAD_DRV_REG24,
    0x25, ARK7128_PAD_DRV_REG25,
    0x26, ARK7128_PAD_DRV_REG26,
    0x27, ARK7128_PAD_DRV_REG27,

    0xE8, 0x00,
    0x21, 0xc5,
    0x22, 0x04,
    0x19, 0x02,
    0x60, 0x04,

    0xE8, 0x00,
    0x60, 0x03,

    0xA1, 0xE6,
    0x17, 0x86, /**<< hactive_start */
    0x18, 0x20,
    0x19, 0xD2,
    0x1A, 0x16, /**<< vactive_start */
    0x1B, 0x10,
    0x1C, 0xE0,
};

/**
 * CVBS PAL main table (720x288).
 *
 * The differences from NTSC are mainly the active-area start in
 * 0x17/0x1A/0x1B/0x1C and clock-related registers such as 0x39; the write order
 * is likewise verified on hardware.
 *
 * Note: PAL timing has not been fully validated on the current board/cable (see
 * docs), but the format table, geometry and sync codes are kept complete, so it
 * can be selected and retested directly after a board or source change.
 */
static const rt_uint8_t ark7128_tbl_cvbs_pal[][2] =
{
    /** stride * lines = 1440 * 288 */
    0xE8, 0x10,
    0x06, 0x24,
    0x0A, 0x0C,
    0x0C, 0x37,
    0x0F, 0x60,
    0x12, 0x80,
    0x20, 0x4D,
    0x38, 0x00,
    0x39, 0x10,
    0x54, 0x98,

    0x20, 0x55,
    0x21, 0x55,
    0x22, 0x55,

    0xEE, 0xFF,
    0xEF, 0xFF,

    /** PAL keeps the vendor's weakest drive setting (unlike the NTSC table measured on this board; retune after a board change) */
    0x24, 0x00,
    0x25, 0x00,
    0x26, 0x00,
    0x27, 0x00,

    0xE8, 0x00,
    0x21, 0xc5,
    0x22, 0x04,
    0x19, 0x02,
    0x60, 0x04,

    0xE8, 0x00,
    0x60, 0x03,

    0xA1, 0xE6,
    0x17, 0x8A, /**<< hactive_start */
    0x18, 0x20,
    0x19, 0xD2,
    0x1A, 0x20, /**< vactive_start */
    0x1B, 0x20,
    0x1C, 0x40,
};

/**
 * Extra registers per format (in page order; 0xE8 is written only when the page
 * differs from the previous entry).
 * 0xE4 is a read-modify-write: (v & 0xCF) | 0x20, writing that format's extra
 * parameter.
 */
static const struct ark7128_reg_op ark7128_extra_ntsc[] =
{
    { 0x10, 0x02, 0x4B, ARK7128_OP_WRITE },
    /**
     * ITU output clock (vp page 0x08):
     *   bit7   = invert the output clock (0 = not inverted, 1 = inverted)
     *   bit6:0 = output clock phase delay, 128 steps
     * Currently 0x00 (not inverted, phase 0).
     *
     * Historic sweep (2026-11-06, skip the first frame of each phase, then
     * capture 3 consecutive frames): 0xC0 (inverted + phase 64) gave 1239~1279
     * green pixels and 0 misaligned lines, identical across three frames; 0x40
     * (not inverted + phase 64) gave 1943~2993 and 7~12 misaligned lines in the
     * same run. 0xC0 also ran a 99 s preview: 29.8 fps, fail=0.
     * If the line-start phase becomes unstable on another board, retry here.
     */
    { 0x08, 0xF5, 0x00, ARK7128_OP_WRITE },
    { 0x08, 0xD4, 0x77, ARK7128_OP_WRITE },
    { 0x00, 0x19, 0xD2, ARK7128_OP_WRITE },
    { 0x00, 0xE4, 0x00, ARK7128_OP_RMW_E4 },
    { 0x00, 0xD1, 0x82, ARK7128_OP_WRITE },
};

static const struct ark7128_reg_op ark7128_extra_ahd25[] =
{
    { 0x08, 0xD4, 0x77, ARK7128_OP_WRITE },
    { 0x00, 0x19, 0x02, ARK7128_OP_WRITE },
    { 0x00, 0xE4, 0x00, ARK7128_OP_RMW_E4 },
    { 0x00, 0xD1, 0x88, ARK7128_OP_WRITE },
};

static const struct ark7128_reg_op ark7128_extra_ahd30[] =
{
    { 0x08, 0xD4, 0x77, ARK7128_OP_WRITE },
    { 0x00, 0x19, 0x02, ARK7128_OP_WRITE },
    { 0x00, 0xE4, 0x00, ARK7128_OP_RMW_E4 },
    { 0x00, 0xD1, 0x84, ARK7128_OP_WRITE },
};

static const struct ark7128_reg_op ark7128_extra_pal[] =
{
    { 0x10, 0x02, 0x4B, ARK7128_OP_WRITE },
    { 0x08, 0xF5, 0x00, ARK7128_OP_WRITE },
    { 0x08, 0xD4, 0x77, ARK7128_OP_WRITE },
    { 0x00, 0x19, 0xD2, ARK7128_OP_WRITE },
    /** CVBS signal: adjust the field active position of the even field */
    { 0x00, 0x2B, 0xBC, ARK7128_OP_WRITE },
    { 0x00, 0xE4, 0x00, ARK7128_OP_RMW_E4 },
    { 0x00, 0xD1, 0x81, ARK7128_OP_WRITE },
};

/*
 * BT.656 SAV/EAV sync codes are defined by the protocol and are independent of
 * the decoder chip, so they live in the generic layer bt656.c (one set for
 * progressive, one for interlaced). This file only describes the geometry and
 * register tables of each format.
 */

static const struct ark7128_format ark7128_fmt_ahd25 =
{
    .info =
    {
        .name        = "AHD 720P25",
        .width       = 1280,
        .height      = 720,
        .stride      = 2560,
        .interlace   = 0,
        .frame_bytes = 1280U * 720U * 2U,
    },
    .mode       = ARK7128_MODE_AHD_720P25,
    .reg_tbl    = ark7128_tbl_ahd720p,
    .reg_pairs  = sizeof(ark7128_tbl_ahd720p) / 2U,
    .extra      = ark7128_extra_ahd25,
    .extra_cnt  = sizeof(ark7128_extra_ahd25) / sizeof(ark7128_extra_ahd25[0]),
};

static const struct ark7128_format ark7128_fmt_ahd30 =
{
    .info =
    {
        .name        = "AHD 720P30",
        .width       = 1280,
        .height      = 720,
        .stride      = 2560,
        .interlace   = 0,
        .frame_bytes = 1280U * 720U * 2U,
    },
    .mode       = ARK7128_MODE_AHD_720P30,
    .reg_tbl    = ark7128_tbl_ahd720p,
    .reg_pairs  = sizeof(ark7128_tbl_ahd720p) / 2U,
    .extra      = ark7128_extra_ahd30,
    .extra_cnt  = sizeof(ark7128_extra_ahd30) / sizeof(ark7128_extra_ahd30[0]),
};

static const struct ark7128_format ark7128_fmt_ntsc =
{
    .info =
    {
        .name        = "CVBS NTSC",
        .width       = 720,
        /** One field = 240 active lines (NTSC standard) */
        .height      = 240,
        /** SAV/EAV are consumed by the FSM, so one line in the buffer is 720x2 = 1440 pure pixels */
        .stride      = 1440,
        .interlace   = 1,
        /** Single field: 240 lines x 1440 = 345,600 bytes */
        .frame_bytes = 1440U * 240U,
    },
    .mode       = ARK7128_MODE_CVBS_NTSC,
    .reg_tbl    = ark7128_tbl_cvbs_ntsc,
    .reg_pairs  = sizeof(ark7128_tbl_cvbs_ntsc) / 2U,
    .extra      = ark7128_extra_ntsc,
    .extra_cnt  = sizeof(ark7128_extra_ntsc) / sizeof(ark7128_extra_ntsc[0]),
};

/**
 * CVBS PAL: geometry 720x288 single field, same sync codes as NTSC
 * (FSC/LSC/LEC/FEC identical).
 *
 * The complete format table is kept; PAL timing has not been fully validated on
 * the current board/cable. After selecting it, use `camera snapshot` to confirm
 * that all 288 lines (414,720 B) are received.
 */
static const struct ark7128_format ark7128_fmt_pal =
{
    .info =
    {
        .name        = "CVBS PAL",
        .width       = 720,
        .height      = 288,
        .stride      = 1440,
        .interlace   = 1,
        /** Single field: 288 lines x 1440 = 414,720 bytes */
        .frame_bytes = 1440U * 288U,
    },
    .mode       = ARK7128_MODE_CVBS_PAL,
    .reg_tbl    = ark7128_tbl_cvbs_pal,
    .reg_pairs  = sizeof(ark7128_tbl_cvbs_pal) / 2U,
    .extra      = ark7128_extra_pal,
    .extra_cnt  = sizeof(ark7128_extra_pal) / sizeof(ark7128_extra_pal[0]),
};

/**
 * The only place that interprets the video format selection (Kconfig:
 * ARK7128 -> Video input format). Switching formats only needs menuconfig; the
 * driver internals need no #if.
 */
const struct ark7128_format *ark7128_get_format(void)
{
    /** Note: the rtconfig.h generated by the SDK has no CONFIG_ prefix */
#if defined(ARK7128_VIDEO_AHD_720P_30)
    return &ark7128_fmt_ahd30;
#elif defined(ARK7128_VIDEO_AHD_720P_25)
    return &ark7128_fmt_ahd25;
#elif defined(ARK7128_VIDEO_CVBS_PAL)
    return &ark7128_fmt_pal;
#else
    return &ark7128_fmt_ntsc;
#endif
}

static const struct ark7128_format *const ark7128_format_list[] =
{
    &ark7128_fmt_ntsc,
    &ark7128_fmt_pal,
    &ark7128_fmt_ahd25,
    &ark7128_fmt_ahd30,
    RT_NULL,
};

const struct ark7128_format *ark7128_format_by_mode(ark7128_mode_t mode)
{
    rt_uint32_t i;

    for (i = 0; ark7128_format_list[i] != RT_NULL; i++)
    {
        if (ark7128_format_list[i]->mode == mode)
        {
            return ark7128_format_list[i];
        }
    }

    return RT_NULL;
}
