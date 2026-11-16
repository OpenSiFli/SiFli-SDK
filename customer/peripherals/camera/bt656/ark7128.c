/**
 * @file ark7128.c
 * @brief ARK7128 chip adaptation layer: I2C, register table, pins, and
 *        registration with the generic BT.656 layer.
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @details
 * This file only handles what is specific to this chip: I2C address and paging,
 * register tables, analog input channel, lock detection and DCMI pins. Capture,
 * DMA and buffer lending are done by bt656.c.
 *
 * Board: dpi-hdk-lb57e (SF32LB57x HCPU + DPI panel).
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "mem_section.h"
#include "ark7128.h"
#include "ark7128_cfg.h"
#include "ark7128_i2c.h"
#include "bt656_decoder.h"

#define DBG_TAG "ark7128"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/*
 * Buffer pool for continuous capture. Interlaced (CVBS) uses 3 blocks in
 * PSRAM1; progressive (AHD) needs 1.84 MB per frame, which does not fit twice
 * in PSRAM1, so it stays in PSRAM2 (the l2_non_ret_bss_frambuf* sections of the
 * linker script).
 */
#if defined(ARK7128_VIDEO_AHD_720P_25) || defined(ARK7128_VIDEO_AHD_720P_30)
    #define ARK7128_STREAM_BUFS         2U
    #define ARK7128_STREAM_BUF_MAX      (1280U * 720U * 2U)
    #define ARK7128_STREAM_IN_PSRAM2    1
#else
    #define ARK7128_STREAM_BUFS         3U
    #define ARK7128_STREAM_BUF_MAX      (1440U * 288U)
    #define ARK7128_STREAM_IN_PSRAM2    0
#endif

#if ARK7128_STREAM_IN_PSRAM2
#if defined(__CC_ARM) || defined(__CLANG_ARM)
L2_NON_RET_BSS_SECT_BEGIN(frambuf)
ALIGN(64) static rt_uint8_t
ark7128_stream_buf[ARK7128_STREAM_BUFS][ARK7128_STREAM_BUF_MAX];
L2_NON_RET_BSS_SECT_END
#else
L2_NON_RET_BSS_SECT(frambuf,
                    ALIGN(64) static rt_uint8_t
                    ark7128_stream_buf[ARK7128_STREAM_BUFS][ARK7128_STREAM_BUF_MAX]);
#endif
#else
#if defined(__CC_ARM) || defined(__CLANG_ARM)
L2_NON_RET_BSS_SECT_BEGIN(ark7128_stream)
ALIGN(64) static rt_uint8_t
ark7128_stream_buf[ARK7128_STREAM_BUFS][ARK7128_STREAM_BUF_MAX];
L2_NON_RET_BSS_SECT_END
#else
L2_NON_RET_BSS_SECT(ark7128_stream,
                    ALIGN(64) static rt_uint8_t
                    ark7128_stream_buf[ARK7128_STREAM_BUFS][ARK7128_STREAM_BUF_MAX]);
#endif
#endif

/* The DCMI and I2C pins are configured by the board pinmux (bsp_pinmux.c). */

static int ark7128_init(void)
{
    int ret;

    ret = ark7128_i2c_init();
    if (ret != RT_EOK)
    {
        return ret;
    }

    if (ark7128_i2c_config() != RT_EOK)
    {
        LOG_E("decoder not configured (check chip id / wiring)");
        return -RT_EIO;
    }

    return RT_EOK;
}

static int ark7128_deinit(void)
{
    ark7128_i2c_deinit();
    return RT_EOK;
}

static int ark7128_configure(const struct bt656_format *fmt)
{
    (void)fmt;

    /* Re-lock the PLL on every new session so the decoder output does not stay
     * at the old horizontal offset */
    ark7128_i2c_decoder_reset();
    rt_thread_mdelay(20);

    if (ark7128_i2c_config() != RT_EOK)
    {
        return -RT_EIO;
    }

    return RT_EOK;
}

static int ark7128_wait_ready(rt_int32_t timeout_ms)
{
    return ark7128_i2c_wait_lock(timeout_ms);
}

static int ark7128_set_source(rt_uint8_t source)
{
    return ark7128_i2c_select_source(source);
}

static rt_uint16_t ark7128_detect(void)
{
    return ark7128_i2c_detect_chip();
}

static rt_uint8_t ark7128_signal_locked(void)
{
    return ark7128_i2c_signal_check();
}

static const struct bt656_format *ark7128_format(void)
{
    const struct ark7128_format *fmt = ark7128_get_format();

    return (fmt != RT_NULL) ? &fmt->info : RT_NULL;
}

static const struct bt656_decoder_ops ark7128_ops =
{
    .name          = "ark7128",
    .init          = ark7128_init,
    .deinit        = ark7128_deinit,
    .configure     = ark7128_configure,
    .wait_ready    = ark7128_wait_ready,
    .set_source    = ark7128_set_source,
    .detect        = ark7128_detect,
    .signal_locked = ark7128_signal_locked,
    .format        = ark7128_format,
    .buffer        = ark7128_stream_buf,
    .buffer_count  = ARK7128_STREAM_BUFS,
    .buffer_stride = ARK7128_STREAM_BUF_MAX,
};

int ark7128_register(void)
{
    return bt656_register_decoder(&ark7128_ops);
}
INIT_BOARD_EXPORT(ark7128_register);
