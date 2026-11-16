/**
 * @file ark7128_i2c.c
 * @brief ARK7128 I2C control layer: paged register access, configuration
 *        download and lock detection.
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @brief ARK7128 I2C control layer.
 *
 * Slave address 0x58; register access means "write 0xE8 = page first, then
 * read/write reg inside that page". Pins and bus are fixed (dpi-hdk-lb57e):
 * I2C1 / PA23(SCL) / PA22(SDA).
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "ark7128_i2c.h"
#include "ark7128_cfg.h"

#define DBG_TAG "ark7128.i2c"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct rt_i2c_bus_device *adap = RT_NULL;

/* Register access is a multi-transfer transaction (page write + register
 * access), and the input channel may be switched from the shell thread while
 * the preview thread reads the lock state. Serialize whole transactions so one
 * thread cannot change the page or the TPG source mid-way. The mutex is
 * recursive, so nested calls (e.g. config -> select_source -> write_page) are
 * safe. */
static struct rt_mutex s_i2c_lock;
static rt_bool_t s_i2c_lock_inited;

/** Input channel currently programmed into the decoder */
static rt_uint8_t s_input_source = ARK7128_INPUT_SEL;

static void ark7128_i2c_lock(void)
{
    if (s_i2c_lock_inited)
    {
        (void)rt_mutex_take(&s_i2c_lock, RT_WAITING_FOREVER);
    }
}

static void ark7128_i2c_unlock(void)
{
    if (s_i2c_lock_inited)
    {
        (void)rt_mutex_release(&s_i2c_lock);
    }
}

int ark7128_i2c_write(uint8_t reg, uint8_t val)
{
    rt_uint8_t buf = val;
    rt_uint8_t retries = 0;
    rt_size_t ret;

    if (adap == RT_NULL)
    {
        return -RT_EIO;
    }

    while (retries < 5)
    {
        ret = rt_i2c_mem_write(adap, ARK7128_SLAVE_ADDR, reg, 8, &buf, 1);
        if (ret == 1)
        {
            break;
        }
        retries++;
    }

    if (retries >= 5)
    {
        LOG_E("%s timeout", __func__);
        return -RT_EIO;
    }

    return RT_EOK;
}

int ark7128_i2c_read(uint8_t reg, uint8_t *val)
{
    rt_uint8_t buf = 0;
    rt_uint8_t retries = 0;
    rt_size_t ret;

    if ((adap == RT_NULL) || (val == RT_NULL))
    {
        return -RT_EIO;
    }

    while (retries < 5)
    {
        ret = rt_i2c_mem_read(adap, ARK7128_SLAVE_ADDR, reg, 8, &buf, 1);
        if (ret == 1)
        {
            break;
        }
        retries++;
    }

    if (retries >= 5)
    {
        LOG_E("%s timeout", __func__);
        return -RT_EIO;
    }

    *val = buf;
    return RT_EOK;
}

int ark7128_i2c_write_pairs(const uint8_t *tbl, rt_uint32_t pairs)
{
    rt_uint32_t i;

    if (tbl == RT_NULL)
    {
        return -RT_EINVAL;
    }

    for (i = 0; i < pairs; i++)
    {
        if (ark7128_i2c_write(tbl[i * 2], tbl[i * 2 + 1]) != RT_EOK)
        {
            return -RT_EIO;
        }
    }

    return RT_EOK;
}

int ark7128_i2c_write_page(uint8_t page, uint8_t reg, uint8_t val)
{
    int ret;

    ark7128_i2c_lock();

    ret = ark7128_i2c_write(0xE8, page);
    if (ret == RT_EOK)
    {
        ret = ark7128_i2c_write(reg, val);
    }

    ark7128_i2c_unlock();

    return ret;
}

int ark7128_i2c_init(void)
{
    /* One-time creation; the lifetime matches the driver, so it is never
     * destroyed */
    if (!s_i2c_lock_inited)
    {
        (void)rt_mutex_init(&s_i2c_lock, "ark7128_i2c", RT_IPC_FLAG_PRIO);
        s_i2c_lock_inited = RT_TRUE;
    }

    if (adap != RT_NULL)
    {
        return RT_EOK;
    }

    /* The I2C pins are configured by the board pinmux (bsp_pinmux.c) */
    adap = rt_i2c_bus_device_find(ARK7128_I2C_BUS_NAME);
    if (adap == RT_NULL)
    {
        LOG_E("open %s failed", ARK7128_I2C_BUS_NAME);
        return -RT_EIO;
    }

    LOG_I("i2c bus %s ready", ARK7128_I2C_BUS_NAME);
    return RT_EOK;
}

void ark7128_i2c_deinit(void)
{
    adap = RT_NULL;
}

uint16_t ark7128_i2c_detect_chip(void)
{
    uint8_t hi = 0;
    uint8_t lo = 0;
    uint16_t id = 0;

    ark7128_i2c_lock();
    if ((ark7128_i2c_read(0xF7, &hi) == RT_EOK) &&
            (ark7128_i2c_read(0xF8, &lo) == RT_EOK))
    {
        id = (uint16_t)((hi << 8) | lo);
    }
    ark7128_i2c_unlock();

    return id;
}

int ark7128_i2c_signal_raw(uint8_t *raw)
{
    uint8_t v = 0;
    int rc;

    if (raw == RT_NULL)
    {
        return -RT_EINVAL;
    }

    ark7128_i2c_lock();

    /* Selecting the page is part of the transaction: if it fails, register
     * 0x26 belongs to an unknown page and must not be read */
    if (ark7128_i2c_write(0xE8, 0x00) != RT_EOK)
    {
        ark7128_i2c_unlock();
        return -RT_EIO;
    }

    rc = ark7128_i2c_read(0x26, &v);
    ark7128_i2c_unlock();

    *raw = v;

    return (rc == RT_EOK) ? RT_EOK : -RT_EIO;
}

uint8_t ark7128_i2c_signal_check(void)
{
    uint8_t raw = 0;

    if (ark7128_i2c_signal_raw(&raw) != RT_EOK)
    {
        /*
         * An I2C failure is reported as "not locked". The recovery policy in
         * bt656_get() then treats it like no signal and keeps waiting instead
         * of resetting the DCMI block, which would not fix a dead bus either.
         */
        return 0;
    }

    return ((raw & 0x06) == 0x06) ? 1U : 0U;
}

int ark7128_i2c_wait_lock(rt_int32_t timeout_ms)
{
    rt_int32_t waited = 0;
    rt_int32_t stable = 0;

    if (timeout_ms <= 0)
    {
        timeout_ms = 800;
    }

    while (waited < timeout_ms)
    {
        if (ark7128_i2c_signal_check())
        {
            stable++;
            if (stable >= 5)
            {
                return RT_EOK;
            }
        }
        else
        {
            stable = 0;
        }
        rt_thread_mdelay(20);
        waited += 20;
    }

    return -RT_ETIMEOUT;
}

int ark7128_i2c_select_source(uint8_t source)
{
    int ret;

    if (source > 3U)
    {
        return -RT_EINVAL;
    }

    ark7128_i2c_lock();

    /* Vendor encoding: 0x10 * ch + ch (0x00/0x11/0x22/0x33) */
    ret = ark7128_i2c_write_page(0x10, 0x0B, (uint8_t)(0x10U * source + source));
    if (ret == RT_EOK)
    {
        s_input_source = source;
    }

    ark7128_i2c_unlock();

    return ret;
}

void ark7128_i2c_decoder_reset(void)
{
    uint8_t val = 0;

    ark7128_i2c_lock();

    (void)ark7128_i2c_write(0xE8, 0x00);
    (void)ark7128_i2c_read(0xA1, &val);
    (void)ark7128_i2c_write(0xA1, val & 0xFD);
    rt_thread_mdelay(50);
    (void)ark7128_i2c_write(0xA1, val | 0x02);
    rt_thread_mdelay(100);

    ark7128_i2c_unlock();
}

/** @brief Download the decoder configuration: main register table +
 *  format-specific extra registers + input channel.
 * Table contents all come from ark7128_cfg.c; the write order is verified on
 * hardware.
 *
 * @note The caller must hold the I2C transaction lock.
 */
static int ark7128_i2c_config_locked(void)
{
    const struct ark7128_format *fmt = ark7128_get_format();
    rt_uint8_t cur_page = 0xFFU;
    rt_uint32_t i;

    if (fmt == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if (ark7128_i2c_detect_chip() != ARK7128_CHIP_ID)
    {
        LOG_W("chip id mismatch, register table not loaded");
        return -RT_EIO;
    }

    LOG_I("config %s (%ux%u, stride %u)", fmt->info.name,
          (unsigned int)fmt->info.width, (unsigned int)fmt->info.height,
          (unsigned int)fmt->info.stride);

    if (ark7128_i2c_write_pairs((const uint8_t *)fmt->reg_tbl, fmt->reg_pairs) != RT_EOK)
    {
        return -RT_EIO;
    }

    for (i = 0; i < fmt->extra_cnt; i++)
    {
        const struct ark7128_reg_op *op = &fmt->extra[i];

        if (op->page != cur_page)
        {
            if (ark7128_i2c_write(0xE8, op->page) != RT_EOK)
            {
                return -RT_EIO;
            }
            cur_page = op->page;
        }

        if (op->op == ARK7128_OP_RMW_E4)
        {
            rt_uint8_t v = 0;

            if (ark7128_i2c_read(op->reg, &v) != RT_EOK)
            {
                return -RT_EIO;
            }
            if (ark7128_i2c_write(op->reg, (v & 0xCF) | 0x20) != RT_EOK)
            {
                return -RT_EIO;
            }
        }
        else if (ark7128_i2c_write(op->reg, op->val) != RT_EOK)
        {
            return -RT_EIO;
        }
    }

    /* Re-apply the channel the user selected, not the compile-time default a
     * restart would otherwise overrule */
    return ark7128_i2c_select_source(s_input_source);
}

/** @brief Download the register table of the current format (see
 *  ark7128_i2c_config_locked()); runs as one I2C transaction.
 */
int ark7128_i2c_config(void)
{
    int ret;

    ark7128_i2c_lock();
    ret = ark7128_i2c_config_locked();
    ark7128_i2c_unlock();

    return ret;
}

ark7128_mode_t ark7128_i2c_get_mode(void)
{
    const struct ark7128_format *fmt = ark7128_get_format();

    return (fmt != RT_NULL) ? fmt->mode : ARK7128_MODE_UNKNOWN;
}
