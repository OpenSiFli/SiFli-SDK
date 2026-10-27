/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "aw9523.h"
#include "rtconfig.h"

#ifdef BSP_USING_RTTHREAD

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "aw9523"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct rt_i2c_bus_device *aw_i2c_bus = RT_NULL;
static struct rt_mutex aw9523_mutex;
static volatile uint8_t aw9523_ready = 0;

static int aw9523_write_reg(uint8_t reg, uint8_t val)
{
    struct rt_i2c_msg msg;
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = val;

    msg.addr  = AW9523_I2C_ADDR;
    msg.flags = RT_I2C_WR;
    msg.buf   = buf;
    msg.len   = 2;

    if (rt_i2c_transfer(aw_i2c_bus, &msg, 1) != 1)
    {
        LOG_E("write reg 0x%02x fail", reg);
        return -1;
    }
    return 0;
}

static int aw9523_read_reg(uint8_t reg, uint8_t *val)
{
    struct rt_i2c_msg msgs[2];
    uint8_t reg_buf = reg;

    msgs[0].addr  = AW9523_I2C_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg_buf;
    msgs[0].len   = 1;

    msgs[1].addr  = AW9523_I2C_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = val;
    msgs[1].len   = 1;

    if (rt_i2c_transfer(aw_i2c_bus, msgs, 2) != 2)
    {
        LOG_E("read reg 0x%02x fail", reg);
        return -1;
    }
    return 0;
}

int aw9523_init(void)
{
    static uint8_t mutex_inited = 0;
    uint8_t id = 0;

    /* The mutex is a static object, it must be initialized only once:
       re-initializing a registered kernel object trips rt_object_init(). */
    if (!mutex_inited)
    {
        rt_mutex_init(&aw9523_mutex, "aw9523", RT_IPC_FLAG_PRIO);
        mutex_inited = 1;
    }

    rt_mutex_take(&aw9523_mutex, RT_WAITING_FOREVER);

    /* Already configured, nothing to do. Makes repeated calls harmless. */
    if (aw9523_ready)
    {
        rt_mutex_release(&aw9523_mutex);
        return 0;
    }

    aw_i2c_bus = rt_i2c_bus_device_find(AW9523_I2C_NAME);
    if (aw_i2c_bus == RT_NULL)
    {
        LOG_E("can not find i2c bus %s", AW9523_I2C_NAME);
        rt_mutex_release(&aw9523_mutex);
        return -1;
    }

    if (rt_device_open(&aw_i2c_bus->parent, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open i2c bus %s fail", AW9523_I2C_NAME);
        aw_i2c_bus = RT_NULL;
        rt_mutex_release(&aw9523_mutex);
        return -1;
    }

    /* Wait for the power rail to settle before the first transfer. */
    rt_thread_mdelay(5);

    /* The ID register is fixed to 0x23. */
    if (aw9523_read_reg(AW9523_REG_ID, &id) != 0 || id != 0x23)
    {
        LOG_E("id check fail, not aw9523");
        rt_device_close(&aw_i2c_bus->parent);
        aw_i2c_bus = RT_NULL;
        rt_mutex_release(&aw9523_mutex);
        return -1;
    }

    /* GCR: D4=1 push-pull outputs on P0, D[1:0]=11 LED current 1/4 range. */
    aw9523_write_reg(AW9523_REG_GCR, 0x13);

    /* Switch all pins to GPIO mode (1=GPIO, 0=LED). */
    aw9523_write_reg(AW9523_REG_LEDMODE0, 0xFF);
    aw9523_write_reg(AW9523_REG_LEDMODE1, 0xFF);

    /* Initial outputs: P0_4~P0_7 low, P1_0~P1_3 high (LED off),
       P1_4~P1_7 low (reset held). */
    aw9523_write_reg(AW9523_REG_OUTPUT0, 0x00);
    aw9523_write_reg(AW9523_REG_OUTPUT1, 0x0F);

    /* Direction: P0_0/P0_1 inputs (USB/TF detect), the rest outputs. */
    aw9523_write_reg(AW9523_REG_CONFIG0, 0x03);
    aw9523_write_reg(AW9523_REG_CONFIG1, 0x00);

    aw9523_ready = 1;
    rt_mutex_release(&aw9523_mutex);
    return 0;
}

int aw9523_set_pin(uint8_t pin, uint8_t val)
{
    uint8_t port = (pin >> 4) & 0x01;
    uint8_t bit  = pin & 0x07;
    uint8_t reg  = AW9523_REG_OUTPUT0 + port;
    uint8_t cur  = 0;
    int ret;

    if (!aw9523_ready)
        return -1;

    rt_mutex_take(&aw9523_mutex, RT_WAITING_FOREVER);

    ret = aw9523_read_reg(reg, &cur);
    if (ret == 0)
    {
        if (val)
            cur |= (1 << bit);
        else
            cur &= ~(1 << bit);
        ret = aw9523_write_reg(reg, cur);
    }

    rt_mutex_release(&aw9523_mutex);
    return ret;
}

uint8_t aw9523_get_pin(uint8_t pin)
{
    uint8_t port = (pin >> 4) & 0x01;
    uint8_t bit  = pin & 0x07;
    uint8_t reg  = AW9523_REG_INPUT0 + port;
    uint8_t val  = 0;

    if (!aw9523_ready)
        return 0;

    rt_mutex_take(&aw9523_mutex, RT_WAITING_FOREVER);
    if (aw9523_read_reg(reg, &val) != 0)
        val = 0;
    rt_mutex_release(&aw9523_mutex);

    return (val >> bit) & 0x01;
}

#else /* !BSP_USING_RTTHREAD: bare-metal environment without the I2C framework */

int aw9523_init(void)
{
    return -1;
}

int aw9523_set_pin(uint8_t pin, uint8_t val)
{
    (void)pin;
    (void)val;
    return -1;
}

uint8_t aw9523_get_pin(uint8_t pin)
{
    (void)pin;
    return 0;
}

#endif /* BSP_USING_RTTHREAD */
