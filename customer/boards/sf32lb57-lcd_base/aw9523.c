/*
 * AW9523B I2C GPIO 扩展器驱动
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "aw9523.h"
#include "bsp_board.h"   /* 引入 rtconfig.h, 定义 BSP_USING_RTTHREAD */

#ifdef BSP_USING_RTTHREAD

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "aw9523"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct rt_i2c_bus_device *aw_i2c_bus = RT_NULL;

static void aw9523_write_reg(uint8_t reg, uint8_t val)
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
        LOG_E("write reg 0x%02x fail", reg);
}

static uint8_t aw9523_read_reg(uint8_t reg)
{
    struct rt_i2c_msg msgs[2];
    uint8_t reg_buf = reg;
    uint8_t val = 0;

    msgs[0].addr  = AW9523_I2C_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg_buf;
    msgs[0].len   = 1;

    msgs[1].addr  = AW9523_I2C_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = &val;
    msgs[1].len   = 1;

    if (rt_i2c_transfer(aw_i2c_bus, msgs, 2) != 2)
        LOG_E("read reg 0x%02x fail", reg);

    return val;
}

int aw9523_init(void)
{
    aw_i2c_bus = rt_i2c_bus_device_find(AW9523_I2C_NAME);
    if (aw_i2c_bus == RT_NULL)
    {
        LOG_E("can not find i2c bus %s", AW9523_I2C_NAME);
        return -1;
    }

    rt_device_open(&aw_i2c_bus->parent, RT_DEVICE_FLAG_RDWR);

    /* 上电稳定后再通信 */
    rt_thread_mdelay(5);

    /* 校验 ID (固定 0x23) */
    if (aw9523_read_reg(AW9523_REG_ID) != 0x23)
    {
        LOG_E("id check fail, not aw9523");
        return -1;
    }

    /* GCR: D4=1 P0 推挽输出, D[1:0]=11 LED 电流 1/4 档 */
    aw9523_write_reg(AW9523_REG_GCR, 0x13);

    /* 全部切到 GPIO 模式 (1=GPIO, 0=LED) */
    aw9523_write_reg(AW9523_REG_LEDMODE0, 0xFF);
    aw9523_write_reg(AW9523_REG_LEDMODE1, 0xFF);

    /* 初始输出: P0_4~P0_7 关(0), P1_0~P1_3 高(1, LED 灭), P1_4~P1_7 低(0, 复位保持) */
    aw9523_write_reg(AW9523_REG_OUTPUT0, 0x00);
    aw9523_write_reg(AW9523_REG_OUTPUT1, 0x0F);

    /* 方向: P0_0/P0_1 输入(USB/TF 检测), 其余输出 */
    aw9523_write_reg(AW9523_REG_CONFIG0, 0x03);
    aw9523_write_reg(AW9523_REG_CONFIG1, 0x00);

    return 0;
}

void aw9523_set_pin(uint8_t pin, uint8_t val)
{
    uint8_t port = (pin >> 4) & 0x01;
    uint8_t bit  = pin & 0x07;
    uint8_t reg  = AW9523_REG_OUTPUT0 + port;
    uint8_t cur  = aw9523_read_reg(reg);

    if (val)
        cur |= (1 << bit);
    else
        cur &= ~(1 << bit);

    aw9523_write_reg(reg, cur);
}

uint8_t aw9523_get_pin(uint8_t pin)
{
    uint8_t port = (pin >> 4) & 0x01;
    uint8_t bit  = pin & 0x07;
    uint8_t reg  = AW9523_REG_INPUT0 + port;

    return (aw9523_read_reg(reg) >> bit) & 0x01;
}

#else /* !BSP_USING_RTTHREAD: bootloader 裸机环境, 无 I2C 框架, 空实现 */

int aw9523_init(void)
{
    return -1;
}

void aw9523_set_pin(uint8_t pin, uint8_t val)
{
    (void)pin;
    (void)val;
}

uint8_t aw9523_get_pin(uint8_t pin)
{
    (void)pin;
    return 0;
}

#endif /* BSP_USING_RTTHREAD */
