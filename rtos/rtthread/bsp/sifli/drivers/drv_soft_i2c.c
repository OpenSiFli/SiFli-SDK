/**
  ******************************************************************************
  * @file   drv_soft_i2c.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <board.h>
#include "drv_soft_i2c.h"
#include "drv_config.h"

#ifdef BSP_USING_SOFT_I2C

//#define DRV_DEBUG
#define LOG_TAG              "drv.i2c"
#include <drv_log.h>

#if !defined(BSP_USING_SOFT_I2C1) && !defined(BSP_USING_SOFT_I2C2) && !defined(BSP_USING_SOFT_I2C3) && !defined(BSP_USING_SOFT_I2C4)
    #error "Please define at least one BSP_USING_I2Cx"
    /* this driver can be disabled at menuconfig → RT-Thread Components → Device Drivers */
#endif

static const struct sifli_soft_i2c_config soft_i2c_config[] =
{
#ifdef BSP_USING_SOFT_I2C1
    I2C1_BUS_CONFIG,
#endif
#ifdef BSP_USING_SOFT_I2C2
    I2C2_BUS_CONFIG,
#endif
#ifdef BSP_USING_SOFT_I2C3
    I2C3_BUS_CONFIG,
#endif
#ifdef BSP_USING_SOFT_I2C4
    I2C4_BUS_CONFIG,
#endif
};

static struct sifli_i2c i2c_obj[sizeof(soft_i2c_config) / sizeof(soft_i2c_config[0])];

/**
 * This function initializes the i2c pin.
 *
 * @param i2c i2c dirver class.
 */
static void sifli_i2c_gpio_init(struct sifli_i2c *i2c)
{
    struct sifli_soft_i2c_config *cfg = (struct sifli_soft_i2c_config *)i2c->ops.data;

    rt_pin_mode(cfg->scl, PIN_MODE_OUTPUT_OD);
    rt_pin_mode(cfg->sda, PIN_MODE_OUTPUT_OD);

    rt_pin_write(cfg->scl, PIN_HIGH);
    rt_pin_write(cfg->sda, PIN_HIGH);
}

/**
 * This function sets the sda pin.
 *
 * @param cfg config class.
 * @param The sda pin state.
 */
static void sifli_set_sda(void *data, rt_int32_t state)
{
    struct sifli_soft_i2c_config *cfg = (struct sifli_soft_i2c_config *)data;
    if (state)
    {
        rt_pin_mode(cfg->sda, PIN_MODE_INPUT_PULLUP);
        rt_pin_write(cfg->sda, PIN_HIGH);
    }
    else
    {
        rt_pin_mode(cfg->sda, PIN_MODE_OUTPUT_OD);
        rt_pin_write(cfg->sda, PIN_LOW);
    }
}

/**
 * This function sets the scl pin.
 *
 * @param cfg config class.
 * @param The scl pin state.
 */
static void sifli_set_scl(void *data, rt_int32_t state)
{
    struct sifli_soft_i2c_config *cfg = (struct sifli_soft_i2c_config *)data;
    if (state)
    {
        rt_pin_write(cfg->scl, PIN_HIGH);
    }
    else
    {
        rt_pin_write(cfg->scl, PIN_LOW);
    }
}

/**
 * This function gets the sda pin state.
 *
 * @param The sda pin state.
 */
static rt_int32_t sifli_get_sda(void *data)
{
    struct sifli_soft_i2c_config *cfg = (struct sifli_soft_i2c_config *)data;
    rt_uint8_t sda;
    rt_pin_mode(cfg->sda, PIN_MODE_INPUT_PULLUP);
    sda = rt_pin_read(cfg->sda);
//    rt_pin_mode(cfg->sda, PIN_MODE_OUTPUT_OD);
    return sda;
}

/**
 * This function gets the scl pin state.
 *
 * @param The scl pin state.
 */
static rt_int32_t sifli_get_scl(void *data)
{
    struct sifli_soft_i2c_config *cfg = (struct sifli_soft_i2c_config *)data;
    return rt_pin_read(cfg->scl);
}
/**
 * The time delay function.
 *
 * @param microseconds.
 */
static void sifli_udelay(rt_uint32_t us)
{
    rt_uint32_t ticks;
    rt_uint32_t told, tnow, tcnt = 0;
    rt_uint32_t reload = SysTick->LOAD;

    ticks = us * reload / (1000000 / RT_TICK_PER_SECOND);
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

static const struct rt_i2c_bit_ops sifli_bit_ops_default =
{
    .data     = RT_NULL,
    .set_sda  = sifli_set_sda,
    .set_scl  = sifli_set_scl,
    .get_sda  = sifli_get_sda,
    .get_scl  = sifli_get_scl,
    .udelay   = sifli_udelay,
    .delay_us = 1,
    .timeout  = 100
};

/**
 * if i2c is locked, this function will unlock it
 *
 * @param sifli config class
 *
 * @return RT_EOK indicates successful unlock.
 */
static rt_err_t sifli_i2c_bus_unlock(const struct sifli_soft_i2c_config *cfg)
{
    rt_int32_t i = 0;

    if (PIN_LOW == rt_pin_read(cfg->sda))
    {
        while (i++ < 9)
        {
            rt_pin_write(cfg->scl, PIN_HIGH);
            sifli_udelay(100);
            rt_pin_write(cfg->scl, PIN_LOW);
            sifli_udelay(100);
        }
    }
    if (PIN_LOW == rt_pin_read(cfg->sda))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

/* I2C initialization function */
int rt_sw_i2c_init(void)
{
    rt_size_t obj_num = sizeof(i2c_obj) / sizeof(struct sifli_i2c);
    rt_err_t result;

    for (int i = 0; i < obj_num; i++)
    {
        i2c_obj[i].ops = sifli_bit_ops_default;
        i2c_obj[i].ops.data = (void *)&soft_i2c_config[i];
        i2c_obj[i].i2c2_bus.priv = &i2c_obj[i].ops;
        sifli_i2c_gpio_init(&i2c_obj[i]);
        result = rt_i2c_bit_add_bus(&i2c_obj[i].i2c2_bus, soft_i2c_config[i].bus_name);
        RT_ASSERT(result == RT_EOK);
        sifli_i2c_bus_unlock(&soft_i2c_config[i]);

        LOG_D("software simulation %s init done, pin scl: %d, pin sda %d",
              soft_i2c_config[i].bus_name,
              soft_i2c_config[i].scl,
              soft_i2c_config[i].sda);
    }

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_sw_i2c_init);

#endif /* RT_USING_I2C */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
