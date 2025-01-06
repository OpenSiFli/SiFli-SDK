/**
  ******************************************************************************
  * @file   mpu6050_wrapper.c
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

#include <rtthread.h>
#include "board.h"
#include "mpu6050_wrapper.h"

static struct rt_i2c_bus_device *sensor_i2c;

int sensor_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
    int ret = -1;
    rt_size_t ret_len;
    if (sensor_i2c)
    {
        ret_len =  rt_i2c_mem_write(sensor_i2c, slave_addr, reg_addr, 8, (void *)data, length);
        if (ret_len == length)
        {
            ret = 0;
        }
    }

    return ret;
}
int sensor_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    int ret = -1;
    rt_size_t ret_len;
    if (sensor_i2c)
    {
        ret_len =  rt_i2c_mem_read(sensor_i2c, slave_addr, reg_addr, 8, data, length);
        if (ret_len == length)
        {
            ret = 0;
        }
    }

    return ret;
}

int sensor_get_ms(unsigned long *ms)
{
    rt_tick_t tick;

    tick = rt_tick_get();

    if (RT_TICK_PER_SECOND == 1000)
    {
        *ms = tick;
    }
    else
    {
        *ms = (uint64_t)tick * RT_TICK_PER_SECOND / 1000;
    }

    return 0;
}

void sensor_mdelay(unsigned long ms)
{
    rt_thread_mdelay(ms);
}

int sensor_reg_int_cb(void (*cb)(void))
{
    return 0;
}


int sensor_i2c_init(void)
{
    sensor_i2c = rt_i2c_bus_device_find(SENSOR_I2C);
    return RT_EOK;
}
INIT_BOARD_EXPORT(sensor_i2c_init);



/**
 *  @}
 */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
