/**
  ******************************************************************************
  * @file   drv_soft_i2c.h
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

#ifndef __DRV_I2C__
#define __DRV_I2C__

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

/* sifli config class */
struct sifli_soft_i2c_config
{
    rt_uint8_t scl;
    rt_uint8_t sda;
    const char *bus_name;
};
/* sifli i2c dirver class */
struct sifli_i2c
{
    struct rt_i2c_bit_ops ops;
    struct rt_i2c_bus_device i2c2_bus;
};

#ifdef BSP_USING_SOFT_I2C1
#define I2C1_BUS_CONFIG                                  \
    {                                                    \
        .scl = BSP_I2C1_SCL_PIN,                         \
        .sda = BSP_I2C1_SDA_PIN,                         \
        .bus_name = "i2c1",                              \
    }
#endif

#ifdef BSP_USING_SOFT_I2C2
#define I2C2_BUS_CONFIG                                  \
    {                                                    \
        .scl = BSP_I2C2_SCL_PIN,                         \
        .sda = BSP_I2C2_SDA_PIN,                         \
        .bus_name = "i2c2",                              \
    }
#endif

#ifdef BSP_USING_SOFT_I2C3
#define I2C3_BUS_CONFIG                                  \
    {                                                    \
        .scl = BSP_I2C3_SCL_PIN,                         \
        .sda = BSP_I2C3_SDA_PIN,                         \
        .bus_name = "i2c3",                              \
    }
#endif

#ifdef BSP_USING_SOFT_I2C4
#define I2C4_BUS_CONFIG                                  \
    {                                                    \
        .scl = BSP_I2C4_SCL_PIN,                         \
        .sda = BSP_I2C4_SDA_PIN,                         \
        .bus_name = "i2c4",                              \
    }
#endif
int rt_hw_i2c_init(void);

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
