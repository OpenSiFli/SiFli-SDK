/**
  ******************************************************************************
  * @file   sc7a22_reg.c
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

#include "stk8328c_reg.h"



int32_t stk8328c_read_reg(sifdev_sensor_ctx_t *ctx, uint8_t reg, uint8_t *data,
                          uint16_t len)
{
    int32_t ret;
    ret = ctx->read_reg(ctx->handle, reg, data, len);
    return ret;
}

int32_t stk8328c_write_reg(sifdev_sensor_ctx_t *ctx, uint8_t reg, uint8_t *data,
                           uint16_t len)
{
    int32_t ret;
    ret = ctx->write_reg(ctx->handle, reg, data, len);
    return ret;
}

int32_t stk8328c_device_id_get(sifdev_sensor_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    ret = stk8328c_read_reg(ctx, STK_REG_CHIPID, buff, 1);
    return ret;
}

int32_t stk8328c_reset_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = stk8328c_write_reg(ctx, STK_REG_SOFT_RESET, &val, 1);

    return ret;
}

int32_t stk8328c_xl_full_scale_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = stk8328c_write_reg(ctx, STK832x_REG_RANGESEL, &val, 1);
    return ret;
}

int32_t stk8328c_fifo_mode_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = stk8328c_write_reg(ctx, STK_REG_FIFO_MODE, &val, 1);
    return ret;
}

int32_t stk8328c_fifo_count_get(sifdev_sensor_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    ret = stk8328c_read_reg(ctx, STK_REG_FIFO_COUNT, buff, 1);
    return ret;
}

int32_t stk8328c_eng_mode_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = stk8328c_write_reg(ctx, STK_REG_ENG_MODE, &val, 1);
    return ret;
}

int32_t stk8328c_i2c_wdt_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = stk8328c_write_reg(ctx, STK_REG_I2C_WDT, &val, 1);
    return ret;
}

int32_t stk8328c_band_width_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = stk8328c_write_reg(ctx, STK_REG_BAND_WIDTH, &val, 1);
    return ret;
}

int32_t stk8328c_es_mode_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = stk8328c_write_reg(ctx, STK_REG_ES_MODE, &val, 1);
    return ret;
}

int32_t stk8328c_power_mode_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = stk8328c_write_reg(ctx, STK_REG_POWMODE, &val, 1);
    return ret;
}

int32_t stk8328c_read_fifo_data(sifdev_sensor_ctx_t *ctx, uint8_t *data, uint16_t len)
{
    int32_t ret;
    ret = stk8328c_read_reg(ctx, STK_REG_FIFO_DATA, data, len);
    return ret;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
