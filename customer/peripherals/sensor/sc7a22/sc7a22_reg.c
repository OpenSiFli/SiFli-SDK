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

#include "sc7a22_reg.h"



int32_t sc7a22_read_reg(sifdev_sensor_ctx_t *ctx, uint8_t reg, uint8_t *data,
                        uint16_t len)
{
    int32_t ret;
    ret = ctx->read_reg(ctx->handle, reg, data, len);
    return ret;
}

int32_t sc7a22_write_reg(sifdev_sensor_ctx_t *ctx, uint8_t reg, uint8_t *data,
                         uint16_t len)
{
    int32_t ret;
    ret = ctx->write_reg(ctx->handle, reg, data, len);
    return ret;
}

int32_t sc7a22_device_id_get(sifdev_sensor_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    ret = sc7a22_read_reg(ctx, SC7A22_WHO_AM_I_REG, buff, 1);
    return ret;
}

int32_t sc7a22_reset_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = sc7a22_write_reg(ctx, SC7A22_SOFT_RESET_REG, &val, 1);

    return ret;
}

int32_t sc7a22_xl_filter_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = sc7a22_write_reg(ctx, SC7A22_CTRL0_REG, &val, 1);

    return ret;
}

int32_t sc7a22_xl_data_rate_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = sc7a22_write_reg(ctx, SC7A22_CTRL1_REG, &val, 1);

    return ret;

}

int32_t sc7a22_i2c_enable_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = sc7a22_write_reg(ctx, SC7A22_I2C_ENABLE_REG, &val, 1);

    return ret;
}

int32_t sc7a22_xl_full_scale_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = sc7a22_write_reg(ctx, SC7A22_CTRL4_REG, &val, 1);
    return ret;
}

int32_t sc7a22_fifo_mode_set(sifdev_sensor_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = sc7a22_write_reg(ctx, SC7A22_FIFO_CTRL_REG, &val, 1);
    return ret;
}

int32_t sc7a22_fifo_count_get(sifdev_sensor_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    ret = sc7a22_read_reg(ctx, SC7A22_FIFO_STATUS_REG, buff, 1);
    return ret;
}

int32_t sc7a22_fifo_raw_data_get(sifdev_sensor_ctx_t *ctx, uint8_t *buff, uint8_t len)
{
    int32_t ret;
    ret = sc7a22_read_reg(ctx, SC7A22_FIFO_DATA_REG, buff, len);
    return ret;
}

// note: ctrl reg5 val is diff with sc7a22 and sc7a20.
int32_t sc7a22_fifo_enable_set(sifdev_sensor_ctx_t *ctx, uint8_t val)

{
    int32_t ret;
    ret = sc7a22_write_reg(ctx, SC7A22_CTRL5_REG, &val, 1);
    return ret;
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
