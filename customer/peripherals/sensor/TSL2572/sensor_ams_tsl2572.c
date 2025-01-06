/**
  ******************************************************************************
  * @file   sensor_bst_bmp280.c
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


#include "sensor_ams_tsl2572.h"

#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.tsl2572"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


static struct tsl2572_device *tsl_dev;

static rt_err_t _tsl2572_init(void)
{
    if (!TSL2572_Init())
    {
        tsl_dev = rt_calloc(1, sizeof(struct tsl2572_device));
        if (tsl_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        tsl_dev->bus = (rt_device_t)TSL2572_Get_Bus();
        tsl_dev->i2c_addr = TSL2572_ADDRESS;
        tsl_dev->id = TSL2572_Get_ID();
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _tsl2572_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_LIGHT)
    {
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _tsl2572_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        break;
    case RT_SENSOR_POWER_NORMAL:
        break;
    case RT_SENSOR_POWER_LOW:
        // can change osrs_p, osrs_t, odr ?
        break;
    case RT_SENSOR_POWER_HIGH:
        break;
    default:
        break;
    }
    return RT_EOK;
}

static rt_err_t _tsl2572_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    return RT_EOK;
}

static rt_size_t _tsl2572_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    uint32_t value;
    if (sensor->info.type == RT_SENSOR_CLASS_LIGHT)
    {
        value = TSL2572_Read_Visible();

        data->type = RT_SENSOR_CLASS_LIGHT;
        data->data.light = (int32_t)value;
        data->timestamp = rt_sensor_get_ts();

    }

    return 1;
}

static rt_size_t tsl2572_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _tsl2572_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t tsl2572_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = tsl_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _tsl2572_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _tsl2572_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _tsl2572_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    tsl2572_fetch_data,
    tsl2572_control
};

int rt_hw_tsl2572_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_als = RT_NULL;

    result = _tsl2572_init();
    if (result != RT_EOK)
    {
        LOG_E("tsl2572 init err code: %d", result);
        goto __exit;
    }

    /* temperature sensor register */
    {
        sensor_als = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_als == RT_NULL)
            return -1;

        sensor_als->info.type       = RT_SENSOR_CLASS_LIGHT;
        sensor_als->info.vendor     = RT_SENSOR_VENDOR_AMS;
        sensor_als->info.model      = "tsl2572_als";
        sensor_als->info.unit       = RT_SENSOR_UNIT_LUX;
        sensor_als->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_als->info.range_max  = 65525;
        sensor_als->info.range_min  = 0;
        sensor_als->info.period_min = 5;

        rt_memcpy(&sensor_als->config, cfg, sizeof(struct rt_sensor_config));
        sensor_als->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_als, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_als)
        rt_free(sensor_als);
    if (tsl_dev)
        rt_free(tsl_dev);
    return -RT_ERROR;
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
