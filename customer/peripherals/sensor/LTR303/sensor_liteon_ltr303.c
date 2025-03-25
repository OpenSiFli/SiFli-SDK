/**
  ******************************************************************************
  * @file   sensor_liteon_ltr303.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2025,  Sifli Technology
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


#include "sensor_liteon_ltr303.h"

#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.ltr-303"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_size_t ltr303_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    // ltr303_device_t hdev = sensor->parent.user_data;
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;

    if (sensor->info.type == RT_SENSOR_CLASS_LIGHT)
    {
        uint16_t light_value;
        light_value = LTR303_ReadVisible();

        data->type = RT_SENSOR_CLASS_LIGHT;
        data->data.light = (rt_int32_t)(light_value);
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

rt_err_t ltr303_set_power(ltr303_device_t hdev, rt_uint8_t power)
{
    (void)hdev;
    if (power == RT_SENSOR_POWER_NORMAL)
    {
        LTR303_PowerOn();
    }
    else if (power == RT_SENSOR_POWER_DOWN)
    {
        LTR303_PowerOff();
    }
    else
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t ltr303_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    // ltr303_device_t hdev = sensor->parent.user_data;
    ltr303_device_t hdev;
    switch (cmd)
    {
        case RT_SENSOR_CTRL_SET_POWER:
        {
            result = ltr303_set_power(hdev, (rt_uint32_t)args & 0xff);
            break;
        }
        case RT_SENSOR_CTRL_SELF_TEST:
        {
            result =  -RT_EINVAL;
            break;
        }
        default:
        {
            result = RT_EOK;
            break;
        }
    }

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    ltr303_fetch_data,
    ltr303_control
};

int rt_hw_ltr303_init(const char *name, struct rt_sensor_config *cfg)
{
    int result = -RT_ERROR;
    rt_sensor_t sensor = RT_NULL;
    LTR303_Init(cfg);

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (RT_NULL == sensor)
    {
        LOG_E("calloc failed");
        return -RT_ERROR;
    }

    sensor->info.type       = RT_SENSOR_CLASS_LIGHT;
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
    sensor->info.model      = "ltr303_light";
    sensor->info.unit       = RT_SENSOR_UNIT_LUX;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 65535;
    sensor->info.range_min  = 1;
    sensor->info.period_min = 120;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }
    else
    {
        LOG_I("light sensor init success");
        return RT_EOK;
    }
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
