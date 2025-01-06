/**
  ******************************************************************************
  * @file   sensor_gps_uc6226.c
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



#include "board.h"

#include "sensor.h"
#include "sensor_gps_uc6226.h"

#ifdef RT_USING_SENSOR

//#define DBG_TAG "sensor.uc6226"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct uc6226_device *gps_dev;

static rt_err_t _uc6226_init(void)
{
    if (!um_gps_init())
    {
        gps_dev = rt_calloc(1, sizeof(struct uc6226_device));
        if (gps_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        gps_dev->bus = (rt_device_t)gps_hal_get_uart();
        gps_dev->dev_addr = 0;
        gps_dev->id = 0;
        //gps_dev->config;
        //um_gps_open();
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _uc6226_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_GPS)
    {
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _uc6226_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    if (mode == RT_SENSOR_MODE_POLLING)
    {
        LOG_D("set mode to POLLING");
    }
    else
    {
        LOG_D("Unsupported mode, code is %d", mode);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _uc6226_set_odr(rt_sensor_t sensor, rt_uint32_t rate)
{
    if (rate > 0 && rate < 10)
    {
        LOG_D("set output rate to %d\n", rate);
        if (rate >= 5)
            um_gps_set_freq(200);
        else if (rate >= 2)
            um_gps_set_freq(500);
        else
            um_gps_set_freq(1000);
    }
    else
    {
        LOG_D("Unsupported rate, code is %d", rate);
        return -RT_ERROR;
    }
    return RT_EOK;
}


static rt_err_t _uc6226_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    rt_err_t res = RT_EOK;
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        res = um_gps_close();
        break;
    case RT_SENSOR_POWER_NORMAL:
        res = um_gps_open();
        break;
    case RT_SENSOR_POWER_LOW:
        break;
    case RT_SENSOR_POWER_HIGH:
        break;
    default:
        break;
    }
    return res;
}

static rt_err_t _uc6226_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    //LOG_I("_uc6226_self_test with mode %d\n", mode);
    res = um_gps_self_check();
    if (res != 0)
    {
        LOG_I("_uc6226_self_test selt test failed with %d\n", res);
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_size_t _uc6226_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    double nor, east, alti;
    if (sensor->info.type == RT_SENSOR_CLASS_GPS)
    {
        um_gps_get_location(&nor, &east, &alti);

        data->type = RT_SENSOR_CLASS_GPS;
        data->data.gps.lati = nor;
        data->data.gps.longi = east;
        data->data.gps.alti = alti;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_size_t uc6226_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _uc6226_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t uc6226_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = gps_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _uc6226_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = _uc6226_set_odr(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _uc6226_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _uc6226_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _uc6226_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    uc6226_fetch_data,
    uc6226_control
};

int rt_hw_uc6226_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_gps = RT_NULL, sensor_baro = RT_NULL;

    result = _uc6226_init();
    if (result != RT_EOK)
    {
        LOG_E("uc6226 init err code: %d", result);
        goto __exit;
    }

    /* GPS sensor register */
    {
        sensor_gps = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_gps == RT_NULL)
            return -1;

        sensor_gps->info.type       = RT_SENSOR_CLASS_GPS;
        sensor_gps->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_gps->info.model      = "uc6226_gps";
        sensor_gps->info.unit       = RT_SENSOR_UNIT_DEG;
        sensor_gps->info.intf_type  = RT_SENSOR_INTF_UART;
        sensor_gps->info.range_max  = 180;
        sensor_gps->info.range_min  = -180;
        sensor_gps->info.period_min = 1000;

        rt_memcpy(&sensor_gps->config, cfg, sizeof(struct rt_sensor_config));
        sensor_gps->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_gps, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_gps)
        rt_free(sensor_gps);
    if (gps_dev)
        rt_free(gps_dev);
    return -RT_ERROR;
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
