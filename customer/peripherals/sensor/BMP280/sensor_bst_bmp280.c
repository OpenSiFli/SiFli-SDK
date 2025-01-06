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


#include "sensor_bst_bmp280.h"

#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.bmp280"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


static struct bmp280_device *bmp_dev;

static rt_err_t _bmp280_init(void)
{
    if (!BMP280_Init())
    {
        bmp_dev = rt_calloc(1, sizeof(struct bmp280_device));
        if (bmp_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        bmp_dev->bus = (rt_device_t)BMP280GetBus();
        bmp_dev->i2c_addr = BMP280GetDevAddr();
        bmp_dev->id = BMP280GetDevId();
        //mpu_dev->config;
        //BMP280_open();
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _bmp280_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _bmp280_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

static rt_err_t _bmp280_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        BMP280_close();
        break;
    case RT_SENSOR_POWER_NORMAL:
        BMP280_open();
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

static rt_err_t _bmp280_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    //LOG_I("_bmp280_self_test with mode %d\n", mode);
    res = BMP280_SelfCheck();
    if (res < 0)
    {
        LOG_I("_bmp280_self_test selt test failed with %d\n", res);
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_size_t _bmp280_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    int32_t x, y, z;
    static int32_t init_flag = 0;
    if ((sensor->info.type == RT_SENSOR_CLASS_TEMP) || (sensor->info.type == RT_SENSOR_CLASS_BARO))
    {
        BMP280_CalTemperatureAndPressureAndAltitude(&x, &y, &z);
        if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
        {
            data->type = RT_SENSOR_CLASS_TEMP;
            data->data.temp = (int32_t)x;
            data->timestamp = rt_sensor_get_ts();
        }
        else
        {
            data->type = RT_SENSOR_CLASS_BARO;
            data->data.baro = (int32_t)y;
            data->timestamp = rt_sensor_get_ts();
        }
        init_flag++;
    }
    if (init_flag < 8)
        return 0;

    return 1;
}

static rt_size_t bmp280_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _bmp280_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t bmp280_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = bmp_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _bmp280_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _bmp280_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _bmp280_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _bmp280_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    bmp280_fetch_data,
    bmp280_control
};

int rt_hw_bmp280_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_temp = RT_NULL, sensor_baro = RT_NULL;

    result = _bmp280_init();
    if (result != RT_EOK)
    {
        LOG_E("bmp280 init err code: %d", result);
        goto __exit;
    }

    /* temperature sensor register */
    {
        sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_temp == RT_NULL)
            return -1;

        sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
        sensor_temp->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_temp->info.model      = "bmp280_temp";
        sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
        sensor_temp->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_temp->info.range_max  = 85;
        sensor_temp->info.range_min  = -40;
        sensor_temp->info.period_min = 5;

        rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
        sensor_temp->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    /* barometer sensor register */
    {
        sensor_baro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_baro == RT_NULL)
            goto __exit;

        sensor_baro->info.type       = RT_SENSOR_CLASS_BARO;
        sensor_baro->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_baro->info.model      = "bmp280_bora";
        sensor_baro->info.unit       = RT_SENSOR_UNIT_PA;
        sensor_baro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_baro->info.range_max  = 110000;
        sensor_baro->info.range_min  = 30000;
        sensor_baro->info.period_min = 5;

        rt_memcpy(&sensor_baro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_baro->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_baro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_temp)
        rt_free(sensor_temp);
    if (sensor_baro)
        rt_free(sensor_baro);
    if (bmp_dev)
        rt_free(bmp_dev);
    return -RT_ERROR;
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
