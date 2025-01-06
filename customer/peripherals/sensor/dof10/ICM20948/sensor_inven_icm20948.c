/**
  ******************************************************************************
  * @file   sensor_inven_icm20948.c
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

#ifdef RT_USING_SENSOR

#include "sensor_inven_icm20948.h"

#define DBG_TAG "sensor.icm20948"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#include "sensor.h"

static struct icm20948_device *icm_dev;

static rt_err_t _icm20948_init(void)
{
    if (invmsICM20948Init())
    {
        icm_dev = rt_calloc(1, sizeof(struct icm20948_device));
        if (icm_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        icm_dev->bus = invmsICM20948GetBus();
        icm_dev->i2c_addr = invmsICM20948GetDevAddr();
        icm_dev->id = invmsICM20948GetDevId();
        //mpu_dev->config;
    }

    return RT_EOK;
}

static rt_err_t _icm20948_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        uint8_t range_ctr;

        if (range < 2000)
            range_ctr = REG_VAL_BIT_ACCEL_FS_2g;
        else if (range < 4000)
            range_ctr = REG_VAL_BIT_ACCEL_FS_4g;
        else if (range < 8000)
            range_ctr = REG_VAL_BIT_ACCEL_FS_8g;
        else
            range_ctr = REG_VAL_BIT_ACCEL_FS_16g;

        LOG_D("acce set range %d", range_ctr);

        invmsICM20948SetAccelRange(range_ctr);
        icm_dev->config.accel_range = range_ctr;
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        uint8_t range_ctr;

        if (range < 250000UL)
            range_ctr = REG_VAL_BIT_GYRO_FS_250DPS;
        else if (range < 500000UL)
            range_ctr = REG_VAL_BIT_GYRO_FS_500DPS;
        else if (range < 1000000UL)
            range_ctr = REG_VAL_BIT_GYRO_FS_1000DPS;
        else
            range_ctr = REG_VAL_BIT_GYRO_FS_2000DPS;

        LOG_D("gyro set range %d", range);

        invmsICM20948SetGyroRange(range_ctr);
        icm_dev->config.gyro_range = range_ctr;
    }
    return RT_EOK;
}

static rt_err_t _icm20948_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

static rt_err_t _icm20948_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    return RT_EOK;
}

static rt_size_t _icm20948_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    int16_t x, y, z;
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        invmsICM20948AccelRead(&x, &y, &z);

        data->type = RT_SENSOR_CLASS_ACCE;
        data->data.acce.x = (int32_t)x;
        data->data.acce.y = (int32_t)y;
        data->data.acce.z = (int32_t)z;
        data->timestamp = rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        invmsICM20948GyroRead(&x, &y, &z);
        data->type = RT_SENSOR_CLASS_GYRO;
        data->data.gyro.x = (int32_t)x * 100;
        data->data.gyro.y = (int32_t)y * 100;
        data->data.gyro.z = (int32_t)z * 100;
        data->timestamp = rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        invmsICM20948MagRead(&x, &y, &z);
        data->type = RT_SENSOR_CLASS_MAG;
        data->data.mag.x = (int32_t)x;
        data->data.mag.y = (int32_t)y;
        data->data.mag.z = (int32_t)z;
        data->timestamp = rt_sensor_get_ts();
    }
    return 1;
}

static rt_size_t icm20948_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _icm20948_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t icm20948_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = icm_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _icm20948_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _icm20948_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _icm20948_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    icm20948_fetch_data,
    icm20948_control
};

int rt_hw_icm20948_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_acce = RT_NULL, sensor_gyro = RT_NULL, sensor_mag = RT_NULL;

    /* accelerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            return -1;

        sensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_INVENSENSE;
        sensor_acce->info.model      = "icm20948_acc";
        sensor_acce->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_acce->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_acce->info.range_max  = 16000;
        sensor_acce->info.range_min  = 2000;
        sensor_acce->info.period_min = 5;

        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));
        sensor_acce->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_acce, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    /* gyroscope sensor register */
    {
        sensor_gyro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_gyro == RT_NULL)
            goto __exit;

        sensor_gyro->info.type       = RT_SENSOR_CLASS_GYRO;
        sensor_gyro->info.vendor     = RT_SENSOR_VENDOR_INVENSENSE;
        sensor_gyro->info.model      = "icm20948_gyro";
        sensor_gyro->info.unit       = RT_SENSOR_UNIT_MDPS;
        sensor_gyro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_gyro->info.range_max  = 2000000;
        sensor_gyro->info.range_min  = 250000;
        sensor_gyro->info.period_min = 5;

        rt_memcpy(&sensor_gyro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_gyro->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_gyro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
    /* magnetometer/compass sensor register */
    {
        sensor_mag = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_gyro == RT_NULL)
            goto __exit;

        sensor_gyro->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_gyro->info.vendor     = RT_SENSOR_VENDOR_INVENSENSE;
        sensor_gyro->info.model      = "icm20948_mag";
        sensor_gyro->info.unit       = RT_SENSOR_UNIT_MGAUSS;
        sensor_gyro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_gyro->info.range_max  = 4900000;
        sensor_gyro->info.range_min  = 4900000;
        sensor_gyro->info.period_min = 5;

        rt_memcpy(&sensor_gyro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_gyro->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_gyro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    result = _icm20948_init();
    if (result != RT_EOK)
    {
        LOG_E("icm20948 init err code: %d", result);
        goto __exit;
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_acce)
        rt_free(sensor_acce);
    if (sensor_gyro)
        rt_free(sensor_gyro);
    if (sensor_mag)
        rt_free(sensor_mag);
    if (icm_dev)
        rt_free(icm_dev);
    return -RT_ERROR;
}

#endif // RT_USING_SENSOR/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
