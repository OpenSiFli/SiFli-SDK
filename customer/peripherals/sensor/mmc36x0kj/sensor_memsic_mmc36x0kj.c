/**
  ******************************************************************************
  * @file   sensor_memsic_mmc36x0kj.c
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


#include "sensor_memsic_mmc36x0kj.h"
#include "MMC36X_Customer.h"

#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.mmc36x0kj"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


static struct mmc36x0kj_device *mmc_dev;

static rt_err_t _mmc36x0kj_init(void)
{
    if (MMC36X0KJ_Initialization() == 1)
    {
        mmc_dev = rt_calloc(1, sizeof(struct mmc36x0kj_device));
        if (mmc_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        mmc_dev->bus = (rt_device_t)MMC36X0KJ_get_bus();
        mmc_dev->i2c_addr = MMC36X0KJ_7BITI2C_ADDRESS;
        mmc_dev->id = MMC36X0KJ_PRODUCT_ID;
        //mpu_dev->config;
        //MMC36X0KJ_Enable();
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _mmc36x0kj_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    return RT_EOK;
}

static rt_err_t _mmc36x0kj_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

static rt_err_t _mmc36x0kj_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    LOG_I("_mmc36x0kj_self_test with mode %d\n", mode);
    res = MMC36X0KJ_self_check();
    if (res < 0)
    {
        LOG_I("mmc36x0kj selt test failed with %d\n", res);
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_err_t _mmc36x0kj_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        // MT register will auto clear after read, so disable seems no use
        MMC36X0KJ_Disable();
        break;
    case RT_SENSOR_POWER_NORMAL:
        MMC36X0KJ_Enable();
        break;
    case RT_SENSOR_POWER_LOW:
        break;
    case RT_SENSOR_POWER_HIGH:
        break;
    default:
        break;
    }
    return RT_EOK;
}

static rt_size_t _mmc36x0kj_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    float mag[3];
    if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        MMC36X0KJ_GetData(mag);
        data->type = RT_SENSOR_CLASS_MAG;
        data->data.mag.x = (int32_t)(mag[0] * 1000);
        data->data.mag.y = (int32_t)(mag[1] * 1000);
        data->data.mag.z = (int32_t)(mag[2] * 1000);
        data->timestamp = rt_sensor_get_ts();
    }
    return 1;
}

static rt_size_t mmc36x0kj_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _mmc36x0kj_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t mmc36x0kj_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = mmc_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _mmc36x0kj_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _mmc36x0kj_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _mmc36x0kj_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _mmc36x0kj_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    mmc36x0kj_fetch_data,
    mmc36x0kj_control
};

int rt_hw_mmc36x0kj_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_mag = RT_NULL;

    result = _mmc36x0kj_init();
    if (result != RT_EOK)
    {
        LOG_E("mmc36x0kj init err code: %d", result);
        goto __exit;
    }

    /* magnetometer/compass sensor register */
    {
        sensor_mag = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_mag == RT_NULL)
            goto __exit;

        sensor_mag->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_mag->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_mag->info.model      = "mmc36x0kj_mag";
        sensor_mag->info.unit       = RT_SENSOR_UNIT_MGAUSS;
        sensor_mag->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_mag->info.range_max  = 60000;
        sensor_mag->info.range_min  = -60000;
        sensor_mag->info.period_min = 5;

        rt_memcpy(&sensor_mag->config, cfg, sizeof(struct rt_sensor_config));
        sensor_mag->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_mag, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:

    if (sensor_mag)
        rt_free(sensor_mag);
    if (mmc_dev)
        rt_free(mmc_dev);
    return -RT_ERROR;
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
