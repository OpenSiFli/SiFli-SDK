/**
  ******************************************************************************
  * @file   sensor_ti_afe4404.c
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



#include "sensor_ti_afe4404.h"
#include "pps960.h"

#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.afe4404"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


static struct afe4404_device *afe_dev;

static rt_err_t _afe4404_init(void)
{
    if (init_pps960_sensor() == 0)
    {
        afe_dev = rt_calloc(1, sizeof(struct afe4404_device));
        if (afe_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        afe_dev->bus = (rt_device_t)pps960_get_i2c_handle();
        afe_dev->i2c_addr = pps960_get_dev_addr();
        afe_dev->id = 0;
        //open_pps960();
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _afe4404_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    return RT_EOK;
}

static rt_err_t _afe4404_self_test(rt_sensor_t sensor, rt_int8_t mode)
{
    int res;

    LOG_D("afe4404 test mode %d\n", mode);
    res = pps960_self_check();
    if (res != 0)
    {
        LOG_D("afe4404 selt test failed with %d\n", res);
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_err_t _afe4404_hr_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

static rt_err_t _afe4404_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    // gpio disable/enable?
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        close_pps960();
        break;
    case RT_SENSOR_POWER_NORMAL:
        open_pps960();
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

static rt_size_t _afe4404_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_HR)
    {
        data->type = RT_SENSOR_CLASS_HR;
        data->data.hr = pps960_get_hr();
        data->timestamp = rt_sensor_get_ts();
    }
    return 1;
}

static rt_size_t afe4404_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _afe4404_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t afe4404_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    LOG_D("hr cmd %d\n", cmd);

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = afe_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _afe4404_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _afe4404_hr_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _afe4404_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _afe4404_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    afe4404_fetch_data,
    afe4404_control
};

int rt_hw_afe4404_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_hr = RT_NULL;

    result = _afe4404_init();
    if (result != RT_EOK)
    {
        LOG_E("afe4404 init err code: %d", result);
        goto __exit;
    }

    /* magnetometer/compass sensor register */
    {
        sensor_hr = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_hr == RT_NULL)
            goto __exit;

        sensor_hr->info.type       = RT_SENSOR_CLASS_HR;
        sensor_hr->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_hr->info.model      = "afe4404_hr";
        sensor_hr->info.unit       = RT_SENSOR_UNIT_BPM;
        sensor_hr->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_hr->info.range_max  = 220;
        sensor_hr->info.range_min  = 30;
        sensor_hr->info.period_min = 1;
        sensor_hr->data_len = 0;
        sensor_hr->data_buf = NULL;

        rt_memcpy(&sensor_hr->config, cfg, sizeof(struct rt_sensor_config));
        sensor_hr->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_hr, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:

    if (sensor_hr)
        rt_free(sensor_hr);
    if (afe_dev)
        rt_free(afe_dev);
    return -RT_ERROR;
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
