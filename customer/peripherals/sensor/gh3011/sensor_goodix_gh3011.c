/**
  ******************************************************************************
  * @file   sensor_goodix_gh3011.c
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



#include "sensor_goodix_gh3011.h"
#include "gh3011.h"
#include "gh3011_comm.h"

#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.gh3011"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


static struct gh3011_device *gh3011_dev;
static rt_sensor_t sensor_hr = RT_NULL;
static uint8_t gh3011_inited = 0;


static rt_err_t _gh3011_init(void)
{
    if (init_gh3011_sensor() == 0)
    {
        gh3011_dev = rt_calloc(1, sizeof(struct gh3011_device));
        if (gh3011_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        gh3011_dev->bus = (rt_device_t)gh3011_get_i2c_handle();
        gh3011_dev->i2c_addr = gh3011_get_dev_addr();
        gh3011_dev->id = 0;
        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _gh3011_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    return RT_EOK;
}

static rt_err_t _gh3011_self_test(rt_sensor_t sensor, rt_int8_t mode)
{
    int res;

    LOG_D("gh3011 test mode %d\n", mode);
    res = gh3011_self_check();
    if (res != 0)
    {
        LOG_D("gh3011 selt test failed with %d\n", res);
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_err_t _gh3011_hr_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

static rt_err_t _gh3011_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    // gpio disable/enable?
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        close_gh3011();
        rt_hw_gh3011_deinit();
        break;
    case RT_SENSOR_POWER_NORMAL:
        open_gh3011();
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

static rt_size_t _gh3011_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_HR)
    {
        data->type = RT_SENSOR_CLASS_HR;
        data->data.hr =  gh3011_get_hr();
        data->timestamp = rt_sensor_get_ts();
    }
    return 1;
}

static rt_size_t gh3011_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _gh3011_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t gh3011_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    LOG_D("hr cmd %d\n", cmd);

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
    {
        hr_sensor_info_t *info = (hr_sensor_info_t *)args;
        result = rt_hw_gh3011_init()

                 if (result != RT_EOK)
                     info->hr_id = 0;
        else
            info->hr_id = gh3011_dev->id;
        break;
    }
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _gh3011_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _gh3011_hr_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _gh3011_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _gh3011_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    gh3011_fetch_data,
    gh3011_control
};

int rt_hw_gh3011_register(const char *name, struct rt_sensor_config *cfg)
{
    int result;
    /* magnetometer/compass sensor register */
    {
        sensor_hr = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_hr == RT_NULL)
            goto __exit;

        sensor_hr->info.type       = RT_SENSOR_CLASS_HR;
        sensor_hr->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_hr->info.model      = "gh3011_hr";
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
    {
        rt_free(sensor_hr);
        sensor_hr = RT_NULL;
    }
    return -RT_ERROR;
}

int gh3011_sensor_register(void)
{
    int ret = 0;
    struct rt_sensor_config cfg;
    cfg.intf.dev_name = GH3011_I2C_BUS;
    cfg.irq_pin.pin = GH3011_INT_BIT; // note: if driver is LCPU, iqr_pin must be config
    ret = rt_hw_gh3011_register(HR_MODEL_NAME, &cfg)

          return ret;
}

INIT_COMPONENT_EXPORT(gh3011_sensor_register);

int rt_hw_gh3011_init(void)
{
    rt_int8_t result = RT_EOK;
    if (!gh3011_inited)
    {
        result = _gh3011_init();
        if (result != RT_EOK)
        {
            LOG_E("afe4404 init err code: %d", result);
            if (gh3011_dev)
            {
                rt_free(gh3011_dev);
                gh3011_dev = RT_NULL;
            }
        }
        else
        {
            gh3011_inited = 1;
        }
    }
    return result;
}

int rt_hw_gh3011_deinit(void)
{
    int ret = RT_EOK;

    if (gh3011_dev)
    {
        rt_free(gh3011_dev);
        gh3011_dev = RT_NULL;
    }
    gh3011_inited = 0;
    return ret;
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
