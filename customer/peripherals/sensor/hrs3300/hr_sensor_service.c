/**
  ******************************************************************************
  * @file   hr_sensor_service.c
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

#include "rtthread.h"

#if defined(HR_USING_HRS3300)
#include "hx_hrs3300.h"
#include "hr_sensor_service.h"

#ifdef PMIC_CONTROL_SERVICE
    #include "pmic_service.h"
#endif

extern void Hrs3300_bp_alg_open(void);
extern void Hrs3300_bp_alg_close(void);
extern bool Hrs3300_alg_open(void);
extern void Hrs3300_alg_close(void);

static struct hr_sensor_device *hr_dev = RT_NULL;
static rt_sensor_t hr_sensor = RT_NULL;

static rt_err_t hrs_open(void)
{
    rt_err_t result = RT_EOK;

    rt_kprintf("hrs_hw_init\n");

    if (!hr_dev)
    {
#ifdef PMIC_CONTROL_SERVICE
        pmic_service_control(PMIC_CONTROL_HR, 1);
#else
        hr_hw_power_onoff(true);
#endif
        rt_thread_mdelay(20);

        hr_dev = rt_calloc(1, sizeof(struct hr_sensor_device));

        if (!hr_dev)
        {
            result = RT_ENOMEM;
            goto err;
        }

        if (RT_EOK != hr_hw_init())
        {
            hr_hw_deinit();
            result = RT_ERROR;
            rt_free(hr_dev);
            goto err;
        }

        hr_dev->bus = (rt_device_t)Hrs3300_get_device_handler();
        hr_dev->i2c_addr = Hrs3300_get_dev_addr();
        hr_dev->id = Hrs3300_get_dev_id();

        Hrs3300_chip_enable();
        Hrs3300_alg_open();

        rt_kprintf("hrs_open\n");

        return result;
    }

err:
    rt_kprintf("hrs3300 init err code: %d\n", result);
    hr_dev = RT_NULL;
    return result;
}

static rt_err_t hrs_close(void)
{
    rt_err_t ret = RT_EOK;

    rt_kprintf("hrs_close\n");

    if (hr_dev)
    {

        Hrs3300_alg_close();
        Hrs3300_chip_disable();

        //hr_hw_deinit();

#ifdef PMIC_CONTROL_SERVICE
        pmic_service_control(PMIC_CONTROL_HR, 0);
#else
        hr_hw_power_onoff(false);
#endif
        rt_free(hr_dev);
        hr_dev = RT_NULL;
    }

    rt_kprintf("hrs_close1\n");

    return ret;
}

static rt_err_t hrs_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    //rt_kprintf("-------range = %d\r\n",range);
    switch (range)
    {
    case RT_SENSOR_POWER_LOW:
        break;
    case RT_SENSOR_POWER_HIGH:
        break;
    default:
        ;
    }

    return RT_EOK;
}

static rt_err_t hrs_self_test(rt_sensor_t sensor, rt_int8_t mode)
{
    rt_kprintf("hrs3300 test mode %d\n", mode);
    rt_err_t res = hr_hw_self_check();
    if (res != 0)
    {
        rt_kprintf("hrs3300 selt test failed with %d\n", res);
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_err_t hrs_hr_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    if (mode == RT_SENSOR_MODE_POLLING)
    {
        rt_kprintf("set mode to POLLING\n");
    }
    else
    {
        rt_kprintf("Unsupported mode, code is %d\n", mode);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t hrs_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    rt_err_t ret = RT_EOK;

    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        hrs_close();
        break;
    case RT_SENSOR_POWER_NORMAL:
        ret = hrs_open();
        break;
    case RT_SENSOR_POWER_LOW:
        break;
    case RT_SENSOR_POWER_HIGH:
        break;
    default:
        ;
    }

    return ret;
}

static rt_size_t hrs_polling_get_data(rt_sensor_t sensor, void *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_HR)
    {
        //Always get raw data
        hr_raw_data_t raw;
        raw.hrm_raw = Hrs3300_read_hrs();
        raw.alg_raw = Hrs3300_read_als();  // 20170430
        rt_memcpy(data, &raw, sizeof(hr_raw_data_t));
        return sizeof(hr_raw_data_t);
    }

    return 0;
}

static rt_size_t hrs_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    if (!hr_dev || !buf) return 0;

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return hrs_polling_get_data(sensor, buf);
    }

    return 0;
}

static rt_err_t hrs_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    rt_kprintf("hrs_control: cmd 0x%x\n", cmd);

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
    {
        hr_sensor_info_t *info = (hr_sensor_info_t *)args;
        if (hr_dev)
            info->hr_id = hr_dev->id;
        else
            info->hr_id = 0;
        rt_kprintf("hrs_control %d\n", info->hr_id);
        break;
    }
    case RT_SENSOR_CTRL_SET_RANGE:
        if (!hr_dev) return RT_ERROR;
        result = hrs_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = hrs_hr_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = hrs_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        if (!hr_dev) return RT_ERROR;
        result = hrs_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    hrs_fetch_data,
    hrs_control
};

static int rt_hw_hrs_register(const char *name, struct rt_sensor_config *cfg)
{
    /* magnetometer/compass sensor register */
    {
        hr_sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (hr_sensor == RT_NULL)
            goto __exit;

        hr_sensor->info.type       = RT_SENSOR_CLASS_HR;
        hr_sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        hr_sensor->info.model      = "hrs3300_hr";
        hr_sensor->info.unit       = RT_SENSOR_UNIT_BPM;
        hr_sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
        hr_sensor->info.range_max  = 220;
        hr_sensor->info.range_min  = 30;
        hr_sensor->info.period_min = 1;
        hr_sensor->data_len = 0;
        hr_sensor->data_buf = NULL;

        rt_memcpy(&hr_sensor->config, cfg, sizeof(struct rt_sensor_config));
        hr_sensor->ops = &sensor_ops;

        rt_err_t result = rt_hw_sensor_register(hr_sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            rt_kprintf("device register err code: %d", result);
            goto __exit;
        }
    }

    rt_kprintf("sensor init success %s!\n", name);
    return RT_EOK;

__exit:

    if (hr_sensor != RT_NULL)
    {
        rt_free(hr_sensor);
        hr_sensor = RT_NULL;
    }

    return -RT_ERROR;

}

int hrs_register(void)
{
    //all pin will be configed in drv.
    struct rt_sensor_config cfg = {0};
    cfg.irq_pin.pin = RT_PIN_NONE; //disable pin operation of sensor_close
    int ret = rt_hw_hrs_register(HR_MODEL_NAME, &cfg);

    return ret;
}

INIT_COMPONENT_EXPORT(hrs_register);

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
