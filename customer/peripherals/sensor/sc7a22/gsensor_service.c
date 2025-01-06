/**
  ******************************************************************************
  * @file   gsensor_service.c
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

#ifdef RT_USING_SENSOR
#include "gsensor_service.h"
#include "gsensor_algo.h"

static struct sc7a22_device *gsensor_dev = RT_NULL;
static rt_sensor_t gsensor_acce = RT_NULL;

static rt_err_t gsensors_open(void)
{
    rt_err_t ret = RT_EOK;

    if (!gsensor_dev)
    {
        if (sc7a22_init() == 0)
        {
            gsensor_dev = rt_calloc(1, sizeof(struct sc7a22_device));
            if (gsensor_dev == RT_NULL)
            {
                ret = RT_ENOMEM;
                goto err;
            }
            gsensor_dev->bus = (rt_device_t)sc7a22_get_bus_handle();
            gsensor_dev->i2c_addr = sc7a22_get_dev_addr();
            gsensor_dev->id = sc7a22_get_dev_id();
            sc7a22_open();
            return RT_EOK;
        }

        ret = RT_ERROR;
err:
        rt_kprintf("gsensors_open init err! code: %d\n", ret);
    }

    return ret;

}

static rt_err_t gsensors_close(void)
{
    rt_err_t ret = RT_EOK;
    sc7a22_close();
    if (gsensor_dev)
    {
        rt_free(gsensor_dev);
        gsensor_dev = RT_NULL;
    }
    return ret;
}


static rt_err_t gseneors_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        uint8_t range_ctr;
#if 0
        if (range <= 2000)
            range_ctr = SC7A22_2g;
        else if (range <= 4000)
            range_ctr = SC7A22_4g;
        else if (range <= 8000)
            range_ctr = SC7A22_8g;
        else
            range_ctr = SC7A22_16g;

        rt_kprintf("acce set range %d\n", range_ctr);

        //sc7a22_accel_set_range(range_ctr);
#endif
    }

    return RT_EOK;
}

static rt_err_t gsensors_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

static rt_err_t gsensors_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    rt_err_t ret = RT_EOK;

    rt_kprintf("gsensors_set_power: power %d\n", power);

    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        gsensors_close();
        break;
    case RT_SENSOR_POWER_NORMAL:
        ret = gsensors_open();
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

static rt_err_t gsensors_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    //LOG_I("_sc7a22_self_test with mode %d\n", mode);
    rt_err_t ret = sc7a22_self_check();
    if (ret != 0)
    {
        rt_kprintf("gsensors_self_test selt test failed with %d\n", ret);
        return -RT_EIO;
    }

    return RT_EOK;
}

#if defined (GSENSOR_UES_FIFO)
static int gsensors_read_all_fifo_data(void *buf, int len)
{
    static uint8_t sdata[7]; //sc7a20 read 7 bytes, 1st byte nouse, 2~7 bytes is 3 axis x y z data.
    gsensors_fifo_t *fifo = (gsensors_fifo_t *)buf;

    fifo->num = len;

    for (int i = 0; i < len; i ++)
    {
        // note: sc7a22 need to tunning this fun, it's diff with sc7a20, so don't open it to use before tunning.
        //sc7a22_read_fifo(&data[0], sizeof(data));
        sc7a20_read_fifo(&sdata[0], sizeof(sdata));
        fifo->buf[i].acce_data[0]  = (int16_t)((sdata[2] << 8) | (sdata[1])) * GSENSOR_ACCE_PARA;
        fifo->buf[i].acce_data[1]  = (int16_t)((sdata[4] << 8) | (sdata[3])) * GSENSOR_ACCE_PARA;
        fifo->buf[i].acce_data[2]  = (int16_t)((sdata[6] << 8) | (sdata[5])) * GSENSOR_ACCE_PARA;
    }

    return 0;
}
#endif

static rt_size_t gsensors_polling_get_data(rt_sensor_t sensor, void *data)
{
    rt_size_t fifo_len = sc7a22_get_fifo_count();

    //abnormal process, to avoid buf overflow
    if (fifo_len > SENSORS_ALGO_BUF_ARRAY)
    {
        rt_kprintf("fifo_len %d > %d !!!\n", fifo_len, SENSORS_ALGO_BUF_ARRAY);
        fifo_len = SENSORS_ALGO_BUF_ARRAY;
    }

    gsensors_read_all_fifo_data(data, fifo_len);

    sc7a22_set_fifo_mode(SC7A22_FIFO_BYPASS_VAL);
    sc7a22_set_fifo_mode(SC7A22_FIFO_MODE_VAL);

    return fifo_len;
}

static rt_size_t gsensors_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    if (!gsensor_dev || !buf)
    {
        ((gsensors_fifo_t *)buf)->num = 0;
        return 0;
    }

    // if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    return gsensors_polling_get_data(sensor, buf);

    return 0;
}

static rt_err_t gsensors_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t ret = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        if (!gsensor_dev) return RT_ERROR;
        *(uint8_t *)args = gsensor_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        if (!gsensor_dev) return RT_ERROR;
        ret = gseneors_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        ret = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        ret = gsensors_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        ret = gsensors_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        if (!gsensor_dev) return RT_ERROR;
        ret = gsensors_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return ret;
}

static struct rt_sensor_ops sensor_ops =
{
    gsensors_fetch_data,
    gsensors_control
};


static int rt_hw_gsensors_register(const char *name, struct rt_sensor_config *cfg)
{
    int ret;
    /* accelerometer sensor register */
    {
        gsensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (gsensor_acce == RT_NULL)
            return -1;

        gsensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        gsensor_acce->info.vendor     = RT_SENSOR_VENDOR_STM;
        gsensor_acce->info.model      = "sc7a22_acc";
        gsensor_acce->info.unit       = RT_SENSOR_UNIT_MG;
#if (SC7A22_USING_I2C == 1)
        gsensor_acce->info.intf_type  = RT_SENSOR_INTF_I2C;
#else
        gsensor_acce->info.intf_type  = RT_SENSOR_INTF_SPI;
#endif
        gsensor_acce->info.range_max  = 16000;
        gsensor_acce->info.range_min  = 2000;
        gsensor_acce->info.period_min = 5;

        rt_memcpy(&gsensor_acce->config, cfg, sizeof(struct rt_sensor_config));
        gsensor_acce->ops = &sensor_ops;

        ret = rt_hw_sensor_register(gsensor_acce, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (ret != RT_EOK)
        {
            rt_kprintf("device register err code: %d\n", ret);
            goto __exit;
        }
    }


    rt_kprintf("sensor register success!\n");
    return RT_EOK;

__exit:
    if (gsensor_acce)
    {
        rt_free(gsensor_acce);
        gsensor_acce = RT_NULL;
    }

    return -RT_ERROR;
}

int gsensors_register(void)
{
    //all pin will be configed in drv.
    struct rt_sensor_config cfg = {0};
    cfg.irq_pin.pin = RT_PIN_NONE; //disable pin operation of sensor_close
    int ret = rt_hw_gsensors_register(GSENSOR_MODEL_NAME, &cfg);
    return ret;
}

INIT_COMPONENT_EXPORT(gsensors_register);

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
