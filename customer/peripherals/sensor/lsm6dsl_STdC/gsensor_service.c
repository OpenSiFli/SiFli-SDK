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
#include "lsm6dsl.h"
#include "gsensor_algo.h"

static struct lsm6dsl_device *gsensor_dev = RT_NULL;
static rt_sensor_t gsensor_acce = RT_NULL;
static rt_tick_t curtick = 0, pretick = 0;

static rt_err_t gsensors_open(void)
{
    rt_err_t ret = RT_EOK;

    if (!gsensor_dev)
    {
        if (lsm6dsl_init() == 0)
        {
            gsensor_dev = rt_calloc(1, sizeof(struct lsm6dsl_device));
            if (gsensor_dev == RT_NULL)
            {
                ret = RT_ENOMEM;
                goto err;
            }
            gsensor_dev->bus = (rt_device_t)lsm6dsl_get_bus_handle();
            gsensor_dev->i2c_addr = lsm6dsl_get_dev_addr();
            gsensor_dev->id = lsm6dsl_get_dev_id();
            lsm6dsl_open();
            curtick = rt_tick_get_millisecond();
            pretick = curtick;
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
    lsm6dsl_close();
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

        if (range <= 2000)
            range_ctr = LSM6DSL_2g;
        else if (range <= 4000)
            range_ctr = LSM6DSL_4g;
        else if (range <= 8000)
            range_ctr = LSM6DSL_8g;
        else
            range_ctr = LSM6DSL_16g;

        rt_kprintf("acce set range %d\n", range_ctr);

        lsm6dsl_accel_set_range(range_ctr);
        gsensor_dev->config.accel_range = range_ctr;
    }
#ifdef USING_GYRO_SENSOR
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        uint8_t range_ctr;

        if (range <= 125000UL)
            range_ctr = LSM6DSL_125dps;
        else if (range <= 250000UL)
            range_ctr = LSM6DSL_250dps;
        else if (range <= 500000UL)
            range_ctr = LSM6DSL_500dps;
        else if (range <= 1000000UL)
            range_ctr = LSM6DSL_1000dps;
        else
            range_ctr = LSM6DSL_2000dps;

        rt_kprintf("gyro set range %d\n", range);

        lsm6dsl_gyro_set_range(range_ctr);
        gsensor_dev->config.gyro_range = range_ctr;
    }
#endif
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

    rt_kprintf("gsensors_set_power:power %d\n", power);

    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        gsensors_close();
        break;
    case RT_SENSOR_POWER_NORMAL:
        ret = gsensors_open();
        break;
    case RT_SENSOR_POWER_LOW:
        // enable fifo mode?
        //lsm6dsl_fifo_enable(1, 1, 2);
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
    //LOG_I("gsensors_self_test with mode %d\n", mode);
    rt_err_t ret = lsm6dsl_self_check();
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
    uint8_t data[2];
    int pattern_id;
    uint16_t fcount[6] = {0}, *p;
    int i = 0, j = 0;
    gsensors_fifo_t *fifo = (gsensors_fifo_t *)buf;
    fifo->num = len / GSENSOR_AXIS;

    //abnormal process, to avoid buf overflow
    if (fifo->num > SENSORS_ALGO_BUF_ARRAY)
    {
        //rt_kprintf("fifo->num %d > %d !!!\n", fifo->num, SENSORS_ALGO_BUF_ARRAY);
        fifo->num = SENSORS_ALGO_BUF_ARRAY;
    }

    len = fifo->num * GSENSOR_AXIS;


    for (i = 0; i < len; i ++)
    {
        j  = i / GSENSOR_AXIS;
        pattern_id = lsm6dsl_get_fifo_pattern();
        //rt_kprintf("lsm6dsl_get_fifo_pattern: ret %d\r\n",pattern_id);

        lsm6dsl_read_fifo(&data[0], sizeof(data));

        float v = ((int16_t)((data[1] << 8) | (data[0]))) * GSENSOR_ACCE_PARA;
        switch (pattern_id)
        {
#ifdef USING_GYRO_SENSOR
        case LSM6DSL_FIFO_PATTERN_GX1:
            fifo->buf[j].gyro_data[0] = v;
            break;
        case LSM6DSL_FIFO_PATTERN_GY2:
            fifo->buf[j].gyro_data[1] = v;
            break;
        case LSM6DSL_FIFO_PATTERN_GZ3:
            fifo->buf[j].gyro_data[2] = v;
            break;
#endif
        case LSM6DSL_FIFO_PATTERN_XLX1:
            fifo->buf[j].acce_data[0] = v;
            break;
        case LSM6DSL_FIFO_PATTERN_XLY2:
            fifo->buf[j].acce_data[1] = v;
            break;
        case LSM6DSL_FIFO_PATTERN_XLZ3:
            fifo->buf[j].acce_data[2] = v;
            break;
        default:
            RT_ASSERT(0);
        }
        //rt_kprintf("buf[%d] pattern[%d] = %f ;\n", j, pattern_id, v);
    }

    return 0;
}
#endif

static rt_size_t gsensors_polling_get_data(rt_sensor_t sensor, void *data)
{

    //rt_kprintf("gsensors_polling_get_data type %d 0x%x 0x%x\n",sensor->info.type,sensor,data);
#if defined (GSENSOR_UES_FIFO)
    //int waterm, over_run, full, empty;
    //int16_t x, y, z;

    //waterm = lsm6dsl_get_waterm_status();
    //over_run = lsm6dsl_get_overrun_status();
    //full = lsm6dsl_get_fifo_full_status();
    //empty = lsm6dsl_get_fifo_empty_status();
    //rt_kprintf("lsm6dsl_get_waterm_status:waterm %d %d %d %d\r\n",waterm,over_run,full,empty);
    //rt_kprintf("gsensor_algo_scheduler: tick %d\n", rt_tick_get_millisecond());
    curtick = rt_tick_get_millisecond();
    rt_size_t fifo_len = lsm6dsl_get_fifo_count();
    //rt_kprintf("lsm6dsl_read_fifo:len %d\r\n",fifo_len);

    //RT_ASSERT(fifo_len <= GSENSOR_FIFO_SIZE * GSENSOR_AXIS);
    if (fifo_len > GSENSOR_FIFO_SIZE * GSENSOR_AXIS)
    {
        rt_kprintf("gsensor data interval erro, interval= %d(should be 200); data len erro, len= %d (should be < 52) ;\r\n", curtick - pretick, fifo_len);
        fifo_len = GSENSOR_FIFO_SIZE * GSENSOR_AXIS;
    }

    gsensors_read_all_fifo_data(data, fifo_len);

    lsm6dsl_set_fifo_mode(LSM6DSL_BYPASS_MODE);
    lsm6dsl_set_fifo_mode(LSM6DSL_FIFO_MODE);
    pretick = curtick;
    return fifo_len;
#else
    //Considering low power consumption, only FIFO mode is supported
    return 0;
#endif
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
        gsensor_acce->info.model      = "lsm6dsl_acc";
        gsensor_acce->info.unit       = RT_SENSOR_UNIT_MG;
#if (LSM6DSL_USING_I2C == 1)
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

#if 0
    /* gyroscope sensor register */
    {
        sensor_gyro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_gyro == RT_NULL)
            goto __exit;

        sensor_gyro->info.type       = RT_SENSOR_CLASS_GYRO;
        sensor_gyro->info.vendor     = RT_SENSOR_VENDOR_STM;
        sensor_gyro->info.model      = "lsm6dsl_gyro";
        sensor_gyro->info.unit       = RT_SENSOR_UNIT_MDPS;
        sensor_gyro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_gyro->info.range_max  = 2000000;
        sensor_gyro->info.range_min  = 250000;
        sensor_gyro->info.period_min = 5;

        rt_memcpy(&sensor_gyro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_gyro->ops = &sensor_ops;

        ret = rt_hw_sensor_register(sensor_gyro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (ret != RT_EOK)
        {
            LOG_E("device register err code: %d", ret);
            goto __exit;
        }
    }
    /* step sensor register */
    {
        sensor_step = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_step == RT_NULL)
            goto __exit;

        sensor_step->info.type       = RT_SENSOR_CLASS_STEP;
        sensor_step->info.vendor     = RT_SENSOR_VENDOR_STM;
        sensor_step->info.model      = "lsm6dsl_step";
        sensor_step->info.unit       = RT_SENSOR_UNIT_ONE;
        sensor_step->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_step->info.range_max  = 200000;
        sensor_step->info.range_min  = 1;
        sensor_step->info.period_min = 0;

        rt_memcpy(&sensor_step->config, cfg, sizeof(struct rt_sensor_config));
        sensor_step->ops = &sensor_ops;

        ret = rt_hw_sensor_register(sensor_step, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (ret != RT_EOK)
        {
            LOG_E("device register err code: %d", ret);
            goto __exit;
        }
    }
#endif
    rt_kprintf("sensor register success!\n");
    return RT_EOK;

__exit:
    if (gsensor_acce)
    {
        rt_free(gsensor_acce);
        gsensor_acce = RT_NULL;
    }
#if 0
    if (sensor_gyro)
        rt_free(sensor_gyro);
    if (sensor_step)
        rt_free(sensor_step);
#endif
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
