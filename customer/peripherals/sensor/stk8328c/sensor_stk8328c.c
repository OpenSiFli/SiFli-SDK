/**
  ******************************************************************************
  * @file   sensor_stk8328c.c
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

#include "sensor_stk8328c.h"
#include "sensor_service.h"


#ifdef RT_USING_SENSOR

#include <rtdbg.h>


static struct stk8328c_device *stk_dev = RT_NULL;
static rt_sensor_t sensor_acce = RT_NULL;
static uint8_t stk8328c_inited = 0;


static rt_err_t _stk8328c_init(void)
{
    if (stk8328c_init() == 0)
    {
        stk_dev = rt_calloc(1, sizeof(struct stk8328c_device));
        if (stk_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }
        stk_dev->bus = (rt_device_t)stk8328c_get_bus_handle();
        stk_dev->i2c_addr = stk8328c_get_dev_addr();
        stk_dev->id = stk8328c_get_dev_id();

        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _stk8328c_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {

    }

    return RT_EOK;
}

static rt_err_t _stk8328c_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

static rt_err_t _stk8328c_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    LOG_E("_stk8328c_set_power:power %d\n", power);
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        stk8328c_close();
        rt_hw_stk8328c_deinit();
        break;
    case RT_SENSOR_POWER_NORMAL:
        stk8328c_open();
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

static rt_err_t _stk8328c_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    //LOG_I("_stk8328c_self_test with mode %d\n", mode);
    res = stk8328c_self_check();
    if (res != 0)
    {
        LOG_I("_stk8328c_self_test selt test failed with %d\n", res);
        return -RT_EIO;
    }

    return RT_EOK;
}

static int _stk8328c_read_all_fifo_data(uint8_t *buf, int len)
{
    static uint8_t sdata[6];
    uint16_t fcount[3] = {0};
    gsensors_fifo_t *f_buf = (gsensors_fifo_t *)buf;
    rt_memset(&f_buf->acce_fifo->acce_x[0], 0, sizeof(gsensors_acce_t));
    for (int i = 0; i < len; i ++)
    {

        stk8328c_read_fifo(&sdata[0], sizeof(sdata));

        f_buf->acce_fifo->acce_x[fcount[0]] = (int16_t)((sdata[1] << 8) | (sdata[0]));
        //rt_kprintf("STK8328C_FIFO_PATTERN_XLX1:0x%x 0x%x %d %d\r\n",sdata[0],sdata[1],f_buf->acce_fifo->acce_x[fcount[0]],fcount[0]);
        fcount[0] ++;


        f_buf->acce_fifo->acce_y[fcount[1]] = (int16_t)((sdata[3] << 8) | (sdata[2]));
        //rt_kprintf("STK8328C_FIFO_PATTERN_XLY1:0x%x 0x%x %d %d\r\n",sdata[2],sdata[3],f_buf->acce_fifo->acce_y[fcount[1]],fcount[1]);
        fcount[1] ++;


        f_buf->acce_fifo->acce_z[fcount[2]] = (int16_t)((sdata[5] << 8) | (sdata[4]));
        //rt_kprintf("STK8328C_FIFO_PATTERN_XLZ1:0x%x 0x%x %d %d\r\n",sdata[4],sdata[5],f_buf->acce_fifo->acce_z[fcount[2]],fcount[2]);
        fcount[2] ++;

    }

    return 0;
}

static rt_size_t _stk8328c_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    //rt_kprintf("_stk8328c_polling_get_data type %d 0x%x 0x%x\n",sensor->info.type,sensor,data);

    int32_t fifo_data_len;
    int waterm, over_run, full, empty;
    int16_t x, y, z;
    gsensors_fifo_t *buf_fifo = (gsensors_fifo_t *)data;

    fifo_data_len = stk8328c_get_fifo_count();
    //rt_kprintf("stk8328c_get_fifo_count:len %d\r\n",fifo_data_len);

    if (fifo_data_len <=  GSENSOR_FIFO_SIZE)
    {
        _stk8328c_read_all_fifo_data((uint8_t *)buf_fifo, fifo_data_len);
        buf_fifo->total_count = fifo_data_len * 3 ; // for single standard on gsensor_algo.c "algo_buf_len"
    }

    stk8328c_set_fifo_mode(STK_BYPASS_MODE_VAL);
    stk8328c_set_fifo_mode(STK_FIFO_MODE_VAL);

    return (uint32_t)buf_fifo->total_count;


}

static rt_size_t stk8328c_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);
    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _stk8328c_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t stk8328c_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
    {
        result = rt_hw_stk8328c_init();

        if (result != RT_EOK)
            *(uint8_t *)args = 0;
        else
            *(uint8_t *)args = stk_dev->id;

        break;
    }
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _stk8328c_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _stk8328c_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _stk8328c_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _stk8328c_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    stk8328c_fetch_data,
    stk8328c_control
};

int rt_hw_stk8328c_register(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    /* accelerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            return -1;

        sensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_STM;
        sensor_acce->info.model      = "stk8328c_acc";
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


    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_acce)
    {
        rt_free(sensor_acce);
        sensor_acce = RT_NULL;
    }

    return -RT_ERROR;

}

int stk8328c_sensor_register(void)
{
    int ret = 0;
    struct rt_sensor_config cfg;
    cfg.intf.dev_name = STK8328C_BUS_NAME;
    cfg.irq_pin.pin = STK8328C_INT1;    //it must be config
    ret = rt_hw_stk8328c_register(GSENSOR_MODEL_NAME, &cfg);
    return ret;
}

INIT_COMPONENT_EXPORT(stk8328c_sensor_register);

int rt_hw_stk8328c_init(void)
{
    rt_int8_t result = RT_EOK;

    if (!stk8328c_inited)
    {
        result = _stk8328c_init();
        if (result != RT_EOK)
        {
            LOG_E("stk8328c init err code: %d", result);
            if (stk_dev)
            {
                rt_free(stk_dev);
                stk_dev = RT_NULL;
            }
        }
        else
        {
            stk8328c_inited = 1;
        }
    }
    return result;
}

int rt_hw_stk8328c_deinit(void)
{
    int ret = RT_EOK;

    if (stk_dev)
    {
        rt_free(stk_dev);
        stk_dev = RT_NULL;
    }
    stk8328c_inited = 0;
    return ret;
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
