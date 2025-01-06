/**
  ******************************************************************************
  * @file   sensor_hr_pah8007.c
  * @author wk software development team
  ******************************************************************************
*/


#include "sensor_hr_pah8007.h"
#include "slw_pah8007.h"
#include "pah_8007.h"
#include "ulog.h"
#include <rtthread.h>


#ifdef RT_USING_SENSOR

//#define DBG_TAG "sensor.pah8007"
//#define DBG_LVL DBG_INFO
//#include <rtdbg.h>


static struct pah8007_device *pah8007_dev;

static rt_err_t _pah8007_init(void)
{
    if (pah8007_init() == 0)
    {
        pah8007_dev = rt_calloc(1, sizeof(struct pah8007_device));
        if (pah8007_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }

        pah8007_dev->bus = (rt_device_t)pah8007_get_bus();
        pah8007_dev->i2c_addr = pah8007_get_dev_addr();
        pah8007_dev->id = 0;

        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _pah8007_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    return RT_EOK;
}



static rt_err_t _pah8007_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    return RT_EOK;
}

static rt_err_t _pah8007_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

static rt_err_t _pah8007_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    rt_err_t ret = RT_EOK;
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
    case RT_SENSOR_POWER_NORMAL:
    case RT_SENSOR_POWER_LOW:
    // can change osrs_p, osrs_t, odr ?
    case RT_SENSOR_POWER_HIGH:
    default:
        ret = RT_EOK;
        break;
    }
    return ret;
}

static rt_size_t _pah8007_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_HR)
    {
        data->type = RT_SENSOR_CLASS_HR;
        //data->data.hr = rand()%100+100; // for test nq , manual set hr value only for test
        ppg_sensor_task_polling();
        data->data.hr = pah8007_alg_task();
        data->timestamp = rt_sensor_get_ts();

    }

    return 1;
}

static rt_size_t pah8007_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _pah8007_polling_get_data(sensor, buf);
    }
    else
    {
        return 0;
    }
}


static rt_err_t pah8007_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = pah8007_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _pah8007_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _pah8007_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _pah8007_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _pah8007_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}


static struct rt_sensor_ops sensor_ops =
{
    pah8007_fetch_data,
    pah8007_control
};


int rt_hw_pah8007_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_hr = RT_NULL;

    rt_kprintf("rt_hw_pah8007_init\r\n");

    result = _pah8007_init();
    if (result != RT_EOK)
    {
        rt_kprintf("pah8007 init err code: %d\n", result);
        goto __exit;
    }

    /* sensor_gyro sensor register */
    {
        sensor_hr = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_hr == RT_NULL)
            goto __exit;

        sensor_hr->info.type       = RT_SENSOR_CLASS_HR;
        sensor_hr->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_hr->info.model      = "pah8007_hr";
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
    if (pah8007_dev)
        rt_free(pah8007_dev);
    return -RT_ERROR;
}




static void read_hr_entry(void *parameter)
{
    rt_device_t hr_dev = RT_NULL;
    struct rt_sensor_data temp_data;
    rt_size_t res = 0;

    rt_kprintf("read_hr_entry\r\n");

    hr_dev = rt_device_find("hr_pah80");

    if (hr_dev == RT_NULL)
    {
        rt_kprintf("not found hr_pah8007 device\r\n");
        return;
    }

    if (rt_device_open(hr_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open hr_pah8007 failed\r\n");
        return;
    }

    ppg_sensor_start(AUTO_MODE);
    //extern void pah_sensor_stop(void);
    //pah_sensor_stop();  //cgdeng for test low power

    while (1)
    {

        res = rt_device_read(hr_dev, 0, &temp_data, 1);  //cgdeng for test low power

        if (res == 0)
        {
            rt_kprintf("read hr data failed! result\r\n");
        }
        else
        {
            rt_kprintf("hr=[%d]\r\n", temp_data.data.hr);
        }

        rt_thread_delay(2000);
    }
}

static int hr_read_sample(void)
{
    rt_thread_t temp_thread;

    temp_thread = rt_thread_create("hr_r",
                                   read_hr_entry,
                                   RT_NULL,
                                   1024,
                                   RT_THREAD_PRIORITY_MAX / 2,
                                   20);
    if (temp_thread != RT_NULL)
    {
        rt_thread_startup(temp_thread);
    }

    return RT_EOK;
}
INIT_APP_EXPORT(hr_read_sample);


static int rt_hw_pah8007_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = "PAH8007_I2C_BUS";
    cfg.intf.user_data = (void *)0x48;  /* i2c slave addr */
    cfg.irq_pin.pin = RT_PIN_NONE;  //TODO:????pin IRQ
    rt_hw_pah8007_init("pah8007", &cfg);/* pah8007 */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_pah8007_port);



#endif   // RT_USING_SENSOR
