
#include "sensor_sc7r30.h"
#include "sc7r30.h"
#include "sc7r30_driver.h"


#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.sc7r30"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


static struct sc7r30_device *sc7r30_dev;

static rt_err_t _sc7r30_init(void)
{
    if (HeartRate_Init() == 0)
    {
        sc7r30_dev = rt_calloc(1, sizeof(struct sc7r30_device));
        if (sc7r30_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }

        sc7r30_dev->bus = (rt_device_t)sc7r30_get_bus();
        sc7r30_dev->i2c_addr = sc7r30_get_dev_addr();
        sc7r30_dev->id = 0;//SC7A20_CHIP_ID;//sc7r30_get_id();

        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _sc7r30_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    return RT_EOK;
}



static rt_err_t _sc7r30_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    return RT_EOK;
}

static rt_err_t _sc7r30_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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

static rt_err_t _sc7r30_set_power(rt_sensor_t sensor, rt_uint8_t power)
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

static rt_size_t _sc7r30_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_HR)
    {
        data->type = RT_SENSOR_CLASS_HR;
        data->data.hr = HeartRat_ReadData();
        //data->data.hr = rand()%100+100; // for test nq , manual set hr value only for test
        data->timestamp = rt_sensor_get_ts();

    }

    return RT_EOK;
}

static rt_size_t sc7r30_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _sc7r30_polling_get_data(sensor, buf);
    }
    else
    {
        return 0;
    }
}


static rt_err_t sc7r30_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = sc7r30_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _sc7r30_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _sc7r30_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _sc7r30_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _sc7r30_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}


static struct rt_sensor_ops sensor_ops =
{
    sc7r30_fetch_data,
    sc7r30_control
};


int rt_hw_sc7r30_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_hr = RT_NULL;

    result = _sc7r30_init();
    if (result != RT_EOK)
    {
        rt_kprintf("sc7r30 init err code: %d\n", result);
        goto __exit;
    }

    /* sensor_gyro sensor register */
    {
        sensor_hr = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_hr == RT_NULL)
            goto __exit;

        sensor_hr->info.type       = RT_SENSOR_CLASS_HR;
        sensor_hr->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_hr->info.model      = "sc7r30_hr";
        sensor_hr->info.unit       = RT_SENSOR_UNIT_BPM;
        sensor_hr->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_hr->info.range_max  = 142;
        sensor_hr->info.range_min  = 50;
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
    if (sc7r30_dev)
        rt_free(sc7r30_dev);
    return -RT_ERROR;
}




static void read_hr_entry(void *parameter)
{
    rt_device_t hr_dev = RT_NULL;
    struct rt_sensor_data temp_data;
    rt_size_t res = 0;

    hr_dev = rt_device_find("hr_sc7r3");

    if (hr_dev == RT_NULL)
    {
        rt_kprintf("not found hr_sc7r30 device\r\n");
        return;
    }

    if (rt_device_open(hr_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("open hr_sc7r30 failed\r\n");
        return;
    }



    while (1)
    {

        res = rt_device_read(hr_dev, 0, &temp_data, 1);

        if (res == 0)
        {
            rt_kprintf("read data failed! result\n");
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



static int rt_hw_sc7r30_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = "SC7R30_I2C_BUS";//"i2c2";      /* i2c bus */
    cfg.intf.user_data = (void *)0x38;  /* i2c slave addr */
    cfg.irq_pin.pin = RT_PIN_NONE;  //TODO:使用基于pin IRQ
    rt_hw_sc7r30_init("sc7r30", &cfg);/* sc7r30 */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_sc7r30_port);



#endif   // RT_USING_SENSOR
