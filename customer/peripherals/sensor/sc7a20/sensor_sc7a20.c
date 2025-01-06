
#include "sensor_sc7a20.h"
#include "sc7a20.h"
#include "sc7a20_driver.h"
#include "SL_Watch_Algorithm.h"

#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.sc7a20"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


static struct sc7a20_device *sc7a20_dev;
static uint8_t sc7a20_inited = 0;


static rt_err_t _sc7a20_init(void)
{
    if (Triaxial_Sensor_Init() == 0)
    {
        sc7a20_dev = rt_calloc(1, sizeof(struct sc7a20_device));
        if (sc7a20_dev == RT_NULL)
        {
            return RT_ENOMEM;
        }

        sc7a20_dev->bus = (rt_device_t)sc7a20_get_bus();
        sc7a20_dev->i2c_addr = sc7a20_get_dev_addr();
        sc7a20_dev->id = 0;

        return RT_EOK;
    }

    return RT_ERROR;
}

static rt_err_t _sc7a20_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        return RT_EOK;
    }

    return RT_ERROR;
}



static rt_err_t _sc7a20_self_test(rt_sensor_t sensor, rt_uint8_t mode)
{
    int res;

    return RT_EOK;
}


static rt_err_t _sc7a20_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
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


static rt_err_t _sc7a20_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    switch (power)
    {
    case RT_SENSOR_POWER_DOWN:
        rt_hw_sc7a20_deinit();
        break;
    case RT_SENSOR_POWER_NORMAL:
        break;
    case RT_SENSOR_POWER_LOW:
        // can change osrs_p, osrs_t, odr ?
        break;
    case RT_SENSOR_POWER_HIGH:
        break;
    default:
        break;
    }
    return RT_EOK;
}



static rt_size_t _sc7a20_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    //int32_t x, y, z;
    uint32_t x;



    if (sensor->info.type == RT_SENSOR_CLASS_STEP)
    {
        x = Gsensor_Read_Data();
        data->type = RT_SENSOR_CLASS_STEP;
        data->data.step = x;
        data->timestamp = rt_sensor_get_ts();
#if 0
        SL_SC7A20_Watch_Algo_Exe(0);                                        //保证算法执行到位
        SL_SC7A20_GET_DATA(sc7a20_xyz[0], sc7a20_xyz[1], sc7a20_xyz[2]);    //获取x y z坐标倿
        data->type = RT_SENSOR_CLASS_ACCE;
        data->data.acce.x = (int32_t)sc7a20_xyz[0];//(int32_t)x;
        data->data.acce.y = (int32_t)sc7a20_xyz[1];//(int32_t)y;
        data->data.acce.z = (int32_t)sc7a20_xyz[2];//(int32_t)z;
        data->timestamp = rt_sensor_get_ts();
#endif

    }

    return 1;
}

static rt_size_t sc7a20_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _sc7a20_polling_get_data(sensor, buf);
    }
    else
        return 0;
}


static rt_err_t sc7a20_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
    {
        result = rt_hw_sc7a20_init();

        if (result != RT_EOK)
            *(uint8_t *)args = 0;
        else
            *(uint8_t *)args = sc7a20_dev->id;

        break;
    }
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _sc7a20_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _sc7a20_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _sc7a20_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = _sc7a20_self_test(sensor, *((rt_uint8_t *)args));
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}


static struct rt_sensor_ops sensor_ops =
{
    sc7a20_fetch_data,
    sc7a20_control
};

int rt_hw_sc7a20_register(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_step = RT_NULL;
    /* sensor_gyro sensor register */
    {
        sensor_step = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_step == RT_NULL)
            goto __exit;

        sensor_step->info.type       = RT_SENSOR_CLASS_STEP;
        sensor_step->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_step->info.model      = "sc7a20_step";
        sensor_step->info.unit       = RT_SENSOR_UNIT_ONE;
        sensor_step->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_step->info.range_max  = 200000;
        sensor_step->info.range_min  = 1;
        sensor_step->info.period_min = 0;

        rt_memcpy(&sensor_step->config, cfg, sizeof(struct rt_sensor_config));
        sensor_step->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_step, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_step)
        rt_free(sensor_step);

}

int sc7a20_sensor_register(void)
{
    int ret = 0;
    struct rt_sensor_config cfg;
    cfg.intf.dev_name = SC7A20_I2C_BUS;
    cfg.irq_pin.pin = SC7A20_INT_PIN; //it must be config
    ret = rt_hw_sc7a20_register(GSENSOR_MODEL_NAME, &cfg);
    return ret;
}

INIT_COMPONENT_EXPORT(sc7a20_sensor_register);

int rt_hw_sc7a20_init(void)
{
    rt_int8_t result = RT_EOK;
    if (!sc7a20_inited)
    {
        rt_kprintf("rt_hw_sc7a20_init\r\n");

        result = _sc7a20_init();
        if (result != RT_EOK)
        {
            rt_kprintf("sc7a20 init err code: %d", result);
            if (sc7a20_dev)
            {
                rt_free(sc7a20_dev);
                sc7a20_dev = RT_NULL;
            }
        }
        else
        {
            sc7a20_inited = 1；
        }
    }
    return result;
}

int rt_hw_sc7a20_deinit(void)
{
    int ret = RT_EOK;

    if (sc7a20_dev)
    {
        rt_free(sc7a20_dev);
        sc7a20_dev = RT_NULL;
    }
    sc7a20_inited = 0;
    return ret;
}


static void read_step_entry(void *parameter)
{
    rt_device_t step_dev = RT_NULL;
    struct rt_sensor_data temp_data;
    rt_size_t res = 0;

    step_dev = rt_device_find("step_sc7");

    if (step_dev == RT_NULL)
    {
        rt_kprintf("not found step_sc7a20 device\r\n");
        return;
    }

    if (rt_device_open(step_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open step_sc7a20 failed\r\n");
        return;
    }


    Triaxial_Sensor_Enable();
    //extern    void Triaxial_Sensor_Disable(void);
    //Triaxial_Sensor_Disable(); //cgdeng test for low power
    while (1)
    {

        res = rt_device_read(step_dev, 0, &temp_data, 1); //cgdeng test forlow power

        if (res == 0)
        {
            rt_kprintf("read data failed! result\n");
        }
        else
        {
            rt_kprintf("temp[%d],\r\n", temp_data.data.step);
        }

        rt_thread_delay(5000);
    }
}

static int step_read_sample(void)
{
    rt_thread_t temp_thread;

    temp_thread = rt_thread_create("step_r",
                                   read_step_entry,
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
INIT_APP_EXPORT(step_read_sample);

static int rt_hw_sc7a20_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = "SC7A20_I2C_BUS";//"i2c4";      /* i2c bus */
    cfg.intf.user_data = (void *)0x19;  /* i2c slave addr */
    rt_hw_sc7a20_init("sc7a20", &cfg);/* sc7a20 */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_sc7a20_port);


#endif   // RT_USING_SENSOR
