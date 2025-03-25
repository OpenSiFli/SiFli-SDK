#include "sensor_memsic_mmc56x3.h"

#ifdef RT_USING_SENSOR

#define DBG_TAG "sensor.mmc56x3"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_size_t mmc56x3_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    // mmc56x3_device_t hdev = sensor->parent.user_data;
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;

    if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        mmc56x3_data_t mag = MMC56x3_ReadData();

        data->type = RT_SENSOR_CLASS_MAG;
        data->data.mag.x = mag.x;
        data->data.mag.y = mag.y;
        data->data.mag.z = mag.z;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

rt_err_t mmc56x3_set_power(rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_NORMAL)
    {
        // mmc56x3_PowerOn();
    }
    else if (power == RT_SENSOR_POWER_DOWN)
    {
        // mmc56x3_PowerOff();
    }
    else
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t mmc56x3_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
        case RT_SENSOR_CTRL_SET_POWER:
        {
            result = mmc56x3_set_power((rt_uint32_t)args & 0xff);
            break;
        }
        case RT_SENSOR_CTRL_SELF_TEST:
        {
            result =  -RT_EINVAL;
            break;
        }
        default:
        {
            result = RT_EOK;
            break;
        }
    }

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    mmc56x3_fetch_data,
    mmc56x3_control
};

int rt_hw_mmc56x3_init(const char *name, struct rt_sensor_config *cfg)
{
    int result = -RT_ERROR;
    rt_sensor_t sensor = RT_NULL;
    MMC56x3_Init(cfg);

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (RT_NULL == sensor)
    {
        LOG_E("calloc failed");
        return -RT_ERROR;
    }

    sensor->info.type       = RT_SENSOR_CLASS_MAG;
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
    sensor->info.model      = "mmc56x3_mag";
    sensor->info.unit       = RT_SENSOR_UNIT_MGAUSS;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 60000;
    sensor->info.range_min  = -60000;
    sensor->info.period_min = 5;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }
    else
    {
        LOG_I("mag sensor init success");
        return RT_EOK;
    }
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
