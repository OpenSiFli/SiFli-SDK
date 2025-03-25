
#include "st_lsm6dsl_sensor_v1.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.st.lsm6dsl"
#define DBG_COLOR
#include <rtdbg.h>

#define SENSOR_ACC_RANGE_2G   2000
#define SENSOR_ACC_RANGE_4G   4000
#define SENSOR_ACC_RANGE_8G   8000
#define SENSOR_ACC_RANGE_16G  16000

static LSM6DSL_Object_t lsm6dsl;
static struct rt_i2c_bus_device *i2c_bus_dev;

static int32_t rt_func_ok(void)
{
    return 0;
}

static int32_t get_tick(void)
{
    return rt_tick_get();
}

static int32_t rt_i2c_write_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    // rt_uint8_t tmp = reg;
    // struct rt_i2c_msg msgs[2];

    // msgs[0].addr  = addr;             /* Slave address */
    // msgs[0].flags = RT_I2C_WR;        /* Write flag */
    // msgs[0].buf   = &tmp;             /* Slave register address */
    // msgs[0].len   = 1;                /* Number of bytes sent */

    // msgs[1].addr  = addr;             /* Slave address */
    // msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    // msgs[1].buf   = data;             /* Read data pointer */
    // msgs[1].len   = len;              /* Number of bytes read */

    // if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    // {
    //     return -RT_ERROR;
    // }
    rt_i2c_mem_write(i2c_bus_dev, addr, reg, 8, data, len);

    return RT_EOK;
}

static int32_t rt_i2c_read_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _lsm6dsl_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t  id, i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;
    LSM6DSL_IO_t io_ctx;

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    /* Configure the accelero driver */
    io_ctx.BusType     = LSM6DSL_I2C_BUS; /* I2C */
    io_ctx.Address     = i2c_addr;
    io_ctx.Init        = rt_func_ok;
    io_ctx.DeInit      = rt_func_ok;
    io_ctx.ReadReg     = rt_i2c_read_reg;
    io_ctx.WriteReg    = rt_i2c_write_reg;
    io_ctx.GetTick     = get_tick;

    if (LSM6DSL_RegisterBusIO(&lsm6dsl, &io_ctx) != LSM6DSL_OK)
    {
        return -RT_ERROR;
    }
    else if (LSM6DSL_ReadID(&lsm6dsl, &id) != LSM6DSL_OK)
    {
        LOG_E("read id failed");
        return -RT_ERROR;
    }
    if (LSM6DSL_Init(&lsm6dsl) != LSM6DSL_OK)
    {
        LOG_E("acc init failed");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _lsm6dsl_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        LSM6DSL_ACC_SetFullScale(&lsm6dsl, range / 1000);
        LOG_D("acce set range %d", range);
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        LSM6DSL_GYRO_SetFullScale(&lsm6dsl, range);
        LOG_D("gyro set range %d", range);
    }

    return RT_EOK;
}

static rt_err_t _lsm6dsl_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        LSM6DSL_ACC_SetOutputDataRate(&lsm6dsl, odr);
        LOG_D("acce set odr %d", odr);
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        LSM6DSL_GYRO_SetOutputDataRate(&lsm6dsl, odr);
        LOG_D("gyro set odr %d", odr);
    }

    return RT_EOK;
}

static rt_err_t _lsm6dsl_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    if (mode == RT_SENSOR_MODE_POLLING)
    {
        LOG_D("set mode to POLLING");
    }
    else if (mode == RT_SENSOR_MODE_INT)
    {
        LOG_D("set mode to RT_SENSOR_MODE_INT");
    }
    else if (mode == RT_SENSOR_MODE_FIFO)
    {
        LSM6DSL_FIFO_Set_Mode(&lsm6dsl, LSM6DSL_FIFO_MODE);

        LSM6DSL_FIFO_Set_INT1_FIFO_Full(&lsm6dsl, 1);

        LOG_D("set mode to RT_SENSOR_MODE_FIFO");
    }
    else
    {
        LOG_D("Unsupported mode, code is %d", mode);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _lsm6dsl_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_DOWN)
    {
        if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
        {
            LSM6DSL_ACC_Disable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
        {
            LSM6DSL_GYRO_Disable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_STEP)
        {
            LSM6DSL_ACC_Disable(&lsm6dsl);
            LSM6DSL_ACC_Disable_Pedometer(&lsm6dsl);
        }
        LOG_D("set power down");
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        lsm6dsl_xl_power_mode_set(&lsm6dsl.Ctx, LSM6DSL_XL_NORMAL);
        if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
        {
            LSM6DSL_ACC_Enable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
        {
            LSM6DSL_GYRO_Enable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_STEP)
        {
            LSM6DSL_ACC_Enable(&lsm6dsl);
            LSM6DSL_ACC_Enable_Pedometer(&lsm6dsl);
        }

        LOG_D("set power normal");
    }
    else if (power == RT_SENSOR_POWER_HIGH)
    {
        lsm6dsl_xl_power_mode_set(&lsm6dsl.Ctx, LSM6DSL_XL_HIGH_PERFORMANCE);

        if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
        {
            LSM6DSL_ACC_Enable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
        {
            LSM6DSL_GYRO_Enable(&lsm6dsl);
        }

        LOG_D("set power high");
    }
    else
    {
        LOG_W("Unsupported mode, code is %d", power);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_size_t _lsm6dsl_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        LSM6DSL_Axes_t acce;

        int ret = LSM6DSL_ACC_GetAxes(&lsm6dsl, &acce);

        data->type = RT_SENSOR_CLASS_ACCE;
        data->data.acce.x = acce.x;
        data->data.acce.y = acce.y;
        data->data.acce.z = acce.z;
        data->timestamp = rt_sensor_get_ts();

    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        LSM6DSL_Axes_t gyro;

        LSM6DSL_GYRO_GetAxes(&lsm6dsl, &gyro);

        data->type = RT_SENSOR_CLASS_GYRO;
        data->data.gyro.x = gyro.x;
        data->data.gyro.y = gyro.y;
        data->data.gyro.z = gyro.z;
        data->timestamp = rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_STEP)
    {
        uint16_t step;

        LSM6DSL_ACC_Get_Step_Count(&lsm6dsl, &step);

        data->type = RT_SENSOR_CLASS_STEP;
        data->data.step = step;
        data->timestamp = rt_sensor_get_ts();
    }
    return 1;
}

static rt_size_t _lsm6dsl_acc_fifo_get_data(rt_sensor_t sensor, struct rt_sensor_data *data, rt_size_t len)
{
    LSM6DSL_Axes_t acce;
    rt_uint8_t i;

    for (i = 0; i < len; i++)
    {
        if (LSM6DSL_ACC_GetAxes(&lsm6dsl, &acce) == 0)
        {
            data[i].type = RT_SENSOR_CLASS_ACCE;
            data[i].data.acce.x = acce.x;
            data[i].data.acce.y = acce.y;
            data[i].data.acce.z = acce.z;
            data[i].timestamp = rt_sensor_get_ts();
        }
        else
            break;
    }
    return i;
}

static RT_SIZE_TYPE lsm6dsl_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);
    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _lsm6dsl_polling_get_data(sensor, buf);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_INT)
    {
        return 0;
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_FIFO)
    {
        return _lsm6dsl_acc_fifo_get_data(sensor, buf, len);
    }
    else
        return 0;
}

static rt_err_t lsm6dsl_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        LSM6DSL_ReadID(&lsm6dsl, args);
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _lsm6dsl_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = _lsm6dsl_set_odr(sensor, (rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _lsm6dsl_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _lsm6dsl_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    lsm6dsl_fetch_data,
    lsm6dsl_control
};

int rt_hw_lsm6dsl_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_acce = RT_NULL, sensor_gyro = RT_NULL, sensor_step = RT_NULL;

#ifdef PKG_USING_LSM6DSL_ACCE
    /* accelerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            return -1;

        sensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_STM;
        sensor_acce->info.model      = "lsm6dsl_acc";
        sensor_acce->info.unit       = RT_SENSOR_UNIT_MGAUSS;
        sensor_acce->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_acce->info.range_max  = SENSOR_ACC_RANGE_16G;
        sensor_acce->info.range_min  = SENSOR_ACC_RANGE_2G;
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
#endif
#ifdef PKG_USING_LSM6DSL_GYRO
    /* gyroscope sensor register */
    {
        sensor_gyro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_gyro == RT_NULL)
            goto __exit;

        sensor_gyro->info.type       = RT_SENSOR_CLASS_GYRO;
        sensor_gyro->info.vendor     = RT_SENSOR_VENDOR_STM;
        sensor_gyro->info.model      = "lsm6dsl_gyro";
        sensor_gyro->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_gyro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_gyro->info.range_max  = SENSOR_ACC_RANGE_16G;
        sensor_gyro->info.range_min  = SENSOR_ACC_RANGE_2G;
        sensor_gyro->info.period_min = 5;

        rt_memcpy(&sensor_gyro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_gyro->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_gyro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
#endif
#ifdef PKG_USING_LSM6DSL_STEP
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
        sensor_step->info.period_min = 5;

        rt_memcpy(&sensor_step->config, cfg, sizeof(struct rt_sensor_config));
        sensor_step->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_step, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
#endif
    result = _lsm6dsl_init(&cfg->intf);
    if (result != RT_EOK)
    {
        LOG_E("_lsm6dsl init err code: %d", result);
        goto __exit;
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_acce)
        rt_free(sensor_acce);
    if (sensor_gyro)
        rt_free(sensor_gyro);
    if (sensor_step)
        rt_free(sensor_step);
    return -RT_ERROR;
}
