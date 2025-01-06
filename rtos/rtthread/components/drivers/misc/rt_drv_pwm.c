/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-07     aozima       the first version
 */

#include <string.h>

#include <drivers/rt_drv_pwm.h>

static rt_err_t _pwm_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    struct rt_device_pwm *pwm = (struct rt_device_pwm *)dev;

    if (pwm->ops->control)
    {
        result = pwm->ops->control(pwm, cmd, args);
    }

    return result;
}


/*
pos: channel
void *buffer: rt_uint32_t pulse[size]
size : number of pulse, only set to sizeof(rt_uint32_t).
*/
static rt_size_t _pwm_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_err_t result = RT_EOK;
    struct rt_device_pwm *pwm = (struct rt_device_pwm *)dev;
    rt_uint32_t *pulse = (rt_uint32_t *)buffer;
    struct rt_pwm_configuration configuration = {0};

    configuration.channel = pos;

    if (pwm->ops->control)
    {
        result = pwm->ops->control(pwm, PWM_CMD_GET,  &configuration);
        if (result != RT_EOK)
        {
            return 0;
        }

        *pulse = configuration.pulse;
    }

    return size;
}

/*
pos: channel
void *buffer: rt_uint32_t pulse[size]
size : number of pulse, only set to sizeof(rt_uint32_t).
*/
static rt_size_t _pwm_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_err_t result = RT_EOK;
    struct rt_device_pwm *pwm = (struct rt_device_pwm *)dev;
    rt_uint32_t *pulse = (rt_uint32_t *)buffer;
    struct rt_pwm_configuration configuration = {0};

    configuration.channel = pos;

    if (pwm->ops->control)
    {
        result = pwm->ops->control(pwm, PWM_CMD_GET, &configuration);
        if (result != RT_EOK)
        {
            return 0;
        }

        configuration.pulse = *pulse;

        result = pwm->ops->control(pwm, PWM_CMD_SET, &configuration);
        if (result != RT_EOK)
        {
            return 0;
        }
    }

    return size;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops pwm_device_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    _pwm_read,
    _pwm_write,
    _pwm_control
};
#endif /* RT_USING_DEVICE_OPS */

rt_err_t rt_device_pwm_register(struct rt_device_pwm *device, const char *name, const struct rt_pwm_ops *ops, const void *user_data)
{
    rt_err_t result = RT_EOK;

    memset(device, 0, sizeof(struct rt_device_pwm));

#ifdef RT_USING_DEVICE_OPS
    device->parent.ops = &pwm_device_ops;
#else
    device->parent.init = RT_NULL;
    device->parent.open = RT_NULL;
    device->parent.close = RT_NULL;
    device->parent.read  = _pwm_read;
    device->parent.write = _pwm_write;
    device->parent.control = _pwm_control;
#endif /* RT_USING_DEVICE_OPS */

    device->parent.type         = RT_Device_Class_Miscellaneous;
    device->ops                 = ops;
    device->parent.user_data    = (void *)user_data;

    result = rt_device_register(&device->parent, name, RT_DEVICE_FLAG_RDWR);

    return result;
}

rt_err_t rt_pwm_enable2(struct rt_device_pwm *device, int channel, uint8_t is_comp)
{
    rt_err_t result = RT_EOK;
    struct rt_pwm_configuration configuration = {0};

    if (!device)
    {
        return -RT_EIO;
    }
    configuration.is_comp = is_comp;
    configuration.channel = channel;
    result = rt_device_control(&device->parent, PWM_CMD_ENABLE, &configuration);

    return result;
}


rt_err_t rt_pwm_disable(struct rt_device_pwm *device, int channel)
{
    rt_err_t result = RT_EOK;
    struct rt_pwm_configuration configuration = {0};

    if (!device)
    {
        return -RT_EIO;
    }

    configuration.channel = channel;
    result = rt_device_control(&device->parent, PWM_CMD_DISABLE, &configuration);

    return result;
}

rt_err_t rt_pwm_set(struct rt_device_pwm *device, int channel, rt_uint32_t period, rt_uint32_t pulse)
{
    rt_err_t result = RT_EOK;
    struct rt_pwm_configuration configuration = {0};

    if (!device)
    {
        return -RT_EIO;
    }

    configuration.channel = channel;
    configuration.period = period;
    configuration.pulse = pulse;
    result = rt_device_control(&device->parent, PWM_CMD_SET, &configuration);

    return result;
}

rt_err_t rt_pwm_set_brk_dead(struct rt_device_pwm *device, rt_uint32_t *bkd, rt_uint32_t dt_ns)
{
    rt_err_t result = RT_EOK;
    struct rt_pwm_configuration configuration = {0};

    if (!device)
    {
        return -RT_EIO;
    }
    memcpy(&configuration.break_dead, bkd, sizeof(uint32_t));
    configuration.is_comp = 1;
    configuration.dead_time = dt_ns;
    result = rt_device_control(&device->parent, PWM_CMD_BREAK_DEAD, &configuration);
    return result;
}

