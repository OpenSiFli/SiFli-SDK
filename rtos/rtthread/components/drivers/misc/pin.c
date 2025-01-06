/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-20     Bernard      the first version
 */

#include <drivers/pin.h>

#ifdef RT_USING_FINSH
    #include <finsh.h>
#endif

#ifndef RT_PIN_NONE
    #define RT_PIN_NONE 0xFFFF
#endif

static rt_err_t update_pin_value(rt_device_t dev, rt_base_t pin, rt_base_t value);

#if (LB55X_CHIP_ID == 3)
    __ROM_USED struct rt_device_pin _hw_pin;
#else
    static struct rt_device_pin _hw_pin;
#endif
static rt_size_t _pin_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    struct rt_device_pin_status *status;
    struct rt_device_pin *pin = (struct rt_device_pin *)dev;

    /* check parameters */
    RT_ASSERT(pin != RT_NULL);

    status = (struct rt_device_pin_status *) buffer;
    if (status == RT_NULL || size != sizeof(*status)) return 0;

    status->status = pin->ops->pin_read(dev, status->pin);
    return size;
}

static rt_size_t _pin_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct rt_device_pin_status *status;
    struct rt_device_pin *pin = (struct rt_device_pin *)dev;

    /* check parameters */
    RT_ASSERT(pin != RT_NULL);

    status = (struct rt_device_pin_status *) buffer;
    if (status == RT_NULL || size != sizeof(*status)) return 0;

    update_pin_value(dev, (rt_base_t)status->pin, (rt_base_t)status->status);
    pin->ops->pin_write(dev, (rt_base_t)status->pin, (rt_base_t)status->status);

    return size;
}

static rt_err_t add_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    int i;
    struct rt_device_pin *pin_dev = (struct rt_device_pin *)dev;
    RT_ASSERT(pin < RT_PIN_NONE);
    RT_ASSERT(mode < 0xff);

    for (i = 0; i < MAX_PIN_MODE; i++)
    {
        if (pin_dev->mode[i].pin == pin || pin_dev->mode[i].pin == RT_PIN_NONE)
            break;
    }
    if (i < MAX_PIN_MODE)
    {
        pin_dev->mode[i].pin = pin;
        pin_dev->mode[i].mode = mode;
        pin_dev->mode[i].irq_enable = 0;
        return RT_EOK;
    }
    else
        return RT_ERROR;
}


static rt_err_t update_pin_irq(rt_device_t dev, rt_base_t pin, rt_base_t irq_enable)
{
    int i;
    struct rt_device_pin *pin_dev = (struct rt_device_pin *)dev;

    RT_ASSERT(pin < RT_PIN_NONE);
    for (i = 0; i < MAX_PIN_MODE; i++)
    {
        if (pin_dev->mode[i].pin == pin)
            break;
    }
    if (i < MAX_PIN_MODE)
    {
        pin_dev->mode[i].irq_enable = irq_enable;
        return RT_EOK;
    }
    else
        return RT_ERROR;

}

static rt_err_t update_pin_value(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    int i;
    struct rt_device_pin *pin_dev = (struct rt_device_pin *)dev;

    RT_ASSERT(pin < RT_PIN_NONE);
    for (i = 0; i < MAX_PIN_MODE; i++)
    {
        if (pin_dev->mode[i].pin == pin)
            break;
    }
    if (i < MAX_PIN_MODE)
    {
        pin_dev->mode[i].value = value;
        return RT_EOK;
    }
    else
        return RT_ERROR;

}

static rt_err_t del_pin_mode(rt_device_t *dev, rt_base_t pin)
{
    int i;
    struct rt_device_pin *pin_dev = (struct rt_device_pin *)dev;

    RT_ASSERT(pin < RT_PIN_NONE);
    for (i = 0; i < MAX_PIN_MODE; i++)
    {
        if (pin_dev->mode[i].pin == pin || pin_dev->mode[i].pin == RT_PIN_NONE)
            break;
    }
    if (pin_dev->mode[i].pin == RT_PIN_NONE)
        return RT_ERROR;
    else
    {
        pin_dev->mode[i].pin = RT_PIN_NONE;
        i++;
        for (; i < MAX_PIN_MODE; i++)
        {
            if (pin_dev->mode[i].pin != RT_PIN_NONE)
            {
                pin_dev->mode[i - 1] = pin_dev->mode[i];
                pin_dev->mode[i].pin = RT_PIN_NONE;
            }
        }
    }
    return RT_EOK;
}

static rt_err_t  _pin_control(rt_device_t dev, int cmd, void *args)
{
    struct rt_device_pin *pin = (struct rt_device_pin *)dev;

    /* check parameters */
    RT_ASSERT(pin != RT_NULL);

    if (RT_DEVICE_CTRL_SUSPEND == cmd)
    {
        if (pin->ops->pin_suspend) pin->ops->pin_suspend(dev, 0);
        return RT_EOK;
    }
    else if (RT_DEVICE_CTRL_RESUME == cmd)
    {
#if 0//def RT_USING_PM
        int i;
        for (i = 0; i < MAX_PIN_MODE; i++)
        {
            if (pin->mode[i].pin == RT_PIN_NONE)
                break;
            pin->ops->pin_mode(dev, (rt_base_t)pin->mode[i].pin, (rt_base_t)pin->mode[i].mode);
            if ((PIN_MODE_OUTPUT == pin->mode[i].mode)
                    || (PIN_MODE_OUTPUT_OD == pin->mode[i].mode))
            {
                pin->ops->pin_write(dev, (rt_base_t)pin->mode[i].pin, (rt_base_t)pin->mode[i].value);
            }
            if (pin->mode[i].irq_enable)
                pin->ops->pin_irq_enable(dev, (rt_base_t)pin->mode[i].pin, (rt_base_t)pin->mode[i].irq_enable);
        }
#endif

        if (pin->ops->pin_resume) pin->ops->pin_resume(dev, 0);

        return RT_EOK;
    }

    if (args == RT_NULL)
    {
        return -RT_ERROR;
    }

    {
        struct rt_device_pin_mode *pin_mode = (struct rt_device_pin_mode *)args;
        add_pin_mode(dev, (rt_base_t)pin_mode->pin, (rt_base_t)pin_mode->mode);
        pin->ops->pin_mode(dev, (rt_base_t)pin_mode->pin, (rt_base_t)pin_mode->mode);
    }

    return 0;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops pin_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    _pin_read,
    _pin_write,
    _pin_control
};
#endif

__ROM_USED int rt_device_pin_register(const char *name, const struct rt_pin_ops *ops, void *user_data)
{
    _hw_pin.parent.type         = RT_Device_Class_Miscellaneous;
    _hw_pin.parent.rx_indicate  = RT_NULL;
    _hw_pin.parent.tx_complete  = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    _hw_pin.parent.ops          = &pin_ops;
#else
    _hw_pin.parent.init         = RT_NULL;
    _hw_pin.parent.open         = RT_NULL;
    _hw_pin.parent.close        = RT_NULL;
    _hw_pin.parent.read         = _pin_read;
    _hw_pin.parent.write        = _pin_write;
    _hw_pin.parent.control      = _pin_control;
#endif

    _hw_pin.ops                 = ops;
    _hw_pin.parent.user_data    = user_data;

    for (int i = 0; i < MAX_PIN_MODE; i++)
    {
        _hw_pin.mode[i].pin = RT_PIN_NONE;
    }

    /* register a character device */
    rt_device_register(&_hw_pin.parent, name, RT_DEVICE_FLAG_RDWR);

    return 0;
}

__ROM_USED rt_err_t rt_pin_attach_irq(rt_int32_t pin, rt_uint32_t mode,
                                      void (*hdr)(void *args), void  *args)
{
    RT_ASSERT(_hw_pin.ops != RT_NULL);
    if (_hw_pin.ops->pin_attach_irq)
    {
        return _hw_pin.ops->pin_attach_irq(&_hw_pin.parent, pin, mode, hdr, args);
    }
    return RT_ENOSYS;
}
__ROM_USED rt_err_t rt_pin_detach_irq(rt_int32_t pin)
{
    RT_ASSERT(_hw_pin.ops != RT_NULL);
    if (_hw_pin.ops->pin_detach_irq)
    {
        return _hw_pin.ops->pin_detach_irq(&_hw_pin.parent, pin);
    }
    return RT_ENOSYS;
}

__ROM_USED rt_err_t rt_pin_irq_enable(rt_base_t pin, rt_uint32_t enabled)
{
    RT_ASSERT(_hw_pin.ops != RT_NULL);
    if (_hw_pin.ops->pin_irq_enable)
    {
        update_pin_irq(&_hw_pin.parent, pin, enabled);
        return _hw_pin.ops->pin_irq_enable(&_hw_pin.parent, pin, enabled);
    }
    return RT_ENOSYS;
}

/* RT-Thread Hardware PIN APIs */
__ROM_USED void rt_pin_mode(rt_base_t pin, rt_base_t mode)
{
    RT_ASSERT(_hw_pin.ops != RT_NULL);
    add_pin_mode(&_hw_pin.parent, pin, mode);
    _hw_pin.ops->pin_mode(&_hw_pin.parent, pin, mode);
}
FINSH_FUNCTION_EXPORT_ALIAS(rt_pin_mode, pinMode, set hardware pin mode);

__ROM_USED void rt_pin_write(rt_base_t pin, rt_base_t value)
{
    RT_ASSERT(_hw_pin.ops != RT_NULL);
    update_pin_value(&_hw_pin.parent, pin, value);
    _hw_pin.ops->pin_write(&_hw_pin.parent, pin, value);
}
FINSH_FUNCTION_EXPORT_ALIAS(rt_pin_write, pinWrite, write value to hardware pin);

__ROM_USED int  rt_pin_read(rt_base_t pin)
{
    RT_ASSERT(_hw_pin.ops != RT_NULL);
    return _hw_pin.ops->pin_read(&_hw_pin.parent, pin);
}
FINSH_FUNCTION_EXPORT_ALIAS(rt_pin_read, pinRead, read status from hardware pin);
