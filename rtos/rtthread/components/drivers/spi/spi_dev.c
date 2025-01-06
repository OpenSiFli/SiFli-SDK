/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/spi.h>

/* SPI bus device interface, compatible with RT-Thread 0.3.x/1.0.x */
static rt_size_t _spi_bus_device_read(rt_device_t dev,
                                      rt_off_t    pos,
                                      void       *buffer,
                                      rt_size_t   size)
{
    struct rt_spi_bus *bus;

    bus = (struct rt_spi_bus *)dev;
    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(bus->owner != RT_NULL);

    return rt_spi_transfer(bus->owner, RT_NULL, buffer, size);
}

static rt_size_t _spi_bus_device_write(rt_device_t dev,
                                       rt_off_t    pos,
                                       const void *buffer,
                                       rt_size_t   size)
{
    struct rt_spi_bus *bus;

    bus = (struct rt_spi_bus *)dev;
    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(bus->owner != RT_NULL);

    return rt_spi_transfer(bus->owner, buffer, RT_NULL, size);
}

static rt_err_t _spi_bus_device_control(rt_device_t dev,
                                        int         cmd,
                                        void       *args)
{
    struct rt_spi_bus *bus;

    bus = (struct rt_spi_bus *)dev;   

    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        break;
    case RT_DEVICE_CTRL_RESUME:
    {
#ifdef RT_USING_PM
        uint8_t power_mode = (uint8_t)((uint32_t)args);
        if ((PM_SLEEP_MODE_STANDBY == power_mode)
             && (bus->owner))
        {
            /* reconfigure bus */
            bus->ops->configure(bus->owner, &bus->owner->config);
        }
#endif
        break;
    }
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops spi_bus_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    _spi_bus_device_read,
    _spi_bus_device_write,
    _spi_bus_device_control
};
#endif

__ROM_USED rt_err_t rt_spi_bus_device_init(struct rt_spi_bus *bus, const char *name, rt_uint32_t flag)
{
    struct rt_device *device;
    RT_ASSERT(bus != RT_NULL);

    device = &bus->parent;

    /* set device type */
    device->type    = RT_Device_Class_SPIBUS;
    /* initialize device interface */
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &spi_bus_ops;
#else
    device->init    = RT_NULL;
    device->open    = RT_NULL;
    device->close   = RT_NULL;
    device->read    = _spi_bus_device_read;
    device->write   = _spi_bus_device_write;
    device->control = _spi_bus_device_control;
#endif

    /* register to device manager */
    //return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR);
    return rt_device_register(device, name, flag);
}

/* SPI Dev device interface, compatible with RT-Thread 0.3.x/1.0.x */
static rt_err_t _spidev_device_open(rt_device_t dev, rt_uint16_t oflag)
{
#if 0
    struct rt_spi_device *device;

    device = (struct rt_spi_device *)dev;
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    return rt_spi_open(device, oflag);
#endif

    RT_ASSERT(dev != RT_NULL);
    rt_uint32_t flag = 0;
    struct rt_spi_device *device = (struct rt_spi_device *)dev;

    /* check device flag with the open flag */
    if ((oflag & RT_DEVICE_FLAG_DMA_RX) && !(dev->flag & RT_DEVICE_FLAG_DMA_RX))
    {
        return -RT_EIO;
    }
    if ((oflag & RT_DEVICE_FLAG_DMA_TX) && !(dev->flag & RT_DEVICE_FLAG_DMA_TX))
    {
        return -RT_EIO;
    }
    if ((oflag & RT_DEVICE_FLAG_INT_RX) && !(dev->flag & RT_DEVICE_FLAG_INT_RX))
    {
        return -RT_EIO;
    }
    if ((oflag & RT_DEVICE_FLAG_INT_TX) && !(dev->flag & RT_DEVICE_FLAG_INT_TX))
    {
        return -RT_EIO;
    }

    /* get open flags */
    dev->open_flag = oflag & 0xff;

    /* keep steam flag */
    /* if ((oflag & RT_DEVICE_FLAG_STREAM) || (dev->parent.open_flag & RT_DEVICE_FLAG_STREAM))
    {
        dev->open_flag |= RT_DEVICE_FLAG_STREAM;
    } */

    if (oflag & RT_DEVICE_FLAG_DMA_RX)
    {
        dev->open_flag |= RT_DEVICE_FLAG_DMA_RX;
        flag |= RT_DEVICE_FLAG_DMA_RX;
        //dev->bus->ops->control(dev, RT_DEVICE_CTRL_CONFIG, (void *) RT_DEVICE_FLAG_DMA_RX);
    }
    else if (oflag & RT_DEVICE_FLAG_INT_RX)
    {
        dev->open_flag |= RT_DEVICE_FLAG_INT_RX;
        flag |= RT_DEVICE_FLAG_INT_RX;
        //device->bus->ops->control(device, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
    }

    if (oflag & RT_DEVICE_FLAG_DMA_TX)
    {
        dev->open_flag |= RT_DEVICE_FLAG_DMA_TX;
        flag |= RT_DEVICE_FLAG_DMA_TX;
        //dev->bus->ops->control(dev, RT_DEVICE_CTRL_CONFIG, (void *)RT_DEVICE_FLAG_DMA_TX);
    }
    else if (oflag & RT_DEVICE_FLAG_INT_TX)
    {
        dev->open_flag |= RT_DEVICE_FLAG_INT_TX;
        flag |= RT_DEVICE_FLAG_INT_TX;
        //device->bus->ops->control(device, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_TX);
    }

    if (0 != flag)
    {
        device->bus->ops->control(device, RT_DEVICE_CTRL_SET_INT, (void *) flag);
    }

    return RT_EOK;


}

static rt_err_t _spidev_device_close(rt_device_t dev)
{
#if 0
    struct rt_spi_device *device;

    device = (struct rt_spi_device *)dev;
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    return rt_spi_close(device);
#endif
    RT_ASSERT(dev != RT_NULL);

    rt_uint32_t flag = 0;
    struct rt_spi_device *device = (struct rt_spi_device *)dev;

    /* this device has more reference count */
    if (dev->ref_count > 1)
    {
        return RT_EOK;
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_RX;
        flag |= RT_DEVICE_FLAG_INT_RX;
        //device->bus->ops->control(device, RT_DEVICE_CTRL_CLR_INT, (void*)RT_DEVICE_FLAG_INT_RX);
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_TX;
        flag |= RT_DEVICE_FLAG_INT_TX;
        //device->bus->ops->control(device, RT_DEVICE_CTRL_CLR_INT, (void*)RT_DEVICE_FLAG_INT_TX);
    }

    if (dev->open_flag & RT_DEVICE_FLAG_DMA_RX)
    {
        dev->open_flag &= ~RT_DEVICE_FLAG_DMA_RX;
        flag |= RT_DEVICE_FLAG_DMA_RX;
        //device->bus->ops->control(device, RT_DEVICE_CTRL_CLR_INT, (void *) RT_DEVICE_FLAG_DMA_RX);
    }

    if (dev->open_flag & RT_DEVICE_FLAG_DMA_TX)
    {
        dev->open_flag &= ~RT_DEVICE_FLAG_DMA_TX;
        flag |= RT_DEVICE_FLAG_DMA_TX;
        //device->bus->ops->control(device, RT_DEVICE_CTRL_CLR_INT, (void *) RT_DEVICE_FLAG_DMA_RX);
    }

    if (0 != flag)
    {
        device->bus->ops->control(device, RT_DEVICE_CTRL_CLR_INT, (void *) flag);
    }

    return RT_EOK;


}


static rt_size_t _spidev_device_read(rt_device_t dev,
                                     rt_off_t    pos,
                                     void       *buffer,
                                     rt_size_t   size)
{
    struct rt_spi_device *device;

    device = (struct rt_spi_device *)dev;
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    return rt_spi_transfer(device, RT_NULL, buffer, size);
}

static rt_size_t _spidev_device_write(rt_device_t dev,
                                      rt_off_t    pos,
                                      const void *buffer,
                                      rt_size_t   size)
{
    struct rt_spi_device *device;

    device = (struct rt_spi_device *)dev;
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    return rt_spi_transfer(device, buffer, RT_NULL, size);
}

static rt_err_t _spidev_device_control(rt_device_t dev,
                                       int         cmd,
                                       void       *args)
{
    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        break;
    case RT_DEVICE_CTRL_RESUME:
    {
#ifdef RT_USING_PM
        uint8_t power_mode = (uint8_t)((uint32_t)args);
        if ((PM_SLEEP_MODE_STANDBY == power_mode)
             && (dev->ref_count > 0))
        {
            /* enable interrupt if necessary  */
            _spidev_device_open(dev, dev->open_flag);
        }
#endif
        break;
    }
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops spi_device_ops =
{
    RT_NULL,
    _spidev_device_open,
    _spidev_device_close,
    _spidev_device_read,
    _spidev_device_write,
    _spidev_device_control
};
#endif

__ROM_USED rt_err_t rt_spidev_device_init(struct rt_spi_device *dev, const char *name, rt_uint32_t flag)
{
    struct rt_device *device;
    RT_ASSERT(dev != RT_NULL);

    device = &(dev->parent);

    /* set device type */
    device->type    = RT_Device_Class_SPIDevice;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &spi_device_ops;
#else
    device->init    = RT_NULL;
    device->open    = _spidev_device_open;
    device->close   = _spidev_device_close;
    device->read    = _spidev_device_read;
    device->write   = _spidev_device_write;
    device->control = _spidev_device_control;
#endif

    /* register to device manager */
    //return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR);
    return rt_device_register(device, name, flag);

}
