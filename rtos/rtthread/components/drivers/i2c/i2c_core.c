/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2012-04-25     weety         first version
 */

#include <rtdevice.h>

__ROM_USED rt_err_t rt_i2c_bus_device_register(struct rt_i2c_bus_device *bus,
        const char               *bus_name,
        rt_uint32_t              flag)
{
    rt_err_t res = RT_EOK;

    rt_mutex_init(&bus->lock, "i2c_bus_lock", RT_IPC_FLAG_FIFO);

    if (bus->timeout == 0) bus->timeout = RT_TICK_PER_SECOND;

    res = rt_i2c_bus_device_device_init(bus, bus_name, flag);

    i2c_dbg("I2C bus [%s] registered\n", bus_name);

    return res;
}

__ROM_USED struct rt_i2c_bus_device *rt_i2c_bus_device_find(const char *bus_name)
{
    struct rt_i2c_bus_device *bus;
    rt_device_t dev = rt_device_find(bus_name);
    if (dev == RT_NULL || dev->type != RT_Device_Class_I2CBUS)
    {
        i2c_dbg("I2C bus %s not exist\n", bus_name);

        return RT_NULL;
    }

    bus = (struct rt_i2c_bus_device *)dev->user_data;

    return bus;
}

__ROM_USED rt_err_t rt_i2c_open(struct rt_i2c_bus_device *dev, rt_uint16_t oflag)
{
    RT_ASSERT(dev != RT_NULL);
    rt_uint32_t flag = 0;

    /* check device flag with the open flag */
    if ((oflag & RT_DEVICE_FLAG_DMA_RX) && !(dev->parent.flag & RT_DEVICE_FLAG_DMA_RX))
    {
        return -RT_EIO;
    }
    if ((oflag & RT_DEVICE_FLAG_DMA_TX) && !(dev->parent.flag & RT_DEVICE_FLAG_DMA_TX))
    {
        return -RT_EIO;
    }
    if ((oflag & RT_DEVICE_FLAG_INT_RX) && !(dev->parent.flag & RT_DEVICE_FLAG_INT_RX))
    {
        return -RT_EIO;
    }
    if ((oflag & RT_DEVICE_FLAG_INT_TX) && !(dev->parent.flag & RT_DEVICE_FLAG_INT_TX))
    {
        return -RT_EIO;
    }

    /* get open flags */
    dev->parent.open_flag = oflag & 0xff;

    /* keep steam flag */
    /* if ((oflag & RT_DEVICE_FLAG_STREAM) || (dev->parent.open_flag & RT_DEVICE_FLAG_STREAM))
    {
        dev->parent.open_flag |= RT_DEVICE_FLAG_STREAM;
    } */

    if (oflag & RT_DEVICE_FLAG_DMA_RX)
    {
        dev->parent.open_flag |= RT_DEVICE_FLAG_DMA_RX;
        flag |= RT_DEVICE_FLAG_DMA_RX;
        //dev->bus->ops->control(dev, RT_DEVICE_CTRL_CONFIG, (void *) RT_DEVICE_FLAG_DMA_RX);
    }
    else if (oflag & RT_DEVICE_FLAG_INT_RX)
    {
        dev->parent.open_flag |= RT_DEVICE_FLAG_INT_RX;
        flag |= RT_DEVICE_FLAG_INT_RX;
        //device->bus->ops->control(device, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
    }

    if (oflag & RT_DEVICE_FLAG_DMA_TX)
    {
        dev->parent.open_flag |= RT_DEVICE_FLAG_DMA_TX;
        flag |= RT_DEVICE_FLAG_DMA_TX;
        //dev->bus->ops->control(dev, RT_DEVICE_CTRL_CONFIG, (void *)RT_DEVICE_FLAG_DMA_TX);
    }
    else if (oflag & RT_DEVICE_FLAG_INT_TX)
    {
        dev->parent.open_flag |= RT_DEVICE_FLAG_INT_TX;
        flag |= RT_DEVICE_FLAG_INT_TX;
        //device->bus->ops->control(device, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_TX);
    }

    if (0 != flag)
    {
        dev->ops->i2c_bus_control(dev, RT_DEVICE_CTRL_SET_INT, (void *)flag);
    }

    return RT_EOK;
}


__ROM_USED rt_err_t rt_i2c_close(struct rt_i2c_bus_device *dev)
{
    RT_ASSERT(dev != RT_NULL);

    rt_uint32_t flag = 0;

    /* this device has more reference count */
    if (dev->parent.ref_count > 1)
    {
        return RT_EOK;
    }

    if (dev->parent.open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        dev->parent.open_flag &= ~RT_DEVICE_FLAG_INT_RX;
        flag |= RT_DEVICE_FLAG_INT_RX;
        //dev->ops->i2c_bus_control(dev, RT_DEVICE_CTRL_CLR_INT, (void*)RT_DEVICE_FLAG_INT_RX);
    }

    if (dev->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        dev->parent.open_flag &= ~RT_DEVICE_FLAG_INT_TX;
        flag |= RT_DEVICE_FLAG_INT_TX;
        //dev->ops->i2c_bus_control(dev, RT_DEVICE_CTRL_CLR_INT, (void*)RT_DEVICE_FLAG_INT_TX);
    }

    if (dev->parent.open_flag & RT_DEVICE_FLAG_DMA_RX)
    {
        dev->parent.open_flag &= ~RT_DEVICE_FLAG_DMA_RX;
        flag |= RT_DEVICE_FLAG_DMA_RX;
        //dev->ops->i2c_bus_control(dev, RT_DEVICE_CTRL_CLR_INT, (void*)RT_DEVICE_FLAG_DMA_RX);
    }

    if (dev->parent.open_flag & RT_DEVICE_FLAG_DMA_TX)
    {
        dev->parent.open_flag &= ~RT_DEVICE_FLAG_DMA_TX;
        flag |= RT_DEVICE_FLAG_DMA_TX;
        //dev->ops->i2c_bus_control(dev, RT_DEVICE_CTRL_CLR_INT, (void*)RT_DEVICE_FLAG_DMA_TX);
    }

    if (0 != flag)
    {
        dev->ops->i2c_bus_control(dev, RT_DEVICE_CTRL_CLR_INT, (void *)flag);
    }

    return RT_EOK;
}


__ROM_USED rt_err_t rt_i2c_control(struct rt_i2c_bus_device *dev,  int cmd, void  *args)
{
    rt_err_t ret = RT_EOK;

    RT_ASSERT(dev != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
    {
        /* suspend device */
        uint8_t power_mode = (uint8_t)((uint32_t)args);
        dev->parent.flag |= RT_DEVICE_FLAG_SUSPENDED;
#ifdef RT_USING_PM
        if (PM_SLEEP_MODE_STANDBY == power_mode)
            ret = dev->ops->i2c_bus_control(dev, cmd, args);
#endif
        break;
    }
    case RT_DEVICE_CTRL_RESUME:
    {
        /* resume device */
#ifdef RT_USING_PM
        uint8_t power_mode = (uint8_t)((uint32_t)args);
        if (PM_SLEEP_MODE_STANDBY == power_mode)
        {
            /*Resume last I2C configuration*/
            dev->ops->i2c_bus_configure(dev, NULL);
            /*Resume IRQ if opened*/
            if (dev->parent.ref_count > 0) rt_i2c_open(dev, dev->parent.open_flag);
        }
#endif

        dev->parent.flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        break;
    }

    case RT_DEVICE_CTRL_CONFIG:
        if (args)
        {
            if (dev->parent.ref_count)
            {
                /* serial device has been opened, to configure it */
                ret = dev->ops->i2c_bus_configure(dev, NULL);
            }
        }

        break;

    default :
        /* control device */
        ret = dev->ops->i2c_bus_control(dev, cmd, args);
        break;
    }

    return ret;
}



__ROM_USED rt_err_t rt_i2c_configure(struct rt_i2c_bus_device        *device,
                                     struct rt_i2c_configuration *cfg)
{
    rt_err_t result;

    RT_ASSERT(device != RT_NULL);

    result = rt_mutex_take(&(device->lock), RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        result = device->ops->i2c_bus_configure(device, cfg);

        /* release lock */
        rt_mutex_release(&(device->lock));
    }

    return result;
}


__ROM_USED rt_size_t rt_i2c_transfer(struct rt_i2c_bus_device *bus,
                                     struct rt_i2c_msg         msgs[],
                                     rt_uint32_t               num)
{
    rt_size_t ret;

    if (bus->ops->master_xfer)
    {
#ifdef RT_I2C_DEBUG
        for (ret = 0; ret < num; ret++)
        {
            i2c_dbg("msgs[%d] %c, addr=0x%02x, len=%d\n", ret,
                    (msgs[ret].flags & RT_I2C_RD) ? 'R' : 'W',
                    msgs[ret].addr, msgs[ret].len);
        }
#endif

        rt_mutex_take(&bus->lock, RT_WAITING_FOREVER);
        ret = bus->ops->master_xfer(bus, msgs, num);
        rt_mutex_release(&bus->lock);

        return ret;
    }
    else
    {
        i2c_dbg("I2C bus operation not supported\n");

        return 0;
    }
}

__ROM_USED rt_size_t rt_i2c_master_send(struct rt_i2c_bus_device *bus,
                                        rt_uint16_t               addr,
                                        rt_uint16_t               flags,
                                        const rt_uint8_t         *buf,
                                        rt_uint32_t               count)
{
    rt_err_t ret;
    struct rt_i2c_msg msg;

    msg.addr  = addr;
    msg.flags = flags & RT_I2C_ADDR_10BIT;
    msg.len   = count;
    msg.buf   = (rt_uint8_t *)buf;

    ret = rt_i2c_transfer(bus, &msg, 1);

    return (ret > 0) ? count : ret;
}

__ROM_USED rt_size_t rt_i2c_master_recv(struct rt_i2c_bus_device *bus,
                                        rt_uint16_t               addr,
                                        rt_uint16_t               flags,
                                        rt_uint8_t               *buf,
                                        rt_uint32_t               count)
{
    rt_err_t ret;
    struct rt_i2c_msg msg;
    RT_ASSERT(bus != RT_NULL);

    msg.addr   = addr;
    msg.flags  = flags & RT_I2C_ADDR_10BIT;
    msg.flags |= RT_I2C_RD;
    msg.len    = count;
    msg.buf    = buf;

    ret = rt_i2c_transfer(bus, &msg, 1);

    return (ret > 0) ? count : ret;
}

__ROM_USED rt_size_t rt_i2c_mem_read(struct rt_i2c_bus_device *bus,
                                     rt_uint16_t dev_addr,
                                     rt_uint16_t mem_addr,
                                     rt_uint16_t mem_addr_size,
                                     void       *buffer,
                                     rt_size_t   count)
{
    rt_size_t ret;
    struct rt_i2c_msg msg;

    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    i2c_dbg("I2C bus dev [%s] reading mem %u bytes.\n", bus->parent.parent.name, count);

    msg.addr   = dev_addr;
    msg.mem_addr = mem_addr;
    msg.mem_addr_size = mem_addr_size;
    msg.flags  = RT_I2C_RD | RT_I2C_MEM_ACCESS;
    msg.len    = count;
    msg.buf    = buffer;

    ret = rt_i2c_transfer(bus, &msg, 1);

    return (ret > 0) ? count : ret;
}

__ROM_USED rt_size_t rt_i2c_mem_write(struct rt_i2c_bus_device *bus,
                                      rt_uint16_t dev_addr,
                                      rt_uint16_t mem_addr,
                                      rt_uint16_t mem_addr_size,
                                      void       *buffer,
                                      rt_size_t   count)
{
    rt_size_t ret;
    struct rt_i2c_msg msg;

    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    i2c_dbg("I2C bus dev [%s] writing mem %u bytes.\n", bus->parent.parent.name, count);

    msg.addr   = dev_addr;
    msg.mem_addr = mem_addr;
    msg.mem_addr_size = mem_addr_size;
    msg.flags  = RT_I2C_WR | RT_I2C_MEM_ACCESS;
    msg.len    = count;
    msg.buf    = buffer;

    ret = rt_i2c_transfer(bus, &msg, 1);

    return (ret > 0) ? count : ret;
}

__ROM_USED int rt_i2c_core_init(void)
{
    return 0;
}
INIT_COMPONENT_EXPORT(rt_i2c_core_init);
