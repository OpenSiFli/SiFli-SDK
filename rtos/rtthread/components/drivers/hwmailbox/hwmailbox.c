/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2015-08-31     heyuanjie87    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>

static rt_err_t rt_hwmailbox_init(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    rt_hwmailbox_t *mailbox;

    mailbox = (rt_hwmailbox_t *)dev;

    mailbox->data_len = 0;
    mailbox->ring_buffer = RT_NULL;

    if (mailbox->ops->init)
    {
        mailbox->ops->init(mailbox);
    }
    else
    {
        result = -RT_ENOSYS;
    }

    return result;
}

static rt_err_t rt_hwmailbox_open(struct rt_device *dev, rt_uint16_t oflag)
{
    rt_err_t result = RT_EOK;
    rt_hwmailbox_t *mailbox;
    uint8_t *pool;
    struct rt_ringbuffer *rb;
    rt_base_t mask;

    mailbox = (rt_hwmailbox_t *)dev;

    if (RT_NULL != mailbox->ring_buffer)
    {
        return result;
    }

    mask = rt_hw_interrupt_disable();

    if (RT_DEVICE_OFLAG_RDONLY & oflag)
    {
        rb = (struct rt_ringbuffer *)mailbox->rx_buffer_addr;
        if (mailbox->ops->control != RT_NULL)
        {
            mailbox->ops->control(mailbox, RT_DEVICE_CTRL_SET_INT, 0);
        }
        else
        {
            result = -RT_ENOSYS;
        }
    }
    else if (RT_DEVICE_OFLAG_WRONLY & oflag)
    {
        /* sender initialize the buffer */
        rb = (struct rt_ringbuffer *)mailbox->tx_buffer_addr;
        pool = (uint8_t *)(rb + 1);
        rt_ringbuffer_wr_init(rb, pool, mailbox->buffer_size - sizeof(struct rt_ringbuffer));

        /* update rd_buf_ptr */
        pool = (uint8_t *)((struct rt_ringbuffer *)(mailbox->rx_buffer_addr) + 1);
        rt_ringbuffer_rd_init(rb, pool, mailbox->buffer_size - sizeof(struct rt_ringbuffer));
        if (mailbox->ops->control != RT_NULL)
        {
            mailbox->ops->control(mailbox, RT_DEVICE_CTRL_SET_INT, 0);
        }
        else
        {
            result = -RT_ENOSYS;
        }
    }
    else
    {
        result = -RT_EIO;
    }

    if (RT_EOK == result)
    {
        mailbox->ring_buffer = rb;
        mailbox->parent.open_flag |= oflag;
    }

    rt_hw_interrupt_enable(mask);

    return result;
}

static rt_err_t rt_hwmailbox_close(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    rt_hwmailbox_t *mailbox;

    mailbox = (rt_hwmailbox_t *)dev;
    if (dev->open_flag & RT_DEVICE_OFLAG_RDONLY)
    {
        if (mailbox->ops->control != RT_NULL)
        {
            mailbox->ops->control(mailbox, RT_DEVICE_CTRL_CLR_INT, 0);
        }
        else
        {
            result = -RT_ENOSYS;
        }
    }
    else if (dev->open_flag & RT_DEVICE_OFLAG_WRONLY)
    {
        if (mailbox->ops->control != RT_NULL)
        {
            mailbox->ops->control(mailbox, RT_DEVICE_CTRL_CLR_INT, 0);
        }
        else
        {
            result = -RT_ENOSYS;
        }
    }
    else
    {
        result = -RT_EIO;
    }

    mailbox->ring_buffer = RT_NULL;

    dev->rx_indicate = RT_NULL;

    return result;
}

static rt_size_t rt_hwmailbox_read(struct rt_device *dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_hwmailbox_t *mailbox;
    rt_size_t data_len;

    RT_ASSERT(NULL != buffer);

    mailbox = (rt_hwmailbox_t *)dev;

    if (0 == mailbox->data_len)
    {
        return 0;
    }

    //mailbox->ops->lock(mailbox, 1);
    data_len = rt_ringbuffer_get_and_update_len(mailbox->ring_buffer, buffer, size, &mailbox->data_len);
    //mailbox->ops->lock(mailbox, 0);

    return data_len;
}

static rt_size_t rt_hwmailbox_write(struct rt_device *dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_hwmailbox_t *mailbox;
    rt_size_t data_len;
    uint32_t start_time;
    uint32_t cnt;
    uint32_t total_len;

    RT_ASSERT(NULL != buffer);

    mailbox = (rt_hwmailbox_t *)dev;

    cnt = 0;
    start_time = rt_tick_get();
    total_len = size;
    while (total_len > 0)
    {
        //mailbox->ops->lock(mailbox, 1);
        data_len = rt_ringbuffer_put(mailbox->ring_buffer, buffer, total_len);
        //mailbox->ops->lock(mailbox, 0);
        RT_ASSERT(data_len <= total_len);
        total_len -= data_len;
        buffer = (uint8_t *)buffer + data_len;

        if (data_len > 0)
        {
            /* trigger mailbox if new data has been written */
            RT_ASSERT(mailbox->ops->trigger);
            mailbox->ops->trigger(mailbox);
        }
        if (rt_tick_get() != start_time)
        {
            cnt++;
            start_time = rt_tick_get();
        }
        if (cnt >= 10)
        {
            break;
        }
    }

    return (size - total_len);
}

static rt_err_t rt_hwmailbox_control(struct rt_device *dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    rt_hwmailbox_t *mailbox;

    mailbox = (rt_hwmailbox_t *)dev;
    if (cmd == RT_DEVICE_CTRL_RESUME)
    {
        /* resume device */
#ifdef RT_USING_PM
        uint8_t power_mode = (uint8_t)(uint32_t)args;
        if ((PM_SLEEP_MODE_STANDBY == power_mode)
                && (dev->ref_count > 0))
        {
            result = rt_hwmailbox_open(dev, dev->open_flag);
        }
#endif
    }
    else if (cmd == RT_DEVICE_CTRL_SUSPEND)
    {
        /* suspend device */
#ifdef RT_USING_PM
        uint8_t power_mode = (uint8_t)(uint32_t)args;
        if ((PM_SLEEP_MODE_STANDBY == power_mode)
                && (dev->ref_count > 0))
        {
            if (mailbox->ops->control != RT_NULL)
            {
                mailbox->ops->control(mailbox, RT_DEVICE_CTRL_CLR_INT, 0);
            }
            mailbox->ring_buffer = RT_NULL;
        }
#endif
    }

    return result;
}

__ROM_USED void rt_device_hwmailbox_isr(rt_hwmailbox_t *mailbox)
{
    mailbox->data_len = rt_ringbuffer_data_len(mailbox->ring_buffer);
    if (mailbox->owner && mailbox->owner->rx_indicate)
    {
        mailbox->owner->rx_indicate(mailbox->owner,
                                    rt_ringbuffer_data_len(mailbox->ring_buffer));
    }
    else if (mailbox->parent.rx_indicate)
    {
        mailbox->parent.rx_indicate(&mailbox->parent,
                                    rt_ringbuffer_data_len(mailbox->ring_buffer));
    }
}

static rt_err_t rt_bidir_hwmailbox_init(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    rt_hwmailbox_t *rx_mailbox;
    rt_hwmailbox_t *tx_mailbox;

    rx_mailbox = ((rt_bidir_hwmailbox_t *)dev)->rx_device;
    tx_mailbox = ((rt_bidir_hwmailbox_t *)dev)->tx_device;

    if (rx_mailbox)
    {
        rt_device_init((rt_device_t)rx_mailbox);
    }
    else
    {
        result = -RT_ENOSYS;
    }

    if (tx_mailbox)
    {
        rt_device_init((rt_device_t)tx_mailbox);
    }
    else
    {
        result = -RT_ENOSYS;
    }

    return result;
}

static rt_err_t rt_bidir_hwmailbox_open(struct rt_device *dev, rt_uint16_t oflag)
{
    rt_err_t result = -RT_ERROR;
    rt_hwmailbox_t *rx_mailbox;
    rt_hwmailbox_t *tx_mailbox;

    rx_mailbox = ((rt_bidir_hwmailbox_t *)dev)->rx_device;
    tx_mailbox = ((rt_bidir_hwmailbox_t *)dev)->tx_device;

    if (rx_mailbox && (RT_DEVICE_OFLAG_RDONLY & oflag))
    {
        result = rt_device_open((rt_device_t)rx_mailbox, RT_DEVICE_OFLAG_RDONLY);
    }

    if (tx_mailbox && (RT_DEVICE_OFLAG_WRONLY & oflag))
    {
        result = rt_device_open((rt_device_t)tx_mailbox, RT_DEVICE_OFLAG_WRONLY);
    }

    return result;
}

static rt_err_t rt_bidir_hwmailbox_close(struct rt_device *dev)
{
    rt_err_t result = -RT_ERROR;
    rt_hwmailbox_t *rx_mailbox;
    rt_hwmailbox_t *tx_mailbox;

    rx_mailbox = ((rt_bidir_hwmailbox_t *)dev)->rx_device;
    tx_mailbox = ((rt_bidir_hwmailbox_t *)dev)->tx_device;

    if (rx_mailbox)
    {
        result = rt_device_close((rt_device_t)rx_mailbox);
    }

    if (tx_mailbox)
    {
        result = rt_device_close((rt_device_t)tx_mailbox);
    }

    dev->rx_indicate = RT_NULL;

    return result;
}

static rt_size_t rt_bidir_hwmailbox_read(struct rt_device *dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_hwmailbox_t *mailbox;
    rt_size_t data_len;

    RT_ASSERT(NULL != buffer);

    mailbox = ((rt_bidir_hwmailbox_t *)dev)->rx_device;
    if (mailbox)
    {
        data_len = rt_device_read((rt_device_t)mailbox, pos, buffer, size);
    }
    else
    {
        data_len = 0;
    }
    return data_len;
}

static rt_size_t rt_bidir_hwmailbox_write(struct rt_device *dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_hwmailbox_t *mailbox;
    rt_size_t data_len;

    RT_ASSERT(NULL != buffer);

    mailbox = ((rt_bidir_hwmailbox_t *)dev)->tx_device;

    if (mailbox)
    {
        data_len = rt_device_write((rt_device_t)mailbox, pos, buffer, size);
    }
    else
    {
        data_len = 0;
    }

    return data_len;
}

static rt_err_t rt_bidir_hwmailbox_control(struct rt_device *dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    rt_bidir_hwmailbox_t *mailbox_dev = (rt_bidir_hwmailbox_t *)dev;

    if (mailbox_dev->tx_device)
    {
        result = rt_hwmailbox_control((rt_device_t)mailbox_dev->tx_device, cmd, args);
    }

    if (mailbox_dev->rx_device)
    {
        result += rt_hwmailbox_control((rt_device_t)mailbox_dev->rx_device, cmd, args);
    }

    return result;
}


#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops hwmailbox_ops =
{
    rt_hwmailbox_init,
    rt_hwmailbox_open,
    rt_hwmailbox_close,
    rt_hwmailbox_read,
    rt_hwmailbox_write,
    rt_hwmailbox_control
};

const static struct rt_device_ops bidir_hwmailbox_ops =
{
    rt_bidir_hwmailbox_init,
    rt_bidir_hwmailbox_open,
    rt_bidir_hwmailbox_close,
    rt_bidir_hwmailbox_read,
    rt_bidir_hwmailbox_write,
    rt_bidir_hwmailbox_control
};

#endif

__ROM_USED rt_err_t rt_device_hwmailbox_register(rt_hwmailbox_t *mailbox, const char *name, rt_uint32_t flag, void *user_data)
{
    struct rt_device *device;

    RT_ASSERT(mailbox != RT_NULL);
    RT_ASSERT(mailbox->ops != RT_NULL);

    device = &(mailbox->parent);

    device->type        = RT_Device_Class_Mailbox;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &hwmailbox_ops;
#else
    device->init        = rt_hwmailbox_init;
    device->open        = rt_hwmailbox_open;
    device->close       = rt_hwmailbox_close;
    device->read        = rt_hwmailbox_read;
    device->write       = rt_hwmailbox_write;
    device->control     = rt_hwmailbox_control;
#endif
    device->user_data   = user_data;

    return rt_device_register(device, name, flag);
}

__ROM_USED rt_err_t rt_device_bidir_hwmailbox_register(rt_bidir_hwmailbox_t *mailbox, const char *name, rt_uint32_t flag, void *user_data)
{
    struct rt_device *device;

    RT_ASSERT(mailbox != RT_NULL);

    if (mailbox->rx_device)
    {
        mailbox->rx_device->owner = (rt_device_t)mailbox;
    }

    if (mailbox->tx_device)
    {
        mailbox->tx_device->owner = (rt_device_t)mailbox;
    }

    device = &(mailbox->parent);

    device->type        = RT_Device_Class_Mailbox;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &bidir_hwmailbox_ops;
#else
    device->init        = rt_bidir_hwmailbox_init;
    device->open        = rt_bidir_hwmailbox_open;
    device->close       = rt_bidir_hwmailbox_close;
    device->read        = rt_bidir_hwmailbox_read;
    device->write       = rt_bidir_hwmailbox_write;
    device->control     = rt_bidir_hwmailbox_control;
#endif
    device->user_data   = user_data;

    return rt_device_register(device, name, flag);
}


