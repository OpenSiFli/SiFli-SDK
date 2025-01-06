/*
 * COPYRIGHT (C) 2018, Real-Thread Information Technology Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-09-12     heyuanjie87  first version.
 * 2014-03-04     Bernard      code cleanup
 */

#include <drivers/watchdog.h>

/* RT-Thread Device Interface */

/*
 * This function initializes serial
 */
static rt_err_t rt_watchdog_init(struct rt_device *dev)
{
    rt_watchdog_t *wtd;

    RT_ASSERT(dev != RT_NULL);
    wtd = (rt_watchdog_t *)dev;
    if (wtd->ops->init)
    {
        return (wtd->ops->init(wtd));
    }

    return (-RT_ENOSYS);
}

static rt_err_t rt_watchdog_open(struct rt_device *dev, rt_uint16_t oflag)
{
    return (RT_EOK);
}

static rt_err_t rt_watchdog_close(struct rt_device *dev)
{
    rt_watchdog_t *wtd;

    RT_ASSERT(dev != RT_NULL);
    wtd = (rt_watchdog_t *)dev;

    if (wtd->ops->control(wtd, RT_DEVICE_CTRL_WDT_STOP, RT_NULL) != RT_EOK)
    {
        rt_kprintf(" This watchdog can not be stoped\n");

        return (-RT_ERROR);
    }

    return (RT_EOK);
}

static rt_err_t rt_watchdog_control(struct rt_device *dev,
                                    int              cmd,
                                    void             *args)
{
    rt_watchdog_t *wtd;

    RT_ASSERT(dev != RT_NULL);
    wtd = (rt_watchdog_t *)dev;

    return (wtd->ops->control(wtd, cmd, args));
}

#ifdef RT_USING_DEVICE_OPS
__ROM_USED const static struct rt_device_ops wdt_ops =
{
    rt_watchdog_init,
    rt_watchdog_open,
    rt_watchdog_close,
    RT_NULL,
    RT_NULL,
    rt_watchdog_control,
};
#endif

/**
 * This function register a watchdog device
 */
__ROM_USED rt_err_t rt_hw_watchdog_register(struct rt_watchdog_device *wtd,
        const char                *name,
        rt_uint32_t                flag,
        void                      *data)
{
    struct rt_device *device;
    RT_ASSERT(wtd != RT_NULL);

    device = &(wtd->parent);

    device->type        = RT_Device_Class_Miscellaneous;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &wdt_ops;
#else
    device->init        = rt_watchdog_init;
    device->open        = rt_watchdog_open;
    device->close       = rt_watchdog_close;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_watchdog_control;
#endif
    device->user_data   = data;

    /* register a character device */
    return rt_device_register(device, name, flag);
}



__ROM_USED rt_device_t wdt_dev;

__ROM_USED void rt_hw_watchdog_pet(void)
{
    if (wdt_dev)
    {
        rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
    }
}

__ROM_USED void rt_hw_watchdog_hook(int enable)
{
    if (enable)
        rt_thread_idle_sethook(rt_hw_watchdog_pet);
    else
        rt_thread_idle_delhook(rt_hw_watchdog_pet);
}

__ROM_USED void rt_hw_watchdog_init(void)
{
    extern int rt_wdt_init(void);
    rt_wdt_init();
    wdt_dev = rt_device_find("wdt");
    if (wdt_dev)
    {
        rt_err_t err = rt_device_open(wdt_dev, RT_DEVICE_FLAG_RDWR);
        if (err == RT_EOK)
        {
            uint32_t count = WDT_TIMEOUT;
            rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &count);
        }
    }
    rt_hw_watchdog_hook(1);
}

__ROM_USED void rt_hw_watchdog_deinit(void)
{
    rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_STOP, NULL);
}

__ROM_USED uint32_t rt_hw_watchdog_get_status(void)
{
    uint32_t wdt_status = 0;
    rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_GET_STATUS, &wdt_status);
    return wdt_status;
}

__ROM_USED void rt_hw_watchdog_set_status(uint32_t status)
{
    rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_SET_STATUS, &status);
    return;
}


