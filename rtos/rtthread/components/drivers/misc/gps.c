/*
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "gps"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>
struct rt_gps_device gps_obj;

static rt_err_t rt_gps_control(struct rt_device *dev, int cmd, void *args);


static rt_bool_t check_notify_cb_exist(struct rt_gps_device *gps_handle, gps_notify_cb cb)
{
    int index = 0;
    for (index = 0; index < GPS_MAX_EVENT_NOTIFY_CB_NUM; index++)
    {
        if (cb == gps_handle->cb_arry.cb[index])
        {
            return RT_TRUE;
        }
    }
    return RT_FALSE;
}

static gps_err_t register_notify_event_cb(struct rt_gps_device *gps_handle, gps_notify_cb cb)
{
    int index = 0;
    gps_err_t ret = GPS_EOK;
    rt_mutex_take(gps_handle->handle_lock, RT_WAITING_FOREVER);
    if (GPS_MAX_EVENT_NOTIFY_CB_NUM == gps_handle->cb_arry.size)
    {
        ret = GPS_ERROR_NOTIFY_CB_FULL;
        goto __exit;
    }

    if (RT_TRUE == check_notify_cb_exist(gps_handle, cb))
    {
        LOG_I("gps notify cb has been register");
        goto __exit;
    }

    for (index = 0; index < GPS_MAX_EVENT_NOTIFY_CB_NUM; index++)
    {
        if (RT_NULL == gps_handle->cb_arry.cb[index])
        {
            gps_handle->cb_arry.cb[index] = cb;
            gps_handle->cb_arry.size++;
            LOG_I("gps notify cb register pass:%d", gps_handle->cb_arry.size);
            break;
        }
    }
__exit:
    rt_mutex_release(gps_handle->handle_lock);
    return ret;
}


static gps_err_t unregister_notify_event_cb(struct rt_gps_device *gps_handle, gps_notify_cb cb)
{
    int index = 0;
    gps_err_t ret = GPS_EOK;
    rt_mutex_take(gps_handle->handle_lock, RT_WAITING_FOREVER);
    if (0 == gps_handle->cb_arry.size)
    {
        LOG_I("gps notify cb not register");
        goto __exit;
    }

    if (RT_FALSE == check_notify_cb_exist(gps_handle, cb))
    {
        LOG_I("gps notify cb not register");
        goto __exit;
    }

    for (index = 0; index < GPS_MAX_EVENT_NOTIFY_CB_NUM; index++)
    {
        if (cb == gps_handle->cb_arry.cb[index])
        {
            gps_handle->cb_arry.cb[index] = RT_NULL;
            gps_handle->cb_arry.size--;
            LOG_I("gps notify cb unregister pass:%d", gps_handle->cb_arry.size);
            break;
        }
    }
__exit:
    rt_mutex_release(gps_handle->handle_lock);
    return ret;
}


static gps_err_t rt_gps_close(struct rt_device *dev)
{
    struct rt_gps_device *gps_handle = RT_NULL;
    gps_err_t ret = GPS_EOK;
    RT_ASSERT(dev != RT_NULL);
    gps_handle = (struct rt_gps_device *)dev;
    //ret = rt_gps_control(dev, GPS_CONTROL_CLOSE_DEVICE, RT_NULL);
    ret = gps_handle->ops->control(gps_handle, GPS_CLOSE_DEVICE, RT_NULL);
    if (ret == RT_EOK)
    {
        dev->open_flag &= ~GPS_DEVICE_FLAG_OPEN;
        gps_handle->status = GPS_STATE_POWER_OFF;
    }
    LOG_I("rt_gps_close ret:%d flag:%X", ret, dev->open_flag);
    return ret;
}

static  gps_err_t rt_gps_open(struct rt_device *dev)
{
    struct rt_gps_device *gps_handle = RT_NULL;
    gps_err_t ret = GPS_EOK;
    RT_ASSERT(dev != RT_NULL);
    gps_handle = (struct rt_gps_device *)dev;
    ret = gps_handle->ops->control(gps_handle, GPS_OPEN_DEVICE, RT_NULL);
    if (ret == RT_EOK)
    {
        dev->open_flag |= GPS_DEVICE_FLAG_OPEN;
        gps_handle->status = GPS_STATE_POWER_ON;
    }
    //dev->open_flag &= ~GPS_DEVICE_FLAG_OPEN;
    LOG_I("rt_gps_open ret:%d flag:%X", ret, dev->open_flag);
    return ret;
}

static void gps_event_notify(gps_notify_t *param)
{
    int index = 0;
    if (0 == gps_obj.cb_arry.size)
    {
        return;
    }

    for (index = 0; index < GPS_MAX_EVENT_NOTIFY_CB_NUM; index++)
    {
        if (RT_NULL != gps_obj.cb_arry.cb[index])
        {
            gps_obj.cb_arry.cb[index](param);
        }
    }
    return;
}


/* event upload for gps at */
__ROM_USED void rt_gps_event_notify(gps_notify_t *param)
{
    rt_mutex_take(gps_obj.handle_lock, RT_WAITING_FOREVER);
    switch (param->event)
    {
    default:
        break;
    }
    gps_event_notify(param);
__exit:
    rt_mutex_release(gps_obj.handle_lock);
    return;
}

static rt_err_t rt_gps_control(struct rt_device *dev,
                               int              cmd,
                               void             *args)
{
    gps_err_t ret = GPS_EOK;
    struct rt_gps_device *gps_handle;
    RT_ASSERT(dev != RT_NULL);
    gps_handle = (struct rt_gps_device *)dev;
    switch (cmd)
    {
    case GPS_REGISTER_NOTIFY:
    {
        gps_notify_cb cb = (gps_notify_cb)args;
        ret = register_notify_event_cb(gps_handle, cb);
    }
    break;

    case GPS_UNREGISTER_NOTIFY:
    {
        gps_notify_cb cb = (gps_notify_cb)args;
        ret = unregister_notify_event_cb(gps_handle, cb);
    }
    break;

    case GPS_OPEN_DEVICE:
    {
        ret = rt_gps_open(dev);
    }
    break;

    case GPS_CLOSE_DEVICE:
    {
        ret = rt_gps_close(dev);
    }
    break;

    case GPS_RESET_DEVICE:
    {
        ret = gps_handle->ops->control(gps_handle, cmd, args);
    }
    break;

    case GPS_QUERY_STATE:
    {
        gps_state_t *pState = (gps_state_t *)args;
        if (!(dev->open_flag & GPS_DEVICE_FLAG_OPEN))
        {
            *pState = GPS_STATE_POWER_OFF;
            return GPS_EOK;
        }
        *pState = GPS_STATE_POWER_ON;
        ret = GPS_EOK;
    }
    break;

    default :
        /* control device */
    {
        if (!(dev->open_flag & GPS_DEVICE_FLAG_OPEN))
        {
            return GPS_ERROR_POWER_OFF;
        }
        ret = gps_handle->ops->control(gps_handle, cmd, args);
    }
    break;
    }
    return ret;
}



#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops gps_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rt_gps_control
};
#endif

/*
 * gps register
 */
__ROM_USED rt_err_t rt_gps_register(struct rt_gps_device *dev_handle, const char *name)
{
    rt_err_t ret;
    struct rt_device *device;
    RT_ASSERT(dev_handle != RT_NULL);

    device = &(dev_handle->parent);

    device->type        = RT_Device_Class_Miscellaneous;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &gps_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_gps_control;
#endif
    device->user_data   = RT_NULL;
    /* register a Miscellaneous device */
    ret = rt_device_register(device, name, RT_DEVICE_FLAG_RDWR);
    return ret;
}

int rt_hw_gps_init(const struct rt_gps_ops *ops)
{
    rt_err_t result = 0;
    gps_obj.ops = ops;
    rt_memset(&gps_obj.cb_arry, 0x00, sizeof(gps_notify_cb_array_t));
    gps_obj.handle_lock = rt_mutex_create("gps_notify", RT_IPC_FLAG_FIFO);
    if (RT_NULL == gps_obj.handle_lock)
    {
        rt_kprintf("gps handle_lock create fail\n");
        return RT_ERROR;
    }
    gps_obj.status = GPS_STATE_POWER_OFF;
    result = rt_gps_register(&gps_obj, GPS_DEVICE_NAME);
    RT_ASSERT(result == RT_EOK);
    return result;
}


