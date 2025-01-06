/*
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "bt_device"
//#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

static rt_err_t _bt_control(struct rt_device *dev, int cmd, void *args);


static rt_bool_t check_notify_cb_exist(struct rt_bt_device *bt_handle, bt_notify_cb cb)
{
    int index = 0;
    for (index = 0; index < BT_MAX_EVENT_NOTIFY_CB_NUM; index++)
    {
        if (cb == bt_handle->cb_arry.cb[index])
        {
            return RT_TRUE;
        }
    }
    return RT_FALSE;
}

static bt_err_t register_notify_event_cb(struct rt_bt_device *bt_handle, bt_notify_cb cb)
{
    int index = 0;
    bt_err_t ret = BT_EOK;
    rt_mutex_take(bt_handle->handle_lock, RT_WAITING_FOREVER);
    if (BT_MAX_EVENT_NOTIFY_CB_NUM == bt_handle->cb_arry.size)
    {
        ret = BT_ERROR_NOTIFY_CB_FULL;
        goto __exit;
    }

    if (RT_TRUE == check_notify_cb_exist(bt_handle, cb))
    {
        LOG_I("bt notify cb has been register");
        goto __exit;
    }

    for (index = 0; index < BT_MAX_EVENT_NOTIFY_CB_NUM; index++)
    {
        if (RT_NULL == bt_handle->cb_arry.cb[index])
        {
            bt_handle->cb_arry.cb[index] = cb;
            bt_handle->cb_arry.size++;
            LOG_I("bt notify cb register pass:%d", bt_handle->cb_arry.size);
            break;
        }
    }
__exit:
    rt_mutex_release(bt_handle->handle_lock);
    return ret;
}


static bt_err_t unregister_notify_event_cb(struct rt_bt_device *bt_handle, bt_notify_cb cb)
{
    int index = 0;
    bt_err_t ret = BT_EOK;
    rt_mutex_take(bt_handle->handle_lock, RT_WAITING_FOREVER);
    if (0 == bt_handle->cb_arry.size)
    {
        LOG_I("bt notify cb not register");
        goto __exit;
    }

    if (RT_FALSE == check_notify_cb_exist(bt_handle, cb))
    {
        LOG_I("bt notify cb not register");
        goto __exit;
    }

    for (index = 0; index < BT_MAX_EVENT_NOTIFY_CB_NUM; index++)
    {
        if (cb == bt_handle->cb_arry.cb[index])
        {
            bt_handle->cb_arry.cb[index] = RT_NULL;
            bt_handle->cb_arry.size--;
            LOG_I("bt notify cb unregister pass:%d", bt_handle->cb_arry.size);
            break;
        }
    }
__exit:
    rt_mutex_release(bt_handle->handle_lock);
    return ret;
}


static bt_err_t _bt_close(struct rt_device *dev)
{
    struct rt_bt_device *bt_handle = RT_NULL;
    bt_err_t ret = BT_EOK;
    RT_ASSERT(dev != RT_NULL);
    bt_handle = (struct rt_bt_device *)dev;
    if (!(dev->open_flag & BT_DEVICE_FLAG_OPEN))
    {
        return ret;
    }
    //ret = rt_bt_control(dev, BT_CONTROL_CLOSE_DEVICE, RT_NULL);
    ret = bt_handle->ops->control(bt_handle, BT_CONTROL_CLOSE_DEVICE, RT_NULL);
    if (ret == RT_EOK)
    {
        dev->open_flag &= ~BT_DEVICE_FLAG_OPEN;
        //bt_device_fsm_handle(bt_handle, BT_CONTROL_CLOSE_DEVICE, RT_NULL);
    }
    LOG_I("rt_bt_close ret:%d flag:%X", ret, dev->open_flag);
    return ret;
}

static  bt_err_t _bt_open(struct rt_device *dev)
{
    struct rt_bt_device *bt_handle = RT_NULL;
    bt_err_t ret = BT_EOK;
    RT_ASSERT(dev != RT_NULL);
    bt_handle = (struct rt_bt_device *)dev;
    if (dev->open_flag & BT_DEVICE_FLAG_OPEN)
    {
        return ret;
    }
    ret = bt_handle->ops->control(bt_handle, BT_CONTROL_OPEN_DEVICE, RT_NULL);
    if (BT_EOK == ret)
    {
        dev->open_flag |= BT_DEVICE_FLAG_OPEN;
        bt_device_fsm_handle(bt_handle, BT_CONTROL_OPEN_DEVICE, RT_NULL);
    }
    //dev->open_flag &= ~BT_DEVICE_FLAG_OPEN;
    LOG_I("rt_bt_open ret:%d flag:%X", ret, dev->open_flag);
    return ret;
}

static void _bt_role_switch(rt_bt_device_t *dev, int cmd)
{
    switch (cmd)
    {
    case BT_CONTROL_SWITCH_TO_SOURCE:
    {
        dev->role = BT_ROLE_MASTER;
    }
    break;

    case BT_CONTROL_SWITCH_TO_SINK:
    {
        dev->role = BT_ROLE_SLAVE;
    }
    break;

    default:
        break;
    }
    return;
}


static rt_err_t _bt_control(struct rt_device *dev,
                            int              cmd,
                            void             *args)
{
    bt_err_t ret = BT_EOK;
    struct rt_bt_device *bt_handle;
    RT_ASSERT(dev != RT_NULL);
    bt_handle = (struct rt_bt_device *)dev;

    if ((RT_DEVICE_CTRL_RESUME == cmd) || (RT_DEVICE_CTRL_SUSPEND == cmd))
    {
        ret = bt_handle->ops->control(bt_handle, cmd, args);
        return ret;
    }

    rt_mutex_take(bt_handle->control_lock, RT_WAITING_FOREVER);
    switch (cmd)
    {
    case BT_CONTROL_REGISTER_NOTIFY:
    {
        bt_notify_cb cb = (bt_notify_cb)args;
        ret = register_notify_event_cb(bt_handle, cb);
    }
    break;

    case BT_CONTROL_UNREGISTER_NOTIFY:
    {
        bt_notify_cb cb = (bt_notify_cb)args;
        ret = unregister_notify_event_cb(bt_handle, cb);
    }
    break;

    case BT_CONTROL_OPEN_DEVICE:
    {
        ret = _bt_open(dev);
    }
    break;

    case BT_CONTROL_CLOSE_DEVICE:
    {
        ret = _bt_close(dev);
    }
    break;

    case BT_CONTROL_QUERY_STATE:
    {
        bt_state_t *pState = (bt_state_t *)args;
        *pState = rt_bt_get_device_state(bt_handle);
        if (!(dev->open_flag & BT_DEVICE_FLAG_OPEN))
        {
            ret = BT_ERROR_POWER_OFF;
        }
    }
    break;

    case BT_CONTROL_QUERY_STATE_EX:
    {
        bt_fsm_t *fsm = (bt_fsm_t *)args;
        *fsm = bt_handle->fsm;
    }
    break;

    case BT_CONTROL_SET_VOLUME:
    case BT_CONTROL_SET_LOCAL_NAME:
    case BT_CONTROL_CHANGE_BD_ADDR:
#ifdef BT_USING_A2DP
    case BT_CONTROL_SET_A2DP_SRC_AUDIO_DEVICE:
    case BT_CONTROL_UNREGISTER_AVSINK_SDP:
    case BT_CONTROL_REGISTER_AVSINK_SDP:
#endif
#ifdef BT_USING_SIFLI
    case BT_CONTROL_GET_BT_MAC:
#endif
#ifdef BT_USING_GATT
    case BT_CONTROL_BT_GATT_SDP_REG_REQ:
    case BT_CONTROL_BT_GATT_SDP_UNREG_REQ:
#endif
        ret = bt_handle->ops->control(bt_handle, cmd, args);
        break;

    default :
        /* control device */
    {
        if (!(dev->open_flag & BT_DEVICE_FLAG_OPEN))
        {
            rt_mutex_release(bt_handle->control_lock);
            return BT_ERROR_POWER_OFF;
        }
        ret = bt_handle->ops->control(bt_handle, cmd, args);
        if (BT_EOK == ret || BT_ERROR_IN_PROGRESS == ret)
        {
            bt_connect_fsm_handle(bt_handle, cmd, args);
#ifdef BT_USING_HF
            bt_call_event_hdl(bt_handle, cmd, args);
#endif
            bt_acl_fsm_handle(bt_handle, cmd, args);
            bt_device_fsm_handle(bt_handle, cmd, args);
            _bt_role_switch(bt_handle, cmd);
        }
    }
    break;
    }
    rt_mutex_release(bt_handle->control_lock);
    return ret;
}

bt_connect_state_t rt_bt_get_connect_state(rt_bt_device_t *dev, bt_profile_t profile)
{
    RT_ASSERT(dev);
    return (bt_connect_state_t)dev->fsm.connect_fsm[0][profile].state_current->data;
}

bt_role_t rt_bt_get_role(rt_bt_device_t *dev)
{
    RT_ASSERT(dev);
    return dev->role;
}


bt_acl_state_t rt_bt_get_acl_state(rt_bt_device_t *dev)
{
    RT_ASSERT(dev);
    return (bt_acl_state_t)dev->fsm.acl_fsm[0].state_current->data;
}

bt_state_t rt_bt_get_device_state(rt_bt_device_t *dev)
{
    RT_ASSERT(dev);
    return (bt_state_t)dev->fsm.device_fsm.state_current->data;
}

bt_media_state_t rt_bt_get_media_state(rt_bt_device_t *dev)
{
    RT_ASSERT(dev);
    return (bt_media_state_t)dev->fsm.media_fsm[0].state_current->data;
}

bt_connect_state_t rt_bt_get_connect_state_by_conn_idx(rt_bt_device_t *dev, uint8_t idx, bt_profile_t profile)
{
    RT_ASSERT(dev);
    if (idx >= BT_MAX_ACL_NUM)
    {
        return BT_STATE_CONNECT_ERROR;
    }
    return (bt_connect_state_t)dev->fsm.connect_fsm[idx][profile].state_current->data;
}


bt_acl_state_t rt_bt_get_acl_state_by_conn_idx(rt_bt_device_t *dev, uint8_t idx)
{
    RT_ASSERT(dev);

    if (idx >= BT_MAX_ACL_NUM)
    {
        return BT_STATE_ACL_ERROR;
    }
    return (bt_acl_state_t)dev->fsm.acl_fsm[idx].state_current->data;
}

bt_media_state_t rt_bt_get_media_state_by_conn_idx(rt_bt_device_t *dev, uint8_t idx)
{
    RT_ASSERT(dev);
    if (idx >= BT_MAX_ACL_NUM)
    {
        return BT_STATE_MEDIA_ERROR;
    }
    return (bt_media_state_t)dev->fsm.media_fsm[idx].state_current->data;
}


bt_call_state_t rt_bt_get_call_state(rt_bt_device_t *dev)
{
    RT_ASSERT(dev);

    if (BT_INVALID_CALL_IDX == dev->call_info.active_idx)
    {
        return BT_CALL_IDLE;
    }

    return (bt_call_state_t)dev->fsm.call_fsm[dev->call_info.active_idx].state_current->data;
}

bt_call_state_t rt_bt_get_call_state_by_idx(rt_bt_device_t *dev, uint8_t idx)
{
    RT_ASSERT(dev);

    if (idx >= BT_MAX_CALL_NUM)
    {
        return BT_CALL_IDLE;
    }

    return (bt_call_state_t)dev->fsm.call_fsm[idx].state_current->data;
}


bt_call_info_t *rt_bt_get_call_info(rt_bt_device_t *dev)
{
    RT_ASSERT(dev);
    rt_mutex_take(dev->call_sem, RT_WAITING_FOREVER);
    bt_call_info_t *call_info = &dev->call_info;
    rt_mutex_release(dev->call_sem);
    return call_info;
}

uint8_t rt_bt_get_hfp_sco_link(rt_bt_device_t *dev)
{
    RT_ASSERT(dev);
    return dev->fsm.sco_link;
}


#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops bt_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rt_bt_control
};
#endif

/*
 * bt register
 */
__ROM_USED rt_err_t rt_bt_register(struct rt_bt_device *dev_handle, const char *name)
{
    rt_err_t ret;
    struct rt_device *device;
    RT_ASSERT(dev_handle != RT_NULL);

    device = &(dev_handle->parent);

    device->type        = RT_Device_Class_Miscellaneous;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &bt_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = _bt_control;
#endif
    device->user_data   = RT_NULL;
    /* register a Miscellaneous device */
    ret = rt_device_register(device, name, RT_DEVICE_FLAG_RDWR);
    return ret;
}
