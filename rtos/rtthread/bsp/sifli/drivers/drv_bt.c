

#include <stdio.h>
#include <string.h>
#include "drv_bt.h"
#define DBG_TAG    "drv_bt"
//#define DBG_LVL    DBG_INFO
#include <rtdbg.h>
#include "bt_device.h"



#ifdef RT_USING_BT
struct rt_bt_device bt_obj = {0};


static void bt_event_notify(bt_notify_t *param)
{
    int index = 0;
    if (0 == bt_obj.cb_arry.size)
    {
        return;
    }

    rt_mutex_take(bt_obj.handle_lock, RT_WAITING_FOREVER);
    bt_notify_cb_array_t cb_arry;
    rt_memcpy(&cb_arry, &bt_obj.cb_arry, sizeof(bt_notify_cb_array_t));
    rt_mutex_release(bt_obj.handle_lock);

    for (index = 0; index < BT_MAX_EVENT_NOTIFY_CB_NUM; index++)
    {
        if (RT_NULL != cb_arry.cb[index])
        {
            cb_arry.cb[index](param);
        }
    }

    return;
}


/* event upload for bt at */
__ROM_USED void rt_bt_event_notify(bt_notify_t *param)
{
    if (RT_NULL == bt_obj.handle_lock)
    {
        return;
    }

    switch (param->event)
    {
    case BT_EVENT_CANCEL_PAGE_IND:
    case BT_EVENT_DISCONNECT:
    {
        bt_disconnect_info_t info = {0};
        for (uint8_t i = 0; i < BT_PROFILE_MAX; i++)
        {
            if (BT_STATE_CONNECTING == rt_bt_get_connect_state(&bt_obj, i))
            {
                info.profile = i;
                bt_connect_fsm_handle(&bt_obj, BT_EVENT_PROFILE_DISCONNECT, &info);
            }
        }
    }
    break;

    case BT_EVENT_CLOSE_COMPLETE:
    case BT_EVENT_PROFILE_DISCONNECT:
    case BT_EVENT_CONNECT_COMPLETE:
    {
        bt_connect_fsm_handle(&bt_obj, param->event, param->args);
    }
    break;

    case BT_EVENT_BT_STACK_READY:
    {
        if (bt_obj.fsm.stack_ready)
        {
            return;
        }

        bt_obj.fsm.stack_ready = 1;
    }
    break;

#ifdef BT_USING_SIRI
    case BT_EVENT_SIRI_ON_COMPLETE:
    {
        bt_obj.fsm.siri_status = 1;
    }
    break;

    case BT_EVENT_SIRI_OFF_COMPLETE:
    {
        bt_obj.fsm.siri_status = 0;
    }
    break;
#endif

    default:
        break;
    }
    bt_media_fsm_handle(&bt_obj, param->event, param->args);
#ifdef BT_USING_HF
    bt_call_event_hdl(&bt_obj, param->event, param->args);
#endif
    bt_event_notify(param);
    bt_acl_fsm_handle(&bt_obj, param->event, param->args);
    bt_device_fsm_handle(&bt_obj, param->event, param->args);

    return;
}


int rt_hw_bt_init(const struct rt_bt_ops *ops, rt_uint16_t open_flag)
{
    rt_err_t result = 0;
    bt_obj.ops = ops;
    rt_memset(&bt_obj.cb_arry, 0x00, sizeof(bt_notify_cb_array_t));
    bt_obj.handle_lock = rt_mutex_create("bt_notify", RT_IPC_FLAG_FIFO);
    if (RT_NULL == bt_obj.handle_lock)
    {
        rt_kprintf("bt handle_lock create fail\n");
        return RT_ERROR;
    }

    bt_obj.control_lock = rt_mutex_create("bt_control", RT_IPC_FLAG_FIFO);
    if (RT_NULL == bt_obj.control_lock)
    {
        rt_kprintf("bt handle_lock create fail\n");
        return RT_ERROR;
    }

    bt_obj.call_sem = rt_mutex_create("call_sem", RT_IPC_FLAG_FIFO);
    RT_ASSERT(bt_obj.call_sem);


    bt_obj.fsm.stack_ready = 0;
    bt_obj.fsm.siri_status = 0;
    bt_obj.fsm.clcc_process_status = CALL_CLCC_COMPLETE;
    bt_obj.role = BT_ROLE_SLAVE;
    bt_obj.config.inband_ring = 1;
    bt_obj.call_info.active_idx = BT_INVALID_CALL_IDX;

    result = rt_bt_register(&bt_obj, BT_DEVICE_NAME);
    RT_ASSERT(result == RT_EOK);

    if ((open_flag & BT_DEVICE_FLAG_OPEN))
    {
        bt_obj.ops->control(&bt_obj, BT_CONTROL_DEVICE_INIT, RT_NULL);
        bt_device_fsm_handle(&bt_obj, BT_CONTROL_DEVICE_INIT, RT_NULL);
        bt_obj.parent.open_flag = open_flag;
    }
    else
    {
        bt_obj.ops->control(&bt_obj, BT_CONTROL_DEVICE_DEINIT, RT_NULL);
    }
    return result;
}

#endif /* RT_USING_BT */

