/*
 */


#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "bt_connect_fsm"
//#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

static void bt_connect_fsm_enter(void *state_data, struct event *event);
static void bt_connect_fsm_exit(void *state_data, struct event *event);

static struct state bt_state_connect_idle, bt_state_disconnecting, bt_state_connected, \
    bt_state_connecting, bt_state_connect_error;

static struct state bt_state_connect_idle =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CONTROL_CONNECT_DEVICE_EX, NULL, NULL, NULL, &bt_state_connecting },
        {BT_EVENT_CONNECT_COMPLETE, NULL, NULL, NULL, &bt_state_connected },
    },
    .transition_nums = 2,
    .data = (void *)BT_STATE_CONNECT_IDLE,
    .action_entry = &bt_connect_fsm_enter,
    .action_exti = &bt_connect_fsm_exit,
};


static struct state bt_state_disconnecting =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_EVENT_PROFILE_DISCONNECT, NULL, NULL, NULL, &bt_state_connect_idle },
        {BT_EVENT_CLOSE_COMPLETE, NULL, NULL, NULL, &bt_state_connect_idle },
        {BT_EVENT_DISCONNECT, NULL, NULL, NULL, &bt_state_connect_idle },
    },
    .transition_nums = 3,
    .data = (void *)BT_STATE_DISCONNECTING,
    .action_entry = &bt_connect_fsm_enter,
    .action_exti = &bt_connect_fsm_exit,
};

static struct state bt_state_connected =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_EVENT_PROFILE_DISCONNECT, NULL, NULL, NULL, &bt_state_connect_idle},
        {BT_CONTROL_DISCONNECT, NULL, NULL, NULL, &bt_state_disconnecting},
        {BT_CONTROL_DISCONNECT_EX, NULL, NULL, NULL, &bt_state_disconnecting},
        {BT_CONTROL_DISCONNECT_BY_CONNIDX, NULL, NULL, NULL, &bt_state_disconnecting},
        {BT_EVENT_CLOSE_COMPLETE, NULL, NULL, NULL, &bt_state_connect_idle },
        {BT_EVENT_DISCONNECT, NULL, NULL, NULL, &bt_state_connect_idle },

    },
    .transition_nums = 6,
    .data = (void *)BT_STATE_CONNECTED,
    .action_entry = &bt_connect_fsm_enter,
    .action_exti = &bt_connect_fsm_exit,
};

static struct state bt_state_connecting =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_EVENT_CONNECT_COMPLETE, NULL, NULL, NULL, &bt_state_connected },
        {BT_EVENT_PROFILE_DISCONNECT, NULL, NULL, NULL, &bt_state_connect_idle },
        {BT_EVENT_CLOSE_COMPLETE, NULL, NULL, NULL, &bt_state_connect_idle },
        {BT_EVENT_DISCONNECT, NULL, NULL, NULL, &bt_state_connect_idle },
        {BT_CONTROL_DISCONNECT_BY_CONNIDX, NULL, NULL, NULL, &bt_state_connect_idle},
    },
    .transition_nums = 5,
    .data = (void *)BT_STATE_CONNECTING,
    .action_entry = &bt_connect_fsm_enter,
    .action_exti = &bt_connect_fsm_exit,
};

static struct state bt_state_connect_error =
{
    .transitions = NULL,
    .transition_nums = 0,
    .data = (void *)BT_STATE_CONNECT_ERROR,
    .action_entry = &bt_connect_fsm_enter
};

static void bt_connect_fsm_enter(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}


static void bt_connect_fsm_exit(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}

int bt_connect_fsm_handle(rt_bt_device_t *dev, int event_type, void *args)
{
    int ret = STATEM_STATE_NOCHANGE;
    struct event connect_event;
    connect_event.type = event_type;
    rt_mutex_take(dev->control_lock, RT_WAITING_FOREVER);
    switch (event_type)
    {
    case BT_CONTROL_DISCONNECT_EX:
    {
        ret =  statem_handle_event(&dev->fsm.connect_fsm[0][*((bt_profile_t *)args)], &connect_event);
    }
    break;

    case BT_EVENT_PROFILE_DISCONNECT:
    {
        bt_disconnect_info_t *info = (bt_disconnect_info_t *)args;
        if (0xFF != info->conn_idx)
        {
            ret =  statem_handle_event(&dev->fsm.connect_fsm[info->conn_idx][info->profile], &connect_event);
        }
        else
        {
            LOG_E("invalid conn index:%x", event_type);
        }
    }
    break;

    case BT_EVENT_CLOSE_COMPLETE:
    case BT_CONTROL_DISCONNECT:
    {
        for (uint8_t index = 0; index < BT_MAX_ACL_NUM; index++)
        {
            for (uint8_t i = 0; i < BT_PROFILE_MAX; i++)
                ret =  statem_handle_event(&dev->fsm.connect_fsm[index][i], &connect_event);
        }
    }
    break;

    case BT_CONTROL_DISCONNECT_BY_CONNIDX:
    {
        uint8_t *idx = (uint8_t *)args;
        if (0xFF != *idx)
        {
            for (uint8_t i = 0; i < BT_PROFILE_MAX; i++)
                ret =  statem_handle_event(&dev->fsm.connect_fsm[*idx][i], &connect_event);
        }
        else
        {
            LOG_E("invalid conn index:%x", event_type);
        }
    }
    break;


    case BT_EVENT_DISCONNECT:
    {
        bt_acl_disconnect_info_t *info = (bt_acl_disconnect_info_t *)args;
        if (0xFF != info->conn_idx)
        {
            for (uint8_t i = 0; i < BT_PROFILE_MAX; i++)
                ret =  statem_handle_event(&dev->fsm.connect_fsm[info->conn_idx][i], &connect_event);
        }
        else
        {
            LOG_E("invalid conn index:%x", event_type);
        }
    }
    break;

    case BT_EVENT_CONNECT_COMPLETE:
    case BT_CONTROL_CONNECT_DEVICE_EX:
    {
        bt_connect_info_t *info = (bt_connect_info_t *)args;

        if (0xFF != info->conn_idx)
        {
            ret =  statem_handle_event(&dev->fsm.connect_fsm[info->conn_idx][info->profile], &connect_event);
        }
        else
        {
            LOG_E("invalid conn index:%x", event_type);
        }
    }
    break;

    default:
        break;
    }
    rt_mutex_release(dev->control_lock);
    return ret ;
}

int bt_connect_fsm_init(void)
{
    rt_bt_device_t *bt_device = (rt_bt_device_t *) rt_device_find(BT_DEVICE_NAME);
    if (RT_NULL == bt_device)
    {
        LOG_E("init bt connect fsm fail\n");
        return RT_ERROR;
    }

    for (uint8_t index = 0; index < BT_MAX_ACL_NUM; index++)
    {
        for (uint8_t profile = 0; profile < BT_PROFILE_MAX; profile++)
        {
            statem_init(&bt_device->fsm.connect_fsm[index][profile], &bt_state_connect_idle, &bt_state_connect_error);
        }
    }
    return BT_EOK;
}