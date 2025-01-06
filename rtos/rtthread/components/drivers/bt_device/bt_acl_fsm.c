/*
 */


#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "bt_acl_fsm"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

static void bt_acl_fsm_enter(void *state_data, struct event *event);
static void bt_acl_fsm_exit(void *state_data, struct event *event);

static struct state bt_state_acl_idle, bt_state_acl_disconnecting, bt_state_acl_connected, \
    bt_state_acl_error;

static struct state bt_state_acl_idle =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_EVENT_ACL_OPENED_IND, NULL, NULL, NULL, &bt_state_acl_connected },
    },
    .transition_nums = 1,
    .data = (void *)BT_STATE_ACL_IDLE,
    .action_entry = &bt_acl_fsm_enter,
    .action_exti = &bt_acl_fsm_exit,
};


static struct state bt_state_acl_disconnecting =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_EVENT_DISCONNECT, NULL, NULL, NULL, &bt_state_acl_idle },
        {BT_EVENT_CLOSE_COMPLETE, NULL, NULL, NULL, &bt_state_acl_idle },
    },
    .transition_nums = 2,
    .data = (void *)BT_STATE_ACL_DISCONNECTING,
    .action_entry = &bt_acl_fsm_enter,
    .action_exti = &bt_acl_fsm_exit,
};

static struct state bt_state_acl_connected =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CONTROL_DISCONNECT, NULL, NULL, NULL, &bt_state_acl_disconnecting},
        {BT_CONTROL_CANCEL_PAGE, NULL, NULL, NULL, &bt_state_acl_disconnecting},
        {BT_EVENT_DISCONNECT, NULL, NULL, NULL, &bt_state_acl_idle },
        {BT_EVENT_CLOSE_COMPLETE, NULL, NULL, NULL, &bt_state_acl_idle },
    },
    .transition_nums = 4,
    .data = (void *)BT_STATE_ACL_CONNECTED,
    .action_entry = &bt_acl_fsm_enter,
    .action_exti = &bt_acl_fsm_exit,
};

static struct state bt_state_acl_error =
{
    .transitions = NULL,
    .transition_nums = 0,
    .data = (void *)BT_STATE_ACL_ERROR,
    .action_entry = &bt_acl_fsm_enter
};

static void bt_acl_fsm_enter(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}


static void bt_acl_fsm_exit(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}

int bt_acl_fsm_handle(rt_bt_device_t *dev, int event_type, void *args)
{
    int ret = STATEM_STATE_NOCHANGE;
    struct event connect_event;
    connect_event.type = event_type;
    switch (event_type)
    {

    case BT_EVENT_DISCONNECT:
    {
        bt_acl_disconnect_info_t *info = (bt_acl_disconnect_info_t *)args;
        if (0xFF != info->conn_idx)
        {
            ret =  statem_handle_event(&dev->fsm.acl_fsm[info->conn_idx], &connect_event);
        }
        else
        {
            LOG_E("invalid conn index:%x", event_type);
        }
    }
    break;

    case BT_CONTROL_CANCEL_PAGE:
    case BT_CONTROL_DISCONNECT:
    case BT_EVENT_CLOSE_COMPLETE:
    {
        for (uint8_t i = 0; i < BT_MAX_ACL_NUM; i++)
        {
            ret =  statem_handle_event(&dev->fsm.acl_fsm[i], &connect_event);
        }
    }
    break;

    case BT_EVENT_ACL_OPENED_IND:
    {
        bt_acl_opened_t *ind = (bt_acl_opened_t *)args;
        if (0x00 == ind->res)
        {
            if (0xFF != ind->conn_idx)
            {
                ret =  statem_handle_event(&dev->fsm.acl_fsm[ind->conn_idx], &connect_event);
            }
        }
    }
    break;

    default:
        break;
    }
    return ret ;
}

int bt_acl_fsm_init(void)
{
    rt_bt_device_t *bt_device = (rt_bt_device_t *) rt_device_find(BT_DEVICE_NAME);
    if (RT_NULL == bt_device)
    {
        LOG_E("init bt connect fsm fail\n");
        return RT_ERROR;
    }

    for (uint8_t i = 0; i < BT_MAX_ACL_NUM; i++)
    {
        statem_init(&bt_device->fsm.acl_fsm[i], &bt_state_acl_idle, &bt_state_acl_error);
    }
    return BT_EOK;
}