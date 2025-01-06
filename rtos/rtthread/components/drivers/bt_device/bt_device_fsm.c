/*
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>


#define DBG_TAG    "bt_device_fsm"
//#define DBG_LVL    DBG_INFO
#include <rtdbg.h>



static void bt_device_fsm_enter(void *state_data, struct event *event);
static void bt_device_fsm_exit(void *state_data, struct event *event);

static struct state bt_state_device_power_off;
static struct state bt_state_device_power_on;
static struct state bt_state_device_power_pair;
static struct state bt_state_device_search;
static struct state bt_state_device_other;

static struct state bt_state_device_power_off =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CONTROL_OPEN_DEVICE, NULL, NULL, NULL, &bt_state_device_power_on },
        {BT_CONTROL_DEVICE_INIT, NULL, NULL, NULL, &bt_state_device_power_on },
    },
    .transition_nums = 2,
    .data = (void *)BT_STATE_POWER_OFF,
    .action_entry = &bt_device_fsm_enter,
    .action_exti = &bt_device_fsm_exit,
};


static struct state bt_state_device_power_on =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        //{BT_CONTROL_CLOSE_DEVICE, NULL, NULL, NULL, &bt_state_device_power_off },
        {BT_CONTROL_SWITCH_ON, NULL, NULL, NULL, &bt_state_device_power_pair },
        {BT_CONTROL_SEARCH_EQUIPMENT, NULL, NULL, NULL, &bt_state_device_search },
        {BT_EVENT_CLOSE_COMPLETE, NULL, NULL, NULL, &bt_state_device_power_off },
        {BT_EVENT_DISCONNECT, NULL, NULL, NULL, &bt_state_device_power_pair },
    },
    .transition_nums = 4,
    .data = (void *)BT_STATE_POWER_ON,
    .action_entry = &bt_device_fsm_enter,
    .action_exti = &bt_device_fsm_exit,
};


static struct state bt_state_device_power_pair =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        //{BT_CONTROL_CLOSE_DEVICE, NULL, NULL, NULL, &bt_state_device_power_off},
        {BT_CONTROL_SEARCH_EQUIPMENT, NULL, NULL, NULL, &bt_state_device_search},
        {BT_CONTROL_SWITCH_OFF, NULL, NULL, NULL, &bt_state_device_power_on },
        {BT_EVENT_CLOSE_COMPLETE, NULL, NULL, NULL, &bt_state_device_power_off},
        {BT_EVENT_CONNECT_COMPLETE, NULL, NULL, NULL, &bt_state_device_power_on},
    },
    .transition_nums = 4,
    .data = (void *)BT_STATE_PAIR,
    .action_entry = &bt_device_fsm_enter,
    .action_exti = &bt_device_fsm_exit,
};


static struct state bt_state_device_search =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        //{BT_CONTROL_CLOSE_DEVICE, NULL, NULL, NULL, &bt_state_device_power_off },
        {BT_CONTROL_CANCEL_SEARCH, NULL, NULL, NULL, &bt_state_device_power_on },
        {BT_EVENT_CLOSE_COMPLETE, NULL, NULL, NULL, &bt_state_device_power_off },
        {BT_EVENT_INQ_FINISHED, NULL, NULL, NULL, &bt_state_device_power_on },
    },
    .transition_nums = 3,
    .data = (void *)BT_STATE_SEARCH,
    .action_entry = &bt_device_fsm_enter,
    .action_exti = &bt_device_fsm_exit,
};


static struct state bt_state_device_other =
{
    .transitions = NULL,
    .transition_nums = 0,
    .data = (void *)BT_STATE_OTHER,
    .action_entry = &bt_device_fsm_enter
};

static void bt_device_fsm_enter(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}


static void bt_device_fsm_exit(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}

static int bt_device_fsm_init(void)
{
    rt_bt_device_t *bt_device = (rt_bt_device_t *) rt_device_find(BT_DEVICE_NAME);
    if (RT_NULL == bt_device)
    {
        LOG_E("init bt device fsm fail\n");
        return RT_ERROR;
    }

    statem_init(&bt_device->fsm.device_fsm, &bt_state_device_power_off, &bt_state_device_other);
    return BT_EOK;
}


int bt_device_fsm_handle(rt_bt_device_t *dev, int event_type, void *args)
{
    int ret = STATEM_STATE_NOCHANGE;
    struct event connect_event;
    connect_event.type = event_type;
    switch (event_type)
    {
    case BT_CONTROL_OPEN_DEVICE:
    case BT_CONTROL_CLOSE_DEVICE:
    case BT_CONTROL_SWITCH_ON:
    case BT_CONTROL_SWITCH_OFF:
    case BT_CONTROL_CANCEL_SEARCH:
    case BT_CONTROL_SEARCH_EQUIPMENT:
    case BT_CONTROL_DEVICE_INIT:
    case BT_EVENT_CLOSE_COMPLETE:
    case BT_EVENT_DISCONNECT:
    case BT_EVENT_CONNECT_COMPLETE:
    {
        ret =  statem_handle_event(&dev->fsm.device_fsm, &connect_event);
    }
    break;

    default:
        break;
    }
    return ret ;
}

int bt_fsm_init(void)
{
    bt_device_fsm_init();
    bt_connect_fsm_init();
    bt_acl_fsm_init();
    bt_media_fsm_init();
#ifdef BT_USING_HF
    bt_call_fsm_init();
#endif
    return BT_EOK;
}

INIT_ENV_EXPORT(bt_fsm_init);