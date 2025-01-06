/*
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#if defined(BT_USING_SIFLI)
    #include "bts2_global.h"
    #include "bts2_app_inc.h"
    #include "hfp_audio_api.h"
#endif


#define DBG_TAG    "bt_call_fsm"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>



static void bt_call_fsm_enter(void *state_data, struct event *event);
static void bt_call_fsm_exit(void *state_data, struct event *event);

static struct state bt_state_call_idle;
static struct state bt_state_call_incoming;
static struct state bt_state_call_outgoing_dailing;
static struct state bt_state_call_outgoing_alerting;
static struct state bt_state_call_active;
static struct state bt_state_call_waiting;
static struct state bt_state_call_onhold;
static struct state bt_state_call_error;



static struct state bt_state_call_idle =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CALL_IDLE, NULL, NULL, NULL, &bt_state_call_idle },
        {BT_CALL_INCOMING, NULL, NULL, NULL, &bt_state_call_incoming },
        {BT_CALL_OUTGOING_DAILING, NULL, NULL, NULL, &bt_state_call_outgoing_dailing },
        {BT_CALL_OUTGOING_ALERTING, NULL, NULL, NULL, &bt_state_call_outgoing_alerting },
        {BT_CALL_ACTIVE, NULL, NULL, NULL, &bt_state_call_active },
        {BT_CALL_ONHOLD, NULL, NULL, NULL, &bt_state_call_onhold },
        {BT_CALL_WAITING, NULL, NULL, NULL, &bt_state_call_waiting },
    },
    .transition_nums = 7,
    .data = (void *)BT_CALL_IDLE,
    .action_entry = &bt_call_fsm_enter,
    .action_exti = &bt_call_fsm_exit,
};


static struct state bt_state_call_incoming =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CALL_IDLE, NULL, NULL, NULL, &bt_state_call_idle },
        {BT_CALL_ACTIVE, NULL, NULL, NULL, &bt_state_call_active },
        {BT_CALL_WAITING, NULL, NULL, NULL, &bt_state_call_waiting },
    },
    .transition_nums = 3,
    .data = (void *)BT_CALL_INCOMING,
    .action_entry = &bt_call_fsm_enter,
    .action_exti = &bt_call_fsm_exit,
};

static struct state bt_state_call_outgoing_dailing =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CALL_IDLE, NULL, NULL, NULL, &bt_state_call_idle },
        {BT_CALL_OUTGOING_ALERTING, NULL, NULL, NULL, &bt_state_call_outgoing_alerting },
        {BT_CALL_ACTIVE, NULL, NULL, NULL, &bt_state_call_active },
    },
    .transition_nums = 3,
    .data = (void *)BT_CALL_OUTGOING_DAILING,
    .action_entry = &bt_call_fsm_enter,
    .action_exti = &bt_call_fsm_exit,
};


static struct state bt_state_call_outgoing_alerting =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CALL_IDLE, NULL, NULL, NULL, &bt_state_call_idle },
        {BT_CALL_ACTIVE, NULL, NULL, NULL, &bt_state_call_active },
    },
    .transition_nums = 2,
    .data = (void *)BT_CALL_OUTGOING_ALERTING,
    .action_entry = &bt_call_fsm_enter,
    .action_exti = &bt_call_fsm_exit,
};

static struct state bt_state_call_active =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CALL_IDLE, NULL, NULL, NULL, &bt_state_call_idle },
        {BT_CALL_ONHOLD, NULL, NULL, NULL, &bt_state_call_onhold },
    },
    .transition_nums = 2,
    .data = (void *)BT_CALL_ACTIVE,
    .action_entry = &bt_call_fsm_enter,
    .action_exti = &bt_call_fsm_exit,
};


static struct state bt_state_call_waiting =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CALL_IDLE, NULL, NULL, NULL, &bt_state_call_idle },
        {BT_CALL_ACTIVE, NULL, NULL, NULL, &bt_state_call_active },
    },
    .transition_nums = 2,
    .data = (void *)BT_CALL_WAITING,
    .action_entry = &bt_call_fsm_enter,
    .action_exti = &bt_call_fsm_exit,
};

static struct state bt_state_call_onhold =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_CALL_IDLE, NULL, NULL, NULL, &bt_state_call_idle },
        {BT_CALL_ACTIVE, NULL, NULL, NULL, &bt_state_call_active },
    },
    .transition_nums = 2,
    .data = (void *)BT_CALL_ONHOLD,
    .action_entry = &bt_call_fsm_enter,
    .action_exti = &bt_call_fsm_exit,
};

static struct state bt_state_call_error =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = NULL,
    .transition_nums = 0,
    .data = (void *)BT_CALL_ERROR,
    .action_entry = &bt_call_fsm_enter,
};



static void bt_call_fsm_enter(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}


static void bt_call_fsm_exit(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}

char *bt_call_state_to_name(bt_call_state_t state)
{
#define CALL_STATE_TO_NAME_CASE(e) case e: return #e
    switch (state)
    {
        CALL_STATE_TO_NAME_CASE(BT_CALL_IDLE);

        CALL_STATE_TO_NAME_CASE(BT_CALL_INCOMING);
        CALL_STATE_TO_NAME_CASE(BT_CALL_OUTGOING_DAILING);
        CALL_STATE_TO_NAME_CASE(BT_CALL_OUTGOING_ALERTING);
        CALL_STATE_TO_NAME_CASE(BT_CALL_ACTIVE);
        CALL_STATE_TO_NAME_CASE(BT_CALL_WAITING);
        CALL_STATE_TO_NAME_CASE(BT_CALL_ONHOLD);
        CALL_STATE_TO_NAME_CASE(BT_CALL_ERROR);
    }
}


int bt_call_fsm_handle(rt_bt_device_t *dev, uint8_t index, int event_type, void *args)
{
    int ret = STATEM_STATE_NOCHANGE;
    struct event connect_event;
    connect_event.type = 0xFF;

    switch (event_type)
    {

    case BT_CONTROL_CLOSE_DEVICE:
    case BT_EVENT_DISCONNECT:
    {
        connect_event.type = BT_CALL_IDLE;
    }
    break;

    case BT_EVENT_CALL_STATUS_IND:
    {
        connect_event.type = *((bt_call_state_t *)args);
    }
    break;

    default:
        break;
    }

    if ((BT_INVALID_CALL_IDX != index) && (0xFF != connect_event.type))
    {
        rt_kprintf("bt_call_fsm_handle idx:%d pre_state:%s cur_state:%s new:%s\n", index,
                   bt_call_state_to_name((bt_call_state_t)dev->fsm.call_fsm[index].state_previous->data),
                   bt_call_state_to_name((bt_call_state_t)dev->fsm.call_fsm[index].state_current->data),
                   bt_call_state_to_name(connect_event.type));
        ret =  statem_handle_event(&dev->fsm.call_fsm[index], &connect_event);
    }
    return ret ;
}

int bt_call_fsm_init(void)
{
    rt_bt_device_t *bt_device = (rt_bt_device_t *) rt_device_find(BT_DEVICE_NAME);
    if (RT_NULL == bt_device)
    {
        LOG_E("init bt call fsm fail\n");
        return RT_ERROR;
    }

#if defined(AUDIO_USING_MANAGER)
    hfp_audio_init();
#endif

    rt_memset(&bt_device->call_info, 0, sizeof(bt_call_info_t));

    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        statem_init(&bt_device->fsm.call_fsm[i], &bt_state_call_idle, &bt_state_call_error);
        bt_device->fsm.call_fsm[i].state_previous = &bt_state_call_idle;
    }
    return BT_EOK;
}

