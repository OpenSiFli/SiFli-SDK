/*
 */


#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "bt_media_fsm"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

static void bt_media_fsm_enter(void *state_data, struct event *event);
static void bt_media_fsm_exit(void *state_data, struct event *event);

static struct state bt_state_media_idle, bt_state_media_play, bt_state_media_pause, \
    bt_state_media_error;

static struct state bt_state_media_idle =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_STATE_MEDIA_PAUSE, NULL, NULL, NULL, &bt_state_media_pause },
        {BT_STATE_MEDIA_PLAY, NULL, NULL, NULL, &bt_state_media_play },
    },
    .transition_nums = 2,
    .data = (void *)BT_STATE_MEDIA_IDLE,
    .action_entry = &bt_media_fsm_enter,
    .action_exti = &bt_media_fsm_exit,
};


static struct state bt_state_media_play =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_STATE_MEDIA_PAUSE, NULL, NULL, NULL, &bt_state_media_pause },
        {BT_STATE_MEDIA_IDLE, NULL, NULL, NULL, &bt_state_media_idle },
    },
    .transition_nums = 2,
    .data = (void *)BT_STATE_MEDIA_PLAY,
    .action_entry = &bt_media_fsm_enter,
    .action_exti = &bt_media_fsm_exit,
};

static struct state bt_state_media_pause =
{
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[])
    {
        {BT_STATE_MEDIA_PLAY, NULL, NULL, NULL, &bt_state_media_play},
        {BT_STATE_MEDIA_IDLE, NULL, NULL, NULL, &bt_state_media_idle},
    },
    .transition_nums = 2,
    .data = (void *)BT_STATE_MEDIA_PAUSE,
    .action_entry = &bt_media_fsm_enter,
    .action_exti = &bt_media_fsm_exit,
};

static struct state bt_state_media_error =
{
    .transitions = NULL,
    .transition_nums = 0,
    .data = (void *)BT_STATE_MEDIA_ERROR,
    .action_entry = &bt_media_fsm_enter
};

static void bt_media_fsm_enter(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}


static void bt_media_fsm_exit(void *state_data, struct event *event)
{
    LOG_D("%s state:%d event:%x", __func__, (int)state_data, event->type);
}

int bt_media_fsm_handle(rt_bt_device_t *dev, int event_type, void *args)
{
    int ret = STATEM_STATE_NOCHANGE;
    struct event connect_event;
    rt_bool_t handle_event = RT_FALSE;
    connect_event.type = BT_STATE_MEDIA_IDLE;

    switch (event_type)
    {
    case BT_EVENT_DISCONNECT:
    {
        bt_acl_disconnect_info_t *info = (bt_acl_disconnect_info_t *)args;
        if (0xFF != info->conn_idx)
        {
            ret =  statem_handle_event(&dev->fsm.media_fsm[info->conn_idx], &connect_event);
        }
        else
        {
            LOG_E("invalid conn index:%x", event_type);
        }
    }
    break;

    case BT_EVENT_CLOSE_COMPLETE:
    {
        for (uint8_t index = 0; index < BT_MAX_ACL_NUM; index++)
        {
            ret = statem_handle_event(&dev->fsm.media_fsm[index], &connect_event);
        }
    }
    break;


    case BT_EVENT_PROFILE_DISCONNECT:
    {
        uint8_t conn_index = 0;
        bt_disconnect_info_t *info = (bt_disconnect_info_t *) args;
        if (BT_PROFILE_A2DP == info->profile)
        {
            if (0xFF != info->conn_idx)
            {
                ret = statem_handle_event(&dev->fsm.media_fsm[conn_index], &connect_event);
            }
            else
            {
                LOG_E("invalid conn index:%x", event_type);
            }
        }
    }
    break;

#ifdef BT_USING_AVRCP
    case BT_EVENT_MUSIC_PLAY_STATUS_CHANGED:
    {
        uint8_t conn_index = 0;
        bt_media_play_status_t *play_status = (bt_media_play_status_t *)args;
        if (0x00 == play_status->status)
        {
            connect_event.type = BT_STATE_MEDIA_PLAY;
        }
        else
        {
            connect_event.type = BT_STATE_MEDIA_PAUSE;
        }
        ret = statem_handle_event(&dev->fsm.media_fsm[conn_index], &connect_event);

    }
    break;
#endif

    default:
        break;
    }

    return ret ;
}

int bt_media_fsm_init(void)
{
    rt_bt_device_t *bt_device = (rt_bt_device_t *) rt_device_find(BT_DEVICE_NAME);
    if (RT_NULL == bt_device)
    {
        LOG_E("init bt media fsm fail\n");
        return RT_ERROR;
    }

    for (uint8_t index = 0; index < BT_MAX_ACL_NUM; index++)
    {
        statem_init(&bt_device->fsm.media_fsm[index], &bt_state_media_idle, &bt_state_media_error);
    }
    return BT_EOK;
}