#include <rtthread.h>
#include <rtdevice.h>

#if defined(BT_USING_SIFLI)
    #include "bts2_global.h"
    #include "bts2_app_inc.h"
    #include "hfp_audio_api.h"
#endif

#define DBG_TAG    "bt_call_message"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

static bt_call_info_t pre_call_info = {0};
static rt_timer_t audio_open_timer = NULL;
static rt_timer_t  clcc_timer_hdl = NULL;

static void bt_call_start_get_clcc(rt_bt_device_t *dev);
static void bt_call_active_update(rt_bt_device_t *dev);

static uint8_t bt_get_call_active_idx(rt_bt_device_t *dev)
{
    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        if (BT_CALL_IDLE != (bt_call_state_t) dev->fsm.call_fsm[i].state_current->data)
        {
            return i;
        }
    }
    return BT_INVALID_CALL_IDX;
}

static uint8_t bt_get_call_idx_by_state(rt_bt_device_t *dev, bt_call_state_t state)
{
    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        if (state == (bt_call_state_t)dev->fsm.call_fsm[i].state_current->data)
        {
            return i;
        }
    }
    return BT_INVALID_CALL_IDX;
}


static uint8_t bt_get_call_idx_by_number(rt_bt_device_t *dev, uint8_t *number, uint8_t size)
{
    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        if (0 == size)   continue;

        if (!rt_memcmp(number, dev->call_info.phone_number[i].number, size))
        {
            return i;
        }
    }
    return BT_INVALID_CALL_IDX;
}


static void bt_call_info_clear(rt_bt_device_t *dev)
{
    rt_memset(&dev->call_info, 0x00, sizeof(bt_call_info_t));
    dev->call_info.active_idx = BT_INVALID_CALL_IDX;
    return;
}

static void bt_call_fsm_clear(rt_bt_device_t *dev)
{
    bt_call_state_t call_state = BT_CALL_IDLE;
    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        bt_call_fsm_handle(dev, i, BT_EVENT_CALL_STATUS_IND, &call_state);
    }
    return;
}

static void bt_call_reset(rt_bt_device_t *dev)
{
    if (CALL_CLCC_IN_PROGRESS == dev->fsm.clcc_process_status)
    {
        rt_mutex_release(dev->call_sem);
    }
    bt_call_info_clear(dev);
    bt_call_fsm_clear(dev);
    dev->fsm.sco_link = 0;
    rt_memset(&pre_call_info, 0x00, sizeof(bt_call_info_t));
    if (clcc_timer_hdl) rt_timer_stop(clcc_timer_hdl);
    if (audio_open_timer) rt_timer_stop(audio_open_timer);
    dev->fsm.clcc_process_status = CALL_CLCC_COMPLETE;
    return;
}


static uint8_t is_valid_bt_call(rt_bt_device_t *dev)
{
    uint8_t invalid_count = 0;
    bt_call_state_t call_state;
    bt_call_state_t pre_call_state;

    if (BT_INVALID_CALL_IDX != dev->call_info.active_idx)
    {
        return 1;
    }

    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        call_state = (bt_call_state_t)dev->fsm.call_fsm[i].state_current->data;
        pre_call_state = (bt_call_state_t)dev->fsm.call_fsm[i].state_previous->data;
        if ((BT_CALL_IDLE == call_state) && (BT_CALL_IDLE == pre_call_state))
        {
            invalid_count++;
        }
    }

    if (BT_MAX_CALL_NUM == invalid_count)
    {
        return 0;
    }
    return 1;
}


static uint8_t bt_call_get_direct_audio_on(rt_bt_device_t *dev)
{
    uint8_t direct_audio_on = 0;
    bt_call_state_t call_state = rt_bt_get_call_state(dev);

    if (!dev->fsm.sco_link)
    {
        return 0;
    }

    switch (call_state)
    {
    case BT_CALL_INCOMING:
    {
        if (dev->call_info.ring_type)
        {
            direct_audio_on = 1;
        }
        break;
    }

    default:
        direct_audio_on = 1;
        break;
    }
    return direct_audio_on;
}

static void bt_call_audio_open_timeout(void      *parameter)
{
    rt_bt_device_t *dev = (rt_bt_device_t *)parameter;
    bt_call_state_t call_state = rt_bt_get_call_state(dev);
#if defined(AUDIO_USING_MANAGER)
    if (bt_call_get_direct_audio_on(dev))
    {
        hfp_aduio_open_path(AUDIO_TYPE_BT_VOICE);
    }
#endif
    return;
}

static void bt_call_audio_delay_open(rt_bt_device_t *dev)
{
    if (NULL == audio_open_timer)
    {
        audio_open_timer = rt_timer_create("bt_audio_open", bt_call_audio_open_timeout, dev,
                                           rt_tick_from_millisecond(300), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }

    RT_ASSERT(audio_open_timer);
    rt_timer_stop(audio_open_timer);
    rt_timer_start(audio_open_timer);
    return;
}


static void bt_call_audio_open(rt_bt_device_t *dev, uint8_t set_audio)
{
    bt_call_state_t call_state = rt_bt_get_call_state(dev);
    uint8_t direct_audio_on = 0;

    if (set_audio)
    {
        if ((CALL_CLCC_COMPLETE == dev->fsm.clcc_process_status) && bt_call_get_direct_audio_on(dev))
        {
            direct_audio_on = 1;
        }

#if defined(AUDIO_USING_MANAGER)
        bts2_app_stru *bts2_app_data = getApp();
        BTS2S_HF_AUDIO_INFO *msg = (BTS2S_HF_AUDIO_INFO *)bts2_app_data->recv_msg;
        hfp_set_audio_voice_para(msg, msg->audio_on, direct_audio_on);
#endif
        if (BT_CALL_ACTIVE == call_state)
        {
            bt_call_audio_delay_open(dev);
        }
    }
    else if (dev->fsm.sco_link)
    {
        bt_call_audio_delay_open(dev);
    }
    return;
}

static void bt_call_audio_close(void)
{
    if (NULL != audio_open_timer)
    {
        rt_timer_stop(audio_open_timer);
    }

#if defined(AUDIO_USING_MANAGER)
    hfp_audio_close_path();
#endif // AUDIO_USING_MANAGER
    return;
}


static void bt_call_status_notify(rt_bt_device_t *dev)
{
    bt_notify_t args;
    bt_call_state_t call_state = rt_bt_get_call_state(dev);
    args.event = BT_EVENT_CALL_STATUS_IND;
    args.args = &call_state;

#if defined(AUDIO_USING_MANAGER) && defined(BT_USING_SIFLI)
    bt_call_audio_open(dev, 0);
#endif

    if (!is_valid_bt_call(dev))
    {
        return;
    }

    dev->call_info.active_state = call_state;
#if defined(AUDIO_USING_MANAGER) && defined(BT_USING_SIFLI)
    dev->call_info.ring_type = (bt_hfp_hf_get_ring_type() & dev->config.inband_ring);
#endif

    rt_bt_event_notify(&args);
    LOG_I("bt_call_status_notify:%s\n", bt_call_state_to_name(call_state));
    return;
}


static void bt_call_clcc_hdl(rt_bt_device_t *dev, bt_clcc_ind_t *ind)
{
    uint16_t size = 0;
    uint8_t idx;
    bt_call_state_t call_status = BT_CALL_IDLE;
    if (ind->idx > BT_MAX_CALL_NUM)  return;
    RT_ASSERT(ind->idx);

    idx = ind->idx - 1;
    size = ((ind->number_size > BT_MAX_PHONE_NUMBER_LEN) ? BT_MAX_PHONE_NUMBER_LEN : ind->number_size);
    dev->call_info.call_num++;

    rt_memset(&dev->call_info.phone_number[idx], 0x00, sizeof(phone_number_t));
    for (uint8_t i = 0; i < ind->number_size; i++)
    {
        if (ind->number[i] != '"')
        {
            dev->call_info.phone_number[idx].number[dev->call_info.phone_number[idx].size] = ind->number[i];
            dev->call_info.phone_number[idx].size++;
            if (dev->call_info.phone_number[idx].size >= BT_MAX_PHONE_NUMBER_LEN)
            {
                break;
            }
        }
    }
    if (dev->call_info.phone_number[idx].size == 1)
    {
        dev->call_info.phone_number[idx].size = 0;
    }

    dev->call_info.active_idx = idx;
    dev->call_info.dir[dev->call_info.active_idx] = ind->dir;
    switch (ind->st)
    {
    case 0:
        call_status = BT_CALL_ACTIVE;
        break;
    case 1:
    case 6:
        call_status = BT_CALL_ONHOLD;
        break;
    case 2:
        call_status = BT_CALL_OUTGOING_DAILING;
        break;
    case 3:
        call_status = BT_CALL_OUTGOING_ALERTING;
        break;

    case 4:
        call_status = BT_CALL_INCOMING;
        break;
    case 5:
        call_status = BT_CALL_WAITING;
        break;

    default:
        break;
    }
#ifdef BT_USING_PBAP
#if defined(BT_CONNECT_SUPPORT_MULTI_LINK)
    bt_cm_bonded_dev_t *conn = bt_cm_get_bonded_dev();
    uint8_t pbap_connect = 0;
    uint8_t i;
    for (i = 0; i < BT_CM_MAX_CONN; i++)
    {
        if (conn->info[i].role == BT_CM_SLAVE &&
                BT_STATE_CONNECTED == rt_bt_get_connect_state_by_conn_idx(dev, i, BT_PROFILE_PBAP))
        {
            pbap_connect = 1;
        }
    }

    if (pbap_connect &&
            (call_status == BT_CALL_OUTGOING_ALERTING || call_status == BT_CALL_INCOMING || call_status == BT_CALL_WAITING))
#else
    if (rt_bt_get_connect_state(dev, BT_PROFILE_PBAP) == BT_STATE_CONNECTED &&
            (call_status == BT_CALL_OUTGOING_ALERTING || call_status == BT_CALL_INCOMING || call_status == BT_CALL_WAITING))
#endif
    {
        dev->ops->control(dev, BT_CONTROL_PBAP_GET_NAME_BY_NUMBER, &dev->call_info.phone_number[idx]);
    }
    else
    {
        rt_memcpy(&dev->call_info.contacts[idx], &pre_call_info.contacts[idx], sizeof(pbap_vcard_list_t));
    }
#endif
    int ret = bt_call_fsm_handle(dev, idx, BT_EVENT_CALL_STATUS_IND, &call_status);
    if (STATEM_STATE_NOCHANGE == ret)
    {
        bt_call_state_t reset_status = BT_CALL_IDLE;
        bt_call_fsm_handle(dev, idx, BT_EVENT_CALL_STATUS_IND, &reset_status);          /*reset status to idle*/
        bt_call_fsm_handle(dev, idx, BT_EVENT_CALL_STATUS_IND, &call_status);
    }
    LOG_I("bt_call_clcc_hdl:%s ,st:%d num:%s active:%d\n", bt_call_state_to_name(call_status), ind->st, dev->call_info.phone_number[dev->call_info.active_idx].number, dev->call_info.active_idx);
    return;
}


static void bt_call_num_no_change_hdl(rt_bt_device_t *dev)
{
    if (1 == dev->call_info.call_num)
    {
        dev->call_info.active_idx = bt_get_call_active_idx(dev);
        return;
    }
    bt_call_active_update(dev);
    return;
}

static void bt_call_info_clear_by_idx(rt_bt_device_t *dev, uint8_t index)
{
    bt_call_state_t call_state = BT_CALL_IDLE;

    RT_ASSERT(index != BT_INVALID_CALL_IDX);
    rt_memset(&dev->call_info.phone_number[index], 0, sizeof(phone_number_t));

    bt_call_fsm_handle(dev, index, BT_EVENT_CALL_STATUS_IND, &call_state);
    return;
}

static void bt_call_info_clear_by_mask(rt_bt_device_t *dev, uint32_t mask)
{
    bt_call_state_t call_state = BT_CALL_IDLE;
    uint32_t call_idx = 0;
    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        if (mask & (1 << i))
        {
            continue;
        }

        rt_memset(&dev->call_info.phone_number[i], 0, sizeof(phone_number_t));
        bt_call_fsm_handle(dev, i, BT_EVENT_CALL_STATUS_IND, &call_state);
    }

    return;
}


static void bt_call_info_remove(rt_bt_device_t *dev)
{
    uint8_t call_idx = 0;
    uint32_t bit_mask = 0;

    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        call_idx = bt_get_call_idx_by_number(dev, pre_call_info.phone_number[i].number,
                                             pre_call_info.phone_number[i].size);
        if (BT_INVALID_CALL_IDX != call_idx)
        {
            bit_mask |= (1 << call_idx);
        }
    }
    LOG_I("%s mask:%d", __func__, bit_mask);
    bt_call_info_clear_by_mask(dev, bit_mask);
    return;
}

static void bt_call_active_update(rt_bt_device_t *dev)
{
    dev->call_info.active_idx = bt_get_call_idx_by_state(dev, BT_CALL_WAITING);
    for (uint8_t i = BT_CALL_WAITING; i > BT_CALL_IDLE; i--)
    {
        dev->call_info.active_idx = bt_get_call_idx_by_state(dev, i);
        if (BT_INVALID_CALL_IDX != dev->call_info.active_idx)
        {
            break;
        }
    }
    return;
}

static void bt_call_num_change_hdl(rt_bt_device_t *dev)
{
    if (dev->call_info.call_num > pre_call_info.call_num)
    {
        bt_call_active_update(dev);
        return;
    }
    bt_call_info_remove(dev);
    bt_call_active_update(dev);
    return;
}

static void bt_call_info_hdl(rt_bt_device_t *dev)
{
    if (pre_call_info.call_num != dev->call_info.call_num)
    {
        bt_call_num_change_hdl(dev);
        rt_memcpy(&pre_call_info, &dev->call_info, sizeof(bt_call_info_t));
        return;
    }

    bt_call_num_no_change_hdl(dev);
    rt_memcpy(&pre_call_info, &dev->call_info, sizeof(bt_call_info_t));
    return;
}



static void bt_sifli_clcc_timeout(void     *parameter)
{
    rt_bt_device_t *dev = (rt_bt_device_t *)parameter;
    if (CALL_CLCC_START == dev->fsm.clcc_process_status)
    {
        dev->ops->control(dev, BT_CONTROL_GET_REMOTE_PHONE_NUMER, RT_NULL);
    }
    else
    {
        LOG_I("%s:start clcc get again!", __func__);
        bt_call_start_get_clcc(dev);
    }
    return;
}


static void bt_call_start_get_clcc(rt_bt_device_t *dev)
{
    if (NULL == clcc_timer_hdl)
    {
        clcc_timer_hdl = rt_timer_create("bt_clcc", bt_sifli_clcc_timeout, dev,
                                         rt_tick_from_millisecond(100), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }

    RT_ASSERT(clcc_timer_hdl);
    if (CALL_CLCC_COMPLETE == dev->fsm.clcc_process_status)
    {
        dev->fsm.clcc_process_status = CALL_CLCC_START;
    }
    rt_timer_stop(clcc_timer_hdl);
    rt_timer_start(clcc_timer_hdl);
    return;
}


uint8_t bt_call_event_hdl(rt_bt_device_t *dev, uint32_t event, void *args)
{
    switch (event)
    {
    case BT_EVENT_CIND_IND:
    {
        bt_call_start_get_clcc(dev);
        break;
    }

    case BT_EVENT_CLCC_IND:
    {
        /* clcc ind may be reported after the ACL disconnect Ind event, see:ext-redmine#902 */
#if !defined(BT_CONNECT_SUPPORT_MULTI_LINK)
        if (BT_STATE_CONNECTED != rt_bt_get_connect_state(dev, BT_PROFILE_HFP))
        {
            return 0;
        }
#endif

        if (CALL_CLCC_START == dev->fsm.clcc_process_status)
        {
            dev->fsm.clcc_process_status = CALL_CLCC_IN_PROGRESS;
            rt_mutex_take(dev->call_sem, RT_WAITING_FOREVER);
#ifdef BT_USING_PBAP
            rt_memcpy(pre_call_info.contacts, dev->call_info.contacts, sizeof(pre_call_info.contacts));
#endif
            bt_call_info_clear(dev);
        }
        bt_call_clcc_hdl(dev, args);
        break;
    }

    case BT_EVENT_CLCC_COMPLETE:
    {
        if (CALL_CLCC_IN_PROGRESS != dev->fsm.clcc_process_status)
        {
            bt_call_info_clear(dev);
        }

        bt_call_info_hdl(dev);

        if (CALL_CLCC_IN_PROGRESS == dev->fsm.clcc_process_status)
        {
            rt_mutex_release(dev->call_sem);
        }
        dev->fsm.clcc_process_status = CALL_CLCC_COMPLETE;
        bt_call_status_notify(dev);
        break;
    }

    case BT_EVENT_CALL_lINK_ESTABLISHED:
    {
        uint8_t *reason = (uint8_t *)args;

        if (*reason)
        {
            dev->fsm.sco_link = 0;
        }
        else
        {
            dev->fsm.sco_link = 1;
            bt_call_audio_open(dev, 1);
        }
        break;
    }

    case BT_EVENT_CALL_LINK_DOWN:
    {
        dev->fsm.sco_link = 0;
        bt_call_audio_close();
        break;
    }

    case BT_EVENT_CONNECT_COMPLETE:
    {
        bt_connect_info_t *info = (bt_connect_info_t *)args;
        if (BT_PROFILE_HFP == info->profile)
        {
            bt_call_start_get_clcc(dev);
        }
        break;
    }

    case BT_EVENT_CLOSE_COMPLETE:
    case BT_EVENT_DISCONNECT:
    {
        bt_call_reset(dev);
        break;
    }

    case BT_EVENT_VOL_CHANGED:
    {
        bt_volume_set_t *vol_set = (bt_volume_set_t *)args;
        if (BT_VOLUME_CALL == vol_set->mode)
        {
#ifdef AUDIO_USING_MANAGER
            audio_server_set_private_volume(AUDIO_TYPE_BT_VOICE, vol_set->volume.call_volume);
#endif
        }
        break;
    }

    case BT_EVENT_PROFILE_DISCONNECT:
    {
        bt_disconnect_info_t *info = (bt_disconnect_info_t *)args;
        if (BT_PROFILE_HFP == info->profile)
        {
            bt_call_audio_close();
            bt_call_reset(dev);
        }
        break;
    }

    case BT_CONTROL_SET_INBAND_RING:
    {
        dev->config.inband_ring = *((int *)args);
        break;
    }

    }
    return 0;
}

