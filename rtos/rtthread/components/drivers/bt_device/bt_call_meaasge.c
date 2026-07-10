/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>

#if defined(BT_FINSH)
    #include "bts2_global.h"
    #include "bts2_app_inc.h"
    #include "hfp_audio_api.h"
#endif
#include "bt_connection_manager.h"

#define DBG_TAG    "bt_call_message"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

#define BT_SCO_CFM_TIMEOUT (4000)

typedef struct
{
    struct rt_work open;
    struct rt_work close;
    struct rt_work delay_open;
} bt_audio_work_t;

static bt_call_info_t pre_call_info = {0};
static rt_timer_t  clcc_timer_hdl = NULL;
static bt_audio_work_t bt_audio_work;
static struct rt_work sco_cfm_work;
static uint8_t clcc_retry_count;
static void bt_call_start_get_clcc(rt_bt_device_t *dev);
static void bt_call_active_update(rt_bt_device_t *dev);
static void bt_audio_work_submit(rt_bt_device_t *dev, struct rt_work *work);

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

static uint8_t bt_get_call_number_by_idx(uint8_t *number, uint8_t size)
{
    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        if (0 == size) continue;

        if (!rt_memcmp(number, &pre_call_info.phone_number[i].number, size))
        {
            return i;
        }
    }
    uint8_t num[BT_MAX_PHONE_NUMBER_LEN] = {0};
    for (uint8_t i = 0; i < BT_MAX_CALL_NUM; i++)
    {
        if (!rt_memcmp(&pre_call_info.phone_number[i].number, num, BT_MAX_PHONE_NUMBER_LEN))
        {
            return i;
        }
    }
    RT_ASSERT(0);
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
    rt_memset(dev->fsm.sco_link, 0x00, sizeof(dev->fsm.sco_link));
    clcc_retry_count = 0;
    rt_memset(&pre_call_info, 0x00, sizeof(bt_call_info_t));
    if (clcc_timer_hdl) rt_timer_stop(clcc_timer_hdl);
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

    if (!dev->fsm.sco_link[dev->active_idx])
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
    case BT_CALL_IDLE:
    {
        if (dev->config.is_direct_audio_on)
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

static void bt_call_audio_delay_open(rt_bt_device_t *dev)
{
#if defined(AUDIO_USING_MANAGER) && defined(BT_FINSH) && !defined(CFG_BT_VOICE_RELAY)
    if (bt_call_get_direct_audio_on(dev))
    {
        //rt_mutex_take(dev->handle_lock, RT_WAITING_FOREVER);
        hfp_aduio_open_path(AUDIO_TYPE_BT_VOICE);
        //rt_mutex_release(dev->handle_lock);
    }
#endif
    return;
}

extern bts2_app_stru *bts2g_app_p;

static void bt_call_audio_open(rt_bt_device_t *dev, uint8_t set_audio)
{
    bt_call_state_t call_state = rt_bt_get_call_state(dev);
    uint8_t direct_audio_on = 0;

    if (set_audio)
    {
        if (dev->config.is_direct_audio_on)
        {
            direct_audio_on = 1;
        }
        else if ((CALL_CLCC_COMPLETE == dev->fsm.clcc_process_status) && bt_call_get_direct_audio_on(dev))
        {
            direct_audio_on = 1;
        }

#if defined(AUDIO_USING_MANAGER) && defined(BT_FINSH) && !defined(CFG_BT_VOICE_RELAY)
        hfp_set_audio_voice_para(&dev->sco_para[dev->active_idx], dev->sco_para[dev->active_idx].audio_on, direct_audio_on);
#endif
        if (BT_CALL_ACTIVE == call_state)
        {
            rt_workqueue_cancel_work(dev->wq, &bt_audio_work.delay_open);
            bt_audio_work_submit(dev, &bt_audio_work.delay_open);
        }
    }
    else if (dev->fsm.sco_link[dev->active_idx])
    {
#if defined(AUDIO_USING_MANAGER) && defined(BT_FINSH) && !defined(CFG_BT_VOICE_RELAY)
        hfp_set_audio_voice_para(&dev->sco_para[dev->active_idx], 1, direct_audio_on);
#endif
        rt_workqueue_cancel_work(dev->wq, &bt_audio_work.delay_open);
        bt_audio_work_submit(dev, &bt_audio_work.delay_open);
    }
    return;
}

static void bt_call_audio_close(rt_bt_device_t *dev)
{
    rt_workqueue_cancel_work(dev->wq, &bt_audio_work.delay_open);
#if defined(AUDIO_USING_MANAGER) && !defined(CFG_BT_VOICE_RELAY)
    //rt_mutex_take(dev->handle_lock, RT_WAITING_FOREVER);
    hfp_audio_close_path();
    //rt_mutex_release(dev->handle_lock);
#endif // AUDIO_USING_MANAGER
    return;
}


static void bt_call_status_notify(rt_bt_device_t *dev, rt_bool_t audio_op)
{
    bt_notify_t args;
    bt_call_state_t call_state = rt_bt_get_call_state(dev);
    args.event = BT_EVENT_CALL_STATUS_IND;
    args.args = &call_state;

#if defined(AUDIO_USING_MANAGER) && defined(BT_USING_SIFLI)
    if (audio_op)
    {
        dev->set_audio = 0;
        bt_audio_work_submit(dev, &bt_audio_work.open);
    }
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
    return;
}

static uint8_t bt_call_conn_idx(rt_bt_device_t *dev)
{
#if defined(BT_CONNECT_SUPPORT_MULTI_LINK)
    uint8_t i;
    for (i = 0; i < BT_CM_DEVICE_MAX_CONN; i++)
    {
        if (rt_bt_get_connect_dev_by_idx(dev, i)->link_type == BT_LINK_PHONE &&
                BT_STATE_CONNECTED == rt_bt_get_connect_state_by_conn_idx(dev, i, BT_PROFILE_HFP))
        {
            return i;
        }
    }
    return i;
#else
    return 0;
#endif
}

static void bt_call_clcc_hdl(rt_bt_device_t *dev, bt_clcc_ind_t *ind)
{
    uint8_t idx;
    bt_call_state_t call_status = BT_CALL_IDLE;
    phone_number_t phone_number = {0};

    rt_memset(&phone_number, 0x00, sizeof(phone_number_t));
    for (uint8_t i = 0; i < ind->number_size; i++)
    {
        if (ind->number[i] != '"')
        {
            phone_number.number[phone_number.size] = ind->number[i];
            phone_number.size++;
            if (phone_number.size >= BT_MAX_PHONE_NUMBER_LEN)
            {
                break;
            }
        }
    }
    if (phone_number.size == 1)
    {
        phone_number.size = 0;
    }
    if (!phone_number.size)
    {
        return;
    }

    idx = bt_get_call_number_by_idx(phone_number.number, phone_number.size);
    LOG_I("%s idx:0x%x call_num:%d", __func__, idx, dev->call_info.call_num);
    rt_memcpy(&dev->call_info.phone_number[idx], &phone_number, sizeof(phone_number_t));

    dev->call_info.call_num++;
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
    dev->call_info.conn_idx = bt_call_conn_idx(dev);
#ifdef BT_USING_PBAP_NUM_BY_NAME
#if defined(BT_CONNECT_SUPPORT_MULTI_LINK)
    if (rt_bt_get_connect_state_by_conn_idx(dev, dev->call_info.conn_idx, BT_PROFILE_PBAP) == BT_STATE_CONNECTED &&
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

static void bt_audio_open_work_handler(struct rt_work *work, void *work_data)
{
    rt_bt_device_t *dev = (rt_bt_device_t *)work_data;
    LOG_I("%s set_audio:%d", __func__, dev->set_audio);
    bt_call_audio_open(dev, dev->set_audio);
    return;
}

static void bt_audio_close_work_handler(struct rt_work *work, void *work_data)
{
    rt_bt_device_t *dev = (rt_bt_device_t *)work_data;
    LOG_I("%s", __func__);
    bt_call_audio_close(dev);
    return;
}

static void bt_audio_delay_open_work_handler(struct rt_work *work, void *work_data)
{
    rt_bt_device_t *dev = (rt_bt_device_t *)work_data;
    LOG_I("%s", __func__);
    bt_call_audio_delay_open(dev);
    return;
}

static void bt_audio_work_submit(rt_bt_device_t *dev, struct rt_work *work)
{
    rt_err_t ret;
    do
    {
        ret = rt_workqueue_submit_work(dev->wq, work, 300);
        rt_thread_mdelay(10);
    }
    while (-RT_EBUSY == ret);
    LOG_I("%s ret:%d", __func__, ret);
    return;
}

static void bt_sco_cfm_work_handler(struct rt_work *work, void *work_data)
{
    rt_bt_device_t *dev = (rt_bt_device_t *)work_data;
    bt_notify_device_sco_info_t sco_info = {0};
    sco_info.sco_res = 1;
    sco_info.sco_type = BT_NOTIFY_HFP_HF;
    sco_info.conn_idx = dev->active_idx;
    LOG_I("%s", __func__);
    bt_notify_t args;
    args.event = BT_EVENT_CALL_lINK_ESTABLISHED;
    args.args = &sco_info;
    if (!dev->fsm.sco_link[dev->active_idx])
    {
        rt_bt_event_notify(&args);
    }
    return;
}

void bt_call_init(rt_bt_device_t *dev)
{
    rt_work_init(&bt_audio_work.open, bt_audio_open_work_handler, dev);
    rt_work_init(&bt_audio_work.close, bt_audio_close_work_handler, dev);
    rt_work_init(&bt_audio_work.delay_open, bt_audio_delay_open_work_handler, dev);
    rt_work_init(&sco_cfm_work, bt_sco_cfm_work_handler, dev);
    return;
}

uint8_t bt_call_get_state_change(bt_cind_ind_t *cind_data)
{
    uint8_t call_state_change = 0;
    switch (cind_data->type)
    {
    case BT_CIND_CALL_TYPE:              //(0,1)
    case BT_CIND_CALLSETUP_TYPE:         //(0,3)
    case BT_CIND_CALLHELD_TYPE:          //(0,2)
        call_state_change = 1;
        break;
    default:
        break;
    }
    return call_state_change;
}

uint8_t bt_call_event_hdl(rt_bt_device_t *dev, uint32_t event, void *args)
{
    rt_err_t ret;
    switch (event)
    {
    case BT_EVENT_CIND_IND:
    {
        clcc_retry_count = 0;
        if (bt_call_get_state_change(args))
        {
            //dev->ops->control(dev, BT_CONTROL_EXIT_SNIFF, RT_NULL);
            bt_call_start_get_clcc(dev);
        }
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
#ifdef BT_USING_PBAP_NUM_BY_NAME
            rt_memcpy(pre_call_info.contacts, dev->call_info.contacts, sizeof(pre_call_info.contacts));
#endif
            bt_call_info_clear(dev);
        }
        else
        {
            rt_memcpy(&pre_call_info.phone_number[dev->call_info.active_idx], &dev->call_info.phone_number[dev->call_info.active_idx], sizeof(phone_number_t));
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
        if ((0 == dev->call_info.call_num) && dev->fsm.sco_link[dev->active_idx] && (0 == clcc_retry_count))
        {
            /*retry for get clcc info */
            clcc_retry_count++;
            LOG_I("%s retry get clcc info:%d", __func__, clcc_retry_count);
            bt_call_start_get_clcc(dev);
        }
        else
        {
            bt_call_status_notify(dev, RT_TRUE);
        }
        break;
    }

    case BT_EVENT_CALL_lINK_ESTABLISHED:
    {
        bt_notify_device_sco_info_t *sco_info = (bt_notify_device_sco_info_t *)args;
        if (sco_info->sco_type == BT_NOTIFY_HFP_HF)
        {
            dev->active_idx = sco_info->conn_idx;
        }
        if (sco_info->sco_res)
        {
            dev->fsm.sco_link[sco_info->conn_idx] = 0;
        }
        else
        {
            dev->fsm.sco_link[sco_info->conn_idx] = 1;
            dev->set_audio = 1;
            dev->sco_para[dev->active_idx] = sco_info->para;
            if (sco_info->sco_type == BT_NOTIFY_HFP_HF)
            {
                bt_audio_work_submit(dev, &bt_audio_work.open);
            }
        }
        if (sco_info->sco_type == BT_NOTIFY_HFP_HF)
        {
            ret = rt_workqueue_cancel_work(dev->wq, &sco_cfm_work);
            if (!ret)
            {
                LOG_I("cancel sco cfm work ret:%d", ret);
            }
        }
        break;
    }

    case BT_EVENT_CALL_LINK_DOWN:
    {
        bt_notify_device_sco_info_t *sco_info = (bt_notify_device_sco_info_t *)args;
        if (sco_info->sco_type == BT_NOTIFY_HFP_HF)
        {
            dev->active_idx = sco_info->conn_idx;
        }
        dev->fsm.sco_link[sco_info->conn_idx] = 0;
        if (sco_info->sco_type == BT_NOTIFY_HFP_HF)
        {
            bt_audio_work_submit(dev, &bt_audio_work.close);
        }
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
            bt_audio_work_submit(dev, &bt_audio_work.close);
            bt_call_reset(dev);
            bt_call_status_notify(dev, RT_FALSE);
        }
        break;
    }

    case BT_CONTROL_SET_INBAND_RING:
    {
        dev->config.inband_ring = *((int *)args);
        break;
    }

    case BT_CONTROL_SET_DIRECT_AUDIO_ON:
    {
        dev->config.is_direct_audio_on = *((uint8_t *)args);// 0:call - 1:audio_open
        break;
    }

    case BT_EVENT_AT_CMD_CFM_STATUS:
    {
#if defined(BT_FINSH)
        bt_at_cmd_cfm_t *cfm = (bt_at_cmd_cfm_t *)args;
        if ((HFP_HF_AT_BCC == cfm->cmd_id) && (!cfm->res))
        {
            rt_workqueue_cancel_work(dev->wq, &sco_cfm_work);
            rt_workqueue_submit_work(dev->wq, &sco_cfm_work, BT_SCO_CFM_TIMEOUT);
        }
#endif
        break;
    }

    }
    return 0;
}



