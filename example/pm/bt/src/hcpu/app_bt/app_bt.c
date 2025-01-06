

#include <stdio.h>
#include <string.h>
#include "app_bt.h"
//#include "rt_def.h"



#define DBG_TAG               "app_bt"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

uint8_t app_bg_thread_stack[2048];


static rt_device_t bt_device = RT_NULL;

static bt_err_t app_bt_state_check(bt_state_t state)
{
    bt_err_t ret = BT_EOK;
    bt_state_t cur_state = BT_STATE_POWER_OFF;
    ret = app_bt_query_state(&cur_state);
    if (BT_EOK == ret)
    {
        rt_kprintf("app_bt_state_check cur_state %d state %d\n", cur_state, state);
        if (cur_state != state)
        {
            return BT_ERROR_STATE;
        }
    }
    LOG_I("%s: ret:%x state:%d query_state:%d", __func__, ret, state, cur_state);
    return ret;
}

static bt_err_t call_state_check(void)
{
    bt_err_t ret = BT_EOK;
    bt_state_t state = BT_STATE_POWER_OFF;
    ret = app_bt_query_state(&state);
    if (BT_EOK == ret)
    {
        if ((BT_STATE_ON_MEDIA != state) && \
                (BT_STATE_CONNECTED != state))
        {
            return BT_ERROR_STATE;
        }
    }
    LOG_I("%s: ret:%x state:%d", __func__, ret, state);
    return ret;
}


bt_err_t app_bt_search_device_start(void)
{
    bt_err_t ret = BT_EOK;
    ret = app_bt_state_check(BT_STATE_PAIR);
    if (BT_EOK != ret)
    {
        return ret;
    }
    ret = rt_device_control(bt_device, BT_CONTROL_SEARCH_EQUIPMENT, RT_NULL);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_bt_search_device_stop(void)
{
    bt_err_t ret = BT_EOK;
    bt_state_t state = BT_STATE_POWER_OFF;
    ret = app_bt_state_check(BT_STATE_SEARCH);
    if (BT_EOK != ret)
    {
        return ret;
    }
    ret = rt_device_control(bt_device, BT_CONTROL_CANCEL_SEARCH, RT_NULL);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_bt_connect_start(const bt_mac_t *addr)
{
    bt_err_t ret = BT_EOK;
    bt_state_t state = BT_STATE_POWER_OFF;
    if (RT_NULL == addr)
    {
        return BT_ERROR_INPARAM;
    }

    ret = app_bt_state_check(BT_STATE_PAIR);
    if (BT_EOK != ret)
    {
        return ret;
    }

    ret = rt_device_control(bt_device, BT_CONTROL_CONNECT_DEVICE, (void *)addr);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_bt_disconnect(void)
{
    bt_err_t ret = BT_EOK;
    bt_state_t state = BT_STATE_POWER_OFF;
    app_bt_query_state(&state);
    if (state < BT_STATE_CONNECTING || state > BT_STATE_ON_MEDIA)
    {
        return ret;
    }
    ret = rt_device_control(bt_device, BT_CONTROL_DISCONNECT, RT_NULL);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_bt_query_state(bt_state_t *state)
{
    bt_err_t ret = BT_EOK;
    if (RT_NULL == state)
    {
        return BT_ERROR_INPARAM;
    }
    ret = rt_device_control(bt_device, BT_CONTROL_QUERY_STATE, state);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_bt_nonblock_query_state(void)
{
    bt_err_t ret = BT_EOK;
    ret = rt_device_control(bt_device, BT_CONTROL_QUERY_STATE_NONBLOCK, RT_NULL);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}


bt_err_t app_bt_register_notify(bt_notify_cb cb)
{
    bt_err_t ret = BT_EOK;
    ret = rt_device_control(bt_device, BT_CONTROL_REGISTER_NOTIFY, cb);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_bt_unregister_notify(bt_notify_cb cb)
{
    bt_err_t ret = BT_EOK;
    ret = rt_device_control(bt_device, BT_CONTROL_UNREGISTER_NOTIFY, cb);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_open_bt(void)
{
    bt_err_t ret = BT_EOK;
    ret = rt_device_control(bt_device, BT_CONTROL_OPEN_DEVICE, RT_NULL);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_close_bt(void)
{
    bt_err_t ret = BT_EOK;
    ret = rt_device_control(bt_device, BT_CONTROL_CLOSE_DEVICE, RT_NULL);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

void app_bt_notify(bt_notify_t *args)
{
    LOG_I("%s event:%d", __func__, args->event);
    return;
}

bt_err_t app_bt_make_call(const char *number, rt_size_t size)
{
    bt_err_t ret = BT_EOK;
    phone_number_t args;
    if (RT_NULL == number)
    {
        return BT_ERROR_INPARAM;
    }
    ret = call_state_check();
    if (BT_EOK != ret)
    {
        return ret;
    }
    args.number = number;
    args.size = size;
    ret = rt_device_control(bt_device, BT_CONTROL_MAKE_CALL, &args);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_bt_answer_call(void)
{
    bt_err_t ret = BT_EOK;
    ret = rt_device_control(bt_device, BT_CONTROL_PHONE_CONNECT, RT_NULL);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_bt_dial_back(void)
{
    bt_err_t ret = BT_EOK;
    ret = call_state_check();
    if (BT_EOK != ret)
    {
        return ret;
    }
    ret = rt_device_control(bt_device, BT_CONTROL_DIAL_BACK, RT_NULL);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

bt_err_t app_bt_set_volume(bt_volume_set_t *volume)
{
    bt_err_t ret = BT_EOK;
    if (RT_NULL == volume)
    {
        return BT_ERROR_INPARAM;
    }
    ret = rt_device_control(bt_device, BT_CONTROL_SET_VOLUME, volume);
    LOG_I("%s ret:%x", __func__, ret);
    return ret;
}

#ifdef BT_USING_DTMF
bt_err_t app_bt_dtmf_dial(bt_dtmf_key_t key)
{
    bt_err_t ret = BT_EOK;
    bt_dtmf_key_t dial_key = key;
    ret = rt_device_control(bt_device, BT_CONTROL_DTMF_DIAL, &dial_key);
    LOG_I("%s dial_key:%d ret:%x", __func__, dial_key, ret);
    return ret;
}
#endif



int32_t app_bt(int32_t argc, char **argv)
{
    bt_err_t ret = BT_EOK;
    if (argc < 2)
    {
        rt_kprintf("Wrong argument\n");
    }
    rt_kprintf("cmd:%s\n", argv[1]);
    if (!strcmp(argv[1], "open"))
    {
        ret = app_open_bt();
    }
    else if (!strcmp(argv[1], "close"))
    {
        ret = app_close_bt();
    }
    else if (!strcmp(argv[1], "search_start"))
    {
        ret = app_bt_search_device_start();
    }
    else if (!strcmp(argv[1], "search_stop"))
    {
        ret = app_bt_search_device_stop();
    }
    else if (!strcmp(argv[1], "connect"))
    {
        if (rt_strlen(argv[2]) == 12)
        {
            int reuslt = 0;
            bt_mac_t addr = {0};
            reuslt = sscanf(argv[2], "%02X%02X%02X%02X%02X%02X", (uint32_t *)&addr.addr[0], \
                            (uint32_t *)&addr.addr[1], \
                            (uint32_t *)&addr.addr[2], \
                            (uint32_t *)&addr.addr[3], \
                            (uint32_t *)&addr.addr[4], \
                            (uint32_t *)&addr.addr[5]);
            if (BT_MAX_MAC_LEN != reuslt)
            {
                rt_kprintf("mac argument error\n");
                return 0;
            }
            ret = app_bt_connect_start(&addr);
        }
        else
        {
            rt_kprintf("mac argument error\n");
        }
    }
    else if (!strcmp(argv[1], "disconnect"))
    {
        ret = app_bt_disconnect();
    }
    else if (!strcmp(argv[1], "query"))
    {
        bt_state_t state = BT_STATE_POWER_OFF;
        ret = app_bt_query_state(&state);
        LOG_I("state:%d", state);
    }
    else if (!strcmp(argv[1], "query_nonblock"))
    {
        ret = app_bt_nonblock_query_state();
    }
    else if (!strcmp(argv[1], "register"))
    {
        ret = app_bt_register_notify(app_bt_notify);
    }
    else if (!strcmp(argv[1], "unregister"))
    {
        ret = app_bt_unregister_notify(app_bt_notify);
    }
    else if (!strcmp(argv[1], "make_call"))
    {
        ret = app_bt_make_call(argv[2], rt_strlen(argv[2]));
    }
    else if (!strcmp(argv[1], "answer_call"))
    {
        ret = app_bt_answer_call();
    }
    else if (!strcmp(argv[1], "dial_back"))
    {
        ret = app_bt_dial_back();
    }
    else if (!strcmp(argv[1], "vol_set"))
    {
        bt_volume_set_t vol = {0};
        vol.volume.call_volume = 100;
        vol.mode = BT_VOLUME_CALL;
        ret = app_bt_set_volume(&vol);
    }
#ifdef BT_USING_DTMF
    else if (!strcmp(argv[1], "dtmf"))
    {
        bt_dtmf_key_t key;

        if ('*' == *argv[2])
        {
            key = BT_DTMF_KEY_STAR;
        }
        else if ('#' == *argv[2])
        {
            key = BT_DTMF_KEY_HASH;
        }
        else
        {
            bt_dtmf_key_t key = atoi(argv[2]);
        }
        rt_kprintf("dtmf key inputkey %c,outkey %x\n", *argv[2], key);
        ret = app_bt_dtmf_dial(key);
    }
#endif


    LOG_I("%s %s ret:%x", __func__, argv[1], ret);
    return 0;
}

MSH_CMD_EXPORT(app_bt, app_bt cmd);

int app_bt_comm_init(void)
{
    bt_device = rt_device_find("bt_device_sifli");
    if (RT_NULL == bt_device)
    {
        LOG_E("init bt fail\n");
        return RT_ERROR;
    }
    return BT_EOK;
}

INIT_PRE_APP_EXPORT(app_bt_comm_init);


/*int app_preprocess_thread_init(void)
{
    rt_err_t err;
    rt_mq_t g_app_preprocess_queue;
    static struct rt_thread g_app_bg_thread;

    g_app_preprocess_queue = rt_mq_create("app_preprocess_queue", sizeof(void *), 30, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_app_preprocess_queue);

    err = rt_thread_init(&g_app_bg_thread, "app_bg", app_preprocess_thread_entry, RT_NULL,
                         app_bg_thread_stack, sizeof(app_bg_thread_stack), RT_THREAD_PRIORITY_MIDDLE, 2);
    RT_ASSERT(RT_EOK == err);

    rt_thread_startup(&g_app_bg_thread);


    return 0;
}

INIT_PRE_APP_EXPORT(app_preprocess_thread_init);*/




