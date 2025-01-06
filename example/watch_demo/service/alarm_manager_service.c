#include "data_service_provider.h"
#include "alarm_manager_service.h"
#include "share_prefs.h"
#include "drivers/alarm.h"
#include "sf_type.h"
#include "time.h"

#define LOG_TAG  "svc.alm"
#include "log.h"


#ifndef BSP_ALARM_MAX
    #define BSP_ALARM_MAX 8
#endif

typedef struct
{
    rt_alarm_t p_hw_alarm;
    alarm_contxt_t *ctx;
} mgr_alarm_ctx_t;



static datas_handle_t  this_service = NULL;
static mgr_alarm_ctx_t *p_mgr_alarms     = NULL;
static alarm_contxt_t  *p_mgr_alarms_ctx     = NULL;
static int32_t         g_alarms_num;
static struct rt_mutex s_alm_mgr_mutex;
static void hw_alarm_callback(rt_alarm_t alarm, time_t timestamp);

//TODO:
extern time_t time(time_t *raw_time);
struct tm *_localtime_r(const time_t *t, struct tm *r);

static rt_alarm_t setup_hw_alarm(alarm_contxt_t *alm_ctx)
{
    struct rt_alarm_setup setup;
    time_t timestamp;
    struct tm now;

    if (NULL == alm_ctx) return NULL;
    if (ALARM_STATE_DISABLE == alm_ctx->state) return NULL;

    if (ALARM_REPEAT_ONE_SHOT == alm_ctx->days)
    {
        setup.flag           = RT_ALARM_ONESHOT;
    }
    else //find out next alarm wday
    {
        /* get time of now */

        timestamp = time(RT_NULL);
//TODO: if __STRICT_ANSI__  is defined,  localtime_r is not available in armcc
#if defined(__CC_ARM) || defined(__CLANG_ARM) || defined(_MSC_VER)
        _localtime_r(&timestamp, &now);
#elif defined(__GNUC__)
        localtime_r(&timestamp, &now);
#else
        localtime_s(&timestamp, &now);
#endif
        for (; (alm_ctx->days & (1 << now.tm_wday)) != 0; now.tm_wday++)
        {
            if (now.tm_wday > 6) now.tm_wday = 0;
        }

        setup.flag           = RT_ALARM_WEEKLY;
        setup.wktime.tm_wday = now.tm_wday;
    }


    setup.wktime.tm_year = RT_ALARM_TM_NOW;
    setup.wktime.tm_mon  = RT_ALARM_TM_NOW;
    setup.wktime.tm_mday = RT_ALARM_TM_NOW;
    setup.wktime.tm_hour = alm_ctx->hour;
    setup.wktime.tm_min  = alm_ctx->minute;
    setup.wktime.tm_sec  = 0;

    return rt_alarm_create(hw_alarm_callback, &setup);
}


static void hw_alarm_callback(rt_alarm_t alarm, time_t timestamp)
{
    int32_t i;

    for (i = 0; i < g_alarms_num; i++)
    {
        mgr_alarm_ctx_t *p_alm;

        p_alm = p_mgr_alarms + i;

        if (p_alm->p_hw_alarm == alarm)
        {
#ifndef _MSC_VER
            //restart alarm if NOT oneshot
            if (p_alm->ctx->days != ALARM_REPEAT_ONE_SHOT)
            {
                p_alm->p_hw_alarm = setup_hw_alarm(p_alm->ctx);
                rt_alarm_start(p_alm->p_hw_alarm);
            }
            else
            {
                rt_alarm_delete(p_alm->p_hw_alarm);
                p_alm->p_hw_alarm = NULL;
                p_alm->ctx->state = ALARM_STATE_DISABLE;
            }
#endif

            break;
        }
    }
}


static void update_alarms(void)
{
    int32_t i;

    rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
    for (i = 0; i < g_alarms_num; i++)
    {
        mgr_alarm_ctx_t *p_alm;
        p_alm = p_mgr_alarms + i;

#ifndef _MSC_VER
        struct rt_alarm_setup setup;
        if (p_alm->p_hw_alarm)
        {
            rt_alarm_delete(p_alm->p_hw_alarm);
            p_alm->p_hw_alarm = NULL;
        }

        if (ALARM_STATE_ENABLE == p_alm->ctx->state)
        {
            p_alm->p_hw_alarm = setup_hw_alarm(p_alm->ctx);
            rt_alarm_start(p_alm->p_hw_alarm);
        }
#endif
    }

    rt_mutex_release(&s_alm_mgr_mutex);
}

#ifndef _MSC_VER
/**
 * @brief Read alarm data and set to HW alarm
 */
static void init(void)
{
    share_prefs_t  *prefs;
    int32_t i;
    int32_t read_len, content_len;

    rt_mutex_init(&s_alm_mgr_mutex, "alm_mgr", RT_IPC_FLAG_FIFO);

    prefs = share_prefs_open("alarm", SHAREPREFS_MODE_PRIVATE);
    if (prefs != NULL)
        g_alarms_num = share_prefs_get_int(prefs, "num", 0);
    else
        g_alarms_num = 0;


    if (g_alarms_num > BSP_ALARM_MAX)
        g_alarms_num = BSP_ALARM_MAX;

    p_mgr_alarms_ctx = (alarm_contxt_t *) rt_malloc(sizeof(alarm_contxt_t) * BSP_ALARM_MAX);
    SF_ASSERT(p_mgr_alarms_ctx != NULL);
    memset(p_mgr_alarms_ctx, 0, sizeof(alarm_contxt_t) * BSP_ALARM_MAX);

    if (g_alarms_num > 0)
    {
        //read data to p_mgr_alarms_ctx buffer
        content_len = sizeof(alarm_contxt_t) * g_alarms_num;
        read_len = share_prefs_get_block(prefs, "content", p_mgr_alarms_ctx, content_len);
        if (read_len != content_len)
            SF_ASSERT(0);
    }

    //convert to p_mgr_alarms
    p_mgr_alarms = (mgr_alarm_ctx_t *) rt_malloc(sizeof(mgr_alarm_ctx_t) * BSP_ALARM_MAX);
    SF_ASSERT(p_mgr_alarms != NULL);
    memset(p_mgr_alarms, 0, sizeof(mgr_alarm_ctx_t) * BSP_ALARM_MAX);

    for (i = 0; i < g_alarms_num; i++)
    {
        p_mgr_alarms[i].p_hw_alarm = NULL;
        p_mgr_alarms[i].ctx = &p_mgr_alarms_ctx[i];
    }
    update_alarms();

    if (prefs != NULL)    share_prefs_close(prefs);

}
#else
/**
 * @brief init alarm from simulator
 */
static void init(void)
{
    rt_mutex_init(&s_alm_mgr_mutex, "alm_mgr", RT_IPC_FLAG_FIFO);

    g_alarms_num = 2;

    if (g_alarms_num > 0)
    {
        int32_t i, content_len;

        //read data to p_mgr_alarms_ctx buffer
        content_len = sizeof(alarm_contxt_t) * BSP_ALARM_MAX;
        p_mgr_alarms_ctx = (alarm_contxt_t *) rt_malloc(content_len);
        SF_ASSERT(p_mgr_alarms_ctx != NULL);


        //convert to p_mgr_alarms
        p_mgr_alarms = (mgr_alarm_ctx_t *) rt_malloc(sizeof(mgr_alarm_ctx_t) * BSP_ALARM_MAX);
        SF_ASSERT(p_mgr_alarms != NULL);

        for (i = 0; i < g_alarms_num; i++)
        {
            p_mgr_alarms_ctx[i].state = ALARM_STATE_ENABLE;
            p_mgr_alarms_ctx[i].hour =  10 + i;
            p_mgr_alarms_ctx[i].minute = 32 + i;
            p_mgr_alarms_ctx[i].days = ALARM_REPEAT_EVERYDAY;
            p_mgr_alarms_ctx[i].snooze = ALARM_SNOOZE_ENABLE;


            p_mgr_alarms[i].p_hw_alarm = NULL;
            p_mgr_alarms[i].ctx = &p_mgr_alarms_ctx[i];
        }

        update_alarms();
    }

}

#endif

static void get_alarm_data(data_msg_t *msg, int32_t idx)
{
    rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
    if (idx < 0) //No alarm exist, or reach alarm list end
        datas_send_response_data(this_service, msg, 0, (uint8_t *)NULL);
    else
    {
        alarm_msg_t rsp_msg;

        rsp_msg.idx = idx;
        memcpy(&rsp_msg.ctx, &p_mgr_alarms_ctx[idx], sizeof(alarm_contxt_t));
        datas_send_response_data(this_service, msg, sizeof(alarm_msg_t), (uint8_t *)&rsp_msg);
    }

    rt_mutex_release(&s_alm_mgr_mutex);
}

static int32_t msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        static uint8_t is_init = 0;
        if (!is_init)
        {
            init();
            is_init = 1;
        }
        break;
    }

    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        break;
    }

    case MSG_SERVICE_CONFIG_REQ:
    {
        break;
    }

    case ALARMMGR_MSG_GET_ALARM_LIST_NEXT_REQ:
    {
        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);
        int32_t idx;

        if ((g_alarms_num > 0) && (p_mgr_alarms_ctx != NULL))
        {
            if (0 == msg->len) //get first one
            {
                idx = 0;
            }
            else if (p_alm_msg->idx < g_alarms_num - 1)
            {
                idx = p_alm_msg->idx + 1; //get next one
            }
            else
            {
                idx = -1;  //reach end of the list
            }
        }
        else
        {
            idx = -1;
        }

        get_alarm_data(msg, idx);
    }
    break;
    case ALARMMGR_MSG_GET_ALARM_REQ:
    {
        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);
        get_alarm_data(msg, p_alm_msg->idx);
        break;
    }
    case ALARMMGR_MSG_ADD_ALARM_REQ:
    {
        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);
        rt_err_t result = RT_ERROR;

        if (p_alm_msg && g_alarms_num < BSP_ALARM_MAX)
        {
            rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
            p_mgr_alarms[g_alarms_num].p_hw_alarm = NULL;
            p_mgr_alarms[g_alarms_num].ctx = &p_mgr_alarms_ctx[g_alarms_num];
            memcpy(&p_mgr_alarms_ctx[g_alarms_num], &p_alm_msg->ctx, sizeof(alarm_contxt_t));

            g_alarms_num++;
            update_alarms();
            rt_mutex_release(&s_alm_mgr_mutex);
            result = RT_EOK;
        }
        else
        {
            result = -RT_EFULL;
        }

        datas_send_response(service, msg, result);
    }
    break;

    case ALARMMGR_MSG_EDIT_ALARM_REQ:
    {
        rt_err_t result = RT_ERROR;

        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);

        if (p_alm_msg)
        {
            if ((p_alm_msg->idx >= 0) && (p_alm_msg->idx < g_alarms_num))
            {
                rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
                memcpy(&p_mgr_alarms_ctx[p_alm_msg->idx], &p_alm_msg->ctx, sizeof(alarm_contxt_t));
                update_alarms();
                rt_mutex_release(&s_alm_mgr_mutex);

                result = RT_EOK;
            }
            else
            {
                result = RT_ERROR;
            }
        }
        else
        {
            result = RT_EINVAL;
        }

        datas_send_response(service, msg, result);
    }
    break;


    case ALARMMGR_MSG_ENABLE_ALARM_REQ:
    case ALARMMGR_MSG_DISABLE_ALARM_REQ:
    {
        rt_err_t result = RT_ERROR;

        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);

        rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
        if (p_alm_msg)
        {
            if ((p_alm_msg->idx >= 0) && (p_alm_msg->idx < g_alarms_num))
            {
                if (ALARMMGR_MSG_ENABLE_ALARM_REQ == msg->msg_id)
                    p_mgr_alarms[p_alm_msg->idx].ctx->state = ALARM_STATE_ENABLE;
                else
                    p_mgr_alarms[p_alm_msg->idx].ctx->state = ALARM_STATE_DISABLE;

                update_alarms();

                result = RT_EOK;
            }
            else
            {
                result = RT_ERROR;
            }
        }
        else
        {
            result = RT_EINVAL;
        }
        rt_mutex_release(&s_alm_mgr_mutex);

        datas_send_response(service, msg, result);
    }
    break;

    case ALARMMGR_MSG_DELETE_ALARM_REQ:
    {
        rt_err_t result = RT_ERROR;

        alarm_msg_t *p_alm_msg = (alarm_msg_t *)data_service_get_msg_body(msg);

        rt_mutex_take(&s_alm_mgr_mutex, RT_WAITING_FOREVER);
        if (p_alm_msg)
        {
            if ((p_alm_msg->idx >= 0) && (p_alm_msg->idx < g_alarms_num))
            {
                int32_t i;
#ifndef _MSC_VER
                rt_alarm_delete(p_mgr_alarms[p_alm_msg->idx].p_hw_alarm);
#endif

                for (i = p_alm_msg->idx; i < g_alarms_num - 1; i++)
                {
                    memcpy(&p_mgr_alarms_ctx[i], &p_mgr_alarms_ctx[i + 1], sizeof(alarm_contxt_t));
                    p_mgr_alarms[i].p_hw_alarm = p_mgr_alarms[i + 1].p_hw_alarm;
                }
                g_alarms_num--;
                result = RT_EOK;
            }
            else
            {
                result = RT_ERROR;
            }
        }
        else
        {
            result = RT_EINVAL;
        }
        rt_mutex_release(&s_alm_mgr_mutex);

        datas_send_response(service, msg, result);
    }
    break;


    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}

static data_service_config_t service_cb =
{
    .max_client_num = 3,
    .queue = RT_NULL,
    .data_filter = NULL,
    .msg_handler = msg_handler,
};


#ifndef _MSC_VER
    static
#endif
int alarm_manager_service_register(void)
{
    rt_err_t ret = RT_EOK;


    this_service = datas_register("alarmmgr", &service_cb);

    return ret;
}

INIT_PRE_APP_EXPORT(alarm_manager_service_register);

