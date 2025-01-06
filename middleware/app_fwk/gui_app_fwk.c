/*********************
 *      INCLUDES
 *********************/
#include "gui_app_int.h"

#include "bf0_lib.h"


//#define STOP_APP_IF_PAUSED



#define DBG_TAG           "APP.FWK"
#define DBG_LVL           DBG_LOG
#include "rtdbg.h"



/* messagequeue define */
#define GUI_APP_MBX_SIZE  10
struct rt_mailbox gui_app_mbx;

static rt_uint32_t gui_app_mbx_buffer_pool[GUI_APP_MBX_SIZE];
static task_t task_handler = NULL;

static rt_thread_t host_rtos_task = NULL;

static builtin_app_desc_t *builtin_app_table_begin = NULL;
static builtin_app_desc_t *builtin_app_table_end = NULL;
static uint8_t test_mode = 0;



#define MAX_LOCALE_NAME   (10)
#define SYS_CFG_DIR "/sys"
#define LOCALE_SETTING_FILE_NAME  SYS_CFG_DIR "/"  "locale.cfg"
static char locale_name[MAX_LOCALE_NAME];

#if defined (BSP_USING_LVGL_INPUT_AGENT)
    extern int indev_agent_mode(void);
    #define CHECK_CUR_RTOS_TASK()  RT_ASSERT(indev_agent_mode() || test_mode || rt_thread_self() == host_rtos_task) //Make sure API be invoked in host thread only
#else
    #define CHECK_CUR_RTOS_TASK()  RT_ASSERT(test_mode || rt_thread_self() == host_rtos_task) //Make sure API be invoked in host thread only
#endif
/*
static gui_app_close_anim_type_t gui_app_close_anim_type;

static bool destory_ani_done = true;
*/
static app_sche_state s_sche_state = APP_SCHE_SUSPEND;


#if 0
static void app_destroy_anim_exec_callback(void *app_id, lv_anim_value_t value)
{
    lv_obj_t *main_win = gui_app_get_main_window((gui_app_id_t)app_id);

    if (main_win)
    {
        lv_area_t area;

        lv_obj_get_coords(main_win, &area);

        switch (gui_app_close_anim_type)
        {
        case GUI_APP_CLOSE_ANIM_SLIDING_DOWN:
            lv_obj_set_pos(main_win, area.x1, value);
            break;
        case GUI_APP_CLOSE_ANIM_SLIDING_CENTER:
            lv_obj_set_size(main_win, value, value);
            lv_obj_align(main_win, NULL, LV_ALIGN_CENTER, 0, 0);
            break;
        case GUI_APP_CLOSE_ANIM_FADIN:
            lv_obj_set_opa_scale(main_win, value);
            break;
        default:
            break;
        }
    }

}

/**
 * destory last app  after animation finished
 * \n
 *
 * @param a
 * \n
 * @see
 */
static void app_destroy_anim_ready_callback(lv_anim_t *a)
{
    gui_app_id_t app_id;

    app_id = (gui_app_id_t)a->var;
    if (app_id < GUI_APP_ID_NUM)
    {
        if (gui_app_list[app_id].destroy)
        {
            gui_app_list[app_id].destroy();
            g_last_app_id = GUI_APP_ID_MAIN;
        }
    }
    destory_ani_done = true;
}
#endif


void gui_app_enable_input_device(bool enable)
{
    port_app_sche_enable_indev(enable, false);
}

void gui_app_enable_input_device_except_tp(bool enable)
{
    port_app_sche_enable_indev(enable, true);
}

char *fwk_msg_to_name(gui_app_frmmsg_type_t msg)
{
#define EVENT_TO_NAME_CASE(msg) case msg: return #msg
    switch (msg)
    {
        EVENT_TO_NAME_CASE(GUI_APP_MSG_RUN_APP);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_RUN_APP_I);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_EXIT_APP);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_GOBACK);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_OPEN_PAGE);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_RM_PAGE);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_CLEANUP);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_MANUAL_GOBACK_ANIM);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_GOBACK_TO_PAGE);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_SUSPEND_SCHEDULER);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_RESUME_SCHEDULER);
        EVENT_TO_NAME_CASE(GUI_APP_MSG_ABORT_TRANS_ANIM);


    default:
        return "UNKNOW";
    }
}


static rt_err_t send_msg_to_gui_app_task(gui_app_msg_t *msg)
{
    rt_err_t err;
    uint32_t *p_msg;

    //Disable input device in case of mailbox overflow at corner case.
    if (GUI_APP_MSG_MANUAL_GOBACK_ANIM != msg->msg_id)
        gui_app_enable_input_device(false);
    else
        gui_app_enable_input_device_except_tp(false);


    //Restore msg to allocated memmory
    p_msg = (uint32_t *) rt_malloc(sizeof(gui_app_msg_t));
    if (NULL == p_msg)
    {
        LOG_I("Malloc msg fail.");
        RT_ASSERT(0);
    }
    else
    {
        msg->tick = rt_tick_get();
        rt_memcpy(p_msg, msg, sizeof(gui_app_msg_t));
    }

    //Send to mailbox
    err = rt_mb_send(&gui_app_mbx, (rt_uint32_t) p_msg);

    if (err != RT_EOK)
    {
        LOG_I("send to gui_app_mbx err:%d", err);
        RT_ASSERT(0);
    }
    else
    {
        if (GUI_APP_MSG_RUN_APP == msg->msg_id)
        {
            LOG_D("send msg[%s] [%s] to gui_app_mbx tick:%d.", fwk_msg_to_name(msg->msg_id), msg->content.cmd, msg->tick);
            printf_intent(&msg->content.intnt);
        }
        else
            LOG_D("send msg[%s] [0x%x] to gui_app_mbx tick:%d.", fwk_msg_to_name(msg->msg_id), msg->handler, msg->tick);
    }

    return err;
}


static void sche_task(task_t h)
{
    app_schedule_task(&gui_app_mbx);
}

/**
 * run an app
 * \n
 *
 * @param cmd - format:[app_cmd] [param0] [param1] [param2] ...
 *
 *
 * \n
 * @see
 */
int gui_app_run(const char *cmd)
{
    gui_app_msg_t msg;

    rt_err_t err;
    memset(&msg, 0, sizeof(gui_app_msg_t));

    intent_t i = intent_init(cmd);

    if (NULL == i)
    {
        err = RT_ERROR;
    }
    else
    {
        memcpy(&msg.content.intnt, i, sizeof(msg.content.intnt));
        msg.msg_id = GUI_APP_MSG_RUN_APP;
        msg.handler = NULL;

        err = send_msg_to_gui_app_task(&msg);
        intent_deinit(i);
    }

    return err;
}


int gui_app_exit(const char *id)
{
    gui_app_msg_t msg;
    rt_err_t err;

    memset(&msg, 0, sizeof(gui_app_msg_t));
    msg.msg_id = GUI_APP_MSG_EXIT_APP;
    msg.handler = NULL;
    strncpy(msg.content.cmd, id, GUI_APP_CMD_MAX_LEN);

    err = send_msg_to_gui_app_task(&msg);

    return err;
}

int gui_app_run_by_intent(intent_t intent)
{
    gui_app_msg_t msg;
    rt_err_t err;

    msg.msg_id = GUI_APP_MSG_RUN_APP;
    msg.handler = NULL;

    memcpy(&msg.content.intnt, intent, sizeof(msg.content.intnt));

    err = send_msg_to_gui_app_task(&msg);

    return err;
}

gui_runing_app_t *gui_app_get_actived(void)
{
    CHECK_CUR_RTOS_TASK();

    return app_schedule_get_active();
}

bool gui_app_is_all_closed(void)
{
    if (app_schedule_get_active())
    {

        return false;
    }
    else
    {
        return true;
    }
}

subpage_node_t *gui_app_page_get_actived(void)
{
    CHECK_CUR_RTOS_TASK();
    gui_runing_app_t *p_app = app_schedule_get_active();

    if (NULL != p_app)
    {
        if (rt_list_isempty(&p_app->page_list))
            return NULL;
        else
        {
            return rt_list_first_entry(&p_app->page_list, subpage_node_t, node);
        }
    }
    else
        return NULL;
}

bool gui_app_is_actived(char *id)
{
    bool ret;
    gui_runing_app_t *p_app;

    rt_enter_critical();
    p_app = app_schedule_get_active();

    if (NULL != p_app)
    {
        ret = (0 == strcmp(p_app->id, id)) ? RT_TRUE : RT_FALSE;
    }
    else
    {
        ret = RT_FALSE;
    }

    rt_exit_critical();

    return ret;
}

bool gui_app_is_page_present(char *id)
{
    bool ret;

    CHECK_CUR_RTOS_TASK();

    gui_runing_app_t *this_app = app_schedule_get_this();

    if (this_app)
    {
        ret = (NULL != app_schedule_get_page_in_app(this_app, id)) ? RT_TRUE : RT_FALSE;
    }
    else
    {
        ret = RT_FALSE;
    }

    return ret;
}


void gui_app_self_exit(void)
{
    gui_app_msg_t msg;
    gui_runing_app_t    *p_this_app;
    CHECK_CUR_RTOS_TASK();

    p_this_app = app_schedule_get_this();

    if (p_this_app)
    {
        memset(&msg, 0, sizeof(gui_app_msg_t));
        msg.msg_id = GUI_APP_MSG_EXIT_APP;
        msg.handler = NULL;
        strncpy(msg.content.cmd, &p_this_app->id[0], GUI_APP_CMD_MAX_LEN);

        send_msg_to_gui_app_task(&msg);
    }
}

void gui_app_cleanup(void)
{
    gui_app_msg_t msg;

    memset(&msg, 0, sizeof(gui_app_msg_t));
    msg.msg_id = GUI_APP_MSG_CLEANUP;
    msg.handler = NULL;

    send_msg_to_gui_app_task(&msg);
}

void gui_app_abort_trans_anim(void)
{
    gui_app_msg_t msg;

    memset(&msg, 0, sizeof(gui_app_msg_t));
    msg.msg_id = GUI_APP_MSG_ABORT_TRANS_ANIM;
    msg.handler = NULL;

    send_msg_to_gui_app_task(&msg);
}

void gui_app_cleanup_bg_apps(void)
{
    gui_app_msg_t msg;

    memset(&msg, 0, sizeof(gui_app_msg_t));
    msg.msg_id = GUI_APP_MSG_CLEANUP_BACKGROUND;
    msg.handler = NULL;

    send_msg_to_gui_app_task(&msg);
}

void gui_app_cleanup_now(void)
{
    gui_app_msg_t msg;
    uint8_t max_cnt = 3;
    CHECK_CUR_RTOS_TASK();

    if (0 == app_schedule_get_running_apps())
    {
        //None app is running now, ignore.
        return;
    }

    memset(&msg, 0, sizeof(gui_app_msg_t));
    msg.msg_id = GUI_APP_MSG_CLEANUP;
    msg.handler = NULL;

    LOG_I("gui_app_cleanup_now Start");


    app_schedule_enable_trans_anim(false);
    do
    {
        RT_ASSERT(!app_scheduler_is_suspend());
        send_msg_to_gui_app_task(&msg);
        sche_task(task_handler);

        RT_ASSERT(0 < max_cnt--);
        LOG_I("gui_app_cleanup_now Cnt %d", max_cnt);
    }
    while (app_schedule_get_running_apps() != 0);
    app_schedule_enable_trans_anim(true);

    LOG_I("gui_app_cleanup_now Done.");
}

void gui_app_run_now(const char *cmd)
{
    CHECK_CUR_RTOS_TASK();
    uint8_t max_cnt = 3;
    rt_err_t err;

    LOG_I("gui_app_run_now Start");
    //Make sure previous msg handled.
    app_schedule_enable_trans_anim(false);

    gui_app_abort_trans_anim();
    do
    {
        app_schedule_task(&gui_app_mbx);

        LOG_I("gui_app_exec_now Cnt %d", max_cnt++);

        /*'s_sche_state' will always be 'APP_SCHE_SUSPEND'
          if sheduler is suspend
         */
        if (app_scheduler_is_suspend()) break;
    }
    while (APP_SCHE_SUSPEND == s_sche_state);


    intent_t intent = intent_init(cmd);

    if (NULL != intent)
    {
        err = gui_app_run_by_intent(intent);
    }
    else
    {
        err = RT_ERROR;
    }

    if (RT_EOK == err)
    {
        do
        {
            RT_ASSERT(!app_scheduler_is_suspend());
            sche_task(task_handler);

            RT_ASSERT(0 < max_cnt--);
            LOG_I("gui_app_run_now Cnt %d", max_cnt);
        }
        while (NULL == app_schedule_is_app_running(intent_get_action(intent)));
        LOG_I("gui_app_run_now Done.");
    }
    else
    {
        LOG_E("gui_app_run_now Fail %d.", err);
    }

    intent_deinit(intent);
    app_schedule_enable_trans_anim(true);


}

intent_t gui_app_get_intent(void)
{
    CHECK_CUR_RTOS_TASK();

    gui_runing_app_t *run_app = app_schedule_get_this();

    if (NULL != run_app)
    {
        return &(run_app->param);
    }
    else
        return NULL;
}

void *gui_app_get_page_userdata(const char *pg_id)
{
    gui_runing_app_t *this_app;
    subpage_node_t *p_page;
    CHECK_CUR_RTOS_TASK();

    this_app = app_schedule_get_this();
    if (this_app)
    {
        p_page = app_schedule_get_page_in_app(this_app, pg_id);

        if (p_page) return p_page->user_data;
    }

    return NULL;
}

int gui_app_set_page_userdata(const char *pg_id, void *usr_data)
{
    gui_runing_app_t *this_app;
    subpage_node_t *p_page;
    CHECK_CUR_RTOS_TASK();

    this_app = app_schedule_get_this();
    if (this_app)
    {
        p_page = app_schedule_get_page_in_app(this_app, pg_id);

        if (p_page)
        {
            p_page->user_data = usr_data;
            return RT_EOK;
        }
    }

    return RT_ERROR;
}



/**
 * @brief Get current subpage's user data
 * @return NULL if no data
 */
void *gui_app_this_page_userdata(void)
{
    subpage_node_t *p_page;
    CHECK_CUR_RTOS_TASK();

    p_page = app_schedule_get_this_page();

    if (p_page)
        return p_page->user_data;
    else
        return NULL;
}



static builtin_app_desc_t *builtin_app_next(const builtin_app_desc_t *ptr_app)
{
    ptr_app++;

    //handle the align between setion and structure.
#if defined(BSP_USING_PC_SIMULATOR)
    //ptr_app = (builtin_app_desc_t *)((uint8_t *)ptr_app + 8);

    for (uint32_t tmp = (uint32_t)ptr_app ;  tmp < (uint32_t)builtin_app_table_end; tmp++)
    {
        if (MSC_APP_STRUCT_MAGIC_HEAD == ((builtin_app_desc_t *)tmp)->magic_flag)
        {
            ptr_app = (builtin_app_desc_t *)tmp;
            break;
        }
    }
#endif

    if ((unsigned int *)ptr_app >= (unsigned int *) builtin_app_table_end)
        return NULL;
    else
        return (builtin_app_desc_t *)ptr_app;
}


const builtin_app_desc_t *gui_builtin_app_list_open(void)
{
    return builtin_app_table_begin;
}

const builtin_app_desc_t *gui_builtin_app_list_get_next(const builtin_app_desc_t *ptr_app)
{
    return builtin_app_next(ptr_app);
}

void gui_builtin_app_list_close(const builtin_app_desc_t *ptr_app)
{

}

uint32_t gui_app_get_running_apps(void)
{
    CHECK_CUR_RTOS_TASK();

    return app_schedule_get_running_apps();
}

void gui_app_exec_now(void)
{
    uint32_t max_cnt = 0;

    //Must run on host task.
    CHECK_CUR_RTOS_TASK();

    LOG_I("gui_app_exec_now Start");

    app_schedule_enable_trans_anim(false);


    gui_app_abort_trans_anim();
    do
    {
        app_schedule_task(&gui_app_mbx);

        LOG_I("gui_app_exec_now Cnt %d", max_cnt++);

        /*'s_sche_state' will always be 'APP_SCHE_SUSPEND'
          if sheduler is suspend
         */
        if (app_scheduler_is_suspend()) break;
    }
    while (APP_SCHE_SUSPEND == s_sche_state);

    app_schedule_enable_trans_anim(true);

    LOG_I("gui_app_exec_now Done.");

}

static void app_load(const char *id, app_entity_info *p_app_info)
{
#ifdef DL_APP_SUPPORT
    struct rt_dlmodule *module = RT_NULL;
#endif

    const builtin_app_desc_t *item = NULL;

    LOG_I("finding %s in builtin apps...", id);
    item = gui_builtin_app_list_open();

    while (item != NULL)
    {

        if (0 == strcmp(item->id, id))
            break;

        item = gui_builtin_app_list_get_next(item);
    }
    gui_builtin_app_list_close(item);

    if (item == NULL)
    {
        item = gui_script_app_list_get_next(item);
        while (item != NULL)
        {

            if (0 == strcmp(item->id, id))
            {
                // Make sure to close file search, TODO: Check python as well
                gui_script_app_list_get_next((const builtin_app_desc_t *) -1);
                break;
            }
            item = gui_script_app_list_get_next(item);
        }
    }



    if (NULL != item)
    {
        p_app_info->entry_func = (uint32_t) item->entry;
        p_app_info->user_data    = 0;
    }
    else
    {
#ifdef DL_APP_SUPPORT
        LOG_I("find %s in dl_apps...", id);

        // TODO: find in dl apps folder
        //module = (struct rt_dlmodule *)dlopen(id, 0);
        module = (struct rt_dlmodule *)gui_app_dlopen(id, 0);

        if (module)
        {
            /*
                        LOG_I("dl-app init_f:%x,clean_f:%x,entry_f:%x", (uint32_t)module->init_func,
                                                                    (uint32_t)module->cleanup_func,
                                                                    (uint32_t)module->entry_addr);
            */

            p_app_info->entry_func = (uint32_t) dlsym(module, "app_main");
            p_app_info->user_data    = (uint32_t) module;
        }
        else
        {
            p_app_info->entry_func = 0;
            p_app_info->user_data = 0;

            LOG_I("find app %s in dl_apps FAILED!", id);
        }
#else
        p_app_info->entry_func = 0;
        p_app_info->user_data = 0;
#endif
    }


}

static void app_destory(const char *app_id, const app_entity_info *p_app_info)
{
#ifdef DL_APP_SUPPORT
    if (p_app_info->user_data)
    {
        //dlclose((void *)p_app->module_id);
        gui_app_dlclose((void *)p_app_info->user_data);
    }
#endif

}

static void app_debug_func(uint8_t level, const char *fmt, ...)
{

    va_list args;
    va_start(args, fmt);

#ifdef RT_USING_ULOG
    rt_uint32_t ulog_level;

    switch (level)
    {
    case APP_SCHE_LOG_LEVEL_DEBUG:
        ulog_level = LOG_LVL_DBG;
        break;
    case APP_SCHE_LOG_LEVEL_INFO:
        ulog_level = LOG_LVL_INFO;
        break;
    case APP_SCHE_LOG_LEVEL_ASSERT:
        ulog_level = LOG_LVL_ASSERT;
        break;

    case APP_SCHE_LOG_LEVEL_ERROR:
    default:
        ulog_level = LOG_LVL_ERROR;
        break;

    }

    ulog_voutput(ulog_level, "APP.SCHE", true, fmt, args);
#else
    static char log_buf[RT_CONSOLEBUF_SIZE];
    rt_size_t length;

    length = rt_vsnprintf(log_buf, sizeof(log_buf) - 1, fmt, args);
    if (length > RT_CONSOLEBUF_SIZE - 1)
        length = RT_CONSOLEBUF_SIZE - 1;
    log_buf[length] = '\0';

    switch (level)
    {
    case APP_SCHE_LOG_LEVEL_DEBUG:
        rt_kputs("[APP.SCHE.D]");
        break;
    case APP_SCHE_LOG_LEVEL_INFO:
        rt_kputs("[APP.SCHE.I]");
        break;
    case APP_SCHE_LOG_LEVEL_ASSERT:
        rt_kputs("[APP.SCHE.A]");
        break;

    case APP_SCHE_LOG_LEVEL_ERROR:
    default:
        rt_kputs("[APP.SCHE.E]");
        break;

    }

    rt_kputs(log_buf);
    rt_kputs("\n");
#endif

    if (level == APP_SCHE_LOG_LEVEL_ASSERT)
    {
        list_schedule_app();
        RT_ASSERT(0);
    }


    va_end(args);
}







#if defined (_MSC_VER)
#pragma section("BuiltinAppTab$0", read)
__declspec(allocate("BuiltinAppTab$0")) RT_USED static const builtin_app_desc_t __builtin_app_table_start =
{
    MSC_APP_STRUCT_MAGIC_HEAD,
#ifdef LV_USING_EXT_RESOURCE_MANAGER
    1,              // Not used.
#else
    "__USTART",
#endif
    NULL,
    "",
    NULL
};

#pragma section("BuiltinAppTab$1.end", read)
__declspec(allocate("BuiltinAppTab$1.end"))RT_USED static const builtin_app_desc_t __builtin_app_table_end =
{
    MSC_APP_STRUCT_MAGIC_HEAD,
#ifdef LV_USING_EXT_RESOURCE_MANAGER
    2,              // Not used
#else
    "__USTOP",
#endif
    NULL,
    "",
    NULL
};
#endif


static void app_scheduler_idle_hook(app_sche_state state)
{
    if (APP_SCHE_IDLE == state)
    {
#if defined(RT_USING_FINSH) && defined (SOLUTION_WATCH)
        extern void app_mem_check(void);
        app_mem_check();
#endif
    }
    /*
        Resume indev even if animation is playing, so user can abort it.
        And none obj event of app should be executed during animation.
    */
    gui_app_enable_input_device(true);

    s_sche_state = state;
}

app_sche_state app_schedule_state_get(void)
{
    return s_sche_state;
}

static void subpage_msg_handler(gui_page_msg_cb_t func, gui_app_msg_type_t msg_id, const char *app_id, const char *subpage_id)
{
    //LOG_I("%s.%s do [%d]",app_id,subpage_id, msg_id);

    (void) subpage_id;
    func(msg_id, (void *)app_id);
}


void gui_app_init(void)
{
#if defined(__CC_ARM) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))                                 /* ARM C Compiler */
    extern const int BuiltinAppTab$$Base;
    extern const int BuiltinAppTab$$Limit;
    builtin_app_table_begin = (builtin_app_desc_t *) &BuiltinAppTab$$Base;
    builtin_app_table_end = (builtin_app_desc_t *)   &BuiltinAppTab$$Limit;
#elif defined (__ICCARM__) || defined(__ICCRX__)      /* for IAR Compiler */
#error "tobe contribute"
#elif defined (__GNUC__)                              /* for GCC Compiler */
    extern const int BuiltinAppTab_start;
    extern const int BuiltinAppTab_end;
    builtin_app_table_begin = (builtin_app_desc_t *)&BuiltinAppTab_start;
    builtin_app_table_end = (builtin_app_desc_t *) &BuiltinAppTab_end;

#elif defined (_MSC_VER)
    uint32_t *ptr_begin, *ptr_end;
    ptr_begin = (uint32_t *)&__builtin_app_table_start;
    ptr_begin += (sizeof(builtin_app_desc_t) / sizeof(uint32_t));
    while (*((uint32_t *)ptr_begin) == 0) ptr_begin ++;

    ptr_end = (uint32_t *)&__builtin_app_table_end;
    ptr_end --;
    while (*((uint32_t *)ptr_end) == 0) ptr_end --;
    ptr_end++;


    builtin_app_table_begin = (builtin_app_desc_t *)ptr_begin;
    builtin_app_table_end = (builtin_app_desc_t *)ptr_end;
#endif /* defined(__CC_ARM) */

    if (0 != app_schedule_init(app_load, app_destory, app_debug_func, app_scheduler_idle_hook, rt_malloc, rt_free, subpage_msg_handler))
    {
        RT_ASSERT(0);
    }
    else
    {
        /* messagequeue init */
        rt_mb_init(&gui_app_mbx, "appfwk",
                   &gui_app_mbx_buffer_pool[0],
                   GUI_APP_MBX_SIZE,
                   RT_IPC_FLAG_FIFO);

        task_handler = port_app_sche_task_create(sche_task, GUI_LIB_REFR_PERIOD_CNT(2), NULL);


        app_trans_animation_init();

        app_schedule_change_main_app_id("Main");
#ifdef GUI_MAX_RUNNING_APPS
        app_schedule_max_running_apps(GUI_MAX_RUNNING_APPS);
#endif /* GUI_MAX_RUNNING_APPS */
        app_schedule_destory_suspend_apps(0);
    }


    host_rtos_task = rt_thread_self();
}

void gui_app_deinit(void)
{
    CHECK_CUR_RTOS_TASK();

    port_app_sche_task_del(task_handler);
    rt_mb_detach(&gui_app_mbx);

}

int gui_app_create_page_ext(const char *pg_id, gui_page_msg_cb_t handler, void *usr_data)
{
    gui_app_msg_t msg;
    CHECK_CUR_RTOS_TASK();

    if (strlen(pg_id) < sizeof(msg.content.page.name))
    {
        memset(&msg, 0, sizeof(gui_app_msg_t));
        msg.msg_id = GUI_APP_MSG_OPEN_PAGE;
        msg.handler = app_schedule_get_this();

        strcpy(msg.content.page.name, pg_id);
        msg.content.page.msg_handler = handler;
        msg.content.page.user_data = usr_data;

        return send_msg_to_gui_app_task(&msg);
    }
    else
    {
        LOG_I("gui_app_create_page err : invalid id: %s", pg_id);
        return RT_EINVAL;
    }

}

int gui_app_create_page_for_app_ext(const char *app_id, const char *pg_id, gui_page_msg_cb_t handler, void *usr_data)
{
    gui_app_msg_t msg;
    gui_runing_app_t    *p_app;
    CHECK_CUR_RTOS_TASK();

    p_app = app_schedule_is_app_running(app_id);
    if (!p_app)
    {
        LOG_E("gui_app_create_page_for_app err : invalid id: %s", app_id);
        return RT_EINVAL;
    }

    if (strlen(pg_id) < sizeof(msg.content.page.name))
    {
        memset(&msg, 0, sizeof(gui_app_msg_t));
        msg.msg_id = GUI_APP_MSG_OPEN_PAGE;
        msg.handler = p_app;

        strcpy(msg.content.page.name, pg_id);
        msg.content.page.msg_handler = handler;
        msg.content.page.user_data = usr_data;

        return send_msg_to_gui_app_task(&msg);
    }
    else
    {
        LOG_I("gui_app_create_page_for_app err : invalid id: %s", pg_id);
        return RT_EINVAL;
    }

}

int gui_app_regist_msg_handler_ext(const char *id, gui_page_msg_cb_t handler, void *usr_data)
{
    gui_runing_app_t *p_app;
    CHECK_CUR_RTOS_TASK();

    //Find app handler by id
    p_app = app_schedule_get_this();
    if (p_app)
        if (0 != strncmp(p_app->id, id, GUI_APP_ID_MAX_LEN))
        {
            p_app = app_schedule_is_app_running(id);
        }

    if (p_app)
        if (0 == strncmp(p_app->id, id, GUI_APP_ID_MAX_LEN))
        {

            gui_app_msg_t msg;

            memset(&msg, 0, sizeof(gui_app_msg_t));
            msg.msg_id = GUI_APP_MSG_OPEN_PAGE;
            msg.handler = p_app;

            strcpy(msg.content.page.name, "root");
            msg.content.page.msg_handler = handler;
            msg.content.page.user_data   = usr_data;

            return send_msg_to_gui_app_task(&msg);
        }

    return RT_ERROR; //NOT find app
}

void gui_app_remove_page(const char *pg_id)
{
    gui_app_msg_t msg;
    CHECK_CUR_RTOS_TASK();

    if (strlen(pg_id) < sizeof(msg.content.page.name))
    {
        memset(&msg, 0, sizeof(gui_app_msg_t));
        msg.msg_id = GUI_APP_MSG_RM_PAGE;
        msg.handler = app_schedule_get_this();

        strcpy(msg.content.page.name, pg_id);


        send_msg_to_gui_app_task(&msg);
    }
    else
    {
        LOG_I("gui_app_remove_page err : invalid id: %s", pg_id);
    }

}

void gui_app_refr_page(const char *pg_id)
{
    gui_app_msg_t msg;
    CHECK_CUR_RTOS_TASK();

    if (strlen(pg_id) < sizeof(msg.content.page.name))
    {
        memset(&msg, 0, sizeof(gui_app_msg_t));
        msg.msg_id = GUI_APP_MSG_REFRESH_PAGE;
        msg.handler = app_schedule_get_this();

        strcpy(msg.content.page.name, pg_id);


        send_msg_to_gui_app_task(&msg);
    }
    else
    {
        LOG_I("gui_app_refr_page err : invalid id: %s", pg_id);
    }

}

int gui_app_goback_to_page(const char *pg_id)
{
    gui_app_msg_t msg;
    CHECK_CUR_RTOS_TASK();

    if (strlen(pg_id) < sizeof(msg.content.page.name))
    {
        gui_runing_app_t *this_app;
        subpage_node_t *p_page;

        this_app = app_schedule_get_this();
        if (this_app)
        {
            p_page = app_schedule_get_page_in_app(this_app, pg_id);

            if (p_page)
            {
                memset(&msg, 0, sizeof(gui_app_msg_t));
                msg.msg_id = GUI_APP_MSG_GOBACK_TO_PAGE;
                msg.handler = app_schedule_get_this();

                strcpy(msg.content.page.name, pg_id);
                return send_msg_to_gui_app_task(&msg);
            }
            else
            {
                LOG_E("gui_app_goback_to_page : can't fine pg_id: %s, in app %s", pg_id, this_app->id);
            }
        }

        return -RT_EEMPTY;
    }
    else
    {
        LOG_E("gui_app_goback_to_page : invalid pg_id: %s", pg_id);
        return -RT_EINVAL;
    }
}

int gui_app_goback(void)
{
    gui_app_msg_t msg;
    CHECK_CUR_RTOS_TASK();

    memset(&msg, 0, sizeof(gui_app_msg_t));
    msg.msg_id = GUI_APP_MSG_GOBACK;
    msg.handler = app_schedule_get_this();

    return send_msg_to_gui_app_task(&msg);
}


int gui_app_manual_goback_anim(void)
{
    gui_app_msg_t msg;
    CHECK_CUR_RTOS_TASK();

    memset(&msg, 0, sizeof(gui_app_msg_t));
    msg.msg_id = GUI_APP_MSG_MANUAL_GOBACK_ANIM;
    msg.handler = app_schedule_get_this();

    return send_msg_to_gui_app_task(&msg);
}

int gui_app_fwk_suspend(void)
{
    gui_app_msg_t msg;

    memset(&msg, 0, sizeof(gui_app_msg_t));
    msg.msg_id = GUI_APP_MSG_SUSPEND_SCHEDULER;

    return send_msg_to_gui_app_task(&msg);
}

int gui_app_fwk_resume(void)
{
    if (app_scheduler_is_suspend())
    {
        app_scheduler_resume();
        gui_app_msg_t msg;

        memset(&msg, 0, sizeof(gui_app_msg_t));
        msg.msg_id = GUI_APP_MSG_RESUME_SCHEDULER;

        return send_msg_to_gui_app_task(&msg);
    }
    else
    {
        return RT_ERROR;
    }
}

void gui_app_fwk_test_start(void)
{
    test_mode = 1;
}
void gui_app_fwk_test_stop(void)
{
    test_mode = 0;
}


#ifdef FINSH_USING_MSH
#include <finsh.h>
static rt_err_t app_run(int argc, char **argv)
{
    char cmd_buf[GUI_APP_CMD_MAX_LEN];
    rt_err_t res;
    int i, buf_len, str_len;

    if (argc < 2)
        return -RT_ERROR;

    LOG_I("");
    buf_len = GUI_APP_CMD_MAX_LEN - 1;
    memset(cmd_buf, 0, sizeof(cmd_buf));

    for (i = 1; (i < argc) && (buf_len > 0); i++)
    {
        str_len = strlen(argv[i]) + 1/*1 space*/;
        if (buf_len < str_len)
        {
            break;
        }
        else
        {
            strcat(cmd_buf, " ");
            strcat(cmd_buf, argv[i]);
        }
        buf_len -= str_len;
    }


    res = gui_app_run(cmd_buf);
    return res;
}
FINSH_FUNCTION_EXPORT(app_run, run an app);
MSH_CMD_EXPORT(app_run, run an app);

static rt_err_t app_cleanup(int argc, char **argv)
{
    gui_app_cleanup();
    return 0;
}

FINSH_FUNCTION_EXPORT(app_cleanup, cleanup all background app);
MSH_CMD_EXPORT(app_cleanup, cleanup all background app);

static rt_err_t app_goback(int argc, char **argv)
{
    gui_app_fwk_test_start();
    gui_app_goback();
    gui_app_fwk_test_stop();
    return 0;
}
FINSH_FUNCTION_EXPORT(app_goback, go back current page);
MSH_CMD_EXPORT(app_goback, go back current page);


static rt_err_t app_exit(int argc, char **argv)
{
    if (argc > 1)
    {
        gui_app_exit(argv[1]);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT(app_exit, exit specifed app);
MSH_CMD_EXPORT(app_exit, exit specifed app);

rt_err_t list_app(int argc, char **argv)
{
    {
        const builtin_app_desc_t *item = NULL;
        LOG_I("##Built in apps##");

        item = gui_builtin_app_list_open();

        LOG_I("id\tname\ticon\tentry");
        while (item != NULL)
        {

#ifdef LV_USING_EXT_RESOURCE_MANAGER
            LOG_I("%s,\t%s,\t%p,\t%p", item->id, LV_EXT_STR_GET(item->name), item->icon, item->entry);
#else
            LOG_I("%s,\t%s,\t%p,\t%p", item->id, item->name, item->icon, item->entry);
#endif /* LV_USING_EXT_RESOURCE_MANAGER */

            item = gui_builtin_app_list_get_next(item);
        }

        gui_builtin_app_list_close(item);
    }

    LOG_I("##Runing apps##");
    list_schedule_app();

    return 0;
}

FINSH_FUNCTION_EXPORT(list_app, list all runing app);
MSH_CMD_EXPORT(list_app, list all runing app);



rt_err_t app_sche_print_perf_tick(int argc, char **argv)
{
    if (argc > 1)
    {
        app_scheduler_print_perf_tick((uint8_t)strtol(argv[1], 0, 10));
    }

    return 0;
}


MSH_CMD_EXPORT(app_sche_print_perf_tick, Print app &subpage costed ticks);

#endif


#ifdef RT_USING_FINSH
int avitive_app_is_main(void)
{
    gui_runing_app_t *cur_app = app_schedule_get_active();

    //if (0 == strcmp(cur_app->id, "Main_list"))
    //{
    //    return 2;
    //}

    if (cur_app && 0 == strcmp(cur_app->id, "Main"))
    {
        return 1;
    }


    return 0;
}

char *get_avitive_app_id(void)
{
    gui_runing_app_t *cur_app = app_schedule_get_active();

    return cur_app->id;
}


#endif


RTM_EXPORT(gui_app_run);
RTM_EXPORT(gui_app_self_exit);
RTM_EXPORT(gui_app_regist_msg_handler_ext);


