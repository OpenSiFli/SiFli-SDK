/*********************
 *      INCLUDES
 *********************/

#include "gui_app_int.h"

#define DEFAULT_MAIN_APP_ID  "Main"
typedef enum
{
    app_st_none      = 0x00,
    app_st_launched  = 0x01,
    app_st_running   = 0x02,
    app_st_suspended   = 0x03, //All subpages of app will be stopped
    app_st_destoryed   = 0x04, //None subpages in app
} app_state_enum;

typedef enum
{
    APP_EXEC_NONE     = 0x00,

    /*Start app(call app entry function)*/
    APP_EXEC_START    = 0x01,

    /*suspend app, move to suspend_app_list*/
    APP_EXEC_SUSPEND    = 0x02,

    /*Call app->entry_f to restart app*/
    APP_EXEC_RESTART  = 0x03,

    /*
        Move to 'rm_app_list' and destory it.
    */
    APP_EXEC_DESTORY  = 0x04,

    /*Load app*/
    APP_EXEC_LOAD     = 0x05,

} app_exe_enum;

typedef enum
{
    /*Play go-back animation*/
    APP_FLAG_GOBACK_ANIM   = 0x01,
} APP_FLAG_ENUM;

static uint32_t max_running_apps = 2; //!< mainmenu & avtive app
static uint8_t en_suspend_app = 0; //Save the history of apps stopped by app scheduler when > max_running_apps
static uint8_t en_check_duplicated_subpage = 1;
static uint8_t en_check_duplicated_apps = 1;
/**
    gui app running list

    sorted by last resume time descend

    example: open app A->B->C
        List' next is C, C'next is B, B'next is A

        Next direction:   List--->C--->B--->A
                           ^                |
                           |________________|
*/
static rt_list_t running_app_list;

/*
    Two type apps here:
    1. Apps suspend by scheduler because running_apps > max_running_apps,
        and they will be restarted when 'goback' executed.
    2. Apps exited(go back) by user, and will be destoryed after all msgs executed.



    sorted by suspended time descend
*/
static rt_list_t suspend_app_list;

/**
 * focoused app, visible
 */
static rt_list_t *actived_app = NULL;


/**
 * scheduling app by app_schedule
 */
static rt_list_t *schedule_app = NULL;

/**
 * scheduling page by app_schedule
 */
static rt_list_t *schedule_page = NULL;

/**
 * dummy screen handler
 */
static screen_t dummy_scr = NULL;
/*
    Registed function pointers
*/
static app_load_func sche_app_ld_func = NULL;
static app_destory_func sche_app_dstry_func = NULL;
static app_sche_debug_func sche_app_debug_func = NULL;
static app_sche_hook_func   sche_idle_hook_func = NULL;
static app_sche_malloc_func sche_malloc_func = NULL;
static app_sche_free_func   sche_free_func = NULL;
static app_sche_subpage_func sche_subpage_func = NULL;

#define TRANS_ANIMATION  1
#ifdef TRANS_ANIMATION
    static bool disable_trans_anim = false;
    static bool trans_anim_playing = false;
    static bool manual_trans_anim = false; //Execute 'GUI_APP_MSG_MANUAL_GOBACK_ANIM' msg.
    static screen_t act_scr_before_app_trans_anim = NULL;
    static subpage_node_t *trans_anim_pg_enter = NULL;
    static subpage_node_t *trans_anim_pg_exit  = NULL;
#endif

/*
1. Close all other app while open/resume main app
2. Open default app while the last app being closed.
*/
static const char *p_main_app_id = DEFAULT_MAIN_APP_ID;
/*
 Always play goback animation while open first subpage of desktop app
*/
static const char *p_desktop_app_id = DEFAULT_MAIN_APP_ID;
static rt_sem_t mutex_lock = NULL;
/*break app_schedule_task loop, and not execute it util suspend_task be 0*/
#define scheduler_resumed 0
#define scheduler_suspended 1
static uint8_t suspend_task = scheduler_resumed;
/* 0 - not in app_schedule task, 1 - just enter task, 2 - printed enter log*/
static uint8_t scheduler_loop_st = 0;
static rt_mailbox_t task_msg_mbx;
/*Print app&subpage performance tick*/
static uint8_t app_subgpage_perf_tick = 0;

#define app_sche_log(LEVEL, ...)  do{ if(sche_app_debug_func) {\
                                            if(1 == scheduler_loop_st){\
                                                sche_app_debug_func(APP_SCHE_LOG_LEVEL_DEBUG, "----------------app_schedule_task---------------start");\
                                                scheduler_loop_st = 2;\
                                            }\
                                            sche_app_debug_func(LEVEL, ##__VA_ARGS__);\
                                       }\
                                    }while(0)

#define app_sche_d(...)                     app_sche_log(APP_SCHE_LOG_LEVEL_DEBUG,  ##__VA_ARGS__)
#define app_sche_i(...)                     app_sche_log(APP_SCHE_LOG_LEVEL_INFO,   ##__VA_ARGS__)
#define app_sche_e(...)                     app_sche_log(APP_SCHE_LOG_LEVEL_ERROR,  ##__VA_ARGS__)
#define app_sche_assert(...)                app_sche_log(APP_SCHE_LOG_LEVEL_ASSERT, ##__VA_ARGS__)
#define app_sche_malloc(size)          sche_malloc_func((size))
#define app_sche_free(ptr)             sche_free_func((ptr))


#define app_sche_lock()     lock(1)
#define app_sche_unlock()   lock(-1)
static uint32_t app_scheduler_new(void);
static void app_destory(rt_list_t *app_node);
static void app_destory_list(rt_list_t *list);
static uint32_t app_run(rt_mailbox_t msg_mbx, const _intent *i, uint32_t tick);
static uint32_t app_run_by_id(rt_mailbox_t msg_mbx, const char *id);

/************************************

  Export APIs

************************************/
bool app_scheduler_is_suspend(void)
{
    return (scheduler_suspended == suspend_task);
}

/*
void app_scheduler_suspend(void)
{
    if(scheduler_resumed == suspend_task)
        suspend_task = scheduler_suspend_request;
}
*/

void app_scheduler_resume(void)
{

    if (scheduler_suspended == suspend_task)
    {
        suspend_task = scheduler_resumed;
    }
}

gui_runing_app_t *app_schedule_get_active(void)
{
    if (NULL != actived_app)
        return rt_list_entry(actived_app, gui_runing_app_t, node);
    else
        return NULL;
}

/**
 * find app in all running app list
 * \n
 *
 * @param id - launche app id
 * @return running app handler
 * \n
 * @see
 */
gui_runing_app_t *app_schedule_is_app_running(const char *id)
{
    rt_list_t *ptr;
    gui_runing_app_t *p_app = NULL;


    rt_list_for_each(ptr, &running_app_list)
    {
        p_app = rt_list_entry(ptr, gui_runing_app_t, node);
        if (0 == strcmp(p_app->id, id))
            return p_app;
    }

    return NULL;
}

uint32_t app_schedule_get_running_apps(void)
{
    return rt_list_len(&running_app_list);
}

/**
 * Get current app handler
 * return schedule_app if call from page's msg handler
 * return actived_app if call from service_cbk/input_dev_cbk
 * \n
 *
 * @return
 * \n
 * @see
 */
gui_runing_app_t *app_schedule_get_this(void)
{
    if (NULL != schedule_app)
        return rt_list_entry(schedule_app, gui_runing_app_t, node);
    else if (NULL != actived_app)
    {
        //check current scr
        gui_runing_app_t *p_app = rt_list_entry(actived_app, gui_runing_app_t, node);

        if (!rt_list_isempty(&p_app->page_list))
        {
            subpage_node_t *p_page = rt_list_first_entry(&p_app->page_list, subpage_node_t, node);

            if ((port_app_sche_get_act_scr() == p_page->scr)
#ifdef TRANS_ANIMATION
                    || (trans_anim_playing && act_scr_before_app_trans_anim == p_page->scr)
#endif /* TRANS_ANIMATION */
               )
                return p_app;
        }
    }

    return NULL;
}



subpage_node_t *app_schedule_get_this_page(void)
{
    if (NULL != schedule_page)
        return rt_list_entry(schedule_page, subpage_node_t, node);
    else
    {
        gui_runing_app_t *this_app = app_schedule_get_this();

        if (NULL != this_app)
        {
            if (!rt_list_isempty(&this_app->page_list))
            {
                return rt_list_first_entry(&this_app->page_list, subpage_node_t, node);
            }
        }
    }

    return NULL;
}

subpage_node_t *app_schedule_get_page_in_app(gui_runing_app_t *app, const char *page_id)
{
    if (app)
    {
        rt_list_t *ptr;
        rt_list_for_each(ptr, &app->page_list)
        {
            subpage_node_t *p_page = rt_list_entry(ptr, subpage_node_t, node);
            if (0 == strcmp(p_page->name, page_id))
            {
                return p_page;
            }
        }
    }
    return NULL;
}


/************************************

  Private functions

************************************/
static uint32_t tick_elaps(uint32_t prev_tick)
{
    uint32_t act_time = (uint32_t) rt_tick_get();

    /*If there is no overflow in sys_time simple subtract*/
    if (act_time >= prev_tick)
    {
        prev_tick = act_time - prev_tick;
    }
    else
    {
        prev_tick = UINT32_MAX - prev_tick + 1;
        prev_tick += act_time;
    }

    return prev_tick;
}

static void lock(int8_t v)
{
    rt_err_t err;
    if (v > 0)
    {
        err = rt_sem_take(mutex_lock, rt_tick_from_millisecond(10));//10 ms delay for waitting 'gui_app_cleanup_now' finished in gui_app_test
    }
    else
    {
        err = rt_sem_release(mutex_lock);
    }

    if ((RT_EOK != err) || (mutex_lock->value > 1))
    {
        app_sche_assert("Recursive %d", mutex_lock->value);
    }
}
static gui_runing_app_t *get_runing_app_handler(const char *id)
{
    rt_list_t *ptr;

    rt_list_for_each(ptr, &running_app_list)
    {
        gui_runing_app_t *run_app = rt_list_entry(ptr, gui_runing_app_t, node);
        if (0 == strcmp(run_app->id, id))
        {
            return run_app;
        }
    }

    return NULL;
}

static gui_runing_app_t *get_suspend_app_handler(const char *id)
{
    rt_list_t *ptr;

    rt_list_for_each(ptr, &suspend_app_list)
    {
        gui_runing_app_t *run_app = rt_list_entry(ptr, gui_runing_app_t, node);
        if ((0 == strcmp(run_app->id, id)) && (run_app->target_state < app_st_destoryed))
        {
            return run_app;
        }
    }

    return NULL;
}

static gui_runing_app_t *get_suspend_or_destoryed_app_handler(const char *id)
{
    rt_list_t *ptr;

    rt_list_for_each(ptr, &suspend_app_list)
    {
        gui_runing_app_t *run_app = rt_list_entry(ptr, gui_runing_app_t, node);
        if (0 == strcmp(run_app->id, id))
        {
            return run_app;
        }
    }

    return NULL;
}

static bool is_running_app_handler(gui_runing_app_t *handler)
{
    rt_list_t *ptr;

    rt_list_for_each(ptr, &running_app_list)
    {
        if (rt_list_entry(ptr, gui_runing_app_t, node) == handler)
            return true;
    }

    return false;

}



static bool is_suspend_or_destoryed_app_handler(gui_runing_app_t *handler)
{
    rt_list_t *ptr;

    rt_list_for_each(ptr, &suspend_app_list)
    {
        if (rt_list_entry(ptr, gui_runing_app_t, node) == handler)
            return true;
    }

    return false;

}

static bool is_last_page_present(void)
{
    if (1 == rt_list_len(&running_app_list))
    {
        gui_runing_app_t *p_app = rt_list_entry(running_app_list.next, gui_runing_app_t, node);

        if (1 == rt_list_len(&p_app->page_list)) return true;
    }

    return false;
}





static char *app_get_state_name(app_state_enum state)
{
    switch (state)
    {
    case app_st_launched:
        return "LAUNCHED";
    case app_st_running:
        return "RUNNING";
    case app_st_suspended:
        return "SUSPEND";
    case app_st_destoryed:
        return "DESTORYED";
    default:
        return "UNKOWN STATE";
    }
}

static void app_set_target_state(gui_runing_app_t *p_app, app_state_enum target_state)
{
    if (((app_st_running <= p_app->state) && (app_st_running == target_state))
            || (target_state > p_app->state))
    {
        if (p_app->target_state != target_state) //Filter out duplicated setting
        {
            app_sche_d("app[%s] tgt_state [%s] -> [%s]",
                       p_app->id,
                       app_get_state_name(p_app->state),
                       app_get_state_name(target_state));
            p_app->target_state = target_state;
        }
    }
    else
    {
        ; //invalid target state, ignore
    }
}

/**
 * tell what to do by current and target state
 * \n
 *
 * @param cur_state
 * @param target_state
 * @return do_msg
 * \n
 * @see
 */
static app_exe_enum app_state_machine(app_state_enum cur_state, app_state_enum target_state)
{
    if (((app_st_running == target_state) && (app_st_suspended == cur_state))
            || ((app_st_running == target_state) && (app_st_destoryed == cur_state)))
    {
        return APP_EXEC_RESTART;
    }
    else if (target_state > cur_state)
    {
        switch (cur_state)
        {
        case app_st_launched:
            if (app_st_suspended == target_state) return APP_EXEC_SUSPEND;
            if (app_st_destoryed == target_state) return APP_EXEC_DESTORY;
            return APP_EXEC_START;

        case app_st_running:
            return APP_EXEC_SUSPEND;

        case app_st_suspended:
            return APP_EXEC_DESTORY;


        default:
            return -1;
        }
    }

    return -2;
}


static char *page_get_state_name(page_state_enum state)
{
    switch (state)
    {
    case page_st_created:
        return "CREATED";
    case page_st_started:
        return "STARTED";
    case page_st_resumed:
        return "RESUMED";
    case page_st_paused:
        return "PAUSED";
    case page_st_stoped:
        return "STOPED";
    default:
        return "UNKOWN STATE";

    }
}

static void page_set_target_state(subpage_node_t *page, page_state_enum target_state)
{
    if (((page_st_paused == page->state) && (page_st_resumed == target_state))
            || (target_state > page->state))
    {
        if (page->target_state != target_state) //Filter out duplicated setting
        {
            app_sche_d("page[%s][%s] tgt_state [%s] -> [%s]",
                       page->parent->id, page->name,
                       page_get_state_name(page->state),
                       page_get_state_name(target_state));
            page->target_state = target_state;
        }
    }
    else
    {
        ; //invalid target state, ignore
    }
}

/**
 * tell what to do by current and target state
 * \n
 *
 * @param cur_state
 * @param target_state
 * @return do_msg
 * \n
 * @see
 */
static gui_app_msg_type_t page_state_machine(page_state_enum cur_state, page_state_enum target_state)
{
    if ((page_st_resumed == target_state) && (page_st_paused == cur_state))
    {
        return GUI_APP_MSG_ONRESUME;
    }

    if (target_state > cur_state)
    {
        switch (cur_state)
        {
        case page_st_created:
            return GUI_APP_MSG_ONSTART;
        case page_st_started:
            return GUI_APP_MSG_ONRESUME;
        case page_st_resumed:
            return GUI_APP_MSG_ONPAUSE;
        case page_st_paused:
            return GUI_APP_MSG_ONSTOP;

        default:
            return -1;
        }
    }

    return -2;
}



static void app_set_all_page_state(gui_runing_app_t *p_app, page_state_enum target_state)
{
    rt_list_t *ptr;

    if (NULL == p_app)
    {
        return;
    }

    rt_list_for_each(ptr, &p_app->page_list)
    {
        subpage_node_t *p_page = rt_list_entry(ptr, subpage_node_t, node);
        page_set_target_state(p_page, target_state);
    }

}

static void app_set_all_app_all_page_state(page_state_enum target_state)
{
    rt_list_t *ptr;

    rt_list_for_each(ptr, &running_app_list)
    {
        gui_runing_app_t *ptr_app = rt_list_entry(ptr, gui_runing_app_t, node);

        app_set_all_page_state(ptr_app, target_state);
    }

}

/**
* kill all running app
* \n
*
* \n
* @see
*/
static void app_stop_running(void)
{

    if (dummy_scr)
    {
        app_sche_d("Stop running app");
        port_app_sche_reset_indev(port_app_sche_get_act_scr());
        port_app_sche_load_scr(dummy_scr);
        actived_app = NULL;

        rt_list_t *pn_app;

        rt_list_for_each(pn_app, &running_app_list)
        {
            app_destory(pn_app);
        }
    }
    else
    {
        app_sche_assert("dummy_scr is invalid!");
    }
}

/**
* kill all app
* \n
*
* \n
* @see
*/
static void app_stop_all(void)
{

    if (dummy_scr)
    {
        app_sche_d("Stop all app");
        port_app_sche_reset_indev(port_app_sche_get_act_scr());
        port_app_sche_load_scr(dummy_scr);
        actived_app = NULL;

        rt_list_t *pn_app;

        rt_list_for_each(pn_app, &running_app_list)
        {
            app_destory(pn_app);
        }
        rt_list_for_each(pn_app, &suspend_app_list)
        {
            app_destory(pn_app);
        }
    }
    else
    {
        app_sche_assert("dummy_scr is invalid!");
    }
}

/**
* kill all background apps
* \n
*
* \n
* @see
*/
static void app_stop_all_backgrounds(void)
{
    rt_list_t *tmp_actived = actived_app;
    rt_list_t *prev_node;

    app_sche_d("app_stop_all_backgrounds");

    //Remove active app from running list.
    if (tmp_actived)
    {
        prev_node = tmp_actived->prev;
        rt_list_remove(tmp_actived);
    }

    //Destory all othe apps
    {
        rt_list_t *pn_app;

        rt_list_for_each(pn_app, &running_app_list)
        {
            app_destory(pn_app);
        }
        rt_list_for_each(pn_app, &suspend_app_list)
        {
            app_destory(pn_app);
        }
    }

    /* Add active app back to list*/
    if (tmp_actived)
    {
        rt_list_insert_after(prev_node, tmp_actived);
    }
}

/*
    Pause/Stop all other app,
    Resume first page of app if it is existed.
    and bring it to front.
*/
static void app_resume(rt_list_t *app_node)
{
    if (NULL == app_node) return;

    gui_runing_app_t *run_app = rt_list_entry(app_node, gui_runing_app_t, node);
    rt_list_t *prev_node = app_node->prev;

    /*1. Remove app from running_app_list*/
    rt_list_remove(app_node);

    /*2. Pause or stop all app in running_app_list*/
    if (0 == strcmp(p_main_app_id, run_app->id))
    {
        /*
         Stop other pages when return to main to reduce memory fragement
        */

        app_sche_d("Stop all app, and return to main app");
        app_destory_list(&running_app_list);
        app_destory_list(&suspend_app_list);
    }
    else
    {
        app_sche_d("Pause all app, and resume to %s app", run_app->id);
        app_set_all_app_all_page_state(page_st_paused);
    }

    /* 3. Resume first page*/
    run_app = rt_list_entry(app_node, gui_runing_app_t, node);
    if (rt_list_isempty(&run_app->page_list))
    {
        //App is just launched,
        //the message of creating root page must be in msg_queue
        app_sche_i("app[%s] has none subpage exist!", run_app->id);
    }
    else
    {
        subpage_node_t *p_last_page = rt_list_tail_entry(&run_app->page_list, subpage_node_t, node);

        if (page_st_resumed == p_last_page->target_state)
        {
            ;//A page was created at tail of list
        }
        else
        {
            subpage_node_t *p_first_page = rt_list_first_entry(&run_app->page_list, subpage_node_t, node);
            page_set_target_state(p_first_page, page_st_resumed);
        }
    }

    /* 4.  Add app back to running_app_list*/
    rt_list_insert_after(prev_node, app_node);
    app_set_target_state(run_app, app_st_running);
}

static void app_restart(rt_list_t *app_node)
{
    if (NULL == app_node) return;
    gui_runing_app_t *p_app = rt_list_entry(app_node, gui_runing_app_t, node);

    app_sche_d("app_restart [%s]", p_app->id);

    //Move app node to the head of runing_app_list
    rt_list_remove(&p_app->node);
    rt_list_insert_after(&running_app_list, &p_app->node);
    app_set_target_state(p_app, app_st_running);
}

static void app_suspend(rt_list_t *app_node)
{
    if (NULL == app_node) return;
    gui_runing_app_t *p_app = rt_list_entry(app_node, gui_runing_app_t, node);

    app_sche_d("app_suspend [%s]", p_app->id);

    if (rt_list_isempty(&p_app->page_list))
    {
    }
    else
    {
        //Wait 'schedule_subpage' to move app node to 'suspend_app_list'
        app_set_all_page_state(p_app, page_st_stoped);
    }
    app_set_target_state(p_app, app_st_suspended);
}


/**
 * @brief Set all subpages to stopped state, and app to destoryed state
 * @param app_node -
 */
static void app_destory(rt_list_t *app_node)
{
    if (NULL == app_node) return;
    gui_runing_app_t *p_app = rt_list_entry(app_node, gui_runing_app_t, node);

    app_sche_d("app_destory [%s]", p_app->id);

    if (rt_list_isempty(&p_app->page_list))
    {
    }
    else
    {
        //Wait 'schedule_subpage' to move app node to 'suspend_app_list' and destory by 'do_destory'
        app_set_all_page_state(p_app, page_st_stoped);
    }
    app_set_target_state(p_app, app_st_destoryed);
}

static void app_destory_list(rt_list_t *list)
{
    if (list)
    {
        rt_list_t *ptr, *ptr_next;
        rt_list_for_each_safe(ptr, ptr_next, list)
        {
            app_destory(ptr);
        }
    }
}


/*
    Restart last suspended app, restart main app if no app was suspended
*/
static void app_restart_last_suspend(void)
{
    app_sche_i("app_restart_last_suspend");

    //All running app should be destoryed, before restart last suspend app
    app_destory_list(&running_app_list);

    rt_list_t *ptr, *ptr_next;
    rt_list_for_each_safe(ptr, ptr_next, &suspend_app_list)
    {
        gui_runing_app_t *p_app = rt_list_entry(ptr, gui_runing_app_t, node);

        if (app_st_suspended == p_app->target_state)
        {
            app_restart(ptr);
            return;
        }
    }


    app_sche_i("No app was suspended, return to Main");
    app_run_by_id(task_msg_mbx, p_main_app_id);
}


static void app_subpage_do(gui_runing_app_t *p_app, subpage_node_t *subpage, gui_app_msg_type_t msg_id)
{
    if (NULL == p_app)
    {
        return;
    }

    if (NULL == subpage)
    {
        subpage = rt_list_first_entry(&p_app->page_list, subpage_node_t, node);
    }

    if (subpage->msg_handler)
    {
        char *state_str;
        switch (msg_id)
        {
        case GUI_APP_MSG_ONSTART:
            state_str = "ONSTART";
            break;
        case GUI_APP_MSG_ONRESUME:
            state_str = "ONRESUME";
            break;
        case GUI_APP_MSG_ONPAUSE:
            state_str = "ONPAUSE";
            break;
        case GUI_APP_MSG_ONSTOP:
            state_str = "ONSTOP";
            break;
        default:
            state_str = "UNKNOWN";
            break;
        }

        app_sche_i("page[%s][%s] do %s, %x", p_app->id, subpage->name, state_str, subpage);
        schedule_app  = &p_app->node;
        schedule_page = &subpage->node;
        uint32_t tick_cnt, tick_start = rt_tick_get();

        if (sche_subpage_func)
            sche_subpage_func(subpage->msg_handler, msg_id, p_app->id, subpage->name);
        else
            subpage->msg_handler(msg_id, p_app->id);

        tick_cnt = tick_elaps(tick_start);
        subpage->tick_cnt += tick_cnt;
        p_app->tick_cnt += tick_cnt;

        schedule_page = NULL;
        schedule_app = NULL;

        if (app_subgpage_perf_tick)
        {
            app_sche_i("page[%s][%s] do %s done.cost %d ticks.", p_app->id, subpage->name, state_str, tick_cnt);
        }
    }
}

static int app_do(gui_runing_app_t *p_app, app_exe_enum op)
{
    char *op_str;
    switch (op)
    {
    case APP_EXEC_LOAD:
        op_str = "LOAD";
        break;
    case APP_EXEC_START:
        op_str = "START";
        break;
    case APP_EXEC_RESTART:
        op_str = "RESTART";
        break;
    case APP_EXEC_SUSPEND:
        op_str = "SUSPEND";
        break;
    case APP_EXEC_DESTORY:
        op_str = "DESTROY";
        break;
    default:
        op_str = "UNKNOWN";
        break;
    }

    app_sche_i("app[%s] do %s, %x", p_app->id, op_str, p_app);


    int ret_v = 0;
    rt_tick_t tick_cnt, tick_start = rt_tick_get();
    switch (op)
    {
    case APP_EXEC_LOAD:
    {
        app_entity_info app_info;

        sche_app_ld_func(p_app->id, &app_info);
        if (app_info.entry_func)
        {
            p_app->app_data = (void *)app_info.user_data;
            p_app->entry_f = (gui_app_entry_func_ptr_t) app_info.entry_func;
            p_app->state = app_st_launched;

            rt_list_insert_after(&running_app_list, &p_app->node);
        }
        else
        {
            app_sche_i("can't find app %s entry function!", p_app->id);
            ret_v = -1;
        }
    }
    break;

    case APP_EXEC_START:
    case APP_EXEC_RESTART:
    {
        printf_intent(&p_app->param);
        ret_v = p_app->entry_f(&p_app->param);
        p_app->state = app_st_running;
    }
    break;

    case APP_EXEC_SUSPEND:
    {
        //Move app node to the head of suspend_app_list
        rt_list_remove(&p_app->node);
        rt_list_insert_after(&suspend_app_list, &p_app->node);
        p_app->state = app_st_suspended;
        p_app->flag |= APP_FLAG_GOBACK_ANIM;

        if (p_app->target_state < app_st_suspended)
        {
            app_sche_assert("Invalid st %d", p_app->target_state);
        }
    }
    break;

    case APP_EXEC_DESTORY:
    {
        app_entity_info app_info;
        rt_list_remove(&p_app->node);

        app_info.entry_func = (uint32_t) p_app->entry_f;
        app_info.user_data    = (uint32_t) p_app->app_data;
        sche_app_dstry_func((const char *)p_app->id, (const app_entity_info *)&app_info);
    }
    break;
    default:
        app_sche_assert("Invalid op %d", op);
        break;
    }
    tick_cnt = tick_elaps(tick_start);
    p_app->tick_cnt += tick_cnt;

    if (ret_v != 0)
    {
        app_sche_e("app[%s] do %s failed! err_v = %d", p_app->id, op_str, ret_v);
    }

    if (app_subgpage_perf_tick)
    {
        app_sche_i("app[%s] do %s done.cost %d ticks", p_app->id, op_str, tick_cnt);
    }

    return ret_v;
}

static int app_launch_new(const char *id, intent_t intent)
{
    int ret_v = -1;
    gui_runing_app_t *run_app;

    run_app = (gui_runing_app_t *) app_sche_malloc(sizeof(gui_runing_app_t));
    if (run_app)
    {
        memset(run_app, 0, sizeof(gui_runing_app_t));

        rt_list_init(&run_app->page_list);
        strcpy(run_app->id, id);
        memcpy(&(run_app->param), intent, sizeof(run_app->param));

        ret_v = app_do(run_app, APP_EXEC_LOAD);

        if (ret_v != 0)
        {
            app_sche_free(run_app);
        }
        else
        {
            app_set_target_state(run_app, app_st_running);
        }
    }
    else
    {
        app_sche_i("malloc app %s node failed!", id);
    }

    return ret_v;
}



/**
 * Exit current or paused app:
 *   1. Stop all pages of specified app
 *   2. Resume previous app's first page if it's present while specified app is actived one.
 *
 * \n
 *
 * @param app_node
 * \n
 * @see
 */
static void app_exit(rt_list_t *app_node)
{
    gui_runing_app_t *p_app;
    if (NULL == app_node) return;

    /*set all pages in app_node to stoped*/
    p_app = rt_list_entry(app_node, gui_runing_app_t, node);
    app_sche_d("app_exit [%s]", p_app->id);

    app_destory(app_node);

    if (app_node == actived_app)
    {
        if (rt_list_len(&running_app_list) > 1)
        {
            if (app_node->next != &running_app_list)
            {
                /*Resume previous app*/
                p_app = rt_list_entry(app_node->next, gui_runing_app_t, node);
            }
            else
            {
                /*Resume latest app*/
                p_app = rt_list_entry(running_app_list.next, gui_runing_app_t, node);
            }

            if (!rt_list_isempty(&p_app->page_list))
            {
                subpage_node_t *p_page;
                /*Resume app's last page*/
                p_page = rt_list_first_entry(&p_app->page_list, subpage_node_t, node);
                page_set_target_state(p_page, page_st_resumed);
            }
            else
            {
                app_sche_i("Can't resume app[%s], none subpage exist!", p_app->id);
            }
        }
        else
        {
            app_sche_e("Exit last app.");
            //app_stop_all();
            app_restart_last_suspend();
        }
    }


}

static rt_err_t app_goback(rt_list_t *app_node)
{
    if (NULL == app_node) return RT_EEMPTY;

    //get current page
    gui_runing_app_t *run_app = rt_list_entry(app_node, gui_runing_app_t, node);

    app_sche_d("app_goback app[%s]", run_app->id);

    if (1 == rt_list_len(&run_app->page_list)) //go back last page, close app
    {
        app_exit(app_node);
        return RT_EEMPTY;
    }
    else //goback to previous page
    {
        subpage_node_t *p_cur_page = rt_list_first_entry(&run_app->page_list, subpage_node_t, node);
        subpage_node_t *p_old_page = rt_list_entry(p_cur_page->node.next, subpage_node_t, node);
        app_sche_d("app[%s] page[%s]->page[%s]", run_app->id,
                   p_cur_page->name, p_old_page->name);

        page_set_target_state(p_cur_page, page_st_stoped);
        page_set_target_state(p_old_page, page_st_resumed);
    }

    return RT_EOK;
}

static rt_err_t app_create_page(rt_list_t *app_node, const char *page_id, gui_page_msg_cb_t page_handler, void *usr_data)
{
    if (NULL == app_node) return RT_EEMPTY;

    //get current page
    gui_runing_app_t *run_app = rt_list_entry(app_node, gui_runing_app_t, node);


    //stack current page to a history list
    subpage_node_t *p_new_page = (subpage_node_t *) app_sche_malloc(sizeof(struct _subpage_node));

    if (NULL == p_new_page)
    {
        app_sche_d("app[%s] create page[%s] NO MEMORY!", run_app->id, page_id);
        return RT_ENOMEM;
    }

    app_sche_d("app[%s] create page[%s] %x", run_app->id, page_id, p_new_page);
    memset(p_new_page, 0, sizeof(struct _subpage_node));

    //load new page data
    p_new_page->parent = run_app;
    if (strlen(page_id) < sizeof(p_new_page->name))
        strcpy(p_new_page->name, page_id);
    else
        strncpy(p_new_page->name, page_id, sizeof(p_new_page->name) - 1);

    p_new_page->msg_handler = page_handler;
    p_new_page->user_data = usr_data;
    p_new_page->state = page_st_created;
#ifdef TRANS_ANIMATION
    app_trans_anim_init_cfg(&p_new_page->a_group.a_enter, GUI_APP_TRANS_ANIM_ENTER_DEFAULT);
    app_trans_anim_init_cfg(&p_new_page->a_group.a_exit, GUI_APP_TRANS_ANIM_EXIT_DEFAULT);
    p_new_page->a_group.prio_up = 0;
    p_new_page->a_group.prio_down = 0;
#endif /* TRANS_ANIMATION */

    /*Pause all other pages of this app*/
    app_set_all_page_state(run_app, page_st_paused);

    //Resume this new page
    page_set_target_state(p_new_page, page_st_resumed);

    /* Add this new page to the end of app's page list,
        and bring it the head of page_list by 'schedule_subpage'
        after current page are paused.

        NOT to insert to page_list's next
    */
    rt_list_insert_before(&run_app->page_list, &p_new_page->node);

    //Bring app to foreground
    app_resume(app_node);

    return RT_EOK;
}

#define FIRST_SUBPAGE(p_app)  (subpage_node_t *) rt_list_first_entry(&(p_app)->page_list, subpage_node_t, node)
static rt_err_t app_rm_page(rt_list_t *app_node, const char *page_id)
{
    if (NULL == app_node) return RT_EEMPTY;

    //get current page
    gui_runing_app_t *run_app = rt_list_entry(app_node, gui_runing_app_t, node);
    subpage_node_t *p_page = app_schedule_get_page_in_app(run_app, page_id);

    if (p_page)
    {
        //find it.
        if (page_st_resumed == p_page->state)
        {
            app_sche_d("app_rm_page rm avtive page");
            return app_goback(app_node);
        }
        else
        {
            app_sche_d("app_rm_page app[%s],page[%s]", run_app->id, page_id);
            page_set_target_state(p_page, page_st_stoped);
            return RT_EOK;
        }
    }
    else
    {
        app_sche_e("app_rm_page can't find page[%s] in app[%s]", page_id, run_app->id);
        return RT_ERROR;
    }

}

static rt_err_t app_goback_to_page(rt_list_t *app_node, const char *page_id)
{
    if (NULL == app_node) return RT_EEMPTY;

    //get current page
    gui_runing_app_t *p_app = rt_list_entry(app_node, gui_runing_app_t, node);
    subpage_node_t *p_to_page = app_schedule_get_page_in_app(p_app, page_id);

    if (p_to_page)
    {
        //find it.
        app_sche_d("[%s] app_goback_to_page page[%s]", p_app->id, page_id);

        rt_list_t *ptr;

        rt_list_for_each(ptr, &p_app->page_list)
        {
            if (ptr != &p_to_page->node)
            {
                subpage_node_t *p_stop_page = rt_list_entry(ptr, subpage_node_t, node);
                page_set_target_state(p_stop_page, page_st_stoped);
            }
            else
            {
                page_set_target_state(p_to_page, page_st_resumed);
                break;
            }
        }

        return RT_EOK;
    }
    else
    {
        app_sche_e("app_goback_to_page can't find page[%s] in app[%s]", page_id, p_app->id);
        return RT_ERROR;
    }

}


static rt_err_t app_refr_page(rt_list_t *app_node, const char *page_id)
{
    if (NULL == app_node) return RT_EEMPTY;

    //get current page
    gui_runing_app_t *p_app = rt_list_entry(app_node, gui_runing_app_t, node);
    subpage_node_t *p_to_page = app_schedule_get_page_in_app(p_app, page_id);

    if (p_to_page)
    {
        //find it.
        app_sche_d("[%s] refresh subpage[%s]", p_app->id, page_id);

        if (page_st_resumed == p_to_page->state)
        {
            app_subpage_do(p_app, p_to_page, GUI_APP_MSG_ONPAUSE);
            app_subpage_do(p_app, p_to_page, GUI_APP_MSG_ONRESUME);
        }

        return RT_EOK;
    }
    else
    {
        app_sche_e("app_refr_subpage can't find page[%s] in app[%s]", page_id, p_app->id);
        return RT_ERROR;
    }

}


#ifdef TRANS_ANIMATION


static void app_trans_anim_finish_callback(TransResult_T res)
{
    if (TRANS_RES_ABORTED == res)
    {
        app_sche_i("Trans-anim aborted, revert state...");
        if (trans_anim_pg_exit)
        {
            page_set_target_state(trans_anim_pg_exit,  page_st_resumed);
            page_set_target_state(trans_anim_pg_enter, page_st_paused);

            if (trans_anim_pg_exit->parent != trans_anim_pg_enter->parent)
            {
                app_resume(&trans_anim_pg_exit->parent->node);
            }
        }
        else
        {
            page_set_target_state(trans_anim_pg_enter, page_st_resumed);
        }
    }
    else
    {
        app_sche_i("Trans-anim finish, res=%d", res);
    }

    if (port_app_sche_get_act_scr() != act_scr_before_app_trans_anim)
    {
        app_sche_assert("Error active screen");
    }

    act_scr_before_app_trans_anim = NULL;
    trans_anim_pg_exit = NULL;
    trans_anim_pg_enter = NULL;


    /*
        The app's screen must be loaded just before this cbk,
        and this cbk may execute in animation task or app_schedule task(if animation be aborted),
        lv_refr_task may be inserted before app_schedule_task,
        then we may glance the trans_anim_pg_enter->scr.

        So we finish all page state change since last trans animation start here.
    */
    while (app_scheduler_new())
    {
    }

    /*
        Transform animation finished, resume app schedule task
    */
    trans_anim_playing = false;
    app_sche_d("--Resume app_schedule_task--");
}
#endif

static bool is_prev_page_wait_to_stop(gui_runing_app_t *p_app, subpage_node_t *subpage)
{
    subpage_node_t *prev_subpage;

    if (!p_app || !subpage) return false;

    if (p_app->page_list.next == &subpage->node) //subpage is first one
    {
        return false;
    }

    prev_subpage = rt_list_entry(subpage->node.prev, subpage_node_t, node);


    return (page_st_stoped == prev_subpage->target_state) && (page_st_stoped != prev_subpage->state);

}

static uint32_t app_run(rt_mailbox_t msg_mbx, const _intent *i, uint32_t tick)
{
    rt_err_t err;

    //Restore msg to allocated memmory
    gui_app_msg_t *p_msg = (gui_app_msg_t *) app_sche_malloc(sizeof(gui_app_msg_t));
    if (NULL == p_msg)
    {
        app_sche_assert("Malloc msg fail.");
    }
    else
    {
        rt_memcpy(&p_msg->content.intnt, i, sizeof(_intent));
    }

    p_msg->tick = tick;
    p_msg->msg_id = GUI_APP_MSG_RUN_APP_I;

    //Send to mailbox
    err = rt_mb_send(msg_mbx, (rt_uint32_t) p_msg);

    if (err != RT_EOK)
    {
        app_sche_assert("send to gui_app_mbx err:%d", err);
    }
    else
    {
        app_sche_d("Inner send[GUI_APP_MSG_RUN_APP_I] [%s] to gui_app_mbx[%d] tick:%d.", p_msg->content.cmd, msg_mbx->entry, p_msg->tick);
        printf_intent(&p_msg->content.intnt);
    }

    return 0;
}

static uint32_t app_run_by_id(rt_mailbox_t msg_mbx, const char *id)
{
    rt_err_t err;

    intent_t i = intent_init(id);

    if (NULL == i)
    {
        err = RT_ERROR;
    }
    else
    {
        err = app_run(msg_mbx, i, rt_tick_get());
        intent_deinit(i);
    }

    return err;
}

/**
 * @brief Execute apps to target state
 * @return > 0 if app state changed
 */
static uint32_t schedule_apps(void)
{
    rt_list_t *pn_app, *temp;

    rt_list_for_each_safe(pn_app, temp, &running_app_list)
    {
        gui_runing_app_t *cur_app;
        cur_app = rt_list_entry(pn_app, gui_runing_app_t, node);

        schedule_app = pn_app;
        if (cur_app->state != cur_app->target_state)
        {
            app_exe_enum ops = app_state_machine(cur_app->state, cur_app->target_state);

            switch (ops)
            {
            case APP_EXEC_START:
            {
                app_do(cur_app, APP_EXEC_START);
            }
            break;

            case APP_EXEC_RESTART:
            {
                app_do(cur_app, APP_EXEC_RESTART);
            }
            break;

            case APP_EXEC_SUSPEND:
            {
                /* App's page list is empty, move app from running list to suspend_app_list*/
                if (rt_list_isempty(&cur_app->page_list))
                {
                    app_do(cur_app, APP_EXEC_SUSPEND);
                }
                else
                {
                    ;//Suspend after all page stop
                }
            }
            break;

            case APP_EXEC_DESTORY:
            {
                if (rt_list_isempty(&cur_app->page_list))
                {
                    ;//app_destory(&cur_app->node);
                }
                else
                {
                    ;//Destory after all page stop
                }
            }
            break;



            default:
                app_sche_assert("Invalid ops %d", ops);
                break;
            }

        }

        schedule_app = NULL;
    }


    rt_list_for_each_safe(pn_app, temp, &suspend_app_list)
    {
        gui_runing_app_t *cur_app;
        cur_app = rt_list_entry(pn_app, gui_runing_app_t, node);

        if ((app_st_suspended != cur_app->state) || (!rt_list_isempty(&cur_app->page_list)))
        {
            app_sche_assert("Suspend app[%s] state:%d subpages:%d", cur_app->id, cur_app->state, rt_list_len(&cur_app->page_list));
        }
    }

    /*exit the oldest app */
    {
        uint16_t runing_app_cnt = 0;

        rt_list_for_each_safe(pn_app, temp, &running_app_list)
        {
            gui_runing_app_t *cur_app;
            cur_app = rt_list_entry(pn_app, gui_runing_app_t, node);

            if ((cur_app->state < app_st_suspended) && (cur_app->target_state < app_st_suspended))
            {
                //app_sche_d("running apps idx[%d]:[%s].",runing_app_cnt, cur_app->id);
                if (++runing_app_cnt > max_running_apps)
                {
                    app_sche_d("running apps > MAX_RUNNING_APPS.");
                    if (en_suspend_app)
                        app_suspend(pn_app);
                    else
                        app_destory(pn_app);
                }
            }
            else
            {
                //Examine suspended/destroyed app: all subpages should target to stopped
                rt_list_t *ptr;

                rt_list_for_each(ptr, &cur_app->page_list)
                {
                    subpage_node_t *p_page = rt_list_entry(ptr, subpage_node_t, node);

                    if (p_page->target_state != page_st_stoped)
                    {
                        app_sche_assert("Suspend/Destroyed app[%s] page[%s] not been stopped.",
                                        cur_app->id, p_page->name);
                    }
                }
            }
        }
    }

    return 0; //Some app would not reach target_status, if their subpage msgs are not executed.
}

/**
 *
 * \n
 *
 * @return > 0 if page state changed
 * \n
 * @see
 */
static uint32_t schedule_subpage(void)
{
    rt_list_t *pn_app;
    rt_list_t *pn_page;

    rt_list_t rm_page_list;

    uint32_t page_st_changes = 0;

    gui_runing_app_t *cur_app;
    subpage_node_t *cur_page;


#ifdef TRANS_ANIMATION
    bool pg_enter_was_paused = false;
    bool pg_exit_being_stop  = false;
#endif /* TRANS_ANIMATION */

    if (rt_list_isempty(&running_app_list)) goto SCHEDULE_SUBPAGE_END;

    rt_list_init(&rm_page_list);

    rt_list_for_each(pn_app, &running_app_list)
    {
        rt_list_t *active_page = NULL;
        cur_app = rt_list_entry(pn_app, gui_runing_app_t, node);

        //in case page was not created
        if (rt_list_isempty(&cur_app->page_list)) continue;

        rt_list_for_each(pn_page, &cur_app->page_list)
        {
            cur_page = rt_list_entry(pn_page, subpage_node_t, node);

            if (cur_page->state != cur_page->target_state)
            {
                page_st_changes++;


                gui_app_msg_type_t msg = page_state_machine(cur_page->state, cur_page->target_state);

                switch (msg)
                {
                case GUI_APP_MSG_ONSTART:
                {
                    screen_t old_scr;

                    /*create new scr and load it,and app can create ui in function onstart*/
                    old_scr = port_app_sche_get_act_scr();
                    cur_page->scr = port_app_sche_create_scr();
                    port_app_sche_load_scr(cur_page->scr);
                    app_subpage_do(cur_app, cur_page, GUI_APP_MSG_ONSTART);
                    /*
                        revert old scr, cause we are not want to see new scr
                        util it's onresume
                    */
                    port_app_sche_load_scr(old_scr);

                    cur_page->state = page_st_started;
                }
                break;

                case GUI_APP_MSG_ONRESUME:
                {
                    //do resume
                    actived_app = &cur_app->node;
                    port_app_sche_reset_indev(port_app_sche_get_act_scr());
                    port_app_sche_load_scr(cur_page->scr);
                    app_subpage_do(cur_app, cur_page, GUI_APP_MSG_ONRESUME);

                    active_page = pn_page;

#ifdef TRANS_ANIMATION
                    pg_enter_was_paused = (page_st_paused == cur_page->state);
                    trans_anim_pg_enter = cur_page;
#endif

                    cur_page->state = page_st_resumed;
                }
                break;
                case GUI_APP_MSG_ONPAUSE:
                {
                    //do pause
                    app_subpage_do(cur_app, cur_page, GUI_APP_MSG_ONPAUSE);

#ifdef TRANS_ANIMATION
                    if (port_app_sche_get_act_scr() == cur_page->scr)
                    {
                        pg_exit_being_stop = (page_st_stoped == cur_page->target_state);
                        //record exit anim and play with enter anim
                        trans_anim_pg_exit = cur_page;
                    }
                    else
                    {
                        //Background exited page
                    }
#endif
                    cur_page->state = page_st_paused;

                }
                break;
                case GUI_APP_MSG_ONSTOP:
                {
                    if (is_prev_page_wait_to_stop(cur_app, cur_page)) /*Let prev page do stop first, Issue 1190: First open last stop */
                    {
                        page_st_changes--;/*Skip this msg and will be do stop after prev page stop*/
                    }
                    else if (port_app_sche_get_act_scr() != cur_page->scr) /*Must resume another page before delete the active one*/
                    {
                        rt_list_t *prev_page;
                        //do stop
                        app_subpage_do(cur_app, cur_page, GUI_APP_MSG_ONSTOP);


                        port_app_sche_del_scr(cur_page->scr);
                        cur_page->state = page_st_stoped;

#ifdef TRANS_ANIMATION
                        //page node RAM will be free, reset trans_anim_pg_exit
                        if (trans_anim_pg_exit == cur_page)  trans_anim_pg_exit = NULL;
                        if (trans_anim_pg_enter == cur_page) trans_anim_pg_enter = NULL;
#endif
                        //move stoped page to rm_page_list
                        prev_page = pn_page->prev;
                        rt_list_remove(pn_page);
                        rt_list_insert_before(&rm_page_list, pn_page);
                        pn_page = prev_page;
                    }
                    else
                    {
                        page_st_changes--;//Wait other page repalce active screen, do ONSTOP next turn
                    }
                }
                break;

                default:
                    app_sche_assert("app[%s] page[%s] : Invalid msg %d", cur_app->id, cur_page->name, msg);
                    break;
                }
            }
        }

        /*Move active page to the head of cur_app*/
        if (active_page != NULL)
        {
            rt_list_remove(active_page);
            rt_list_insert_after(&cur_app->page_list, active_page);
        }
    }

    /* Move actived app to the head of running_app_list */
    if (actived_app != NULL)
    {
        rt_list_remove(actived_app);
        rt_list_insert_after(&running_app_list, actived_app);
    }


    /*
        remove stoped pages
    */
    if (!rt_list_isempty(&rm_page_list))
    {
        rt_list_t *pn_next_page;
        rt_list_for_each_safe(pn_page, pn_next_page, &rm_page_list)
        {
            cur_page = rt_list_entry(pn_page, subpage_node_t, node);
            cur_app = cur_page->parent;

            app_sche_d("app[%s] remove&free page[%s] %x", cur_app->id, cur_page->name, cur_page);
            app_sche_free(cur_page);
        }

    }



#ifdef TRANS_ANIMATION
    //Play animation if there is page resumed
    if (trans_anim_pg_enter)
    {
        if (disable_trans_anim)
        {
            app_sche_i("Animation disbale, skip.");
            trans_anim_pg_enter = NULL;
            trans_anim_pg_exit  = NULL;
        }
        else if (task_msg_mbx->entry) /* Pre-abort animation if there is msg, http://10.21.20.167/issues/4476 */
        {
            app_sche_i("Animation pre-abort.");
            trans_anim_pg_enter = NULL;
            trans_anim_pg_exit  = NULL;
        }
        else
        {
            if (!trans_anim_playing)
            {
                const gui_app_trans_anim_group_t *g_anim_enter = NULL;
                const gui_app_trans_anim_group_t *g_anim_exit = NULL;
                screen_t anim_enter_scr = NULL;
                screen_t anim_exit_scr  = NULL;

                rt_err_t anim_setup_err;
                bool press_back = pg_enter_was_paused && pg_exit_being_stop;
                if ((0 == strcmp(p_desktop_app_id, trans_anim_pg_enter->parent->id))
                        && (rt_list_len(&trans_anim_pg_enter->node) <= 1))
                {
                    //The main app's first subpage always performs 'go-back' animation
                    press_back = true;
                }
                if (trans_anim_pg_enter->parent->flag & APP_FLAG_GOBACK_ANIM)
                {
                    press_back = true;
                }

                app_sche_i("Try setup trans-anim");

                if (trans_anim_pg_exit)
                {
                    g_anim_exit     = &trans_anim_pg_exit->a_group;
                    anim_exit_scr = trans_anim_pg_exit->scr;
                }

                g_anim_enter = &trans_anim_pg_enter->a_group;
                anim_enter_scr = trans_anim_pg_enter->scr;

                /*
                    it's good time to peformance animation:

                    new screen is reday to refresh, and old screen
                    was draw in buf_act
                */
                //hold gui_app_scheduler task util trans animation finished
                trans_anim_playing = true;


                act_scr_before_app_trans_anim = port_app_sche_get_act_scr();

                anim_setup_err = app_trans_animation_setup(g_anim_enter, g_anim_exit, anim_enter_scr, anim_exit_scr,
                                 app_trans_anim_finish_callback, press_back, manual_trans_anim);

                if (RT_EOK != anim_setup_err)
                {
                    if (RT_ENOSYS == anim_setup_err)
                    {
                        app_sche_i("Trans-anim NOT supported, skip.");
                    }
                    else if (RT_EEMPTY == anim_setup_err)
                    {
                        app_sche_i("Trans-anim is OFF, skip.");
                    }
                    else
                    {
                        app_sche_i("Trans-anim setup failed(%d).", anim_setup_err);
                    }

                    trans_anim_playing = false;
                    act_scr_before_app_trans_anim = NULL;

                    trans_anim_pg_exit = NULL;
                    trans_anim_pg_enter = NULL;
                }
                else
                {
                    app_sche_i("Trans-anim setup done.");
                }
            }
            else
            {
                app_sche_i("Previous animation not finished, skip.");
                trans_anim_pg_enter = NULL;
                trans_anim_pg_exit  = NULL;
            }
        }

    }
#endif

SCHEDULE_SUBPAGE_END:

#ifdef TRANS_ANIMATION
    /*
        In case trans_anim is not start, reset manual anim state to stopped.
    */
    if (manual_trans_anim && !trans_anim_playing)
    {
        app_sche_i("Start manual trans anim fail,reset.");
        app_trans_animation_reset();
    }
    manual_trans_anim = false;
#endif /* TRANS_ANIMATION */

    return page_st_changes;
}

static void do_destory_apps(void)
{
    rt_list_t *pn_app, *pn_next_app;
    rt_list_for_each_safe(pn_app, pn_next_app, &suspend_app_list)
    {
        gui_runing_app_t *cur_app;
        cur_app = rt_list_entry(pn_app, gui_runing_app_t, node);

        if (app_st_destoryed == cur_app->target_state)
        {
            app_do(cur_app, APP_EXEC_DESTORY);
            app_sche_free(cur_app);
        }
    }
}

//Assert if there is another subpage has same id in each running app
static void check_duplicated_subpage(void)
{
    if (!en_check_duplicated_subpage) return;

    rt_list_t *pn_app;
    rt_list_for_each(pn_app, &running_app_list)
    {
        gui_runing_app_t *p_app = rt_list_entry(pn_app, gui_runing_app_t, node);

        rt_list_t *pn_cur_page;
        rt_list_for_each(pn_cur_page, &p_app->page_list)
        {
            subpage_node_t *p_cur_page = rt_list_entry(pn_cur_page, subpage_node_t, node);

            rt_list_t *pn_page;
            for (pn_page = pn_cur_page->next; pn_page != &p_app->page_list; pn_page = pn_page->next)
            {
                subpage_node_t *p_page = rt_list_entry(pn_page, subpage_node_t, node);

                //app_sche_d("check_duplicated_pages %s  %s  %s",p_app->id, p_cur_page->name,p_page->name);
                if ((0 == strcmp(p_page->name, p_cur_page->name)) && (p_page != p_cur_page))
                {
                    app_sche_assert("Found duplicated page[%s] in app[%s] %x", p_cur_page->name, p_cur_page->parent->id, p_cur_page);
                }
            }
        }
    }

}
static void check_duplicated_apps(void)
{
    if (!en_check_duplicated_apps) return;


    rt_list_t *ptr_cur;

    rt_list_for_each(ptr_cur, &running_app_list)
    {
        gui_runing_app_t *cur_app = rt_list_entry(ptr_cur, gui_runing_app_t, node);
        rt_list_t *ptr;
        for (ptr = ptr_cur->next; ptr != &running_app_list; ptr = ptr->next)
        {
            gui_runing_app_t *p_app = rt_list_entry(ptr, gui_runing_app_t, node);

            //app_sche_d("check_duplicated_apps %s  %s",cur_app->id, p_app->id);
            if ((0 == strcmp(p_app->id, cur_app->id)) && (p_app != cur_app))
            {
                app_sche_assert("Found duplicated app [%s], %x", cur_app->id, cur_app);
            }
        }
    }

}

static uint32_t app_scheduler_new(void)
{
    uint32_t ret_v;
    check_duplicated_apps();
    check_duplicated_subpage();
    ret_v = schedule_apps();
    ret_v += schedule_subpage();
    ret_v += schedule_apps();

    return ret_v;
}

void app_schedule_task(rt_mailbox_t msg_mbx)
{
    bool done_nothing = true;
    uint8_t recursive_cnt = 0;//To avoid recursive by sent 'GUI_APP_MSG_RUN_APP_I'

    gui_app_msg_t *p_msg;

    task_msg_mbx = msg_mbx;

    if (scheduler_suspended == suspend_task)  return;

    app_sche_lock();
    scheduler_loop_st = 1;

#ifdef TRANS_ANIMATION
    if (trans_anim_playing)
    {
        /*
           hold gui_app_scheduler task util trans animation finished
        */

        if (msg_mbx->entry != 0)
        {
            app_sche_i("Abort app trans anim, msg_entry = %d", msg_mbx->entry);
            app_trans_anim_abort();
        }

        /*Wait trans_anim_playing be 0 by gui_trans_animation's callback*/
        goto EXIT_TASK;
    }
#endif /* TRANS_ANIMATION */

    //dispatch msg
    while (RT_EOK == rt_mb_recv(msg_mbx, (rt_uint32_t *)&p_msg, RT_WAITING_NO))
    {
        done_nothing = false;

        app_sche_i(">>Execute msg[%s] tick:%d", fwk_msg_to_name(p_msg->msg_id), p_msg->tick);
        switch (p_msg->msg_id)
        {
        case GUI_APP_MSG_RUN_APP_I:
        case GUI_APP_MSG_RUN_APP:
        {
            //int argc;
            //char *argv[GUI_APP_CMD_ARG_MAX];
            char *id;
            //gui_app_msg_t msg_bak;
            /* split arguments */
            //memset(argv, 0x00, sizeof(argv));
            //memcpy(&msg_bak, &msg, sizeof(msg));
            //argc = app_cmd_split(p_msg->content.cmd, rt_strlen(p_msg->content.cmd), argv);
            id = &(p_msg->content.intnt.content[0]);
            //app_sche_i("gui_app_run cmd:%s", id);
            if (strlen(id) == 0)
            {
                app_sche_i("err: app_run id == 0");
            }
            else
            {
                gui_runing_app_t *run_app = get_runing_app_handler(id);

                if (run_app)
                {
                    app_sche_d("Found app[%s] in running list. Args are -", id);
                    if (0 == memcmp(&(run_app->param), &(p_msg->content.intnt), sizeof(run_app->param)))
                    {
                        app_sche_d("- same. Resume it");

                        app_resume(&run_app->node);
                    }
                    else
                    {
                        app_sche_d("- not same. Exit it and re-entry");

                        if (rt_list_isempty(&run_app->page_list) && (0 == msg_mbx->entry))
                        {
                            /*Task will be stuck in this while loop if we not assert here*/
                            app_sche_assert("Can't exit app because it's page list is empty!");
                        }


                        /*destory app*/
                        if (1 == rt_list_len(&running_app_list))
                            app_stop_running();//Make sure current subpage stopped, 'app_destory' can't do it.
                        else
                            app_exit(&run_app->node);


                        /*retry when app is exit, and reuse current message's tick*/
                        app_run(msg_mbx, &(p_msg->content.intnt), p_msg->tick);
                    }
                }
                else
                {
                    //run_app = get_suspend_app_handler(id);
                    run_app = get_suspend_or_destoryed_app_handler(id);

                    if (run_app)
                    {
                        //Restart suspend or destoryed app always to bring up their subpages
                        app_sche_d("Found app[%s] in suspend list. Exit it and re-entry", id);

                        /*destory app*/
                        app_set_target_state(run_app, app_st_destoryed);

                        //No creating subpage msg is pending, destory app immediately
                        if (0 == msg_mbx->entry) do_destory_apps();

                        /*retry when app is exit, and reuse current message's tick*/
                        app_run(msg_mbx, &(p_msg->content.intnt), p_msg->tick);
                    }
                    else
                    {
                        /* no running entity*/
                        app_launch_new(id, &(p_msg->content.intnt));
                    }
                }

            }
        }
        break;

        case GUI_APP_MSG_EXIT_APP:
        {
            char *id;
            id = &(p_msg->content.intnt.content[0]);

            gui_runing_app_t *p_app = get_runing_app_handler(id);
            if (!p_app)
                p_app = get_suspend_app_handler(id);

            if (p_app)
            {
                app_sche_i("Exit specified app[%s]", id);
                app_exit(&(p_app->node));
            }
            else
            {
                app_sche_e("Exit app[%s]:Not found", id);
            }
        }
        break;

        case GUI_APP_MSG_GOBACK:
        {
            if (is_running_app_handler(p_msg->handler))
            {
                if (is_last_page_present())
                {
                    if (rt_list_len(&suspend_app_list) > 0)
                    {
                        app_sche_i("Go back last page");
                        app_restart_last_suspend();
                    }
                    else
                    {
                        app_sche_i("Go back last page, return to Main");
                        app_run_by_id(msg_mbx, p_main_app_id);
                    }
                }
                else
                {
                    app_goback(&(p_msg->handler->node));
                }
            }

        }
        break;

#ifdef TRANS_ANIMATION
        case GUI_APP_MSG_MANUAL_GOBACK_ANIM:
        {
            if (is_running_app_handler(p_msg->handler))
            {
                if (is_last_page_present())
                {
                    if (rt_list_len(&suspend_app_list) > 0)
                    {
                        app_sche_i("Go back last page");
                        app_trans_animation_reset();
                        app_restart_last_suspend();
                    }
                    else
                    {
                        app_sche_i("Go back last page, return to Main");
                        app_trans_animation_reset();
                        app_run_by_id(msg_mbx, p_main_app_id);
                    }
                }
                else
                {
                    app_goback(&(p_msg->handler->node));
                    manual_trans_anim = true;
                }
            }
            else
            {
                app_trans_animation_reset();
            }
        }
        break;
#endif /* TRANS_ANIMATION */

        case GUI_APP_MSG_OPEN_PAGE:
        {
            if (is_running_app_handler(p_msg->handler))
                app_create_page(&(p_msg->handler->node), p_msg->content.page.name, p_msg->content.page.msg_handler, p_msg->content.page.user_data);
            else if (is_suspend_or_destoryed_app_handler(p_msg->handler))
            {
                app_sche_i("Create page on suspend app");

                p_msg->handler->flag &= ~APP_FLAG_GOBACK_ANIM;
                app_restart(&(p_msg->handler->node));
                p_msg->handler->state = app_st_running;//Not call app entry function
                app_create_page(&(p_msg->handler->node), p_msg->content.page.name, p_msg->content.page.msg_handler, p_msg->content.page.user_data);
            }
            else
            {
                app_sche_assert("Create page %s[%x] error, invalid app handler %x!", p_msg->content.page.name, p_msg->content.page.msg_handler, p_msg->handler);
            }
        }
        break;

        case GUI_APP_MSG_RM_PAGE:
        {
            if (is_running_app_handler(p_msg->handler))
            {
                if (is_last_page_present())
                {
                    if (rt_list_len(&suspend_app_list) > 0)
                    {
                        app_sche_i("Remove last page");
                        app_restart_last_suspend();
                    }
                    else
                    {
                        app_sche_i("Remove last page, return to Main");
                        app_run_by_id(msg_mbx, p_main_app_id);
                    }
                }
                else
                {
                    app_rm_page(&(p_msg->handler->node), p_msg->content.page.name);
                }
            }
            else
            {
                app_sche_i("Remove invalid subpage %x", p_msg->handler);
            }
        }
        break;

        case GUI_APP_MSG_GOBACK_TO_PAGE:
        {
            if (is_running_app_handler(p_msg->handler))
                app_goback_to_page(&(p_msg->handler->node), p_msg->content.page.name);
        }
        break;

        case GUI_APP_MSG_REFRESH_PAGE:
        {
            if (is_running_app_handler(p_msg->handler))
                app_refr_page(&(p_msg->handler->node), p_msg->content.page.name);
        }
        break;

        case GUI_APP_MSG_CLEANUP_BACKGROUND:
        {
            app_stop_all_backgrounds();
        }
        break;

        case GUI_APP_MSG_CLEANUP:
        {
            app_stop_all();
        }
        break;

        case GUI_APP_MSG_SUSPEND_SCHEDULER:
        {
            app_sche_d("Pause all app and suspend scheduler");
            app_set_all_app_all_page_state(page_st_paused);
            suspend_task = scheduler_suspended;
        }
        break;

        case GUI_APP_MSG_RESUME_SCHEDULER:
        {
            app_sche_d("Resume last subpage after resume scheduler");
            if (NULL != actived_app)
                app_resume(actived_app);
        }
        break;

        default:
            app_sche_i("app_schedule_task unknow msg %d", p_msg->msg_id);

            break;
        }

        if (GUI_APP_MSG_RUN_APP_I == p_msg->msg_id)
        {
            recursive_cnt++;
        }
        else
        {
            recursive_cnt = 0;
        }

        /*Free msg*/
        app_sche_free(p_msg);
        p_msg = NULL;

        while (app_scheduler_new())
        {
#ifdef TRANS_ANIMATION
            if (trans_anim_playing)
            {
                app_sche_d("--Pause app_schedule_task--");
                goto END_LOOP;
            }
#endif /* TRANS_ANIMATION */
        }



        if (scheduler_suspended == suspend_task) break;
        if (recursive_cnt > 2)
        {
            app_sche_e("Detected recursive loop, clear destroyed apps, and break it");
            do_destory_apps();
            break;
        }
    }



#ifdef TRANS_ANIMATION
END_LOOP:
#endif

    /*
          No more unhandle msg in queue now, destory apps here.
     */
    if (0 == msg_mbx->entry)
    {
        do_destory_apps();
    }


    /*
      The lv input device may be enable in sche_idle_hook,
      so trans animation could be abort by input.
    */
    if (sche_idle_hook_func)
    {
        app_sche_state state = APP_SCHE_IDLE;

#ifdef TRANS_ANIMATION
        if (trans_anim_playing) state = APP_SCHE_SUSPEND;
#endif
        if (scheduler_suspended == suspend_task) state = APP_SCHE_SUSPEND;

        if (msg_mbx->entry) state = APP_SCHE_SUSPEND;

        sche_idle_hook_func(state);
    }


EXIT_TASK:
    if ((2 == scheduler_loop_st) || (!done_nothing))
    {
        app_sche_d("app_schedule_task done.");
    }
    scheduler_loop_st = 0;
    app_sche_unlock();

}


uint32_t app_schedule_init(
    app_load_func ld_func,
    app_destory_func app_dstry_func,
    app_sche_debug_func debug_func,
    app_sche_hook_func idle_hook,
    app_sche_malloc_func malloc_func,
    app_sche_free_func free_func,
    app_sche_subpage_func subpage_func
)
{
    rt_list_init(&running_app_list);
    rt_list_init(&suspend_app_list);


    dummy_scr = port_app_sche_create_scr();
    sche_app_ld_func = ld_func;
    sche_app_dstry_func = app_dstry_func;
    sche_app_debug_func = debug_func;
    sche_idle_hook_func = idle_hook;
    sche_malloc_func = malloc_func;
    sche_free_func = free_func;
    sche_subpage_func = subpage_func;

    if ((NULL == dummy_scr) || (NULL == sche_malloc_func) || (NULL == sche_free_func) || (NULL == sche_app_ld_func) || (NULL == sche_app_dstry_func))
    {
        return 1;
    }

    mutex_lock = rt_sem_create("app_sche", 1, 0);
    app_schedule_change_main_app_id(DEFAULT_MAIN_APP_ID);
    return 0;
}

void app_schedule_set_idle_hook(app_sche_hook_func idle_hook)
{
    sche_idle_hook_func = idle_hook;
}

void app_schedule_max_running_apps(uint32_t v)
{
    max_running_apps = v;
}

uint32_t app_schedule_get_max_running_apps(void)
{
    return max_running_apps;
}

void app_schedule_enable_trans_anim(bool en)
{
    disable_trans_anim = !en;
}

uint32_t app_schedule_change_main_app_id(const char *id)
{
    if (NULL == id) return 1;
    p_main_app_id = id;
    return 0;
}
uint32_t app_schedule_change_desktop_app_id(const char *id)
{
    if (NULL == id) return 1;
    p_desktop_app_id = id;
    return 0;
}


void app_schedule_destory_suspend_apps(uint32_t v)
{
    en_suspend_app = (v != 0) ? 0 : 1;
}

void app_schedule_check_duplicate(uint8_t app, uint8_t subpage)
{
    en_check_duplicated_apps = app;
    en_check_duplicated_subpage = subpage;
}

void app_scheduler_print_perf_tick(uint8_t en)
{
    app_subgpage_perf_tick = en;
}

void display_app_list(rt_list_t *app_list)
{
    rt_list_t *pn_app;
    rt_list_t *pn_page;

    rt_list_for_each(pn_app, app_list)
    {
        gui_runing_app_t *cur_app;
        cur_app = rt_list_entry(pn_app, gui_runing_app_t, node);

        app_sche_i("app[%s] param[%s]  %x, state[%s], tgt_st[%s], tick_cnt=%d", cur_app->id, cur_app->param.content, cur_app,
                   app_get_state_name(cur_app->state), app_get_state_name(cur_app->target_state), cur_app->tick_cnt);

        rt_list_for_each(pn_page, &cur_app->page_list)
        {
            subpage_node_t *cur_page;
            cur_page = rt_list_entry(pn_page, subpage_node_t, node);

            app_sche_i("\tpage[%s] state[%s]  %x scr=%x tick_cnt=%d", cur_page->name,
                       page_get_state_name(cur_page->state), cur_page, cur_page->scr, cur_page->tick_cnt);
        }
    }
}

rt_err_t list_schedule_app(void)
{
    app_sche_i("-------Running app list-------");
    display_app_list(&running_app_list);
    app_sche_i("-------Suspend app list-------");
    display_app_list(&suspend_app_list);

    return 0;
}

page_state_enum app_schedule_get_page_state(const char *app_id, const char *page_id)
{
    rt_list_t *pn_app;
    rt_list_t *pn_page;

    RT_ASSERT(app_id && page_id);

    rt_list_for_each(pn_app, &running_app_list)
    {
        gui_runing_app_t *cur_app;
        cur_app = rt_list_entry(pn_app, gui_runing_app_t, node);

        //app_sche_d("%s:app[%s] param[%s]  %x \n", __func__, cur_app->id, cur_app->param.content, cur_app);
        if (0 != rt_strcmp(app_id, cur_app->id))
        {
            continue;
        }
        rt_list_for_each(pn_page, &cur_app->page_list)
        {
            subpage_node_t *cur_page;
            cur_page = rt_list_entry(pn_page, subpage_node_t, node);
            if (0 == rt_strcmp(page_id, cur_page->name))
            {
                return cur_page->state;
            }
            //app_sche_d("%s:tpage[%s] state[%s]  %x scr=%x \n", __func__, cur_page->name, page_get_state_name(cur_page->state), cur_page, cur_page->scr);
        }
    }
    return page_st_stoped;
}


