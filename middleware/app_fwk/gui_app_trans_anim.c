#include <rtthread.h>
#include "gui_app_int.h"




#define DBG_TAG           "APP.ANI"
#define DBG_LVL          DBG_INFO //DBG_LOG //
#include "rtdbg.h"

#ifndef _MSC_VER
    #include "drv_ext_dma.h"
#endif

#define TRAN_ANIM_PLAY_TIME  300



#define trans_anim_log_i LOG_I
#define trans_anim_log_d LOG_D

#define IS_ZOOM_TRANFORM_ANIM_TYPE(at) ((GUI_APP_TRANS_ANIM_ZOOM_IN == (at)) || (GUI_APP_TRANS_ANIM_ZOOM_OUT == (at)))
#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif


#if !defined (APP_TRANS_ANIMATION_NONE)

typedef enum
{
    ma_state_stopped,
    ma_state_started, //Sent 'GUI_APP_MSG_MANUAL_GOBACK_ANIM' already, but not been executed.
    ma_state_active,  //Playing manual animation,and can control by anim_manual_control
    ma_state_freerun,
    ma_state_aborted, //Stop manual animation before it is active, use free run animation instead.
} manual_anim_state_t;




static screen_t app_trans_scr = NULL;
static screen_t app_trans_old_scr = NULL;
static gui_anim_obj_t enter_anim_obj, exit_anim_obj;

static app_trans_anim_xcb_t app_trans_cbk = NULL;
static gui_app_trans_anim_t enter_anim_cfg, exit_anim_cfg;
static bool app_trans_anim_back;
//static bool is_manual_anim_flag = false;
static manual_anim_state_t manual_anim_s = ma_state_stopped;
static app_trans_ex_cb_t app_trans_ex_cb = NULL;
static uint32_t manual_animation_start_process;
static void print_trans_anim_info(const char *prefix, const gui_app_trans_anim_t *cfg);


static char *ma_state_name(manual_anim_state_t s)
{
#define TO_NAME_CASE(v) case v: return #v
    switch (s)
    {
        TO_NAME_CASE(ma_state_stopped);
        TO_NAME_CASE(ma_state_started);
        TO_NAME_CASE(ma_state_active);
        TO_NAME_CASE(ma_state_freerun);
        TO_NAME_CASE(ma_state_aborted);
    default:
        return "UNKNOW";
    }
}

static manual_anim_state_t ma_state_change(manual_anim_state_t dst_state)
{
    manual_anim_state_t ret_v;
    manual_anim_state_t cur_state = manual_anim_s;

    if (cur_state == dst_state) return dst_state;

    trans_anim_log_i("manual_anim_s [%s] -> [%s]", ma_state_name(cur_state), ma_state_name(dst_state));

    if (ma_state_stopped == dst_state)
    {
        if (ma_state_started != cur_state)
            ret_v = ma_state_stopped;
        else
            ; //Wait until 'GUI_APP_MSG_MANUAL_GOBACK_ANIM' been execute.
    }
    else if (ma_state_aborted == dst_state)
    {
        ret_v = ma_state_aborted;
    }
    else
    {
        RT_ASSERT(cur_state < dst_state);

        ret_v = dst_state;
    }

    manual_anim_s = ret_v;
    return ret_v;
}


static gui_anim_value_t app_trans_anim_process_2_value(gui_anim_value_t start, gui_anim_value_t end, uint32_t process)
{

    /*Calculate the current step*/
    // uint32_t step;


    /* Get the new value which will be proportional to `step`
     * and the `start` and `end` values*/
    int32_t new_value;
    new_value = (int32_t)process * (end - start) / MANUAL_TRAN_ANIM_MAX_PROCESS;
    new_value += start;
    return (gui_anim_value_t)new_value;
}


static void app_trans_anim_clean(void)
{
    bool is_manual_anim = false;

    app_trans_anim_free_run_clean();

    if (enter_anim_obj)
    {
        app_trans_animation_obj_destroy(enter_anim_obj);
        enter_anim_obj = NULL;
    }

    if (exit_anim_obj)
    {
        app_trans_animation_obj_destroy(exit_anim_obj);
        exit_anim_obj = NULL;
    }


    if (app_trans_old_scr)
    {
        port_app_sche_reset_indev(port_app_sche_get_act_scr());
        port_app_sche_load_scr(app_trans_old_scr);
        app_trans_old_scr = NULL;
    }
    if (app_trans_scr)
    {
        is_manual_anim = (bool) port_app_sche_get_user_data(app_trans_scr);
        port_app_sche_del_scr(app_trans_scr);
        app_trans_scr = NULL;
    }


    /*
        In case of free run animation be aborted by manual anim
    */
    if (is_manual_anim)
    {
//      is_manual_anim_flag = false;
        ma_state_change(ma_state_stopped);
    }
}


static void app_trans_anim_cbk(TransResult_T  res)
{
    if (app_trans_cbk)
    {
        app_trans_cbk(res);
        app_trans_cbk = NULL;
    }

    if (app_trans_ex_cb)
    {
        app_trans_ex_cb();
        app_trans_ex_cb = NULL;
    }

}


static rt_err_t app_trans_anim_process(gui_anim_obj_t img, const gui_app_trans_anim_t *a_cfg, uint32_t flag, gui_anim_value_t process)
{
    if (NULL == a_cfg) return RT_EINVAL;
    if (NULL == img) return RT_EINVAL;

    switch (a_cfg->type)
    {
    /*zoom*/
    case GUI_APP_TRANS_ANIM_ZOOM_IN:
    case GUI_APP_TRANS_ANIM_ZOOM_OUT:
#if APP_TRANS_ANIM_FULL_SCALE == APP_TRANS_ANIM_SNAPSHOT_SCALE
        app_trans_anim_set_pivot(img, &a_cfg->cfg.zoom.pivot);
#endif
        if (a_cfg->cfg.zoom.zoom_end != a_cfg->cfg.zoom.zoom_start)
        {
            app_trans_anim_set_zoom(img, app_trans_anim_process_2_value(a_cfg->cfg.zoom.zoom_start, a_cfg->cfg.zoom.zoom_end, process));
        }
        if (a_cfg->cfg.zoom.opa_start != a_cfg->cfg.zoom.opa_end)
        {
            app_trans_anim_set_opa_scale(img, app_trans_anim_process_2_value(a_cfg->cfg.zoom.opa_start, a_cfg->cfg.zoom.opa_end, process));
        }
        break;

    /* push */
    case GUI_APP_TRANS_ANIM_PUSH_RIGHT_IN:
    case GUI_APP_TRANS_ANIM_PUSH_RIGHT_OUT:
    case GUI_APP_TRANS_ANIM_PUSH_LEFT_IN:
    case GUI_APP_TRANS_ANIM_PUSH_LEFT_OUT:
        if (a_cfg->cfg.push.x_start != a_cfg->cfg.push.x_end)
        {
            app_trans_anim_set_x(img, app_trans_anim_process_2_value(a_cfg->cfg.push.x_start, a_cfg->cfg.push.x_end, process));
        }

        if (a_cfg->cfg.push.opa_start != a_cfg->cfg.push.opa_end)
        {
            app_trans_anim_set_opa_scale(img, app_trans_anim_process_2_value(a_cfg->cfg.push.opa_start, a_cfg->cfg.push.opa_end, process));
        }

        break;

    case GUI_APP_TRANS_ANIM_NONE:
        return RT_EOK;

    case GUI_APP_TRANS_ANIM_CUSTOM:
        if (a_cfg->cfg.cust.exe_cb)
            a_cfg->cfg.cust.exe_cb(img, a_cfg->cfg.cust.user_data, flag, process);
        break;

    default:
        return RT_ERROR;

    }

    return RT_EOK;
}

/*
    Process 0 ~ 1000
*/
static void anim_manual_control(gui_anim_value_t process)
{
    if (app_trans_anim_back)
    {
        app_trans_anim_process(exit_anim_obj,  &exit_anim_cfg, FLAG_TRANS_ANIM_REVERSE | FLAG_TRANS_ANIM_FG, process);
        app_trans_anim_process(enter_anim_obj, &enter_anim_cfg, FLAG_TRANS_ANIM_REVERSE, process);
    }
    else
    {
        app_trans_anim_process(exit_anim_obj,  &exit_anim_cfg, 0, process);
        app_trans_anim_process(enter_anim_obj, &enter_anim_cfg, FLAG_TRANS_ANIM_FG, process);
    }
}

static void anim_free_run_ready(TransResult_T res)
{
    trans_anim_log_i("anim_free_run_ready %d \n", res);

    app_trans_anim_clean();
    app_trans_anim_cbk(res);
}

#define FREE_RUN_START(start, end, duration) app_trans_anim_free_run_start((start),(end),(duration),anim_manual_control,anim_free_run_ready)

rt_err_t anim_manual_free_run(uint32_t process)
{
    rt_err_t err;

    if (process > (MANUAL_TRAN_ANIM_MAX_PROCESS / 2))
        err = FREE_RUN_START(process, MANUAL_TRAN_ANIM_MAX_PROCESS, TRAN_ANIM_PLAY_TIME * (MANUAL_TRAN_ANIM_MAX_PROCESS - process) / MANUAL_TRAN_ANIM_MAX_PROCESS);
    else
        err = FREE_RUN_START(process, 0, TRAN_ANIM_PLAY_TIME * process / MANUAL_TRAN_ANIM_MAX_PROCESS);

    return err;
}


//Overwrite top obj's animation to match another animation if it was default
static rt_err_t overwrite_default_anim(bool is_back, const gui_app_trans_anim_group_t *g_enter,
                                       const gui_app_trans_anim_group_t *g_exit)
{
#ifdef APP_TRANS_ANIM_POLICY_1
    if ((is_back) && (exit_anim_cfg.type == GUI_APP_TRANS_ANIM_EXIT_DEFAULT)) //We won't see anim effect as 'exit_anim_obj' will full cover 'enter_anim_obj'.
    {
        if (IS_ZOOM_TRANFORM_ANIM_TYPE(enter_anim_cfg.type))
        {
            app_trans_anim_init_cfg(&exit_anim_cfg, enter_anim_cfg.type);
#ifdef APP_TRANS_ANIMATION_SCALE_NEXT
            app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
#endif
        }
        else if (GUI_APP_TRANS_ANIM_PUSH_RIGHT_IN == enter_anim_cfg.type)
        {
            app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_PUSH_RIGHT_OUT);
            app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
        }
        else if (GUI_APP_TRANS_ANIM_PUSH_RIGHT_OUT == enter_anim_cfg.type)
        {
            app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_PUSH_RIGHT_IN);
            app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
        }
        else
        {
            //Overwrite exit_anim with entering subpage's exit_animation_cfg.
            memcpy(&exit_anim_cfg, &g_enter->a_exit, sizeof(gui_app_trans_anim_t));

            //Not copy user data for custom animtion, for seperating anim.
            if (GUI_APP_TRANS_ANIM_CUSTOM == g_enter->a_exit.type) exit_anim_cfg.cfg.cust.user_data = NULL;
        }
    }
    else if ((!is_back) && (enter_anim_cfg.type == GUI_APP_TRANS_ANIM_ENTER_DEFAULT)) //Same as previous
    {
        if (IS_ZOOM_TRANFORM_ANIM_TYPE(exit_anim_cfg.type))
        {
            app_trans_anim_init_cfg(&enter_anim_cfg, exit_anim_cfg.type);
#ifdef APP_TRANS_ANIMATION_SCALE_NEXT
            app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
#endif
        }
        else if (GUI_APP_TRANS_ANIM_PUSH_RIGHT_IN == exit_anim_cfg.type)
        {
            app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_PUSH_RIGHT_OUT);
            app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
        }
        else if (GUI_APP_TRANS_ANIM_PUSH_RIGHT_OUT == exit_anim_cfg.type)
        {
            app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_PUSH_RIGHT_IN);
            app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
        }
        else
        {
            memcpy(&enter_anim_cfg, &g_exit->a_enter, sizeof(gui_app_trans_anim_t));

            //Not copy user data for custom animtion, for seperating anim.
            if (GUI_APP_TRANS_ANIM_CUSTOM == g_exit->a_enter.type) enter_anim_cfg.cfg.cust.user_data = NULL;
        }
    }
#endif /* APP_TRANS_ANIM_POLICY_1 */

    if ((GUI_APP_TRANS_ANIM_EXIT_DEFAULT == exit_anim_cfg.type) && (GUI_APP_TRANS_ANIM_ENTER_DEFAULT == enter_anim_cfg.type))
    {
        if (is_back)
        {
            app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_PUSH_RIGHT_OUT);
            app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
        }
        else
        {
            app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_PUSH_RIGHT_IN);
            app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_NONE);
        }
    }

    return RT_EOK;
}


rt_err_t app_trans_anim_abort(void)
{
    trans_anim_log_i("app_trans_anim_abort");

    if (app_trans_scr)
    {
        TransResult_T  res;

        bool is_manual_anim = (bool) port_app_sche_get_user_data(app_trans_scr);

        if (is_manual_anim)
            res = TRANS_RES_ABORTED;
        else
            res = TRANS_RES_FINISHED;//Normal animation can not return 'TRANS_RES_ABORTED'

        app_trans_anim_clean();
        app_trans_anim_cbk(res);
    }

    return RT_EOK;
}

/**
 * @brief Setup an animation to tansform from screen 'enter_scr' to 'exit_scr'
 * @param enter_scr -
 * @param exit_scr -
 * @param cbk - cbk when animation finish
 * @param is_back - is goback anmation
 * @param g_enter - animation groups of enter_scr, include enter&exit 2 animation.
 * @param g_exit - same as above, but for exit_scr.
 * @param is_manual_anim -
 * @return
 */
rt_err_t app_trans_animation_setup(const gui_app_trans_anim_group_t *g_enter,
                                   const gui_app_trans_anim_group_t *g_exit,
                                   const screen_t enter_scr,
                                   const screen_t exit_scr,
                                   app_trans_anim_xcb_t cbk, bool is_back, bool is_manual_anim)
{
    rt_err_t err = RT_EOK;
    rt_tick_t s_tick = rt_tick_get();
    bool b_exit_scale, b_enter_scale;
    int exit_buf_index = 0;
    int enter_buf_index = 0;
    RT_ASSERT((NULL == app_trans_scr) && (NULL == app_trans_cbk)); //Make sure previous anim done.
    RT_ASSERT(NULL != g_enter);

#ifdef APP_TRANS_ANIM_POLICY_1
    if (GUI_APP_TRANS_ANIM_NONE == g_enter->a_enter.type)
    {
        /*Enter animation is disable, skip trans animtion*/
        return RT_EEMPTY;
    }

    /* Backup paramters*/
    {
        memcpy(&enter_anim_cfg, &g_enter->a_enter, sizeof(gui_app_trans_anim_t));

        if (g_exit)
            memcpy(&exit_anim_cfg,  &g_exit->a_exit,  sizeof(gui_app_trans_anim_t));
        else
            //app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_EXIT_DEFAULT);
            return RT_ERROR;
    }
#else

    const gui_app_trans_anim_group_t *p_act_group = NULL;

    if (!exit_scr)
    {
        /*First page loaded, skip trans animtion*/
        return RT_EEMPTY;
    }

    if (is_back)
    {
        if (!g_exit) return RT_ERROR;

        else if (g_enter->prio_down >= g_exit->prio_up)
            p_act_group = g_enter;
        else
            p_act_group = g_exit;
    }
    else
    {
        if (!g_exit)
            p_act_group = g_enter;
        else if (g_enter->prio_up > g_exit->prio_down)
            p_act_group = g_enter;
        else
            p_act_group = g_exit;
    }


    memcpy(&enter_anim_cfg, &p_act_group->a_enter, sizeof(gui_app_trans_anim_t));
    memcpy(&exit_anim_cfg, &p_act_group->a_exit, sizeof(gui_app_trans_anim_t));

    if ((GUI_APP_TRANS_ANIM_NONE == enter_anim_cfg.type) && (GUI_APP_TRANS_ANIM_NONE == exit_anim_cfg.type))
    {
        /*Enter animation is disable, skip trans animtion*/
        return RT_EEMPTY;
    }

#endif

    trans_anim_log_d("app_trans_animation_setup start is_back=%d\n", is_back);
    print_trans_anim_info("enter_anim", &enter_anim_cfg);
    print_trans_anim_info("exit_anim", &exit_anim_cfg);

#ifdef APP_TRANS_ANIMATION_OVERWRITE
    /*Unsupported transform animation*/
    if (IS_ZOOM_TRANFORM_ANIM_TYPE(enter_anim_cfg.type)) app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_ENTER_DEFAULT);
    if (IS_ZOOM_TRANFORM_ANIM_TYPE(exit_anim_cfg.type)) app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_EXIT_DEFAULT);
#endif

    overwrite_default_anim(is_back, g_enter, g_exit);


    /*
        Create a new screen for play animation
    */
    app_trans_old_scr = port_app_sche_get_act_scr();
    RT_ASSERT(app_trans_old_scr != NULL);
    port_app_sche_reset_indev(app_trans_old_scr);

    app_trans_scr = port_app_sche_create_scr();
    RT_ASSERT(app_trans_scr != NULL);
    port_app_sche_load_scr(app_trans_scr);
    port_app_sche_set_user_data(app_trans_scr, (uint32_t) is_manual_anim);

    app_trans_cbk = cbk;
    //is_manual_anim_flag = is_manual_anim;
    app_trans_anim_back = is_back;

#if defined(APP_TRANS_ANIMATION_SCALE_NEXT)

    if (GUI_APP_TRANS_ANIM_NONE != exit_anim_cfg.type)
    {
        b_exit_scale = true;
    }
    else
    {
        b_exit_scale = false;
    }
    if (GUI_APP_TRANS_ANIM_NONE != enter_anim_cfg.type)
    {
        b_enter_scale = true;
    }
    else
    {
        b_enter_scale = false;
    }
    RT_ASSERT((!b_enter_scale && b_exit_scale) || (b_enter_scale && !b_exit_scale));

#elif defined(APP_TRANS_ANIMATION_OVERWRITE)
    b_exit_scale = false;
    b_enter_scale = false;
#else
    b_exit_scale = true;
    b_enter_scale = true;
    exit_buf_index = 0;
    enter_buf_index = 1;
#endif

    exit_anim_obj  = app_trans_animation_obj_create(exit_scr, true, b_exit_scale, exit_buf_index);
    enter_anim_obj = app_trans_animation_obj_create(enter_scr, false, b_enter_scale, enter_buf_index);

    if (is_back)
    {
        app_trans_animation_obj_move_foreground(exit_anim_obj);
    }

    if (RT_EOK == err)
    {
        if (is_manual_anim)
        {
            if (ma_state_started == manual_anim_s)
            {
                trans_anim_log_i("Start manual animation");
                anim_manual_control(manual_animation_start_process);
                ma_state_change(ma_state_active);
            }
            else
            {
                trans_anim_log_i("Manual animation been abort.");
                RT_ASSERT(ma_state_aborted == manual_anim_s);
                anim_manual_free_run(manual_animation_start_process);
            }
        }
        else
        {
            trans_anim_log_i("Start free run animation");
            err = FREE_RUN_START(0, MANUAL_TRAN_ANIM_MAX_PROCESS, TRAN_ANIM_PLAY_TIME);
        }
    }

    trans_anim_log_i("app_trans_animation_setup end, cost=%dms\n", (rt_tick_get() - s_tick) * 1000 / RT_TICK_PER_SECOND);

    if (err != RT_EOK)
    {
        app_trans_anim_clean();
    }

    return err;

}


rt_err_t gui_app_manual_animation_start(uint32_t process)
{
    process = MIN(process, MANUAL_TRAN_ANIM_MAX_PROCESS);

    //if ((NULL == app_trans_scr) && !is_manual_anim_flag)
    if ((ma_state_stopped == manual_anim_s) && (NULL == app_trans_scr))
    {
        rt_err_t err;

        trans_anim_log_i("gui_app_manual_animation_start process=%d",  process);
        manual_animation_start_process = process;
        err = gui_app_manual_goback_anim();

        if (RT_EOK == err)
        {
            ma_state_change(ma_state_started);
        }
        else
        {
            trans_anim_log_i("gui_app_manual_animation_start Failed(%d).", err);
        }
        return RT_EOK;
    }
    else
    {
        trans_anim_log_i("Previous anim not finished yet(%d).", manual_anim_s);
        return RT_ERROR;
    }
}


rt_err_t gui_app_manual_animation_update(uint32_t process)
{
    process = MIN(process, MANUAL_TRAN_ANIM_MAX_PROCESS);

    //if ((app_trans_scr != NULL) && is_manual_anim_flag)
    if (ma_state_active == manual_anim_s)
    {
        manual_animation_start_process = process;
        trans_anim_log_d("gui_app_manual_animation_update process=%d",  process);
        anim_manual_control(process);
    }
    else
    {
        ;//Not entry manual screen, ignore update.
    }

    return RT_EOK;
}


rt_err_t gui_app_manual_animation_stop(uint32_t process)
{
    rt_err_t err;

    process = MIN(process, MANUAL_TRAN_ANIM_MAX_PROCESS);

    //if ((app_trans_scr != NULL) && is_manual_anim_flag && (NULL == anim_var.exec_cb))
    if (ma_state_active == manual_anim_s)
    {
        trans_anim_log_i("gui_app_manual_animation_stop at process=%d",  process);
        err = anim_manual_free_run(process);
        RT_ASSERT(RT_EOK == err);
        ma_state_change(ma_state_freerun);
    }
    else if (ma_state_stopped != manual_anim_s)
    {
        trans_anim_log_i("gui_app_manual_animation Force stop ");
        manual_animation_start_process = process;
        ma_state_change(ma_state_aborted);
    }

    return RT_EOK;
}





void gui_app_trans_anim_init_cfg(gui_app_trans_anim_t *cfg, gui_app_trans_anim_type_t type)
{
    app_trans_anim_init_cfg(cfg, type);
}

static void print_trans_anim_info(const char *prefix, const gui_app_trans_anim_t *cfg)
{
    if (NULL == cfg) return;

    switch (cfg->type)
    {
    case GUI_APP_TRANS_ANIM_PUSH_RIGHT_IN:
    case GUI_APP_TRANS_ANIM_PUSH_RIGHT_OUT:
    case GUI_APP_TRANS_ANIM_PUSH_LEFT_IN:
    case GUI_APP_TRANS_ANIM_PUSH_LEFT_OUT:
        trans_anim_log_d("%s [PUSH_L&R],x[%d -> %d],opa[%d -> %d]", prefix,
                         cfg->cfg.push.x_start, cfg->cfg.push.x_end,
                         cfg->cfg.push.opa_start, cfg->cfg.push.opa_end);
        break;

    case GUI_APP_TRANS_ANIM_ZOOM_IN:
    case GUI_APP_TRANS_ANIM_ZOOM_OUT:
        trans_anim_log_d("%s [ZOOM_INOUT],pi[%d,%d],zoom[%d -> %d],opa[%d -> %d]", prefix,
                         cfg->cfg.zoom.pivot.x, cfg->cfg.zoom.pivot.y,
                         cfg->cfg.zoom.zoom_start, cfg->cfg.zoom.zoom_end,
                         cfg->cfg.zoom.opa_start, cfg->cfg.zoom.opa_end);
        break;

    case GUI_APP_TRANS_ANIM_NONE:
        trans_anim_log_d("%s [NONE]", prefix);
        break;

    case GUI_APP_TRANS_ANIM_ENTER_DEFAULT:
        trans_anim_log_d("%s [ENTER_DEFAULT]", prefix);
        break;

    case GUI_APP_TRANS_ANIM_EXIT_DEFAULT:
        trans_anim_log_d("%s [EXIT_DEFAULT]", prefix);
        break;

    default:
        break;
    }
}


void gui_app_set_enter_trans_anim(gui_app_trans_anim_t *cfg)
{
    subpage_node_t *cur_page = app_schedule_get_this_page();

    if (cur_page)
    {
        if (cfg)
            memcpy(&cur_page->a_group.a_enter, cfg, sizeof(gui_app_trans_anim_t));
        else
            memset(&cur_page->a_group.a_enter, 0, sizeof(gui_app_trans_anim_t));
    }
    else
    {
        RT_ASSERT(RT_FALSE);
    }
}


void gui_app_set_exit_trans_anim(gui_app_trans_anim_t *cfg)
{
    subpage_node_t *cur_page = app_schedule_get_this_page();

    if (cur_page)
    {
        if (cfg)
            memcpy(&cur_page->a_group.a_exit, cfg, sizeof(gui_app_trans_anim_t));
        else
            memset(&cur_page->a_group.a_exit, 0, sizeof(gui_app_trans_anim_t));
    }
    else
    {
        RT_ASSERT(RT_FALSE);
    }
}

void gui_app_set_trans_anim_prio(int8_t up, int8_t down)
{
    subpage_node_t *cur_page = app_schedule_get_this_page();

    if (cur_page)
    {
        cur_page->a_group.prio_up = up;
        cur_page->a_group.prio_down = down;
    }
    else
    {
        RT_ASSERT(RT_FALSE);
    }
}

void app_trans_end_cb_register(app_trans_ex_cb_t callback)
{
    app_trans_ex_cb = callback;
}


rt_err_t app_trans_animation_reset(void)
{
    trans_anim_log_i("trans_animation Force reset ");

    app_trans_anim_clean();
    ma_state_change(ma_state_aborted);
    ma_state_change(ma_state_stopped);


    return RT_EOK;
}



#else  /* APP_TRANS_ANIMATION_NONE */

void app_trans_animation_init(void)
{
    return;
}

void app_trans_anim_init_cfg(gui_app_trans_anim_t *cfg, gui_app_trans_anim_type_t type)
{
    return;
}

void gui_app_trans_anim_init_cfg(gui_app_trans_anim_t *cfg, gui_app_trans_anim_type_t type)
{
    return;
}


void gui_app_set_enter_trans_anim(gui_app_trans_anim_t *cfg)
{
    return;
}

rt_err_t app_trans_anim_abort(void)
{
    return RT_EOK;
}

void gui_app_set_exit_trans_anim(gui_app_trans_anim_t *cfg)
{
    return;
}

void gui_app_set_trans_anim_prio(int8_t up, int8_t down)
{
    return;
}


rt_err_t app_trans_animation_setup(const gui_app_trans_anim_group_t *g_enter,
                                   const gui_app_trans_anim_group_t *g_exit,
                                   const screen_t enter_scr,
                                   const screen_t exit_scr,
                                   app_trans_anim_xcb_t cbk, bool is_back, bool is_manual_anim)
{

    return RT_ENOSYS;

}

rt_err_t gui_app_manual_animation_start(uint32_t process)
{
    return RT_EOK;
}



rt_err_t gui_app_manual_animation_update(uint32_t process)
{
    return RT_EOK;
}



rt_err_t gui_app_manual_animation_stop(uint32_t process)
{
    return RT_EOK;
}

rt_err_t app_trans_animation_reset(void)
{

    return RT_EOK;
}


#endif /* APP_TRANS_ANIMATION_NONE */

