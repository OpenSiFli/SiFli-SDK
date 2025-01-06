/*********************
 *      INCLUDES
 *********************/
#include "gui_app_int.h"
#include "bf0_lib.h"

#ifdef PKG_USING_LITTLEVGL2RTT
screen_t port_app_sche_create_scr(void)
{
#if defined(DISABLE_LVGL_V8)&&defined(DISABLE_LVGL_V9)
    return (screen_t) lv_obj_create(NULL, NULL);
#else
    return (screen_t) lv_obj_create(NULL);
#endif

}

void port_app_sche_del_scr(screen_t scr)
{
    lv_obj_del((lv_obj_t *) scr);
}



void port_app_sche_load_scr(screen_t scr)
{
    lv_scr_load((lv_obj_t *)scr);

#if defined(DISABLE_LVGL_V8)&&defined(DISABLE_LVGL_V9)
    lv_obj_set_scrollbar_mode((lv_obj_t *)scr, LV_SCROLLBAR_MODE_OFF);
#endif /* !DISABLE_LVGL_V8 */
}


screen_t port_app_sche_get_act_scr(void)
{
    return (screen_t) lv_scr_act();

}

void port_app_sche_set_user_data(screen_t scr, uint32_t v)
{
#if defined(DISABLE_LVGL_V8)&&defined(DISABLE_LVGL_V9)
    lv_obj_set_user_data((lv_obj_t *)scr, (lv_obj_user_data_t) v);
#else
    lv_obj_set_user_data((lv_obj_t *)scr, (void *) v);
#endif /* DISABLE_LVGL_V8 */
}

uint32_t port_app_sche_get_user_data(screen_t scr)
{
    return (uint32_t) lv_obj_get_user_data((lv_obj_t *)scr);
}

void port_app_sche_reset_indev(screen_t scr)
{
#if defined(DISABLE_LVGL_V8)&&defined(DISABLE_LVGL_V9)
    lv_obj_reset_indev((lv_obj_t *)scr);
#else
    lv_indev_reset(NULL, NULL);
#endif /* DISABLE_LVGL_V8 */
}




task_t port_app_sche_task_create(task_handler_t task_handler, uint32_t period, void *user_data)
{
    uint32_t lv_period;

    if (period & GUI_LIB_REFR_PERIOD_FLAG)
    {
        lv_period = LV_DISP_DEF_REFR_PERIOD * (period & ~GUI_LIB_REFR_PERIOD_FLAG);
    }
    else
    {
        lv_period = period;
    }

#if defined(DISABLE_LVGL_V8)&&defined(DISABLE_LVGL_V9)
    return (task_t) lv_task_create((lv_task_cb_t)
                                   task_handler, lv_period, LV_TASK_PRIO_MID, (void *)user_data);

#else
    return (task_t) lv_timer_create((lv_timer_cb_t)task_handler, lv_period, user_data);
#endif /* DISABLE_LVGL_V8 */
}


void port_app_sche_task_del(task_t task_handler)
{
#if defined(DISABLE_LVGL_V8)&&defined(DISABLE_LVGL_V9)
    lv_task_del((lv_task_t *)task_handler);
#else
    lv_timer_del((lv_timer_t *)task_handler);
#endif /* DISABLE_LVGL_V8 */
}

void port_app_sche_enable_indev(bool enable, bool expect_tp)
{
    lv_indev_t *i = lv_indev_get_next(NULL);
    while (i)
    {
#ifdef DISABLE_LVGL_V9
        if ((i->driver->type != LV_INDEV_TYPE_POINTER) || (!expect_tp))
        {
            if (!enable)
            {
                i->proc.reset_query = 1;
            }
            i->proc.disabled = enable ? 0 : 1;
        }
#else
        if ((lv_indev_get_type(i) != LV_INDEV_TYPE_POINTER) || (!expect_tp))
        {
            if (!enable)
            {
                lv_indev_reset(i, NULL);
            }
            lv_indev_enable(i, enable);
        }
#endif
        i = lv_indev_get_next(i);
    }

}

#endif /* PKG_USING_LITTLEVGL2RTT */

