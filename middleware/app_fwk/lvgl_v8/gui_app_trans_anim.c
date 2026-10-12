/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

#ifdef SOLUTION
    #include "global.h"
#endif /* SOLUTION */
#include "gui_app_int.h"
#include "app_mem.h"
#include "lv_gesture.h"

#define IS_SCHE_IDLE()              (LV_GESTURE_SCHE_IDLE == lv_gesture_get_sche_state())
#define COORD_OFT_PROC(val)         ((val > 0) ? (val * GUI_APP_TRANS_PROGRESS_MAX / LV_HOR_RES) : 0)

/**
 * The minimum value for a gesture to trigger a swipe.
 */
#define GESTURE_DRAG_LIMIT          (3)

/**
 * The minimum value for a gesture to goback.
 */
#define GESTURE_GOBACK_THRESHOLD    (30)

static uint32_t switchanim_skip_snapshot = false;
static app_trans_anim_xcb_t g_anim_stop_cb = NULL;
static lv_obj_t *g_switch_anim = NULL;
static gui_app_trans_anim_t g_enter_para;
static gui_app_trans_anim_t g_exit_para;
static lv_switchanim_finish_cb finish_cb = NULL;
#if !defined (APP_TRANS_ANIMATION_NONE)

#include "lvsf_switchanim.h"
#define SWITCH_ANIM_PERIOD 300
void gui_app_gesture_init(void);
void app_trans_animation_init(void)
{
    gui_app_gesture_init();
    lv_switchanim_init();
}

void app_trans_anim_finish_cb(bool is_ok)
{
    g_enter_para.flag = 0;
    g_exit_para.flag = 0;
    TransResult_T  res = is_ok ? TRANS_RES_FINISHED : TRANS_RES_ABORTED;
    if (g_anim_stop_cb)
        g_anim_stop_cb(res);
    if (finish_cb)
        finish_cb(is_ok);
    finish_cb = NULL;
    lv_obj_del(g_switch_anim);
    g_switch_anim = NULL;
    g_anim_stop_cb = NULL;
}

rt_err_t app_trans_anim_abort(void)
{

    if (g_switch_anim)
    {
        TransResult_T  res;

        bool is_manual_anim = (bool)lv_obj_get_user_data(g_switch_anim);

        if (is_manual_anim)
            res = TRANS_RES_ABORTED;
        else
            res = TRANS_RES_FINISHED;//Normal animation can not return 'TRANS_RES_ABORTED'

        lv_switchanim_manual_finish(g_switch_anim, true);
        if (g_anim_stop_cb)
            g_anim_stop_cb(res);
        lv_obj_del(g_switch_anim);
    }
    g_switch_anim = NULL;
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
rt_err_t app_trans_animation_setup(const gui_app_trans_anim_t *g_enter,
                                   const gui_app_trans_anim_t *g_exit,
                                   const screen_t enter_scr,
                                   const screen_t exit_scr,
                                   app_trans_anim_xcb_t cbk, bool is_back, bool is_manual_anim)
{
    //RT_ASSERT(NULL != g_switch_anim);
    if (is_manual_anim && !lv_gesture_is_valid())
    {
        return RT_EEMPTY;
    }
    g_anim_stop_cb = cbk;
    //return RT_EEMPTY;
    lv_switchanim_overwrite(g_enter, &g_enter_para, g_exit, &g_exit_para, is_back);
    if (switchanim_skip_snapshot)
        g_exit_para.flag |= LV_SWITCHANIM_FLAG_SKIP_SNAPHOT;
    if (LV_SWITCHANIM_NONE == g_enter_para.major)
    {
        if (finish_cb) finish_cb(true);
        finish_cb = NULL;
        return RT_EEMPTY;
    }
    if (g_switch_anim) lv_obj_del(g_switch_anim);
    g_switch_anim = lv_switchanim_create(NULL, (lv_obj_t *)enter_scr, (lv_obj_t *)exit_scr, &g_enter_para, &g_exit_para);
    lv_switchanim_set_finish_cb(g_switch_anim, app_trans_anim_finish_cb);
    lv_obj_set_user_data(g_switch_anim, (void *)is_manual_anim);
    if (is_manual_anim)
        lv_switchanim_manual_run(g_switch_anim, 0);
    else
        lv_switchanim_auto_run(g_switch_anim, SWITCH_ANIM_PERIOD, 1, false);
    return RT_EOK;
}

void  app_trans_animation_hook(const screen_t scr, gui_app_msg_type_t msg)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONPAUSE:
    {
#if !defined (APP_TRANS_ANIMATION_NONE)
        uint8_t *p_buf = (uint8_t *)app_anim_buf_alloc(APP_TRANS_ANIM_SNAPSHOT_SIZE, 1);
        if (NULL == p_buf) return;
        switchanim_skip_snapshot = true;
        lv_img_dsc_t img_dsc;
        img_dsc.header.always_zero = 0;
        img_dsc.header.reserved = 0;
        img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
        img_dsc.header.w = APP_TRANS_ANIM_SNAPSHOT_WIDTH;
        img_dsc.header.h = APP_TRANS_ANIM_SNAPSHOT_HEIGHT;
        img_dsc.data = p_buf;
        img_dsc.data_size = APP_TRANS_ANIM_SNAPSHOT_SIZE;

        lv_obj_t *obj = (lv_obj_t *)scr;
        if (APP_TRANS_ANIM_FULL_SCALE == APP_TRANS_ANIM_SNAPSHOT_SCALE)
            lv_snapshot_obj_to_dsc(obj, &obj->coords, &img_dsc);
        else
            lv_snapshot_obj_to_dsc_scale(obj, &obj->coords, &img_dsc);
#endif
        break;
    }

    default:
        break;
    }
}

rt_err_t gui_app_manual_animation_start(uint32_t process)
{
    //rt_kprintf("animation start\n");
    if (g_switch_anim) return RT_EOK;
    gui_app_manual_goback_anim();
    return RT_EOK;
}

rt_err_t gui_app_manual_animation_update(uint32_t process)
{
    //rt_kprintf("animation update %d \n", process);
    if (NULL == g_switch_anim) return RT_EOK;
    lv_switchanim_manual_run(g_switch_anim, process);
    return RT_EOK;
}

rt_err_t gui_app_manual_animation_stop(uint32_t process)
{
    if (NULL == g_switch_anim) return RT_EOK;
    //rt_kprintf("animation stop %d\n", process);
    lv_switchanim_auto_run(g_switch_anim, SWITCH_ANIM_PERIOD, process, process < (GUI_APP_TRANS_PROGRESS_MAX / 2));
    return RT_EOK;
}

void gui_app_set_enter_trans_anim(gui_app_trans_anim_t *cfg)
{
    subpage_node_t *cur_page = app_schedule_get_this_page();

    if (cur_page)
    {
        if (cfg)
            memcpy(&cur_page->a_enter, cfg, sizeof(gui_app_trans_anim_t));
        else
            memset(&cur_page->a_enter, 0, sizeof(gui_app_trans_anim_t));
    }
    else
    {
        RT_ASSERT(RT_FALSE);
    }
}

void app_trand_end_cb_exec(void)
{
    if (finish_cb && !g_switch_anim)
    {
        finish_cb(false);
        finish_cb = NULL;
    }
}

#else
void app_trans_animation_init(void)
{

}

rt_err_t app_trans_anim_abort(void)
{
    return RT_EOK;
}

rt_err_t app_trans_animation_setup(const gui_app_trans_anim_t *g_enter,
                                   const gui_app_trans_anim_t *g_exit,
                                   const screen_t enter_scr,
                                   const screen_t exit_scr,
                                   app_trans_anim_xcb_t cbk, bool is_back, bool is_manual_anim)
{
    if (finish_cb) finish_cb(true);
    finish_cb = NULL;
    return RT_EEMPTY;
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

void app_trand_end_cb_exec(void)
{

}

#endif

void gui_app_set_finsh_cb(lv_switchanim_finish_cb cb)
{
    finish_cb = cb;
}

void gui_app_init_anim(gui_app_trans_anim_t *anim)
{
    anim->major = LV_SWITCHANIM_DEFAULT;
    anim->minor = LV_SWITCHANIM_MINOR_DEFAULT;
    anim->minor_aux = LV_SWITCHANIM_MINOR_DEFAULT;
    anim->prior = LV_SWITCHANIM_PRIOR_DEFAULT;
    anim->user_data = 0;
}

rt_err_t app_trans_animation_reset(void)
{
    if (g_switch_anim)
    {
        lv_switchanim_manual_finish(g_switch_anim, true);
        lv_obj_del(g_switch_anim);
        g_switch_anim = NULL;
    }
    return RT_EOK;
}

void gui_app_set_anim_user_data(void *enter_user, void *exit_user)
{
    subpage_node_t *cur_page = app_schedule_get_this_page();
    cur_page->a_enter.user_data = enter_user;
    cur_page->a_exit.user_data = exit_user;
}

void gui_app_set_anim_prior(int16_t enter_prior, int16_t exit_prior)
{
    subpage_node_t *cur_page = app_schedule_get_this_page();
    cur_page->a_enter.prior = enter_prior;
    cur_page->a_exit.prior = exit_prior;
}

void gui_app_set_enter_anim_type(uint16_t major, uint16_t minor, int16_t minor_aux)
{
    subpage_node_t *cur_page = app_schedule_get_this_page();
    cur_page->a_enter.major = major;
    cur_page->a_enter.minor = minor;
    cur_page->a_enter.minor_aux = minor_aux;
}

void gui_app_set_exit_anim_type(uint16_t major, uint16_t minor, int16_t minor_aux)
{
    subpage_node_t *cur_page = app_schedule_get_this_page();
    cur_page->a_exit.major = major;
    cur_page->a_exit.minor = minor;
    cur_page->a_exit.minor_aux = minor_aux;
}

void gui_app_set_exit_trans_anim(gui_app_trans_anim_t *cfg)
{
    subpage_node_t *cur_page = app_schedule_get_this_page();

    if (cur_page)
    {
        if (cfg)
            memcpy(&cur_page->a_exit, cfg, sizeof(gui_app_trans_anim_t));
        else
            memset(&cur_page->a_exit, 0, sizeof(gui_app_trans_anim_t));
    }
    else
    {
        RT_ASSERT(RT_FALSE);
    }
}

void gui_app_close_anim()
{
    subpage_node_t *cur_page = app_schedule_get_this_page();
    cur_page->a_exit.major = LV_SWITCHANIM_NONE;
    cur_page->a_exit.minor = LV_SWITCHANIM_NONE;
    cur_page->a_exit.minor_aux = LV_SWITCHANIM_NONE;
    cur_page->a_exit.prior = LV_SWITCHANIM_PRIOR_HIGH;
    cur_page->a_enter.major = LV_SWITCHANIM_NONE;
    cur_page->a_enter.minor = LV_SWITCHANIM_NONE;
    cur_page->a_enter.minor_aux = LV_SWITCHANIM_NONE;
    cur_page->a_enter.prior = LV_SWITCHANIM_PRIOR_HIGH;
}

/**
 * Manual animation, activated by sliding gestures within the activation area.
 */
static int32_t gui_app_gesture_event_cb_manual_anim(lv_gesture_event_t code, int32_t pos)
{
    switch (code)
    {
    case LV_GESTURE_ANIM_START:
        if (IS_SCHE_IDLE())
        {
            lv_indev_wait_release(lv_indev_get_act());
            return gui_app_manual_animation_start(COORD_OFT_PROC(pos));
        }
        else
            return RT_ERROR;
    case LV_GESTURE_ANIM_PRESSING:
        return gui_app_manual_animation_update(COORD_OFT_PROC(pos));
    case LV_GESTURE_ANIM_END:
        return gui_app_manual_animation_stop(COORD_OFT_PROC(pos));
    case LV_GESTURE_GOBACK:
        if (pos > (LV_HOR_RES_MAX >> 1))
        {
            gui_app_goback();
        }
    default:
        return RT_ERROR;
    }
    return RT_EOK;
}

void gui_app_gesture_init(void)
{
    /** Slide from left to right, manual animation and close swipe right anywhere to goback**/
    lv_gesture_init(gui_app_gesture_event_cb_manual_anim);
    lv_gesture_set_area_line(LV_HOR_RES / 4);
    lv_gesture_set_goback(false);
    lv_gesture_enable();

#if !defined (APP_TRANS_ANIMATION_NONE) && (defined(LCD_FB_USING_TWO_COMPRESSED) || defined(LCD_FB_USING_ONE_COMPRESSED))
    app_schedule_set_anim_hook(app_trans_animation_hook);
#endif
}

void gui_app_gesture_set_parem(lv_coord_t left_area, uint8_t goback_en)
{
    if (!IS_SCHE_IDLE())
    {
        //rt_kprintf("current state is not idle state,please set laster.\n");
        return;
    }
    lv_gesture_set_area_line(left_area);
    lv_gesture_set_goback(goback_en);
}
