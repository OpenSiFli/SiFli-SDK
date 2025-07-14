#include <rtthread.h>
#include "lvgl.h"
#include "gui_app_int.h"
#include "app_schedule_port.h"

#define DBG_TAG           "APP.ANI"
#define DBG_LVL          DBG_INFO //DBG_LOG //
#include "rtdbg.h"

#ifndef _MSC_VER
    #include "drv_ext_dma.h"
#endif


#define trans_anim_log_i LOG_I
#define trans_anim_log_d LOG_D

typedef int16_t lv_anim_value_t;

#if !defined (APP_TRANS_ANIMATION_NONE)

static lv_anim_t anim_var;

static lv_timer_t *async_cb_task = NULL;


static gui_anim_free_run_cb free_run_done;
static gui_anim_exe_cb      anim_exe_cb;
extern void *app_anim_buf_alloc(size_t nbytes, uint8_t index);
extern void *app_anim_buf_free(void *ptr);

extern void *get_disp_buf(uint32_t size);

/**************            Private functions           **************/

static void protect_scr_event_handler(lv_event_t *e)
{
    lv_hit_test_info_t *info = lv_event_get_param(e);
    info->res = false;
    e->stop_processing = 0;
}

static void wait_refr_task_cb(lv_timer_t *task)
{
    if (1 == task->repeat_count) //The task will be deleted after this function return.
    {
        TransResult_T res = (TransResult_T)(uint32_t)task->user_data;
        trans_anim_log_i("wait_refr_task_cb %d\n", res);
        free_run_done(res);
        free_run_done = NULL;
    }
}

static void free_run_ready(lv_anim_t *a)
{
    TransResult_T  res = TRANS_RES_FINISHED;
    //Manual anim: stay on current page
    RT_ASSERT(a);
    if (a) if (0 == a->end_value) res = TRANS_RES_ABORTED;


    trans_anim_log_i("free_run_ready %d \n", res);

    //Delay 30 ms to clean&exit trans-anim, to make sure the last frame of trans-anim is displayed.
    async_cb_task = lv_timer_create(wait_refr_task_cb, 2 * LV_DISP_DEF_REFR_PERIOD, (void *)res);
    if (async_cb_task != NULL)
    {
        lv_timer_set_repeat_count(async_cb_task, 2);
    }
    else
    {
        //Error occurs, clean&exit trans-anim directly.
        free_run_done(res);
        free_run_done = NULL;
    }

}


static void free_run_exec(void *var, lv_anim_value_t process)
{
    trans_anim_log_d("free_run_exec %d \n", process);

    if (anim_exe_cb) anim_exe_cb((gui_anim_value_t) process);
}

static void app_trans_anim_move_all_childs(const lv_obj_t *src_parent, const lv_obj_t *dst_parent)
{
    while (lv_obj_get_child_cnt(src_parent) > 0)
    {
        lv_obj_set_parent(lv_obj_get_child(src_parent, 0), (lv_obj_t *) dst_parent);
    }
}

static lv_img_dsc_t *app_trans_create_img_buf(const uint8_t *buf)
{
    lv_img_dsc_t *img_desc;
    img_desc = lv_malloc(sizeof(lv_img_dsc_t));
    RT_ASSERT(img_desc != NULL);

    memset(img_desc, 0, sizeof(lv_img_dsc_t));
    img_desc->header.cf = LV_IMG_CF_TRUE_COLOR;
    img_desc->header.w = APP_TRANS_ANIM_SNAPSHOT_WIDTH;
    img_desc->header.h = APP_TRANS_ANIM_SNAPSHOT_HEIGHT;
    img_desc->data = buf;
    img_desc->data_size = img_desc->header.w * img_desc->header.h * (APP_SNAPSHOT_COLOR_DEPTH >> 3);

    return img_desc;
}


/**************     Implement gui_app_trans_anim.c's APIs  **************/

gui_anim_obj_t app_trans_animation_obj_create(const screen_t scr, bool is_cur_screen, bool b_scale, int buf_index)
{
    lv_obj_t *ret_anim_obj;

    lv_coord_t hor_res = lv_disp_get_hor_res(NULL);
    lv_coord_t ver_res = lv_disp_get_ver_res(NULL);
    lv_area_t mask;
    lv_disp_t *disp = lv_refr_get_disp_refreshing();//v9
    if (NULL == disp) disp = lv_disp_get_default();

#ifdef DISABLE_LVGL_V9
    lv_draw_ctx_t *draw_ctx = disp->driver->draw_ctx;
    lv_disp_draw_buf_t *vdb = lv_disp_get_draw_buf(disp);
#else
#define vdb disp
#endif
    mask.x1 = 0;
    mask.x2 = hor_res - 1;
    mask.y1 = 0;
    mask.y2 = ver_res - 1;
    lv_img_dsc_t *screen_snapshot = NULL;

    if (b_scale)
    {
        char *trans_anim_buf = NULL;
        lv_res_t res = LV_RES_INV;

        trans_anim_buf = (char *)app_anim_buf_alloc(APP_TRANS_ANIM_SNAPSHOT_SIZE, buf_index);

        RT_ASSERT(trans_anim_buf);

        screen_snapshot = app_trans_create_img_buf((uint8_t *)trans_anim_buf);
        lv_img_dsc_t img_scr;
        img_scr.header.always_zero = 0;
        img_scr.header.w = hor_res;
        img_scr.header.h = ver_res;
        img_scr.data_size  = (LV_COLOR_DEPTH * hor_res * ver_res) / 8;
        img_scr.header.cf = LV_IMG_CF_TRUE_COLOR;
        img_scr.data = (uint8_t *)get_disp_buf((LV_COLOR_DEPTH * hor_res * ver_res) / 8);

#ifdef SCALE_TRANS_ANIM_USE_DIVIDED_FB
        lv_img_dsc_t *tmp_img = lv_img_buf_alloc(hor_res, (ver_res / SCALE_TRANS_ANIM_USE_DIVIDED_FB) + SCALE_TRANS_ANIM_USE_DIVIDED_FB,
                                disp->driver.buffer->cf);
#endif /* SCALE_TRANS_ANIM_USE_DIVIDED_FB */

        /*Dump old&new screen to PSRAM buffer*/

        trans_anim_log_d("disp:%x, drv:%x, buf:%x, act:%x, buf1:%x buf2:%x\n",
                         lv_disp_get_default(),
                         lv_disp_get_default()->driver,
                         lv_disp_get_default()->driver.buffer,
                         (uint32_t)lv_disp_get_default()->driver.buffer->buf_act,
                         (uint32_t)lv_disp_get_default()->driver.buffer->buf1,
                         (uint32_t)lv_disp_get_default()->driver.buffer->buf2
                        );


        trans_anim_log_i("buf = %x", trans_anim_buf);
#ifdef DISABLE_LVGL_V9
        if (draw_ctx->wait_for_finish) draw_ctx->wait_for_finish(draw_ctx);
#endif
        if (is_cur_screen)
        {
            //copy old screen buffer to buf_a
            uint32_t start = lv_tick_get();
#if (APP_TRANS_ANIM_FULL_SCALE == APP_TRANS_ANIM_SNAPSHOT_SCALE && LV_COLOR_DEPTH == APP_SNAPSHOT_COLOR_DEPTH)
            //app_trans_dump_buf_act((char *)screen_snapshot->data);
#ifdef DISABLE_LVGL_V9
            res = lv_refr_dump_buf_to_img_now(screen_snapshot);
            if (LV_RES_OK != res)
            {
                lv_snapshot_take_to_buf((lv_obj_t *)scr,
                                        screen_snapshot->header.cf,
                                        screen_snapshot,
                                        (uint8_t *)screen_snapshot->data,
                                        screen_snapshot->data_size);
            }
#endif
#elif !defined(LV_FRAME_BUF_SCHEME_2)
            //directly use frame buffer.
            //app_trans_dump_buf_act_with_zoom((char *)screen_snapshot->data);
            res = lv_refr_dump_buf_to_img_now(screen_snapshot);
#elif defined(SCALE_TRANS_ANIM_USE_DIVIDED_FB)
            lv_refr_area_to_zoom_img_now((lv_obj_t *)scr, &mask, tmp_img, screen_snapshot);
#else
            if (disp->driver.wait_cb) disp->driver.wait_cb(&disp->driver);

            //for scheme2(dual buffer but not full screen), need set buf_act to buf1, re-draw and snapshot
            vdb->buf_act = vdb->buf1;
            lv_refr_area_to_img_now((lv_obj_t *)scr, &mask, &img_scr);
            if (disp && disp->driver.gpu_wait_cb) disp->driver.gpu_wait_cb(&disp->driver);
            //app_trans_dump_buf_act_with_zoom((char *)screen_snapshot->data);
            res = lv_refr_dump_buf_to_img_now(screen_snapshot);
            RT_ASSERT(LV_RES_OK == res);
#endif
            trans_anim_log_d("app_trans_animation_setup: buf_act %d \n", lv_tick_get() - start);
        }


        if (LV_RES_OK != res) //draw new screen to buf_b
        {
            /* Create a dummy display to fool the lv_draw function.
             * It will think it draws to real screen. */
            uint32_t start = lv_tick_get();

#if (APP_TRANS_ANIM_FULL_SCALE == APP_TRANS_ANIM_SNAPSHOT_SCALE && LV_COLOR_DEPTH == APP_SNAPSHOT_COLOR_DEPTH)
            //lv_refr_area_to_img_now((lv_obj_t *)scr, &mask, screen_snapshot);
            lv_snapshot_take_to_buf((lv_obj_t *)scr,
                                    screen_snapshot->header.cf,
                                    screen_snapshot,
                                    (uint8_t *)screen_snapshot->data,
                                    screen_snapshot->data_size);
#elif defined(SCALE_TRANS_ANIM_USE_DIVIDED_FB)
            lv_refr_area_to_zoom_img_now((lv_obj_t *)scr, &mask, tmp_img, screen_snapshot);
#else
            if (disp->driver.wait_cb) disp->driver.wait_cb(&disp->driver);

#if !defined(LV_FRAME_BUF_SCHEME_2)
            lv_refr_area_to_img_now((lv_obj_t *)scr, &mask, &img_scr);
#else
            //for scheme2(dual buffer but not full screen), need set buf_act to buf1, re-draw and snapshot
            vdb->buf_act = vdb->buf1;
            lv_refr_area_to_img_now(lv_scr_act(), &mask, &img_scr);
#endif
            if (disp->driver.gpu_wait_cb) disp->driver.gpu_wait_cb(&disp->driver);
            //app_trans_dump_buf_act_with_zoom((char *)screen_snapshot->data);
            res = lv_refr_dump_buf_to_img_now(screen_snapshot);
            RT_ASSERT(LV_RES_OK == res);
#endif
// #ifdef DISABLE_LVGL_V9
//             if (draw_ctx->wait_for_finish) draw_ctx->wait_for_finish(draw_ctx);
// #endif
            trans_anim_log_d("app_trans_animation_setup: refr_now %d \n", lv_tick_get() - start);
        }

#ifdef SCALE_TRANS_ANIM_USE_DIVIDED_FB
        if (tmp_img)
        {
            lv_img_buf_free(tmp_img);
            tmp_img = NULL;
        }
#endif /* SCALE_TRANS_ANIM_USE_DIVIDED_FB */

        ret_anim_obj = lv_img_create(lv_scr_act());
        RT_ASSERT(ret_anim_obj != NULL);


        lv_img_set_src(ret_anim_obj, screen_snapshot);
        lv_obj_align(ret_anim_obj, LV_ALIGN_CENTER, 0, 0);
#if APP_TRANS_ANIM_FULL_SCALE != APP_TRANS_ANIM_SNAPSHOT_SCALE
        lv_img_set_zoom(ret_anim_obj, APP_TRANS_ANIM_ZOOM_NONE);
#endif


        //lv_obj_set_style_local_opa_scale(ret_anim_obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
        //lv_obj_set_style_local_transform_zoom(ret_anim_obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_IMG_ZOOM_NONE);
        lv_obj_set_style_img_opa(ret_anim_obj, LV_OPA_COVER, LV_PART_MAIN);
    }
    else
    {
        /* Copy all childrens of app's screens to new parent which belongs to 'app_trans_scr' */

        ret_anim_obj = lv_obj_create(lv_scr_act());
        RT_ASSERT(ret_anim_obj != NULL);

        //Use one signal handler, as they are same type
        lv_obj_add_event_cb(ret_anim_obj, protect_scr_event_handler, LV_EVENT_HIT_TEST, 0);
        lv_obj_add_flag(ret_anim_obj, LV_OBJ_FLAG_ADV_HITTEST);
        lv_obj_set_scrollbar_mode(ret_anim_obj, LV_SCROLLBAR_MODE_OFF);

        lv_obj_set_size(ret_anim_obj, hor_res, ver_res);


        //Move all enter_scr child to enter_anim_obj
        lv_obj_set_user_data(ret_anim_obj, scr);
        if (scr) app_trans_anim_move_all_childs((const lv_obj_t *)scr, ret_anim_obj);

        lv_obj_set_style_opa(ret_anim_obj, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_pad_all(ret_anim_obj, 0, LV_PART_MAIN);
        //lv_obj_set_style_local_transform_zoom(ret_anim_obj, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_IMG_ZOOM_NONE);
    }

    trans_anim_log_i("app_trans_animation_obj_create = %x", ret_anim_obj);

    return (gui_anim_obj_t) ret_anim_obj;
}


void app_trans_animation_obj_destroy(gui_anim_obj_t obj)
{
    trans_anim_log_i("app_trans_animation_obj_destroy = %x", obj);

    if (lv_obj_check_type(obj, &lv_img_class))
    {
        const lv_img_dsc_t *img_desc = lv_img_get_src((lv_obj_t *)obj);
        if (img_desc)
        {
            trans_anim_log_d("Free image data buffer, %p\n", img_desc);
            lv_img_cache_invalidate_src(img_desc);
            app_anim_buf_free((void *)img_desc->data);
            lv_mem_free((void *)img_desc);
            //lv_img_set_src((lv_obj_t *)obj, NULL);
        }
    }
    else
    {
        const lv_obj_t *old_scr = lv_obj_get_user_data((lv_obj_t *)obj);
        app_trans_anim_move_all_childs((const lv_obj_t *)obj, old_scr);
    }
}

void app_trans_animation_obj_move_foreground(gui_anim_obj_t obj)
{
    lv_obj_move_foreground((void *)obj);
}

void app_trans_anim_set_opa_scale(gui_anim_obj_t var, gui_anim_value_t opa_scale)
{
    trans_anim_log_d("app_trans_anim_set_opa_scale %x, %d\n", var, opa_scale);
    //lv_obj_set_style_local_image_opa((lv_obj_t *)var, LV_OBJ_PART_MAIN, opa_scale);
    //lv_obj_set_style_local_image_opa((lv_obj_t *)var, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, opa_scale);
    lv_obj_set_style_img_opa((lv_obj_t *)var, opa_scale, LV_PART_MAIN);

}

void app_trans_anim_set_zoom(gui_anim_obj_t var, gui_anim_value_t zoom)
{
    trans_anim_log_d("app_trans_anim_set_zoom %p, %d\n", var, zoom);
    lv_img_set_zoom((lv_obj_t *)var, zoom);
}

void app_trans_anim_set_x(gui_anim_obj_t var, gui_anim_value_t x)
{
    trans_anim_log_d("app_trans_anim_set_x %x, %d\n", var, x);
    lv_obj_set_x((lv_obj_t *)var, x);
}

void app_trans_anim_set_pivot(gui_anim_obj_t var, const gui_point_t *p)
{
    trans_anim_log_d("app_trans_anim_set_pivot %x, %d,%d\n", var, p->x, p->y);

    lv_img_set_pivot((lv_obj_t *)var, (lv_coord_t)p->x, (lv_coord_t)p->y);
}


rt_err_t app_trans_anim_free_run_start(gui_anim_value_t start,
                                       gui_anim_value_t end,
                                       uint32_t duration,
                                       gui_anim_exe_cb anim_process,
                                       gui_anim_free_run_cb done_cb)
{
    free_run_done = done_cb;
    anim_exe_cb   = anim_process;

    lv_anim_init(&anim_var);
    lv_anim_set_var(&anim_var, &anim_var);
    lv_anim_set_time(&anim_var, duration);
    lv_anim_set_exec_cb(&anim_var, (lv_anim_exec_xcb_t) free_run_exec);
    lv_anim_set_values(&anim_var, start, end);
    lv_anim_set_ready_cb(&anim_var, free_run_ready);
    lv_anim_set_early_apply(&anim_var, true);

    if (0)
    {
        // lv_anim_set_path_cb(&anim_var, lv_anim_path_overshoot);
    }
    trans_anim_log_d("app_trans_anim_free_run_start start=%d, end=%d, duration=%d\n", start, end, duration);

    lv_anim_start(&anim_var);

    return RT_EOK;
}

rt_err_t app_trans_anim_free_run_clean(void)
{
    trans_anim_log_i("app_trans_anim_free_run_clean\n");

    free_run_done = NULL;
    anim_exe_cb   = NULL;
    lv_anim_del(NULL, (lv_anim_exec_xcb_t) free_run_exec);
    lv_anim_init(&anim_var);

    if (async_cb_task)
    {
        lv_timer_del(async_cb_task);
        async_cb_task = NULL;
    }

    return RT_EOK;
}


void app_trans_animation_init(void)
{


}


void app_trans_anim_init_cfg(gui_app_trans_anim_t *cfg, gui_app_trans_anim_type_t type)
{
    memset(cfg, 0, sizeof(gui_app_trans_anim_t));
    cfg->type = type;

    switch (type)
    {
    case GUI_APP_TRANS_ANIM_PUSH_RIGHT_IN:
        cfg->cfg.push.opa_start = LV_OPA_20;
        cfg->cfg.push.opa_end = LV_OPA_COVER;
        cfg->cfg.push.x_start = LV_HOR_RES;
#if !defined(APP_TRANS_ANIMATION_OVERWRITE) && APP_TRANS_ANIM_FULL_SCALE != APP_TRANS_ANIM_SNAPSHOT_SCALE
        cfg->cfg.push.x_end = (LV_HOR_RES - (APP_TRANS_ANIM_SNAPSHOT_WIDTH)) >> 1;
#else
        cfg->cfg.push.x_end = 0;
#endif
        break;
    case GUI_APP_TRANS_ANIM_PUSH_RIGHT_OUT:
        cfg->cfg.push.opa_start = LV_OPA_COVER;
        cfg->cfg.push.opa_end = LV_OPA_20;
#if !defined(APP_TRANS_ANIMATION_OVERWRITE) && APP_TRANS_ANIM_FULL_SCALE != APP_TRANS_ANIM_SNAPSHOT_SCALE
        cfg->cfg.push.x_start = (LV_HOR_RES - (APP_TRANS_ANIM_SNAPSHOT_WIDTH)) >> 1;
#else
        cfg->cfg.push.x_start = 0;
#endif
        cfg->cfg.push.x_end = LV_HOR_RES;
        break;
    case GUI_APP_TRANS_ANIM_PUSH_LEFT_IN:
        cfg->cfg.push.opa_start = LV_OPA_20;
        cfg->cfg.push.opa_end = LV_OPA_COVER;
        cfg->cfg.push.x_start = -LV_HOR_RES;
        cfg->cfg.push.x_end = 0;
        break;
    case GUI_APP_TRANS_ANIM_PUSH_LEFT_OUT:
        cfg->cfg.push.opa_start = LV_OPA_COVER;
        cfg->cfg.push.opa_end = LV_OPA_20;
        cfg->cfg.push.x_start = 0;
        cfg->cfg.push.x_end = -LV_HOR_RES;
        break;
    case GUI_APP_TRANS_ANIM_ZOOM_IN:
        cfg->cfg.zoom.pivot.x = LV_HOR_RES >> 1;
        cfg->cfg.zoom.pivot.y = LV_VER_RES >> 1;
        cfg->cfg.zoom.zoom_start = 0;
        cfg->cfg.zoom.zoom_end = APP_TRANS_ANIM_ZOOM_NONE;
        cfg->cfg.zoom.opa_start = LV_OPA_0;
        cfg->cfg.zoom.opa_end = LV_OPA_COVER;
        break;
    case GUI_APP_TRANS_ANIM_ZOOM_OUT:
        cfg->cfg.zoom.pivot.x = LV_HOR_RES >> 1;
        cfg->cfg.zoom.pivot.y = LV_VER_RES >> 1;
        cfg->cfg.zoom.zoom_start = APP_TRANS_ANIM_ZOOM_NONE;
        cfg->cfg.zoom.zoom_end = 0;
        cfg->cfg.zoom.opa_start = LV_OPA_COVER;
        cfg->cfg.zoom.opa_end = LV_OPA_0;
        break;

    default:
        break;
    }
}

#endif /* !APP_TRANS_ANIMATION_NONE*/

