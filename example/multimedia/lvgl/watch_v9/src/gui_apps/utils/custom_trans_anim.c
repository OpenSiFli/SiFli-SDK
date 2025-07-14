/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "custom_trans_anim.h"


#ifdef APP_TRANS_ANIMATION_SCALE
static void enter_or_exit_anim0(gui_anim_obj_t img, void *user_data, uint32_t flag, gui_anim_value_t process)
{
#if  LV_USE_IMG
    if (!lv_obj_check_type(img, &lv_image_class)) return;
#endif

    int16_t angle;
    lv_opa_t opa_v = 255 - (255 * process / MANUAL_TRAN_ANIM_MAX_PROCESS);

    if (0 == (flag & FLAG_TRANS_ANIM_REVERSE))
    {
        //Counter clock wise [3600 ~ 3000]
        angle = 3600 - (600 * process / MANUAL_TRAN_ANIM_MAX_PROCESS);
    }
    else
    {
        //Clock wise [3600 ~ 3660]
        angle = 3600 + (600 * process / MANUAL_TRAN_ANIM_MAX_PROCESS);
    }

    if ((0 == (flag & (FLAG_TRANS_ANIM_FG | FLAG_TRANS_ANIM_REVERSE)))
            || ((FLAG_TRANS_ANIM_FG | FLAG_TRANS_ANIM_REVERSE) == (flag & (FLAG_TRANS_ANIM_FG | FLAG_TRANS_ANIM_REVERSE))))
    {
        ; //Current visble screen on framebuffer
    }
    else
    {
        opa_v = 255 - opa_v;
        //Target screen
        if (flag & FLAG_TRANS_ANIM_REVERSE)
            angle -= 600; //At left of current screen
        else
            angle += 600; //At right of current screen
    }

    lv_img_set_angle(img, angle);
    lv_img_set_pivot(img, lv_obj_get_width(img) >> 1,
                     lv_obj_get_width(img) + lv_obj_get_height(img) - 1);

    lv_obj_set_style_img_opa(img, opa_v, LV_PART_MAIN);


    //rt_kprintf("enter_anim obj %x, flag=%d, process=%d \n", img, flag, process);
}

static void enter_anim1(gui_anim_obj_t img, void *user_data, uint32_t flag, gui_anim_value_t process)
{
#if  LV_USE_IMG
    if (!lv_obj_check_type(img, &lv_image_class)) return;
#endif

    if (flag & FLAG_TRANS_ANIM_FG)
    {
        int16_t angle  = 900 * (MANUAL_TRAN_ANIM_MAX_PROCESS - process) / MANUAL_TRAN_ANIM_MAX_PROCESS;
        lv_opa_t opa_v = 255 * process / MANUAL_TRAN_ANIM_MAX_PROCESS;

        if (flag & FLAG_TRANS_ANIM_REVERSE)
        {
            angle = 900 - angle;
            opa_v = 255 - opa_v;
        }

        //lv_img_set_angle(img, angle);
        //lv_img_set_pivot(img, lv_obj_get_width(img) - 1, lv_obj_get_height(img) - 1);

        lv_obj_set_style_img_opa(img, opa_v, LV_PART_MAIN);
    }


    //rt_kprintf("enter_anim obj %x, flag=%d, process=%d \n", img, flag, process);
}

static void exit_anim1(gui_anim_obj_t img, void *user_data, uint32_t flag, gui_anim_value_t process)
{
#if  LV_USE_IMG
    if (!lv_obj_check_type(img, &lv_image_class)) return;
#endif

    if (flag & FLAG_TRANS_ANIM_FG)
    {
        int16_t angle  = 900 * (MANUAL_TRAN_ANIM_MAX_PROCESS - process) / MANUAL_TRAN_ANIM_MAX_PROCESS;
        lv_opa_t opa_v = 255 * process / MANUAL_TRAN_ANIM_MAX_PROCESS;

        if (flag & FLAG_TRANS_ANIM_REVERSE)
        {
            angle = 900 - angle;
            opa_v = 255 - opa_v;
        }

        //lv_img_set_angle(img, angle);
        //lv_img_set_pivot(img, lv_obj_get_width(img) - 1, lv_obj_get_height(img) - 1);

        lv_obj_set_style_img_opa(img, opa_v, LV_PART_MAIN);
    }

    //rt_kprintf("exit_anim obj %x, flag=%d, process=%d \n", img, flag, process);
}

#if (1 == LV_USE_GPU)&&defined(LV_USE_GPU)
static void draw_img_anim2(lv_event_t *e)
{

    // extern void img_rotate_opa_frac2(lv_img_dsc_t *dest, lv_img_dsc_t *src, int16_t angle, uint32_t pitch_x, uint32_t pitch_y,
    //                                  const lv_area_t *src_coords, const lv_area_t *dst_coords,
    //                                  const lv_area_t *output_coords, lv_point_t *pivot, lv_opa_t opa, lv_color_t ax_color,
    //                                  uint16_t src_off_x_frac, uint16_t src_off_y_frac, uint8_t use_dest_as_bg,
    //                                  lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords);

    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    lv_image_t *img = (lv_image_t *)obj;

    if (code == LV_EVENT_DRAW_POST)
    {
        e->stop_processing  = 1;
    }
    else if (code == LV_EVENT_DRAW_MAIN)
    {
        lv_coord_t tw = lv_obj_get_style_transform_width(obj, LV_PART_MAIN);

#ifndef DISABLE_LVGL_V8
        lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
        const lv_area_t *buf_area = draw_ctx->buf_area;
        const lv_area_t *clip_area = draw_ctx->clip_area;
        void *p_fb = draw_ctx->buf;
#else
        lv_layer_t *layer = lv_event_get_layer(e);
        const lv_area_t *buf_area = &layer->buf_area;
        const lv_area_t *clip_area = &layer->_clip_area;
        void *p_fb = layer->draw_buf->data;
#endif /* DISABLE_LVGL_V8 */


        lv_img_dsc_t *p_src_img = (lv_img_dsc_t *)img->src;



        lv_img_dsc_t dest;
        dest.data = p_fb;
        dest.header.w = lv_area_get_width(buf_area);
        dest.header.h = lv_area_get_height(buf_area);
        dest.data_size = dest.header.w * dest.header.h * sizeof(lv_color_t);
        dest.header.cf = LV_IMG_CF_TRUE_COLOR;
        dest.header.always_zero = 0;


        //To epic scale
        uint32_t pitch_x = p_src_img->header.w * EPIC_INPUT_SCALE_NONE / (uint32_t)tw;

#ifdef SF32LB55X
        if (pitch_x < (EPIC_INPUT_SCALE_NONE * 8))
#endif /* SF32LB55X */
        {
            lv_area_t tmp_src_area;
            lv_area_copy(&tmp_src_area, &obj->coords);

            lv_point_t tmp_pivot   = {LV_HOR_RES_MAX >> 1, 0};
            //LOG_I("draw_img_anim2: %x w=%d, hidden=%d,buf[%d~%d]", obj, tw, lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN),
            //                                draw_ctx->buf_area->y1, draw_ctx->buf_area->y2);

            // img_rotate_opa_frac2(&dest, p_src_img, 0, pitch_x, EPIC_INPUT_SCALE_NONE,
            //                      &tmp_src_area, buf_area,
            //                      clip_area, &tmp_pivot, LV_OPA_COVER, LV_COLOR_CYAN,
            //                      0, 0, 1,
            //                      0, NULL, NULL);
        }
        e->stop_processing  = 1;

    }

}

static void enter_or_exit_anim2(gui_anim_obj_t img, void *user_data, uint32_t flag, gui_anim_value_t process)
{
#if  LV_USE_IMG
    if (!lv_obj_check_type(img, &lv_image_class)) return;
#endif

    lv_coord_t width;
    bool visible;

    if (process <= (MANUAL_TRAN_ANIM_MAX_PROCESS >> 1))
    {
        width = (LV_HOR_RES_MAX) * ((MANUAL_TRAN_ANIM_MAX_PROCESS >> 1) - process) / (MANUAL_TRAN_ANIM_MAX_PROCESS >> 1);
        visible = 1;
    }
    else
    {
        width = (LV_HOR_RES_MAX) * (process - (MANUAL_TRAN_ANIM_MAX_PROCESS >> 1)) / (MANUAL_TRAN_ANIM_MAX_PROCESS >> 1);
        visible = 0;
    }

    if (FLAG_TRANS_ANIM_FG == (flag & (FLAG_TRANS_ANIM_FG | FLAG_TRANS_ANIM_REVERSE)))
    {
        //FG screen of entering animation
        visible = !visible;
    }
    else if (FLAG_TRANS_ANIM_REVERSE == (flag & (FLAG_TRANS_ANIM_FG | FLAG_TRANS_ANIM_REVERSE)))
    {
        //BG screen of exiting animation(go-back animation)
        visible = !visible;
    }

    if (visible)
        lv_obj_clear_flag(img, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(img, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_style_transform_width(img, width, LV_PART_MAIN);

    lv_obj_remove_event_cb(img, draw_img_anim2);
    lv_obj_remove_event_cb(img, draw_img_anim2);
    lv_obj_add_event_cb(img, draw_img_anim2, LV_EVENT_DRAW_MAIN | LV_EVENT_PREPROCESS, 0);
    lv_obj_add_event_cb(img, draw_img_anim2, LV_EVENT_DRAW_POST | LV_EVENT_PREPROCESS, 0);

    //rt_kprintf("exit_anim obj %x, w=%d, v=%d \n", img, width, visible);
}
#endif /* 1 == LV_USE_GPU */

static void enter_or_exit_anim_default(gui_anim_obj_t img, void *user_data, uint32_t flag, gui_anim_value_t process)
{
#if  LV_USE_IMG
    if (!lv_obj_check_type(img, &lv_image_class)) return;
#endif
    lv_point_t *pivot = user_data;
    uint16_t zoom;
    uint16_t zoom_start, zoom_end;

    lv_opa_t opa_v = 255 * process / MANUAL_TRAN_ANIM_MAX_PROCESS;

    if (flag & FLAG_TRANS_ANIM_REVERSE)
    {
        if (flag & FLAG_TRANS_ANIM_FG)
        {
            zoom_start = LV_SCALE_NONE;
            zoom_end = LV_SCALE_NONE >> 2;
            opa_v = 255 - opa_v;
        }
        else
        {
            zoom_start = LV_SCALE_NONE << 2;
            zoom_end = LV_SCALE_NONE;
        }
    }
    else
    {
        if (flag & FLAG_TRANS_ANIM_FG)
        {
            zoom_start = LV_SCALE_NONE >> 2;
            zoom_end = LV_SCALE_NONE;
        }
        else
        {
            zoom_start = LV_SCALE_NONE;
            zoom_end = LV_SCALE_NONE << 2;
            opa_v = 255 - opa_v;
        }
    }

    zoom = zoom_start + (zoom_end - zoom_start) * process / MANUAL_TRAN_ANIM_MAX_PROCESS;



    lv_img_set_zoom(img, zoom);


    if (pivot)
        lv_img_set_pivot(img, pivot->x, pivot->y);
    else
        lv_img_set_pivot(img, lv_obj_get_width(img) >> 1, lv_obj_get_height(img) >> 1);

    lv_obj_set_style_img_opa(img, opa_v, LV_PART_MAIN);


    //rt_kprintf("enter_anim obj %x, flag=%d, process=%d \n", img, flag, process);
}


void cust_trans_anim_config(CUST_ANIM_TYPE_E type, lv_point_t *pivot)
{
    gui_app_trans_anim_t enter_anim_cfg, exit_anim_cfg;

    switch (type)
    {
    case CUST_ANIM_TYPE_0:
        gui_app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_CUSTOM);
        gui_app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_CUSTOM);
        enter_anim_cfg.cfg.cust.exe_cb = enter_or_exit_anim0;
        enter_anim_cfg.cfg.cust.user_data = NULL;

        exit_anim_cfg.cfg.cust.exe_cb = enter_or_exit_anim0;
        exit_anim_cfg.cfg.cust.user_data = NULL;
        break;

    case CUST_ANIM_TYPE_1:
        gui_app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_CUSTOM);
        gui_app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_CUSTOM);
        enter_anim_cfg.cfg.cust.exe_cb = enter_anim1;
        enter_anim_cfg.cfg.cust.user_data = NULL;

        exit_anim_cfg.cfg.cust.exe_cb = exit_anim1;
        exit_anim_cfg.cfg.cust.user_data = NULL;
        break;

#if (1 == LV_USE_GPU)&&defined(LV_USE_GPU)
    case CUST_ANIM_TYPE_2:
        gui_app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_CUSTOM);
        gui_app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_CUSTOM);
        enter_anim_cfg.cfg.cust.exe_cb = enter_or_exit_anim2;
        enter_anim_cfg.cfg.cust.user_data = NULL;

        exit_anim_cfg.cfg.cust.exe_cb = enter_or_exit_anim2;
        exit_anim_cfg.cfg.cust.user_data = NULL;
        break;
#endif

    case CUST_ANIM_TYPE_3:
    default:
        gui_app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_CUSTOM);
        gui_app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_CUSTOM);
        enter_anim_cfg.cfg.cust.exe_cb = enter_or_exit_anim_default;
        enter_anim_cfg.cfg.cust.user_data = NULL;

        exit_anim_cfg.cfg.cust.exe_cb = enter_or_exit_anim_default;
        exit_anim_cfg.cfg.cust.user_data = pivot;
        break;
    }


    gui_app_set_enter_trans_anim(&enter_anim_cfg);
    gui_app_set_exit_trans_anim(&exit_anim_cfg);
}
#else
void cust_trans_anim_config(CUST_ANIM_TYPE_E type, lv_point_t *pivot)
{
}
#endif /* APP_TRANS_ANIMATION_SCALE */

