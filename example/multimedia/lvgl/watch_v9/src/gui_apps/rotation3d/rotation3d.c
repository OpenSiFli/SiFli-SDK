/*********************
 *      INCLUDES
 *********************/


#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
// #include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "app_mem.h"


#if (1 == LV_USE_GPU) && (!defined(SF32LB55X))
#include "drv_epic.h"
#if (IMAGE_CACHE_IN_SRAM_SIZE >= 258*1000)
    #define ROTATE_BG_IMG_VARIABLE clock_rotate_bg_bg   //Image size 257766 bytes, 359x359
#elif (IMAGE_CACHE_IN_SRAM_SIZE >= 81*1000)
    #define ROTATE_BG_IMG_VARIABLE clock_rotate_bg_bg_small //Image size 80000 bytes, 200x200
#else
    #define ROTATE_BG_IMG_VARIABLE clock_rotate_bg_bg_tiny //Image size 45000 bytes, 150x150
#endif

LV_IMG_DECLARE(ROTATE_BG_IMG_VARIABLE);

LV_IMG_DECLARE(d3d_rotate);

/**

    Assume screen top-left as origin(0,0,0)

^
 \  Z ( z-axis 's positive direction is out of screen)
  \
   \
    \
     O---------------->
     |            X
     |
     |
     |
     |
     |  Y
     v




**/


#define ROTATION_3D_CACHE_SRC

typedef struct
{
    lv_obj_t *image[2];
    lv_img_dsc_t *src_image[2];
#ifdef ROTATION_3D_CACHE_SRC
    lv_img_dsc_t *src_image_sram;
#endif /* ROTATION_3D_CACHE_SRC */

    uint8_t rotation_dir; /*0 - Vertical, 1 - horizontal*/
    int16_t last_degree;

    int16_t image_rotate_degree;
    lv_timer_t *redraw_task;
    lv_timer_t *auto_rotate_task;

    uint32_t log_level;
} rotation3d_ctx_t;

#define ROTATE3D_LOG_DETAL  0x01


static rotation3d_ctx_t rotation3d;


extern void *app_anim_buf_alloc(size_t nbytes, uint8_t index);
extern void *app_anim_buf_free(void *ptr);

extern void fill_color_opa(lv_img_cf_t cf, lv_color_t *dest_buf, lv_coord_t dest_width,
                           const lv_area_t *fill_area, lv_color_t color, lv_opa_t opa,
                           lv_img_cf_t mask_cf, const lv_opa_t *mask_map, const lv_area_t *mask_coords);
extern void img_rotate_adv1(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                            const lv_area_t *p_src_coords, const lv_area_t *p_dst_coords,
                            const lv_area_t *p_output_coords, lv_opa_t opa, lv_color_t ax_color,
                            lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom);

extern void img_rotate_adv2(lv_img_dsc_t *dest, const lv_img_dsc_t *src, int16_t angle,
                            const lv_area_t *p_src_coords, const lv_area_t *p_dst_coords,
                            const lv_area_t *p_output_coords, lv_opa_t opa, lv_color_t ax_color,
                            lv_point_t *pivot, lv_coord_t pivot_z,  lv_coord_t src_z, uint16_t src_zoom);



static lv_coord_t src_img_z =  -447;
static lv_coord_t pivot_z =  -626;
static uint16_t src_zoom = LV_IMG_ZOOM_NONE;

static void redraw_task_handler(lv_timer_t *task)
{
    if (rotation3d.last_degree != rotation3d.image_rotate_degree)
        rotation3d.last_degree = rotation3d.image_rotate_degree;
    else
        return;

    if (rotation3d.image[0]) lv_obj_invalidate(rotation3d.image[0]);
}
static void auto_rotate_task_handler(lv_timer_t *task)
{
    static int16_t d = 10;

    //  if((rotation3d.image_rotate_degree + d > 900) || (rotation3d.image_rotate_degree + d < 0))
    //  d = -d;
    //rotation3d.image_rotate_degree += d;


    rotation3d.image_rotate_degree = (rotation3d.image_rotate_degree + d) % 900;

}


static void img_event_handler(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_LONG_PRESSED == event)
    {
        if (rotation3d.auto_rotate_task)
        {
            lv_timer_del(rotation3d.auto_rotate_task);
            rotation3d.auto_rotate_task = NULL;
        }
        else
        {
            rotation3d.auto_rotate_task = lv_timer_create(auto_rotate_task_handler, 16, (void *)0);
        }
    }
    else if (LV_EVENT_PRESSING == event)
    {
        lv_indev_t *indev = lv_indev_get_act();

        lv_point_t p;

        lv_indev_get_point(indev, &p);

        //rt_kprintf("point %d,%d \n",p.x,p.y);

        if (0 == rotation3d.rotation_dir)
        {
            if ((p.y >= rotation3d.image[0]->coords.y1) && (p.y <= rotation3d.image[0]->coords.y2))
            {
                rotation3d.image_rotate_degree = ((rotation3d.image[0]->coords.y2 - p.y) * (900) / lv_area_get_height(&rotation3d.image[0]->coords));
            }
        }
        else
        {
            if ((p.x >= rotation3d.image[0]->coords.x1) && (p.x <= rotation3d.image[0]->coords.x2))
            {
                rotation3d.image_rotate_degree = ((rotation3d.image[0]->coords.x2 - p.x) * (900) / lv_area_get_width(&rotation3d.image[0]->coords));
            }
        }
    }
    else if (LV_EVENT_DRAW_MAIN == event)
    {
        uint32_t start_tick = rt_tick_get();
        uint32_t prev_epic_cnt = drv_get_epic_handle()->PerfCnt;

        lv_obj_t *obj = lv_event_get_target(e);

#ifndef DISABLE_LVGL_V8
        lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
        const lv_area_t *buf_area = draw_ctx->buf_area;
        const lv_area_t *clip_area = draw_ctx->clip_area;
        void *p_fb = draw_ctx->buf;
#else
        lv_layer_t *draw_ctx = lv_event_get_layer(e);
        const lv_area_t *buf_area = &draw_ctx->buf_area;
        const lv_area_t *clip_area = &draw_ctx->_clip_area;
        void *p_fb = draw_ctx->draw_buf->data;
#endif /* DISABLE_LVGL_V8 */

        lv_img_dsc_t dest;

        dest.data = p_fb;
        dest.header.w = lv_area_get_width(buf_area);
        dest.header.h = lv_area_get_height(buf_area);
        dest.data_size = dest.header.w * dest.header.h * sizeof(lv_color_t);
        dest.header.cf = LV_IMG_CF_TRUE_COLOR;
        dest.header.always_zero = 0;

        const lv_img_dsc_t *image0 = rotation3d.src_image[0];
        const lv_img_dsc_t *image1 = rotation3d.src_image[1];
        lv_area_t src_coords;
        lv_point_t pivot;

        //Align src_image to the center of obj.
        src_coords.x1 = (lv_area_get_width(&obj->coords) - image0->header.w) / 2;
        src_coords.y1 = (lv_area_get_height(&obj->coords) - image0->header.h) / 2;
        src_coords.x2 = src_coords.x1 + image0->header.w - 1;
        src_coords.y2 = src_coords.y1 + image0->header.h - 1;
        pivot.x = src_coords.x1 + image0->header.w / 2;
        pivot.y = src_coords.y1 + image0->header.h / 2;






        if (rotation3d.log_level & ROTATE3D_LOG_DETAL)
        {
            rt_kprintf("\n\n\nredraw degree:%d \n", rotation3d.image_rotate_degree);
        }

        //Clear output image
        ///lv_memset_ff((uint8_t *)rotation3d.rotated_image.data, rotation3d.rotated_image.data_size);
        /*
                fill_color_opa(rotation3d.rotated_image.header.cf,
                               (lv_color_t *)rotation3d.rotated_image.data,
                               rotation3d.rotated_image.header.w,
                               &dst_coords, LV_COLOR_BLACK, LV_OPA_COVER,
                               0, NULL, NULL);
        */
        lv_draw_rect_dsc_t draw_dsc;
        lv_draw_rect_dsc_init(&draw_dsc);
        draw_dsc.bg_color = lv_color_black();
        lv_draw_rect(draw_ctx, &draw_dsc, &obj->coords);

        if (0 == rotation3d.rotation_dir)
        {
            img_rotate_adv1(&dest, image0, rotation3d.image_rotate_degree,
                            &src_coords, buf_area, clip_area, LV_OPA_COVER, LV_COLOR_BLACK,
                            &pivot, pivot_z, src_img_z, src_zoom);



            img_rotate_adv1(&dest, image1, rotation3d.image_rotate_degree - 900,
                            &src_coords, buf_area, clip_area, LV_OPA_COVER, LV_COLOR_BLACK,
                            &pivot, pivot_z, src_img_z, src_zoom);

        }
        else
        {

            img_rotate_adv2(&dest, image0, rotation3d.image_rotate_degree,
                            &src_coords, buf_area, clip_area, LV_OPA_COVER, LV_COLOR_BLACK,
                            &pivot, pivot_z, src_img_z, src_zoom);



            img_rotate_adv2(&dest, image1, rotation3d.image_rotate_degree - 900,
                            &src_coords, buf_area, clip_area, LV_OPA_COVER, LV_COLOR_BLACK,
                            &pivot, pivot_z, src_img_z, src_zoom);

        }


        if (rotation3d.log_level & ROTATE3D_LOG_DETAL)
        {

            uint32_t cur_epic_cnt = drv_get_epic_handle()->PerfCnt;
            uint32_t cost_cnt;

            if (cur_epic_cnt >= prev_epic_cnt)
                cost_cnt = cur_epic_cnt - prev_epic_cnt;
            else
                cost_cnt = cur_epic_cnt + (UINT32_MAX - prev_epic_cnt + 1);

            rt_kprintf("redraw ticks=%d epic=%d ms \n", rt_tick_get() - start_tick,
                       cost_cnt / (HAL_RCC_GetHCLKFreq(CORE_ID_HCPU) / 1000));
        }

        e->stop_processing = 1;
    }
}

static void on_start(void)
{
    uint8_t last_dir = rotation3d.rotation_dir;

    memset(&rotation3d, 0, sizeof(rotation3d));

    rotation3d.rotation_dir = (last_dir + 1) & 1;
    rotation3d.last_degree = 0;
    rotation3d.image_rotate_degree = 900;

    rotation3d.src_image[0] = (lv_img_dsc_t *) LV_EXT_IMG_GET(d3d_rotate);
    rotation3d.src_image[1] = (lv_img_dsc_t *) LV_EXT_IMG_GET(d3d_rotate);

    if (0 == rotation3d.rotation_dir)
    {
        src_img_z =  0 - (rotation3d.src_image[0]->header.h * 5 / 4);
        pivot_z =  0 - (rotation3d.src_image[0]->header.h * 7 / 4);
    }
    else
    {
        src_img_z =  0 - (rotation3d.src_image[0]->header.w * 5 / 4);
        pivot_z =  0 - (rotation3d.src_image[0]->header.w * 7 / 4);
    }
    src_zoom = LV_MIN((LV_IMG_ZOOM_NONE * LV_HOR_RES_MAX) / rotation3d.src_image[0]->header.w,
                      (LV_IMG_ZOOM_NONE * LV_VER_RES_MAX) / rotation3d.src_image[0]->header.h);

    rt_kprintf("last_dir=%d cur=%d %d,%d,%d\n", last_dir, rotation3d.rotation_dir, src_img_z, pivot_z, src_zoom);



    rotation3d.image[0] = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rotation3d.image[0], LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(rotation3d.image[0], LV_ALIGN_CENTER, 0, 0);


    lv_obj_add_flag(rotation3d.image[0], LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(rotation3d.image[0], img_event_handler, LV_EVENT_ALL | LV_EVENT_PREPROCESS, NULL);

    lv_img_cache_invalidate_src(NULL);
}

static void on_pause(void)
{
#ifdef ROTATION_3D_CACHE_SRC
    if (rotation3d.src_image_sram)
    {
        rotation3d.src_image[0] = (lv_img_dsc_t *) LV_EXT_IMG_GET(d3d_rotate);
        rotation3d.src_image[1] = (lv_img_dsc_t *) LV_EXT_IMG_GET(d3d_rotate);


        app_cache_copy_free(rotation3d.src_image_sram);
        rotation3d.src_image_sram = NULL;
    }
#endif /* ROTATION_3D_CACHE_SRC */

    if (rotation3d.redraw_task)
    {
        lv_timer_del(rotation3d.redraw_task);
        rotation3d.redraw_task = NULL;
    }

    if (rotation3d.auto_rotate_task)
    {
        lv_timer_del(rotation3d.auto_rotate_task);
        rotation3d.auto_rotate_task = NULL;
    }
}

static void on_resume(void)
{
#ifdef ROTATION_3D_CACHE_SRC
    if (!rotation3d.src_image_sram)
    {
        rotation3d.src_image_sram = app_cache_copy_alloc(rotation3d.src_image[0], ROTATE_MEM);
        RT_ASSERT(rotation3d.src_image_sram);

        rotation3d.src_image[0] = rotation3d.src_image_sram;
        rotation3d.src_image[1] = rotation3d.src_image_sram;
    }
#endif

    if (!rotation3d.redraw_task)
    {
        rotation3d.redraw_task = lv_timer_create(redraw_task_handler, 16, (void *)0);
    }


    if (!rotation3d.auto_rotate_task)
    {
        rotation3d.auto_rotate_task = lv_timer_create(auto_rotate_task_handler, 16, (void *)0);
    }
}

static void on_stop(void)
{
}


static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

LV_IMG_DECLARE(img_passbook);
#define APP_ID "rotation3d"
static int app_main(intent_t i)
{



    gui_app_regist_msg_handler(APP_ID, msg_handler);

    return 0;
}



BUILTIN_APP_EXPORT(LV_EXT_STR_ID(rotation3d), LV_EXT_IMG_GET(img_passbook), APP_ID, app_main);


#ifdef FINSH_USING_MSH
#include <finsh.h>



static rt_err_t rotate_run(int argc, char **argv)
{
    if (argc > 1)
    {
        rotation3d.image_rotate_degree = ((int16_t)strtol(argv[1], 0, 10));
    }
    if (argc > 4)
    {
        src_img_z = strtol(argv[2], 0, 10);
        pivot_z   = strtol(argv[3], 0, 10);
        src_zoom  = strtol(argv[4], 0, 10);
    }
    if (argc > 5)
    {
        rotation3d.log_level = strtoul(argv[5], 0, 10);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(rotate_run, rotate_run);


extern void lv_gpu_adv_log(uint32_t level);
static rt_err_t rotate_gpu_log(int argc, char **argv)
{
    if (argc > 1)
    {
        uint32_t level = (uint32_t)strtol(argv[1], 0, 16);

        lv_gpu_adv_log(level);
    }


    return RT_EOK;
}
MSH_CMD_EXPORT(rotate_gpu_log, lv_gpu_adv_log);


#endif



#endif
