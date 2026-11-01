/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvsf_switchanim.h"
#include "app_mem.h"

#if LVSF_USING_SWITCHANIM != 0

#if !LV_USE_SNAPSHOT
    #error "LV_USE_SNAPSHOT must be enabled when LVSF_USING_SWITCHANIM is enabled"
#endif

#define LVSF_ANIM_PERIOD_MS 3000

#if APP_TRANS_ANIM_WITH_MASK
/* The circle shape mask is generated once, resident in PSRAM, so the snapshot
 * render can install it as an LVGL MAP draw-mask; the EPIC GPU then blends the
 * snapshot through it in hardware. */
static uint8_t *s_mask_buf = NULL;
static lv_draw_mask_map_param_t s_mask_param;
static int16_t s_mask_id = -1;

static void transanim_init_circle_mask(uint8_t *buf, uint32_t size, uint32_t w, uint32_t h)
{
    RT_ASSERT(size >= w * h);
    uint32_t r = LV_MIN(w, h);
    /* Center the circle in the buffer: on a non-square display it must still be
     * centered, otherwise only the top-left r x r area is shaped and the rest of
     * the rows/columns are zeroed by the mask apply below. */
    lv_area_t circle_area = { (lv_coord_t)((w - r) / 2), (lv_coord_t)((h - r) / 2), 0, 0 };
    circle_area.x2 = circle_area.x1 + r - 1;
    circle_area.y2 = circle_area.y1 + r - 1;

    lv_draw_mask_radius_param_t circle_mask;
    lv_draw_mask_radius_init(&circle_mask, &circle_area, r / 2, false);

    int16_t mask_id = lv_draw_mask_add(&circle_mask, NULL);
    rt_memset(buf, 0xFF, size);
    uint8_t *p_buf = buf;
    for (uint32_t y = 0; y < h; y++)
    {
        lv_draw_mask_res_t res = lv_draw_mask_apply(p_buf, 0, y, w);
        if (res == LV_DRAW_MASK_RES_TRANSP)
            lv_memset(p_buf, 0x00, w);
        p_buf += w;
    }
    lv_draw_mask_remove_id(mask_id);
    lv_draw_mask_free_param(&circle_mask);
}

/* Install/remove the resident circle mask around a snapshot render. Shared by
 * the transition animation and the pre-captured exit snapshot in
 * gui_app_trans_anim.c, so both snapshots are shaped identically. */
void lv_switchanim_snapshot_mask_add(void)
{
    if (s_mask_buf && s_mask_id < 0)
        s_mask_id = lv_draw_mask_add(&s_mask_param, NULL);
}

void lv_switchanim_snapshot_mask_remove(void)
{
    if (s_mask_id >= 0)
    {
        lv_draw_mask_remove_id(s_mask_id);
        s_mask_id = -1;
    }
}
#endif /* APP_TRANS_ANIM_WITH_MASK */

/* Clear the snapshot buffer before rendering. Skipped for the exit snapshot
 * reusing a pre-captured buffer (LV_SWITCHANIM_FLAG_SKIP_SNAPHOT) so its content
 * is not wiped out. */
#if APP_TRANS_ANIM_WITH_MASK && (!defined(DRV_EPIC_NEW_API) || defined (BSP_USING_PC_SIMULATOR))
    #define TRANS_ANIM_SNAPSHOT_CLEAR(dsc) rt_memset((void *)(dsc)->data, 0, (dsc)->data_size)
#else
    #define TRANS_ANIM_SNAPSHOT_CLEAR(dsc) do { } while (0)
#endif

#define MY_CLASS  &lv_switchanim_class

struct _lv_baseanim
{
    lv_obj_t *screen;
    lv_obj_t *anim_obj;
    lv_baseanim_para_t *anim_para;
    lv_baseanim_cb *anim_cb;
    lv_baseanim_type type;
};

typedef struct
{
    lv_obj_t obj_base;
    lv_baseanim_t enter_anim;
    lv_baseanim_t exit_anim;
    lv_timer_t *anim_task;
    lv_switchanim_finish_cb finish_cb;
    uint32_t start_val;
    uint32_t end_val;
#if 0
    uint32_t cur_tick;
    uint32_t target_tick;
#else
    uint32_t pre_time;
    uint32_t start_tick;
    uint32_t period;
#endif
    uint8_t path_type;
    bool en_reverse;
} lv_switchanim_ext_t;

lv_ll_t g_baseanim_anim_list;

extern void *get_disp_buf(uint32_t size);

static void lv_switchanim_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_switchanim_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_baseanim_init_fb_dsc(lv_img_dsc_t *img_scr, lv_disp_t *disp);
static void lv_switchanim_ready(lv_obj_t *switchanim);
static void lv_switchanim_progress(lv_obj_t *switchanim, int32_t progress);
void lv_baseanim_init_anim(lv_baseanim_t *anim, lv_obj_t *parent);

const lv_obj_class_t lv_switchanim_class =
{
    .constructor_cb = lv_switchanim_constructor,
    .destructor_cb = lv_switchanim_destructor,
    .width_def = LV_HOR_RES_MAX,
    .height_def = LV_VER_RES_MAX,
    .instance_size = sizeof(lv_switchanim_ext_t),
    .base_class = &lv_obj_class,
};

static void lv_switchanim_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)obj;
    obj->render_async = 0;
    ext->enter_anim.screen = NULL;
    ext->enter_anim.anim_para = NULL;
    ext->enter_anim.type = LV_BASEANIM_ENTER_TYPE;
    ext->exit_anim.screen = NULL;
    ext->exit_anim.anim_para = NULL;
    ext->exit_anim.type = LV_BASEANIM_EXIT_TYPE;
    ext->anim_task = NULL;
#if 0
    ext->cur_tick = 0;
    ext->target_tick = LVSF_ANIM_PERIOD_MS / LV_DISP_DEF_REFR_PERIOD;
#else
    ext->pre_time = 0;
    ext->start_tick = lv_tick_get();
    ext->period = LVSF_ANIM_PERIOD_MS;
#endif

    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    LV_TRACE_OBJ_CREATE("finished");
}
static void lv_switchanim_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)obj;
    if (ext->anim_task)
        lv_timer_del(ext->anim_task);
}

void lv_switchanim_init()
{
    _lv_ll_init(&g_baseanim_anim_list, sizeof(lv_baseanim_cb));
    lv_switchanim_load();

#if APP_TRANS_ANIM_WITH_MASK
    /* Use the compile-time max resolution: LV_HOR_RES/LV_VER_RES are runtime
     * queries and may return 0 if the display is not registered yet. */
    lv_coord_t w = LV_HOR_RES_MAX;
    lv_coord_t h = LV_VER_RES_MAX;
    s_mask_buf = app_cache_alloc(w * h, CACHE_PSRAM);
    if (s_mask_buf)
    {
        transanim_init_circle_mask(s_mask_buf, w * h, w, h);
        lv_area_t mask_area = { 0, 0, w - 1, h - 1 };
        lv_draw_mask_map_init(&s_mask_param, &mask_area, s_mask_buf);
    }
    else
    {
        /* The snapshot stays TRUE_COLOR_ALPHA without a mask to fill its alpha
         * channel, so it ends up fully transparent and the transition animation
         * becomes invisible. Report the reason instead of failing silently. */
        rt_kprintf("lv_switchanim: no memory for %dx%d snapshot mask, "
                   "transition mask disabled\n", w, h);
    }
#endif
}

void lv_switchanim_deinit()
{
    _lv_ll_clear(&g_baseanim_anim_list);
#if APP_TRANS_ANIM_WITH_MASK
    /* Detach the mask from the LVGL mask list before releasing the A8 map it
     * points to, otherwise later draws would read freed memory. */
    lv_switchanim_snapshot_mask_remove();
    if (s_mask_buf)
    {
        app_cache_free(s_mask_buf);
        s_mask_buf = NULL;
    }
#endif
}

lv_baseanim_cb *lv_switchanim_find_anim(uint32_t major)
{
    lv_baseanim_cb *res = NULL;
    _LV_LL_READ(&g_baseanim_anim_list, res)
    {
        if (major == res->major) break;
    }
    return res;
}

void lv_switchanim_unregister_anim(uint32_t major)
{
    lv_baseanim_cb *item = NULL;
    _LV_LL_READ(&g_baseanim_anim_list, item)
    {
        if (major == item->major)
        {
            _lv_ll_remove(&g_baseanim_anim_list, item);
            lv_mem_free(item);
        }
    }
}

lv_baseanim_cb *lv_switchanim_register_anim(char *name, uint32_t major, lv_baseanim_progress_cb progress_cb)
{
    if (NULL == progress_cb) return NULL;
    lv_baseanim_cb *res = lv_switchanim_find_anim(major);
    if (NULL == res)
        res = (lv_baseanim_cb *)_lv_ll_ins_tail(&g_baseanim_anim_list);
    LV_ASSERT_NULL(res);

    lv_coord_t str_len = strlen(name);
    str_len = str_len > SWITCH_ANIM_NAME_LEN ? SWITCH_ANIM_NAME_LEN : str_len;
    lv_memcpy(&res->name, name, str_len);
    res->major = major;
    res->progress_cb = progress_cb;
    return res;
}

lv_obj_t *lv_baseanim_get_original_screen(lv_baseanim_t *baseanim)
{
    lv_obj_t *res = NULL;
    if (baseanim)
        res = baseanim->screen;
    return res;
}

lv_baseanim_para_t *lv_baseanim_get_para(lv_baseanim_t *baseanim)
{
    lv_baseanim_para_t *res = NULL;
    if (baseanim)
        res = baseanim->anim_para;
    return res;
}

lv_baseanim_type lv_baseanim_get_type(lv_baseanim_t *baseanim)
{
    lv_baseanim_type res = LV_BASEANIM_NONE_TYPE;
    if (baseanim)
        res = baseanim->type;
    return res;
}

static void lv_baseanim_init_snapshot_dsc(lv_img_dsc_t *img_dsc, uint8_t index, uint16_t align)
{
    uint8_t *trans_anim_buf = NULL;
    trans_anim_buf = (uint8_t *)app_anim_buf_alloc(APP_TRANS_ANIM_SNAPSHOT_SIZE, index);
    RT_ASSERT(trans_anim_buf);
    lv_coord_t w = APP_TRANS_ANIM_SNAPSHOT_WIDTH;
    lv_coord_t h = APP_TRANS_ANIM_SNAPSHOT_HEIGHT;
    if (align)
    {
        w = (w >> 4) << 4;
        if (w != LV_HOR_RES)
            h =  LV_VER_RES * w / LV_HOR_RES + 1;
        else
            h = LV_VER_RES;
        h = LV_MIN(h, APP_TRANS_ANIM_SNAPSHOT_HEIGHT);
    }

    img_dsc->header.always_zero = 0;
    img_dsc->header.reserved = 0;
    img_dsc->header.cf = APP_TRANS_ANIM_SNAPSHOT_CF;
    img_dsc->header.w = w;
    img_dsc->header.h = h;
    img_dsc->data = trans_anim_buf;
    img_dsc->data_size = lv_img_buf_get_img_size(w, h, APP_TRANS_ANIM_SNAPSHOT_CF);
}

static void lv_baseanim_init_fb_dsc(lv_img_dsc_t *img_scr, lv_disp_t *disp)
{
    lv_coord_t hor_res = lv_disp_get_hor_res(disp);
    lv_coord_t ver_res = lv_disp_get_ver_res(disp);
    img_scr->header.always_zero = 0;
    img_scr->header.w = hor_res;
    img_scr->header.h = ver_res;
    img_scr->data_size = (LV_COLOR_DEPTH * hor_res * ver_res) / 8;
    img_scr->header.cf = disp->driver->draw_buf->cf;
    int32_t px_size = lv_img_cf_get_px_size(img_scr->header.cf);
    int32_t draw_size = hor_res * ver_res * ((px_size + 7) >> 3);
    void *draw_buf = get_disp_buf(draw_size);
    LV_ASSERT_NULL(draw_buf);
    img_scr->data = (uint8_t *)draw_buf;
}

void lv_snaphot_dsc_free(lv_event_t *e)
{
    lv_obj_t *img = lv_event_get_current_target(e);
    RT_ASSERT(lv_scr_act() != lv_obj_get_parent(img));
    const void *dsc = lv_img_get_src(img);
    if (dsc)
        lv_mem_free((void *)dsc);
    lv_img_set_src(img, NULL);
}

lv_obj_t *lv_baseanim_create_snapshot(lv_baseanim_t *baseanim, lv_obj_t *parent)
{
    uint8_t anim_buf_index = LV_BASEANIM_ENTER_TYPE == baseanim->type ? 0 : 1;

    lv_img_dsc_t *snapshot_dsc;
    snapshot_dsc = lv_mem_alloc(sizeof(lv_img_dsc_t));
    RT_ASSERT(snapshot_dsc != NULL);
    lv_baseanim_para_t *para = lv_baseanim_get_para(baseanim);
    lv_baseanim_init_snapshot_dsc(snapshot_dsc, anim_buf_index, para->flag & LV_SWITCHANIM_FLAG_16_ALIGN);

    lv_disp_t *disp = _lv_refr_get_disp_refreshing();
    if (NULL == disp) disp = lv_disp_get_default();

#ifdef DISABLE_LVGL_V9
    lv_draw_ctx_t *draw_ctx = disp->driver->draw_ctx;
    lv_disp_draw_buf_t *vdb = lv_disp_get_draw_buf(disp);
    if (draw_ctx->wait_for_finish) draw_ctx->wait_for_finish(draw_ctx);
#endif

#if APP_TRANS_ANIM_WITH_MASK
    lv_switchanim_snapshot_mask_add();
#endif

    if (LV_BASEANIM_EXIT_TYPE == baseanim->type)
    {
        lv_baseanim_para_t *para = lv_baseanim_get_para(baseanim);
        if (!(para->flag & LV_SWITCHANIM_FLAG_SKIP_SNAPHOT))
        {
            TRANS_ANIM_SNAPSHOT_CLEAR(snapshot_dsc);
#if defined(LCD_FB_USING_TWO_COMPRESSED) || defined(LCD_FB_USING_ONE_COMPRESSED)
#if (APP_TRANS_ANIM_FULL_SCALE != APP_TRANS_ANIM_SNAPSHOT_SCALE)
            lv_snapshot_obj_to_dsc_scale(baseanim->screen, &baseanim->screen->coords, snapshot_dsc);
#else
            lv_obj_update_layout(baseanim->screen);
            lv_snapshot_obj_to_dsc(baseanim->screen, &baseanim->screen->coords, snapshot_dsc);
#endif
#else
            lv_snapshot_fb_to_dsc(snapshot_dsc);
#endif
        }
    }
    else if (LV_BASEANIM_ENTER_TYPE == baseanim->type)
    {
        TRANS_ANIM_SNAPSHOT_CLEAR(snapshot_dsc);
#if (APP_TRANS_ANIM_FULL_SCALE != APP_TRANS_ANIM_SNAPSHOT_SCALE)
        lv_snapshot_obj_to_dsc_scale(baseanim->screen, &baseanim->screen->coords, snapshot_dsc);
#else
        lv_obj_update_layout(baseanim->screen);
        lv_snapshot_obj_to_dsc(baseanim->screen, &baseanim->screen->coords, snapshot_dsc);
#endif
    }
#ifdef DISABLE_LVGL_V9
    if (draw_ctx->wait_for_finish) draw_ctx->wait_for_finish(draw_ctx);
#endif
#if APP_TRANS_ANIM_WITH_MASK
    lv_switchanim_snapshot_mask_remove();
#endif
    lv_obj_t *image = lv_img_create(parent);
    lv_img_set_src(image, snapshot_dsc);
    lv_obj_add_event_cb(image, lv_snaphot_dsc_free, LV_EVENT_DELETE, snapshot_dsc);
    //lv_img_set_antialias(image, false);
#if(APP_TRANS_ANIM_FULL_SCALE != APP_TRANS_ANIM_SNAPSHOT_SCALE)
    lv_coord_t img_w = snapshot_dsc->header.w;
    lv_coord_t hor_res = lv_disp_get_hor_res(disp);
    lv_coord_t zoom = (hor_res * LV_IMG_ZOOM_NONE + img_w - 1) / img_w;
    lv_img_set_zoom(image, zoom);
    lv_obj_refr_size(image);
    lv_obj_align(image, LV_ALIGN_CENTER, 0, 0);
    lv_obj_refr_pos(image);
#endif
    return image;
}

void lv_baseanim_init_anim(lv_baseanim_t *anim, lv_obj_t *parent)
{
    //ini default
    anim->anim_cb = lv_switchanim_find_anim(LV_SWITCHANIM_DEFAULT_TYPE);
    LV_ASSERT_NULL(anim->anim_cb);
    if (anim->anim_para)
    {
        lv_baseanim_cb *anim_cb = lv_switchanim_find_anim(anim->anim_para->major);
        if (anim_cb) anim->anim_cb = anim_cb;
    }
    anim->anim_obj = lv_baseanim_create_snapshot(anim, parent);
    LV_ASSERT_NULL(anim->anim_obj);
}

lv_obj_t *lv_switchanim_create(lv_obj_t *parent, lv_obj_t *enter_screen, lv_obj_t *exit_screen,
                               lv_baseanim_para_t *enter_para, lv_baseanim_para_t *exit_para)
{
    lv_obj_t *simplist = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(simplist);
    lv_obj_remove_style_all(simplist);
    //lv_obj_set_style_bg_color(simplist, LV_COLOR_RED, LV_PART_MAIN | LV_STATE_DEFAULT);
    //lv_obj_set_style_bg_opa(simplist, LV_OPA_100, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)simplist;
    ext->enter_anim.screen = enter_screen;
    ext->enter_anim.anim_para = enter_para;
    ext->exit_anim.screen = exit_screen;
    ext->exit_anim.anim_para = exit_para;
    return simplist;
}

void lv_switchanim_manual_run(lv_obj_t *switchanim, int32_t progress)
{
    LV_ASSERT_OBJ(switchanim, MY_CLASS);
    progress = progress > LV_SWITCHANIM_PROGRESS_MAX ? LV_SWITCHANIM_PROGRESS_MAX : progress;
    progress = progress < 0 ? 0 : progress;

    if (switchanim != lv_scr_act())
    {
        lv_scr_load(switchanim);
        lv_switchanim_ready(switchanim);
    }
    lv_switchanim_progress(switchanim, progress);
}
void lv_switchanim_refr_cb(lv_timer_t *p_timer)
{
    lv_obj_t *switchanim = (lv_obj_t *)p_timer->user_data;
    LV_ASSERT_OBJ(switchanim, MY_CLASS);
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)(switchanim);
    bool is_finish = false;
    int32_t progress = 0;
#if 0
    is_finish = ext->cur_tick > ext->target_tick;
    progress = _lv_map(ext->cur_tick, 0, ext->target_tick, 0, LV_SWITCHANIM_PROGRESS_MAX);
    ext->cur_tick++;
#else
    is_finish = ext->pre_time >= ext->period;
    ext->pre_time = lv_tick_elaps(ext->start_tick);
    progress = lv_map(ext->pre_time, 0, ext->period, ext->start_val, ext->end_val);

#endif

    if (is_finish)
    {
        lv_scr_load(ext->enter_anim.screen);
        lv_timer_del(p_timer);

        ext->anim_task = NULL;
        if (ext->finish_cb)
            ext->finish_cb(!ext->en_reverse);
        return;
    }

    if (LV_BASEANIM_PATH_EASE_IN == ext->path_type)
        progress = lv_bezier3(progress, 0, 20, 50, 1024);
    else if (LV_BASEANIM_PATH_EASE_OUT == ext->path_type)
        progress = lv_bezier3(progress, 0, 950, 1000, 1024);
    else if (LV_BASEANIM_PATH_EASE_IN_OUT == ext->path_type)
        progress = lv_bezier3(progress, 0, 20, 1000, 1024);

    lv_switchanim_progress(switchanim, progress);
}

void lv_switchanim_auto_run(lv_obj_t *switchanim, uint32_t period, uint32_t progres, bool reverse)
{
    LV_ASSERT_OBJ(switchanim, MY_CLASS);
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)(switchanim);

    if (switchanim != lv_scr_act())
    {
        lv_scr_load(switchanim);
        lv_switchanim_ready(switchanim);
    }
    if (!ext->anim_task)
    {
        ext->en_reverse = reverse;
#if 0
        ext->target_tick = period / LV_DISP_DEF_REFR_PERIOD;
        ext->target_tick++;
        ext->cur_tick = (progres * ext->target_tick) >> 10;
        ext->cur_tick = is_normal ? ext->cur_tick : (ext->target_tick - ext->cur_tick);
#else
        progres = LV_MIN(progres, LV_SWITCHANIM_PROGRESS_MAX);
        ext->start_val = progres;
        ext->end_val = LV_SWITCHANIM_PROGRESS_MAX;
        ext->pre_time = 0;
        ext->start_tick = lv_tick_get();
        ext->period = period * (LV_SWITCHANIM_PROGRESS_MAX - progres) / LV_SWITCHANIM_PROGRESS_MAX;
        if (ext->en_reverse)
        {
            ext->period = period - ext->period;
            ext->end_val = 0;
        }
#endif
        ext->anim_task = lv_timer_create(lv_switchanim_refr_cb, LV_DISP_DEF_REFR_PERIOD, (void *)switchanim);
    }
    if (ext->anim_task)
        lv_switchanim_refr_cb(ext->anim_task);
}

void lv_switchanim_set_finish_cb(lv_obj_t *switchanim, lv_switchanim_finish_cb finish_cb)
{
    LV_ASSERT_OBJ(switchanim, MY_CLASS);
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)(switchanim);
    ext->finish_cb = finish_cb;
}

void lv_switchanim_set_path(lv_obj_t *switchanim, lv_baseanim_path path)
{
    LV_ASSERT_OBJ(switchanim, MY_CLASS);
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)(switchanim);
    ext->path_type = path;
}

void lv_switchanim_manual_finish(lv_obj_t *switchanim, bool is_enter)
{
    LV_ASSERT_OBJ(switchanim, MY_CLASS);
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)(switchanim);
    if (is_enter)
        lv_scr_load(ext->enter_anim.screen);
    else
        lv_scr_load(ext->exit_anim.screen);

    if (ext->enter_anim.anim_obj)
    {
        lv_baseanim_t *anim = &ext->enter_anim;
        lv_obj_del(anim->anim_obj);
        anim->anim_obj = NULL;
    }
    if (ext->exit_anim.anim_obj)
    {
        lv_baseanim_t *anim = &ext->exit_anim;
        lv_obj_del(anim->anim_obj);
        anim->anim_obj = NULL;
    }

}

static void lv_switchanim_ready(lv_obj_t *switchanim)
{
    LV_ASSERT_OBJ(switchanim, MY_CLASS);
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)(switchanim);
    if (ext->exit_anim.screen)
        lv_baseanim_init_anim(&ext->exit_anim, switchanim);
    if (ext->enter_anim.screen)
        lv_baseanim_init_anim(&ext->enter_anim, switchanim);
    if (ext->exit_anim.screen)
        lv_obj_move_foreground(ext->exit_anim.anim_obj);
}
static void lv_switchanim_progress(lv_obj_t *switchanim, int32_t progress)
{
    LV_ASSERT_OBJ(switchanim, MY_CLASS);
    lv_switchanim_ext_t *ext = (lv_switchanim_ext_t *)(switchanim);
    if (ext->enter_anim.anim_obj)
        ext->enter_anim.anim_cb->progress_cb(&ext->enter_anim, ext->enter_anim.anim_obj, progress);
    if (ext->exit_anim.anim_obj)
        ext->exit_anim.anim_cb->progress_cb(&ext->exit_anim, ext->exit_anim.anim_obj, progress);
}

#endif
