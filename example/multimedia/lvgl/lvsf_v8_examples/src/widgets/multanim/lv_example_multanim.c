/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "lvsf.h"
#include "lvsf_multanim.h"
#include <rtdbg.h>
#include <stdbool.h>
#include <stdint.h>

static lv_obj_t *img1;
static lv_obj_t *img2;
static lv_obj_t *multanim;
static lv_obj_t *status_label;
static lv_multanim_type current_type = LV_MULTANIM_ZOOM;

#if LV_USE_GPU
#define MULTANIM_HAS_EPIC 1
#else
#define MULTANIM_HAS_EPIC 0
#endif

#ifdef USING_VGLITE
#define MULTANIM_HAS_VGLITE 1
#else
#define MULTANIM_HAS_VGLITE 0
#endif

typedef struct
{
    lv_multanim_type type;
    const char *name;
    uint8_t enabled;
    uint16_t duration;
    lv_multanim_dir dir;
    int16_t range;
    lv_coord_t zoom_start;
    lv_coord_t zoom_end;
    int8_t vp_start_x_pct;
    int8_t vp_start_y_pct;
    int8_t vp_end_x_pct;
    int8_t vp_end_y_pct;
    uint8_t mask_mode;
} anim_item_t;

enum
{
    MULTANIM_MASK_NONE = 0,
    MULTANIM_MASK_FADE,
    MULTANIM_MASK_OPEN,
};

static const anim_item_t anim_items[] =
{
    { LV_MULTANIM_ZOOM,    "Zoom",    1,                  900, LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE >> 1,   0,   0,   0,   0, MULTANIM_MASK_NONE },
    { LV_MULTANIM_3D,      "3D",      MULTANIM_HAS_EPIC, 1000, LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, 640,                       0,   0,   0,   0, MULTANIM_MASK_NONE },
    { LV_MULTANIM_SWITCH,  "Switch",  MULTANIM_HAS_EPIC, 1000, LV_MULTANIM_HOR, 1200, LV_IMG_ZOOM_NONE, 760,                     -18,   0,  18,   0, MULTANIM_MASK_NONE },
    { LV_MULTANIM_TURN,    "Turn",    MULTANIM_HAS_EPIC,  950, LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE,       -12,   0,  12,   0, MULTANIM_MASK_NONE },
    { LV_MULTANIM_SCALE,   "Scale",   MULTANIM_HAS_EPIC,  950, LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, 420,                       0,   0,   0,   0, MULTANIM_MASK_NONE },
    { LV_MULTANIM_FADE,    "Fade",    1,                  850, LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE,         0,   0,   0,   0, MULTANIM_MASK_FADE },
    { LV_MULTANIM_OPEN,    "Open",    1,                  950, LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE,         0,   0,   0,   0, MULTANIM_MASK_OPEN },
    { LV_MULTANIM_ROLL,    "Roll",    MULTANIM_HAS_EPIC, 1100, LV_MULTANIM_HOR,  620, LV_IMG_ZOOM_NONE, 900,                     -28,   0,  28,   0, MULTANIM_MASK_NONE },
    { LV_MULTANIM_DRAG,    "Drag",    0,                  900, LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE,         0,   0,   0,   0, MULTANIM_MASK_NONE }, /* 目前控件里实现是空的 */
    { LV_MULTANIM_BOOK,    "Book",    MULTANIM_HAS_VGLITE,1100,LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE >> 1,    0,   0,   0,   0, MULTANIM_MASK_NONE },
    { LV_MULTANIM_SHUTTLE, "Shuttle", MULTANIM_HAS_VGLITE,1100,LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE,         0,   0,   0,   0, MULTANIM_MASK_NONE },
    { LV_MULTANIM_SHUTTER, "Shutter", MULTANIM_HAS_VGLITE,1200,LV_MULTANIM_HOR,  900, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE,         0,   0,   0,   0, MULTANIM_MASK_NONE },
};

#define ANIM_ITEM_COUNT (sizeof(anim_items) / sizeof(anim_items[0]))

#define OPEN_MASK_W 64
#define OPEN_MASK_H 32
#define FADE_MASK_W 64
#define FADE_MASK_H 24
#define FADE_FEATHER_W 14
static uint8_t open_mask_l_map[OPEN_MASK_W * OPEN_MASK_H];
static uint8_t open_mask_r_map[OPEN_MASK_W * OPEN_MASK_H];
static uint8_t fade_mask_l_map[FADE_MASK_W * FADE_MASK_H];
static uint8_t fade_mask_r_map[FADE_MASK_W * FADE_MASK_H];
static lv_img_dsc_t open_mask_l_dsc;
static lv_img_dsc_t open_mask_r_dsc;
static lv_img_dsc_t fade_mask_l_dsc;
static lv_img_dsc_t fade_mask_r_dsc;
static bool open_masks_inited;

static void mask_dsc_init(lv_img_dsc_t *dsc, lv_coord_t w, lv_coord_t h, const uint8_t *map, uint32_t size)
{
    lv_memset_00(dsc, sizeof(*dsc));
    dsc->header.always_zero = 0;
    dsc->header.w = w;
    dsc->header.h = h;
    dsc->header.cf = LV_IMG_CF_ALPHA_8BIT;
    dsc->data = map;
    dsc->data_size = size;
}

static void open_masks_init(void)
{
    if (open_masks_inited) return;

    for (uint32_t y = 0; y < OPEN_MASK_H; y++)
    {
        for (uint32_t x = 0; x < OPEN_MASK_W; x++)
        {
            uint8_t opa_l = (uint8_t)((OPEN_MASK_W - 1 - x) * 255U / (OPEN_MASK_W - 1));
            uint8_t opa_r = (uint8_t)(x * 255U / (OPEN_MASK_W - 1));
            open_mask_l_map[y * OPEN_MASK_W + x] = opa_l;
            open_mask_r_map[y * OPEN_MASK_W + x] = opa_r;
        }
    }

    for (uint32_t y = 0; y < FADE_MASK_H; y++)
    {
        for (uint32_t x = 0; x < FADE_MASK_W; x++)
        {
            uint8_t opa_l = 255;
            uint8_t opa_r = 255;

            if (x >= (FADE_MASK_W - FADE_FEATHER_W))
            {
                opa_l = (uint8_t)((FADE_MASK_W - 1 - x) * 255U / (FADE_FEATHER_W - 1));
            }

            if (x < FADE_FEATHER_W)
            {
                opa_r = (uint8_t)(x * 255U / (FADE_FEATHER_W - 1));
            }

            fade_mask_l_map[y * FADE_MASK_W + x] = opa_l;
            fade_mask_r_map[y * FADE_MASK_W + x] = opa_r;
        }
    }

    mask_dsc_init(&open_mask_l_dsc, OPEN_MASK_W, OPEN_MASK_H, open_mask_l_map, sizeof(open_mask_l_map));
    mask_dsc_init(&open_mask_r_dsc, OPEN_MASK_W, OPEN_MASK_H, open_mask_r_map, sizeof(open_mask_r_map));
    mask_dsc_init(&fade_mask_l_dsc, FADE_MASK_W, FADE_MASK_H, fade_mask_l_map, sizeof(fade_mask_l_map));
    mask_dsc_init(&fade_mask_r_dsc, FADE_MASK_W, FADE_MASK_H, fade_mask_r_map, sizeof(fade_mask_r_map));

    open_masks_inited = true;
}

static const anim_item_t *anim_item_get(lv_multanim_type type)
{
    for (uint32_t i = 0; i < (uint32_t)ANIM_ITEM_COUNT; i++)
    {
        if (anim_items[i].type == type) return &anim_items[i];
    }
    return NULL;
}

static const char *anim_name_get(lv_multanim_type type)
{
    const anim_item_t *item = anim_item_get(type);
    return item ? item->name : "Unknown";
}

static lv_point_t anim_viewpoint_from_pct(int8_t pct_x, int8_t pct_y)
{
    lv_area_t coords;
    lv_obj_get_coords(multanim, &coords);

    lv_coord_t w = lv_area_get_width(&coords);
    lv_coord_t h = lv_area_get_height(&coords);

    lv_point_t point;
    point.x = (coords.x1 + coords.x2) >> 1;
    point.y = (coords.y1 + coords.y2) >> 1;
    point.x += (w * pct_x) / 100;
    point.y += (h * pct_y) / 100;
    return point;
}

static void anim_mask_apply(uint8_t mask_mode)
{
    switch (mask_mode)
    {
    case MULTANIM_MASK_FADE:
        lv_multanim_set_mask(multanim, &fade_mask_l_dsc, &fade_mask_r_dsc);
        break;
    case MULTANIM_MASK_OPEN:
        lv_multanim_set_mask(multanim, &open_mask_l_dsc, &open_mask_r_dsc);
        break;
    default:
        lv_multanim_set_mask(multanim, NULL, NULL);
        break;
    }
}

static void anim_ready_cb(lv_anim_t *a)
{
    LV_UNUSED(a);
    LOG_I("Animation completed: type=%s(%d)", anim_name_get(current_type), (int)current_type);
}

static void update_animation(void)
{
    const anim_item_t *item = anim_item_get(current_type);
    if (!multanim) return;
    if (!item) return;

    lv_anim_del(multanim, NULL);
    LOG_I("Switch to animation type: %s(%d)", anim_name_get(current_type), (int)current_type);

    lv_point_t start_v = anim_viewpoint_from_pct(item->vp_start_x_pct, item->vp_start_y_pct);
    lv_point_t end_v = anim_viewpoint_from_pct(item->vp_end_x_pct, item->vp_end_y_pct);

    lv_multanim_set_type(multanim, current_type);
    lv_multanim_set_dir(multanim, item->dir);
    lv_multanim_set_range(multanim, item->range);
    lv_multanim_set_zoom(multanim, item->zoom_start, item->zoom_end);
    lv_multanim_set_viewpoint(multanim, &start_v, &end_v);
    anim_mask_apply(item->mask_mode);
    lv_multanim_start_anim(multanim, item->duration, 0, 1024, anim_ready_cb);

    if (status_label)
    {
        lv_label_set_text_fmt(status_label, "%s  z:%d->%d  r:%d",
                              item->name, (int)item->zoom_start, (int)item->zoom_end, (int)item->range);
    }
}

static void btn_type_event_handler(lv_event_t *e)
{
    uintptr_t index = (uintptr_t)lv_event_get_user_data(e);
    if (index >= (uintptr_t)ANIM_ITEM_COUNT) return;
    if (!anim_items[index].enabled) return;

    LOG_I("Button clicked: index=%d, animation=%s", (int)index, anim_items[index].name);
    current_type = anim_items[index].type;
    update_animation();
}

void lv_example_multanim(void)
{
    LOG_I("=== Multanim Example Started ===");

    lv_obj_t *scr = lv_scr_act();
    lv_coord_t hor = lv_disp_get_hor_res(NULL);
    lv_coord_t ver = lv_disp_get_ver_res(NULL);

    /* 底部控制区高度：避免挡住主要动画显示，同时按钮区域可滚动 */
    lv_coord_t ctrl_h = ver / 3;
    if (ctrl_h < 110) ctrl_h = 110;
    if (ctrl_h > 220) ctrl_h = 220;

    /* 1) 主动画区域 */
    multanim = lv_multanim_create(scr);
    lv_obj_set_size(multanim, hor, ver - ctrl_h);
    lv_obj_align(multanim, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(multanim, LV_OBJ_FLAG_SCROLLABLE);

    img1 = lv_img_create(multanim);
    LV_IMG_DECLARE(animimg001);
    lv_img_set_src(img1, &animimg001);
    lv_obj_center(img1);
    lv_obj_add_flag(img1, LV_OBJ_FLAG_HIDDEN);

    img2 = lv_img_create(multanim);
    LV_IMG_DECLARE(animimg003);
    lv_img_set_src(img2, &animimg003);
    lv_obj_center(img2);
    lv_obj_add_flag(img2, LV_OBJ_FLAG_HIDDEN);

    lv_multanim_set_major_img(multanim, img1);
    lv_multanim_set_minor_img(multanim, img2);
    lv_multanim_set_dir(multanim, LV_MULTANIM_HOR);

    /* mask 动画单独用自己的 mask 配置 */
    open_masks_init();
    lv_multanim_set_mask(multanim, NULL, NULL);

    /* 2) 底部控制面板 */
    lv_obj_t *ctrl = lv_obj_create(scr);
    lv_obj_set_size(ctrl, hor, ctrl_h);
    lv_obj_align(ctrl, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_clear_flag(ctrl, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(ctrl, 0, 0);
    lv_obj_set_style_radius(ctrl, 0, 0);
    lv_obj_set_style_bg_color(ctrl, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(ctrl, LV_OPA_50, 0);
    lv_obj_set_style_pad_all(ctrl, 8, 0);
    lv_obj_set_style_pad_row(ctrl, 6, 0);
    lv_obj_set_flex_flow(ctrl, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scrollbar_mode(ctrl, LV_SCROLLBAR_MODE_OFF);

    status_label = lv_label_create(ctrl);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_16, 0);
    lv_label_set_text(status_label, "Anim: Zoom");

    lv_obj_t *hint = lv_label_create(ctrl);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_opa(hint, LV_OPA_70, 0);
    lv_label_set_text(hint, "Gray: need VGLite / TODO");

    lv_obj_t *btn_wrap = lv_obj_create(ctrl);
    lv_obj_set_width(btn_wrap, LV_PCT(100));
    lv_obj_set_flex_grow(btn_wrap, 1);
    lv_obj_set_style_bg_opa(btn_wrap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_wrap, 0, 0);
    lv_obj_set_style_pad_all(btn_wrap, 0, 0);
    lv_obj_set_style_pad_row(btn_wrap, 6, 0);
    lv_obj_set_style_pad_column(btn_wrap, 6, 0);
    lv_obj_set_flex_flow(btn_wrap, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(btn_wrap, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_scroll_dir(btn_wrap, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(btn_wrap, LV_SCROLLBAR_MODE_AUTO);

    int cols = (hor >= 360) ? 4 : 3;
    lv_coord_t gap = 6;
    lv_coord_t btn_h = 34;
    lv_coord_t avail_w = hor - 16; /* ctrl 左右 padding=8 */
    lv_coord_t btn_w = (avail_w - (cols - 1) * gap) / cols;

    for (uint32_t i = 0; i < (uint32_t)ANIM_ITEM_COUNT; i++)
    {
        lv_obj_t *btn = lv_btn_create(btn_wrap);
        lv_obj_set_size(btn, btn_w, btn_h);
        if (anim_items[i].enabled)
        {
            lv_obj_add_event_cb(btn, btn_type_event_handler, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
        }
        else
        {
            lv_obj_add_state(btn, LV_STATE_DISABLED);
        }

        lv_obj_t *btn_label = lv_label_create(btn);
        lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_12, 0);
        lv_label_set_text(btn_label, anim_items[i].name);
        lv_obj_center(btn_label);
    }

    /* 默认启动一个不会触发额外依赖的动画 */
    current_type = LV_MULTANIM_ZOOM;
    update_animation();
    LOG_I("=== UI Setup Complete ===");
}
