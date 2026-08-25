/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lvsf_txtanim.h
 *
 */

#ifndef LVSF_TXTANIM_H
#define LVSF_TXTANIM_H

#include "lvgl.h"
#include "lvsf_conf_internal.h"
#include "rtthread.h"

#if LVSF_USE_TXTANIM != 0

#ifdef __cplusplus
extern "C" {
#endif

/* Color format used by *_ex APIs: 0xAARRGGBB */
#define LV_TXTANIM_COLOR32(a, r, g, b) \
    ((((uint32_t)(a) & 0xFF) << 24) | (((uint32_t)(r) & 0xFF) << 16) | \
     (((uint32_t)(g) & 0xFF) << 8) | ((uint32_t)(b) & 0xFF))

#define LV_TXTANIM_COLOR_RGB(r, g, b) LV_TXTANIM_COLOR32(0xFF, r, g, b)

enum
{
    LV_TXT_ANIM_BASE = 0,
    LV_TXT_ANIM_ONE_BY_ONE,
    LV_TXT_ANIM_LIT_BY_LIT,
    LV_TXT_ANIM_SCROLL,
    LV_TXT_ANIM_CUSHION,
    LV_TXT_ANIM_MIRROR,
    LV_TXT_ANIM_SIN,
    LV_TXT_ANIM_MAX
};
typedef uint16_t lv_txtanim_type_t;

enum
{
    LV_TXT_FLAG_POS = (1 << 0),
    LV_TXT_FLAG_LOOP = (1 << 1),
    LV_TXT_FLAG_OPA = (1 << 2),
    LV_TXT_FLAG_ZOOM = (1 << 3),
    LV_TXT_FLAG_SIZE = (1 << 4),
    LV_TXT_FLAG_ROTATE = (1 << 5),
    LV_TXT_FLAG_CHANGE_DIR = (1 << 6),
    LV_TXT_FLAG_RECOLOR = (1 << 7),
    LV_TXT_FLAG_EASE_IN = (1 << 8),
    LV_TXT_FLAG_EASE_OUT = (1 << 9),
};
typedef uint16_t lv_txtanim_flag_t;

enum
{
    LV_TXT_MODE_LOOP,
    LV_TXT_MODE_VER,
    LV_TXT_MODE_VER_SWAP,
    LV_TXT_MODE_HOR_SWAP,
    LV_TXT_MODE_HOR_SWAP_CENTER
};
typedef uint16_t lv_txtanim_mode_t;

typedef struct
{
    lv_opa_t opa;
    lv_coord_t x;
    lv_coord_t y;
    lv_coord_t w;
    lv_coord_t h;
    lv_point_t pivot;
    lv_coord_t zoom;
    lv_coord_t angle;
} lv_txtanim_para_t;

typedef void *(*lv_txtanim_alloc_cb)(size_t size);
typedef void *(*lv_txtanim_calloc_cb)(rt_size_t count, rt_size_t size);
typedef void (*lv_txtanim_free_cb)(void *ptr);

typedef struct
{
    lv_obj_t base;
    uint8_t *buf;
    uint32_t buf_size;
    lv_ll_t list;
    uint16_t percent;
    uint16_t ch_cnt;
    uint16_t w_max;
    uint16_t w;
    uint16_t h;
    uint16_t mirror_cnt;
    uint16_t mirror_type;
    lv_coord_t mirror_gap;
    lv_txtanim_para_t org;
    lv_txtanim_para_t dst;
    lv_txtanim_type_t type;
    lv_txtanim_flag_t flage;
    bool is_ver;
    lv_txtanim_alloc_cb lv_txtanim_alloc;
    lv_txtanim_calloc_cb lv_txtanim_calloc;
    lv_txtanim_free_cb lv_txtanim_free;
} lv_txtanim_t;

typedef struct
{
    lv_obj_t *snapshot;
    lv_img_dsc_t dsc;
    lv_coord_t offset_x;
    lv_coord_t offset_y;
    lv_coord_t org_x;
    lv_coord_t org_y;
    uint32_t letter;
    uint16_t ch_start;
    uint16_t ch_w;
    uint16_t index;
    uint16_t layout;
} lv_txt_char_t;

extern const lv_obj_class_t lv_txtanim_class;

enum
{
    LV_TXT_PART_ORG_X = (1 << 0),
    LV_TXT_PART_ORG_Y = (1 << 1),
    LV_TXT_PART_ORG = (3 << 0),
    LV_TXT_PART_DST_X = (1 << 2),
    LV_TXT_PART_DST_Y = (1 << 3),
    LV_TXT_PART_DST = (3 << 2),
    LV_TXT_PART_BOTH = 0xF,
    LV_TXT_PART_CHART = (1 << 4),
};
typedef uint8_t lv_txtanim_part_t;

enum
{
    LV_TXT_ALIGN_TOP_MID,
    LV_TXT_ALIGN_TOP_LEFT,
    LV_TXT_ALIGN_TOP_RIGHT,
    LV_TXT_ALIGN_CENTER,
    LV_TXT_ALIGN_LEFT_MID,
    LV_TXT_ALIGN_RIGHT_MID,
    LV_TXT_ALIGN_BOTTON_MID,
    LV_TXT_ALIGN_BOTTON_LEFT,
    LV_TXT_ALIGN_BOTTON_RIGHT,
    LV_TXT_ALIGN_HEAD,
    LV_TXT_ALIGN_TAIL,
    LV_TXT_ALIGN_RAND_OUT,
    LV_TXT_ALIGN_RAND_IN,
    LV_TXT_ALIGN_RAND,
};
typedef uint16_t lv_txtanim_align_t;

enum
{
    LV_TXTANIM_MIRROR_HEAD,
    LV_TXTANIM_MIRROR_TAIL,
    LV_TXTANIM_MIRROR_CENTER,
    LV_TXTANIM_MIRROR_HIDDEN
};
typedef uint16_t lv_txtanim_mirror_t;

lv_obj_t *lv_txtanim_create(lv_obj_t *parent);
uint32_t lv_txtanim_draw_txt_seprate(lv_obj_t *txtanim, const char *txt,
                                      uint32_t len, lv_color_t color,
                                      const lv_font_t *font);
uint32_t lv_txtanim_draw_txt_seprate_ex(lv_obj_t *txtanim, const char *txt,
                                         uint32_t len, uint32_t color,
                                         const lv_font_t *font);
uint32_t lv_txtanim_draw_txt_whole(lv_obj_t *txtanim, const char *txt,
                                    uint32_t len, lv_color_t color,
                                    const lv_font_t *font, uint16_t pad_w,
                                    uint16_t pad_h, uint8_t hollow);
uint32_t lv_txtanim_draw_txt_whole_ex(lv_obj_t *txtanim, const char *txt,
                                       uint32_t len, uint32_t color,
                                       const lv_font_t *font, uint16_t pad_w,
                                       uint16_t pad_h, uint8_t hollow);
void lv_txtanim_draw_txt(lv_img_dsc_t *dsc, lv_coord_t x, lv_coord_t y,
                         lv_font_glyph_dsc_t *glyph, const uint8_t *src,
                         lv_color_t color, uint8_t is_hollow);
void lv_txtanim_draw_txt_ext(lv_img_dsc_t *dsc, lv_coord_t x, lv_coord_t y,
                             lv_font_glyph_dsc_t *glyph, const uint8_t *src,
                             uint32_t color, uint8_t is_hollow);
void lv_txtanim_set_type(lv_obj_t *txtanim, lv_txtanim_type_t type,
                         lv_txtanim_flag_t flag);
void lv_txtanim_reset_layout(lv_obj_t *txtanim, lv_coord_t w, lv_coord_t h,
                             lv_coord_t gap, lv_txtanim_mode_t mode);
void lv_txtanim_set_pos(lv_obj_t *txtanim, lv_coord_t org_x, lv_coord_t org_y,
                        lv_coord_t dst_x, lv_coord_t dst_y);
void lv_txtanim_align(lv_obj_t *txtanim, lv_txtanim_part_t part,
                      lv_txtanim_align_t align, lv_coord_t x, lv_coord_t y);
void lv_txtanim_clone_item(lv_obj_t *txtanim, lv_obj_t *copy);
void lv_txtanim_set_mirror(lv_obj_t *txtanim, uint16_t cnt, lv_coord_t gap,
                           lv_txtanim_mirror_t mirror);
void lv_txtanim_set_pos_rel(lv_obj_t *txtanim, lv_txtanim_part_t part,
                            lv_coord_t offset_x, lv_coord_t offset_y);
void lv_txtanim_pos_change(lv_obj_t *txtanim);
lv_txtanim_para_t *lv_txtanim_get_para(lv_obj_t *txtanim, bool org);
void lv_txtanim_set_size(lv_obj_t *txtanim, lv_coord_t org_w, lv_coord_t org_h,
                         lv_coord_t dst_w, lv_coord_t dst_h);
void lv_txtanim_set_percent(lv_obj_t *txtanim, uint8_t percent);
void lv_txtanim_set_opa(lv_obj_t *txtanim, lv_opa_t org_opa, lv_opa_t dst_opa);
void lv_txtanim_set_zoom(lv_obj_t *txtanim, lv_coord_t org_zoom,
                         lv_coord_t dst_zoom);
void lv_txtanim_set_pivot(lv_obj_t *txtanim, lv_coord_t org_x, lv_coord_t org_y,
                          lv_coord_t dst_x, lv_coord_t dst_y);
lv_txtanim_type_t lv_txtanim_get_type(lv_obj_t *txtanim);
lv_coord_t lv_txtanim_get_max_width(lv_obj_t *txtanim);
lv_coord_t lv_txtanim_get_txt_width(lv_obj_t *txtanim);
lv_coord_t lv_txtanim_get_txt_height(lv_obj_t *txtanim);
void lv_txtanim_anim_progess(lv_obj_t *txtanim, int32_t process);
void lv_txtanim_start_anim(lv_obj_t *txtanim, uint32_t start, uint32_t end,
                           uint32_t time, uint32_t delay_time, bool play_back,
                           bool loop, lv_anim_ready_cb_t ready_cb);
void lv_txtanim_set_mem_cb(lv_obj_t *txtanim,
                           lv_txtanim_calloc_cb calloc_cb,
                           lv_txtanim_alloc_cb alloc_cb,
                           lv_txtanim_free_cb free_cb);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LVSF_USE_TXTANIM */

#endif /* LVSF_TXTANIM_H */
