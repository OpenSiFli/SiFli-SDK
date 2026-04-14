/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/

#include "demo_multlist_cards.h"

#define _MODULE_NAME_ "multlist"
#include "app_module.h"

#define DEMO_MULTLIST_LIST_ITEM_EDGE 110
#define DEMO_MULTLIST_IMAGE_COUNT (sizeof(s_image_src) / sizeof(s_image_src[0]))

static const void *s_image_src[] =
{
    APP_GET_IMG(demo_list_0),
    APP_GET_IMG(demo_list_1),
    APP_GET_IMG(demo_list_2),
    APP_GET_IMG(demo_list_3),
};

static lv_img_dsc_t *s_card_cache[DEMO_MULTLIST_CARD_STYLE_COUNT][DEMO_MULTLIST_IMAGE_COUNT];
static lv_coord_t s_card_w[DEMO_MULTLIST_CARD_STYLE_COUNT];
static lv_coord_t s_card_h[DEMO_MULTLIST_CARD_STYLE_COUNT];
static bool s_card_size_ready = false;

static bool demo_multlist_get_src_size(const void *src, lv_coord_t *w, lv_coord_t *h)
{
    lv_img_header_t header;

    if(src == NULL) return false;
    if(lv_img_decoder_get_info(src, &header) != LV_RES_OK) return false;
    if(header.w == 0 || header.h == 0) return false;

    if(w) *w = header.w;
    if(h) *h = header.h;
    return true;
}

static void demo_multlist_fit_size(const void *src,
                                   lv_coord_t max_w,
                                   lv_coord_t max_h,
                                   lv_coord_t *out_w,
                                   lv_coord_t *out_h)
{
    lv_coord_t src_w;
    lv_coord_t src_h;
    uint32_t scale_w;
    uint32_t scale_h;
    uint32_t scale;

    if(max_w < 1) max_w = 1;
    if(max_h < 1) max_h = 1;

    if(!demo_multlist_get_src_size(src, &src_w, &src_h))
    {
        if(out_w) *out_w = max_w;
        if(out_h) *out_h = max_h;
        return;
    }

    scale_w = (uint32_t)max_w * 1024U / (uint32_t)src_w;
    scale_h = (uint32_t)max_h * 1024U / (uint32_t)src_h;
    scale = LV_MIN(scale_w, scale_h);
    if(scale == 0U) scale = 1U;

    if(out_w) *out_w = LV_MAX((lv_coord_t)(((uint32_t)src_w * scale) / 1024U), 1);
    if(out_h) *out_h = LV_MAX((lv_coord_t)(((uint32_t)src_h * scale) / 1024U), 1);
}

static uint16_t demo_multlist_fit_zoom(const void *src, lv_coord_t max_w, lv_coord_t max_h)
{
    lv_coord_t src_w;
    lv_coord_t src_h;
    uint32_t zoom_w;
    uint32_t zoom_h;
    uint32_t zoom;

    if(max_w < 1) max_w = 1;
    if(max_h < 1) max_h = 1;

    if(!demo_multlist_get_src_size(src, &src_w, &src_h)) return LV_IMG_ZOOM_NONE;

    zoom_w = (uint32_t)max_w * LV_IMG_ZOOM_NONE / (uint32_t)src_w;
    zoom_h = (uint32_t)max_h * LV_IMG_ZOOM_NONE / (uint32_t)src_h;
    zoom = LV_MIN(zoom_w, zoom_h);
    if(zoom == 0U) zoom = 1U;

    return (uint16_t)zoom;
}

static void demo_multlist_free_card_cache(void)
{
    uint32_t style;
    uint32_t index;

    for(style = 0; style < DEMO_MULTLIST_CARD_STYLE_COUNT; style++)
    {
        for(index = 0; index < DEMO_MULTLIST_IMAGE_COUNT; index++)
        {
            if(s_card_cache[style][index] != NULL)
            {
                app_cache_img_free(s_card_cache[style][index]);
                s_card_cache[style][index] = NULL;
            }
        }
    }
}

static void demo_multlist_calc_card_sizes(void)
{
    lv_coord_t scr_w = lv_disp_get_hor_res(NULL);
    lv_coord_t scr_h = lv_disp_get_ver_res(NULL);
    lv_coord_t gap = LV_MAX(LV_MIN(scr_w, scr_h) / 36, 6);
    lv_coord_t list_max_w = LV_MIN(DEMO_MULTLIST_LIST_ITEM_EDGE - 18, (scr_w - gap * 4) / 3);
    lv_coord_t list_max_h = LV_MIN(DEMO_MULTLIST_LIST_ITEM_EDGE - 18, (scr_h - gap * 4) / 3);
    lv_coord_t anim_max_w = LV_MAX(scr_w * 3 / 5, 1);
    lv_coord_t anim_max_h = LV_MAX(scr_h * 3 / 5, 1);
    lv_coord_t new_w;
    lv_coord_t new_h;

    demo_multlist_fit_size(s_image_src[0], list_max_w, list_max_h, &new_w, &new_h);
    s_card_w[DEMO_MULTLIST_CARD_STYLE_LIST] = new_w;
    s_card_h[DEMO_MULTLIST_CARD_STYLE_LIST] = new_h;

    demo_multlist_fit_size(s_image_src[0], anim_max_w, anim_max_h, &new_w, &new_h);
    s_card_w[DEMO_MULTLIST_CARD_STYLE_ANIM] = new_w;
    s_card_h[DEMO_MULTLIST_CARD_STYLE_ANIM] = new_h;
}

static lv_img_dsc_t *demo_multlist_create_card_snapshot(demo_multlist_card_style_t style,
                                                        const void *src)
{
    lv_coord_t card_w = s_card_w[style];
    lv_coord_t card_h = s_card_h[style];
    lv_coord_t pad = (style == DEMO_MULTLIST_CARD_STYLE_ANIM) ? 6 : 4;
    lv_img_dsc_t *dsc;
    lv_obj_t *card;
    lv_obj_t *img;

    if(src == NULL || card_w <= 0 || card_h <= 0) return NULL;

    card = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(card);
    lv_obj_set_size(card, card_w, card_h);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_bg_color(card, lv_color_make(24, 32, 48), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 0, 0);

    img = lv_img_create(card);
    lv_img_set_size_mode(img, LV_IMG_SIZE_MODE_REAL);
    lv_img_set_src(img, src);
    lv_img_set_zoom(img, demo_multlist_fit_zoom(src,
                                                LV_MAX(card_w - pad * 2, 1),
                                                LV_MAX(card_h - pad * 2, 1)));
    lv_obj_refr_size(img);
    lv_obj_center(img);
    lv_obj_update_layout(card);

    dsc = app_cache_img_alloc(card_w, card_h, LV_IMG_CF_TRUE_COLOR, 0, CACHE_PSRAM);
    if(dsc != NULL)
    {
        rt_memset((void *)dsc->data, 0, dsc->data_size);
        lv_snapshot_obj_to_dsc(card, &card->coords, dsc);
    }

    lv_obj_del(card);
    return dsc;
}

void demo_multlist_card_cache_init(void)
{
    lv_coord_t old_list_w = s_card_w[DEMO_MULTLIST_CARD_STYLE_LIST];
    lv_coord_t old_list_h = s_card_h[DEMO_MULTLIST_CARD_STYLE_LIST];
    lv_coord_t old_anim_w = s_card_w[DEMO_MULTLIST_CARD_STYLE_ANIM];
    lv_coord_t old_anim_h = s_card_h[DEMO_MULTLIST_CARD_STYLE_ANIM];

    demo_multlist_calc_card_sizes();
    if(!s_card_size_ready ||
       old_list_w != s_card_w[DEMO_MULTLIST_CARD_STYLE_LIST] ||
       old_list_h != s_card_h[DEMO_MULTLIST_CARD_STYLE_LIST] ||
       old_anim_w != s_card_w[DEMO_MULTLIST_CARD_STYLE_ANIM] ||
       old_anim_h != s_card_h[DEMO_MULTLIST_CARD_STYLE_ANIM])
    {
        demo_multlist_free_card_cache();
    }

    s_card_size_ready = true;
}

void demo_multlist_card_cache_deinit(void)
{
    demo_multlist_free_card_cache();
    s_card_size_ready = false;
}

uint32_t demo_multlist_get_image_count(void)
{
    return DEMO_MULTLIST_IMAGE_COUNT;
}

const void *demo_multlist_get_image_src(uint32_t index)
{
    return s_image_src[index % DEMO_MULTLIST_IMAGE_COUNT];
}

const lv_img_dsc_t *demo_multlist_get_card_snapshot(demo_multlist_card_style_t style,
                                                    uint32_t index)
{
    uint32_t slot;

    if(style >= DEMO_MULTLIST_CARD_STYLE_COUNT) return NULL;

    if(!s_card_size_ready)
    {
        demo_multlist_card_cache_init();
    }

    slot = index % DEMO_MULTLIST_IMAGE_COUNT;
    if(s_card_cache[style][slot] == NULL)
    {
        s_card_cache[style][slot] = demo_multlist_create_card_snapshot(style, s_image_src[slot]);
    }

    return s_card_cache[style][slot];
}

void demo_multlist_get_card_size(demo_multlist_card_style_t style,
                                 lv_coord_t *w,
                                 lv_coord_t *h)
{
    if(!s_card_size_ready)
    {
        demo_multlist_card_cache_init();
    }

    if(style >= DEMO_MULTLIST_CARD_STYLE_COUNT)
    {
        if(w) *w = 0;
        if(h) *h = 0;
        return;
    }

    if(w) *w = s_card_w[style];
    if(h) *h = s_card_h[style];
}
