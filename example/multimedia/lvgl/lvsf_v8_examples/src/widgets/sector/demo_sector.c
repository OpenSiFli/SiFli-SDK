/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"               /* pulls lvsf_conf_internal.h -> LVSF_USE_* macros */
#include "lvsf_sector.h"
#include "lvsf_obj_ext.h"       /* lv_obj_set_range_scale / lv_obj_set_range_value */
#include "demo_sector.h"

/*********************
 *      DEFINES
 *********************/
#define SECTOR_SZ 140           /* sector (and source image) size, px */

/**********************
 *  STATIC FUNCTIONS
 **********************/

/* Fill a buffer with a solid color and wrap a true-color image descriptor over
 * it -- lets the demo build the source image at runtime without an asset. */
static void make_solid_img(lv_img_dsc_t *dsc, lv_color_t *buf, lv_coord_t sz, lv_color_t color)
{
    for (int i = 0; i < (int)(sz * sz); i++)
        buf[i] = color;
    dsc->header.cf = LV_IMG_CF_TRUE_COLOR;
    dsc->header.always_zero = 0;
    dsc->header.reserved = 0;
    dsc->header.w = sz;
    dsc->header.h = sz;
    dsc->data_size = (uint32_t)(sz * sz) * sizeof(lv_color_t);
    dsc->data = (const uint8_t *)buf;
}

/* Sweep the value 0..100 and back so the wedge grows then shrinks;
 * lv_sector_set_value rebuilds the angular mask each tick. */
static void sector_ramp_cb(lv_timer_t *t)
{
    lv_obj_t *sector = (lv_obj_t *)t->user_data;
    static int v = 0;
    static int step = 2;
    v += step;
    if (v >= 100) { v = 100; step = -2; }
    else if (v <= 0) { v = 0; step = 2; }
    lv_sector_set_value(sector, v);
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_sector.
 *
 * sector reveals an image through an angular (pie-wedge) mask: the mapped value
 * sets how much of the circle is shown. It is built on lv_img, so set the source
 * image with lv_img_set_src; map the value with lv_obj_set_range_scale (angle
 * span) and lv_obj_set_range_value (value range); call lv_sector_validate to
 * allocate the mask, then drive lv_sector_set_value. The image opacity is set
 * just below LV_OPA_COVER so the base lv_img cover-check reports NOT_COVER and
 * the parent repaints under the masked-out region (otherwise stale pixels show).
 */
void demo_sector_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "sector");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    static lv_color_t buf[SECTOR_SZ * SECTOR_SZ];
    static lv_img_dsc_t dsc;
    make_solid_img(&dsc, buf, SECTOR_SZ, lv_palette_main(LV_PALETTE_ORANGE));

    lv_obj_t *sector = lv_sector_create(scr);
    lv_img_set_src(sector, &dsc);
    lv_obj_set_size(sector, SECTOR_SZ, SECTOR_SZ);
    lv_obj_center(sector);
    lv_obj_set_style_img_opa(sector, LV_OPA_COVER - 1, LV_PART_MAIN);
    lv_obj_set_range_scale(sector, 0, 360);    /* angle span: full circle */
    lv_obj_set_range_value(sector, 0, 100);    /* value range mapped onto the span */
    lv_sector_validate(sector);                /* allocate the angular mask buffer */

    lv_timer_create(sector_ramp_cb, 60, sector);
}
