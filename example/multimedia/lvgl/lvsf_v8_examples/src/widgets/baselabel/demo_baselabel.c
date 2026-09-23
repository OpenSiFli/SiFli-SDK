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
#include "lvsf_baselabel.h"
#include "lvsf_obj_ext.h"       /* lv_obj_set_gmdata_cb / set_source_id / create_refresh_timer */
#include "demo_baselabel.h"

/**********************
 *  STATIC FUNCTIONS
 **********************/

/* Data callback: the refresh timer calls this with the bound source id(s); a
 * real app would read those data sources, here we just track elapsed seconds.
 * The callback formats the value into the label -- this is how baselabel turns
 * a live data source into displayed text. */
static int32_t uptime_gmdata_cb(lv_obj_t *label, uint32_t *id_tab, uint8_t id_num)
{
    (void)id_tab;
    (void)id_num;
    static uint32_t secs = 0;
    secs++;
    char buf[32];
    lv_snprintf(buf, sizeof(buf), "uptime  %02u:%02u",
                (unsigned)((secs / 60) % 100), (unsigned)(secs % 60));
    lv_baselabel_set_text(label, buf);
    return 0;
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_baselabel.
 *
 * baselabel is a label with a built-in data-refresh path: bind one or more data
 * source ids, give it a data callback and a refresh timer, and it periodically
 * pulls the data and updates its text -- so the label tracks live values without
 * the app polling. (It also keeps the usual lv_label features: set_text/_fmt,
 * long modes, recolor, a string table, etc.) This demo shows an uptime that
 * ticks every second.
 */
void demo_baselabel_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "baselabel");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t *label = lv_baselabel_create(scr);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_36, 0);
    lv_obj_set_style_text_color(label, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_baselabel_set_text(label, "uptime  00:00");   /* initial text before first refresh */
    lv_obj_center(label);

    /* data-driven refresh: bind a source id, a data callback and a 1 s timer */
    static uint32_t source_id = 0x1105;
    lv_obj_set_source_id(label, &source_id, 1);
    lv_obj_set_gmdata_cb(label, uptime_gmdata_cb);
    lv_obj_create_refresh_timer(label, 1000, lv_baselabel_refresh_timer);
    lv_obj_refresh_start(label);
}
