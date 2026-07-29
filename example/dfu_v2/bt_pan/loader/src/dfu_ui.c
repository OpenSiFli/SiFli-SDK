/*
 * @file dfu_ui.c
 * @brief DFU V2 OTA UI — Dark ring-progress theme
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dfu_ui.h"
#include "littlevgl2rtt.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/*============================================================================
 * Color palette
 *============================================================================*/
#define CLR_BG          lv_color_hex(0x0A0A0A)  /* near-black background   */
#define CLR_RING_BG     lv_color_hex(0x1A1A1A)  /* ring track              */
#define CLR_RING_NORMAL lv_color_hex(0x00A0FF)  /* blue — transferring     */
#define CLR_RING_OK     lv_color_hex(0x00E676)  /* green — success         */
#define CLR_RING_ERR    lv_color_hex(0xFF5252)  /* red — failure           */
#define CLR_TEXT_DIM    lv_color_hex(0x555555)  /* muted text              */
#define CLR_TEXT_MID    lv_color_hex(0x888888)  /* status text             */
#define CLR_TEXT_HI     lv_color_hex(0xFFFFFF)  /* percentage number       */
#define CLR_SOURCE      lv_color_hex(0x444444)  /* source label            */
#define CLR_BT_ON       lv_color_hex(0x448AFF)  /* BT connected            */
#define CLR_NET_ON      lv_color_hex(0x00E676)  /* NET connected           */

/*============================================================================
 * Reference design size (matches physical LCD)
 *============================================================================*/
#define BASE_WIDTH  390
#define BASE_HEIGHT 450

/* Ring geometry (at scale 1.0) */
#define RING_DIAMETER  260
#define RING_WIDTH     18

/*============================================================================
 * UI objects
 *============================================================================*/
/* Top bar */
static lv_obj_t *s_bt_label  = NULL;   /* "BT" status text        */
static lv_obj_t *s_net_label = NULL;   /* "NET" status text       */

/* Central ring */
static lv_obj_t *s_ring      = NULL;   /* lv_arc                  */
static lv_obj_t *s_pct_label = NULL;   /* "67%" inside ring       */

/* Bottom text */
static lv_obj_t *s_status    = NULL;   /* "准备中..."              */
static lv_obj_t *s_source    = NULL;   /* "[CDC]"                 */

/* Message queue */
rt_mq_t ota_ui_msg_queue = RT_NULL;

/* Cached scale factor */
static float g_scale = 1.0f;

/* Chinese font */
extern const lv_font_t ota_font;

/*============================================================================
 * Helpers
 *============================================================================*/

static float get_scale_factor(void)
{
    lv_disp_t *disp = lv_disp_get_default();
    lv_coord_t w = lv_disp_get_hor_res(disp);
    lv_coord_t h = lv_disp_get_ver_res(disp);
    float sx = (float)w / BASE_WIDTH;
    float sy = (float)h / BASE_HEIGHT;
    return (sx < sy) ? sx : sy;
}

static void ui_free(char *str)
{
    if (str) rt_free(str);
}

/*============================================================================
 * Public: send message to UI queue (thread-safe)
 *============================================================================*/

void dfu_ui_update_message(ui_msg_type_t type, char *string)
{
    if (ota_ui_msg_queue == RT_NULL)
    {
        rt_kprintf("UI mq NULL, drop type=%d\n", type);
        return;
    }

    ui_msg_t *msg = (ui_msg_t *)rt_malloc(sizeof(ui_msg_t));
    if (msg == RT_NULL)
    {
        rt_kprintf("UI msg alloc fail type=%d\n", type);
        return;
    }

    msg->type = type;
    msg->data = string ? rt_strdup(string) : RT_NULL;

    rt_err_t ret = rt_mq_send(ota_ui_msg_queue, &msg, sizeof(ui_msg_t *));
    if (ret != RT_EOK)
    {
        rt_kprintf("UI mq send fail type=%d err=%d\n", type, ret);
        if (msg->data) rt_free(msg->data);
        rt_free(msg);
    }
}

/*============================================================================
 * UI construction
 *============================================================================*/

static rt_err_t ui_create(float scale)
{
    /* ── Styles ── */
    static lv_style_t st_pct;       /* big percentage number */
    static lv_style_t st_status;    /* status text           */
    static lv_style_t st_small;     /* BT/NET label, source  */

    lv_style_init(&st_pct);
    lv_style_init(&st_status);
    lv_style_init(&st_small);

    /* Pick font sizes by scale bracket */
    if (scale <= 0.6f)
    {
        lv_style_set_text_font(&st_pct,    lv_font_ubuntu_36);
        lv_style_set_text_font(&st_status, lv_font_ubuntu_16);
        lv_style_set_text_font(&st_small,  lv_font_ubuntu_12);
    }
    else if (scale <= 0.85f)
    {
        lv_style_set_text_font(&st_pct,    lv_font_ubuntu_56);
        lv_style_set_text_font(&st_status, lv_font_ubuntu_24);
        lv_style_set_text_font(&st_small,  lv_font_ubuntu_16);
    }
    else  /* scale ~1.0 for 390×450 */
    {
        lv_style_set_text_font(&st_pct,    lv_font_ubuntu_56);
        lv_style_set_text_font(&st_status, lv_font_ubuntu_24);
        lv_style_set_text_font(&st_small,  lv_font_ubuntu_16);
    }

    lv_style_set_text_align(&st_pct,    LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_align(&st_status, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_align(&st_small,  LV_TEXT_ALIGN_CENTER);

    /* ── Root screen ── */
    lv_obj_t *scr = lv_obj_create(lv_screen_active());
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(scr, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_set_style_bg_color(scr, CLR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_set_style_radius(scr, 0, 0);

    /* ── Top bar: BT + NET status labels ── */
    s_bt_label = lv_label_create(scr);
    lv_label_set_text(s_bt_label, "BT");
    lv_obj_add_style(s_bt_label, &st_small, 0);
    lv_obj_set_style_text_color(s_bt_label, CLR_TEXT_DIM, 0);
    lv_obj_set_style_text_letter_space(s_bt_label, (int)(2 * scale), 0);
    lv_obj_set_align(s_bt_label, LV_ALIGN_TOP_LEFT);
    lv_obj_set_x(s_bt_label, (int)(16 * scale));
    lv_obj_set_y(s_bt_label, (int)(16 * scale));

    /* NET status label (right of BT) */
    s_net_label = lv_label_create(scr);
    lv_label_set_text(s_net_label, "NET");
    lv_obj_add_style(s_net_label, &st_small, 0);
    lv_obj_set_style_text_color(s_net_label, CLR_TEXT_DIM, 0);
    lv_obj_set_style_text_letter_space(s_net_label, (int)(2 * scale), 0);
    lv_obj_set_align(s_net_label, LV_ALIGN_TOP_LEFT);
    lv_obj_set_x(s_net_label, (int)(70 * scale));
    lv_obj_set_y(s_net_label, (int)(16 * scale));

    /* ── Central arc (progress ring) ── */
    int ring_sz = (int)(RING_DIAMETER * scale);
    int arc_w   = (int)(RING_WIDTH * scale);
    if (arc_w < 8) arc_w = 8;

    s_ring = lv_arc_create(scr);
    lv_obj_set_size(s_ring, ring_sz, ring_sz);
    lv_obj_set_align(s_ring, LV_ALIGN_CENTER);
    lv_obj_set_y(s_ring, (int)(-25 * scale));   /* shift up slightly */

    lv_arc_set_rotation(s_ring, 270);            /* 12 o'clock start  */
    lv_arc_set_bg_angles(s_ring, 0, 360);
    lv_arc_set_range(s_ring, 0, 100);
    lv_arc_set_value(s_ring, 0);
    lv_obj_remove_style(s_ring, NULL, LV_PART_KNOB);
    lv_obj_remove_flag(s_ring, LV_OBJ_FLAG_CLICKABLE);

    /* Track (background) */
    lv_obj_set_style_arc_color(s_ring, CLR_RING_BG, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_ring, arc_w, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(s_ring, true, LV_PART_MAIN);

    /* Indicator (foreground) */
    lv_obj_set_style_arc_color(s_ring, CLR_RING_NORMAL, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(s_ring, arc_w, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(s_ring, true, LV_PART_INDICATOR);

    /* ── Percentage label (centered inside ring) ── */
    s_pct_label = lv_label_create(scr);
    lv_label_set_text(s_pct_label, "0%");
    lv_obj_add_style(s_pct_label, &st_pct, 0);
    lv_obj_set_style_text_color(s_pct_label, CLR_TEXT_HI, 0);
    lv_obj_set_align(s_pct_label, LV_ALIGN_CENTER);
    lv_obj_set_y(s_pct_label, (int)(-25 * scale));  /* same as ring */

    /* ── Status text (below ring) ── */
    s_status = lv_label_create(scr);
    lv_obj_set_width(s_status, (int)(300 * scale));
    lv_obj_set_style_text_align(s_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(s_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(s_status, &ota_font, 0);  /* Chinese font */
    lv_obj_set_style_text_color(s_status, CLR_TEXT_MID, 0);
    lv_obj_set_align(s_status, LV_ALIGN_CENTER);
    lv_obj_set_y(s_status, (int)(120 * scale));

    /* Use ota_font for Chinese text "准备中..." */
    lv_label_set_text(s_status, "准备中...");

    /* ── Source label ── */
    s_source = lv_label_create(scr);
    lv_label_set_text(s_source, "");
    lv_obj_add_style(s_source, &st_small, 0);
    lv_obj_set_style_text_color(s_source, CLR_SOURCE, 0);
    lv_obj_set_style_text_letter_space(s_source, (int)(2 * scale), 0);
    lv_obj_set_align(s_source, LV_ALIGN_CENTER);
    lv_obj_set_y(s_source, (int)(155 * scale));

    return RT_EOK;
}

/*============================================================================
 * Message handlers
 *============================================================================*/

static void handle_ble(const char *data)
{
    if (!data || !s_bt_label) return;

    bool on = (strcmp(data, UI_MSG_DATA_BLE_CONNECTED) == 0);
    lv_obj_set_style_text_color(s_bt_label, on ? CLR_BT_ON : CLR_TEXT_DIM, 0);
}

static void handle_net(const char *data)
{
    if (!data || !s_net_label) return;

    bool on = (strcmp(data, UI_MSG_DATA_NET_CONNECTED) == 0);
    lv_obj_set_style_text_color(s_net_label, on ? CLR_NET_ON : CLR_TEXT_DIM, 0);
}

static void handle_progress(const char *data)
{
    if (!data) return;

    int pct = atoi(data);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    if (s_ring)
    {
        lv_arc_set_value(s_ring, pct);
    }

    if (s_pct_label)
    {
        char buf[8];
        rt_snprintf(buf, sizeof(buf), "%d%%", pct);
        lv_label_set_text(s_pct_label, buf);
    }

    rt_kprintf("PROGRESS: %d%%\n", pct);
}

static void handle_progress_color(const char *data)
{
    if (!data || !s_ring) return;

    if (strcmp(data, PROGRESS_COLOR_NORMAL) == 0)
    {
        lv_obj_set_style_arc_color(s_ring, CLR_RING_NORMAL, LV_PART_INDICATOR);
    }
    else if (strcmp(data, PROGRESS_COLOR_SUCCESS) == 0)
    {
        lv_obj_set_style_arc_color(s_ring, CLR_RING_OK, LV_PART_INDICATOR);
    }
    else if (strcmp(data, PROGRESS_COLOR_ERROR) == 0)
    {
        lv_obj_set_style_arc_color(s_ring, CLR_RING_ERR, LV_PART_INDICATOR);
    }
}

static void handle_status_text(const char *data)
{
    if (data && s_status)
    {
        lv_label_set_text(s_status, data);
    }
}

static void handle_source(const char *data)
{
    if (!s_source) return;

    if (data == NULL || strcmp(data, UI_MSG_DATA_SOURCE_NONE) == 0)
    {
        lv_label_set_text(s_source, "");
    }
    else if (strcmp(data, UI_MSG_DATA_SOURCE_PAN) == 0)
    {
        lv_label_set_text(s_source, "[PAN]");
        lv_obj_set_style_text_color(s_source, CLR_RING_NORMAL, 0);
    }
}

static void handle_success(void)
{
    if (s_ring)
    {
        lv_arc_set_value(s_ring, 100);
        lv_obj_set_style_arc_color(s_ring, CLR_RING_OK, LV_PART_INDICATOR);
    }

    if (s_pct_label)
    {
        lv_label_set_text(s_pct_label, "100%");
        lv_obj_set_style_text_color(s_pct_label, CLR_RING_OK, 0);
    }

    if (s_status)
    {
        lv_label_set_text(s_status, "更新成功");
        lv_obj_set_style_text_color(s_status, CLR_RING_OK, 0);
    }

    rt_kprintf("OTA Success\n");
}

static void handle_failure(const char *data)
{
    if (s_ring)
    {
        lv_obj_set_style_arc_color(s_ring, CLR_RING_ERR, LV_PART_INDICATOR);
    }

    if (s_pct_label)
    {
        lv_obj_set_style_text_color(s_pct_label, CLR_RING_ERR, 0);
    }

    if (s_status)
    {
        lv_label_set_text(s_status, data ? data : "更新失败");
        lv_obj_set_style_text_color(s_status, CLR_RING_ERR, 0);
    }

    rt_kprintf("OTA Failure\n");
}

/*============================================================================
 * UI task entry (called from ui_thread_entry in main.c)
 *============================================================================*/

void dfu_ui_task(void *args)
{
    rt_kprintf("dfu_ui_task start\n");
    (void)args;

    /* Init LVGL display driver */
    rt_err_t ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        rt_kprintf("littlevGL init failed\n");
        return;
    }

    /* Message queue */
    ota_ui_msg_queue = rt_mq_create("ota_ui_msg", sizeof(ui_msg_t *), 20,
                                    RT_IPC_FLAG_FIFO);
    if (ota_ui_msg_queue == RT_NULL)
    {
        rt_kprintf("Failed to create UI mq\n");
        return;
    }

    /* Compute scale and build UI */
    g_scale = get_scale_factor();
    ret = ui_create(g_scale);
    if (ret != RT_EOK)
    {
        rt_kprintf("UI create failed\n");
        return;
    }

    /* Event loop */
    while (1)
    {
        ui_msg_t *msg;
        while (rt_mq_recv(ota_ui_msg_queue, &msg, sizeof(ui_msg_t *), 0)
               == RT_EOK)
        {
            if (msg == RT_NULL)
                continue;

            switch (msg->type)
            {
            case UI_MSG_UPDATE_PROGRESS:
                handle_progress(msg->data);
                break;

            case UI_MSG_UPDATE_PROGRESS_COLOR:
                handle_progress_color(msg->data);
                break;

            case UI_MSG_UPDATE_FILES:
                handle_status_text(msg->data);
                break;

            case UI_MSG_UPDATE_SOURCE:
                handle_source(msg->data);
                break;

            case UI_MSG_SHOW_SUCCESS_POPUP:
                handle_success();
                break;

            case UI_MSG_SHOW_FAILURE_POPUP:
                handle_failure(msg->data);
                break;

            /* BLE / NET status */
            case UI_MSG_UPDATE_BLE:
                handle_ble(msg->data);
                break;

            case UI_MSG_UPDATE_NET:
                handle_net(msg->data);
                break;

            case UI_MSG_UPDATE_USB:
            case UI_MSG_UPDATE_BUTTON:
            default:
                break;
            }

            if (msg->data)
                ui_free(msg->data);
            rt_free(msg);
        }

        rt_uint32_t ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
}