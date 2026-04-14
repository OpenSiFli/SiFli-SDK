/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dfu_pan_ui.h"
#include "bt_pan_ota.h"
#include "littlevgl2rtt.h"
#include "lv_obj_pos.h"
#include "register.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
lv_obj_t *uic_Image1;

lv_obj_t *ui_emoji;
lv_obj_t *ui_ble;
lv_obj_t *ui_network;
lv_obj_t *ui_progress;
lv_obj_t *ui_prolabel;
lv_obj_t *ui_status;
lv_obj_t *Update_sign;

lv_obj_t *version_list_container = NULL;
lv_obj_t *close_button = NULL;
lv_obj_t *close_button_label = NULL;

static lv_obj_t *confirm_popup = NULL;
static lv_obj_t *confirm_label = NULL;
static lv_obj_t *confirm_btn = NULL;
static lv_obj_t *cancel_btn = NULL;
static lv_obj_t *confirm_btn_label = NULL;
int selected_version_index = -1;
// Image resources
extern const lv_image_dsc_t ota_emoji;
extern const lv_image_dsc_t ota_ble;
extern const lv_image_dsc_t ota_ble_close;
extern const lv_image_dsc_t ota_network_icon_img;
extern const lv_image_dsc_t ota_network_icon_img_close;
rt_mq_t ota_ui_msg_queue = RT_NULL;
static float g_scale = 1.0f;
static char g_confirm_text_buffer[128];
// Screen Reference value
#define BASE_WIDTH 390
#define BASE_HEIGHT 450

static float get_scale_factor(void)
{
    lv_disp_t *disp = lv_disp_get_default();
    lv_coord_t scr_width = lv_disp_get_hor_res(disp);
    lv_coord_t scr_height = lv_disp_get_ver_res(disp);

    float scale_x = (float)scr_width / BASE_WIDTH;
    float scale_y = (float)scr_height / BASE_HEIGHT;

    return (scale_x < scale_y) ? scale_x : scale_y;
}
static void ui_free(char *str)
{
    if (str)
    {
        rt_free(str);
    }
}

// Message queue update
void dfu_pan_ui_update_message(ui_msg_type_t type, char *string)
{
    if (ota_ui_msg_queue != RT_NULL)
    {
        ui_msg_t *msg = (ui_msg_t *)rt_malloc(sizeof(ui_msg_t));
        if (msg != RT_NULL)
        {
            msg->type = type;
            msg->data = rt_strdup(string);
            rt_err_t result =
                rt_mq_send(ota_ui_msg_queue, &msg, sizeof(ui_msg_t *));
            if (result != RT_EOK)
            {
                rt_kprintf("Failed to send UI message: type=%d, error=%d\n",
                           type, result);
                rt_free(msg->data);
                rt_free(msg);
            }
            else
            {
                rt_kprintf("send success: %d\n", result);
            }
        }
        else
        {
            rt_kprintf("Failed to allocate memory for UI message: type=%d\n",
                       type);
        }
    }
    else
    {
        rt_kprintf("UI message queue is NULL, message not sent: type=%d\n",
                   type);
    }
}

extern const lv_font_t ota_font;
rt_err_t ota_ui_obj_init(float scale)
{
    static lv_style_t style;

    lv_style_init(&style);
    if (scale <= 0.5f)
    {
        lv_style_set_text_font(&style, lv_font_ubuntu_12);
    }
    else if (scale <= 0.7f)
    {
        lv_style_set_text_font(&style, lv_font_ubuntu_16);
    }
    else if (scale <= 1.0f)
    {
        lv_style_set_text_font(&style, lv_font_ubuntu_24);
    }
    else if (scale <= 1.3f)
    {
        lv_style_set_text_font(&style, lv_font_ubuntu_36);
    }
    else if (scale <= 1.7f)
    {
        lv_style_set_text_font(&style, lv_font_ubuntu_56);
    }
    else
    {
        lv_style_set_text_font(&style, lv_font_ubuntu_72);
    }
    lv_style_set_text_align(&style, LV_TEXT_ALIGN_CENTER);

    LV_IMAGE_DECLARE(ota_emoji);
    LV_IMAGE_DECLARE(ota_ble_close);
    LV_IMAGE_DECLARE(ota_ble);
    LV_IMAGE_DECLARE(ota_network_icon_img);
    LV_IMAGE_DECLARE(ota_network_icon_img_close);

    lv_obj_t *ui_Screen1 = lv_obj_create(lv_screen_active());
    lv_obj_remove_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(ui_Screen1, 0, 0);
    lv_obj_set_size(ui_Screen1, LV_PCT(100), LV_PCT(100));

    ui_emoji = lv_image_create(ui_Screen1);
    lv_image_set_src(ui_emoji, &ota_emoji);
    lv_obj_set_width(ui_emoji, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_emoji, LV_SIZE_CONTENT);
    lv_obj_set_x(ui_emoji, (int)(-129 * scale));
    lv_obj_set_y(ui_emoji, (int)(-160 * scale));
    lv_obj_set_align(ui_emoji, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_emoji, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(ui_emoji, LV_OBJ_FLAG_SCROLLABLE);

    ui_ble = lv_image_create(ui_Screen1);
    lv_image_set_src(ui_ble, &ota_ble_close);
    lv_obj_set_width(ui_ble, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_ble, LV_SIZE_CONTENT);
    lv_obj_set_x(ui_ble, (int)(-129 * scale));
    lv_obj_set_y(ui_ble, (int)(-23 * scale));
    lv_obj_set_align(ui_ble, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ble, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(ui_ble, LV_OBJ_FLAG_SCROLLABLE);

    ui_network = lv_image_create(ui_Screen1);
    lv_image_set_src(ui_network, &ota_network_icon_img_close);
    lv_obj_set_width(ui_network, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_network, LV_SIZE_CONTENT);
    lv_obj_set_x(ui_network, (int)(-129 * scale));
    lv_obj_set_y(ui_network, (int)(114 * scale));
    lv_obj_set_align(ui_network, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_network, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(ui_network, LV_OBJ_FLAG_SCROLLABLE);

    ui_progress = lv_arc_create(ui_Screen1);
    lv_obj_set_size(ui_progress, (int)(200 * scale), (int)(200 * scale));
    lv_obj_set_x(ui_progress, (int)(63 * scale));
    lv_obj_set_y(ui_progress, (int)(-75 * scale));
    lv_obj_set_align(ui_progress, LV_ALIGN_CENTER);
    lv_arc_set_rotation(ui_progress, 270);
    lv_arc_set_bg_angles(ui_progress, 0, 360);
    lv_arc_set_value(ui_progress, 100);
    lv_obj_remove_style(ui_progress, NULL, LV_PART_KNOB);

    lv_obj_set_style_arc_color(ui_progress, lv_color_hex(0x333333),
                               LV_PART_MAIN);
    lv_obj_set_style_arc_width(ui_progress, (int)(25 * scale), LV_PART_MAIN);

    lv_obj_set_style_arc_color(ui_progress, lv_color_hex(0x00a0ff),
                               LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(ui_progress, (int)(25 * scale),
                               LV_PART_INDICATOR);

    lv_obj_remove_flag(ui_progress, LV_OBJ_FLAG_CLICKABLE);

    ui_prolabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_prolabel, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_prolabel, LV_SIZE_CONTENT);
    lv_obj_set_x(ui_prolabel, (int)(66 * scale));
    lv_obj_set_y(ui_prolabel, (int)(-74 * scale));
    lv_obj_set_align(ui_prolabel, LV_ALIGN_CENTER);
    lv_obj_add_style(ui_prolabel, &style, 0);
    lv_label_set_text(ui_prolabel, "0%");

    ui_status = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_status, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_status, LV_SIZE_CONTENT);
    lv_obj_set_x(ui_status, (int)(64 * scale));
    lv_obj_set_y(ui_status, (int)(49 * scale));
    lv_obj_set_align(ui_status, LV_ALIGN_CENTER);
    lv_obj_add_style(ui_status, &style, 0);
    lv_label_set_text(ui_status, "准备中...");
    lv_obj_set_style_text_font(ui_status, &ota_font, 0);

    return RT_EOK;
}

void dfu_pan_ui_task(void *args)
{
    rt_kprintf("dfu_pan_ui_task start\n");
    rt_err_t ret = RT_EOK;
    rt_uint32_t ms;

    /* init littlevGL */
    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        rt_kprintf("littlevGL init failed\n");
        return;
    }

    ota_ui_msg_queue =
        rt_mq_create("ota_ui_msg", sizeof(ui_msg_t *), 20, RT_IPC_FLAG_FIFO);
    if (ota_ui_msg_queue == RT_NULL)
    {
        rt_kprintf("Failed to create UI message queue\n");
        return;
    }

    float scale = get_scale_factor();
    g_scale = scale;

    ret = ota_ui_obj_init(scale);
    if (ret != RT_EOK)
    {
        rt_kprintf("UI init failed\n");
        return;
    }

    while (1)
    {
        ui_msg_t *msg;
        while (rt_mq_recv(ota_ui_msg_queue, &msg, sizeof(ui_msg_t *), 0) ==
               RT_EOK)
        {
            if (msg == RT_NULL)
            {
                rt_kprintf("Received NULL message pointer\n");
                continue;
            }
            switch (msg->type)
            {
            case UI_MSG_UPDATE_BLE:
                if (msg->data)
                {
                    if (strcmp(msg->data, UI_MSG_DATA_BLE_CONNECTED) == 0)
                    {
                        lv_image_set_src(ui_ble, &ota_ble);
                    }
                    else if (strcmp(msg->data, UI_MSG_DATA_BLE_DISCONNECTED) ==
                             0)
                    {
                        lv_image_set_src(ui_ble, &ota_ble_close);
                    }
                }
                break;
            case UI_MSG_UPDATE_NET:
                if (msg->data)
                {
                    if (strcmp(msg->data, UI_MSG_DATA_NET_CONNECTED) == 0)
                    {
                        lv_image_set_src(ui_network, &ota_network_icon_img);
                    }
                    else if (strcmp(msg->data, UI_MSG_DATA_NET_DISCONNECTED) ==
                             0)
                    {
                        lv_image_set_src(ui_network,
                                         &ota_network_icon_img_close);
                    }
                }
                break;
            case UI_MSG_UPDATE_PROGRESS:
                if (msg->data)
                {
                    rt_kprintf("PROGRESS: update progress: %s\n", msg->data);
                    lv_arc_set_value(ui_progress, atoi(msg->data));
                    lv_label_set_text(ui_prolabel, msg->data);

                    lv_obj_set_style_arc_color(
                        ui_progress, lv_color_hex(0x90EE90), LV_PART_INDICATOR);
                }
                break;
            case UI_MSG_UPDATE_PROGRESS_COLOR:
                if (msg->data)
                {
                    if (strcmp(msg->data, PROGRESS_COLOR_NORMAL) == 0)
                    {

                        lv_obj_set_style_arc_color(ui_progress,
                                                   lv_color_hex(0x00a0ff),
                                                   LV_PART_INDICATOR);
                    }
                    else if (strcmp(msg->data, PROGRESS_COLOR_SUCCESS) == 0)
                    {

                        lv_obj_set_style_arc_color(ui_progress,
                                                   lv_color_hex(0x90EE90),
                                                   LV_PART_INDICATOR);
                    }
                    else if (strcmp(msg->data, PROGRESS_COLOR_ERROR) == 0)
                    {

                        lv_obj_set_style_arc_color(ui_progress,
                                                   lv_color_hex(0xFF0000),
                                                   LV_PART_INDICATOR);
                    }
                }
                break;
            case UI_MSG_UPDATE_FILES:
                if (msg->data)
                {
                    lv_label_set_text(ui_status, msg->data);
                }
                break;
            case UI_MSG_SHOW_FAILURE_POPUP:
                if (msg->data)
                {
                    lv_label_set_text(ui_status, msg->data);
                }
                break;
            }

            if (msg->data)
            {
                ui_free(msg->data);
            }
            rt_free(msg);
        }

        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
}