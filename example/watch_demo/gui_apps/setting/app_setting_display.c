

#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "ui_datasrv_subscriber.h"
#include "power_manager_service.h"


typedef struct
{
    lv_obj_t *brightness;
    lv_obj_t *auto_off;
    lv_obj_t *rotate;
    datac_handle_t pwr_srv_hdl;
} display_ctx_t;


static display_ctx_t *p_display = NULL;
static int powermgr_srv_callback(data_callback_arg_t *arg);


static void back_btn_event_callback(lv_event_t *e)
{
    //lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_goback();

    }
}

static void datac_send_data(datac_handle_t handle, uint16_t msg_id, uint8_t *data, uint16_t data_len)
{
    data_msg_t msg;
    uint8_t *msg_payload;

    msg_payload = data_service_init_msg(&msg, msg_id, data_len);
    memcpy(msg_payload, data, data_len);
    datac_send_msg(handle, &msg);
}

static void rotate_scr_anim(void *obj, int32_t v)
{
    lv_obj_invalidate(lv_scr_act());
}


static void event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_VALUE_CHANGED)
    {
        if (p_display->brightness == obj)
        {
            uint16_t v = (uint16_t) lv_slider_get_value(obj);

            datac_send_data(p_display->pwr_srv_hdl,
                            PWRMGR_MSG_LCD_BRIGHTNESS_SET_REQ,
                            (uint8_t *)&v, sizeof(uint16_t));
        }
        else if (p_display->auto_off == obj)
        {
            uint16_t v = (uint16_t) lv_slider_get_value(obj);

            datac_send_data(p_display->pwr_srv_hdl,
                            PWRMGR_MSG_LCD_AUTO_OFF_TIME_SET_REQ,
                            (uint8_t *)&v, sizeof(uint16_t));
        }
        else if (p_display->rotate == obj)
        {
            uint16_t v;

            if (lv_obj_get_state(obj)&LV_STATE_CHECKED)
                v = 1;
            else
                v = 0;

            datac_send_data(p_display->pwr_srv_hdl,
                            PWRMGR_MSG_LCD_ROTATE_180_SET_REQ,
                            (uint8_t *)&v, sizeof(v));


            if (v)
                lv_disp_set_rotation(lv_disp_get_default(), LV_DISP_ROT_180);
            else
                lv_disp_set_rotation(lv_disp_get_default(), LV_DISP_ROT_NONE);

            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, p_display->rotate);
            lv_anim_set_values(&a, 0, 180);

            lv_anim_set_exec_cb(&a, rotate_scr_anim);
            lv_anim_set_time(&a, 100);
            lv_anim_start(&a);


        }
        else
        {
        }
    }
}


static void on_start(void)
{
    // header
    lv_obj_t *title = lv_lvsfheader_create(lv_scr_act());
    lv_lvsfheader_set_title(title, "display");
    lv_lvsfheader_set_visible_item(title, LVSF_HEADER_BRANCH);
    lv_lvsfheader_back_event_cb(title, back_btn_event_callback);


    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    //lv_obj_set_layout(cont, LV_LAYOUT_COLUMN_LEFT); //TODO: ???
    lv_obj_align_to(cont, title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    lv_obj_set_size(cont, LV_HOR_RES, LV_VER_RES - lv_obj_get_height(title));

    lv_obj_t *label = lv_label_create(cont);
    lv_label_set_text(label, "brightness");
    p_display->brightness = lv_slider_create(cont);
    lv_obj_set_width(p_display->brightness, lv_obj_get_width(cont));
    lv_obj_align_to(p_display->brightness, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, LV_DPX(2));


    label = lv_label_create(cont);
    lv_label_set_text(label, "auto off");
    lv_obj_align_to(label, p_display->brightness, LV_ALIGN_OUT_BOTTOM_LEFT, 0, LV_DPX(10));
    p_display->auto_off = lv_slider_create(cont);
    lv_obj_align_to(p_display->auto_off, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, LV_DPX(2));
    lv_obj_set_width(p_display->auto_off, lv_obj_get_width(cont));


    label = lv_label_create(cont);
    lv_label_set_text(label, "rotate");
    lv_obj_align_to(label, p_display->auto_off, LV_ALIGN_OUT_BOTTOM_LEFT, 0, LV_DPX(10));
    p_display->rotate = lv_switch_create(cont);
    lv_obj_align_to(p_display->rotate, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, LV_DPX(2));



    lv_obj_add_event_cb(p_display->brightness, event_cb, LV_EVENT_ALL, 0);
    lv_obj_add_event_cb(p_display->auto_off, event_cb, LV_EVENT_ALL, 0);
    lv_obj_add_event_cb(p_display->rotate, event_cb, LV_EVENT_ALL, 0);


    //Get data from service
    p_display->pwr_srv_hdl = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_display->pwr_srv_hdl);
    ui_datac_subscribe(p_display->pwr_srv_hdl, "powermgr", powermgr_srv_callback, 0);
}

static void on_resume(void)
{

}

static void on_pause(void)
{
}

static void on_stop(void)
{
    if (p_display)
    {
        if (DATA_CLIENT_INVALID_HANDLE != p_display->pwr_srv_hdl)
        {
            datac_close(p_display->pwr_srv_hdl);
            p_display->pwr_srv_hdl = DATA_CLIENT_INVALID_HANDLE;
        }

        lv_mem_free(p_display);
        p_display = NULL;
    }
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

static int powermgr_srv_callback(data_callback_arg_t *arg)
{
    //rt_kprintf("powermgr_srv_callback  0x%04x\n", arg->msg_id);
    if (!p_display && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (p_display)
        {
            if (rsp->result >= 0)
            {
                data_msg_t msg;

                data_service_init_msg(&msg, PWRMGR_MSG_LCD_BRIGHTNESS_GET_REQ, 0);
                datac_send_msg(p_display->pwr_srv_hdl, &msg);

                data_service_init_msg(&msg, PWRMGR_MSG_LCD_AUTO_OFF_TIME_GET_REQ, 0);
                datac_send_msg(p_display->pwr_srv_hdl, &msg);

                data_service_init_msg(&msg, PWRMGR_MSG_LCD_ROTATE_180_GET_REQ, 0);
                datac_send_msg(p_display->pwr_srv_hdl, &msg);
            }
        }
    }
    break;

    case PWRMGR_MSG_LCD_BRIGHTNESS_GET_RSP:
    {
        range_msg_t *p_range;
        p_range = (range_msg_t *)arg->data;

        rt_kprintf("PWRMGR_MSG_LCD_BRIGHTNESS_GET_RSP cur=%d[%d,%d]\n",
                   p_range->cur, p_range->min, p_range->max);
        lv_bar_set_range(p_display->brightness, p_range->min, p_range->max);
        lv_bar_set_value(p_display->brightness, p_range->cur, LV_ANIM_ON);
    }
    break;

    case PWRMGR_MSG_LCD_AUTO_OFF_TIME_GET_RSP:
    {
        range_msg_t *p_range;

        p_range = (range_msg_t *)arg->data;

        rt_kprintf("PWRMGR_MSG_LCD_AUTO_OFF_TIME_GET_RSP cur=%d[%d,%d]\n",
                   p_range->cur, p_range->min, p_range->max);

        lv_bar_set_range(p_display->auto_off, p_range->min, p_range->max);
        lv_bar_set_value(p_display->auto_off, p_range->cur, LV_ANIM_ON);

    }
    break;

    case PWRMGR_MSG_LCD_ROTATE_180_GET_RSP:
    case PWRMGR_MSG_LCD_ROTATE_180_SET_RSP:
    {
        uint16_t r = *((uint16_t *)arg->data);

        rt_kprintf("PWRMGR_MSG_LCD_ROTATE_180_GET/SET_RSP %d\n", r);

        if (r)
            lv_obj_add_state(p_display->rotate, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(p_display->rotate, LV_STATE_CHECKED);
    }
    break;

    case PWRMGR_MSG_LCD_AUTO_OFF_TIME_SET_RSP:
    case PWRMGR_MSG_LCD_BRIGHTNESS_SET_RSP:
    default:
        break;
    }

    return 0;
}


void app_setting_display_main(void)
{
    RT_ASSERT(NULL == p_display);
    p_display = lv_mem_alloc(sizeof(display_ctx_t));
    LV_ASSERT_MALLOC(p_display);
    p_display->pwr_srv_hdl = DATA_CLIENT_INVALID_HANDLE;

    gui_app_create_page("display", msg_handler);
}
