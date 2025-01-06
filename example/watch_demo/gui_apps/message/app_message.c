/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"

#define APP_ID "message"

typedef struct
{
    lv_ex_data_t *title;
    lv_ex_data_t *content;
    lv_ex_data_t *app_name;
    void *title_handle;
    void *content_handle;
    void *app_name_handle;
} app_message_ctx_t;

LV_IMG_DECLARE(img_mail);



static app_message_ctx_t *p_app_message_ctx = NULL;


void cont_event_callback(lv_event_t *event)
{
    gui_app_self_exit();
}



static lv_obj_t *lbl_content;
static lv_obj_t *lbl_title;
static lv_obj_t *lbl_app_name;


void app_message_init(void *param)
{
    lv_ex_binding_t binding;

    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, 240, 240);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(cont, cont_event_callback,LV_EVENT_SHORT_CLICKED,NULL);

    lv_obj_t *label;
    label = lv_label_create(cont);

    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, 200);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 10, 0);
    lbl_content = label;
    RT_ASSERT(p_app_message_ctx->content);
    binding.target = lbl_content;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    p_app_message_ctx->content_handle = lv_ex_bind_data(p_app_message_ctx->content, &binding);

    label = lv_label_create(cont);
    lv_obj_align_to(label, lbl_content,  LV_ALIGN_OUT_TOP_LEFT, 0, -20);
    lbl_title = label;
    RT_ASSERT(p_app_message_ctx->title);
    binding.target = lbl_title;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    p_app_message_ctx->title_handle = lv_ex_bind_data(p_app_message_ctx->title, &binding);


    label = lv_label_create(cont);
    //lv_label_set_text(label, LV_EXT_STR_GET_BY_KEY(sifli_name, "Message"));
    lv_obj_align(label, LV_ALIGN_TOP_MID, -10, 20);
    lbl_app_name = label;
    RT_ASSERT(p_app_message_ctx->app_name);
    binding.target = lbl_app_name;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    p_app_message_ctx->app_name_handle = lv_ex_bind_data(p_app_message_ctx->app_name, &binding);
}

static void on_start(void)
{
    app_message_init(NULL);
}

static void on_resume(void)
{

}

static void on_pause(void)
{
}

static void on_stop(void)
{
    if (p_app_message_ctx->content_handle)
    {
        lv_ex_unbind_data(p_app_message_ctx->content, p_app_message_ctx->content_handle);
    }
    if (p_app_message_ctx->title_handle)
    {
        lv_ex_unbind_data(p_app_message_ctx->title, p_app_message_ctx->title_handle);
    }
    if (p_app_message_ctx->app_name_handle)
    {
        lv_ex_unbind_data(p_app_message_ctx->app_name, p_app_message_ctx->app_name_handle);
    }
}

static void main_msg_handler(gui_app_msg_type_t msg, void *param)
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


static void new_msg_popup_event_callback(lv_event_t *event)
{
    rt_kprintf("new_msg_popup_event_callback %s\n", lv_event_to_name(event->code));
    if (LV_EVENT_READY == event->code)
    {
        //Entry message main page
        gui_app_create_page("main", main_msg_handler);
        gui_app_remove_page("popup");
    }
    else if (LV_EVENT_CANCEL == event->code)
    {
        gui_app_goback();
    }
}


static void popupmsg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        const char *name = intent_get_string(gui_app_get_intent(), "newfrom");

        lv_obj_t *pop_container = lv_lvsfpopup_create(lv_scr_act());

        lv_lvsfpopup_set_icon(pop_container, LV_EXT_IMG_GET(img_mail));
        lv_lvsfpopup_set_title(pop_container, "New msg from:");
        lv_lvsfpopup_set_content(pop_container, name);

        lv_lvsfpopup_set_confirm_btn_txt(pop_container, "View");
        lv_lvsfpopup_set_cancel_btn_txt(pop_container, "Ignore");
        lv_obj_align(pop_container, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_event_cb(pop_container, new_msg_popup_event_callback,LV_EVENT_ALL,NULL);
    }
    break;

    case GUI_APP_MSG_ONRESUME:
        break;

    case GUI_APP_MSG_ONPAUSE:
        break;

    case GUI_APP_MSG_ONSTOP:
        break;
    default:
        break;
    }
}


static int app_main(intent_t i)
{
    const char *name = intent_get_string(i, "newfrom");

    if (name)
    {
        //Display new message popup page
        gui_app_create_page("popup", popupmsg_handler);
    }
    else
    {
        //Entry message main page
        //gui_app_regist_msg_handler(APP_ID, msg_handler);
        gui_app_create_page("main", main_msg_handler);
    }

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(message), LV_EXT_IMG_GET(img_mail), APP_ID, app_main);

void app_message_data_init(void)
{
    if (!p_app_message_ctx)
    {
        p_app_message_ctx = (app_message_ctx_t *) rt_malloc(sizeof(app_message_ctx_t));
        memset(p_app_message_ctx, 0, sizeof(app_message_ctx_t));
    }

    p_app_message_ctx->title = lv_ex_data_create("msg.title", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_message_ctx->title);
    p_app_message_ctx->content = lv_ex_data_create("msg.content", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_message_ctx->content);
    p_app_message_ctx->app_name = lv_ex_data_create("msg.app", LV_EX_DATA_STRING);
    RT_ASSERT(p_app_message_ctx->app_name);
}


void app_message_set_app_name(const uint8_t *s)
{
    if (p_app_message_ctx->app_name)
    {
#if 0
        if ((0 == strncmp("\xE4\xBF\xA1\xE6", (char *)s, strlen("\xE4\xBF\xA1\xE6")))
                || (0 == strncmp("Message", (char *)s, strlen("Message"))))
        {
            lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)"SMS");
        }
        else if ((0 == strncmp("\xe7\x94\xb5\xe8", (char *)s, strlen("\xe7\x94\xb5\xe8")))
                 || (0 == strncmp("Phone", (char *)s, strlen("Phone"))))
        {
            lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)"Phone");
        }
        else
        {
            lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)"WeChat");
        }
#else
        lv_ex_data_set_value(p_app_message_ctx->app_name, (void *)s);
#endif
    }
}

void app_message_set_title(const uint8_t *s)
{
    if (p_app_message_ctx->title)
    {
        lv_ex_data_set_value(p_app_message_ctx->title, (void *)s);
    }

}

void app_message_set_content(const uint8_t *s)
{
    if (p_app_message_ctx->content)
    {
        lv_ex_data_set_value(p_app_message_ctx->content, (void *)s);
    }
}


