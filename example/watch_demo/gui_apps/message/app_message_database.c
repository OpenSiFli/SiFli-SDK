/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles.h"
#include "bf0_ble_ancs.h"
#include "data_service_subscriber.h"
#include "app_message.h"
#if !defined(_MSC_VER)
    #include "ancs_service.h"
#endif
#include "intent.h"

//static uint8_t g_ble_message[50];



static datac_handle_t message_service_handle;



static int app_ble_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
#ifndef _MSC_VER
        RT_ASSERT(arg->data);
        int16_t len = arg->data_len;
        ancs_service_noti_attr_t *value = (ancs_service_noti_attr_t *)arg->data;
        ble_ancs_attr_value_t *att_value = &value->value[0];
        for (uint32_t i = 0; i < value->attr_count; i++)
        {
            if (att_value->attr_id == BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME)
            {
                uint8_t *app_name = rt_malloc(att_value->len + 1);
                rt_memcpy(app_name, att_value->data, att_value->len);
                app_name[att_value->len] = 0;
                app_message_set_app_name(app_name);
                rt_free(app_name);
            }
            else if (att_value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE)
            {
                uint8_t *title = rt_malloc(att_value->len + 1);
                rt_memcpy(title, att_value->data, att_value->len);
                title[att_value->len] = 0;
                app_message_set_title(title);
                rt_free(title);
            }
            else if (att_value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE)
            {
                uint8_t *message = rt_malloc(att_value->len + 1);
                rt_memcpy(message, att_value->data, att_value->len);
                message[att_value->len] = 0;
                app_message_set_content(message);
                rt_free(message);
            }
            att_value = (ble_ancs_attr_value_t *)((uint8_t *)att_value + sizeof(ble_ancs_attr_value_t) + att_value->len);
        }
#endif
//        gui_app_run("message");
        {
            intent_t i = intent_init("message");
            intent_set_string(i, "newfrom", "Unknown user");
            intent_runapp(i);
            intent_deinit(i);
        }
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
    }
    return 0;
}


void send_a_sms(int argc, char **argv)
{
    if (argc > 3)
    {
        app_message_set_app_name((const uint8_t *)"test_app");
        app_message_set_title((const uint8_t *)argv[2]);
        app_message_set_content((const uint8_t *)argv[3]);

        intent_t i = intent_init("message");
        intent_set_string(i, "newfrom", argv[1]);
        intent_runapp(i);
        intent_deinit(i);
    }
}
MSH_CMD_EXPORT_ALIAS(send_a_sms, sms, sms [from_who] [title] [content]);


int app_message_database_init(void)
{
    message_service_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != message_service_handle);
    datac_subscribe(message_service_handle, "ANCS", app_ble_callback, 0);
    return 0;
}


INIT_APP_EXPORT(app_message_database_init);


