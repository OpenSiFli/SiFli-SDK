#include "data_service_provider.h"
#include "power_manager_service.h"

#define LOG_TAG  "svc.pm"
#include "log.h"



#define LCD_BRIGHTNESS_MIN 3
#define LCD_BRIGHTNESS_MAX 100

#define LCD_AUTO_OFF_MIN  15
#define LCD_AUTO_OFF_MAX   300

static datas_handle_t this_service = NULL;
static uint16_t lcd_brightness = LCD_BRIGHTNESS_MAX;
static uint16_t lcd_auto_off_seconds = 30;
static uint16_t lcd_rotated = 0;


static int32_t msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        break;
    }


    case PWRMGR_MSG_LCD_BRIGHTNESS_GET_REQ:
    case PWRMGR_MSG_LCD_BRIGHTNESS_SET_REQ:
    {
        range_msg_t r;

        if (PWRMGR_MSG_LCD_BRIGHTNESS_SET_REQ == msg->msg_id)
        {
            uint16_t *brightness = (uint16_t *)data_service_get_msg_body(msg);
            if ((*brightness < LCD_BRIGHTNESS_MIN) || (*brightness > LCD_BRIGHTNESS_MAX))
            {
                LOG_I("lcd_brighness invalid value %d", *brightness);
            }
            else
            {
#ifndef _MSC_VER
                rt_device_t device = RT_NULL;
                device = rt_device_find("lcd");

                rt_err_t ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
                if (-RT_EBUSY == ret)
                {
                    rt_device_control(device, RTGRAPHIC_CTRL_SET_BRIGHTNESS, brightness);
                }
                else if (RT_EOK == ret)
                {
                    rt_device_close(device);
                }


                LOG_I("lcd_brighness set to %d", *brightness);

                lcd_brightness = *brightness;
#endif
            }
        }


        r.min = LCD_BRIGHTNESS_MIN;
        r.cur = lcd_brightness;
        r.max = LCD_BRIGHTNESS_MAX;

        datas_send_response_data(this_service, msg, sizeof(r), (uint8_t *)&r);
    }
    break;


    case PWRMGR_MSG_LCD_AUTO_OFF_TIME_SET_REQ:
    case PWRMGR_MSG_LCD_AUTO_OFF_TIME_GET_REQ:
    {
        range_msg_t r;

        if (PWRMGR_MSG_LCD_AUTO_OFF_TIME_SET_REQ == msg->msg_id)
        {
            uint16_t *off_seconds = (uint16_t *)data_service_get_msg_body(msg);

            if ((*off_seconds < LCD_AUTO_OFF_MIN) || (*off_seconds > LCD_AUTO_OFF_MAX))
            {
                LOG_I("lcd_auto_off invalid value %d", *off_seconds);
            }
            else
            {
                lcd_auto_off_seconds = *off_seconds;
            }

            LOG_I("lcd_auto_off_seconds set to %d", lcd_auto_off_seconds);
        }

        r.min = LCD_AUTO_OFF_MIN;
        r.cur = lcd_auto_off_seconds;
        r.max = LCD_AUTO_OFF_MAX;

        datas_send_response_data(this_service, msg, sizeof(r), (uint8_t *)&r);
    }
    break;

    case PWRMGR_MSG_LCD_ROTATE_180_GET_REQ:
    case PWRMGR_MSG_LCD_ROTATE_180_SET_REQ:
    {
        if (PWRMGR_MSG_LCD_ROTATE_180_SET_REQ == msg->msg_id)
        {
            uint16_t rotate = *((uint16_t *)data_service_get_msg_body(msg));


            {
#ifndef _MSC_VER
                struct rt_device_graphic_info info;

                rt_device_t device = RT_NULL;
                device = rt_device_find("lcd");
                rt_err_t ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

                if (-RT_EBUSY == ret)
                {
                    if (rt_device_control(device, RTGRAPHIC_CTRL_GET_INFO, &info) == RT_EOK)
                    {
                        rt_device_control(device, RTGRAPHIC_CTRL_ROTATE_180, NULL);

#if 0 //Rotate tp device in LVGL
                        struct rt_device_rect_info lcd_rect;

                        lcd_rect.x = 0;
                        lcd_rect.y = 0;
                        lcd_rect.width = info.width;
                        lcd_rect.height = info.height;

                        rt_device_t tp_device = rt_device_find("touch");
                        rt_err_t ret_tp = rt_device_open(tp_device, RT_DEVICE_OFLAG_RDWR);
                        if (-RT_EBUSY == ret_tp)
                        {
                            rt_device_control(tp_device, RTGRAPHIC_CTRL_ROTATE_180, &lcd_rect);
                        }
                        else if (RT_EOK == ret)
                        {
                            rt_device_close(tp_device);
                        }
#endif /* 0 */
                    }
                }
                else if (RT_EOK == ret)
                {
                    rt_device_close(device);
                }



                LOG_I("lcd_rotate %d", rotate);



                lcd_rotated = rotate;
#endif
            }

        }



        datas_send_response_data(this_service, msg, sizeof(lcd_rotated), (uint8_t *)&lcd_rotated);
    }
    break;

    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}

static data_service_config_t power_manager_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .data_filter = NULL,
    .msg_handler = msg_handler,
};

#ifndef _MSC_VER
    static
#endif
int power_manager_service_register(void)
{
    rt_err_t ret = RT_EOK;
    this_service = datas_register("powermgr", &power_manager_service_cb);


    return ret;
}

INIT_COMPONENT_EXPORT(power_manager_service_register);
