#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"


#if 1//USE_KEYBOARD
static lv_key_handler_t g_kp_handler = NULL;
static lv_key_handler_t g_kp_default_handler = NULL;

int32_t keypad_do_event(lv_key_t key, lv_indev_state_t event)
{
    if (g_kp_handler)
    {
        if (LV_BLOCK_EVENT == g_kp_handler(key, event))
        {
            return 0;
        }
    }

    if (g_kp_default_handler)
    {
        g_kp_default_handler(key, event);
    }

    return 0;
}

int32_t keypad_handler_register(lv_key_handler_t h)
{
    g_kp_handler = h;
    return 0;
}

int32_t keypad_default_handler_register(lv_key_handler_t h)
{
    g_kp_default_handler = h;
    return 0;
}

#endif /* USE_KEYBOARD */

#if 1// USE_MOUSEWHEEL

static lv_defaultwheel_handler_t s_wheel_default_handler = NULL;
static void *s_user_data = NULL;

int32_t wheel_do_event(int16_t diff, lv_indev_state_t event)
{
    if (s_wheel_default_handler)
    {
        s_wheel_default_handler(diff, event, s_user_data);
    }
    return 0;
}

int32_t wheel_default_handler_register(lv_defaultwheel_handler_t h, void *user_data)
{
    s_wheel_default_handler = h;
    s_user_data = user_data;
    return 0;
}
#endif /* USE_MOUSEWHEEL */
