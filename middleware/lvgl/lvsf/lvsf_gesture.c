#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"

#if defined(GUI_APP_FRAMEWORK)&&(!defined (APP_TRANS_ANIMATION_NONE))

#include "gui_app_fwk.h"

#define LVSF_GESTURE_DETECT_WIDTH     (20)
#define LVSF_GESTURE_DETECT_HEIGHT    (10)
#define LVSF_GESTURE_LIMIT   (LV_DPI_DEF / 4)
/*
    0 - left
    1 - top
    2 - right
    3 - bottom
*/
lv_obj_t *gesture_detect_objs[4];
lv_obj_t *gesture_img_objs[4];
static bool gesture_is_enabled = true;
bool gesture_is_active = false;
static uint8_t  gesture_enable_reg = 0xff;

uint8_t lvsf_gesture_enable_register(uint8_t enable)
{
    uint8_t old_enable = gesture_enable_reg;
    gesture_enable_reg = enable;
    return old_enable;
}

void lvsf_gesture_bars_realign(void)
{
    if (gesture_detect_objs[0])
    {
        if (gesture_img_objs[0])
        {
            lv_obj_set_size(gesture_detect_objs[0], lv_obj_get_self_width(gesture_img_objs[0]) + LVSF_GESTURE_DETECT_WIDTH, LV_VER_RES_MAX);
            lv_obj_align(gesture_img_objs[0], LV_ALIGN_LEFT_MID, 0, 0);
            lv_obj_add_flag(gesture_img_objs[0], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_set_size(gesture_detect_objs[0], LVSF_GESTURE_DETECT_WIDTH, LV_VER_RES_MAX);
        }
        lv_obj_align(gesture_detect_objs[0], LV_ALIGN_OUT_LEFT_MID, LVSF_GESTURE_DETECT_WIDTH, 0);
    }
    if (gesture_detect_objs[2])
    {
        lv_obj_set_size(gesture_detect_objs[2], LVSF_GESTURE_DETECT_WIDTH, LV_VER_RES_MAX);
        lv_obj_align(gesture_detect_objs[2], LV_ALIGN_RIGHT_MID, 0, 0);
    }

    if (gesture_detect_objs[1])
    {
        lv_obj_set_size(gesture_detect_objs[1], LV_HOR_RES_MAX, LVSF_GESTURE_DETECT_HEIGHT);
        lv_obj_align(gesture_detect_objs[1], LV_ALIGN_TOP_MID, 0, 0);
    }
    if (gesture_detect_objs[3])
    {
        lv_obj_set_size(gesture_detect_objs[3], LV_HOR_RES_MAX, LVSF_GESTURE_DETECT_HEIGHT);
        lv_obj_align(gesture_detect_objs[3], LV_ALIGN_BOTTOM_MID, 0, 0);
    }
}

static void gesture_enable_update(bool enable)
{
    for (uint32_t i = 0; i < 4; i++)
        if (gesture_detect_objs[i])
        {
            if (enable)
                lv_obj_clear_flag(gesture_detect_objs[i], LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(gesture_detect_objs[i], LV_OBJ_FLAG_HIDDEN);
        }
}

static void left_bar_event_handler(lv_event_t *e)
{
#define x_2_process(x) ((x > 0) ? (x * MANUAL_TRAN_ANIM_MAX_PROCESS / LV_HOR_RES) : 0)
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    static lv_coord_t drag_offset = 0;
    //LOG_I("left_bar_event_handler  got event %s", lv_event_to_name(event));
    if (event == LV_EVENT_SCROLL_BEGIN)
    {
        if (gesture_img_objs[0])
            lv_obj_clear_flag(gesture_img_objs[0], LV_OBJ_FLAG_HIDDEN);
        gesture_is_active = true;
        drag_offset = obj->coords.x1;

        gui_app_manual_animation_start(x_2_process(obj->coords.x1));
    }
    else if (event == LV_EVENT_PRESSING)
    {
        drag_offset = LV_ABS(obj->coords.x1 - drag_offset);

        if (drag_offset > LVSF_GESTURE_LIMIT)
        {
        }

        if (gesture_is_active) gui_app_manual_animation_update(x_2_process(obj->coords.x1));
    }
    else if (event == LV_EVENT_SCROLL_END)
    {
        drag_offset = LV_ABS(obj->coords.x1 - drag_offset);

        if (gesture_is_active) gui_app_manual_animation_stop(x_2_process(obj->coords.x1));


        lvsf_gesture_bars_realign();
        gesture_is_active = false;
        gesture_enable_update(gesture_is_enabled);
    }
}

void lvsf_gesture_init(lv_obj_t *parent)
{
    lv_obj_t *bar;

    memset(gesture_detect_objs, 0, sizeof(gesture_detect_objs));
    memset(gesture_img_objs, 0, sizeof(gesture_img_objs));

    //Create left and right invisible bar
    bar = lv_obj_create(parent);
    //lv_obj_set_drag(bar, true);
    //lv_obj_set_drag_dir(bar, LV_DRAG_DIR_HOR);
    lv_obj_set_scroll_dir(bar, LV_DIR_HOR);
    lv_obj_add_flag(bar, LV_OBJ_FLAG_PRESS_LOCK);
    //lv_obj_set_event_cb(bar, left_bar_event_handler);
    lv_obj_add_event_cb(bar, left_bar_event_handler, (lv_event_code_t)NULL, 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(bar, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN | LV_STATE_DEFAULT);
    gesture_detect_objs[0] = bar;

#if 0 //Reserved for future
    gesture_detect_objs[2] = lv_obj_create(parent, bar);


    /*Invisible top and bottom bar*/
    bar = lv_obj_create(parent, bar);
    lv_obj_set_drag_dir(bar, LV_DRAG_DIR_VER);
    gesture_detect_objs[1] = bar;

    gesture_detect_objs[3] = lv_obj_create(parent, bar);
#endif

    lvsf_gesture_bars_realign();
}

void lvsf_gesture_deinit(void)
{
    for (uint32_t i = 0; i < 4; i++)
    {
        if (gesture_detect_objs[i])
            lv_obj_del(gesture_detect_objs[i]);
    }
    memset(gesture_detect_objs, 0, sizeof(gesture_detect_objs));
    memset(gesture_img_objs, 0, sizeof(gesture_img_objs));
}

void lvsf_gesture_set_image(uint32_t idx, const void *src_img)
{
    gesture_img_objs[idx] = lv_img_create(gesture_detect_objs[idx]);
    lv_img_set_src(gesture_img_objs[idx], src_img);
    lv_obj_add_flag(gesture_img_objs[idx], LV_OBJ_FLAG_HIDDEN);
    lvsf_gesture_bars_realign();
}


void lvsf_gesture_disable(void)
{
    gesture_is_enabled = false;

    //Not hidden objs while gesture is active
    if (!gesture_is_active) gesture_enable_update(gesture_is_enabled);


}

void lvsf_gesture_enable(void)
{
    gesture_is_enabled = true;

    //Not hidden objs while gesture is active
    if (!gesture_is_active) gesture_enable_update(gesture_is_enabled);
}
#else

//Dummy function to fix lvgl_input_agent extern error
uint8_t lvsf_gesture_enable_register(uint8_t enable)
{
    return 0;
}

#endif /* GUI_APP_FRAMEWORK && !APP_TRANS_ANIMATION */

