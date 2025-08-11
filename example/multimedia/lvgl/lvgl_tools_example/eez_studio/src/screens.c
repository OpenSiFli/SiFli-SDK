#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;
uint32_t active_theme_index = 0;

void create_screen_img_shanghai()
{
    lv_obj_t *obj = lv_obj_create(0);
    objects.img_shanghai = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 400, 400);
    {
        lv_obj_t *parent_obj = obj;
        {
            lv_obj_t *obj = lv_img_create(parent_obj);
            objects.obj0 = obj;
            lv_obj_set_pos(obj, 0, 0);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_img_set_src(obj, &img_shanghai);
            //lv_obj_add_event_cb(obj, action_p1_inc_btn_clicked, LV_EVENT_PRESSED, (void *)0);
            lv_obj_set_style_img_recolor(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_PRESSED);
        }
    }

    tick_screen_img_shanghai();
}

void tick_screen_img_shanghai()
{
}

void create_user_widget_test(lv_obj_t *parent_obj, int startWidgetIndex)
{
    (void)startWidgetIndex;
    lv_obj_t *obj = parent_obj;
    {
        lv_obj_t *parent_obj = obj;
    }
}

void tick_user_widget_test(int startWidgetIndex)
{
    (void)startWidgetIndex;
}



typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] =
{
    tick_screen_img_shanghai,
};
void tick_screen(int screen_index)
{
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId)
{
    tick_screen_funcs[screenId - 1]();
}

void create_screens()
{
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);

    create_screen_img_shanghai();
}
