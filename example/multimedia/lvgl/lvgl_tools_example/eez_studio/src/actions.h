#ifndef EEZ_LVGL_UI_EVENTS_H
#define EEZ_LVGL_UI_EVENTS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void action_p1_inc_btn_clicked(lv_event_t *e);
extern void action_p1_dec_btn_clicked(lv_event_t *e);
extern void action_p1_get_value_btn_clicked(lv_event_t *e);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_EVENTS_H*/