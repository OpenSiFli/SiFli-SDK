
#ifndef APP_SCHEDULE_PORT_H
#define APP_SCHEDULE_PORT_H



#define APP_TRANS_ANIM_FULL_SCALE (100)
#ifndef APP_TRANS_ANIM_SNAPSHOT_SCALE
    #define APP_TRANS_ANIM_SNAPSHOT_SCALE 100
#endif /* !APP_TRANS_ANIM_SNAPSHOT_SCALE */

#ifndef APP_SNAPSHOT_COLOR_DEPTH
    #define APP_SNAPSHOT_COLOR_DEPTH LV_COLOR_DEPTH
#endif

#define APP_TRANS_ANIM_PERCENT APP_TRANS_ANIM_SNAPSHOT_SCALE / APP_TRANS_ANIM_FULL_SCALE

#define APP_TRANS_ANIM_ZOOM_NONE ((uint16_t)(LV_IMG_ZOOM_NONE * 100 / APP_TRANS_ANIM_SNAPSHOT_SCALE ))
#define APP_TRANS_ANIM_ZOOM_MIN (APP_TRANS_ANIM_ZOOM_NONE >> 1)
#define APP_TRANS_ANIM_SNAPSHOT_ZOOM ((uint16_t)(LV_IMG_ZOOM_NONE * APP_TRANS_ANIM_PERCENT))
#define APP_TRANS_ANIM_SNAPSHOT_WIDTH (LV_HOR_RES_MAX * APP_TRANS_ANIM_PERCENT)
#define APP_TRANS_ANIM_SNAPSHOT_HEIGHT (LV_VER_RES_MAX * APP_TRANS_ANIM_PERCENT)

#define APP_TRANS_ANIM_SNAPSHOT_SIZE (APP_TRANS_ANIM_SNAPSHOT_WIDTH * APP_TRANS_ANIM_SNAPSHOT_HEIGHT * APP_SNAPSHOT_COLOR_DEPTH / 8)




#define GUI_LIB_REFR_PERIOD_FLAG   (1<<30)
#define GUI_LIB_REFR_PERIOD_CNT(x) (GUI_LIB_REFR_PERIOD_FLAG|(x))
screen_t port_app_sche_create_scr(void);
void port_app_sche_del_scr(screen_t scr);
void port_app_sche_load_scr(screen_t scr);
screen_t port_app_sche_get_act_scr(void);
void port_app_sche_set_user_data(screen_t scr, uint32_t v);
uint32_t port_app_sche_get_user_data(screen_t scr);
void port_app_sche_reset_indev(screen_t scr);
task_t port_app_sche_task_create(task_handler_t task_handler, uint32_t period, void *user_data);
void port_app_sche_task_del(task_t task_handler);
void port_app_sche_enable_indev(bool enable, bool expect_tp);

#endif /* APP_SCHEDULE_PORT_H */

