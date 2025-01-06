#ifndef _SM_API_H_
#define _SM_API_H_
#include "sm_internal.h"
typedef enum
{
    SM_TOUCH_PRESS = 0,
    SM_TOUCH_RELEASE,
    SM_TOUCH_PRESSING,
    SM_VOLUP,
    SM_VOLDOWN,
    SM_BACK, //back last window in stream media
} sm_event_t;

int streaming_media_start(char *app_name, char *app_id, int fmt);
void streaming_media_stop(void);
int sm_ctrl_send_tp(sm_event_t ss_st, float x, float y);
int sm_decode_dim(int *width, int *height);
int sm_video_get(uint8_t *data);
int sm_video_get_period(void);
uint64_t sm_cache_unused(void);
void set_need_iframe();
void set_video_order_config(uint32_t width, uint32_t height, uint32_t bitsrate);
uint32_t sm_debug_get_info(uint32_t *v, uint32_t *a, uint32_t *dv, uint32_t *da);

#endif
