#ifndef LV_SEQIMG_H
#define LV_SEQIMG_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const lv_obj_class_t lv_seqimg_class;

lv_obj_t *lv_seqimg_create(lv_obj_t *parent);
void lv_seqimg_src_array(lv_obj_t *obj, const lv_img_dsc_t **dsc_array, uint16_t size);
void lv_seqimg_select(lv_obj_t *obj, uint16_t index);
void lv_seqimg_play(lv_obj_t *obj);
void lv_seqimg_pause(lv_obj_t *obj);
void lv_seqimg_set_period(lv_obj_t *obj, uint32_t period);

#ifdef __cplusplus
}
#endif

#endif /* LV_SEQIMG_H */