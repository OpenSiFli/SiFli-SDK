/**
 * @file lvsf_ezipa.h
 *
 */

#ifndef LVSF_EZIPA_H
#define LVSF_EZIPA_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

lv_obj_t *lv_lvsfezipa_create(lv_obj_t *parent);

void lv_lvsfezipa_set_src(lv_obj_t *ezipa, const char *src_ezipa);

void lv_lvsfezipa_play(lv_obj_t *ezipa);
void lv_lvsfezipa_stop(lv_obj_t *ezipa);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_EZIPA_H*/


