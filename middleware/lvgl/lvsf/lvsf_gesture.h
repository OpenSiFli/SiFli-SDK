#ifndef LVSF_GESTURE_H
#define LVSF_GESTURE_H

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

void lvsf_gesture_init(lv_obj_t *parent);
void lvsf_gesture_deinit(void);

void lvsf_gesture_set_image(uint32_t idx, const void *src_img);

void lvsf_gesture_disable(void);
void lvsf_gesture_enable(void);
void lvsf_gesture_bars_realign(void);


/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_GESTURE_H*/






