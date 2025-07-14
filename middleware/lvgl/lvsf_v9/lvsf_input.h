#ifndef LVSF_INPUT_H
#define LVSF_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

#define LV_BLOCK_EVENT     1
#define LV_POST_EVENT      0


/**********************
 *      TYPEDEFS
 **********************/
typedef int32_t (*lv_key_handler_t)(lv_key_t key, lv_indev_state_t event);
typedef int32_t (*lv_defaultwheel_handler_t)(int16_t diff, lv_indev_state_t event, void *user_data);

/**********************
 * GLOBAL PROTOTYPES
 **********************/





int32_t keypad_handler_register(lv_key_handler_t h);
int32_t keypad_default_handler_register(lv_key_handler_t h);
int32_t keypad_do_event(lv_key_t key, lv_indev_state_t event);



int32_t wheel_do_event(int16_t diff, lv_indev_state_t event);
int32_t wheel_default_handler_register(lv_defaultwheel_handler_t h, void *user_data);



/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_INPUT_H*/




