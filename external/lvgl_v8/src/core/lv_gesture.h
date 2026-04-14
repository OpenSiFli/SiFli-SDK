#ifndef LV_GESTURE_H
#define LV_GESTURE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

/**
* Gesture event type
*/
enum
{
    LV_GESTURE_ANIM_START,
    LV_GESTURE_ANIM_PRESSING,
    LV_GESTURE_ANIM_END,
    LV_GESTURE_GOBACK,
};
typedef uint8_t lv_gesture_event_t;
typedef struct 
{
    uint32_t time;
    lv_dir_t dir;
    uint32_t dis;
}gesture_para_t;
#define LV_EVENT_GESTURE_DIR (LV_EVENT_PREPROCESS - 1)
/**
 * Gesture event callback structure
 */

typedef int32_t (*lv_gesture_event_cb_t)(lv_gesture_event_t, int32_t);

typedef uint8_t(*lv_gesture_start_cb_t)(lv_point_t*);
/**
 * Gesture schedule state value
 */
enum
{
    LV_GESTURE_SCHE_IDLE,
    LV_GESTURE_SCHE_SUSPEND,
};

typedef uint8_t lv_gesture_sche_state_t;

/**
 * Gesture enable/disable value
 */
enum
{
    LV_GESTURE_NONE,
	LV_GESTURE_ENABLE,
	LV_GESTURE_DISABLE
};


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Set gesture schedule state, align to application framework state.
 * @param sche_idle true: enter into idle, false: suspend
 */
void lv_gesture_set_sche_state(bool sche_idle);

/**
 * Get gesture schedule state.
 * @return LV_GESTURE_SCHE_IDLE' or 'LV_GESTURE_SCHE_SUSPEND'
 */
lv_gesture_sche_state_t lv_gesture_get_sche_state(void);

/**
 * Enable gesture function, the status switch will be performed after idle is entered.
 */
void lv_gesture_enable(void);

/**
 * Disable gesture function, the status switch will be performed after idle is entered.
 */
void lv_gesture_disable(void);

/**
 * Set gesture paramers
 * @param left_area the gesture will trigger an animation when the start point in the line left. 0 for no animation,LV_HOR_RES_MAX for animation¡£
 * @param goback_en the gesture will do not trigger an animation when the start point in the line right.Closely related to left_area parameter
 * @param event_cb User-defined callback
 */
void lv_gesture_init(lv_gesture_event_cb_t event_cb);

int lv_gesture_is_valid();

lv_coord_t lv_gesture_set_area_line(lv_coord_t left_area);

lv_coord_t lv_gesture_get_area_line();

void lv_gesture_set_goback(uint8_t goback_en);

uint8_t lv_gesture_get_goback();

/**
 * Set the callback for the start of the gesture animation.
 * @param event_cb gesture animation start callback.
 */
void lv_gesture_set_start_cb(lv_gesture_start_cb_t event_cb);

/**
 * Set whether an object ignores gestures.
 * @param obj pointer to an object
 * @param en true : enable gesture ignore , false : disable
 */
void lv_gesture_set_ignore(lv_obj_t *obj, bool en);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_GESTURE_H*/


