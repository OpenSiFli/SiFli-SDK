/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LVSF_TIMELINE_H__
#define __LVSF_TIMELINE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if LVSF_USING_TIMELINE != 0


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef void (*lv_timeline_exec_xcb_t)(void *, int32_t, void *);
//typedef void (*lv_timeline_start_cb_t)(struct _lv_anim_t *);

typedef void (*lv_timeline_ready_cb_t)(void *, void *);

typedef struct
{
    lv_obj_t *obj;
    void *var;                   /**<Variable to animate*/
    lv_timeline_exec_xcb_t exec_cb;   /**< Function to execute to animate*/
    //lv_anim_start_cb_t start_cb; /**< Call it when the animation is starts (considering `delay`)*/
    lv_timeline_ready_cb_t ready_cb; /**< Call it when the animation is ready*/

    int32_t start_value;               /**< Start value*/
    int32_t end_value;                 /**< End value*/

    int32_t start_time;               /**< Start value*/
    int32_t end_time;                 /**< End value*/

    int32_t exec_cnt;
    int32_t exec_act_cnt;
    void *user_data;

    rt_list_t list;

} lv_timeline_node_t;

/*Ext data of footer*/
typedef struct
{
    lv_obj_t obj;
    lv_obj_t *parent;
    void *var;                   /**<Variable to animate*/
    lv_timeline_exec_xcb_t exec_cb;   /**< Function to execute to animate*/
    lv_timeline_ready_cb_t ready_cb; /**< Call it when the animation is ready*/

    int32_t time;               /**< Animation time in ms*/
    int32_t exec_cnt;
    int32_t exec_act_cnt;

    lv_timer_t *refr_timer;
    uint32_t refr_period;
    void *user_data;

    rt_list_t list;

} lv_timeline_ext_t;

typedef struct
{
    uint32_t refr_period;


} lv_timeline_cfg_t;

extern const lv_obj_class_t lv_timeline_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
/**
 * Create a footer object
 * @param parent pointer to a parent object.
 *                  If NULL then a screen will be created
 * @param copy pointer to a base object, if not NULL then the new object will be copied from it
 * @return pointer to the new footer object
 */
lv_obj_t *lv_timeline_create(lv_obj_t *parent);

/**
 * Set running time of an object
 * @param timeline pointer to a timeline object
 * @param time running time for the timeline object
 */
void lv_timeline_set_time(lv_obj_t *timeline, int32_t time);

/**
 * Set ready_cb of a timeline object
 * @param timeline pointer to a timeline object
 * @param ready_cb
 * @param user_data
 */

void lv_timeline_set_ready_cb(lv_obj_t *timeline, lv_timeline_ready_cb_t ready_cb, void *user_data);

/**
 * Insert an node into the timeline
 * @param timeline pointer to a timeline object
 * @param start_value
 * @param end_value
 * @param start_time
 * @param end_time
 * @param xcb
 * @param ready_cb
 * @param user_data
 */
void lv_timeline_add_element(lv_obj_t *timeline, int32_t start_value, int32_t end_value,
                             int32_t start_time, int32_t end_time, lv_timeline_exec_xcb_t xcb, lv_timeline_ready_cb_t ready_cb, void *user_data);

/**
 * Start timeline task
 * @param timeline pointer to a timeline object

 */
void lv_timeline_start(lv_obj_t *timeline);

/**
 * Pause timeline task
 * @param timeline pointer to a timeline object

 */
void lv_timeline_pause(lv_obj_t *timeline);


lv_obj_t *lv_timeline_get_var(lv_obj_t *timeline);

void lv_timeline_set_var(lv_obj_t *timeline, void *var);

/**********************
 *      MACROS
 **********************/
#endif /*LVSF_USING_TIMELINE*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*__LVSF_TIMELINE_H__*/

