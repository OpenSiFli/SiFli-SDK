/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef _LVSF_ENCODER_H_
#define _LVSF_ENCODER_H_

#include "lvsf_conf_internal.h"

#if LVSF_USING_ENCODER != 0

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

typedef void (*lv_encoder_cb_t)(struct _lv_obj_t *obj, lv_key_t key);
typedef void (*lv_encoder_start_cb_t)(struct _lv_obj_t *obj);
typedef void (*lv_encoder_ready_cb_t)(struct _lv_obj_t *obj);
typedef void (*lv_encoder_exec_cb_t)(struct _lv_obj_t *obj);

/*struct of encoder*/
typedef struct
{
    lv_obj_t obj;
    lv_encoder_cb_t cb;   /**< encoder's event callback. */
    bool request;         /**< request the encoder, must be paired with release. */

    bool slowdown_en;
    lv_key_t up_key;
    lv_key_t down_key;

    lv_obj_t *bind_obj;
    lv_anim_t *scrl_anim;
    lv_anim_t *slowdown_anim;

    int32_t scrl_anim_period;
    int32_t scrl_anim_period_acce;
    int32_t scrl_anim_period_max;
    int32_t slowdown_anim_period;
    int32_t scrl_timeout_tick;

    lv_encoder_start_cb_t start_cb;
    lv_encoder_ready_cb_t ready_cb;
    lv_encoder_exec_cb_t exec_cb;

    lv_coord_t hor_init_speed;
    lv_coord_t ver_init_speed;
    lv_coord_t hor_acce_speed;
    lv_coord_t ver_acce_speed;
    lv_coord_t hor_scrl_speed;
    lv_coord_t ver_scrl_speed;
    lv_coord_t hor_speed_max;
    lv_coord_t ver_speed_max;
    lv_dir_t scrl_dir;
    lv_dir_t act_dir;
    lv_dir_t last_dir;

} lv_encoder_ext_t;

typedef struct
{
    int32_t scrl_anim_period;
    int32_t scrl_anim_period_acce;
    int32_t scrl_anim_period_max;
    int32_t slowdown_anim_period;
    int32_t scrl_timeout_tick;

    lv_coord_t hor_init_speed;
    lv_coord_t ver_init_speed;
    lv_coord_t hor_acce_speed;
    lv_coord_t ver_acce_speed;
    lv_coord_t hor_speed_max;
    lv_coord_t ver_speed_max;

    lv_key_t up_key;
    lv_key_t down_key;

} lv_encoder_cfg_t;

extern const lv_obj_class_t lv_encoder_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a encoder objects
 * @param par pointer to an object, it will be the parent of the new encoder
 * @return pointer to the created encoder
 */
lv_obj_t *lv_encoder_create(lv_obj_t *parent);

/**
 * Set callback to the encoder objects
 * @param encoder pointer to encoder object,
 * @param cb event callback to encoder object
 */
void lv_encoder_set_cb(lv_obj_t *encoder, lv_encoder_cb_t cb);

/**
 * Set one encoder object to actived,and others will be requested
 * @param encoder pointer to encoder object,
 */
void lv_encoder_set_actived(lv_obj_t *encoder);

/**
 * Clean actived encoder object,and release all encoder object
 * @param encoder pointer to encoder object,
 */
void lv_encoder_actived_cleanup(lv_obj_t *encoder);

/**
 * Request the encoder objects
 * @param encoder pointer to encoder object,
 */
void lv_encoder_request(lv_obj_t *encoder);

/**
 * Release the encoder objects
 * @param encoder pointer to encoder object,
 */
void lv_encoder_release(lv_obj_t *encoder);

/**
 * Sets whether to suspend the encoder
 * @param en true: suspend the encoder, false: resume the encoder
 */
void lv_encoder_set_suapend(bool en);

/**
 * Set the speed at which the scroll encoder moves the page.
 * @param encoder pointer to an object
 * @param hor_init_speed Initial scroll speed in the horizontal direction
 * @param ver_init_speed Initial scroll speed in the vertical direction
 * @param hor_acce_speed Acceleration speed in the horizontal direction
 * @param ver_acce_speed Acceleration speed in the vertical direction
 * @param hor_speed_max Max scroll speed in the horizontal direction
 * @param ver_speed_max Max scroll speed in the vertical direction
 */
void lv_encoder_set_speed(lv_obj_t *encoder, lv_coord_t hor_init_speed, lv_coord_t ver_init_speed,
                          lv_coord_t hor_acce_speed, lv_coord_t ver_acce_speed, lv_coord_t hor_speed_max, lv_coord_t ver_speed_max);

/**
 * Set the animation period for the encoder scroll moves the page.
 * @param multlist pointer to an object
 * @param scrl_anim_period scroll page animation period
 * @param scrl_anim_period_max scroll page animation max period
 * @param slowdown_anim_period slowdown animation period
 */
void lv_encoder_set_period(lv_obj_t *encoder, int32_t scrl_anim_period,
                           int32_t scrl_anim_period_max, int32_t slowdown_anim_period);

/**
 * Set scroll direction
 * @param encoder pointer to a object
 * @param drag_dir scroll direction
 */
void lv_encoder_set_scrl_dir(lv_obj_t *encoder, lv_dir_t dir);

/**
 * Bind the obj that will be scrolled
 * @param encoder pointer to a object
 * @param obj target obj
 * @param start_cb scroll start callback, customized definition, for example: the page control deletes a running bounce-back animation
 * @param ready_cb scroll end callback, customized definition, for example: the page control create a running bounce-back animation
 * @param exec_cb scroll process callback, customized definition, this callback is performed on each move.
 */
void lv_encoder_bind_obj(lv_obj_t *encoder, lv_obj_t *obj, lv_encoder_start_cb_t start_cb,
                         lv_encoder_ready_cb_t ready_cb, lv_encoder_exec_cb_t exec_cb);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LVSF_USING_ENCODER */

#endif /*_LVSF_ENCODER_H_*/

