/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_SWITCHANIM_H
#define LVSF_SWITCHANIM_H

#if LVSF_USING_SWITCHANIM != 0

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "lvsf_conf_internal.h"
#include "section.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define APP_TRANS_ANIM_FULL_SCALE (100)
#ifndef APP_TRANS_ANIM_SNAPSHOT_SCALE
#define APP_TRANS_ANIM_SNAPSHOT_SCALE 100
#endif /* !APP_TRANS_ANIM_SNAPSHOT_SCALE */


#define APP_TRANS_ANIM_PERCENT APP_TRANS_ANIM_SNAPSHOT_SCALE / APP_TRANS_ANIM_FULL_SCALE

#define APP_TRANS_ANIM_ZOOM_NONE ((uint16_t)(LV_IMG_ZOOM_NONE * 100 / APP_TRANS_ANIM_SNAPSHOT_SCALE ))
#define APP_TRANS_ANIM_ZOOM_MIN (APP_TRANS_ANIM_ZOOM_NONE >> 1)
#define APP_TRANS_ANIM_SNAPSHOT_ZOOM ((uint16_t)(LV_IMG_ZOOM_NONE * APP_TRANS_ANIM_PERCENT))
#define APP_TRANS_ANIM_SNAPSHOT_WIDTH (LV_HOR_RES_MAX * APP_TRANS_ANIM_PERCENT)
#define APP_TRANS_ANIM_SNAPSHOT_HEIGHT (LV_VER_RES_MAX * APP_TRANS_ANIM_PERCENT)

#define APP_TRANS_ANIM_SNAPSHOT_SIZE (APP_TRANS_ANIM_SNAPSHOT_WIDTH * APP_TRANS_ANIM_SNAPSHOT_HEIGHT * LV_COLOR_DEPTH / 8)

enum
{
    LV_SWITCHANIM_NONE = 0,
    LV_SWITCHANIM_PUSH,
    LV_SWITCHANIM_ZOOM,
    LV_SWITCHANIM_MASK,
    LV_SWITCHANIM_SPLIT,
    LV_SWITCHANIM_TURN_AXIS,
    LV_SWITCHANIM_TURN_3D,
    LV_SWITCHANIM_TURN_HALF,
    LV_SWITCHANIM_TURN_SWITCH,
    LV_SWITCHANIM_TURN_BOOK,
    LV_SWITCHANIM_SHUTTLE,
    LV_SWITCHANIM_SHUTTER,
    LV_SWITCHANIM_DEFAULT = 0xff
};
enum
{
    GUI_SWITCH_ANIM_ZOOM = 0,
    GUI_SWITCH_ANIM_OPA,
    GUI_SWITCH_ANIM_DEFAULE = 0xffffffff
};
enum
{
    LV_BASEANIM_NONE_TYPE = 0,
    LV_BASEANIM_ENTER_TYPE,
    LV_BASEANIM_EXIT_TYPE,
};


#define LV_SWITCHANIM_MINOR_DEFAULT 0

enum
{
    LV_PUSHANIM_AUX = LV_SWITCHANIM_MINOR_DEFAULT,
    LV_PUSHANIM_LEFT,
    LV_PUSHANIM_RIGHT,
};

enum
{
    LV_ZOOMANIM_AUX = LV_SWITCHANIM_MINOR_DEFAULT,
    LV_ZOOMANIM_SMALL,
    LV_ZOOMANIM_LARGER,
    LV_ZOOMANIM_MAINMENU = LV_ZOOMANIM_LARGER,
};

enum
{
    LV_MASKANIM_FADE = LV_SWITCHANIM_MINOR_DEFAULT,
    LV_MASKANIM_OPEN,
};

enum
{
    LV_TURNAXIS_X = LV_SWITCHANIM_MINOR_DEFAULT,
    LV_TURNAXIS_Y,
};

enum
{
    LV_TURN3D_HOR = LV_SWITCHANIM_MINOR_DEFAULT,
    LV_TURN3D_VER,
};

enum
{
    LV_TURNHALF_HOR = LV_SWITCHANIM_MINOR_DEFAULT,
    LV_TURNHALF_VER,
};

enum
{
    LV_TURNBOOK_HOR = LV_SWITCHANIM_MINOR_DEFAULT,
    LV_TURNBOOK_VER,
};

#define LV_SWITCHANIM_PRIOR_DEFAULT 2
#define LV_SWITCHANIM_PRIOR_LOWEST 0
#define LV_SWITCHANIM_PRIOR_LOW 1
#define LV_SWITCHANIM_PRIOR_MIDDLE 2
#define LV_SWITCHANIM_PRIOR_HIGH 3
#define LV_SWITCHANIM_PRIOR_HIGHEST 4

#define LV_SWITCHANIM_DEFAULT_TYPE LV_SWITCHANIM_PUSH


#define SWITCH_ANIM_NAME_LEN 20
#define SWITCH_ANIM_SECTION_NAME switch_anim
#define _str(name) #name
#define str(name) _str(name)
#define BUILTIN_ANIMATION(anim_name,anim_major, anim_progress_cb)                 \
        SECTION_ITEM_REGISTER(SWITCH_ANIM_SECTION_NAME, static const lv_baseanim_cb CONCAT_2(anim_name, _sw_anim)) = \
        {                                                               \
            .name = str(anim_name) ,\
            .major = anim_major,                                              \
            .progress_cb = anim_progress_cb,                                                                   \
        };


#define LV_SWITCHANIM_PROGRESS_MAX 1024
#define LV_COORD_SWAP(A,B) lv_coord_t A##B = A ; A=B;B=A##B;

enum
{
    LV_BASEANIM_PATH_LINE = 0,
    LV_BASEANIM_PATH_EASE_IN,
    LV_BASEANIM_PATH_EASE_OUT,
    LV_BASEANIM_PATH_EASE_IN_OUT,
};
typedef uint8_t lv_baseanim_path;
typedef uint32_t lv_baseanim_type;

typedef struct _lv_baseanim lv_baseanim_t;
typedef struct _lv_baseanim_cb lv_baseanim_cb;

enum
{
    LV_SWITCHANIM_FLAG_IS_BACK      = (1 << 0),
    LV_SWITCHANIM_FLAG_SKIP_SNAPHOT = (1 << 2),
    LV_SWITCHANIM_FLAG_16_ALIGN     = (1 << 3),

};

typedef struct
{
    uint16_t major;
    uint16_t minor;
    uint16_t minor_aux;
    int16_t prior;
    void *user_data;
    uint32_t flag;
} lv_baseanim_para_t;

typedef void (*lv_switchanim_finish_cb)(bool);
typedef void (*lv_baseanim_progress_cb)(lv_baseanim_t *baseanim, lv_obj_t *anim_obj, int32_t progress);

extern const lv_obj_class_t lv_switchanim_class;

struct _lv_baseanim_cb
{
    char name[SWITCH_ANIM_NAME_LEN];
    uint32_t major;
    lv_baseanim_progress_cb progress_cb;
};

/**
@brief some set/get function for lv_baseanim_t
@return none.
 */

lv_obj_t *lv_baseanim_get_original_screen(lv_baseanim_t *baseanim);
lv_baseanim_para_t *lv_baseanim_get_para(lv_baseanim_t *baseanim);
lv_baseanim_type lv_baseanim_get_type(lv_baseanim_t *baseanim);

/**
@brief initialize animation list,and load all animations
@return none.
 */
void lv_switchanim_init();

/**
@brief load all animations
@return none.
 */
void lv_switchanim_load();

/**
@brief uninitialize animation list,
@return none.
 */
void lv_switchanim_deinit();

/**
@brief register a instance animation to list
@param[in] name The name of baseanim
@param[in] major The major no of baseanim
@param[in] create_cb Called when the baseanim is created
@param[in] progress_cb Called when the progress changes
@param[in] finish_cb Called when the baseanim is finish
@param[in] abnormal_cb Called when the baseanim is abnormal termination
@return none.
 */
lv_baseanim_cb *lv_switchanim_register_anim(char *name, uint32_t major, lv_baseanim_progress_cb progress_cb);

/**
@brief remove a instance animation in list
@param[in] major The major no of baseanim to remove
@return none.
 */
void lv_switchanim_unregister_anim(uint32_t major);

/**
@brief Create switchanim instance handle.
@param[in] enter_org The original type of enter animation
@param[out] enter_anim The animation overwritten based on priority
@param[in] exit_org The original type of exit animation
@param[out] exit_anim The animation overwritten based on priority
@param[in] flag Forward animation or backward animation
@return none.
 */
void lv_switchanim_overwrite(const lv_baseanim_para_t *enter_org, \
                             lv_baseanim_para_t *enter_anim, \
                             const lv_baseanim_para_t *exit_org, \
                             lv_baseanim_para_t *exit_anim, \
                             uint32_t flag);
/**
@brief Create switchanim instance handle.
@param[in] parent the parent of the switchanim, NULL for root screen
@param[in] enter_screen The enter screen
@param[in] exit_screen The exit screen
@param[in] enter_para The parameter of enter anim
@param[in] exit_para The parameter of exit anim
@return switchanim instance handle.
 */
lv_obj_t *lv_switchanim_create(lv_obj_t *parent, lv_obj_t *enter_screen, lv_obj_t *exit_screen,
                               lv_baseanim_para_t *enter_para, lv_baseanim_para_t *exit_para);

/**
@brief For follow-up animations, implement animation frames based on the percentage of progress
@param[in] switchanim the instance handle of switchanim.
@param[in] progress The enter screen
@return none.
 */
void lv_switchanim_manual_run(lv_obj_t *switchanim, int32_t progress);

/**
@brief For follow-up animations, Manually ending animation
@param[in] switchanim the instance handle of switchanim.
@param[in] is_enter Decide which screen to load after the animation ends
@return none.
 */
void lv_switchanim_manual_finish(lv_obj_t *switchanim, bool is_enter);

/**
@brief Enable automatic playback of animations.
@param[in] switchanim the instance handle of switchanim.
@param[in] period The time period for animation playback
@param[in] progress The starting progress of animation playback
@param[in] reverse Decide whether to reverse the animation playback
@return none.
 */
void lv_switchanim_auto_run(lv_obj_t *switchanim, uint32_t period, uint32_t progress, bool reverse);

/**
@brief Enable automatic playback of animations.
@param[in] switchanim the instance handle of switchanim.
@param[in] finish_cb Set the callback function ,called at the end of animation playback
@return none.
 */
void lv_switchanim_set_finish_cb(lv_obj_t *switchanim, lv_switchanim_finish_cb finish_cb);

/**
@brief Enable automatic playback of animations.
@param[in] switchanim the instance handle of switchanim.
@param[in] path Set the animation path for progress growth mode,
@return none.
 */
void lv_switchanim_set_path(lv_obj_t *switchanim, lv_baseanim_path path);

/**
@brief set default animation
@return none.
 */
void lv_switchanim_set_def_anim(uint16_t anim_major, uint16_t anim_minor);


lv_baseanim_cb *lv_switchanim_find_anim(uint32_t major);

#ifdef __cplusplus
}
#endif // 

#endif

#endif
