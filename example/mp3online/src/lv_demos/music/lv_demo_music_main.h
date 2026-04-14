/**
 * @file lv_demo_music_main.h
 *
 */

#ifndef LV_DEMO_MUSIC_MAIN_H
#define LV_DEMO_MUSIC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lv_demo_music.h"
#if LV_USE_DEMO_MUSIC
typedef enum {
    UI_MSG_INIT,
    UI_MSG_PLAY_PAUSE,
    UI_MSG_PREV,
    UI_MSG_NEXT,
    UI_MSG_UPDATE_ALBUM
} ui_msg_type_t;

typedef struct {
    ui_msg_type_t type;
    uint32_t param;
} ui_msg_t;
/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
lv_obj_t *_lv_demo_music_main_create(lv_obj_t *parent);
void _lv_demo_music_main_close(void);

void _lv_demo_music_play(uint32_t id);
void _lv_demo_music_resume(void);
void _lv_demo_music_pause(void);
void _lv_demo_music_album_next(bool next);

/**********************
 *      MACROS
 **********************/
#endif /*LV_USE_DEMO_MUSIC*/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_DEMO_MUSIC_MAIN_H*/
