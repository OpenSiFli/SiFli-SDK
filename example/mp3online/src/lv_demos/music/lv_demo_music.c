/**
 * @file lv_demo_music.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_demo_music.h"

#if LV_USE_DEMO_MUSIC

#include "lv_demo_music_main.h"
#include "lv_demo_music_list.h"
#include "mp3_playlist.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_obj_t *ctrl;
static lv_obj_t *list;

static const char *title_list[] =
{
    //"Waiting for true love",
    "Hope Is the Thing With Feathers",
    "Need a Better Future",
    "Vibrations",
    "Why now?",
    "Never Look Back",
    "It happened Yesterday",
    "Feeling so High",
    "Go Deeper",
    "Find You There",
    "Until the End",
    "Unknown",
    "Unknown",
    "Unknown",
    "Unknown",
};

static const char *artist_list[] =
{
    //"The John Smith Band",
    "Chevy",
    "My True Name",
    "Robotics",
    "John Smith",
    "My True Name",
    "Robotics",
    "Robotics",
    "Unknown artist",
    "Unknown artist",
    "Unknown artist",
    "Unknown artist",
    "Unknown artist",
    "Unknown artist",
    "Unknown artist",
    "Unknown artist",
};

static const char *genre_list[] =
{
    //"Rock - 1997",
    "Pop - 2024",
    "Drum'n bass - 2016",
    "Psy trance - 2020",
    "Metal - 2015",
    "Metal - 2015",
    "Metal - 2015",
    "Metal - 2015",
    "Metal - 2015",
    "Metal - 2015",
    "Metal - 2015",
    "Metal - 2015",
    "Metal - 2015",
    "Metal - 2015",
    "Metal - 2015",
};

static const uint32_t time_list[] =
{
    //1 * 60 + 14,
    3 * 60 + 49,
    2 * 60 + 26,
    1 * 60 + 54,
    2 * 60 + 24,
    2 * 60 + 37,
    3 * 60 + 33,
    1 * 60 + 56,
    3 * 60 + 31,
    2 * 60 + 20,
    2 * 60 + 19,
    2 * 60 + 20,
    2 * 60 + 19,
    2 * 60 + 20,
    2 * 60 + 19,
};

#if LV_DEMO_MUSIC_AUTO_PLAY
    static lv_timer_t *auto_step_timer;
#endif

static lv_color_t original_screen_bg_color;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_demo_music(void)
{
    original_screen_bg_color = lv_obj_get_style_bg_color(lv_scr_act(), 0);
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x343247), 0);

    ctrl = _lv_demo_music_main_create(lv_scr_act());
    list = _lv_demo_music_list_create(ctrl);
}

void lv_demo_music_close(void)
{
    /*Delete all aniamtions*/
    lv_anim_del(NULL, NULL);

    _lv_demo_music_list_close();
    _lv_demo_music_main_close();

    lv_obj_clean(lv_scr_act());

    lv_obj_set_style_bg_color(lv_scr_act(), original_screen_bg_color, 0);
}

const char *_lv_demo_music_get_title(uint32_t track_id)
{
#if 0
    if (track_id >= sizeof(title_list) / sizeof(title_list[0])) return NULL;
    return title_list[track_id];
#endif
    return mp3_playlist_get_song_title(track_id);
}

const char *_lv_demo_music_get_artist(uint32_t track_id)
{
#if 0
    if (track_id >= sizeof(artist_list) / sizeof(artist_list[0])) return NULL;
    return artist_list[track_id];
#endif
    return mp3_playlist_get_song_artist(track_id);
}

const char *_lv_demo_music_get_genre(uint32_t track_id)
{
    if (track_id >= sizeof(genre_list) / sizeof(genre_list[0])) return NULL;
    return genre_list[track_id];
}


extern uint32_t mp3_stream_get_total_seconds(void);
uint32_t _lv_demo_music_get_track_length(uint32_t track_id)
{
#if 0
    if (track_id >= sizeof(time_list) / sizeof(time_list[0])) return 0;
    return time_list[track_id];
#endif
    return mp3_stream_get_total_seconds();
}

#endif /*LV_USE_DEMO_MUSIC*/
