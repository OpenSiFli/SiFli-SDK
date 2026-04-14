
#ifndef __MP3_JPG_H__
#define __MP3_JPG_H__

#include <lvgl.h>

typedef void(* mp3_dl_img_user_callback)(const char *filename, int result);

int mp3_dl_img(const char *url, const char *filename, mp3_dl_img_user_callback callback);
bool mp3_img_is_downloading(void);
bool mp3_img_file_is_ready(const char *filename);

#endif
