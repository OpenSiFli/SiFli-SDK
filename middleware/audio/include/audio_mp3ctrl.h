#ifndef AUDIO_MP3CTRL_H
#define AUDIO_MP3CTRL_H     1

#include <audio_server.h>

typedef struct mp3ctrl_t *mp3ctrl_handle;

typedef struct
{
    uint32_t total_time_in_seconds;
    uint32_t samplerate;
    uint32_t one_channel_sampels;
    uint8_t  channels;
} mp3_info_t;

typedef struct
{
    char *title;
    char *artist;
    char *album;
    char year[5 * 3 + 1];
} mp3_id3_info_t;

typedef struct
{
    const char *filename; //new filename for mp3ctrl_open or new buffer for mp3ctrl_open_buffer
    uint32_t len;         //buffer len, if handle is return by mp3ctrl_open, len must be -1;
} mp3_ioctl_cmd_param_t;
/*
open:
    return NULL if file error
    should process callback(cmd, 0 , 0), cmd maybe
        as_callback_cmd_suspended     --- was suspend by high priority audio
        as_callback_cmd_resumed       --- resumed from suspend
        as_callback_cmd_play_to_end   --- has play to the end

        as_callback_cmd_user + 0
        in (audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
            cmd is as_callback_cmd_user + 0
            (uint32_t)callback_userdata is current play time in seconds
*/
mp3ctrl_handle mp3ctrl_open(audio_type_t type, const char *filename, audio_server_callback_func callback, void *callback_userdata);
mp3ctrl_handle mp3ctrl_open_buffer(audio_type_t type, const char *buf, uint32_t buf_len, audio_server_callback_func callback, void *callback_userdata);
/**
    cmd:
       0    set loop times, param is loop times
       1    switch to new file or new buffer, param is (mp3_ioctl_cmd_param_t *)
            if handle is return by mp3ctrl_open, can't switch to new buffer;
            if handle is  return by mp3ctrl_open_buffer, can't switch to new file
       2    set thread priority, param is priority value
*/
int mp3ctrl_ioctl(mp3ctrl_handle handle, int cmd, uint32_t param);
int mp3ctrl_close(mp3ctrl_handle handle);
int mp3ctrl_play(mp3ctrl_handle handle);
int mp3ctrl_pause(mp3ctrl_handle handle);
int mp3ctrl_resume(mp3ctrl_handle handle);
int mp3ctrl_seek(mp3ctrl_handle handle, off_t frame_offset);
int mp3ctrl_getinfo(const char *filename, mp3_info_t *info);
int mp3_get_id3_start(const char *filename, mp3_id3_info_t *info);
void mp3_get_id3_end(mp3_id3_info_t *info);
#endif
